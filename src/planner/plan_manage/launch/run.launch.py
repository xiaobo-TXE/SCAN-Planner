# ============================================================================
# 文件：launch/run.launch.py —— 最常用的启动入口
#
# 【怎么用】
#   ros2 launch scan_planner run.launch.py is_real_world:=false navi_mode:=1 \
#     sensor_type:=lidar controller_mode:=closed_loop use_gpu:=false
#
# 【这个文件做的事情】
#   1. 根据 is_real_world 决定连“真机”还是“仿真”的话题
#   2. 启动规划器 scan_planner_node
#   3. 启动机器人模型显示 robot_state_publisher
#   4. 启动控制器（开环或闭环），仿真时再加上假机器人和步态动画
#   5. 仿真时顺便把 simulator.launch.py（造地图+传感器）也拉起来
#
# 【重要概念】remappings = 话题改名。
#   规划器内部写的是 body_pose，但真机上真实话题叫 /lio/robo/odom，
#   这里就把两者“接”起来。仿真时则接到 /quad_0/body_pose。
# ============================================================================
"""Main ROS 2 launch entry point for simulation and real-robot remapping."""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


# 把字符串形式的开关（如 "true"/"false"）转成 Python 的 True/False。
def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


# 读取机型档案（config/robots/<机型>.yaml），返回整份内容。
# 档案里同时放“节点参数段”和“话题段”，换机型只要加一个文件、启动时换名字。
def _load_robot_profile(scan_share, profile):
    profile_dir = os.path.join(scan_share, "config", "robots")
    path = os.path.join(profile_dir, profile + ".yaml")
    if not os.path.isfile(path):
        available = []
        if os.path.isdir(profile_dir):
            available = sorted(name[:-5] for name in os.listdir(profile_dir)
                               if name.endswith(".yaml"))
        raise RuntimeError(
            "unknown robot profile '%s'; available profiles: %s"
            % (profile, ", ".join(available) if available else "(none)")
        )
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


# 从档案里取出某个节点的参数段（形如 <节点名>: {ros__parameters: {...}}）。
def _profile_params(profile, node_name):
    return dict(profile.get(node_name, {}).get("ros__parameters", {}) or {})


# 机型档案 -> 仿真里要显示的机器人模型：
#   robot:=a2   → a2_description/urdf/a2.urdf（A2 实机 URDF，21 link / 20 joint，腿可动）
#   robot:=go2  → go2_description/xacro/robot.xacro
#   robot:=go2w → go2w_description/xacro/robot.xacro（由 Go2 派生，见该包 README）
# 表里没有的（含仿真默认的 robot:=none）回退到 Go2，保持原有行为。
_ROBOT_MODELS = {
    "a2": ("a2_description", ("urdf", "a2.urdf"), []),
    "go2": ("go2_description", ("xacro", "robot.xacro"), [" use_gazebo:=false"]),
    "go2w": ("go2w_description", ("xacro", "robot.xacro"), [" use_gazebo:=false"]),
}
_DEFAULT_ROBOT_MODEL = _ROBOT_MODELS["go2"]


# 按 robot 机型拼出 robot_description 的 xacro 命令行（纯 URDF 也能被 xacro 处理）。
def _robot_description_command(robot):
    package, parts, extra = _ROBOT_MODELS.get(robot, _DEFAULT_ROBOT_MODEL)
    model = os.path.join(get_package_share_directory(package), *parts)
    return ["xacro ", model] + extra


# 真正干活的函数：在这里根据参数决定启动哪些节点、做哪些话题重映射。
# 用 OpaqueFunction 包裹，是为了能在“运行时”拿到参数值再决定。
def _setup(context):
    scan_share = get_package_share_directory("scan_planner")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    is_real = _as_bool(LaunchConfiguration("is_real_world").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    sensor_type = LaunchConfiguration("sensor_type").perform(context)
    controller_mode = LaunchConfiguration("controller_mode").perform(context)
    keypoints_file = LaunchConfiguration("keypoints_file").perform(context)
    reference_path_file = LaunchConfiguration("reference_path_file").perform(context)
    navi_mode = int(LaunchConfiguration("navi_mode").perform(context))
    if sensor_type not in ("lidar", "depth"):
        raise RuntimeError("sensor_type must be 'lidar' or 'depth'")
    if controller_mode not in ("open_loop", "closed_loop"):
        raise RuntimeError("controller_mode must be 'open_loop' or 'closed_loop'")
    if navi_mode not in (1, 2, 3):
        raise RuntimeError("navi_mode must be 1, 2, or 3")
    if navi_mode == 2 and (not keypoints_file or not os.path.isfile(keypoints_file)):
        raise RuntimeError(
            "navi_mode=2 requires keypoints_file to reference a ROS 2 parameter YAML"
        )
    # reference_path_file：可选参数，指定后额外拉起自带的参考路径发布器，
    # 用参数文件里的示例航点驱动 navi_mode=3（只在 mode 3 下才有意义）。
    if reference_path_file and navi_mode != 3:
        raise RuntimeError("reference_path_file is only valid when navi_mode=3")
    if reference_path_file and not os.path.isfile(reference_path_file):
        raise RuntimeError(
            "reference_path_file must reference an existing ROS 2 parameter YAML"
        )

    # ---- 机型档案（robot profile）----
    # 机型相关的东西（机器人尺寸、速度上限、地图尺寸、真机话题名、初始位置）放在
    # config/robots/<机型>.yaml 里，换机型只要加一个档案、启动时换名字，不必改代码：
    #   robot:=a2    叠加 config/robots/a2.yaml
    #   robot:=none  不叠加，纯用 planner.yaml / controllers.yaml 的通用默认（仿真常用）
    # 不显式指定时：真机默认用 a2 档案，仿真不叠加。
    robot = LaunchConfiguration("robot").perform(context).strip()
    if not robot:
        robot = "a2" if is_real else "none"
    profile = {} if robot == "none" else _load_robot_profile(scan_share, robot)
    profile_topics = profile.get("robot_topics", {}) or {}

    # 初始位置优先级：命令行 init_x/init_y/init_z > 档案例 robot_topics.init
    # > 按导航模式给的默认值（navi_mode=1 与其余模式的起点不同）。
    mode_default_init = (-19.0, 1.0, 0.25) if navi_mode == 1 else (-5.5, 5.5, 0.5)
    profile_init = profile_topics.get("init")
    if profile_init is not None and len(profile_init) != 3:
        raise RuntimeError("robot_topics.init must be a [x, y, z] list")
    fallback_init = tuple(profile_init) if profile_init else tuple(mode_default_init)
    initial_position = []
    for name, default in zip(("init_x", "init_y", "init_z"), fallback_init):
        value = LaunchConfiguration(name).perform(context)
        initial_position.append(default if value == "" else float(value))
    init_x, init_y, init_z = initial_position

    # 下面这段是“二选一”：
    #   if 分支 = 真机：话题接 LIO 里程计/激光、相机内参用真机标定值；
    #   else 分支 = 仿真：话题接 /quad_0/*，点云已经是世界系、不需要外参。
    # 真机话题名优先取机型档案里的 robot_topics；档案没写就用下面这组默认
    # （当前按 A2 上实际运行的 LIO/super_lio 配置，点云已在世界系下）。
    if is_real:
        body_pose = profile_topics.get("body_pose", "/lio/robo/odom")
        sensor_pose = profile_topics.get("sensor_pose", "/lio/imu/odom")
        cloud = profile_topics.get("cloud", "/lio/cloud_world")
        depth = profile_topics.get("depth", "/camera/aligned_depth_to_color/image_raw")
        cloud_is_world = bool(profile_topics.get("cloud_is_world", True))
        need_extrinsic = True
        intrinsics = {
            "grid_map.cx": 317.19183349609375,
            "grid_map.cy": 256.4806823730469,
            "grid_map.fx": 609.5884399414062,
            "grid_map.fy": 609.22021484375,
        }
    else:
        body_pose = "/quad_0/body_pose"
        sensor_pose = "/quad_0/camera_pose" if sensor_type == "depth" else "/quad_0/lidar_pose"
        cloud = "/quad_0/cloud"
        depth = "/quad_0/depth"
        cloud_is_world = True
        need_extrinsic = False
        intrinsics = {}

    common = {"use_sim_time": use_sim_time}
    # 机型档案里与各节点对应的参数段（没有就是空 dict，等于不覆盖）
    planner_profile_params = _profile_params(profile, "scan_planner_node")
    controller_profile_params = _profile_params(profile, "closed_loop_controller")
    sim_profile_params = _profile_params(profile, "go2_kinematic_sim")
    planner_overrides = {
        **common,
        **intrinsics,
        "fsm.navi_mode": navi_mode,
        "grid_map.sensor_type": sensor_type,
        "grid_map.cloud_is_world": cloud_is_world,
        "grid_map.need_extrinsic": need_extrinsic,
    }
    # ---- 节点 1：规划器本体（最重要）----
    # parameters 按顺序叠加：先 planner.yaml，再机型档案，再（可选）航点文件，最后覆盖项。
    actions = [
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[planner_yaml, planner_profile_params]
            + ([keypoints_file] if keypoints_file else [])
            + [planner_overrides],
            remappings=[
                ("body_pose", body_pose),
                ("sensor_pose", sensor_pose),
                ("cloud", cloud),
                ("depth", depth),
                ("move_base_simple/goal", "/move_base_simple/goal"),
                # navi_mode=3 的输入：外部给的全局路径（真机上是 global_planner 发出的）
                ("initial_path", "/global_path"),
                # 规划器额外把当前局部轨迹发出来，方便观察/录制
                ("local_path", "/local_path"),
            ],
        )
    ]
    # ---- 节点 2：机器人模型显示（仅仿真）----
    # 用 xacro 生成 robot_description，让 RViz 能画出当前机型的机器人外形
    # （模型由 robot 档案经 _ROBOT_MODELS 决定，见上方表）。
    # 真机上机器人模型与 TF 由宇树自身的驱动栈提供，重复发布反而会冲突，
    # 所以只在仿真（is_real_world:=false）时启动这个节点。
    if not is_real:
        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    common,
                    {"robot_description": Command(_robot_description_command(robot))},
                ],
            )
        )

    if controller_mode == "open_loop":
        actions.append(
            Node(
                package="scan_planner",
    # ---- 控制器（方案 A）：开环 ----
    # 直接把轨迹当成里程计，不做控制。适合单独看规划效果。
                executable="open_loop_controller",
                name="open_loop_controller",
                output="screen",
                parameters=[
                    controllers_yaml,
                    common,
                    {"init_x": init_x, "init_y": init_y, "init_z": init_z},
                ],
                remappings=[
                    ("planning/bspline", "/planning/bspline"),
                    ("body_pose", body_pose),
                ],
            )
        )
    else:
        actions.append(
            Node(
                package="scan_planner",
    # ---- 控制器（方案 B）：闭环（默认）----
    # 真正算速度指令 cmd_vel。真机发到 /cmd_vel，仿真发到 /quad_0/cmd_vel。
                executable="closed_loop_controller",
                name="closed_loop_controller",
                output="screen",
                parameters=[controllers_yaml, controller_profile_params, common],
                remappings=[
                    ("body_pose", body_pose),
                    ("cmd_vel", "/cmd_vel" if is_real else "/quad_0/cmd_vel"),
                ],
            )
        )
        if not is_real:
            actions.append(
                Node(
                    package="scan_planner",
    # ---- 假机器人（仅仿真）----
    # 接收 cmd_vel 算出位置，再发 body_pose，和控制器组成闭环。
                    executable="go2_kinematic_sim",
                    name="go2_kinematic_sim",
                    output="screen",
                    parameters=[
                        controllers_yaml,
                        sim_profile_params,
                        common,
                        {
                            "init_x": init_x,
                            "init_y": init_y,
                            "init_z": init_z,
                            "publish_tf": False,
                        },
                    ],
                    remappings=[
                        ("body_pose", "/quad_0/body_pose"),
                        ("cmd_vel", "/quad_0/cmd_vel"),
                    ],
                )
            )

    # ---- 节点 4：自带的参考路径发布器（可选，仅 navi_mode=3）----
    # 只用来跑示例：从参数文件读一条航点表，等里程计就绪后发到参考路径话题上。
    # 真机上通常不需要它——外部 global_planner 会自己发全局路径。
    if reference_path_file:
        actions.append(
            Node(
                package="scan_planner",
                executable="reference_path_publisher.py",
                name="reference_path_publisher",
                output="screen",
                parameters=[reference_path_file, common],
                remappings=[
                    ("body_pose", body_pose),
                    # 必须发到规划器实际订阅的全局路径话题上，否则这条示例
                    # 路径没人接收（规划器侧映射的是 /global_path）。
                    ("initial_path", "/global_path"),
                ],
            )
        )

    if not is_real:
        actions.extend(
            [
                Node(
                    package="scan_planner",
    # ---- 步态动画（仅仿真）----
    # 纯视觉效果：让 RViz 里的 Go2 腿动起来。
                    executable="go2_gait_publisher",
                    name="go2_gait_publisher",
                    output="screen",
                    parameters=[controllers_yaml, common],
                    remappings=[("body_pose", body_pose)],
                ),
    # ---- 拉起仿真环境 ----
    # 包含 simulator.launch.py：造地图 + 模拟传感器 + 里程计可视化。
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(scan_share, "launch", "simulator.launch.py")
                    ),
                    launch_arguments={
                        name: LaunchConfiguration(name)
                        for name in (
                            "is_real_world",
                            "sensor_type",
                            "use_gpu",
                            "use_pcd_map",
                            "pcd_map_file",
                            "map_size_x",
                            "map_size_y",
                            "map_size_z",
                            "use_sim_time",
                        )
                    }.items(),
                ),
            ]
        )
    return actions


# 声明所有可在命令行覆盖的参数（--ros-args 之外的 launch 参数），
# 默认值与 README 里写的一致；最后用 OpaqueFunction 调用上面的 _setup。
def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("is_real_world", default_value="true"),
            DeclareLaunchArgument("navi_mode", default_value="3"),
            DeclareLaunchArgument("sensor_type", default_value="lidar"),
            DeclareLaunchArgument("controller_mode", default_value="closed_loop"),
            DeclareLaunchArgument("keypoints_file", default_value=""),
            DeclareLaunchArgument("reference_path_file", default_value=""),
            DeclareLaunchArgument("robot", default_value=""),
            DeclareLaunchArgument("use_gpu", default_value="false"),
            DeclareLaunchArgument("use_pcd_map", default_value="false"),
            DeclareLaunchArgument("pcd_map_file", default_value=""),
            DeclareLaunchArgument("map_size_x", default_value="40.0"),
            DeclareLaunchArgument("map_size_y", default_value="40.0"),
            DeclareLaunchArgument("map_size_z", default_value="5.0"),
            DeclareLaunchArgument("init_x", default_value=""),
            DeclareLaunchArgument("init_y", default_value=""),
            DeclareLaunchArgument("init_z", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )
