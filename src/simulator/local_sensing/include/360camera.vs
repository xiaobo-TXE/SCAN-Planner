// ============================================================================
// 【GPU 渲染的核心着色器】360camera.vs —— 顶点着色器
//
// 【它做什么（最重要的一点）】
//   它决定“每个 3D 点被画到图像的哪个像素上”。
//   这里做的不是普通的透视投影，而是【极坐标投影】：
//     横坐标 ← 点的水平角（方位角）
//     纵坐标 ← 点的竖直角（俯仰角）
//     深度   ← 点到传感器的距离
//   显卡的深度缓冲会自动只保留每格最近的那个点 —— 这就是“遮挡”。
//
// 【数据怎么流动】
//   3D 点 → 本着色器算出像素坐标+深度 → 光栅化 → 深度缓冲
//   → 由 CPU 读回深度图 → 还原成点云（见 opengl_sim.hpp 的 read_depth）
//
// 【注意】文件里大量被注释掉的代码，是作者调试不同投影方式时留下的，可忽略。
// ============================================================================
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

out vec3 ourColor;

// 从 CPU 传进来的参数：视图/投影矩阵、射程、旋转、位置、视场、分辨率
uniform mat4 view;
uniform mat4 projection;
uniform vec2 range;

uniform mat3 rot;
uniform vec3 pos;

// uniform vec3 ()
uniform vec2 fov;
uniform vec2 res;
// uniform float downsample_res;
// uniform float polar_res;

// 【入口】每个顶点都会执行一次
void main()
{
	float PI = 3.14159265358;
	// float yaw_fov = 360.0;
	// float vertical_fov = 50.0;
	// float downsample_res = 0.1;
	// float polar_res = 0.2;
	// float cover_dis = 0.55 * 1.7321 * downsample_res;
 // 一个点大致能盖住多大范围（用于设置它画多大）
	float cover_dis = 0.55 * 1.7321 * res.x;

	//log(aPos.z/sqrt(aPos.x*aPos.x + aPos.y*aPos.y) + 1/sqrt(1 + pow(aPos.z/sqrt(aPos.x*aPos.x + aPos.y*aPos.y),2)))
	//atan2(aPos.z,sqrt(aPos.x*aPos.x + aPos.y*aPos.y))
	// if(aPos.z/sqrt(aPos.x*aPos.x + aPos.y*aPos.y) > 0 )
	// {
		// sqrt(aPos.x*aPos.x + aPos.y*aPos.y + aPos.z*aPos.z)
		// (atan2(aPos.y,aPos.x)+PI)/(2*PI)
			// gl_Position = vec4(atan(aPos.y/aPos.x),atan(aPos.z/sqrt(aPos.x*aPos.x + aPos.y*aPos.y)), 0.0f, 1.0f);
			// gl_Position = vec4(atan(aPos.y/aPos.x),atan(aPos.z/sqrt(aPos.x*aPos.x + aPos.y*aPos.y)), 0.0f, 1.0f);
			// gl_Position = vec4((aPos.y*aPos.x),(aPos.z*(aPos.x*aPos.x + aPos.y*aPos.y)), 0.0f, 1.0f);

	// }
	// gl_Position = projection * view * vec4(aPos, 1.0f);
	
	//normal speed
	// gl_Position = vec4(aPos,1.0f);

	//normal speed
	// gl_Position = vec4(aPos*aPos,1.0f);

	//normal speed
	// gl_Position = vec4(aPos.x*aPos.x, aPos.y*aPos.y, 0.0f,1.0f);

	// cost 0.5s 1 frame
	// gl_Position = vec4(atan(aPos.y/aPos.x),atan(aPos.z/sqrt(aPos.x*aPos.x + aPos.y*aPos.y)), 0.0f, 1.0f);

	//trans from world to body coordinate
 // 把点从世界坐标系转到“传感器坐标系”（先平移再旋转）
	vec3 bodypos = rot * (aPos - pos);
	// vec4 bodypos4 = view * vec4(aPos, 1.0f);
	// vec3 bodypos;
	// bodypos.x = bodypos4.x;
	// bodypos.y = bodypos4.y;
	// bodypos.z = bodypos4.z;

	//opengl polar coordinate, large point cloud slow
	float y_x;// 
	float z_yx;//
 // 点到传感器的直线距离
	float depth = sqrt(bodypos.x*bodypos.x + bodypos.y*bodypos.y + (bodypos.z)*(bodypos.z));///range.y
	// depth = -depth* (zFar + zNear) / (zFar - zNear); 

 // 超出射程 → 丢到画面外（相当于这个点不画）
	if(depth > range.y || depth < range.x)
	{
		gl_Position = vec4(0.0f,0.0f,0.0f,0.0f);
		ourColor = aColor;
	}else{
  // 【极坐标投影】水平角 → 图像横坐标
		y_x = -(atan(bodypos.y,bodypos.x))/(PI)* (360.0/fov.x);// 
  // 【极坐标投影】竖直角 → 图像纵坐标
		z_yx = (atan((bodypos.z),sqrt(bodypos.x*bodypos.x + bodypos.y*bodypos.y))/PI)* (360.0/fov.y) ;//
		// depth = (depth + 1.0) * 2.0;
  // 用（横角, 竖角, 距离）当作画点坐标；深度写进 z，供深度缓冲做遮挡
		gl_Position = vec4(y_x, z_yx, depth/range.y, 1.0f);
		// int half_cover_angle = ceil((asin(cover_dis / 1.0) / (PI * polar_res / 180.0)));//dot(aPos,aPos)
  // 近处的点画大一些：模拟“一个点会覆盖多个角度格子”
		gl_PointSize = 2*ceil((asin(cover_dis / (depth)) / (PI * res.y / 180.0)));
		// gl_PointSize = 1;		
		// if(abs(y_x) < 0.05 && abs(z_yx) < 0.05)
		// {
		// 	ourColor = vec3(depth,0.0f,(depth));
		// }else{
			// ourColor = vec3(depth,(depth),(depth));
		// }
		
	}

	// //opengl polar coordinate, large point cloud slow
	// float y_x;// 
	// float z_yx;//
	// float depth = sqrt(aPos.x*aPos.x + aPos.y*aPos.y + (aPos.z -1)*(aPos.z -1));///range.y
	// // depth = -depth* (zFar + zNear) / (zFar - zNear); 

	// if(depth > range.y || depth < range.x)
	// {
	// 	gl_Position = vec4(0.0f,0.0f,0.0f,0.0f);
	// 	ourColor = aColor;
	// }else{
	// 	y_x = -(atan(aPos.y,aPos.x))/(PI)* (360.0/fov.x);// 
	// 	z_yx = (atan((aPos.z-1),sqrt(aPos.x*aPos.x + aPos.y*aPos.y))/PI)* (360.0/fov.y) ;//
	// 	// depth = (depth + 1.0) * 2.0;
	// 	gl_Position = vec4(y_x, z_yx, depth/range.y, 1.0f);
	// 	// int half_cover_angle = ceil((asin(cover_dis / 1.0) / (PI * polar_res / 180.0)));//dot(aPos,aPos)
	// 	gl_PointSize = 2*ceil((asin(cover_dis / (depth)) / (PI * res.y / 180.0)));
	// 	// gl_PointSize = 1;		
	// 	// if(abs(y_x) < 0.05 && abs(z_yx) < 0.05)
	// 	// {
	// 	// 	ourColor = vec3(depth,0.0f,(depth));
	// 	// }else{
	// 		ourColor = vec3(depth,(depth),(depth));
	// 	// }
		
	// }


	//opengl three image
	// gl_Position = projection *view * vec4(aPos, 1.0f);// 

	// gl_Position = vec4(0.0f,0.0f,-10.0f,1.0f);
	// gl_PointSize = 5;
	// if(gl_Position.z < range.x || gl_Position.z > range.y)
	// {
	// 	gl_Position = vec4(0.0f,0.0f,0.0f,0.0f);
	// }
 // 颜色原样传给片段着色器
	ourColor = aColor;
}
