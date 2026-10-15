// ============================================================================
// 【备用着色器】camera.vs —— 常规透视投影的顶点着色器
//
// 【说明】
//   这里做的是最普通的“相机透视投影”，本项目默认不使用它
//   （渲染走的是 360camera.vs 的极坐标投影）。保留它是因为 GPU 版的
//   某些调试 / 备用路径会用到。
// ============================================================================
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

out vec3 ourColor;

uniform mat4 view;
uniform mat4 projection;
uniform vec2 range;

void main()
{
 // 标准透视投影：世界坐标 → 裁剪空间坐标
	gl_Position = projection * view * vec4(aPos, 1.0f);
 // 超出射程的点丢到画面外
	if(gl_Position.z < range.x || gl_Position.z > range.y)
	{
		gl_Position = vec4(0.0f,0.0f,0.0f,0.0f);
	}
	ourColor = aColor;
}