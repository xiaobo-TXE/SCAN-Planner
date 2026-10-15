// ============================================================================
// 【片段着色器】camera.fs
//
// 【说明】本项目只关心“深度”，不关心颜色，所以这里只是把顶点颜色原样输出。
// ============================================================================
#version 330 core
out vec4 FragColor;

in vec3 ourColor;

// texture samplers
// uniform sampler2D texture1;
// uniform sampler2D texture2;

void main()
{
	// linearly interpolate between both textures (80% container, 20% awesomeface)
	// FragColor = mix(texture(texture1, TexCoord), texture(texture2, TexCoord), 0.2);
 // 直接把传入的颜色作为输出像素（不做任何纹理混合）
	FragColor = vec4(ourColor, 1.0f);
}