#version 460 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNorm;

out vec3 fragPos;
out vec3 normal;

uniform mat4 modelView;
uniform mat4 normalModelView;
uniform mat4 projection;

void main()
{
	fragPos = vec3(modelView * vec4(aPos, 1.0));
	normal = normalize(mat3(normalModelView) * aNorm);

    gl_Position = projection * modelView * vec4(aPos, 1.0);
}
