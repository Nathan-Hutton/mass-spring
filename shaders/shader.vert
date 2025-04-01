#version 460 core

layout (location = 0) in vec3 aPos;

out vec3 fragPosVert;
out vec3 posViewSpace;

uniform mat4 view;
uniform mat4 projection;

void main()
{
    posViewSpace = vec3(view * vec4(aPos, 1.0));
	fragPosVert = vec3(view * vec4(aPos, 1.0));

    gl_Position = projection * view * vec4(aPos, 1.0);
}
