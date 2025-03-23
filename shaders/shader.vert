#version 460 core

layout (location = 0) in vec3 aPos;

out vec3 fragPosVert;
out vec3 posViewSpace;

uniform mat4 modelView;
uniform mat4 normalModelView;
uniform mat4 projection;

void main()
{
    posViewSpace = vec3(modelView * vec4(aPos, 1.0));
	fragPosVert = vec3(modelView * vec4(aPos, 1.0));

    gl_Position = projection * modelView * vec4(aPos, 1.0);
}
