#version 460 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNorm;

out vec3 fragPosVert;
//out vec3 posWorldSpace;
out vec3 normalVert;

//uniform mat4 model;
uniform mat4 modelView;
uniform mat4 normalModelView;
uniform mat4 projection;

void main()
{
    //posWorldSpace = vec3(model * vec4(aPos, 1.0));
	fragPosVert = vec3(modelView * vec4(aPos, 1.0));
	normalVert = normalize(mat3(normalModelView) * aNorm);

    gl_Position = projection * modelView * vec4(aPos, 1.0);
}
