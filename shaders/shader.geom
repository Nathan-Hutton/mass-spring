#version 460 core

layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

in vec3 fragPosVert[];
in vec3 normalVert[];

out vec3 fragPos;
out vec3 normal;

void main()
{
    for (int i = 0; i < 3; ++i)
    {
        fragPos = fragPosVert[i];
        normal = normalVert[i];
        gl_Position = gl_in[i].gl_Position;
        EmitVertex();
    }
}
