#version 460 core

layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

in vec3 fragPosVert[];
in vec3 posViewSpace[];

out vec3 fragPos;
out vec3 normal;

void main()
{
    vec3 faceNormal = normalize(cross(posViewSpace[1] - posViewSpace[0], posViewSpace[2] - posViewSpace[0]));
    for (int i = 0; i < 3; ++i)
    {
        fragPos = fragPosVert[i];
        normal = faceNormal;
        gl_Position = gl_in[i].gl_Position;
        EmitVertex();
    }
}
