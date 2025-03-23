#version 460 core

layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

in vec3 fragPosVert[];
//in vec3 normalVert[];
in vec3 posViewSpace[];

out vec3 fragPos;
out vec3 normal;

void main()
{
    vec3 faceNormal = normalize(cross(posViewSpace[1] - posViewSpace[0], posViewSpace[2] - posViewSpace[0]));
    //vec3 faceNormal = vec3(1.0, 1.0f, 1.0f);
    //if (
    //    posViewSpace[0].x < -4.8f &&
    //    posViewSpace[0].x > -5.2f &&
    //    posViewSpace[0].y < -0.8f &&
    //    posViewSpace[0].y > -1.2f &&

    //    posViewSpace[1].x < 5.2f &&
    //    posViewSpace[1].x > 4.8f &&
    //    posViewSpace[1].y < -0.8f &&
    //    posViewSpace[1].y > -1.2f &&

    //    posViewSpace[2].x < 5.2f &&
    //    posViewSpace[2].x > 4.8f &&
    //    posViewSpace[2].y < 9.2f &&
    //    posViewSpace[2].y > 8.8f
    //    )
    //    faceNormal = vec3(1.0f, 0.0f, 0.0f);

    //vec3 faceNormal = vec3(1.0, 1.0f, 1.0f);
    //if (
    //    posViewSpace[0].x < -9.8f &&
    //    posViewSpace[0].x > -10.2f &&
    //    posViewSpace[0].z < 10.2f &&
    //    posViewSpace[0].z > 9.8f &&

    //    posViewSpace[1].x < 10.2f &&
    //    posViewSpace[1].x > 9.8f &&
    //    posViewSpace[1].z < 10.2f &&
    //    posViewSpace[1].z > 9.8f &&

    //    posViewSpace[2].x < 10.2f &&
    //    posViewSpace[2].x > 9.8f &&
    //    posViewSpace[2].z < -9.8f &&
    //    posViewSpace[2].z > -10.2f
    //    )
    //    faceNormal = vec3(1.0f, 0.0f, 0.0f);

    for (int i = 0; i < 3; ++i)
    {
        fragPos = fragPosVert[i];
        normal = faceNormal;
        gl_Position = gl_in[i].gl_Position;
        EmitVertex();
    }
}
