#version 460 core

uniform uint objectIndex;
uniform uint drawIndex;

out uvec3 fragColor;

void main()
{
    fragColor = uvec3(objectIndex, drawIndex, gl_PrimitiveID);
}
