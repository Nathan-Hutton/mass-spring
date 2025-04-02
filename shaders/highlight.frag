#version 460

out vec4 fragColor;

uniform uint selectedTriangle;

void main()
{
    if (gl_PrimitiveID != int(selectedTriangle))
        discard;

    fragColor = vec4(1.0f, 0.0f, 0.0f, 1.0f);
}
