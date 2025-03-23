#version 460 core

out vec4 fragColor;

in vec3 fragPos;
in vec3 normal;

uniform vec3 lightDir;
uniform vec3 diffuseMaterialColor;

void main()
{
    // Light/material properties
    const vec3 ambientColor = vec3(0.15f, 0.15f, 0.15f) * diffuseMaterialColor;
    const vec3 specularMaterialColor = vec3(1.0f, 1.0f, 1.0f);
    const vec3 lightColor = vec3(1.0f, 1.0f, 1.0f);

    // Calculations
    const vec3 viewDir = normalize(-fragPos);
    const vec3 diffuseComponent = max(dot(lightDir, normal), 0.0f) * diffuseMaterialColor;

    const vec3 halfVec = normalize(lightDir + viewDir);
    const vec3 specularComponent = pow(max(dot(halfVec, normal), 0.0f), 100.0f) * specularMaterialColor;

    fragColor = vec4(ambientColor + lightColor * (diffuseComponent + specularComponent), 1.0f);
}
