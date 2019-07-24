#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;

out vec3 fragNormal;
out vec4 shadowCoord;

uniform mat4 view;
uniform mat4 projection;
uniform mat4 lightSpaceMatrix;

void main()
{
    gl_Position = projection * view * vec4(position, 1.0f);
    fragNormal = normalize(normal);
    shadowCoord = lightSpaceMatrix * vec4(position, 1.0f);
}
