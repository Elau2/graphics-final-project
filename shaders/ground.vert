#version 330 core
// ground.vert

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

uniform mat4 uView;
uniform mat4 uProj;

out vec3 vWorldPos;
out vec3 vWorldNormal;

void main()
{
    vWorldPos    = aPos;
    vWorldNormal = aNormal;
    gl_Position  = uProj * uView * vec4(aPos, 1.0);
}
