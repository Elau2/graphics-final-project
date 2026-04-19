#version 330 core
// mesh.vert -- per-vertex Phong setup for fragment rendering.

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProj;
uniform mat3 uNormal; // transpose(inverse(mat3(uModel)))

out vec3 vWorldPos;
out vec3 vWorldNormal;

void main()
{
    vec4 wp = uModel * vec4(aPos, 1.0);
    vWorldPos = wp.xyz;
    vWorldNormal = normalize(uNormal * aNormal);
    gl_Position = uProj * uView * wp;
}
