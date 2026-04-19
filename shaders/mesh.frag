#version 330 core
// mesh.frag -- Phong lighting with a single directional light.
// A subtle edge-darkening Fresnel term gives fragments more "crystal" feel.

in vec3 vWorldPos;
in vec3 vWorldNormal;

uniform vec3 uViewPos;
uniform vec3 uLightDir;   // direction toward -light (i.e. where light goes)
uniform vec3 uLightCol;
uniform vec3 uAmbient;
uniform vec3 uBaseColor;

out vec4 FragColor;

void main()
{
    vec3 N = normalize(vWorldNormal);
    vec3 L = normalize(-uLightDir);
    vec3 V = normalize(uViewPos - vWorldPos);
    vec3 H = normalize(L + V);

    float diff = max(dot(N, L), 0.0);
    float spec = pow(max(dot(N, H), 0.0), 42.0);

    // Subtle rim/Fresnel for depth.
    float fres = pow(1.0 - max(dot(N, V), 0.0), 3.0) * 0.35;

    vec3 color = uAmbient * uBaseColor
               + uLightCol * uBaseColor * diff
               + uLightCol * spec * 0.25
               + fres * vec3(0.9, 0.92, 1.0);

    // A faint vertical gradient keeps pure-white fragments from feeling flat.
    color *= mix(0.92, 1.05, clamp(vWorldPos.y * 0.15 + 0.5, 0.0, 1.0));

    FragColor = vec4(color, 1.0);
}
