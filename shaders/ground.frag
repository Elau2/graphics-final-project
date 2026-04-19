#version 330 core
// ground.frag -- anti-aliased checkerboard fading into the distance so
// the fragments have a visible spatial reference without a heavy texture.

in vec3 vWorldPos;
in vec3 vWorldNormal;

uniform vec3 uLightDir;
uniform vec3 uLightCol;
uniform vec3 uAmbient;

out vec4 FragColor;

// Anti-aliased checkerboard using the derivative width of the checker coord.
// See https://iquilezles.org/articles/checkerfiltering/
float checkerboard(vec2 uv)
{
    vec2 w = fwidth(uv) + 0.0001;
    vec2 i = 2.0 * (abs(fract((uv - 0.5 * w) * 0.5) - 0.5)
                  - abs(fract((uv + 0.5 * w) * 0.5) - 0.5)) / w;
    return 0.5 - 0.5 * i.x * i.y;
}

void main()
{
    float c = checkerboard(vWorldPos.xz * 1.0);
    vec3 light = mix(vec3(0.22, 0.23, 0.26), vec3(0.30, 0.31, 0.34), c);

    vec3 N = normalize(vWorldNormal);
    vec3 L = normalize(-uLightDir);
    float diff = max(dot(N, L), 0.0);

    vec3 color = uAmbient * light + uLightCol * light * diff;

    // Fade to background near the horizon so the quad doesn't look finite.
    float dist = length(vWorldPos.xz);
    float fade = 1.0 - smoothstep(6.0, 11.5, dist);
    color = mix(vec3(0.07, 0.08, 0.10), color, fade);

    FragColor = vec4(color, 1.0);
}
