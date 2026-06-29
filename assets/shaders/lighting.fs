#version 330
in vec3 fragPosition;
in vec2 fragTexCoord;
in vec3 fragNormal;
in vec4 fragColor;
out vec4 finalColor;

uniform vec3 lightDir;
uniform vec3 lightColor;
uniform vec3 ambientColor;
uniform vec3 ambientUp;
uniform vec3 ambientDown;
uniform vec3 cameraPos;
uniform sampler2D studMap;
uniform float studScale = 0.5;
uniform float hasStuds = 0.0;
uniform float studStrength = 0.4;

void main() {
    vec3 baseColor = fragColor.rgb;

    vec4 studTex = texture(studMap, fragTexCoord * studScale);

    float studLum = dot(studTex.rgb, vec3(0.299, 0.587, 0.114));
    float studSignal = mix(0.5, studLum, studTex.a);

    float studShade = 1.0 + (studSignal - 0.5) * studStrength * 2.0;
    baseColor *= mix(1.0, studShade, hasStuds);

    vec3 N = normalize(fragNormal);
    vec3 L = normalize(-lightDir);
    vec3 V = normalize(cameraPos - fragPosition);

    float hemiMix = N.y * 0.5 + 0.5;
    vec3 ambient = mix(ambientDown, ambientUp, hemiMix) * ambientColor;

    float ndotl = dot(N, L);
    float diff = ndotl * 0.5 + 0.5;
    diff = diff * diff;
    diff = diff * 1.3 - 0.2;
    diff = clamp(diff, 0.2, 0.95);
    vec3 diffuse = lightColor * diff;

    float fill = max(-ndotl, 0.0) * 0.18;
    vec3 fillColor = mix(ambientUp, ambientDown, 0.5) * fill;

    vec3 H = normalize(L + V);
    float spec = pow(max(dot(N, H), 0.0), 24.0);
    vec3 specular = lightColor * spec * 0.08;

    float rim = 1.0 - max(dot(N, V), 0.0);
    rim = pow(rim, 3.0) * 0.25;
    vec3 rimColor = mix(ambientDown, lightColor, 0.5) * rim;

    float pseudoAO = mix(0.85, 1.0, hemiMix);

    vec3 lit = baseColor * (ambient + diffuse + fillColor) * pseudoAO + specular + rimColor;

    float dist = length(cameraPos - fragPosition);
    float fogFactor = clamp((dist - 80.0) / (200.0 - 80.0), 0.0, 1.0);
    fogFactor = fogFactor * fogFactor;
    vec3 fogColor = mix(vec3(0.50, 0.38, 0.42), vec3(0.60, 0.45, 0.48), N.y * 0.5 + 0.5);
    lit = mix(lit, fogColor, fogFactor);

    //float gray = dot(lit, vec3(0.299, 0.587, 0.114));
    //lit = mix(vec3(gray), lit, 1.25);

    lit = pow(lit, vec3(1.0/2.2));
    finalColor = vec4(lit, fragColor.a);
}