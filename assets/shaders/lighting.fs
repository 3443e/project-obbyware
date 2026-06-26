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

uniform float shininess = 24.0;
uniform float specularStrength = 0.08;
uniform vec3  fogColor;
uniform float fogStart = 80.0;
uniform float fogEnd = 200.0;

void main() {
    vec3 baseColor = fragColor.rgb;
    vec3 N = normalize(fragNormal);
    vec3 L = normalize(-lightDir);

    float hemiMix = N.y * 0.5 + 0.5;
    vec3 ambient = mix(ambientDown, ambientUp, hemiMix) * ambientColor;

    float diff = max(dot(N, L), 0.0);
    diff = diff * 0.5 + 0.5;
    diff = diff * diff;
    diff = diff * 1.6 - 0.3;
    diff = clamp(diff, 0.15, 1.0);
    vec3 diffuse = lightColor * diff;

    vec3 V = normalize(cameraPos - fragPosition);
    vec3 H = normalize(L + V);
    float spec = pow(max(dot(N, H), 0.0), shininess);
    vec3 specular = lightColor * spec * specularStrength;

    vec3 lit = baseColor * (ambient + diffuse) + specular;

    // fog
    float dist = length(cameraPos - fragPosition);
    float fogFactor = clamp((dist - fogStart) / (fogEnd - fogStart), 0.0, 1.0);
    fogFactor = fogFactor * fogFactor;  // ease-in
    lit = mix(lit, fogColor, fogFactor);

    float gray = dot(lit, vec3(0.299, 0.587, 0.114));
    lit = mix(vec3(gray), lit, 1.25);

    lit = pow(lit, vec3(1.0/2.2));

    finalColor = vec4(lit, fragColor.a);
}