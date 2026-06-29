#version 330

in vec3 fragPosition;
in vec2 fragTexCoord;
in vec3 fragNormal;
in vec4 fragColor;
out vec4 finalColor;

uniform vec3 lightDir;        // direction light travels (we negate for L)
uniform vec3 lightColor;
uniform vec3 ambientColor;    // global ambient multiplier
uniform vec3 ambientUp;       // sky color
uniform vec3 ambientDown;     // ground color
uniform vec3 cameraPos;

uniform sampler2D studMap;
uniform float studScale    = 0.5;
uniform float studStrength = 0.4;

void main() {
    vec3 N = normalize(fragNormal);
    vec3 L = normalize(-lightDir);
    vec3 V = normalize(cameraPos - fragPosition);
    vec3 H = normalize(L + V);

    vec3 baseColor = fragColor.rgb;

    vec4 studTex = texture(studMap, fragTexCoord * studScale);
    float studLum  = dot(studTex.rgb, vec3(0.299, 0.587, 0.114));
    // studTex.a = 1 for real stud texture, 0 for uh yeha

    float studSignal = mix(0.5, studLum, studTex.a);
    float studShade = 1.0 + (studSignal - 0.5) * studStrength * 2.0;
    baseColor *= studShade;

    float hemiMix = N.y * 0.5 + 0.5;
    vec3  ambient = mix(ambientDown, ambientUp, hemiMix) * ambientColor;
    float pseudoAO = mix(0.88, 1.0, hemiMix);

    // - sun diffuse
    float ndotl = dot(N, L);
    float wrap = ndotl * 0.5 + 0.5;
    float diff = wrap * wrap;
    diff = clamp(diff * 1.15 - 0.05, 0.0, 1.0);
    vec3 diffuse = lightColor * diff;

    float fillAmt = max(-ndotl, 0.0) * 0.20;
    vec3 fillCol = mix(ambientUp, ambientDown, 0.5);
    vec3 fill = fillCol * fillAmt;

    float specPow = 32.0;
    float spec = pow(max(dot(N, H), 0.0), specPow);
    vec3 specular = lightColor * spec * 0.12;

    float NdotV = max(dot(N, V), 0.0);
    float rim = pow(1.0 - NdotV, 4.0) * 0.35;
    vec3  rimCol  = mix(ambientUp, lightColor, 0.4) * rim;

    vec3 lit = baseColor * (ambient + diffuse + fill) * pseudoAO + specular + rimCol;

    // gamma
    lit = lit / (lit + vec3(1.0));
    lit = pow(lit, vec3(1.0 / 2.2));

    finalColor = vec4(lit, fragColor.a);
}