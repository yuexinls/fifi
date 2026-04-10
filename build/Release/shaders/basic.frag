#version 450 core

in vec3 vNormal;
in vec3 vFragPos;

uniform vec3 uColor;
uniform vec3 uLightPos;
uniform vec3 uViewPos;

out vec4 FragColor;

void main() {
    vec3 norm     = normalize(vNormal);
    vec3 lightDir = normalize(uLightPos - vFragPos);
    vec3 viewDir  = normalize(uViewPos  - vFragPos);

    // ambient
    vec3 ambient  = 0.15 * uColor;

    // diffuse (lambertian)
    float diff    = max(dot(norm, lightDir), 0.0);
    vec3 diffuse  = diff * uColor;

    // specular (blinn phonk-.. phong sorry)
    vec3 halfway  = normalize(lightDir + viewDir);
    float spec    = pow(max(dot(norm, halfway), 0.0), 64.0);
    vec3 specular = spec * vec3(1.0);

    FragColor = vec4(ambient + diffuse + specular, 1.0);
}


