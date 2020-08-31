#version 330 core

struct Material {
    float shininess;
};
struct Light {
    vec3 direction;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

out vec4 FragColor;

in vec2 TexCoords;
in vec3 Normal;
in vec3 FragPos;

uniform sampler2D texture_diffuse1;
uniform sampler2D texture_specular1;
uniform vec3 viewPos;
uniform Material material;
uniform Light light;

void main()
{
    // Ambient
    vec3 ambient = light.ambient * texture(texture_diffuse1, TexCoords).rgb;

    // Diffuse
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(-light.direction);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = light.diffuse * diff * texture(texture_diffuse1, TexCoords).rgb;

    // Specular
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    vec3 specular = light.specular * spec * texture(texture_specular1, TexCoords).rgb;

    // Shading
    vec3 result = ambient + diffuse + specular;
    result = clamp(result, 0.0, 1.0);

    FragColor = vec4(result, 1.0);
}
