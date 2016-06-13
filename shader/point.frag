#version 150 core
in vec3 f_pos;
in vec3 f_normal;
out vec4 fc;
void main()
{
  //fc = vec4(diffuseColor, 1.0);

  vec3 lightColor  = vec3(1.0, 1.0, 1.0);
  vec3 lightPos    = vec3(0.0, 0.0, 5.0);
  vec4 objectColor = vec4(0.0, 0.5, 0.0, 0.5);
  vec3 viewPos     = vec3(2.0, 0.0, 0.0);

  // ambient
  float ambientStrength = 0.5f;
  vec3 ambient = ambientStrength * lightColor;

  // Diffuse
  float diffuseStrength = 1.0f;
  vec3 lightDir = normalize(lightPos - f_pos);
  float diff = max(dot(f_normal, lightDir), 0.0);
  vec3 diffuse = diffuseStrength * diff * lightColor;

  // TODO: specular

  vec3 xyz = (ambient + diffuse) * objectColor.xyz;
  fc = vec4(xyz, objectColor.a);
}
