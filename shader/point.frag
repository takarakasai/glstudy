#version 150 core
in vec4 f_pos;
in vec3 f_normal;
in vec2 f_tex;
uniform vec4 materialColor;
uniform sampler2D texDiff;
out vec4 fc;
void main()
{
  //fc = vec4(diffuseColor, 1.0);

  vec4 lightColor  = vec4(1.0, 1.0, 1.0, 1.0);
  vec4 lightPos    = vec4(5.0, 5.0, 5.0, 1.0);
  vec4 objectColor = materialColor;
  vec3 viewPos     = vec3(2.0, 0.0, 0.0);

  // ambient
  float ambientStrength = 0.5f;
  vec3 ambient = ambientStrength * lightColor.xyz;

  // Diffuse
  float diffuseStrength = 1.0f;
  vec3 norm = normalize(f_normal);
  vec3 lightDir = normalize(lightPos - f_pos).xyz;
  float diff = max(dot(norm, lightDir), 0.0);
  vec3 diffuse = diffuseStrength * diff * lightColor.xyz;

  // TODO: specular
  vec3 view = -normalize(f_pos.xyz);
  vec3 halfv = normalize(lightDir + view);
  vec3 specular = pow(max(dot(normalize(halfv), normalize(lightDir)), 0.0), 20/*shininess*/) * lightColor.xyz;

  vec3 xyz = (ambient + diffuse + specular) * objectColor.xyz;
  //fc = vec4(xyz, objectColor.a);
  // mix(x, y, a) == x*(1-a) + y*a

  vec4 texmap = texture(texDiff, f_tex);
  vec4 objmap = vec4(xyz, objectColor.a);
  if (objectColor.a == 0) {
    fc = mix(objmap, texmap, texmap.a);
  } else {
    //fc = mix(texmap, objmap, objmap.a);
    fc = vec4(xyz, objectColor.a);
  }
  //fc = vec4(xyz, objectColor.a);
}

