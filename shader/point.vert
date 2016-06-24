#version 150 core
uniform mat4 transformMatrix;
uniform mat4 projectionMatrix;
in vec4 pv;
in vec4 normal;
in vec2 tex;
out vec4 f_pos;
out vec3 f_normal;
out vec2 f_tex;
void main()
{
  gl_Position = projectionMatrix * transformMatrix * pv;
  f_pos       =                    transformMatrix * pv;
  // TODO: remove 1000.0 with using an uniform variable.
  mat4 rot = mat4(1000.0);
  rot[3][3] = 1.0;
  f_normal    =                    (transformMatrix * rot * normal).xyz;
  f_tex       = tex;
}

