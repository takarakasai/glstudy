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
  f_normal    =                    (transformMatrix * normal).xyz;
  f_tex       = tex;
}

