#version 150 core
uniform mat4 transformMatrix;
uniform mat4 projectionMatrix;
in vec4 pv;
in vec4 normal;
out vec4 f_pos;
out vec3 f_normal;
void main()
{
  gl_Position = projectionMatrix * transformMatrix * pv;
  f_pos       =                    transformMatrix * pv;
  f_normal    =                    (transformMatrix * normal).xyz;
}

