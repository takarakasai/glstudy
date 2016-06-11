#version 150 core
uniform vec2 size;
uniform float dpm;
uniform mat4 transformMatrix;
uniform mat4 projectionMatrix;
in vec4 pv;
in vec4 normal;
out vec3 f_pos;
out vec3 f_normal;
void main()
{
  gl_Position = projectionMatrix * transformMatrix * pv;
  f_pos       =                   (transformMatrix * pv).xyz;
  f_normal    =                   (transformMatrix * normal).xyz;
}

