#version 150 core
uniform vec2 size;
uniform float dpm;
uniform mat4 transformMatrix;
uniform mat4 projectionMatrix;
in vec4 pv;
out vec3 diffuseColor;
void main()
{
  gl_Position = projectionMatrix * transformMatrix * pv;
  diffuseColor = vec3(gl_Position.z);
}

