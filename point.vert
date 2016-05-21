#version 150 core
uniform vec2 size;
uniform float dpm;
uniform mat4 projectionMatrix;
in vec4 pv;
//varying vec3 diffuseColor;
out vec3 diffuseColor;
void main()
{
  gl_Position = projectionMatrix * pv;
  diffuseColor = vec3(gl_Position.z);
}

