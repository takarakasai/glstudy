#version 150 core
//uniform float aspect;
uniform vec2 size;
uniform float dpm;
uniform mat4 projectionMatrix;
in vec4 pv;

varying vec3 diffuseColor;

void main()
{
  //gl_Position = pv * vec4(1.0 * dpm / (size/2.0), 1.0, 1.0);
  gl_Position = projectionMatrix * pv;
  diffuseColor = vec3(gl_Position.z);
}

