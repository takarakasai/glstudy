#version 150 core
in vec3 diffuseColor;
out vec4 fc;
//varying vec3 diffuseColor;
void main()
{
 fc = vec4(diffuseColor, 1.0);
 //gl_FragColor = vec4(diffuseColor, 1.0);
 //gl_FragColor.rgb = mix(vec3(0.8, 0.8, 0.8), vec3(0.1), 0.4);
 //gl_FragColor.a = 1.0;
}
