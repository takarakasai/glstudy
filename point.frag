#version 150
//in vec4 gl_FragCoord;
out vec4 fc;

varying vec3 diffuseColor;

void main()
{
 gl_FragColor = vec4(diffuseColor, 1.0);
 //if (gl_FragCoord.z < 1.0) {
 //  gl_FragColor.rgb = mix(vec3(0.8, 0.8, 0.8), vec3(0.1), 0.4);
 //  gl_FragColor.a = 1.0;
 //} else {
 //  gl_FragColor.rgb = vec3(gl_FragCoord.x/640, gl_FragCoord.y/480, gl_FragCoord.z);
 //  //gl_FragColor.a = gl_FragCoord.z;
 //  gl_FragColor.a = 1.0;
 //}
}
