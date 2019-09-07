>sonar:vertex
#version 120
uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_projection;
uniform vec3 u_color;
uniform float u_maxdepth;
attribute vec3 a_position;
varying float depth;

void main () {
    // Allow length changing within the shader
    gl_Position = u_projection * u_view * u_model * vec4(a_position, 1.0);
    depth = clamp(length(gl_Position) / u_maxdepth, 1.0);
}

>sonar:fragment
#version 120
uniform vec3 u_color;
varying float depth;

void main () {
    gl_FragColor = vec4(depth, depth, depth, 1.0);
}