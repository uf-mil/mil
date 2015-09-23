>thrust_indicator:vertex
#version 120
uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_projection;
uniform vec3 u_color;
uniform float length_scale;
attribute vec3 a_position;

void main () {
    // Allow length changing within the shader
    vec3 scaled_position = vec3(a_position.xy, a_position.z * length_scale);
    gl_Position = u_projection * u_view * u_model * vec4(scaled_position, 1.0);
}

>thrust_indicator:fragment
#version 120
uniform vec3 u_color;

void main () {
    gl_FragColor = vec4(u_color, 1.0);
}