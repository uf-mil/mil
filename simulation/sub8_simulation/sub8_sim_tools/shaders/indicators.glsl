>thrust_indicator:vertex
#version 120
// uniform mat3 u_orientation;
uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_projection;
uniform vec4 u_color;
uniform vec3 u_light_position;
uniform float u_length_scale;
attribute vec3 a_position;

void main () {
    // Allow length changing within the shader
    vec3 scaled_position = vec3(a_position.xy, a_position.z * u_length_scale);
    gl_Position = u_projection * u_view * u_model * vec4(scaled_position, 1.0);
}

>thrust_indicator:fragment
#version 120
uniform vec4 u_color;

void main () {
    gl_FragColor = u_color;
}