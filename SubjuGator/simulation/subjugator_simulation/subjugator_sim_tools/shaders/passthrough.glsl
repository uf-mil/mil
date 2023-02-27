>mesh:vertex

#version 120
uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_projection;
uniform mat4 u_normal;
uniform vec4 u_color;

attribute vec3 a_position;

void main () {
    gl_Position = u_projection * u_view * u_model * vec4(a_position, 1.0);
}

>mesh:fragment

#version 120
uniform vec4 u_color;

void main () {
    gl_FragColor = u_color;
}

>debug:vertex

#version 120
uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_projection;
attribute vec3 a_position;
// attribute vec2 texcoord;
varying vec3 v_col;

void main()
{
    gl_Position = u_projection * u_view * u_model * vec4(a_position, 1.0);
    v_col = abs(clamp(a_position, -1, 1));
    // v_texcoord = texcoord;
}

>debug:fragment
#version 120
// varying vec2 v_texcoord;
varying vec3 v_col;
void main()
{
    // float r = texture2D(texture, v_texcoord).r;
    // gl_FragColor = vec4(0.2, v_texcoord.x, v_texcoord.y, 1.0);
    gl_FragColor = vec4(v_col, 1.0);
    // gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);
}
