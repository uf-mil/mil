// [1] http://www.labri.fr/perso/nrougier/teaching/opengl/scripts/lighted-cube.py
// [2] https://en.wikipedia.org/wiki/Blinn%E2%80%93Phong_shading_model#OpenGL_Shading_Language_code_sample

>lambert:vertex
uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_projection;
uniform vec4 u_color;


attribute vec3 a_position;
attribute vec3 a_normal;
attribute vec4 a_color;

varying vec3 v_position;
varying vec3 v_normal;
varying vec4 v_color;

void main()
{
    v_normal = a_normal;
    v_position = a_position;
    v_color = u_color;
    gl_Position = u_projection * u_view * u_model * vec4(a_position, 1.0);
}

>lambert:fragment
uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_normal;

uniform vec3 u_light_intensity;
uniform vec3 u_light_position;

varying vec3 v_position;
varying vec3 v_normal;
varying vec4 v_color;

void main()
{
    // Calculate normal in world coordinates
    vec3 normal = normalize(u_normal * vec4(v_normal, 1.0)).xyz;

    // Calculate the location of this fragment (pixel) in world coordinates
    vec3 position = vec3(u_view * u_model * vec4(v_position, 1));

    // Calculate the vector from this pixels surface to the light source
    vec3 surface_to_light = u_light_position - position;

    // Calculate the cosine of the angle of incidence (brightness)
    float brightness = dot(normal, surface_to_light) /
                      (length(surface_to_light) * length(normal));
    brightness = clamp(brightness, 0.0, 1.0);

    gl_FragColor = v_color * brightness * vec4(u_light_intensity, 1.0);
}

