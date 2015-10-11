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

>phong:vertex
uniform mat4 u_projection;
uniform mat4 u_view;
uniform mat4 u_model;
uniform mat4 u_normal;

attribute vec3 a_position;
attribute vec3 a_normal;

varying vec3 v_normal;
varying vec3 v_position;

void main(){
    gl_Position = u_projection * u_view * u_model * vec4(a_position, 1.0);
    vec4 vertPos4 = u_view * u_model * vec4(a_position, 1.0);
    v_position = vec3(vertPos4) / vertPos4.w;
    v_normal = vec3(u_normal * vec4(a_normal, 0.0));
}

>phong:fragment
// Shamelessly lifted from wikipedia
uniform vec3 u_light_position;
uniform vec3 u_light_intensity;
uniform vec4 u_color;
uniform vec3 u_specular_color;// = vec3(1.0, 1.0, 1.0);
uniform float u_shininess;// = 16.0;

varying vec3 v_normal;
varying vec3 v_position;

// uniform vec3 ambient_color = vec3(0.1, 0.0, 0.0);
// const float u_shininess = 16.0;
const float screen_gamma = 2.2; // Assume the monitor is calibrated to the sRGB color space

void main() {
    vec3 ambient_color = u_color.rgb;
    vec3 normal = normalize(v_normal);
    vec3 light_direction = normalize(u_light_position - v_position);

    float lambertian = max(dot(light_direction,normal), 0.0);
    float specular = 0.0;

    if(lambertian > 0.0) {

        vec3 view_direction = normalize(-v_position);

        // this is blinn phong
        vec3 half_direction = normalize(light_direction + view_direction);
        float specular_angle = max(dot(half_direction, normal), 0.0);
        specular = pow(specular_angle, u_shininess);
           
    }
  vec3 color_linear = ambient_color +
                     lambertian * u_light_intensity +
                     specular * u_specular_color;
  // apply gamma correction (assume ambient_color, u_light_intensity and u_specular_color
  // have been linearized, i.e. have no gamma correction in them)
  vec3 color_gamma_corrected = pow(color_linear, vec3(1.0 / screen_gamma));
  // use the gamma corrected color in the fragment
  gl_FragColor = vec4(color_gamma_corrected, u_color.a);
}