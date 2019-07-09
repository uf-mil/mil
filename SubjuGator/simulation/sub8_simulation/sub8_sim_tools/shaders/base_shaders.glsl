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
uniform vec3 u_lights[2];

uniform vec4 u_color;
uniform vec3 u_specular_color;
uniform float u_shininess;
uniform int u_numLights = 1;


varying vec3 v_normal;
varying vec3 v_position;

const float screen_gamma = 2.2; // Assume the monitor is calibrated to the sRGB color space

void main() {
    vec3 normal = normalize(v_normal);
    vec3 ambient_color = u_color.rgb;
    vec3 color_linear = ambient_color; 
     
    int i;
    for(i = 0; i!= u_numLights * 2; i += 2){
        vec3 lightPos = u_lights[i];
        vec3 intensity = u_lights[i+1];
        vec3 speccolor = u_specular_color;
        vec3 light_direction = normalize(lightPos - v_position);
        float lambertian = max(dot(light_direction,normal), 0.0);
        float specular = 0.0;

        if(lambertian > 0.0) {

             vec3 view_direction = normalize(-v_position);

                // this is blinn phong
            vec3 half_direction = normalize(light_direction + view_direction);
            float specular_angle = max(dot(half_direction, normal), 0.0);
            specular = pow(specular_angle, u_shininess);

        }
        color_linear += (intensity * lambertian) +
                            (specular * speccolor);
       
    }

  vec3 color_gamma_corrected = pow(color_linear, vec3(1.0 / screen_gamma));
  // use the gamma corrected color in the fragment
  gl_FragColor = vec4(color_gamma_corrected, u_color.a);
  }