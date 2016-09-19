uniform vec3 u_key_light_direction;
uniform vec3 u_key_light_colour;
uniform vec3 u_fill_light_direction;
uniform vec3 u_fill_light_colour;
uniform vec3 u_back_light_direction;
uniform vec3 u_back_light_colour;
uniform vec3 u_ambient_colour;

uniform vec3 u_colour;

in vec3 f_camera_position;
in vec3 f_camera_normal;

void main() {
    vec3 normal = normalize(f_camera_normal);
    vec3 key = max(0.0, dot(u_key_light_direction, normal)) *
      u_key_light_colour;
    vec3 fill = max(0.0, dot(u_fill_light_direction, normal)) *
      u_fill_light_colour;
    vec3 back = max(0.0, dot(u_back_light_direction, normal)) *
      u_back_light_colour;

    gl_FragColor = vec4(u_colour * (key + fill + back + u_ambient_colour), 1.0);
}
