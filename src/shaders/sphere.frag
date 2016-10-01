uniform mat4 u_projection;

uniform vec3 u_key_light_direction;
uniform vec3 u_key_light_colour;
uniform vec3 u_fill_light_direction;
uniform vec3 u_fill_light_colour;
uniform vec3 u_back_light_direction;
uniform vec3 u_back_light_colour;
uniform vec3 u_ambient_colour;

flat in vec3 f_camera_position;
flat in vec3 f_colour;
flat in float f_quadratic_c;
smooth in vec3 f_ray;

out vec4 output_colour;

void main() {
    vec3 ray = normalize(f_ray);
    float quadratic_b = -2.0 * dot(ray, f_camera_position);
    float discriminant = (quadratic_b * quadratic_b) - (4.0 * f_quadratic_c);
    if (discriminant < 0.0) {
        discard;
    }
    vec3 point_camera_position =
      ray * (-0.5 * (quadratic_b + sqrt(discriminant)));
    vec3 normal = normalize(point_camera_position - f_camera_position);

    vec3 key = max(0.0, dot(u_key_light_direction, normal)) *
      u_key_light_colour;
    vec3 fill = max(0.0, dot(u_fill_light_direction, normal)) *
      u_fill_light_colour;
    vec3 back = max(0.0, dot(u_back_light_direction, normal)) *
      u_back_light_colour;

    vec4 point_clip_position = u_projection * vec4(point_camera_position, 1.0);
    float depth = point_clip_position.z / point_clip_position.w;

    /*gl_FragDepth = ((gl_DepthRange.diff * depth) +*/
    /*                gl_DepthRange.near + gl_DepthRange.far) * 0.5;*/
    gl_FragColor = vec4(f_colour * (key + fill + back + u_ambient_colour), 1.0);
}
