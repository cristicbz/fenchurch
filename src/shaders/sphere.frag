uniform mat4 u_projection;
uniform vec3 u_light_direction;

flat in vec3 f_camera_position;
flat in vec3 f_colour;
flat in float f_quadratic_c;
smooth in vec3 f_ray;

out vec4 output_colour;

void main() {
    vec3 ray = normalize(f_ray);
    float quadratic_b = 2.0 * dot(ray, -f_camera_position);
    float discriminant = (quadratic_b * quadratic_b) - (4.0 * f_quadratic_c);
    if (discriminant < 0.0) {
        discard;
    }
    vec3 point_camera_position =
      ray * (-0.5 * (quadratic_b + sqrt(discriminant)));
    vec3 normal = normalize(point_camera_position - f_camera_position);
    float diffuse = max(0.025, dot(u_light_direction, normal));

    vec4 point_clip_position = u_projection * vec4(point_camera_position, 1.0);
    float depth = point_clip_position.z / point_clip_position.w;

    gl_FragDepth = ((gl_DepthRange.diff * depth) +
                    gl_DepthRange.near + gl_DepthRange.far) * 0.5;
    gl_FragColor = vec4(f_colour * diffuse, 1.0);
}
