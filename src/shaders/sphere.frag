uniform mat4 u_projection;
uniform vec3 u_light_direction;

flat in vec3 f_camera_position;
flat in float f_radius;
smooth in vec2 f_mapping;

out vec4 output_colour;

void main() {
    vec3 ray = normalize(vec3(f_mapping * f_radius, 0.0) + f_camera_position);

    float quadratic_b = 2.0 * dot(ray, -f_camera_position);
    float quadratic_c = dot(f_camera_position, f_camera_position) - (f_radius * f_radius);

    float discriminant = (quadratic_b * quadratic_b) - (4 * quadratic_c);
    if (discriminant < 0.0) {
        discard;
    }
    float sqrt_discriminant = sqrt(discriminant);
    float solution_1 = (-quadratic_b + sqrt_discriminant) * 0.5;
    float solution_2 = (-quadratic_b - sqrt_discriminant) * 0.5;

    float intersection = min(solution_1, solution_2);
    vec3 point_camera_position = ray * intersection;
    vec3 normal = normalize(point_camera_position - f_camera_position);
    float diffuse = max(0.0, dot(u_light_direction, normal));

    vec4 point_clip_position = u_projection * vec4(point_camera_position, 1.0);
    float depth = point_clip_position.z / point_clip_position.w;

    gl_FragDepth = ((gl_DepthRange.diff * depth) + gl_DepthRange.near + gl_DepthRange.far) / 2.0;
    gl_FragColor = vec4(vec3(1.0, 1.0, 1.0) * diffuse, 1.0);
}
