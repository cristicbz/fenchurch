uniform mat4 u_modelview;
uniform mat4 u_projection;

in vec3 a_position;
in float a_radius;
in int a_corner_index;

flat out vec3 f_camera_position;
flat out float f_radius;
smooth out vec2 f_mapping;

const float g_rectify = 1.5;
const vec2 mappings[4] = vec2[](
    vec2(-1.0, -1.0) * g_rectify,
    vec2(-1.0, 1.0) * g_rectify,
    vec2(1.0, -1.0) * g_rectify,
    vec2(1.0, 1.0) * g_rectify);

void main() {
  vec2 mapping = mappings[a_corner_index];
  vec4 camera_position = u_modelview * vec4(a_position, 1.0);

  f_mapping = mapping;
  f_camera_position = camera_position.xyz;
  f_radius = a_radius;

  gl_Position = u_projection * vec4(camera_position.xy + mapping * a_radius, camera_position.zw);
}
