uniform mat4 u_modelview;
uniform mat4 u_projection;
uniform sampler1D u_positions;
uniform sampler1D u_radii;
uniform sampler1D u_colours;

flat out vec3 f_camera_position;
flat out float f_quadratic_c;
flat out vec3 f_colour;
smooth out vec3 f_ray;

const float g_rectify = 1.7;
const vec2 mappings[6] = vec2[](
    vec2(-1.0, -1.0) * g_rectify,
    vec2(1.0, -1.0) * g_rectify,
    vec2(-1.0, 1.0) * g_rectify,

    vec2(1.0, -1.0) * g_rectify,
    vec2(1.0, 1.0) * g_rectify,
    vec2(-1.0, 1.0) * g_rectify);


void main() {
  vec2 mapping = mappings[gl_VertexID % 6];
  int sphere_index = gl_VertexID / 6;
  vec3 a_position = texelFetch(u_positions, sphere_index, 0).rgb;
  vec3 a_colour = texelFetch(u_colours, sphere_index, 0).rgb;
  float a_radius = texelFetch(u_radii, sphere_index, 0).r;

  vec4 camera_position = u_modelview * vec4(a_position, 1.0);

  f_camera_position = camera_position.xyz;
  f_ray = vec3(mapping * a_radius, 0.0) + f_camera_position;
  f_quadratic_c = (dot(f_camera_position, f_camera_position) -
                   (a_radius * a_radius));
  f_colour = a_colour;

  gl_Position = u_projection * vec4(camera_position.xy + mapping * a_radius, camera_position.zw);
}
