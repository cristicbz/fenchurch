#extension GL_EXT_gpu_shader4 : enable

uniform mat4 u_projection;

layout(points) in;
layout(triangle_strip, max_vertices=4) out;

in VertexData {
  vec3 camera_position;
  float radius;
} g_vertices[];

out FragmentData {
  flat vec3 f_camera_position;
  flat float f_radius;
  smooth vec2 f_mapping;
};

const float g_size_rectify = 1.5;

void emit(vec2 mapping) {
}

void main() {
  vec2 mapping;
  vec4 camera_corner_position;

  mapping = vec2(-1.0, -1.0);
  f_mapping = mapping * g_size_rectify;
  f_camera_position = g_vertices[0].camera_position;
  f_radius = g_vertices[0].radius;
  camera_corner_position = vec4(g_vertices[0].camera_position, 1.0);
  camera_corner_position.xy += mapping * g_vertices[0].radius * g_size_rectify;
  gl_Position = u_projection * camera_corner_position;
  EmitVertex();

  mapping = vec2(-1.0, 1.0);
  f_mapping = mapping * g_size_rectify;
  f_camera_position = g_vertices[0].camera_position;
  f_radius = g_vertices[0].radius;
  camera_corner_position = vec4(g_vertices[0].camera_position, 1.0);
  camera_corner_position.xy += mapping * g_vertices[0].radius * g_size_rectify;
  gl_Position = u_projection * camera_corner_position;
  EmitVertex();

  mapping = vec2(1.0, -1.0);
  f_mapping = mapping * g_size_rectify;
  f_camera_position = g_vertices[0].camera_position;
  f_radius = g_vertices[0].radius;
  camera_corner_position = vec4(g_vertices[0].camera_position, 1.0);
  camera_corner_position.xy += mapping * g_vertices[0].radius * g_size_rectify;
  gl_Position = u_projection * camera_corner_position;
  EmitVertex();

  mapping = vec2(1.0, 1.0);
  f_mapping = mapping * g_size_rectify;
  f_camera_position = g_vertices[0].camera_position;
  f_radius = g_vertices[0].radius;
  camera_corner_position = vec4(g_vertices[0].camera_position, 1.0);
  camera_corner_position.xy += mapping * g_vertices[0].radius * g_size_rectify;
  gl_Position = u_projection * camera_corner_position;
  EmitVertex();
}
