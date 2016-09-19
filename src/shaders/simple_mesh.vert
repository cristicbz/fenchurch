uniform mat4 u_modelview;
uniform mat4 u_normal_matrix;
uniform mat4 u_projection;

in vec3 position;
in vec3 normal;

out vec3 f_camera_position;
out vec3 f_camera_normal;

void main() {
  vec4 modelview_position = u_modelview * vec4(position, 1.0);
  f_camera_position = vec3(modelview_position);
  f_camera_normal = normalize(vec3(u_normal_matrix * vec4(normal, 1.0)));
  gl_Position = u_projection * modelview_position;
}
