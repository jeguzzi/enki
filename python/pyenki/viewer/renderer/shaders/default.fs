#version 400

uniform sampler2D u_texture;
uniform bool u_has_texture;
in vec4 texc;
in vec3 v_color;
out vec4 fragColor;

void main(void) {
  if (u_has_texture) {
    fragColor = texture(u_texture, texc.st) * vec4(v_color, 1.0);
  } else {
    fragColor = vec4(v_color, 1.0);
  }
}