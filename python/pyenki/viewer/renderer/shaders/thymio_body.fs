#version 400

uniform sampler2D u_texture;
uniform sampler2D led0;
uniform sampler2D led1;
uniform sampler2D led2;
uniform vec4 led_colors[27];

in vec4 texc;
in vec3 v_color;
out vec4 fragColor;

struct LED {
  vec2 center;
  vec2 size;
};

const LED top = LED(vec2(0.5, 0.5), vec2(1.0, 1.0));
const LED bottom_left_0 = LED(vec2(0.6074, 0.8159), vec2(0.1133, 0.2939));
const LED bottom_left_1 = LED(vec2(0.7309, 0.2163), vec2(0.1885, 0.1396));
const LED bottom_right = LED(vec2(0.6636, 0.5703), vec2(0.2236, 0.1875));
const LED leds[24] = LED[24](LED(vec2(0.0980, 0.2360), vec2(0.0350, 0.0450)),
                             LED(vec2(0.1740, 0.2360), vec2(0.0350, 0.0450)),
                             LED(vec2(0.1360, 0.1980), vec2(0.0450, 0.0350)),
                             LED(vec2(0.1360, 0.2740), vec2(0.0450, 0.0350)),
                             LED(vec2(0.0310, 0.2360), vec2(0.0400, 0.0800)),
                             LED(vec2(0.0657, 0.3063), vec2(0.0650, 0.0650)),
                             LED(vec2(0.1360, 0.3410), vec2(0.0800, 0.0400)),
                             LED(vec2(0.2063, 0.3063), vec2(0.0650, 0.0650)),
                             LED(vec2(0.2410, 0.2360), vec2(0.0400, 0.0800)),
                             LED(vec2(0.2063, 0.1657), vec2(0.0650, 0.0650)),
                             LED(vec2(0.1360, 0.1310), vec2(0.0800, 0.0400)),
                             LED(vec2(0.0657, 0.1657), vec2(0.0650, 0.0650)),
                             LED(vec2(0.5586, 0.9541), vec2(0.0600, 0.0600)),
                             LED(vec2(0.5644, 0.8721), vec2(0.0600, 0.0600)),
                             LED(vec2(0.5673, 0.7559), vec2(0.0600, 0.0600)),
                             LED(vec2(0.5693, 0.6944), vec2(0.0600, 0.0600)),
                             LED(vec2(0.5664, 0.5742), vec2(0.0600, 0.0600)),
                             LED(vec2(0.5615, 0.4815), vec2(0.0600, 0.0600)),
                             LED(vec2(0.8759, 0.3711), vec2(0.0600, 0.0600)),
                             LED(vec2(0.5449, 0.3711), vec2(0.0600, 0.0600)),
                             LED(vec2(0.7163, 0.1572), vec2(0.0771, 0.0878)),
                             LED(vec2(0.7163, 0.1572), vec2(0.0771, 0.0878)),
                             LED(vec2(0.7974, 0.6250), vec2(0.0910, 0.0910)),
                             LED(vec2(0.7773, 0.5664), vec2(0.0400, 0.0400)));

bool contained_in_led(LED led, vec2 position) {
  return all(lessThanEqual(abs(led.center - position), 0.5 * led.size));
}

vec4 transfer(vec4 color) { return pow(color, vec4(0.35, 0.30, 0.40, 1.0)); }

vec4 compute_color(vec4 texture_color, vec4 led_color) {
  return transfer(texture_color * led_color);
}

void main(void) {
  vec2 p = texc.st;
  vec4 led0_color = texture(led0, p);
  vec4 led1_color = texture(led1, p);
  vec4 led2_color = texture(led2, p);
  vec3 color = texture(u_texture, p).xyz * v_color;
  // vec4 led_color = vec4(0.0);
  if (contained_in_led(top, p)) {
    vec4 led_color = compute_color(led0_color, led_colors[0]);
    color = mix(color, led_color.xyz, led_color.a);
  }
  if (contained_in_led(bottom_left_0, p) ||
      contained_in_led(bottom_left_1, p)) {
    vec4 led_color = compute_color(led1_color, led_colors[1]);
    color = mix(color, led_color.xyz, led_color.a);
  }
  if (contained_in_led(bottom_right, p)) {
    vec4 led_color = compute_color(led1_color, led_colors[2]);
    color = mix(color, led_color.xyz, led_color.a);
  }
  for (int i = 0; i < 24; i++) {
    if (contained_in_led(leds[i], p)) {
      vec4 led_color = compute_color(led2_color, led_colors[i + 3]);
      color = mix(color, led_color.xyz, led_color.a);
    }
  }
  // led_color = clamp(led_color, 0.0, 1.0);
  // gl_FragColor = mix(color, led_color, led_color.a);
  fragColor = vec4(color, 1.0);
}