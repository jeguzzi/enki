#version 400

in vec4 vertex;
in vec4 texCoord;
in vec3 vertex_normal;
out vec4 texc;
uniform mat4 proj;
uniform mat4 matrix;
uniform mat3 u_normal_matrix;

struct LightSource {
  vec3 ambient;
  vec3 diffuse;
  vec3 specular;
  vec3 position;
};

uniform LightSource lightSource;

struct LightModel {
  vec3 ambient;
};

uniform LightModel lightModel;

struct Material {
  vec3 emission;
  vec3 ambient;
  vec3 diffuse;
  vec3 specular;
  float shininess;
};

uniform Material material;

out vec3 v_color;

void main(void) {
  vec3 normal = normalize(u_normal_matrix * vertex_normal);
  gl_Position = proj * matrix * vertex;
  vec3 halfVector = normalize(lightSource.position + vec3(0, 0, 1));
  float nDotVP = max(1e-3, dot(normal, normalize(lightSource.position)));
  float nDotHV = max(1e-3, dot(normal, halfVector));
  float pf = mix(0.0, pow(nDotHV, material.shininess), step(0.0, nDotVP));
  vec3 ambient = lightSource.ambient;
  vec3 diffuse = lightSource.diffuse * nDotVP;
  vec3 specular = lightSource.specular * pf;
  vec3 sceneColor = material.emission + material.ambient * lightModel.ambient;
  v_color = clamp(sceneColor + ambient * material.ambient +
                      diffuse * material.diffuse + specular * material.specular,
                  0.0, 1.0);
  texc = texCoord;
}
