attribute highp vec3 vertexIn;
attribute highp vec3 normal;

out Vertex
{
  vec3 normal;
} vertex;

void main(void)
{
	gl_Position = vec4(vertexIn, 1.0);
	vertex.normal = normal;
}
