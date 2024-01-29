#version 330 core
layout (points) in;
layout (line_strip, max_vertices = 2) out;

in Vertex
{
  vec3 normal;
} vertex[];

uniform float normalLength;
uniform mat4 modelViewProjectionMatrix;


void main(void)
{
	vec3 P = gl_in[0].gl_Position.xyz;
    vec3 N = vertex[0].normal;

    gl_Position = modelViewProjectionMatrix * vec4(P, 1.0);
    EmitVertex();

	gl_Position = modelViewProjectionMatrix * vec4(P + N * normalLength, 1);
    EmitVertex();
    
    EndPrimitive();
}