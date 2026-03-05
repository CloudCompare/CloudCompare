//////////////////////////////////////////////////////////////////////////////////////
//
//	Screen Space Ambient Occlusion
//
//		C.B.    - 03/05/2008
//		D.G-M.  - 10/22/2010
//		Adapted from notes by Crytek and Inigo Quilez
//
//	IN: Depth buffer of the scene
//      r = recorded z, in [0:1]
//	OUT: AO shaded image
//
////////////////////////////////////////////////////////////////////////////////////////

//#extension GL_ARB_draw_buffers : enable

//////////////////////////////////////////////////////////////////////////////////////
//
uniform sampler2D s_Depth;
uniform sampler2D s_Reflect;
uniform sampler2D s2_Colors;

uniform float R;			// Radius of neighborhood sphere
uniform float F;			// Amplification of shading
uniform float Kz;			// Distance attenuation factor
uniform int   UseReflect;	// If 1 use random reflect of neighbours
uniform vec3  P[32];		// The neighbours in unit sphere
const   int	  N = 32;		// Number of neighbours
//
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
//
//	SSAO
//
//	First implementation: use projected coordinates
//
float computeSSAO()
{
	float z = texture2D(s_Depth, gl_TexCoord[0].st).r;

	if (z > 0.999)
		return 1.0;

	float SSAO = 0.0;
	vec3  P0   = vec3(gl_TexCoord[0].xy, z);
	vec3  PS, REF;
	float dz, zNeighbor;
	for (int c = 0; c < N; c++)
	{
		// TODO: implement weighting function
		if (UseReflect == 0)
		{
			PS = P0 + R * P[c];
		}
		else
		{
            // reflect vector
			REF = texture2D(s_Reflect, gl_TexCoord[0].st).xyz;
			PS = P0 + R * reflect(P[c], 2.0*(REF - vec3(0.5))); // map the points from [0;1] to [-1;1]
		}

		zNeighbor = texture2D(s_Depth, PS.xy).r;
		dz = max(0.0, min(PS.z, 1.0) - zNeighbor);	// min is to avoid going behind background
		SSAO += dz / (1.0 + Kz*dz*dz);
	}

	SSAO = (SSAO*F) / N;
	return max(1.0 - SSAO, 0.0);
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////


void main(void)
{
	vec4 C = texture2D(s2_Colors, gl_TexCoord[0].st).rgba;
	float SSAO = computeSSAO();
	gl_FragData[0] = vec4(SSAO * C.rgb, C.a);
}
