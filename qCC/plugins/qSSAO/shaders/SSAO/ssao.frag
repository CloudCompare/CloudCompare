//////////////////////////////////////////////////////////////////////////////////////
//
//
//	Screen Space Ambient Occlusion
//
//		C.B.    - 03/05/2008
//		D.G-M.  - 10/22/2010
//		Adapted from notes by Crytek and Inigo Quilez
//
//	IN:	Depth buffer of the scene
//			r = recorded z, in [0:1]
//	OUT:	AO shaded image
//
////////////////////////////////////////////////////////////////////////////////////////

//#extension GL_ARB_draw_buffers : enable

//////////////////////////////////////////////////////////////////////////////////////
//
uniform	sampler2D	s2_Z;
uniform sampler2D	s2_R;
uniform sampler2D	s2_C;

uniform float	R;		//	Radius of neighborhood sphere
uniform	float	F;		//	Amplification of shading
uniform float	Kz;		//	distance attenuation factor
uniform int	    B_REF;		//	if 1 use random reflect of neighbours
uniform vec3	P[256];		//	The Neighbours in unit sphere
const	int	    N = 32;		//	Number of neighbouri
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
	float	z	=	texture2D(s2_Z,gl_TexCoord[0].st).x;

	if(z>0.999)
		return 1.0;

	float	SSAO	=	0.;
	vec3	P0	    =	vec3(gl_TexCoord[0].st,z);
	vec3	PS,REF;
	float	dz,zs;
	for(int c=0;c<N;c++)
	{
		//	TO DO: implement weighting function
		if(B_REF == 0)
		{
			PS	=	P0	+	R * P[c];
		}
		else
		{
            //	reflect vector
			REF	=	texture2D(s2_R,gl_TexCoord[0].st).xyz;
			PS	=	P0	+	R *  reflect(P[c], 2.*(REF-vec3(0.5)) );
		}

		zs      =	texture2D(s2_Z,PS.xy).r;
		dz      =	max(0.0, min(PS.z,1.0)-zs);	// min is to avoid going behind background
		SSAO    +=	dz * F * 1./(1.+Kz*dz*dz) / float(N);

	}
	return	max(1.0-SSAO,0.0);
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////


void main (void)
{
	vec4	C	    =	texture2D(s2_C,gl_TexCoord[0].st).rgba;

	float	SSAO	=	computeSSAO();

	gl_FragData[0]	=	SSAO*C;
}
