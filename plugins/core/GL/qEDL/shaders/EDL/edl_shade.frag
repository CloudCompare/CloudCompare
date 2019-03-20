//////////////////////////////////////////////////////////////////////////////////////
//
//
//	EyeDome Lighting - oriented light version
//
//		C.B.    - 04/23/2008
//		D.G-M.  - 10/21/2010
//		D.G-M.  - 02/17/2014
//
//	IN:	    Depth buffer of the scene
//			r (red component) = recorded z
//	OUT:	EDL shaded image
//
///////////////////////////////////////////////////////////////////////////////////////
//#extension GL_ARB_draw_buffers : enable
//#version 110

/**************************************************/
uniform	sampler2D	s1_color;
uniform	sampler2D	s2_depth;

uniform float		Pix_scale;		    //	(relative) pixel scale in image
uniform vec2		Neigh_pos_2D[8];	//	array of neighbors (2D positions)
uniform float		Exp_scale;			//	exponential scale factor (for computed AO)

uniform float		Zm;					//	minimal depth in image
uniform float		ZM;					//	maximal depth in image

uniform float		Sx;
uniform float		Sy;

uniform float		Zoom;				// image display zoom (so as to always use - approximately - the same pixels)
uniform int			PerspectiveMode;	// whether perspective mode is enabled (1) or not (0) - for z-Buffer compensation

uniform vec3		Light_dir;
/**************************************************/


//  Obscurance (pseudo angle version)
//	z		neighbour relative elevation
//	dist	distance to the neighbourx
float obscurance(float z, float dist)
{
	return max(0.0, z) / dist;
}

float ztransform(float z_b)
{
	if (PerspectiveMode == 1)
	{
		//'1/z' depth-buffer transformation correction
		float z_n = 2.0 * z_b - 1.0;
		z_b = 2.0 * Zm / (ZM + Zm - z_n * (ZM - Zm));
		z_b = z_b * ZM / (ZM - Zm);
	}

	return clamp(1.0 - z_b, 0.0, 1.0);
}

float computeObscurance(float depth, float scale)
{
	// Light-plane point
	vec4 P = vec4( Light_dir.xyz , -dot(Light_dir.xyz,vec3(0.0,0.0,depth)) );

	float sum = 0.0;

	// contribution of each neighbor
	for(int c=0; c<8; c++)
	{
		vec2 N_rel_pos = scale * Zoom / vec2(Sx,Sy) * Neigh_pos_2D[c];	//neighbor relative position
		vec2 N_abs_pos = gl_TexCoord[0].st + N_rel_pos;					//neighbor absolute position
		
		//version with background shading
		float Zn = ztransform( texture2D(s2_depth,N_abs_pos).r );		//depth of the real neighbor
		float Znp = dot( vec4(N_rel_pos, Zn, 1.0) , P );				//depth of the in-plane neighbor

		sum += obscurance( Znp, scale );
	}

	return	sum;
}

void main (void)
{
	//ambient occlusion
    vec3 rgb = texture2D(s1_color,gl_TexCoord[0].st).rgb;
	float depth = ztransform( texture2D(s2_depth,gl_TexCoord[0].st).r );

	if( depth > 0.01 )
	{
		float f = computeObscurance(depth, Pix_scale);
		f = exp(-Exp_scale*f);

		gl_FragData[0]	=	vec4(f*rgb, 1.0);
	}
	else
	{
		gl_FragData[0]	=	vec4(rgb, 1.0);
	}
}
