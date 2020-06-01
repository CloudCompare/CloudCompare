//////////////////////////////////////////////////////////////////////////////////////
//
//
//	EyeDome Lighting	-	Compositing
//
//		C.B.    - 04/23/2008
//		D.G-M.  - 10/21/2010
//
//	IN:
//		s2_I1		-	full scale shading image
//		s2_I2		-	half-size shading image
//		s2_I4		-	quarter-size shading image
//		s2_D		-	depth image
//	OUT:
//		composited image
//
//////////////////////////////////////////////////////////////////////////////////////
//#extension GL_ARB_draw_buffers : enable

/**************************************************/
uniform	sampler2D	s2_I1;	//	X1 scale
uniform sampler2D	s2_I2;	//	X2 scale
uniform sampler2D	s2_I4;	//	X4 scale
uniform sampler2D	s2_D;	// initial depth texture
//
uniform float		A0;
uniform float		A1;
uniform float		A2;
/**************************************************/

void main (void)
{
	float d = texture2D(s2_D,gl_TexCoord[0].st).r;
	if( d > 0.999)
	{
		gl_FragData[0].rgb = texture2D(s2_I1,gl_TexCoord[0].st).rgb;
		gl_FragData[0].a = 1.;
		return;
	}

    //color version
	vec3 C1 = texture2D(s2_I1,gl_TexCoord[0].st).rgb;
	vec3 C2 = texture2D(s2_I2,gl_TexCoord[0].st).rgb;
	vec3 C4 = texture2D(s2_I4,gl_TexCoord[0].st).rgb;
	vec3 C = (A0*C1 + A1*C2 + A2*C4) / (A0+A1+A2);

	gl_FragData[0] = vec4(C.x,C.y,C.z,1.);
}
