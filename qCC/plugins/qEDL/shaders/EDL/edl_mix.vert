//////////////////////////////////////////////////////////////////////////////////////
//
//
//	EyeDome Lighting	-	Compositing
//
//		C.B. - 23 avril 2008
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
void	main ()
{
	gl_TexCoord[0]	=	gl_MultiTexCoord0;	
	gl_Position     =	ftransform ();
}
