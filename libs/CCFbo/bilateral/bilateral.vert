//////////////////////////////////////////////////////////////////////////////////////
//
//
//	Bilateral filtering
//
//		C.B.    - 08/16/2008
//      D.G-M.  - 10/20/2010
//
//	IN:
//		s2_I	-	Image to blur
//		s2_D	-	Modulating depth image
//
//	OUT:
//		Filtered image
//
///////////////////////////////////////////////////////////////////////////////////////
//#extension GL_EXT_gpu_shader4 : enable
//#version 110
//#extension GL_ARB_draw_buffers : enable

void	main ()
{
	gl_TexCoord[0]	=	gl_MultiTexCoord0;
	gl_Position     =	ftransform ();
}
