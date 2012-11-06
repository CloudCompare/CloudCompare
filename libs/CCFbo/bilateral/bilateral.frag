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
//#extension GL_ARB_draw_buffers : enable


/****************************************************/
uniform	sampler2D	s2_I;
uniform	sampler2D	s2_D;
uniform	float		SX;
uniform	float		SY;
uniform	int		    N;					//	filter size (full width, necessarily odd, like 3, 5...) <= 15!
uniform	int         use_gauss_p;
uniform float		gauss_p[226];		//	pixel distance based damping coefs (max = 15*15).
uniform float		sigma;
uniform float		sigmaz;
/****************************************************/

/****************************************************/
vec3	C;
float	z;
/****************************************************/

void main (void)
{
	//C	=	texture2D(s2_I,gl_TexCoord[0].st).rgb;
	z	=	texture2D(s2_D,gl_TexCoord[0].st).r;

	float	Wsum	=	0.0;		// sum of all weights
	vec3	Csum	=	vec3(0.);	// sum of all contributions
	int	    hN		=	N/2;		// filter half width
	vec2	coordi	=	vec2(0.,0.);
	vec3	Ci;
	float   dist,dz,zi,Fi,Gi;

	int	c,d;
	for(c=-hN;c<hN+1;c++)
	{
		//neighbor position (X)
        coordi.x = float(c)*SX;

        for(d=-hN;d<hN+1;d++)
        {
            //neighbor position (Y)
            coordi.y    = float(d)*SY;

            //neighbor color
            Ci	        =	texture2D(s2_I,gl_TexCoord[0].st+coordi).rgb;

            //pixel distance based damping
            if (use_gauss_p==1)
            {
                Fi	        =	gauss_p[(c+hN)*N+(d+hN)];
            }
            else
            {
                dist	    =	float(c*c+d*d)/(sigma*float(hN*hN));
                Fi		    =	exp(-dist*dist/2.0);
            }

            //pixel depth difference based damping
            if (sigmaz>0.0)
            {
				//neighbor depth
				zi	        =	texture2D(s2_D,gl_TexCoord[0].st+coordi).r;
				dz	        =	(z-zi)*(z-zi)/sigmaz;
				Gi	        =	exp(-dz*dz/2.0);
				Fi			*=	Gi;
			}

            Csum	+=	Ci * Fi;
            Wsum	+=	Fi;
        }
	}

	//output
	gl_FragColor = vec4(Csum/Wsum,1.0);
}

