//////////////////////////////////////////////////////////////////////////////////////
//
//
//	EyeDome Lighting - oriented light version
//
//		C.B.    - 04/23/2008
//		D.G-M.  - 10/21/2010
//
//	IN:	    Depth buffer of the scene
//			r = recorded z, in [0:1]
//	OUT:	EDL shaded image
//
///////////////////////////////////////////////////////////////////////////////////////
//#extension GL_ARB_draw_buffers : enable
//#version 110

/**************************************************/
uniform	sampler2D	s1_color;
uniform	sampler2D	s2_depth;
uniform float		d;		    //	extension dans l'image
uniform	int		    Nnb;		//	nombre de voisins par rayon
uniform vec4		N[8];		//	tableau des voisin
uniform float		P;		    //	power exponent
uniform float		F_scale;	//	scale factor for computed AO

uniform float		Zm;	//	minimal z in image
uniform float		ZM;	//	maximal z in image

uniform float		sx;
uniform float		sy;
//
uniform int		ATMOSPHERIC_ON;
uniform float		Z_FOCUS;
//
uniform int		SCREEN_FOCUS_ON;
uniform float		SCREEN_FOCUS_X;
uniform float		SCREEN_FOCUS_Y;
uniform float		SCREEN_FOCUS_SIG;
//
uniform vec3		L;
/**************************************************/

/**************************************************/
vec3		chocolate	=	vec3(68.,23.,14.)/255.;
vec3		english		=	vec3(0.8,0.4,1.);
vec3		lagoon		=	vec3(0.8,0.8,1.);
vec3		ivory		=	vec3(1.,1.,0.8);
vec3		white		=	vec3(1.,1.,1.);

float		t;
vec4		Zn[8];	// profondeurs des voisins
float		D[8];	// ombrage généré par les voisins
vec4		tn, tnw, tw, tsw, ts, tse, te, tne;
float		dn, dnw, dw, dsw, ds, dse, de, dne;
float		S;	//	image step, corresponds to one pixel size
/**************************************************/


//////////////////////////////////////////////////////////////////////////////////////
//
//	Local shading functions
//

//	Simple positive difference
//	zi	elevation of current pixel
//	zj	elevation of its neighbour
float	deltaP(float zi, float zj)
{
	return max(0.,zj-zi);
}

//	Pseudo angle, with S=1 (pixel distance)
//	zi	elevation of current pixel
//	zj	elevation of its neighbour
//	delta	distance between the two
float	angleP(float zi, float zj, float delta)
{
	float	dist	=	delta/S;
	return max(0.,zj-zi) / dist;
}

//	Pseudo angle, square attenuation
//	zi	elevation of current pixel
//	zj	elevation of its neighbour
//	delta	distance between the two
float	anglePattenuation(float zi, float zj, float delta)
{
	float	dist	=	delta/S;
	return max(0.,zj-zi) / (dist*dist);
}

//	True angle
float	trueAngleP(float zi, float zj, float delta)
{
	return atan(max(0.,zj-zi),(delta/S));
}

//  Obscurance
//	zi	elevation of current pixel
//	zj	elevation of its neighbour
//	delta	distance between the two
float	obscurance(float zi, float zj, float delta)
{
	return	angleP(zi,zj,delta);
}

//
//	Local shading functions
//
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
//
//	Z transformation
//
float	zflip(float z)
{
	return	1. - z;
}

float	zscale(float z)
{
	return	clamp((z-Zm)/(ZM-Zm),0.,1.);
}


const float K = 20.;
const float k = 0.1;
const float Am = 0.1;
const float AM = 0.9;
float Zam;
float ZaM;

void 	initZ()
{
	Zam = Z_FOCUS + (Am-0.5)/K;
	ZaM = Z_FOCUS + (AM-0.5)/K;
}


float	ztransform(float z)
{
	float Z = 1.-zscale(z);
	/*if(ATMOSPHERIC_ON==1)
	{
		if(SCREEN_FOCUS_ON==1)
		{
			float	Zmouse	=	1.-zscale(texture2D(s2_depth,vec2(SCREEN_FOCUS_X,SCREEN_FOCUS_Y)).r);
			Z	=	1./(1.+exp(-20.*(Z-Zmouse)));
		}
		else
		{
//			if(Z<Zam)
//				Z = max(0.,k*(Z-Zam)+Am);
//			else if(Z>ZaM)
//				Z = min(1.,k*(Z-ZaM)+AM);
//			else
//				Z = min(1.,max(0.,K*(Z-Z_FOCUS)+0.5));
			Z	=	1./(1.+exp(-K*(Z-Z_FOCUS)));
		}
	}
	//*/
	return Z;
}
//
//	Z transformation
//
//////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////
//
//	NEIGHBORHOOD	SHADING
//
void computeNeighbours8_1(float dist)
{
	// Plan Lumiere-point
	vec4	P	=	vec4( L.xyz , -dot(L.xyz,vec3(0.,0.,t)) );

	//	0 at the back of the scene
	int	c;
	vec2	V;	// pixel voisin
	float	di	=	dist;
	float	Znp[8];	// profondeur des 8 voisins sur le plan

	for(c=0; c<8;c++)
	{
		V	=	gl_TexCoord[0].st + di*vec2(sx,sy)*N[c].xy;
		Zn[c].x	=	ztransform(texture2D(s2_depth,V).r);		// profondeur du voisin réél dans l'image

		//	VERSION qui ombre le fond
		Znp[c]	=	dot( vec4(di*vec2(sx,sy)*N[c].xy,Zn[c].x,1.) , P );		// ztransform( (-dot( di*vec2(sx,sy)*N[c].xy,P.xy) - P.w) / P.z);
	}

	dn		=	obscurance( 0., Znp[0] ,di*sx);
	dnw		=	obscurance( 0., Znp[1],di*sx);
	dw		=	obscurance( 0., Znp[2] ,di*sx);
	dsw		=	obscurance( 0., Znp[3],di*sx);
	ds		=	obscurance( 0., Znp[4] ,di*sx);
	dse		=	obscurance( 0., Znp[5],di*sx);
	de		=	obscurance( 0., Znp[6] ,di*sx);
	dne		=	obscurance( 0., Znp[7],di*sx);
}

void computeNeighbours8_2(float dist)
{
	float	di;
	float	r	=	sqrt(0.5);

	float	di1	=	dist;
	float	di2	=	2.*dist;

	di		=	di1;
	tn.x	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(0,di)).r);
	tnw.x	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(-r*di,r*di)).r);
	tw.x	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(-di,0)).r);
	tsw.x	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(-r*di,-r*di)).r);
	ts.x	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(0,-di)).r);
	tse.x	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(r*di,-r*di)).r);
	te.x	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(di,0)).r);
	tne.x	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(r*di,r*di)).r);



	di		=	di2;
	tn.y	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(0,di)).r);
	tnw.y	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(-r*di,r*di)).r);
	tw.y	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(-di,0)).r);
	tsw.y	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(-r*di,-r*di)).r);
	tse.y	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(r*di,-r*di)).r);
	te.y	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(di,0)).r);
	tne.y	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(r*di,r*di)).r);

	dn		=	max(obscurance(t,tn.x,di1),obscurance(t,tn.y,di2));
	dnw		=	max(obscurance(t,tnw.x,di1),obscurance(t,tnw.y,di2));
	dw		=	max(obscurance(t,tw.x,di1),obscurance(t,tw.y,di2));
	dsw		=	max(obscurance(t,tsw.x,di1),obscurance(t,tsw.y,di2));
	ds		=	max(obscurance(t,ts.x,di1),obscurance(t,ts.y,di2));
	dse		=	max(obscurance(t,tse.x,di1),obscurance(t,tse.y,di2));
	de		=	max(obscurance(t,te.x,di1),obscurance(t,te.y,di2));
	dne		=	max(obscurance(t,tne.x,di1),obscurance(t,tne.y,di2));
}

void computeNeighbours8_4(float dist)
{
	float	di;
	float	r	=	sqrt(0.5);

	float	di1	=	dist;
	float	di2	=	2.*dist;
	float	di3	=	3.*dist;
	float	di4	=	4.*dist;

	di		=	di1;
	tn.x	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(0,di)).r);
	tnw.x	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(-r*di,r*di)).r);
	tw.x	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(-di,0)).r);
	tsw.x	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(-r*di,-r*di)).r);
	ts.x	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(0,-di)).r);
	tse.x	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(r*di,-r*di)).r);
	te.x	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(di,0)).r);
	tne.x	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(r*di,r*di)).r);


	di		=	di2;
	tn.y	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(0,di)).r);
	tnw.y	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(-r*di,r*di)).r);
	tw.y	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(-di,0)).r);
	tsw.y	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(-r*di,-r*di)).r);
	tse.y	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(r*di,-r*di)).r);
	te.y	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(di,0)).r);
	tne.y	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(r*di,r*di)).r);

	di		=	di3;
	tn.z	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(0,di)).r);
	tnw.z	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(-r*di,r*di)).r);
	tw.z	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(-di,0)).r);
	tsw.z	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(-r*di,-r*di)).r);
	ts.z	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(0,-di)).r);
	tse.z	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(r*di,-r*di)).r);
	te.z	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(di,0)).r);
	tne.z	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(r*di,r*di)).r);


	di		=	di4;
	tn.w	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(0,di)).r);
	tnw.w	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(-r*di,r*di)).r);
	tw.w	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(-di,0)).r);
	tsw.w	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(-r*di,-r*di)).r);
	tse.w	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(r*di,-r*di)).r);
	te.w	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(di,0)).r);
	tne.w	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st+vec2(r*di,r*di)).r);

	//	max along each direction
	dn	=	max( max(obscurance(t,tn.x,di1),obscurance(t,tn.y,di2)) , max(obscurance(t,tn.z,di3),obscurance(t,tn.w,di4)) );
 	dnw	=	max( max(obscurance(t,tnw.x,di1),obscurance(t,tnw.y,di2)) , max(obscurance(t,tnw.z,di3),obscurance(t,tnw.w,di4)) );
	dw	=	max( max(obscurance(t,tw.x,di1),obscurance(t,tw.y,di2)) , max(obscurance(t,tw.z,di3),obscurance(t,tw.w,di4)) );
	dsw	=	max( max(obscurance(t,tsw.x,di1),obscurance(t,tsw.y,di2)) , max(obscurance(t,tsw.z,di3),obscurance(t,tsw.w,di4)) );
	ds	=	max( max(obscurance(t,ts.x,di1),obscurance(t,ts.y,di2)) , max(obscurance(t,ts.z,di3),obscurance(t,ts.w,di4)) );
	dse	=	max( max(obscurance(t,tse.x,di1),obscurance(t,tse.y,di2)) , max(obscurance(t,tse.z,di3),obscurance(t,tse.w,di4)) );
	de	=	max( max(obscurance(t,te.x,di1),obscurance(t,te.y,di2)) , max(obscurance(t,te.z,di3),obscurance(t,te.w,di4)) );
	dne	=	max( max(obscurance(t,tne.x,di1),obscurance(t,tne.y,di2)) , max(obscurance(t,tne.z,di3),obscurance(t,tne.w,di4)) );
}
//
//	NEIGHBORHOOD	SHADING
//
//////////////////////////////////////////////////////////////////////////////////////


float	computeObscurance(float F,float scale,float weight)
{
	if(Nnb==1)
		computeNeighbours8_1( scale );
	else if(Nnb==2)
		computeNeighbours8_2( scale * S );
	else
		computeNeighbours8_4( scale * S );

	float	S	=	F;
	float	WE	=	weight;

	S			+=	dn	* WE;
	S			+=	dnw	* WE;
	S			+=	dw	* WE;
	S			+=	dsw	* WE;
	S			+=	ds	* WE;
	S			+=	dse	* WE;
	S			+=	de	* WE;
	S			+=	dne	* WE;


	return	S;
}

void	ambientOcclusion()
{
	int	    filter	=	0;
	float	weight	=	20.;	// 2. * 3.14159;
	float   F		=	computeObscurance(0.0,d,weight);
	float	T       =   F;

	float	fact	=	F_scale;
	if(ATMOSPHERIC_ON==1)
	{
		float dist;
		if(SCREEN_FOCUS_ON==1)
		{
			float	Zmouse	=	1.-zscale(texture2D(s2_depth,vec2(SCREEN_FOCUS_X,SCREEN_FOCUS_Y)).r);
			dist	=	(t-Zmouse)*(t-Zmouse);
		}
		else
		{
			dist 	=	(t-ztransform(Z_FOCUS))*(t-ztransform(Z_FOCUS));
        }
		fact		*=	min(1.,0.05+0.95*exp(-dist/(2.*SCREEN_FOCUS_SIG*SCREEN_FOCUS_SIG)));
	}

	if(SCREEN_FOCUS_ON==1)
	{
		float	X,Y;
		X	=	gl_TexCoord[0].s;
		Y	=	gl_TexCoord[0].t;
		float	dist	= (X-SCREEN_FOCUS_X)*(X-SCREEN_FOCUS_X) + (Y-SCREEN_FOCUS_Y)*(Y-SCREEN_FOCUS_Y);
		fact	*=	min(1.,0.05+0.95*exp(-dist/(2.*SCREEN_FOCUS_SIG*SCREEN_FOCUS_SIG)));
	}

	F	=	exp(-fact*T);

	//gl_FragData[0]	=	vec4(F,F,F,1.);

    vec3 C = texture2D(s1_color,gl_TexCoord[0].st).rgb; //white / ivory / english / lagoon / chocolate
	//gl_FragData[0]	=	vec4( F*t*C,1.);
	if( t > 0.01 )
	{
		gl_FragData[0]	=	vec4( F*C,1.);
	}
	else
	{
		gl_FragData[0]	=	vec4( C,1.);
	}
}


void main (void)
{
	S	=	sx;
	t	=	ztransform(texture2D(s2_depth,gl_TexCoord[0].st).r);
	initZ();
	ambientOcclusion();
}
