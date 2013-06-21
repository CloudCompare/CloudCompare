//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "CCMiscTools.h"

//local
#include "CCConst.h"

//system
#include <algorithm>
#include <string.h>

/*** MACROS FOR TRIBOXOVERLAP ***/

#ifdef FINDMINMAX
#undef FINDMINMAX
#endif
#define FINDMINMAX(x0,x1,x2,minV,maxV) minV = maxV = x0;if(x1<minV) minV=x1; else if(x1>maxV) maxV=x1;if(x2<minV) minV=x2; else if(x2>maxV) maxV=x2;

/*======================== X-tests ========================*/
#define AXISTEST_X01(a, b, fa, fb) minV = a*v0[1] - b*v0[2];maxV = a*v2[1] - b*v2[2]; if(maxV<minV) std::swap(minV,maxV); rad = (fa + fb) * boxhalfsize;if(minV>rad || maxV<-rad) return 0;
#define AXISTEST_X2(a, b, fa, fb) minV = a*v0[1] - b*v0[2];maxV = a*v1[1] - b*v1[2]; if(maxV<minV) std::swap(minV,maxV); rad = (fa + fb) * boxhalfsize;if(minV>rad || maxV<-rad) return 0;

/*======================== Y-tests ========================*/
#define AXISTEST_Y02(a, b, fa, fb) minV = -a*v0[0] + b*v0[2];maxV = -a*v2[0] + b*v2[2]; if(maxV<minV) std::swap(minV,maxV); rad = (fa + fb) * boxhalfsize; if(minV>rad || maxV<-rad) return 0;
#define AXISTEST_Y1(a, b, fa, fb) minV = -a*v0[0] + b*v0[2];maxV = -a*v1[0] + b*v1[2]; if(maxV<minV) std::swap(minV,maxV); rad = (fa + fb) * boxhalfsize;if(minV>rad || maxV<-rad) return 0;

/*======================== Z-tests ========================*/
#define AXISTEST_Z12(a, b, fa, fb) minV = a*v1[0] - b*v1[1];maxV = a*v2[0] - b*v2[1]; if(maxV<minV) std::swap(minV,maxV); rad = (fa + fb) * boxhalfsize;if(minV>rad || maxV<-rad) return 0;
#define AXISTEST_Z0(a, b, fa, fb) minV = a*v0[0] - b*v0[1];maxV = a*v1[0] - b*v1[1]; if(maxV<minV) std::swap(minV,maxV); rad = (fa + fb) * boxhalfsize;if(minV>rad || maxV<-rad) return 0;

using namespace CCLib;

/******** Geometry *********/

void CCMiscTools::MakeMinAndMaxCubical(CCVector3& dimMin, CCVector3& dimMax, double enlargeFactor/*=0.01*/)
{
    CCVector3 dd=dimMax-dimMin;
	CCVector3 md=dimMax+dimMin;

    PointCoordinateType maxDD = dd.x;
	if (dd.y>maxDD)
		maxDD=dd.y;
	if (dd.z>maxDD)
		maxDD=dd.z;

	maxDD *= (PointCoordinateType)(1.0+enlargeFactor);
	dd = CCVector3(maxDD);
	dimMin = (md-dd)*0.5;
	dimMax = dimMin+dd;
}

void CCMiscTools::EnlargeBox(CCVector3& dimMin, CCVector3& dimMax, double coef)
{
    CCVector3 dd=dimMax-dimMin;
	CCVector3 md=dimMax+dimMin;

	dd *= (PointCoordinateType)(1.0+coef);
	dimMin = (md-dd)*0.5;
	dimMax = dimMin+dd;
}

/**** tribox3.c *****/

bool planeBoxOverlap(PointCoordinateType normal[3], PointCoordinateType vert[3], PointCoordinateType maxbox)	// -NJMP-
{
    PointCoordinateType vmin[3],vmax[3];
    for (int q=0;q<=2;q++)
    {
        if (normal[q]>0)
        {
            vmin[q]=-maxbox - vert[q];
            vmax[q]= maxbox - vert[q];
        }
        else
        {
            vmin[q]= maxbox - vert[q];
            vmax[q]=-maxbox - vert[q];
        }
    }

    if (Vector3Tpl<PointCoordinateType>::vdot(normal,vmin)>0)
        return false;
    if (Vector3Tpl<PointCoordinateType>::vdot(normal,vmax)>=0)
        return true;

    return false;
}

bool CCMiscTools::TriBoxOverlap(PointCoordinateType* boxcenter, PointCoordinateType boxhalfsize, const CCVector3* triverts[3])
{
    /*    use separating axis theorem to test overlap between triangle and box */
    /*    need to test for overlap in these directions: */
    /*    1) the {X,Y,Z}-directions (actually, since we use the AABB of the triangle */
    /*       we do not even need to test these) */
    /*    2) normal of the triangle */
    /*    3) crossproduct(edge from tri, {X,Y,Z}-direction) */
    /*       this gives 3x3=9 more tests */

	PointCoordinateType minV,maxV;

    /* move everything so that the boxcenter is in (0,0,0) */
    PointCoordinateType v0[3],v1[3],v2[3];
    CCVector3::vsubstract(triverts[0]->u,boxcenter,v0);
    CCVector3::vsubstract(triverts[1]->u,boxcenter,v1);
    CCVector3::vsubstract(triverts[2]->u,boxcenter,v2);

    PointCoordinateType e0[3];
    CCVector3::vsubstract(v1,v0,e0);      /* compute triangle edge 0 */

    /* Bullet 3: */

	/*  test the 9 tests first (this was faster) */
	PointCoordinateType rad,fex,fey,fez;		// -NJMP- "d" local variable removed
	//fex = fabsf(e0[0]);
	fey = fabsf(e0[1]);
	fez = fabsf(e0[2]);

	AXISTEST_X01(e0[2], e0[1], fez, fey);
	fex = fabsf(e0[0]); //DGM: not necessary before!
	AXISTEST_Y02(e0[2], e0[0], fez, fex);
	AXISTEST_Z12(e0[1], e0[0], fey, fex);

	PointCoordinateType e1[3];
	CCVector3::vsubstract(v2,v1,e1);      /* compute triangle edge 1 */

	//fex = fabsf(e1[0]);
	fey = fabsf(e1[1]);
	fez = fabsf(e1[2]);

	AXISTEST_X01(e1[2], e1[1], fez, fey);
	fex = fabsf(e1[0]); //DGM: not necessary before!
	AXISTEST_Y02(e1[2], e1[0], fez, fex);
	AXISTEST_Z0(e1[1], e1[0], fey, fex);

	PointCoordinateType e2[3];
	CCVector3::vsubstract(v0,v2,e2);      /* compute triangle edge 2 */

	//fex = fabsf(e2[0]);
	fey = fabsf(e2[1]);
	fez = fabsf(e2[2]);

	AXISTEST_X2(e2[2], e2[1], fez, fey);
	fex = fabsf(e2[0]); //DGM: not necessary before!
	AXISTEST_Y1(e2[2], e2[0], fez, fex);
	AXISTEST_Z12(e2[1], e2[0], fey, fex);

    /* Bullet 1: */

    /*  first test overlap in the {X,Y,Z}-directions */
    /*  find minV, maxV of the triangle each direction, and test for overlap in */
    /*  that direction -- this is equivalent to testing a minimal AABB around */
    /*  the triangle against the AABB */

    /* test in 0-direction */
    FINDMINMAX(v0[0],v1[0],v2[0],minV,maxV);
    if (minV>boxhalfsize || maxV<-boxhalfsize)
        return false;

    /* test in 1-direction */
    FINDMINMAX(v0[1],v1[1],v2[1],minV,maxV);
    if (minV>boxhalfsize || maxV<-boxhalfsize)
        return false;

    /* test in 2-direction */
    FINDMINMAX(v0[2],v1[2],v2[2],minV,maxV);
    if (minV>boxhalfsize || maxV<-boxhalfsize)
        return false;

    /* Bullet 2: */
    /*  test if the box intersects the plane of the triangle */
    /*  compute plane equation of triangle: normal*0+d=0 */

	//PointCoordinateType normal[3];
    CCVector3::vcross(e0,e1,/*normal*/e2); //DGM: we use 'e2' instead of 'normal' to save heap memory
    //if (!planeBoxOverlap(/*normal*/e2,v0,boxhalfsize))
    //    return false;
	//DGM: instead, we place the 'planeBoxOverlap' code directly here!
	//int planeBoxOverlap(PointCoordinateType normal[3], PointCoordinateType vert[3], PointCoordinateType maxbox)	// -NJMP-
	{
		//PointCoordinateType vmin[3],vmax[3]; //DGM: we use e0 and e1 instead of vmin and vmax
		if (/*normal*/e2[0]>0)
		{
			/*vmin*/e0[0]=-boxhalfsize - v0[0];
			/*vmax*/e1[0]= boxhalfsize - v0[0];
		}
		else
		{
			/*vmin*/e0[0]= boxhalfsize - v0[0];
			/*vmax*/e1[0]=-boxhalfsize - v0[0];
		}
		if (/*normal*/e2[1]>0)
		{
			/*vmin*/e0[1]=-boxhalfsize - v0[1];
			/*vmax*/e1[1]= boxhalfsize - v0[1];
		}
		else
		{
			/*vmin*/e0[1]= boxhalfsize - v0[1];
			/*vmax*/e1[1]=-boxhalfsize - v0[1];
		}
		if (/*normal*/e2[2]>0)
		{
			/*vmin*/e0[2]=-boxhalfsize - v0[2];
			/*vmax*/e1[2]= boxhalfsize - v0[2];
		}
		else
		{
			/*vmin*/e0[2]= boxhalfsize - v0[2];
			/*vmax*/e1[2]=-boxhalfsize - v0[2];
		}

		if (	Vector3Tpl<PointCoordinateType>::vdot(/*normal*/e2,/*vmin*/e0)>0
			||	Vector3Tpl<PointCoordinateType>::vdot(/*normal*/e2,/*vmax*/e1)<0)
			return false;
	}

    return true;   /* box and triangle overlaps */
}

void CCMiscTools::ComputeBaseVectors(const PointCoordinateType *aPlane, PointCoordinateType* u, PointCoordinateType* v, PointCoordinateType* n)
{
    //on cree un vecteur appartenant au plan (et donc orthogonal a "a")
    //c'est le premier de la base
    u[0] = -aPlane[1];
    u[1] = aPlane[0];
    u[2] = 0;

    //on deduit le deuxieme vecteur de la base par produit vectoriel
    CCVector3::vcross(aPlane,u,v);

    //on normalise u et v
    CCVector3::vnormalize(u);
    CCVector3::vnormalize(v);

    if (n)
    {
        //et enfin, n, vecteur unitaire normal au plan
        n[0] = aPlane[0];
        n[1] = aPlane[1];
        n[2] = aPlane[2];
        CCVector3::vnormalize(n);
    }
}

int gcd(int num1, int num2)
{
    int remainder = num2 % num1;

    return (remainder != 0 ? gcd(remainder,num1) : num1);
}

//TRANSCRIPTED BY DGM FROM MATLAB FUNCTION "partsphere.m" (Paul Leopardi, 2003-10-13, for UNSW School of Mathematics)
float* CCMiscTools::SampleSphere(unsigned N)
{
    if (N <= 0)
		return 0;

    float* dirs = new float[3*N];
	if (!dirs) //not enough memory
		return 0;
    memset(dirs,0,3*N*sizeof(float));
    dirs[2]=1.0;

    if (N == 1)
		return dirs;

    double area = 4.0*M_PI/double(N);
    double beta = acos(1.0-2.0/double(N));
    static const double eps = 2.2204e-16;
    double fuzz = eps*2.0*double(N);
    int Ltemp = (int)ceil( (M_PI-2.0*beta)/sqrt(area)-fuzz );
	int L = 2+(Ltemp>1 ? Ltemp : 1);

	double* mbar = new double[L];
	if (!mbar) //not enough memory
	{
		delete[] dirs;
		return 0;
	}

	memset(mbar,0,L*sizeof(double));
    mbar[0] = 1.0;

	double theta = (M_PI-2.0*beta)/double(L-2);
    int i;
    for (i=1;i<L-1;++i)
        mbar[i] = double(N)*(cos(theta*double(i-1)+beta)-cos(theta*double(i)+beta))*0.5;
    mbar[L-1] = 1.0;

    int *m = new int[L];
	if (!m) //not enough memory
	{
		delete[] dirs;
		delete[] mbar;
		return 0;
	}
	memset(m,0,L*sizeof(int));
    m[0] = 1;

    double alpha = 0.0;
    for (i=1;i<L;++i)
    {
        if ((mbar[i]-floor(mbar[i]+alpha+fuzz)) < 0.5)
            m[i] = int(floor(mbar[i]+alpha+fuzz));
        else
            m[i] = int(ceil(mbar[i]+alpha-fuzz));

        alpha += mbar[i]-m[i];
    }

    double z = 1.0-double(2+m[1])/double(N);
    double *offset = new double[L-1];
	if (!offset) //not enough memory
	{
		delete[] dirs;
		delete[] mbar;
		delete[] m;
		return 0;
	}
    memset(offset,0,(L-1)*sizeof(double));
    //offset[0] = 0.0;

    int n=3;
    for (i=1;i<L-1;++i)
    {
        static const double twist=4.0;
        if (m[i-1]!=0 && m[i]!=0)
			offset[i] = offset[i-1]+double(gcd(m[i],m[i-1]))/double(2*m[i]*m[i-1])+std::min(twist,floor(double(m[i-1])/twist))/double(m[i-1]);
        else
            offset[i] = 0.0;

        double temp = double(m[i])/double(N);
        double top = z+temp;
        double bot = z-temp;

        double h = cos((acos(top)+acos(bot))*0.5);
        double r = sqrt(1.0-h*h);

        for (int j=0;j<m[i];++j)
        {
            double s = offset[i]+double(j)/double(m[i]);
            dirs[n++]=float(r*cos(2.0*M_PI*s));
            dirs[n++]=float(r*sin(2.0*M_PI*s));
            dirs[n++]=float(h);
        }

        z -= double(m[i]+m[i+1])/double(N);
    }

    //i = L;
    //dirs[3*(N-1)]=0.0;
    //dirs[3*(N-1)+1]=0.0;
    dirs[3*(N-1)+2]=-1.0;

    delete[] mbar;
    delete[] m;
    delete[] offset;

    return dirs;
}
