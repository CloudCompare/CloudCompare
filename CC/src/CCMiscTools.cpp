//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
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
#include <cassert>
#include <cstring>

#ifdef USE_VLD
//VLD
#include <vld.h>
#endif

/*** MACROS FOR TRIBOXOVERLAP ***/
/*** TRIBOXOVERLAP code is largely inspired from Tomas Akenine-Möller's algorithm
	http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/tribox3.txt
**/

#ifdef FINDMINMAX
#undef FINDMINMAX
#endif
#define FINDMINMAX(x0,x1,x2,minV,maxV) minV = maxV = x0; if(x1<minV) minV=x1; else if(x1>maxV) maxV=x1;if(x2<minV) minV=x2; else if(x2>maxV) maxV=x2;

/*======================== X-tests ========================*/
#define AXISTEST_X01(a, b, fa, fb) minV = a*v0[1] - b*v0[2]; maxV = a*v2[1] - b*v2[2]; if (maxV<minV) std::swap(minV,maxV); rad = fa * boxhalfSize.y + fb * boxhalfSize.z; if (minV>rad || maxV<-rad) return 0;
#define AXISTEST_X2(a, b, fa, fb) minV = a*v0[1] - b*v0[2]; maxV = a*v1[1] - b*v1[2]; if (maxV<minV) std::swap(minV,maxV); rad = fa * boxhalfSize.y + fb * boxhalfSize.z; if (minV>rad || maxV<-rad) return 0;

/*======================== Y-tests ========================*/
#define AXISTEST_Y02(a, b, fa, fb) minV = -a*v0[0] + b*v0[2]; maxV = -a*v2[0] + b*v2[2]; if (maxV<minV) std::swap(minV,maxV); rad = fa * boxhalfSize.x + fb * boxhalfSize.z; if (minV>rad || maxV<-rad) return 0;
#define AXISTEST_Y1(a, b, fa, fb) minV = -a*v0[0] + b*v0[2]; maxV = -a*v1[0] + b*v1[2]; if (maxV<minV) std::swap(minV,maxV); rad = fa * boxhalfSize.x + fb * boxhalfSize.z; if (minV>rad || maxV<-rad) return 0;

/*======================== Z-tests ========================*/
#define AXISTEST_Z12(a, b, fa, fb) minV = a*v1[0] - b*v1[1]; maxV = a*v2[0] - b*v2[1]; if (maxV<minV) std::swap(minV,maxV); rad = fa * boxhalfSize.x + fb * boxhalfSize.y; if (minV>rad || maxV<-rad) return 0;
#define AXISTEST_Z0(a, b, fa, fb) minV = a*v0[0] - b*v0[1]; maxV = a*v1[0] - b*v1[1]; if (maxV<minV) std::swap(minV,maxV); rad = fa * boxhalfSize.x + fb * boxhalfSize.y; if (minV>rad || maxV<-rad) return 0;

using namespace CCLib;

/******** Geometry *********/

void CCMiscTools::MakeMinAndMaxCubical(CCVector3& dimMin, CCVector3& dimMax, double enlargeFactor/*=0.01*/)
{
	//get box max dimension
	PointCoordinateType maxDD = 0;
	{
		CCVector3 diag = dimMax-dimMin;
		maxDD = std::max(diag.x,diag.y);
		maxDD = std::max(maxDD,diag.z);
	}

	//enlarge it if necessary
	if (enlargeFactor > 0)
		maxDD = static_cast<PointCoordinateType>( static_cast<double>(maxDD) * (1.0+enlargeFactor) );

	//build corresponding 'square' box
	{
		CCVector3 dd(maxDD,maxDD,maxDD);
		CCVector3 md = dimMax+dimMin;
	
		dimMin = (md-dd) * static_cast<PointCoordinateType>(0.5);
		dimMax = dimMin+dd;
	}
}

void CCMiscTools::EnlargeBox(CCVector3& dimMin, CCVector3& dimMax, double coef)
{
	CCVector3 dd = (dimMax-dimMin) * static_cast<PointCoordinateType>(1.0+coef);
	CCVector3 md = dimMax+dimMin;

	dimMin = (md-dd) * static_cast<PointCoordinateType>(0.5);
	dimMax = dimMin+dd;
}

/**** tribox3.c *****/

bool CCMiscTools::TriBoxOverlap(const CCVector3& boxcenter, const CCVector3& boxhalfSize, const CCVector3* triverts[3])
{
	/*    use separating axis theorem to test overlap between triangle and box */
	/*    need to test for overlap in these directions: */
	/*    1) the {X,Y,Z}-directions (actually, since we use the AABB of the triangle */
	/*       we do not even need to test these) */
	/*    2) normal of the triangle */
	/*    3) crossproduct(edge from tri, {X,Y,Z}-direction) */
	/*       this gives 3x3=9 more tests */

	/* move everything so that the boxcenter is in (0,0,0) */
	PointCoordinateType v0[3];
	PointCoordinateType v1[3];
	PointCoordinateType v2[3];
	CCVector3::vsubstract(triverts[0]->u, boxcenter.u, v0);
	CCVector3::vsubstract(triverts[1]->u, boxcenter.u, v1);
	CCVector3::vsubstract(triverts[2]->u, boxcenter.u, v2);

	PointCoordinateType e0[3];
	CCVector3::vsubstract(v1,v0,e0);      /* compute triangle edge 0 */

	/* Bullet 3: */

	/*  test the 9 tests first (this was faster) */
	PointCoordinateType rad;
	PointCoordinateType fex;
	PointCoordinateType fey;
	PointCoordinateType fez;
	//fex = std::abs(e0[0]);
	fey = std::abs(e0[1]);
	fez = std::abs(e0[2]);

	PointCoordinateType minV;
	PointCoordinateType maxV;
	AXISTEST_X01(e0[2], e0[1], fez, fey);
	fex = std::abs(e0[0]); //DGM: not necessary before!
	AXISTEST_Y02(e0[2], e0[0], fez, fex);
	AXISTEST_Z12(e0[1], e0[0], fey, fex);

	PointCoordinateType e1[3];
	CCVector3::vsubstract(v2,v1,e1);      /* compute triangle edge 1 */

	//fex = std::abs(e1[0]);
	fey = std::abs(e1[1]);
	fez = std::abs(e1[2]);

	AXISTEST_X01(e1[2], e1[1], fez, fey);
	fex = std::abs(e1[0]); //DGM: not necessary before!
	AXISTEST_Y02(e1[2], e1[0], fez, fex);
	AXISTEST_Z0(e1[1], e1[0], fey, fex);

	PointCoordinateType e2[3];
	CCVector3::vsubstract(v0,v2,e2);      /* compute triangle edge 2 */

	//fex = std::abs(e2[0]);
	fey = std::abs(e2[1]);
	fez = std::abs(e2[2]);

	AXISTEST_X2(e2[2], e2[1], fez, fey);
	fex = std::abs(e2[0]); //DGM: not necessary before!
	AXISTEST_Y1(e2[2], e2[0], fez, fex);
	AXISTEST_Z12(e2[1], e2[0], fey, fex);

	/* Bullet 1: */

	/*  first test overlap in the {X,Y,Z}-directions */
	/*  find minV, maxV of the triangle each direction, and test for overlap in */
	/*  that direction -- this is equivalent to testing a minimal AABB around */
	/*  the triangle against the AABB */

	/* test in 0-direction */
	FINDMINMAX(v0[0], v1[0], v2[0], minV, maxV);
	if (minV > boxhalfSize.x || maxV<-boxhalfSize.x)
		return false;

	/* test in 1-direction */
	FINDMINMAX(v0[1], v1[1], v2[1], minV, maxV);
	if (minV>boxhalfSize.y || maxV<-boxhalfSize.y)
		return false;

	/* test in 2-direction */
	FINDMINMAX(v0[2], v1[2], v2[2], minV, maxV);
	if (minV>boxhalfSize.z || maxV < -boxhalfSize.z)
		return false;

	/* Bullet 2: */
	/*  test if the box intersects the plane of the triangle */
	/*  compute plane equation of triangle: normal*0+d=0 */

	//PointCoordinateType normal[3];
	CCVector3::vcross(e0,e1,/*normal*/e2); //DGM: we use 'e2' instead of 'normal' to save heap memory
	{
		//PointCoordinateType vmin[3],vmax[3]; //DGM: we use e0 and e1 instead of vmin and vmax
		if (/*normal*/e2[0]>0)
		{
			/*vmin*/e0[0] = -boxhalfSize.x - v0[0];
			/*vmax*/e1[0] =  boxhalfSize.x - v0[0];
		}
		else
		{
			/*vmin*/e0[0] =  boxhalfSize.x - v0[0];
			/*vmax*/e1[0] = -boxhalfSize.x - v0[0];
		}
		if (/*normal*/e2[1]>0)
		{
			/*vmin*/e0[1] = -boxhalfSize.y - v0[1];
			/*vmax*/e1[1] =  boxhalfSize.y - v0[1];
		}
		else
		{
			/*vmin*/e0[1] =  boxhalfSize.y - v0[1];
			/*vmax*/e1[1] = -boxhalfSize.y - v0[1];
		}
		if (/*normal*/e2[2]>0)
		{
			/*vmin*/e0[2] = -boxhalfSize.z - v0[2];
			/*vmax*/e1[2] =  boxhalfSize.z - v0[2];
		}
		else
		{
			/*vmin*/e0[2] =  boxhalfSize.z - v0[2];
			/*vmax*/e1[2] = -boxhalfSize.z - v0[2];
		}

		if (   CCVector3::vdot(/*normal*/e2,/*vmin*/e0) > 0
			|| CCVector3::vdot(/*normal*/e2,/*vmax*/e1) < 0)
		{
			return false;
		}
	}

	return true;   /* box and triangle overlaps */
}

bool CCMiscTools::TriBoxOverlapd(const CCVector3d& boxcenter, const CCVector3d& boxhalfSize, const CCVector3d triverts[3])
{
	/*    use separating axis theorem to test overlap between triangle and box */
	/*    need to test for overlap in these directions: */
	/*    1) the {X,Y,Z}-directions (actually, since we use the AABB of the triangle */
	/*       we do not even need to test these) */
	/*    2) normal of the triangle */
	/*    3) crossproduct(edge from tri, {X,Y,Z}-direction) */
	/*       this gives 3x3=9 more tests */

	/* move everything so that the boxcenter is in (0,0,0) */
	double v0[3];
	double v1[3];
	double v2[3];
	CCVector3d::vsubstract(triverts[0].u, boxcenter.u, v0);
	CCVector3d::vsubstract(triverts[1].u, boxcenter.u, v1);
	CCVector3d::vsubstract(triverts[2].u, boxcenter.u, v2);

	double e0[3];
	CCVector3d::vsubstract(v1, v0, e0);      /* compute triangle edge 0 */

	/* Bullet 3: */

	/*  test the 9 tests first (this was faster) */
	double rad = 0.0;
	double fex = 0.0;
	double fey = std::abs(e0[1]);
	double fez = std::abs(e0[2]);

	double minV = 0.0;
	double maxV = 0.0;
	AXISTEST_X01(e0[2], e0[1], fez, fey);
	fex = std::abs(e0[0]); //DGM: not necessary before!
	AXISTEST_Y02(e0[2], e0[0], fez, fex);
	AXISTEST_Z12(e0[1], e0[0], fey, fex);

	double e1[3];
	CCVector3d::vsubstract(v2, v1, e1);      /* compute triangle edge 1 */

	//fex = std::abs(e1[0]);
	fey = std::abs(e1[1]);
	fez = std::abs(e1[2]);

	AXISTEST_X01(e1[2], e1[1], fez, fey);
	fex = std::abs(e1[0]); //DGM: not necessary before!
	AXISTEST_Y02(e1[2], e1[0], fez, fex);
	AXISTEST_Z0(e1[1], e1[0], fey, fex);

	double e2[3];
	CCVector3d::vsubstract(v0, v2, e2);      /* compute triangle edge 2 */

	//fex = std::abs(e2[0]);
	fey = std::abs(e2[1]);
	fez = std::abs(e2[2]);

	AXISTEST_X2(e2[2], e2[1], fez, fey);
	fex = std::abs(e2[0]); //DGM: not necessary before!
	AXISTEST_Y1(e2[2], e2[0], fez, fex);
	AXISTEST_Z12(e2[1], e2[0], fey, fex);

	/* Bullet 1: */

	/*  first test overlap in the {X,Y,Z}-directions */
	/*  find minV, maxV of the triangle each direction, and test for overlap in */
	/*  that direction -- this is equivalent to testing a minimal AABB around */
	/*  the triangle against the AABB */

	/* test in 0-direction */
	FINDMINMAX(v0[0], v1[0], v2[0], minV, maxV);
	if (minV>boxhalfSize.x || maxV<-boxhalfSize.x)
		return false;

	/* test in 1-direction */
	FINDMINMAX(v0[1], v1[1], v2[1], minV, maxV);
	if (minV>boxhalfSize.y || maxV<-boxhalfSize.y)
		return false;

	/* test in 2-direction */
	FINDMINMAX(v0[2], v1[2], v2[2], minV, maxV);
	if (minV>boxhalfSize.z || maxV<-boxhalfSize.z)
		return false;

	/* Bullet 2: */
	/*  test if the box intersects the plane of the triangle */
	/*  compute plane equation of triangle: normal*0+d=0 */

	//double normal[3];
	CCVector3d::vcross(e0, e1,/*normal*/e2); //DGM: we use 'e2' instead of 'normal' to save heap memory
	{
		//double vmin[3],vmax[3]; //DGM: we use e0 and e1 instead of vmin and vmax
		if (/*normal*/e2[0]>0)
		{
			/*vmin*/e0[0] = -boxhalfSize.x - v0[0];
			/*vmax*/e1[0] = boxhalfSize.x - v0[0];
		}
		else
		{
			/*vmin*/e0[0] = boxhalfSize.x - v0[0];
			/*vmax*/e1[0] = -boxhalfSize.x - v0[0];
		}
		if (/*normal*/e2[1]>0)
		{
			/*vmin*/e0[1] = -boxhalfSize.y - v0[1];
			/*vmax*/e1[1] = boxhalfSize.y - v0[1];
		}
		else
		{
			/*vmin*/e0[1] = boxhalfSize.y - v0[1];
			/*vmax*/e1[1] = -boxhalfSize.y - v0[1];
		}
		if (/*normal*/e2[2]>0)
		{
			/*vmin*/e0[2] = -boxhalfSize.z - v0[2];
			/*vmax*/e1[2] = boxhalfSize.z - v0[2];
		}
		else
		{
			/*vmin*/e0[2] = boxhalfSize.z - v0[2];
			/*vmax*/e1[2] = -boxhalfSize.z - v0[2];
		}

		if (   CCVector3d::vdot(/*normal*/e2,/*vmin*/e0) > 0
			|| CCVector3d::vdot(/*normal*/e2,/*vmax*/e1) < 0)
		{
			return false;
		}
	}

	return true;   /* box and triangle overlaps */
}

void CCMiscTools::ComputeBaseVectors(const CCVector3 &N, CCVector3& X, CCVector3& Y)
{
	CCVector3 Nunit = N;
	Nunit.normalize();

	//we create a first vector orthogonal to the input one
	X = Nunit.orthogonal(); //X is also normalized

	//we deduce the orthogonal vector to the input one and X
	Y = N.cross(X);
	//Y.normalize(); //should already be normalized!
}

void CCMiscTools::ComputeBaseVectors(const CCVector3d &N, CCVector3d& X, CCVector3d& Y)
{
	CCVector3d Nunit = N;
	Nunit.normalize();

	//we create a first vector orthogonal to the input one
	X = Nunit.orthogonal(); //X is also normalized

	//we deduce the orthogonal vector to the input one and X
	Y = N.cross(X);
	//Y.normalize(); //should already be normalized!
}
