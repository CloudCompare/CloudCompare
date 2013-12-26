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

#include "Neighbourhood.h"

//local
#include "GenericIndexedMesh.h"
#include "GenericIndexedCloudPersist.h"
#include "Matrix.h"
#include "Delaunay2dMesh.h"
#include "ConjugateGradient.h"
#include "DistanceComputationTools.h"
#include "ChunkedPointCloud.h"
#include "SimpleMesh.h"

//system
#include <assert.h>

using namespace CCLib;

Neighbourhood::Neighbourhood(GenericIndexedCloudPersist* associatedCloud)
	: structuresValidity(DEPRECATED)
	, m_associatedCloud(associatedCloud)
{
	assert(m_associatedCloud);
}

void Neighbourhood::reset()
{
	structuresValidity = DEPRECATED;
}

const CCVector3* Neighbourhood::getGravityCenter()
{
	if (!(structuresValidity & GRAVITY_CENTER))
		computeGravityCenter();
	return ((structuresValidity & GRAVITY_CENTER) ? &theGravityCenter : 0);
}

const PointCoordinateType* Neighbourhood::getLSQPlane()
{
	if (!(structuresValidity & LSQ_PLANE))
		computeLeastSquareBestFittingPlane();
	return ((structuresValidity & LSQ_PLANE) ? theLSQPlaneEquation : 0);
}

const CCVector3* Neighbourhood::getLSQPlaneX()
{
	if (!(structuresValidity & LSQ_PLANE))
		computeLeastSquareBestFittingPlane();
	return ((structuresValidity & LSQ_PLANE) ? theLSQPlaneVectors : 0);
}

const CCVector3* Neighbourhood::getLSQPlaneY()
{
	if (!(structuresValidity & LSQ_PLANE))
		computeLeastSquareBestFittingPlane();
	return ((structuresValidity & LSQ_PLANE) ? theLSQPlaneVectors+1 : 0);
}

const CCVector3* Neighbourhood::getLSQPlaneNormal()
{
	if (!(structuresValidity & LSQ_PLANE))
		computeLeastSquareBestFittingPlane();
	return ((structuresValidity & LSQ_PLANE) ? theLSQPlaneVectors+2 : 0);
}

const PointCoordinateType* Neighbourhood::getHeightFunction(uchar* dimsHF/*=0*/)
{
	if (!(structuresValidity & HEIGHT_FUNCTION))
		computeHeightFunction();
	if (dimsHF)
	{
		dimsHF[0]=theHeightFunctionDirections[0];
		dimsHF[1]=theHeightFunctionDirections[1];
		dimsHF[2]=theHeightFunctionDirections[2];
	}
	return ((structuresValidity & HEIGHT_FUNCTION) ? theHeightFunction : 0);
}

const double* Neighbourhood::get3DQuadric()
{
	if (!(structuresValidity & QUADRIC_3D))
		compute3DQuadric();
	return ((structuresValidity & QUADRIC_3D) ? the3DQuadric : 0);
}

void Neighbourhood::computeGravityCenter()
{
	//invalidate precedent centroid (if any)
	structuresValidity &= (~GRAVITY_CENTER);

	assert(m_associatedCloud);
	unsigned count = (m_associatedCloud ? m_associatedCloud->size() : 0);
	if (!count)
		return;

	CCVector3d Psum(0.0);

	//sums
	#ifdef CC_NEIGHBOURHOOD_PRECISION_COMPUTINGS
	PointCoordinateType bbMin[3],bbMax[3];
	m_associatedCloud->getBoundingBox(bbMin,bbMax);
	CCVector3 C = (CCVector3(bbMin)+CCVector3(bbMax))*0.5;
	#endif

	for (unsigned i=0;i<count;++i)
	{
		const CCVector3* P = m_associatedCloud->getPoint(i);
		#ifdef CC_NEIGHBOURHOOD_PRECISION_COMPUTINGS
		Xsum += (double)(P->x-C.x);
		Ysum += (double)(P->y-C.y);
		Zsum += (double)(P->z-C.z);
		#else
		Psum.x += (double)P->x;
		Psum.y += (double)P->y;
		Psum.z += (double)P->z;
		#endif
	}

	Psum /= (double)count;
	theGravityCenter.x = (PointCoordinateType)Psum.x;
	theGravityCenter.y = (PointCoordinateType)Psum.y;
	theGravityCenter.z = (PointCoordinateType)Psum.z;

	#ifdef CC_NEIGHBOURHOOD_PRECISION_COMPUTINGS
	theGravityCenter += C;
	#endif

	structuresValidity |= GRAVITY_CENTER;
}

CCLib::SquareMatrixd Neighbourhood::computeCovarianceMatrix()
{
	assert(m_associatedCloud);
	unsigned count = (m_associatedCloud ? m_associatedCloud->size() : 0);
	if (!count)
		return CCLib::SquareMatrixd();

	//we get centroid
	const CCVector3* G = getGravityCenter();
	assert(G);

    //we build up covariance matrix
	double mXX = 0.0;
	double mYY = 0.0;
	double mZZ = 0.0;
	double mXY = 0.0;
	double mXZ = 0.0;
	double mYZ = 0.0;

	for (unsigned i=0;i<count;++i)
	{
		CCVector3 P = *m_associatedCloud->getPoint(i) - *G;

		mXX += (double)(P.x*P.x);
		mYY += (double)(P.y*P.y);
		mZZ += (double)(P.z*P.z);
		mXY += (double)(P.x*P.y);
		mXZ += (double)(P.x*P.z);
		mYZ += (double)(P.y*P.z);
	}

	//symmetry
	CCLib::SquareMatrixd covMat(3);
	covMat.m_values[0][0] = mXX/(double)count;
	covMat.m_values[1][1] = mYY/(double)count;
	covMat.m_values[2][2] = mZZ/(double)count;
	covMat.m_values[1][0] = covMat.m_values[0][1] = mXY/(double)count;
	covMat.m_values[2][0] = covMat.m_values[0][2] = mXZ/(double)count;
	covMat.m_values[2][1] = covMat.m_values[1][2] = mYZ/(double)count;

	return covMat;
}

PointCoordinateType Neighbourhood::computeLargestRadius()
{
	assert(m_associatedCloud);
	unsigned pointCount = (m_associatedCloud ? m_associatedCloud->size() : 0);
	if (pointCount < 2)
		return 0;

	//get the centroid
	const CCVector3* G = getGravityCenter();
	if (!G)
		return PC_NAN;

	PointCoordinateType maxSquareDist = 0;
	for (unsigned i=0; i<pointCount; ++i)
	{
		const CCVector3* P = m_associatedCloud->getPoint(i);
		PointCoordinateType d2 = (*P-*G).norm2();
		if (d2 > maxSquareDist)
			maxSquareDist = d2;
	}

	return sqrt(maxSquareDist);
}

bool Neighbourhood::computeLeastSquareBestFittingPlane()
{
	//invalidate precedent LS plane (if any)
	structuresValidity &= (~LSQ_PLANE);

	assert(m_associatedCloud);
	unsigned pointCount = (m_associatedCloud ? m_associatedCloud->size() : 0);

	//we need at least 3 points to compute a plane
	assert(CC_LOCAL_MODEL_MIN_SIZE[LS] >= 3);
	if (pointCount < CC_LOCAL_MODEL_MIN_SIZE[LS])
	{
		//not enough points!
		return false;
	}

	CCVector3 G(0,0,0);
	if (pointCount > 3)
	{
		//we determine plane normal by computing the smallest eigen value of M = 1/n * S[(p-�)*(p-�)']
		CCLib::SquareMatrixd eig = computeCovarianceMatrix().computeJacobianEigenValuesAndVectors();

		//invalid matrix?
		if (!eig.isValid())
			return false;

		//get normal
		{
			double vec[3];
			//the smallest eigen vector corresponds to the "least square best fitting plane" normal
			eig.getMinEigenValueAndVector(vec);
			theLSQPlaneVectors[2].x = (PointCoordinateType)vec[0];
			theLSQPlaneVectors[2].y = (PointCoordinateType)vec[1];
			theLSQPlaneVectors[2].z = (PointCoordinateType)vec[2];
		}

		//get also X
		{
			double vec[3];
			eig.getMaxEigenValueAndVector(vec);
			theLSQPlaneVectors[0].x = (PointCoordinateType)vec[0];
			theLSQPlaneVectors[0].y = (PointCoordinateType)vec[1];
			theLSQPlaneVectors[0].z = (PointCoordinateType)vec[2];
		}

		//get the centroid (should already be up-to-date - see computeCovarianceMatrix)
		G = *getGravityCenter();
	}
	else
	{
		//we simply compute the normal of the 3 points by cross product!
		const CCVector3* A = m_associatedCloud->getPoint(0);
		const CCVector3* B = m_associatedCloud->getPoint(1);
		const CCVector3* C = m_associatedCloud->getPoint(2);

		//get X (AB by default) and Y (AC - will be updated later) and deduce N = X ^ Y
		theLSQPlaneVectors[0] = (*B-*A);
		theLSQPlaneVectors[1] = (*C-*A);
		theLSQPlaneVectors[2] = theLSQPlaneVectors[0].cross(theLSQPlaneVectors[1]);

		//the plane passes through any of the 3 points
		G = *A;
	}

	//make sure all vectors are unit!
	if (theLSQPlaneVectors[2].norm2() < ZERO_TOLERANCE)
	{
		//this means that the points are colinear!
		theLSQPlaneVectors[2] = CCVector3(0,0,1); //any normal will do
	}
	else
	{
		theLSQPlaneVectors[2].normalize();
	}
	//normalize X as well
	theLSQPlaneVectors[0].normalize();
	//and update Y
	theLSQPlaneVectors[1] = theLSQPlaneVectors[2].cross(theLSQPlaneVectors[1]);


	//deduce the proper equation
	theLSQPlaneEquation[0] = theLSQPlaneVectors[2].x;
	theLSQPlaneEquation[1] = theLSQPlaneVectors[2].y;
	theLSQPlaneEquation[2] = theLSQPlaneVectors[2].z;

	//eventually we just have to compute the 'constant' coefficient a3
	//we use the fact that the plane pass through G --> GM.N = 0 (scalar prod)
	//i.e. a0*G[0]+a1*G[1]+a2*G[2]=a3
	theLSQPlaneEquation[3] = G.dot(theLSQPlaneVectors[2]);

	structuresValidity |= LSQ_PLANE;

	return true;
}

bool Neighbourhood::computeHeightFunction()
{
	//invalidate precedent quadric (if any)
	structuresValidity &= (~HEIGHT_FUNCTION);

	assert(m_associatedCloud);
	if (!m_associatedCloud)
		return false;

	unsigned count = m_associatedCloud->size();
	
	assert(CC_LOCAL_MODEL_MIN_SIZE[HF] >= 5);
	if (count < CC_LOCAL_MODEL_MIN_SIZE[HF])
		return false;

	const PointCoordinateType* lsq = getLSQPlane();
	if (!lsq)
        return false;

	//we get centroid (should already be up-to-date - see computeCovarianceMatrix)
	const CCVector3* G = getGravityCenter();
	assert(G);

	//get the best projection axis
	uchar idx_X=0/*x*/,idx_Y=1/*y*/,idx_Z=2/*z*/; //default configuration: z is the "normal" direction, we use (x,y) as the base plane
	PointCoordinateType nxx = lsq[0]*lsq[0];
	PointCoordinateType nyy = lsq[1]*lsq[1];
	PointCoordinateType nzz = lsq[2]*lsq[2];
	if (nxx > nyy)
	{
		if (nxx > nzz)
		{
			//as x is the "normal" direction, we use (y,z) as the base plane
			idx_X=1/*y*/; idx_Y=2/*z*/; idx_Z=0/*x*/;
		}
	}
	else
	{
	    if (nyy > nzz)
		{
			//as y is the "normal" direction, we use (z,x) as the base plane
			idx_X=2/*z*/; idx_Y=0/*x*/; idx_Z=1/*y*/;
		}
    }

	//compute the A matrix and b vector
	float *A = new float[6*count];
	float *b = new float[count];

	float lmax2=0; //max (squared) dimension

    //for all points
	{
		float* _A=A;
		float* _b=b;
		for (unsigned i=0;i<count;++i)
		{
			CCVector3 P = *m_associatedCloud->getPoint(i) - *G;

			float lX = (float)P.u[idx_X];
			float lY = (float)P.u[idx_Y];
			float lZ = (float)P.u[idx_Z];

			*_A++ = 1.0;
			*_A++ = lX;
			*_A++ = lY;
			*_A = lX*lX;
			//by the way, we track the max 'X' squared dimension
			if (*_A>lmax2)
				lmax2=*_A;
			++_A;
			*_A++ = lX*lY;
			*_A = lY*lY;
			//by the way, we track the max 'Y' squared dimension
			if (*_A>lmax2)
				lmax2=*_A;
			++_A;

			*_b++ = lZ;
			lZ *= lZ;
			//and don't forget to track the max 'Z' squared dimension as well
			if (lZ>lmax2)
				lmax2=lZ;
		}
	}

	//conjugate gradient initialization
	//we solve tA.A.X=tA.b
	ConjugateGradient<6,double> cg;
	CCLib::SquareMatrixd& tAA = cg.A();
	double* tAb = cg.b();

	//compute tA.A and tA.b
	{
		for (unsigned i=0; i<6; ++i)
		{
			//tA.A part
			for (unsigned j=i; j<6; ++j)
			{
				double tmp = 0;
				float* _Ai = A+i;
				float* _Aj = A+j;
				for (unsigned k=0; k<count; ++k)
				{
					//tmp += A[(6*k)+i]*A[(6*k)+j];
					tmp += (double)((*_Ai) * (*_Aj));
					_Ai += 6;
					_Aj += 6;
				}
				tAA.m_values[j][i] = tAA.m_values[i][j] = tmp;
			}

			//tA.b part
			{
				double tmp = 0;
				float* _Ai = A+i;
				float* _b = b;
				for (unsigned k=0; k<count; ++k)
				{
					//tmp += A[(6*k)+i]*b[k];
					tmp += (double)((*_Ai) * (*_b++));
					_Ai += 6;
				}
				tAb[i] = tmp;
			}
		}
	}

	//first guess for X: plane equation (a0.x+a1.y+a2.z=a3 --> z = a3/a2 - a0/a2.x - a1/a2.y)
	double X0[6];
	X0[0] = (double)/*lsq[3]/lsq[idx_Z]*/0; //DGM: warning, points have already been recentred around the gravity center! So forget about a3
	X0[1] = (double)(-lsq[idx_X]/lsq[idx_Z]);
	X0[2] = (double)(-lsq[idx_Y]/lsq[idx_Z]);
	X0[3] = 0;
	X0[4] = 0;
	X0[5] = 0;

	//special case: a0 = a1 = a2 = 0! //happens for perfectly flat surfaces!
	if (X0[1] == 0.0 && X0[2] == 0.0)
		X0[0] = 1.0;

	//init. conjugate gradient
	cg.initConjugateGradient(X0);

    //conjugate gradient iterations
	{
		double convergenceThreshold = (double)lmax2 * 1e-8;  //max. error for convergence = 1e-8 of largest cloud dimension (empirical!)
		for (unsigned i=0; i<1500; ++i)
		{
			double lastError = cg.iterConjugateGradient(X0);
			if (lastError < convergenceThreshold) //converged
				break;
		}
	}
	//fprintf(fp,"X%i=(%f,%f,%f,%f,%f,%f)\n",i,X0[0],X0[1],X0[2],X0[3],X0[4],X0[5]);
	//fprintf(fp,"lastError=%E/%E\n",lastError,convergenceThreshold);
	//fclose(fp);

	delete[] A;
	A=0;
	delete[] b;
	b=0;

	//output
	{
		for (unsigned i=0;i<6;++i)
			theHeightFunction[i]=(PointCoordinateType)X0[i];
		theHeightFunctionDirections[0]=idx_X;
		theHeightFunctionDirections[1]=idx_Y;
		theHeightFunctionDirections[2]=idx_Z;

		structuresValidity |= HEIGHT_FUNCTION;
	}

	return true;
}

bool Neighbourhood::compute3DQuadric()
{
	//invalidate precedent quadric (if any)
	structuresValidity &= (~QUADRIC_3D);

	assert(m_associatedCloud);
	if (!m_associatedCloud)
		return false;

    //computes a 3D quadric of the form ax2 +by2 +cz2 + 2exy + 2fyz + 2gzx + 2lx + 2my + 2nz + d = 0
    //"THREE-DIMENSIONAL SURFACE CURVATURE ESTIMATION USING QUADRIC SURFACE PATCHES", I. Douros & B. Buxton, University College London

	//we get centroid
	const CCVector3* G = getGravityCenter();
	assert(G);

    //we look for the eigen vector associated to the minimum eigen value of a matrix A
    //where A=transpose(D)*D, and D=[xi^2 yi^2 zi^2 xiyi yizi xizi xi yi zi 1] (i=1..N)

	unsigned i,l,c,count = m_associatedCloud->size();
	CCVector3 P;

    //we compute M = [x2 y2 z2 xy yz xz x y z 1] for all points
    PointCoordinateType* M = new PointCoordinateType[count*10];
    if (!M)
        return false;

	PointCoordinateType* _M = M;
	for (i=0;i<count;++i)
	{
		P = *m_associatedCloud->getPoint(i) - *G;

        //we fill the ith line
	    (*_M++) = P.x * P.x;
	    (*_M++) = P.y * P.y;
	    (*_M++) = P.z * P.z;
	    (*_M++) = P.x * P.y;
	    (*_M++) = P.y * P.z;
	    (*_M++) = P.x * P.z;
	    (*_M++) = P.x;
	    (*_M++) = P.y;
	    (*_M++) = P.z;
	    (*_M++) = 1.0;
	}

    //D = tM.M
    SquareMatrixd D(10);
	for (l=0; l<10; ++l)
	{
        for (c=0; c<10; ++c)
        {
            double sum=0.0;
            _M = M;
            for (i=0; i<count; ++i)
            {
                sum += (double)(_M[l] * _M[c]);
                _M+=10;
            }

            D.m_values[l][c] = sum;
        }
	}

    //we don't need M anymore
	delete[] M;
	M=0;

    //now we compute eigen values and vectors of D
	SquareMatrixd eig = D.computeJacobianEigenValuesAndVectors();
    //failure?
	if (!eig.isValid())
		return false;

	//we get the eigen vector corresponding to the minimum eigen value
	double vec[10];
	/*double lambdaMin = */eig.getMinEigenValueAndVector(vec);

    //we store result
	for (i=0;i<10;++i)
        the3DQuadric[i]=vec[i];

	structuresValidity |= QUADRIC_3D;

	return true;
}

GenericIndexedMesh* Neighbourhood::triangulateOnPlane(bool duplicateVertices/*=false*/, PointCoordinateType maxEdgeLength/*=0*/)
{
	if (m_associatedCloud->size()<CC_LOCAL_MODEL_MIN_SIZE[TRI])
	{
		//can't compute LSF plane with less than 3 points!
		return 0;
	}

	//project the points on this plane
	GenericIndexedMesh* mesh = 0;
	std::vector<CCVector2> points2D;

	if (projectPointsOn2DPlane<CCVector2>(points2D))
	{
		Delaunay2dMesh* dm = new Delaunay2dMesh();

		//triangulate the projected points
		if (!dm->build(points2D,0))
		{
			delete dm;
			return 0;
		}

		//change the default mesh's reference
		if (duplicateVertices)
		{
			ChunkedPointCloud* cloud = new ChunkedPointCloud();
			unsigned count = m_associatedCloud->size();
			if (!cloud->reserve(count))
			{
				delete dm;
				delete cloud;
				return 0;
			}
			for (unsigned i=0; i<count; ++i)
				cloud->addPoint(*m_associatedCloud->getPoint(i));
			dm->linkMeshWith(cloud,true);
		}
		else
		{
			dm->linkMeshWith(m_associatedCloud,false);
		}

		//remove triangles with too long edges
		if (maxEdgeLength > 0)
		{
			dm->removeTrianglesLongerThan(maxEdgeLength);
			if (dm->size() == 0)
			{
				//no more triangles?
				delete dm;
				dm = 0;
			}
		}
		mesh = static_cast<GenericIndexedMesh*>(dm);
	}

	return mesh;
}

GenericIndexedMesh* Neighbourhood::triangulateFromQuadric(unsigned nStepX, unsigned nStepY)
{
	if (nStepX<2 || nStepY<2)
		return 0;

	//qaudric fit
	const PointCoordinateType* Q = getHeightFunction(); //Q: Z = a + b.X + c.Y + d.X^2 + e.X.Y + f.Y^2
	if (!Q)
		return 0;

	const PointCoordinateType& a=Q[0];
	const PointCoordinateType& b=Q[1];
	const PointCoordinateType& c=Q[2];
	const PointCoordinateType& d=Q[3];
	const PointCoordinateType& e=Q[4];
	const PointCoordinateType& f=Q[5];

	const uchar hfX = theHeightFunctionDirections[0];
	const uchar hfY = theHeightFunctionDirections[1];
	const uchar hfZ = theHeightFunctionDirections[2];

	//gravity center (should be ok if HF is ok)
	const CCVector3* G = getGravityCenter();
	assert(G);

	//bounding box
	PointCoordinateType bbMin[3], bbMax[3];
	m_associatedCloud->getBoundingBox(bbMin,bbMax);
	CCVector3 bboxDiag = CCVector3(bbMax)-CCVector3(bbMin);

	//Sample points on Quadric and triangulate them!
	PointCoordinateType spanX = bboxDiag.u[hfX];
	PointCoordinateType spanY = bboxDiag.u[hfY];
	PointCoordinateType stepX = spanX/(nStepX-1);
	PointCoordinateType stepY = spanY/(nStepY-1);

	ChunkedPointCloud* vertices = new ChunkedPointCloud();
	if (!vertices->reserve(nStepX*nStepY))
	{
		delete vertices;
		return 0;
	}

	SimpleMesh* quadMesh = new SimpleMesh(vertices,true);
	if (!quadMesh->reserve((nStepX-1)*(nStepY-1)*2))
	{
		delete quadMesh;
		return 0;
	}

	for (unsigned x=0;x<nStepX;++x)
	{
		CCVector3 P;
		P.x = bbMin[hfX] + stepX*(PointCoordinateType)x - G->u[hfX];
		for (unsigned y=0;y<nStepY;++y)
		{
			P.y = bbMin[hfY] + stepY*(PointCoordinateType)y - G->u[hfY];
			P.z = a+b*P.x+c*P.y+d*P.x*P.x+e*P.x*P.y+f*P.y*P.y;

			CCVector3 Pc;
			Pc.u[hfX] = P.x;
			Pc.u[hfY] = P.y;
			Pc.u[hfZ] = P.z;
			Pc += *G;

			vertices->addPoint(Pc.u);

			if (x>0 && y>0)
			{
				unsigned iA = (x-1) * nStepY + y-1;
				unsigned iB = iA+1;
				unsigned iC = iA+nStepY;
				unsigned iD = iB+nStepY;

				quadMesh->addTriangle(iA,iC,iB);
				quadMesh->addTriangle(iB,iC,iD);
			}
		}
	}

	return quadMesh;
}

ScalarType Neighbourhood::computeCurvature(unsigned neighbourIndex, CC_CURVATURE_TYPE cType)
{
	//we get 2D1/2 quadric parameters
	const PointCoordinateType* H = getHeightFunction();
	if (!H)
        return NAN_VALUE;

	//compute centroid
	const CCVector3* G = getGravityCenter();

    //we compute curvature at the input neighbour position + we recenter it by the way
	CCVector3 Q = *m_associatedCloud->getPoint(neighbourIndex) - *G;

	uchar X = theHeightFunctionDirections[0];
	uchar Y = theHeightFunctionDirections[1];

	//z = a+b.x+c.y+d.x^2+e.x.y+f.y^2
	//PointCoordinateType& a = H[0];
	const PointCoordinateType& b = H[1];
	const PointCoordinateType& c = H[2];
	const PointCoordinateType& d = H[3];
	const PointCoordinateType& e = H[4];
	const PointCoordinateType& f = H[5];

    //See "CURVATURE OF CURVES AND SURFACES � A PARABOLIC APPROACH" by ZVI HAR�EL
	const PointCoordinateType fxx = 2.0f*d; // 2d
	const PointCoordinateType& fxy = e; // e
	const PointCoordinateType fyy = 2.0f*f; // 2f
	const PointCoordinateType fx = b+fxx*Q.u[X]+fxy*Q.u[Y]; // b+2d*X+eY
	const PointCoordinateType fy = c+fyy*Q.u[Y]+fxy*Q.u[X]; // c+2f*Y+eX

    switch (cType)
    {
        case GAUSSIAN_CURV:
			{
				//to sign the curvature, we need a normal!
				PointCoordinateType c = fabs((fxx*fyy - fxy*fxy)/(1 + fx*fx + fy*fy));
				return static_cast<ScalarType>(c);
			}

        case MEAN_CURV:
            {
                PointCoordinateType fx2 = fx*fx;
                PointCoordinateType fy2 = fy*fy;
                //to sign the curvature, we need a normal!
				PointCoordinateType c = fabs(((1+fx2)*fyy - 2*fx*fy*fxy + (1+fy2)*fxx)/(2*pow(1+fx2+fy2,static_cast<PointCoordinateType>(1.5))));
				return static_cast<ScalarType>(c);
            }

		default:
			assert(false);
    }

	return NAN_VALUE;
}

ScalarType Neighbourhood::computeCurvature2(unsigned neighbourIndex, CC_CURVATURE_TYPE cType)
{
	//we get 3D quadric parameters
	const double* Q = get3DQuadric();
	if (!Q)
        return NAN_VALUE;

	//we get centroid (should have already been computed during Quadric computation)
	const CCVector3* Gc = getGravityCenter();

    //we compute curvature at the input neighbour position + we recenter it by the way
	CCVector3 Pc = *m_associatedCloud->getPoint(neighbourIndex) - *Gc;

    double a=Q[0];
    const double b=Q[1]/a;
    const double c=Q[2]/a;
    const double e=Q[3]/a;
    const double f=Q[4]/a;
    const double g=Q[5]/a;
    const double l=Q[6]/a;
    const double m=Q[7]/a;
    const double n=Q[8]/a;
    //const double d=Q[9]/a;
    a=1.0;

    const double& x=Pc.x;
    const double& y=Pc.y;
    const double& z=Pc.z;

    //first order partial derivatives
    const double Fx = 2.*a*x+e*y+g*z+l;
    const double Fy = 2.*b*y+e*x+f*z+m;
    const double Fz = 2.*c*z+f*y+g*x+n;

    const double Fx2 = Fx*Fx;
    const double Fy2 = Fy*Fy;
    const double Fz2 = Fz*Fz;

    //coefficients E,F,G
    const double E = 1.+Fx2/Fz2;
    const double F = Fx2/Fz2;
    const double G = 1.+Fy2/Fz2;

    //second order partial derivatives
    //const double Fxx = 2.*a;
    //const double Fyy = 2.*b;
    //const double Fzz = 2.*c;
    //const double Fxy = e;
    //const double Fyx = e;
    //const double Fyz = f;
    //const double Fzy = f;
    //const double Fxz = g;
    //const double Fzx = g;

    //gradient norm
    const double gradF = sqrt(Fx2+Fy2+Fz2);

    //coefficients L,M,N
    SquareMatrixd D(3);
    D.m_values[0][0] = 2.*a;//Fxx;
    D.m_values[0][1] = g;//Fxz;
    D.m_values[0][2] = Fx;
    D.m_values[1][0] = g;//Fzx;
    D.m_values[1][1] = 2.*c;//Fzz;
    D.m_values[1][2] = Fz;
    D.m_values[2][0] = Fx;
    D.m_values[2][1] = Fz;
    D.m_values[2][2] = 0.0;
    const double L = D.computeDet()/(Fz2*gradF);

    D.m_values[0][0] = e;//Fxy;
    D.m_values[0][1] = f;//Fyz;
    D.m_values[0][2] = Fy;
    /*D.m_values[1][0] = Fzx;
    D.m_values[1][1] = Fzz;
    D.m_values[1][2] = Fz;
    D.m_values[2][0] = Fx;
    D.m_values[2][1] = Fz;
    D.m_values[2][2] = 0.0;
    //*/
    const double M = D.computeDet()/(Fz2*gradF);

      D.m_values[0][0] = 2.*b;//Fyy;
    //D.m_values[0][1] = f;//Fyz;
    //D.m_values[0][2] = Fy;
      D.m_values[1][0] = f;//Fzy;
    //D.m_values[1][1] = Fzz;
    //D.m_values[1][2] = Fz;
      D.m_values[2][0] = Fy;
    //D.m_values[2][1] = Fz;
    //D.m_values[2][2] = 0.0;
    const double N = D.computeDet()/(Fz2*gradF);

    //compute curvature
    SquareMatrixd A(2);
    A.m_values[0][0] = L;
    A.m_values[0][1] = M;
    A.m_values[1][0] = M;
    A.m_values[1][1] = N;

    SquareMatrixd B(2);
    B.m_values[0][0] = E;
    B.m_values[0][1] = F;
    B.m_values[1][0] = F;
    B.m_values[1][1] = G;

    /*FILE* fp = fopen("computeCurvature2_trace.txt","wt");
    fprintf(fp,"Fx=%f\n",Fx);
    fprintf(fp,"Fy=%f\n",Fx);
    fprintf(fp,"Fz=%f\n",Fx);
    fprintf(fp,"E=%f\n",E);
    fprintf(fp,"F=%f\n",F);
    fprintf(fp,"G=%f\n",G);
    fprintf(fp,"L=%f\n",L);
    fprintf(fp,"M=%f\n",M);
    fprintf(fp,"N=%f\n",N);
    fprintf(fp,"A:\n%f %f\n%f %f\n",A.m_values[0][0],A.m_values[0][1],A.m_values[1][0],A.m_values[1][1]);
    fprintf(fp,"B:\n%f %f\n%f %f\n",B.m_values[0][0],B.m_values[0][1],B.m_values[1][0],B.m_values[1][1]);
    //fclose(fp);
    //*/

    //Gaussian curvature K = det(A)/det(B)
    if (cType==GAUSSIAN_CURV)
    {
        ScalarType K = (ScalarType)(A.computeDet() / B.computeDet());
        if (K<-1.0)
            K=-1.0;
        else if (K>1.0)
            K=1.0;
        return K;
    }
    //*/

    SquareMatrixd Binv = B.inv();
    if (!Binv.isValid())
        return NAN_VALUE;

    SquareMatrixd C = B.inv() * A;

    //Mean curvature H = 1/2 trace(B^-1 * A)
    if (cType==MEAN_CURV)
    {
        ScalarType H = (ScalarType)(0.5*C.trace());
        if (H<-1.0)
            H=-1.0;
        else if (H>1.0)
            H=1.0;
        return fabs(H);
    }
    //*/

    //principal curvatures are the eigenvalues k1 and k2 of B^-1 * A
    /*SquareMatrixd eig = C.computeJacobianEigenValuesAndVectors();
	if (!eig.isValid())
		return NAN_VALUE;

    const double k1 = eig.getEigenValueAndVector(0);
    const double k2 = eig.getEigenValueAndVector(1);
    //*/

    /*FILE* fp = fopen("computeCurvature2_trace.txt","a");
    fprintf(fp,"Binv:\n%f %f\n%f %f\n",Binv.values[0][0],Binv.values[0][1],Binv.values[1][0],Binv.values[1][1]);
    fprintf(fp,"C:\n%f %f\n%f %f\n",C.values[0][0],C.values[0][1],C.values[1][0],C.values[1][1]);
    fprintf(fp,"k1=%f\n",k1);
    fprintf(fp,"k2=%f\n",k2);
    fclose(fp);
    //*/

    /*switch (cType)
    {
        case GAUSSIAN_CURV:
            //Gaussian curvature
            return k1*k2;
        case MEAN_CURV:
            //Mean curvature
            return (k1+k2)/2.0;
    }
    //*/

    return NAN_VALUE;
}

