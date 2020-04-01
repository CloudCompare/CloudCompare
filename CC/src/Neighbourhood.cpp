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

#include <Neighbourhood.h>

//local
#include <ConjugateGradient.h>
#include <Delaunay2dMesh.h>
#include <DistanceComputationTools.h>
#include <PointCloud.h>
#include <SimpleMesh.h>

//System
#include <algorithm>

//Eigenvalues decomposition
//#define USE_EIGEN
#ifdef USE_EIGEN
#include <eigen/Eigen/Eigenvalues>
#else
#include <Jacobi.h>
#endif

using namespace CCLib;

#ifdef USE_EIGEN
Eigen::MatrixXd ToEigen(const SquareMatrixd& M)
{
	unsigned sz = M.size();

	Eigen::MatrixXd A(sz, sz);
	for (unsigned c = 0; c < sz; ++c)
	{
		for (unsigned r = 0; r < sz; ++r)
		{
			A(r, c) = M.getValue(r, c);
		}
	}

	return A;
}
#endif

Neighbourhood::Neighbourhood(GenericIndexedCloudPersist* associatedCloud)
	: m_quadricEquationDirections(0, 1, 2)
	, m_structuresValidity(FLAG_DEPRECATED)
	, m_associatedCloud(associatedCloud)
{
	memset(m_quadricEquation,  0, sizeof(PointCoordinateType)*6);
	memset(m_lsPlaneEquation,  0, sizeof(PointCoordinateType)*4);
	
	assert(m_associatedCloud);
}

void Neighbourhood::reset()
{
	m_structuresValidity = FLAG_DEPRECATED;
}

const CCVector3* Neighbourhood::getGravityCenter()
{
	if (!(m_structuresValidity & FLAG_GRAVITY_CENTER))
		computeGravityCenter();
	return ((m_structuresValidity & FLAG_GRAVITY_CENTER) ? &m_gravityCenter : nullptr);
}

void Neighbourhood::setGravityCenter(const CCVector3& G)
{
	m_gravityCenter = G;
	m_structuresValidity |= FLAG_GRAVITY_CENTER;
}

const PointCoordinateType* Neighbourhood::getLSPlane()
{
	if (!(m_structuresValidity & FLAG_LS_PLANE))
		computeLeastSquareBestFittingPlane();
	return ((m_structuresValidity & FLAG_LS_PLANE) ? m_lsPlaneEquation : nullptr);
}

void Neighbourhood::setLSPlane(	const PointCoordinateType eq[4],
								const CCVector3& X,
								const CCVector3& Y,
								const CCVector3& N)
{
	memcpy(m_lsPlaneEquation, eq, sizeof(PointCoordinateType)*4);
	m_lsPlaneVectors[0] = X;
	m_lsPlaneVectors[1] = Y;
	m_lsPlaneVectors[2] = N;
	
	m_structuresValidity |= FLAG_LS_PLANE;
}

const CCVector3* Neighbourhood::getLSPlaneX()
{
	if (!(m_structuresValidity & FLAG_LS_PLANE))
		computeLeastSquareBestFittingPlane();
	return ((m_structuresValidity & FLAG_LS_PLANE) ? m_lsPlaneVectors : nullptr);
}

const CCVector3* Neighbourhood::getLSPlaneY()
{
	if (!(m_structuresValidity & FLAG_LS_PLANE))
		computeLeastSquareBestFittingPlane();
	return ((m_structuresValidity & FLAG_LS_PLANE) ? m_lsPlaneVectors + 1 : nullptr);
}

const CCVector3* Neighbourhood::getLSPlaneNormal()
{
	if (!(m_structuresValidity & FLAG_LS_PLANE))
		computeLeastSquareBestFittingPlane();
	return ((m_structuresValidity & FLAG_LS_PLANE) ? m_lsPlaneVectors + 2 : nullptr);
}

const PointCoordinateType* Neighbourhood::getQuadric(Tuple3ub* dims/*=0*/)
{
	if (!(m_structuresValidity & FLAG_QUADRIC))
	{
		computeQuadric();
	}

	if (dims)
	{
		*dims = m_quadricEquationDirections;
	}

	return ((m_structuresValidity & FLAG_QUADRIC) ? m_quadricEquation : nullptr);
}

void Neighbourhood::computeGravityCenter()
{
	//invalidate previous centroid (if any)
	m_structuresValidity &= (~FLAG_GRAVITY_CENTER);

	assert(m_associatedCloud);
	unsigned count = (m_associatedCloud ? m_associatedCloud->size() : 0);
	if (!count)
		return;

	//sum
	CCVector3d Psum(0,0,0);
	for (unsigned i=0; i<count; ++i)
	{
		const CCVector3* P = m_associatedCloud->getPoint(i);
		Psum.x += P->x;
		Psum.y += P->y;
		Psum.z += P->z;
	}

	setGravityCenter( {
						  static_cast<PointCoordinateType>(Psum.x / count),
						  static_cast<PointCoordinateType>(Psum.y / count),
						  static_cast<PointCoordinateType>(Psum.z / count)
					  } );
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

	for (unsigned i = 0; i < count; ++i)
	{
		const CCVector3 P = *m_associatedCloud->getPoint(i) - *G;

		mXX += static_cast<double>(P.x)*P.x;
		mYY += static_cast<double>(P.y)*P.y;
		mZZ += static_cast<double>(P.z)*P.z;
		mXY += static_cast<double>(P.x)*P.y;
		mXZ += static_cast<double>(P.x)*P.z;
		mYZ += static_cast<double>(P.y)*P.z;
	}

	//symmetry
	CCLib::SquareMatrixd covMat(3);
	covMat.m_values[0][0] = mXX/count;
	covMat.m_values[1][1] = mYY/count;
	covMat.m_values[2][2] = mZZ/count;
	covMat.m_values[1][0] = covMat.m_values[0][1] = mXY/count;
	covMat.m_values[2][0] = covMat.m_values[0][2] = mXZ/count;
	covMat.m_values[2][1] = covMat.m_values[1][2] = mYZ/count;

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
	{
		assert(false);
		return PC_NAN;
	}

	double maxSquareDist = 0;
	for (unsigned i=0; i<pointCount; ++i)
	{
		const CCVector3* P = m_associatedCloud->getPoint(i);
		const double d2 = (*P-*G).norm2();
		if (d2 > maxSquareDist)
			maxSquareDist = d2;
	}

	return static_cast<PointCoordinateType>(sqrt(maxSquareDist));
}

bool Neighbourhood::computeLeastSquareBestFittingPlane()
{
	//invalidate previous LS plane (if any)
	m_structuresValidity &= (~FLAG_LS_PLANE);

	assert(m_associatedCloud);
	unsigned pointCount = (m_associatedCloud ? m_associatedCloud->size() : 0);

	//we need at least 3 points to compute a plane
	static_assert(CC_LOCAL_MODEL_MIN_SIZE[LS] >= 3, "Invalid CC_LOCAL_MODEL_MIN_SIZE size");
	if (pointCount < CC_LOCAL_MODEL_MIN_SIZE[LS])
	{
		//not enough points!
		return false;
	}

	CCVector3 G(0, 0, 0);
	if (pointCount > 3)
	{
		CCLib::SquareMatrixd covMat = computeCovarianceMatrix();

#ifdef USE_EIGEN
		Eigen::Matrix3d A = ToEigen(covMat);
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
		es.compute(A);

		//eigen values (and vectors) are sorted in ascending order
		const auto& eVec = es.eigenvectors();
		
		//get normal
		m_lsPlaneVectors[2] = CCVector3::fromArray(eVec.col(0).data()); //smallest eigenvalue
		//get also X (Y will be deduced by cross product, see below
		m_lsPlaneVectors[0] = CCVector3::fromArray(eVec.col(2).data()); //biggest eigenvalue
#else
		//we determine plane normal by computing the smallest eigen value of M = 1/n * S[(p-µ)*(p-µ)']
		CCLib::SquareMatrixd eigVectors;
		std::vector<double> eigValues;
		if (!Jacobi<double>::ComputeEigenValuesAndVectors(covMat, eigVectors, eigValues, true))
		{
			//failed to compute the eigen values!
			return false;
		}

		//get normal
		{
			CCVector3d vec(0, 0, 1);
			double minEigValue = 0;
			//the smallest eigen vector corresponds to the "least square best fitting plane" normal
			Jacobi<double>::GetMinEigenValueAndVector(eigVectors, eigValues, minEigValue, vec.u);
			m_lsPlaneVectors[2] = CCVector3::fromArray(vec.u);
		}

		//get also X (Y will be deduced by cross product, see below
		{
			CCVector3d vec;
			double maxEigValue = 0;
			Jacobi<double>::GetMaxEigenValueAndVector(eigVectors, eigValues, maxEigValue, vec.u);
			m_lsPlaneVectors[0] = CCVector3::fromArray(vec.u);
		}
#endif
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
		m_lsPlaneVectors[0] = (*B - *A);
		m_lsPlaneVectors[1] = (*C - *A);
		m_lsPlaneVectors[2] = m_lsPlaneVectors[0].cross(m_lsPlaneVectors[1]);

		//the plane passes through any of the 3 points
		G = *A;
	}

	//make sure all vectors are unit!
	if (m_lsPlaneVectors[2].norm2() < ZERO_TOLERANCE)
	{
		//this means that the points are colinear!
		//m_lsPlaneVectors[2] = CCVector3(0,0,1); //any normal will do
		return false;
	}
	else
	{
		m_lsPlaneVectors[2].normalize();
	}
	//normalize X as well
	m_lsPlaneVectors[0].normalize();
	//and update Y
	m_lsPlaneVectors[1] = m_lsPlaneVectors[2].cross(m_lsPlaneVectors[0]);

	//deduce the proper equation
	m_lsPlaneEquation[0] = m_lsPlaneVectors[2].x;
	m_lsPlaneEquation[1] = m_lsPlaneVectors[2].y;
	m_lsPlaneEquation[2] = m_lsPlaneVectors[2].z;

	//eventually we just have to compute the 'constant' coefficient a3
	//we use the fact that the plane pass through G --> GM.N = 0 (scalar prod)
	//i.e. a0*G[0]+a1*G[1]+a2*G[2]=a3
	m_lsPlaneEquation[3] = G.dot(m_lsPlaneVectors[2]);

	m_structuresValidity |= FLAG_LS_PLANE;

	return true;
}

bool Neighbourhood::computeQuadric()
{
	//invalidate previous quadric (if any)
	m_structuresValidity &= (~FLAG_QUADRIC);

	assert(m_associatedCloud);
	if (!m_associatedCloud)
		return false;

	unsigned count = m_associatedCloud->size();
	
	static_assert(CC_LOCAL_MODEL_MIN_SIZE[QUADRIC] >= 5, "Invalid CC_LOCAL_MODEL_MIN_SIZE size");
	if (count < CC_LOCAL_MODEL_MIN_SIZE[QUADRIC])
		return false;

	const PointCoordinateType* lsPlane = getLSPlane();
	if (!lsPlane)
		return false;

	//we get centroid (should already be up-to-date - see computeCovarianceMatrix)
	const CCVector3* G = getGravityCenter();
	assert(G);

	//get the best projection axis
	Tuple3ub idx(0/*x*/,1/*y*/,2/*z*/); //default configuration: z is the "normal" direction, we use (x,y) as the base plane
	const PointCoordinateType nxx = lsPlane[0]*lsPlane[0];
	const PointCoordinateType nyy = lsPlane[1]*lsPlane[1];
	const PointCoordinateType nzz = lsPlane[2]*lsPlane[2];
	if (nxx > nyy)
	{
		if (nxx > nzz)
		{
			//as x is the "normal" direction, we use (y,z) as the base plane
			idx.x = 1/*y*/; idx.y = 2/*z*/; idx.z = 0/*x*/;
		}
	}
	else
	{
		if (nyy > nzz)
		{
			//as y is the "normal" direction, we use (z,x) as the base plane
			idx.x = 2/*z*/; idx.y = 0/*x*/; idx.z = 1/*y*/;
		}
	}

	//compute the A matrix and b vector
	std::vector<float> A;
	std::vector<float> b;
	try
	{
		A.resize(6 * count, 0);
		b.resize(count, 0);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	float lmax2 = 0; //max (squared) dimension

    //for all points
	{
		float* _A = A.data();
		float* _b = b.data();
		for (unsigned i = 0; i < count; ++i)
		{
			CCVector3 P = *m_associatedCloud->getPoint(i) - *G;

			float lX = static_cast<float>(P.u[idx.x]);
			float lY = static_cast<float>(P.u[idx.y]);
			float lZ = static_cast<float>(P.u[idx.z]);

			*_A++ = 1.0f;
			*_A++ = lX;
			*_A++ = lY;
			*_A = lX*lX;
			//by the way, we track the max 'X' squared dimension
			if (*_A > lmax2)
				lmax2 = *_A;
			++_A;
			*_A++ = lX*lY;
			*_A = lY*lY;
			//by the way, we track the max 'Y' squared dimension
			if (*_A > lmax2)
				lmax2 = *_A;
			++_A;

			*_b++ = lZ;
			lZ *= lZ;
			//and don't forget to track the max 'Z' squared dimension as well
			if (lZ > lmax2)
				lmax2 = lZ;
		}
	}

	//conjugate gradient initialization
	//we solve tA.A.X=tA.b
	ConjugateGradient<6,double> cg;
	const CCLib::SquareMatrixd& tAA = cg.A();
	double* tAb = cg.b();

	//compute tA.A and tA.b
	{
		for (unsigned i=0; i<6; ++i)
		{
			//tA.A part
			for (unsigned j=i; j<6; ++j)
			{
				double tmp = 0;
				float* _Ai = &(A[i]);
				float* _Aj = &(A[j]);
				for (unsigned k = 0; k < count; ++k, _Ai += 6, _Aj += 6)
				{
					//tmp += A[(6*k)+i] * A[(6*k)+j];
					tmp += static_cast<double>(*_Ai) * static_cast<double>(*_Aj);
				}
				tAA.m_values[j][i] = tAA.m_values[i][j] = tmp;
			}

			//tA.b part
			{
				double tmp = 0;
				float* _Ai = &(A[i]);
				for (unsigned k = 0; k<count; ++k, _Ai += 6)
				{
					//tmp += A[(6*k)+i]*b[k];
					tmp += static_cast<double>(*_Ai) * static_cast<double>(b[k]);
				}
				tAb[i] = tmp;
			}
		}

#if 0
		//trace tA.A and tA.b to a file
		FILE* f = nullptr;
		fopen_s(&f, "CG_trace.txt", "wt");
		if (f)
		{
			fprintf_s(f, "lmax2 = %3.12f\n", lmax2);

			{
				float Amin = 0, Amax = 0;
				Amin = Amax = A[0];
				for (unsigned i = 1; i < 6 * count; ++i)
				{
					Amin = std::min(A[i], Amin);
					Amax = std::max(A[i], Amax);
				}
				fprintf_s(f, "A in [%3.12f ; %3.12f]\n", Amin, Amax);
			}
			{
				float bmin = 0, bmax = 0;
				bmin = bmax = b[0];
				for (unsigned i = 1; i < count; ++i)
				{
					bmin = std::min(b[i], bmin);
					bmax = std::max(b[i], bmax);
				}
				fprintf_s(f, "b in [%3.12f ; %3.12f]\n", bmin, bmax);
			}

			fprintf_s(f, "tA.A\n");
			for (unsigned i = 0; i<6; ++i)
			{
				for (unsigned j = 0; j < 6; ++j)
				{
					fprintf_s(f, "%3.12f ", tAA.m_values[i][j]);
				}
				fprintf_s(f, "\n");
			}

			//tA.b part
			fprintf_s(f, "tA.b\n");
			for (unsigned i = 0; i<6; ++i)
			{
				fprintf_s(f, "%3.12f ", tAb[i]);
			}
			fprintf_s(f, "\n");

			fclose(f);
		}
#endif
	}

	//first guess for X: plane equation (a0.x+a1.y+a2.z=a3 --> z = a3/a2 - a0/a2.x - a1/a2.y)
	double X0[6] = {static_cast<double>(/*lsPlane[3]/lsPlane[idx.z]*/0), //DGM: warning, points have already been recentred around the gravity center! So forget about a3
					static_cast<double>(-lsPlane[idx.x]/lsPlane[idx.z]),
					static_cast<double>(-lsPlane[idx.y]/lsPlane[idx.z]),
					0,
					0,
					0 };

	//special case: a0 = a1 = a2 = 0! //happens for perfectly flat surfaces!
	if (X0[1] == 0 && X0[2] == 0)
	{
		X0[0] = 1.0;
	}

	//init. conjugate gradient
	cg.initConjugateGradient(X0);

	//conjugate gradient iterations
	{
		const double convergenceThreshold = lmax2 * 1.0e-8;  //max. error for convergence = 1e-8 of largest cloud dimension (empirical!)
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

	//output
	{
		for (unsigned i=0; i<6; ++i)
		{
			m_quadricEquation[i] = static_cast<PointCoordinateType>(X0[i]);
		}
		m_quadricEquationDirections = idx;

		m_structuresValidity |= FLAG_QUADRIC;
	}

	return true;
}

bool Neighbourhood::compute3DQuadric(double quadricEquation[10])
{
	if (!m_associatedCloud || !quadricEquation)
	{
		//invalid (input) parameters
		assert(false);
		return false;
	}

	//computes a 3D quadric of the form ax2 +by2 +cz2 + 2exy + 2fyz + 2gzx + 2lx + 2my + 2nz + d = 0
	//"THREE-DIMENSIONAL SURFACE CURVATURE ESTIMATION USING QUADRIC SURFACE PATCHES", I. Douros & B. Buxton, University College London

	//we get centroid
	const CCVector3* G = getGravityCenter();
	assert(G);

	//we look for the eigen vector associated to the minimum eigen value of a matrix A
	//where A=transpose(D)*D, and D=[xi^2 yi^2 zi^2 xiyi yizi xizi xi yi zi 1] (i=1..N)

	unsigned count = m_associatedCloud->size();

	//we compute M = [x2 y2 z2 xy yz xz x y z 1] for all points
	std::vector<PointCoordinateType> M;
	{
		try
		{
			M.resize(count*10);
		}
		catch (const std::bad_alloc&)
		{
			return false;
		}

		PointCoordinateType* _M = M.data();
		for (unsigned i = 0; i < count; ++i)
		{
			const CCVector3 P = *m_associatedCloud->getPoint(i) - *G;

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
			(*_M++) = 1;
		}
	}

	//D = tM.M
	SquareMatrixd D(10);
	for (unsigned l = 0; l < 10; ++l)
	{
		for (unsigned c = 0; c < 10; ++c)
		{
			double sum = 0;
			const PointCoordinateType* _M = M.data();
			for (unsigned i = 0; i < count; ++i, _M += 10)
				sum += static_cast<double>(_M[l] * _M[c]);

			D.m_values[l][c] = sum;
		}
	}

	//we don't need M anymore
	M.resize(0);

	//now we compute eigen values and vectors of D
#ifdef USE_EIGEN
	Eigen::MatrixXd A = ToEigen(D);
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es;
	es.compute(A);

	//eigen values (and vectors) are sorted in ascending order
	//(we get the eigen vector corresponding to the minimum eigen value)
	const auto& minEigenVec = es.eigenvectors().col(0);

	for (unsigned i = 0; i < D.size(); ++i)
	{
		quadricEquation[i] = minEigenVec[i];
	}
#else
	CCLib::SquareMatrixd eigVectors;
	std::vector<double> eigValues;
	if (!Jacobi<double>::ComputeEigenValuesAndVectors(D, eigVectors, eigValues, true))
	{
		//failure
		return false;
	}

	//we get the eigen vector corresponding to the minimum eigen value
	double minEigValue = 0;
	Jacobi<double>::GetMinEigenValueAndVector(eigVectors, eigValues, minEigValue, quadricEquation);
#endif

	return true;
}

GenericIndexedMesh* Neighbourhood::triangulateOnPlane(	bool duplicateVertices/*=false*/,
														PointCoordinateType maxEdgeLength/*=0*/,
														char* errorStr/*=0*/)
{
	if (m_associatedCloud->size() < CC_LOCAL_MODEL_MIN_SIZE[TRI])
	{
		//can't compute LSF plane with less than 3 points!
		if (errorStr)
			strcpy(errorStr,"Not enough points");
		return nullptr;
	}

	//safety check: Triangle lib will crash if the points are all the same!
	if (computeLargestRadius() < ZERO_TOLERANCE)
	{
		return nullptr;
	}

	//project the points on this plane
	GenericIndexedMesh* mesh = nullptr;
	std::vector<CCVector2> points2D;

	if (projectPointsOn2DPlane<CCVector2>(points2D))
	{
		Delaunay2dMesh* dm = new Delaunay2dMesh();

		//triangulate the projected points
		if (!dm->buildMesh(points2D,0,errorStr))
		{
			delete dm;
			return nullptr;
		}

		//change the default mesh's reference
		if (duplicateVertices)
		{
			PointCloud* cloud = new PointCloud();
			const unsigned count = m_associatedCloud->size();
			if (!cloud->reserve(count))
			{
				if (errorStr)
					strcpy(errorStr,"Not enough memory");
				delete dm;
				delete cloud;
				return nullptr;
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
			dm->removeTrianglesWithEdgesLongerThan(maxEdgeLength);
			if (dm->size() == 0)
			{
				//no more triangles?
				if (errorStr)
					strcpy(errorStr,"Not triangle left after pruning");
				delete dm;
				dm = nullptr;
			}
		}
		mesh = static_cast<GenericIndexedMesh*>(dm);
	}

	return mesh;
}

GenericIndexedMesh* Neighbourhood::triangulateFromQuadric(unsigned nStepX, unsigned nStepY)
{
	if (nStepX<2 || nStepY<2)
		return nullptr;

	//qaudric fit
	const PointCoordinateType* Q = getQuadric(); //Q: Z = a + b.X + c.Y + d.X^2 + e.X.Y + f.Y^2
	if (!Q)
		return nullptr;

	const PointCoordinateType& a = Q[0];
	const PointCoordinateType& b = Q[1];
	const PointCoordinateType& c = Q[2];
	const PointCoordinateType& d = Q[3];
	const PointCoordinateType& e = Q[4];
	const PointCoordinateType& f = Q[5];

	const unsigned char X = m_quadricEquationDirections.x;
	const unsigned char Y = m_quadricEquationDirections.y;
	const unsigned char Z = m_quadricEquationDirections.z;

	//gravity center (should be ok if the quadric is ok)
	const CCVector3* G = getGravityCenter();
	assert(G);

	//bounding box
	CCVector3 bbMin;
	CCVector3 bbMax;
	m_associatedCloud->getBoundingBox(bbMin, bbMax);
	CCVector3 bboxDiag = bbMax - bbMin;

	//Sample points on Quadric and triangulate them!
	const PointCoordinateType spanX = bboxDiag.u[X];
	const PointCoordinateType spanY = bboxDiag.u[Y];
	const PointCoordinateType stepX = spanX / (nStepX - 1);
	const PointCoordinateType stepY = spanY / (nStepY - 1);

	PointCloud* vertices = new PointCloud();
	if (!vertices->reserve(nStepX*nStepY))
	{
		delete vertices;
		return nullptr;
	}

	SimpleMesh* quadMesh = new SimpleMesh(vertices, true);
	if (!quadMesh->reserve((nStepX - 1)*(nStepY - 1) * 2))
	{
		delete quadMesh;
		return nullptr;
	}

	for (unsigned x = 0; x < nStepX; ++x)
	{
		CCVector3 P;
		P.x = bbMin[X] + stepX * x - G->u[X];
		for (unsigned y = 0; y < nStepY; ++y)
		{
			P.y = bbMin[Y] + stepY * y - G->u[Y];
			P.z = a + b * P.x + c * P.y + d * P.x*P.x + e * P.x*P.y + f * P.y*P.y;

			CCVector3 Pc;
			Pc.u[X] = P.x;
			Pc.u[Y] = P.y;
			Pc.u[Z] = P.z;
			Pc += *G;

			vertices->addPoint(Pc);

			if (x>0 && y>0)
			{
				const unsigned iA = (x - 1) * nStepY + y - 1;
				const unsigned iB = iA + 1;
				const unsigned iC = iA + nStepY;
				const unsigned iD = iB + nStepY;

				quadMesh->addTriangle(iA,iC,iB);
				quadMesh->addTriangle(iB,iC,iD);
			}
		}
	}

	return quadMesh;
}

ScalarType Neighbourhood::computeMomentOrder1(const CCVector3& P)
{
	if (!m_associatedCloud || m_associatedCloud->size() < 3)
	{
		//not enough points
		return NAN_VALUE;
	}

	SquareMatrixd eigVectors;
	std::vector<double> eigValues;
	if (!Jacobi<double>::ComputeEigenValuesAndVectors(computeCovarianceMatrix(), eigVectors, eigValues, true))
	{
		//failed to compute the eigen values
		return NAN_VALUE;
	}

	Jacobi<double>::SortEigenValuesAndVectors(eigVectors, eigValues); //sort the eigenvectors in decreasing order of their associated eigenvalues

	double m1 = 0.0;
	double m2 = 0.0;
	CCVector3d e2;
	Jacobi<double>::GetEigenVector(eigVectors, 1, e2.u);

	for (unsigned i = 0; i < m_associatedCloud->size(); ++i)
	{
		double dotProd = CCVector3d::fromArray((*m_associatedCloud->getPoint(i) - P).u).dot(e2);
		m1 += dotProd;
		m2 += dotProd * dotProd;
	}

	//see "Contour detection in unstructured 3D point clouds", Hackel et al 2016
	return (m2 < std::numeric_limits<double>::epsilon() ? NAN_VALUE : static_cast<ScalarType>((m1 * m1) / m2));
}

double Neighbourhood::computeFeature(GeomFeature feature)
{
	if (!m_associatedCloud || m_associatedCloud->size() < 3)
	{
		//not enough points
		return std::numeric_limits<double>::quiet_NaN();
	}
	
	SquareMatrixd eigVectors;
	std::vector<double> eigValues;
	if (!Jacobi<double>::ComputeEigenValuesAndVectors(computeCovarianceMatrix(), eigVectors, eigValues, true))
	{
		//failed to compute the eigen values
		return std::numeric_limits<double>::quiet_NaN();
	}

	Jacobi<double>::SortEigenValuesAndVectors(eigVectors, eigValues); //sort the eigenvectors in decreasing order of their associated eigenvalues

	//shortcuts
	const double& l1 = eigValues[0];
	const double& l2 = eigValues[1];
	const double& l3 = eigValues[2];

	double value = std::numeric_limits<double>::quiet_NaN();

	switch (feature)
	{
	case EigenValuesSum:
		value = l1 + l2 + l3;
		break;
	case Omnivariance:
		value = pow(l1 * l2 * l3, 1.0/3.0);
		break;
	case EigenEntropy:
		value = -(l1 * log(l1) + l2 * log(l2) + l3 * log(l3));
		break;
	case Anisotropy:
		if (std::abs(l1) > std::numeric_limits<double>::epsilon())
			value = (l1 - l3) / l1;
		break;
	case Planarity:
		if (std::abs(l1) > std::numeric_limits<double>::epsilon())
			value = (l2 - l3) / l1;
		break;
	case Linearity:
		if (std::abs(l1) > std::numeric_limits<double>::epsilon())
			value = (l1 - l2) / l1;
		break;
	case PCA1:
		{
			double sum = l1 + l2 + l3;
			if (std::abs(sum) > std::numeric_limits<double>::epsilon())
				value = l1 / sum;
		}
		break;
	case PCA2:
		{
			double sum = l1 + l2 + l3;
			if (std::abs(sum) > std::numeric_limits<double>::epsilon())
				value = l2 / sum;
		}
		break;
	case SurfaceVariation:
		{
			double sum = l1 + l2 + l3;
			if (std::abs(sum) > std::numeric_limits<double>::epsilon())
				value = l3 / sum;
		}
		break;
	case Sphericity:
		if (std::abs(l1) > std::numeric_limits<double>::epsilon())
			value = l3 / l1;
		break;
	case Verticality:
		{
			CCVector3d Z(0, 0, 1);
			CCVector3d e3(Z);
			Jacobi<double>::GetEigenVector(eigVectors, 2, e3.u);

			value = 1.0 - std::abs(Z.dot(e3));
		}
		break;
	case EigenValue1:
		value = l1;
		break;
	case EigenValue2:
		value = l2;
		break;
	case EigenValue3:
		value = l3;
		break;
	default:
		assert(false);
		break;
	}

	return value;
}

ScalarType Neighbourhood::computeRoughness(const CCVector3& P)
{
	const PointCoordinateType* lsPlane = getLSPlane();
	if (lsPlane)
	{
		return std::abs(DistanceComputationTools::computePoint2PlaneDistance(&P, lsPlane));
	}
	else
	{
		return NAN_VALUE;
	}
}

ScalarType Neighbourhood::computeCurvature(const CCVector3& P, CurvatureType cType)
{
	switch (cType)
	{
	case GAUSSIAN_CURV:
	case MEAN_CURV:
		{
			//we get 2D1/2 quadric parameters
			const PointCoordinateType* H = getQuadric();
			if (!H)
				return NAN_VALUE;

			//compute centroid
			const CCVector3* G = getGravityCenter();

			//we compute curvature at the input neighbour position + we recenter it by the way
			const CCVector3 Q(P - *G);

			const unsigned char X = m_quadricEquationDirections.x;
			const unsigned char Y = m_quadricEquationDirections.y;

			//z = a+b.x+c.y+d.x^2+e.x.y+f.y^2
			//const PointCoordinateType& a = H[0];
			const PointCoordinateType& b = H[1];
			const PointCoordinateType& c = H[2];
			const PointCoordinateType& d = H[3];
			const PointCoordinateType& e = H[4];
			const PointCoordinateType& f = H[5];

			//See "CURVATURE OF CURVES AND SURFACES – A PARABOLIC APPROACH" by ZVI HAR’EL
			const PointCoordinateType  fx	= b + (d*2) * Q.u[X] + (e  ) * Q.u[Y];	// b+2d*X+eY
			const PointCoordinateType  fy	= c + (e  ) * Q.u[X] + (f*2) * Q.u[Y];	// c+2f*Y+eX
			const PointCoordinateType  fxx	= d*2;									// 2d
			const PointCoordinateType  fyy	= f*2;									// 2f
			const PointCoordinateType& fxy	= e;									// e

			const PointCoordinateType fx2 = fx*fx;
			const PointCoordinateType fy2 = fy*fy;
			const PointCoordinateType q = (1 + fx2 + fy2);

			switch (cType)
			{
			case GAUSSIAN_CURV:
				{
					//to sign the curvature, we need a normal!
					const PointCoordinateType K = std::abs(fxx*fyy - fxy * fxy) / (q*q);
					return static_cast<ScalarType>(K);
				}

			case MEAN_CURV:
				{
					//to sign the curvature, we need a normal!
					const PointCoordinateType H2 = std::abs(((1 + fx2)*fyy - 2 * fx*fy*fxy + (1 + fy2)*fxx)) / (2 * sqrt(q)*q);
					return static_cast<ScalarType>(H2);
				}

			default:
				assert(false);
				break;
			}
		}
		break;

	case NORMAL_CHANGE_RATE:
		{
			assert(m_associatedCloud);
			unsigned pointCount = (m_associatedCloud ? m_associatedCloud->size() : 0);

			//we need at least 4 points
			if (pointCount < 4)
			{
				//not enough points!
				return pointCount == 3 ? 0 : NAN_VALUE;
			}

			//we determine plane normal by computing the smallest eigen value of M = 1/n * S[(p-µ)*(p-µ)']
			CCLib::SquareMatrixd covMat = computeCovarianceMatrix();
			CCVector3d e(0, 0, 0);
#ifdef USE_EIGEN
			Eigen::Matrix3d A = ToEigen(covMat);
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
			es.compute(A);

			//eigen values (and vectors) are sorted in ascending order
			const auto& eVal = es.eigenvalues();

			//compute curvature as the rate of change of the surface
			e = CCVector3d::fromArray(eVal.data());
#else
			CCLib::SquareMatrixd eigVectors;
			std::vector<double> eigValues;
			if (!Jacobi<double>::ComputeEigenValuesAndVectors(covMat, eigVectors, eigValues, true))
			{
				//failure
				return NAN_VALUE;
			}

			//compute curvature as the rate of change of the surface
			e.x = eigValues[0];
			e.y = eigValues[1];
			e.z = eigValues[2];
#endif
			const double sum = e.x + e.y + e.z; //we work with absolute values
			if (sum < ZERO_TOLERANCE)
			{
				return NAN_VALUE;
			}

			const double eMin = std::min(std::min(e.x, e.y), e.z);
			return static_cast<ScalarType>(eMin / sum);
		}
		break;

	default:
		assert(false);
	}

	return NAN_VALUE;
}
