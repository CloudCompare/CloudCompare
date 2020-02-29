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

#ifndef CC_NEIGHBOURHOOD_HEADER
#define CC_NEIGHBOURHOOD_HEADER

//Local
#include "CCMiscTools.h"
#include "GenericIndexedCloudPersist.h"
#include "SquareMatrix.h"


namespace CCLib
{

class GenericIndexedMesh;

//! A specific point could structure to handle subsets of points, provided with several geometric processings
/** Typically suited for "nearest neighbours".
	It implements the GenericIndexCloud interface by
	inheriting the ReferenceCloud class.
**/
class CC_CORE_LIB_API Neighbourhood
{
	public:

		//! Geometric properties/elements that can be computed from the set of points (see Neighbourhood::getGeometricalElement)
		enum GeomElement {		FLAG_DEPRECATED			= 0,
								FLAG_GRAVITY_CENTER		= 1,
								FLAG_LS_PLANE			= 2,
								FLAG_QUADRIC			= 4 };

		//! Curvature type
		enum CurvatureType {	GAUSSIAN_CURV = 1,
								MEAN_CURV,
								NORMAL_CHANGE_RATE};

		//! Default constructor
		/** \param associatedCloud reference cloud
		**/
		explicit Neighbourhood(GenericIndexedCloudPersist* associatedCloud);

		//! Default destructor
		virtual ~Neighbourhood() = default;

		//! Resets structure (depreactes all associated geometrical fetaures)
		virtual void reset();

		//! Returns associated cloud
		GenericIndexedCloudPersist* associatedCloud() const { return m_associatedCloud; }

		//! Applies 2D Delaunay triangulation
		/** Cloud selection is first projected on the best least-square plane.
			\param duplicateVertices whether to duplicate vertices (a new point cloud is created) or to use the associated one)
			\param maxEdgeLength max edge length for output triangles (0 = ignored)
			\param errorStr error (if any) [optional]
		***/
		GenericIndexedMesh* triangulateOnPlane(	bool duplicateVertices = false,
												PointCoordinateType maxEdgeLength = 0,
												char* errorStr = nullptr);

		//! Fit a quadric on point set (see getQuadric) then triangulates it inside bounding box
		GenericIndexedMesh* triangulateFromQuadric(unsigned stepsX, unsigned stepsY);

		enum InputVectorsUsage { UseOXYasBase, UseYAsUpDir, None };

		//! Projects points on the best fitting LS plane
		/** Projected points are stored in the points2D vector.
			\param points2D output set
			\param planeEquation custom plane equation (otherwise the default Neighbouhood's one is used)
			\param O if set, the local plane base origin will be output here
			\param X if set, the local plane base X vector will be output here
			\param Y if set, the local plane base Y vector will be output here
			\param useOXYasBase whether O, X and Y should be used as input base
			\return success
		**/
		template<class Vec2D> bool projectPointsOn2DPlane(	std::vector<Vec2D>& points2D,
															const PointCoordinateType* planeEquation = nullptr,
															CCVector3* O = nullptr,
															CCVector3* X = nullptr,
															CCVector3* Y = nullptr,
															InputVectorsUsage vectorsUsage = None)
		{
			//need at least one point ;)
			unsigned count = (m_associatedCloud ? m_associatedCloud->size() : 0);
			if (!count)
				return false;

			//if no custom plane equation is provided, get the default best LS one
			if (!planeEquation)
			{
				planeEquation = getLSPlane();
				if (!planeEquation)
					return false;
			}

			//reserve memory for output set
			try
			{
				points2D.resize(count);
			}
			catch (const std::bad_alloc&)
			{
				//out of memory
				return false;
			}

			//we construct the plane local base
			CCVector3 G(0, 0, 0), u(1, 0, 0), v(0, 1, 0);
			if ((vectorsUsage == UseOXYasBase) && O && X && Y)
			{
				G = *O;
				u = *X;
				v = *Y;
			}
			else
			{
				CCVector3 N(planeEquation);
				if ((vectorsUsage == UseYAsUpDir) && Y)
				{
					v = (*Y - Y->dot(N) * N);
					v.normalize();
					u = v.cross(N);
				}
				else
				{
					CCMiscTools::ComputeBaseVectors(N, u, v);
				}
				//get the barycenter
				const CCVector3* _G = getGravityCenter();
				assert(_G);
				G = *_G;
			}

			//project the points
			for (unsigned i = 0; i < count; ++i)
			{
				//we recenter current point
				const CCVector3 P = *m_associatedCloud->getPoint(i) - G;

				//then we project it on plane (with scalar prods)
				points2D[i] = Vec2D(P.dot(u), P.dot(v));
			}

			//output the local base if necessary
			if (vectorsUsage != UseOXYasBase)
			{
				if (O)
					*O = G;
				if (X)
					*X = u;
				if (Y)
					*Y = v;
			}

			return true;
		}

		//! Geometric feature computed from eigen values/vectors
		/** Most of them are defined in "Contour detection in unstructured 3D point clouds", Hackel et al, 2016
			PCA1 and PCA2 are defined in "3D terrestrial lidar data classification of complex natural scenes using a multi-scale dimensionality criterion: Applications in geomorphology", Brodu and Lague, 2012
		**/
		enum GeomFeature
		{
			EigenValuesSum = 1,
			Omnivariance,
			EigenEntropy,
			Anisotropy,
			Planarity,
			Linearity,
			PCA1,
			PCA2,
			SurfaceVariation,
			Sphericity,
			Verticality,
			EigenValue1,
			EigenValue2,
			EigenValue3
		};

		//! Computes the given feature on a set of point
		/** \return feature value
		**/
		double computeFeature(GeomFeature feature);

		//! Computes the 1st order moment of a set of point (based on the eigenvalues)
		/** \return 1st order moment at a given position P
			DGM: The article states that the result should be between 0 and 1,
			but this is actually wrong (as (a + b)^2 can be > a^2 + b^2)
		**/
		ScalarType computeMomentOrder1(const CCVector3& P);

		//! Computes the roughness of a set of point (by fitting a 2D plane)
		/** \return roughness value at a given position P
			\warning The point P shouldn't be in the set of points
		**/
		ScalarType computeRoughness(const CCVector3& P);

		//! Computes the curvature of a set of point (by fitting a 2.5D quadric)
		/** \return curvature value at a given position P or NAN_VALUE if the computation failed
			\warning The curvature value is always unsigned
		**/
		ScalarType computeCurvature(const CCVector3& P, CurvatureType cType);

		/**** GETTERS ****/

		//! Returns gravity center
		/** \return 0 if computation failed
		**/
		const CCVector3* getGravityCenter();

		//! Sets gravity center
		/** Handle with care!
			\param G gravity center
		**/
		void setGravityCenter(const CCVector3& G);

		//! Returns best interpolating plane equation (Least-square)
		/** Returns an array of the form [a,b,c,d] such as:
				ax + by + cz = d
			\return 0 if computation failed
		**/
		const PointCoordinateType* getLSPlane();

		//! Sets the best interpolating plane equation (Least-square)
		/** Handle with care!
			\param eq plane equation (ax + by + cz = d)
			\param X local base X vector
			\param Y local base Y vector
			\param N normal vector
		**/
		void setLSPlane(	const PointCoordinateType eq[4],
							const CCVector3& X,
							const CCVector3& Y,
							const CCVector3& N);

		//! Returns best interpolating plane (Least-square) 'X' base vector
		/** This corresponds to the largest eigen value (i.e. the largest cloud dimension)
			\return 0 if computation failed
		**/
		const CCVector3* getLSPlaneX();
		//! Returns best interpolating plane (Least-square) 'Y' base vector
		/** This corresponds to the second largest eigen value (i.e. the second largest cloud dimension)
			\return 0 if computation failed
		**/
		const CCVector3* getLSPlaneY();
		//! Returns best interpolating plane (Least-square) normal vector
		/** This corresponds to the smallest eigen value (i.e. the second largest cloud dimension)
			\return 0 if computation failed
		**/
		const CCVector3* getLSPlaneNormal();

		//! Returns the best interpolating 2.5D quadric
		/** Returns an array of the form [a,b,c,d,e,f] such as:
				Z = a + b.X + c.Y + d.X^2 + e.X.Y + f.Y^2
			\warning: 'X','Y' and 'Z' are output in dims (optional):
				dims = [index(X),index(Y),index(Z)] where: 0=x, 1=y, 2=z
			\return 0 if computation failed
		**/
		const PointCoordinateType* getQuadric(Tuple3ub* dims = nullptr);

		//! Computes the best interpolating quadric (Least-square)
		/** \param[out] quadricEquation an array of 10 coefficients [a,b,c,d,e,f,g,l,m,n] such as
				         a.x^2+b.y^2+c.z^2+2e.x.y+2f.y.z+2g.z.x+2l.x+2m.y+2n.z+d = 0
			\return success
		**/
		bool compute3DQuadric(double quadricEquation[10]);

		//! Computes the covariance matrix
		CCLib::SquareMatrixd computeCovarianceMatrix();

		//! Returns the set 'radius' (i.e. the distance between the gravity center and the its farthest point)
		PointCoordinateType computeLargestRadius();

	protected:

		//! 2.5D Quadric equation
		/** Array [a,b,c,d,e,f] such that Z = a + b.X + c.Y + d.X^2 + e.X.Y + f.Y^2.
			\warning: 'X','Y' and 'Z' are defined by 'm_quadricEquationDirections'
			Only valid if 'structuresValidity & QUADRIC != 0'.
		**/
		PointCoordinateType m_quadricEquation[6];
		
		//! 2.5D Quadric equation dimensions
		/** Array (index(X),index(Y),index(Z)) where: 0=x, 1=y, 2=z
			Only valid if 'structuresValidity & QUADRIC != 0'.
		**/
		Tuple3ub m_quadricEquationDirections;
		
		//! Least-square best fitting plane parameters
		/** Array [a,b,c,d] such that ax+by+cz = d
			Only valid if 'structuresValidity & LS_PLANE != 0'.
		**/
		PointCoordinateType m_lsPlaneEquation[4];
		
		//! Least-square best fitting plane base vectors
		/** Only valid if 'structuresValidity & LS_PLANE != 0'.
		**/
		CCVector3 m_lsPlaneVectors[3];
		
		//! Gravity center
		/** Only valid if 'structuresValidity & GRAVITY_CENTER != 0'.
		**/
		CCVector3 m_gravityCenter;
		
		//! Geometrical elements validity (flags)
		unsigned char m_structuresValidity;

		//! Computes the gravity center
		void computeGravityCenter();
		//! Computes the least-square best fitting plane
		bool computeLeastSquareBestFittingPlane();
		//! Computes best fitting 2.5D quadric
		bool computeQuadric();

		//! Associated cloud
		GenericIndexedCloudPersist* m_associatedCloud;
};

}

#endif //CC_NEIGHBOURHOOD_HEADER
