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

#ifndef CC_NEIGHBOURHOOD_HEADER
#define CC_NEIGHBOURHOOD_HEADER

//Local
#include "CCCoreLib.h"
#include "GenericIndexedCloudPersist.h"
#include "Matrix.h"
#include "CCGeom.h"
#include "CCMiscTools.h"

//system
#include <vector>

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
		enum CC_GEOM_ELEMENT {  DEPRECATED					= 0,
								GRAVITY_CENTER				= 1,
								LSQ_PLANE					= 2,
								HEIGHT_FUNCTION				= 4,
								HEIGHT_FUNCTION_DIRECTIONS	= 8,
								QUADRIC_3D					= 16};

		//! Curvature type
		enum CC_CURVATURE_TYPE {GAUSSIAN_CURV, MEAN_CURV};

		//! Default constructor
		/** \param associatedCloud reference cloud
		**/
		Neighbourhood(GenericIndexedCloudPersist* associatedCloud);

		//! Default destructor
		virtual ~Neighbourhood() {}

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
												char* errorStr = 0);

		//! Fit a quadric on point set (see getHeightFunction) then triangulates it inside bounding box
		GenericIndexedMesh* triangulateFromQuadric(unsigned stepsX, unsigned stepsY);

		//! Projects points on the best fitting LS plane
		/** Projected points are stored in the points2D vector.
			\param points2D output set
			\param planeEquation custom plane equation (otherwise the default Neighbouhood's one is used)
			\param O if set, the local plane base origin will be output here
			\param X if set, the local plane base X vector will be output here
			\param Y if set, the local plane base Y vector will be output here
			\return success
		**/
		template<class Vec2D> bool projectPointsOn2DPlane(	std::vector<Vec2D>& points2D,
															const PointCoordinateType* planeEquation = 0,
															CCVector3* O = 0,
															CCVector3* X = 0,
															CCVector3* Y = 0)
		{
			//need at least one point ;)
			unsigned count = (m_associatedCloud ? m_associatedCloud->size() : 0);
			if (!count)
				return false;

			//if no custom plane equation is provided, get the default best LS one
			if (!planeEquation)
			{
				planeEquation = getLSQPlane();
				if (!planeEquation)
					return false;
			}

			//reserve memory for output set
			try
			{
				points2D.resize(count);
			}
			catch (std::bad_alloc)
			{
				//out of memory
				return false;
			}

			//we construct the plane local base
			CCVector3 u(1,0,0), v(0,1,0);
			{
				CCVector3 N(planeEquation);
				CCMiscTools::ComputeBaseVectors(N,u,v);
			}

			//get the barycenter
			const CCVector3* G = getGravityCenter();
			assert(G);

			//project the points
			for (unsigned i=0; i<count; ++i)
			{
				//we recenter current point
				CCVector3 P = *m_associatedCloud->getPoint(i) - *G;

				//then we project it on plane (with scalar prods)
				points2D[i] = Vec2D(P.dot(u),P.dot(v));
			}

			//output the local base if necessary
			if (O) *O = *G;
			if (X) *X = u;
			if (Y) *Y = v;

			return true;
		}

		//! Computes point set curvature with height function
		/** \return curvature value (warning: unsigned value!) or NAN_VALUE if computation failed.
		**/
		ScalarType computeCurvature(unsigned neighbourIndex, CC_CURVATURE_TYPE cType);

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
		const PointCoordinateType* getLSQPlane();

		//! Sets the best interpolating plane equation (Least-square)
		/** Handle with care!
			\param eq plane equation (ax + by + cz = d)
			\param X local base X vector
			\param Y local base Y vector
			\param N normal vector
		**/
		void setLSQPlane(	const PointCoordinateType eq[4],
							const CCVector3& X,
							const CCVector3& Y,
							const CCVector3& N);

		//! Returns best interpolating plane (Least-square) 'X' base vector
		/** This corresponds to the largest eigen value (i.e. the largest cloud dimension)
			\return 0 if computation failed
		**/
		const CCVector3* getLSQPlaneX();
		//! Returns best interpolating plane (Least-square) 'Y' base vector
		/** This corresponds to the second largest eigen value (i.e. the second largest cloud dimension)
			\return 0 if computation failed
		**/
		const CCVector3* getLSQPlaneY();
		//! Returns best interpolating plane (Least-square) normal vector
		/** This corresponds to the smallest eigen value (i.e. the second largest cloud dimension)
			\return 0 if computation failed
		**/
		const CCVector3* getLSQPlaneNormal();

		//! Returns the best interpolating 'height function'
		/** Returns an array of the form [a,b,c,d,e,f] such as:
				Z = a + b.X + c.Y + d.X^2 + e.X.Y + f.Y^2
			\warning: 'X','Y' and 'Z' are output in dimsHF (optional):
				dimsHF = [index(X),index(Y),index(Z)] where: 0=x, 1=y, 2=z
			\return 0 if computation failed
		**/
		const PointCoordinateType* getHeightFunction(uchar* dimsHF = 0);

		//! Returns the best interpolating quadric (Least-square)
		/** Returns an array [a,b,c,d,e,f,g,l,m,n] such as
				a.x^2+b.y^2+c.z^2+2e.x.y+2f.y.z+2g.z.x+2l.x+2m.y+2n.z+d = 0
			\return 0 if computation failed
		**/
		const double* get3DQuadric();

		//! Computes the covariance matrix
		CCLib::SquareMatrixd computeCovarianceMatrix();

		//! Returns the largest radius (i.e. the distance to the farthest point to the centroid)
		PointCoordinateType computeLargestRadius();

	protected:

		//! Height function parameters
		/** Array [a,b,c,d,e,f] such as: Z = a + b.X + c.Y + d.X^2 + e.X.Y + f.Y^2.
			\warning: 'X','Y' and 'Z' are defined by 'theHeightFunctionDirections'
			Only valid if 'structuresValidity & HEIGHT_FUNCTION != 0'.
		**/
		PointCoordinateType theHeightFunction[6];
		//! Height function dimensions
		/** Array (index(X),index(Y),index(Z)) where: 0=x, 1=y, 2=z.
			Only valid if 'structuresValidity & HEIGHT_FUNCTION != 0'.
		**/
		uchar theHeightFunctionDirections[3];
		
		//! Least-square best fitting plane parameters
		/** Array [a,b,c,d] such as: ax+by+cz=d.
			Only valid if 'structuresValidity & LSQ_PLANE != 0'.
		**/
		PointCoordinateType theLSQPlaneEquation[4];
		//! Least-square best fitting plane base vectors
		CCVector3 theLSQPlaneVectors[3];
		
		//! Least-square best fitting 3D quadric parameters
		/** Array [a,b,c,d,e,f,g,l,m,n] such as
			a.x^2+b.y^2+c.z^2+2e.x.y+2f.y.z+2g.z.x+2l.x+2m.y+2n.z+d = 0.
			Shouldn't be used directly: use _the3DQuadric instead.
			Only valid if 'structuresValidity & QUADRIC_3D != 0'.
		**/
		double the3DQuadric[10];
		
		//! Gravity center
		/** Only valid if 'structuresValidity & GRAVITY_CENTER != 0'.
		**/
		CCVector3 theGravityCenter;
		
		//! Geometrical elements validity
		uchar structuresValidity;

		//! Computes the gravity center
		void computeGravityCenter();
		//! Computes the least-square best fitting plane
		bool computeLeastSquareBestFittingPlane();
		//! Computes best fitting height function
		bool computeHeightFunction();
		//! Computes best fitting 3D quadric function
		bool compute3DQuadric();

		//! Associated cloud
		GenericIndexedCloudPersist* m_associatedCloud;
};

}

#endif //CC_NEIGHBOURHOOD_HEADER
