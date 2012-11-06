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
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#ifndef CCLIB_NEIGHBOURHOOD_HEADER
#define CCLIB_NEIGHBOURHOOD_HEADER

#include "GenericIndexedCloudPersist.h"
#include "Matrix.h"
#include "CCGeom.h"

#include <vector>

namespace CCLib
{

class GenericIndexedMesh;

//For more precise computations
//#define CC_NEIGHBOURHOOD_PRECISION_COMPUTINGS

//! 2D points container
typedef std::vector<CCVector2> CC2DPointsConainer;

//! A specific point could structure to handle subsets of points, provided with several geometric processings
/** Typically suited for "nearest neighbours".
	It implements the GenericIndexCloud interface by
	inheriting the ReferenceCloud class.
**/

#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"
class CC_DLL_API Neighbourhood
#else
class Neighbourhood
#endif
{
	public:

		//! Geometric properties/elements that can be computed from the set of points (see Neighbourhood::getGeometricalElement)
		enum CC_GEOM_ELEMENT {  DEPRECATED=0,
                                GRAVITY_CENTER=1,
                                LSQ_PLANE=2,
                                HEIGHT_FUNCTION=4,
                                HEIGHT_FUNCTION_DIRECTIONS=8,
                                QUADRIC_3D=16};

		//! Curvature type
		enum CC_CURVATURE_TYPE {GAUSSIAN_CURV, MEAN_CURV};

		//! Default constructor
		/** \param associatedCloud reference cloud
		**/
		Neighbourhood(GenericIndexedCloudPersist* associatedCloud);

		//! Default destructor
		virtual ~Neighbourhood() {};

		//! Resets structure (depreactes all associated geometrical fetaures)
		virtual void reset();

		//! Returns associated cloud
		GenericIndexedCloudPersist* associatedCloud() const {return m_associatedCloud;}

		//inherited from ReferenceCloud
		/*virtual void addPointIndex(unsigned globalIndex);
		virtual bool addPointIndex(unsigned firstIndex, unsigned lastIndex);
		virtual void setPointIndex(unsigned localIndex, unsigned globalIndex);
        virtual bool resize(unsigned n);
        virtual void swap(unsigned i, unsigned j);
		virtual void clear(bool releaseMemory);
		virtual void removePointGlobalIndex(unsigned localIndex);
		**/

		//! Applies 2D Delaunay triangulation
		/** Cloud selection is first projected on the best least-square plane.
			\param duplicateVertices whether to duplicate vertices (a new point cloud is created) or to use the associated one)
		***/
		GenericIndexedMesh* triangulateOnPlane(bool duplicateVertices=false);

		//! Fit a quadric on point set (see getHeightFunction) then triangulates it inside bounding box
		GenericIndexedMesh* triangulateFromQuadric(unsigned stepsX, unsigned stepsY);

		//! Projects points on the best least-square plane.
		/** Projected points are stored in the2DPoints.
		**/
		bool projectPointsOnPlane(const PointCoordinateType* thePlaneEquation, 
									CC2DPointsConainer &the2DPoints, 
									DistanceType &error, 
									bool distanceNeeded=false);

		//! Computes point set curvature with height function
		/** \return curvature value (warning: unsigned value!) or HIDDEN_VALUE if computation failed.
		**/
		DistanceType computeCurvature(unsigned neighbourIndex, CC_CURVATURE_TYPE cType);

		//! Computes point set curvature with 3D Quadric (WORK IN PROGRESS)
		/** \return curvature value (warning: signed value!) or BIG_VALUE if computation failed.
        **/
		DistanceType computeCurvature2(unsigned neighbourIndex, CC_CURVATURE_TYPE cType);

		/**** GETTERS ****/

		//! Returns gravity center
		/** \return 0 if computation failed
		**/
		const CCVector3* getGravityCenter();

		//! Returns best interpolating plane (Least-square)
		/** Returns an array of the form [a,b,c,d] such as:
				ax+by+cz=d
			\return 0 if computation failed
		**/
		const PointCoordinateType* getLSQPlane();

		//! Returns the best interpolating 'height function'
		/** Returns an array of the form [a,b,c,d,e,f] such as:
				Z = a + b.X + c.Y + d.X^2 + e.X.Y + f.Y^2
            Warning: 'X','Y' and 'Z' are output in dimsHF (optional):
				dimsHF=[index(X),index(Y),index(Z)] where: 0=x, 1=y, 2=z
			\return 0 if computation failed
		**/
		const PointCoordinateType* getHeightFunction(uchar* dimsHF=0);

		//! Returns the best interpolating quadric (Least-square)
		/** Returns an array [a,b,c,d,e,f,g,l,m,n] such as
				a.x^2+b.y^2+c.z^2+2e.x.y+2f.y.z+2g.z.x+2l.x+2m.y+2n.z+d = 0
			\return 0 if computation failed
		**/
		const double* get3DQuadric();

		//! Computes the covariance matrix
		CCLib::SquareMatrixd computeCovarianceMatrix();

	protected:

		//! Height function parameters accessor
		/** If zero, parameters are not yet computed.
		**/
		PointCoordinateType* _theHeightFunction;
		//! Height function parameters
		/** Array [a,b,c,d,e,f] such as: Z = a + b.X + c.Y + d.X^2 + e.X.Y + f.Y^2.
            Warning: 'X','Y' and 'Z' are defined by 'theHeightFunctionDirections'
            Shouldn't be used directly: use _theHeightFunction instead.
		**/
		PointCoordinateType theHeightFunction[6];
		//! Height function dimensions
		/** Array (index(X),index(Y),index(Z)) where: 0=x, 1=y, 2=z.
            Only valid if _theHeightFunction is non zero.
		**/
		uchar theHeightFunctionDirections[3];
		
		//! Least-square best fitting plane parameters accessor
		/** If zero, parameters are not yet computed.
		**/
		PointCoordinateType* _theLSQPlane;
		//! Least-square best fitting plane parameters
		/** Array [a,b,c,d] such as: ax+by+cz=d.
            Shouldn't be used directly: use _theLSQPlane instead.
		**/
		PointCoordinateType theLSQPlane[4];
		
		//! Least-square best fitting 3D quadric parameters accessor
		/** If zero, parameters are not yet computed.
		**/
		double* _the3DQuadric;
		//! Least-square best fitting 3D quadric parameters
		/** Array [a,b,c,d,e,f,g,l,m,n] such as
            a.x^2+b.y^2+c.z^2+2e.x.y+2f.y.z+2g.z.x+2l.x+2m.y+2n.z+d = 0.
            Shouldn't be used directly: use _the3DQuadric instead.
		**/
		double the3DQuadric[10];
		
		//! Gravity center accessor
		/** If zero, parameters are not yet computed.
		**/
		CCVector3* _theGravityCenter;
		//! Gravity center
		/** Shouldn't be used directly: use _theGravityCenter instead.
		**/
		CCVector3 theGravityCenter;
		
		//! Geometrical elements validity
		uchar structuresValidity;

		//! Computes the gravity center
		void computeGravityCenter();
		//! Computes the least-square best fitting plane
		bool computeLeastSquareBestFittingPlan();
		//! Computes best fitting height function
		bool computeHeightFunction();
		//! Computes best fitting 3D quadric function
		bool compute3DQuadric();

		//! Associated cloud
		GenericIndexedCloudPersist* m_associatedCloud;
};

}

#endif
