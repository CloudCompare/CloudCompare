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

#ifndef POINT_PROJECTION_TOOLS_HEADER
#define POINT_PROJECTION_TOOLS_HEADER

//Local
#include "CCConst.h"
#include "CCToolbox.h"
#include "PointCloud.h"
#include "SquareMatrix.h"

//System
#include <list>

//! Triangulation types
enum CC_TRIANGULATION_TYPES {	DELAUNAY_2D_AXIS_ALIGNED  = 1,		/**< Delaunay 2D triangulation in an axis-aligned plane **/
								DELAUNAY_2D_BEST_LS_PLANE = 2,		/**< Delaunay 2D with points projected on the best least square fitting plane **/
};

namespace CCLib
{

class GenericIndexedMesh;
class GenericProgressCallback;

//! Several point cloud re-projection algorithms ("developpee", translation, rotation, etc.)
class CC_CORE_LIB_API PointProjectionTools : public CCToolbox
{
public:

	//! A scaled geometrical transformation (scale + rotation + translation)
	/** P' = s.R.P + T
	**/
	struct Transformation
	{
		//! Rotation
		CCLib::SquareMatrix R;
		//! Translation
		CCVector3 T;
		//! Scale
		PointCoordinateType s;

		//! Default constructor
		Transformation() : s(PC_ONE) {}

		//! Applies the transformation to a point
		inline CCVector3 apply(const CCVector3& P) const { return s * (R * P) + T; }

		//! Applies the transformation to a cloud
		/** \warning THIS METHOD IS NOT COMPATIBLE WITH PARALLEL STRATEGIES
			\warning The caller should invalidate the bounding-box manually afterwards
		**/
		CC_CORE_LIB_API void apply(GenericIndexedCloudPersist& cloud) const;
	};

	//! Develops a cylinder-shaped point cloud around its main axis
	/** Generates a "developpee" of a cylinder-shaped point cloud.
		WARNING: this method uses the cloud global iterator
		\param cloud the point cloud to be developed
		\param radius the cylinder radius
		\param dim the dimension along which the cylinder axis is aligned (X=0, Y=1, Z=2)
		\param center a 3D point (as a 3 values array) belonging to the cylinder axis
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return the "developed" cloud
	**/
	static PointCloud* developCloudOnCylinder(	GenericCloud* cloud,
												PointCoordinateType radius,
												unsigned char dim = 2,
												CCVector3* center = nullptr,
												GenericProgressCallback* progressCb = nullptr);

	//! Develops a cone-shaped point cloud around its main axis
	/** Generates a "developpee" of a cone-shaped point cloud.
		WARNING: this method uses the cloud global iterator
		\param cloud the point cloud to be developed
		\param dim the dimension along which the cone axis is aligned (X=0, Y=1, Z=2)
		\param baseRadius the radius of the base of the cone
		\param alpha the angle of the cone "opening"
		\param center the 3D point corresponding to the intersection between the cone axis and its base
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return the "developed" cloud
	**/
	static PointCloud* developCloudOnCone(	GenericCloud* cloud,
											unsigned char dim,
											PointCoordinateType baseRadius,
											float alpha,
											const CCVector3& center,
											GenericProgressCallback* progressCb = nullptr);

	//! Applys a geometrical transformation to a point cloud
	/** \param cloud the point cloud to be "transformed"
		\param trans the geometrical transformation
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return the "transformed" cloud
	**/
	static PointCloud* applyTransformation(	GenericCloud* cloud,
											Transformation& trans,
											GenericProgressCallback* progressCb = nullptr);

	//! Applys a geometrical transformation to a single point
	/** \param P the point
		\param trans the geometrical transformation
		\return the "transformed" point
	**/
	//static CCVector3 applyTransformation(const CCVector3& P, Transformation& trans);

	//! Computes a 2.5D Delaunay triangulation
	/** The triangulation can be either computed on the points projected
		in the XY plane (by default), or projected on the best least-square
		fitting plane. The triangulation is in 2D (in the plane) but the
		3D points are connected, so it's a kind of 2.5D triangulation (that
		may present however several topological aberrations ;).
		\param cloud a point cloud
		\param type the triangulation strategy
		\param maxEdgeLength max edge length for output triangles (0 = ignored)
		\param dim projection dimension (for axis-aligned meshes)
		\param errorStr error (if any) [optional]
		\return a mesh
	**/
	static GenericIndexedMesh* computeTriangulation(GenericIndexedCloudPersist* cloud,
													CC_TRIANGULATION_TYPES type = DELAUNAY_2D_AXIS_ALIGNED,
													PointCoordinateType maxEdgeLength = 0,
													unsigned char dim = 2,
													char* errorStr = nullptr);

	//! Indexed 2D vector
	/** Used for convex and concave hull computation
	**/
	class IndexedCCVector2 : public CCVector2
	{
	public:

		//! Default constructor
		IndexedCCVector2() : CCVector2(), index(0) {}
		//! Constructor
		IndexedCCVector2(PointCoordinateType x, PointCoordinateType y) : CCVector2(x,y), index(0) {}
		//! Constructor
		IndexedCCVector2(PointCoordinateType x, PointCoordinateType y, unsigned i) : CCVector2(x,y), index(i) {}
		//! Copy constructor
		IndexedCCVector2(const CCVector2& v) : CCVector2(v), index(0) {}

		//! Point index
		unsigned index;
	};

	//! Determines the convex hull of a set of points
	/** Returns a list of points on the convex hull in counter-clockwise order.
		Implementation of Andrew's monotone chain 2D convex hull algorithm.
		Asymptotic complexity: O(n log n).
		(retrieved from http://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain)
		WARNING: the input 'points' set will be sorted!
		\param points input set of points
		\param hullPoints output points (on the convex hull)
		\return success
	**/
	static bool extractConvexHull2D(std::vector<IndexedCCVector2>& points,
									std::list<IndexedCCVector2*>& hullPoints);

	//! Determines the 'concave' hull of a set of points
	/** Inspired from JIN-SEO PARK AND SE-JONG OH, "A New Concave Hull Algorithm
		and Concaveness Measure for n-dimensional Datasets", 2012
		Calls extractConvexHull2D (see associated warnings).
		\param points input set of points
		\param hullPoints output points (on the convex hull)
		\param maxSquareLength maximum square length (ignored if <= 0, in which case the method simply returns the convex hull!)
		\return success
	**/
	static bool extractConcaveHull2D(	std::vector<IndexedCCVector2>& points,
										std::list<IndexedCCVector2*>& hullPoints,
										PointCoordinateType maxSquareLength = 0);

	//! Returns true if the AB and CD segments intersect each other
	static bool segmentIntersect(const CCVector2& A, const CCVector2& B, const CCVector2& C, const CCVector2& D);
};

}

#endif //POINT_PROJECTION_TOOLS_HEADER
