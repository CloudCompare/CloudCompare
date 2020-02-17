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

#ifndef DISTANCE_COMPUTATION_TOOLS_HEADER
#define DISTANCE_COMPUTATION_TOOLS_HEADER

//Local
#include "CCConst.h"
#include "CCToolbox.h"
#include "DgmOctree.h"
#include "SquareMatrix.h"
#include "Polyline.h"

namespace CCLib
{

class GenericTriangle;
class GenericIndexedMesh;
class GenericCloud;
class GenericIndexedCloudPersist;
class ReferenceCloud;
class PointCloud;
class GenericProgressCallback;
struct OctreeAndMeshIntersection;
class ScalarField;

//! Several entity-to-entity distances computation algorithms (cloud-cloud, cloud-mesh, point-triangle, etc.)
class CC_CORE_LIB_API DistanceComputationTools : public CCToolbox
{
public: //distance to clouds or meshes

	//! Cloud-to-cloud "Hausdorff" distance computation parameters
	struct Cloud2CloudDistanceComputationParams
	{
		//! Level of subdivision of the octree at witch to apply the distance computation algorithm
		/** If set to 0 (default) the algorithm will try to guess the best level automatically.
		**/
		unsigned char octreeLevel;

		//! Maximum search distance (true distance won't be computed if greater)
		/** Set to -1 to deactivate (default).
			\warning Not compatible with closest point set determination (see CPSet)
		**/
		ScalarType maxSearchDist;

		//! Whether to use multi-thread or single thread mode
		/** If maxSearchDist > 0, single thread mode will be forced.
		**/
		bool multiThread;

		//! Maximum number of threads to use (0 = max)
		int maxThreadCount;

		//! Type of local 3D modeling to use
		/** Default: NO_MODEL. Otherwise see CC_LOCAL_MODEL_TYPES.
		**/
		CC_LOCAL_MODEL_TYPES localModel;

		//! Whether to use a fixed number of neighbors or a (sphere) radius for nearest neighbours search
		/** For local models only (i.e. ignored if localModel = NO_MODEL).
		**/
		bool useSphericalSearchForLocalModel;

		//! Number of neighbours for nearest neighbours search (local model)
		/** For local models only (i.e. ignored if localModel = NO_MODEL).
			Ignored if useSphericalSearchForLocalModel is true.
		**/
		unsigned kNNForLocalModel;

		//! Radius for nearest neighbours search (local model)
		/** For local models only (i.e. ignored if localModel = NO_MODEL).
			Ignored if useSphericalSearchForLocalModel is true.
		**/
		ScalarType radiusForLocalModel;

		//! Whether to use an approximation for local model computation
		/** For local models only (i.e. ignored if localModel = NO_MODEL).
			Computation is much faster but less "controlled".
		**/
		bool reuseExistingLocalModels;

		//! Container of (references to) points to store the "Closest Point Set"
		/** The Closest Point Set corresponds to (the reference to) each compared point's closest neighbour.
			\warning Not compatible with max search distance (see maxSearchDist)
		**/
		ReferenceCloud* CPSet;

		//! Split distances (one scalar field per dimension: X, Y and Z)
		ScalarField* splitDistances[3];

		//! Whether to keep the existing distances as is (if any) or not
		/** By default, any previous distances/scalar values stored in the 'enabled' scalar field will be
			reset before computing them again.
		**/
		bool resetFormerDistances;

		//! Default constructor/initialization
		Cloud2CloudDistanceComputationParams()
			: octreeLevel(0)
			, maxSearchDist(0)
			, multiThread(true)
			, maxThreadCount(0)
			, localModel(NO_MODEL)
			, useSphericalSearchForLocalModel(false)
			, kNNForLocalModel(0)
			, radiusForLocalModel(0)
			, reuseExistingLocalModels(false)
			, CPSet(nullptr)
			, resetFormerDistances(true)
		{
			splitDistances[0] = splitDistances[1] = splitDistances[2] = nullptr;
		}
	};

	//! Computes the "nearest neighbour distance" between two point clouds (formerly named "Hausdorff distance")
	/** The main algorithm and its different versions (with or without local modeling) are described in
		Daniel Girardeau-Montaut's PhD manuscript (Chapter 2, section 2.3). It is the standard way to compare
		directly two dense (and globally close) point clouds.
		\warning The current scalar field of the compared cloud should be enabled. By default it will be reset to
		NAN_VALUE but one can avoid this by defining the Cloud2CloudDistanceComputationParams::resetFormerDistances
		parameters to false. But even in this case, only values above Cloud2CloudDistanceComputationParams::maxSearchDist
		will remain untouched.
		\warning Max search distance (Cloud2CloudDistanceComputationParams::maxSearchDist > 0) is not compatible with the
		determination of the Closest Point Set (Cloud2CloudDistanceComputationParams::CPSet)
		\param comparedCloud the compared cloud (the distances will be computed on these points)
		\param referenceCloud the reference cloud (the distances will be computed relatively to these points)
		\param params distance computation parameters
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param compOctree the pre-computed octree of the compared cloud (warning: both octrees must have the same cubical bounding-box - it is automatically computed if 0)
		\param refOctree the pre-computed octree of the reference cloud (warning: both octrees must have the same cubical bounding-box - it is automatically computed if 0)
		\return 0 if ok, a negative value otherwise
	**/
	static int computeCloud2CloudDistance(	GenericIndexedCloudPersist* comparedCloud,
											GenericIndexedCloudPersist* referenceCloud,
											Cloud2CloudDistanceComputationParams& params,
											GenericProgressCallback* progressCb = nullptr,
											DgmOctree* compOctree = nullptr,
											DgmOctree* refOctree = nullptr);

	//! Cloud-to-mes distances computation parameters
	struct Cloud2MeshDistanceComputationParams
	{
		//! The level of subdivision of the octree at witch to apply the algorithm
		unsigned char octreeLevel;

		//! Max search distance (acceleration)
		/** Default value: 0. If greater than 0, then the algorithm won't compute distances over this value
		**/
		ScalarType maxSearchDist;

		//! Use distance map (acceleration)
		/** If true the distances will be aproximated by a Distance Transform.
			\warning Incompatible with signed distances or Closest Point Set.
		**/
		bool useDistanceMap;

		//! Whether to compute signed distances or not
		/** If true, the computed distances will be signed (in this case, the Distance Transform can't be used
			and therefore useDistanceMap will be ignored)
		**/
		bool signedDistances;

		//! Whether triangle normals should be computed in the 'direct' order (true) or 'indirect' (false)
		bool flipNormals;

		//! Whether to use multi-thread or single thread mode (if maxSearchDist > 0, single thread mode is forced)
		bool multiThread;

		//! Maximum number of threads to use (0 = max)
		int maxThreadCount;

		//! Cloud to store the Closest Point Set
		/** The cloud should be initialized but empty on input. It will have the same size as the compared cloud on output.
			\warning Not compatible with maxSearchDist > 0.
		**/
		PointCloud* CPSet;

		//! Default constructor
		Cloud2MeshDistanceComputationParams()
			: octreeLevel(0)
			, maxSearchDist(0)
			, useDistanceMap(false)
			, signedDistances(false)
			, flipNormals(false)
			, multiThread(true)
			, maxThreadCount(0)
			, CPSet(nullptr)
		{}
	};

	//! Computes the distance between a point cloud and a mesh
	/** The algorithm, inspired from METRO by Cignoni et al., is described
		in Daniel Girardeau-Montaut's PhD manuscript (Chapter 2, section 2.2).
		It is the general way to compare a point cloud with a triangular mesh.
		\param pointCloud the compared cloud (the distances will be computed on these points)
		\param mesh the reference mesh (the distances will be computed relatively to its triangles)
		\param params parameters
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param cloudOctree the pre-computed octree of the compared cloud (warning: its bounding box should be equal to the union of both point cloud and mesh bbs and it should be cubical - it is automatically computed if 0)
		\return 0 if ok, a negative value otherwise
	**/
	static int computeCloud2MeshDistance(	GenericIndexedCloudPersist* pointCloud,
											GenericIndexedMesh* mesh,
											Cloud2MeshDistanceComputationParams& params,
											GenericProgressCallback* progressCb = nullptr,
											DgmOctree* cloudOctree = nullptr);

public: //approximate distances to clouds or meshes

	//! Computes approximate distances between two point clouds
	/** This methods uses an exact Distance Transform to approximate the real distances.
		Therefore, the greater the octree level is (it is used to determine the grid step), the finer
		the result will be (but more memory and time will be needed).
		\param comparedCloud the compared cloud
		\param referenceCloud the reference cloud
		\param octreeLevel the octree level at which to compute the Distance Transform
		\param maxSearchDist max search distance (or any negative value if no max distance is defined)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param compOctree the pre-computed octree of the compared cloud (warning: both octrees must have the same cubical bounding-box - it is automatically computed if 0)
		\param refOctree the pre-computed octree of the reference cloud (warning: both octrees must have the same cubical bounding-box - it is automatically computed if 0)
		\return negative error code or a positive value in case of success
	**/
	static int computeApproxCloud2CloudDistance(GenericIndexedCloudPersist* comparedCloud,
												GenericIndexedCloudPersist* referenceCloud,
												unsigned char octreeLevel,
												PointCoordinateType maxSearchDist = 0,
												GenericProgressCallback* progressCb = nullptr,
												DgmOctree* compOctree = nullptr,
												DgmOctree* refOctree = nullptr);

public: //distance to simple entities (triangles, planes, etc.)

	//! Computes the distance between a point and a triangle
	/** WARNING: if not signed, the returned distance is SQUARED!
		\param P a 3D point
		\param theTriangle a 3D triangle
		\param signedDist whether to compute the signed or positive (SQUARED) distance
		\param nearestP optional: returns the nearest point on the triangle
		\return the distance between the point and the triangle
	**/
	static ScalarType computePoint2TriangleDistance(const CCVector3* P,
													const GenericTriangle* theTriangle,
													bool signedDist,
													CCVector3* nearestP = nullptr);

	//! Computes the (signed) distance between a point and a plane
	/** \param P a 3D point
		\param planeEquation plane equation: [a,b,c,d] as 'ax+by+cz=d' with norm(a,bc)==1
		\return the signed distance between the point and the plane
	**/
	static ScalarType computePoint2PlaneDistance(const CCVector3* P, const PointCoordinateType* planeEquation);

	//! Computes the square of the distance between a point and a line segment
	/** \param point a 3D point
		\param start the start of line segment
		\param end the end of line segment
		\return the distance squared between the point and the line segment
	**/
	static ScalarType computePoint2LineSegmentDistSquared(const CCVector3* point, const CCVector3* start, const CCVector3* end);

	//! Computes the distance between each point in a cloud and a cone
	/** \param cloud a 3D point cloud
		\param coneP1 center point associated with the larger radii
		\param coneP2 center point associated with the smaller radii
		\param coneR1 cone radius at coneP1 (larger)
		\param coneR2 cone radius at coneP2 (smaller)
		\param signedDist whether to compute the signed or positive (absolute) distance (optional)
		\param solutionType if true the scalar field will be set to which solution was selected 1-4 (optional)
		\param[out] rms will be set with the Root Mean Square (RMS) distance between a cloud and a cylinder (optional)
		\return negative error code or a positive value in case of success
	**/
	static int computeCloud2ConeEquation(GenericIndexedCloudPersist* cloud, const CCVector3& coneP1, const CCVector3& coneP2, const PointCoordinateType coneR1, const PointCoordinateType coneR2, bool signedDistances = true, bool solutionType = false, double* rms = nullptr);

	//! Computes the distance between each point in a cloud and a cylinder
	/** \param cloud a 3D point cloud
		\param cylinderP1 center bottom point
		\param cylinderP2 center top point
		\param cylinderRadius cylinder radius
		\param signedDist whether to compute the signed or positive (absolute) distance (optional)
		\param solutionType if true the scalar field will be set to which solution was selected 1-4 (optional)
		\param[out] rms will be set with the Root Mean Square (RMS) distance between a cloud and a cylinder (optional)
		\return negative error code or a positive value in case of success
	**/
	static int computeCloud2CylinderEquation(GenericIndexedCloudPersist* cloud, const CCVector3& cylinderP1, const CCVector3& cylinderP2, const PointCoordinateType cylinderRadius, bool signedDistances = true, bool solutionType = false, double* rms = nullptr);

	//! Computes the distance between each point in a cloud and a sphere
	/** \param cloud a 3D point cloud
		\param sphereCenter sphere 3d center point
		\param sphereRadius sphere radius
		\param signedDist whether to compute the signed or positive (absolute) distance (optional)
		\param[out] rms will be set with the Root Mean Square (RMS) distance between a cloud and a sphere (optional)
		\return negative error code or a positive value in case of success
	**/
	static int computeCloud2SphereEquation(GenericIndexedCloudPersist *cloud, const CCVector3& sphereCenter, const PointCoordinateType sphereRadius, bool signedDistances = true, double* rms = nullptr);

	//! Computes the distance between each point in a cloud and a plane
	/** \param cloud a 3D point cloud
		\param planeEquation plane equation: [a,b,c,d] as 'ax+by+cz=d' with norm(a,bc)==1
		\param signedDist whether to compute the signed or positive (absolute) distance (optional)
		\param[out] rms will be set with the Root Mean Square (RMS) distance between a cloud and a plane (optional)
		\return negative error code or a positive value in case of success
	**/
	static int computeCloud2PlaneEquation(GenericIndexedCloudPersist* cloud, const PointCoordinateType* planeEquation, bool signedDistances = true, double * rms = nullptr);
	
	static int computeCloud2RectangleEquation(GenericIndexedCloudPersist *cloud, PointCoordinateType widthX, PointCoordinateType widthY, const SquareMatrix& rotationTransform, const CCVector3& center, bool signedDist = true, double* rms = nullptr);

	static int computeCloud2BoxEquation(GenericIndexedCloudPersist* cloud, const CCVector3& boxDimensions, const SquareMatrix& rotationTransform, const CCVector3& boxCenter, bool signedDist = true, double* rms = nullptr);
	
	//! Computes the distance between each point in a cloud and a polyline
	/** \param cloud a 3D point cloud
		\param polyline the polyline to measure to
		\param[out] rms will be set with the Root Mean Square (RMS) distance between a cloud and a plane (optional)
		\return negative error code or a positive value in case of success
	**/
	static int computeCloud2PolylineEquation(GenericIndexedCloudPersist* cloud, const Polyline* polyline, double* rms = nullptr);

	//! Error estimators
	enum ERROR_MEASURES
	{
		RMS,						/**< Root Mean Square error **/
		MAX_DIST_68_PERCENT,		/**< Max distance @ 68% (1 sigma) **/
		MAX_DIST_95_PERCENT,		/**< Max distance @ 98% (2 sigmas) **/
		MAX_DIST_99_PERCENT,		/**< Max distance @ 99% (3 sigmas) **/
		MAX_DIST,					/**< Max distance **/
	};

	enum DISTANCE_COMPUTATION_RESULTS
	{
		CANCELED_BY_USER = -1000,
		ERROR_NULL_COMPAREDCLOUD,
		ERROR_NULL_COMPAREDOCTREE,
		ERROR_OUT_OF_MEMORY,
		ERROR_ENABLE_SCALAR_FIELD_FAILURE,
		ERROR_EMPTY_COMPAREDCLOUD,
		ERROR_NULL_REFERENCECLOUD,
		ERROR_EMPTY_REFERENCECLOUD,
		ERROR_NULL_REFERENCEMESH,
		ERROR_EMPTY_REFERENCEMESH,
		NULL_PLANE_EQUATION,
		ERROR_NULL_OCTREE,
		ERROR_NULL_OCTREE_AND_MESH_INTERSECTION,
		ERROR_CANT_USE_MAX_SEARCH_DIST_AND_CLOSEST_POINT_SET,
		ERROR_EXECUTE_FUNCTION_FOR_ALL_CELLS_AT_LEVEL_FAILURE,
		ERROR_EXECUTE_GET_POINTS_IN_CELL_BY_INDEX_FAILURE,
		ERROR_EXECUTE_CLOUD_MESH_DIST_CELL_FUNC_MT_FAILURE,
		ERROR_GET_CELL_CODES_FAILURE,
		ERROR_GET_CELL_CODES_AND_INDEXES_FAILURE,
		ERROR_GET_CELL_INDEXES_FAILURE,
		ERROR_PROPAGATE_DISTANCE_FAILURE,
		ERROR_SEED_POINT_INDEX_GREATER_THAN_COMPAREDCLOUD_SIZE,
		ERROR_INIT_DISTANCE_TRANSFORM_GRID_FAILURE,
		ERROR_INIT_PER_CELL_TRIANGLE_LIST_FAILURE,
		ERROR_INTERSECT_MESH_WITH_OCTREE_FAILURE,
		ERROR_COMPUTE_CLOUD2_MESH_DISTANCE_WITH_OCTREE_FAILURE,
		ERROR_COMPUTE_CLOUD2_CLOUD_DISTANCE_FAILURE,
		ERROR_OCTREE_LEVEL_LT_ONE,
		ERROR_OCTREE_LEVEL_GT_MAX_OCTREE_LEVEL,
		ERROR_SYNCHRONIZE_OCTREES_FAILURE,
		ERROR_PLANE_NORMAL_LT_ZERO,
		ERROR_INVALID_PRIMITIVE_DIMENSIONS,
		ERROR_CONE_R1_LT_CONE_R2,
		ERROR_CONELENGTH_ZERO,
		ERROR_COULDNOT_SYNCRONIZE_OCTREES,
		ERROR_BUILD_OCTREE_FAILURE,
		ERROR_BUILD_FAST_MARCHING_FAILURE,
		ERROR_UNKOWN_ERRORMEASURES_TYPE,
		SUCCESS = 1,
	};

	//! Computes the "distance" (see ERROR_MEASURES) between a point cloud and a plane
	/** \param cloud a point cloud
		\param planeEquation plane equation: [a,b,c,d] as 'ax+by+cz=d'
		\param measureType measure type
	**/
	static ScalarType ComputeCloud2PlaneDistance(	CCLib::GenericCloud* cloud,
													const PointCoordinateType* planeEquation,
													ERROR_MEASURES measureType);

	//! Computes the maximum distance between a point cloud and a plane
	/** WARNING: this method uses the cloud global iterator
		\param cloud a point cloud
		\param planeEquation plane equation: [a,b,c,d] as 'ax+by+cz=d'
		\param percent percentage of lowest values ignored
		\return the max distance @ 'percent' % between the point and the plane
	**/
	static ScalarType ComputeCloud2PlaneRobustMax(	GenericCloud* cloud,
													const PointCoordinateType* planeEquation,
													float percent);

	//! Computes the maximum distance between a point cloud and a plane
	/** WARNING: this method uses the cloud global iterator
		\param cloud a point cloud
		\param planeEquation plane equation: [a,b,c,d] as 'ax+by+cz=d'
		\return the max distance between the point and the plane
	**/
	static ScalarType ComputeCloud2PlaneMaxDistance(GenericCloud* cloud,
													const PointCoordinateType* planeEquation);

	//! Computes the Root Mean Square (RMS) distance between a cloud and a plane
	/** Sums the squared distances between each point of the cloud and the plane, then computes the mean value.
		WARNING: this method uses the cloud global iterator
		\param cloud a point cloud
		\param planeEquation plane equation: [a,b,c,d] as 'ax+by+cz=d'
		\return the RMS of distances (or NaN if an error occurred)
	**/
	static ScalarType computeCloud2PlaneDistanceRMS(	GenericCloud* cloud,
														const PointCoordinateType* planeEquation);

	//! Returns the (squared) distance from a point to a segment
	/** \param P 3D point
		\param A first point of the segment
		\param B first point of the segment
		\param onlyOrthogonal computes distance only if P lies 'in front' of AB (returns -1.0 otherwise)
		\return squared distance (or potentially -1.0 if onlyOrthogonal is true)
	**/
	static PointCoordinateType ComputeSquareDistToSegment(	const CCVector2& P,
															const CCVector2& A,
															const CCVector2& B,
															bool onlyOrthogonal = false);

public: //other methods

	//! Computes geodesic distances over a point cloud "surface" (starting from a seed point)
	/** This method uses the FastMarching algorithm. Thereofre it needs an octree level as input
		parameter in order to create the corresponding 3D grid. The greater this level is, the finer
		the result will be, but more memory will be required as well.
		Moreover to get an interesting result the cells size should not be too small (the propagation
		will be stoped more easily on any encountered 'hole').
		\param cloud the point cloud
		\param seedPointIndex the index of the point from where to start the propagation
		\param octreeLevel the octree at which to perform the Fast Marching propagation
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return true if the method succeeds
	**/
	static bool computeGeodesicDistances(	GenericIndexedCloudPersist* cloud,
											unsigned seedPointIndex,
											unsigned char octreeLevel,
											GenericProgressCallback* progressCb = nullptr);

	//! Computes the differences between two scalar fields associated to equivalent point clouds
	/** The compared cloud should be smaller or equal to the reference cloud. Its points should be
		at the same position in space as points in the other cloud. The algorithm simply computes
		the difference between the scalar values associated to each couple of equivalent points.
		\warning The result is stored in the active scalar field (input) of the comparedCloud.
		\warning Moreover, the output scalar field should be different than the input scalar field!
		\warning Be sure to activate an OUTPUT scalar field on both clouds
		\param comparedCloud the compared cloud
		\param referenceCloud the reference cloud
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
	**/
	static int diff(GenericIndexedCloudPersist* comparedCloud,
					GenericIndexedCloudPersist* referenceCloud,
					GenericProgressCallback* progressCb = nullptr);

	//! Return codes for DistanceComputationTools::synchronizeOctrees
	enum SOReturnCode { EMPTY_CLOUD, SYNCHRONIZED, DISJOINT, OUT_OF_MEMORY };

	//! Synchronizes (and re-build if necessary) two octrees
	/** Initializes the octrees before computing the distance between two clouds.
		Check if both octree have the same sizes and limits (in 3D) and rebuild
		them if necessary.
		\param comparedCloud the cloud corresponding to the first octree
		\param referenceCloud the cloud corresponding to the second octree
		\param comparedOctree the first octree
		\param referenceOctree the second octree
		\param maxSearchDist max search distance (or any negative value if no max distance is defined)
		\param progressCb the client method can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return return code
	**/
	static SOReturnCode synchronizeOctrees(	GenericIndexedCloudPersist* comparedCloud,
											GenericIndexedCloudPersist* referenceCloud,
											DgmOctree* &comparedOctree,
											DgmOctree* &referenceOctree,
											PointCoordinateType maxSearchDist = 0,
											GenericProgressCallback* progressCb = nullptr);

	//! Returns whether multi-threading (parallel) computation is supported or not
	static bool MultiThreadSupport();

protected:

	//! Intersects a mesh with a grid structure
	/** This method is used by computeCloud2MeshDistance.
		\param theIntersection a specific structure to store the result of the intersection
		\param octreeLevel the octree subdivision level corresponding to the grid
		\param progressCb the client method can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
	**/
	static int intersectMeshWithOctree(	OctreeAndMeshIntersection* theIntersection,
										unsigned char octreeLevel,
										GenericProgressCallback* progressCb = nullptr);

	//! Computes the distances between a point cloud and a mesh projected into a grid structure
	/** This method is used by computeCloud2MeshDistance, after intersectMeshWithOctree has been called.
		\param theIntersection a specific structure corresponding the intersection of the mesh with the grid
		\param params parameters
		\param progressCb the client method can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return -1 if an error occurred (e.g. not enough memory) and 0 otherwise
	**/
	static int computeCloud2MeshDistanceWithOctree(	OctreeAndMeshIntersection* theIntersection,
													Cloud2MeshDistanceComputationParams& params,
													GenericProgressCallback* progressCb = nullptr);

	//! Computes the "nearest neighbour distance" without local modeling for all points of an octree cell
	/** This method has the generic syntax of a "cellular function" (see DgmOctree::localFunctionPtr).
		Specific parameters are transmitted via the "additionalParameters" structure.
		There are 3 additional parameters :
		- (GenericCloud*) the compared cloud
		- (GenericCloud*) the reference cloud
		- (DgmOctree*) the octree corresponding to the compared cloud
		\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
		\param nProgress optional (normalized) progress notification (per-point)
	**/
	static bool computeCellHausdorffDistance(	const DgmOctree::octreeCell& cell,
												void** additionalParameters,
												NormalizedProgress* nProgress = nullptr);

	//! Computes the "nearest neighbour distance" with local modeling for all points of an octree cell
	/** This method has the generic syntax of a "cellular function" (see DgmOctree::localFunctionPtr).
		Specific parameters are transmitted via the "additionalParameters" structure.
		There are 4 additional parameters :
		- (GenericCloud*) the compared cloud
		- (GenericCloud*) the reference cloud
		- (DgmOctree*) the octree corresponding to the compared cloud
		- (CC_LOCAL_MODEL_TYPES*) type of local model to apply
		\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
		\param nProgress optional (normalized) progress notification (per-point)
	**/
	static bool computeCellHausdorffDistanceWithLocalModel(	const DgmOctree::octreeCell& cell,
															void** additionalParameters,
															NormalizedProgress* nProgress = nullptr);
};

}

#endif //DISTANCE_COMPUTATION_TOOLS_HEADER
