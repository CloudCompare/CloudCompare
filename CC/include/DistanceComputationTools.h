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

#ifndef DISTANCE_COMPUTATION_TOOLS_HEADER
#define DISTANCE_COMPUTATION_TOOLS_HEADER

//Local
#include "CCCoreLib.h"
#include "CCToolbox.h"
#include "CCConst.h"
#include "DgmOctree.h"

namespace CCLib
{

class GenericTriangle;
class GenericIndexedMesh;
class GenericCloud;
class GenericIndexedCloud;
class GenericIndexedCloudPersist;
class ReferenceCloud;
class GenericProgressCallback;
class ChamferDistanceTransform;
struct OctreeAndMeshIntersection;

//INTERNAL TESTS
//#define DO_CLOUD2MESH_DISTANCE_TESTS
#ifdef ENABLE_MT_OCTREE
#define ENABLE_CLOUD2MESH_DIST_MT
#endif

//! Several entity-to-entity distances computation algorithms (cloud-cloud, cloud-mesh, point-triangle, etc.)
class CC_CORE_LIB_API DistanceComputationTools : public CCToolbox
{
public:

	//! Cloud-to-cloud "Hausdorff" distance computation parameters
	struct Cloud2CloudDistanceComputationParams
	{
		//! Level of subdivision of the octree at witch to apply the distance computation algorithm
		/** If set to 0 (default) the algorithm will try to guess the best level automatically.
		**/
		uchar octreeLevel;

		//! Maximum search distance (true distance won't be computed if greater)
		/** Set to -1 to deactivate (default).
			Not compatible with closest point set determination (see CPSet).
		**/
		ScalarType maxSearchDist;

		//! Whether to use multi-thread or single thread mode
		/** If maxSearchDist>=0, single thread mode is forced.
		**/
		bool multiThread;

		//! Type of local 3D modeling to use
		/** Default: NO_MODEL. Otherwise see CC_LOCAL_MODEL_TYPES.
		**/
		CC_LOCAL_MODEL_TYPES localModel;

		//! Whether to use a fixed number of neighbors or a (sphere) radius for nearest neighbours seach
		/** For local models only (i.e. ignored if localModel = NO_MODEL).
		**/
		bool useSphericalSearchForLocalModel;

		//! Number of neighbours for nearest neighbours seach (local model)
		/** For local models only (i.e. ignored if localModel = NO_MODEL).
			Ignored if useSphericalSearchForLocalModel is true.
		**/
		unsigned kNNForLocalModel;

		//! Radius for nearest neighbours seach (local model)
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
		**/
		ReferenceCloud* CPSet;

		//! Default constructor/initialization
		Cloud2CloudDistanceComputationParams()
			: octreeLevel(0)
			, maxSearchDist(-1.0)
			, multiThread(true)
			, localModel(NO_MODEL)
			, useSphericalSearchForLocalModel(false)
			, kNNForLocalModel(0)
			, radiusForLocalModel(0)
			, reuseExistingLocalModels(false)
			, CPSet(0)
		{}
	};

	//! Computes the "nearest neighbour distance" between two point clouds (formerly named "Hausdorff distance")
	/** The main algorithm and its different versions (with or without local modeling) are described in
		Daniel Girardeau-Montaut's PhD manuscript (Chapter 2, section 2.3). It is the standard way to compare
		directly two dense (and globally close) point clouds.
		Warning: the current scalar field  of the compared cloud should be enabled and initialized either to
		HIDDEN_VALUE or to an approximated distance (strictly bigger than the actual distance!).
		\param comparedCloud the compared cloud (the distances will be computed on these points)
		\param referenceCloud the reference cloud (the distances will be computed relatively to these points)
		\param params distance computation parameters
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param compOctree the pre-computed octree of the compared cloud (warning: both octrees must have the same cubical bounding-box - it is automatically computed if 0)
		\param refOctree the pre-computed octree of the reference cloud (warning: both octrees must have the same cubical bounding-box - it is automatically computed if 0)
		\return 0 if ok, a negative value otherwise
	**/
	static int computeHausdorffDistance(GenericIndexedCloudPersist* comparedCloud,
										GenericIndexedCloudPersist* referenceCloud,
										Cloud2CloudDistanceComputationParams& params,
										GenericProgressCallback* progressCb = 0,
										DgmOctree* compOctree = 0,
										DgmOctree* refOctree = 0);

	//! Computes the distance between a point cloud and a mesh
	/** The algorithm, inspired from METRO by Cignoni et al., is described
		in Daniel Girardeau-Montaut's PhD manuscript (Chapter 2, section 2.2).
		It is the general way to compare a point cloud with a triangular mesh.
		\param pointCloud the compared cloud (the distances will be computed on these points)
		\param theMesh the reference mesh (the distances will be computed relatively to its triangles)
		\param octreeLevel the level of subdivision of the octree at witch to apply the algorithm
		\param maxSearchDist if greater than 0 (default value: '-1'), then the algorithm won't compute distances over this value (acceleration)
		\param useDistanceMap if true, the distances over "maxSearchDist" will be aproximated by the Chamfer 3-4-5 distance transform (acceleration)
		\param signedDistances if true, the computed distances will be signed (in this case, Chamfer distances can't be computed and useDistanceMap is ignored)
		\param flipNormals specify whether triangle normals should be computed in the 'direct' order (true) or 'indirect' (false)
		\param multiThread specify whether to use multi-thread or single thread mode (if maxSearchDist>=0, single thread mode is forced)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param cloudOctree the pre-computed octree of the compared cloud (warning: its bounding box should be equal to the union of both point cloud and mesh bbs and it should be cubical - it is automatically computed if 0)
		\return 0 if ok, a negative value otherwise
	**/
	static int computePointCloud2MeshDistance(	GenericIndexedCloudPersist* pointCloud,
												GenericIndexedMesh* theMesh,
												uchar octreeLevel,
												ScalarType maxSearchDist = -1.0,
												bool useDistanceMap = false,
												bool signedDistances = false,
												bool flipNormals = false,
												bool multiThread = true,
												GenericProgressCallback* progressCb = 0,
												DgmOctree* cloudOctree = 0);

	/*** Basic entity level ***/

	//! Computes the distance between a point and a triangle
	/** WARNING: if not signed, the returned distance is SQUARED!
		\param P a 3D point
		\param theTriangle a 3D triangle
		\param signedDist whether to compute the signed or positive (SQUARED) distance
		\return the distance between the point and the triangle
	**/
	static ScalarType computePoint2TriangleDistance(const CCVector3* P, const GenericTriangle* theTriangle, bool signedDist);

	//! Computes the (signed) distance between a point and a plane
	/** \param P a 3D point
		\param planeEquation plane equation: [a,b,c,d] as 'ax+by+cz=d' with norm(a,bc)==1
		\return the signed distance between the point and the plane
	**/
	static ScalarType computePoint2PlaneDistance(const CCVector3* P, const PointCoordinateType* planeEquation);

	/*** OTHER METHODS ***/

	//! Computes a the geodesic distances over a point cloud "surface", starting from a seed point
	/** This method uses the FastMarching algorithm, and thereofre it needs an octree level as input
		parameter in order to compute a 3D grid. The greater this level is, the finer the result is,
		but more memory will be needed. Moreover, for interesting results, the cells size should be
		not too small in order to avoid creating holes in the approximated surface (the propagation will
		be stoped).
		\param cloud the point cloud
		\param seedPointIndex the index of the point from where to start the propagation
		\param octreeLevel the octree at which to perform the Fast Marching propagation
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return true if the method succeeds
	**/
	static bool computeGeodesicDistances(	GenericIndexedCloudPersist* cloud,
											unsigned seedPointIndex,
											uchar octreeLevel,
											GenericProgressCallback* progressCb = 0);

	//! Computes the differences between two scalar fields associated to equivalent point clouds
	/** The compared cloud should be smaller or equal to the reference cloud. Its points should be at the same
		position in place as their equivalents in the other cloud. The algorithm perform a simple difference
		between the scalar values associated to each couple of equivalent points. The result is stored in a
		the active scalar field (input) of the comparedCloud. Moreover, the output scalar field should
		be different from the input scalar field !
		Warning: be sure to activate an OUTPUT scalar field on both input clouds
		\param comparedCloud the compared cloud
		\param referenceCloud the reference cloud
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
	**/
	static int diff(GenericIndexedCloudPersist* comparedCloud,
					GenericIndexedCloudPersist* referenceCloud,
					GenericProgressCallback* progressCb = 0);

	//! Error estimators
	enum ERROR_MEASURES
	{
		RMS,						/**< Root Mean Square error **/
		MAX_DIST_68_PERCENT,		/**< Max distance @ 68% (1 sigma) **/
		MAX_DIST_95_PERCENT,		/**< Max distance @ 98% (2 sigmas) **/
		MAX_DIST_99_PERCENT,		/**< Max distance @ 99% (3 sigmas) **/
		MAX_DIST,					/**< Max distance **/
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

	//! Computes the Chamfer distances (approximated distances) between two point clouds
	/** This methods uses a 3D grid to perfrom the Chamfer Distance propagation.
		Therefore, the greater the octree level (used to determine the grid step) is, the finer
		is the result, but more memory (and time) will be needed.
		\param cType the Chamfer Distance type (1-1-1, 3-4-5, etc.)
		\param comparedCloud the compared cloud
		\param referenceCloud the reference cloud
		\param octreeLevel the octree level at which to perform the Chamfer Distance propagation
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param compOctree the pre-computed octree of the compared cloud (warning: both octrees must have the same cubical bounding-box - it is automatically computed if 0)
		\param refOctree the pre-computed octree of the reference cloud (warning: both octrees must have the same cubical bounding-box - it is automatically computed if 0)
	**/
	static int computeChamferDistanceBetweenTwoClouds(	CC_CHAMFER_DISTANCE_TYPE cType,
														GenericIndexedCloudPersist* comparedCloud,
														GenericIndexedCloudPersist* referenceCloud,
														uchar octreeLevel,
														GenericProgressCallback* progressCb = 0,
														DgmOctree* compOctree = 0,
														DgmOctree* refOctree = 0);

	//! Synchronizes (and re-build if necessary) two octrees
	/** Initializes the octrees before computing the distance between two clouds.
		Check if both octree have the same sizes and limits (in 3D) and rebuild
		them if necessary.
		\param comparedCloud the cloud corresponding to the first octree
		\param referenceCloud the cloud corresponding to the second octree
		\param comparedOctree the first octree
		\param referenceOctree the second octree
		\param progressCb the client method can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return false if a problem has occurred during the process
	**/
	static bool synchronizeOctrees(	GenericIndexedCloudPersist* comparedCloud,
									GenericIndexedCloudPersist* referenceCloud,
									DgmOctree* &comparedOctree,
									DgmOctree* &referenceOctree,
									GenericProgressCallback* progressCb = 0);

protected:

	//! Projects a mesh into a grid structure
	/** This method is used by computePointCloud2MeshDistance.
		\param theIntersection a specific structure to store the result of the intersection
		\param octreeLevel the octree subdivision level corresponding to the grid
		\param progressCb the client method can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
	**/
	static int intersectMeshWithOctree(	OctreeAndMeshIntersection* theIntersection,
										uchar octreeLevel,
										GenericProgressCallback* progressCb = 0);

	//! Computes the distances between a point cloud and a mesh projected into a grid structure
	/** This method is used by computePointCloud2MeshDistance, after intersectMeshWithOctree has been called.
		\param theIntersection a specific structure corresponding the intersection of the mesh with the grid
		\param octreeLevel the octree subdivision level corresponding to the grid
		\param signedDistances specify whether to compute signed or positive (squared) distances
		\param flipTriangleNormals if 'signedDistances' is true,  specify whether triangle normals should be computed in the 'direct' order (true) or 'indirect' (false)
		\param maxSearchDist if greater than 0 (default value: '-1'), then the algorithm won't compute distances over this value
		\param progressCb the client method can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return -1 if an error occurred (e.g. not enough memory) and 0 otherwise
	**/
	static int computePointCloud2MeshDistanceWithOctree(OctreeAndMeshIntersection* theIntersection,
														uchar octreeLevel,
														bool signedDistances,
														bool flipTriangleNormals,
														ScalarType maxSearchDist = -1.0,
														GenericProgressCallback* progressCb = 0);

#ifdef ENABLE_CLOUD2MESH_DIST_MT
	//! Multi-thread version of computePointCloud2MeshSignedDistanceWithOctree and computePointCloud2MeshSquareDistanceWithOctree
	/** Warning: doesn't support the 'maxSearchDist' feature.
		\param theIntersection a specific structure corresponding the intersection of the mesh with the grid
		\param octreeLevel the octree subdivision level corresponding to the grid
		\param signedDistances whether to compute signed or positive (squared) distances
		\param flipTriangleNormals if 'signedDistances' is true, specify whether triangle normals should be computed in the 'direct' order (true) or 'indirect' (false)
		\param progressCb the client method can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
	**/
	static int computePointCloud2MeshDistanceWithOctree_MT(	OctreeAndMeshIntersection* theIntersection,
															uchar octreeLevel,
															bool signedDistances,
															bool flipTriangleNormals=false,
															GenericProgressCallback* progressCb = 0);
#endif

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
												NormalizedProgress* nProgress = 0);

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
															NormalizedProgress* nProgress = 0);
};

}

#endif //DISTANCE_COMPUTATION_TOOLS_HEADER
