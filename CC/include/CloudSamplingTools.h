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

#ifndef CLOUD_SAMPLING_TOOLS_HEADER
#define CLOUD_SAMPLING_TOOLS_HEADER

//Local
#include "CCToolbox.h"
#include "DgmOctree.h"

namespace CCLib
{

class GenericIndexedCloud;
class GenericIndexedCloudPersist;
class GenericIndexedMesh;
class GenericProgressCallback;
class PointCloud;
class ReferenceCloud;
class ReferenceCloudPersist;

//! Several point cloud resampling algorithms (octree-based, random, etc.)
class CC_CORE_LIB_API CloudSamplingTools : public CCToolbox
{
public:

	//! Octree cell resampling methods (see CloudSamplingTools::resampleCloudWithOctree)
	enum RESAMPLING_CELL_METHOD {CELL_CENTER, CELL_GRAVITY_CENTER};

	//! Octree cell simplifying methods (see CloudSamplingTools::subsampleCloudWithOctree)
	enum SUBSAMPLING_CELL_METHOD {RANDOM_POINT, NEAREST_POINT_TO_CELL_CENTER};

	//! Resamples a point cloud (process based on the octree)
	/** A resampling algorithm is applied inside each cell of the octree. The
		different resampling methods are represented as an enumerator
		(see RESAMPLING_CELL_METHOD) and consist in simple processes
		such as replacing all the points lying in a cell by the cell center or
		by the points gravity center.
		\param cloud the point cloud to resample
		\param octreeLevel the octree level at which to perform the resampling process
		\param resamplingMethod resampling method (applied to each octree cell)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param inputOctree if the octree has been already computed, it can be used by the process (avoid recomputation)
		\return the resampled cloud (new cloud)
	**/
	static PointCloud* resampleCloudWithOctreeAtLevel(	GenericIndexedCloudPersist* cloud,
														unsigned char octreeLevel,
														RESAMPLING_CELL_METHOD resamplingMethod,
														GenericProgressCallback* progressCb = nullptr,
														DgmOctree* inputOctree = nullptr);

	//! Resamples a point cloud (process based on the octree)
	/** Same as 'resampleCloudWithOctreeAtLevel' method, apart the fact that instead
		of giving a specific octree subdivision level as input parameter, one can specify
		an approximative number of points for the resulting cloud (algorithm will
		automatically determine the corresponding octree level).
		\param cloud the point cloud to resample
		\param newNumberOfPoints desired number of points (approximative)
		\param resamplingMethod resampling method (applied to each octree cell)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param inputOctree if the octree has been already computed, it can be used by the process (avoid recomputation)
		\return the resampled cloud (new cloud)
	**/
	static GenericIndexedCloud* resampleCloudWithOctree(GenericIndexedCloudPersist* cloud,
														int newNumberOfPoints,
														RESAMPLING_CELL_METHOD resamplingMethod,
														GenericProgressCallback* progressCb = nullptr,
														DgmOctree* inputOctree = nullptr);

	//! Subsamples a point cloud (process based on the octree)
	/** A subsampling algorithm is applied inside each cell of the octree. The
		different subsampling methods are represented as an enumerator
		(see SUBSAMPLING_CELL_METHOD) and consist in simple processes
		such as choosing a random point, or the one closest to the cell center.
		\param cloud point cloud to subsample
		\param octreeLevel octree level at which to perform the subsampling process
		\param subsamplingMethod subsampling method (applied to each octree cell)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param inputOctree if the octree has been already computed, it can be used by the process (avoid recomputation)
		\return a reference cloud corresponding to the subsampling 'selection'
	**/
	static ReferenceCloud* subsampleCloudWithOctreeAtLevel(GenericIndexedCloudPersist* cloud,
															unsigned char octreeLevel,
															SUBSAMPLING_CELL_METHOD subsamplingMethod,
															GenericProgressCallback* progressCb = nullptr,
															DgmOctree* inputOctree = nullptr);

	//! Subsamples a point cloud (process based on the octree)
	/** Same as 'subsampleCloudWithOctreeAtLevel' method, apart the fact that instead
		of giving a specific octree subdivision level as input parameter, one can specify
		an approximative number of points for the resulting cloud (algorithm will
		automatically determine the corresponding octree level).
		\param cloud point cloud to subsample
		\param newNumberOfPoints desired number of points (approximative)
		\param subsamplingMethod resampling method (applied to each octree cell)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param inputOctree if the octree has been already computed, it can be used by the process (avoid recomputation)
		\return a reference cloud corresponding to the subsampling 'selection'
	**/
	static ReferenceCloud* subsampleCloudWithOctree(GenericIndexedCloudPersist* cloud,
													int newNumberOfPoints,
													SUBSAMPLING_CELL_METHOD subsamplingMethod,
													GenericProgressCallback* progressCb = nullptr,
													DgmOctree* inputOctree = nullptr);

	//! Subsamples a point cloud (process based on random selections)
	/** A very simple subsampling algorithm that simply consists in selecting
		"n" different points, in a random way.
		\param cloud point cloud to subsample
		\param newNumberOfPoints desired number of points (exact)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return a reference cloud corresponding to the subsampling 'selection'
	**/
	static ReferenceCloud* subsampleCloudRandomly(	GenericIndexedCloudPersist* cloud,
													unsigned newNumberOfPoints,
													GenericProgressCallback* progressCb = nullptr);

	//! Parameters for the scalar-field based modulation of a parameter
	struct SFModulationParams
	{
		//! Default constructor
		explicit SFModulationParams(bool state = false) : enabled(state), a(0.0), b(1.0) {}

		//! Whether the modulation is enabled or not
		bool enabled;
		//! Modulation scheme: y = a.sf + b
		double a;
		//! Modulation scheme: y = a.sf + b
		double b;
	};

	//! Resamples a point cloud (process based on inter point distance)
	/** The cloud is resampled so that there is no point nearer than a given distance to other points
		It works by picking a reference point, removing all points which are to close to this point, and repeating these two steps until the result is reached
		\param cloud the point cloud to resample
		\param minDistance the distance under which a point in the resulting cloud cannot have any neighbour
		\param modParams parameters of the subsampling behavior modulation with a scalar field (optional)
		\param octree associated octree if available
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return a reference cloud corresponding to the resampling 'selection'
	**/
	static ReferenceCloud* resampleCloudSpatially(	GenericIndexedCloudPersist* cloud,
													PointCoordinateType minDistance,
													const SFModulationParams& modParams,
													DgmOctree* octree = nullptr,
													GenericProgressCallback* progressCb = nullptr);

	//! Statistical Outliers Removal (SOR) filter
	/** This filter removes points based on their mean distance to their distance (by comparing it to the average distance of all points to their neighbors).
		It is equivalent to PCL StatisticalOutlierRemoval filter (see http://pointclouds.org/documentation/tutorials/statistical_outlier.php)
		\param cloud the point cloud to resample
		\param knn number of neighbors
		\param nSigma number of sigmas under which the points should be kept
		\param octree associated octree if available
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return a reference cloud corresponding to the filtered cloud
	**/
	static ReferenceCloud* sorFilter(	GenericIndexedCloudPersist* cloud,
										int knn = 6,
										double nSigma = 1.0,
										DgmOctree* octree = nullptr,
										GenericProgressCallback* progressCb = nullptr);

	//! Noise filter based on the distance to the approximate local surface
	/** This filter removes points based on their distance relatively to the best fit plane computed on their neighbors.
		\param cloud the point cloud to resample
		\param kernelRadius neighborhood radius
		\param nSigma number of sigmas under which the points should be kept
		\param removeIsolatedPoints whether to remove isolated points (i.e. with 3 points or less in the neighborhood)
		\param useKnn whether to use a constant number of neighbors instead of a radius
		\param knn number of neighbors (if useKnn is true)
		\param useAbsoluteError whether to use an absolute error instead of 'n' sigmas
		\param absoluteError absolute error (if useAbsoluteError is true)
		\param octree associated octree if available
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return a reference cloud corresponding to the filtered cloud
	**/
	static ReferenceCloud* noiseFilter(	GenericIndexedCloudPersist* cloud,
										PointCoordinateType kernelRadius,
										double nSigma,
										bool removeIsolatedPoints = false,
										bool useKnn = false,
										int knn = 6,
										bool useAbsoluteError = true,
										double absoluteError = 0.0,
										DgmOctree* octree = nullptr,
										GenericProgressCallback* progressCb = nullptr);

protected:

	//! "Cellular" function to replace one set of points (contained in an octree cell) by a unique point
	/** This function is meant to be applied to all cells of the octree
		(it is of the form DgmOctree::localFunctionPtr). It replaces all
		points in a cell by a unique one, according to different rules.
		Method parameters (defined in "additionalParameters") are :
		- (PointCloud*) point cloud to store new points
		- (RESAMPLING_CELL_METHOD*) resampling method
		\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
		\param nProgress optional (normalized) progress notification (per-point)
	**/
	static bool resampleCellAtLevel(const DgmOctree::octreeCell& cell,
									void** additionalParameters,
									NormalizedProgress* nProgress = nullptr);

	//! "Cellular" function to select a unique point inside an octree cell
	/** This function is meant to be applied to all cells of the octree
		(it is of the form DgmOctree::localFunctionPtr). It chooses one point
		from the set of points inside a cell, according to different rules.
		Method parameters (defined in "additionalParameters") are :
		- (ReferenceCloud*) reference point cloud to store selected points
		- (SUBSAMPLING_CELL_METHOD*) subampling method
		\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
		\param nProgress optional (normalized) progress notification (per-point)
	**/
	static bool subsampleCellAtLevel(	const DgmOctree::octreeCell& cell,
										void** additionalParameters,
										NormalizedProgress* nProgress = nullptr);

	//! "Cellular" function to apply the noise filter inside an octree cell
	/** This function is meant to be applied to all cells of the octree
		(it is of the form DgmOctree::localFunctionPtr).
		\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
		\param nProgress optional (normalized) progress notification (per-point)
	**/
	static bool applyNoiseFilterAtLevel(const DgmOctree::octreeCell& cell,
										void** additionalParameters,
										NormalizedProgress* nProgress = nullptr);

	//! "Cellular" function to apply the SOR filter inside an octree cell
	/** This function is meant to be applied to all cells of the octree
		(it is of the form DgmOctree::localFunctionPtr).
		\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
		\param nProgress optional (normalized) progress notification (per-point)
	**/
	static bool applySORFilterAtLevel(	const DgmOctree::octreeCell& cell,
										void** additionalParameters,
										NormalizedProgress* nProgress = nullptr);
};

}

#endif //CLOUD_SAMPLING_TOOLS_HEADER
