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

#ifndef CLOUD_SAMPLING_TOOLS
#define CLOUD_SAMPLING_TOOLS

#include "CCToolbox.h"
#include "DgmOctree.h"

namespace CCLib
{

class GenericProgressCallback;
class GenericIndexedCloud;
class GenericIndexedCloudPersist;
class GenericIndexedMesh;
class ReferenceCloud;
class ReferenceCloudPersist;
class SimpleCloud;

//! Several point cloud resampling algorithms (octree-based, random, etc.)
#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"
class CC_DLL_API CloudSamplingTools : public CCToolbox
#else
class CloudSamplingTools : public CCToolbox
#endif
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
		\param theCloud the point cloud to resample
		\param octreeLevel the octree level at which to perform the resampling process
		\param resamplingMethod resampling method (applied to each octree cell)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param _theOctree if the octree has been already computed, it can be used by the process (avoid recomputation)
		\return the resampled cloud (new cloud)
	**/
	static SimpleCloud* resampleCloudWithOctreeAtLevel(GenericIndexedCloudPersist* theCloud,
                                                                uchar octreeLevel,
                                                                RESAMPLING_CELL_METHOD resamplingMethod,
                                                                GenericProgressCallback* progressCb=0,
                                                                DgmOctree* _theOctree=0);

	//! Resamples a point cloud (process based on the octree)
	/** Same as 'resampleCloudWithOctreeAtLevel' method, appart the fact that instead
		of giving a specific octree subdivision level as input parameter, one can specify
		an approximative number of points for the resulting cloud (algorithm will
		automatically determine the corresponding octree level).
		\param theCloud the point cloud to resample
		\param newNumberOfPoints desired number of points (approximative)
		\param resamplingMethod resampling method (applied to each octree cell)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param _theOctree if the octree has been already computed, it can be used by the process (avoid recomputation)
		\return the resampled cloud (new cloud)
	**/
	static GenericIndexedCloud* resampleCloudWithOctree(GenericIndexedCloudPersist* theCloud,
                                                        int newNumberOfPoints,
                                                        RESAMPLING_CELL_METHOD resamplingMethod,
                                                        GenericProgressCallback* progressCb=0,
                                                        DgmOctree* _theOctree=0);

	//! Subsamples a point cloud (process based on the octree)
	/** A subsampling algorithm is applied inside each cell of the octree. The
		different subsampling methods are represented as an enumerator
		(see SUBSAMPLING_CELL_METHOD) and consist in simple processes
		such as choosing a random point, or the one closest to the cell center.
		\param theCloud point cloud to subsample
		\param octreeLevel octree level at which to perform the subsampling process
		\param subsamplingMethod subsampling method (applied to each octree cell)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param _theOctree if the octree has been already computed, it can be used by the process (avoid recomputation)
		\return a reference cloud corresponding to the subsampling 'selection'
	**/
	static ReferenceCloud* subsampleCloudWithOctreeAtLevel(GenericIndexedCloudPersist* theCloud,
                                                            uchar octreeLevel,
                                                            SUBSAMPLING_CELL_METHOD subsamplingMethod,
                                                            GenericProgressCallback* progressCb=0,
                                                            DgmOctree* _theOctree=0);

	//! Subsamples a point cloud (process based on the octree)
	/** Same as 'subsampleCloudWithOctreeAtLevel' method, appart the fact that instead
		of giving a specific octree subdivision level as input parameter, one can specify
		an approximative number of points for the resulting cloud (algorithm will
		automatically determine the corresponding octree level).
		\param theCloud point cloud to subsample
		\param newNumberOfPoints desired number of points (approximative)
		\param subsamplingMethod resampling method (applied to each octree cell)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param _theOctree if the octree has been already computed, it can be used by the process (avoid recomputation)
		\return a reference cloud corresponding to the subsampling 'selection'
	**/
	static ReferenceCloud* subsampleCloudWithOctree(GenericIndexedCloudPersist* theCloud,
                                                    int newNumberOfPoints,
                                                    SUBSAMPLING_CELL_METHOD subsamplingMethod,
                                                    GenericProgressCallback* progressCb=0,
                                                    DgmOctree* _theOctree=0);

	//! Subsamples a point cloud (process based on random selections)
	/** A very simple subsampling algorithm that simply consists in selecting
		"n" different points, in a random way.
		\param theCloud point cloud to subsample
		\param newNumberOfPoints desired number of points (exact)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return a reference cloud corresponding to the subsampling 'selection'
	**/
	static ReferenceCloud* subsampleCloudRandomly(GenericIndexedCloudPersist* theCloud,
                                                    unsigned newNumberOfPoints,
                                                    GenericProgressCallback* progressCb=0);

	//! Resamples a point cloud (process based on inter point distance)
	/** The cloud is resampled so that there is no point nearer than a given distance to other points
        It works by picking a reference point, removing all points which are to close to this point, and repeating these two steps until the result is reached
		\param theCloud the point cloud to resample
		\param minDistance the distance under which a point in the resulting cloud cannot have any neighbour
		\param theOctree associated octree if available
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return a reference cloud corresponding to the resampling 'selection'
	**/
	static ReferenceCloud* resampleCloudSpatially(GenericIndexedCloudPersist* theCloud,
                                                    float minDistance,
													DgmOctree* theOctree=0,
													GenericProgressCallback* progressCb=0);

protected:

	//! "Cellular" function to replace one set of points (contained in an octree cell) by a unique point
	/** This function is meant to be applied to all cells of the octree
		(it is of the form DgmOctree::localFunctionPtr). It replaces all
		points in a cell by a unique one, according to different rules.
		Method parameters (defined in "additionalParameters") are :
		- (SimpleCloud*) point cloud to store new points
		- (RESAMPLING_CELL_METHOD*) resampling method
	**/
	static bool resampleCellAtLevel(const DgmOctree::octreeCell& cell,
                                    void** additionalParameters);

	//! "Cellular" function to select a unique point inside an octree cell
	/** This function is meant to be applied to all cells of the octree
		(it is of the form DgmOctree::localFunctionPtr). It chooses one point
		from the set of points inside a cell, according to different rules.
		Method parameters (defined in "additionalParameters") are :
		- (ReferenceCloud*) reference point cloud to store selected points
		- (SUBSAMPLING_CELL_METHOD*) subampling method
		\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
	**/
	static bool subsampleCellAtLevel(const DgmOctree::octreeCell& cell,
                                        void** additionalParameters);
};

}

#endif
