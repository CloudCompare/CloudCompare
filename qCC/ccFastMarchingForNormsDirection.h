//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
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
//$Author:: dgm                                                            $
//$Rev:: 2172                                                              $
//$LastChangedDate:: 2012-06-24 18:33:24 +0200 (dim., 24 juin 2012)        $
//**************************************************************************
//

#ifndef CC_FAST_MARCHING_DIRECTION_HEADER
#define CC_FAST_MARCHING_DIRECTION_HEADER

//CCLib
#include <CCTypes.h>
#include <FastMarching.h>
#include <GenericProgressCallback.h>

//qCC_db
#include <ccAdvancedTypes.h>

//System
#include <queue>

class ccGenericPointCloud;
class ccPointCloud;

//! Fast Marching algorithm for normals direction resolution
/** Extends the FastMarchingAlgorithm class (basic algorithm).
**/
class ccFastMarchingForNormsDirection : public CCLib::FastMarching
{

public:

    //! Static entry point (helper)
	static int ResolveNormsDirectionByFrontPropagation(ccPointCloud* theCloud,
                                                        NormsIndexesTableType* theNorms,
                                                        uchar octreeLevel,
                                                        CCLib::GenericProgressCallback* progressCb=0,
                                                        CCLib::DgmOctree* _theOctree=0);
	//! Default constructor
	ccFastMarchingForNormsDirection();

	//! Initializes the grid with a point cloud (and ist corresponding octree)
	/** The points should be associated to scalar values.
		The Fast Marching grid will have the same characteristics as
		the octree considered at a given level of subdivision. The local
		front acceleration in each cell is deduced from the scalar values
		associated to the points lying in the octree cell (mean value).
		\param aList the point cloud
		\param theNorms the normals array
		\param theOctree the associated octree
		\param gridLevel the level of subdivision
		\return a negative value if something went wrong
	**/
	int init(ccGenericPointCloud* aList,
            NormsIndexesTableType* theNorms,
            CCLib::DgmOctree* theOctree,
            uchar gridLevel);

	//! Finalizes an iteration process
	/** Resets the different lists and the grid. This method should be
		called after each propagation.
	**/
	void endPropagation();

	//! updates a list of point flags, indicating the points alreay treated
	/** \return the number of resolved points **/
	int updateResolvedTable(ccGenericPointCloud* theCloud,
                            GenericChunkedArray<1,uchar> &resolved,
                            NormsIndexesTableType* theNorms);

	//! Find peaks of local acceleration values
	/** This method is usefull when using this Fast Marching
		algorithm in a Watershed process. Peak cells are
		automatically set as "seeds".
	**/
	void findPeaks();

	//inherited methods (see FastMarchingAlgorithm)
	int propagate();

protected:

    //! A Fast Marching grid cell for normals direction resolution
    class DirectionCell : public CCLib::FastMarching::Cell
    {
    public:
        //! The local front acceleration
        PointCoordinateType N[3];
        //! the code of the equivalent cell in the octree
        OctreeCellCodeType cellCode;
        //Temp value
        DistanceType v;
        //marker
        bool treated;
    };

	//inherited methods (see FastMarchingAlgorithm)
	virtual float computeT(unsigned index);
	virtual float computeTCoefApprox(CCLib::FastMarching::Cell* currentCell, CCLib::FastMarching::Cell* neighbourCell) {return 1.0;};
	virtual int step();
	virtual void addTrialCell(unsigned index, float T);
	virtual unsigned getNearestTrialCell();
	virtual void initTrialCells();
	virtual bool instantiateGrid(unsigned size);

	//! Compute the "biggest" (latest) front arrival time of the ACTIVE cells
	void initLastT();

	//! TRIAL cells list
	std::vector<unsigned> trialCells;

	// Usefull variables
	float lastT;
};

#endif
