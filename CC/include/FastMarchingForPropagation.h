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

#ifndef FAST_MARCHING_PROP_HEADER
#define FAST_MARCHING_PROP_HEADER

#include "FastMarching.h"
#include "CCTypes.h"

namespace CCLib
{

class ReferenceCloud;
class GenericCloud;

//! Fast Marching algorithm for surfacical front propagation
/** Re-implements the FastMarching class.
**/

class FastMarchingForPropagation : public FastMarching
{

public:

	//! Default constructor
	FastMarchingForPropagation();

	//! Initializes the grid with a point cloud (and ist corresponding octree)
	/** The points should be associated to scalar values.
		Warning: be sure to activate an OUTPUT scalar field on the input cloud
		The Fast Marching grid will have the same characteristics as
		the octree considered at a given level of subdivision. The local
		front acceleration in each cell is deduced from the scalar values
		associated to the points lying in the octree cell (mean value).
		\param aList the point cloud
		\param theOctree the associated octree
		\param gridLevel the level of subdivision
		\param constantAcceleration specifies if the acceleration is constant or shoul be computed from the cell points scalar values
		\return a negative value if something went wrong
	**/
	int init(GenericCloud* aList, DgmOctree* theOctree, uchar gridLevel, bool constantAcceleration=false);

	/** Finalizes an iteration process
        Resets the different lists and the grid. This method should be
		called after each propagation.
	**/
	void endPropagation();

	//! Returns a list of the points (references to) reached by the propagation process
	/** Returns a cloud of points (references to) corresponding to the points that are
		lying in cells that have been visited by the last propagation process.
		\return a cloud of references to points
	**/
	ReferenceCloud* extractPropagatedPoints();

	//! Sets the propagation timings as distances for each point
	/** \return true if ok, false otherwise
	**/
	bool setPropagationTimingsAsDistances();

	//! Sets the threshold for propagation stop
	/** This threshold corresponds to the maximum front arrival time
		increase allowed. If the delta between the fornt arrival time
		at two consecutive cells is higher, the propagation process is
		stoped.
		\param value the threshold
	**/
	void setDetectionThreshold(float value) {detectionThreshold=value;};

	//! Sets the accceleration exageration factor
	/** In order to detect the front arrival time jumps (see
		FastMarchingForPropagation::setDetectionThreshold), it
		is possible to exagerate the result of the acceleration
		computation with this factor.
		\param value the acceleration exageration factor
	**/
	void setJumpCoef(float value) {jumpCoef=value;};

	//! Find peaks of local acceleration values
	/** This method is useful when using this Fast Marching
		algorithm in a Watershed process. Peak cells are
		automatically set as "seeds".
	**/
	void findPeaks();

	//inherited methods (see FastMarching)
	virtual int propagate();

protected:

	//inherited methods (see FastMarching)
	virtual float computeT(unsigned index);
	virtual float computeTCoefApprox(Cell* currentCell, Cell* neighbourCell);
	virtual int step();
	virtual void addTrialCell(unsigned index, float T);
	virtual unsigned getNearestTrialCell();
	virtual bool instantiateGrid(unsigned size);

	//! Compute the "biggest" (latest) front arrival time of the ACTIVE cells
	void initLastT();

	//! TRIAL cells list
	std::vector<unsigned> trialCells;

	//! Accceleration exageration factor
	float jumpCoef;
	//! Threshold for propagation stop
	float detectionThreshold;
	//! Latest front arrival time of the ACTIVE cells
	float lastT;

    //! A Fast Marching grid cell for surfacical propagation
    class PropagationCell : public Cell
    {
    public:
        //! The local front acceleration
        float f;
        //! the code of the equivalent cell in the octree
        OctreeCellCodeType cellCode;
    };
};

}

#endif
