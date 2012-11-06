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

#ifndef FAST_MARCHING_HEADER
#define FAST_MARCHING_HEADER

#ifdef _MSC_VER
//To get rid of the really annoying warnings about template class exportation
#pragma warning( disable: 4251 )
#pragma warning( disable: 4530 )
#endif

#include "CCConst.h"
#include <vector>

namespace CCLib
{

class DgmOctree;

// Macro: cell position [i,j,k] to table (3D grid) index
#define FM_pos2index(pos) unsigned(pos[0]-minFillIndexes[0])+unsigned(pos[1]-minFillIndexes[1])*decY+unsigned(pos[2]-minFillIndexes[2])*decZ+indexDec

//! Direct neighbouring cells positions (6-connexity)
const int neighboursPosShift[] = {0,-1,0,
								1,0,0,
								0,+1,0,
								-1,0,0,
								0,0,-1,
								0,0,1};
#define CC_FM_NUMBER_OF_NEIGHBOURS 6

//! All neighbouring cells positions (26-connexity)
const int neighbours3DPosShift[] = {
								-1,-1,-1,
								-1,-1,0,
								-1,-1,1,
								-1,0,-1,
								-1,0,0,
								-1,0,1,
								-1,1,-1,
								-1,1,0,
								-1,1,1,
								0,-1,-1,
								0,-1,0,
								0,-1,1,
								0,0,-1,
								0,0,1,
								0,1,-1,
								0,1,0,
								0,1,1,
								1,-1,-1,
								1,-1,0,
								1,-1,1,
								1,0,-1,
								1,0,0,
								1,0,1,
								1,1,-1,
								1,1,0,
								1,1,1};
#define CC_FM_NUMBER_OF_3D_NEIGHBOURS 26

// Infinite arrival time value (for grid initialization)
#define FM_INF FLOAT_MAX

//! Fast Marching algorithm (front propagation)
/** Implementation of the Fast Marching algorithm [Sethian 1996].
	Inspired from the "vtkFastMarching" class of the "Slicer"
	project (http://www.slicer.org).
**/

#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"

class CC_DLL_API FastMarching
#else
class FastMarching
#endif
{
public:

	//! The FastMarchingAlgorithm constructor
	FastMarching();

	//! The FastMarchingAlgorithm destructor
	virtual ~FastMarching();

	//! Sets a given cell as "seed"
	/** \param pos the cell position in the grid (3 integer coordinates)
	**/
	void setSeedCell(int pos[]);

	//! Propagates the front
	/** The seeds should have been initialized before
		\return propagation result (negative value = problem)
	**/
	virtual int propagate()=0;

	//! Returns the front arrival time at a given cell
	/** This method should be only called after the propagation
		succeeded. The coordinates of the cell can be
		absolute (relatively to the octree borders) or
		relative (relatively to the Fast Marching grid).
		\param pos the cell position (3 integer coordinates)
		\param absoluteCoordinates specifies if the coordinates are absolute or relative
	**/
	float getTime(int pos[], bool absoluteCoordinates=false);

protected:

    //! A generic Fast Marching grid cell
    class Cell
    {
    public:

        //! Possible states of a Fast Marching grid cell
        enum CC_FM_CELL_STATE {EMPTY_CELL=0,FAR_CELL=1,TRIAL_CELL=2,ACTIVE_CELL=3};

        //! Cell state
        CC_FM_CELL_STATE state;

        //! Front arrival time
        float T;
    };

	//! Intializes the grid
	/** Reserves memory for the grid
		\return a negative value if a problem occured
	**/
	virtual int initGrid();

	//! Computes the front arrival time at a given cell
	/** the cell is represented by its index in the cell list
		\param index the cell index
		\return the computed front arrival time
	**/
	virtual float computeT(unsigned index)=0;

	//! Computes the front acceleration between two cells
	/** \param currentCell the "central" cell
		\param neighbourCell the other cell
		\return the front acceleration
	**/
	virtual float computeTCoefApprox(Cell* currentCell, Cell* neighbourCell)=0;

	//! Propagates the front (one step)
	/** \return a negative value if a problem occured **/
	virtual int step()=0;

	//! Initializes the TRIAL cells list
	/** See the Fast Marching algorithm theory for more information
	**/
	void initTrialCells();

    //! Instantiates grid in memory
    /** Grid is also filled with zeros.
        \param size grid size
        \return success
    **/
	virtual bool instantiateGrid(unsigned size);

	//! Add a cell to the TRIAL cells list
	/** \param index index of the cell
		\param T front arrival time at this cell
	**/
	virtual void addTrialCell(unsigned index, float T)=0;

	//! Returns the TRIAL cell with the smallest front arrival time
	/** \return the index of the "first" TRIAL cell
	**/
	virtual unsigned getNearestTrialCell()=0; //renvoie 0 si problème

	//! ACTIVE cells list
	std::vector<unsigned> activeCells;

	//! Specifiies whether structure is initialized or not
	bool initialized;
	//! Grid size along the X dimension
	unsigned dx;
	//! Grid size along the Y dimension
	unsigned dy;
	//! Grid size along the Z dimension
	unsigned dz;
	//! Shift for cell access acceleration (Y dimension)
	unsigned decY;
	//! Shift for cell access acceleration (Y dimension)
	unsigned decZ;
	//! First index of innerbound grid
	unsigned indexDec;
	//! Grid size
	unsigned gridSize;
	//! Grid used to process Fast Marching
	Cell** theGrid;
	//! Grid occupancy (min indexes)
	const int *minFillIndexes;
	//! Grid occupancy (min indexes)
    const int *maxFillIndexes;
	//! Equivalent octree subdivision level
	uchar gridLevel;
	//! Octree cell size at equivalent subdivision level
	float cellSize;
	//! Associated octree
	DgmOctree* theOctree;

	//! Neighbours coordinates shifts in grid
	int neighboursIndexShift[CC_FM_NUMBER_OF_NEIGHBOURS];
	//! Neighbours distance weight
	float neighboursDistance[CC_FM_NUMBER_OF_NEIGHBOURS];

};

}

#endif
