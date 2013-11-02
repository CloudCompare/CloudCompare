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

#ifndef FAST_MARCHING_HEADER
#define FAST_MARCHING_HEADER

#ifdef _MSC_VER
//To get rid of the really annoying warnings about template class exportation
#pragma warning( disable: 4251 )
#pragma warning( disable: 4530 )
#endif

#include "CCConst.h"

//system
#include <vector>
#include <float.h>

namespace CCLib
{

class DgmOctree;

// Macro: cell position [i,j,k] to table (3D grid) index
#define FM_pos2index(pos)	( static_cast<unsigned>(pos[0] - m_minFillIndexes[0]) \
							+ static_cast<unsigned>(pos[1] - m_minFillIndexes[1]) * m_decY \
							+ static_cast<unsigned>(pos[2] - m_minFillIndexes[2]) * m_decZ \
							+ m_indexDec )

// Number of neighbuors in 6 connexity mode (common faces)
#define CC_FM_NUMBER_OF_NEIGHBOURS 6

//! 6-connexity neighbouring cells positions (common faces)
const int c_FastMarchingNeighbourPosShift[] = {	 0,-1, 0,
												 1, 0, 0,
												 0, 1, 0,
												-1, 0, 0,
												 0, 0,-1,
												 0, 0, 1 };

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

	//! Default constructor
	FastMarching();

	//! Destructor
	virtual ~FastMarching();

	//! Sets a given cell as "seed"
	/** \param pos the cell position in the grid (3 integer coordinates)
	**/
	virtual void setSeedCell(int pos[]);

	//! Propagates the front
	/** The seeds should have already been initialized
		\return propagation result (errors = negative values)
	**/
	virtual int propagate() = 0;

	//! Returns the front arrival time at a given cell
	/** This method should only be called after the propagation
		succeeded. The coordinates of the cell can be absolute
		(i.e. exressed relatively to the octree borders) or relative
		(i.e. expressed relatively to the Fast Marching grid).
		\param pos the cell position (3 integer coordinates)
		\param absoluteCoordinates whether the cell coordinates are absolute or relative
	**/
	float getTime(int pos[], bool absoluteCoordinates = false) const;

protected:

    //! A generic Fast Marching grid cell
    struct Cell
    {
    public:

		//! Returns infinite time value
		inline static float T_INF() { return FLT_MAX; }

		//! Possible states of a Fast Marching grid cell
        enum STATE {	EMPTY_CELL	= 0,
						FAR_CELL	= 1,
						TRIAL_CELL	= 2,
						ACTIVE_CELL	= 3 };

		//! Default constructor
		Cell()
			: state(FAR_CELL)
			, T(T_INF())
		{}

        //! Cell state
        STATE state;

        //! Front arrival time
        float T;
    };

	//! Intializes the grid as a snapshot of an octree structure at a given subdivision level
	/** \param octree input octree
		\param gridLevel subdivision level
		\return a negative value if a problem occurred
	**/
	virtual int initGridWithOctree(DgmOctree* octree, uchar gridLevel);

	//! Intializes the grid with a given step and dimensions
	/** \param step grid step
		\param dim grid dimensions in 3D
		\return a negative value if a problem occurred
	**/
	virtual int initGrid(float step, unsigned dim[3]);

	//! Intializes other dimension-related variables and finishes the initialization job
	virtual int initOther();

	//! Computes the front arrival time at a given cell
	/** the cell is represented by its index in the cell list
		\param index the cell index
		\return the computed front arrival time
	**/
	virtual float computeT(unsigned index) = 0;

	//! Computes the front acceleration between two cells
	/** \param currentCell the "central" cell
		\param neighbourCell the other cell
		\return the front acceleration
	**/
	virtual float computeTCoefApprox(Cell* currentCell, Cell* neighbourCell) const = 0;

	//! Propagates the front (one step)
	/** \return a negative value if a problem occurred
	**/
	virtual int step() = 0;

	//! Initializes the TRIAL cells list
	/** See the Fast Marching algorithm theory for more information
	**/
	void initTrialCells();

    //! Instantiates grid in memory
    /** Grid is also filled with zeros.
        \param size grid size
        \return success
    **/
	virtual bool instantiateGrid(unsigned size) = 0;

	//! Grid instantiation helper
	template <class T> bool instantiateGridTpl(unsigned size)
	{
		if (m_theGrid)
			return false;

		T** grid = new T*[size];
		if (!grid)
			return false;
		memset(grid,0,size*sizeof(T*));

		m_theGrid = (Cell**)grid;

		return true;
	}

	//! Add a cell to the TRIAL cells list
	/** \param index index of the cell
	**/
	virtual void addTrialCell(unsigned index);

	//! Returns the TRIAL cell with the smallest front arrival time
	/** \return the index of the "earliest" TRIAL cell (or 0 in case of error)
	**/
	virtual unsigned getNearestTrialCell();

	//! ACTIVE cells list
	std::vector<unsigned> m_activeCells;

	//! Specifiies whether structure is initialized or not
	bool m_initialized;
	//! Grid size along the X dimension
	unsigned m_dx;
	//! Grid size along the Y dimension
	unsigned m_dy;
	//! Grid size along the Z dimension
	unsigned m_dz;
	//! Shift for cell access acceleration (Y dimension)
	unsigned m_decY;
	//! Shift for cell access acceleration (Y dimension)
	unsigned m_decZ;
	//! First index of innerbound grid
	unsigned m_indexDec;
	//! Grid size
	unsigned m_gridSize;
	//! Grid used to process Fast Marching
	Cell** m_theGrid;
	//! TRIAL cells list
	std::vector<unsigned> m_trialCells;

	//! Associated octree
	DgmOctree* m_octree;
	//! Equivalent octree subdivision level
	uchar m_gridLevel;
	//! Octree cell size at equivalent subdivision level
	float m_cellSize;
	//! Octree min fill indexes at 'm_gridLevel'
	int m_minFillIndexes[3];

	//! Neighbours coordinates shifts in grid
	int m_neighboursIndexShift[CC_FM_NUMBER_OF_NEIGHBOURS];
	//! Neighbours distance weight
	float m_neighboursDistance[CC_FM_NUMBER_OF_NEIGHBOURS];

};

}

#endif //FAST_MARCHING_HEADER
