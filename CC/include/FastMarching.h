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

#ifndef FAST_MARCHING_HEADER
#define FAST_MARCHING_HEADER

//Local
#include "CCConst.h"
#include "CCGeom.h"

//system
#include <cfloat>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace CCLib
{

class DgmOctree;

// Maximum number of neighbors
#define CC_FM_MAX_NUMBER_OF_NEIGHBOURS 26

//! Grid neighboring cells positions
const int c_FastMarchingNeighbourPosShift[] = {	//6  common faces
												 0,-1, 0,
												 1, 0, 0,
												 0, 1, 0,
												-1, 0, 0,
												 0, 0,-1,
												 0, 0, 1,
												// 20 other neighbors
												-1,-1,-1,
												-1,-1, 0,
												-1,-1, 1,
												-1, 0,-1,
												-1, 0, 1,
												-1, 1,-1,
												-1, 1, 0,
												-1, 1, 1,
												 0,-1,-1,
												 0,-1, 1,
												 0, 1,-1,
												 0, 1, 1,
												 1,-1,-1,
												 1,-1, 0,
												 1,-1, 1,
												 1, 0,-1,
												 1, 0, 1,
												 1, 1,-1,
												 1, 1, 0,
												 1, 1, 1 };

//! Fast Marching algorithm (front propagation)
/** Implementation of the Fast Marching algorithm [Sethian 1996].
	Inspired from the "vtkFastMarching" class of the "Slicer"
	project (http://www.slicer.org).
**/
class CC_CORE_LIB_API FastMarching
{
public:

	//! Default constructor
	FastMarching();

	//! Destructor
	virtual ~FastMarching();

	//! Sets a given cell as "seed"
	/** \param pos the cell position in the grid (3 integer coordinates)
		\return whether the cell could be set as a seed or not
	**/
	virtual bool setSeedCell(const Tuple3i& pos);

	//! Propagates the front
	/** The seeds should have already been initialized
		\return propagation result (errors = negative values)
	**/
	virtual int propagate() = 0;

	/** Finalizes an iteration process
		Resets the different lists and the grid. This method should be
		called after each propagation (before starting a new one).
	**/
	virtual void cleanLastPropagation();

	//! Returns the front arrival time at a given cell
	/** This method should only be called after the propagation
		succeeded. The coordinates of the cell can be absolute
		(i.e. expressed relatively to the octree borders) or relative
		(i.e. expressed relatively to the Fast Marching grid).
		\param pos the cell position (3 integer coordinates)
		\param absoluteCoordinates whether the cell coordinates are absolute or relative
	**/
	virtual float getTime(Tuple3i& pos, bool absoluteCoordinates = false) const;

	//! Sets extended connectivity mode
	/** To use common edges instead of common faces (much slower)
	**/
	virtual void setExtendedConnectivity(bool state) { m_numberOfNeighbours = state ? 26 : 6; }

protected:

	// Macro: cell position [i,j,k] to table (3D grid) index
	inline unsigned pos2index(const Tuple3i& pos) const
	{
		return	  static_cast<unsigned>(pos.x - m_minFillIndexes.x)
				+ static_cast<unsigned>(pos.y - m_minFillIndexes.y) * m_rowSize
				+ static_cast<unsigned>(pos.z - m_minFillIndexes.z) * m_sliceSize + m_indexShift;
	}

	//! A generic Fast Marching grid cell
	class Cell
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

		//! Virtual destructor
		virtual ~Cell() = default;

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
	virtual int initGridWithOctree(DgmOctree* octree, unsigned char gridLevel);

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
	virtual float computeT(unsigned index);

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
	virtual void initTrialCells();

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

		m_theGrid = reinterpret_cast<Cell **>(grid);

		return true;
	}

	//! Add a cell to the TRIAL cells list
	/** \param index index of the cell
	**/
	virtual void addTrialCell(unsigned index);

	//! Add a cell to the ACTIVE cells list
	/** \param index index of the cell
	**/
	virtual void addActiveCell(unsigned index);

	//! Add a cell to the IGNORED cells list
	/** \param index index of the cell
	**/
	virtual void addIgnoredCell(unsigned index);

	//! Returns the TRIAL cell with the smallest front arrival time
	/** \return the index of the "earliest" TRIAL cell (or 0 in case of error)
	**/
	virtual unsigned getNearestTrialCell();

	//! Resets the state of cells in a given list
	/** Warning: the list will be cleared!
	**/
	void resetCells(std::vector<unsigned>& list);

	//! ACTIVE cells list
	std::vector<unsigned> m_activeCells;
	//! TRIAL cells list
	std::vector<unsigned> m_trialCells;
	//! IGNORED cells lits
	std::vector<unsigned> m_ignoredCells;

	//! Specifiies whether structure is initialized or not
	bool m_initialized;
	//! Grid size along the X dimension
	unsigned m_dx;
	//! Grid size along the Y dimension
	unsigned m_dy;
	//! Grid size along the Z dimension
	unsigned m_dz;
	//! Shift for cell access acceleration (Y dimension)
	unsigned m_rowSize;
	//! Shift for cell access acceleration (Z dimension)
	unsigned m_sliceSize;
	//! First index of innerbound grid
	unsigned m_indexShift;
	//! Grid size
	unsigned m_gridSize;
	//! Grid used to process Fast Marching
	Cell** m_theGrid;

	//! Associated octree
	DgmOctree* m_octree;
	//! Equivalent octree subdivision level
	unsigned char m_gridLevel;
	//! Octree cell size at equivalent subdivision level
	float m_cellSize;
	//! Octree min fill indexes at 'm_gridLevel'
	Tuple3i m_minFillIndexes;

	//! Current number of neighbours (6 or 26)
	unsigned m_numberOfNeighbours;
	//! Neighbours coordinates shifts in grid
	int m_neighboursIndexShift[CC_FM_MAX_NUMBER_OF_NEIGHBOURS];
	//! Neighbours distance weight
	float m_neighboursDistance[CC_FM_MAX_NUMBER_OF_NEIGHBOURS];

};

}

#endif //FAST_MARCHING_HEADER
