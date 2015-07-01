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

#ifndef GRID_3D_HEADER
#define GRID_3D_HEADER

//Local
#include "CCGeom.h"
#include "CCConst.h"

//System
#include <vector>
#include <assert.h>

namespace CCLib
{

//! Simple 3D grid structure
/** The grid data is contiguous in memory.
**/
template< class Type > class Grid3D
{

public:

	//! Cell type
	typedef Type GridElement;

	//! Default constructor
	Grid3D()
		: m_innerSize      (0,0,0)
		, m_margin         (0)
		, m_rowSize        (0)
		, m_sliceSize      (0)
		, m_innerCellCount (0)
		, m_totalCellCount (0)
		, m_marginShift    (0)
	{}

	//! Returns the grid dimensions
	inline const Tuple3ui& size() const { return m_innerSize; }

	//! Returns whether the grid has been initialized or not
	inline bool isInitialized() const { return m_totalCellCount != 0; }

	//! Initializes the grid
	/** The grid must be explicitelty initialized prior to any action.
		\param di grid size along the X dimension
		\param dj grid size along the Y dimension
		\param dk grid size along the Z dimension
		\param margin grid margin
		\param defaultCellValue default cell value
		\return true if the initialization succeeded
	**/
	bool init(unsigned di, unsigned dj, unsigned dk, unsigned margin, GridElement defaultCellValue = 0)
	{
		m_innerSize      = Tuple3ui(di,dj,dk);
		m_margin         = margin;
		m_innerCellCount = m_innerSize.x * m_innerSize.y * m_innerSize.z;
		m_rowSize        = (m_innerSize.x + 2*m_margin);
		m_sliceSize      = (m_innerSize.y + 2*m_margin) * m_rowSize;
		m_totalCellCount = (m_innerSize.z + 2*m_margin) * m_sliceSize;
		m_marginShift    = m_margin * (1 + m_rowSize + m_sliceSize);

		if (m_totalCellCount == 0)
		{
			assert(false);
			return false;
		}

		//grid initialization
		try
		{
			m_grid.resize(m_totalCellCount,defaultCellValue);
		}
		catch (std::bad_alloc)
		{
			//not enough memory
			m_totalCellCount = 0;
			return false;
		}

		return true;
	}

	//! Sets the value of a given cell
	/** \param i the cell coordinate along the X dimension
		\param j the cell coordinate along the Y dimension
		\param k the cell coordinate along the Z dimension
	**/
	inline void setValue(int i, int j, int k, GridElement value)
	{
		m_grid[pos2index(i,j,k)] = value;
	}

	//! Sets the value of a given cell
	/** \param cellPos the cell position
	**/
	inline void setValue(Tuple3i& cellPos, GridElement value)
	{
		m_grid[pos2index(cellPos.x,cellPos.y,cellPos.z)] = value;
	}

	//! Returns the value of a given cell (const version)
	/** \param i the cell coordinate along the X dimension
		\param j the cell coordinate along the Y dimension
		\param k the cell coordinate along the Z dimension
		\return the cell value
	**/
	inline const GridElement& getValue(int i, int j, int k) const
	{
		return m_grid[pos2index(i,j,k)];
	}
	//! Returns the value of a given cell
	/** \param i the cell coordinate along the X dimension
		\param j the cell coordinate along the Y dimension
		\param k the cell coordinate along the Z dimension
		\return the cell value
	**/
	inline GridElement& getValue(int i, int j, int k)
	{
		return m_grid[pos2index(i,j,k)];
	}

	//! Returns the value of a given cell const version)
	/** \param cellPos the cell position
		\return the cell value
	**/
	const GridElement& getValue(Tuple3i& cellPos) const
	{
		return m_grid[pos2index(cellPos.x,cellPos.y,cellPos.z)];
	}
	//! Returns the value of a given cell
	/** \param cellPos the cell position
		\return the cell value
	**/
	GridElement& getValue(Tuple3i& cellPos)
	{
		return m_grid[pos2index(cellPos.x,cellPos.y,cellPos.z)];
	}

	//! Gives access to the internal grid data (with margin)
	inline GridElement* data() { return &(m_grid[0]); }
	//! Gives access to the internal grid data (with margin) (const version)
	inline const GridElement* data() const { return &(m_grid[0]); }

	//! Returns the number of cell count (whithout margin)
	inline unsigned innerCellCount() const { return m_innerCellCount; }
	//! Returns the total number of cell count (with margin)
	inline unsigned totalCellCount() const { return m_totalCellCount; }

protected:

	//! Converts a 3D position to an absolute index
	inline int pos2index(int i, int j, int k) const { return i + j * static_cast<int>(m_rowSize) + k * static_cast<int>(m_sliceSize) + static_cast<int>(m_marginShift); }

    //! Grid data
	std::vector<GridElement> m_grid;

	//! Dimensions of the grid (without margin)
	Tuple3ui m_innerSize;
	//! Margin
	unsigned m_margin;
    //! 1D row size (with margin)
	unsigned m_rowSize;
    //! 2D slice size (with margin)
	unsigned m_sliceSize;
	//! 3D grid size without margin
	unsigned m_innerCellCount;
	//! 3D grid size with margin
	unsigned m_totalCellCount;
	//! First index of real data (i.e. after marin)
	unsigned m_marginShift;
};

}

#endif //GRID_3D_HEADER
