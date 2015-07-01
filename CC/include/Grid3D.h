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
template< class Type > class Grid3D
{

public:

	//! Cell type
	typedef Type GridElement;

	//! Default constructor
	/** \param di grid size along the X dimension
		\param dj grid size along the Y dimension
		\param dk grid size along the Z dimension
	**/
	Grid3D(unsigned di, unsigned dj, unsigned dk, unsigned margin)
		: m_gridX       (di)
		, m_gridY       (dj)
		, m_gridZ       (dk)
		, m_margin      (margin)
		, m_rowSize     (static_cast<int>(m_gridX+2*m_margin)              )
		, m_sliceSize   (static_cast<int>(m_gridY+2*m_margin) * m_rowSize  )
		, m_gridSize    (static_cast<int>(m_gridZ+2*m_margin) * m_sliceSize)
		, m_marginShift (m_margin + m_rowSize + m_sliceSize)
	{}

	//! Initializes the grid
	/** This memory for the grid must be explicitelty reserved prior to any action.
		\return true if the initialization succeeded
	**/
	bool init(GridElement defaultCellValue)
	{
		if (m_gridSize == 0)
		{
			assert(false);
			return false;
		}

		//grid initialization
		try
		{
			m_grid.resize(m_gridSize,defaultCellValue);
		}
		catch (std::bad_alloc)
		{
			//not enough memory
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

	//! Returns the value of a given cell
	/** \param i the cell coordinate along the X dimension
		\param j the cell coordinate along the Y dimension
		\param k the cell coordinate along the Z dimension
		\return the cell value
	**/
	inline GridElement getValue(int i, int j, int k) const
	{
		return m_grid[pos2index(i,j,k)];
	}

	//! Returns the value of a given cell
	/** \param cellPos the cell position
		\return the cell value
	**/
	GridElement getValue(Tuple3i& cellPos) const
	{
		return m_grid[pos2index(cellPos.x,cellPos.y,cellPos.z)];
	}

protected:

	//! Converts a 3D position to an absolute index
	inline int pos2index(int i, int j, int k) const { return i + j * m_rowSize + k * m_sliceSize + m_marginShift; }

	//! Converts an absolute index to a 3D position
	inline Tuple3i index2pos(int index) const
	{
		assert(index >= m_marginShift && index < m_gridSize);
		
		Tuple3i pos;
		//index = i + j * m_rowSize + k * m_sliceSize + m_marginShift
		index -= m_marginShift;
		//index = i + j * m_rowSize + k * m_sliceSize
		pos.z = index / m_sliceSize;
		index -= pos.z * m_sliceSize;
		//index = i + j * m_rowSize
		pos.y = index / m_rowSize;
		index -= pos.y * m_rowSize;
		//index = i
		pos.x = index;
		
		return pos;
	}

    //! Grid data
	std::vector<GridElement> m_grid;

	//! Grid dimension along the X dimension (without margin)
	unsigned m_gridX;
	//! Grid dimension along the Y dimension (without margin)
	unsigned m_gridY;
	//! Grid dimension along the Z dimension (without margin)
	unsigned m_gridZ;
	//! Margin
	unsigned m_margin;
    //! 1D row size (with margin)
	int m_rowSize;
    //! 2D slice size (with margin)
	int m_sliceSize;
	//! 3D grid size (with margin)
	int m_gridSize;
	//! First index of real data (i.e. after marin)
	int m_marginShift;
};

}

#endif //GRID_3D_HEADER
