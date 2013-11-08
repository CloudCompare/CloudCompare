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

#include "FastMarching.h"

//local
#include "DgmOctree.h"

//system
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

using namespace CCLib;

FastMarching::FastMarching()
	: m_initialized(false)
	, m_dx(0)
	, m_dy(0)
	, m_dz(0)
	, m_decY(0)
	, m_decZ(0)
	, m_indexDec(0)
	, m_gridSize(0)
	, m_theGrid(0)
	, m_octree(0)
	, m_gridLevel(0)
	, m_cellSize(1.0f)
{
	memset(m_minFillIndexes,0,sizeof(int)*3);
}

FastMarching::~FastMarching()
{
	if (m_theGrid)
	{
		for (unsigned i=0; i<m_gridSize; ++i)
			if (m_theGrid[i])
				delete m_theGrid[i];

		delete[] m_theGrid;
	}
}

float FastMarching::getTime(int pos[], bool absoluteCoordinates) const
{
	unsigned index = 0;

	if (absoluteCoordinates)
	{
		index = FM_pos2index(pos);
	}
	else
	{
		index =	  static_cast<unsigned>(pos[0]+1)
				+ static_cast<unsigned>(pos[1]+1) * m_decY
				+ static_cast<unsigned>(pos[2]+1) * m_decZ;
	}

	assert(m_theGrid[index]);

	return m_theGrid[index]->T;
}

int FastMarching::initGrid(float step, unsigned dim[3])
{
	m_octree = 0;
	m_gridLevel = 0;
	m_cellSize = step;
	m_minFillIndexes[0] = 0;
	m_minFillIndexes[1] = 0;
	m_minFillIndexes[2] = 0;

	m_dx = dim[0];
	m_dy = dim[1];
	m_dz = dim[2];

	return initOther();
}

int FastMarching::initGridWithOctree(DgmOctree* octree, uchar gridLevel)
{
	if (!octree || gridLevel > DgmOctree::MAX_OCTREE_LEVEL)
		return -2;

	const int* minFillIndexes = octree->getMinFillIndexes(gridLevel);
	const int* maxFillIndexes = octree->getMaxFillIndexes(gridLevel);

	m_octree = octree;
	m_gridLevel = gridLevel;
	m_cellSize = octree->getCellSize(gridLevel);
	m_minFillIndexes[0] = minFillIndexes[0];
	m_minFillIndexes[1] = minFillIndexes[1];
	m_minFillIndexes[2] = minFillIndexes[2];

	m_dx = static_cast<unsigned>(maxFillIndexes[0]-minFillIndexes[0]+1);
	m_dy = static_cast<unsigned>(maxFillIndexes[1]-minFillIndexes[1]+1);
	m_dz = static_cast<unsigned>(maxFillIndexes[2]-minFillIndexes[2]+1);

	return initOther();
}

int FastMarching::initOther()
{
	m_decY = m_dx+2;
	m_decZ = m_decY*(m_dy+2);
	m_gridSize = m_decZ*(m_dz+2);
	m_indexDec = 1+m_decY+m_decZ;

	for (unsigned i=0; i<CC_FM_NUMBER_OF_NEIGHBOURS; ++i)
	{
		m_neighboursIndexShift[i] =	  c_FastMarchingNeighbourPosShift[i*3  ]
									+ c_FastMarchingNeighbourPosShift[i*3+1] * static_cast<int>(m_decY)
									+ c_FastMarchingNeighbourPosShift[i*3+2] * static_cast<int>(m_decZ);

		m_neighboursDistance[i] =	sqrt(static_cast<float>(c_FastMarchingNeighbourPosShift[i*3  ] * c_FastMarchingNeighbourPosShift[i*3  ]+
															c_FastMarchingNeighbourPosShift[i*3+1] * c_FastMarchingNeighbourPosShift[i*3+1]+
															c_FastMarchingNeighbourPosShift[i*3+2] * c_FastMarchingNeighbourPosShift[i*3+2]))
									* m_cellSize;
	}

	m_activeCells.clear();

	if (!instantiateGrid(m_gridSize))
        return -3;

	return 0;
}

void FastMarching::cleanLastPropagation()
{
	//reset cells state
	//DGM: we don't use anymore the m_activeCells and
	//m_trialCells vectors to optimize the number of cells
	//tested as some cells may be 'lost' (i.e. not in those
	//vectors).
	for (unsigned i=0; i<m_gridSize; ++i)
	{
		if (m_theGrid[i])
		{
			m_theGrid[i]->state = Cell::FAR_CELL;
			m_theGrid[i]->T = Cell::T_INF();
		}
	}

	m_activeCells.clear();
	m_trialCells.clear();
}

bool FastMarching::setSeedCell(int pos[])
{
	unsigned index = FM_pos2index(pos);

	assert(index<m_gridSize);

	Cell* aCell = m_theGrid[index];
	assert(aCell);

	if (aCell && aCell->state != Cell::ACTIVE_CELL)
	{
		//we add the cell to the "ACTIVE" set
		aCell->T = 0;
		addActiveCell(index);
		return true;
	}
	else
	{
		return false;
	}
}

void FastMarching::initTrialCells()
{
	for (size_t j=0; j<m_activeCells.size(); ++j)
	{
		const unsigned& index = m_activeCells[j];
		Cell* aCell = m_theGrid[index];

		assert(aCell != 0);

		for (unsigned i=0; i<CC_FM_NUMBER_OF_NEIGHBOURS; ++i)
		{
			unsigned nIndex = index + m_neighboursIndexShift[i];
			Cell* nCell = m_theGrid[nIndex];
			//if the neighbor exists
			if (nCell)
			{
				//and if it's not already in the TRIAL set
				if (nCell->state == Cell::FAR_CELL)
				{
					nCell->T = m_neighboursDistance[i] * computeTCoefApprox(aCell,nCell);
					addTrialCell(nIndex);
				}
			}
		}
	}
}

void FastMarching::addTrialCell(unsigned index)
{
	m_theGrid[index]->state = Cell::TRIAL_CELL;
	m_trialCells.push_back(index);
}

void FastMarching::addActiveCell(unsigned index)
{
	m_theGrid[index]->state = Cell::ACTIVE_CELL;
	m_activeCells.push_back(index);
}

unsigned FastMarching::getNearestTrialCell()
{
	if (m_trialCells.empty())
		return 0; //0 = error

	//we look for the "TRIAL" cell with the minimum time (T)
	size_t minTCellIndexPos = 0;
	unsigned minTCellIndex = m_trialCells[minTCellIndexPos];
	CCLib::FastMarching::Cell* minTCell = m_theGrid[minTCellIndex];
	assert(minTCell != 0);

	for (size_t i=1; i<m_trialCells.size(); ++i)
	{
		unsigned cellIndex = m_trialCells[i];
		CCLib::FastMarching::Cell* cell = m_theGrid[cellIndex];
		assert(cell != 0);
		
		if (cell->T < minTCell->T)
		{
			minTCellIndexPos = i;
			minTCellIndex = cellIndex;
			minTCell = cell;
		}
	}

	//we remove this cell from the TRIAL set
	m_trialCells[minTCellIndexPos] = m_trialCells.back();
	m_trialCells.pop_back();

	return minTCellIndex;
}
