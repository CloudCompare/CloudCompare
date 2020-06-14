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

#include "FastMarching.h"

//local
#include "DgmOctree.h"


using namespace CCLib;

FastMarching::FastMarching()
	: m_initialized(false)
	, m_dx(0)
	, m_dy(0)
	, m_dz(0)
	, m_rowSize(0)
	, m_sliceSize(0)
	, m_indexShift(0)
	, m_gridSize(0)
	, m_theGrid(nullptr)
	, m_octree(nullptr)
	, m_gridLevel(0)
	, m_cellSize(1.0f)
	, m_minFillIndexes(0, 0, 0)
	, m_numberOfNeighbours(6)
{
	memset(m_neighboursIndexShift, 0, sizeof(int)   * CC_FM_MAX_NUMBER_OF_NEIGHBOURS);
	memset(m_neighboursDistance,   0, sizeof(float) * CC_FM_MAX_NUMBER_OF_NEIGHBOURS);
}

FastMarching::~FastMarching()
{
	if (m_theGrid)
	{
		for (unsigned i = 0; i < m_gridSize; ++i)
		{
			if (m_theGrid[i])
			{
				delete m_theGrid[i];
			}
		}

		delete[] m_theGrid;
		m_theGrid = nullptr;
	}
}

float FastMarching::getTime(Tuple3i& pos, bool absoluteCoordinates) const
{
	unsigned index = 0;

	if (absoluteCoordinates)
	{
		index = pos2index(pos);
	}
	else
	{
		index =	static_cast<unsigned>(pos.x + 1)
			+	static_cast<unsigned>(pos.y + 1) * m_rowSize
			+	static_cast<unsigned>(pos.z + 1) * m_sliceSize;
	}

	assert(m_theGrid[index]);

	return m_theGrid[index]->T;
}

int FastMarching::initGrid(float step, unsigned dim[3])
{
	m_octree = nullptr;
	m_gridLevel = 0;
	m_cellSize = step;
	m_minFillIndexes = Tuple3i(0, 0, 0);

	m_dx = dim[0];
	m_dy = dim[1];
	m_dz = dim[2];

	return initOther();
}

int FastMarching::initGridWithOctree(DgmOctree* octree, unsigned char gridLevel)
{
	if (!octree || gridLevel > DgmOctree::MAX_OCTREE_LEVEL)
		return -2;

	const int* minFillIndexes = octree->getMinFillIndexes(gridLevel);
	const int* maxFillIndexes = octree->getMaxFillIndexes(gridLevel);

	m_octree = octree;
	m_gridLevel = gridLevel;
	m_cellSize = static_cast<float>(octree->getCellSize(gridLevel));
	m_minFillIndexes.x = minFillIndexes[0];
	m_minFillIndexes.y = minFillIndexes[1];
	m_minFillIndexes.z = minFillIndexes[2];

	m_dx = static_cast<unsigned>(maxFillIndexes[0]-minFillIndexes[0]+1);
	m_dy = static_cast<unsigned>(maxFillIndexes[1]-minFillIndexes[1]+1);
	m_dz = static_cast<unsigned>(maxFillIndexes[2]-minFillIndexes[2]+1);

	return initOther();
}

int FastMarching::initOther()
{
	m_rowSize = m_dx + 2;
	m_sliceSize = m_rowSize * (m_dy + 2);
	m_gridSize = m_sliceSize * (m_dz + 2);
	m_indexShift = 1 + m_rowSize + m_sliceSize;

	for (unsigned i=0; i<CC_FM_MAX_NUMBER_OF_NEIGHBOURS; ++i)
	{
		m_neighboursIndexShift[i] =	  c_FastMarchingNeighbourPosShift[i*3  ]
									+ c_FastMarchingNeighbourPosShift[i*3+1] * static_cast<int>(m_rowSize)
									+ c_FastMarchingNeighbourPosShift[i*3+2] * static_cast<int>(m_sliceSize);

		m_neighboursDistance[i] =	sqrt(static_cast<float>(c_FastMarchingNeighbourPosShift[i*3  ] * c_FastMarchingNeighbourPosShift[i*3  ]+
															c_FastMarchingNeighbourPosShift[i*3+1] * c_FastMarchingNeighbourPosShift[i*3+1]+
															c_FastMarchingNeighbourPosShift[i*3+2] * c_FastMarchingNeighbourPosShift[i*3+2]))
									* m_cellSize;
	}

	m_activeCells.resize(0);
	m_trialCells.resize(0);
	m_ignoredCells.resize(0);

	if (!instantiateGrid(m_gridSize))
        return -3;

	return 0;
}

void FastMarching::resetCells(std::vector<unsigned>& list)
{
	for (std::vector<unsigned>::const_iterator it = list.begin(); it != list.end(); ++it)
	{
		if (m_theGrid[*it])
		{
			m_theGrid[*it]->state = Cell::FAR_CELL;
			m_theGrid[*it]->T = Cell::T_INF();
		}
	}
	list.clear();
}

void FastMarching::cleanLastPropagation()
{
	//reset all cells state
	resetCells(m_activeCells);
	resetCells(m_trialCells);
	resetCells(m_ignoredCells);
}

bool FastMarching::setSeedCell(const Tuple3i& pos)
{
	unsigned index = pos2index(pos);

	assert(index < m_gridSize);

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
	for (unsigned int index : m_activeCells)
	{
		Cell* aCell = m_theGrid[index];

		assert(aCell != nullptr);

		for (unsigned i = 0; i < m_numberOfNeighbours; ++i)
		{
			unsigned nIndex = index + m_neighboursIndexShift[i];
			Cell* nCell = m_theGrid[nIndex];
			//if the neighbor exists
			if (nCell)
			{
				//and if it's not already in the TRIAL set
				if (nCell->state == Cell::FAR_CELL)
				{
					nCell->T = m_neighboursDistance[i] * computeTCoefApprox(aCell, nCell);
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

void FastMarching::addIgnoredCell(unsigned index)
{
	m_theGrid[index]->state = Cell::EMPTY_CELL;
	m_ignoredCells.push_back(index);
}

unsigned FastMarching::getNearestTrialCell()
{
	if (m_trialCells.empty())
		return 0; //0 = error

	//we look for the "TRIAL" cell with the minimum time (T)
	std::size_t minTCellIndexPos = 0;
	unsigned minTCellIndex = m_trialCells[minTCellIndexPos];
	CCLib::FastMarching::Cell* minTCell = m_theGrid[minTCellIndex];
	assert(minTCell != nullptr);

	for (std::size_t i=1; i<m_trialCells.size(); ++i)
	{
		unsigned cellIndex = m_trialCells[i];
		CCLib::FastMarching::Cell* cell = m_theGrid[cellIndex];
		assert(cell != nullptr);
		
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

float FastMarching::computeT(unsigned index)
{
	Cell* theCell = m_theGrid[index];
	if (!theCell)
		return Cell::T_INF();

	//arrival time FROM the neighbors
	double T[CC_FM_MAX_NUMBER_OF_NEIGHBOURS] = { 0 };
	{
		for (unsigned n = 0; n < m_numberOfNeighbours; ++n)
		{
			int nIndex = static_cast<int>(index) + m_neighboursIndexShift[n];
			Cell* nCell = m_theGrid[nIndex];
			if (nCell && (nCell->state == Cell::TRIAL_CELL || nCell->state == Cell::ACTIVE_CELL))
			{
				//compute front arrival time
				T[n] = static_cast<double>(nCell->T) + static_cast<double>(m_neighboursDistance[n]) * static_cast<double>(computeTCoefApprox(nCell, theCell));
			}
			else
			{
				//no front yet
				T[n] = static_cast<double>(Cell::T_INF());
			}
		}
	}

	double A = 0;
	double B = 0;
	double C = 0;
	double Tij = static_cast<double>(theCell->T/*Cell::T_INF()*/);

	//Quadratic eq. along X
	{
		//look for the minimum arrival time from +/-X
		double Tmin = static_cast<double>(Cell::T_INF());
		for (unsigned n = 0; n < m_numberOfNeighbours; ++n)
			if (CCLib::c_FastMarchingNeighbourPosShift[n * 3] != 0)
				if (T[n] < Tmin)
					Tmin = T[n];
		if (Tij > Tmin)
		{
			A += 1.0;
			B += -2.0 * Tmin;
			C += Tmin * Tmin;
		}
	}

	//Quadratic eq. along Y
	{
		//look for the minimum arrival time from +/-Y
		double Tmin = static_cast<double>(Cell::T_INF());
		for (unsigned n = 0; n < m_numberOfNeighbours; ++n)
			if (CCLib::c_FastMarchingNeighbourPosShift[n * 3 + 1] != 0)
				if (T[n] < Tmin)
					Tmin = T[n];
		if (Tij > Tmin)
		{
			A += 1.0;
			B += -2.0 * Tmin;
			C += Tmin * Tmin;
		}
	}

	//Quadratic eq. along Z
	{
		//look for the minimum arrival time from +/-Z
		double Tmin = static_cast<double>(Cell::T_INF());
		for (unsigned n = 0; n < m_numberOfNeighbours; ++n)
			if (CCLib::c_FastMarchingNeighbourPosShift[n * 3 + 2] != 0)
				if (T[n] < Tmin)
					Tmin = T[n];
		if (Tij > Tmin)
		{
			A += 1.0;
			B += -2.0 * Tmin;
			C += Tmin * Tmin;
		}
	}

	//DGM: why?
	//C -=  static_cast<double>(m_cellSize*m_cellSize);

	//solve the quadratic equation
	double delta = B*B - 4.0*A*C;

	//cases when the quadratic equation is singular
	if (A == 0 || delta < 0)
	{
		//take the 'earliest' neighbour
		Tij = T[0];
		for (unsigned n = 1; n < m_numberOfNeighbours; n++)
			if (T[n] < Tij)
				Tij = T[n];
	}
	else
	{
		//Note that the new crossing must be GREATER than the average
		//of the active neighbors, since only EARLIER elements are active.
		//Therefore the plus sign is appropriate.
		Tij = (-B + sqrt(delta)) / (2.0*A);
	}

	return static_cast<float>(Tij);
}
