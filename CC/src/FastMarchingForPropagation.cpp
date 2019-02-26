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

#include "FastMarchingForPropagation.h"

//local
#include "DgmOctree.h"
#include "ReferenceCloud.h"
#include "ScalarFieldTools.h"


using namespace CCLib;

FastMarchingForPropagation::FastMarchingForPropagation()
	: FastMarching()
	, m_jumpCoef(0)							//resistance a l'avancement du front, en fonction de Cell->f (ici, pas de resistance)
	, m_detectionThreshold(Cell::T_INF())	//saut relatif de la valeur d'arrivee qui arrete la propagation (ici, "desactive")
{
}

int FastMarchingForPropagation::init(	GenericCloud* theCloud,
										DgmOctree* theOctree,
										unsigned char level,
										bool constantAcceleration/*=false*/)
{
	assert(theCloud && theOctree);

	int result = initGridWithOctree(theOctree,level);
	if (result < 0)
		return result;

	//on remplit la grille
	DgmOctree::cellCodesContainer cellCodes;
	theOctree->getCellCodes(level,cellCodes,true);

	ReferenceCloud Yk(theOctree->associatedCloud());

	while (!cellCodes.empty())
	{
		if (!theOctree->getPointsInCell(cellCodes.back(),level,&Yk,true))
		{
			//not enough memory?
			return -1;
		}
		
		//on transforme le code de cellule en position
		Tuple3i cellPos;
		theOctree->getCellPos(cellCodes.back(),level,cellPos,true);

		//on renseigne la grille
		unsigned gridPos = pos2index(cellPos);

		PropagationCell* aCell = new PropagationCell;
		aCell->cellCode = cellCodes.back();
		aCell->f = (constantAcceleration ? 1.0f : static_cast<float>(ScalarFieldTools::computeMeanScalarValue(&Yk)));

		m_theGrid[gridPos] = aCell;

		cellCodes.pop_back();
	}

	m_initialized = true;

	return 0;
}

int FastMarchingForPropagation::step()
{
	if (!m_initialized)
		return -1;

	unsigned minTCellIndex = getNearestTrialCell();
	if (minTCellIndex == 0)
	{
		//fl_alert("No more trial cells !");
		return 0;
	}

	Cell* minTCell =  m_theGrid[minTCellIndex];
	assert(minTCell != nullptr);

	//last arrival time
	float lastT = (m_activeCells.empty() ? 0 : m_theGrid[m_activeCells.back()]->T);

	if (minTCell->T-lastT > m_detectionThreshold * m_cellSize)
	{
		//reset();
		return 0;
	}

	assert(minTCell->state != Cell::ACTIVE_CELL);

	if (minTCell->T < Cell::T_INF())
	{
		//we add this cell to the "ACTIVE" set
		addActiveCell(minTCellIndex);

		assert(minTCell->T >= lastT);

		//add its neighbors to the TRIAL set
		Cell* nCell;
		for (unsigned i=0;i<m_numberOfNeighbours;++i)
		{
			//get neighbor cell
			unsigned nIndex = minTCellIndex + m_neighboursIndexShift[i];
			nCell = m_theGrid[nIndex];
			if (nCell)
			{
				//if it' not yet a TRIAL cell
				if (nCell->state == Cell::FAR_CELL)
				{
					nCell->T = computeT(nIndex);
					addTrialCell(nIndex);
				}
				else if (nCell->state == Cell::TRIAL_CELL)
				//otherwise we must update it's arrival time
				{
					float t_old = nCell->T;
					float t_new = computeT(nIndex);

					if (t_new < t_old)
						nCell->T = t_new;
				}
			}
		}
	}
	else
	{
		addIgnoredCell(minTCellIndex);
	}

	return 1;
}

int FastMarchingForPropagation::propagate()
{
	initTrialCells();

	int result = 1;
	while (result > 0)
	{
		result = step();
	}

	return result;
}

bool FastMarchingForPropagation::extractPropagatedPoints(ReferenceCloud* points)
{
	if (!m_initialized || !m_octree || m_gridLevel > DgmOctree::MAX_OCTREE_LEVEL || !points)
		return false;

	points->clear();

	for (unsigned int activeCellIndex : m_activeCells)
	{
		PropagationCell* aCell = static_cast<PropagationCell*>(m_theGrid[activeCellIndex]);
		
		if (!m_octree->getPointsInCell(aCell->cellCode, m_gridLevel, points, true, false))
			return false;
	}

	//raz de la norme du gradient du point, pour qu'il ne soit plus pris en compte par la suite !
	points->placeIteratorAtBeginning();
	for (unsigned k = 0; k < points->size(); ++k)
	{
		points->setCurrentPointScalarValue(NAN_VALUE);
		points->forwardIterator();
	}

	return true;
}

bool FastMarchingForPropagation::setPropagationTimingsAsDistances()
{
	if (!m_initialized || !m_octree || m_gridLevel > DgmOctree::MAX_OCTREE_LEVEL)
		return false;

	ReferenceCloud Yk(m_octree->associatedCloud());

	for (unsigned int activeCellIndex : m_activeCells)
	{
		PropagationCell* aCell = static_cast<PropagationCell*>(m_theGrid[activeCellIndex]);
	
		if (!m_octree->getPointsInCell(aCell->cellCode,m_gridLevel,&Yk,true))
		{
			//not enough memory?
			return false;
		}

		Yk.placeIteratorAtBeginning();
		for (unsigned k=0; k<Yk.size(); ++k)
		{
			Yk.setCurrentPointScalarValue(aCell->T);
			Yk.forwardIterator();
		}
		//Yk.clear(); //useless
	}

	return true;
}

#ifdef _MSC_VER
//Visual 2012 (and previous versions) don't know expm1
#if _MSC_VER <= 1700

// Compute exp(x) - 1 without loss of precision for small values of x.
template <typename T> T expm1(T x)
{
	if (std::abs(x) < 1e-5)
		return x + (x*x)/2;
	else
		return exp(x) - 1;
}

#endif
#endif

float FastMarchingForPropagation::computeTCoefApprox(Cell* currentCell, Cell* neighbourCell) const
{
	PropagationCell* cCell = static_cast<PropagationCell*>(currentCell);
	PropagationCell* nCell = static_cast<PropagationCell*>(neighbourCell);
	return expm1(m_jumpCoef * (cCell->f-nCell->f));
}

void FastMarchingForPropagation::findPeaks()
{
	if (!m_initialized)
		return;

	//on fait bien attention a ne pas initialiser les cellules sur les bords
	for (unsigned k=0; k<m_dz; ++k)
	{
		int pos[3] = { 0, 0, static_cast<int>(k) };

		for (unsigned j=0; j<m_dy; ++j)
		{
			pos[1] = static_cast<int>(j);

			for (unsigned i=0; i<m_dx; ++i)
			{
				pos[0] = static_cast<int>(i);

				unsigned index =  static_cast<unsigned>(pos[0]+1)
								+ static_cast<unsigned>(pos[1]+1) * m_rowSize
								+ static_cast<unsigned>(pos[2]+1) * m_sliceSize;
				
				PropagationCell* theCell = reinterpret_cast<PropagationCell*>(m_theGrid[index]);

				if (theCell)
				{
					bool isMin = true;
					bool isMax = true;

					//theCell->state = ACTIVE_CELL;

					for (int n : m_neighboursIndexShift)
					{
						const PropagationCell* nCell = reinterpret_cast<const PropagationCell*>(m_theGrid[index+n]);
						if (nCell)
						{
							if (nCell->f > theCell->f)
								isMax = false;
							else if (nCell->f < theCell->f)
								isMin = false;
						}
					}

					if (isMin != isMax)
					{
						//if (isMin)
						//	theCell->T = 1.0;
						//else
						//	theCell->T = 2.0;

						if (isMax)
						{
							theCell->T = 0;
							addActiveCell(index);
						}
					}
					//else theCell->T = 0;
				}
			}
		}
	}
}
