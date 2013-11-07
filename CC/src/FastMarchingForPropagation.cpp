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

#include "FastMarchingForPropagation.h"

//local
#include "GenericIndexedCloudPersist.h"
#include "DgmOctree.h"
#include "ReferenceCloud.h"
#include "ScalarFieldTools.h"

//system
#include <string.h>
#include <assert.h>

using namespace CCLib;

FastMarchingForPropagation::FastMarchingForPropagation()
	: FastMarching()
	, m_jumpCoef(0)							//resistance a l'avancement du front, en fonction de Cell->f (ici, pas de resistance)
	, m_detectionThreshold(Cell::T_INF())	//saut relatif de la valeur d'arrivee qui arrete la propagation (ici, "desactive")
{
}

int FastMarchingForPropagation::init(	GenericCloud* theCloud,
										DgmOctree* theOctree,
										uchar level,
										bool constantAcceleration/*=false*/)
{
	int result = initGridWithOctree(theOctree,level);
	if (result < 0)
		return result;

	//on remplit la grille
	DgmOctree::cellCodesContainer cellCodes;
	theOctree->getCellCodes(level,cellCodes,true);

	while (!cellCodes.empty())
	{
		ReferenceCloud* Yk = theOctree->getPointsInCell(cellCodes.back(),level,true);
		if (Yk)
		{
			//on transforme le code de cellule en position
			int cellPos[3];
			theOctree->getCellPos(cellCodes.back(),level,cellPos,true);

			//on renseigne la grille
			unsigned gridPos = FM_pos2index(cellPos);

			PropagationCell* aCell = new PropagationCell;
			aCell->cellCode = cellCodes.back();
			aCell->f = (constantAcceleration ? 1.0f : ScalarFieldTools::computeMeanScalarValue(Yk));

			m_theGrid[gridPos] = aCell;
		}

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
	assert(minTCell != 0);

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
		//on rajoute cette cellule au groupe des cellules "ACTIVE"
		minTCell->state = Cell::ACTIVE_CELL;
		m_activeCells.push_back(minTCellIndex);

		assert(minTCell->T >= lastT);

		//on doit rajouter ses voisines au groupe TRIAL
		unsigned nIndex;
		Cell* nCell;
		for (int i=0;i<CC_FM_NUMBER_OF_NEIGHBOURS;++i)
		{
			nIndex = minTCellIndex + m_neighboursIndexShift[i];
			//pointeur vers la cellule voisine
			nCell = m_theGrid[nIndex];

			//si elle est definie
			if (nCell)
			{
				//et si elle n'est pas encore dans un groupe, on la rajoute
				if (nCell->state == Cell::FAR_CELL)
				{
					nCell->state = Cell::TRIAL_CELL;
					nCell->T = computeT(nIndex);

					addTrialCell(nIndex);
				}
				else if (nCell->state == Cell::TRIAL_CELL)
				//sinon, il faut recaculer T
				{
					float t_old = nCell->T;
					float t_new = computeT(nIndex);

					if (t_new < t_old)
						nCell->T = t_new;
				}
			}
		}
	}

	return 1;
}

float FastMarchingForPropagation::computeT(unsigned index)
{
	double Tij = static_cast<PropagationCell*>(m_theGrid[index])->T;
	double Fij = static_cast<PropagationCell*>(m_theGrid[index])->f; //weight

	PropagationCell *nCell = 0;

	nCell = (PropagationCell*)m_theGrid[index+m_neighboursIndexShift[3]];
	double Txm = (nCell ? nCell->T + m_neighboursDistance[3]*(exp(m_jumpCoef*(nCell->f-Fij))-1.0): Cell::T_INF());
	nCell = (PropagationCell*)m_theGrid[index+m_neighboursIndexShift[1]];
	double Txp = (nCell ? nCell->T + m_neighboursDistance[1]*(exp(m_jumpCoef*(nCell->f-Fij))-1.0) : Cell::T_INF());
	nCell = (PropagationCell*)m_theGrid[index+m_neighboursIndexShift[0]];
	double Tym = (nCell ? nCell->T + m_neighboursDistance[0]*(exp(m_jumpCoef*(nCell->f-Fij))-1.0) : Cell::T_INF());
	nCell = (PropagationCell*)m_theGrid[index+m_neighboursIndexShift[2]];
	double Typ = (nCell ? nCell->T + m_neighboursDistance[2]*(exp(m_jumpCoef*(nCell->f-Fij))-1.0) : Cell::T_INF());
	nCell = (PropagationCell*)m_theGrid[index+m_neighboursIndexShift[4]];
	double Tzm = (nCell ? nCell->T + m_neighboursDistance[4]*(exp(m_jumpCoef*(nCell->f-Fij))-1.0) : Cell::T_INF());
	nCell = (PropagationCell*)m_theGrid[index+m_neighboursIndexShift[5]];
	double Tzp = (nCell ? nCell->T + m_neighboursDistance[5]*(exp(m_jumpCoef*(nCell->f-Fij))-1.0) : Cell::T_INF());

	//if (Gij-Gxm < 0) front must propagate faster, i.e. exp(m_jumpCoef*ANS)>1.0 --> m_jumpCoef>0

	double A=0.0, B=0.0, C=0.0;

	//Quadratic eq. along X
	double Tmin = std::min(Txm,Txp);
	if (Tij>Tmin)
	{
		A += 1.0;
		B += -2.0 * Tmin;
		C += Tmin * Tmin;
	}

	//Quadratic eq. along Y
	Tmin = std::min(Tym,Typ);
	if (Tij>Tmin)
	{
		A += 1.0;
		B += -2.0 * Tmin;
		C += Tmin * Tmin;
	}

	//Quadratic eq. along Z
	Tmin = std::min(Tzm,Tzp);
	if (Tij>Tmin)
	{
		A += 1.0;
		B += -2.0 * Tmin;
		C += Tmin * Tmin;
	}

	C -=  m_cellSize * m_cellSize;

	double delta = B*B - 4.0*A*C;

	// cases when the quadratic equation is singular
	if (A == 0 || delta < 0.0)
	{
		Tij = Cell::T_INF();

		for(int n=0; n<CC_FM_NUMBER_OF_NEIGHBOURS; n++)
		{
			int candidateIndex = index + m_neighboursIndexShift[n];
			PropagationCell* cCell = (PropagationCell*)m_theGrid[candidateIndex];
			if (cCell)
			{
				if( (cCell->state == Cell::TRIAL_CELL) || (cCell->state == Cell::ACTIVE_CELL) )
				{
					float candidateT = cCell->T + m_neighboursDistance[n]*exp((cCell->f-(float)Fij)*m_jumpCoef);
					//if (Gij-cCell->f > 0) front must propagate faster, i.e. exp(m_jumpCoef*ANS)>1.0 --> m_jumpCoef>0

					if(candidateT<Tij)
						Tij=candidateT;
				}
			}
		}

		assert( Tij < Cell::T_INF() );
		if(Tij >= Cell::T_INF())
			return Cell::T_INF();

		//assert( Tij<10000 );
		return (float)Tij;
	}

	//Solve the quadratic equation. Note that the new crossing
	//must be GREATER than the average of the active neighbors,
	//since only EARLIER elements are active. Therefore the plus
	//sign is appropriate.
	double TijNew = (-B + sqrt(delta))/(2.0*A);

	return (float)TijNew;
}

int FastMarchingForPropagation::propagate()
{
	int iteration = 0;
	int result = 1;

	//initialisation de la liste des "TRIAL" cells
	initTrialCells();

	while (result>0)
	{
		result = step();

		++iteration;
	}

	return result;
}

ReferenceCloud* FastMarchingForPropagation::extractPropagatedPoints()
{
	if (!m_initialized || !m_octree || m_gridLevel > DgmOctree::MAX_OCTREE_LEVEL)
		return 0;

	ReferenceCloud* Zk = new ReferenceCloud(m_octree->associatedCloud());

	for (unsigned i=0; i<m_activeCells.size(); ++i)
	{
		PropagationCell* aCell = (PropagationCell*)m_theGrid[m_activeCells[i]];
		ReferenceCloud* Yk = m_octree->getPointsInCell(aCell->cellCode,m_gridLevel,true);

		if (!Zk->reserve(Yk->size())) //not enough memory
		{
			delete Zk;
			return 0;
		}

		Yk->placeIteratorAtBegining();
		for (unsigned k=0;k<Yk->size();++k)
		{
			Zk->addPointIndex(Yk->getCurrentPointGlobalIndex()); //can't fail (see above)
			//raz de la norme du gradient du point, pour qu'il ne soit plus pris en compte par la suite !
			Yk->setCurrentPointScalarValue(NAN_VALUE);
			Yk->forwardIterator();
		}

		//Yk->clear(); //inutile
	}

	return Zk;
}

bool FastMarchingForPropagation::setPropagationTimingsAsDistances()
{
	if (!m_initialized || !m_octree || m_gridLevel > DgmOctree::MAX_OCTREE_LEVEL)
		return false;

	for (unsigned i=0;i<m_activeCells.size();++i)
	{
		PropagationCell* aCell = (PropagationCell*)m_theGrid[m_activeCells[i]];
		ReferenceCloud* Yk = m_octree->getPointsInCell(aCell->cellCode,m_gridLevel,true);

		Yk->placeIteratorAtBegining();
		for (unsigned k=0;k<Yk->size();++k)
		{
			Yk->setCurrentPointScalarValue(aCell->T);
			Yk->forwardIterator();
		}
		//Yk->clear(); //inutile
	}

	return true;
}

float FastMarchingForPropagation::computeTCoefApprox(Cell* currentCell, Cell* neighbourCell) const
{
	return exp(m_jumpCoef*(((PropagationCell*)currentCell)->f-((PropagationCell*)neighbourCell)->f));
}

#define CC_FM_NUMBER_OF_3D_NEIGHBOURS 26

//! 26-connexity neighbouring cells positions (common edges)
const int neighbours3DPosShift[] = {-1,-1,-1,
									-1,-1, 0,
									-1,-1, 1,
									-1, 0,-1,
									-1, 0, 0,
									-1, 0, 1,
									-1, 1,-1,
									-1, 1, 0,
									-1, 1, 1,
									 0,-1,-1,
									 0,-1, 0,
									 0,-1, 1,
									 0, 0,-1,
									 0, 0, 1,
									 0, 1,-1,
									 0, 1, 0,
									 0, 1, 1,
									 1,-1,-1,
									 1,-1, 0,
									 1,-1, 1,
									 1, 0,-1,
									 1, 0, 0,
									 1, 0, 1,
									 1, 1,-1,
									 1, 1, 0,
									 1, 1, 1 };

void FastMarchingForPropagation::findPeaks()
{
	if (!m_initialized)
		return;

	//pre-compute shifts
	int neighbours3DIndexShift[CC_FM_NUMBER_OF_3D_NEIGHBOURS];
	//calculs de decalages pour voisnages
	for (unsigned n=0; n<CC_FM_NUMBER_OF_3D_NEIGHBOURS; ++n)
	{
		neighbours3DIndexShift[n] =	  neighbours3DPosShift[n*3]
									+ neighbours3DPosShift[n*3+1] * static_cast<int>(m_decY)
									+ neighbours3DPosShift[n*3+2] * static_cast<int>(m_decZ);
	}

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
								+ static_cast<unsigned>(pos[1]+1) * m_decY
								+ static_cast<unsigned>(pos[2]+1) * m_decZ;
				
				PropagationCell* theCell = reinterpret_cast<PropagationCell*>(m_theGrid[index]);

				bool isMin = true;
				bool isMax = true;

				if (theCell)
				{
					//theCell->state = ACTIVE_CELL;

					for (unsigned n=0; n<CC_FM_NUMBER_OF_3D_NEIGHBOURS; ++n)
					{
						const PropagationCell* nCell = reinterpret_cast<const PropagationCell*>(m_theGrid[index+neighbours3DIndexShift[n]]);
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
							theCell->state = Cell::ACTIVE_CELL;
							theCell->T = 0;
							m_activeCells.push_back(index);
						}
					}
					//else theCell->T = 0;
				}
			}
		}
	}
}
