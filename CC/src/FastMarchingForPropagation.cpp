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
	, jumpCoef(0.0f)					//resistance a l'avancement du front, en fonction de Cell->f (ici, pas de resistance)
	, detectionThreshold(Cell::T_INF())	//saut relatif de la valeur d'arrivee qui arrete la propagation (ici, "desactive")
	, lastT(0.0f)						//derniere valeur d'arrivee
{
}

bool FastMarchingForPropagation::instantiateGrid(unsigned size)
{
    assert(theGrid==0);

	PropagationCell** _theGrid = new PropagationCell*[size];
	if (!_theGrid)
        return false;
	memset(_theGrid,0,size*sizeof(PropagationCell*));

	theGrid = (Cell**)_theGrid;

	return true;
}

int FastMarchingForPropagation::init(GenericCloud* theCloud,
										DgmOctree* theOctree,
										uchar level,
										bool constantAcceleration/*=false*/)
{
	int result = initGrid(theOctree,level);
	if (result<0)
		return result;

	//on remplit la grille
	DgmOctree::cellCodesContainer cellCodes;
	theOctree->getCellCodes(level,cellCodes,true);

	while (!cellCodes.empty())
	{
		//on transforme le code de cellule en position
		int cellPos[3];
		theOctree->getCellPos(cellCodes.back(),level,cellPos,true);

		//on renseigne la grille
		unsigned gridPos = FM_pos2index(cellPos);

		PropagationCell* aCell = new PropagationCell;
		aCell->state = Cell::FAR_CELL;
		aCell->T = Cell::T_INF();
		aCell->cellCode = cellCodes.back();

		ReferenceCloud* Yk = theOctree->getPointsInCell(cellCodes.back(),level,true);
		aCell->f = (constantAcceleration ? 1.0f : ScalarFieldTools::computeMeanScalarValue(Yk),false);

		//Yk->clear(); //inutile

		theGrid[gridPos] = (Cell*)aCell;

		cellCodes.pop_back();
	}

	initialized = true;

	return 0;
}

int FastMarchingForPropagation::step()
{
	if (!initialized)
		return -1;

	unsigned minTCellIndex = getNearestTrialCell();

	if (minTCellIndex==0)
	{
		//fl_alert("No more trial cells !");
		return 0;
	}

	Cell* minTCell =  theGrid[minTCellIndex];
	assert(minTCell != 0);

	if (minTCell->T-lastT > detectionThreshold*m_cellSize)
	{
		//endPropagation();
		return 0;
	}

	assert(minTCell->state != Cell::ACTIVE_CELL);

	if (minTCell->T < Cell::T_INF())
	{
		//on rajoute cette cellule au groupe des cellules "ACTIVE"
		minTCell->state = Cell::ACTIVE_CELL;
		activeCells.push_back(minTCellIndex);

		lastT = minTCell->T;

		//on doit rajouter ses voisines au groupe TRIAL
		unsigned nIndex;
		Cell* nCell;
		for (int i=0;i<CC_FM_NUMBER_OF_NEIGHBOURS;++i)
		{
			nIndex = minTCellIndex + neighboursIndexShift[i];
			//pointeur vers la cellule voisine
			nCell = theGrid[nIndex];

			//si elle est definie
			if (nCell)
			{
				//et si elle n'est pas encore dans un groupe, on la rajoute
				if (nCell->state==Cell::FAR_CELL)
				{
					nCell->state = Cell::TRIAL_CELL;
					nCell->T = computeT(nIndex);

					addTrialCell(nIndex,nCell->T);
					//Console::print("Cell %i added to TRIAL\n",nIndex);
				}
				else if (nCell->state == Cell::TRIAL_CELL)
				//sinon, il faut recaculer T
				{
					float t_old = nCell->T;
					float t_new = computeT(nIndex);

					if (t_new<t_old)
						nCell->T = t_new;
				}
			}
		}
	}

	return 1;
}

float FastMarchingForPropagation::computeT(unsigned index)
{
	double Tij = ((PropagationCell*)theGrid[index])->T;
	double Fij = ((PropagationCell*)theGrid[index])->f; //weight

	PropagationCell *nCell = 0;

	nCell = (PropagationCell*)theGrid[index+neighboursIndexShift[3]];
	double Txm = (nCell ? nCell->T + neighboursDistance[3]*(exp(jumpCoef*(nCell->f-Fij))-1.0): Cell::T_INF());
	nCell = (PropagationCell*)theGrid[index+neighboursIndexShift[1]];
	double Txp = (nCell ? nCell->T + neighboursDistance[1]*(exp(jumpCoef*(nCell->f-Fij))-1.0) : Cell::T_INF());
	nCell = (PropagationCell*)theGrid[index+neighboursIndexShift[0]];
	double Tym = (nCell ? nCell->T + neighboursDistance[0]*(exp(jumpCoef*(nCell->f-Fij))-1.0) : Cell::T_INF());
	nCell = (PropagationCell*)theGrid[index+neighboursIndexShift[2]];
	double Typ = (nCell ? nCell->T + neighboursDistance[2]*(exp(jumpCoef*(nCell->f-Fij))-1.0) : Cell::T_INF());
	nCell = (PropagationCell*)theGrid[index+neighboursIndexShift[4]];
	double Tzm = (nCell ? nCell->T + neighboursDistance[4]*(exp(jumpCoef*(nCell->f-Fij))-1.0) : Cell::T_INF());
	nCell = (PropagationCell*)theGrid[index+neighboursIndexShift[5]];
	double Tzp = (nCell ? nCell->T + neighboursDistance[5]*(exp(jumpCoef*(nCell->f-Fij))-1.0) : Cell::T_INF());

	//if (Gij-Gxm < 0) front must propagate faster, i.e. exp(jumpCoef*ANS)>1.0 --> jumpCoef>0

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

	C -=  m_cellSize*m_cellSize;

	double delta = B*B - 4.0*A*C;

	// cases when the quadratic equation is singular
	if (A==0 || delta < 0.0)
	{
		Tij = Cell::T_INF();

		for(int n=0; n<CC_FM_NUMBER_OF_NEIGHBOURS; n++)
		{
			int candidateIndex = index + neighboursIndexShift[n];
			PropagationCell* cCell = (PropagationCell*)theGrid[candidateIndex];
			if (cCell)
			{
				if( (cCell->state==Cell::TRIAL_CELL) || (cCell->state==Cell::ACTIVE_CELL) )
				{
					float candidateT = cCell->T + neighboursDistance[n]*exp((cCell->f-(float)Fij)*jumpCoef);
					//if (Gij-cCell->f > 0) front must propagate faster, i.e. exp(jumpCoef*ANS)>1.0 --> jumpCoef>0

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

void FastMarchingForPropagation::initLastT()
{
	Cell* aCell;
	lastT = 0.0;
	for (unsigned i=0; i<activeCells.size(); i++)
	{
		aCell = theGrid[activeCells[i]];
		lastT=std::max(lastT,aCell->T);
	}
}

int FastMarchingForPropagation::propagate()
{
	int iteration = 0;
	int result = 1;

	//initialisation de la liste des "TRIAL" cells
	initTrialCells();

	initLastT();

	while (result>0)
	{
		result = step();

		++iteration;
	}

	return result;
}

ReferenceCloud* FastMarchingForPropagation::extractPropagatedPoints()
{
	if (!initialized)
		return 0;
	assert(m_octree);

	ReferenceCloud* Zk = new ReferenceCloud(m_octree->associatedCloud());

	for (unsigned i=0; i<activeCells.size(); ++i)
	{
		PropagationCell* aCell = (PropagationCell*)theGrid[activeCells[i]];
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
	if (!initialized)
		return false;
	assert(m_octree);

	for (unsigned i=0;i<activeCells.size();++i)
	{
		PropagationCell* aCell = (PropagationCell*)theGrid[activeCells[i]];
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


void FastMarchingForPropagation::endPropagation()
{
	while (!activeCells.empty())
	{
		PropagationCell* aCell = (PropagationCell*)theGrid[activeCells.back()];
		delete aCell;
		theGrid[activeCells.back()]=0;

		activeCells.pop_back();
	}

	while (!trialCells.empty())
	{
		Cell* aCell = theGrid[trialCells.back()];
		assert(aCell != 0);

		aCell->state = Cell::FAR_CELL;
		aCell->T = Cell::T_INF();

		trialCells.pop_back();
	}

	lastT = 0.0f;
}


//rajouter un element a la structure "untidy priority queue"
void FastMarchingForPropagation::addTrialCell(unsigned index, float T)
{
	trialCells.push_back(index);
}

//recuperer le premier element de la structure "untidy priority queue"
unsigned FastMarchingForPropagation::getNearestTrialCell() //renvoie 0 si probleme
{
	if (trialCells.empty()) return 0;

	//on trouve la cellule de "TRIAL" qui a le T minimum
	std::vector<unsigned>::const_iterator p = trialCells.begin();

	int i=0,k=0;
	unsigned minTCellIndex = *p;
	Cell* minTCell = theGrid[minTCellIndex];
	assert(minTCell != 0);
	++p;
	++i;

	while (p != trialCells.end())
	{
		assert(theGrid[*p] != 0);
		if (theGrid[*p]->T <minTCell->T)
		{
			minTCellIndex = *p;
			minTCell = theGrid[minTCellIndex];
			k=i;
		}
		++p;
		++i;
	}

	//on l'enleve de la liste
	trialCells[k]=trialCells[trialCells.size()-1];
	trialCells.pop_back();

	return minTCellIndex;
}

float FastMarchingForPropagation::computeTCoefApprox(Cell* currentCell, Cell* neighbourCell)
{
	return exp(jumpCoef*(((PropagationCell*)currentCell)->f-((PropagationCell*)neighbourCell)->f));
}

//pour le watershed
void FastMarchingForPropagation::findPeaks()
{
	if (!initialized) return;

	//on remplit la grille
	unsigned i,j,k;
	int n;

	int neighbours3DIndexShift[CC_FM_NUMBER_OF_3D_NEIGHBOURS];
	//calculs de decalages pour voisnages
	for (n=0;n<CC_FM_NUMBER_OF_3D_NEIGHBOURS;++n)
	{
		neighbours3DIndexShift[n] = neighbours3DPosShift[n*3]+
									neighbours3DPosShift[n*3+1]*int(decY)+
									neighbours3DPosShift[n*3+2]*int(decZ);
	}

	//on fait bien attention a ne pas initialiser les cellules sur les bords
	int pos[3];
	unsigned index;
	for (k=0;k<dz;++k)
	{
		pos[2] = k;
		for (j=0;j<dy;++j)
		{
			pos[1] = j;
			for (i=0;i<dx;++i)
			{
				pos[0] = i;

				index = unsigned(pos[0]+1)+unsigned(pos[1]+1)*decY+unsigned(pos[2]+1)*decZ;
				PropagationCell* theCell = (PropagationCell*)theGrid[index];

				bool isMin=true;
				bool isMax=true;

				if (theCell)
				{
					//theCell->state = ACTIVE_CELL;

					for (n=0;n<CC_FM_NUMBER_OF_3D_NEIGHBOURS;++n)
					{
						PropagationCell* nCell = (PropagationCell*)theGrid[index+neighbours3DIndexShift[n]];
						if (nCell)
						{
							if (nCell->f > theCell->f)
								isMax = false;
							else if (nCell->f < theCell->f)
								isMin = false;
						}
						//theGrid[index];
					}

					if (isMin != isMax)
					{
						/*if (isMin) theCell->T = 1.0;
						else theCell->T = 2.0;*/

						if (isMax)
						{
							theCell->state = Cell::ACTIVE_CELL;
							theCell->T = 0.0;
							activeCells.push_back(index);
						}
					}
					//else theCell->T=0.0;
				}
			}
		}
	}
}
