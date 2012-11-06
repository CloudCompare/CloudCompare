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

#include "FastMarchingForPropagation.h"

#include "GenericIndexedCloudPersist.h"
#include "DgmOctree.h"
#include "ReferenceCloud.h"
#include "DistanceComputationTools.h"

#include <assert.h>

using namespace CCLib;

FastMarchingForPropagation::FastMarchingForPropagation()
//: FastMarching()
{
	//pour la segmentation
	lastT = 0.0; //dernière valeur d'arrivée
	detectionThreshold = FM_INF; //saut relatif de la valeur d'arrivée qui arrête la propagation (ici, "désactivé")
	jumpCoef = 0.0; //resistance à l'avancement du front, en fonction de Cell->f (ici, pas de resistance)
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

int FastMarchingForPropagation::init(GenericCloud* aList, DgmOctree* _theOctree, uchar level, bool constantAcceleration)
{
	//on commence par créer une grille 3D
	theOctree = _theOctree;
	gridLevel = level;

	int result = initGrid();

	if (result<0) return result;

	//printf("cellSize=%f\n",cellSize);

	//on remplit la grille
	DgmOctree::cellCodesContainer cellCodes;
	theOctree->getCellCodes(gridLevel,cellCodes,true);

	int cellPos[3];

	while (!cellCodes.empty())
	{
		//on transforme le code de cellule en position
		theOctree->getCellPos(cellCodes.back(),gridLevel,cellPos,true);

		//on renseigne la grille
		unsigned gridPos = FM_pos2index(cellPos);

		PropagationCell* aCell = new PropagationCell;
		aCell->state = Cell::FAR_CELL;
		aCell->T = FM_INF;
		aCell->cellCode = cellCodes.back();

		ReferenceCloud* Yk = theOctree->getPointsInCell(cellCodes.back(),gridLevel,true);
		aCell->f = (constantAcceleration ? 1.0 : DistanceComputationTools::computeMeanDist(Yk),false);

		//Yk->clear(); //inutile

		//printf("code %i -> cell(%i,%i,%i) --> %i\n",cellCodes.back(),cellPos[0],cellPos[1],cellPos[2],gridPos);

		theGrid[gridPos] = (Cell*)aCell;

		cellCodes.pop_back();
	}

	initialized = true;

	return 0;
}

int FastMarchingForPropagation::step()
{
	if (!initialized)
	{
		//printf("Not yet initialized !");
		return -1;
	}

	unsigned minTCellIndex = getNearestTrialCell();

	if (minTCellIndex==0)
	{
		//fl_alert("No more trial cells !");
		return 0;
	}

	//printf("minTCellIndex=%i\n",minTCellIndex);

	Cell* minTCell =  theGrid[minTCellIndex];
	assert(minTCell != 0);

	if (minTCell->T-lastT > detectionThreshold*cellSize)
	{
		//endPropagation();
		return 0;
	}

	assert(minTCell->state != Cell::ACTIVE_CELL);

	if (minTCell->T < FM_INF)
	{
		//on rajoute cette cellule au groupe des cellules "ACTIVE"
		minTCell->state = Cell::ACTIVE_CELL;
		activeCells.push_back(minTCellIndex);

		//printf("Cell %i added to ACTIVE\n",minTCellIndex);
		//fprintf(fp,"%f %f %f\n",((PropagationCell*)minTCell)->f,minTCell->T,(minTCell->T-lastT)/cellSize);

		lastT = minTCell->T;

		//on doit rajouter ses voisines au groupe TRIAL
		unsigned nIndex;
		Cell* nCell;
		for (int i=0;i<CC_FM_NUMBER_OF_NEIGHBOURS;++i)
		{
			nIndex = minTCellIndex + neighboursIndexShift[i];
			//pointeur vers la cellule voisine
			nCell = theGrid[nIndex];

			//si elle est définie
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
	double A, B, C;

	A = B = C = 0.0;

	double Tij, Txm, Txp, Tym, Typ, Tzm, Tzp, TijNew;

	Tij = ((PropagationCell*)theGrid[index])->T;

	double Fij = ((PropagationCell*)theGrid[index])->f; //poids

	PropagationCell *nCell;


	nCell = (PropagationCell*)theGrid[index+neighboursIndexShift[3]];
	Txm = (nCell ? nCell->T + neighboursDistance[3]*(exp(jumpCoef*(nCell->f-Fij))-1.0): FM_INF);
	nCell = (PropagationCell*)theGrid[index+neighboursIndexShift[1]];
	Txp = (nCell ? nCell->T + neighboursDistance[1]*(exp(jumpCoef*(nCell->f-Fij))-1.0) : FM_INF);
	nCell = (PropagationCell*)theGrid[index+neighboursIndexShift[0]];
	Tym = (nCell ? nCell->T + neighboursDistance[0]*(exp(jumpCoef*(nCell->f-Fij))-1.0) : FM_INF);
	nCell = (PropagationCell*)theGrid[index+neighboursIndexShift[2]];
	Typ = (nCell ? nCell->T + neighboursDistance[2]*(exp(jumpCoef*(nCell->f-Fij))-1.0) : FM_INF);
	nCell = (PropagationCell*)theGrid[index+neighboursIndexShift[4]];
	Tzm = (nCell ? nCell->T + neighboursDistance[4]*(exp(jumpCoef*(nCell->f-Fij))-1.0) : FM_INF);
	nCell = (PropagationCell*)theGrid[index+neighboursIndexShift[5]];
	Tzp = (nCell ? nCell->T + neighboursDistance[5]*(exp(jumpCoef*(nCell->f-Fij))-1.0) : FM_INF);

	//si (Gij-Gxm < 0) ça doit aller plus vite, i.e. exp(jumpCoef*ANS)>1.0, donc jumpCoef>0

	//Eq. quadratique selon X
	double Tmin = ccMin(Txm,Txp);
	if (Tij>Tmin)
	{
		A += 1.0;
		B += -2.0 * Tmin;
		C += Tmin * Tmin;
	}

	//Eq. quadratique selon Y
	Tmin = ccMin(Tym,Typ);
	if (Tij>Tmin)
	{
		A += 1.0;
		B += -2.0 * Tmin;
		C += Tmin * Tmin;
	}

	//Eq. quadratique selon Z
	Tmin = ccMin(Tzm,Tzp);
	if (Tij>Tmin)
	{
		A += 1.0;
		B += -2.0 * Tmin;
		C += Tmin * Tmin;
	}

	C -=  cellSize*cellSize;

	double delta = B*B - 4.0*A*C;

	// cases when the quadratic equation is singular
	if ((A==0) || (delta < 0.0))
	{
		int candidateIndex;
		float candidateT;
		Tij=FM_INF;

		for(int n=0;n<CC_FM_NUMBER_OF_NEIGHBOURS;n++)
		{
			candidateIndex = index + neighboursIndexShift[n];
			PropagationCell* cCell = (PropagationCell*)theGrid[candidateIndex];
			if (cCell)
			{
				if( (cCell->state==Cell::TRIAL_CELL) || (cCell->state==Cell::ACTIVE_CELL) )
				{
					candidateT = cCell->T + neighboursDistance[n]*exp((cCell->f-(float)Fij)*jumpCoef);
					//si (Gij-cCell->f > 0) ça doit aller plus vite, donc exp(jumpCoef*ANS)>1.0, donc jumpCoef>0

					if(candidateT<Tij) Tij=candidateT;
				}
			}
		}

		assert( Tij<FM_INF );
		if(Tij>=FM_INF)
		{
			//printf("Error in computeT(...): !( Tij<INF )");
			return (float)FM_INF;
		}

		//assert( Tij<10000 );
		return (float)Tij;
	}

	/*
	* Solve the quadratic equation. Note that the new crossing
	* must be GREATER than the average of the active neighbors,
	* since only EARLIER elements are active. Therefore the plus
	* sign is appropriate.
	*/
	TijNew = (-B + sqrt(delta))/(double(2.0*A));

	return (float)TijNew;
}

void FastMarchingForPropagation::initLastT()
{
	Cell* aCell;
	lastT = 0.0;
	for (unsigned i=0; i<activeCells.size(); i++)
	{
		aCell = theGrid[activeCells[i]];
		lastT=ccMax(lastT,aCell->T);
	}
}

int FastMarchingForPropagation::propagate()
{
	int iteration = 0;
	int result=1;

	//fp = fopen("trace_time.txt","wt");

	//initialisation de la liste des "TRIAL" cells
	initTrialCells();

	initLastT();

	while (result>0)
	{
		result = step();

		//printf("%i\n",iteration);

		++iteration;
	}

	//fclose(fp);

	//if (result<0) printf("[FastMarchingForPropagation::propagate] fin (probleme)");

	return result;
}

ReferenceCloud* FastMarchingForPropagation::extractPropagatedPoints()
{
	if (!initialized)
		return 0;

	ReferenceCloud* Zk = new ReferenceCloud(theOctree->associatedCloud());

	unsigned i=0;
	for (;i<activeCells.size();++i)
	{
		PropagationCell* aCell = (PropagationCell*)theGrid[activeCells[i]];
		ReferenceCloud* Yk = theOctree->getPointsInCell(aCell->cellCode,gridLevel,true);

		if (!Zk->reserve(Yk->size())) //not enough memory
		{
			delete Zk;
			return 0;
		}

		Yk->placeIteratorAtBegining();
		for (unsigned k=0;k<Yk->size();++k)
		{
			Zk->addPointIndex(Yk->getCurrentPointGlobalIndex());
			//raz de la norme du gradient du point, pour qu'il ne soit plus pris en compte par la suite !
			Yk->setCurrentPointScalarValue(SEGMENTED_VALUE);
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

	for (unsigned i=0;i<activeCells.size();++i)
	{
		PropagationCell* aCell = (PropagationCell*)theGrid[activeCells[i]];
		ReferenceCloud* Yk = theOctree->getPointsInCell(aCell->cellCode,gridLevel,true);

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

		assert(aCell!=0);

		aCell->state = Cell::FAR_CELL;
		aCell->T = FM_INF;

		trialCells.pop_back();
	}

	lastT=0.0;
}


//rajouter un élément à la structure "untidy priority queue"
void FastMarchingForPropagation::addTrialCell(unsigned index, float T)
{
	trialCells.push_back(index);
}

//récupérer le premier élément de la structure "untidy priority queue"
unsigned FastMarchingForPropagation::getNearestTrialCell() //renvoie 0 si problème
{
	if (trialCells.empty()) return 0;

	//on trouve la cellule de "TRIAL" qui à le T minimum
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

	//on l'enlève de la liste
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

	//on fait bien attention à ne pas initialiser les cellules sur les bords
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
