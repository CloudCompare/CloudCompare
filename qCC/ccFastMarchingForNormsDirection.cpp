//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccFastMarchingForNormsDirection.h"

//qCC_db
#include <ccNormalVectors.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccOctree.h>
#include <ccLog.h>

#include <assert.h>

ccFastMarchingForNormsDirection::ccFastMarchingForNormsDirection()
	: CCLib::FastMarching()
	, lastT(0.0f)
{
}

bool ccFastMarchingForNormsDirection::instantiateGrid(unsigned size)
{
    assert(theGrid==0);

	DirectionCell** _theGrid = new DirectionCell*[size];
	if (!_theGrid)
        return false;
	memset(_theGrid,0,size*sizeof(DirectionCell*));

	theGrid = (CCLib::FastMarching::Cell**)_theGrid;

	return true;
}

int ccFastMarchingForNormsDirection::init(ccGenericPointCloud* aList,
                                            NormsIndexesTableType* theNorms,
                                            CCLib::DgmOctree* theOctree,
                                            uchar level)
{
	int result = initGrid(theOctree, level);
	if (result<0)
		return result;

	//on remplit la grille
	CCLib::DgmOctree::cellCodesContainer cellCodes;
	theOctree->getCellCodes(level,cellCodes,true);

	while (!cellCodes.empty())
	{
		//on transforme le code de cellule en position
		int cellPos[3];
		theOctree->getCellPos(cellCodes.back(),level,cellPos,true);

		//on renseigne la grille
		unsigned gridPos = FM_pos2index(cellPos);

		DirectionCell* aCell = new DirectionCell;
		aCell->state = CCLib::FastMarching::Cell::FAR_CELL;
		aCell->T = Cell::T_INF();
		aCell->processed = false;
		aCell->v = 0;
		aCell->cellCode = cellCodes.back();

		CCLib::ReferenceCloud* Yk = theOctree->getPointsInCell(cellCodes.back(),level,true);
		if (Yk)
		{
			ccOctree::ComputeRobustAverageNorm(Yk,aList,aCell->N.u);
			//Yk->clear(); //inutile
			theGrid[gridPos] = (CCLib::FastMarching::Cell*)aCell;
		}

		cellCodes.pop_back();
	}

	initialized = true;

	return 0;
}

int ccFastMarchingForNormsDirection::step()
{
	if (!initialized)
		return -1;

	unsigned minTCellIndex = getNearestTrialCell();

	if (minTCellIndex == 0)
		return 0;

	//printf("minTCellIndex=%i\n",minTCellIndex);

	CCLib::FastMarching::Cell* minTCell =  theGrid[minTCellIndex];
	assert(minTCell != NULL);

	assert(minTCell->state != CCLib::FastMarching::Cell::ACTIVE_CELL);

	if (minTCell->T < Cell::T_INF())
	{
		//on rajoute cette cellule au groupe des cellules "ACTIVE"
		minTCell->state = CCLib::FastMarching::Cell::ACTIVE_CELL;
		activeCells.push_back(minTCellIndex);

		lastT = minTCell->T;

		//on doit rajouter ses voisines au groupe TRIAL
		for (int i=0;i<CC_FM_NUMBER_OF_NEIGHBOURS;++i)
		{
			unsigned nIndex = minTCellIndex + neighboursIndexShift[i];
			//pointeur vers la cellule voisine
			CCLib::FastMarching::Cell* nCell = theGrid[nIndex];

			//si elle est definie
			if (nCell)
			{
				//et si elle n'est pas encore dans un groupe, on la rajoute
				if (nCell->state==CCLib::FastMarching::Cell::FAR_CELL)
				{
					nCell->state = CCLib::FastMarching::Cell::TRIAL_CELL;
					nCell->T = computeT(nIndex);

					addTrialCell(nIndex,nCell->T);
					//Console::print("Cell %i added to TRIAL\n",nIndex);
				}
				else if (nCell->state == CCLib::FastMarching::Cell::TRIAL_CELL)
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

float ccFastMarchingForNormsDirection::computeT(unsigned index)
{
	DirectionCell* theCell = (DirectionCell*)theGrid[index];

	CCVector3& N = theCell->N;

	const PointCoordinateType c_dotProdThreshold = (PointCoordinateType)0.25;
	PointCoordinateType directionAgreements[6] = {0,0,0,0,0,0};
	
	//arrival time of the 6 neighbors
	double	Txm=Cell::T_INF(),
			Txp=Cell::T_INF(),
			Tym=Cell::T_INF(),
			Typ=Cell::T_INF(),
			Tzm=Cell::T_INF(),
			Tzp=Cell::T_INF();

	DirectionCell* nCell = (DirectionCell*)theGrid[index+neighboursIndexShift[1]];
	if (nCell)
	{
		Txp = nCell->T + neighboursDistance[1];
		if (nCell->processed)
            directionAgreements[1] = nCell->N.dot(N)*nCell->v;
		if (nCell->processed)
		{
         PointCoordinateType ps = nCell->N.dot(N);
			if (fabs(ps) >= c_dotProdThreshold)
                directionAgreements[1] = ps*nCell->v;
		}
	}
	nCell = (DirectionCell*)theGrid[index+neighboursIndexShift[3]];
	if (nCell)
	{
		Txm = nCell->T + neighboursDistance[3];
		if (nCell->processed)
		{
         PointCoordinateType ps = nCell->N.dot(N);
			if (fabs(ps) >= c_dotProdThreshold)
				directionAgreements[3] = ps*nCell->v;
		}
	}
	nCell = (DirectionCell*)theGrid[index+neighboursIndexShift[0]];
	if (nCell)
	{
		Tym = nCell->T + neighboursDistance[0];
		if (nCell->processed)
		{
         PointCoordinateType ps = nCell->N.dot(N);
			if (fabs(ps) >= c_dotProdThreshold)
				directionAgreements[0] = ps*nCell->v;
		}
	}
	nCell = (DirectionCell*)theGrid[index+neighboursIndexShift[2]];
	if (nCell)
	{
		Typ = nCell->T + neighboursDistance[2];
		if (nCell->processed)
		{
         PointCoordinateType ps = nCell->N.dot(N);
			if (fabs(ps) >= c_dotProdThreshold)
				directionAgreements[2] = ps*nCell->v;
		}
	}
	nCell = (DirectionCell*)theGrid[index+neighboursIndexShift[4]];
	if (nCell)
	{
		Tzm = nCell->T + neighboursDistance[4];
		if (nCell->processed)
		{
         PointCoordinateType ps = nCell->N.dot(N);
			if (fabs(ps) >= c_dotProdThreshold)
				directionAgreements[4] = ps*nCell->v;
		}
	}
	nCell = (DirectionCell*)theGrid[index+neighboursIndexShift[5]];
	if (nCell)
	{
		Tzp = nCell->T + neighboursDistance[5];
		if (nCell->processed)
		{
         PointCoordinateType ps = nCell->N.dot(N);
			if (fabs(ps) >= c_dotProdThreshold)
				directionAgreements[5] = ps*nCell->v;
		}
	}

	double A=0.0, B=0.0, C=0.0;

	double Tij = theCell->T;

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

	C -=  (double)(m_cellSize*m_cellSize);

	double delta = B*B - 4.0*A*C;

	// cases when the quadratic equation is singular
	if (A==0 || delta < 0.0)
	{
		Tij = Cell::T_INF();

		for(int n=0;n<CC_FM_NUMBER_OF_NEIGHBOURS;n++)
		{
			int candidateIndex = index + neighboursIndexShift[n];
			DirectionCell* cCell = (DirectionCell*)theGrid[candidateIndex];
			if (cCell)
			{
				if( (cCell->state==CCLib::FastMarching::Cell::TRIAL_CELL) || (cCell->state==CCLib::FastMarching::Cell::ACTIVE_CELL) )
				{
					float candidateT = cCell->T + neighboursDistance[n];
					if(candidateT < Tij)
						Tij = candidateT;
				}
			}
		}

		/*assert( Tij<Cell::T_INF() );
		if(Tij>=Cell::T_INF())
		{
			//printf("Error in computeT(...): !( Tij<INF )");
			return (float)Cell::T_INF();
		}*/

		//assert( Tij<10000 );
		//return (float)Tij;
	}
	else
	{
		/*
		* Solve the quadratic equation. Note that the new crossing
		* must be GREATER than the average of the active neighbors,
		* since only EARLIER elements are active. Therefore the plus
		* sign is appropriate.
		*/
		Tij = (-B + sqrt(delta))/(double(2.0*A));
	}

	if (!theCell->processed)
	{
		//on depouille le vote
		uchar i=0,j=0;
		float positive=0.0,negative=0.0;
		for (uchar k=0;k<6;++k)
		{
			if (directionAgreements[k]<-0.5)
			{
				negative-=directionAgreements[k];
				++i;
				//negative+=directionAgreements[k]*directionAgreements[k];
			}
			else if (directionAgreements[k]>0.5)
			{
				positive+=directionAgreements[k];
				++j;
				//positive+=directionAgreements[k]*directionAgreements[k];
			}
		}

		//si le vote penche plutot pour l'autre sens
		if (std::max(negative,positive)>0.7)
		{
			if (negative>positive)
			{
				N *= -1.0;
				theCell->v = float(i)/float(i+j);
				theCell->processed = true;
			}
			else //if (positive>0.0)
			{
				theCell->v = float(j)/float(i+j);
				theCell->processed = true;
			}
		}
		else return Cell::T_INF();
		//*/
		//theCell->v = negative-positive;
		//theCell->v = fabs(positive-negative)/float(i);
	}

	return (float)Tij;
}

void ccFastMarchingForNormsDirection::initLastT()
{
	lastT = 0.0f;
	for (size_t i=0; i<activeCells.size(); i++)
	{
		CCLib::FastMarching::Cell* aCell = theGrid[activeCells[i]];
		lastT = std::max(lastT,aCell->T);
	}
}

int ccFastMarchingForNormsDirection::propagate()
{
	int iteration = 0;
	int result=1;

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

int ccFastMarchingForNormsDirection::updateResolvedTable(ccGenericPointCloud* theCloud,
                                                            GenericChunkedArray<1,uchar> &resolved,
                                                            NormsIndexesTableType* theNorms)
{
	if (!initialized)
		return -1;
	assert(m_octree);

	int count=0;
	for (unsigned i=0;i<activeCells.size();++i)
	{
		DirectionCell* aCell = (DirectionCell*)theGrid[activeCells[i]];
		CCLib::ReferenceCloud* Yk = m_octree->getPointsInCell(aCell->cellCode,m_gridLevel,true);
		if (!Yk)
			continue;

		for (unsigned k=0; k<Yk->size(); ++k)
		{
			unsigned index = Yk->getPointGlobalIndex(k);
			resolved.setValue(index,1); //resolvedValue=1

			const normsType& norm = theNorms->getValue(index);
			if (CCVector3::vdot(ccNormalVectors::GetNormal(norm),aCell->N.u)<0.0)
			{
				const PointCoordinateType* N = ccNormalVectors::GetNormal(norm);
				PointCoordinateType newN[3]= { -N[0], -N[1], -N[2] };
				theNorms->setValue(index,ccNormalVectors::GetNormIndex(newN));
			}

			//norm = NormalVectors::getNormIndex(aCell->N);
			//theNorms->setValue(index,&norm);

			theCloud->setPointScalarValue(index,aCell->T);
			//theCloud->setPointScalarValue(index,aCell->v);
			++count;
		}
	}

	return count;
}


void ccFastMarchingForNormsDirection::endPropagation()
{
	while (!activeCells.empty())
	{
		DirectionCell* aCell = (DirectionCell*)theGrid[activeCells.back()];
		delete aCell;
		theGrid[activeCells.back()]=NULL;

		activeCells.pop_back();
	}

	while (!trialCells.empty())
	{
		CCLib::FastMarching::Cell* aCell = theGrid[trialCells.back()];

		assert(aCell!=NULL);

		aCell->state = CCLib::FastMarching::Cell::FAR_CELL;
		aCell->T = Cell::T_INF();

		trialCells.pop_back();
	}

	lastT = 0.0f;
}


//rajouter un element a la structure "untidy priority queue"
void ccFastMarchingForNormsDirection::addTrialCell(unsigned index, float T)
{
	trialCells.push_back(index);
}

//recuperer le premier element de la structure "untidy priority queue"
unsigned ccFastMarchingForNormsDirection::getNearestTrialCell() //renvoie 0 si probleme
{
	if (trialCells.empty()) return 0;

	//on trouve la cellule de "TRIAL" qui a le T minimum
	std::vector<unsigned>::const_iterator p = trialCells.begin();

	int i=0,k=0;
	unsigned minTCellIndex = *p;
	CCLib::FastMarching::Cell* minTCell = theGrid[minTCellIndex];
	assert(minTCell != NULL);
	++p;
	++i;

	while (p != trialCells.end())
	{
		assert(theGrid[*p] != NULL);
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

void ccFastMarchingForNormsDirection::initTrialCells()
{
	DirectionCell *aCell,*nCell;
	unsigned i,j,index,nIndex;

	for (j=0;j<activeCells.size();++j)
	{
		index = activeCells[j];
		aCell = (DirectionCell*)theGrid[index];

		assert(aCell != NULL);
		aCell->v = 1.0;

		for (i=0;i<CC_FM_NUMBER_OF_NEIGHBOURS;++i)
		{
			nIndex = index + neighboursIndexShift[i];
			//pointeur vers la cellule voisine
			nCell = (DirectionCell*)theGrid[nIndex];

			//si elle est definie
			if (nCell)
			{
				//et si elle n'est pas encore dans un groupe
				if (nCell->state==CCLib::FastMarching::Cell::FAR_CELL)
				{
					nCell->state = CCLib::FastMarching::Cell::TRIAL_CELL;
					nCell->T = neighboursDistance[i]*computeTCoefApprox(aCell,nCell);

					//on doit s'occuper de leur normales !!!
					PointCoordinateType ps = aCell->N.dot(nCell->N);
					if (ps < 0)
						nCell->N *= (PointCoordinateType)-1.0;

					nCell->v = (ScalarType)1.0;
					nCell->processed = true;

					addTrialCell(nIndex,nCell->T);

					//Console::print("cell %i added to TRIAL\n",nIndex);
				}
			}
		}
	}
}

int ccFastMarchingForNormsDirection::ResolveNormsDirectionByFrontPropagation(ccPointCloud* theCloud,
                                                                                NormsIndexesTableType* theNorms,
                                                                                uchar octreeLevel,
                                                                                CCLib::GenericProgressCallback* progressCb,
                                                                                CCLib::DgmOctree* _theOctree)
{
    assert(theCloud);

	int i,numberOfPoints = theCloud->size();
	if (numberOfPoints<1)
        return -2;

	//on calcule l'octree si besoin
	CCLib::DgmOctree* theOctree = _theOctree;
	if (!theOctree)
	{
		theOctree = new CCLib::DgmOctree(theCloud);
		if (theOctree->build(progressCb)<1)
		{
			delete theOctree;
			return -3;
		}
	}

	//temporary SF
	int oldSfIdx = theCloud->getCurrentDisplayedScalarFieldIndex();
	int sfIdx = theCloud->getScalarFieldIndexByName("FM_Propagation");
	if (sfIdx<0)
		sfIdx = theCloud->addScalarField("FM_Propagation");
	if (sfIdx>=0)
		theCloud->setCurrentScalarField(sfIdx);
	else
	{
		ccLog::Warning("[ccFastMarchingForNormsDirection] Couldn't create temporary scalar field! Not enough memory?");
		if (!_theOctree)
			delete theOctree;
		return -5;
	}
	theCloud->enableScalarField();

	//vecteur indiquant si le point a ete traite
	GenericChunkedArray<1,uchar>* resolved = new GenericChunkedArray<1,uchar>();
	if (!resolved->resize(numberOfPoints,true,0)) //defaultResolvedValue=0
	{
		ccLog::Warning("[ccFastMarchingForNormsDirection] Not enough memory!");
		theCloud->deleteScalarField(sfIdx);
		theCloud->setCurrentScalarField(oldSfIdx);
		if (!_theOctree)
			delete theOctree;
		resolved->release();
		return -5;
	}

	//on va faire la propagation avec l'algorithme de Fast Marching
	ccFastMarchingForNormsDirection* fm = new ccFastMarchingForNormsDirection();

	int result = fm->init(theCloud,theNorms,theOctree,octreeLevel);
	if (result<0)
	{
		ccLog::Error("[ccFastMarchingForNormsDirection] Something went wrong during initialization ...\n");
		theCloud->deleteScalarField(sfIdx);
		theCloud->setCurrentScalarField(oldSfIdx);
		resolved->release();
		if (!_theOctree)
			delete theOctree;
		delete fm;
		return -4;
	}

	int resolvedPoints=0;
	if (progressCb)
	{
		progressCb->reset();
		progressCb->setMethodTitle("Norms direction");
		char buffer[256];
		sprintf(buffer,"Octree level: %i\nNumber of points: %i",octreeLevel,numberOfPoints);
		progressCb->setInfo(buffer);
		progressCb->start();
	}

	int octreeLength = (1<<octreeLevel)-1;

	while (true)
	{
		//on cherche un point non encore traite
		resolved->placeIteratorAtBegining();
		for (i=0;i<numberOfPoints;++i)
		{
			if (resolved->getCurrentValue()==0)
				break;
			resolved->forwardIterator();
		}

		//si tous les points ont ete traites, on peut arreter !
		if (i==numberOfPoints)
			break;

		//on lance la propagation a partir du point trouve
		const CCVector3 *thePoint = theCloud->getPoint(i);

		int pos[3];
		theOctree->getTheCellPosWhichIncludesThePoint(thePoint,pos,octreeLevel);
		//clipping (important !)
		pos[0] = std::min(octreeLength,pos[0]);
		pos[1] = std::min(octreeLength,pos[1]);
		pos[2] = std::min(octreeLength,pos[2]);
		fm->setSeedCell(pos);

		int result = fm->propagate();

		//si la propagation s'est bien passee
		if (result>=0)
		{
			int count = fm->updateResolvedTable(theCloud,*resolved,theNorms);

			if (progressCb)
			{
				if (count>=0)
				{
					resolvedPoints += count;
					progressCb->update(float(resolvedPoints)/float(numberOfPoints)*100.0);
				}
			}

			fm->endPropagation();
		}
	}

	if (progressCb)
		progressCb->stop();

	delete fm;

	resolved->release();
	resolved=0;

	if (!_theOctree)
		delete theOctree;

	theCloud->deleteScalarField(sfIdx);
	theCloud->setCurrentScalarField(oldSfIdx);

	return 0;
}
