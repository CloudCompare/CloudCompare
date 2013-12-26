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

//system
#include <assert.h>

ccFastMarchingForNormsDirection::ccFastMarchingForNormsDirection()
	: CCLib::FastMarching()
{
}

static CCVector3 ComputeRobustAverageNorm(	CCLib::ReferenceCloud* subset,
											ccGenericPointCloud* sourceCloud)
{
	if (!subset || subset->size()==0 || !sourceCloud)
		return CCVector3(0,0,1);

	assert(sourceCloud->hasNormals());
	assert(subset->getAssociatedCloud() == static_cast<CCLib::GenericIndexedCloud*>(sourceCloud));

	//we simply take the first normal as reference (DGM: seems to work better than the LSQ plane!)
	const PointCoordinateType* N = sourceCloud->getPointNormal(subset->getPointGlobalIndex(0));

	//now we can compute the mean normal, using the first normal as reference for the sign
	CCVector3 Nout(0,0,0);
	unsigned n = subset->size();
	for (unsigned i=0; i<n; ++i)
	{
		const PointCoordinateType* Ni = sourceCloud->getPointNormal(subset->getPointGlobalIndex(i));
		//compute the scalar product between the ith point normal and the robust one
		PointCoordinateType ps = CCVector3::vdot(Ni,N);
		if (ps < 0)
		{
			CCVector3::vsubstract(Nout.u,Ni,Nout.u);
		}
		else
		{
			CCVector3::vadd(Nout.u,Ni,Nout.u);
		}
	}

	Nout.normalize();

	return Nout;
}

int ccFastMarchingForNormsDirection::init(ccGenericPointCloud* cloud,
                                            NormsIndexesTableType* theNorms,
                                            CCLib::DgmOctree* theOctree,
                                            uchar level)
{
	int result = initGridWithOctree(theOctree, level);
	if (result < 0)
		return result;

	//fill the grid with the octree
	CCLib::DgmOctree::cellCodesContainer cellCodes;
	theOctree->getCellCodes(level,cellCodes,true);

	while (!cellCodes.empty())
	{
		CCLib::ReferenceCloud* Yk = theOctree->getPointsInCell(cellCodes.back(),level,true);
		if (Yk)
		{
			//convert the octree cell code to grid position
			int cellPos[3];
			theOctree->getCellPos(cellCodes.back(),level,cellPos,true);

			//convert it to FM cell pos index
			unsigned gridPos = FM_pos2index(cellPos);

			//create corresponding cell
			DirectionCell* aCell = new DirectionCell;
			{
				//aCell->signConfidence = 1;
				aCell->cellCode = cellCodes.back();
				aCell->N = ComputeRobustAverageNorm(Yk,cloud);
				aCell->C = *(CCLib::Neighbourhood(Yk).getGravityCenter());
			}
			
			m_theGrid[gridPos] = aCell;
		}

		cellCodes.pop_back();
	}

	m_initialized = true;

	return 0;
}

float ccFastMarchingForNormsDirection::computePropagationConfidence(DirectionCell* originCell, DirectionCell* destCell) const
{
	//1) it depends on the angle between the current cell's orientation
	//   and its neighbor's orientation (symmetric)
	//2) it depends on whether the neighbor's relative position is
	//   compatible with the current cell orientation (symmetric)
	CCVector3 AB = destCell->C - originCell->C;
	AB.normalize();

	float psOri = fabs(static_cast<float>(AB.dot(originCell->N))); //ideal: 90 degrees
	float psDest = fabs(static_cast<float>(AB.dot(destCell->N))); //ideal: 90 degrees
	float oriConfidence = (psOri + psDest)/2; //between 0 and 1 (ideal: 0)
	
	return 1.0f - oriConfidence;
}

void ccFastMarchingForNormsDirection::resolveCellOrientation(unsigned index)
{
	DirectionCell* theCell = static_cast<DirectionCell*>(m_theGrid[index]);
	CCVector3& N = theCell->N;

	//we resolve the normal direction by looking at the (already processed) neighbors
	bool inverseNormal = false;
	float bestConf = 0;
//#define USE_BEST_NEIGHBOR_ONLY
#ifndef USE_BEST_NEIGHBOR_ONLY
	unsigned nPos = 0;
	float confPos = 0;
	unsigned nNeg = 0;
	float confNeg = 0;
#endif
	for (unsigned i=0; i<m_numberOfNeighbours; ++i)
	{
		DirectionCell* nCell = static_cast<DirectionCell*>(m_theGrid[static_cast<int>(index) + m_neighboursIndexShift[i]]);
		if (nCell && nCell->state == DirectionCell::ACTIVE_CELL)
		{
			//compute the confidence for each neighbor
			float confidence = computePropagationConfidence(nCell,theCell);
#ifdef USE_BEST_NEIGHBOR_ONLY
			if (confidence > bestConf)
			{
				bestConf = confidence;
				float ps = static_cast<float>(nCell->N.dot(N));
				inverseNormal = (ps < 0);
			}
#else
			//voting
			float ps = static_cast<float>(nCell->N.dot(N));
			if (ps < 0)
			{
				nNeg++;
				confNeg += confidence;
			}
			else
			{
				nPos++;
				confPos += confidence;
			}
#endif
		}
	}
	
#ifndef USE_BEST_NEIGHBOR_ONLY
	inverseNormal = (nNeg == nPos ? confNeg > confPos : nNeg > nPos);
	bestConf = inverseNormal ? confNeg : confPos; //DGM: absolute confidence seems to work better...
	//bestConf = inverseNormal ? confNeg/static_cast<float>(nNeg) : confPos/static_cast<float>(nPos);
#endif
	if (inverseNormal)
	{
		N *= -1;
	}
	theCell->signConfidence = bestConf;
	assert(theCell->signConfidence > 0);
}

#ifdef _DEBUG
//for debug purposes only
static unsigned s_cellIndex = 0;
#endif
int ccFastMarchingForNormsDirection::step()
{
	if (!m_initialized)
		return -1;

	//get 'earliest' cell
	unsigned minTCellIndex = getNearestTrialCell();
	if (minTCellIndex == 0)
		return 0;

	CCLib::FastMarching::Cell* minTCell =  m_theGrid[minTCellIndex];
	assert(minTCell && minTCell->state != DirectionCell::ACTIVE_CELL);

	if (minTCell->T < Cell::T_INF())
	{
#ifdef _DEBUG
		if (s_cellIndex == 0)
		{
			//process seed cells first!
			for (size_t i=0; i<m_activeCells.size(); ++i)
				static_cast<DirectionCell*>(m_theGrid[m_activeCells[i]])->scalar = static_cast<float>(0);
			s_cellIndex++;
		}
		static_cast<DirectionCell*>(minTCell)->scalar = static_cast<float>(s_cellIndex++);
#endif

		//resolve the cell orientation
		resolveCellOrientation(minTCellIndex);
		//we add this cell to the "ACTIVE" set
		addActiveCell(minTCellIndex);

		//add its neighbors to the TRIAL set
		for (unsigned i=0;i<m_numberOfNeighbours;++i)
		{
			//get neighbor cell
			unsigned nIndex = minTCellIndex + m_neighboursIndexShift[i];
			CCLib::FastMarching::Cell* nCell = m_theGrid[nIndex];
			if (nCell)
			{
				//if it' not yet a TRIAL cell
				if (nCell->state == DirectionCell::FAR_CELL)
				{
					nCell->T = computeT(nIndex);
					addTrialCell(nIndex);
				}
				//otherwise we must update it's arrival time
				else if (nCell->state == DirectionCell::TRIAL_CELL)
				{
					const float& t_old = nCell->T;
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

float ccFastMarchingForNormsDirection::computeTCoefApprox(CCLib::FastMarching::Cell* originCell, CCLib::FastMarching::Cell* destCell) const
{
	DirectionCell* oCell = static_cast<DirectionCell*>(originCell);
	DirectionCell* dCell = static_cast<DirectionCell*>(destCell);
	float orientationConfidence = computePropagationConfidence(oCell,dCell); //between 0 and 1 (ideal: 1)

	return (1.0f-orientationConfidence) * oCell->signConfidence;
}

int ccFastMarchingForNormsDirection::propagate()
{
	//init "TRIAL" set with seed's neighbors
	initTrialCells();

	int result = 1;
	while (result > 0)
	{
		result = step();
	}

	return result;
}

unsigned ccFastMarchingForNormsDirection::updateResolvedTable(	ccGenericPointCloud* theCloud,
																GenericChunkedArray<1,uchar> &resolved,
																NormsIndexesTableType* theNorms)
{
	if (!m_initialized || !m_octree || m_gridLevel > CCLib::DgmOctree::MAX_OCTREE_LEVEL)
		return 0;

	unsigned count = 0;
	for (size_t i=0; i<m_activeCells.size(); ++i)
	{
		DirectionCell* aCell = static_cast<DirectionCell*>(m_theGrid[m_activeCells[i]]);
		CCLib::ReferenceCloud* Yk = m_octree->getPointsInCell(aCell->cellCode,m_gridLevel,true);
		if (!Yk)
			continue;

		for (unsigned k=0; k<Yk->size(); ++k)
		{
			unsigned index = Yk->getPointGlobalIndex(k);
			resolved.setValue(index,1);

			const normsType& norm = theNorms->getValue(index);
			const PointCoordinateType* N = ccNormalVectors::GetNormal(norm);

			//inverse point normal if necessary
			if (CCVector3::vdot(N,aCell->N.u) < 0)
			{
				PointCoordinateType newN[3]= { -N[0], -N[1], -N[2] };
				theNorms->setValue(index,ccNormalVectors::GetNormIndex(newN));
			}

#ifdef _DEBUG
			theCloud->setPointScalarValue(index,aCell->T);
			//theCloud->setPointScalarValue(index,aCell->signConfidence);
			//theCloud->setPointScalarValue(index,aCell->scalar);
#endif
			
			++count;
		}
	}

	return count;
}

void ccFastMarchingForNormsDirection::initTrialCells()
{
	//we expect at most one 'ACTIVE' cell (i.e. the current seed)
	size_t seedCount = m_activeCells.size();
	assert(seedCount <= 1);

	if (seedCount == 1)
	{
		unsigned index = m_activeCells.front();
		DirectionCell* seedCell = static_cast<DirectionCell*>(m_theGrid[index]);

		assert(seedCell != NULL);
		assert(seedCell->T == 0);
		assert(seedCell->signConfidence == 1);

		//add all its neighbour cells to the TRIAL set
		for (unsigned i=0; i<m_numberOfNeighbours; ++i)
		{
			unsigned nIndex = index + m_neighboursIndexShift[i];
			DirectionCell* nCell = (DirectionCell*)m_theGrid[nIndex];
			//if the neighbor exists (it shouldn't be in the TRIAL or ACTIVE sets)
			if (nCell/* && nCell->state == DirectionCell::FAR_CELL*/)
			{
				assert(nCell->state == DirectionCell::FAR_CELL);
				addTrialCell(nIndex);

				//compute its approximate arrival time
				nCell->T = seedCell->T + m_neighboursDistance[i] * computeTCoefApprox(seedCell,nCell);
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

	unsigned numberOfPoints = theCloud->size();
	if (numberOfPoints == 0)
        return -1;

	//we compute the octree if none is provided
	CCLib::DgmOctree* theOctree = _theOctree;
	if (!theOctree)
	{
		theOctree = new CCLib::DgmOctree(theCloud);
		if (theOctree->build(progressCb)<1)
		{
			delete theOctree;
			return -2;
		}
	}

	//temporary SF
	int oldSfIdx = theCloud->getCurrentDisplayedScalarFieldIndex();
	int sfIdx = theCloud->getScalarFieldIndexByName("FM_Propagation");
	if (sfIdx < 0)
		sfIdx = theCloud->addScalarField("FM_Propagation");
	if (sfIdx >= 0)
		theCloud->setCurrentScalarField(sfIdx);
	else
	{
		ccLog::Warning("[ccFastMarchingForNormsDirection] Couldn't create temporary scalar field! Not enough memory?");
		if (!_theOctree)
			delete theOctree;
		return -3;
	}

	if (!theCloud->enableScalarField())
	{
		ccLog::Warning("[ccFastMarchingForNormsDirection] Couldn't enable temporary scalar field! Not enough memory?");
		theCloud->deleteScalarField(sfIdx);
		theCloud->setCurrentScalarField(oldSfIdx);
		if (!_theOctree)
			delete theOctree;
		return -4;
	}

	//flags indicating if each point has been processed or not
	GenericChunkedArray<1,uchar>* resolved = new GenericChunkedArray<1,uchar>();
	if (!resolved->resize(numberOfPoints,true,0)) //defaultResolvedValue = 0
	{
		ccLog::Warning("[ccFastMarchingForNormsDirection] Not enough memory!");
		theCloud->deleteScalarField(sfIdx);
		theCloud->setCurrentScalarField(oldSfIdx);
		if (!_theOctree)
			delete theOctree;
		resolved->release();
		return -5;
	}

	//Fast Marching propagation
	ccFastMarchingForNormsDirection fm;

	int result = fm.init(theCloud,theNorms,theOctree,octreeLevel);
	if (result < 0)
	{
		ccLog::Error("[ccFastMarchingForNormsDirection] Something went wrong during initialization...");
		theCloud->deleteScalarField(sfIdx);
		theCloud->setCurrentScalarField(oldSfIdx);
		resolved->release();
		if (!_theOctree)
			delete theOctree;
		return -6;
	}

	//progress notification
	if (progressCb)
	{
		progressCb->reset();
		progressCb->setMethodTitle("Norms direction");
		progressCb->setInfo(qPrintable(QString("Octree level: %1\nPoints: %2").arg(octreeLevel).arg(numberOfPoints)));
		progressCb->start();
	}

	const int octreeWidth = (1<<octreeLevel)-1;

	//enable 26-connectivity
	//fm.setExtendedConnectivity(true);

	//while non-processed points remain...
	unsigned resolvedPoints = 0;
	int lastProcessedPoint = -1;
	while (true)
	{
		//find the next non-processed point
		do
		{
			++lastProcessedPoint;
		}
		while (lastProcessedPoint < static_cast<int>(numberOfPoints) && resolved->getValue(lastProcessedPoint) != 0);

		//all points have been processed? Then we can stop.
		if (lastProcessedPoint == static_cast<int>(numberOfPoints))
			break;

		//we start the propagation from this point
		//its corresponding cell in fact ;)
		const CCVector3 *thePoint = theCloud->getPoint(lastProcessedPoint);
		int pos[3];
		theOctree->getTheCellPosWhichIncludesThePoint(thePoint,pos,octreeLevel);

		//clipping (in case the octree is not 'complete')
		pos[0] = std::min(octreeWidth,pos[0]);
		pos[1] = std::min(octreeWidth,pos[1]);
		pos[2] = std::min(octreeWidth,pos[2]);

		//set corresponding FM cell as 'seed'
		fm.setSeedCell(pos);

		//launch propagation
		int propagationResult = fm.propagate();

		//if it's a success
		if (propagationResult >= 0)
		{
			//compute the number of points processed during this pass
			unsigned count = fm.updateResolvedTable(theCloud,*resolved,theNorms);

			if (count != 0)
			{
				resolvedPoints += count;
				if (progressCb)
					progressCb->update(static_cast<float>(resolvedPoints)/static_cast<float>(numberOfPoints)*100.0f);
			}

			fm.cleanLastPropagation();
		}
		else
		{
			ccLog::Error("An error occurred during front propagation! Process cancelled...");
			break;
		}
	}

	if (progressCb)
		progressCb->stop();

	resolved->release();
	resolved = 0;

	if (!_theOctree)
		delete theOctree;

	theCloud->showNormals(true);
#ifdef _DEBUG
	theCloud->setCurrentDisplayedScalarField(sfIdx);
	theCloud->getCurrentDisplayedScalarField()->computeMinAndMax();
	theCloud->showSF(true);
#else
	theCloud->deleteScalarField(sfIdx);
	theCloud->setCurrentScalarField(oldSfIdx);
#endif

	return 0;
}
