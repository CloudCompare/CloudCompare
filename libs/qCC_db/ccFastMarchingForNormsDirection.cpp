//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccFastMarchingForNormsDirection.h"

//Local
#include "ccGenericPointCloud.h"
#include "ccLog.h"
#include "ccNormalVectors.h"
#include "ccOctree.h"
#include "ccPointCloud.h"
#include "ccProgressDialog.h"
#ifdef QT_DEBUG
#include "ccScalarField.h"
#endif

//system
#include <cassert>

ccFastMarchingForNormsDirection::ccFastMarchingForNormsDirection()
	: CCLib::FastMarching()
{
}

static CCVector3 ComputeRobustAverageNorm(	CCLib::ReferenceCloud* subset,
											ccGenericPointCloud* sourceCloud)
{
	if (!subset || subset->size() == 0 || !sourceCloud)
		return CCVector3(0,0,1);

	assert(sourceCloud->hasNormals());
	assert(subset->getAssociatedCloud() == static_cast<CCLib::GenericIndexedCloud*>(sourceCloud));

	//we simply take the first normal as reference (DGM: seems to work better than the LS plane!)
	const CCVector3& N = sourceCloud->getPointNormal(subset->getPointGlobalIndex(0));

	//now we can compute the mean normal, using the first normal as reference for the sign
	CCVector3 Nout(0,0,0);
	unsigned n = subset->size();
	for (unsigned i=0; i<n; ++i)
	{
		const CCVector3& Ni = sourceCloud->getPointNormal(subset->getPointGlobalIndex(i));
		//compute the scalar product between the ith point normal and the robust one
		PointCoordinateType ps = Ni.dot(N);
		if (ps < 0)
			Nout -= Ni;
		else
			Nout += Ni;
	}

	Nout.normalize();

	return Nout;
}

int ccFastMarchingForNormsDirection::init(	ccGenericPointCloud* cloud,
											NormsIndexesTableType* theNorms,
											ccOctree* theOctree,
											unsigned char level)
{
	int result = initGridWithOctree(theOctree, level);
	if (result < 0)
		return result;

	//fill the grid with the octree
	CCLib::DgmOctree::cellCodesContainer cellCodes;
	theOctree->getCellCodes(level,cellCodes,true);

	CCLib::ReferenceCloud Yk(theOctree->associatedCloud());

	while (!cellCodes.empty())
	{
		if (!theOctree->getPointsInCell(cellCodes.back(),level,&Yk,true))
		{
			//not enough memory
			return -1;
		}
		
		//convert the octree cell code to grid position
		Tuple3i cellPos;
		theOctree->getCellPos(cellCodes.back(),level,cellPos,true);

		//convert it to FM cell pos index
		unsigned gridPos = pos2index(cellPos);

		//create corresponding cell
		DirectionCell* aCell = new DirectionCell;
		{
			//aCell->signConfidence = 1;
			aCell->cellCode = cellCodes.back();
			aCell->N = ComputeRobustAverageNorm(&Yk,cloud);
			aCell->C = *CCLib::Neighbourhood(&Yk).getGravityCenter();
		}

		m_theGrid[gridPos] = aCell;

		cellCodes.pop_back();
	}

	m_initialized = true;

	return 0;
}

float ccFastMarchingForNormsDirection::computePropagationConfidence(DirectionCell* originCell, DirectionCell* destCell) const
{
	//1) it depends on the angle between the current cell's orientation
	//	and its neighbor's orientation (symmetric)
	//2) it depends on whether the neighbor's relative position is
	//	compatible with the current cell orientation (symmetric)
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

#ifdef QT_DEBUG
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

	CCLib::FastMarching::Cell* minTCell = m_theGrid[minTCellIndex];
	assert(minTCell && minTCell->state != DirectionCell::ACTIVE_CELL);

	if (minTCell->T < Cell::T_INF())
	{
#ifdef QT_DEBUG
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

unsigned ccFastMarchingForNormsDirection::updateResolvedTable(	ccGenericPointCloud* cloud,
																std::vector<unsigned char>& resolved,
																NormsIndexesTableType* theNorms)
{
	if (!m_initialized || !m_octree || m_gridLevel > CCLib::DgmOctree::MAX_OCTREE_LEVEL)
		return 0;

	CCLib::ReferenceCloud Yk(m_octree->associatedCloud());

	unsigned count = 0;
	for (unsigned int cell : m_activeCells)
	{
		DirectionCell* aCell = static_cast<DirectionCell*>(m_theGrid[cell]);
		if (!m_octree->getPointsInCell(aCell->cellCode, m_gridLevel, &Yk, true))
		{
			//not enough memory
			return 0;
		}

		for (unsigned k = 0; k < Yk.size(); ++k)
		{
			unsigned index = Yk.getPointGlobalIndex(k);
			resolved[index] = 1;

			const CompressedNormType& norm = theNorms->getValue(index);
			const CCVector3& N = ccNormalVectors::GetNormal(norm);

			//inverse point normal if necessary
			if (N.dot(aCell->N) < 0)
			{
				theNorms->setValue(index, ccNormalVectors::GetNormIndex(-N));
			}

#ifdef QT_DEBUG
			cloud->setPointScalarValue(index, aCell->T);
			//cloud->setPointScalarValue(index,aCell->signConfidence);
			//cloud->setPointScalarValue(index,aCell->scalar);
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

		assert(seedCell != nullptr);
		assert(seedCell->T == 0);
		assert(seedCell->signConfidence == 1);

		//add all its neighbour cells to the TRIAL set
		for (unsigned i = 0; i < m_numberOfNeighbours; ++i)
		{
			unsigned nIndex = index + m_neighboursIndexShift[i];
			DirectionCell* nCell = static_cast<DirectionCell*>(m_theGrid[nIndex]);
			//if the neighbor exists (it shouldn't be in the TRIAL or ACTIVE sets)
			if (nCell/* && nCell->state == DirectionCell::FAR_CELL*/)
			{
				assert(nCell->state == DirectionCell::FAR_CELL);
				addTrialCell(nIndex);

				//compute its approximate arrival time
				nCell->T = seedCell->T + m_neighboursDistance[i] * computeTCoefApprox(seedCell, nCell);
			}
		}
	}
}

int ccFastMarchingForNormsDirection::OrientNormals(	ccPointCloud* cloud,
													unsigned char octreeLevel,
													ccProgressDialog* progressCb)
{
	if (!cloud || !cloud->normals())
	{
		const QString	name( (cloud == nullptr) ? QStringLiteral("[unnamed]") : cloud->getName() );
		
		ccLog::Warning(QString("[orientNormalsWithFM] Cloud '%1' is invalid (or cloud has no normals)").arg( name ));
		assert(false);
		return 0;
	}
	NormsIndexesTableType* theNorms = cloud->normals();

	unsigned numberOfPoints = cloud->size();
	if (numberOfPoints == 0)
		return -1;

	//we need the octree
	if (!cloud->getOctree())
	{
		if (!cloud->computeOctree(progressCb))
		{
			ccLog::Warning(QString("[orientNormalsWithFM] Could not compute octree on cloud '%1'").arg(cloud->getName()));
			return 0;
		}
	}
	ccOctree::Shared octree = cloud->getOctree();
	assert(octree);

	//temporary SF
#ifndef QT_DEBUG
	bool sfWasDisplayed = cloud->sfShown();
#endif
	int oldSfIdx = cloud->getCurrentDisplayedScalarFieldIndex();
	int sfIdx = cloud->getScalarFieldIndexByName("FM_Propagation");
	if (sfIdx < 0)
		sfIdx = cloud->addScalarField("FM_Propagation");
	if (sfIdx >= 0)
	{
		cloud->setCurrentScalarField(sfIdx);
	}
	else
	{
		ccLog::Warning("[orientNormalsWithFM] Couldn't create temporary scalar field! Not enough memory?");
		return -3;
	}

	if (!cloud->enableScalarField())
	{
		ccLog::Warning("[orientNormalsWithFM] Couldn't enable temporary scalar field! Not enough memory?");
		cloud->deleteScalarField(sfIdx);
		cloud->setCurrentScalarField(oldSfIdx);
		return -4;
	}

	//flags indicating if each point has been processed or not
	std::vector<unsigned char> resolved;
	try
	{
		resolved.resize(numberOfPoints, 0);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[orientNormalsWithFM] Not enough memory!");
		cloud->deleteScalarField(sfIdx);
		cloud->setCurrentScalarField(oldSfIdx);
		return -5;
	}

	//Fast Marching propagation
	ccFastMarchingForNormsDirection fm;

	int result = fm.init(cloud, theNorms, octree.data(), octreeLevel);
	if (result < 0)
	{
		ccLog::Error("[orientNormalsWithFM] Something went wrong during initialization...");
		cloud->deleteScalarField(sfIdx);
		cloud->setCurrentScalarField(oldSfIdx);
		return -6;
	}

	//progress notification
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setMethodTitle("Norms direction");
			progressCb->setInfo(qPrintable(QString("Octree level: %1\nPoints: %2").arg(octreeLevel).arg(numberOfPoints)));
		}
		progressCb->update(0);
		progressCb->start();
	}

	const int octreeWidth = (1<<octreeLevel)-1;

	//enable 26-connectivity
	//fm.setExtendedConnectivity(true);

	//while non-processed points remain...
	unsigned resolvedPoints = 0;
	int lastProcessedPoint = -1;
	bool success = true;
	while (success)
	{
		//find the next non-processed point
		do
		{
			++lastProcessedPoint;
		}
		while (lastProcessedPoint < static_cast<int>(numberOfPoints) && resolved[lastProcessedPoint] != 0);

		//all points have been processed? Then we can stop.
		if (lastProcessedPoint == static_cast<int>(numberOfPoints))
			break;

		//we start the propagation from this point
		//its corresponding cell in fact ;)
		const CCVector3 *thePoint = cloud->getPoint(lastProcessedPoint);
		Tuple3i cellPos;
		octree->getTheCellPosWhichIncludesThePoint(thePoint, cellPos, octreeLevel);

		//clipping (in case the octree is not 'complete')
		cellPos.x = std::min(octreeWidth, cellPos.x);
		cellPos.y = std::min(octreeWidth, cellPos.y);
		cellPos.z = std::min(octreeWidth, cellPos.z);

		//set corresponding FM cell as 'seed'
		fm.setSeedCell(cellPos);

		//launch propagation
		int propagationResult = fm.propagate();

		//if it's a success
		if (propagationResult >= 0)
		{
			//compute the number of points processed during this pass
			unsigned count = fm.updateResolvedTable(cloud, resolved, theNorms);

			if (count != 0)
			{
				resolvedPoints += count;
				if (progressCb)
					progressCb->update(resolvedPoints / (numberOfPoints * 100.0f));
			}

			fm.cleanLastPropagation();
		}
		else
		{
			ccLog::Error("An error occurred during front propagation! Process cancelled...");
			success = false;
		}
	}

	if (progressCb)
		progressCb->stop();

	cloud->showNormals(true);
#ifdef QT_DEBUG
	cloud->setCurrentDisplayedScalarField(sfIdx);
	cloud->getCurrentDisplayedScalarField()->computeMinAndMax();
	cloud->showSF(true);
#else
	cloud->deleteScalarField(sfIdx);
	cloud->setCurrentScalarField(oldSfIdx);
	cloud->showSF(sfWasDisplayed);
#endif

	return (success ? 1 : 0);
}
