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
	, m_lastT(0)
{
}

static CCVector3 ComputeRobustAverageNorm(	CCLib::ReferenceCloud* subset,
											ccGenericPointCloud* sourceCloud )
{
	if (!subset || subset->size()==0 || !sourceCloud)
		return CCVector3(0,0,1);

	assert(sourceCloud->hasNormals());
	assert(subset->getAssociatedCloud() == static_cast<CCLib::GenericIndexedCloud*>(sourceCloud));

	//we compute the (least square) best fit plane
	CCVector3 Nplane;
	//const PointCoordinateType* Nlsq = CCLib::Neighbourhood(subset).getLSQPlane();
	//if (Nlsq)
	//{
	//	Nplane = CCVector3(Nlsq);
	//}
	//else //not enough points?
	{
		//we simply take the first one!
		const PointCoordinateType* N = sourceCloud->getPointNormal(subset->getPointGlobalIndex(0));
		Nplane = CCVector3(N);
	}

	//now we can compute the mean normal, using the plane normal as reference for their sign
	CCVector3 Nout(0,0,0);
	unsigned n = subset->size();
	for (unsigned i=0; i<n; ++i)
	{
		const PointCoordinateType* Ni = sourceCloud->getPointNormal(subset->getPointGlobalIndex(i));
		//compute the scalar product between the ith point normal and the robust one
		PointCoordinateType ps = CCVector3::vdot(Ni,Nplane.u);
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
			aCell->cellCode = cellCodes.back();
			aCell->N = ComputeRobustAverageNorm(Yk,cloud);
			aCell->population = Yk->size();
			m_theGrid[gridPos] = aCell;
		}

		cellCodes.pop_back();
	}

	m_initialized = true;

	return 0;
}

void ccFastMarchingForNormsDirection::resolveCellOrientation(unsigned index)
{
	DirectionCell* theCell = static_cast<DirectionCell*>(m_theGrid[index]);
	//orientation has already been determined!
	if (theCell->processed)
		return;
	CCVector3& N = theCell->N;

	//we resolve the normal direction by looking at the (already processed) neighbors
	//static const float c_angle_75deg = cos(75.0 * CC_DEG_TO_RAD);
	//static const float c_angle_60deg = cos(60.0 * CC_DEG_TO_RAD);
	//static const float c_angle_45deg = cos(45.0 * CC_DEG_TO_RAD);
	//static const float c_angle_30deg = cos(30.0 * CC_DEG_TO_RAD);
	//static const float c_angle_15deg = cos(15.0 * CC_DEG_TO_RAD);

	float positiveConf = 0;
	float negativeConf = 0;

	for (unsigned i=0; i<6; ++i)
	{
		DirectionCell* nCell = static_cast<DirectionCell*>(m_theGrid[static_cast<int>(index) + m_neighboursIndexShift[i]]);
		if (nCell && nCell->processed)
		{
			//compute the confidence for each neighbor
			float confidence = 1.0f;

			//1) it depends on the fact that the neighbor's relative position is
			//   compatible with the current cell orientation
			const int* neighbourPosShift = CCLib::c_FastMarchingNeighbourPosShift + 3*i;
			CCVector3 Nn(	static_cast<PointCoordinateType>(neighbourPosShift[0]),
							static_cast<PointCoordinateType>(neighbourPosShift[1]),
							static_cast<PointCoordinateType>(neighbourPosShift[2]) );

			float psNeighborOri = 1.0f - fabs(static_cast<float>(Nn.dot(N))); //ideal: 90 degrees
			confidence *= psNeighborOri;

			//2) it depends on the angle between the current cell's and the neighbor's orientations
			float psRelativeOri = static_cast<float>(nCell->N.dot(N));
			confidence *= psRelativeOri;
			
			//3) eventually it depends on the neighbor cell population
			confidence *= sqrt(static_cast<float>(nCell->population));

			if (psRelativeOri < 0)
			{
				negativeConf += confidence;
			}
			else //if (psRelativeOri >= 0)
			{
				positiveConf += confidence;
			}
		}
	}

	if (negativeConf > positiveConf)
	{
		N *= -1;
		theCell->signConfidence = negativeConf;
	}
	else //if (positive > negative)
	{
		theCell->signConfidence = positiveConf;
	}
	theCell->processed = true;
}

int ccFastMarchingForNormsDirection::step()
{
	if (!m_initialized)
		return -1;

	//get 'earliest' cell
	unsigned minTCellIndex = getNearestTrialCell();

	if (minTCellIndex == 0)
		return 0;

	CCLib::FastMarching::Cell* minTCell =  m_theGrid[minTCellIndex];
	assert(minTCell && minTCell->state != CCLib::FastMarching::Cell::ACTIVE_CELL);

	if (minTCell->T < Cell::T_INF())
	{
		//we add this cell to the "ACTIVE" set
		minTCell->state = CCLib::FastMarching::Cell::ACTIVE_CELL;
		m_activeCells.push_back(minTCellIndex);
		//resolve its orientation by the way
		resolveCellOrientation(minTCellIndex);
		//update current front propagation time
		m_lastT = minTCell->T;

		//add its neighbors to the TRIAL set
		for (unsigned i=0;i<CC_FM_NUMBER_OF_NEIGHBOURS;++i)
		{
			//get neighbor cell
			unsigned nIndex = minTCellIndex + m_neighboursIndexShift[i];
			CCLib::FastMarching::Cell* nCell = m_theGrid[nIndex];
			if (nCell)
			{
				//if it' not yet a TRIAL cell
				if (nCell->state == CCLib::FastMarching::Cell::FAR_CELL)
				{
					nCell->state = CCLib::FastMarching::Cell::TRIAL_CELL;
					nCell->T = computeT(nIndex);

					addTrialCell(nIndex);
				}
				//otherwise we must update it's arrival time
				else if (nCell->state == CCLib::FastMarching::Cell::TRIAL_CELL)
				{
					const float& t_old = nCell->T;
					float t_new = computeT(nIndex);

					if (t_new < t_old)
						nCell->T = t_new;
				}
			}
		}
	}

	return 1;
}

float ccFastMarchingForNormsDirection::computeTCoefApprox(CCLib::FastMarching::Cell* currentCell, CCLib::FastMarching::Cell* neighbourCell) const
{
	//the more the cells orientation are alike, the faster
	PointCoordinateType ps = static_cast<DirectionCell*>(currentCell)->N.dot(static_cast<DirectionCell*>(neighbourCell)->N);
	return std::max(0.0f,1.0f/std::max(static_cast<float>(fabs(ps)),1.0e-6f));
}

float ccFastMarchingForNormsDirection::computeT(unsigned index)
{
	DirectionCell* theCell = static_cast<DirectionCell*>(m_theGrid[index]);

	//arrival time FROM the 6 neighbors
	double T[6] = { 0,0,0,0,0,0 };
	{
		for (unsigned n=0; n<6; ++n)
		{
			int nIndex = static_cast<int>(index) + m_neighboursIndexShift[n];
			DirectionCell* nCell = static_cast<DirectionCell*>(m_theGrid[nIndex]);
			if (nCell)
			{
				T[n] = nCell->T + m_neighboursDistance[n] * computeTCoefApprox(nCell,theCell);
			}
			else
			{
				//no front yet
				T[n] = Cell::T_INF();
			}
		}
	}

	double A=0, B=0, C=0;
	double Tij = theCell->T;

	//Quadratic eq. along X
	{
		//see c_FastMarchingNeighbourPosShift for correspondances
		const double& Txm = T[3];
		const double& Txp = T[1];
		double Tmin = std::min(Txm,Txp);
		if (Tij > Tmin)
		{
			A += 1.0;
			B += -2.0 * Tmin;
			C += Tmin * Tmin;
		}
	}

	//Quadratic eq. along Y
	{
		//see c_FastMarchingNeighbourPosShift for correspondances
		const double& Tym = T[0];
		const double& Typ = T[2];
		double Tmin = std::min(Tym,Typ);
		if (Tij > Tmin)
		{
			A += 1.0;
			B += -2.0 * Tmin;
			C += Tmin * Tmin;
		}
	}

	//Quadratic eq. along Z
	{
		//see c_FastMarchingNeighbourPosShift for correspondances
		const double& Tzm = T[4];
		const double& Tzp = T[5];
		double Tmin = std::min(Tzm,Tzp);
		if (Tij > Tmin)
		{
			A += 1.0;
			B += -2.0 * Tmin;
			C += Tmin * Tmin;
		}
	}

	C -=  static_cast<double>(m_cellSize*m_cellSize);

	double delta = B*B - 4.0*A*C;

	//cases when the quadratic equation is singular
	if (A == 0 || delta < 0)
	{
		Tij = Cell::T_INF();

		for (unsigned n=0; n<CC_FM_NUMBER_OF_NEIGHBOURS; n++)
		{
			int nIndex = static_cast<int>(index) + m_neighboursIndexShift[n];
			DirectionCell* nCell = static_cast<DirectionCell*>(m_theGrid[nIndex]);
			if (nCell)
			{
				if (nCell->state == CCLib::FastMarching::Cell::TRIAL_CELL || nCell->state == CCLib::FastMarching::Cell::ACTIVE_CELL)
				{
					float candidateT = nCell->T + m_neighboursDistance[n] * computeTCoefApprox(nCell,theCell);
					//take the 'closest' neighbour
					if (candidateT < Tij)
						Tij = candidateT;
				}
			}
		}
	}
	else
	{
		//Solve the quadratic equation. Note that the new crossing
		//must be GREATER than the average of the active neighbors,
		//since only EARLIER elements are active. Therefore the plus
		//sign is appropriate.
		Tij = (-B + sqrt(delta))/(2.0*A);
	}

	return static_cast<float>(Tij);
}

void ccFastMarchingForNormsDirection::initLastT()
{
	m_lastT = 0;
	for (size_t i=0; i<m_activeCells.size(); i++)
	{
		CCLib::FastMarching::Cell* cell = m_theGrid[m_activeCells[i]];
		assert(cell);
		m_lastT = std::max(m_lastT,cell->T);
	}
}

int ccFastMarchingForNormsDirection::propagate()
{
	initTrialCells();
	initLastT();

	int result = 1;
	while (result > 0)
	{
		result = step();
	}

	return result;
}

int ccFastMarchingForNormsDirection::updateResolvedTable(	ccGenericPointCloud* theCloud,
                                                            GenericChunkedArray<1,uchar> &resolved,
                                                            NormsIndexesTableType* theNorms)
{
	if (!m_initialized || !m_octree || m_gridLevel > CCLib::DgmOctree::MAX_OCTREE_LEVEL)
		return -1;

	int count = 0;
	for (unsigned i=0; i<m_activeCells.size(); ++i)
	{
		DirectionCell* aCell = (DirectionCell*)m_theGrid[m_activeCells[i]];
		CCLib::ReferenceCloud* Yk = m_octree->getPointsInCell(aCell->cellCode,m_gridLevel,true);
		if (!Yk)
			continue;

		for (unsigned k=0; k<Yk->size(); ++k)
		{
			unsigned index = Yk->getPointGlobalIndex(k);
			resolved.setValue(index,1); //resolvedValue=1

			const normsType& norm = theNorms->getValue(index);
			const PointCoordinateType* N = ccNormalVectors::GetNormal(norm);
			if (CCVector3::vdot(N,aCell->N.u) < 0)
			{
				PointCoordinateType newN[3]= { -N[0], -N[1], -N[2] };
				theNorms->setValue(index,ccNormalVectors::GetNormIndex(newN));
			}

			theCloud->setPointScalarValue(index,aCell->T);
			//theCloud->setPointScalarValue(index,aCell->signConfidence);
			++count;
		}
	}

	return count;
}

void ccFastMarchingForNormsDirection::endPropagation()
{
	while (!m_activeCells.empty())
	{
		DirectionCell* aCell = static_cast<DirectionCell*>(m_theGrid[m_activeCells.back()]);
		delete aCell;
		m_theGrid[m_activeCells.back()] = NULL;

		m_activeCells.pop_back();
	}

	while (!m_trialCells.empty())
	{
		CCLib::FastMarching::Cell* aCell = m_theGrid[m_trialCells.back()];

		assert(aCell != NULL);

		aCell->state = CCLib::FastMarching::Cell::FAR_CELL;
		aCell->T = Cell::T_INF();

		m_trialCells.pop_back();
	}

	m_lastT = 0;
}

void ccFastMarchingForNormsDirection::initTrialCells()
{
	//for all 'ACTIVE' cells (i.e. seeds at this point)
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
		for (unsigned i=0; i<CC_FM_NUMBER_OF_NEIGHBOURS; ++i)
		{
			unsigned nIndex = index + m_neighboursIndexShift[i];
			DirectionCell* nCell = (DirectionCell*)m_theGrid[nIndex];
			//if the neighbor exists (it shouldn't be in the TRIAL or ACTIVE sets)
			if (nCell/* && nCell->state == CCLib::FastMarching::Cell::FAR_CELL*/)
			{
				assert(nCell->state == CCLib::FastMarching::Cell::FAR_CELL);
				nCell->state = CCLib::FastMarching::Cell::TRIAL_CELL;
				addTrialCell(nIndex);

				//compute its approximate arrival time
				nCell->T = seedCell->T + m_neighboursDistance[i] * computeTCoefApprox(seedCell,nCell);

				//make sure that the normal sign is concordant
				PointCoordinateType ps = seedCell->N.dot(nCell->N);
				if (ps < 0)
					nCell->N *= -1;
				//mark those cells as 'processed' (i.e. their nomal is fixed)
				nCell->processed = true;
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
        return -2;

	//we compute the octree if none is provided
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
	if (sfIdx < 0)
		sfIdx = theCloud->addScalarField("FM_Propagation");
	if (sfIdx >= 0)
		theCloud->setCurrentScalarField(sfIdx);
	else
	{
		ccLog::Warning("[ccFastMarchingForNormsDirection] Couldn't create temporary scalar field! Not enough memory?");
		if (!_theOctree)
			delete theOctree;
		return -5;
	}

	if (!theCloud->enableScalarField())
	{
		ccLog::Warning("[ccFastMarchingForNormsDirection] Couldn't enable temporary scalar field! Not enough memory?");
		if (!_theOctree)
			delete theOctree;
		return -5;
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
		return -4;
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
		int result = fm.propagate();

		//if it's a success
		if (result >= 0)
		{
			//compute the number of points processed during this pass
			int count = fm.updateResolvedTable(theCloud,*resolved,theNorms);

			if (count >=0)
			{
				resolvedPoints += static_cast<unsigned>(count);
				if  (progressCb)
					progressCb->update(static_cast<float>(resolvedPoints)/static_cast<float>(numberOfPoints)*100.0f);
			}

			fm.endPropagation();
		}
	}

	if (progressCb)
		progressCb->stop();

	resolved->release();
	resolved = 0;

	if (!_theOctree)
		delete theOctree;

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
