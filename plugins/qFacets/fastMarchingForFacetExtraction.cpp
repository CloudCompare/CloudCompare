//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qFacets                       #
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
//#                           COPYRIGHT: BRGM                              #
//#                                                                        #
//##########################################################################

#include "fastMarchingForFacetExtraction.h"

//qCC_db
#include <ccNormalVectors.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccOctree.h>
#include <ccLog.h>

//Qt
#include <QApplication>

//system
#include <assert.h>

//! 26-connexity neighbouring cells positions (common edges)
const int c_3dNeighboursPosShift[] = {-1,-1,-1,
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

FastMarchingForFacetExtraction::FastMarchingForFacetExtraction()
	: CCLib::FastMarching()
	, m_currentFacetPoints(0)
	, m_currentFacetError(0)
	, m_maxError(0)
	, m_errorMeasure(CCLib::DistanceComputationTools::RMS)
	, m_useRetroProjectionError(false)
{
}

FastMarchingForFacetExtraction::~FastMarchingForFacetExtraction()
{
	if (m_currentFacetPoints)
		delete m_currentFacetPoints;
}

static bool ComputeCellStats(	CCLib::ReferenceCloud* subset,
								CCVector3& N,
								CCVector3& C,
								ScalarType& error,
								CCLib::DistanceComputationTools::ERROR_MEASURES errorMeasure)
{
	error = 0;

	if (!subset || subset->size() == 0)
		return false;

	//we compute the gravity center
	CCLib::Neighbourhood Yk(subset);
	C = *Yk.getGravityCenter();

	//we compute the (least square) best fit plane
	const PointCoordinateType* planeEquation = Yk.getLSPlane();
	if (planeEquation)
	{
		N = CCVector3(planeEquation); //normal = first 3 components
		error = CCLib::DistanceComputationTools::ComputeCloud2PlaneDistance(subset, planeEquation, errorMeasure);
	}
	else
	{
		//not enough points?
		N = CCVector3(0,0,0);
	}

	return true;
}

int FastMarchingForFacetExtraction::init(	ccGenericPointCloud* cloud,
											CCLib::DgmOctree* theOctree,
											unsigned char level,
											ScalarType maxError,
											CCLib::DistanceComputationTools::ERROR_MEASURES errorMeasure,
											bool useRetroProjectionError,
											CCLib::GenericProgressCallback* progressCb/*=0*/)
{
	m_maxError = maxError;
	m_errorMeasure = errorMeasure;
	m_useRetroProjectionError = useRetroProjectionError;

	if (progressCb)
	{
		progressCb->reset();
		progressCb->setMethodTitle("Fast Marching grid initialization");
		progressCb->setInfo(qPrintable(QString("Level: %1").arg(level)));
		progressCb->start();
	}

	int result = initGridWithOctree(theOctree, level);
	if (result < 0)
		return result;

	//fill the grid with the octree
	CCLib::DgmOctree::cellCodesContainer cellCodes;
	theOctree->getCellCodes(level,cellCodes,true);
	size_t cellCount = cellCodes.size();

	CCLib::NormalizedProgress* nProgress = 0;
	if (progressCb)
	{
		progressCb->setInfo(qPrintable(QString("Level: %1\nCells: %2").arg(level).arg(cellCount)));
		nProgress = new CCLib::NormalizedProgress(progressCb,static_cast<unsigned>(cellCount));
	}

	CCLib::ReferenceCloud Yk(theOctree->associatedCloud());
	while (!cellCodes.empty())
	{
		if (theOctree->getPointsInCell(cellCodes.back(),level,&Yk,true))
		{
			//convert the octree cell code to grid position
			Tuple3i cellPos;
			theOctree->getCellPos(cellCodes.back(),level,cellPos,true);

			CCVector3 N,C;
			ScalarType error;
			if (ComputeCellStats(&Yk,N,C,error,m_errorMeasure))
			{
				//convert octree cell pos to FM cell pos index
				unsigned gridPos = pos2index(cellPos);

				//create corresponding cell
				PlanarCell* aCell = new PlanarCell;
				aCell->cellCode = cellCodes.back();
				aCell->N = N;
				aCell->C = C;
				aCell->planarError = error;
				m_theGrid[gridPos] = aCell;
			}
			else
			{
				//an error occurred?!
				return -10;
			}
		}

		cellCodes.pop_back();

		if (nProgress && !nProgress->oneStep())
		{
			//process cancelled by user
			progressCb->stop();
			delete nProgress;
			return -1;
		}
	}

	if (nProgress)
	{
		progressCb->stop();
		delete nProgress;
		nProgress = 0;
	}
		
	m_initialized = true;

	return 0;
}

int FastMarchingForFacetExtraction::step()
{
	if (!m_initialized)
		return -1;

	//get 'earliest' cell
	unsigned minTCellIndex = getNearestTrialCell();
	if (minTCellIndex == 0)
		return 0;

	CCLib::FastMarching::Cell* minTCell =  m_theGrid[minTCellIndex];
	assert(minTCell && minTCell->state != PlanarCell::ACTIVE_CELL);

	if (minTCell->T < Cell::T_INF())
	{
		assert(m_currentFacetPoints);
		unsigned sizeBefore = m_currentFacetPoints->size();

		//check if we can add the cell to the current "ACTIVE" set
		ScalarType error = addCellToCurrentFacet(minTCellIndex);

		if (error >= 0)
		{
			if (error > m_maxError)
			{
				//resulting error would be too high
				m_currentFacetPoints->resize(sizeBefore);
				//we leave the cell as is (in the EMPTY state)
				//so that we won't look at it again!
				addIgnoredCell(minTCellIndex);
			}
			else
			{
				m_currentFacetError = error;

				//add the cell to the "ACTIVE" set
				addActiveCell(minTCellIndex);

				//add its neighbors to the TRIAL set
				for (unsigned i=0; i<m_numberOfNeighbours; ++i)
				{
					//get neighbor cell
					unsigned nIndex = minTCellIndex + m_neighboursIndexShift[i];
					CCLib::FastMarching::Cell* nCell = m_theGrid[nIndex];
					if (nCell)
					{
						//if it' not yet a TRIAL cell
						if (nCell->state == PlanarCell::FAR_CELL)
						{
							nCell->T = computeT(nIndex);
							addTrialCell(nIndex);
						}
						//otherwise we must update it's arrival time
						else if (nCell->state == PlanarCell::TRIAL_CELL)
						{
							const float& t_old = nCell->T;
							float t_new = computeT(nIndex);

							if (t_new < t_old)
								nCell->T = t_new;
						}
					}
				}
			}
		}
		else
		{
			//an error occurred
			return -1;
		}
	}
	else
	{
		addIgnoredCell(minTCellIndex);
	}

	return 1;
}

float FastMarchingForFacetExtraction::computeTCoefApprox(CCLib::FastMarching::Cell* originCell, CCLib::FastMarching::Cell* destCell) const
{
	PlanarCell* oCell = static_cast<PlanarCell*>(originCell);
	PlanarCell* dCell = static_cast<PlanarCell*>(destCell);

	//compute the 'confidence' relatively to the neighbor cell
	//1) it depends on the angle between the current cell's orientation
	//	and its neighbor's orientation (symmetric)
	//2) it depends on whether the neighbor's relative position is
	//	compatible with the current cell orientation (symmetric)
	float orientationConfidence = 0;
	{
		CCVector3 AB = dCell->C - oCell->C;
		AB.normalize();

		float psOri = fabs(static_cast<float>(AB.dot(oCell->N))); //ideal: 90 degrees
		float psDest = fabs(static_cast<float>(AB.dot(dCell->N))); //ideal: 90 degrees
		orientationConfidence = (psOri + psDest)/2; //between 0 and 1 (ideal: 0)
	}

	//add reprojection error into balance
	if (m_useRetroProjectionError && m_octree && oCell->N.norm2() != 0)
	{
		PointCoordinateType theLSQPlaneEquation[4];
		theLSQPlaneEquation[0] = oCell->N.x;
		theLSQPlaneEquation[1] = oCell->N.y;
		theLSQPlaneEquation[2] = oCell->N.z;
		theLSQPlaneEquation[3] = oCell->C.dot(oCell->N);

		CCLib::ReferenceCloud Yk(m_octree->associatedCloud());
		if (m_octree->getPointsInCell(oCell->cellCode,m_gridLevel,&Yk,true))
		{
			ScalarType reprojError = CCLib::DistanceComputationTools::ComputeCloud2PlaneDistance(&Yk,theLSQPlaneEquation,m_errorMeasure);
			if (reprojError >= 0)
				return (1.0f-orientationConfidence) * static_cast<float>(reprojError);
		}
	}

	return (1.0f-orientationConfidence) /** oCell->planarError*/;
}

int FastMarchingForFacetExtraction::propagate()
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

unsigned FastMarchingForFacetExtraction::updateFlagsTable(	ccGenericPointCloud* theCloud,
																GenericChunkedArray<1,unsigned char> &flags,
																unsigned facetIndex)
{
	if (!m_initialized || !m_currentFacetPoints)
		return 0;

	unsigned pointCount = m_currentFacetPoints->size();
	for (unsigned k=0; k<pointCount; ++k)
	{
		unsigned index = m_currentFacetPoints->getPointGlobalIndex(k);
		flags.setValue(index,1);

		theCloud->setPointScalarValue(index,static_cast<ScalarType>(facetIndex));
	}

	if (m_currentFacetPoints)
		m_currentFacetPoints->clear(false);

	/*for (size_t i=0; i<m_activeCells.size(); ++i)
	{
		//we remove the processed cell so as to be sure not to consider them again!
		CCLib::FastMarching::Cell* cell = m_theGrid[m_activeCells[i]];
		m_theGrid[m_activeCells[i]] = 0;
		if (cell)
			delete cell;
	}
	//*/
	
	//unsigned pointCount = 0;
	CCLib::ReferenceCloud Yk(m_octree->associatedCloud());
	for (size_t i=0; i<m_activeCells.size(); ++i)
	{
		PlanarCell* aCell = static_cast<PlanarCell*>(m_theGrid[m_activeCells[i]]);
		if (!m_octree->getPointsInCell(aCell->cellCode,m_gridLevel,&Yk,true))
			continue;

		for (unsigned k=0; k<Yk.size(); ++k)
		{
			unsigned index = Yk.getPointGlobalIndex(k);
			assert(flags.getValue(index) == 1);
			//flags.setValue(index,1);			
			//++pointCount;
		}

		m_theGrid[m_activeCells[i]] = 0;
		delete aCell;
	}

	return pointCount;
}

bool FastMarchingForFacetExtraction::setSeedCell(const Tuple3i& pos)
{
	if (!CCLib::FastMarching::setSeedCell(pos))
		return false;

	if (m_octree)
	{
		if (!m_currentFacetPoints)
		{
			m_currentFacetPoints = new CCLib::ReferenceCloud(m_octree->associatedCloud());
		}

		unsigned index = pos2index(pos);
		m_currentFacetError = addCellToCurrentFacet(index);
		if (m_currentFacetError < 0) //invalid error?
			return false;
	}

	return true;
}

ScalarType FastMarchingForFacetExtraction::addCellToCurrentFacet(unsigned index)
{
	if (!m_currentFacetPoints || !m_initialized || !m_octree || m_gridLevel > CCLib::DgmOctree::MAX_OCTREE_LEVEL)
		return -1;

	PlanarCell* cell = static_cast<PlanarCell*>(m_theGrid[index]);
	if (!cell)
		return -1;

	CCLib::ReferenceCloud Yk(m_octree->associatedCloud());
	if (!m_octree->getPointsInCell(cell->cellCode,m_gridLevel,&Yk,true))
		return -1;

	if (!m_currentFacetPoints->add(Yk))
	{
		//not enough memory?
		return -1;
	}

	//update error
	CCVector3 N,C;
	ScalarType error;
	ComputeCellStats(m_currentFacetPoints,N,C,error,m_errorMeasure);

	return error;
}

void FastMarchingForFacetExtraction::initTrialCells()
{
	//we expect at most one 'ACTIVE' cell (i.e. the current seed)
	size_t seedCount = m_activeCells.size();
	assert(seedCount <= 1);

	if (seedCount == 1 && m_currentFacetError <= m_maxError)
	{
		unsigned index = m_activeCells.front();
		PlanarCell* seedCell = static_cast<PlanarCell*>(m_theGrid[index]);

		assert(seedCell != NULL);
		assert(seedCell->T == 0);

		//add all its neighbour cells to the TRIAL set
		for (unsigned i=0; i<m_numberOfNeighbours; ++i)
		{
			unsigned nIndex = index + m_neighboursIndexShift[i];
			PlanarCell* nCell = (PlanarCell*)m_theGrid[nIndex];
			//if the neighbor exists (it shouldn't be in the TRIAL or ACTIVE sets)
			if (nCell/* && nCell->state == PlanarCell::FAR_CELL*/)
			{
				assert(nCell->state == PlanarCell::FAR_CELL);
				addTrialCell(nIndex);

				//compute its approximate arrival time
				nCell->T = seedCell->T + m_neighboursDistance[i] * computeTCoefApprox(seedCell,nCell);
			}
		}
	}
}

int FastMarchingForFacetExtraction::ExtractPlanarFacets(	ccPointCloud* theCloud,
															unsigned char octreeLevel,
															ScalarType maxError,
															CCLib::DistanceComputationTools::ERROR_MEASURES errorMeasure,
															bool useRetroProjectionError/*=true*/,
															CCLib::GenericProgressCallback* progressCb/*=0*/,
															CCLib::DgmOctree* _theOctree/*=0*/)
{
	assert(theCloud);

	unsigned numberOfPoints = theCloud->size();
	if (numberOfPoints == 0)
		return -1;

	if (!theCloud->getCurrentOutScalarField())
		return -2;

	if (progressCb)
	{
		//just spawn the dialog so that we can see the
		//octree computation (in case the CPU charge prevents
		//the dialog from being shown)
		progressCb->start();
		QApplication::processEvents();
	}

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

	if (progressCb)
	{
		progressCb->setMethodTitle("Fast Marching for facets extraction");
		progressCb->setInfo("Initializing...");
		progressCb->start();
		QApplication::processEvents();
	}
	if (!theCloud->enableScalarField())
	{
		ccLog::Warning("[FastMarchingForFacetExtraction] Couldn't enable scalar field! Not enough memory?");
		if (!_theOctree)
			delete theOctree;
		return -4;
	}

	//raz SF values
	{
		for (unsigned i=0; i!=theCloud->size(); ++i)
			theCloud->setPointScalarValue(i,0);
	}

	//flags indicating if each point has been processed or not
	GenericChunkedArray<1,unsigned char>* flags = new GenericChunkedArray<1,unsigned char>();
	if (!flags->resize(numberOfPoints,true,0)) //defaultFlagValue = 0
	{
		ccLog::Warning("[FastMarchingForFacetExtraction] Not enough memory!");
		if (!_theOctree)
			delete theOctree;
		flags->release();
		return -5;
	}

	//Fast Marching propagation
	FastMarchingForFacetExtraction fm;

	int result = fm.init(	theCloud,
							theOctree,
							octreeLevel,
							maxError,
							errorMeasure,
							useRetroProjectionError,
							progressCb);
	if (result < 0)
	{
		ccLog::Error("[FastMarchingForFacetExtraction] Something went wrong during initialization...");
		flags->release();
		if (!_theOctree)
			delete theOctree;
		return -6;
	}

	//progress notification
	if (progressCb)
	{
		progressCb->reset();
		progressCb->setMethodTitle("Facets extraction");
		progressCb->setInfo(qPrintable(QString("Octree level: %1\nPoints: %2").arg(octreeLevel).arg(numberOfPoints)));
		progressCb->start();
		QApplication::processEvents();
	}

	const int octreeWidth = (1<<octreeLevel)-1;

	//enable 26-connectivity mode
	//fm.setExtendedConnectivity(true);

	//while non-processed points remain...
	unsigned resolvedPoints = 0;
	int lastProcessedPoint = -1;
	unsigned facetIndex = 0;
	while (true)
	{
		//find the next non-processed point
		do
		{
			++lastProcessedPoint;
		}
		while (lastProcessedPoint < static_cast<int>(numberOfPoints) && flags->getValue(lastProcessedPoint) != 0);

		//all points have been processed? Then we can stop.
		if (lastProcessedPoint == static_cast<int>(numberOfPoints))
			break;

		//we start the propagation from this point
		//its corresponding cell in fact ;)
		const CCVector3 *thePoint = theCloud->getPoint(lastProcessedPoint);
		Tuple3i pos;
		theOctree->getTheCellPosWhichIncludesThePoint(thePoint,pos,octreeLevel);

		//clipping (in case the octree is not 'complete')
		pos.x = std::min(octreeWidth,pos.x);
		pos.y = std::min(octreeWidth,pos.y);
		pos.z = std::min(octreeWidth,pos.z);

		//set corresponding FM cell as 'seed'
		if (!fm.setSeedCell(pos))
		{
			//an error occurred?!
			//result = -7;
			//break;
			continue;
		}

		//launch propagation
		int propagationResult = fm.propagate();

		//compute the number of points processed during this pass
		unsigned count = fm.updateFlagsTable(theCloud,*flags,propagationResult >= 0 ? ++facetIndex : 0); //0 = invalid facet index

		if (count != 0)
		{
			resolvedPoints += count;
			if (progressCb)
			{
				if (progressCb->isCancelRequested())
				{
					result = -7;
					break;
				}
				progressCb->update(static_cast<float>(resolvedPoints)/static_cast<float>(numberOfPoints)*100.0f);
			}
		}

		fm.cleanLastPropagation();
	}

	if (progressCb)
		progressCb->stop();

	flags->release();
	flags = 0;

	if (!_theOctree)
		delete theOctree;

	return result;
}
