//##########################################################################
//#																		   #
//#								  CCLIB									   #
//#																		   #
//#	 This program is free software; you can redistribute it and/or modify  #
//#	 it under the terms of the GNU Library General Public License as	   #
//#	 published by the Free Software Foundation; version 2 or later of the  #
//#	 License.															   #
//#																		   #
//#	 This program is distributed in the hope that it will be useful,	   #
//#	 but WITHOUT ANY WARRANTY; without even the implied warranty of		   #
//#	 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the		   #
//#	 GNU General Public License for more details.						   #
//#																		   #
//#			 COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)			   #
//#																		   #
//##########################################################################

#include <DgmOctree.h>

//local
#include <CCMiscTools.h>
#include <GenericProgressCallback.h>
#include <ParallelSort.h>
#include <RayAndBox.h>
#include <ReferenceCloud.h>
#include <ScalarField.h>

//system
#include <cstdio>
#include <set>

//DGM: tests in progress
//#define COMPUTE_NN_SEARCH_STATISTICS
//#define ADAPTATIVE_BINARY_SEARCH

#ifdef USE_QT
#ifndef CC_DEBUG
//enables multi-threading handling
#define ENABLE_MT_OCTREE
#endif
#endif

using namespace CCLib;

/**********************************/
/* PRE COMPUTED VALUES AND TABLES */
/**********************************/

//! Const value: ln(2)
static const double LOG_NAT_2 = log(2.0);

//! Pre-computed bit shift values (one for each level)
struct BitShiftValues
{
	//! Default initialization
	BitShiftValues()
	{
		//we compute all possible values
		for (unsigned char level = 0; level <= DgmOctree::MAX_OCTREE_LEVEL; ++level)
		{
			values[level] = (3 * (CCLib::DgmOctree::MAX_OCTREE_LEVEL - level));
		}
	}

	//! Values
	unsigned char values[DgmOctree::MAX_OCTREE_LEVEL+1];
};
static BitShiftValues PRE_COMPUTED_BIT_SHIFT_VALUES;

//! Pre-computed cell codes for all potential cell positions (along a unique dimension)
struct MonoDimensionalCellCodes
{
	//! Total number of positions/values
	/** There are 1024 possible values at level 10, and 2M. at level 21.
		\warning Never pass a 'constant initializer' by reference
	**/
	static const int VALUE_COUNT = CCLib::DgmOctree::MAX_OCTREE_LENGTH;

	//! Default initialization
	MonoDimensionalCellCodes()
	{
		//we compute all possible values for cell codes
		//(along a unique dimension, the other ones are just shifted)
		for (int value = 0; value < VALUE_COUNT; ++value)
		{
			int mask = VALUE_COUNT;
			CCLib::DgmOctree::CellCode code = 0;
			for (unsigned char k = 0; k < CCLib::DgmOctree::MAX_OCTREE_LEVEL; k++)
			{
				mask >>= 1;
				code <<= 3;
				if (value & mask)
				{
					code |= 1;
				}
			}
			values[value] = code;
		}

		//we compute all possible masks as well! (all dimensions)
		//CCLib::DgmOctree::CellCode baseMask = (1 << (3 * CCLib::DgmOctree::MAX_OCTREE_LEVEL));
		//for (int level = CCLib::DgmOctree::MAX_OCTREE_LEVEL; level >= 0; --level)
		//{
		//	masks[level] = baseMask - 1;
		//	baseMask >>= 3;
		//}
	}

	//! Mono-dimensional cell codes
	CCLib::DgmOctree::CellCode values[VALUE_COUNT];

	//! Mono-dimensional cell masks
	//CCLib::DgmOctree::CellCode masks[CCLib::DgmOctree::MAX_OCTREE_LEVEL + 1];
};
static MonoDimensionalCellCodes PRE_COMPUTED_POS_CODES;

/**********************************/
/*		  STATIC ACCESSORS		  */
/**********************************/

unsigned char DgmOctree::GET_BIT_SHIFT(unsigned char level)
{
	//return (3 * (CCLib::DgmOctree::MAX_OCTREE_LEVEL - level));
	return PRE_COMPUTED_BIT_SHIFT_VALUES.values[level];
}

int DgmOctree::OCTREE_LENGTH(int level)
{
	return (1 << level);
}

DgmOctree::CellCode DgmOctree::GenerateTruncatedCellCode(const Tuple3i& cellPos, unsigned char level)
{
	assert( cellPos.x >= 0 && cellPos.x < MonoDimensionalCellCodes::VALUE_COUNT
		&&	cellPos.y >= 0 && cellPos.y < MonoDimensionalCellCodes::VALUE_COUNT
		&&	cellPos.z >= 0 && cellPos.z < MonoDimensionalCellCodes::VALUE_COUNT);

	const unsigned char dec = MAX_OCTREE_LEVEL - level;

	return
		(
			 PRE_COMPUTED_POS_CODES.values[cellPos.x << dec]
		|	(PRE_COMPUTED_POS_CODES.values[cellPos.y << dec] << 1)
		|	(PRE_COMPUTED_POS_CODES.values[cellPos.z << dec] << 2)

		) >> GET_BIT_SHIFT(level);
}

#ifndef OCTREE_CODES_64_BITS
DgmOctree::CellCode DgmOctree::GenerateTruncatedCellCode(const Tuple3s& cellPos, unsigned char level)
{
	assert( cellPos.x >= 0 && cellPos.x < MonoDimensionalCellCodes::VALUE_COUNT
		&&	cellPos.y >= 0 && cellPos.y < MonoDimensionalCellCodes::VALUE_COUNT
		&&	cellPos.z >= 0 && cellPos.z < MonoDimensionalCellCodes::VALUE_COUNT);

	const unsigned char dec = MAX_OCTREE_LEVEL - level;

	return
		(	 PRE_COMPUTED_POS_CODES.values[cellPos.x << dec]
		|	(PRE_COMPUTED_POS_CODES.values[cellPos.y << dec] << 1)
		|	(PRE_COMPUTED_POS_CODES.values[cellPos.z << dec] << 2)

		) >> GET_BIT_SHIFT(level);
}
#endif

static inline DgmOctree::CellCode GenerateCellCodeForDim(int pos)
{
	return PRE_COMPUTED_POS_CODES.values[pos];
}

bool DgmOctree::MultiThreadSupport()
{
#ifdef ENABLE_MT_OCTREE
	return true;
#else
	return false;
#endif
}

/**********************************/
/*		  EVERYTHING ELSE!		  */
/**********************************/

DgmOctree::DgmOctree(GenericIndexedCloudPersist* cloud)
	: m_theAssociatedCloud(cloud)
	, m_numberOfProjectedPoints(0)
	, m_nearestPow2(0)
{
	clear();

	assert(m_theAssociatedCloud);
}

void DgmOctree::clear()
{
	//reset internal tables
	m_dimMin = m_pointsMin = m_dimMax = m_pointsMax = CCVector3(0, 0, 0);

	m_numberOfProjectedPoints = 0;
	m_nearestPow2 = 0;
	m_thePointsAndTheirCellCodes.resize(0);

	memset(m_fillIndexes, 0, sizeof(int)*(MAX_OCTREE_LEVEL + 1) * 6);
	memset(m_cellSize, 0, sizeof(PointCoordinateType)*(MAX_OCTREE_LEVEL + 2));
	updateCellCountTable();
}

int DgmOctree::build(GenericProgressCallback* progressCb)
{
	if (!m_thePointsAndTheirCellCodes.empty())
		clear();

	updateMinAndMaxTables();

	return genericBuild(progressCb);
}

int DgmOctree::build(	const CCVector3& octreeMin,
						const CCVector3& octreeMax,
						const CCVector3* pointsMinFilter/*=0*/,
						const CCVector3* pointsMaxFilter/*=0*/,
						GenericProgressCallback* progressCb/*=0*/)
{
	if (!m_thePointsAndTheirCellCodes.empty())
		clear();

	m_dimMin = octreeMin;
	m_dimMax = octreeMax;

	//the user can specify boundaries for points different than the octree box!
	m_pointsMin = (pointsMinFilter ? *pointsMinFilter : m_dimMin);
	m_pointsMax = (pointsMaxFilter ? *pointsMaxFilter : m_dimMax);

	return genericBuild(progressCb);
}

int DgmOctree::genericBuild(GenericProgressCallback* progressCb)
{
	unsigned pointCount = (m_theAssociatedCloud ? m_theAssociatedCloud->size() : 0);
	if (pointCount == 0)
	{
		//no cloud/point?!
		return -1;
	}

	//allocate memory
	try
	{
		m_thePointsAndTheirCellCodes.resize(pointCount); //resize + operator[] is faster than reserve + push_back!
	}
	catch (.../*const std::bad_alloc&*/) //out of memory
	{
		return -1;
	}
	
	m_numberOfProjectedPoints = 0;
	m_nearestPow2 = 0;

	//update the pre-computed 'cell size per level of subdivision' array
	updateCellSizeTable();

	//progress notification (optional)
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setMethodTitle("Build Octree");
			char infosBuffer[256];
			sprintf(infosBuffer, "Projecting %u points\nMax. depth: %i", pointCount, MAX_OCTREE_LEVEL);
			progressCb->setInfo(infosBuffer);
		}
		progressCb->update(0);
		progressCb->start();
	}
	NormalizedProgress nprogress(progressCb, pointCount, 90); //first phase: 90% (we keep 10% for sort)

	//fill indexes table (we'll fill the max. level, then deduce the others from this one)
	int* fillIndexesAtMaxLevel = m_fillIndexes + (MAX_OCTREE_LEVEL * 6);

	//for all points
	cellsContainer::iterator it = m_thePointsAndTheirCellCodes.begin();
	for (unsigned i=0; i<pointCount; i++)
	{
		const CCVector3* P = m_theAssociatedCloud->getPoint(i);

		//does the point falls in the 'accepted points' box?
		//(potentially different from the octree box - see DgmOctree::build)
		if (	(P->x >= m_pointsMin[0]) && (P->x <= m_pointsMax[0])
			&&	(P->y >= m_pointsMin[1]) && (P->y <= m_pointsMax[1])
			&&	(P->z >= m_pointsMin[2]) && (P->z <= m_pointsMax[2]) )
		{
			//compute the position of the cell that includes this point
			Tuple3i cellPos;
			getTheCellPosWhichIncludesThePoint(P, cellPos);

			//clipping X
			if (cellPos.x < 0)
				cellPos.x = 0;
			else if (cellPos.x >= MAX_OCTREE_LENGTH)
				cellPos.x = MAX_OCTREE_LENGTH-1;
			//clipping Y
			if (cellPos.y < 0)
				cellPos.y = 0;
			else if (cellPos.y >= MAX_OCTREE_LENGTH)
				cellPos.y = MAX_OCTREE_LENGTH-1;
			//clipping Z
			if (cellPos.z < 0)
				cellPos.z = 0;
			else if (cellPos.z >= MAX_OCTREE_LENGTH)
				cellPos.z = MAX_OCTREE_LENGTH-1;

			it->theIndex = i;
			it->theCode = GenerateTruncatedCellCode(cellPos, MAX_OCTREE_LEVEL);

			if (m_numberOfProjectedPoints)
			{
				if (fillIndexesAtMaxLevel[0] > cellPos.x)
					fillIndexesAtMaxLevel[0] = cellPos.x;
				else if (fillIndexesAtMaxLevel[3] < cellPos.x)
					fillIndexesAtMaxLevel[3] = cellPos.x;

				if (fillIndexesAtMaxLevel[1] > cellPos.y)
					fillIndexesAtMaxLevel[1] = cellPos.y;
				else if (fillIndexesAtMaxLevel[4] < cellPos.y)
					fillIndexesAtMaxLevel[4] = cellPos.y;

				if (fillIndexesAtMaxLevel[2] > cellPos.z)
					fillIndexesAtMaxLevel[2] = cellPos.z;
				else if (fillIndexesAtMaxLevel[5] < cellPos.z)
					fillIndexesAtMaxLevel[5] = cellPos.z;
			}
			else
			{
				fillIndexesAtMaxLevel[0] = fillIndexesAtMaxLevel[3] = cellPos.x;
				fillIndexesAtMaxLevel[1] = fillIndexesAtMaxLevel[4] = cellPos.y;
				fillIndexesAtMaxLevel[2] = fillIndexesAtMaxLevel[5] = cellPos.z;
			}

			++it;
			++m_numberOfProjectedPoints;
		}

		if (!nprogress.oneStep())
		{
			m_thePointsAndTheirCellCodes.resize(0);
			m_numberOfProjectedPoints = 0;
			if (progressCb)
			{
				progressCb->stop();
			}
			return 0;
		}
	}

	//we deduce the lower levels 'fill indexes' from the highest level
	{
		for (int k = MAX_OCTREE_LEVEL - 1; k >= 0; k--)
		{
			int* fillIndexes = m_fillIndexes + (k*6);
			for (int dim=0; dim<6; ++dim)
			{
				fillIndexes[dim] = (fillIndexes[dim+6] >> 1);
			}
		}
	}

	if (m_numberOfProjectedPoints < pointCount)
		m_thePointsAndTheirCellCodes.resize(m_numberOfProjectedPoints); //smaller --> should always be ok

	if (progressCb && progressCb->textCanBeEdited())
	{
		progressCb->setInfo("Sorting cells...");
	}

	//we sort the 'cells' by ascending code order
	ParallelSort(m_thePointsAndTheirCellCodes.begin(), m_thePointsAndTheirCellCodes.end(), IndexAndCode::codeComp);

	//update the pre-computed 'number of cells per level of subdivision' array
	updateCellCountTable();

	//end of process notification
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			char buffer[256];
			if (m_numberOfProjectedPoints == pointCount)
			{
				sprintf(buffer, "[Octree::build] Octree successfully built... %u points (ok)!", m_numberOfProjectedPoints);
			}
			else
			{
				if (m_numberOfProjectedPoints == 0)
					sprintf(buffer, "[Octree::build] Warning : no point projected in the Octree!");
				else
					sprintf(buffer, "[Octree::build] Warning: some points have been filtered out (%u/%u)", pointCount - m_numberOfProjectedPoints, pointCount);
			}
			progressCb->setInfo(buffer);
		}

		//DGM: the dialog may close itself once we set the progress to 100% (hiding the above information!)
		progressCb->update(100.0f);
		progressCb->stop();
	}

	m_nearestPow2 = (1 << static_cast<int>( log(static_cast<double>(m_numberOfProjectedPoints-1)) / LOG_NAT_2 ));
   
	return static_cast<int>(m_numberOfProjectedPoints);
}

void DgmOctree::updateMinAndMaxTables()
{
	if (!m_theAssociatedCloud)
		return;

	m_theAssociatedCloud->getBoundingBox(m_pointsMin,m_pointsMax);
	m_dimMin = m_pointsMin;
	m_dimMax = m_pointsMax;

	CCMiscTools::MakeMinAndMaxCubical(m_dimMin,m_dimMax);
}

void DgmOctree::updateCellSizeTable()
{
	//update the cell dimension for each subdivision level
	m_cellSize[0] = m_dimMax.x - m_dimMin.x;

	unsigned long long d = 1;
	for (int k=1; k<=MAX_OCTREE_LEVEL; k++)
	{
		d <<= 1;
		m_cellSize[k] = m_cellSize[0] / d;
	}
}

void DgmOctree::updateCellCountTable()
{
	//level 0 is just the octree bounding-box
	for (unsigned char i=0; i<=MAX_OCTREE_LEVEL; ++i)
	{
		computeCellsStatistics(i);
	}
}

void DgmOctree::computeCellsStatistics(unsigned char level)
{
	assert(level <= MAX_OCTREE_LEVEL);

	//empty octree case?!
	if (m_thePointsAndTheirCellCodes.empty())
	{
		//DGM: we make as if there were 1 point to avoid some degenerated cases!
		m_cellCount[level] = 1;
		m_maxCellPopulation[level] = 1;
		m_averageCellPopulation[level] = 1.0;
		m_stdDevCellPopulation[level] = 0.0;
		return;
	}

	//level '0' specific case
	if (level == 0)
	{
		m_cellCount[level] = 1;
		m_maxCellPopulation[level] = static_cast<unsigned>(m_thePointsAndTheirCellCodes.size());
		m_averageCellPopulation[level] = static_cast<double>(m_thePointsAndTheirCellCodes.size());
		m_stdDevCellPopulation[level] = 0.0;
		return;
	}

	//binary shift for cell code truncation
	unsigned char bitDec = GET_BIT_SHIFT(level);

	//iterator on octree elements
	cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin();

	//we init scan with first element
	CellCode predCode = (p->theCode >> bitDec);
	unsigned counter = 0;
	unsigned cellCounter = 0;
	unsigned maxCellPop = 0;
	double sum = 0.0;
	double sum2 = 0.0;

	for (; p != m_thePointsAndTheirCellCodes.end(); ++p)
	{
		CellCode currentCode = (p->theCode >> bitDec);
		if (predCode != currentCode)
		{
			sum += static_cast<double>(cellCounter);
			sum2 += static_cast<double>(cellCounter) * static_cast<double>(cellCounter);

			if (maxCellPop<cellCounter)
				maxCellPop = cellCounter;

			//new cell
			predCode = currentCode;
			cellCounter = 0;
			++counter;
		}
		++cellCounter;
	}

	//don't forget last cell!
	sum += static_cast<double>(cellCounter);
	sum2 += static_cast<double>(cellCounter) * static_cast<double>(cellCounter);
	if (maxCellPop < cellCounter)
		maxCellPop = cellCounter;
	++counter;

	assert(counter > 0);
	m_cellCount[level] = counter;
	m_maxCellPopulation[level] = maxCellPop;
	m_averageCellPopulation[level] = sum/static_cast<double>(counter);
	m_stdDevCellPopulation[level] = sqrt(sum2/static_cast<double>(counter) - m_averageCellPopulation[level]*m_averageCellPopulation[level]);
}

void DgmOctree::getBoundingBox(CCVector3& bbMin, CCVector3& bbMax) const
{
	bbMin = m_dimMin;
	bbMax = m_dimMax;
}

void DgmOctree::getCellPos(CellCode code, unsigned char level, Tuple3i& cellPos, bool isCodeTruncated) const
{
	//binary shift for cell code truncation
	if (!isCodeTruncated)
	{
		code >>= GET_BIT_SHIFT(level);
	}

	cellPos = Tuple3i(0,0,0);

	int bitMask = 1;
	for (unsigned char k=0; k<level; ++k)
	{
		if (code & 4)
			cellPos.z |= bitMask;
		if (code & 2)
			cellPos.y |= bitMask;
		if (code & 1)
			cellPos.x |= bitMask;

		code >>= 3;
		bitMask <<= 1;
	}
}

void DgmOctree::computeCellLimits(CellCode code, unsigned char level, CCVector3& cellMin, CCVector3& cellMax, bool isCodeTruncated) const
{
	Tuple3i cellPos;
	getCellPos(code,level,cellPos,isCodeTruncated);

	const PointCoordinateType& cs = getCellSize(level);

	cellMin.x = m_dimMin[0] + cs * cellPos.x;
	cellMin.y = m_dimMin[1] + cs * cellPos.y;
	cellMin.z = m_dimMin[2] + cs * cellPos.z;

	cellMax = cellMin + CCVector3(cs,cs,cs);
}

bool DgmOctree::getPointsInCell(CellCode cellCode,
								unsigned char level,
								ReferenceCloud* subset,
								bool isCodeTruncated/*=false*/,
								bool clearOutputCloud/* = true*/) const
{
	unsigned char bitDec = GET_BIT_SHIFT(level);
	if (!isCodeTruncated)
	{
		cellCode >>= bitDec;
	}

	unsigned cellIndex = getCellIndex(cellCode, bitDec);
	//check that cell exists!
	if (cellIndex < m_numberOfProjectedPoints)
	{
		return getPointsInCellByCellIndex(subset, cellIndex, level, clearOutputCloud);
	}
	else if (clearOutputCloud)
	{
		subset->clear();
	}

	return true;
}

unsigned DgmOctree::getCellIndex(CellCode truncatedCellCode, unsigned char bitDec) const
{
	//inspired from the algorithm proposed by MATT PULVER (see http://eigenjoy.com/2011/01/21/worlds-fastest-binary-search/)
	//DGM:	it's not faster, but the code is simpler ;)
	unsigned i = 0;
	unsigned b = m_nearestPow2;
   
	for ( ; b ; b >>= 1 )
	{
		unsigned j = i | b;
		if ( j < m_numberOfProjectedPoints)
		{
			CellCode middleCode = (m_thePointsAndTheirCellCodes[j].theCode >> bitDec);
			if (middleCode < truncatedCellCode )
			{
				//what we are looking for is on the right
				i = j;
			}
			else if (middleCode == truncatedCellCode)
			{
				//we must check that it's the first element equal to input code
				if (j == 0 || (m_thePointsAndTheirCellCodes[j-1].theCode >> bitDec) != truncatedCellCode)
				{
					//what we are looking for is right here
					return j;
				}
				//otherwise what we are looking for is on the left!
			}
		}
	}

	return (m_thePointsAndTheirCellCodes[i].theCode >> bitDec) == truncatedCellCode ? i : m_numberOfProjectedPoints;
}

//optimized version with profiling
#ifdef COMPUTE_NN_SEARCH_STATISTICS
static double s_jumps = 0.0;
static double s_binarySearchCount = 0.0;
#endif

#ifdef ADAPTATIVE_BINARY_SEARCH
unsigned DgmOctree::getCellIndex(CellCode truncatedCellCode, unsigned char bitDec, unsigned begin, unsigned end) const
{
	assert(truncatedCellCode != INVALID_CELL_CODE);
	assert(end >= begin);
	assert(end < m_numberOfProjectedPoints);

#ifdef COMPUTE_NN_SEARCH_STATISTICS
	s_binarySearchCount += 1;
#endif

	//if query cell code is lower than or equal to the first octree cell code, then it's
	//either the good one or there's no match
	CellCode beginCode = (m_thePointsAndTheirCellCodes[begin].theCode >> bitDec);
	if (truncatedCellCode < beginCode)
		return m_numberOfProjectedPoints;
	else if (truncatedCellCode == beginCode)
		return begin;

	//if query cell code is higher than the last octree cell code, then there's no match
	CellCode endCode = (m_thePointsAndTheirCellCodes[end].theCode >> bitDec);
	if (truncatedCellCode > endCode)
		return m_numberOfProjectedPoints;

	while (true)
	{
		float centralPoint = 0.5f + 0.75f*(static_cast<float>(truncatedCellCode-beginCode)/(-0.5f)); //0.75 = speed coef (empirical)
		unsigned middle = begin + static_cast<unsigned>(centralPoint*float(end-begin));
		CellCode middleCode = (m_thePointsAndTheirCellCodes[middle].theCode >> bitDec);

		if (middleCode < truncatedCellCode)
		{
			//no more cell in-between?
			if (middle == begin)
				return m_numberOfProjectedPoints;

			begin = middle;
			beginCode = middleCode;
		}
		else if (middleCode > truncatedCellCode)
		{
			//no more cell in-between?
			if (middle == begin)
				return m_numberOfProjectedPoints;

			end = middle;
			endCode = middleCode;
		}
		else
		{
			//if the previous point doesn't correspond, then we have just found the first good one!
			if ((m_thePointsAndTheirCellCodes[middle-1].theCode >> bitDec) != truncatedCellCode)
				return middle;
			end = middle;
			endCode = middleCode;
		}

#ifdef COMPUTE_NN_SEARCH_STATISTICS
		s_jumps += 1.0;
#endif
	}

	//we shouldn't get there!
	return m_numberOfProjectedPoints;
}

#else

unsigned DgmOctree::getCellIndex(CellCode truncatedCellCode, unsigned char bitDec, unsigned begin, unsigned end) const
{
	assert(truncatedCellCode != INVALID_CELL_CODE);
	assert(end >= begin && end < m_numberOfProjectedPoints);

#ifdef COMPUTE_NN_SEARCH_STATISTICS
	s_binarySearchCount += 1;
#endif

	//inspired from the algorithm proposed by MATT PULVER (see http://eigenjoy.com/2011/01/21/worlds-fastest-binary-search/)
	//DGM:	it's not faster, but the code is simpler ;)
	unsigned i = 0;
	unsigned count = end-begin+1;
	unsigned b = (1 << static_cast<int>( log(static_cast<double>(count-1)) / LOG_NAT_2 ));
	for ( ; b ; b >>= 1 )
	{
		unsigned j = i | b;
		if ( j < count)
		{
			CellCode middleCode = (m_thePointsAndTheirCellCodes[begin+j].theCode >> bitDec);
			if (middleCode < truncatedCellCode )
			{
				//what we are looking for is on the right
				i = j;
			}
			else if (middleCode == truncatedCellCode)
			{
				//we must check that it's the first element equal to input code
				if (j == 0 || (m_thePointsAndTheirCellCodes[begin+j-1].theCode >> bitDec) != truncatedCellCode)
				{
					//what we are looking for is right here
					return j + begin;
				}
				//otheriwse what we are looking for is on the left!
			}
		}

#ifdef COMPUTE_NN_SEARCH_STATISTICS
		s_jumps += 1.0;
#endif
	}

	i += begin;

	return (m_thePointsAndTheirCellCodes[i].theCode >> bitDec) == truncatedCellCode ? i : m_numberOfProjectedPoints;
}
#endif

unsigned DgmOctree::findPointNeighbourhood( const CCVector3* queryPoint,
											ReferenceCloud* Yk,
											unsigned maxNumberOfNeighbors,
											unsigned char level,
											double& maxSquareDist,
											double maxSearchDist/*=0*/,
											int* finalNeighbourhoodSize/*=nullptr*/) const
{
	assert(queryPoint);
	NearestNeighboursSearchStruct nNSS;
	nNSS.queryPoint = *queryPoint;
	nNSS.level = level;
	nNSS.minNumberOfNeighbors = maxNumberOfNeighbors;
	bool inbounds = false;
	getTheCellPosWhichIncludesThePoint(&nNSS.queryPoint, nNSS.cellPos, nNSS.level, inbounds);
	nNSS.alreadyVisitedNeighbourhoodSize = inbounds ? 0 : 1;

	computeCellCenter(nNSS.cellPos, level, nNSS.cellCenter);
	nNSS.maxSearchSquareDistd = (maxSearchDist > 0 ? maxSearchDist * maxSearchDist : 0);

	//special case: N=1
	if (maxNumberOfNeighbors == 1)
	{
		maxSquareDist = findTheNearestNeighborStartingFromCell(nNSS);

		if (finalNeighbourhoodSize)
		{
			*finalNeighbourhoodSize = nNSS.alreadyVisitedNeighbourhoodSize;
		}

		if (maxSquareDist >= 0)
		{
			Yk->addPointIndex(nNSS.theNearestPointIndex);
			return 1;
		}
		else
		{
			return 0;
		}
	}

	//general case: N>1
	unsigned nnFound = findNearestNeighborsStartingFromCell(nNSS);
	if (nnFound)
	{
		//nnFound can be superior to maxNumberOfNeighbors
		//so we only keep the 'maxNumberOfNeighbors' firsts
		nnFound = std::min(nnFound, maxNumberOfNeighbors);

		for (unsigned j = 0; j < nnFound; ++j)
			Yk->addPointIndex(nNSS.pointsInNeighbourhood[j].pointIndex);

		maxSquareDist = nNSS.pointsInNeighbourhood.back().squareDistd;
	}
	else
	{
		maxSquareDist = -1.0;
	}

	if (finalNeighbourhoodSize)
	{
		*finalNeighbourhoodSize = nNSS.alreadyVisitedNeighbourhoodSize;
	}

	return nnFound;
}

void DgmOctree::getCellDistanceFromBorders( const Tuple3i& cellPos,
											unsigned char level,
											int* cellDists) const
{
	const int* fillIndexes = m_fillIndexes+6*level;

	int* _cellDists = cellDists;
	*_cellDists++ = cellPos.x - fillIndexes[0];
	*_cellDists++ = fillIndexes[3] - cellPos.x;
	*_cellDists++ = cellPos.y - fillIndexes[1];
	*_cellDists++ = fillIndexes[4] - cellPos.y;
	*_cellDists++ = cellPos.z - fillIndexes[2];
	*_cellDists++ = fillIndexes[5] - cellPos.z;
}

void DgmOctree::getCellDistanceFromBorders( const Tuple3i& cellPos,
											unsigned char level,
											int neighbourhoodLength,
											int* limits) const
{
	const int* fillIndexes = m_fillIndexes + 6*level;

	int* _limits = limits;
	for (int dim = 0; dim < 3; ++dim)
	{
		//min dim.
		{
			int a = cellPos.u[dim] - fillIndexes[dim];
			if (a < -neighbourhoodLength)
				a = -neighbourhoodLength;
			else if (a > neighbourhoodLength)
				a = neighbourhoodLength;
			*_limits++ = a;
		}

		//max dim.
		{
			int b = fillIndexes[3 + dim] - cellPos.u[dim];
			if (b < -neighbourhoodLength)
				b = -neighbourhoodLength;
			else if (b > neighbourhoodLength)
				b = neighbourhoodLength;
			*_limits++ = b;
		}
	}
}

void DgmOctree::getNeighborCellsAround( const Tuple3i& cellPos,
										cellIndexesContainer &neighborCellsIndexes,
										int neighbourhoodLength,
										unsigned char level) const
{
	assert(neighbourhoodLength > 0);

	//get distance form cell to octree neighbourhood borders
	int limits[6];
	getCellDistanceFromBorders(cellPos, level, neighbourhoodLength, limits);

	//limits are expressed in terms of cells at the CURRENT 'level'!
	const int &iMin = limits[0];
	const int &iMax = limits[1];
	const int &jMin = limits[2];
	const int &jMax = limits[3];
	const int &kMin = limits[4];
	const int &kMax = limits[5];

	//binary shift for cell code truncation
	const unsigned char bitDec = GET_BIT_SHIFT(level);

	for (int i = -iMin; i <= iMax; i++)
	{
		bool iBorder = (abs(i) == neighbourhoodLength); //test: are we on a plane of equation 'X = +/-neighbourhoodLength'?
		CellCode c0 = GenerateCellCodeForDim(cellPos.x + i);

		for (int j = -jMin; j <= jMax; j++)
		{
			CellCode c1 = c0 | (GenerateCellCodeForDim(cellPos.y + j) << 1);

			if (iBorder || (abs(j) == neighbourhoodLength)) //test: are we already on one of the X or Y borders?
			{
				for (int k = -kMin; k <= kMax; k++)
				{
					CellCode c2 = c1 | (GenerateCellCodeForDim(cellPos.z + k) << 2);

					unsigned index = getCellIndex(c2, bitDec);
					if (index < m_numberOfProjectedPoints)
					{
						neighborCellsIndexes.push_back(index);
					}
				}

			}
			else //otherwise we are inside the neighbourhood
			{
				if (kMin == neighbourhoodLength) //test: does the plane of equation 'Z = -neighbourhoodLength' is inside the octree box?
				{
					CellCode c2 = c1 | (GenerateCellCodeForDim(cellPos.z - neighbourhoodLength) << 2);

					unsigned index = getCellIndex(c2, bitDec);
					if (index < m_numberOfProjectedPoints)
					{
						neighborCellsIndexes.push_back(index);
					}
				}

				if (kMax == neighbourhoodLength) //test: does the plane of equation 'Z = +neighbourhoodLength' is inside the octree box?
				{
					CellCode c2 = c1 + (GenerateCellCodeForDim(cellPos.z + kMax) << 2);

					unsigned index = getCellIndex(c2, bitDec);
					if (index < m_numberOfProjectedPoints)
					{
						neighborCellsIndexes.push_back(index);
					}
				}
			}
		}
	}
}

void DgmOctree::getPointsInNeighbourCellsAround(NearestNeighboursSearchStruct &nNSS,
												int neighbourhoodLength,
												bool getOnlyPointsWithValidScalar/*=false*/) const
{
	assert(neighbourhoodLength >= nNSS.alreadyVisitedNeighbourhoodSize);

	//get distance form cell to octree neighbourhood borders
	int limits[6];
	getCellDistanceFromBorders(nNSS.cellPos, nNSS.level, neighbourhoodLength, limits);

	//limits are expressed in terms of cells at the CURRENT 'level'!
	const int &iMin = limits[0];
	const int &iMax = limits[1];
	const int &jMin = limits[2];
	const int &jMax = limits[3];
	const int &kMin = limits[4];
	const int &kMax = limits[5];

	//binary shift for cell code truncation
	const unsigned char bitDec = GET_BIT_SHIFT(nNSS.level);

	for (int i = -iMin; i <= iMax; i++)
	{
		bool iBorder = (abs(i) == neighbourhoodLength); //test: are we on a plane of equation 'X = +/-neighbourhoodLength'?
		CellCode c0 = GenerateCellCodeForDim(nNSS.cellPos.x + i);

		for (int j = -jMin; j <= jMax; j++)
		{
			CellCode c1 = c0 | (GenerateCellCodeForDim(nNSS.cellPos.y + j) << 1);

			//if i or j is on the boundary
			if (iBorder || (abs(j) == neighbourhoodLength)) //test: are we already on one of the X or Y borders?
			{
				for (int k = -kMin; k <= kMax; k++)
				{
					CellCode c2 = c1 | (GenerateCellCodeForDim(nNSS.cellPos.z + k) << 2);

					unsigned index = getCellIndex(c2, bitDec);
					if (index < m_numberOfProjectedPoints)
					{
						//we increase 'pointsInNeighbourCells' capacity with average cell size
						try
						{
							nNSS.pointsInNeighbourhood.reserve(nNSS.pointsInNeighbourhood.size() + static_cast<unsigned>(ceil(m_averageCellPopulation[nNSS.level])));
						}
						catch (.../*const std::bad_alloc&*/) //out of memory
						{
							//DGM TODO: Shall we stop? shall we try to go on, as we are not sure that we will actually need this much points?
							assert(false);
						}
						for (cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin() + index; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == c2); ++p)
						{
							if (!getOnlyPointsWithValidScalar || ScalarField::ValidValue(m_theAssociatedCloud->getPointScalarValue(p->theIndex)))
							{
								nNSS.pointsInNeighbourhood.emplace_back(m_theAssociatedCloud->getPointPersistentPtr(p->theIndex), p->theIndex);
							}
						}
					}
				}

			}
			else //otherwise we are inside the neighbourhood
			{
				if (kMin == neighbourhoodLength) //test: does the plane of equation 'Z = -neighbourhoodLength' is inside the octree box?
				{
					CellCode c2 = c1 | (GenerateCellCodeForDim(nNSS.cellPos.z - neighbourhoodLength) << 2);

					unsigned index = getCellIndex(c2, bitDec);
					if (index < m_numberOfProjectedPoints)
					{
						//we increase 'nNSS.pointsInNeighbourhood' capacity with average cell size
						try
						{
							nNSS.pointsInNeighbourhood.reserve(nNSS.pointsInNeighbourhood.size() + static_cast<unsigned>(ceil(m_averageCellPopulation[nNSS.level])));
						}
						catch (.../*const std::bad_alloc&*/) //out of memory
						{
							//DGM TODO: Shall we stop? shall we try to go on, as we are not sure that we will actually need this much points?
							assert(false);
						}
						for (cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin() + index; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == c2); ++p)
						{
							if (!getOnlyPointsWithValidScalar || ScalarField::ValidValue(m_theAssociatedCloud->getPointScalarValue(p->theIndex)))
							{
								nNSS.pointsInNeighbourhood.emplace_back(m_theAssociatedCloud->getPointPersistentPtr(p->theIndex), p->theIndex);
							}
						}
					}
				}

				if (kMax == neighbourhoodLength) //test: does the plane of equation 'Z = +neighbourhoodLength' is inside the octree box? (note that neighbourhoodLength > 0)
				{
					CellCode c2 = c1 | (GenerateCellCodeForDim(nNSS.cellPos.z + neighbourhoodLength) << 2);

					unsigned index = getCellIndex(c2, bitDec);
					if (index < m_numberOfProjectedPoints)
					{
						//we increase 'nNSS.pointsInNeighbourhood' capacity with average cell size
						try
						{
							nNSS.pointsInNeighbourhood.reserve(nNSS.pointsInNeighbourhood.size() + static_cast<unsigned>(ceil(m_averageCellPopulation[nNSS.level])));
						}
						catch (.../*const std::bad_alloc&*/) //out of memory
						{
							//DGM TODO: Shall we stop? shall we try to go on, as we are not sure that we will actually need this much points?
							assert(false);
						}
						for (cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin() + index; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == c2); ++p)
						{
							if (!getOnlyPointsWithValidScalar || ScalarField::ValidValue(m_theAssociatedCloud->getPointScalarValue(p->theIndex)))
							{
								nNSS.pointsInNeighbourhood.emplace_back(m_theAssociatedCloud->getPointPersistentPtr(p->theIndex), p->theIndex);
							}
						}
					}
				}
			}
		}
	}
}

#ifdef TEST_CELLS_FOR_SPHERICAL_NN
void DgmOctree::getPointsInNeighbourCellsAround(NearestNeighboursSphericalSearchStruct &nNSS,
												int minNeighbourhoodLength,
												int maxNeighbourhoodLength) const
{
	assert(minNeighbourhoodLength >= nNSS.alreadyVisitedNeighbourhoodSize);

	//binary shift for cell code truncation
	unsigned char bitDec = GET_BIT_SHIFT(nNSS.level);
	CellDescriptor cellDesc;

	if (minNeighbourhoodLength == 0) //special case
	{
		//we don't look if the cell is inside the octree as it is generally the case
		CellCode truncatedCellCode = GenerateTruncatedCellCode(nNSS.cellPos,nNSS.level);
		unsigned index = getCellIndex(truncatedCellCode,bitDec);
		if (index < m_numberOfProjectedPoints)
		{
			//add cell descriptor to cells list
			cellDesc.center = CCVector3(nNSS.cellCenter);
			cellDesc.index = 0;
			nNSS.cellsInNeighbourhood.push_back(cellDesc);

			for (cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+index; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == truncatedCellCode); ++p)
			{
				PointDescriptor newPoint(m_theAssociatedCloud->getPointPersistentPtr(p->theIndex),p->theIndex);
				nNSS.pointsInSphericalNeighbourhood.push_back(newPoint);
			}
		}
		if (maxNeighbourhoodLength == 0)
			return;
		++minNeighbourhoodLength;
	}

	//get distance form cell to octree neighbourhood borders
	int limits[6];
	if (!getCellDistanceFromBorders(nNSS.cellPos,
									nNSS.level,
									maxNeighbourhoodLength,
									limits))
		return;

	int &iMinAbs = limits[0];
	int &iMaxAbs = limits[1];
	int &jMinAbs = limits[2];
	int &jMaxAbs = limits[3];
	int &kMinAbs = limits[4];
	int &kMaxAbs = limits[5];

	unsigned old_index = 0;
	CellCode old_c2 = 0;
	Tuple3i currentCellPos;

	//first part: i in [-maxNL,-minNL] and (j,k) in [-maxNL,maxNL]
	if (iMinAbs>=minNeighbourhoodLength)
	{
		for (int v0=nNSS.cellPos.x-iMinAbs; v0<=nNSS.cellPos.x-minNeighbourhoodLength; ++v0)
		{
			CellCode c0 = GenerateCellCodeForDim(v0);
			currentCellPos.x = v0;
			for (int v1=nNSS.cellPos.y-jMinAbs; v1<=nNSS.cellPos.y+jMaxAbs; ++v1)
			{
				CellCode c1 = c0 | (GenerateCellCodeForDim(v1)<<1);
				currentCellPos.y = v1;
				for (int v2=nNSS.cellPos.z-kMinAbs; v2<=nNSS.cellPos.z+kMaxAbs; ++v2)
				{
					CellCode c2 = c1 | (GenerateCellCodeForDim(v2)<<2);

					//look for corresponding cell
					unsigned index = (old_c2 < c2 ? getCellIndex(c2,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(c2,bitDec,0,old_index));
					if (index < m_numberOfProjectedPoints)
					{
						//add cell descriptor to cells list
						currentCellPos.z = v2;
						computeCellCenter(currentCellPos,nNSS.level,cellDesc.center);
						cellDesc.index = nNSS.pointsInSphericalNeighbourhood.size();
						nNSS.cellsInNeighbourhood.push_back(cellDesc);

						for (cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+index; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == c2); ++p)
						{
							PointDescriptor newPoint(m_theAssociatedCloud->getPointPersistentPtr(p->theIndex),p->theIndex);
							nNSS.pointsInSphericalNeighbourhood.push_back(newPoint);
						}

						old_index = index;
						old_c2 = c2;
					}
				}
			}
		}
		iMinAbs = minNeighbourhoodLength-1; //minNeighbourhoodLength>1
	}

	//second part: i in [minNL,maxNL] and (j,k) in [-maxNL,maxNL]
	if (iMaxAbs >= minNeighbourhoodLength)
	{
		for (int v0=nNSS.cellPos.x+minNeighbourhoodLength; v0<=nNSS.cellPos.x+iMaxAbs; ++v0)
		{
			CellCode c0 = GenerateCellCodeForDim(v0);
			currentCellPos.x = v0;
			for (int v1=nNSS.cellPos.y-jMinAbs; v1<=nNSS.cellPos.y+jMaxAbs; ++v1)
			{
				CellCode c1 = c0 | (GenerateCellCodeForDim(v1)<<1);
				currentCellPos.y = v1;
				for (int v2=nNSS.cellPos.z-kMinAbs; v2<=nNSS.cellPos.z+kMaxAbs; ++v2)
				{
					CellCode c2 = c1 | (GenerateCellCodeForDim(v2)<<2);

					//look for corresponding cell
					unsigned index = (old_c2 < c2 ? getCellIndex(c2,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(c2,bitDec,0,old_index));
					if (index < m_numberOfProjectedPoints)
					{
						//add cell descriptor to cells list
						currentCellPos.z = v2;
						computeCellCenter(currentCellPos,nNSS.level,cellDesc.center);
						cellDesc.index = nNSS.pointsInSphericalNeighbourhood.size();
						nNSS.cellsInNeighbourhood.push_back(cellDesc);

						for (cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+index; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == c2); ++p)
						{
							PointDescriptor newPoint(m_theAssociatedCloud->getPointPersistentPtr(p->theIndex),p->theIndex);
							nNSS.pointsInSphericalNeighbourhood.push_back(newPoint);
						}

						old_index = index;
						old_c2 = c2;
					}
				}
			}
		}
		iMaxAbs = minNeighbourhoodLength-1; //minNeighbourhoodLength>1
	}

	//third part: j in [-maxNL,-minNL] and (i,k) in [-maxNL,maxNL]
	if (jMinAbs>=minNeighbourhoodLength)
	{
		for (int v1=nNSS.cellPos.y-jMinAbs; v1<=nNSS.cellPos.y-minNeighbourhoodLength; ++v1)
		{
			CellCode c1 = (GenerateCellCodeForDim(v1) << 1);
			currentCellPos.y = v1;
			for (int v0=nNSS.cellPos.x-iMinAbs; v0<=nNSS.cellPos.x+iMaxAbs; ++v0)
			{
				CellCode c0 = c1 | GenerateCellCodeForDim(v0);
				currentCellPos.x = v0;
				for (int v2=nNSS.cellPos.z-kMinAbs; v2<=nNSS.cellPos.z+kMaxAbs; ++v2)
				{
					CellCode c2 = c0 | (GenerateCellCodeForDim(v2)<<2);

					//look for corresponding cell
					unsigned index = (old_c2 < c2 ? getCellIndex(c2,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(c2,bitDec,0,old_index));
					if (index < m_numberOfProjectedPoints)
					{
						//add cell descriptor to cells list
						currentCellPos.z = v2;
						computeCellCenter(currentCellPos,nNSS.level,cellDesc.center);
						cellDesc.index = nNSS.pointsInSphericalNeighbourhood.size();
						nNSS.cellsInNeighbourhood.push_back(cellDesc);

						for (cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+index; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == c2); ++p)
						{
							PointDescriptor newPoint(m_theAssociatedCloud->getPointPersistentPtr(p->theIndex),p->theIndex);
							nNSS.pointsInSphericalNeighbourhood.push_back(newPoint);
						}

						old_index = index;
						old_c2 = c2;
					}
				}
			}
		}
		jMinAbs = minNeighbourhoodLength-1; //minNeighbourhoodLength>1
	}

	//fourth part: j in [minNL,maxNL] and (i,k) in [-maxNL,maxNL]
	if (jMaxAbs>=minNeighbourhoodLength)
	{
		for (int v1=nNSS.cellPos.y+minNeighbourhoodLength; v1<=nNSS.cellPos.y+jMaxAbs; ++v1)
		{
			CellCode c1 = (GenerateCellCodeForDim(v1) << 1);
			currentCellPos.y = v1;
			for (int v0=nNSS.cellPos.x-iMinAbs; v0<=nNSS.cellPos.x+iMaxAbs; ++v0)
			{
				CellCode c0 = c1 | GenerateCellCodeForDim(v0);
				currentCellPos.x = v0;
				for (int v2=nNSS.cellPos.z-kMinAbs; v2<=nNSS.cellPos.z+kMaxAbs; ++v2)
				{
					CellCode c2 = c0 | (GenerateCellCodeForDim(v2)<<2);

					//look for corresponding cell
					unsigned index = (old_c2 < c2 ? getCellIndex(c2,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(c2,bitDec,0,old_index));
					if (index < m_numberOfProjectedPoints)
					{
						//add cell descriptor to cells list
						currentCellPos.z = v2;
						computeCellCenter(currentCellPos,nNSS.level,cellDesc.center);
						cellDesc.index = nNSS.pointsInSphericalNeighbourhood.size();
						nNSS.cellsInNeighbourhood.push_back(cellDesc);

						for (cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+index; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == c2); ++p)
						{
							PointDescriptor newPoint(m_theAssociatedCloud->getPointPersistentPtr(p->theIndex),p->theIndex);
							nNSS.pointsInSphericalNeighbourhood.push_back(newPoint);
						}

						old_index = index;
						old_c2 = c2;
					}
				}
			}
		}
		jMaxAbs = minNeighbourhoodLength-1; //minNeighbourhoodLength>1
	}

	//fifth part: k in [-maxNL,-minNL] and (i,k) in [-maxNL,maxNL]
	if (kMinAbs>=minNeighbourhoodLength)
	{
		for (int v2=nNSS.cellPos.z-kMinAbs; v2<=nNSS.cellPos.z-minNeighbourhoodLength; ++v2)
		{
			CellCode c2 = (GenerateCellCodeForDim(v2)<<2);
			currentCellPos.z = v2;
			for (int v0=nNSS.cellPos.x-iMinAbs; v0<=nNSS.cellPos.x+iMaxAbs; ++v0)
			{
				CellCode c0 = c2 | GenerateCellCodeForDim(v0);
				currentCellPos.x = v0;
				for (int v1=nNSS.cellPos.y-jMinAbs; v1<=nNSS.cellPos.y+jMaxAbs; ++v1)
				{
					CellCode c1 = c0 | (GenerateCellCodeForDim(v1)<<1);
					//look for corresponding cell
					unsigned index = (old_c2 < c1 ? getCellIndex(c1,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(c1,bitDec,0,old_index));
					if (index < m_numberOfProjectedPoints)
					{
						//add cell descriptor to cells list
						currentCellPos.y = v1;
						computeCellCenter(currentCellPos,nNSS.level,cellDesc.center);
						cellDesc.index = nNSS.pointsInSphericalNeighbourhood.size();
						nNSS.cellsInNeighbourhood.push_back(cellDesc);

						for (cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+index; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == c1); ++p)
						{
							PointDescriptor newPoint(m_theAssociatedCloud->getPointPersistentPtr(p->theIndex),p->theIndex);
							nNSS.pointsInSphericalNeighbourhood.push_back(newPoint);
						}

						old_index = index;
						old_c2=c1;
					}
				}
			}
		}
		//kMinAbs = minNeighbourhoodLength-1; //minNeighbourhoodLength>1
	}

	//sixth and last part: k in [minNL,maxNL] and (i,k) in [-maxNL,maxNL]
	if (kMaxAbs>=minNeighbourhoodLength)
	{
		for (int v2=nNSS.cellPos.z+minNeighbourhoodLength; v2<=nNSS.cellPos.z+kMaxAbs; ++v2)
		{
			CellCode c2 = (GenerateCellCodeForDim(v2)<<2);
			currentCellPos.z = v2;
			for (int v0=nNSS.cellPos.x-iMinAbs; v0<=nNSS.cellPos.x+iMaxAbs; ++v0)
			{
				CellCode c0 = c2 | GenerateCellCodeForDim(v0);
				currentCellPos.x = v0;
				for (int v1=nNSS.cellPos.y-jMinAbs; v1<=nNSS.cellPos.y+jMaxAbs; ++v1)
				{
					CellCode c1 = c0 | (GenerateCellCodeForDim(v1)<<1);
					//look for corresponding cell
					unsigned index = (old_c2 < c1 ? getCellIndex(c1,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(c1,bitDec,0,old_index));
					if (index < m_numberOfProjectedPoints)
					{
						//add cell descriptor to cells list
						currentCellPos.y = v1;
						computeCellCenter(currentCellPos,nNSS.level,cellDesc.center);
						cellDesc.index = nNSS.pointsInSphericalNeighbourhood.size();
						nNSS.cellsInNeighbourhood.push_back(cellDesc);

						for (cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+index; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == c1); ++p)
						{
							PointDescriptor newPoint(m_theAssociatedCloud->getPointPersistentPtr(p->theIndex),p->theIndex);
							nNSS.pointsInSphericalNeighbourhood.push_back(newPoint);
						}

						old_index = index;
						old_c2=c1;
					}
				}
			}
		}
		//kMaxAbs = minNeighbourhoodLength-1; //minNeighbourhoodLength>1
	}
}
#endif

double DgmOctree::findTheNearestNeighborStartingFromCell(NearestNeighboursSearchStruct &nNSS) const
{
	//binary shift for cell code truncation
	unsigned char bitDec = GET_BIT_SHIFT(nNSS.level);

	//cell size at the current level of subdivision
	const PointCoordinateType& cs = getCellSize(nNSS.level);

	//already visited cells (relative distance to the cell that includes the query point)
	int visitedCellDistance = nNSS.alreadyVisitedNeighbourhoodSize;
	//minimum (a priori) relative distance to get eligible points (see 'eligibleDist' below)
	int eligibleCellDistance = visitedCellDistance;

	//if we have not already looked for the first cell (the one including the query point)
	if (visitedCellDistance == 0)
	{
		//'visitedCellDistance == 0' means that no cell has ever been processed!
		//No cell should be inside 'minimalCellsSetToVisit'
		assert(nNSS.minimalCellsSetToVisit.empty());

		//check for existence of an 'including' cell
		CellCode truncatedCellCode = GenerateTruncatedCellCode(nNSS.cellPos, nNSS.level);
		unsigned index = (truncatedCellCode == INVALID_CELL_CODE ? m_numberOfProjectedPoints : getCellIndex(truncatedCellCode,bitDec));

		visitedCellDistance = 1;

		//it this cell does exist...
		if (index < m_numberOfProjectedPoints)
		{
			//we add it to the 'cells to visit' set
			nNSS.minimalCellsSetToVisit.push_back(index);
			eligibleCellDistance = 1;
		}
		//otherwise, we may be very far from the nearest octree cell
		//(let's try to get there asap)
		else
		{
			//fill indexes for current level
			const int* _fillIndexes = m_fillIndexes + 6*nNSS.level;
			int diagonalDistance = 0;
			for (int dim=0; dim<3; ++dim)
			{
				//distance to min border of octree along each axis
				int distToBorder = *_fillIndexes - nNSS.cellPos.u[dim];
				//if its negative, lets look the other side
				if (distToBorder < 0)
				{
					//distance to max border of octree along each axis
					distToBorder = nNSS.cellPos.u[dim] - _fillIndexes[3];
				}

				if (distToBorder > 0)
				{
					visitedCellDistance = std::max(distToBorder,visitedCellDistance);
					diagonalDistance += distToBorder*distToBorder;
				}

				//next dimension
				++_fillIndexes;
			}

			//the nearest octree cell
			diagonalDistance = static_cast<int>(ceil(sqrt(static_cast<float>(diagonalDistance))));
			eligibleCellDistance = std::max(diagonalDistance,1);

			if (nNSS.maxSearchSquareDistd > 0)
			{
				//Distance to the nearest point
				double minDist = static_cast<double>(eligibleCellDistance-1) * cs;
				//if we are already outside of the search limit, we can quit
				if (minDist*minDist > nNSS.maxSearchSquareDistd)
				{
					return -1.0;
				}
			}
		}

		//update
		nNSS.alreadyVisitedNeighbourhoodSize = visitedCellDistance;
	}

	//for each dimension, we look for the min distance between the query point and the cell border.
	//This distance (minDistToBorder) corresponds to the maximal radius of a sphere centered on the
	//query point and totally included inside the cell
	PointCoordinateType minDistToBorder = ComputeMinDistanceToCellBorder(nNSS.queryPoint, cs, nNSS.cellCenter);

	//cells for which we have already computed the distances from their points to the query point
	unsigned alreadyProcessedCells = 0;

	//Min (squared) distance of neighbours
	double minSquareDist = -1.0;

	while (true)
	{
		//if we do have found points but that were too far to be eligible
		if (minSquareDist > 0)
		{
			//what would be the correct neighbourhood size to be sure of it?
			int newEligibleCellDistance = static_cast<int>(ceil((static_cast<PointCoordinateType>(sqrt(minSquareDist)) - minDistToBorder) / cs));
			eligibleCellDistance = std::max(newEligibleCellDistance, eligibleCellDistance);
		}

		//we get the (new) cells around the current neighbourhood
		while (nNSS.alreadyVisitedNeighbourhoodSize < eligibleCellDistance) //DGM: warning, alreadyVisitedNeighbourhoodSize == 1 means that we have only visited the first cell (distance=0)
		{
			getNeighborCellsAround(nNSS.cellPos, nNSS.minimalCellsSetToVisit, nNSS.alreadyVisitedNeighbourhoodSize, nNSS.level);
			++nNSS.alreadyVisitedNeighbourhoodSize;
		}

		//we compute distances for the new points
		DgmOctree::cellIndexesContainer::const_iterator q;
		for (q = nNSS.minimalCellsSetToVisit.begin() + alreadyProcessedCells; q != nNSS.minimalCellsSetToVisit.end(); ++q)
		{
			//current cell index (== index of its first point)
			unsigned m = *q;

			//we scan the whole cell to see if it contains a closer point
			cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin() + m;
			CellCode code = (p->theCode >> bitDec);
			while (m < m_numberOfProjectedPoints && (p->theCode >> bitDec) == code)
			{
				//square distance to query point
				double dist2 = (*m_theAssociatedCloud->getPointPersistentPtr(p->theIndex) - nNSS.queryPoint).norm2d();
				//we keep track of the closest one
				if (dist2 < minSquareDist || minSquareDist < 0)
				{
					nNSS.theNearestPointIndex = p->theIndex;
					minSquareDist = dist2;
					if (dist2 == 0) //no need to process any further
						break;
				}
				++m;
				++p;
			}
		}
		alreadyProcessedCells = static_cast<unsigned>(nNSS.minimalCellsSetToVisit.size());

		//equivalent spherical neighbourhood radius (as we are actually looking to 'square' neighbourhoods,
		//we must check that the nearest points inside such neighbourhoods are indeed near enough to fall
		//inside the biggest included sphere). Otherwise we must look further.
		double eligibleDist = static_cast<double>(eligibleCellDistance - 1) * cs + minDistToBorder;
		double squareEligibleDist = eligibleDist * eligibleDist;

		//if we have found an eligible point
		if (minSquareDist >= 0 && minSquareDist <= squareEligibleDist)
		{
			if (nNSS.maxSearchSquareDistd <= 0 || minSquareDist <= nNSS.maxSearchSquareDistd)
				return minSquareDist;
			else
				return -1.0;
		}
		else
		{
			//no eligible point? Maybe we are already too far?
			if (nNSS.maxSearchSquareDistd > 0 && squareEligibleDist >= nNSS.maxSearchSquareDistd)
				return -1.0;
		}

		//default strategy: increase neighbourhood size of +1 (for next step)
		++eligibleCellDistance;
	}

	//we should never get here!
	assert(false);

	return -1.0;
}

//search for at least "minNumberOfNeighbors" points around a query point
unsigned DgmOctree::findNearestNeighborsStartingFromCell(	NearestNeighboursSearchStruct &nNSS,
															bool getOnlyPointsWithValidScalar/*=false*/) const
{
	//binary shift for cell code truncation
	unsigned char bitDec = GET_BIT_SHIFT(nNSS.level);

	//cell size at the current level of subdivision
	const PointCoordinateType& cs = getCellSize(nNSS.level);

	//already visited cells (relative distance to the cell that includes the query point)
	int visitedCellDistance=nNSS.alreadyVisitedNeighbourhoodSize;
	//minimum (a priori) relative distance to get eligible points (see 'eligibleDist' below)
	int eligibleCellDistance=visitedCellDistance;

	//shall we look inside the first cell (the one including the query point)?
	if (visitedCellDistance == 0)
	{
		//visitedCellDistance == 0 means that no cell has ever been processed! No point should be inside 'pointsInNeighbourhood'
		assert(nNSS.pointsInNeighbourhood.empty());

		//check for existence of 'including' cell
		CellCode truncatedCellCode = GenerateTruncatedCellCode(nNSS.cellPos, nNSS.level);
		unsigned index = (truncatedCellCode == INVALID_CELL_CODE ? m_numberOfProjectedPoints : getCellIndex(truncatedCellCode,bitDec));

		visitedCellDistance = 1;

		//it this cell does exist...
		if (index < m_numberOfProjectedPoints)
		{
			//we grab the points inside
			cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+index;
			while (p!=m_thePointsAndTheirCellCodes.end() && (p->theCode >> bitDec) == truncatedCellCode)
			{
				if (!getOnlyPointsWithValidScalar || ScalarField::ValidValue(m_theAssociatedCloud->getPointScalarValue(p->theIndex)))
				{
					nNSS.pointsInNeighbourhood.emplace_back(m_theAssociatedCloud->getPointPersistentPtr(p->theIndex), p->theIndex);
					++p;
				}
			}

			eligibleCellDistance = 1;
		}
		//otherwise, we may be very far from the nearest octree cell
		//(let's try to get there asap)
		else
		{
			//fill indexes for current level
			const int* _fillIndexes = m_fillIndexes + 6*nNSS.level;
			int diagonalDistance = 0;
			for (int dim=0; dim<3; ++dim)
			{
				//distance to min border of octree along each axis
				int distToBorder = *_fillIndexes - nNSS.cellPos.u[dim];
				//if its negative, lets look the other side
				if (distToBorder < 0)
				{
					//distance to max border of octree along each axis
					distToBorder = nNSS.cellPos.u[dim] - _fillIndexes[3];
				}

				if (distToBorder > 0)
				{
					visitedCellDistance = std::max(distToBorder,visitedCellDistance);
					diagonalDistance += distToBorder*distToBorder;
				}

				//next dimension
				++_fillIndexes;
			}

			//the nearest octree cell
			diagonalDistance = static_cast<int>(ceil(sqrt(static_cast<float>(diagonalDistance))));
			eligibleCellDistance = std::max(diagonalDistance,1);

			if (nNSS.maxSearchSquareDistd > 0)
			{
				//Distance of the nearest point
				double minDist = static_cast<double>(eligibleCellDistance-1) * cs;
				//if we are already outside of the search limit, we can quit
				if (minDist*minDist > nNSS.maxSearchSquareDistd)
				{
					return 0;
				}
			}
		}
	}

	//for each dimension, we look for the min distance between the query point and the cell border.
	//This distance (minDistToBorder) corresponds to the maximal radius of a sphere centered on the
	//query point and totally included inside the cell
	PointCoordinateType minDistToBorder = ComputeMinDistanceToCellBorder(nNSS.queryPoint,cs,nNSS.cellCenter);

	//eligible points found
	unsigned eligiblePoints = 0;

	//points for which we have already computed the distance to the query point
	unsigned alreadyProcessedPoints = 0;

	//Min (squared) distance of non eligible points
	double minSquareDist = -1.0;

	//while we don't have enough 'nearest neighbours'
	while (eligiblePoints<nNSS.minNumberOfNeighbors)
	{
		//if we do have found points but that were too far to be eligible
		if (minSquareDist > 0)
		{
			//what would be the correct neighbourhood size to be sure of it?
			int newEligibleCellDistance = static_cast<int>(ceil((static_cast<PointCoordinateType>(sqrt(minSquareDist))-minDistToBorder)/cs));
			eligibleCellDistance = std::max(newEligibleCellDistance,eligibleCellDistance);
		}

		//we get the (new) points lying in the added area
		while (visitedCellDistance < eligibleCellDistance) //DGM: warning, visitedCellDistance == 1 means that we have only visited the first cell (distance=0)
		{
			getPointsInNeighbourCellsAround(nNSS,visitedCellDistance,getOnlyPointsWithValidScalar);
			++visitedCellDistance;
		}

		//we compute distances for the new points
		NeighboursSet::iterator q;
		for (q = nNSS.pointsInNeighbourhood.begin()+alreadyProcessedPoints; q != nNSS.pointsInNeighbourhood.end(); ++q)
			q->squareDistd = (*q->point - nNSS.queryPoint).norm2d();
		alreadyProcessedPoints = static_cast<unsigned>(nNSS.pointsInNeighbourhood.size());

		//equivalent spherical neighbourhood radius (as we are actually looking to 'square' neighbourhoods,
		//we must check that the nearest points inside such neighbourhoods are indeed near enough to fall
		//inside the biggest included sphere). Otherwise we must look further.
		double eligibleDist = static_cast<double>(eligibleCellDistance-1) * cs + minDistToBorder;
		double squareEligibleDist = eligibleDist * eligibleDist;

		//let's test all the previous 'not yet eligible' points and the new ones
		unsigned j = eligiblePoints;
		for (q = nNSS.pointsInNeighbourhood.begin()+eligiblePoints; q != nNSS.pointsInNeighbourhood.end(); ++q,++j)
		{
			//if the point is eligible
			if (q->squareDistd <= squareEligibleDist)
			{
				if (eligiblePoints<j)
					std::swap(nNSS.pointsInNeighbourhood[eligiblePoints],nNSS.pointsInNeighbourhood[j]);
				++eligiblePoints;
			}
			//otherwise we track the nearest one
			else if (q->squareDistd < minSquareDist || j == eligiblePoints)
			{
				minSquareDist = q->squareDistd;
			}
		}

		//Maybe we are already too far?
		if (nNSS.maxSearchSquareDistd > 0 && squareEligibleDist >= nNSS.maxSearchSquareDistd)
			break;

		//default strategy: increase neighbourhood size of +1 (for next step)
		++eligibleCellDistance;
	}

	//update the neighbourhood size (for next call, if the query point lies in the same cell)
	nNSS.alreadyVisitedNeighbourhoodSize = visitedCellDistance;

	//we sort the eligible points
	std::sort(nNSS.pointsInNeighbourhood.begin(), nNSS.pointsInNeighbourhood.begin() + eligiblePoints, PointDescriptor::distComp);

	//we return the number of eligible points found
	return eligiblePoints;
}

int DgmOctree::getPointsInSphericalNeighbourhood(	const CCVector3& sphereCenter,
													PointCoordinateType radius,
													NeighboursSet& neighbours,
													unsigned char level/*=0*/) const
{
	//cell size
	const PointCoordinateType& cs = getCellSize(level);
	PointCoordinateType halfCellSize = cs/2;

	//squared radius
	double squareRadius = static_cast<double>(radius) * static_cast<double>(radius);
	//constant value for cell/sphere inclusion test
	double maxDiagFactor = squareRadius + (0.75*cs + SQRT_3*radius)*cs;

	//we are going to test all the cells that may intersect the sphere
	CCVector3 corner = sphereCenter - CCVector3(radius,radius,radius);
	Tuple3i cornerPos;
	getTheCellPosWhichIncludesThePoint(&corner, cornerPos, level);

	//don't need to look outside the octree limits!
	cornerPos.x = std::max<int>(cornerPos.x,0);
	cornerPos.y = std::max<int>(cornerPos.y,0);
	cornerPos.z = std::max<int>(cornerPos.z,0);

	//corresponding cell limits
	CCVector3 boxMin(	m_dimMin[0] + cs * cornerPos.x,
						m_dimMin[1] + cs * cornerPos.y,
						m_dimMin[2] + cs * cornerPos.z );

	//max number of cells for this dimension
	int maxCellCount = OCTREE_LENGTH(level);
	//binary shift for cell code truncation
	unsigned char bitDec = GET_BIT_SHIFT(level);

	CCVector3 cellMin = boxMin;
	Tuple3i cellPos(cornerPos.x, 0, 0);
	while (cellMin.x < sphereCenter.x + radius && cellPos.x < maxCellCount)
	{
		CCVector3 cellCenter(cellMin.x + halfCellSize, 0, 0);

		cellMin.y = boxMin.y;
		cellPos.y = cornerPos.y;
		while (cellMin.y < sphereCenter.y + radius && cellPos.y < maxCellCount)
		{
			cellCenter.y = cellMin.y + halfCellSize;

			cellMin.z = boxMin.z;
			cellPos.z = cornerPos.z;
			while (cellMin.z < sphereCenter.z + radius && cellPos.z < maxCellCount)
			{
				cellCenter.z = cellMin.z + halfCellSize;
				//test this cell
				//1st test: is it close enough to the sphere center?
				if ((cellCenter - sphereCenter).norm2d() <= maxDiagFactor) //otherwise cell is totally outside
				{
					//2nd test: does this cell exists?
					CellCode truncatedCellCode = GenerateTruncatedCellCode(cellPos, level);
					unsigned cellIndex = getCellIndex(truncatedCellCode,bitDec);

					//if yes get the corresponding points
					if (cellIndex < m_numberOfProjectedPoints)
					{
						//we look for the first index in 'm_thePointsAndTheirCellCodes' corresponding to this cell
						cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+cellIndex;
						CellCode searchCode = (p->theCode >> bitDec);

						//while the (partial) cell code matches this cell
						for ( ; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == searchCode); ++p)
						{
							const CCVector3* P = m_theAssociatedCloud->getPoint(p->theIndex);
							double d2 = (*P - sphereCenter).norm2d();
							//we keep the points falling inside the sphere
							if (d2 <= squareRadius)
							{
								neighbours.emplace_back(P, p->theIndex, d2);
							}
						}
					}
				}

				//next cell
				cellMin.z += cs;
				++cellPos.z;
			}

			//next cell
			cellMin.y += cs;
			++cellPos.y;
		}

		//next cell
		cellMin.x += cs;
		++cellPos.x;
	}

	return static_cast<int>(neighbours.size());
}

std::size_t DgmOctree::getPointsInBoxNeighbourhood(BoxNeighbourhood& params) const
{
	//cell size
	const PointCoordinateType& cs = getCellSize(params.level);

	//we are going to test all the cells that may intersect this box
	//first we extract the box... bounding box ;)
	CCVector3 minCorner;
	CCVector3 maxCorner;
	if (params.axes)
	{
		//normalize axes (just in case)
		params.axes[0].normalize();
		params.axes[1].normalize();
		params.axes[2].normalize();

		const PointCoordinateType& dx = params.dimensions.x;
		const PointCoordinateType& dy = params.dimensions.y;
		const PointCoordinateType& dz = params.dimensions.z;
		
		CCVector3 corners[8] =
		{
			CCVector3(-dx/2, -dy/2, -dz/2),
			CCVector3(-dx/2, -dy/2,	 dz/2),
			CCVector3(-dx/2,  dy/2,	 dz/2),
			CCVector3(-dx/2,  dy/2, -dz/2),
			CCVector3( dx/2, -dy/2, -dz/2),
			CCVector3( dx/2, -dy/2,	 dz/2),
			CCVector3( dx/2,  dy/2,	 dz/2),
			CCVector3( dx/2,  dy/2, -dz/2)
		};

		//position of the box vertices in the octree coordinate system
		for (unsigned char i=0; i<8; ++i)
		{
			corners[i] = corners[i].x * params.axes[0] + corners[i].y * params.axes[1] + corners[i].z * params.axes[2];
			if (i)
			{
				if (corners[i].x < minCorner.x)
					minCorner.x = corners[i].x;
				else if (corners[i].x > maxCorner.x)
					maxCorner.x = corners[i].x;

				if (corners[i].y < minCorner.y)
					minCorner.y = corners[i].y;
				else if (corners[i].y > maxCorner.y)
					maxCorner.y = corners[i].y;

				if (corners[i].z < minCorner.z)
					minCorner.z = corners[i].z;
				else if (corners[i].z > maxCorner.z)
					maxCorner.z = corners[i].z;
			}
			else
			{
				minCorner = maxCorner = corners[0];
			}
		}

		//up to now min and max corners where centered on (0,0,0)
		minCorner = params.center + minCorner;
		maxCorner = params.center + maxCorner;
	}
	else
	{
		minCorner = params.center - params.dimensions/2;
		maxCorner = params.center + params.dimensions/2;
	}

	Tuple3i minCornerPos;
	getTheCellPosWhichIncludesThePoint(&minCorner, minCornerPos, params.level);
	Tuple3i maxCornerPos;
	getTheCellPosWhichIncludesThePoint(&maxCorner, maxCornerPos, params.level);

	const int* minFillIndexes = getMinFillIndexes(params.level);
	const int* maxFillIndexes = getMaxFillIndexes(params.level);

	//don't need to look outside the octree limits!
	minCornerPos.x = std::max<int>(minCornerPos.x, minFillIndexes[0]);
	minCornerPos.y = std::max<int>(minCornerPos.y, minFillIndexes[1]);
	minCornerPos.z = std::max<int>(minCornerPos.z, minFillIndexes[2]);

	maxCornerPos.x = std::min<int>(maxCornerPos.x, maxFillIndexes[0]);
	maxCornerPos.y = std::min<int>(maxCornerPos.y, maxFillIndexes[1]);
	maxCornerPos.z = std::min<int>(maxCornerPos.z, maxFillIndexes[2]);

	//half cell diagonal
	CCVector3 boxHalfDimensions = params.dimensions / 2;
	CCVector3 maxHalfDist = boxHalfDimensions;
	if (params.axes)
	{
		PointCoordinateType halfDiag = static_cast<PointCoordinateType>(cs * sqrt(3.0)/2.0);
		maxHalfDist += CCVector3(halfDiag, halfDiag, halfDiag);
	}

	//binary shift for cell code truncation
	unsigned char bitDec = GET_BIT_SHIFT(params.level);

	for (int i=minCornerPos.x; i<=maxCornerPos.x; ++i)
	{
		for (int j=minCornerPos.y; j<=maxCornerPos.y; ++j)
		{
			for (int k=minCornerPos.z; k<=maxCornerPos.z; ++k)
			{
				//additional inclusion test
				if (params.axes)
				{
					CCVector3 cellCenter = m_dimMin + CCVector3(static_cast<PointCoordinateType>(i + 0.5),
																static_cast<PointCoordinateType>(j + 0.5),
																static_cast<PointCoordinateType>(k + 0.5)) * cs;

					//project the cell center in the box C.S.
					CCVector3 Q = cellCenter - params.center;
					Q = CCVector3(	params.axes[0].dot(Q),
									params.axes[1].dot(Q),
									params.axes[2].dot(Q) );

					//rough inclusion test
					if (	std::abs(Q.x) > maxHalfDist.x
						||	std::abs(Q.y) > maxHalfDist.y
						||	std::abs(Q.z) > maxHalfDist.z )
					{
						//skip this cell
						continue;
					}
				}

				//test if this cell exists
				Tuple3i cellPos(i, j, k);
				CellCode truncatedCellCode = GenerateTruncatedCellCode(cellPos, params.level);
				unsigned cellIndex = getCellIndex(truncatedCellCode,bitDec);

				//if yes, we can test the corresponding points
				if (cellIndex < m_numberOfProjectedPoints)
				{
					//we look for the first index in 'm_thePointsAndTheirCellCodes' corresponding to this cell
					cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+cellIndex;
					CellCode searchCode = (p->theCode >> bitDec);

					//while the (partial) cell code matches this cell
					for ( ; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == searchCode); ++p)
					{
						const CCVector3* P = m_theAssociatedCloud->getPoint(p->theIndex);
						CCVector3 Q = *P - params.center;

						if (params.axes)
						{
							//project the point in the box C.S.
							Q = CCVector3(	params.axes[0].dot(Q),
											params.axes[1].dot(Q),
											params.axes[2].dot(Q) );
						}

						//we keep the points that fall inside the box
						if (	std::abs(Q.x) <= boxHalfDimensions.x
							&&	std::abs(Q.y) <= boxHalfDimensions.y
							&&	std::abs(Q.z) <= boxHalfDimensions.z )
						{
							params.neighbours.emplace_back(P, p->theIndex, 0);
						}
					}
				}
			}
		}
	}

	return params.neighbours.size();
}

std::size_t DgmOctree::getPointsInCylindricalNeighbourhood(CylindricalNeighbourhood& params) const
{
	//cell size
	const PointCoordinateType& cs = getCellSize(params.level);
	PointCoordinateType halfCellSize = cs/2;

	//squared radius
	double squareRadius = static_cast<double>(params.radius) * static_cast<double>(params.radius);
	//constant value for cell/sphere inclusion test
	double maxDiagFactor = squareRadius + (0.75*cs + SQRT_3*params.radius)*cs;
	PointCoordinateType maxLengthFactor = params.maxHalfLength + static_cast<PointCoordinateType>(cs*SQRT_3/2);
	PointCoordinateType minLengthFactor = params.onlyPositiveDir ? 0 : -maxLengthFactor;
	
	PointCoordinateType minHalfLength = params.onlyPositiveDir ? 0 : -params.maxHalfLength;

	//we are going to test all the cells that may intersect this cylinder
	//dumb bounding-box estimation: place two spheres at the ends of the cylinder
	CCVector3 minCorner;
	CCVector3 maxCorner;
	{
		CCVector3 C1 = params.center + params.dir * params.maxHalfLength;
		CCVector3 C2 = params.center + params.dir * minHalfLength;
		CCVector3 corner1 = C1 - CCVector3(params.radius,params.radius,params.radius);
		CCVector3 corner2 = C1 + CCVector3(params.radius,params.radius,params.radius);
		CCVector3 corner3 = C2 - CCVector3(params.radius,params.radius,params.radius);
		CCVector3 corner4 = C2 + CCVector3(params.radius,params.radius,params.radius);

		minCorner.x = std::min(std::min(corner1.x,corner2.x),std::min(corner3.x,corner4.x));
		minCorner.y = std::min(std::min(corner1.y,corner2.y),std::min(corner3.y,corner4.y));
		minCorner.z = std::min(std::min(corner1.z,corner2.z),std::min(corner3.z,corner4.z));

		maxCorner.x = std::max(std::max(corner1.x,corner2.x),std::max(corner3.x,corner4.x));
		maxCorner.y = std::max(std::max(corner1.y,corner2.y),std::max(corner3.y,corner4.y));
		maxCorner.z = std::max(std::max(corner1.z,corner2.z),std::max(corner3.z,corner4.z));
	}

	Tuple3i cornerPos;
	getTheCellPosWhichIncludesThePoint(&minCorner, cornerPos, params.level);

	const int* minFillIndexes = getMinFillIndexes(params.level);
	const int* maxFillIndexes = getMaxFillIndexes(params.level);

	//don't need to look outside the octree limits!
	cornerPos.x = std::max<int>(cornerPos.x,minFillIndexes[0]);
	cornerPos.y = std::max<int>(cornerPos.y,minFillIndexes[1]);
	cornerPos.z = std::max<int>(cornerPos.z,minFillIndexes[2]);

	//corresponding cell limits
	CCVector3 boxMin(	m_dimMin[0] + cs*static_cast<PointCoordinateType>(cornerPos.x),
						m_dimMin[1] + cs*static_cast<PointCoordinateType>(cornerPos.y),
						m_dimMin[2] + cs*static_cast<PointCoordinateType>(cornerPos.z) );

	//binary shift for cell code truncation
	unsigned char bitDec = GET_BIT_SHIFT(params.level);

	CCVector3 cellMin = boxMin;
	Tuple3i cellPos( cornerPos.x, 0, 0 );
	while (cellMin.x < maxCorner.x && cellPos.x <= maxFillIndexes[0])
	{
		CCVector3 cellCenter(cellMin.x + halfCellSize, 0, 0);

		cellMin.y = boxMin.y;
		cellPos.y = cornerPos.y;
		while (cellMin.y < maxCorner.y && cellPos.y <= maxFillIndexes[1])
		{
			cellCenter.y = cellMin.y + halfCellSize;

			cellMin.z = boxMin.z;
			cellPos.z = cornerPos.z;
			while (cellMin.z < maxCorner.z && cellPos.z <= maxFillIndexes[2])
			{
				cellCenter.z = cellMin.z + halfCellSize;
				//test this cell
				//1st test: is it close enough to the cylinder axis?
				CCVector3 OC = (cellCenter - params.center);
				PointCoordinateType dot = OC.dot(params.dir);
				double d2 = (OC - params.dir * dot).norm2d();
				if (d2 <= maxDiagFactor && dot <= maxLengthFactor && dot >= minLengthFactor) //otherwise cell is totally outside
				{
					//2nd test: does this cell exists?
					CellCode truncatedCellCode = GenerateTruncatedCellCode(cellPos, params.level);
					unsigned cellIndex = getCellIndex(truncatedCellCode,bitDec);

					//if yes get the corresponding points
					if (cellIndex < m_numberOfProjectedPoints)
					{
						//we look for the first index in 'm_thePointsAndTheirCellCodes' corresponding to this cell
						cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+cellIndex;
						CellCode searchCode = (p->theCode >> bitDec);

						//while the (partial) cell code matches this cell
						for ( ; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == searchCode); ++p)
						{
							const CCVector3* P = m_theAssociatedCloud->getPoint(p->theIndex);

							//we keep the points falling inside the sphere
							CCVector3 OP = (*P - params.center);
							dot = OP.dot(params.dir);
							d2 = (OP - params.dir * dot).norm2d();
							if (d2 <= squareRadius && dot >= minHalfLength && dot <= params.maxHalfLength)
							{
								params.neighbours.emplace_back(P, p->theIndex, dot); //we save the distance relatively to the center projected on the axis!
							}
						}
					}
				}

				//next cell
				cellMin.z += cs;
				++cellPos.z;
			}

			//next cell
			cellMin.y += cs;
			++cellPos.y;
		}

		//next cell
		cellMin.x += cs;
		++cellPos.x;
	}

	return params.neighbours.size();
}

std::size_t DgmOctree::getPointsInCylindricalNeighbourhoodProgressive(ProgressiveCylindricalNeighbourhood& params) const
{
	//cell size
	const PointCoordinateType& cs = getCellSize(params.level);
	PointCoordinateType halfCellSize = cs/2;

	//squared radius
	double squareRadius = static_cast<double>(params.radius) * static_cast<double>(params.radius);
	//constant value for cell/sphere inclusion test
	double maxDiagFactor = squareRadius + (0.75*cs + SQRT_3*params.radius)*cs;
	PointCoordinateType maxLengthFactor = params.maxHalfLength + static_cast<PointCoordinateType>(cs*SQRT_3/2);
	PointCoordinateType minLengthFactor = params.onlyPositiveDir ? 0 : -maxLengthFactor;

	//increase the search cylinder's height
	params.currentHalfLength += params.radius;
	//no need to chop the max cylinder if the parts are too small!
	//(takes also into account any 'overflow' above maxHalfLength ;)
	if (params.maxHalfLength-params.currentHalfLength < params.radius/2)
		params.currentHalfLength = params.maxHalfLength;

	PointCoordinateType currentHalfLengthMinus = params.onlyPositiveDir ? 0 : -params.currentHalfLength;

	//first process potential candidates from the previous pass
	{
		for (std::size_t k=0; k<params.potentialCandidates.size(); /*++k*/)
		{
			//potentialCandidates[k].squareDist = 'dot'!
			if (	params.potentialCandidates[k].squareDistd >= currentHalfLengthMinus
				&&	params.potentialCandidates[k].squareDistd <= params.currentHalfLength)
			{
				params.neighbours.push_back(params.potentialCandidates[k]);
				//and remove it from the potential list
				std::swap(params.potentialCandidates[k],params.potentialCandidates.back());
				params.potentialCandidates.pop_back();
			}
			else
			{
				++k;
			}
		}
	}

	//we are going to test all the cells that may intersect this cylinder
	//dumb bounding-box estimation: place two spheres at the ends of the cylinder
	CCVector3 minCorner;
	CCVector3 maxCorner;
	{
		CCVector3 C1 = params.center + params.dir * params.currentHalfLength;
		CCVector3 C2 = params.center + params.dir * currentHalfLengthMinus;
		CCVector3 corner1 = C1 - CCVector3(params.radius,params.radius,params.radius);
		CCVector3 corner2 = C1 + CCVector3(params.radius,params.radius,params.radius);
		CCVector3 corner3 = C2 - CCVector3(params.radius,params.radius,params.radius);
		CCVector3 corner4 = C2 + CCVector3(params.radius,params.radius,params.radius);

		minCorner.x = std::min(std::min(corner1.x,corner2.x),std::min(corner3.x,corner4.x));
		minCorner.y = std::min(std::min(corner1.y,corner2.y),std::min(corner3.y,corner4.y));
		minCorner.z = std::min(std::min(corner1.z,corner2.z),std::min(corner3.z,corner4.z));

		maxCorner.x = std::max(std::max(corner1.x,corner2.x),std::max(corner3.x,corner4.x));
		maxCorner.y = std::max(std::max(corner1.y,corner2.y),std::max(corner3.y,corner4.y));
		maxCorner.z = std::max(std::max(corner1.z,corner2.z),std::max(corner3.z,corner4.z));
	}

	Tuple3i cornerPos;
	getTheCellPosWhichIncludesThePoint(&minCorner, cornerPos, params.level);

	const int* minFillIndexes = getMinFillIndexes(params.level);
	const int* maxFillIndexes = getMaxFillIndexes(params.level);

	//don't need to look outside the octree limits!
	cornerPos.x = std::max<int>(cornerPos.x,minFillIndexes[0]);
	cornerPos.y = std::max<int>(cornerPos.y,minFillIndexes[1]);
	cornerPos.z = std::max<int>(cornerPos.z,minFillIndexes[2]);

	//corresponding cell limits
	CCVector3 boxMin(	m_dimMin[0] + cs*static_cast<PointCoordinateType>(cornerPos.x),
						m_dimMin[1] + cs*static_cast<PointCoordinateType>(cornerPos.y),
						m_dimMin[2] + cs*static_cast<PointCoordinateType>(cornerPos.z) );

	//binary shift for cell code truncation
	unsigned char bitDec = GET_BIT_SHIFT(params.level);

	Tuple3i cellPos(cornerPos.x, 0, 0);
	CCVector3 cellMin = boxMin;
	while (cellMin.x < maxCorner.x && cellPos.x <= maxFillIndexes[0])
	{
		CCVector3 cellCenter(cellMin.x + halfCellSize, 0, 0);

		cellMin.y = boxMin.y;
		cellPos.y = cornerPos.y;
		while (cellMin.y < maxCorner.y && cellPos.y <= maxFillIndexes[1])
		{
			cellCenter.y = cellMin.y + halfCellSize;

			cellMin.z = boxMin.z;
			cellPos.z = cornerPos.z;
			while (cellMin.z < maxCorner.z && cellPos.z <= maxFillIndexes[2])
			{
				cellCenter.z = cellMin.z + halfCellSize;

				//don't test already tested cells!
				if (	cellPos.x < params.prevMinCornerPos.x || cellPos.x >= params.prevMaxCornerPos.x
					||	cellPos.y < params.prevMinCornerPos.y || cellPos.y >= params.prevMaxCornerPos.y
					||	cellPos.z < params.prevMinCornerPos.z || cellPos.z >= params.prevMaxCornerPos.z )
				{
					//test this cell
					//1st test: is it close enough to the cylinder axis?
					CCVector3 OC = (cellCenter - params.center);
					PointCoordinateType dot = OC.dot(params.dir);
					double d2 = (OC - params.dir * dot).norm2d();
					if (d2 <= maxDiagFactor && dot <= maxLengthFactor && dot >= minLengthFactor) //otherwise cell is totally outside
					{
						//2nd test: does this cell exists?
						CellCode truncatedCellCode = GenerateTruncatedCellCode(cellPos, params.level);
						unsigned cellIndex = getCellIndex(truncatedCellCode,bitDec);

						//if yes get the corresponding points
						if (cellIndex < m_numberOfProjectedPoints)
						{
							//we look for the first index in 'm_thePointsAndTheirCellCodes' corresponding to this cell
							cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+cellIndex;
							CellCode searchCode = (p->theCode >> bitDec);

							//while the (partial) cell code matches this cell
							for ( ; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == searchCode); ++p)
							{
								const CCVector3* P = m_theAssociatedCloud->getPoint(p->theIndex);

								//we keep the points falling inside the sphere
								CCVector3 OP = (*P - params.center);
								dot = OP.dot(params.dir);
								d2 = (OP - params.dir * dot).norm2d();
								if (d2 <= squareRadius)
								{
									//potential candidate?
									if (dot >= currentHalfLengthMinus && dot <= params.currentHalfLength)
									{
										params.neighbours.emplace_back(P, p->theIndex, dot); //we save the distance relatively to the center projected on the axis!
									}
									else if (params.currentHalfLength < params.maxHalfLength)
									{
										//we still keep it in the 'potential candidates' list
										params.potentialCandidates.emplace_back(P, p->theIndex, dot); //we save the distance relatively to the center projected on the axis!
									}
								}
							}
						}
					}
				}

				//next cell
				cellMin.z += cs;
				++cellPos.z;
			}

			//next cell
			cellMin.y += cs;
			++cellPos.y;
		}

		//next cell
		cellMin.x += cs;
		++cellPos.x;
	}

	params.prevMinCornerPos = cornerPos;
	params.prevMaxCornerPos = cellPos;

	return params.neighbours.size();
}

#ifdef COMPUTE_NN_SEARCH_STATISTICS
static double s_skippedPoints = 0.0;
static double s_testedPoints = 0.0;
#endif

//search for all neighbors inside a sphere
//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors!
int DgmOctree::findNeighborsInASphereStartingFromCell(NearestNeighboursSphericalSearchStruct &nNSS, double radius, bool sortValues) const
{
#ifdef TEST_CELLS_FOR_SPHERICAL_NN
	if (!nNSS.ready)
	{
		//current level cell size
		const PointCoordinateType& cs=getCellSize(nNSS.level);

		//we deduce the minimum cell neighbourhood size (integer) that includes the search sphere
		//for ANY point in the cell
		int minNeighbourhoodSize = static_cast<int>(ceil(radius/cs+SQRT_3/2.0));

		nNSS.cellsInNeighbourhood.reserve(minNeighbourhoodSize*minNeighbourhoodSize*minNeighbourhoodSize);

		if (nNSS.alreadyVisitedNeighbourhoodSize == 1 && nNSS.cellsInNeighbourhood.empty() && !nNSS.pointsInNeighbourhood.empty())
		{
			//in this case, we assume the points already in 'pointsInNeighbourhood' are the 1st cell points
			nNSS.cellsInNeighbourhood.push_back(CellDescriptor(nNSS.cellCenter,0));
			nNSS.pointsInSphericalNeighbourhood = nNSS.pointsInNeighbourhood;
		}

		getPointsInNeighbourCellsAround(nNSS,nNSS.alreadyVisitedNeighbourhoodSize,minNeighbourhoodSize);

		if (nNSS.pointsInNeighbourhood.size()<nNSS.pointsInSphericalNeighbourhood.size())
			nNSS.pointsInNeighbourhood.resize(nNSS.pointsInSphericalNeighbourhood.size());

		//don't forget to update the visited neighbourhood size!
		nNSS.alreadyVisitedNeighbourhoodSize = minNeighbourhoodSize+1;

		nNSS.ready = true;
	}
#else
	//current level cell size
	const PointCoordinateType& cs = getCellSize(nNSS.level);

	//we compute the minimal distance between the query point and all cell borders
	const PointCoordinateType minDistToBorder = ComputeMinDistanceToCellBorder(nNSS.queryPoint,cs,nNSS.cellCenter);

	//we deduce the minimum cell neighbourhood size (integer) that includes the search sphere
	const int minNeighbourhoodSize = 1+(radius>minDistToBorder ? static_cast<int>(ceil((radius-minDistToBorder)/cs)) : 0);

	//if we don't have visited such a neighbourhood...
	if (nNSS.alreadyVisitedNeighbourhoodSize<minNeighbourhoodSize)
	{
		//... let's look for the corresponding points
		for (int i=nNSS.alreadyVisitedNeighbourhoodSize; i<minNeighbourhoodSize; ++i)
			getPointsInNeighbourCellsAround(nNSS,i);

		//don't forget to update the visited neighbourhood size!
		nNSS.alreadyVisitedNeighbourhoodSize = minNeighbourhoodSize;
	}
#endif

	//squared distances comparison is faster!
	const double squareRadius = radius * radius;
	unsigned numberOfEligiblePoints = 0;

#ifdef TEST_CELLS_FOR_SPHERICAL_NN
	//cell limit relatively to sphere tight bounding box
	//const PointCoordinateType& half_cs=getCellSize(nNSS.level+1); //half cell size at current level
	//CCVector3 limitMin = nNSS.queryPoint-CCVector3(radius,radius,radius)-CCVector3(half_cs,half_cs,half_cs);
	//CCVector3 limitMax = nNSS.queryPoint+CCVector3(radius,radius,radius)+CCVector3(half_cs,half_cs,half_cs);

	//cell by cell scan
	for (NeighbourCellsSet::iterator c = nNSS.cellsInNeighbourhood.begin(); c!=nNSS.cellsInNeighbourhood.end(); ++c)
	{
		//we check the cell bounding box
		/*if (limitMax.x < c->center.x || limitMin.x > c->center.x
		|| limitMax.y < c->center.y || limitMin.y > c->center.y
		|| limitMax.z < c->center.z || limitMin.z > c->center.z)
		continue; //sphere totally outside the cell
		//*/

		//distance between the new cell center and the sphere center
		PointCoordinateType d2 = (c->center - nNSS.queryPoint).norm2();

		//is this cell totally out of the sphere?
		if (d2 <= nNSS.minOutD2)
		{
			NeighboursSet::iterator p = nNSS.pointsInSphericalNeighbourhood.begin()+c->index;
			unsigned count = ((c+1) != nNSS.cellsInNeighbourhood.end() ? (c+1)->index : nNSS.pointsInSphericalNeighbourhood.size()) - c->index;
			if (!sortValues && d2 <= nNSS.maxInD2) //totally inside? (+ we can skip distances computation)
			{
				//... we had them to the 'eligible points' part of the container
				std::copy(p,p+count,nNSS.pointsInNeighbourhood.begin()+numberOfEligiblePoints);
				numberOfEligiblePoints += count;
#ifdef COMPUTE_NN_SEARCH_STATISTICS
				s_skippedPoints += static_cast<double>(count);
#endif
			}
			else
			{
				for (unsigned j=0; j<count; ++j,++p)
				{
					p->squareDist = (*p->point - nNSS.queryPoint).norm2();
#ifdef COMPUTE_NN_SEARCH_STATISTICS
					s_testedPoints += 1.0;
#endif
					//if the distance is inferior to the sphere radius...
					if (p->squareDistd <= squareRadius)
					{
						//... we had it to the 'eligible points' part of the container
						nNSS.pointsInNeighbourhood[numberOfEligiblePoints++] = *p;
					}
				}
			}
		}
		//else cell is totally outside
		{
#ifdef COMPUTE_NN_SEARCH_STATISTICS
			unsigned count = ((c+1) != nNSS.cellsInNeighbourhood.end() ? (c+1)->index : nNSS.pointsInSphericalNeighbourhood.size()) - c->index;
			s_skippedPoints += static_cast<double>(count);
#endif
		}
	}

#else //TEST_CELLS_FOR_SPHERICAL_NN

	//point by point scan
	std::size_t i = 0;
	
	for ( PointDescriptor &pDescr : nNSS.pointsInNeighbourhood )
	{
		pDescr.squareDistd = (*pDescr.point - nNSS.queryPoint).norm2d();
		
		//if the distance is inferior to the sphere radius...
		if (pDescr.squareDistd <= squareRadius)
		{
			//... we add it to the 'eligible points' part of the container
			if (i > numberOfEligiblePoints)
			{
				std::swap(nNSS.pointsInNeighbourhood[i], nNSS.pointsInNeighbourhood[numberOfEligiblePoints]);
			}

			++numberOfEligiblePoints;
			
#ifdef COMPUTE_NN_SEARCH_STATISTICS
			s_testedPoints += 1.0;
#endif
		}
		
		++i;
	}

#endif //!TEST_CELLS_FOR_SPHERICAL_NN

	//eventually (if requested) we sort the eligible points
	if (sortValues && numberOfEligiblePoints > 0)
	{
		std::sort(nNSS.pointsInNeighbourhood.begin(), nNSS.pointsInNeighbourhood.begin() + numberOfEligiblePoints, PointDescriptor::distComp);
	}

	//return the number of eligible points
	return numberOfEligiblePoints;
}

unsigned char DgmOctree::findBestLevelForAGivenNeighbourhoodSizeExtraction(PointCoordinateType radius) const
{
	static const PointCoordinateType c_neighbourhoodSizeExtractionFactor = static_cast<PointCoordinateType>(2.5);
	PointCoordinateType aim = std::max<PointCoordinateType>(0, radius / c_neighbourhoodSizeExtractionFactor);
	
	int level = 1;
	PointCoordinateType minValue = getCellSize(1) - aim;
	minValue *= minValue;
	for (int i = 2; i <= MAX_OCTREE_LEVEL; ++i)
	{
		//we need two points per cell ideally
		if (m_averageCellPopulation[i] < 1.5)
			break;

		//The level with cell size as near as possible to the aim
		PointCoordinateType cellSizeDelta = getCellSize(i) - aim;
		cellSizeDelta *= cellSizeDelta;

		if (cellSizeDelta < minValue)
		{
			level = i;
			minValue = cellSizeDelta;
		}
	}

	return static_cast<unsigned char>(level);
}

unsigned char DgmOctree::findBestLevelForComparisonWithOctree(const DgmOctree* theOtherOctree) const
{
	unsigned ptsA = getNumberOfProjectedPoints();
	unsigned ptsB = theOtherOctree->getNumberOfProjectedPoints();

	unsigned char maxOctreeLevel = MAX_OCTREE_LEVEL;
	
	if (std::min(ptsA,ptsB) < 16)
		maxOctreeLevel = std::min(maxOctreeLevel, static_cast<unsigned char>(5)); //very small clouds
	else if (std::max(ptsA,ptsB) < 2000000)
		maxOctreeLevel = std::min(maxOctreeLevel, static_cast<unsigned char>(10)); //average size clouds

	double estimatedTime[MAX_OCTREE_LEVEL]{};
	unsigned char bestLevel = 1;
	
	for (int i=1; i<maxOctreeLevel; ++i) //warning: i >= 1
	{
		int diffA = 0;
		int diffB = 0;
		int cellsA = 0;
		int cellsB = 0;
		
		bool success = diff( i,
							 m_thePointsAndTheirCellCodes, theOtherOctree->m_thePointsAndTheirCellCodes,
							 diffA, diffB,
							 cellsA, cellsB );

		if ( !success )
		{
			continue;
		}
		
		//we use a linear model for prediction
		estimatedTime[i] = ((static_cast<double>(ptsA)*ptsB) / cellsB) * 0.001 + diffA;

		if (estimatedTime[i] < estimatedTime[bestLevel])
		{
			bestLevel = i;
		}
	}

	return bestLevel;
}

unsigned char DgmOctree::findBestLevelForAGivenPopulationPerCell(unsigned indicativeNumberOfPointsPerCell) const
{
	for (unsigned char level = MAX_OCTREE_LEVEL; level > 0; --level)
	{
		if (m_averageCellPopulation[level] > indicativeNumberOfPointsPerCell) //density can only increase. If it's above the target, no need to look further
		{
			//we take the closest match between this level and the previous one
			if (level == MAX_OCTREE_LEVEL || (m_averageCellPopulation[level] - indicativeNumberOfPointsPerCell <= indicativeNumberOfPointsPerCell - m_averageCellPopulation[level + 1])) //by definition "m_averageCellPopulation[level + 1] <= indicativeNumberOfPointsPerCell"
			{
				return level;
			}
			else
			{
				return level + 1;
			}
		}
	}

	return 1;
}

unsigned char DgmOctree::findBestLevelForAGivenCellNumber(unsigned indicativeNumberOfCells) const
{
	//we look for the level giviing the number of points per cell as close to the query
	unsigned char bestLevel=1;
	//number of cells for this level
	int n = getCellNumber(bestLevel);
	//error relatively to the query
	int oldd = abs(n-static_cast<int>(indicativeNumberOfCells));

	n = getCellNumber(bestLevel+1);
	int d = abs(n-static_cast<int>(indicativeNumberOfCells));

	while (d < oldd && bestLevel < MAX_OCTREE_LEVEL)
	{
		++bestLevel;
		oldd = d;
		n = getCellNumber(bestLevel+1);
		d = abs(n-static_cast<int>(indicativeNumberOfCells));
	}

	return bestLevel;
}

double DgmOctree::computeMeanOctreeDensity(unsigned char level) const
{
	return static_cast<double>(m_numberOfProjectedPoints)/static_cast<double>(getCellNumber(level));
}

bool DgmOctree::getCellCodesAndIndexes(unsigned char level, cellsContainer& vec, bool truncatedCodes/*=false*/) const
{
	try
	{
		//binary shift for cell code truncation
		unsigned char bitDec = GET_BIT_SHIFT(level);

		cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin();

		CellCode predCode = (p->theCode >> bitDec)+1; //pred value must be different than the first element's

		for (unsigned i = 0; i < m_numberOfProjectedPoints; ++i, ++p)
		{
			CellCode currentCode = (p->theCode >> bitDec);

			if (predCode != currentCode)
				vec.emplace_back(i, truncatedCodes ? currentCode : p->theCode);

			predCode = currentCode;
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
	return true;
}

bool DgmOctree::getCellCodes(unsigned char level, cellCodesContainer& vec, bool truncatedCodes/*=false*/) const
{
	try
	{
		//binary shift for cell code truncation
		unsigned char bitDec = GET_BIT_SHIFT(level);

		cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin();

		CellCode predCode = (p->theCode >> bitDec)+1; //pred value must be different than the first element's

		for (unsigned i=0; i<m_numberOfProjectedPoints; ++i,++p)
		{
			CellCode currentCode = (p->theCode >> bitDec);

			if (predCode != currentCode)
				vec.push_back(truncatedCodes ? currentCode : p->theCode);

			predCode = currentCode;
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
	return true;
}

bool DgmOctree::getCellIndexes(unsigned char level, cellIndexesContainer& vec) const
{
	try
	{
		vec.resize(m_cellCount[level]);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	//binary shift for cell code truncation
	unsigned char bitDec = GET_BIT_SHIFT(level);

	cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin();

	CellCode predCode = (p->theCode >> bitDec)+1; //pred value must be different than the first element's

	for (unsigned i=0,j=0; i<m_numberOfProjectedPoints; ++i,++p)
	{
		CellCode currentCode = (p->theCode >> bitDec);

		if (predCode != currentCode)
			vec[j++] = i;

		predCode = currentCode;
	}

	return true;
}

bool DgmOctree::getPointsInCellByCellIndex( ReferenceCloud* cloud,
											unsigned cellIndex,
											unsigned char level,
											bool clearOutputCloud/* = true*/) const
{
	assert(cloud && cloud->getAssociatedCloud() == m_theAssociatedCloud);

	//binary shift for cell code truncation
	unsigned char bitDec = GET_BIT_SHIFT(level);

	//we look for the first index in 'm_thePointsAndTheirCellCodes' corresponding to this cell
	cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+cellIndex;
	CellCode searchCode = (p->theCode >> bitDec);

	if (clearOutputCloud)
	{
		cloud->clear();
	}

	//while the (partial) cell code matches this cell
	while ((p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == searchCode))
	{
		if (!cloud->addPointIndex(p->theIndex))
			return false;
		++p;
	}

	return true;
}

ReferenceCloud* DgmOctree::getPointsInCellsWithSortedCellCodes( cellCodesContainer& cellCodes,
																unsigned char level,
																ReferenceCloud* subset,
																bool areCodesTruncated/*=false*/) const
{
	assert(subset);

	//binary shift for cell code truncation
	unsigned char bitDec1 = GET_BIT_SHIFT(level); //shift for this octree codes
	unsigned char bitDec2 = (areCodesTruncated ? 0 : bitDec1); //shift for the input codes

	cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin();
	CellCode toExtractCode;
	CellCode currentCode = (p->theCode >> bitDec1); //pred value must be different than the first element's

	subset->clear();

	cellCodesContainer::const_iterator q=cellCodes.begin();
	unsigned ind_p = 0;
	while (ind_p<m_numberOfProjectedPoints)
	{
		//we skip codes while the searched code is below the current one
		while (((toExtractCode = (*q >> bitDec2)) < currentCode) && (q != cellCodes.end()))
			++q;

		if (q == cellCodes.end())
			break;

		//now we skip current codes to catch the search one!
		while ((ind_p < m_numberOfProjectedPoints) && (currentCode <= toExtractCode))
		{
			if (currentCode == toExtractCode)
				subset->addPointIndex(p->theIndex);

			++p;
			if (++ind_p < m_numberOfProjectedPoints)
				currentCode = p->theCode >> bitDec1;
		}
	}

	return subset;
}


void DgmOctree::diff(const cellCodesContainer& codesA, const cellCodesContainer& codesB, cellCodesContainer& diffA, cellCodesContainer& diffB) const
{
	if (codesA.empty() && codesB.empty())
		return;

	cellCodesContainer::const_iterator pA = codesA.begin();
	cellCodesContainer::const_iterator pB = codesB.begin();

	//cell codes should already be sorted!
	while (pA != codesA.end() && pB != codesB.end())
	{
		if (*pA < *pB)
			diffA.push_back(*pA++);
		else if (*pA > *pB)
			diffB.push_back(*pB++);
		else
		{
			++pA;
			++pB;
		}
	}

	while (pA != codesA.end())
		diffA.push_back(*pA++);
	while (pB != codesB.end())
		diffB.push_back(*pB++);
}

bool DgmOctree::diff(unsigned char octreeLevel, const cellsContainer &codesA, const cellsContainer &codesB, int &diffA, int &diffB, int &cellsA, int &cellsB) const
{
	diffA = 0;
	diffB = 0;
	cellsA = 0;
	cellsB = 0;

	if (codesA.empty() && codesB.empty())
	{
		return false;
	}
	
	cellsContainer::const_iterator pA = codesA.begin();
	cellsContainer::const_iterator pB = codesB.begin();

	//binary shift for cell code truncation
	unsigned char bitDec = GET_BIT_SHIFT(octreeLevel);

	CellCode predCodeA = pA->theCode >> bitDec;
	CellCode predCodeB = pB->theCode >> bitDec;

	CellCode currentCodeA = 0;
	CellCode currentCodeB = 0;

	//cell codes should already be sorted!
	while ((pA != codesA.end()) && (pB != codesB.end()))
	{
		if (predCodeA < predCodeB)
		{
			++diffA;
			++cellsA;
			while ((pA != codesA.end()) && ((currentCodeA = (pA->theCode >> bitDec)) == predCodeA)) ++pA;
			predCodeA = currentCodeA;
		}
		else if (predCodeA > predCodeB)
		{
			++diffB;
			++cellsB;
			while ((pB != codesB.end()) && ((currentCodeB = (pB->theCode >> bitDec)) == predCodeB)) ++pB;
			predCodeB = currentCodeB;
		}
		else
		{
			while ((pA != codesA.end()) && ((currentCodeA = (pA->theCode >> bitDec)) == predCodeA)) ++pA;
			predCodeA = currentCodeA;
			++cellsA;
			while ((pB != codesB.end()) && ((currentCodeB = (pB->theCode >> bitDec)) == predCodeB)) ++pB;
			predCodeB = currentCodeB;
			++cellsB;
		}
	}

	while (pA != codesA.end())
	{
		++diffA;
		++cellsA;
		while ((pA != codesA.end()) && ((currentCodeA = (pA->theCode >> bitDec)) == predCodeA)) ++pA;
		predCodeA = currentCodeA;
	}
	while (pB != codesB.end())
	{
		++diffB;
		++cellsB;
		while ((pB != codesB.end()) && ((currentCodeB = (pB->theCode >> bitDec)) == predCodeB)) ++pB;
		predCodeB = currentCodeB;
	}
	
	return true;
}

int DgmOctree::extractCCs(unsigned char level, bool sixConnexity, GenericProgressCallback* progressCb) const
{
	std::vector<CellCode> cellCodes;
	getCellCodes(level,cellCodes);
	return extractCCs(cellCodes, level, sixConnexity, progressCb);
}

struct IndexAndCodeExt
{
#ifdef OCTREE_CODES_64_BITS
	using IndexType = unsigned long long;
#else
	using IndexType = unsigned;
#endif
	
	//! index
	IndexType theIndex;

	//! cell code
	DgmOctree::CellCode theCode;

	//! Default constructor
	IndexAndCodeExt()
		: theIndex(0)
		, theCode(0)
	{}

	//! Compares two IndexAndCodeExt instances based on their index
	/** \param a first IndexAndCode structure
		\param b second IndexAndCode structure
		\return whether the index of 'a' is smaller than the index of 'b'
	**/
	static bool indexComp(const IndexAndCodeExt& a, const IndexAndCodeExt& b) throw()
	{
		return a.theIndex < b.theIndex;
	}

};

int DgmOctree::extractCCs(const cellCodesContainer& cellCodes, unsigned char level, bool sixConnexity, GenericProgressCallback* progressCb) const
{
	std::size_t numberOfCells = cellCodes.size();
	if (numberOfCells == 0) //no cells!
		return -1;

	//filled octree cells
	std::vector<IndexAndCodeExt> ccCells;
	try
	{
		ccCells.resize(numberOfCells);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return -2;
	}

	//we compute the position of each cell (grid coordinates)
	Tuple3i indexMin;
	Tuple3i indexMax;
	{
		//binary shift for cell code truncation
		unsigned char bitDec = GET_BIT_SHIFT(level);

		for (std::size_t i = 0; i < numberOfCells; i++)
		{
			ccCells[i].theCode = (cellCodes[i] >> bitDec);

			Tuple3i cellPos;
			getCellPos(ccCells[i].theCode, level, cellPos, true);

			//we look for the actual min and max dimensions of the input cells set
			//(which may not be the whole set of octree cells!)
			if (i != 0)
			{
				for (unsigned char k = 0; k < 3; k++)
				{
					if (cellPos.u[k] < indexMin.u[k])
						indexMin.u[k] = cellPos.u[k];
					else if (cellPos.u[k] > indexMax.u[k])
						indexMax.u[k] = cellPos.u[k];
				}
			}
			else
			{
				indexMin.x = indexMax.x = cellPos.x;
				indexMin.y = indexMax.y = cellPos.y;
				indexMin.z = indexMax.z = cellPos.z;
			}

			//Warning: the cells will have to be sorted inside a slice afterwards!
			ccCells[i].theIndex = (static_cast<IndexAndCodeExt::IndexType>(cellPos.x))
								+ (static_cast<IndexAndCodeExt::IndexType>(cellPos.y) << level)
								+ (static_cast<IndexAndCodeExt::IndexType>(cellPos.z) << (2 * level));
		}
	}

	//we deduce the size of the grid that totally includes input cells
	Tuple3i gridSize = indexMax - indexMin + Tuple3i(1, 1, 1);

	//we sort the cells
	ParallelSort(ccCells.begin(), ccCells.end(), IndexAndCodeExt::indexComp); //ascending index code order

	const int& di = gridSize.x;
	const int& dj = gridSize.y;
	const int& step = gridSize.z;

	//relative neighbos positions (either 6 or 26 total - but we only use half of it)
	unsigned char neighborsInCurrentSlice = 0;
	unsigned char neighborsInPrecedingSlice = 0;
	int currentSliceNeighborsShifts[4];
	int precedingSliceNeighborsShifts[9]; //maximum size to simplify code...

	if (sixConnexity) //6-connexity
	{
		neighborsInCurrentSlice = 2;
		currentSliceNeighborsShifts[0] = -(di + 2);
		currentSliceNeighborsShifts[1] = -1;

		neighborsInPrecedingSlice = 1;
		precedingSliceNeighborsShifts[0] = 0;
	}
	else //26-connexity
	{
		neighborsInCurrentSlice = 4;
		currentSliceNeighborsShifts[0] = -1 - (di + 2);
		currentSliceNeighborsShifts[1] = -(di + 2);
		currentSliceNeighborsShifts[2] = 1 - (di + 2);
		currentSliceNeighborsShifts[3] = -1;

		neighborsInPrecedingSlice = 9;
		precedingSliceNeighborsShifts[0] = -1 - (di + 2);
		precedingSliceNeighborsShifts[1] = -(di + 2);
		precedingSliceNeighborsShifts[2] = 1 - (di + 2);
		precedingSliceNeighborsShifts[3] = -1;
		precedingSliceNeighborsShifts[4] = 0;
		precedingSliceNeighborsShifts[5] = 1;
		precedingSliceNeighborsShifts[6] = -1 + (di + 2);
		precedingSliceNeighborsShifts[7] = (di + 2);
		precedingSliceNeighborsShifts[8] = 1 + (di + 2);
	}

	//shared structures (to avoid repeated allocations)
	std::vector<int> neighboursVal;
	std::vector<int> neighboursMin;
	try
	{
		neighboursVal.reserve(neighborsInCurrentSlice + neighborsInPrecedingSlice);
		neighboursMin.reserve(neighborsInCurrentSlice + neighborsInPrecedingSlice);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return -2;
	}

	//temporary virtual 'slices'
	int sliceSize = (di + 2) * (dj + 2); //add a margin to avoid "boundary effects"
	std::vector<int> slice;
	std::vector<int> oldSlice;
	//equivalence table between 'on the fly' labels
	std::vector<int> equivalentLabels;
	std::vector<int> cellIndexToLabel;

	try
	{
		slice.resize(sliceSize);
		oldSlice.resize(sliceSize, 0); //previous slice is empty by default
		equivalentLabels.resize(numberOfCells + 2, 0);
		cellIndexToLabel.resize(numberOfCells, 0);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return -2;
	}

	//progress notification
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setMethodTitle("Components Labeling");
			char buffer[256];
			sprintf(buffer, "Box: [%i*%i*%i]", gridSize.x, gridSize.y, gridSize.z);
			progressCb->setInfo(buffer);
		}
		progressCb->update(0);
		progressCb->start();
	}

	//current label
	std::size_t currentLabel = 1;

	//process each slice
	{
		unsigned counter = 0;
		const IndexAndCodeExt::IndexType gridCoordMask = (static_cast<IndexAndCodeExt::IndexType>(1) << level) - 1;
		std::vector<IndexAndCodeExt>::const_iterator _ccCells = ccCells.begin();
		NormalizedProgress nprogress(progressCb, step);

		for (int k = indexMin.z; k < indexMin.z + step; k++)
		{
			//initialize the 'current' slice
			std::fill(slice.begin(), slice.end(), 0);

			//for each cell of the slice
			while (counter < numberOfCells && static_cast<int>(_ccCells->theIndex >> (level << 1)) == k)
			{
				int iind = static_cast<int>(_ccCells->theIndex & gridCoordMask);
				int jind = static_cast<int>((_ccCells->theIndex >> level) & gridCoordMask);
				int cellIndex = (iind - indexMin.x + 1) + (jind - indexMin.y + 1)*(di + 2);
				++_ccCells;

				//we look if the cell has neighbors inside the slice
				int* _slice = &(slice[cellIndex]);
				{
					for (unsigned char n = 0; n < neighborsInCurrentSlice; n++)
					{
						assert(cellIndex + currentSliceNeighborsShifts[n] < sliceSize);
						const int& neighborLabel = _slice[currentSliceNeighborsShifts[n]];
						if (neighborLabel > 1)
							neighboursVal.push_back(neighborLabel);
					}
				}

				//and in the previous slice
				const int* _oldSlice = &(oldSlice[cellIndex]);
				{
					for (unsigned char n = 0; n < neighborsInPrecedingSlice; n++)
					{
						assert(cellIndex + precedingSliceNeighborsShifts[n] < sliceSize);
						const int& neighborLabel = _oldSlice[precedingSliceNeighborsShifts[n]];
						if (neighborLabel > 1)
							neighboursVal.push_back(neighborLabel);
					}
				}

				//number of neighbors for current cell
				std::size_t p = neighboursVal.size();

				if (p == 0) //no neighbor
				{
					*_slice = static_cast<int>(++currentLabel); //we create a new label
				}
				else if (p == 1) //1 neighbor
				{
					*_slice = neighboursVal.back(); //we'll use its label
					neighboursVal.pop_back();
				}
				else //more than 1 neighbor?
				{
					//we get the smallest label
					ParallelSort(neighboursVal.begin(), neighboursVal.end());
					
					int smallestLabel = neighboursVal[0];

					//if they are not the same
					if (smallestLabel != neighboursVal.back())
					{
						int lastLabel = 0;
						neighboursMin.clear();
						//we get the smallest equivalent label for each neighbor's branch
						{
							for (std::size_t n = 0; n < p; n++)
							{
								// ... we start from its C.C. index
								int label = neighboursVal[n];
								//if it's not the same as the previous neighbor's
								if (label != lastLabel)
								{
									//we update the 'before' value
									assert(label < static_cast<int>(numberOfCells)+2);
									lastLabel = label;

									//we look for its real equivalent value
									while (equivalentLabels[label] > 1)
									{
										label = equivalentLabels[label];
										assert(label < static_cast<int>(numberOfCells)+2);
									}

									neighboursMin.push_back(label);
								}
							}
						}

						//get the smallest one
						ParallelSort(neighboursMin.begin(), neighboursMin.end());
						
						smallestLabel = neighboursMin.front();

						//update the equivalence table by the way
						//for all other branches
						lastLabel = smallestLabel;
						{
							for (std::size_t n = 1; n < neighboursMin.size(); n++)
							{
								int label = neighboursMin[n];
								assert(label < static_cast<int>(numberOfCells)+2);
								//we don't process it if it's the same label as the previous neighbor
								if (label != lastLabel)
								{
									equivalentLabels[label] = smallestLabel;
									lastLabel = label;
								}
							}
						}
					}

					//update current cell label
					*_slice = smallestLabel;
					neighboursVal.clear();
				}

				cellIndexToLabel[counter++] = *_slice;
			}

			if (counter == numberOfCells)
				break;

			std::swap(slice, oldSlice);

			nprogress.oneStep();
		}
	}

	//release some memory
	slice.resize(0);
	oldSlice.resize(0);

	if (progressCb)
	{
		progressCb->stop();
	}

	if (currentLabel < 2)
	{
		//No component found
		return -3;
	}

	//path compression (http://en.wikipedia.org/wiki/Union_find)
	assert(currentLabel < numberOfCells + 2);
	{
		for (std::size_t i = 2; i <= currentLabel; i++)
		{
			int label = equivalentLabels[i];
			assert(label < static_cast<int>(numberOfCells)+2);
			while (equivalentLabels[label] > 1) //equivalentLabels[0] == 0 !!!
			{
				label = equivalentLabels[label];
				assert(label < static_cast<int>(numberOfCells)+2);
			}
			equivalentLabels[i] = label;
		}
	}

	//update leaves
	{
		for (std::size_t i = 0; i < numberOfCells; i++)
		{
			int label = cellIndexToLabel[i];
			assert(label < static_cast<int>(numberOfCells)+2);
			if (equivalentLabels[label] > 1)
				cellIndexToLabel[i] = equivalentLabels[label];
		}
	}

	//hack: we use "equivalentLabels" to count how many components will have to be created
	int numberOfComponents = 0;
	{
		std::fill(equivalentLabels.begin(), equivalentLabels.end(), 0);

		for (std::size_t i = 0; i < numberOfCells; i++)
		{
			assert(cellIndexToLabel[i] > 1 && cellIndexToLabel[i] < static_cast<int>(numberOfCells)+2);
			equivalentLabels[cellIndexToLabel[i]] = 1;
		}

		//we create (following) indexes for each components
		for (std::size_t i = 2; i < numberOfCells + 2; i++)
			if (equivalentLabels[i] == 1)
				equivalentLabels[i] = ++numberOfComponents; //labels start at '1'
	}
	assert(equivalentLabels[0] == 0);
	assert(equivalentLabels[1] == 0);

	//we flag each component's points with its label
	{
		if (progressCb)
		{
			if (progressCb->textCanBeEdited())
			{
				char buffer[256];
				sprintf(buffer, "Components: %i", numberOfComponents);
				progressCb->setMethodTitle("Connected Components Extraction");
				progressCb->setInfo(buffer);
			}
			progressCb->update(0);
			progressCb->start();
		}
		NormalizedProgress nprogress(progressCb, static_cast<unsigned>(numberOfCells));

		ReferenceCloud Y(m_theAssociatedCloud);
		for (std::size_t i = 0; i < numberOfCells; i++)
		{
			assert(cellIndexToLabel[i] < static_cast<int>(numberOfCells)+2);

			const int& label = equivalentLabels[cellIndexToLabel[i]];
			assert(label > 0);
			getPointsInCell(ccCells[i].theCode, level, &Y, true);
			Y.placeIteratorAtBeginning();
			ScalarType d = static_cast<ScalarType>(label);
			for (unsigned j = 0; j < Y.size(); ++j)
			{
				Y.setCurrentPointScalarValue(d);
				Y.forwardIterator();
			}

			nprogress.oneStep();
		}

		if (progressCb)
		{
			progressCb->stop();
		}
	}

	return numberOfComponents;
}

/*** Octree-based cloud traversal mechanism ***/

DgmOctree::octreeCell::octreeCell(const DgmOctree* _parentOctree)
	: parentOctree(_parentOctree)
	, truncatedCode(0)
	, index(0)
	, points(nullptr)
	, level(0)
{
	if (parentOctree && parentOctree->m_theAssociatedCloud)
	{
		points = new ReferenceCloud(parentOctree->m_theAssociatedCloud);
	}
	else
	{
		assert(false);
	}
}

DgmOctree::octreeCell::octreeCell(const octreeCell& cell)
	: parentOctree(cell.parentOctree)
	, truncatedCode(cell.truncatedCode)
	, index(cell.index)
	, points(nullptr)
	, level(cell.level)
{
	//copy constructor shouldn't be used (we can't properly share the 'points' reference)
	assert(false);
}

DgmOctree::octreeCell::~octreeCell()
{
	delete points;
}

#ifdef ENABLE_MT_OCTREE

#include <QtCore>
#include <QApplication>
#include <QtConcurrentMap>
#include <QThreadPool>

/*** FOR THE MULTI THREADING WRAPPER ***/
struct octreeCellDesc
{
	DgmOctree::CellCode truncatedCode;
	unsigned i1, i2;
	unsigned char level;
};

static DgmOctree* s_octree_MT = nullptr;
static DgmOctree::octreeCellFunc s_func_MT = nullptr;
static void** s_userParams_MT = nullptr;
static GenericProgressCallback* s_progressCb_MT = nullptr;
static NormalizedProgress* s_normProgressCb_MT = nullptr;
static bool s_cellFunc_MT_success = true;

void LaunchOctreeCellFunc_MT(const octreeCellDesc& desc)
{
	//skip cell if process is aborted/has failed
	if (!s_cellFunc_MT_success)
	{
		return;
	}

	const DgmOctree::cellsContainer& pointsAndCodes = s_octree_MT->pointsAndTheirCellCodes();

	//cell descriptor
	DgmOctree::octreeCell cell(s_octree_MT);
	cell.level = desc.level;
	cell.index = desc.i1;
	cell.truncatedCode = desc.truncatedCode;
	if (cell.points->reserve(desc.i2 - desc.i1 + 1))
	{
		for (unsigned i = desc.i1; i <= desc.i2; ++i)
		{
			cell.points->addPointIndex(pointsAndCodes[i].theIndex);
		}

		s_cellFunc_MT_success &= (*s_func_MT)(cell, s_userParams_MT, s_normProgressCb_MT);
	}
	else
	{
		s_cellFunc_MT_success = false;
	}

	if (!s_cellFunc_MT_success)
	{
		//TODO: display a message to make clear that the cancel order has been acknowledged!
		if (s_progressCb_MT)
		{
			if (s_progressCb_MT->textCanBeEdited())
			{
				s_progressCb_MT->setInfo("Cancelling...");
			}
		}

		//if (s_normProgressCb_MT)
		//{
		//	if (!s_normProgressCb_MT->oneStep())
		//	{
		//		s_cellFunc_MT_success = false;
		//		return;
		//	}
		//}
	}
}

#endif

unsigned DgmOctree::executeFunctionForAllCellsAtLevel(	unsigned char level,
														octreeCellFunc func,
														void** additionalParameters,
														bool multiThread/*=false*/,
														GenericProgressCallback* progressCb/*=0*/,
														const char* functionTitle/*=0*/,
														int maxThreadCount/*=0*/)
{
	if (m_thePointsAndTheirCellCodes.empty())
		return 0;

#ifdef ENABLE_MT_OCTREE

	//cells that will be processed by QtConcurrent::map
	const unsigned cellsNumber = getCellNumber(level);
	std::vector<octreeCellDesc> cells;

	if (multiThread)
	{
		try
		{
			cells.reserve(cellsNumber);
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			//we use standard way (DGM TODO: we should warn the user!)
			multiThread = false;
		}
	}

	if (!multiThread)
#endif
	{
		//we get the maximum cell population for this level
		unsigned maxCellPopulation = m_maxCellPopulation[level];

		//cell descriptor (initialize it with first cell/point)
		octreeCell cell(this);
		if (!cell.points->reserve(maxCellPopulation)) //not enough memory
			return 0;
		cell.level = level;
		cell.index = 0;

		//binary shift for cell code truncation
		unsigned char bitDec = GET_BIT_SHIFT(level);

		//iterator on cell codes
		cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin();

		//init with first cell
		cell.truncatedCode = (p->theCode >> bitDec);
		cell.points->addPointIndex(p->theIndex); //can't fail (see above)
		++p;

		//number of cells for this level
		unsigned cellCount = getCellNumber(level);

		//progress bar
		if (progressCb)
		{
			if (progressCb->textCanBeEdited())
			{
				if (functionTitle)
				{
					progressCb->setMethodTitle(functionTitle);
				}
				char buffer[512];
				sprintf(buffer, "Octree level %i\nCells: %u\nMean population: %3.2f (+/-%3.2f)\nMax population: %u", level, cellCount, m_averageCellPopulation[level], m_stdDevCellPopulation[level], m_maxCellPopulation[level]);
				progressCb->setInfo(buffer);
			}
			progressCb->update(0);
			progressCb->start();
		}
		NormalizedProgress nprogress(progressCb, m_theAssociatedCloud->size());

		bool result = true;

#ifdef COMPUTE_NN_SEARCH_STATISTICS
		s_skippedPoints = 0;
		s_testedPoints = 0;
		s_jumps = 0.0;
		s_binarySearchCount = 0.0;
#endif

		//for each point
		for (; p != m_thePointsAndTheirCellCodes.end(); ++p)
		{
			//check if it belongs to the current cell
			CellCode nextCode = (p->theCode >> bitDec);
			if (nextCode != cell.truncatedCode)
			{
				//if not, we call the user function on the previous cell
				result = (*func)(cell, additionalParameters, &nprogress);

				if (!result)
					break;

				//and we start a new cell
				cell.index += cell.points->size();
				cell.points->clear();
				cell.truncatedCode = nextCode;

				//if (!nprogress.oneStep())
				//{
				//	//process canceled by user
				//	result = false;
				//	break;
				//}
			}

			cell.points->addPointIndex(p->theIndex); //can't fail (see above)
		}

		//don't forget last cell!
		if (result)
			result = (*func)(cell, additionalParameters, &nprogress);

#ifdef COMPUTE_NN_SEARCH_STATISTICS
		FILE* fp=fopen("octree_log.txt","at");
		if (fp)
		{
			fprintf(fp,"Function: %s\n",functionTitle ? functionTitle : "unknown");
			fprintf(fp,"Tested:   %f (%3.1f %%)\n",s_testedPoints,100.0*s_testedPoints/std::max(1.0,s_testedPoints+s_skippedPoints));
			fprintf(fp,"skipped: %f (%3.1f %%)\n",s_skippedPoints,100.0*s_skippedPoints/std::max(1.0,s_testedPoints+s_skippedPoints));
			fprintf(fp,"Binary search count: %.0f\n",s_binarySearchCount);
			if (s_binarySearchCount > 0.0)
				fprintf(fp,"Mean jumps: %f\n",s_jumps/s_binarySearchCount);
			fprintf(fp,"\n");
			fclose(fp);
		}
#endif

		//if something went wrong, we return 0
		return (result ? cellCount : 0);
	}
#ifdef ENABLE_MT_OCTREE
	else
	{
		assert(cells.capacity() == cellsNumber);

		//binary shift for cell code truncation
		unsigned char bitDec = GET_BIT_SHIFT(level);

		//iterator on cell codes
		cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin();

		//cell descriptor (init. with first point/cell)
		octreeCellDesc cellDesc;
		cellDesc.i1 = 0;
		cellDesc.i2 = 0;
		cellDesc.level = level;
		cellDesc.truncatedCode = (p->theCode >> bitDec);
		++p;

		//sweep through the octree
		for (; p!=m_thePointsAndTheirCellCodes.end(); ++p)
		{
			CellCode nextCode = (p->theCode >> bitDec);

			if (nextCode != cellDesc.truncatedCode)
			{
				cells.push_back(cellDesc);
				cellDesc.i1=cellDesc.i2+1;
			}

			cellDesc.truncatedCode = nextCode;
			++cellDesc.i2;
		}
		//don't forget the last cell!
		cells.push_back(cellDesc);

		//static wrap
		s_octree_MT = this;
		s_func_MT = func;
		s_userParams_MT = additionalParameters;
		s_cellFunc_MT_success = true;
		s_progressCb_MT = progressCb;
		if (s_normProgressCb_MT)
		{
			delete s_normProgressCb_MT;
			s_normProgressCb_MT = 0;
		}

		//progress notification
		if (progressCb)
		{
			if (progressCb->textCanBeEdited())
			{
				if (functionTitle)
				{
					progressCb->setMethodTitle(functionTitle);
				}
				char buffer[512];
				sprintf(buffer, "Octree level %i\nCells: %i\nAverage population: %3.2f (+/-%3.2f)\nMax population: %u", level, static_cast<int>(cells.size()), m_averageCellPopulation[level], m_stdDevCellPopulation[level], m_maxCellPopulation[level]);
				progressCb->setInfo(buffer);
			}
			progressCb->update(0);
			s_normProgressCb_MT = new NormalizedProgress(progressCb,m_theAssociatedCloud->size());
			progressCb->start();
		}

#ifdef COMPUTE_NN_SEARCH_STATISTICS
		s_skippedPoints = 0;
		s_testedPoints = 0;
		s_jumps = 0.0;
		s_binarySearchCount = 0.0;
#endif

		if (maxThreadCount == 0)
		{
			maxThreadCount = QThread::idealThreadCount();
		}
		QThreadPool::globalInstance()->setMaxThreadCount(maxThreadCount);
		QtConcurrent::blockingMap(cells, LaunchOctreeCellFunc_MT);

#ifdef COMPUTE_NN_SEARCH_STATISTICS
		FILE* fp = fopen("octree_log.txt", "at");
		if (fp)
		{
			fprintf(fp, "Function: %s\n", functionTitle ? functionTitle : "unknown");
			fprintf(fp, "Tested:  %f (%3.1f %%)\n", s_testedPoints, 100.0*s_testedPoints / std::max(1.0, s_testedPoints + s_skippedPoints));
			fprintf(fp, "skipped: %f (%3.1f %%)\n", s_skippedPoints, 100.0*s_skippedPoints / std::max(1.0, s_testedPoints + s_skippedPoints));
			fprintf(fp, "Binary search count: %.0f\n", s_binarySearchCount);
			if (s_binarySearchCount > 0.0)
				fprintf(fp, "Mean jumps: %f\n", s_jumps / s_binarySearchCount);
			fprintf(fp, "\n");
			fclose(fp);
		}
#endif

		s_octree_MT = nullptr;
		s_func_MT = nullptr;
		s_userParams_MT = nullptr;

		if (progressCb)
		{
			progressCb->stop();
			if (s_normProgressCb_MT)
				delete s_normProgressCb_MT;
			s_normProgressCb_MT = nullptr;
			s_progressCb_MT = nullptr;
		}

		//if something went wrong, we clear everything and return 0!
		if (!s_cellFunc_MT_success)
			cells.clear();

		return static_cast<unsigned>(cells.size());
	}
#endif
}

//Down-top traversal (for standard and mutli-threaded versions)
#define ENABLE_DOWN_TOP_TRAVERSAL
#define ENABLE_DOWN_TOP_TRAVERSAL_MT

unsigned DgmOctree::executeFunctionForAllCellsStartingAtLevel(unsigned char startingLevel,
	octreeCellFunc func,
	void** additionalParameters,
	unsigned minNumberOfPointsPerCell,
	unsigned maxNumberOfPointsPerCell,
	bool multiThread/* = true*/,
	GenericProgressCallback* progressCb/*=0*/,
	const char* functionTitle/*=0*/,
	int maxThreadCount/*=0*/)
{
	if (m_thePointsAndTheirCellCodes.empty())
		return 0;

	const unsigned cellsNumber = getCellNumber(startingLevel);

#ifdef ENABLE_MT_OCTREE

	//cells that will be processed by QtConcurrent::map
	std::vector<octreeCellDesc> cells;
	if (multiThread)
	{
		try
		{
			cells.reserve(cellsNumber); //at least!
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory?
			//we use standard way (DGM TODO: we should warn the user!)
			multiThread = false;
		}
	}

	if (!multiThread)
#endif
	{
		//we get the maximum cell population for this level
		unsigned maxCellPopulation = m_maxCellPopulation[startingLevel];

		//cell descriptor
		octreeCell cell(this);
		if (!cell.points->reserve(maxCellPopulation)) //not enough memory
			return 0;
		cell.level = startingLevel;
		cell.index = 0;

		//progress notification
		if (progressCb)
		{
			if (progressCb->textCanBeEdited())
			{
				if (functionTitle)
				{
					progressCb->setMethodTitle(functionTitle);
				}
				char buffer[1024];
				sprintf(buffer, "Octree levels %i - %i\nCells: %i - %i\nAverage population: %3.2f (+/-%3.2f) - %3.2f (+/-%3.2f)\nMax population: %u - %u",
					startingLevel, MAX_OCTREE_LEVEL,
					getCellNumber(startingLevel), getCellNumber(MAX_OCTREE_LEVEL),
					m_averageCellPopulation[startingLevel], m_stdDevCellPopulation[startingLevel],
					m_averageCellPopulation[MAX_OCTREE_LEVEL], m_stdDevCellPopulation[MAX_OCTREE_LEVEL],
					m_maxCellPopulation[startingLevel], m_maxCellPopulation[MAX_OCTREE_LEVEL]);
				progressCb->setInfo(buffer);
			}
			progressCb->update(0);
			progressCb->start();
		}
#ifndef ENABLE_DOWN_TOP_TRAVERSAL
		NormalizedProgress nprogress(progressCb,m_theAssociatedCloud->size());
#endif

		//binary shift for cell code truncation at current level
		unsigned char currentBitDec = GET_BIT_SHIFT(startingLevel);

#ifdef ENABLE_DOWN_TOP_TRAVERSAL
		bool firstSubCell = true;
#else
		unsigned char shallowSteps = 0;
#endif

		//pointer on the current octree element
		cellsContainer::const_iterator startingElement = m_thePointsAndTheirCellCodes.begin();

		bool result = true;

		//let's sweep through the octree
		while (cell.index < m_numberOfProjectedPoints)
		{
			//new cell
			cell.truncatedCode = (startingElement->theCode >> currentBitDec);
			//we can already 'add' (virtually) the first point to the current cell description struct
			unsigned elements = 1;

			//progress notification
#ifndef ENABLE_DOWN_TOP_TRAVERSAL
			//if (cell.level == startingLevel)
			//{
			//	if (!nprogress.oneStep())
			//	{
			//		result=false;
			//		break;
			//	}
			//}
#else
			//in this mode, we can't update progress notification regularly...
			if (progressCb)
			{
				progressCb->update(100.0f*static_cast<float>(cell.index) / static_cast<float>(m_numberOfProjectedPoints));
				if (progressCb->isCancelRequested())
				{
					result = false;
					break;
				}
			}
#endif

			//let's test the following points
			for (cellsContainer::const_iterator p = startingElement + 1; p != m_thePointsAndTheirCellCodes.end(); ++p)
			{
				//next point code (at current level of subdivision)
				CellCode currentTruncatedCode = (p->theCode >> currentBitDec);
				//same code? Then it belongs to the same cell
				if (currentTruncatedCode == cell.truncatedCode)
				{
					//if we have reached the user specified limit
					if (elements == maxNumberOfPointsPerCell)
					{
						bool keepGoing = true;

						//we should go deeper in the octree (as long as the current element
						//belongs to the same cell as the first cell element - in which case
						//the cell will still be too big)
						while (cell.level < MAX_OCTREE_LEVEL)
						{
							//next level
							++cell.level;
							currentBitDec -= 3;
							cell.truncatedCode = (startingElement->theCode >> currentBitDec);

							//not the same cell anymore?
							if (cell.truncatedCode != (p->theCode >> currentBitDec))
							{
								//we must re-check all the previous inserted points at this new level
								//to determine the end of this new cell
								p = startingElement;
								elements = 1;
								while (((++p)->theCode >> currentBitDec) == cell.truncatedCode)
									++elements;

								//and we must stop point collection here
								keepGoing = false;

#ifdef ENABLE_DOWN_TOP_TRAVERSAL
								//in this case, the next cell won't be the first sub-cell!
								firstSubCell = false;
#endif
								break;
							}
						}

						//we should stop point collection here
						if (!keepGoing)
							break;
					}

					//otherwise we 'add' the point to the cell descriptor
					++elements;
				}
				else //code is different --> not the same cell anymore
				{
#ifndef ENABLE_DOWN_TOP_TRAVERSAL
					//we may have to go shallower ... as long as the parent cell is different
					assert(shallowSteps == 0);
					CellCode cellTruncatedCode = cell.truncatedCode;
					while (cell.level > startingLevel+shallowSteps)
					{
						cellTruncatedCode>>=3;
						currentTruncatedCode>>=3;
						//this cell and the following share the same parent
						if (cellTruncatedCode == currentTruncatedCode)
							break;
						++shallowSteps;
					}

					//we must stop point collection here
					break;
#else
					//we are at the end of the cell
					bool keepGoing = false;
					//can we continue collecting points?
					if (cell.level > startingLevel)
					{
						//this cell and the following share the same parent?
						if ((cell.truncatedCode >> 3) == (currentTruncatedCode >> 3))
						{
							//if this cell is the first one, and we don't have enough points
							//we can simply proceed with its parent cell
							if (firstSubCell && elements < minNumberOfPointsPerCell)
							{
								//previous level
								--cell.level;
								currentBitDec += 3;
								cell.truncatedCode >>= 3;

								//we 'add' the point to the cell descriptor
								++elements;
								//and we can continue collecting points
								keepGoing = true;
							}

							//as this cell and the next one share the same parent,
							//the next cell won't be the first sub-cell!
							firstSubCell = false;
						}
						else
						{
							//as this cell and the next one have different parents,
							//the next cell is the first sub-cell!
							firstSubCell = true;
						}
					}
					else
					{
						//at the ceiling level, all cells are considered as 'frist' sub-cells
						firstSubCell = true;
					}

					//we must stop point collection here
					if (!keepGoing)
						break;
#endif
				}
			}

			//we can now really 'add' the points to the cell descriptor
			cell.points->clear();
			//DGM: already done earlier
			/*if (!cell.points->reserve(elements)) //not enough memory
			{
			result=false;
			break;
			}
			*/
			for (unsigned i = 0; i < elements; ++i)
			{
				cell.points->addPointIndex((startingElement++)->theIndex);
			}

			//call user method on current cell
			result = (*func)(cell, additionalParameters,
#ifndef ENABLE_DOWN_TOP_TRAVERSAL
				&nProgress
#else
				nullptr
#endif
				);

			if (!result)
				break;

			//proceed to next cell
			cell.index += elements;

#ifndef ENABLE_DOWN_TOP_TRAVERSAL
			if (shallowSteps)
			{
				//we should go shallower
				assert(cell.level-shallowSteps >= startingLevel);
				cell.level-=shallowSteps;
				currentBitDec += 3*shallowSteps;
				shallowSteps = 0;
			}
#endif
		}

		if (progressCb)
		{
			progressCb->stop();
		}

		//if something went wrong, we return 0
		return (result ? cellsNumber : 0);
	}
#ifdef ENABLE_MT_OCTREE
	else
	{
		assert(cells.capacity() == cellsNumber);

		//cell descriptor (init. with first point/cell)
		octreeCellDesc cellDesc;
		cellDesc.i1 = 0;
		cellDesc.i2 = 0;
		cellDesc.level = startingLevel;

		//binary shift for cell code truncation at current level
		unsigned char currentBitDec = GET_BIT_SHIFT(startingLevel);

#ifdef ENABLE_DOWN_TOP_TRAVERSAL_MT
		bool firstSubCell = true;
#else
		unsigned char shallowSteps = 0;
#endif
		//pointer on the current octree element
		cellsContainer::const_iterator startingElement = m_thePointsAndTheirCellCodes.begin();

		//we compute some statistics on the fly
		unsigned long long popSum = 0;
		unsigned long long popSum2 = 0;
		unsigned long long maxPop = 0;

		//let's sweep through the octree
		while (cellDesc.i1 < m_numberOfProjectedPoints)
		{
			//new cell
			cellDesc.truncatedCode = (startingElement->theCode >> currentBitDec);
			//we can already 'add' (virtually) the first point to the current cell description struct
			unsigned elements = 1;

			//let's test the following points
			for (cellsContainer::const_iterator p = startingElement+1; p != m_thePointsAndTheirCellCodes.end(); ++p)
			{
				//next point code (at current level of subdivision)
				CellCode currentTruncatedCode = (p->theCode >> currentBitDec);
				//same code? Then it belongs to the same cell
				if (currentTruncatedCode == cellDesc.truncatedCode)
				{
					//if we have reached the user specified limit
					if (elements == maxNumberOfPointsPerCell)
					{
						bool keepGoing = true;

						//we should go deeper in the octree (as long as the current element
						//belongs to the same cell as the first cell element - in which case
						//the cell will still be too big)
						while (cellDesc.level < MAX_OCTREE_LEVEL)
						{
							//next level
							++cellDesc.level;
							currentBitDec -= 3;
							cellDesc.truncatedCode = (startingElement->theCode >> currentBitDec);

							//not the same cell anymore?
							if (cellDesc.truncatedCode != (p->theCode >> currentBitDec))
							{
								//we must re-check all the previously inserted points at this new level
								//to determine the end of this new cell
								p = startingElement;
								elements=1;
								while (((++p)->theCode >> currentBitDec) == cellDesc.truncatedCode)
									++elements;

								//and we must stop point collection here
								keepGoing = false;

#ifdef ENABLE_DOWN_TOP_TRAVERSAL_MT
								//in this case, the next cell won't be the first sub-cell!
								firstSubCell = false;
#endif
								break;
							}
						}

						//we should stop point collection here
						if (!keepGoing)
							break;
					}

					//otherwise we 'add' the point to the cell descriptor
					++elements;
				}
				else //code is different --> not the same cell anymore
				{
#ifndef ENABLE_DOWN_TOP_TRAVERSAL_MT
					//we may have to go shallower ... as long as the parent cell is different
					assert(shallowSteps == 0);
					CellCode cellTruncatedCode = cellDesc.truncatedCode;
					while (cellDesc.level > startingLevel+shallowSteps)
					{
						cellTruncatedCode>>=3;
						currentTruncatedCode>>=3;
						//this cell and the following share the same parent
						if (cellTruncatedCode == currentTruncatedCode)
							break;
						++shallowSteps;
					}

					//we must stop point collection here
					break;
#else
					//we are at the end of the cell
					bool keepGoing = false;
					//can we continue collecting points?
					if (cellDesc.level > startingLevel)
					{
						//this cell and the following share the same parent?
						if ((cellDesc.truncatedCode>>3) == (currentTruncatedCode>>3))
						{
							//if this cell is the first one, and we don't have enough points
							//we can simply proceed with its parent cell
							if (firstSubCell && elements < minNumberOfPointsPerCell)
							{
								//previous level
								--cellDesc.level;
								currentBitDec+=3;
								cellDesc.truncatedCode>>=3;

								//we 'add' the point to the cell descriptor
								++elements;
								//and we can continue collecting points
								keepGoing = true;
							}

							//as this cell and the next one share the same parent,
							//the next cell won't be the first sub-cell!
							firstSubCell=false;
						}
						else
						{
							//as this cell and the next one have different parents,
							//the next cell is the first sub-cell!
							firstSubCell = true;
						}
					}
					else
					{
						//at the ceiling level, all cells are considered as 'frist' sub-cells
						firstSubCell = true;
					}

					//we must stop point collection here
					if (!keepGoing)
						break;
#endif
				}
			}

			//we can now 'add' this cell to the list
			cellDesc.i2 = cellDesc.i1 + (elements-1);
			cells.push_back(cellDesc);
			popSum += static_cast<unsigned long long>(elements);
			popSum2 += static_cast<unsigned long long>(elements*elements);
			if (maxPop < elements)
				maxPop = elements;

			//proceed to next cell
			cellDesc.i1 += elements;
			startingElement += elements;

#ifndef ENABLE_DOWN_TOP_TRAVERSAL_MT
			if (shallowSteps)
			{
				//we should go shallower
				assert(cellDesc.level-shallowSteps >= startingLevel);
				cellDesc.level-=shallowSteps;
				currentBitDec += 3*shallowSteps;
				shallowSteps = 0;
			}
#endif
		}

		//statistics
		double mean = static_cast<double>(popSum) / cells.size();
		double stddev = sqrt(static_cast<double>(popSum2 - popSum*popSum)) / cells.size();

		//static wrap
		s_octree_MT = this;
		s_func_MT = func;
		s_userParams_MT = additionalParameters;
		s_cellFunc_MT_success = true;
		if (s_normProgressCb_MT)
			delete s_normProgressCb_MT;
		s_normProgressCb_MT = nullptr;

		//progress notification
		if (progressCb)
		{
			if (progressCb->textCanBeEdited())
			{
				if (functionTitle)
				{
					progressCb->setMethodTitle(functionTitle);
				}
				char buffer[1024];
				sprintf(buffer, "Octree levels %i - %i\nCells: %i\nAverage population: %3.2f (+/-%3.2f)\nMax population: %llu", startingLevel, MAX_OCTREE_LEVEL, static_cast<int>(cells.size()), mean, stddev, maxPop);
				progressCb->setInfo(buffer);
			}
			if (s_normProgressCb_MT)
			{
				delete s_normProgressCb_MT;
			}
			s_normProgressCb_MT = new NormalizedProgress(progressCb,static_cast<unsigned>(cells.size()));
			progressCb->update(0);
			progressCb->start();
		}

#ifdef COMPUTE_NN_SEARCH_STATISTICS
		s_skippedPoints = 0;
		s_testedPoints = 0;
		s_jumps = 0.0;
		s_binarySearchCount = 0.0;
#endif

		if (maxThreadCount == 0)
		{
			maxThreadCount = QThread::idealThreadCount();
		}
		QThreadPool::globalInstance()->setMaxThreadCount(maxThreadCount);
		QtConcurrent::blockingMap(cells, LaunchOctreeCellFunc_MT);

#ifdef COMPUTE_NN_SEARCH_STATISTICS
		FILE* fp=fopen("octree_log.txt","at");
		if (fp)
		{
			fprintf(fp,"Function: %s\n",functionTitle ? functionTitle : "unknown");
			fprintf(fp,"Tested:   %f (%3.1f %%)\n",s_testedPoints,100.0*s_testedPoints/std::max(1.0,s_testedPoints+s_skippedPoints));
			fprintf(fp,"skipped: %f (%3.1f %%)\n",s_skippedPoints,100.0*s_skippedPoints/std::max(1.0,s_testedPoints+s_skippedPoints));
			fprintf(fp,"Binary search count: %.0f\n",s_binarySearchCount);
			if (s_binarySearchCount > 0.0)
				fprintf(fp,"Mean jumps: %f\n",s_jumps/s_binarySearchCount);
			fprintf(fp,"\n");
			fclose(fp);
		}
#endif

		s_octree_MT = nullptr;
		s_func_MT = nullptr;
		s_userParams_MT = nullptr;

		if (progressCb)
		{
			progressCb->stop();
			if (s_normProgressCb_MT)
				delete s_normProgressCb_MT;
			s_normProgressCb_MT = nullptr;
		}

		//if something went wrong, we clear everything and return 0!
		if (!s_cellFunc_MT_success)
			cells.resize(0);

		return static_cast<unsigned>(cells.size());
	}
#endif
}

bool DgmOctree::rayCast(const CCVector3& rayAxis,
						const CCVector3& rayOrigin,
						double maxRadiusOrFov,
						bool isFOV,
						RayCastProcess process,
						std::vector<PointDescriptor>& output) const
{
	if (m_thePointsAndTheirCellCodes.empty())
	{
		//nothing to do
		assert(false);
		return false;
	}

	CCVector3 margin(0, 0, 0);
	double maxSqRadius = 0;
	if (!isFOV)
	{
		margin = CCVector3(1, 1, 1) * static_cast<PointCoordinateType>(maxRadiusOrFov);
		maxSqRadius = maxRadiusOrFov*maxRadiusOrFov;
	}

	//first test with the total bounding box
	Ray<PointCoordinateType> ray(rayAxis, rayOrigin);
	if (!AABB<PointCoordinateType>(m_dimMin - margin, m_dimMax + margin).intersects(ray))
	{
		//no intersection
		output.resize(0);
		return true;
	}

	//no need to go too deep
	const unsigned char maxLevel = findBestLevelForAGivenPopulationPerCell(10);

	//starting level of subdivision
	unsigned char level = 1;
	//binary shift for cell code truncation at current level
	unsigned char currentBitDec = GET_BIT_SHIFT(level);
	//current cell code
	CellCode currentCode = INVALID_CELL_CODE;
	//whether the current cell should be skipped or not
	bool skipThisCell = false;
	//smallest FOV (i.e. nearest point)
	double smallestOrderDist = -1.0;

#ifdef CC_DEBUG
	m_theAssociatedCloud->enableScalarField();
#endif

	//ray with origin expressed in the local coordinate system!
	Ray<PointCoordinateType> rayLocal(rayAxis, rayOrigin - m_dimMin);

	//let's sweep through the octree
	for (cellsContainer::const_iterator it = m_thePointsAndTheirCellCodes.begin(); it != m_thePointsAndTheirCellCodes.end(); ++it)
	{
		CellCode truncatedCode = (it->theCode >> currentBitDec);
		
		//new cell?
		if (truncatedCode != (currentCode >> currentBitDec))
		{
			//look for the biggest 'parent' cell that englobes this cell and the previous one (if any)
			while (level > 1)
			{
				unsigned char bitDec = GET_BIT_SHIFT(level-1);
				if ((it->theCode >> bitDec) == (currentCode >> bitDec))
				{
					//same parent cell, we can stop here
					break;
				}
				--level;
			}

			currentCode = it->theCode;

			//now try to go deeper with the new cell
			while (level < maxLevel)
			{
				Tuple3i cellPos;
				getCellPos(it->theCode, level, cellPos, false);

				//first test with the total bounding box
				const PointCoordinateType& halfCellSize = getCellSize(level) / 2;
				CCVector3 cellCenter(	(2* cellPos.x + 1) * halfCellSize,
										(2* cellPos.y + 1) * halfCellSize,
										(2* cellPos.z + 1) * halfCellSize);

				CCVector3 halfCell = CCVector3(halfCellSize, halfCellSize, halfCellSize);

				if (isFOV)
				{
					double radialSqDist;
					double sqDistToOrigin;
					rayLocal.squareDistances(cellCenter, radialSqDist, sqDistToOrigin);

					double dx = sqrt(sqDistToOrigin);
					double dy = std::max<double>(0, sqrt(radialSqDist) - SQRT_3 * halfCellSize);
					double fov_rad = atan2(dy, dx);

					skipThisCell = (fov_rad > maxRadiusOrFov);
				}
				else
				{
					skipThisCell = !AABB<PointCoordinateType>(	cellCenter - halfCell - margin,
																cellCenter + halfCell + margin).intersects(rayLocal);
				}

				if (skipThisCell)
					break;
				else
					++level;
			}
			currentBitDec = GET_BIT_SHIFT(level);
		}

#ifdef CC_DEBUG
		m_theAssociatedCloud->setPointScalarValue(it->theIndex, level);
#endif

		if (!skipThisCell)
		{
			//test the point
			const CCVector3* P = m_theAssociatedCloud->getPoint(it->theIndex);

			double radialSqDist = ray.radialSquareDistance(*P);
			double orderDist = -1.0;
			bool isElligible = false;

			if (isFOV)
			{
				double sqDist = ray.squareDistanceToOrigin(*P);
				double fov_rad = atan2(sqrt(radialSqDist), sqrt(sqDist));
				isElligible = (fov_rad <= maxRadiusOrFov);
				orderDist = fov_rad;
#ifdef CC_DEBUG
				//m_theAssociatedCloud->setPointScalarValue(it->theIndex, fov_rad);
				//m_theAssociatedCloud->setPointScalarValue(it->theIndex, sqrt(sqDist));
#endif
			}
			else
			{
				isElligible = (radialSqDist <= maxSqRadius);
#ifdef CC_DEBUG
				//m_theAssociatedCloud->setPointScalarValue(it->theIndex, sqrt(radialSqDist));
#endif
			}

			if (isElligible)
			{
				try
				{
					switch (process)
					{
					case RC_NEAREST_POINT:

						if (orderDist < 0)
						{
							//to give better chances to points that are closer to the viewer)
							double sqDist = ray.squareDistanceToOrigin(*P);
							orderDist = sqrt(radialSqDist * sqDist);
						}
					
						//keep only the 'nearest' point
						if (output.empty())
						{
							output.resize(1, PointDescriptor(P, it->theIndex, radialSqDist));
							smallestOrderDist = orderDist;
						}
						else
						{
							if (orderDist < smallestOrderDist)
							{
								output.back() = PointDescriptor(P, it->theIndex, radialSqDist);
								smallestOrderDist = orderDist;
							}
						}
						break;

					case RC_CLOSE_POINTS:
					 
						//store all the points that are close enough to the ray
						output.emplace_back(P, it->theIndex, radialSqDist);
						break;

					default:
						assert(false);
						return false;
					}
				}
				catch (const std::bad_alloc&)
				{
					//not enough memory
					return false;
				}
			}
		}
	}

	return true;
}
