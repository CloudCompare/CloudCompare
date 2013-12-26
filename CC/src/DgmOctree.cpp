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

#include "DgmOctree.h"

//local
#include "ReferenceCloud.h"
#include "GenericProgressCallback.h"
#include "GenericIndexedCloudPersist.h"
#include "CCMiscTools.h"
#include "ScalarField.h"

//system
#include <algorithm>
#include <string.h>
#include <assert.h>

//DGM: tests in progress
//#define COMPUTE_NN_SEARCH_STATISTICS
//#define ADAPTATIVE_BINARY_SEARCH
//#define OCTREE_TREE_TEST

#ifdef OCTREE_TREE_TEST

//! DGM TEST octree tree-like structure cell
class octreeTreeCell
{
public:
	uchar relativePos; //from 0 to 7
	octreeTreeCell* parent;
	octreeTreeCell** children; //up to 8 ones
	uchar childrenCount;

	//default constructor
	octreeTreeCell()
		: relativePos(0)
		, parent(0)
		, children(0)
		, childrenCount(0)
	{
	}

	virtual ~octreeTreeCell()
	{
		for (uchar i=0;i<childrenCount;++i)
			delete children[i];
		delete children;
	}

};

//! DGM TEST octree tree-like structure leaf cell
class octreeTreeCellLeaf : public octreeTreeCell
{
public:

	::std::vector<unsigned> pointIndexes;

	//default constructor
	octreeTreeCellLeaf() : octreeTreeCell()
	{
	}

};

static octreeTreeCell* s_root = 0;

octreeTreeCell* getCell(OctreeCellCodeType truncatedCellCode, uchar level)
{
	assert(s_root);
	octreeTreeCell* currentCell = s_root;
	uchar bitDec = 3*level;

	//we look for cell by descending down the tree (until we found it!)
	for (uchar currentLevel=0;currentLevel<level;++currentLevel)
	{
		bitDec -= 3;
		uchar childPos = (uchar)((truncatedCellCode >> bitDec) & 7);
		uchar i=0,count=currentCell->childrenCount;
		for (;i<count;++i)
		{
			if (currentCell->children[i]->relativePos == childPos)
			{
				currentCell = currentCell->children[i];
				if (!currentCell->childrenCount) //leafcell
					return currentCell;
				break;
			}
			else if (currentCell->children[i]->relativePos > childPos) //the child was not found?
				return 0;
		}
		if (i == count) //the child was not found?
			return 0;
	}

	return currentCell;
}

void getPointsInTreeCell(octreeTreeCell* cell, CCLib::DgmOctree::NeighboursSet& set, CCLib::GenericIndexedCloudPersist* cloud)
{
	if (cell->childrenCount) //no points in current cell, only children!
	{
		for (uchar i=0;i<cell->childrenCount;++i)
			getPointsInTreeCell(cell->children[i],set,cloud);
	}
	else //leaf cell
	{
		octreeTreeCellLeaf* leafCell = static_cast<octreeTreeCellLeaf*>(cell);
		assert(leafCell);

		//finally, we can grab points inside the leaf cell
		unsigned n = set.size();
		try
		{
			set.resize(n+leafCell->pointIndexes.size());
		}
		catch (.../*const std::bad_alloc&*/) //out of memory
		{
			//not enough memory --> DGM: what to do?
		}

		for (std::vector<unsigned>::const_iterator it_index = leafCell->pointIndexes.begin(); it_index!=leafCell->pointIndexes.end(); ++it_index,++n)
		{
			set[n].pointIndex = *it_index;
			set[n].point = cloud->getPointPersistentPtr(*it_index);
		}
	}
}

#endif

using namespace CCLib;

DgmOctree::DgmOctree(GenericIndexedCloudPersist* aCloud)
{
    assert(aCloud);

    clear();

    m_theAssociatedCloud = aCloud;
    m_dumpCloud = new ReferenceCloud(m_theAssociatedCloud);
}

DgmOctree::~DgmOctree()
{
    if (m_dumpCloud)
        delete m_dumpCloud;

#ifdef OCTREE_TREE_TEST
	if (s_root)
		delete s_root;
	s_root=0;
#endif
}

void DgmOctree::clear()
{
    //reset internal tables
    m_dimMin = m_pointsMin = m_dimMax = m_pointsMax = CCVector3(0,0,0);

    m_numberOfProjectedPoints = 0;
    m_thePointsAndTheirCellCodes.clear();

    memset(m_fillIndexes,0,sizeof(int)*(MAX_OCTREE_LEVEL+1)*6);
    memset(m_cellSize,0,sizeof(PointCoordinateType)*(MAX_OCTREE_LEVEL+2));
	updateCellCountTable();
}

int DgmOctree::build(GenericProgressCallback* progressCb)
{
    if (!m_thePointsAndTheirCellCodes.empty())
		clear();

    updateMinAndMaxTables();

    return genericBuild(progressCb);
}

int DgmOctree::build(const CCVector3& octreeMin, const CCVector3& octreeMax, const CCVector3* pointsMinFilter/*=0*/, const CCVector3* pointsMaxFilter/*=0*/, GenericProgressCallback* progressCb)
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

    //update the pre-computed 'cell size per level of subidivision' array
    updateCellSizeTable();

    //progress notification (optional)
	NormalizedProgress* nprogress = 0;
    if (progressCb)
    {
		nprogress = new NormalizedProgress(progressCb,pointCount,90); //first phase: 90% (we keep 10% for sort)
        progressCb->reset();
        progressCb->setMethodTitle("Build Octree");
        char infosBuffer[256];
        sprintf(infosBuffer,"Projecting %u points\nMax. depth: %i",pointCount,MAX_OCTREE_LEVEL);
        progressCb->setInfo(infosBuffer);
        progressCb->start();
    }

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
			int cellPos[3];
            getTheCellPosWhichIncludesThePoint(P,cellPos);

			//clipping X
            if (cellPos[0] < 0)
                cellPos[0] = 0;
            else if (cellPos[0] > MAX_OCTREE_LENGTH)
                cellPos[0] = MAX_OCTREE_LENGTH;
            //clipping Y
            if (cellPos[1] < 0)
                cellPos[1] = 0;
            else if (cellPos[1] > MAX_OCTREE_LENGTH)
                cellPos[1] = MAX_OCTREE_LENGTH;
            //clipping Z
            if (cellPos[2] < 0)
                cellPos[2] = 0;
            else if (cellPos[2] > MAX_OCTREE_LENGTH)
                cellPos[2] = MAX_OCTREE_LENGTH;

            it->theIndex = i;
            it->theCode = generateTruncatedCellCode(cellPos,MAX_OCTREE_LEVEL);

            ++it;
            ++m_numberOfProjectedPoints;
        }

		if (nprogress && !nprogress->oneStep())
		{
			m_thePointsAndTheirCellCodes.clear();
			m_numberOfProjectedPoints = 0;
			progressCb->stop();
			delete nprogress;
			return 0;
		}
    }

    if (m_numberOfProjectedPoints < pointCount)
        m_thePointsAndTheirCellCodes.resize(m_numberOfProjectedPoints); //smaller --> should always be ok

	if (progressCb)
		progressCb->setInfo("Sorting cells...");

    //we sort the 'cells' by ascending code order
	std::sort(m_thePointsAndTheirCellCodes.begin(),m_thePointsAndTheirCellCodes.end(),IndexAndCode::codeComp);

    //update the pre-computed 'number of cells per level of subidivision' array
    updateCellCountTable();

    //end of process notification
    if (progressCb)
    {
		progressCb->update(100.0f);

        char buffer[256];
		if (m_numberOfProjectedPoints == pointCount)
		{
            sprintf(buffer,"[Octree::build] Octree successfully built... %i points (ok)!",m_numberOfProjectedPoints);
		}
		else
		{
			if (m_numberOfProjectedPoints == 0)
				sprintf(buffer,"[Octree::build] Warning : no point projected in the Octree!");
			else
				sprintf(buffer,"[Octree::build] Warning: some points have been filtered out (%u/%u)",pointCount-m_numberOfProjectedPoints,pointCount);
		}

        progressCb->setInfo(buffer);
        progressCb->stop();

		if (nprogress)
		{
			delete nprogress;
			nprogress = 0;
		}
    }

#ifdef OCTREE_TREE_TEST
	if (m_numberOfProjectedPoints>1)
	{
		//Test build a tree from the cell code list
		octreeTreeCell* root = new octreeTreeCell();

		//begin 'recursion'
		uchar currentLevel = 0;
		uchar currentBitDec = GET_BIT_SHIFT(currentLevel);
		OctreeCellCodeType currentCode = INVALID_CELL_CODE;
		OctreeCellCodeType currentTruncatedCode = INVALID_CELL_CODE;
		std::vector<octreeTreeCell*> cellStack;
		octreeTreeCellLeaf* currentLeafCell = 0;
		cellStack.push_back(root);

		cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin();
		for (; p != m_thePointsAndTheirCellCodes.end(); ++p)
		{
			//different cell?
			if ((p->theCode >> currentBitDec) != currentTruncatedCode)
			{
				//let's try to find a common cell (there should be one! the root in the worst case)
				while (currentLevel > 0)
				{
					currentLevel--;
					currentBitDec+=3;
					cellStack.pop_back();
					if ((p->theCode >> currentBitDec) == (currentCode >> currentBitDec))
						break;
				}


				//now let's go deeper to find next leaf cell
				currentLeafCell = 0;
				while (!currentLeafCell)
				{
					++currentLevel;
					currentBitDec -= 3;
					currentTruncatedCode = (p->theCode >> currentBitDec);
					octreeTreeCell* newCell = 0;

					//a leaf cell is either a cell at maximum depth, or a cell with a unique point inside
					bool leafCell = (currentLevel == MAX_OCTREE_LEVEL || (p+1) == m_thePointsAndTheirCellCodes.end() || currentTruncatedCode != ((p+1)->theCode >> currentBitDec));
					if (!leafCell)
					{
						newCell = new octreeTreeCell();
					}
					else
					{
						currentLeafCell = new octreeTreeCellLeaf();
						newCell = currentLeafCell;
					}

					octreeTreeCell* parentCell = cellStack.back();
					newCell->parent = parentCell;
					newCell->relativePos = (currentTruncatedCode & 7); //1+2+4 (3 first bits)
					//make parent children array grow (one element)
					parentCell->children = (octreeTreeCell**)realloc(parentCell->children,sizeof(octreeTreeCell*)*(parentCell->childrenCount+1));
					parentCell->children[parentCell->childrenCount] = newCell;
					++parentCell->childrenCount;

					cellStack.push_back(newCell);
				}

				currentCode = p->theCode;
				currentTruncatedCode = (currentCode >> currentBitDec);
			}

			//add point to current cell
			assert(currentLeafCell);
			currentLeafCell->pointIndexes.push_back(p->theIndex);
		}

		assert(!s_root);
		s_root = root;
		//delete root;
		//root=0;

		//check
#ifdef _DEBUG
		for (it=m_thePointsAndTheirCellCodes.begin();it!=m_thePointsAndTheirCellCodes.end();++it)
			assert(getCell(it->theCode,MAX_OCTREE_LEVEL));
#endif
	}
#endif

    return static_cast<int>(m_numberOfProjectedPoints);
}

void DgmOctree::updateMinAndMaxTables()
{
    if (!m_theAssociatedCloud)
        return;

    m_theAssociatedCloud->getBoundingBox(m_pointsMin.u,m_pointsMax.u);
	m_dimMin = m_pointsMin;
	m_dimMax = m_pointsMax;

    CCMiscTools::MakeMinAndMaxCubical(m_dimMin,m_dimMax);
}

void DgmOctree::updateCellSizeTable()
{
    //update the cell dimension for each subidivision level
    for (int dim=0; dim<3; ++dim)
    {
        PointCoordinateType dd = m_dimMax[dim]-m_dimMin[dim];
        PointCoordinateType ll = std::max<PointCoordinateType>(m_pointsMin[dim]-m_dimMin[dim],0);
        PointCoordinateType lr = std::max<PointCoordinateType>(m_dimMax[dim]-m_pointsMax[dim],static_cast<PointCoordinateType>(ZERO_TOLERANCE)); //to make sure that ceil(lr/dd)>0 (see below)

		for (int k=0; k<=MAX_OCTREE_LEVEL; k++)
        {
            if (dim == 0)
                m_cellSize[k] = dd;

            m_fillIndexes[k*6+dim] = static_cast<int>(floor(ll/dd));			//number of empty cells on the 'left'
            m_fillIndexes[k*6+3+dim] = (1<<k)-static_cast<int>(ceil(lr/dd));	//number of empty cells on the 'rigth'
			
			assert(m_fillIndexes[k*6+3+dim] >= m_fillIndexes[k*6+dim]);

			dd /= 2;
        }
		if (dim == 0)
			m_cellSize[MAX_OCTREE_LEVEL+1] = m_cellSize[MAX_OCTREE_LEVEL] / 2; //usefull for some algorithms
    }
}

void DgmOctree::updateCellCountTable()
{
	//level 0 is just the octree bounding-box
	for (uchar i=0; i<=MAX_OCTREE_LEVEL; ++i)
		computeCellsStatistics(i);
}

void DgmOctree::computeCellsStatistics(uchar level)
{
	assert(level<=MAX_OCTREE_LEVEL);

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
    uchar bitDec = GET_BIT_SHIFT(level);

	//iterator on octree elements
    cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin();

	//we init scan with first element
    OctreeCellCodeType predCode = (p->theCode >> bitDec);
    unsigned counter = 0;
	unsigned cellCounter = 0;
	unsigned maxCellPop = 0;
	double sum=0.0, sum2=0.0;

    for (; p != m_thePointsAndTheirCellCodes.end(); ++p)
    {
        OctreeCellCodeType currentCode = (p->theCode >> bitDec);
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
	m_stdDevCellPopulation[level] = sqrt(sum2 - m_averageCellPopulation[level]*m_averageCellPopulation[level])/static_cast<double>(counter);
}

//! Pre-computed cell codes for all potential cell positions (along a unique dimension)
struct MonoDimensionalCellCodes
{
	//! Total number of positions/values (1024 at level 10, 2M. at level 21)
	static const int VALUE_COUNT = OCTREE_LENGTH(CCLib::DgmOctree::MAX_OCTREE_LEVEL);
	
	//! Default initialization
	MonoDimensionalCellCodes()
	{
		//we compute all possible values for cell codes
		//(along a unique dimension, the other ones are just shifted)
		for (int value = 0; value < VALUE_COUNT; ++value)
		{
			int mask = VALUE_COUNT;
			CCLib::DgmOctree::OctreeCellCodeType code = 0;
			for (uchar k=0; k<CCLib::DgmOctree::MAX_OCTREE_LEVEL; k++)
			{
				mask >>= 1;
				code <<= 3;
				if (value & mask)
					code |= 1;
			}
			values[value] = code;
		}

		//we compute all possible masks as well! (all dimensions)
		//CCLib::DgmOctree::OctreeCellCodeType baseMask = (1 << (3*CCLib::DgmOctree::MAX_OCTREE_LEVEL));
		//for (int level=CCLib::DgmOctree::MAX_OCTREE_LEVEL; level>=0; --level)
		//{
		//	masks[level] = baseMask-1;
		//	baseMask >>= 3;
		//}
	}

	//! Mono-dimensional cell codes
	CCLib::DgmOctree::OctreeCellCodeType values[VALUE_COUNT];
	
	//CCLib::DgmOctree::OctreeCellCodeType masks[CCLib::DgmOctree::MAX_OCTREE_LEVEL+1];
};
static MonoDimensionalCellCodes PRE_COMPUTED_POS_CODES;

DgmOctree::OctreeCellCodeType DgmOctree::generateTruncatedCellCode(const int pos[], uchar level) const
{
	assert(		pos[0] >= 0 && pos[0] < MonoDimensionalCellCodes::VALUE_COUNT
			&&	pos[1] >= 0 && pos[1] < MonoDimensionalCellCodes::VALUE_COUNT
			&&	pos[2] >= 0 && pos[2] < MonoDimensionalCellCodes::VALUE_COUNT );

	return	(	PRE_COMPUTED_POS_CODES.values[pos[0]]
			|	(PRE_COMPUTED_POS_CODES.values[pos[1]] << 1)
			|	(PRE_COMPUTED_POS_CODES.values[pos[2]] << 2)
			) >> GET_BIT_SHIFT(level);
}

#ifndef OCTREE_CODES_64_BITS
DgmOctree::OctreeCellCodeType DgmOctree::generateTruncatedCellCode(const short pos[], uchar level) const
{
	assert(		pos[0] >= 0 && pos[0] < MonoDimensionalCellCodes::VALUE_COUNT
			&&	pos[1] >= 0 && pos[1] < MonoDimensionalCellCodes::VALUE_COUNT
			&&	pos[2] >= 0 && pos[2] < MonoDimensionalCellCodes::VALUE_COUNT );

	return	(	PRE_COMPUTED_POS_CODES.values[pos[0]]
			|	(PRE_COMPUTED_POS_CODES.values[pos[1]] << 1)
			|	(PRE_COMPUTED_POS_CODES.values[pos[2]] << 2)
			) >> GET_BIT_SHIFT(level);
}
#endif

static inline DgmOctree::OctreeCellCodeType GenerateCellCodeForDim(int pos)
{
	return PRE_COMPUTED_POS_CODES.values[pos];
}

void DgmOctree::getBoundingBox(PointCoordinateType bbMin[], PointCoordinateType bbMax[]) const
{
    memcpy(bbMin, m_dimMin.u, sizeof(PointCoordinateType)*3);
    memcpy(bbMax, m_dimMax.u, sizeof(PointCoordinateType)*3);
}

void DgmOctree::getCellPos(OctreeCellCodeType code, uchar level, int pos[], bool isCodeTruncated) const
{
	//binary shift for cell code truncation
	if (!isCodeTruncated)
		code >>= GET_BIT_SHIFT(level);

	pos[0]= pos[1] = pos[2] = 0;

	int bitMask = 1;
	for (uchar k=0; k<level; ++k)
	{
		if (code & 4)
			pos[2] |= bitMask;
		if (code & 2)
			pos[1] |= bitMask;
		if (code & 1)
			pos[0] |= bitMask;

		code >>= 3;
		bitMask <<= 1;
	}
}

void DgmOctree::computeCellLimits(OctreeCellCodeType code, uchar level, PointCoordinateType cellMin[], PointCoordinateType cellMax[], bool isCodeTruncated) const
{
    int cellPos[3];
    getCellPos(code,level,cellPos,isCodeTruncated);
    
	const PointCoordinateType& cs = getCellSize(level);

    cellMin[0] = m_dimMin[0] + cs*static_cast<PointCoordinateType>(cellPos[0]);
    cellMin[1] = m_dimMin[1] + cs*static_cast<PointCoordinateType>(cellPos[1]);
    cellMin[2] = m_dimMin[2] + cs*static_cast<PointCoordinateType>(cellPos[2]);

	cellMax[0] = cellMin[0] + cs;
    cellMax[1] = cellMin[1] + cs;
    cellMax[2] = cellMin[2] + cs;
}

unsigned DgmOctree::getCellIndex(OctreeCellCodeType cellCode, uchar bitDec, bool isCodeTruncated) const
{
    //query cell index
    OctreeCellCodeType maskedCode = (isCodeTruncated ? cellCode : cellCode >> bitDec);
    //first cell index
    OctreeCellCodeType tempCode = (m_thePointsAndTheirCellCodes[0].theCode >> bitDec);

    //lucky?
    if (maskedCode == tempCode)
        return 0;
    //as 'tempCode' is the first one here, and codes are sorted, there aren't any cell with a smaller code!
    else if (maskedCode < tempCode)
        return m_numberOfProjectedPoints;

    unsigned begin=0, end=m_numberOfProjectedPoints-1;

    while (true)
    {
        //case: "end == begin" or "end == begin+1"
        assert(end>=begin);
        if (end<begin+2)
        {
            if ((m_thePointsAndTheirCellCodes[begin].theCode >> bitDec) == maskedCode)
                return begin;

            if ((m_thePointsAndTheirCellCodes[end].theCode >> bitDec) == maskedCode)
                return end;

            //otherwise, there's no match!
            return m_numberOfProjectedPoints;
        }

        unsigned middle = (begin+end)>>1;
        tempCode = (m_thePointsAndTheirCellCodes[middle].theCode >> bitDec);

        //hit!
        if (tempCode == maskedCode)
        {
            //if the precedent point doesn't correspond, then we fall directly on the first good one!
            if ((m_thePointsAndTheirCellCodes[middle-1].theCode >> bitDec) != maskedCode)
                return middle;

            end = middle;
        }
        else
        {
            if (tempCode < maskedCode)
                begin = middle;
            else //if (tempCode > maskedCode)
                end = middle;
        }
    }

    //shouldn't get here!
    return m_numberOfProjectedPoints;
}

//optimized version with profiling
#ifdef COMPUTE_NN_SEARCH_STATISTICS
static double s_jumps = 0.0;
static double s_binarySearchCount = 0.0;
#endif

#ifdef ADAPTATIVE_BINARY_SEARCH
unsigned DgmOctree::getCellIndex(OctreeCellCodeType truncatedCellCode, uchar bitDec, unsigned begin, unsigned end) const
{
    assert(truncatedCellCode != INVALID_CELL_CODE);
    assert(end>=begin);
    assert(end<m_numberOfProjectedPoints);

#ifdef COMPUTE_NN_SEARCH_STATISTICS
	s_binarySearchCount += 1;
#endif

	//if query cell code is lower than or equal to the first octree cell code, then it's
    //either the good one or there's no match
    OctreeCellCodeType beginCode = (m_thePointsAndTheirCellCodes[begin].theCode >> bitDec);
    if (truncatedCellCode < beginCode)
        return m_numberOfProjectedPoints;
	else if (truncatedCellCode == beginCode)
		return begin;

    //if query cell code is higher than the last octree cell code, then there's no match
    OctreeCellCodeType endCode = (m_thePointsAndTheirCellCodes[end].theCode >> bitDec);
    if (truncatedCellCode > endCode)
        return m_numberOfProjectedPoints;

    while (true)
    {
		float centralPoint = 0.5f + 0.75f*(static_cast<float>(truncatedCellCode-beginCode)/(-0.5f)); //0.75 = speed coef (empirical)
		unsigned middle = begin + static_cast<unsigned>(centralPoint*float(end-begin));
		OctreeCellCodeType middleCode = (m_thePointsAndTheirCellCodes[middle].theCode >> bitDec);

        if (middleCode < truncatedCellCode)
        {
			//no more cell inbetween?
            if (middle == begin)
                return m_numberOfProjectedPoints;

			begin = middle;
			beginCode = middleCode;
        }
        else if (middleCode > truncatedCellCode)
        {
			//no more cell inbetween?
            if (middle == begin)
                return m_numberOfProjectedPoints;

			end = middle;
			endCode = middleCode;
        }
        else
        {
            //if the precedent point doesn't correspond, then we have just found the first good one!
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

unsigned DgmOctree::getCellIndex(OctreeCellCodeType truncatedCellCode, uchar bitDec, unsigned begin, unsigned end) const
{
    assert(truncatedCellCode != INVALID_CELL_CODE);
    assert(end >= begin);
    assert(end < m_numberOfProjectedPoints);

	//if query cell code is lower than or equal to the first octree cell code, then it's
	//either the good one or there's no match
    OctreeCellCodeType middleCode = (m_thePointsAndTheirCellCodes[begin].theCode >> bitDec);
    if (truncatedCellCode <= middleCode)
        return (truncatedCellCode == middleCode ? begin : m_numberOfProjectedPoints);

    //if query cell code is higher than the last octree cell code, then there's no match
    middleCode = (m_thePointsAndTheirCellCodes[end].theCode >> bitDec);
    if (truncatedCellCode > middleCode)
        return m_numberOfProjectedPoints;

#ifdef COMPUTE_NN_SEARCH_STATISTICS
	s_binarySearchCount += 1;
#endif

    while (true)
    {
        unsigned middle = ((begin+end) >> 1);
        middleCode = (m_thePointsAndTheirCellCodes[middle].theCode >> bitDec);

        if (middleCode < truncatedCellCode)
        {
			//no more cell inbetween?
            if (middle == begin)
                return m_numberOfProjectedPoints;
			begin = middle;
        }
        else if (middleCode > truncatedCellCode)
        {
			//no more cell inbetween?
            if (middle == begin)
                return m_numberOfProjectedPoints;
			end = middle;
        }
        else
        {
            //if the precedent point doesn't correspond, then we have just found the first good one!
            if ((m_thePointsAndTheirCellCodes[middle-1].theCode >> bitDec) != truncatedCellCode)
                return middle;
            end = middle;
        }

#ifdef COMPUTE_NN_SEARCH_STATISTICS
		s_jumps += 1.0;
#endif
    }

    //we shouldn't get there!
    return m_numberOfProjectedPoints;
}
#endif

unsigned DgmOctree::findPointNeighbourhood(const CCVector3* queryPoint,
											ReferenceCloud* Yk,
											unsigned maxNumberOfNeighbors,
											uchar level,
											ScalarType &maxSquareDist,
											ScalarType maxSearchDist/*=-1.0*/) const
{
	assert(queryPoint);
    NearestNeighboursSearchStruct nNSS;
    nNSS.queryPoint							= *queryPoint;
    nNSS.level								= level;
    nNSS.minNumberOfNeighbors				= maxNumberOfNeighbors;
    nNSS.alreadyVisitedNeighbourhoodSize	= 0;
    bool inbounds=false;
    getTheCellPosWhichIncludesThePoint(&nNSS.queryPoint,nNSS.cellPos,nNSS.level,inbounds);
    computeCellCenter(nNSS.cellPos,level,nNSS.cellCenter);
    nNSS.truncatedCellCode = (inbounds ? generateTruncatedCellCode(nNSS.cellPos,nNSS.level) : INVALID_CELL_CODE);
    nNSS.maxSearchSquareDist = (maxSearchDist >= 0 ? maxSearchDist*maxSearchDist : (ScalarType)-1.0);

    //special case: N=1
    if (maxNumberOfNeighbors == 1)
    {
        maxSquareDist = findTheNearestNeighborStartingFromCell(nNSS);
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
    if (nnFound == 0)
    {
        maxSquareDist = -1.0;
        return 0;
    }

    //nnFound can be superior to maxNumberOfNeighbors
    //so we only keep the 'maxNumberOfNeighbors' firsts
    nnFound = std::min(nnFound,maxNumberOfNeighbors);

    for (unsigned j=0; j<nnFound; ++j)
        Yk->addPointIndex(nNSS.pointsInNeighbourhood[j].pointIndex);

    maxSquareDist = nNSS.pointsInNeighbourhood.back().squareDist;

    return nnFound;
}

void DgmOctree::getCellDistanceFromBorders(	const int* cellPos,
											uchar level,
											int* cellDists) const
{
    const int* fillIndexes = m_fillIndexes+6*level;

    int* _cellDists = cellDists;
	*_cellDists++ = cellPos[0]-fillIndexes[0];
    *_cellDists++ = fillIndexes[3]-cellPos[0];
	*_cellDists++ = cellPos[1]-fillIndexes[1];
    *_cellDists++ = fillIndexes[4]-cellPos[1];
	*_cellDists++ = cellPos[2]-fillIndexes[2];
    *_cellDists++ = fillIndexes[5]-cellPos[2];
}

void DgmOctree::getCellDistanceFromBorders(	const int* cellPos,
											uchar level,
											int neighbourhoodLength,
											int* limits) const
{
    const int* fillIndexes = m_fillIndexes+6*level;

    int* _limits = limits;
    for (int dim=0; dim<3; ++dim)
    {
        //min dim.
		{
			int a = cellPos[dim]-fillIndexes[dim];
			if (a < -neighbourhoodLength)
				a = -neighbourhoodLength;
			else if (a > neighbourhoodLength)
				a = neighbourhoodLength;
			*_limits++ = a;
		}

        //max dim.
		{
			int b = fillIndexes[3+dim]-cellPos[dim];
			if (b < -neighbourhoodLength)
				b = -neighbourhoodLength;
			else if (b > neighbourhoodLength)
				b = neighbourhoodLength;
			*_limits++ = b;
		}
    }
}

void DgmOctree::getNeighborCellsAround(const int cellPos[],
										cellIndexesContainer &neighborCellsIndexes,
										int neighbourhoodLength,
										uchar level) const
{
    assert(neighbourhoodLength > 0);

    //get distance form cell to octree neighbourhood borders
    int limits[6];
	getCellDistanceFromBorders(cellPos,level,neighbourhoodLength,limits);

	//limits are expressed in terms of cells at the CURRENT 'level'!
    const int &iMin = limits[0];
    const int &iMax = limits[1];
    const int &jMin = limits[2];
    const int &jMax = limits[3];
    const int &kMin = limits[4];
    const int &kMax = limits[5];

	//backup of the previous binary search result (acceleration)
    OctreeCellCodeType old_c2 = 0;
    unsigned old_index = 0;

    //binary shift for cell code truncation
    const uchar bitDec = GET_BIT_SHIFT(level);

    for (int i=-iMin; i<=iMax; i++)
    {
        bool iBorder = (abs(i) == neighbourhoodLength); //test: are we on a plane of equation 'X = +/-neighbourhoodLength'?
        OctreeCellCodeType c0 = GenerateCellCodeForDim(cellPos[0]+i);

        for (int j=-jMin; j<=jMax; j++)
        {
            OctreeCellCodeType c1 = c0 | (GenerateCellCodeForDim(cellPos[1]+j)<<1);

            if (iBorder || (abs(j) == neighbourhoodLength)) //test: are we already on one of the X or Y borders?
            {
                for (int k=-kMin; k<=kMax; k++)
                {
                    OctreeCellCodeType c2 = c1 | (GenerateCellCodeForDim(cellPos[2]+k)<<2);

                    unsigned index = (old_c2<c2 ? getCellIndex(c2,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(c2,bitDec,0,old_index));
                    if (index < m_numberOfProjectedPoints)
                    {
                        neighborCellsIndexes.push_back(index);
                        old_index = index;
                        old_c2 = c2;
                    }
                }

            }
            else //otherwise we are inside the neighbourhood
            {
                if (kMin == neighbourhoodLength) //test: does the plane of equation 'Z = -neighbourhoodLength' is inside the octree box?
                {
                    OctreeCellCodeType c2 = c1 | (GenerateCellCodeForDim(cellPos[2]-neighbourhoodLength)<<2);

                    unsigned index = (old_c2<c2 ? getCellIndex(c2,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(c2,bitDec,0,old_index));
                    if (index < m_numberOfProjectedPoints)
                    {
                        neighborCellsIndexes.push_back(index);
                        old_index = index;
                        old_c2 = c2;
                    }
                }

                if (kMax == neighbourhoodLength) //test: does the plane of equation 'Z = +neighbourhoodLength' is inside the octree box?
                {
                    OctreeCellCodeType c2 = c1+(GenerateCellCodeForDim(cellPos[2]+kMax)<<2);

                    unsigned index = (old_c2<c2 ? getCellIndex(c2,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(c2,bitDec,0,old_index));
                    if (index < m_numberOfProjectedPoints)
                    {
                        neighborCellsIndexes.push_back(index);
                        old_index = index;
                        old_c2 = c2;
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
    getCellDistanceFromBorders(nNSS.cellPos,nNSS.level,neighbourhoodLength,limits);

	//limits are expressed in terms of cells at the CURRENT 'level'!
    const int &iMin = limits[0];
    const int &iMax = limits[1];
    const int &jMin = limits[2];
    const int &jMax = limits[3];
    const int &kMin = limits[4];
    const int &kMax = limits[5];

	//backup of the previous binary search result (acceleration)
    OctreeCellCodeType old_c2 = 0;
	unsigned old_index = 0;

    //binary shift for cell code truncation
    const uchar bitDec = GET_BIT_SHIFT(nNSS.level);

    for (int i=-iMin; i<=iMax; i++)
    {
        bool iBorder = (abs(i) == neighbourhoodLength); //test: are we on a plane of equation 'X = +/-neighbourhoodLength'?
        OctreeCellCodeType c0 = GenerateCellCodeForDim(nNSS.cellPos[0]+i);

        for (int j=-jMin; j<=jMax; j++)
        {
            OctreeCellCodeType c1 = c0 | (GenerateCellCodeForDim(nNSS.cellPos[1]+j)<<1);

            //si i ou j est maximal
            if (iBorder || (abs(j) == neighbourhoodLength)) //test: are we already on one of the X or Y borders?
            {
                for (int k=-kMin; k<=kMax; k++)
                {
                    OctreeCellCodeType c2 = c1 | (GenerateCellCodeForDim(nNSS.cellPos[2]+k)<<2);

                    unsigned index = (old_c2<c2 ? getCellIndex(c2,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(c2,bitDec,0,old_index));
                    if (index < m_numberOfProjectedPoints)
                    {
						//we increase 'pointsInNeighbourCells' capacity with average cell size
						try
						{
							nNSS.pointsInNeighbourhood.reserve(nNSS.pointsInNeighbourhood.size()+static_cast<unsigned>(ceil(m_averageCellPopulation[nNSS.level])));
						}
						catch (.../*const std::bad_alloc&*/) //out of memory
						{
							//DGM TODO: Shall we stop? shall we try to go on, as we are not sure that we will actually need this much points?
							assert(false);
						}
                        for (cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+index; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == c2); ++p)
                        {
                            if (!getOnlyPointsWithValidScalar || ScalarField::ValidValue(m_theAssociatedCloud->getPointScalarValue(p->theIndex)))
                            {
								PointDescriptor newPoint(m_theAssociatedCloud->getPointPersistentPtr(p->theIndex),p->theIndex);
								nNSS.pointsInNeighbourhood.push_back(newPoint);
							}
                        }

                        old_index = index;
                        old_c2 = c2;
                    }
                }

            }
            else //otherwise we are inside the neighbourhood
            {
                if (kMin == neighbourhoodLength) //test: does the plane of equation 'Z = -neighbourhoodLength' is inside the octree box?
                {
                    OctreeCellCodeType c2 = c1 | (GenerateCellCodeForDim(nNSS.cellPos[2]-neighbourhoodLength)<<2);
                    unsigned index = (old_c2<c2 ? getCellIndex(c2,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(c2,bitDec,0,old_index));
                    if (index < m_numberOfProjectedPoints)
                    {
						//we increase 'nNSS.pointsInNeighbourhood' capacity with average cell size
						try
						{
							nNSS.pointsInNeighbourhood.reserve(nNSS.pointsInNeighbourhood.size()+static_cast<unsigned>(ceil(m_averageCellPopulation[nNSS.level])));
						}
						catch (.../*const std::bad_alloc&*/) //out of memory
						{
							//DGM TODO: Shall we stop? shall we try to go on, as we are not sure that we will actually need this much points?
							assert(false);
						}
                        for (cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+index; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == c2); ++p)
                        {
                            if (!getOnlyPointsWithValidScalar || ScalarField::ValidValue(m_theAssociatedCloud->getPointScalarValue(p->theIndex)))
                            {
								PointDescriptor newPoint(m_theAssociatedCloud->getPointPersistentPtr(p->theIndex),p->theIndex);
								nNSS.pointsInNeighbourhood.push_back(newPoint);
							}
                        }

                        old_index = index;
                        old_c2 = c2;
                    }
                }

                if (kMax == neighbourhoodLength) //test: does the plane of equation 'Z = +neighbourhoodLength' is inside the octree box? (note that neighbourhoodLength > 0)
                {
                    OctreeCellCodeType c2 = c1 | (GenerateCellCodeForDim(nNSS.cellPos[2]+neighbourhoodLength)<<2);
                    unsigned index = (old_c2<c2 ? getCellIndex(c2,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(c2,bitDec,0,old_index));
                    if (index < m_numberOfProjectedPoints)
                    {
						//we increase 'nNSS.pointsInNeighbourhood' capacity with average cell size
						try
						{
							nNSS.pointsInNeighbourhood.reserve(nNSS.pointsInNeighbourhood.size()+static_cast<unsigned>(ceil(m_averageCellPopulation[nNSS.level])));
						}
						catch (.../*const std::bad_alloc&*/) //out of memory
						{
							//DGM TODO: Shall we stop? shall we try to go on, as we are not sure that we will actually need this much points?
							assert(false);
						}
                        for (cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+index; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == c2); ++p)
                        {
                            if (!getOnlyPointsWithValidScalar || ScalarField::ValidValue(m_theAssociatedCloud->getPointScalarValue(p->theIndex)))
                            {
								PointDescriptor newPoint(m_theAssociatedCloud->getPointPersistentPtr(p->theIndex),p->theIndex);
								nNSS.pointsInNeighbourhood.push_back(newPoint);
							}
                        }

                        old_index = index;
                        old_c2 = c2;
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
    assert(minNeighbourhoodLength>=nNSS.alreadyVisitedNeighbourhoodSize);

    //binary shift for cell code truncation
    uchar bitDec = GET_BIT_SHIFT(nNSS.level);
	CellDescriptor cellDesc;

	if (minNeighbourhoodLength == 0) //special case
	{
		//we don't look if the cell is inside the octree as it is generally the case
		unsigned index = getCellIndex(nNSS.truncatedCellCode,bitDec,true);
		if (index < m_numberOfProjectedPoints)
		{
			//add cell descriptor to cells list
			cellDesc.center = CCVector3(nNSS.cellCenter);
			cellDesc.index = 0;
			nNSS.cellsInNeighbourhood.push_back(cellDesc);

			for (cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+index; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == nNSS.truncatedCellCode); ++p)
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
    OctreeCellCodeType old_c2 = 0;
	int currentCellPos[3];

	//first part: i in [-maxNL,-minNL] and (j,k) in [-maxNL,maxNL]
	if (iMinAbs>=minNeighbourhoodLength)
	{
		for (int v0=nNSS.cellPos[0]-iMinAbs; v0<=nNSS.cellPos[0]-minNeighbourhoodLength; ++v0)
		{
			OctreeCellCodeType c0 = GenerateCellCodeForDim(v0);
			currentCellPos[0]=v0;
			for (int v1=nNSS.cellPos[1]-jMinAbs; v1<=nNSS.cellPos[1]+jMaxAbs; ++v1)
			{
				OctreeCellCodeType c1 = c0 | (GenerateCellCodeForDim(v1)<<1);
				currentCellPos[1]=v1;
				for (int v2=nNSS.cellPos[2]-kMinAbs; v2<=nNSS.cellPos[2]+kMaxAbs; ++v2)
				{
                    OctreeCellCodeType c2 = c1 | (GenerateCellCodeForDim(v2)<<2);

					//look for corresponding cell
                    unsigned index = (old_c2<c2 ? getCellIndex(c2,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(c2,bitDec,0,old_index));
                    if (index < m_numberOfProjectedPoints)
                    {
						//add cell descriptor to cells list
						currentCellPos[2]=v2;
						computeCellCenter(currentCellPos,nNSS.level,cellDesc.center.u);
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
	if (iMaxAbs>=minNeighbourhoodLength)
	{
		for (int v0=nNSS.cellPos[0]+minNeighbourhoodLength; v0<=nNSS.cellPos[0]+iMaxAbs; ++v0)
		{
			OctreeCellCodeType c0 = GenerateCellCodeForDim(v0);
			currentCellPos[0]=v0;
			for (int v1=nNSS.cellPos[1]-jMinAbs; v1<=nNSS.cellPos[1]+jMaxAbs; ++v1)
			{
				OctreeCellCodeType c1 = c0 | (GenerateCellCodeForDim(v1)<<1);
				currentCellPos[1]=v1;
				for (int v2=nNSS.cellPos[2]-kMinAbs; v2<=nNSS.cellPos[2]+kMaxAbs; ++v2)
				{
                    OctreeCellCodeType c2 = c1 | (GenerateCellCodeForDim(v2)<<2);

					//look for corresponding cell
                    unsigned index = (old_c2<c2 ? getCellIndex(c2,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(c2,bitDec,0,old_index));
                    if (index < m_numberOfProjectedPoints)
                    {
						//add cell descriptor to cells list
						currentCellPos[2]=v2;
						computeCellCenter(currentCellPos,nNSS.level,cellDesc.center.u);
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
		for (int v1=nNSS.cellPos[1]-jMinAbs; v1<=nNSS.cellPos[1]-minNeighbourhoodLength; ++v1)
		{
			OctreeCellCodeType c1 = (GenerateCellCodeForDim(v1) << 1);
			currentCellPos[1]=v1;
			for (int v0=nNSS.cellPos[0]-iMinAbs; v0<=nNSS.cellPos[0]+iMaxAbs; ++v0)
			{
				OctreeCellCodeType c0 = c1 | GenerateCellCodeForDim(v0);
				currentCellPos[0]=v0;
				for (int v2=nNSS.cellPos[2]-kMinAbs; v2<=nNSS.cellPos[2]+kMaxAbs; ++v2)
				{
                    OctreeCellCodeType c2 = c0 | (GenerateCellCodeForDim(v2)<<2);

					//look for corresponding cell
                    unsigned index = (old_c2<c2 ? getCellIndex(c2,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(c2,bitDec,0,old_index));
                    if (index < m_numberOfProjectedPoints)
                    {
						//add cell descriptor to cells list
						currentCellPos[2]=v2;
						computeCellCenter(currentCellPos,nNSS.level,cellDesc.center.u);
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
		for (int v1=nNSS.cellPos[1]+minNeighbourhoodLength; v1<=nNSS.cellPos[1]+jMaxAbs; ++v1)
		{
			OctreeCellCodeType c1 = (GenerateCellCodeForDim(v1) << 1);
			currentCellPos[1]=v1;
			for (int v0=nNSS.cellPos[0]-iMinAbs; v0<=nNSS.cellPos[0]+iMaxAbs; ++v0)
			{
				OctreeCellCodeType c0 = c1 | GenerateCellCodeForDim(v0);
				currentCellPos[0]=v0;
				for (int v2=nNSS.cellPos[2]-kMinAbs; v2<=nNSS.cellPos[2]+kMaxAbs; ++v2)
				{
                    OctreeCellCodeType c2 = c0 | (GenerateCellCodeForDim(v2)<<2);

					//look for corresponding cell
                    unsigned index = (old_c2<c2 ? getCellIndex(c2,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(c2,bitDec,0,old_index));
                    if (index < m_numberOfProjectedPoints)
                    {
						//add cell descriptor to cells list
						currentCellPos[2]=v2;
						computeCellCenter(currentCellPos,nNSS.level,cellDesc.center.u);
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
		for (int v2=nNSS.cellPos[2]-kMinAbs; v2<=nNSS.cellPos[2]-minNeighbourhoodLength; ++v2)
		{
			OctreeCellCodeType c2 = (GenerateCellCodeForDim(v2)<<2);
			currentCellPos[2]=v2;
			for (int v0=nNSS.cellPos[0]-iMinAbs; v0<=nNSS.cellPos[0]+iMaxAbs; ++v0)
			{
				OctreeCellCodeType c0 = c2 | GenerateCellCodeForDim(v0);
				currentCellPos[0]=v0;
				for (int v1=nNSS.cellPos[1]-jMinAbs; v1<=nNSS.cellPos[1]+jMaxAbs; ++v1)
				{
					OctreeCellCodeType c1 = c0 | (GenerateCellCodeForDim(v1)<<1);
					//look for corresponding cell
					unsigned index = (old_c2<c1 ? getCellIndex(c1,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(c1,bitDec,0,old_index));
					if (index < m_numberOfProjectedPoints)
					{
						//add cell descriptor to cells list
						currentCellPos[1]=v1;
						computeCellCenter(currentCellPos,nNSS.level,cellDesc.center.u);
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
		for (int v2=nNSS.cellPos[2]+minNeighbourhoodLength; v2<=nNSS.cellPos[2]+kMaxAbs; ++v2)
		{
			OctreeCellCodeType c2 = (GenerateCellCodeForDim(v2)<<2);
			currentCellPos[2]=v2;
			for (int v0=nNSS.cellPos[0]-iMinAbs; v0<=nNSS.cellPos[0]+iMaxAbs; ++v0)
			{
				OctreeCellCodeType c0 = c2 | GenerateCellCodeForDim(v0);
				currentCellPos[0]=v0;
				for (int v1=nNSS.cellPos[1]-jMinAbs; v1<=nNSS.cellPos[1]+jMaxAbs; ++v1)
				{
					OctreeCellCodeType c1 = c0 | (GenerateCellCodeForDim(v1)<<1);
					//look for corresponding cell
					unsigned index = (old_c2<c1 ? getCellIndex(c1,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(c1,bitDec,0,old_index));
					if (index < m_numberOfProjectedPoints)
					{
						//add cell descriptor to cells list
						currentCellPos[1]=v1;
						computeCellCenter(currentCellPos,nNSS.level,cellDesc.center.u);
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

//trouve le point le plus proche d'un point donne via l'octree et renvoie la distance a celui-ci
ScalarType DgmOctree::findTheNearestNeighborStartingFromCell(NearestNeighboursSearchStruct &nNSS) const
{
    //binary shift for cell code truncation
    uchar bitDec = GET_BIT_SHIFT(nNSS.level);

    //cell size at the current level of subdivison
    const PointCoordinateType& cs = getCellSize(nNSS.level);

    //already visited cells (relative distance to the cell that includes the query point)
    int visitedCellDistance = nNSS.alreadyVisitedNeighbourhoodSize;
    //minimum (a priori) relative distance to get elligible points (see 'elligibleDist' below)
    int elligibleCellDistance = visitedCellDistance;

    //if we have not already looked for the first cell (the one including the query point)?
    if (visitedCellDistance == 0)
    {
        //visitedCellDistance == 0 means that no cell has ever been processed! No cell should be inside 'minimalCellsSetToVisit'
        assert(nNSS.minimalCellsSetToVisit.empty());

        //check for existence of 'including' cell
        unsigned index = (nNSS.truncatedCellCode == INVALID_CELL_CODE ? m_numberOfProjectedPoints : getCellIndex(nNSS.truncatedCellCode,bitDec,true));

        visitedCellDistance = 1;

        //it this cell does exsist...
        if (index < m_numberOfProjectedPoints)
        {
            //we add it to the 'cells to visit' set
            nNSS.minimalCellsSetToVisit.push_back(index);
            elligibleCellDistance = 1;
        }
        //otherwise, we may be very far from the nearest octree cell
        //(let's try to get there asap)
        else
        {
            //fill indexes for current level
            const int* _fillIndexes = m_fillIndexes+6*nNSS.level;
            int distToBorder, diagonalDistance = 0;
            for (int dim=0; dim<3; ++dim)
            {
                //distance to min border of octree along each axis
                distToBorder = *_fillIndexes-nNSS.cellPos[dim];
                //if its negative, lets look the other side
                if (distToBorder<0)
                    //distance to max border of octree along each axis
                    distToBorder = nNSS.cellPos[dim]-_fillIndexes[3];

                if (distToBorder>0)
                {
                    visitedCellDistance = std::max(distToBorder,visitedCellDistance);
                    diagonalDistance += distToBorder*distToBorder;
                }

                //next dimension
                ++_fillIndexes;
            }

            //the nearest octree cell
            diagonalDistance = static_cast<int>(ceil(sqrt(static_cast<float>(diagonalDistance))));
            elligibleCellDistance = std::max(diagonalDistance,1);

            if (nNSS.maxSearchSquareDist >= 0)
            {
                //Distance to the nearest point
                ScalarType minDist = (ScalarType)(elligibleCellDistance-1) * (ScalarType)cs;
                //if we are already outside of the search limit, we can quit
                if (minDist*minDist > nNSS.maxSearchSquareDist)
                {
                    return -1.0;
                    //return nNSS.maxSearchSquareDist;
                }
            }
        }

        //update
        nNSS.alreadyVisitedNeighbourhoodSize=visitedCellDistance;
    }

    //for each dimension, we look for the min distance between the query point and the celle border.
    //This distance (minDistToBorder) correponds to the maximal radius of a sphere centered on the
    //query point and totaly included inside the cell
    PointCoordinateType minDistToBorder = ComputeMinDistanceToCellBorder(&nNSS.queryPoint,cs,nNSS.cellCenter);

    //cells for which we have already computed the distances from their points to the query point
    unsigned alreadyProcessedCells = 0;

    //Min (squared) distance of neighbours
    PointCoordinateType minSquareDist = -1;

    while (true)
    {
        //if we do have found points but that were too far to be elligible
        if (minSquareDist > 0)
        {
            //what would be the correct neighbourhood size to be sure of it?
            int newElligibleCellDistance = static_cast<int>(ceil((sqrt(minSquareDist)-minDistToBorder)/cs));
            elligibleCellDistance = std::max(newElligibleCellDistance,elligibleCellDistance);
        }

        //we get the (new) cells around the current neighbourhood
        while (nNSS.alreadyVisitedNeighbourhoodSize < elligibleCellDistance) //DGM: warning, alreadyVisitedNeighbourhoodSize == 1 means that we have only visited the first cell (distance=0)
        {
            getNeighborCellsAround(nNSS.cellPos,nNSS.minimalCellsSetToVisit,nNSS.alreadyVisitedNeighbourhoodSize,nNSS.level);
            ++nNSS.alreadyVisitedNeighbourhoodSize;
        }

        //we compute distances for the new points
        DgmOctree::cellIndexesContainer::const_iterator q;
        for (q = nNSS.minimalCellsSetToVisit.begin()+alreadyProcessedCells; q != nNSS.minimalCellsSetToVisit.end(); ++q)
        {
            //current cell index (== index of its first point)
            unsigned m = *q;

            //we scan the whole cell to see if it contains a closer point
            cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+m;
            OctreeCellCodeType code = (p->theCode >> bitDec);
            while (m<m_numberOfProjectedPoints && (p->theCode >> bitDec) == code)
            {
                //square distance to query point
                PointCoordinateType dist2 = (*m_theAssociatedCloud->getPointPersistentPtr(p->theIndex) - nNSS.queryPoint).norm2();
                //we keep track of the closest one
                if (dist2 < minSquareDist || minSquareDist < 0)
                {
                    nNSS.theNearestPointIndex = p->theIndex;
                    minSquareDist = dist2;
                }
                ++m;
                ++p;
            }
        }
        alreadyProcessedCells = static_cast<unsigned>(nNSS.minimalCellsSetToVisit.size());

        //equivalent spherical neighbourhood radius (as we are actually looking to 'square' neighbourhoods,
        //we must check that the nearest points inside such neighbourhoods are indeed near enough to fall
        //inside the biggest included sphere). Otherwise we must look further.
        PointCoordinateType elligibleDist = static_cast<PointCoordinateType>(elligibleCellDistance-1) * cs + minDistToBorder;
        PointCoordinateType squareElligibleDist = elligibleDist * elligibleDist;

        //if we have found an elligible point
        if (minSquareDist >= 0 && minSquareDist <= squareElligibleDist)
        {
            if (nNSS.maxSearchSquareDist < 0 || minSquareDist <= nNSS.maxSearchSquareDist)
                return static_cast<ScalarType>(minSquareDist);
			else
                return -1;
        }
		else
		{
			//no elligible point? Maybe we are already too far?
			if (nNSS.maxSearchSquareDist >= 0 && squareElligibleDist > nNSS.maxSearchSquareDist)
				return -1;
		}

        //default strategy: increase neighbourhood size of +1 (for next step)
        ++elligibleCellDistance;
    }

    //we should never get here!
    assert(false);

    return -1;
}

//search for at least "minNumberOfNeighbors" points around a query point
unsigned DgmOctree::findNearestNeighborsStartingFromCell(NearestNeighboursSearchStruct &nNSS,
															bool getOnlyPointsWithValidScalar/*=false*/) const
{
    //binary shift for cell code truncation
    uchar bitDec = GET_BIT_SHIFT(nNSS.level);

    //cell size at the current level of subdivison
    const PointCoordinateType& cs = getCellSize(nNSS.level);

    //already visited cells (relative distance to the cell that includes the query point)
    int visitedCellDistance=nNSS.alreadyVisitedNeighbourhoodSize;
    //minimum (a priori) relative distance to get elligible points (see 'elligibleDist' below)
    int elligibleCellDistance=visitedCellDistance;

    //shall we look inside the first cell (the one including the query point)?
    if (visitedCellDistance == 0)
    {
        //visitedCellDistance == 0 means that no cell has ever been processed! No point should be inside 'pointsInNeighbourhood'
        assert(nNSS.pointsInNeighbourhood.empty());

        //check for existence of 'including' cell
        unsigned index = (nNSS.truncatedCellCode == INVALID_CELL_CODE ? m_numberOfProjectedPoints : getCellIndex(nNSS.truncatedCellCode,bitDec,true));

        visitedCellDistance = 1;

        //it this cell does exist...
        if (index < m_numberOfProjectedPoints)
        {
            //we grab the points inside
            cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+index;
			while (p!=m_thePointsAndTheirCellCodes.end() && (p->theCode >> bitDec) == nNSS.truncatedCellCode)
			{
				if (!getOnlyPointsWithValidScalar || ScalarField::ValidValue(m_theAssociatedCloud->getPointScalarValue(p->theIndex)))
				{
					PointDescriptor newPoint(m_theAssociatedCloud->getPointPersistentPtr(p->theIndex),p->theIndex);
					nNSS.pointsInNeighbourhood.push_back(newPoint);
					++p;
				}
			}

            elligibleCellDistance = 1;
        }
        //otherwise, we may be very far from the nearest octree cell
        //(let's try to get there asap)
        else
        {
            //fill indexes for current level
            const int* _fillIndexes = m_fillIndexes+6*nNSS.level;
            int distToBorder, diagonalDistance = 0;
            for (int dim=0; dim<3; ++dim)
            {
                //distance to min border of octree along each axis
                distToBorder = *_fillIndexes-nNSS.cellPos[dim];
                //if its negative, lets look the other side
                if (distToBorder<0)
                    //distance to max border of octree along each axis
                    distToBorder = nNSS.cellPos[dim]-_fillIndexes[3];

                if (distToBorder>0)
                {
                    visitedCellDistance = std::max(distToBorder,visitedCellDistance);
                    diagonalDistance += distToBorder*distToBorder;
                }

                //next dimension
                ++_fillIndexes;
            }

            //the nearest octree cell
            diagonalDistance = static_cast<int>(ceil(sqrt(static_cast<float>(diagonalDistance))));
            elligibleCellDistance = std::max(diagonalDistance,1);

            if (nNSS.maxSearchSquareDist >= 0)
            {
                //Distance of the nearest point
                ScalarType minDist = (ScalarType)(elligibleCellDistance-1) * (ScalarType)cs;
                //if we are already outside of the search limit, we can quit
                if (minDist*minDist > nNSS.maxSearchSquareDist)
                {
                    return 0;
                }
            }
        }
    }

    //for each dimension, we look for the min distance between the query point and the celle border.
    //This distance (minDistToBorder) correponds to the maximal radius of a sphere centered on the
    //query point and totaly included inside the cell
    PointCoordinateType minDistToBorder = ComputeMinDistanceToCellBorder(&nNSS.queryPoint,cs,nNSS.cellCenter);

    //elligible points found
    unsigned elligiblePoints = 0;

    //points for which we have already computed the distance to the query point
    unsigned alreadyProcessedPoints = 0;

    //Min (squared) distance of non elligible points
    PointCoordinateType minSquareDist = -1;

    //while we don't have enough 'nearest neighbours'
    while (elligiblePoints<nNSS.minNumberOfNeighbors)
    {
        //if we do have found points but that were too far to be elligible
        if (minSquareDist > 0)
        {
            //what would be the correct neighbourhood size to be sure of it?
            int newElligibleCellDistance = static_cast<int>(ceil((sqrt(minSquareDist)-minDistToBorder)/cs));
            elligibleCellDistance = std::max(newElligibleCellDistance,elligibleCellDistance);
        }

        //we get the (new) points lying in the added area
        while (visitedCellDistance < elligibleCellDistance) //DGM: warning, visitedCellDistance == 1 means that we have only visited the first cell (distance=0)
        {
			getPointsInNeighbourCellsAround(nNSS,visitedCellDistance,getOnlyPointsWithValidScalar);
            ++visitedCellDistance;
        }

        //we compute distances for the new points
        NeighboursSet::iterator q;
        for (q = nNSS.pointsInNeighbourhood.begin()+alreadyProcessedPoints; q != nNSS.pointsInNeighbourhood.end(); ++q)
            q->squareDist = static_cast<ScalarType>((*q->point - nNSS.queryPoint).norm2());
        alreadyProcessedPoints = static_cast<unsigned>(nNSS.pointsInNeighbourhood.size());

        //equivalent spherical neighbourhood radius (as we are actually looking to 'square' neighbourhoods,
        //we must check that the nearest points inside such neighbourhoods are indeed near enough to fall
        //inside the biggest included sphere). Otherwise we must look further.
        PointCoordinateType elligibleDist = static_cast<PointCoordinateType>(elligibleCellDistance-1) * cs + minDistToBorder;
        PointCoordinateType squareElligibleDist = elligibleDist * elligibleDist;

        //let's test all the previous 'not yet elligible' points and the new ones
        unsigned j=elligiblePoints;
        for (q = nNSS.pointsInNeighbourhood.begin()+elligiblePoints; q != nNSS.pointsInNeighbourhood.end(); ++q,++j)
        {
            //if the point is elligible
            if (q->squareDist <= squareElligibleDist)
            {
                if (elligiblePoints<j)
                    std::swap(nNSS.pointsInNeighbourhood[elligiblePoints],nNSS.pointsInNeighbourhood[j]);
                ++elligiblePoints;
            }
            //otherwise we track the nearest one
            else if (q->squareDist<minSquareDist || j == elligiblePoints)
            {
                minSquareDist = static_cast<PointCoordinateType>(q->squareDist);
            }
        }

        //Maybe we are already too far?
        if (nNSS.maxSearchSquareDist >= 0 && squareElligibleDist > nNSS.maxSearchSquareDist)
            break;

        //default strategy: increase neighbourhood size of +1 (for next step)
        ++elligibleCellDistance;
    }

    //update the neighbourhood size (for next call, if the query point lies in the same cell)
    nNSS.alreadyVisitedNeighbourhoodSize = visitedCellDistance;

    //we sort the elligible points
	std::sort(nNSS.pointsInNeighbourhood.begin(),nNSS.pointsInNeighbourhood.begin()+elligiblePoints,PointDescriptor::distComp);

    //we return the number of elligible points found
    return elligiblePoints;
}

#ifdef OCTREE_TREE_TEST
struct cellToInspect
{
	octreeTreeCell* cell;
	uchar level;
	CCVector3 corner;
	bool toGrab;
};
#endif

int DgmOctree::getPointsInSphericalNeighbourhood(const CCVector3& sphereCenter, PointCoordinateType radius, NeighboursSet& neighbours) const
{
	//current cell bounding box (=whole octree!)
	CCVector3 bbMin = getOctreeMins();
	CCVector3 bbMax = getOctreeMaxs();

	//sphere tight bounding box
	CCVector3 sphereMin = sphereCenter-CCVector3(radius,radius,radius);
	CCVector3 sphereMax = sphereCenter+CCVector3(radius,radius,radius);

	//number of neighbours (vector is not cleared!)
	unsigned n = 0;

#ifdef OCTREE_TREE_TEST
	//use tree?
	if (s_root)
	{
		float squareRadius = radius*radius;

		//currently inspected cell (children)
		std::vector<cellToInspect> cellsToInspect;
		cellsToInspect.reserve(4*MAX_OCTREE_LEVEL);

		cellToInspect desc;
		desc.cell = s_root;
		desc.level = 0;
		desc.corner = bbMin;
		desc.toGrab = false;
		cellsToInspect.push_back(desc);

		//begin 'recursion'
		do
		{
			cellToInspect current = cellsToInspect.back();
			cellsToInspect.pop_back();

			if (current.cell->childrenCount == 0) //leaf cell
			{
				octreeTreeCellLeaf* leafCell = static_cast<octreeTreeCellLeaf*>(current.cell);
				assert(leafCell);

				//finally, we can grab points inside the leaf cell
				std::vector<unsigned>::const_iterator it_index = leafCell->pointIndexes.begin();

				if (n+leafCell->pointIndexes.size()>neighbours.size())
				{
					try
					{
						neighbours.resize(n+leafCell->pointIndexes.size());
					}
					catch (.../*const std::bad_alloc&*/) //out of memory
					{
						return -1; //not enough memory
					}
				}

				if (current.toGrab) //no need to test, all points are inside neighbourhood
				{
					for (; it_index!=leafCell->pointIndexes.end(); ++it_index)
					{
						//neighbours.push_back(PointDescriptor(0,*it_index,0.0)); //TODO: no pointer, no distance?!
						//neighbours.push_back(PointDescriptor(m_theAssociatedCloud->getPointPersistentPtr(*it_index),*it_index,0.0)); //TODO: no distance?!
						neighbours[n].pointIndex = *it_index;
						neighbours[n++].point = m_theAssociatedCloud->getPointPersistentPtr(*it_index);
					}
				}
				else
				{
					//we test each point
					for (; it_index!=leafCell->pointIndexes.end(); ++it_index)
					{
						const CCVector3* P = m_theAssociatedCloud->getPointPersistentPtr(*it_index);
						PointCoordinateType d2 = (*P - sphereCenter).norm2();
						if (d2<=squareRadius)
							//neighbours.push_back(PointDescriptor(P,*it_index,d2));
							neighbours[n++] = PointDescriptor(P,*it_index,d2);
					}
				}
			}
			else //no points in current cell, only children!
			{
				if (current.toGrab) //don't ask any question and grab all points/children!
				{
					for (uchar i=0;i<current.cell->childrenCount;++i)
					{
						desc.cell = current.cell->children[i];
						desc.toGrab = true;
						//desc.corner //not used anymore
						//desc.level //not used anymore
						cellsToInspect.push_back(desc);
					}
				}
				else //let's have a closer look
				{
					for (uchar i=0;i<current.cell->childrenCount;++i)
					{
						octreeTreeCell* child = current.cell->children[i];

						octreeTreeCellLeaf* leafCell = (child->childrenCount != 0 ? 0 : static_cast<octreeTreeCellLeaf*>(child));
						assert(child->childrenCount != 0 || leafCell);

						//particular case: leaf cell with very few points
						if (leafCell && leafCell->pointIndexes.size()<6)
						{
							if (n+leafCell->pointIndexes.size()>neighbours.size())
							{
								try
								{
									neighbours.resize(n+leafCell->pointIndexes.size());
								}
								catch (.../*const std::bad_alloc&*/) //out of memory
								{
									return -1; //not enough memory
								}
							}

							for (std::vector<unsigned>::const_iterator it_index = leafCell->pointIndexes.begin(); it_index!=leafCell->pointIndexes.end(); ++it_index)
							{
								//we test the point directly!
								const unsigned& index = *it_index;
								const CCVector3* P = m_theAssociatedCloud->getPointPersistentPtr(index);
								PointCoordinateType d2 = (*P - sphereCenter).norm2();
								if (d2<=squareRadius)
									//neighbours.push_back(PointDescriptor(P,index,d2));
									neighbours[n++] = PointDescriptor(P,*it_index,d2);
							}
						}
						else //let's try to prune this branch/leaf by looking at its bbox
						{
							desc.level = current.level+1;
							assert(desc.level<=MAX_OCTREE_LEVEL);
							desc.corner = current.corner;
							//cell size at next level = half of current level cell size
							const PointCoordinateType& cs = getCellSize(desc.level);

							//we compute new cell corner from its relative pos
							if (child->relativePos & 1)
								desc.corner.x += cs;
							if (sphereMax.x < desc.corner.x || sphereMin.x > desc.corner.x+cs) //sphere totally outside the cell
								continue;

							if (child->relativePos & 2)
								desc.corner.y += cs;
							if (sphereMax.y < desc.corner.y || sphereMin.y > desc.corner.y+cs) //sphere totally outside the cell
								continue;

							if (child->relativePos & 4)
								desc.corner.z += cs;
							if (sphereMax.z < desc.corner.z || sphereMin.z > desc.corner.z+cs) //sphere totally outside the cell
								continue;

							const PointCoordinateType& half_cs = getCellSize(desc.level+1); //half cell size at next level

							//distance between the new cell center and the sphere center
							PointCoordinateType d2 = (desc.corner + CCVector3(half_cs,half_cs,half_cs) - sphereCenter).norm2();

							//is this cell totally out of the sphere?
							if (d2 > squareRadius + cs*(0.75*cs+SQRT_3*radius)) //cell is totally outside
							{
								continue;
							}

							//add cell to inspection list
							desc.cell = child;
							//totally inside?
							PointCoordinateType minD = radius-half_cs*static_cast<PointCoordinateType>(SQRT_3);
							if (minD < 0)
								desc.toGrab = false;
							else
								desc.toGrab = (d2 <= minD*minD);
							cellsToInspect.push_back(desc);
						}
					}
				}
			}
		}
		while (!cellsToInspect.empty());
	}
	else
#endif
	{
		uchar level = 0;
		OctreeCellCodeType englobCode = 0;

		//let's find the minimum enclosing cell
		for (uchar nextLevel=1 ; nextLevel<=MAX_OCTREE_LEVEL; ++nextLevel)
		{
			//cell size at next level
			const PointCoordinateType& cs=getCellSize(nextLevel);

			//local code for current level
			OctreeCellCodeType localCode = 0;

			//X
			PointCoordinateType pivot = bbMin.x+cs;
			if (sphereMax.x < pivot) //sphere on the left subcell
			{
				bbMax.x = pivot;
			}
			else if (sphereMin.x >= pivot) //sphere on the right subcell
			{
				bbMin.x = pivot;
				localCode |= 1;
			}
			else
			{
				break; //sphere is across both sides: we are done!
			}

			//Y
			pivot = bbMin.y+cs;
			if (sphereMax.y < pivot) //sphere on the left subcell
			{
				bbMax.y = pivot;
			}
			else if (sphereMin.y >= pivot) //sphere on the right subcell
			{
				bbMin.y = pivot;
				localCode |= 2;
			}
			else
			{
				break; //sphere is across both sides: we are done!
			}

			//Z
			pivot = bbMin.z+cs;
			if (sphereMax.z < pivot) //sphere on the left subcell
			{
				bbMax.z = pivot;
			}
			else if (sphereMin.z >= pivot) //sphere on the right subcell
			{
				bbMin.z = pivot;
				localCode |= 4;
			}
			else
			{
				break; //sphere is across both sides: we are done!
			}

			//next level is ok, we can add the 'local' portion to it
			englobCode <<= 3;
			englobCode |= localCode;
			level = nextLevel;
		}

		unsigned startIndex = 0;
		//neighbours.clear();
		uchar bitDec = GET_BIT_SHIFT(level);

		//appart from the case where we must start from the very begining of the octree (level 0)
		//we are now gonna look to the first point in the englobing cell
		if (englobCode>0)
		{
			startIndex = getCellIndex(englobCode,bitDec,true);
			if (startIndex == m_numberOfProjectedPoints) //cell not in octree?!
				return 0;
		}

		//begin 'recursion'
		uchar currentLevel = level;
		uchar currentBitDec = bitDec;
		bool skipCell=false;
		bool toGrab=false;
		OctreeCellCodeType currentCode = INVALID_CELL_CODE;
		OctreeCellCodeType currentTruncatedCode = INVALID_CELL_CODE;
		//current cell corners
		CCVector3 cellCorners[MAX_OCTREE_LEVEL+1];
		cellCorners[level]=bbMin;

		PointCoordinateType currentSquareDistanceToCellCenter = -1;
		PointCoordinateType squareRadius = radius*radius;

		for (cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+startIndex; p != m_thePointsAndTheirCellCodes.end() && (p->theCode >> bitDec) == englobCode; ++p) //we are looking to all points inside the main englobing cell!
		{
			//different cell?
			if ((p->theCode >> currentBitDec) != currentTruncatedCode)
			{
				skipCell = false;
				toGrab = false;

				//let's try to find a common cell (there should be one! the main one in the worst case)
				while (currentLevel > level)
				{
					currentLevel--;
					currentBitDec+=3;
					if ((p->theCode >> currentBitDec) == (currentCode >> currentBitDec))
						break;
				}

				//new "current" cell code
				currentCode = p->theCode;

				//now let's try to go deeper
				while (currentLevel < MAX_OCTREE_LEVEL)
				{
					CCVector3 cellCorner = cellCorners[currentLevel];
					++currentLevel;
					currentBitDec -= 3;
					currentTruncatedCode = (currentCode >> currentBitDec);

					bool uniquePointCell = ((p+1) == m_thePointsAndTheirCellCodes.end() || currentTruncatedCode != ((p+1)->theCode >> currentBitDec));
					if (uniquePointCell)
					{
						//we test the point directly!
						const CCVector3* P = m_theAssociatedCloud->getPointPersistentPtr(p->theIndex);
						PointCoordinateType d2 = (*P - sphereCenter).norm2();

						if (d2 <= squareRadius)
						{
							if (n+1 > neighbours.size())
							{
								try
								{
									neighbours.resize(n+1);
								}
								catch (.../*const std::bad_alloc&*/) //out of memory
								{
									return -1; //not enough memory
								}
							}
							neighbours[n++] = PointDescriptor(P,p->theIndex,static_cast<ScalarType>(d2));
						}
						skipCell=true;
						break;
					}
					else
					{
						const PointCoordinateType& cs = getCellSize(currentLevel); //cell size at new level

						//we compute new cell center from the last 3 bits
						if (currentTruncatedCode & 1)
							cellCorner.x += cs;
						if (currentTruncatedCode & 2)
							cellCorner.y += cs;
						if (currentTruncatedCode & 4)
							cellCorner.z += cs;
						cellCorners[currentLevel]=cellCorner;

						const PointCoordinateType& half_cs = getCellSize(currentLevel+1); //half cell size
						PointCoordinateType d2 = (cellCorner + CCVector3(half_cs,half_cs,half_cs) - sphereCenter).norm2();

						//is this cell totally out of the sphere?
						if (d2 > squareRadius + cs*(0.75*cs+SQRT_3*radius)) //cell is totally outside
						{
							skipCell = true; //sure of exclusion
							break;
						}
						else if (currentLevel < MAX_OCTREE_LEVEL)
						{
							//or totally inside?
							PointCoordinateType minD = radius-half_cs*static_cast<PointCoordinateType>(SQRT_3);
							if (minD > 0 && d2 <= minD*minD)
							{
								toGrab = true;
								currentSquareDistanceToCellCenter=d2; //sure of inclusion
								break;
							}
						}
						currentSquareDistanceToCellCenter = 0; //not sure of anything
					}
				}
			}

			//shall we skip this point?
			if (!skipCell)
			{
				if (toGrab)
				{
					try
					{
						neighbours.push_back(PointDescriptor(0,p->theIndex,static_cast<ScalarType>(currentSquareDistanceToCellCenter)));
					}
					catch (.../*const std::bad_alloc&*/) //out of memory
					{
						return -1; //not enough memory
					}
					++n;
				}
				else
				{
					//otherwise we have to test the point
					const CCVector3* P = m_theAssociatedCloud->getPointPersistentPtr(p->theIndex);
					PointCoordinateType d2 = (*P - sphereCenter).norm2();

					if (d2<=squareRadius)
					{
						try
						{
							neighbours.push_back(PointDescriptor(P,p->theIndex,static_cast<ScalarType>(d2)));
						}
						catch (.../*const std::bad_alloc&*/) //out of memory
						{
							return -1; //not enough memory
						}
						++n;
					}
				}
			}
		}
	}

	return static_cast<int>(n);
}

#ifdef COMPUTE_NN_SEARCH_STATISTICS
static double s_skippedPoints = 0.0;
static double s_testedPoints = 0.0;
#endif

//search for all neighbors inside a sphere
int DgmOctree::findNeighborsInASphereStartingFromCell(NearestNeighboursSphericalSearchStruct &nNSS, PointCoordinateType radius, bool sortValues) const
{
#ifdef OCTREE_TREE_TEST
	assert(s_root);

	if (!nNSS.ready)
	{
		//current level cell size
		const PointCoordinateType& cs=getCellSize(nNSS.level);

		int n = getPointsInSphericalNeighbourhood(nNSS.cellCenter, radius+cs*static_cast<PointCoordinateType>(SQRT_3/2.0), nNSS.pointsInNeighbourhood);
		nNSS.pointsInNeighbourhood.resize(n);

		nNSS.ready=true;
	}
#else
#ifdef TEST_CELLS_FOR_SPHERICAL_NN
	if (!nNSS.ready)
	{
		//current level cell size
		const PointCoordinateType& cs=getCellSize(nNSS.level);

		//we deduce the minimum cell neighbourhood size (integer) that includes the search sphere
		//for ANY point in the cell
		int minNeighbourhoodSize = static_cast<int>(ceil(radius/cs+static_cast<PointCoordinateType>(SQRT_3/2.0)));

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
    PointCoordinateType minDistToBorder = ComputeMinDistanceToCellBorder(&nNSS.queryPoint,cs,nNSS.cellCenter);

    //we deduce the minimum cell neighbourhood size (integer) that includes the search sphere
    int minNeighbourhoodSize = 1+(radius>minDistToBorder ? int(ceil((radius-minDistToBorder)/cs)) : 0);

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
#endif

	//squared distances comparison is faster!
    PointCoordinateType squareRadius = radius * radius;
    unsigned numberOfElligiblePoints = 0;

#ifdef TEST_CELLS_FOR_SPHERICAL_NN
		//cell limit relatively to sphere tight bounding box
	//const PointCoordinateType& half_cs=getCellSize(nNSS.level+1); //half cell size at current level
	//CCVector3 limitMin = nNSS.queryPoint-CCVector3(radius,radius,radius)-CCVector3(half_cs,half_cs,half_cs);
	//CCVector3 limitMax = nNSS.queryPoint+CCVector3(radius,radius,radius)+CCVector3(half_cs,half_cs,half_cs);

	//cell by cell scan
    for (NeighbourCellsSet::iterator c = nNSS.cellsInNeighbourhood.begin(); c!=nNSS.cellsInNeighbourhood.end(); ++c)
    {
		//we check the cell bounding box
		/*if (limitMax.x < c->center.x || limitMin.x >  c->center.x
			|| limitMax.y < c->center.y || limitMin.y >  c->center.y
			|| limitMax.z < c->center.z || limitMin.z >  c->center.z)
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
				//... we had them to the 'elligible points' part of the container
				std::copy(p,p+count,nNSS.pointsInNeighbourhood.begin()+numberOfElligiblePoints);
				numberOfElligiblePoints += count;
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
					if (p->squareDist <= squareRadius)
					{
						//... we had it to the 'elligible points' part of the container
						nNSS.pointsInNeighbourhood[numberOfElligiblePoints++] = *p;
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
    NeighboursSet::iterator p = nNSS.pointsInNeighbourhood.begin();
    size_t k = nNSS.pointsInNeighbourhood.size();
    for (size_t i=0; i<k; ++i,++p)
    {
        p->squareDist = static_cast<ScalarType>( (*p->point - nNSS.queryPoint).norm2() );
        //if the distance is inferior to the sphere radius...
        if (p->squareDist <= squareRadius)
        {
            //... we had it to the 'elligible points' part of the container
            if (i>numberOfElligiblePoints)
                std::swap(nNSS.pointsInNeighbourhood[i],nNSS.pointsInNeighbourhood[numberOfElligiblePoints]);

            ++numberOfElligiblePoints;
#ifdef COMPUTE_NN_SEARCH_STATISTICS
			s_testedPoints += 1.0;
#endif
        }
    }

#endif //!TEST_CELLS_FOR_SPHERICAL_NN

    //eventually (if requested) we sort the elligible points
    if (sortValues && numberOfElligiblePoints>0)
		std::sort(nNSS.pointsInNeighbourhood.begin(),nNSS.pointsInNeighbourhood.begin()+numberOfElligiblePoints,PointDescriptor::distComp);

    //return the number of elligible points
    return numberOfElligiblePoints;
}

double DgmOctree::computeMeanOctreeDensity(uchar level) const
{
    return static_cast<double>(m_numberOfProjectedPoints)/static_cast<double>(getCellNumber(level));
}

void DgmOctree::getCellCodesAndIndexes(uchar level, cellsContainer& vec, bool truncatedCodes/*=false*/) const
{
    //binary shift for cell code truncation
    uchar bitDec = GET_BIT_SHIFT(level);

    cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin();

    OctreeCellCodeType predCode = (p->theCode >> bitDec)+1; //pred value must be different than the first element's

    for (unsigned i=0; i<m_numberOfProjectedPoints; ++i,++p)
    {
        OctreeCellCodeType currentCode = (p->theCode >> bitDec);

        if (predCode != currentCode)
            vec.push_back(IndexAndCode(i,truncatedCodes ? currentCode : p->theCode));

        predCode = currentCode;
    }
}

void DgmOctree::getCellCodes(uchar level,cellCodesContainer& vec, bool truncatedCodes/*=false*/) const
{
    //binary shift for cell code truncation
    uchar bitDec = GET_BIT_SHIFT(level);

    cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin();

    OctreeCellCodeType predCode = (p->theCode >> bitDec)+1; //pred value must be different than the first element's

    for (unsigned i=0; i<m_numberOfProjectedPoints; ++i,++p)
    {
        OctreeCellCodeType currentCode = (p->theCode >> bitDec);

        if (predCode != currentCode)
            vec.push_back(truncatedCodes ? currentCode : p->theCode);

        predCode = currentCode;
    }
}

bool DgmOctree::getCellIndexes(uchar level, cellIndexesContainer& vec) const
{
	try
	{
		vec.resize(m_cellCount[level]);
	}
	catch(std::bad_alloc)
	{
		return false;
	}

    //binary shift for cell code truncation
    uchar bitDec = GET_BIT_SHIFT(level);

    cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin();

    OctreeCellCodeType predCode = (p->theCode >> bitDec)+1; //pred value must be different than the first element's

    for (unsigned i=0,j=0; i<m_numberOfProjectedPoints; ++i,++p)
    {
        OctreeCellCodeType currentCode = (p->theCode >> bitDec);

        if (predCode != currentCode)
            vec[j++] = i;

        predCode = currentCode;
    }

	return true;
}

void DgmOctree::getPointsInCellByCellIndex(ReferenceCloud* cloud, unsigned cellIndex, uchar level) const
{
    assert(cloud && cloud->getAssociatedCloud() == m_theAssociatedCloud);

    //binary shift for cell code truncation
    uchar bitDec = GET_BIT_SHIFT(level);

    //we look for the first index in 'm_thePointsAndTheirCellCodes' corresponding to this cell
    cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin()+cellIndex;
    OctreeCellCodeType searchCode = (p->theCode >> bitDec);

    cloud->clear(false);

    //while the (partial) cell code matches this cell
    while ((p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == searchCode))
    {
        cloud->addPointIndex(p->theIndex);
        ++p;
    }
}

ReferenceCloud* DgmOctree::getPointsInCellsWithSortedCellCodes(cellCodesContainer& cellCodes, uchar level, bool areCodesTruncated) const
{
    //binary shift for cell code truncation
    uchar bitDec1 = GET_BIT_SHIFT(level); //shift for this octree codes
    uchar bitDec2 = (areCodesTruncated ? 0 : bitDec1); //shift for the input codes

    cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin();
    OctreeCellCodeType toExtractCode,currentCode = (p->theCode >> bitDec1); //pred value must be different than the first element's

    m_dumpCloud->clear(false);

    cellCodesContainer::const_iterator q=cellCodes.begin();
    unsigned ind_p=0;
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
                m_dumpCloud->addPointIndex(p->theIndex);

            ++p;
            if (++ind_p < m_numberOfProjectedPoints)
                currentCode = p->theCode >> bitDec1;
        }
    }

    return m_dumpCloud;
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

    while (pA!=codesA.end())
        diffA.push_back(*pA++);
    while (pB!=codesB.end())
        diffB.push_back(*pB++);
}

void DgmOctree::diff(uchar octreeLevel, const cellsContainer &codesA, const cellsContainer &codesB, int &diffA, int &diffB, int &cellsA, int &cellsB) const
{
    if (codesA.empty() && codesB.empty()) return;

    cellsContainer::const_iterator pA = codesA.begin();
    cellsContainer::const_iterator pB = codesB.begin();

    //binary shift for cell code truncation
    uchar bitDec = GET_BIT_SHIFT(octreeLevel);

    OctreeCellCodeType predCodeA = pA->theCode >> bitDec;
    OctreeCellCodeType predCodeB = pB->theCode >> bitDec;

    OctreeCellCodeType currentCodeA = 0;
    OctreeCellCodeType currentCodeB = 0;

    diffA=diffB=0;
    cellsA=cellsB=0;

    //cell codes should already be sorted!
    while ((pA != codesA.end())&&(pB != codesB.end()))
    {
        if (predCodeA < predCodeB)
        {
            ++diffA;
            ++cellsA;
            while ((pA!=codesA.end())&&((currentCodeA = (pA->theCode >> bitDec)) == predCodeA)) ++pA;
            predCodeA=currentCodeA;
        }
        else if (predCodeA > predCodeB)
        {
            ++diffB;
            ++cellsB;
            while ((pB!=codesB.end())&&((currentCodeB = (pB->theCode >> bitDec)) == predCodeB)) ++pB;
            predCodeB=currentCodeB;
        }
        else
        {
            while ((pA!=codesA.end())&&((currentCodeA = (pA->theCode >> bitDec)) == predCodeA)) ++pA;
            predCodeA=currentCodeA;
            ++cellsA;
            while ((pB!=codesB.end())&&((currentCodeB = (pB->theCode >> bitDec)) == predCodeB)) ++pB;
            predCodeB=currentCodeB;
            ++cellsB;
        }
    }

    while (pA!=codesA.end())
    {
        ++diffA;
        ++cellsA;
        while ((pA!=codesA.end())&&((currentCodeA = (pA->theCode >> bitDec)) == predCodeA)) ++pA;
        predCodeA=currentCodeA;
    }
    while (pB!=codesB.end())
    {
        ++diffB;
        ++cellsB;
        while ((pB!=codesB.end())&&((currentCodeB = (pB->theCode >> bitDec)) == predCodeB)) ++pB;
        predCodeB=currentCodeB;
    }
}

uchar DgmOctree::findBestLevelForAGivenNeighbourhoodSizeExtraction(PointCoordinateType radius) const
{
	PointCoordinateType aim = radius / static_cast<PointCoordinateType>(2.5);
    uchar level = 1;
	PointCoordinateType minValue = getCellSize(1)-aim;
	minValue *= minValue;
    for (uchar i=2; i<=MAX_OCTREE_LEVEL; ++i)
    {
        //The level with cell size as near as possible to the aim
        PointCoordinateType cellSizeDelta = getCellSize(i)-aim;
        cellSizeDelta *= cellSizeDelta;

        if (cellSizeDelta < minValue)
        {
            level = i;
            minValue = cellSizeDelta;
        }
    }

    return level;
}

uchar DgmOctree::findBestLevelForComparisonWithOctree(const DgmOctree* theOtherOctree) const
{
    double estimatedTime[MAX_OCTREE_LEVEL];
    estimatedTime[0] = 0.0;

    unsigned ptsA = getNumberOfProjectedPoints();
    unsigned ptsB = theOtherOctree->getNumberOfProjectedPoints();
    int cellsA,cellsB,diffA,diffB;

    //ATTENTION i>=1
    uchar bestLevel = 1;
    for (uchar i=1; i<MAX_OCTREE_LEVEL; ++i)
    {
        diff(i,m_thePointsAndTheirCellCodes,theOtherOctree->m_thePointsAndTheirCellCodes,diffA,diffB,cellsA,cellsB);

        //we use a linear model for prediction
        estimatedTime[i] = (double(ptsA)*double(ptsB)/double(cellsB)) * 0.001 + double(diffA);

        if (estimatedTime[i] < estimatedTime[bestLevel])
            bestLevel = i;
    }

    return bestLevel;
}

uchar DgmOctree::findBestLevelForAGivenPopulationPerCell(unsigned indicativeNumberOfPointsPerCell) const
{
    uchar level=MAX_OCTREE_LEVEL;
    double density=0,predDensity=0;

    for (level=MAX_OCTREE_LEVEL; level>0; --level)
    {
        predDensity = density;
        density = static_cast<double>(m_numberOfProjectedPoints)/static_cast<double>(getCellNumber(level));
        if (density>=indicativeNumberOfPointsPerCell)
			break;
    }

    if (level<MAX_OCTREE_LEVEL)
	{
		if (level == 0)
		{
			predDensity = density;
			density = static_cast<double>(m_numberOfProjectedPoints);
		}

		//we take the closest match
		if (density-indicativeNumberOfPointsPerCell>indicativeNumberOfPointsPerCell-predDensity)
			++level;
	}

    return level;
}

uchar DgmOctree::findBestLevelForAGivenCellNumber(unsigned indicativeNumberOfCells) const
{
    //we look for the level giviing the number of points per cell as close to the query
    uchar bestLevel=1;
    //number of cells for this level
    int n = getCellNumber(bestLevel);
    //error relatively to the query
    int oldd=abs(n-int(indicativeNumberOfCells));

    n = getCellNumber(bestLevel+1);
    int d=abs(n-int(indicativeNumberOfCells));

    while ((d<oldd)&&(bestLevel<MAX_OCTREE_LEVEL))
    {
        ++bestLevel;
        oldd = d;
        n=getCellNumber(bestLevel+1);
        d=abs(n-int(indicativeNumberOfCells));
    }

    return bestLevel;
}

int DgmOctree::extractCCs(uchar level, bool sixConnexity, GenericProgressCallback* progressCb) const
{
    std::vector<OctreeCellCodeType> cellCodes;
    getCellCodes(level,cellCodes);
    return extractCCs(cellCodes, level, sixConnexity, progressCb);
}

int DgmOctree::extractCCs(const cellCodesContainer& cellCodes, uchar level, bool sixConnexity, GenericProgressCallback* progressCb) const
{
    size_t numberOfCells = cellCodes.size();
	if (numberOfCells == 0) //no cells!
        return -1;

	//filled octree cells
	std::vector<IndexAndCode> ccCells;
	try
	{
		ccCells.resize(numberOfCells);
	}
	catch (std::bad_alloc)
	{
		//not enough memory
		return -2;
	}

    //we compute the position of each cell (grid coordinates)
    int indexMin[3],indexMax[3];
	{
		//binary shift for cell code truncation
		uchar bitDec = GET_BIT_SHIFT(level);

		for (size_t i=0; i<numberOfCells; i++)
		{
			ccCells[i].theCode = (cellCodes[i] >> bitDec);

			int pos[3];
			getCellPos(ccCells[i].theCode,level,pos,true);

			//we look for the actual min and max dimensions of the input cells set
			//(which may not be the whole set of octree cells!)
			if (i != 0)
			{
				for (unsigned char k=0; k<3; k++)
				{
					if (pos[k] < indexMin[k])
						indexMin[k] = pos[k];
					else if (pos[k] > indexMax[k])
						indexMax[k] = pos[k];
				}
			}
			else
			{
				indexMin[0] = indexMax[0] = pos[0];
				indexMin[1] = indexMax[1] = pos[1];
				indexMin[2] = indexMax[2] = pos[2];
			}

			//Warning: the cells will have to be sorted inside a slice afterwards!
			ccCells[i].theIndex = (	static_cast<unsigned>(pos[0])				)
								+ (	static_cast<unsigned>(pos[1]) << level		)
								+ (	static_cast<unsigned>(pos[2]) << (2*level)	);
		}
	}

    //we deduce the size of the grid that totally include input cells
    int gridSize[3];
	{
		for (unsigned char k=0; k<3; k++)
		{
			gridSize[k] = indexMax[k] - indexMin[k] + 1;
		}
	}

    //we sort the cells
    std::sort(ccCells.begin(),ccCells.end(),IndexAndCode::indexComp); //ascending index code order

    const int& di = gridSize[0];
    const int& dj = gridSize[1];
    const int& step = gridSize[2];

    //instrumentation pour la recherche des 4 ou 8 voisins en 2D (donc en 3D --> 6 ou 26 voisins)
    uchar neighborsInCurrentSlice=0, neighborsInPrecedingSlice=0;
    int currentSliceNeighborsShifts[4], precedingSliceNeighborsShifts[9]; //maximum size to simplify code...

	if (sixConnexity) //6-connexity
    {
        neighborsInCurrentSlice = 2;
        currentSliceNeighborsShifts[0] = -(di+2);
        currentSliceNeighborsShifts[1] = -1;

        neighborsInPrecedingSlice = 1;
        precedingSliceNeighborsShifts[0] = 0;
    }
    else //26-connexity
    {
        neighborsInCurrentSlice = 4;
        currentSliceNeighborsShifts[0] = -1-(di+2);
        currentSliceNeighborsShifts[1] = -(di+2);
        currentSliceNeighborsShifts[2] = 1-(di+2);
        currentSliceNeighborsShifts[3] = -1;

        neighborsInPrecedingSlice = 9;
        precedingSliceNeighborsShifts[0] = -1-(di+2);
        precedingSliceNeighborsShifts[1] = -(di+2);
        precedingSliceNeighborsShifts[2] = 1-(di+2);
        precedingSliceNeighborsShifts[3] = -1;
        precedingSliceNeighborsShifts[4] = 0;
        precedingSliceNeighborsShifts[5] = 1;
        precedingSliceNeighborsShifts[6] = -1+(di+2);
        precedingSliceNeighborsShifts[7] = (di+2);
        precedingSliceNeighborsShifts[8] = 1+(di+2);
    }

	//shared structures (to avoid repeated allocations)
    std::vector<int> neighboursVal, neighboursMin;
	try
	{
		neighboursVal.reserve(neighborsInCurrentSlice+neighborsInPrecedingSlice);
		neighboursMin.reserve(neighborsInCurrentSlice+neighborsInPrecedingSlice);
	}
	catch(std::bad_alloc)
	{
		//not enough memory
		return -2;
	}

    //temporary virtual 'slices'
    int sliceSize = (di+2)*(dj+2); //add a margin to avoid "boundary effects"
    int *slice = new int[sliceSize];
    if (!slice) //Not enough memory
        return -2;

    int *oldSlice = new int[sliceSize];
    if (!oldSlice) //Not enough memory
    {
        delete[] slice;
        return -2;
    }

    //equivalence table between 'on the fly' labels
    int* equivalentLabels = new int[numberOfCells+2];
    if (!equivalentLabels) //Not enough memory
    {
        delete[] slice;
        delete[] oldSlice;
        return -2;
    }
    memset(equivalentLabels,0,sizeof(int)*(numberOfCells+2));

    //equivalence between a cell index and its label
    int* cellIndexToLabel = new int[numberOfCells];
    if (!cellIndexToLabel) //Not enough memory
    {
        delete[] slice;
        delete[] oldSlice;
        delete[] equivalentLabels;
        return -2;
    }
    memset(cellIndexToLabel,0,sizeof(int)*numberOfCells);

    //progress notification
	NormalizedProgress* nprogress = 0;
    if (progressCb)
    {
        progressCb->reset();
		nprogress = new NormalizedProgress(progressCb,step);
        progressCb->setMethodTitle("Components Labeling");
        char buffer[256];
		sprintf(buffer,"Box: [%i*%i*%i]",gridSize[0],gridSize[1],gridSize[2]);
        progressCb->setInfo(buffer);
        progressCb->start();
    }

    //preceding slice is empty by default
    memset(oldSlice,0,sizeof(int)*sliceSize);

    //current label
    size_t currentLabel = 1;

    //on parcours chaque tranche
	{
		unsigned counter = 0;
		const unsigned gridCoordMask = (1 << level)-1;
		std::vector<IndexAndCode>::const_iterator _ccCells = ccCells.begin();

		for (int k = indexMin[2]; k < indexMin[2]+step; k++)
		{
			//on initialise la tranche "courante"
			memset(slice,0,sizeof(int)*sliceSize);

			//pour chaque cellule de la tranche
			while (counter<numberOfCells && static_cast<int>(_ccCells->theIndex >> (level<<1)) == k)
			{
				int iind = int(_ccCells->theIndex & gridCoordMask);
				int jind = int((_ccCells->theIndex >> level) & gridCoordMask);
				int cellIndex = (iind-indexMin[0]+1) + (jind-indexMin[1]+1)*(di+2);
				++_ccCells;

				//on regarde si la cellule a des voisins
				//dans la tranche
				int* _slice = slice + cellIndex;
				{
					for (uchar n=0; n<neighborsInCurrentSlice; n++)
					{
						assert(cellIndex+currentSliceNeighborsShifts[n]<sliceSize);
						const int& neighborLabel = _slice[currentSliceNeighborsShifts[n]];
						if (neighborLabel>1)
							neighboursVal.push_back(neighborLabel);
					}
				}

				//et dans la tranche precedente
				int* _oldSlice = oldSlice + cellIndex;
				{
					for (uchar n=0; n<neighborsInPrecedingSlice; n++)
					{
						assert(cellIndex+precedingSliceNeighborsShifts[n]<sliceSize);
						const int& neighborLabel = _oldSlice[precedingSliceNeighborsShifts[n]];
						if (neighborLabel>1)
							neighboursVal.push_back(neighborLabel);
					}
				}

				//number of neighbors for current cell
				uchar p = (uchar)neighboursVal.size();

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
					std::sort(neighboursVal.begin(),neighboursVal.end());
					int smallestLabel = neighboursVal[0];

					//s'ils ne sont pas tous pareils
					if (smallestLabel != neighboursVal.back())
					{
						int lastLabel = 0;
						neighboursMin.clear();
						//we get the smallest equivalent label for each neighbor's branch
						{
							for (uchar n=0; n<p; n++)
							{
								// ... on part de son indice de CC
								int label = neighboursVal[n];
								//s'il n'est pas egal a celui du voisin d'avant
								if (label != lastLabel)
								{
									//on met a jour la notion "d'avant"
									assert(label<static_cast<int>(numberOfCells)+2);
									lastLabel = label;

									//on cherche a quoi il est vraiment equivalent
									while (equivalentLabels[label] > 1)
									{
										label = equivalentLabels[label];
										assert(label<static_cast<int>(numberOfCells)+2);
									}

									neighboursMin.push_back(label);
								}
							}
						}

						//get the smallest one
						std::sort(neighboursMin.begin(),neighboursMin.end());
						smallestLabel = neighboursMin[0];

						//update the equivalence table by the way
						//for all other branches
						lastLabel = smallestLabel;
						{
							for (uchar n=1; n<neighboursMin.size(); n++)
							{
								int label = neighboursMin[n];
								assert(label<static_cast<int>(numberOfCells)+2);
								//we don't process it if it's the same label as the precedent neighbor
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

			std::swap(slice,oldSlice);

			if (nprogress)
				nprogress->oneStep();
		}
	}

    //on libere un peu la memoire
    delete[] slice;
    delete[] oldSlice;

    if (progressCb)
	{
		progressCb->stop();
        if (nprogress)
			delete nprogress;
		nprogress=0;
	}

    if (currentLabel<2) //No CC found !!!
    {
        delete[] cellIndexToLabel;
        delete[] equivalentLabels;
        return -3;
    }

    //path compression (http://en.wikipedia.org/wiki/Union_find)
    assert(currentLabel<numberOfCells+2);
	{
		for (size_t i=2; i<=currentLabel; i++)
		{
			int label = equivalentLabels[i];
			assert(label<static_cast<int>(numberOfCells)+2);
			while (equivalentLabels[label] > 1) //il faut donc que equivalentLabels[0] == 0 !!!
			{
				label = equivalentLabels[label];
				assert(label<static_cast<int>(numberOfCells)+2);
			}
			equivalentLabels[i] = label;
		}
	}

    //update leafs
	{
		for (size_t i=0; i<numberOfCells; i++)
		{
			int label = cellIndexToLabel[i];
			assert(label<static_cast<int>(numberOfCells)+2);
			if (equivalentLabels[label]>1)
				cellIndexToLabel[i] = equivalentLabels[label];
		}
	}

    //hack: we use "equivalentLabels" to count how many components will have to be created
	int numberOfComponents = 0;
	{
		memset(equivalentLabels,0,(numberOfCells+2)*sizeof(int));

		for (size_t i=0; i<numberOfCells; i++)
		{
			assert(cellIndexToLabel[i]>1 && cellIndexToLabel[i]<static_cast<int>(numberOfCells)+2);
			equivalentLabels[cellIndexToLabel[i]]=1;
		}

		//we create (following) indexes for each components
		for (size_t i=2; i<numberOfCells+2; i++)
			if (equivalentLabels[i] == 1)
				equivalentLabels[i]=++numberOfComponents; //labels start at '1'
	}
    assert(equivalentLabels[0] == 0);
    assert(equivalentLabels[1] == 0);

    //we flag each component's points with its label
	{
		if (progressCb)
		{
			progressCb->reset();
			nprogress = new NormalizedProgress(progressCb,static_cast<unsigned>(numberOfCells));
			char buffer[256];
			sprintf(buffer,"Components: %i",numberOfComponents);
			progressCb->setMethodTitle("Connected Components Extraction");
			progressCb->setInfo(buffer);
			progressCb->start();
		}

		for (size_t i=0; i<numberOfCells; i++)
		{
			assert(cellIndexToLabel[i]<static_cast<int>(numberOfCells)+2);

			const int& label = equivalentLabels[cellIndexToLabel[i]];
			assert(label>0);
			ReferenceCloud* Y = getPointsInCell(ccCells[i].theCode,level,true);
			assert(Y);
			if (Y)
			{
				Y->placeIteratorAtBegining();
				ScalarType d = static_cast<ScalarType>(label);
				for (unsigned j=0; j<Y->size(); ++j)
				{
					Y->setCurrentPointScalarValue(d);
					Y->forwardIterator();
				}
			}

			if (nprogress)
				nprogress->oneStep();
		}

		if (progressCb)
		{
			progressCb->stop();
			if (nprogress)
				delete nprogress;
			nprogress=0;
		}
	}

	if (cellIndexToLabel)
		delete[] cellIndexToLabel;
	if (equivalentLabels)
		delete[] equivalentLabels;

    return 0;
}

#ifdef ENABLE_SANKARANARAYANAN_NN_SEARCH
void DgmOctree::prepareCellForNNSearch(OctreeCellCodeType truncatedCellCode, int cellPos[], uchar level, int numberOfNeighbours, NeighboursSet &thePoints) const
{
    int depth=0;

    //les cellules qu'on va visiter
    std::vector<cellDescription> cellsToVisit;

    //en theorie, le vecteur "thePoints" devrait etre vide, mais on sait jamais
    thePoints.clear();

    //binary shift for cell code truncation
    uchar bitDec = GET_BIT_SHIFT(level);

    cellDescription cd;
    //iterateur sur les codes pour reuperer les points dans les cellules
    cellsContainer::const_iterator p;

    //la distance maximale de la cellule qui une fois rajoutee a permis de depasser le nombre de voisins requis
    int M=-1;

    //on commence par regarder si la cellule de depart existe
    unsigned index = getCellIndex(truncatedCellCode,bitDec,true);
    if (index<m_numberOfProjectedPoints)
    {
        cd.truncatedCode = truncatedCellCode;
        cd.firstPointIndex = index;
        cd.minDist = 0;
        cd.maxDist = 3; //1^2+1^2+1^2 ;)

        //cellsToVisit.push_back(cd); //inutile, on va plutot directement recuperer les points

        //on recupere les points dans la cellule
        for (p = m_thePointsAndTheirCellCodes.begin()+cd.firstPointIndex; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == cd.truncatedCode); ++p)
        {
            PointDescriptor newPoint(m_theAssociatedCloud->getPointPersistentPtr(p->theIndex),p->theIndex);
            thePoints.push_back(newPoint);
        }

        if (static_cast<int>(thePoints.size()) >= numberOfNeighbours)
            M = cd.maxDist;
    }

    int limits[6];
    int &a0 = limits[0];
    int &b0 = limits[1];
    int &c0 = limits[2];
    int &d0 = limits[3];
    int &e0 = limits[4];
    int &f0 = limits[5];
    if (!getCellDistanceFromBorders(cellPos,
									level,
									1<<MAX_OCTREE_LEVEL,
									limits))
		return;

    bool imax,jmax;
    int v0,v1,v2,i,j,k,abs_i,abs_j,abs_k,ii,jj,kk,a,b,c,d,e,f;
    int nextDepthMinimumMaxDist=5,nextDepthMinimumMinDist=0;
    OctreeCellCodeType code_x,code_xy,code_xyz,old_c2 = 0;
    unsigned old_index = 0;
    //on ajoute les cellules dans l'ordre MAX_DIST croissant (Cf. article de Sankaranarayanan, Samet & Varshney, PBG06)
    //puis une fois qu'on a assez de points (plus de N), on les ajoute dans l'ordre MIN_DIST croissant
    while (true)
    {
        //si on est en mode MIN_DIST et que la distance minimale des cellules du voisinage qu'on va traiter est superieure a M, alors c'est fini
        if ((M>=0)&&(nextDepthMinimumMinDist>=M)) return; //LA ENCORE > ou >= ?

        //on avance d'un cran dans la recherche de voisinages
        ++depth;

        nextDepthMinimumMaxDist = 2+(depth+2)*(depth+2);
        nextDepthMinimumMinDist = depth*depth;

        //recherche des cellules voisine (profondeur depth > 0)
        a = std::min(a0,depth);
        b = std::min(b0,depth);
        c = std::min(c0,depth);
        d = std::min(d0,depth);
        e = std::min(e0,depth);
        f = std::min(f0,depth);

        v0=cellPos[0]-a;
        for (i=-a; i<=b; i++)
        {
            abs_i = abs(i);
            imax = (abs_i == depth);
            code_x = GenerateCellCodeForDim(v0);

            v1=cellPos[1]-c;
            for (j=-c; j<=d; j++)
            {
                abs_j = abs(j);
                jmax=(abs_j == depth);
                code_xy = code_x+(GenerateCellCodeForDim(v1)<<1);

                //si i ou j est maximal
                if (imax||jmax)
                {
                    v2=cellPos[2]-e;
                    //on est forcement sur le bord du voisinage
                    for (k=-e; k<=f; k++)
                    {
                        code_xyz = code_xy+(GenerateCellCodeForDim(v2)<<2);
                        index = (old_c2<code_xyz ? getCellIndex(code_xyz,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(code_xyz,bitDec,0,old_index));
                        if (index < m_numberOfProjectedPoints)
                        {
                            cd.truncatedCode = code_xyz;
                            cd.firstPointIndex = index;
                            abs_k = abs(k);
                            ii = (abs_i == 0 ? 0 : abs_i-1);
                            jj = (abs_j == 0 ? 0 : abs_j-1);
                            kk = (abs_k == 0 ? 0 : abs_k-1);
                            cd.minDist = ii*ii+jj*jj+kk*kk;
                            ii = abs_i+1;
                            jj = abs_j+1;
                            kk = abs_k+1;
                            cd.maxDist = ii*ii+jj*jj+kk*kk;

                            cellsToVisit.push_back(cd);
                            old_index = index;
                            old_c2=code_xyz;
                        }
                        ++v2;
                    }

                }
                else //on doit se mettre au bord du cube
                {
                    if (e == depth) //cote negatif
                    {
                        v2=cellPos[2]-e;
                        code_xyz = code_xy+(GenerateCellCodeForDim(v2)<<2);
                        index = (old_c2<code_xyz ? getCellIndex(code_xyz,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(code_xyz,bitDec,0,old_index));
                        if (index < m_numberOfProjectedPoints)
                        {
                            cd.truncatedCode = code_xyz;
                            cd.firstPointIndex = index;
                            abs_k = depth; //(abs(-e))
                            ii = (abs_i == 0 ? 0 : abs_i-1);
                            jj = (abs_j == 0 ? 0 : abs_j-1);
                            kk = abs_k-1; //kk = (abs_k > 0 ? abs_k-1 : 0); --> abs_k=depth, and depth>0!
                            cd.minDist = ii*ii+jj*jj+kk*kk;
                            ii = abs_i+1;
                            jj = abs_j+1;
                            kk = abs_k+1;
                            cd.maxDist = ii*ii+jj*jj+kk*kk;

                            cellsToVisit.push_back(cd);
                            old_index = index;
                            old_c2=code_xyz;
                        }
                    }

                    if (f == depth) //cote positif (rq : neighbourhoodLength>0)
                    {
                        v2=cellPos[2]+f;
                        code_xyz = code_xy+(GenerateCellCodeForDim(v2)<<2);
                        index = (old_c2<code_xyz ? getCellIndex(code_xyz,bitDec,old_index,m_numberOfProjectedPoints-1) : getCellIndex(code_xyz,bitDec,0,old_index));
                        if (index < m_numberOfProjectedPoints)
                        {
                            cd.truncatedCode = code_xyz;
                            cd.firstPointIndex = index;
                            abs_k = depth; //abs(f)
                            ii = (abs_i == 0 ? 0 : abs_i-1);
                            jj = (abs_j == 0 ? 0 : abs_j-1);
                            kk = abs_k-1;  //kk = (abs_k > 0 ? abs_k-1 : 0); --> abs_k=depth, and depth>0!
                            cd.minDist = ii*ii+jj*jj+kk*kk;
                            ii = abs_i+1;
                            jj = abs_j+1;
                            kk = abs_k+1;
                            cd.maxDist = ii*ii+jj*jj+kk*kk;

                            cellsToVisit.push_back(cd);
                            old_index = index;
                            old_c2=code_xyz;
                        }
                    }
                }

                ++v1;
            }

            ++v0;
        }

        //maintenant on va trier les cellules en fonction de MIN_DIST ou MAX_DIST (en fonction de notre avancement dans le processus)
        if (!cellsToVisit.empty())
        {
            //si on n'a pas encore atteint le nombre minimal de voisins
            if (M<0)
            {
                //on trie donc en fonction de MAX_DIST (attention, tri descendant ! --> le plus petit est a la fin ;)
				std::sort(cellsToVisit.begin(),cellsToVisit.end(),cellDescription::maxDistComp);

                //on va traiter les cellules une par une, selon MAX_DIST croissant
                int md,currentMaxDist = -1;
                while (!cellsToVisit.empty())
                {
                    md = cellsToVisit.back().maxDist;
                    //si on a atteint un niveau trop avance, on ne peut pas continuer a cette profondeur
                    if (md >= nextDepthMinimumMaxDist) break;

                    //si on passe au "niveau" (de MaxDist) suivant
                    if (md!=currentMaxDist)
                    {
                        //si on a amasse suffisamment de points
                        if (static_cast<int>(thePoints.size()) >= numberOfNeighbours)
                        {
                            M = currentMaxDist; //on change de mode
                            break;
                        }
                        //sinon on passe au niveau suivant
                        currentMaxDist = md;
                    }

                    //on engrange les points
                    cellDescription& cd = cellsToVisit.back();
                    for (p = m_thePointsAndTheirCellCodes.begin()+cd.firstPointIndex; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == cd.truncatedCode); ++p)
                    {
                        PointDescriptor newPoint(m_theAssociatedCloud->getPointPersistentPtr(p->theIndex),p->theIndex);
                        thePoints.push_back(newPoint);
                    }

                    //on passe a la cellule suivante
                    cellsToVisit.pop_back();
                }

                //si on a amasse suffisamment de points, on change de mode
                if (static_cast<int>(thePoints.size()) >= numberOfNeighbours)
					M = currentMaxDist;
            }

            //si on a atteint le nombre minimal de voisins
            if (M>=0) //>=3 meme en theorie !
            {
                //on trie le vecteur en fonction de MIN_DIST (attention, tri descendant !)
                //inutile pour depth=1, car toutes les cellules ont une MIN_DIST == 0 ! (et min(MAX_DIST)=3)
				if (depth>1) std::sort(cellsToVisit.begin(),cellsToVisit.end(),cellDescription::minDistComp);

                //on va traiter les cellules une par une, selon MIN_DIST croissant
                int md,currentMinDist = -1;
                while (!cellsToVisit.empty())
                {
                    md = cellsToVisit.back().minDist;

                    //si on passe au "niveau" (de MinDist) suivant
                    if (md!=currentMinDist)
                    {
                        //si les cellules sont "hors champ"
                        if (md >= M) //DOUTE : > ou >= ?!
                        {
                            //on peut arreter !!!
                            return;
                        }
                        //sinon on passe au niveau suivant
                        currentMinDist = md;
                    }

                    //si on a atteint un niveau trop avance, on ne peut pas continuer a cette profondeur
                    if (md >= nextDepthMinimumMinDist) break;

                    //on engrange les points
                    cellDescription& cd = cellsToVisit.back();
                    for (p = m_thePointsAndTheirCellCodes.begin()+cd.firstPointIndex; (p != m_thePointsAndTheirCellCodes.end()) && ((p->theCode >> bitDec) == cd.truncatedCode); ++p)
                    {
                        PointDescriptor newPoint(m_theAssociatedCloud->getPointPersistentPtr(p->theIndex),p->theIndex);
                        thePoints.push_back(newPoint);
                    }

                    //on passe a la cellule suivante
                    cellsToVisit.pop_back();
                }

                if (currentMinDist >= M) //DOUTE : > ou >= ?!
                    return;
            }
        }
    }
}

void DgmOctree::getNNPointsAmong(NeighboursSet &thePoints, CCVector3* queryPoint, int numberOfNeighbours, ReferenceCloud* Zk, bool alreadySorted) const
{
    assert(static_cast<int>(thePoints.size()) >= numberOfNeighbours);

    NeighboursSet::iterator p;

    //optimisation possible si on connait la position de l'ancien "queryPoint" (article de J. Sankaranarayana et al., PBG06)
    //ATTENTION, numberOfNeighbours ne doit pas avoir change !
    if (alreadySorted)
    {
        //on calcule la distance entre le nouveau point requete et l'ancien plus proche voisin de rang k (le dernier)
        //les plus proches voisins de qd sont forcement dans la sphere de rayon qd, donc dans une bbox carree de largeur 2.qd
        ScalarType qd = (thePoints[numberOfNeighbours-1].point - *queryPoint).norm();

        int j,k=numberOfNeighbours;

        //on s'occupe d'abord des anciens k voisins (on calcule juste leur distance)
        p=thePoints.begin();
        for (j=0; j<k; ++j)
        {
            p->squareDist = (p->point - *queryPoint).norm2();
            ++p;
        }

        //et maintenant on s'occupe du reste, en ne prenant en compte que les points qui tombent dans dans la BBox
        while(p!=thePoints.end())
        {
            //on calcule la difference entre les points du voisinage et le nouveau point requete
            CCVector3 Pdiff = p->point - *queryPoint;
            //s'ils tombent dans la BBbox de demi largeur qd et centree sur le nouveau point requete
            if ((fabs(Pdiff.x)<=qd) && (fabs(Pdiff.y)<=qd) && (fabs(Pdiff.z)<=qd))
            {
                //on calcule sa distance
                p->squareDist = Pdiff.norm2();
                //et on le met au debut du vecteur
                if (j>k)
                    std::swap(*p,thePoints[k]);
                ++k;
            }
            ++p;
            ++j;
        }

        assert(k>=numberOfNeighbours);

        //on ne trie que les "k" premiers points du vecteur (ceux qui tombent dans la BBox)
        std::partial_sort(thePoints.begin(),thePoints.begin()+numberOfNeighbours,thePoints.begin()+k,PointDescriptor());
    }
    //sinon on est oblige de comparer tous les points au point requete
    else
    {
        //on calcule la distance par rapport a tous les points
        for (p=ENABLE_SANKARANARAYANAN_NN_SEARCHthePoints.begin(); p!=thePoints.end(); ++p)
            p->squareDist = (p->point - *queryPoint).norm2();

        //on trie le vecteur pour choper les "numberOfNeighbours" premiers
        std::partial_sort(thePoints.begin(),thePoints.begin()+numberOfNeighbours,thePoints.end(),PointDescriptor());
    }

    p = thePoints.begin();
    for (int i=0; i<numberOfNeighbours; ++i, ++p)
        Zk->addPointIndex(p->theIndex);
}
#endif

/*** Octree-based cloud traversal mechanism ***/

DgmOctree::octreeCell::octreeCell(DgmOctree* _parentOctree)
    : parentOctree(_parentOctree)
    , level(0)
    , truncatedCode(0)
    , index(0)
    , points(0)
{
    assert(parentOctree && parentOctree->m_theAssociatedCloud);
    points = new ReferenceCloud(parentOctree->m_theAssociatedCloud);
}

DgmOctree::octreeCell::~octreeCell()
{
    if (points)
        delete points;
}

unsigned DgmOctree::executeFunctionForAllCellsAtLevel(uchar level,
        octreeCellFunc func,
        void** additionalParameters,
        GenericProgressCallback* progressCb,
        const char* functionTitle)
{
    if (m_thePointsAndTheirCellCodes.empty())
        return 0;

	//we get the maximum cell population for this level
	unsigned maxCellPopulation=m_maxCellPopulation[level];

    //cell descriptor (initialize it with first cell/point)
    octreeCell cell(this);
	if (!cell.points->reserve(maxCellPopulation)) //not enough memory
		return 0;
	cell.level=level;
    cell.index = 0;

	//binary shift for cell code truncation
    uchar bitDec = GET_BIT_SHIFT(level);

    //iterator on cell codes
    cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin();

	//init with first cell
    cell.truncatedCode = (p->theCode >> bitDec);
	cell.points->addPointIndex(p->theIndex); //can't fail (see above)
	++p;

	//number of cells for this level
	unsigned cellCount=getCellNumber(level);

	//progress bar
	NormalizedProgress* nprogress = 0;
    if (progressCb)
    {
        progressCb->reset();
        if (functionTitle)
            progressCb->setMethodTitle(functionTitle);
        char buffer[512];
		sprintf(buffer,"Octree level %i\nCells: %u\nMean population: %3.2f (+/-%3.2f)\nMax population: %i",level,cellCount,m_averageCellPopulation[level],m_stdDevCellPopulation[level],m_maxCellPopulation[level]);
		nprogress = new NormalizedProgress(progressCb,m_theAssociatedCloud->size());
        progressCb->setInfo(buffer);
        progressCb->start();
    }

	bool result = true;

#ifdef COMPUTE_NN_SEARCH_STATISTICS
	s_skippedPoints = 0;
	s_testedPoints = 0;
	s_jumps = 0.0;
	s_binarySearchCount = 0.0;
#endif

    //for each point
    for (; p!=m_thePointsAndTheirCellCodes.end(); ++p)
    {
		//check if it belongs to the current cell
        OctreeCellCodeType nextCode = (p->theCode >> bitDec);
        if (nextCode != cell.truncatedCode)
        {
            //if not, we call the user function on the precedent cell
            result = (*func)(cell,additionalParameters,nprogress);

			if (!result)
				break;

			//and we start a new cell
            cell.index+=cell.points->size();
            cell.points->clear(false);
			cell.truncatedCode = nextCode;

			//if (nprogress && !nprogress->oneStep())
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
		result = (*func)(cell,additionalParameters, nprogress);

#ifdef COMPUTE_NN_SEARCH_STATISTICS
	FILE* fp=fopen("octree_log.txt","at");
	if (fp)
	{
		fprintf(fp,"Function: %s\n",functionTitle ? functionTitle : "unknown");
		fprintf(fp,"Tested:  %f (%3.1f %%)\n",s_testedPoints,100.0*s_testedPoints/std::max(1.0,s_testedPoints+s_skippedPoints));
		fprintf(fp,"skipped: %f (%3.1f %%)\n",s_skippedPoints,100.0*s_skippedPoints/std::max(1.0,s_testedPoints+s_skippedPoints));
		fprintf(fp,"Binary search count: %.0f\n",s_binarySearchCount);
		if (s_binarySearchCount>0.0)
			fprintf(fp,"Mean jumps: %f\n",s_jumps/s_binarySearchCount);
		fprintf(fp,"\n");
		fclose(fp);
	}
#endif

    if (nprogress)
	{
		delete nprogress;
		nprogress = 0;
	}

	//if something went wrong, we return 0
	return (result ? cellCount : 0);
}

#define ENABLE_DOWN_TOP_TRAVERSAL
unsigned DgmOctree::executeFunctionForAllCellsAtStartingLevel(uchar startingLevel,
        octreeCellFunc func,
        void** additionalParameters,
        unsigned minNumberOfPointsPerCell,
        unsigned maxNumberOfPointsPerCell,
        GenericProgressCallback* progressCb,
        const char* functionTitle)
{
    if (m_thePointsAndTheirCellCodes.empty())
        return 0;

	//we get the maximum cell population for this level
	unsigned maxCellPopulation=m_maxCellPopulation[startingLevel];

	//cell descriptor
    octreeCell cell(this);
	if (!cell.points->reserve(maxCellPopulation)) //not enough memory
		return 0;
	cell.level = startingLevel;
	cell.index = 0;

    //progress notification
#ifndef ENABLE_DOWN_TOP_TRAVERSAL
	NormalizedProgress* nprogress = 0;
#endif
	const unsigned cellsNumber = getCellNumber(startingLevel);
    if (progressCb)
    {
        progressCb->reset();
        if (functionTitle)
            progressCb->setMethodTitle(functionTitle);
        char buffer[1024];
		sprintf(buffer,"Octree levels %i - %i\nCells: %i - %i\nMean population: %3.2f (+/-%3.2f) - %3.2f (+/-%3.2f)\nMax population: %i - %i",
				startingLevel,MAX_OCTREE_LEVEL,
				getCellNumber(startingLevel),getCellNumber(MAX_OCTREE_LEVEL),
				m_averageCellPopulation[startingLevel],m_stdDevCellPopulation[startingLevel],
				m_averageCellPopulation[MAX_OCTREE_LEVEL],m_stdDevCellPopulation[MAX_OCTREE_LEVEL],
				m_maxCellPopulation[startingLevel],m_maxCellPopulation[MAX_OCTREE_LEVEL]);
        progressCb->setInfo(buffer);
#ifndef ENABLE_DOWN_TOP_TRAVERSAL
		nprogress = new NormalizedProgress(progressCb,m_theAssociatedCloud->size());
#endif
        progressCb->start();
    }

    //binary shift for cell code truncation at current level
    uchar currentBitDec = GET_BIT_SHIFT(startingLevel);

#ifdef ENABLE_DOWN_TOP_TRAVERSAL
	bool firstSubCell=true;
#else
	uchar shallowSteps = 0;
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
		//	if (nprogress && !nprogress->oneStep())
		//	{
		//		result=false;
		//		break;
		//	}
		//}
#else
		//in this mode, we can't update progress notification regularly...
		if (progressCb)
		{
			progressCb->update(100.0f*static_cast<float>(cell.index)/static_cast<float>(m_numberOfProjectedPoints));
			if (progressCb->isCancelRequested())
			{
				result=false;
				break;
			}
		}
#endif

		//let's test the following points
		for (cellsContainer::const_iterator p = startingElement+1; p != m_thePointsAndTheirCellCodes.end(); ++p)
        {
			//next point code (at current level of subdivision)
            OctreeCellCodeType currentTruncatedCode = (p->theCode >> currentBitDec);
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
					while (cell.level<MAX_OCTREE_LEVEL)
					{
						//next level
						++cell.level;
						currentBitDec -= 3;
						cell.truncatedCode = (startingElement->theCode >> currentBitDec);

						//not the same cell anymore?
						if (cell.truncatedCode != (p->theCode >> currentBitDec))
						{
							//we must re-check all the precedently inserted points at this new level
							//to determine the end of this new cell
							p = startingElement;
							elements=1;
							while (((++p)->theCode >> currentBitDec) == cell.truncatedCode)
								++elements;

							//and we must stop point collection here
							keepGoing=false;

#ifdef ENABLE_DOWN_TOP_TRAVERSAL
							//in this case, the next cell won't be the first sub-cell!
							firstSubCell=false;
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
				OctreeCellCodeType cellTruncatedCode = cell.truncatedCode;
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
					if ((cell.truncatedCode>>3) == (currentTruncatedCode>>3))
					{
						//if this cell is the first one, and we don't have enough points
						//we can simply proceed with its parent cell
						if (firstSubCell && elements < minNumberOfPointsPerCell)
						{
							//precedent level
							--cell.level;
							currentBitDec+=3;
							cell.truncatedCode>>=3;

							//we 'add' the point to the cell descriptor
							++elements;
							//and we can continue collecting points
							keepGoing=true;
						}

						//as this cell and the next one share the same parent,
						//the next cell won't be the first sub-cell!
						firstSubCell=false;
					}
					else
					{
						//as this cell and the next one have differnt parents,
						//the next cell is the first sub-cell!
						firstSubCell=true;
					}
				}
				else
				{
					//at the ceiling level, all cells are considered as 'frist' sub-cells
					firstSubCell=true;
				}

				//we must stop point collection here
				if (!keepGoing)
					break;
#endif
			}
        }

		//we can now really 'add' the points to the cell descriptor
        cell.points->clear(false);
		//DGM: already done earlier
		/*if (!cell.points->reserve(elements)) //not enough memory
		{
			result=false;
			break;
		}
		//*/
		for (unsigned i=0;i<elements;++i)
			cell.points->addPointIndex((startingElement++)->theIndex);

		//call user method on current cell
		result = (*func)(cell,additionalParameters,
#ifndef ENABLE_DOWN_TOP_TRAVERSAL
			nProgress
#else
			0
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
			assert(cell.level-shallowSteps>=startingLevel);
			cell.level-=shallowSteps;
			currentBitDec += 3*shallowSteps;
			shallowSteps=0;
		}
#endif
    }

    if (progressCb)
	{
        progressCb->stop();
#ifndef ENABLE_DOWN_TOP_TRAVERSAL
		delete nprogress;
		nprogress=0;
#endif
	}

	//if something went wrong, we return 0
	return (result ? cellsNumber : 0);
}

#ifdef ENABLE_MT_OCTREE

#include <QtCore/QtCore>
#include <QtGui/QApplication>

/*** MULTI THREADING WRAPPER ***/

struct octreeCellDesc
{
	DgmOctree::OctreeCellCodeType truncatedCode;
	unsigned i1,i2;
	uchar level;
};

static DgmOctree* s_octree_MT = 0;
static DgmOctree::octreeCellFunc s_func_MT = 0;
static void** s_userParams_MT = 0;
static GenericProgressCallback* s_progressCb_MT = 0;
static NormalizedProgress* s_normProgressCb_MT = 0;
static bool s_cellFunc_MT_success = true;

void LaunchOctreeCellFunc_MT(const octreeCellDesc& desc)
{
	//skip cell if process is aborted/has failed
	if (!s_cellFunc_MT_success)
		return;

	const DgmOctree::cellsContainer& pointsAndCodes = s_octree_MT->pointsAndTheirCellCodes();

    //cell descriptor
    DgmOctree::octreeCell cell(s_octree_MT);
	cell.level = desc.level;
	cell.index = desc.i1;
	cell.truncatedCode = desc.truncatedCode;
	if (cell.points->reserve(desc.i2-desc.i1+1))
	{
		for (unsigned i=desc.i1; i<=desc.i2; ++i)
			cell.points->addPointIndex(pointsAndCodes[i].theIndex);

		s_cellFunc_MT_success &= (*s_func_MT)(cell,s_userParams_MT,s_normProgressCb_MT);
	}
	else
	{
		s_cellFunc_MT_success = false;
	}

	if (!s_cellFunc_MT_success)
	{
		//TODO: display a message to make clear that the cancel order has been understood!
		if (s_progressCb_MT)
		{
			s_progressCb_MT->setInfo("Cancelling...");
			QApplication::processEvents();
		}

	//if (s_normProgressCb_MT)
	//{
	//	//QApplication::processEvents(); //let the application breath!
	//	if (!s_normProgressCb_MT->oneStep())
	//	{
	//		s_cellFunc_MT_success = false;
	//		return;
	//	}
	//}
	}
}

unsigned DgmOctree::executeFunctionForAllCellsAtLevel_MT(uchar level,
        octreeCellFunc func,
        void** additionalParameters,
        GenericProgressCallback* progressCb,
        const char* functionTitle)
{
    if (m_thePointsAndTheirCellCodes.empty())
        return 0;

	const unsigned cellsNumber = getCellNumber(level);

	//cells that will be processed by QtConcurrent::map
	std::vector<octreeCellDesc> cells;
	cells.reserve(cellsNumber);
	if (cells.capacity() < cellsNumber) //not enough memory
		//we use standard way (DGM TODO: we should warn the user!)
		return executeFunctionForAllCellsAtLevel(level,func,additionalParameters,progressCb,functionTitle);

    //binary shift for cell code truncation
    uchar bitDec = GET_BIT_SHIFT(level);

    //iterator on cell codes
    cellsContainer::const_iterator p = m_thePointsAndTheirCellCodes.begin();

    //cell descriptor (init. with first point/cell)
	octreeCellDesc cellDesc;
	cellDesc.i1=0;
	cellDesc.i2=0;
	cellDesc.level=level;
    cellDesc.truncatedCode = (p->theCode >> bitDec);
	++p;

    //sweep through the octree
    for (; p!=m_thePointsAndTheirCellCodes.end(); ++p)
    {
        OctreeCellCodeType nextCode = (p->theCode >> bitDec);

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
        progressCb->reset();
        if (functionTitle)
            progressCb->setMethodTitle(functionTitle);
        char buffer[512];
		sprintf(buffer,"Octree level %i\nCells: %i\nMean population: %3.2f (+/-%3.2f)\nMax population: %d",level,static_cast<int>(cells.size()),m_averageCellPopulation[level],m_stdDevCellPopulation[level],m_maxCellPopulation[level]);
        progressCb->setInfo(buffer);
		s_normProgressCb_MT = new NormalizedProgress(progressCb,m_theAssociatedCloud->size());
        progressCb->start();
    }

#ifdef COMPUTE_NN_SEARCH_STATISTICS
	s_skippedPoints = 0;
	s_testedPoints = 0;
	s_jumps = 0.0;
	s_binarySearchCount = 0.0;
#endif

	QtConcurrent::blockingMap(cells, LaunchOctreeCellFunc_MT);

#ifdef COMPUTE_NN_SEARCH_STATISTICS
	FILE* fp=fopen("octree_log.txt","at");
	if (fp)
	{
		fprintf(fp,"Function: %s\n",functionTitle ? functionTitle : "unknown");
		fprintf(fp,"Tested:  %f (%3.1f %%)\n",s_testedPoints,100.0*s_testedPoints/std::max(1.0,s_testedPoints+s_skippedPoints));
		fprintf(fp,"skipped: %f (%3.1f %%)\n",s_skippedPoints,100.0*s_skippedPoints/std::max(1.0,s_testedPoints+s_skippedPoints));
		fprintf(fp,"Binary search count: %.0f\n",s_binarySearchCount);
		if (s_binarySearchCount>0.0)
			fprintf(fp,"Mean jumps: %f\n",s_jumps/s_binarySearchCount);
		fprintf(fp,"\n");
		fclose(fp);
	}
#endif

	s_octree_MT = 0;
	s_func_MT = 0;
	s_userParams_MT = 0;

	if (progressCb)
	{
        progressCb->stop();
		if (s_normProgressCb_MT)
			delete s_normProgressCb_MT;
		s_normProgressCb_MT = 0;
		s_progressCb_MT = 0;
	}

	//if something went wrong, we clear everything and return 0!
	if (!s_cellFunc_MT_success)
		cells.clear();

    return static_cast<unsigned>(cells.size());
}

#define ENABLE_DOWN_TOP_TRAVERSAL_MT
unsigned DgmOctree::executeFunctionForAllCellsAtStartingLevel_MT(uchar startingLevel,
        octreeCellFunc func,
        void** additionalParameters,
        unsigned minNumberOfPointsPerCell,
        unsigned maxNumberOfPointsPerCell,
        GenericProgressCallback* progressCb,
        const char* functionTitle)
{
    if (m_thePointsAndTheirCellCodes.empty())
        return 0;

	const unsigned cellsNumber = getCellNumber(startingLevel);

	//cells that will be processed by QtConcurrent::map
	std::vector<octreeCellDesc> cells;
	cells.reserve(cellsNumber); //at least!
	if (cells.capacity() < cellsNumber) //not enough memory?
		//we use standard way (DGM TODO: we should warn the user!)
		return executeFunctionForAllCellsAtStartingLevel(startingLevel,
														func,
														additionalParameters,
														minNumberOfPointsPerCell,
														maxNumberOfPointsPerCell,
														progressCb,
														functionTitle);

    //cell descriptor (init. with first point/cell)
	octreeCellDesc cellDesc;
	cellDesc.i1=0;
	cellDesc.i2=0;
	cellDesc.level=startingLevel;

	//binary shift for cell code truncation at current level
    uchar currentBitDec = GET_BIT_SHIFT(startingLevel);

#ifdef ENABLE_DOWN_TOP_TRAVERSAL_MT
	bool firstSubCell=true;
#else
	uchar shallowSteps = 0;
#endif

	//pointer on the current octree element
	cellsContainer::const_iterator startingElement = m_thePointsAndTheirCellCodes.begin();

	//we compute some statistics on the fly
	double popSum = 0.0;
	double popSum2 = 0.0;
	unsigned maxPop = 0;

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
            OctreeCellCodeType currentTruncatedCode = (p->theCode >> currentBitDec);
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
					while (cellDesc.level<MAX_OCTREE_LEVEL)
					{
						//next level
						++cellDesc.level;
						currentBitDec -= 3;
						cellDesc.truncatedCode = (startingElement->theCode >> currentBitDec);

						//not the same cell anymore?
						if (cellDesc.truncatedCode != (p->theCode >> currentBitDec))
						{
							//we must re-check all the precedently inserted points at this new level
							//to determine the end of this new cell
							p = startingElement;
							elements=1;
							while (((++p)->theCode >> currentBitDec) == cellDesc.truncatedCode)
								++elements;

							//and we must stop point collection here
							keepGoing=false;

#ifdef ENABLE_DOWN_TOP_TRAVERSAL_MT
							//in this case, the next cell won't be the first sub-cell!
							firstSubCell=false;
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
				OctreeCellCodeType cellTruncatedCode = cellDesc.truncatedCode;
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
							//precedent level
							--cellDesc.level;
							currentBitDec+=3;
							cellDesc.truncatedCode>>=3;

							//we 'add' the point to the cell descriptor
							++elements;
							//and we can continue collecting points
							keepGoing=true;
						}

						//as this cell and the next one share the same parent,
						//the next cell won't be the first sub-cell!
						firstSubCell=false;
					}
					else
					{
						//as this cell and the next one have differnt parents,
						//the next cell is the first sub-cell!
						firstSubCell=true;
					}
				}
				else
				{
					//at the ceiling level, all cells are considered as 'frist' sub-cells
					firstSubCell=true;
				}

				//we must stop point collection here
				if (!keepGoing)
					break;
#endif
			}
        }

		//we can now 'add' this cell to the list
		cellDesc.i2=cellDesc.i1+(elements-1);
		cells.push_back(cellDesc);
		popSum += static_cast<double>(elements);
		popSum2 += static_cast<double>(elements)*static_cast<double>(elements);
		if (maxPop<elements)
			maxPop=elements;

		//proceed to next cell
		cellDesc.i1 += elements;
		startingElement += elements;

#ifndef ENABLE_DOWN_TOP_TRAVERSAL_MT
		if (shallowSteps)
		{
			//we should go shallower
			assert(cellDesc.level-shallowSteps>=startingLevel);
			cellDesc.level-=shallowSteps;
			currentBitDec += 3*shallowSteps;
			shallowSteps=0;
		}
#endif
    }

	//statistics
	double mean = popSum/static_cast<double>(cells.size());
	double stddev = sqrt(popSum2-mean*mean)/static_cast<double>(cells.size());

	//static wrap
	s_octree_MT = this;
	s_func_MT = func;
	s_userParams_MT = additionalParameters;
	s_cellFunc_MT_success = true;
	if (s_normProgressCb_MT)
		delete s_normProgressCb_MT;
	s_normProgressCb_MT = 0;

    //progress notification
    if (progressCb)
    {
        progressCb->reset();
        if (functionTitle)
            progressCb->setMethodTitle(functionTitle);
        char buffer[1024];
		sprintf(buffer,"Octree levels %i - %i\nCells: %i\nMean population: %3.2f (+/-%3.2f)\nMax population: %u",startingLevel,MAX_OCTREE_LEVEL,static_cast<int>(cells.size()),mean,stddev,maxPop);
        progressCb->setInfo(buffer);
		if (s_normProgressCb_MT)
			delete s_normProgressCb_MT;
		s_normProgressCb_MT = new NormalizedProgress(progressCb,static_cast<unsigned>(cells.size()));
        progressCb->start();
    }

#ifdef COMPUTE_NN_SEARCH_STATISTICS
	s_skippedPoints = 0;
	s_testedPoints = 0;
	s_jumps = 0.0;
	s_binarySearchCount = 0.0;
#endif

	QtConcurrent::blockingMap(cells, LaunchOctreeCellFunc_MT);

#ifdef COMPUTE_NN_SEARCH_STATISTICS
	FILE* fp=fopen("octree_log.txt","at");
	if (fp)
	{
		fprintf(fp,"Function: %s\n",functionTitle ? functionTitle : "unknown");
		fprintf(fp,"Tested:  %f (%3.1f %%)\n",s_testedPoints,100.0*s_testedPoints/std::max(1.0,s_testedPoints+s_skippedPoints));
		fprintf(fp,"skipped: %f (%3.1f %%)\n",s_skippedPoints,100.0*s_skippedPoints/std::max(1.0,s_testedPoints+s_skippedPoints));
		fprintf(fp,"Binary search count: %.0f\n",s_binarySearchCount);
		if (s_binarySearchCount>0.0)
			fprintf(fp,"Mean jumps: %f\n",s_jumps/s_binarySearchCount);
		fprintf(fp,"\n");
		fclose(fp);
	}
#endif

	s_octree_MT = 0;
	s_func_MT = 0;
	s_userParams_MT = 0;

	if (progressCb)
	{
        progressCb->stop();
		if (s_normProgressCb_MT)
			delete s_normProgressCb_MT;
		s_normProgressCb_MT=0;
	}

	//if something went wrong, we clear everything and return 0!
	if (!s_cellFunc_MT_success)
		cells.clear();

    return static_cast<unsigned>(cells.size());
}

#endif
