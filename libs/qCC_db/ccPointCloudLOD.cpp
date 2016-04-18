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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#include "ccPointCloudLOD.h"

//Local
#include "ccPointCloud.h"

//Qt
#include <QThread>
#include <QElapsedTimer>

#ifdef USE_LOD_2

//! Thread for background computation
class ccPointCloudLODThread : public QThread
{
public:
	
	//! Default constructor
	ccPointCloudLODThread(ccPointCloud& cloud, ccPointCloudLOD& lod, uint32_t maxCountPerCell)
		: QThread()
		, m_cloud(cloud)
		, m_lod(lod)
		, m_octree(0)
		, m_maxCountPerCell(maxCountPerCell)
	{
	}
	
	//!Destructor
	virtual ~ccPointCloudLODThread()
	{
		terminate();
	}
	
protected:

	//! Fills a node (and returns its relative position) + recursive
	uint8_t fillNode(ccPointCloudLOD::Node& node) const
	{
		const ccOctree::cellsContainer& cellCodes = m_octree->pointsAndTheirCellCodes();
		const unsigned char bitDec = CCLib::DgmOctree::GET_BIT_SHIFT(node.level);
		const CCLib::DgmOctree::CellCode currentTruncatedCellCode = (cellCodes[node.firstCodeIndex].theCode >> bitDec);

		//first count the number of points and compute their center
		{
			node.pointCount = 0;
			CCVector3d sumP(0, 0, 0);
			for (uint32_t codeIndex = node.firstCodeIndex; codeIndex < cellCodes.size() && (cellCodes[codeIndex].theCode >> bitDec) == currentTruncatedCellCode; ++codeIndex)
			{
				++node.pointCount;
				const CCVector3* P = m_cloud.getPoint(cellCodes[codeIndex].theIndex);
				sumP += CCVector3d::fromArray(P->u);
			}

			//compute the radius
			if (node.pointCount > 1)
			{
				sumP /= node.pointCount;
				double maxSquareRadius = 0;
				for (uint32_t i = 0; i < node.pointCount; ++i)
				{
					const CCVector3* P = m_cloud.getPoint(cellCodes[node.firstCodeIndex + i].theIndex);
					double squareRadius = (CCVector3d::fromArray(P->u) - sumP).norm2();
					if (squareRadius > maxSquareRadius)
					{
						maxSquareRadius = squareRadius;
					}
				}
				node.radius = static_cast<float>(sqrt(maxSquareRadius));
			}

			//update the center
			node.center = CCVector3f::fromArray(sumP.u);
		}

		//do we need to subdivide this cell?
		if (node.pointCount > m_maxCountPerCell && node.level+1 < m_lod.m_levels.size())
		{
			for (uint32_t i = 0; i < node.pointCount; )
			{
				int32_t childNodeIndex = m_lod.newCell(node.level + 1);
				ccPointCloudLOD::Node& childNode = m_lod.node(childNodeIndex, node.level + 1);
				childNode.firstCodeIndex = node.firstCodeIndex + i;

				uint8_t childIndex = fillNode(childNode);
				node.childIndexes[childIndex] = childNodeIndex;
				node.childCount++;
				i += childNode.pointCount;
			}
		}

		//return the node relative position
		return static_cast<uint8_t>(currentTruncatedCellCode & 7);
	}

	//! Fills a node (and returns its relative position)
	uint8_t fillNode_flat(ccPointCloudLOD::Node& node) const
	{
		const ccOctree::cellsContainer& cellCodes = m_octree->pointsAndTheirCellCodes();
		const unsigned char bitDec = CCLib::DgmOctree::GET_BIT_SHIFT(node.level);
		const CCLib::DgmOctree::CellCode currentTruncatedCellCode = (cellCodes[node.firstCodeIndex].theCode >> bitDec);

		//first count the number of points and compute their center
		{
			node.pointCount = 0;
			CCVector3d sumP(0, 0, 0);
			for (uint32_t codeIndex = node.firstCodeIndex; codeIndex < cellCodes.size() && (cellCodes[codeIndex].theCode >> bitDec) == currentTruncatedCellCode; ++codeIndex)
			{
				++node.pointCount;
				const CCVector3* P = m_cloud.getPoint(cellCodes[codeIndex].theIndex);
				sumP += CCVector3d::fromArray(P->u);
			}

			//compute the radius
			if (node.pointCount > 1)
			{
				sumP /= node.pointCount;
				double maxSquareRadius = 0;
				for (uint32_t i = 0; i < node.pointCount; ++i)
				{
					const CCVector3* P = m_cloud.getPoint(cellCodes[node.firstCodeIndex + i].theIndex);
					double squareRadius = (CCVector3d::fromArray(P->u) - sumP).norm2();
					if (squareRadius > maxSquareRadius)
					{
						maxSquareRadius = squareRadius;
					}
				}
				node.radius = static_cast<float>(sqrt(maxSquareRadius));
			}

			//update the center
			node.center = CCVector3f::fromArray(sumP.u);
		}

		//return the node relative position
		return static_cast<uint8_t>(currentTruncatedCellCode & 7);
	}

	//reimplemented from QThread
	virtual void run()
	{
		//reset structure
		m_lod.clearData();
		m_lod.setState(ccPointCloudLOD::UNDER_CONSTRUCTION);

		unsigned pointCount = m_cloud.size();
		if (pointCount == 0)
		{
			m_lod.setState(ccPointCloudLOD::BROKEN);
			return;
		}

		ccLog::Print(QString("[LoD] Preparing LoD acceleration structure for cloud '%1' [%2 points]...").arg(m_cloud.getName()).arg(pointCount));
		QElapsedTimer timer;
		timer.start();

		//first we need an octree
		m_octree = m_cloud.getOctree();
		if (!m_octree)
		{
			m_octree = ccOctree::Shared(new ccOctree(&m_cloud));
			if (m_octree->build(0/*progressCallback*/) <= 0)
			{
				//not enough memory
				ccLog::Warning(QString("[LoD] Failed to compute octree on cloud '%1' (not enough memory)").arg(m_cloud.getName()));
				m_lod.setState(ccPointCloudLOD::BROKEN);
				return;
			}

			if (!m_cloud.getOctree()) //be sure that it hasn't been built in the meantime!
			{
				m_cloud.setOctree(m_octree);
			}
		}

		//init LoD structure
		if (!m_lod.initInternal(m_octree))
		{
			//not enough memory
			ccLog::Warning(QString("[LoD] Failed to compute LOD structure on cloud '%1' (not enough memory)").arg(m_cloud.getName()));
			m_lod.setState(ccPointCloudLOD::BROKEN);
			return;
		}

#if 0
		//recursive
		fillNode(m_lod.root());

		m_lod.shrink_to_fit();
		//m_lod.updateMaxRadii();
		m_lod.setMaxLevel(static_cast<unsigned char>(m_lod.m_levels.size()));

		for (size_t i = 1; i < m_lod.m_levels.size(); ++i)
		{
			ccLog::Print(QString("[LoD] Level %1: %2 cells").arg(i).arg(m_lod.m_levels[i].data.size()));
		}

#else
		//init with root node
		fillNode_flat(m_lod.root());

		for (unsigned char currentLevel = 0; currentLevel < m_lod.m_levels.size(); ++currentLevel)
		{
			ccPointCloudLOD::Level& level = m_lod.m_levels[currentLevel];
			if (level.data.empty())
			{
				break;
			}

			//update maxRadius for the previous level
			//{
			//	float maxRadius = 0;
			//	for (ccPointCloudLOD::Node& n : level.data)
			//	{
			//		if (n.radius > maxRadius)
			//		{
			//			maxRadius = n.radius;
			//		}
			//	}
			//	level.maxRadius = maxRadius;
			//}

			//the previous level is now ready!
			m_lod.setMaxLevel(currentLevel);
			ccLog::Print(QString("[LoD] Level %1: %2 cells").arg(currentLevel).arg(level.data.size()));

			//now we can create the next level
			if (currentLevel + 1 < m_lod.m_levels.size())
			{
				for (ccPointCloudLOD::Node& node : level.data)
				{
					//do we need to subdivide this cell?
					if (node.pointCount > m_maxCountPerCell)
					{
						for (uint32_t i = 0; i < node.pointCount;)
						{
							int32_t childNodeIndex = m_lod.newCell(node.level + 1);
							ccPointCloudLOD::Node& childNode = m_lod.node(childNodeIndex, node.level + 1);
							childNode.firstCodeIndex = node.firstCodeIndex + i;

							uint8_t childIndex = fillNode_flat(childNode);
							node.childIndexes[childIndex] = childNodeIndex;
							node.childCount++;
							i += childNode.pointCount;
						}
					}
				}
			}
		}

		m_lod.shrink_to_fit();
#endif

		m_lod.setState(ccPointCloudLOD::INITIALIZED);

		ccLog::Print(QString("[LoD] Acceleration structure ready for cloud '%1' (max level: %2 / mem. = %3 Mb / duration: %4 s.)")
			.arg(m_cloud.getName())
			.arg(m_lod.maxLevel())
			.arg(m_lod.memory() / static_cast<double>(1 << 20), 0, 'f', 2)
			.arg(timer.elapsed() / 1000.0,0,'f',1));
	}

	ccPointCloud& m_cloud;
	ccPointCloudLOD& m_lod;
	ccOctree::Shared m_octree;
	uint32_t m_maxCountPerCell;
};

ccPointCloudLOD::ccPointCloudLOD()
	: m_maxLevel(0)
	, m_lastIndexMap(0)
	, m_octree(0)
	, m_thread(0)
	, m_state(NOT_INITIALIZED)
{
	clearData(); //initializes the root node
}

ccPointCloudLOD::~ccPointCloudLOD()
{
	clear();

	if (m_lastIndexMap)
	{
		m_lastIndexMap->release();
	}
}

size_t ccPointCloudLOD::memory() const
{
	size_t thisSize = sizeof(ccPointCloudLOD);
	
	size_t totalNodeCount = 0;
	for (size_t i = 0; i < m_levels.size(); ++i)
	{
		totalNodeCount += m_levels[i].data.size();
	}
	size_t nodeSize = sizeof(Node);	
	size_t nodesSize = totalNodeCount * nodeSize;

	return nodesSize + thisSize;
}

bool ccPointCloudLOD::init(ccPointCloud* cloud)
{
	if (!cloud)
	{
		assert(false);
		return false;
	}
	
	if (isBroken())
	{
		return false;
	}

	if (!m_thread)
	{
		m_thread = new ccPointCloudLODThread(*cloud, *this, 256);
	}
	else if (m_thread->isRunning())
	{
		//already running?
		assert(false);
		return true;
	}

	m_thread->start();
	return true;
}

unsigned char ccPointCloudLOD::maxLevel()
{
	lock();
	unsigned char maxLevel = m_maxLevel;
	assert(maxLevel <= m_levels.size());
	unlock();
	
	return maxLevel;
}

void ccPointCloudLOD::setMaxLevel(unsigned char maxLevel)
{
	lock();
	m_maxLevel = maxLevel;
	assert(maxLevel <= m_levels.size());
	unlock();
}

void ccPointCloudLOD::clearData()
{
	setMaxLevel(0);
	
	//1 empty (root) node
	m_levels.resize(1);
	m_levels.front().data.resize(1);
	m_levels.front().data.front() = Node();
}

bool ccPointCloudLOD::initInternal(ccOctree::Shared octree)
{
	if (!octree)
	{
		return false;
	}
	
	//clear the structure (just in case)
	clearData();

	QMutexLocker locker(&m_mutex);

	try
	{
		assert(CCLib::DgmOctree::MAX_OCTREE_LEVEL <= 255);
		m_levels.resize(CCLib::DgmOctree::MAX_OCTREE_LEVEL + 1);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
	
	m_octree = octree;
	return true;
}

int32_t ccPointCloudLOD::newCell(unsigned char level)
{
	assert(level != 0);
	assert(level < m_levels.size());
	Level& l = m_levels[level];

	//assert(l.data.size() < l.data.capacity());
	l.data.push_back(Node(level));

	return static_cast<int32_t>(l.data.size()) - 1;
}

//void ccPointCloudLOD::updateMaxRadii()
//{
//	QMutexLocker locker(&m_mutex);
//
//	for (size_t i = 0; i < m_levels.size(); ++i)
//	{
//		if (!m_levels[i].data.empty())
//		{
//			float maxRadius = 0;
//			for (Node& n : m_levels[i].data)
//			{
//				if (n.radius > maxRadius)
//				{
//					maxRadius = n.radius;
//				}
//			}
//			m_levels[i].maxRadius = m_levels[i].data.front().radius;
//		}
//	}
//}

void ccPointCloudLOD::shrink_to_fit()
{
	QMutexLocker locker(&m_mutex);

	for (size_t i = 1; i < m_levels.size(); ++i) //DGM: always keep the root node!
	{
		if (!m_levels[i].data.empty())
		{
			m_levels[i].data.shrink_to_fit();
		}
		else
		{
			//first empty level: we can reduce the number of levels and stop here!
			m_levels.resize(i);
			break;
		}
	}
	m_levels.shrink_to_fit();
}

void ccPointCloudLOD::clear()
{
	m_mutex.lock();

	if (m_thread)
	{
		delete m_thread;
		m_thread = 0;
	}

	m_levels.clear();
	m_state = NOT_INITIALIZED;

	m_mutex.unlock();
}

unsigned char ccPointCloudLOD::resetVisibility()
{
	unsigned char _maxLevel = m_maxLevel;
	m_currentState = RenderParams();

	for (unsigned char i = 0; i <= _maxLevel; ++i)
	{
		for (Node& n : m_levels[i].data)
		{
			n.displayedPointCount = 0;
			n.intersection = Frustum::INSIDE;
		}
	}

	return _maxLevel;
}

class PointCloudLODVisibilityFlagger
{
public:
	
	PointCloudLODVisibilityFlagger(	ccPointCloudLOD& lod,
									const Frustum& frustum,
									unsigned char maxLevel)
		: m_lod(lod)
		, m_frustum(frustum)
		, m_maxLevel(maxLevel)
	{}

	void propagateFlag(ccPointCloudLOD::Node& node, uint8_t flag)
	{
		node.intersection = flag;

		if (node.childCount)
		{
			for (int i = 0; i < 8; ++i)
			{
				if (node.childIndexes[i] >= 0)
				{
					propagateFlag(m_lod.node(node.childIndexes[i], node.level + 1), flag);
				}
			}
		}
	}

	uint32_t flag(ccPointCloudLOD::Node& node)
	{
		node.intersection = m_frustum.sphereInFrustum(node.center, node.radius);

		uint32_t visibleCount = 0;
		switch (node.intersection)
		{
		case Frustum::INSIDE:
			visibleCount = node.pointCount;
			//no need to propagate the visibility to the children as the default value should already be 'INSIDE'
			break;

		case Frustum::INTERSECT:
			//we have to test the children
			{
				bool hasChildren = false;
				if (node.level < m_maxLevel && node.childCount)
				{
					for (int i = 0; i < 8; ++i)
					{
						if (node.childIndexes[i] >= 0)
						{
							ccPointCloudLOD::Node& childNode = m_lod.node(node.childIndexes[i], node.level + 1);
							visibleCount += flag(childNode);
							hasChildren = true;
						}
					}

					if (visibleCount == 0)
					{
						//as no point is visible we can flag this node as being outside/invisible
						node.intersection = Frustum::OUTSIDE;
					}
				}
				else
				{
					//we have to consider that all points are visible
					visibleCount = node.pointCount;
				}
			}
			break;

		case Frustum::OUTSIDE:
			//be sure that all children nodes are flagged as outside!
			propagateFlag(node, Frustum::OUTSIDE);
			break;
		}

		return visibleCount;
	}

	ccPointCloudLOD& m_lod;
	const Frustum& m_frustum;
	unsigned char m_maxLevel;
};

uint32_t ccPointCloudLOD::flagVisibility(const Frustum& frustum)
{
	unsigned char _maxLevel = resetVisibility();

	m_currentState.visiblePoints = PointCloudLODVisibilityFlagger(*this, frustum, _maxLevel).flag(root());

	return m_currentState.visiblePoints;
}

uint32_t ccPointCloudLOD::addNPointsToIndexMap(Node& node, uint32_t count)
{
	if (!m_lastIndexMap)
	{
		assert(false);
		return 0;
	}
	
	uint32_t displayedCount = 0;
	uint32_t thisNodeRemainingCount = (node.pointCount - node.displayedPointCount);

	if (node.childCount)
	{
		for (int i = 0; i < 8; ++i)
		{
			if (node.childIndexes[i] >= 0)
			{
				ccPointCloudLOD::Node& childNode = this->node(node.childIndexes[i], node.level + 1);
				if (childNode.intersection == Frustum::OUTSIDE)
					continue;
				if (childNode.pointCount == childNode.displayedPointCount)
					continue;

				uint32_t childDisplayedCount = 0;
				if (thisNodeRemainingCount < count)
				{
					childDisplayedCount = addNPointsToIndexMap(childNode, count);
					assert(childDisplayedCount < count);
				}
				else
				{
					uint32_t childNodeRemainingCount = (childNode.pointCount - childNode.displayedPointCount);
					double ratio = static_cast<double>(childNodeRemainingCount) / thisNodeRemainingCount;
					unsigned childMaxCount = static_cast<unsigned>(floor(ratio * count + 0.5));
					if (displayedCount + childMaxCount > count)
					{
						assert(count >= displayedCount);
						childMaxCount = count - displayedCount;
					}
					childDisplayedCount = addNPointsToIndexMap(childNode, childMaxCount);
					assert(childDisplayedCount <= childMaxCount);
				}

				displayedCount += childDisplayedCount;
				assert(displayedCount <= count);
			}
		}
	}
	else
	{
		//we can display the points
		uint32_t iStart = node.displayedPointCount;
		uint32_t iStop = std::min(node.displayedPointCount + count, node.pointCount);

		const ccOctree::cellsContainer& cellCodes = m_octree->pointsAndTheirCellCodes();
		for (uint32_t i = iStart; i < iStop; ++i)
		{
			unsigned pointIndex = cellCodes[node.firstCodeIndex + i].theIndex;
			m_lastIndexMap->addElement(pointIndex);
		}

		displayedCount = iStop - iStart;
	}

	node.displayedPointCount += displayedCount;

	return displayedCount;
}

LODIndexSet* ccPointCloudLOD::getIndexMap(unsigned char level, unsigned& maxCount, unsigned& remainingPointsAtThisLevel)
{
	remainingPointsAtThisLevel = 0;
	
	if (!m_octree)
	{
		assert(false);
		maxCount = 0;
		return 0;
	}

	unsigned char _maxLevel = maxLevel();

	if (m_currentState.displayedPoints >= m_currentState.visiblePoints)
	{
		//assert(false);
		maxCount = 0;
		return 0;
	}
	unsigned totalRemainingCount = m_currentState.visiblePoints - m_currentState.displayedPoints;

	if (totalRemainingCount == 0)
	{
		//nothing to do
		maxCount = 0;
		return 0;
	}

	if (level > _maxLevel)
	{
		assert(false);
		return 0;
	}

	bool priorityToLeafNodes = false;
	bool onlyLeafNodes = false;
	if (m_currentState.unfinishedLevel == level)
	{
		if (m_currentState.unfinishedPoints > maxCount)
		{
			totalRemainingCount = m_currentState.unfinishedPoints;
			onlyLeafNodes = true;
		}
		else
		{
			priorityToLeafNodes = true;
		}
	}

	bool displayAll = false;
	if (maxCount > totalRemainingCount)
	{
		maxCount = totalRemainingCount;
		displayAll = true;
	}

	if (!m_lastIndexMap || m_lastIndexMap->currentSize() < maxCount)
	{
		if (m_lastIndexMap)
		{
			m_lastIndexMap->release();
		}
		m_lastIndexMap = new LODIndexSet;
		if (!m_lastIndexMap->resize(maxCount, 0))
		{
			//not enough memory
			m_lastIndexMap->release();
			return 0;
		}
	}
	m_lastIndexMap->setCurrentSize(0);

	const ccOctree::cellsContainer& cellCodes = m_octree->pointsAndTheirCellCodes();

	//for all cells of the input level
	uint32_t thisPassDisplayCount = 0;
	for (Node& node : m_levels[level].data)
	{
		assert(node.intersection != UNDEFINED);
		if (node.intersection == Frustum::OUTSIDE)
			continue;
		if (node.pointCount == node.displayedPointCount)
			continue;

		unsigned nodeMaxCount = 0;
		if (displayAll)
		{
			nodeMaxCount = node.pointCount;
		}
		else
		{
			if (priorityToLeafNodes)
			{
				//we draw all the points of this (leaf) node
				nodeMaxCount = node.pointCount;
				if (m_lastIndexMap->currentSize() + nodeMaxCount > maxCount)
				{
					assert(maxCount >= m_lastIndexMap->currentSize());
					nodeMaxCount = maxCount - m_lastIndexMap->currentSize();
				}
			}
			else if (!onlyLeafNodes || node.childCount == 0)
			{
				unsigned nodeRemainingCount = (node.pointCount - node.displayedPointCount);
				double ratio = static_cast<double>(nodeRemainingCount) / totalRemainingCount;
				nodeMaxCount = static_cast<unsigned>(floor(ratio * maxCount + 0.5));
				//mainly for safety
				if (m_lastIndexMap->currentSize() + nodeMaxCount > maxCount)
				{
					assert(maxCount >= m_lastIndexMap->currentSize());
					nodeMaxCount = maxCount - m_lastIndexMap->currentSize();
				}
			}
			else
			{
				continue;
			}
		}

		uint32_t nodeDisplayCount = addNPointsToIndexMap(node, nodeMaxCount);
		assert(nodeDisplayCount <= nodeMaxCount);
		thisPassDisplayCount += nodeDisplayCount;

		if (node.childCount == 0)
		{
			remainingPointsAtThisLevel += (node.pointCount - node.displayedPointCount);
		}

		if (thisPassDisplayCount == maxCount)
		{
			break;
		}
	}

	assert(thisPassDisplayCount == m_lastIndexMap->currentSize());

	maxCount = thisPassDisplayCount;
	m_currentState.displayedPoints += thisPassDisplayCount;

	if (remainingPointsAtThisLevel)
	{
		m_currentState.unfinishedLevel = static_cast<int>(level);
		m_currentState.unfinishedPoints = remainingPointsAtThisLevel;
	}
	else
	{
		m_currentState.unfinishedLevel = -1;
		m_currentState.unfinishedPoints = 0;
	}
	
	return m_lastIndexMap;
}

#else

//Qt
#include <QAtomicInt>

//! Thread for background computation
class ccPointCloudLODThread : public QThread
{
public:
	
	//! Default constructor
	ccPointCloudLODThread(ccPointCloud& cloud, ccPointCloudLOD& lod)
		: QThread()
		, m_cloud(cloud)
		, m_lod(lod)
		, m_abort(0)
	{}
	
	//!Destructor
	virtual ~ccPointCloudLODThread()
	{
		stop();
	}
	
	//! Stops the thread
	void stop(unsigned long timeout = ULONG_MAX)
	{
		m_abort = 1;
		wait(timeout);
	}

protected:

	//reimplemented from QThread
	virtual void run()
	{
		m_abort = 0;

		//reset structure
		m_lod.clearExtended(false, ccPointCloudLOD::UNDER_CONSTRUCTION);

		unsigned pointCount = m_cloud.size();
		if (pointCount == 0)
		{
			m_lod.setState(ccPointCloudLOD::BROKEN);
			return;
		}

		ccLog::Print(QString("[LoD] Preparing LoD acceleration structure for cloud '%1' [%2 points]...").arg(m_cloud.getName()).arg(pointCount));
		QElapsedTimer timer;
		timer.start();

		//first we need an octree
		//DGM: we don't use the cloud's octree (if any)
		//as we don't know when it will be deleted!
		//(the user can do it anytime for instance)
		ccOctree::Shared octree = m_cloud.getOctree();
		if (!octree)
		{
			octree = ccOctree::Shared(new ccOctree(&m_cloud));
			if (octree->build(0/*progressCallback*/) <= 0)
			{
				//not enough memory
				ccLog::Warning(QString("[LoD] Failed to compute octree on cloud '%1' (not enough memory)").arg(m_cloud.getName()));
				m_lod.setState(ccPointCloudLOD::BROKEN);
				return;
			}

			if (!m_cloud.getOctree())
			{
				m_cloud.setOctree(octree);
			}
		}

		//flags
		GenericChunkedArray<1, unsigned char>* flags = new GenericChunkedArray<1, unsigned char>();
		static const unsigned char NoFlag = 0;
		if (!flags->resize(pointCount, true, NoFlag))
		{
			//not enough memory
			ccLog::Warning(QString("[LoD] Failed to compute LOD structure on cloud '%1' (not enough memory)").arg(m_cloud.getName()));
			m_lod.setState(ccPointCloudLOD::BROKEN);
			flags->release();
			return;
		}

		//init LoD indexes and level descriptors
		if (!m_lod.reserve(	pointCount,
							CCLib::DgmOctree::MAX_OCTREE_LEVEL + 1) ) //level 0 included
		{
			//not enough memory
			ccLog::Warning(QString("[LoD] Failed to compute LOD structure on cloud '%1' (not enough memory)").arg(m_cloud.getName()));
			m_lod.setState(ccPointCloudLOD::BROKEN);
			flags->release();
			return;
		}

		//if (progressCallback)
		//{
		//	progressCallback->setMethodTitle(QObject::tr("L.O.D. display"));
		//	progressCallback->setInfo(QObject::tr("Preparing L.O.D. structure to speed-up the display..."));
		//	progressCallback->start();
		//}
		//CCLib::NormalizedProgress nProgress(progressCallback,pointCount);

		assert(CCLib::DgmOctree::MAX_OCTREE_LEVEL <= 255);
		//first level (default)
		m_lod.addLevel(ccPointCloudLOD::LODLevelDesc(0, 0));

		unsigned remainingCount = pointCount;

		//and the next ones
		for (unsigned char level=1; level<=static_cast<unsigned char>(CCLib::DgmOctree::MAX_OCTREE_LEVEL); ++level)
		{
			//current level descriptor
			ccPointCloudLOD::LODLevelDesc levelDesc;
			ccPointCloudLOD::IndexSet* indexes = m_lod.indexes();
			assert(indexes);
			levelDesc.startIndex = indexes->currentSize();

			//no need to process the points if there are less points remaining than the previous level
			if (remainingCount > m_lod.level(level - 1).count)
			{
				const unsigned char bitDec = CCLib::DgmOctree::GET_BIT_SHIFT(level);

				//for each cell we'll look for the (not-yet-flagged) point which is closest to the cell center
				static const unsigned INVALID_INDEX = (~static_cast<unsigned>(0));
				unsigned nearestIndex = INVALID_INDEX;
				PointCoordinateType nearestSquareDist = 0;
				CCVector3 cellCenter(0, 0, 0);

				CCLib::DgmOctree::CellCode currentTruncatedCellCode = (~static_cast<CCLib::DgmOctree::CellCode>(0));

				//scan the octree structure
				const CCLib::DgmOctree::cellsContainer& thePointsAndTheirCellCodes = octree->pointsAndTheirCellCodes();
				for (CCLib::DgmOctree::cellsContainer::const_iterator c = thePointsAndTheirCellCodes.begin(); c != thePointsAndTheirCellCodes.end(); ++c)
				{
					if (flags->getValue(c->theIndex) != NoFlag)
					{
						//we can skip already flagged points!
						continue;
					}
					CCLib::DgmOctree::CellCode truncatedCode = (c->theCode >> bitDec);

					//new cell?
					if (truncatedCode != currentTruncatedCellCode)
					{
						//process the previous cell
						if (nearestIndex != INVALID_INDEX)
						{
							indexes->addElement(nearestIndex);
							assert(flags->getValue(nearestIndex) == NoFlag);
							flags->setValue(nearestIndex, level);
							levelDesc.count++;
							assert(remainingCount);
							--remainingCount;
							//nProgress.oneStep();
							if (m_abort.load() == 1)
								break;
							nearestIndex = INVALID_INDEX;
						}

						//prepare new cell
						currentTruncatedCellCode = truncatedCode;
						octree->computeCellCenter(currentTruncatedCellCode, level, cellCenter, true);
					}

					//compute distance to the cell center
					const CCVector3* P = m_cloud.getPoint(c->theIndex);
					PointCoordinateType squareDist = (*P - cellCenter).norm2();
					if (nearestIndex == INVALID_INDEX || squareDist < nearestSquareDist)
					{
						nearestSquareDist = squareDist;
						nearestIndex = c->theIndex;
					}
				}

				if (m_abort.load() == 1)
				{
					break;
				}

				//don't forget the last cell!
				if (nearestIndex != INVALID_INDEX)
				{
					indexes->addElement(nearestIndex);
					assert(flags->getValue(nearestIndex) == NoFlag);
					flags->setValue(nearestIndex, level);
					levelDesc.count++;
					assert(remainingCount);
					--remainingCount;
					//nProgress.oneStep();
					nearestIndex = INVALID_INDEX;
				}
			}

			if (remainingCount)
			{
				//no new point was flagged during this round? Then we have reached the maximum level
				if (levelDesc.count == 0 || level == CCLib::DgmOctree::MAX_OCTREE_LEVEL)
				{
					//assert(indexes->currentSize() == levelDesc.startIndex);

					//flag the remaining points with the current level
					for (unsigned i=0; i<pointCount; ++i)
					{
						if (flags->getValue(i) == NoFlag)
						{
							indexes->addElement(i);
							levelDesc.count++;
							assert(remainingCount);
							--remainingCount;
							//nProgress.oneStep();
						}
					}
				}
			}

			if (levelDesc.count)
			{
				m_lod.addLevel(levelDesc);
				ccLog::PrintDebug(QString("[LoD] Cloud %1 - level %2: %3 points").arg(m_cloud.getName()).arg(level).arg(levelDesc.count));
			}
			else
			{
				assert(remainingCount == 0);
			}

			if (indexes->currentSize() == pointCount)
			{
				//all points have been processed? We can stop right now
				break;
			}
		}

		if (m_abort.load() == 0)
		{
			assert(m_lod.indexes() && m_lod.indexes()->currentSize() == pointCount);
			m_lod.shrink();
			m_lod.setState(ccPointCloudLOD::INITIALIZED);
		}
		else
		{
			//reset
			m_lod.clearExtended(false, ccPointCloudLOD::NOT_INITIALIZED);
		}

		flags->release();
		flags = 0;

		ccLog::Print(QString("[LoD] Acceleration structure ready for cloud '%1' (max level: %2 / duration: %3 s.)").arg(m_cloud.getName()).arg(static_cast<int>(m_lod.maxLevel())-1).arg(timer.elapsed() / 1000.0,0,'f',1));
	}

	ccPointCloud& m_cloud;
	ccPointCloudLOD& m_lod;
	
	QAtomicInt	m_abort;
};

bool ccPointCloudLOD::init(ccPointCloud* cloud)
{
	if (!cloud)
	{
		assert(false);
		return false;
	}
	
	if (isBroken())
	{
		return false;
	}

	if (!m_thread)
	{
		m_thread = new ccPointCloudLODThread(*cloud, *this);
	}
	else if (m_thread->isRunning())
	{
		//already running?
		assert(false);
		return true;
	}

	m_thread->start();
	return true;
}

bool ccPointCloudLOD::reserve(unsigned pointCount, int levelCount)
{
	m_mutex.lock();
	//init the levels descriptors
	try
	{
		m_levels.reserve(levelCount);
	}
	catch (const std::bad_alloc&)
	{
		m_mutex.unlock();
		return false;
	}

	//init LoD indexes
	if (!m_indexes)
	{
		m_indexes = new IndexSet;
	}
	if (!m_indexes->reserve(pointCount))
	{
		m_indexes->release();
	}
	m_mutex.unlock();

	return m_indexes != 0;
}

void ccPointCloudLOD::clearExtended(bool autoStopThread, State newState)
{
	if (autoStopThread && m_thread)
	{
		m_thread->stop();
		delete m_thread;
		m_thread = 0;
	}

	m_mutex.lock();
	m_levels.clear();
	if (m_indexes)
	{
		m_indexes->release();
		m_indexes = 0;
	}
	m_state = newState;
	m_mutex.unlock();
}

#endif //USE_LOD_2
