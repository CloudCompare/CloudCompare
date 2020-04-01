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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#include "ccPointCloudLOD.h"

//Local
#include "ccPointCloud.h"

//Qt
#include <QThread>
#include <QElapsedTimer>

//! Thread for background computation
class ccPointCloudLODThread : public QThread
{
	Q_OBJECT
	
public:
	
	//! Default constructor
	ccPointCloudLODThread(ccPointCloud& cloud, ccPointCloudLOD& lod, uint32_t maxCountPerCell)
		: QThread()
		, m_cloud(cloud)
		, m_lod(lod)
		, m_octree(nullptr)
		, m_maxCountPerCell(maxCountPerCell)
		, m_maxLevel(0)
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
#ifdef COMPUTE_REAL_RADIUS
			CCVector3d sumP(0, 0, 0);
#else //otherwise we use the bounding box
			ccBBox bbox;
#endif
			for (uint32_t codeIndex = node.firstCodeIndex; codeIndex < cellCodes.size() && (cellCodes[codeIndex].theCode >> bitDec) == currentTruncatedCellCode; ++codeIndex)
			{
				++node.pointCount;
				const CCVector3* P = m_cloud.getPoint(cellCodes[codeIndex].theIndex);
#ifdef COMPUTE_REAL_RADIUS
				sumP += CCVector3d::fromArray(P->u);
#else
				bbox.add(*P);
#endif
			}

			//compute the radius
#ifdef COMPUTE_REAL_RADIUS
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
#else
			if (node.pointCount > 1)
			{
				node.radius = static_cast<float>(bbox.getDiagNormd());
			}
			node.center = CCVector3f::fromArray(bbox.getCenter().u);
#endif
		}

		//do we need to subdivide this cell?
		if (node.pointCount > m_maxCountPerCell && node.level+1 <= m_maxLevel)
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
			if (m_octree->build(nullptr/*progressCallback*/) <= 0)
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

		//make sure we deprecate the LOD structure when this octree is modified!
		QObject::connect(m_octree.data(), &ccOctree::updated, this, [&](){ m_cloud.clearLOD(); });

		m_maxLevel = static_cast<uint8_t>(std::max<size_t>(1, m_lod.m_levels.size())) - 1;
		assert(m_maxLevel <= CCLib::DgmOctree::MAX_OCTREE_LEVEL);

#if 0 //recursive path
		//recursive
		fillNode(m_lod.root());

		m_lod.shrink_to_fit();
		//m_lod.updateMaxRadii();
		//m_lod.setMaxLevel(m_maxLevel);

		for (size_t i = 1; i < m_lod.m_levels.size(); ++i)
		{
			ccLog::Print(QString("[LoD] Level %1: %2 cells").arg(i).arg(m_lod.m_levels[i].data.size()));
		}

#else //layer by layer

		//init with root node
		fillNode_flat(m_lod.root());

		//first we allow the division of nodes as deep as possible but with a minimum number of points per cell
		for (uint8_t currentLevel = 0; currentLevel < m_maxLevel; ++currentLevel)
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
			ccLog::Print(QString("[LoD] Level %1: %2 cells").arg(currentLevel).arg(level.data.size()));

			//now we can create the next level
			if (currentLevel + 1 < m_maxLevel)
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
		m_maxLevel = static_cast<uint8_t>(std::max<size_t>(1, m_lod.m_levels.size())) - 1;

		//refinement step
		if (true)
		{
			//we look at the 'main' depth level (with the most point)
			uint8_t biggestLevel = 0;
			for (uint8_t i = 1; i <= m_maxLevel; ++i)
			{
				if (m_lod.m_levels[i].data.size() > m_lod.m_levels[biggestLevel].data.size())
				{
					biggestLevel = i;
				}
			}

			//now compute the mean radius for this level
			//double meanRadius = 0;
			//{
			//	const ccPointCloudLOD::Level& level = m_lod.m_levels[biggestLevel];
			//	size_t cellCount = level.data.size();
			//	for (size_t i = 0; i < cellCount; ++i)
			//	{
			//		meanRadius += level.data[i].radius;
			//	}

			//	meanRadius /= cellCount;
			//}

			//and divide again the cells (with a lower limit on the number of points)
			biggestLevel = std::min<uint8_t>(biggestLevel, 10);
			for (uint8_t currentLevel = 0; currentLevel < biggestLevel; ++currentLevel)
			{
				ccPointCloudLOD::Level& level = m_lod.m_levels[currentLevel];
				assert(!level.data.empty());

				size_t cellCountBefore = m_lod.m_levels[currentLevel+1].data.size();
				for (ccPointCloudLOD::Node& node : level.data)
				{
					//do we need to subdivide this cell?
					if (node.childCount == 0 && node.pointCount > 16)
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

				size_t cellCountAfter = m_lod.m_levels[currentLevel+1].data.size();
				ccLog::Print(QString("[LoD][pass 2] Level %1: %2 cells (+%3)").arg(currentLevel+1).arg(cellCountAfter).arg(cellCountAfter - cellCountBefore));
			}

			m_lod.shrink_to_fit();
			m_maxLevel = static_cast<uint8_t>(std::max<size_t>(1, m_lod.m_levels.size()))-1;
		}
#endif

		m_lod.setState(ccPointCloudLOD::INITIALIZED);

		ccLog::Print(QString("[LoD] Acceleration structure ready for cloud '%1' (max level: %2 / mem. = %3 Mb / duration: %4 s.)")
			.arg(m_cloud.getName())
			.arg(m_maxLevel)
			.arg(m_lod.memory() / static_cast<double>(1 << 20), 0, 'f', 2)
			.arg(timer.elapsed() / 1000.0, 0, 'f', 1));
	}

	ccPointCloud& m_cloud;
	ccPointCloudLOD& m_lod;
	ccOctree::Shared m_octree;
	uint32_t m_maxCountPerCell;
	uint8_t m_maxLevel;
};

ccPointCloudLOD::ccPointCloudLOD()
	: m_indexMap(0)
	, m_lastIndexMap(0)
	, m_octree(nullptr)
	, m_thread(nullptr)
	, m_state(NOT_INITIALIZED)
{
	clearData(); //initializes the root node
}

ccPointCloudLOD::~ccPointCloudLOD()
{
	clear();
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

void ccPointCloudLOD::clearData()
{
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
	l.data.emplace_back(level);

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
	if (m_thread && m_thread->isRunning())
	{
		m_thread->terminate();
		m_thread->wait();
	}
	
	m_mutex.lock();

	if (m_thread)
	{
		delete m_thread;
		m_thread = nullptr;
	}

	m_levels.clear();
	m_state = NOT_INITIALIZED;

	m_mutex.unlock();
}

void ccPointCloudLOD::resetVisibility()
{
	if (m_state != INITIALIZED)
	{
		return;
	}

	m_currentState = RenderParams();

	for (size_t l = 0; l < m_levels.size(); ++l)
	{
		for (Node& n : m_levels[l].data)
		{
			n.displayedPointCount = 0;
			n.intersection = Frustum::INSIDE;
		}
	}
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
		, m_hasClipPlanes(false)
	{}

	void setClipPlanes(const ccClipPlaneSet& clipPlanes)
	{
		try
		{
			m_clipPlanes = clipPlanes;
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			m_hasClipPlanes = false;
		}
		m_hasClipPlanes = !m_clipPlanes.empty();
	}

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
		if (m_hasClipPlanes && node.intersection != Frustum::OUTSIDE)
		{
			for (size_t i = 0; i < m_clipPlanes.size(); ++i)
			{
				//distance from center to clip plane
				//we assume the plane normal (= 3 first coefficients) is normalized!
				const Tuple4Tpl<double>& eq = m_clipPlanes[i].equation;
				double dist = eq.x * node.center.x + eq.y * node.center.y + eq.z * node.center.z + eq.w /* / CCVector3d::vnorm(eq.u) */;

				if (dist < node.radius)
				{
					if (dist <= -node.radius)
					{
						node.intersection = Frustum::OUTSIDE;
						break;
					}
					else
					{
						node.intersection = Frustum::INTERSECT;
					}
				}
			}
		}

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
				if (node.level < m_maxLevel && node.childCount)
				{
					for (int i = 0; i < 8; ++i)
					{
						if (node.childIndexes[i] >= 0)
						{
							ccPointCloudLOD::Node& childNode = m_lod.node(node.childIndexes[i], node.level + 1);
							visibleCount += flag(childNode);
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
	ccClipPlaneSet m_clipPlanes;
	bool m_hasClipPlanes;
};

uint32_t ccPointCloudLOD::flagVisibility(const Frustum& frustum, ccClipPlaneSet* clipPlanes/*=0*/)
{
	if (m_state != INITIALIZED)
	{
		assert(false);
		m_currentState = RenderParams();
		return 0;
	}

	resetVisibility();

	PointCloudLODVisibilityFlagger lodVisibility(*this, frustum, static_cast<unsigned char>(m_levels.size()));
	if (clipPlanes)
	{
		lodVisibility.setClipPlanes(*clipPlanes);
	}

	m_currentState.visiblePoints = lodVisibility.flag(root());

	return m_currentState.visiblePoints;
}

uint32_t ccPointCloudLOD::addNPointsToIndexMap(Node& node, uint32_t count)
{
	if (m_indexMap.capacity() == 0)
	{
		assert(false);
		return 0;
	}
	
	uint32_t displayedCount = 0;

	if (node.childCount)
	{
		uint32_t thisNodeRemainingCount = (node.pointCount - node.displayedPointCount);
		assert(count <= thisNodeRemainingCount);
		bool displayAll = (count >= thisNodeRemainingCount);
		
		for (int i = 0; i < 8; ++i)
		{
			if (node.childIndexes[i] >= 0)
			{
				ccPointCloudLOD::Node& childNode = this->node(node.childIndexes[i], node.level + 1);
				if (childNode.intersection == Frustum::OUTSIDE)
					continue;
				if (childNode.pointCount == childNode.displayedPointCount)
					continue;
				uint32_t childNodeRemainingCount = (childNode.pointCount - childNode.displayedPointCount);

				uint32_t childMaxCount = 0;
				if (displayAll)
				{
					childMaxCount = childNodeRemainingCount;
				}
				else
				{
					double ratio = static_cast<double>(childNodeRemainingCount) / thisNodeRemainingCount;
					childMaxCount = static_cast<uint32_t>(ceil(ratio * count));
					if (displayedCount + childMaxCount > count)
					{
						assert(count >= displayedCount);
						childMaxCount = count - displayedCount;
						i = 8; //we can stop right now
					}
				}
				
				uint32_t childDisplayedCount = addNPointsToIndexMap(childNode, childMaxCount);
				//assert(childDisplayedCount == childMaxCount || !displayAll || childNode.intersection != Frustum::INSIDE);
				assert(childDisplayedCount <= childMaxCount);

				displayedCount += childDisplayedCount;
				assert(displayedCount <= count);
			}
		}
	}
	else
	{
		//we can display all the points
		//uint32_t iStart = node.displayedPointCount;
		uint32_t iStop = std::min(node.displayedPointCount + count, node.pointCount);

		displayedCount = iStop - node.displayedPointCount;
		assert(m_indexMap.size() + displayedCount <= m_indexMap.capacity());

		const ccOctree::cellsContainer& cellCodes = m_octree->pointsAndTheirCellCodes();
		for (uint32_t i = node.displayedPointCount; i < iStop; ++i)
		{
			unsigned pointIndex = cellCodes[node.firstCodeIndex + i].theIndex;
			m_indexMap.push_back(pointIndex);
		}
	}

	node.displayedPointCount += displayedCount;

	return displayedCount;
}

LODIndexSet& ccPointCloudLOD::getIndexMap(unsigned char level, unsigned& maxCount, unsigned& remainingPointsAtThisLevel)
{
	remainingPointsAtThisLevel = 0;
	m_lastIndexMap.clear();

	if (!m_octree || level >= m_levels.size())
	{
		assert(false);
		maxCount = 0;
		return m_lastIndexMap; //empty
	}

	if (m_state != INITIALIZED)
	{
		maxCount = 0;
		return m_lastIndexMap; //empty
	}

	if (m_currentState.displayedPoints >= m_currentState.visiblePoints)
	{
		//assert(false);
		maxCount = 0;
		return m_lastIndexMap; //empty
	}

	m_indexMap.clear();
	try
	{
		m_indexMap.reserve(maxCount);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return m_lastIndexMap; //empty
	}

	Level& l = m_levels[level];
	uint32_t thisPassDisplayCount = 0;

	bool earlyStop = false;
	size_t earlyStopIndex = 0;

	//special case: we have to finish/continue at the same level than the previous run
	if (m_currentState.unfinishedLevel == level)
	{
		bool displayAll = (m_currentState.unfinishedPoints <= maxCount);

		//display all leaf cells of the current level
		for (size_t i = 0; i < l.data.size(); ++i)
		{
			Node& node = l.data[i];

			if (node.childCount) //skip non leaf cells
				continue;
			assert(node.intersection != UNDEFINED);
			if (node.intersection == Frustum::OUTSIDE)
				continue;
			if (node.pointCount == node.displayedPointCount)
				continue;

			uint32_t nodeMaxCount = 0;
			uint32_t nodeRemainingCount = (node.pointCount - node.displayedPointCount);
			if (displayAll)
			{
				nodeMaxCount = nodeRemainingCount;
			}
			else
			{
				double ratio = static_cast<double>(nodeRemainingCount) / m_currentState.unfinishedPoints;
				nodeMaxCount = static_cast<uint32_t>(ceil(ratio * maxCount));
				//safety check
				if (m_indexMap.size() + nodeMaxCount >= maxCount)
				{
					assert(maxCount >= m_indexMap.size());
					nodeMaxCount = maxCount - static_cast<uint32_t>(m_indexMap.size());

					earlyStop = true;
					earlyStopIndex = i;

					i = l.data.size(); //we can stop after this node!
				}
			}

			uint32_t nodeDisplayCount = addNPointsToIndexMap(node, nodeMaxCount);
			assert(nodeDisplayCount <= nodeMaxCount);
			
			thisPassDisplayCount += nodeDisplayCount;
			assert(thisPassDisplayCount == m_indexMap.size());
			remainingPointsAtThisLevel += (node.pointCount - node.displayedPointCount);
		}
	}
	
	uint32_t totalRemainingCount = m_currentState.visiblePoints - m_currentState.displayedPoints;
	//remove the already displayed points (= unfinished from previous run)
	assert(totalRemainingCount >= thisPassDisplayCount);
	totalRemainingCount -= thisPassDisplayCount;
	
	//do we still have data to display AND can we display it?
	if (totalRemainingCount != 0 && thisPassDisplayCount < maxCount)
	{
		//we shouldn't have any unfinished work at this point!
		assert(!earlyStop && remainingPointsAtThisLevel == 0);

		uint32_t mapFreeSize = maxCount - thisPassDisplayCount;

		bool displayAll = (mapFreeSize > totalRemainingCount);

		//for all cells of the input level
		for (size_t i = 0; i < l.data.size(); ++i)
		{
			Node& node = l.data[i];

			assert(node.intersection != UNDEFINED);
			if (node.intersection == Frustum::OUTSIDE)
				continue;
			if (node.pointCount == node.displayedPointCount)
				continue;

			uint32_t nodeMaxCount = 0;
			uint32_t nodeRemainingCount = (node.pointCount - node.displayedPointCount);
			if (displayAll)
			{
				nodeMaxCount = nodeRemainingCount;
			}
			else if (node.childCount)
			{
				double ratio = static_cast<double>(nodeRemainingCount) / totalRemainingCount;
				nodeMaxCount = static_cast<uint32_t>(ceil(ratio * mapFreeSize));
				//safety check
				if (m_indexMap.size() + nodeMaxCount >= maxCount)
				{
					assert(maxCount >= m_indexMap.size());
					nodeMaxCount = maxCount - static_cast<uint32_t>(m_indexMap.size());

					earlyStop = true;
					earlyStopIndex = i;

					i = l.data.size(); //we can stop after this node!
				}
			}

			uint32_t nodeDisplayCount = addNPointsToIndexMap(node, nodeMaxCount);
			assert(nodeDisplayCount <= nodeMaxCount);

			thisPassDisplayCount += nodeDisplayCount;
			assert(thisPassDisplayCount == m_indexMap.size());

			if (node.childCount == 0)
			{
				remainingPointsAtThisLevel += (node.pointCount - node.displayedPointCount);
			}
		}
	}

	maxCount = static_cast<unsigned>(m_indexMap.size());
	m_currentState.displayedPoints += static_cast<uint32_t>(m_indexMap.size());

	if (earlyStop)
	{
		//be sure to properly finish to count the number of 'unfinished' points!
		for (size_t i = earlyStopIndex+1; i < l.data.size(); ++i)
		{
			Node& node = l.data[i];

			if (node.childCount) //skip non leaf nodes
				continue;
			assert(node.intersection != UNDEFINED);
			if (node.intersection == Frustum::OUTSIDE)
				continue;
			if (node.pointCount == node.displayedPointCount)
				continue;
			uint32_t nodeRemainingCount = (node.pointCount - node.displayedPointCount);
			remainingPointsAtThisLevel += nodeRemainingCount;
		}
	}

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
	
	m_lastIndexMap = m_indexMap;
	return m_indexMap;
}

#include "ccPointCloudLOD.moc"
