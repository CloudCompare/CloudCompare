// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #                    COPYRIGHT: CloudCompare project                     #
// #                                                                        #
// ##########################################################################

#include "ccPointCloudLOD.h"

// Local
#include "ccFrustum.h"
#include "ccGenericGLDisplay.h"
#include "ccGenericMesh.h"
#include "ccLog.h"
#include "ccPointCloud.h"
#include "ccVBOManager.h"

// Qt
#include <QElapsedTimer>
#include <QThread>

// System
#include <algorithm>
#include <cstdint>
#include <vector>

//! Thread for background computation
class ccPointCloudLODThread : public QThread
{
	Q_OBJECT

  public:
	//! Default constructor
	ccPointCloudLODThread(ccPointCloud& cloud, ccInternalPointCloudLOD& lod, uint32_t maxCountPerCell)
	    : QThread()
	    , m_cloud(cloud)
	    , m_lod(lod)
	    , m_octree(nullptr)
	    , m_maxCountPerCell(maxCountPerCell)
	    , m_maxLevel(0)
	    , m_earlyStop(0)
	{
	}

	//! Destructor
	~ccPointCloudLODThread() override
	{
		if (isRunning())
		{
			ccLog::Warning("[ccPointCloudLODThread] Destructor called when the thread is still running: will have to terminate it...");
			terminate();
		}
	}

	//! Stops the thread
	void stop()
	{
		if (m_octree && m_octree->isBuildInProgress())
		{
			m_octree->cancelBuild();
		}

		m_earlyStop = 1;
		if (!wait(5000))
		{
			ccLog::Warning("[ccPointCloudLODThread] Failed to stop the thread properly, will have to terminate it...");
			terminate();
		}

		m_octree.clear();
		m_earlyStop = 0;
	}

  protected:
	//! Fills a node (and returns its relative position) - no recurrence
	uint8_t fillNode_flat(ccAbstractPointCloudLOD::Node& node) const
	{
		assert(m_octree);

		const ccOctree::cellsContainer&      cellCodes                = m_octree->pointsAndTheirCellCodes();
		const unsigned char                  bitDec                   = CCCoreLib::DgmOctree::GET_BIT_SHIFT(node.level);
		const CCCoreLib::DgmOctree::CellCode currentTruncatedCellCode = (cellCodes[node.firstCodeIndex].theCode >> bitDec);

		// first count the number of points and compute their center
		{
			node.pointCount = 0;
			CCVector3d sumP(0, 0, 0);
			for (uint32_t codeIndex = node.firstCodeIndex; codeIndex < cellCodes.size() && (cellCodes[codeIndex].theCode >> bitDec) == currentTruncatedCellCode; ++codeIndex)
			{
				++node.pointCount;
				const CCVector3* P = m_cloud.getPoint(cellCodes[codeIndex].theIndex);
				sumP += *P;

				if (m_earlyStop)
				{
					return 0;
				}
			}

			// compute the radius
			if (node.pointCount > 1)
			{
				sumP /= node.pointCount;
				double maxSquareRadius = 0;
				for (uint32_t i = 0; i < node.pointCount; ++i)
				{
					const CCVector3* P            = m_cloud.getPoint(cellCodes[node.firstCodeIndex + i].theIndex);
					double           squareRadius = (P->toDouble() - sumP).norm2();
					if (squareRadius > maxSquareRadius)
					{
						maxSquareRadius = squareRadius;
					}

					if (m_earlyStop)
					{
						return 0;
					}
				}
				node.radius = static_cast<float>(sqrt(maxSquareRadius));
			}

			// update the center
			node.center = sumP.toFloat();
		}

		// return the node relative position
		return static_cast<uint8_t>(currentTruncatedCellCode & 7);
	}

	//! Called by run() before quiting (in case the process has to be aborted)
	void abortConstruction()
	{
		m_lod.setState(ccAbstractPointCloudLOD::BROKEN);
		m_octree.clear();
		m_lod.clearData();
		m_earlyStop = 0;
	}

	// reimplemented from QThread
	void run() override
	{
		m_lod.setState(ccAbstractPointCloudLOD::NOT_INITIALIZED);

		if (m_earlyStop != 0)
		{
			ccLog::Error("[LoD] Thread not properly terminated previously... can't run it again");
			return;
		}

		unsigned pointCount = m_cloud.size();
		if (pointCount == 0)
		{
			abortConstruction();
			return;
		}

		// reset structure
		m_lod.setState(ccAbstractPointCloudLOD::UNDER_CONSTRUCTION);
		m_lod.clearData();

		ccLog::Print(QString("[LoD] Preparing LoD acceleration structure for cloud '%1' [%2 points]...").arg(m_cloud.getName()).arg(pointCount));

		QElapsedTimer timer;
		timer.start();

		// first we need an octree
		m_octree = m_cloud.getOctree();
		if (!m_octree)
		{
			// we have to compute the octree
			m_octree.reset(new ccOctree(&m_cloud));
			if (m_octree->build(nullptr) <= 0)
			{
				if (0 == m_earlyStop)
				{
					// not enough memory
					ccLog::Warning(QString("[LoD] Failed to compute octree on cloud '%1' (not enough memory)").arg(m_cloud.getName()));
					m_lod.setState(ccAbstractPointCloudLOD::BROKEN);
				}
				else
				{
					// we have cancelled the octree computation process (see the stop() method)
				}
				m_earlyStop = 1;
			}

			if (m_earlyStop)
			{
				// abort requested
				abortConstruction();
				return;
			}

			if (!m_cloud.getOctree()) // make sure that it hasn't been built in the meantime!
			{
				m_cloud.setOctree(m_octree);
			}
		}

		// init LoD structure
		if (!m_lod.initInternal(m_octree))
		{
			// not enough memory
			ccLog::Warning(QString("[LoD] Failed to compute LOD structure on cloud '%1' (not enough memory)").arg(m_cloud.getName()));
			m_earlyStop = 1;
		}

		if (m_earlyStop)
		{
			// abort requested
			abortConstruction();
			return;
		}

		// make sure we deprecate the LOD structure when this octree is modified!
		QObject::connect(m_octree.data(), &ccOctree::updated, this, [&]()
		                 { m_cloud.clearLOD(); });

		m_maxLevel = static_cast<uint8_t>(std::max<size_t>(1, m_lod.m_levels.size())) - 1;
		assert(m_maxLevel <= CCCoreLib::DgmOctree::MAX_OCTREE_LEVEL);

		// init with root node
		fillNode_flat(m_lod.root());

		if (m_earlyStop)
		{
			// abort requested
			abortConstruction();
			return;
		}

		// first we allow the division of nodes as deep as possible but with a minimum number of points per cell
		for (uint8_t currentLevel = 0; currentLevel < m_maxLevel; ++currentLevel)
		{
			ccAbstractPointCloudLOD::Level& level = m_lod.m_levels[currentLevel];
			if (level.data.empty())
			{
				break;
			}

			// the previous level is now ready!
			ccLog::Print(QString("[LoD] Level %1: %2 cells").arg(currentLevel).arg(level.data.size()));

			// now we can prepare the next level
			if (currentLevel + 1 < m_maxLevel)
			{
				for (ccAbstractPointCloudLOD::Node& node : level.data)
				{
					// do we need to subdivide this cell?
					if (node.pointCount > m_maxCountPerCell)
					{
						for (uint32_t i = 0; i < node.pointCount;)
						{
							int32_t                        childNodeIndex = m_lod.newCell(node.level + 1);
							ccAbstractPointCloudLOD::Node& childNode      = m_lod.node(childNodeIndex, node.level + 1);
							childNode.firstCodeIndex                      = node.firstCodeIndex + i;

							uint8_t childIndex = fillNode_flat(childNode);
							if (m_earlyStop)
							{
								// abort requested
								abortConstruction();
								return;
							}

							node.childIndexes[childIndex] = childNodeIndex;
							node.childCount++;
							i += childNode.pointCount;
						}
					}
				}
			}
		}

		if (m_earlyStop)
		{
			// abort requested
			abortConstruction();
			return;
		}

		m_lod.shrink_to_fit();
		m_maxLevel = static_cast<uint8_t>(std::max<size_t>(1, m_lod.m_levels.size())) - 1;

		// refinement step
		{
			// we look at the 'main' depth level (with the most points)
			uint8_t biggestLevel = 0;
			for (uint8_t i = 1; i <= m_maxLevel; ++i)
			{
				if (m_lod.m_levels[i].data.size() > m_lod.m_levels[biggestLevel].data.size())
				{
					biggestLevel = i;
				}
			}

			// divide again the cells (with a lower limit on the number of points)
			biggestLevel = std::min<uint8_t>(biggestLevel, 10);
			for (uint8_t currentLevel = 0; currentLevel < biggestLevel; ++currentLevel)
			{
				ccAbstractPointCloudLOD::Level& level = m_lod.m_levels[currentLevel];
				assert(!level.data.empty());

				size_t cellCountBefore = m_lod.m_levels[currentLevel + 1].data.size();
				for (ccAbstractPointCloudLOD::Node& node : level.data)
				{
					// do we need to subdivide this cell?
					if (node.childCount == 0 && node.pointCount > 16)
					{
						for (uint32_t i = 0; i < node.pointCount;)
						{
							int32_t                        childNodeIndex = m_lod.newCell(node.level + 1);
							ccAbstractPointCloudLOD::Node& childNode      = m_lod.node(childNodeIndex, node.level + 1);
							childNode.firstCodeIndex                      = node.firstCodeIndex + i;

							uint8_t childIndex = fillNode_flat(childNode);
							if (m_earlyStop)
							{
								// abort requested
								abortConstruction();
								return;
							}

							node.childIndexes[childIndex] = childNodeIndex;
							node.childCount++;
							i += childNode.pointCount;
						}
					}
				}

				size_t cellCountAfter = m_lod.m_levels[currentLevel + 1].data.size();
				ccLog::Print(QString("[LoD][pass 2] Level %1: %2 cells (+%3)").arg(currentLevel + 1).arg(cellCountAfter).arg(cellCountAfter - cellCountBefore));

				if (m_earlyStop)
				{
					// abort requested
					abortConstruction();
					return;
				}
			}

			m_lod.shrink_to_fit();
			m_maxLevel = static_cast<uint8_t>(std::max<size_t>(1, m_lod.m_levels.size())) - 1;
		}

		m_lod.setState(ccAbstractPointCloudLOD::INITIALIZED);

		ccLog::Print(QString("[LoD] Acceleration structure ready for cloud '%1' (max level: %2 / mem. = %3 Mb / duration: %4 s.)")
		                 .arg(m_cloud.getName())
		                 .arg(m_maxLevel)
		                 .arg(m_lod.memory() / static_cast<double>(1 << 20), 0, 'f', 2)
		                 .arg(timer.elapsed() / 1000.0, 0, 'f', 1));

		m_earlyStop = 0;
	}

	ccPointCloud&            m_cloud;
	ccInternalPointCloudLOD& m_lod;
	ccOctree::Shared         m_octree;
	uint32_t                 m_maxCountPerCell;
	uint8_t                  m_maxLevel;
	QAtomicInt               m_earlyStop;
};

ccAbstractPointCloudLOD::ccAbstractPointCloudLOD()
    : m_indexMap(0)
    , m_lastIndexMap(0)
    , m_state(NOT_INITIALIZED)
{
	clearData(); // initializes the root node
}

ccAbstractPointCloudLOD::ccAbstractPointCloudLOD(const std::vector<ccAbstractPointCloudLOD::Level>& lodLayers)
    : ccAbstractPointCloudLOD()
{
	m_levels = lodLayers;
}

size_t ccAbstractPointCloudLOD::memory() const
{
	size_t thisSize = sizeof(ccAbstractPointCloudLOD);

	size_t totalNodeCount = 0;
	for (size_t i = 0; i < m_levels.size(); ++i)
	{
		totalNodeCount += m_levels[i].data.size();
	}
	size_t nodeSize  = sizeof(Node);
	size_t nodesSize = totalNodeCount * nodeSize;

	return nodesSize + thisSize;
}

void ccAbstractPointCloudLOD::clearData()
{
	m_levels.resize(1);
	m_levels.front().data.resize(1);
	m_levels.front().data.front() = Node();
}

int32_t ccAbstractPointCloudLOD::newCell(unsigned char level)
{
	assert(level != 0);
	assert(level < m_levels.size());
	Level& l = m_levels[level];

	// assert(l.data.size() < l.data.capacity());
	l.data.emplace_back(level);

	return static_cast<int32_t>(l.data.size()) - 1;
}

void ccAbstractPointCloudLOD::shrink_to_fit()
{
	QMutexLocker locker(&m_mutex);

	for (size_t i = 1; i < m_levels.size(); ++i) // DGM: always keep the root node!
	{
		if (!m_levels[i].data.empty())
		{
			m_levels[i].data.shrink_to_fit();
		}
		else
		{
			// first empty level: we can reduce the number of levels and stop here!
			m_levels.resize(i);
			break;
		}
	}

	m_levels.shrink_to_fit();
}

void ccAbstractPointCloudLOD::resetVisibility()
{
	if (m_state != INITIALIZED)
	{
		return;
	}

	m_currentState = RenderParams();

	for (Level& level : m_levels)
	{
		for (Node& n : level.data)
		{
			n.displayedPointCount = 0;
			n.intersection        = Frustum::INSIDE;
		}
	}
}

uint32_t ccAbstractPointCloudLOD::flagVisibility(const ccGLCameraParameters& camera, ccClipPlaneSet* clipPlanes /*=nullptr*/)
{
	if (m_state != INITIALIZED)
	{
		assert(false);
		m_currentState = RenderParams();
		return 0;
	}

	resetVisibility();

	auto lodVisibility = getVisibilityFlagger(*this, camera, static_cast<unsigned char>(m_levels.size()));
	if (clipPlanes)
	{
		lodVisibility->setClipPlanes(*clipPlanes);
	}

	m_currentState.visiblePoints = lodVisibility->flag(root());

	return m_currentState.visiblePoints;
}

//! ccInternalPointCloudLOD implementation
ccInternalPointCloudLOD::ccInternalPointCloudLOD()
    : m_octree(nullptr)
    , m_thread(nullptr)
{
}

ccInternalPointCloudLOD::~ccInternalPointCloudLOD()
{
	clear();
}

bool ccInternalPointCloudLOD::init(ccPointCloud* cloud)
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
		// already running?
		assert(false);
		return true;
	}

	m_thread->start();
	return true;
}

void ccInternalPointCloudLOD::clear()
{
	if (m_thread && m_thread->isRunning())
	{
		m_thread->stop();
	}

	m_mutex.lock();

	if (m_thread)
	{
		delete m_thread;
		m_thread = nullptr;
	}

	m_levels.clear();
	m_octree.clear();
	m_state = NOT_INITIALIZED;

	m_mutex.unlock();
}

void ccInternalPointCloudLOD::clearData()
{
	// 1 empty (root) node
	ccAbstractPointCloudLOD::clearData();
	//+ delete the octree
	m_octree.clear();
}

bool ccInternalPointCloudLOD::initInternal(ccOctree::Shared octree)
{
	if (!octree)
	{
		return false;
	}

	// clear the structure (just in case)
	clearData();

	QMutexLocker locker(&m_mutex);

	try
	{
		assert(CCCoreLib::DgmOctree::MAX_OCTREE_LEVEL <= 255);
		m_levels.resize(CCCoreLib::DgmOctree::MAX_OCTREE_LEVEL + 1);
	}
	catch (const std::bad_alloc&)
	{
		// not enough memory
		return false;
	}

	m_octree = octree;

	return true;
}

uint32_t ccInternalPointCloudLOD::addNPointsToIndexMap(Node& node, uint32_t count)
{
	if (m_indexMap.capacity() == 0 || m_octree.isNull())
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
				ccAbstractPointCloudLOD::Node& childNode = this->node(node.childIndexes[i], node.level + 1);
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
					double ratio  = static_cast<double>(childNodeRemainingCount) / thisNodeRemainingCount;
					childMaxCount = static_cast<uint32_t>(ceil(ratio * count));
					if (displayedCount + childMaxCount > count)
					{
						assert(count >= displayedCount);
						childMaxCount = count - displayedCount;
						i             = 8; // we can stop right now
					}
				}

				uint32_t childDisplayedCount = addNPointsToIndexMap(childNode, childMaxCount);
				// assert(childDisplayedCount == childMaxCount || !displayAll || childNode.intersection != Frustum::INSIDE);
				assert(childDisplayedCount <= childMaxCount);

				displayedCount += childDisplayedCount;
				assert(displayedCount <= count);
			}
		}
	}
	else
	{
		// we can display all the points
		// uint32_t iStart = node.displayedPointCount;
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

LODIndexSet& ccInternalPointCloudLOD::getIndexMap(unsigned char level, unsigned& maxCount, unsigned& remainingPointsAtThisLevel)
{
	remainingPointsAtThisLevel = 0;
	m_lastIndexMap.clear();

	if (m_octree.isNull() || level >= m_levels.size())
	{
		assert(false);
		maxCount = 0;
		return m_lastIndexMap; // empty
	}

	if (m_state != INITIALIZED)
	{
		maxCount = 0;
		return m_lastIndexMap; // empty
	}

	if (m_currentState.displayedPoints >= m_currentState.visiblePoints)
	{
		// assert(false);
		maxCount = 0;
		return m_lastIndexMap; // empty
	}

	m_indexMap.clear();
	try
	{
		m_indexMap.reserve(maxCount);
	}
	catch (const std::bad_alloc&)
	{
		// not enough memory
		return m_lastIndexMap; // empty
	}

	Level&   l                    = m_levels[level];
	uint32_t thisPassDisplayCount = 0;

	bool   earlyStop      = false;
	size_t earlyStopIndex = 0;

	// special case: we have to finish/continue at the same level than the previous run
	if (m_currentState.unfinishedLevel == level)
	{
		bool displayAll = (m_currentState.unfinishedPoints <= maxCount);

		// display all leaf cells of the current level
		for (size_t i = 0; i < l.data.size(); ++i)
		{
			Node& node = l.data[i];

			if (node.childCount) // skip non leaf cells
				continue;
			assert(node.intersection != UNDEFINED);
			if (node.intersection == Frustum::OUTSIDE)
				continue;
			if (node.pointCount == node.displayedPointCount)
				continue;

			uint32_t nodeMaxCount       = 0;
			uint32_t nodeRemainingCount = (node.pointCount - node.displayedPointCount);
			if (displayAll)
			{
				nodeMaxCount = nodeRemainingCount;
			}
			else
			{
				double ratio = static_cast<double>(nodeRemainingCount) / m_currentState.unfinishedPoints;
				nodeMaxCount = static_cast<uint32_t>(ceil(ratio * maxCount));
				// safety check
				if (m_indexMap.size() + nodeMaxCount >= maxCount)
				{
					assert(maxCount >= m_indexMap.size());
					nodeMaxCount = maxCount - static_cast<uint32_t>(m_indexMap.size());

					earlyStop      = true;
					earlyStopIndex = i;

					i = l.data.size(); // we can stop after this node!
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
	// remove the already displayed points (= unfinished from previous run)
	assert(totalRemainingCount >= thisPassDisplayCount);
	totalRemainingCount -= thisPassDisplayCount;

	// do we still have data to display AND can we display it?
	if (totalRemainingCount != 0 && thisPassDisplayCount < maxCount)
	{
		// we shouldn't have any unfinished work at this point!
		assert(!earlyStop && remainingPointsAtThisLevel == 0);

		uint32_t mapFreeSize = maxCount - thisPassDisplayCount;

		bool displayAll = (mapFreeSize > totalRemainingCount);

		// for all cells of the input level
		for (size_t i = 0; i < l.data.size(); ++i)
		{
			Node& node = l.data[i];

			assert(node.intersection != UNDEFINED);
			if (node.intersection == Frustum::OUTSIDE)
				continue;
			if (node.pointCount == node.displayedPointCount)
				continue;

			uint32_t nodeMaxCount       = 0;
			uint32_t nodeRemainingCount = (node.pointCount - node.displayedPointCount);
			if (displayAll)
			{
				nodeMaxCount = nodeRemainingCount;
			}
			else if (node.childCount)
			{
				double ratio = static_cast<double>(nodeRemainingCount) / totalRemainingCount;
				nodeMaxCount = static_cast<uint32_t>(ceil(ratio * mapFreeSize));
				// safety check
				if (m_indexMap.size() + nodeMaxCount >= maxCount)
				{
					assert(maxCount >= m_indexMap.size());
					nodeMaxCount = maxCount - static_cast<uint32_t>(m_indexMap.size());

					earlyStop      = true;
					earlyStopIndex = i;

					i = l.data.size(); // we can stop after this node!
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
		// be sure to properly finish to count the number of 'unfinished' points!
		for (size_t i = earlyStopIndex + 1; i < l.data.size(); ++i)
		{
			Node& node = l.data[i];

			if (node.childCount) // skip non leaf nodes
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

	m_currentState.unfinishedLevel  = remainingPointsAtThisLevel ? static_cast<int>(level) : -1;
	m_currentState.unfinishedPoints = remainingPointsAtThisLevel;

	m_lastIndexMap = m_indexMap;
	return m_indexMap;
}

//! ccNestedOctreePointCloudLOD implementation
ccNestedOctreePointCloudLOD::ccNestedOctreePointCloudLOD(const std::vector<ccAbstractPointCloudLOD::Level>& lodLayers)
    : ccAbstractPointCloudLOD(lodLayers){};

bool ccNestedOctreePointCloudLOD::init(ccPointCloud* cloud)
{
	m_state = INITIALIZED;
	return true;
}

void ccNestedOctreePointCloudLOD::clear()
{
	m_levels.clear();
	m_state = NOT_INITIALIZED;
}

LODIndexSet& ccNestedOctreePointCloudLOD::getIndexMap(unsigned char level, unsigned& maxCount, unsigned& remainingPointsAtThisLevel)
{
	m_lastIndexMap.clear();
	remainingPointsAtThisLevel = 0;

	if (m_state != INITIALIZED)
	{
		maxCount = 0;
		return m_lastIndexMap; // empty
	}

	if (m_currentState.displayedPoints >= m_currentState.visiblePoints)
	{
		assert(false);
		maxCount = 0;
		return m_lastIndexMap; // empty
	}

	m_indexMap.clear();
	try
	{
		m_indexMap.reserve(maxCount);
	}
	catch (const std::bad_alloc&)
	{
		// not enough memory
		return m_lastIndexMap; // empty
	}

	Level& l = m_levels[level];

	bool     earlyStop                          = false;
	size_t   earlyStopIndex                     = 0;
	uint32_t computedRemainingPointsAtThisLevel = 0;

	std::vector<Node*> nodePriorityList;

	// Reserve/allocate maximum size for the priority list.
	// Alternatively, we can test without pre-allocating
	// or consider a buffer for each layer, sized according to the node cardinality (at each layer)
	// and use partial sorting to sort the n visible nodes... this should be even faster.
	try
	{
		nodePriorityList.reserve(l.data.size());
	}
	catch (const std::bad_alloc&)
	{
		maxCount = 0;
		return m_lastIndexMap;
	}
	for (auto& node : l.data)
	{
		assert(node.intersection != UNDEFINED);
		if (node.intersection == Frustum::OUTSIDE || node.pointCount == node.displayedPointCount)
			continue;

		computedRemainingPointsAtThisLevel += node.pointCount - node.displayedPointCount;
		nodePriorityList.push_back(&node);
	}

	bool displayAll = computedRemainingPointsAtThisLevel <= maxCount;

	if (!displayAll)
	{
		// Number of cells is small in general and it does not spanning thread to sort this
		std::sort(std::begin(nodePriorityList), std::end(nodePriorityList), [](const Node* a, const Node* b)
		          { return a->score > b->score; });
	}

	for (size_t i = 0; i < nodePriorityList.size(); ++i)
	{
		auto node = nodePriorityList[i];
		if (node->intersection == Frustum::OUTSIDE || node->pointCount == node->displayedPointCount)
			continue;

		// if display all;
		uint32_t       nodeMaxCount       = 0;
		const uint32_t nodeRemainingCount = node->pointCount - node->displayedPointCount;
		if (displayAll || (m_indexMap.size() + nodeRemainingCount <= maxCount))
		{
			nodeMaxCount = nodeRemainingCount;
		}
		else
		{
			// In this case there is an "overflow" of the IndexMap if we fill it with
			// all the content of the node. We take only the number of points
			// that allow to saturate the indexMap.
			// early stop
			assert(maxCount >= m_indexMap.size());
			nodeMaxCount = maxCount - static_cast<uint32_t>(m_indexMap.size());
			earlyStop    = true;
		}

		// we can use a simplified addNPointsToIndexMap
		uint32_t iStop = std::min(node->displayedPointCount + nodeMaxCount, node->pointCount);
		for (uint32_t i = node->displayedPointCount; i < iStop; ++i)
		{
			m_indexMap.push_back(node->firstCodeIndex + i);
		}

		node->displayedPointCount += nodeMaxCount;

		if (earlyStop)
		{
			earlyStopIndex = i;
			break;
		}
	}

	maxCount = static_cast<uint32_t>(m_indexMap.size());
	m_currentState.displayedPoints += maxCount;

	// Compute the remaining point in case of early stop.
	if (earlyStop)
	{
		for (size_t i = earlyStopIndex; i < nodePriorityList.size(); ++i)
		{
			auto& node = nodePriorityList[i];
			// be sure to properly finish to count the number of 'unfinished' points!
			uint32_t nodeRemainingCount = (node->pointCount - node->displayedPointCount);
			remainingPointsAtThisLevel += nodeRemainingCount;
		}
	}

	m_currentState.unfinishedLevel  = remainingPointsAtThisLevel ? static_cast<int>(level) : -1;
	m_currentState.unfinishedPoints = remainingPointsAtThisLevel;
	m_lastIndexMap                  = m_indexMap;
	return m_indexMap;
}

//! ccGenericPointCloudLODVisibilityFlagger implementation
ccGenericPointCloudLODVisibilityFlagger::ccGenericPointCloudLODVisibilityFlagger(ccAbstractPointCloudLOD&    lod,
                                                                                 const ccGLCameraParameters& camera,
                                                                                 unsigned char               maxLevel)
    : m_lod(lod)
    , m_camera(camera)
    , m_frustum(camera.modelViewMat, camera.projectionMat)
    , m_maxLevel(maxLevel)
    , m_hasClipPlanes(false)
{
}

void ccGenericPointCloudLODVisibilityFlagger::setClipPlanes(const ccClipPlaneSet& clipPlanes)
{
	try
	{
		m_clipPlanes = clipPlanes;
	}
	catch (const std::bad_alloc&)
	{
		m_hasClipPlanes = false;
		return;
	}
	m_hasClipPlanes = !m_clipPlanes.empty();
}

void ccGenericPointCloudLODVisibilityFlagger::propagateFlag(ccAbstractPointCloudLOD::Node& node, uint8_t flag)
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

void ccGenericPointCloudLODVisibilityFlagger::clippingIntersection(ccAbstractPointCloudLOD::Node& node)
{
	if (!m_hasClipPlanes)
	{
		return;
	}

	for (const ccClipPlane& clipPlane : m_clipPlanes)
	{
		double dist = clipPlane.equation.x * node.center.x
		              + clipPlane.equation.y * node.center.y
		              + clipPlane.equation.z * node.center.z
		              + clipPlane.equation.w;

		if (dist < node.radius)
		{
			node.intersection = (dist <= -node.radius) ? Frustum::OUTSIDE : Frustum::INTERSECT;
			if (node.intersection == Frustum::OUTSIDE)
				return;
		}
	}
}

uint32_t ccGenericPointCloudLODVisibilityFlagger::flag(ccAbstractPointCloudLOD::Node& node)
{
	node.intersection = m_frustum.sphereInFrustum(node.center, node.radius);
	if (node.intersection != Frustum::OUTSIDE)
	{
		clippingIntersection(node);
	}

	uint32_t visibleCount = 0;
	switch (node.intersection)
	{
	case Frustum::INSIDE:
		visibleCount = node.pointCount;
		break;

	case Frustum::INTERSECT:
		if (node.level < m_maxLevel && node.childCount)
		{
			for (int i = 0; i < 8; ++i)
			{
				if (node.childIndexes[i] >= 0)
				{
					visibleCount += flag(m_lod.node(node.childIndexes[i], node.level + 1));
				}
			}

			if (visibleCount == 0)
			{
				node.intersection = Frustum::OUTSIDE;
			}
		}
		else
		{
			visibleCount = node.pointCount;
		}
		break;

	case Frustum::OUTSIDE:
		propagateFlag(node, Frustum::OUTSIDE);
		break;
	}

	return visibleCount;
}

//! ccNestedOctreePointCloudLODVisibilityFlagger implementation
ccNestedOctreePointCloudLODVisibilityFlagger::ccNestedOctreePointCloudLODVisibilityFlagger(ccAbstractPointCloudLOD&    lod,
                                                                                           const ccGLCameraParameters& camera,
                                                                                           unsigned char               maxLevel,
                                                                                           float                       minPxFootprint)
    : ccGenericPointCloudLODVisibilityFlagger(lod, camera, maxLevel)
    , m_minPxFootprint(minPxFootprint)
    , m_numVisibleNodes(0)
{
}

void ccNestedOctreePointCloudLODVisibilityFlagger::computeNodeFootprint(ccAbstractPointCloudLOD::Node& node)
{
	if (m_camera.perspective)
	{
		const float distance = (m_camera.modelViewMat * node.center).norm();
		if (distance < node.radius)
		{
			node.score = std::numeric_limits<float>::max();
			return;
		}

		const float slope            = std::tan(CCCoreLib::DegreesToRadians(m_camera.fov_deg) * 0.5f);
		const float projectionFactor = (0.5f * (m_camera.viewport[3] - m_camera.viewport[1])) / (slope * distance);
		node.score                   = node.radius * projectionFactor;
	}
	else
	{
		node.score = std::numeric_limits<float>::max();
	}
}

uint32_t ccNestedOctreePointCloudLODVisibilityFlagger::flag(ccAbstractPointCloudLOD::Node& node)
{
	node.intersection = m_frustum.sphereInFrustum(node.center, node.radius);
	if (node.intersection != Frustum::OUTSIDE)
	{
		clippingIntersection(node);
	}

	if (node.intersection != Frustum::OUTSIDE)
	{
		computeNodeFootprint(node);
		if (node.score < m_minPxFootprint)
		{
			// A dedicated flag could be used for that to improve semantic meaning.
			// and it's needed in the PoC of hybrid LOD (VBO+IndexMap)
			node.intersection = Frustum::OUTSIDE;
		}
	}

	uint32_t visibleCount = 0;
	switch (node.intersection)
	{
	case Frustum::INSIDE:
	case Frustum::INTERSECT:
		visibleCount += node.pointCount;
		if (node.level < m_maxLevel && node.childCount)
		{
			for (int i = 0; i < 8; ++i)
			{
				if (node.childIndexes[i] >= 0)
				{
					visibleCount += flag(m_lod.node(node.childIndexes[i], node.level + 1));
				}
			}
		}
		break;
	case Frustum::OUTSIDE:
		propagateFlag(node, Frustum::OUTSIDE);
		break;
	}

	return visibleCount;
}

void ccNestedOctreePointCloudLOD::releaseVBOs(const ccGenericGLDisplay* currentDisplay)
{
	if (managerState == ccAbstractVBOManager::NEW)
		return;

	if (currentDisplay)
	{
		for (auto& level : m_levels)
		{
			for (auto& node : level.data)
			{
				if (node.vbo)
				{
					node.vbo->destroy();
					delete node.vbo;
					node.vbo = nullptr;
				}
			}
		} //'destroy' all vbos
	}
	else
	{
		assert(false);
	}

	hasColors         = false;
	hasNormals        = false;
	colorIsSF         = false;
	sourceSF          = nullptr;
	totalMemSizeBytes = 0;
	managerState      = ccAbstractVBOManager::NEW;
};

bool ccNestedOctreePointCloudLOD::updateVBOs(const ccPointCloud* pc, const ccGenericGLDisplay* currentDisplay, const CC_DRAW_CONTEXT& context, const glDrawParams& glParams)
{
	// TODO: the necessary checks for LOD datastructure
	if (m_state != INITIALIZED)
	{
		// this is unlikely to happen
		assert(false);
		managerState = ccAbstractVBOManager::FAILED;
		return false;
	}

	if (managerState == ccAbstractVBOManager::INITIALIZED)
	{
		// let's check if something has changed
		if (glParams.showColors && (!hasColors || colorIsSF))
		{
			updateFlags |= UPDATE_COLORS;
		}

		if (glParams.showSF
		    && (!hasColors
		        || !colorIsSF
		        || sourceSF != pc->m_currentDisplayedScalarField
		        || pc->m_currentDisplayedScalarField->getModificationFlag() == true))
		{
			updateFlags |= UPDATE_COLORS;
		}

		if (glParams.showNorms && !hasNormals)
		{
			updateFlags |= UPDATE_NORMALS;
		}
		// nothing to do?
		if (updateFlags == 0) // DO NOTHING
		{
			// return true;
		}
	}
	else
	{
		updateFlags = ccAbstractVBOManager::UPDATE_ALL;
	}

	// DGM: the context should be already active as this method should only be called from 'PointCloud::drawMeOnly'
	assert(!glParams.showSF || pc->m_currentDisplayedScalarField);

	hasColors  = glParams.showSF || glParams.showColors;
	colorIsSF  = glParams.showSF;
	sourceSF   = glParams.showSF ? pc->m_currentDisplayedScalarField : nullptr;
	hasNormals = glParams.showNorms;

	size_t totalSizeBytesBefore = totalMemSizeBytes;
	totalMemSizeBytes           = 0;

	for (size_t levelID = 0; levelID < m_levels.size(); levelID++)
	{
		auto& l = m_levels[levelID];
		for (size_t i = 0; i < l.data.size(); ++i)
		{
			Node& node = l.data[i];
			// check if the node is intersected or inside the frustum
			// and if it has data to render
			if (node.intersection == Frustum::OUTSIDE || !node.pointCount)
			{
				if (node.vbo)
				{
					node.vbo->destroy();
					delete node.vbo;
					node.vbo = nullptr;
				}
				continue;
			}

			int  nodeUpdateFlags = updateFlags;
			bool reallocated     = false;

			if (!node.vbo)
			{
				node.vbo = new ccVBO;
			}

			ccVBO* currentVBO   = node.vbo;
			int    vboSizeBytes = currentVBO->init(node.pointCount, hasColors, hasNormals, &reallocated);

			QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
			if (glFunc)
			{
				CatchGLErrors(glFunc->glGetError(), "ccPointCloudLOD::vbo.init");
			}

			if (vboSizeBytes > 0)
			{
				if (reallocated)
				{
					// if the vbo is reallocated, then all its content has been cleared!
					nodeUpdateFlags = UPDATE_ALL;
				}

				currentVBO->bind();

				// load points
				if (nodeUpdateFlags & UPDATE_POINTS)
				{
					currentVBO->write(0, pc->m_points[node.firstCodeIndex].u, sizeof(PointCoordinateType) * node.pointCount * 3);
				}
				// load colors
				if (nodeUpdateFlags & UPDATE_COLORS)
				{
					if (glParams.showSF)
					{
						// copy SF colors in static array
						{
							assert(sourceSF);
							ColorCompType* _sfColors  = ccPointCloud::s_rgbBuffer4ub;
							size_t         chunkStart = static_cast<size_t>(node.firstCodeIndex);
							for (uint32_t j = 0; j < node.pointCount; j++)
							{
								ScalarType sfValue = sourceSF->getValue(chunkStart++);
								// we need to convert scalar value to color into a temporary structure
								const ccColor::Rgb* col = sourceSF->getColor(sfValue);
								if (!col)
									col = &ccColor::lightGreyRGB;
								*_sfColors++ = col->r;
								*_sfColors++ = col->g;
								*_sfColors++ = col->b;
								*_sfColors++ = ccColor::MAX;
							}
						}
						// then send them in VRAM
						currentVBO->write(currentVBO->rgbShift, ccPointCloud::s_rgbBuffer4ub, sizeof(ColorCompType) * node.pointCount * 4);
						// upadte 'modification' flag for current displayed SF
						sourceSF->setModificationFlag(false);
					}
					else if (glParams.showColors)
					{
						currentVBO->write(currentVBO->rgbShift, pc->m_rgbaColors->data() + node.firstCodeIndex, sizeof(ColorCompType) * node.pointCount * 4);
					}
				}
				// load normals
				if (glParams.showNorms && (nodeUpdateFlags & UPDATE_NORMALS))
				{
					assert(pc->m_normals && glFunc);

					// compressed normals set
					const ccNormalVectors* compressedNormals = ccNormalVectors::GetUniqueInstance();

					// compressed normals set
					const CompressedNormType* _normalsIndexes = pc->m_normals->data() + node.firstCodeIndex;
					PointCoordinateType*      outNorms        = ccPointCloud::s_normalBuffer;
					for (uint32_t j = 0; j < node.pointCount; ++j)
					{
						const CCVector3& N = ccNormalVectors::GetNormal(*_normalsIndexes++);
						*(outNorms)++      = N.x;
						*(outNorms)++      = N.y;
						*(outNorms)++      = N.z;
					}
					currentVBO->write(currentVBO->normalShift, ccPointCloud::s_normalBuffer, sizeof(PointCoordinateType) * node.pointCount * 3);
				}
				currentVBO->release();

				// if an error is detected
				QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
				assert(glFunc != nullptr);
				if (CatchGLErrors(glFunc->glGetError(), "ccPointCloud::updateVBOs"))
				{
					vboSizeBytes = -1;
				}
				else
				{
					totalMemSizeBytes += static_cast<size_t>(vboSizeBytes);
				}
			}

			if (vboSizeBytes < 0) // VBO initialization failed
			{
				currentVBO->destroy();
				delete currentVBO;
				currentVBO = nullptr;

				// we can stop here
				managerState = ccAbstractVBOManager::FAILED;
				return false;
			}
			currentVBO->pointCount = node.pointCount;
		}
	}
	managerState = ccAbstractVBOManager::INITIALIZED;
	updateFlags  = 0;

	return true;
}

bool ccNestedOctreePointCloudLOD::renderVBOs(const CC_DRAW_CONTEXT& context, const glDrawParams& glParams)
{
	if (m_state != INITIALIZED)
	{
		// this is unlikely to happen
		assert(false);
		managerState = ccAbstractVBOManager::FAILED;
	}

	if (managerState == ccAbstractVBOManager::FAILED)
	{
		return false;
	}

	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);
	for (size_t levelID = 0; levelID < m_levels.size(); levelID++)
	{
		auto& l = m_levels[levelID];
		for (size_t i = 0; i < l.data.size(); ++i)
		{
			Node& node = l.data[i];
			// check if the node is intersected or inside the frustum
			// and if it has data to render
			if (node.intersection == Frustum::OUTSIDE || node.intersection == UNDEFINED || !node.pointCount || !node.vbo)
				continue;

			if (node.vbo->bind())
			{
				const GLbyte* start = nullptr; // fake pointer used to prevent warnings on Linux
				glFunc->glVertexPointer(3, GL_FLOAT, 3 * sizeof(PointCoordinateType), start);

				if (glParams.showNorms)
				{
					int normalDataShift = node.vbo->normalShift;
					glFunc->glNormalPointer(GL_FLOAT, 3 * sizeof(PointCoordinateType), static_cast<const GLvoid*>(start + normalDataShift));
				}

				if (glParams.showColors || glParams.showSF)
				{
					int colorDataShift = node.vbo->rgbShift;
					glFunc->glColorPointer(4, GL_UNSIGNED_BYTE, 4 * sizeof(ColorCompType), static_cast<const GLvoid*>(start + colorDataShift));
				}
				node.vbo->release();
				// we can use VBOs directly
				glFunc->glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(node.vbo->pointCount));
			}
			else
			{
				ccLog::Warning("[VBO] Failed to bind VBO?! We'll deactivate them then...");
				managerState = ccAbstractVBOManager::FAILED;
				return false;
			}
		}
	}
	return true;
}

#include "ccPointCloudLOD.moc"
