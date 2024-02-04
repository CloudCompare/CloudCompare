#pragma once

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

//qCC_db
#include <ccOctree.h>
#include <ccFrustum.h>

//Qt
#include <QMutex>

//system
#include <stdint.h>
#include <array>
#include <functional>

class ccPointCloud;
class ccPointCloudLODThread;

//! Level descriptor
struct LODLevelDesc
{
	//! Default constructor
	LODLevelDesc() : startIndex(0), count(0) {}
	//! Constructor from a start index and a count value
	LODLevelDesc(unsigned _startIndex, unsigned _count) : startIndex(_startIndex), count(_count) {}
	//! Start index (refers to the 'indexes' table)
	unsigned startIndex;
	//! Index count for this level
	unsigned count;
};

//! L.O.D. indexes set
typedef std::vector<unsigned> LODIndexSet;

//! L.O.D. (Level of Detail) structure
class ccPointCloudLOD
{
public:
	//! Structure initialization state
	enum State { NOT_INITIALIZED, UNDER_CONSTRUCTION, INITIALIZED, BROKEN };

	//! Default constructor
	ccPointCloudLOD();
	//! Destructor
	~ccPointCloudLOD();

	//! Initializes the construction process (asynchronous)
	bool init(ccPointCloud* cloud);

	//! Locks the structure
	inline void lock() const
	{
		m_mutex.lock();
	}
	//! Unlocks the structure
	inline void unlock() const
	{
		m_mutex.unlock();
	}

	//! Returns the current state
	inline State getState() const
	{
		lock();
		State state = m_state;
		unlock();
		return state;
	}

	//! Clears the structure
	void clear();

	//! Returns the associated octree
	const ccOctree::Shared& octree() const
	{
		return m_octree;
	}

	//! Returns whether the structure is null (i.e. not under construction or initialized) or not
	inline bool isNull() const { return getState() == NOT_INITIALIZED; }

	//! Returns whether the structure is initialized or not
	inline bool isInitialized() const { return getState() == INITIALIZED; }

	//! Returns whether the structure is initialized or not
	inline bool isUnderConstruction() const { return getState() == UNDER_CONSTRUCTION; }

	//! Returns whether the structure is broken or not
	inline bool isBroken() const { return getState() == BROKEN; }

	//! Returns the maximum accessible level
	inline unsigned char maxLevel() const
	{
		QMutexLocker locker(&m_mutex);
		return (m_state == INITIALIZED ? static_cast<unsigned char>(std::max<size_t>(1, m_levels.size())) - 1 : 0);
	}

	//! Undefined visibility flag
	static const unsigned char UNDEFINED = 255;
	
	//! Octree 'tree' node
	struct Node
	{
		//Warning: put the non aligned members (< 4 bytes) at the end to avoid too much alignment padding!
		uint32_t				pointCount;					//  4 bytes
		float					radius;						//  4 bytes
		CCVector3f				center;						// 12 bytes
		std::array<int32_t, 8>	childIndexes;				// 32 bytes
		uint32_t				firstCodeIndex;				//  4 bytes
		uint32_t				displayedPointCount;		//  4 bytes
		uint8_t					level;						//  1 byte
		uint8_t					childCount;					//  1 byte
		uint8_t					intersection;				//  1 byte

		//Total												// 63 bytes (64 with alignment)

		//! Default constructor
		Node(uint8_t _level = 0)
			: pointCount(0)
			, radius(0)
			, center(0, 0, 0)
			, childIndexes{-1, -1, -1, -1, -1, -1, -1, -1}
			, firstCodeIndex(0)
			, displayedPointCount(0)
			, level(_level)
			, childCount(0)
			, intersection(UNDEFINED)
		{
		}
	};

	inline Node& node(int32_t index, unsigned char level)
	{
		assert(level < m_levels.size() && index >= 0 && index < m_levels[level].data.size());
		return m_levels[level].data[index];
	}

	inline const Node& node(int32_t index, unsigned char level) const
	{
		assert(level < m_levels.size() && index >= 0 && index < m_levels[level].data.size());
		return m_levels[level].data[index];
	}

	inline Node& root() { return node(0, 0); }

	inline const Node& root() const { return node(0, 0); }

	//! Test all cells visibility with a given frustum
	/** Automatically calls resetVisibility
	**/
	uint32_t flagVisibility(const Frustum& frustum, ccClipPlaneSet* clipPlanes = nullptr);

	//! Builds an index map with the remaining visible points
	LODIndexSet& getIndexMap(unsigned char level, unsigned& maxCount, unsigned& remainingPointsAtThisLevel);

	//! Returns the last index map
	inline const LODIndexSet& getLasIndexMap() const { return m_lastIndexMap; }

	//! Returns whether all points have been displayed or not
	inline bool allDisplayed() const { return m_currentState.displayedPoints >= m_currentState.visiblePoints; }

	//! Returns the memory used by the structure (in bytes)
	size_t memory() const;

protected: //methods

	friend ccPointCloudLODThread;

	//! Reserves memory
	bool initInternal(ccOctree::Shared octree);

	//! Sets the current state
	inline void setState(State state) { lock(); m_state = state; unlock(); }

	//! Clears the internal (nodes) data
	void clearData();

	//! Reserves a new cell at a given level
	/** \return the new cell index in the array corresponding to this level (see m_levels)
	**/
	int32_t newCell(unsigned char level);

	//! Shrinks the internal data to its minimum size
	void shrink_to_fit();

	//! Resets the internal visibility flags
	/** All nodes are flagged as 'INSIDE' (= visible) and their 'visibleCount' attribute is set to 0.
	**/
	void resetVisibility();

	//! Adds a given number of points to the active index map (should be dispatched among the children cells)
	uint32_t addNPointsToIndexMap(Node& node, uint32_t count);

protected: //members

	//! Level data
	struct Level
	{
		std::vector<Node> data;
	};

	//! Per-level cells data
	std::vector<Level> m_levels;

	//! Parameters of the current render state
	struct RenderParams
	{
		RenderParams()
			: visiblePoints(0)
			, displayedPoints(0)
			, unfinishedLevel(-1)
			, unfinishedPoints(0)
		{}

		//! Number of visible points (for the last visibility test)
		uint32_t visiblePoints;
		//! Number of already displayed points
		uint32_t displayedPoints;
		//! Previously unfinished level
		int unfinishedLevel;
		//! Previously unfinished level
		unsigned unfinishedPoints;
	};

	//! Current rendering state
	RenderParams m_currentState;

	//! Index map
	LODIndexSet m_indexMap;

	//! Last index map (pointer on)
	LODIndexSet m_lastIndexMap;

	//! Associated octree
	ccOctree::Shared m_octree;

	//! Computing thread
	ccPointCloudLODThread* m_thread;

	//! For concurrent access
	mutable QMutex m_mutex;

	//! State
	State m_state;
};
