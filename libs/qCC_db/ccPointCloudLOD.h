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

#ifndef CC_POINT_CLOUD_LOD
#define CC_POINT_CLOUD_LOD

//CCLib
#include <GenericChunkedArray.h>

//qCC_db
#include <ccOctree.h>
#include <ccFrustum.h>

//Qt
#include <QMutex>
//system
#include <stdint.h>
#include <assert.h>
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
typedef GenericChunkedArray<1, unsigned> LODIndexSet;

#define USE_LOD_2
#ifdef USE_LOD_2

//! L.O.D. (Level of Detail) structure
class ccPointCloudLOD
{
public:
	//! Structure initialization state
	enum State { NOT_INITIALIZED, UNDER_CONSTRUCTION, INITIALIZED, BROKEN };

	//! Default constructor
	ccPointCloudLOD();
	//! Destructor
	virtual ~ccPointCloudLOD();

	//! Initializes the construction process (asynchronous)
	bool init(ccPointCloud* cloud);

	//! Locks the structure
	inline void lock() { m_mutex.lock(); }
	//! Unlocks the structure
	inline void unlock() { m_mutex.unlock(); }

	//! Returns the current state
	inline State getState() { lock(); State state = m_state; unlock(); return state; }

	//! Clears the structure
	void clear();

	//! Returns the associated octree
	const ccOctree::Shared& octree() const { return m_octree; }

	//! Returns whether the structure is null (i.e. not under construction or initialized) or not
	inline bool isNull() { return getState() == NOT_INITIALIZED; }

	//! Returns whether the structure is initialized or not
	inline bool isInitialized() { return getState() == INITIALIZED; }

	//! Returns whether the structure is initialized or not
	inline bool isUnderConstruction() { return getState() == UNDER_CONSTRUCTION; }

	//! Returns whether the structure is broken or not
	inline bool isBroken() { return getState() == BROKEN; }

	//! Returns the maximum level
	unsigned char maxLevel();

	//! Sets the maximum level
	void setMaxLevel(unsigned char maxLevel);

	static const unsigned char UNDEFINED = 255;
	
	struct Node
	{
		uint8_t					level;						//  1 byte
		uint32_t				pointCount;					//  4 bytes
		float					radius;						//  4 bytes
		CCVector3f				center;						// 12 bytes
		std::array<int32_t, 8>	childIndexes;				// 32 bytes
		uint8_t					childCount;					//  1 byte
		uint32_t				firstCodeIndex;				//  4 bytes

		uint32_t				displayedPointCount;		//  4 bytes
		uint8_t					intersection;				//  1 byte

		//Total												// 63 bytes

		uint8_t					dummy[1];					//  1 bytes
		//Real total										// 64 bytes

		Node(uint8_t _level = 0)
			: level(_level)
			, pointCount(0)
			, radius(0)
			, center(0, 0, 0)
			, firstCodeIndex(0)
			, childCount(0)
			, displayedPointCount(0)
			, intersection(UNDEFINED)
		{
			childIndexes.fill(-1);
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

	inline float maxRadius(unsigned char level) const
	{
		assert(level < m_levels.size());
		return (level < m_levels.size() ? m_levels[level].maxRadius : 0);
	}

	uint32_t flagVisibility(const Frustum& frustum);

	LODIndexSet* getIndexMap(unsigned char depth, unsigned& maxCount, unsigned& remainingPointsAtThisLevel);

	//! Returns whether all points have been displayed or not
	inline bool allDisplayed() const { return m_displayedPoints >= m_visiblePoints; }

protected: //methods

	friend ccPointCloudLODThread;

	//! Reserves memory
	bool initInternal(ccOctree::Shared octree);

	//! Sets the current state
	inline void setState(State state) { lock(); m_state = state; unlock(); }

	//! Clears the structure (with more options)
	void clearExtended(bool autoStopThread, State newState);

	//! Clears the internal (nodes) data
	void clearData();

	//! Reserves a new cell at a given level
	/** \return the new cell index
	**/
	int32_t newCell(unsigned char level);

	//! Shrinks the internal data to its minimum size
	void shrink_to_fit();

	//! Updates the max radius per level FOR ALL CELLS
	void updateMaxRadii();

	//! Resets the internal visibility flags and returns the maximum accessible level
	unsigned char resetVisibility();

	//! Displays a given number of points (should be dispatched among the children cells)
	uint32_t displayNPoints(Node& node, uint32_t count);

protected: //members

	struct Level
	{
		Level()
			: maxRadius(0)
		{}
		
		std::vector<Node> data;
		float maxRadius;
	};

	//! Per-level cells data
	std::vector<Level> m_levels;

	//! Maximum level (ready)
	unsigned char m_maxLevel;

	//! Number of visible points (for the last visibility test)
	uint32_t m_visiblePoints;
	//! Number of already displayed points
	uint32_t m_displayedPoints;
	//! Previously unfinished level
	int m_unfinishedLevel;
	//! Previously unfinished level
	unsigned m_unfinishedPoints;

	//! Last index map
	LODIndexSet* m_lastIndexMap;

	//! Associated octree
	ccOctree::Shared m_octree;

	//! Computing thread
	ccPointCloudLODThread* m_thread;

	//! For concurrent access
	QMutex m_mutex;

	//! State
	State m_state;
};

class PointCloudLODRenderer
{
public:
	
	typedef std::function<void(const ccPointCloudLOD::Node&)> RenderFunc;

	PointCloudLODRenderer(	ccPointCloudLOD& lod,
							RenderFunc func,
							const Frustum& frustum,
							unsigned char maxLevel)
		: m_func(func)
		, m_frustum(frustum)
		, m_lod(lod)
		, m_maxLevel(maxLevel)
	{}

	void render(const ccPointCloudLOD::Node& node, bool testVisibility)
	{
		if (testVisibility)
		{
			switch (m_frustum.sphereInFrustum(node.center, node.radius))
			{
			case Frustum::INSIDE:
				testVisibility = false;
				break;

			case Frustum::INTERSECT:
				testVisibility = true;
				break;

			case Frustum::OUTSIDE:
				return;
			}
		}

		//go on with the display of the children first
		bool hasChildren = false;
		if (node.childCount && node.level < m_maxLevel)
		{
			for (int i = 0; i < 8; ++i)
			{
				if (node.childIndexes[i] >= 0)
				{
					const ccPointCloudLOD::Node& childNode = m_lod.node(node.childIndexes[i], node.level + 1);
					render(childNode, testVisibility);
					hasChildren = true;
				}
			}
		}

		if (!hasChildren)
		{
			m_func(node);
		}
	}

	RenderFunc m_func;
	const Frustum& m_frustum;
	ccPointCloudLOD& m_lod;
	unsigned char m_maxLevel;
};

#else

//! L.O.D. (Level of Detail) structure
class ccPointCloudLOD
{
public:
	//! Structure initialization state
	enum State { NOT_INITIALIZED, UNDER_CONSTRUCTION, INITIALIZED, BROKEN };

	//! Default constructor
	ccPointCloudLOD() : m_indexes(0), m_thread(0), m_state(NOT_INITIALIZED) {}
	//! Destructor
	virtual ~ccPointCloudLOD() { clear(); }

	//! Initializes the construction process (asynchronous)
	bool init(ccPointCloud* cloud);

	//! Locks the structure
	inline void lock() { m_mutex.lock(); }
	//! Unlocks the structure
	inline void unlock() { m_mutex.unlock(); }

	//! Returns the current state
	inline State getState() { lock(); State state = m_state; unlock(); return state; }

	//! Sets the current state
	inline void setState(State state) { lock(); m_state = state; unlock(); }

	//! Clears the structure
	inline void clear() { clearExtended(true, NOT_INITIALIZED); }
	//! Clears the structure (extended version)
	void clearExtended(bool autoStopThread, State newState);

	//! Reserves memory for the indexes
	bool reserve(unsigned pointCount, int levelCount);

	//! Returns whether the structure is null (i.e. not under construction or initialized) or not
	inline bool isNull() { return getState() == NOT_INITIALIZED; }

	//! Returns whether the structure is initialized or not
	inline bool isInitialized() { return getState() == INITIALIZED; }

	//! Returns whether the structure is initialized or not
	inline bool isUnderConstruction() { return getState() == UNDER_CONSTRUCTION; }

	//! Returns whether the structure is broken or not
	inline bool isBroken() { return getState() == BROKEN; }

	//! Returns the indexes (if any)
	inline LODIndexSet* indexes() { return m_indexes; }
	//! Returns the indexes (if any) - const version
	inline const LODIndexSet* indexes() const { return m_indexes; }

	//! Adds a level descriptor
	inline void addLevel(const LODLevelDesc& desc) { lock(); m_levels.push_back(desc); unlock(); }
	//! Shrinks the level descriptor set to its minimal size
	inline void shrink() { lock(); m_levels.resize(m_levels.size()); unlock(); } //DGM: shrink_to_fit is a C++11 method

	//! Returns the maximum level
	inline unsigned char maxLevel() { lock(); size_t count = m_levels.size(); unlock(); return static_cast<unsigned char>(std::min<size_t>(count, 256)); }
	//! Returns a given level descriptor
	inline LODLevelDesc level(unsigned char index) { lock(); LODLevelDesc desc = m_levels[index]; unlock(); return desc; }

protected:

	//! L.O.D. indexes
	/** Point indexes that should be displayed at each level of detail.
	**/
	LODIndexSet* m_indexes;

	//! Actual levels
	std::vector<LODLevelDesc> m_levels;

	//! Computing thread
	ccPointCloudLODThread* m_thread;

	//! For concurrent access
	QMutex m_mutex;

	//! State
	State m_state;
};

#endif

#endif //CC_POINT_CLOUD_LOD