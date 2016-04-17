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

//Qt
#include <QMutex>

class ccPointCloud;
class ccPointCloudLODThread;

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

	//! L.O.D. indexes set
	typedef GenericChunkedArray<1, unsigned> IndexSet;

	//! Returns the indexes (if any)
	inline IndexSet* indexes() { return m_indexes; }
	//! Returns the indexes (if any) - const version
	inline const IndexSet* indexes() const { return m_indexes; }

	//! Level descriptor
	struct LevelDesc
	{
		//! Default constructor
		LevelDesc() : startIndex(0), count(0) {}
		//! Constructor from a start index and a count value
		LevelDesc(unsigned _startIndex, unsigned _count) : startIndex(_startIndex), count(_count) {}
		//! Start index (refers to the 'indexes' table)
		unsigned startIndex;
		//! Index count for this level
		unsigned count;
	};

	//! Adds a level descriptor
	inline void addLevel(const LevelDesc& desc) { lock(); m_levels.push_back(desc); unlock(); }
	//! Shrinks the level descriptor set to its minimal size
	inline void shrink() { lock(); m_levels.resize(m_levels.size()); unlock(); } //DGM: shrink_to_fit is a C++11 method

	//! Returns the maximum level
	inline unsigned char maxLevel() { lock(); size_t count = m_levels.size(); unlock(); return static_cast<unsigned char>(std::min<size_t>(count, 256)); }
	//! Returns a given level descriptor
	inline LevelDesc level(unsigned char index) { lock(); LevelDesc desc = m_levels[index]; unlock(); return desc; }

protected:

	//! L.O.D. indexes
	/** Point indexes that should be displayed at each level of detail.
	**/
	IndexSet* m_indexes;

	//! Actual levels
	std::vector<LevelDesc> m_levels;

	//! Computing thread
	ccPointCloudLODThread* m_thread;

	//! For concurrent access
	QMutex m_mutex;

	//! State
	State m_state;
};

#endif //CC_POINT_CLOUD_LOD