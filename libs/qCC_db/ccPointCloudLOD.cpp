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

//CCLib
#include <DgmOctree.h>

//Qt
#include <QThread>
#include <QAtomicInt>
#include <QElapsedTimer>

//system
#include <assert.h>

//! Thread for background computation
class ccPointCloudLODThread : public QThread
{
public:
	
	//! Default constructor
	ccPointCloudLODThread(ccPointCloud& cloud)
		: QThread()
		, m_cloud(cloud)
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

		ccPointCloudLOD& lod = m_cloud.getLOD();

		//reset structure
		lod.clearExtended(false, ccPointCloudLOD::UNDER_CONSTRUCTION);

		unsigned pointCount = m_cloud.size();
		if (pointCount == 0)
		{
			lod.setState(ccPointCloudLOD::BROKEN);
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
				lod.setState(ccPointCloudLOD::BROKEN);
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
			lod.setState(ccPointCloudLOD::BROKEN);
			flags->release();
			return;
		}

		//init LoD indexes and level descriptors
		if (!lod.reserve(	pointCount,
							CCLib::DgmOctree::MAX_OCTREE_LEVEL + 1) ) //level 0 included
		{
			//not enough memory
			ccLog::Warning(QString("[LoD] Failed to compute LOD structure on cloud '%1' (not enough memory)").arg(m_cloud.getName()));
			lod.setState(ccPointCloudLOD::BROKEN);
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
		lod.addLevel(ccPointCloudLOD::LevelDesc(0, 0));

		unsigned remainingCount = pointCount;

		//and the next ones
		for (unsigned char level=1; level<=static_cast<unsigned char>(CCLib::DgmOctree::MAX_OCTREE_LEVEL); ++level)
		{
			//current level descriptor
			ccPointCloudLOD::LevelDesc levelDesc;
			ccPointCloudLOD::IndexSet* indexes = lod.indexes();
			assert(indexes);
			levelDesc.startIndex = indexes->currentSize();

			//no need to process the points if there are less points remaining than the previous level
			if (remainingCount > lod.level(level - 1).count)
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
				lod.addLevel(levelDesc);
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
			assert(lod.indexes() && lod.indexes()->currentSize() == pointCount);
			lod.shrink();
			lod.setState(ccPointCloudLOD::INITIALIZED);
		}
		else
		{
			//reset
			lod.clearExtended(false, ccPointCloudLOD::NOT_INITIALIZED);
		}

		flags->release();
		flags = 0;

		ccLog::Print(QString("[LoD] Acceleration structure ready for cloud '%1' (max level: %2 / duration: %3 s.)").arg(m_cloud.getName()).arg(static_cast<int>(lod.maxLevel())-1).arg(timer.elapsed() / 1000.0,0,'f',1));
	}

	ccPointCloud& m_cloud;
	
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
		m_thread = new ccPointCloudLODThread(*cloud);
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
