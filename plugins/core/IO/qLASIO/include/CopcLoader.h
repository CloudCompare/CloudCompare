#pragma once

// ##########################################################################
// #                                                                        #
// #                CLOUDCOMPARE PLUGIN: LAS-IO Plugin                      #
// #                            COPCLoader                                  #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 of the License.               #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #                   COPYRIGHT: Hobu, Inc.                           	    #
// #                                                                        #
// ##########################################################################

#include "CopcVlrs.h"
#include "LasDetails.h"

// CC
// TODO LOD
//#include "ccPointCloudLOD.h"

// Qt
#include <QFile>

// Laszip
#include <laszip/laszip_api.h>

// System
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// CCCoreLib
#include <CCGeom.h>
#include <ccLog.h>

using namespace LasDetails;

namespace copc
{
	using MyBox = CCCoreLib::BoundingBoxTpl<double>;

	class CopcLoader
	{
	  public: // methods
		/// Constructor
		///
		/// because the constructor could fail, the user should use isValid() check before
		/// using the instance.
		explicit CopcLoader(const laszip_header* laszipHeader, const QString& fileName);

		~CopcLoader() = default;

		/// We do not want copy constructor and assigment for now.
		CopcLoader(CopcLoader const&)            = delete;
		CopcLoader& operator=(CopcLoader const&) = delete;

		/// Enable and set the max level constraint
		void setMaxLevelConstraint(uint32_t maxLevelConstraint)
		{
			m_hasMaxLevelConstraint = true;
			m_maxLevelConstraint    = maxLevelConstraint;
		}

		/// Release the max level constraint and set the max level to the max level of the octree
		void releaseMaxLevelConstraint()
		{
			m_hasMaxLevelConstraint = false;
			m_maxLevelConstraint    = m_maxLevel;
		}

		/// Enable and set the clipping box constraint
		void setClippingBoxConstraint(const LasDetails::UnscaledExtent& extent)
		{
			m_hasClippingConstraint = true;
			m_ClippingConstraint    = extent;
		}

		/// Release the clipping Constraint and reset the clipping box to the root extent
		void releaseClippingBoxConstraint()
		{
			m_hasClippingConstraint = false;
			m_ClippingConstraint    = m_extent;
		}

		/// Returns an array containing the number of point at each level
		/// (sequantially and from the root to the deepest depth)
		const std::vector<uint64_t>& levelPointCounts() const
		{
			return m_levelPointCounts;
		}

		/// Returns the current full (non clipped) extent
		const LasDetails::UnscaledExtent& extent() const
		{
			return m_extent;
		}

		/// Returns the current clippingExtent
		const LasDetails::UnscaledExtent& clippingExtent() const
		{
			return m_ClippingConstraint;
		}

		/// Returns the Maximal number of Level (i.e; depth)
		/// of the current COPC octree.
		const int32_t maxLevel() const
		{
			return m_maxLevel;
		}

		/// Returns the validity of the viewer.
		///
		/// this is set by the constructor.
		/// User of this class have the responsibility to check this getter
		/// before any usage of the instance of the CopcLoader.
		bool isValid() const
		{
			return m_isValid;
		}

		/// Set global shift. This is used by
		void setGlobalShift(const CCVector3d& globalShift)
		{
			m_globalShift = globalShift;
		}

		/// Get the ChunkIntervals of this COPCfile
		///
		/// The entire ChunkInterval vector/set is sorted by pointOffset to minimize seeking.
		/// It is a vector of references because file reading could have side effect on ChunkIntervals.
		/// Moreover an estimatedPointCount is returned in order to resize the CC point cloud.
		/// estimatedPointCount is only an estimation if a Clipping constraint is set, because points in intersected node
		/// can only be filtered during file reading. if no clipping constraint is set, the estimatedPointCount variable represent the true number of points.
		void getChunkIntervalsSet(std::vector<std::reference_wrapper<ChunkInterval>>& sortedChunkIntervalSet, uint64_t& estimatedPointCount);

		/// Recurse COPC octree and create a CC LOD matching this Hierachy
		// TODO LOD
		//std::vector<ccGenericPointCloudLOD::Level> createLOD();

	  public: // static methods
		/// Determive if the file has the potential to contains COPC structure
		static bool IsPutativeCOPCFile(const laszip_header* laszipHeader)
		{

			return laszipHeader->version_minor >= COPC_LAS_VERSION_MINOR
			       && laszipHeader->point_data_format > COPC_LAS_DATA_FORMAT_GT
			       && laszipHeader->point_data_format < COPC_LAS_DATA_FORMAT_LT
			       && laszipHeader->number_of_variable_length_records > 0
			       && IsCOPCVlr(laszipHeader->vlrs[0]);
		}

		/// Check if a given VLR is a COPC one
		///
		/// Notes: COPC VLR has to be at id 0 of the vlr array.
		static bool IsCOPCVlr(const laszip_vlr_struct& vlr)
		{
			if (strcmp(COPC_USER_ID, vlr.user_id) == 0 && vlr.record_id == COPC_RECORD_ID)
			{
				return true;
			}
			return false;
		}

	  private: // methods
		/// Generate the chunk table hierarchy from COPC entries
		///
		/// Entries are supposed to be contiguous.
		/// They are sorted and then parsed sequentially to retrieve the first point
		/// for each entry.
		///	This is because LAZLib (in its public API) can only seek within a file by point but NOT by byte.
		/// we rely on the same trick than LASLib and use a kind of proxy class (here ChunkInterval)
		/// to transform COPC entries into a datastructure than can match the "limitations" of the LASZip API.
		/// After creation ChunkIntervals are inserted into an octree, i.e a std::unordered_map using voxelKey
		/// as key.
		void generateChunktableIntervalsHierarchy(std::vector<Entry>& entries);

		/// Check if a given VoxelKey pass constraints/filter tests and return its status
		ChunkInterval::eFilterStatus checkConstraints(const VoxelKey& voxelkey);

		/// Reset the satus of the chunkIntervals
		void resetIntervalsStatus();

		/// Recursively propage failure fag to COPC nodes
		void propagateFailureFlag(const VoxelKey& voxelkey);

		/// Recursively Flag ChunkIntervals (i.e check constraints of each VoxelKey)
		uint64_t flagIntervalNodes(const VoxelKey& voxelkey);

		/// Recurse the COPC tree and emplace visited VoxelKeys into
		void recurse(const VoxelKey& voxelkey, std::unordered_set<VoxelKey>& visitedNodes);

		/// Traverse the COPC octree and if all nodes are reachables.
		bool isTraversable();

		/// Consistancy check: check if the root node exists
		bool hasRoot()
		{
			return m_chunkIntervalsHierarchy.count(VoxelKey::Root());
		}

	  private: // static members
		/// static members used to check if a file is a COPC
		static const uint8_t         COPC_LAS_VERSION_MINOR{4};
		static const uint8_t         COPC_LAS_DATA_FORMAT_GT{5};
		static const uint8_t         COPC_LAS_DATA_FORMAT_LT{9};
		static const uint8_t         COPC_RECORD_ID{1};
		static constexpr const char* COPC_USER_ID = "copc";

	  private: // members
		bool           m_isValid{false};
		int32_t        m_maxLevel{0};
		uint64_t       m_numPoints{0};
		CCVector3d     m_globalShift;
		UnscaledExtent m_extent;

		bool m_hasClippingConstraint;
		bool m_hasMaxLevelConstraint;

		int32_t        m_maxLevelConstraint{0};
		UnscaledExtent m_ClippingConstraint;

		std::vector<uint64_t>                       m_levelPointCounts;
		std::unordered_map<VoxelKey, ChunkInterval> m_chunkIntervalsHierarchy;
		Info                                        m_copcInfo;
	};
} // namespace copc
