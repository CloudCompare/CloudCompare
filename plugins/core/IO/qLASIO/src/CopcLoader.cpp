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

// QLASIO
#include "CopcLoader.h"

#include "CopcVlrs.h"

// CCCoreLib
#include <CCGeom.h>
#include <ccLog.h>

// System
#include <algorithm>
#include <queue>

namespace copc
{
	CopcLoader::CopcLoader(const laszip_header* laszipHeader, const QString& fileName)
	{
		{
			// parse the info vlr
			const laszip_vlr copcInfoVlr = laszipHeader->vlrs[0];
			if (copcInfoVlr.record_length_after_header != Info::SIZE)
			{
				ccLog::Warning("[LAS] laszip error: invalid Copc Info");
				return;
			}

			// extract the COPC infos
			QDataStream copcInfoData(QByteArray::fromRawData(reinterpret_cast<const char*>(copcInfoVlr.data), Info::SIZE));
			copcInfoData >> m_copcInfo;

			if (!m_copcInfo.extractExtent(m_extent))
			{
				ccLog::Warning("[LAS] laszip error: invalid Copc extent");
				return;
			}
			m_ClippingConstraint = m_extent;

			QFile copcFile;
			copcFile.setFileName(fileName);

			if (!copcFile.open(QFile::ReadOnly))
			{
				ccLog::Warning("[LAS] laszip error: Unable to open the LAS file to parse COPC pages");
				return;
			}

			// Note: the root page is not the first entry of COPC EVRL data reccord,
			// This means we cannot obtain the EVLR header by seeking straight to `root_hier_offset` minus the EVLR header size (60).
			// To check é&the existence of the COPC EVLR hierarchy, we'll have to iterate thought all
			// EVLRs starting from laszip_header::start_of_first_extended_variable_length_record.
			// We do not check the existence of the EVLR header. we seek direclty to the root_hier_offset
			// and validate each seeking operation by checking for EOF.
			QDataStream        copcDataSource(&copcFile);
			std::queue<Page>   pageQueue;
			std::vector<Entry> entries;
			// Enque the root page
			pageQueue.emplace(m_copcInfo.root_hier_offset, m_copcInfo.root_hier_size / Entry::SIZE);

			size_t numPages = 0;
			while (!pageQueue.empty())
			{
				const Page& page = pageQueue.front();
				++numPages;

				copcFile.seek(page.offset);
				if (copcDataSource.status() == QDataStream::Status::ReadPastEnd)
				{
					ccLog::Warning("[LAS] Failed to read the the COPC pages (unexpected EOF");
					return;
				}
				entries.reserve(entries.size() + page.num_entries);
				QDataStream entryDataStream(&copcFile);
				for (size_t entry_id = 0; entry_id < page.num_entries; ++entry_id)
				{
					Entry entry;
					copcDataSource >> entry;

					// Check the key validity and early return if it fails
					if (!entry.key.isValid())
					{
						ccLog::Warning("[LAS] Failed to read the the COPC pages (invalid entry)");
						return;
					}

					m_maxLevel = std::max(m_maxLevel, entry.key.level);
					if (copcDataSource.status() == QDataStream::Status::ReadPastEnd)
					{
						ccLog::Warning("[LAS] Failed to read the the COPC pages (unexpected EOF");
						return;
					}
					if (entry.isHierarchyPage())
					{
						pageQueue.emplace(entry.offset, entry.byte_size / Entry::SIZE);
					}
					else
					{
						m_numPoints += entry.point_count;
						entries.push_back(std::move(entry));
					}
				}
				pageQueue.pop();
			}

			ccLog::Print("[LAS] COPC file with %u pages / %lu entries / %llu points", numPages, entries.size(), m_numPoints);

			// init our octree structure
			generateChunktableIntervalsHierarchy(entries);

			// Consistency check: Check the number of points in the COPC data structure
			// is equal to the number of points in the laszip header
			if (LasDetails::TrueNumberOfPoints(laszipHeader) != m_numPoints)
			{
				ccLog::Warning("[LAS] Failed to read the the COPC file (number of points in COPC structure does not match the one in LAZ header)");
				return;
			}

			// Consistency check: Does the octree have a root node?
			if (!hasRoot())
			{
				ccLog::Warning("[LAS] COPC file Missing root node");
				return;
			}

			// No this is considered as a valid file
			m_isValid = true;

			// Fill m_levelPointCounts vector (used by UI to provide feedback)
			m_levelPointCounts.resize(m_maxLevel + 1);
			for (const auto& entry : entries)
			{
				m_levelPointCounts[entry.key.level] += entry.point_count;
			}

			// Consistency check: is the octree traversable
			// if not, only issue a warning we will simply not handle the non-traversable part.
			if (!isTraversable())
			{
				ccLog::Warning("[LAS] COPC file contains unreachable nodes");
			}
		}
	}

	void CopcLoader::generateChunktableIntervalsHierarchy(std::vector<Entry>& entries)
	{
		// Sort entries by offset to be able to get the first point of each chunk.
		std::sort(std::begin(entries), std::end(entries), [](const Entry& a, const Entry& b)
		          { return a.offset < b.offset; });
		uint64_t starting_point = 0;
		m_chunkIntervalsHierarchy.reserve(entries.size());
		std::for_each(std::cbegin(entries), std::cend(entries), [&starting_point, this](const Entry& entry)
		              {
			m_chunkIntervalsHierarchy.emplace(std::make_pair(entry.key, ChunkInterval(starting_point, static_cast<uint64_t>(entry.point_count))));
			starting_point += entry.point_count; });
	}

	ChunkInterval::eFilterStatus CopcLoader::checkConstraints(const VoxelKey& voxelkey)
	{
		// Level constraint takes precendence over BB constraint
		if (m_hasMaxLevelConstraint && voxelkey.level > m_maxLevelConstraint)
		{
			return ChunkInterval::eFilterStatus::FAIL;
		}
		if (m_hasClippingConstraint)
		{
			LasDetails::UnscaledExtent voxelExtent;
			bool                       valid = voxelkey.extractExtent(m_extent, voxelExtent);

			// this would always pass:
			// m_exent has been checked for consistency in the constructor
			assert(valid);

			// references for convenience
			const auto aMin = m_ClippingConstraint.minCorner();
			const auto aMax = m_ClippingConstraint.maxCorner();
			const auto bMin = voxelExtent.minCorner();
			const auto bMax = voxelExtent.maxCorner();

			// works because all extents are supposed to be axis aligned
			// test for inclusion
			if (aMax.x >= bMax.x
			    && aMax.y >= bMax.y
			    && aMax.z >= bMax.z
			    && aMin.x <= bMin.x
			    && aMin.y <= bMin.y
			    && aMin.z <= bMin.z)
			{
				return ChunkInterval::eFilterStatus::PASS;
			}
			// test for intersection
			else if (aMax.x >= bMin.x
			         && aMax.y >= bMin.y
			         && aMax.z >= bMin.z
			         && aMin.x <= bMax.x
			         && aMin.y <= bMax.y
			         && aMin.z <= bMax.z)
			{
				return ChunkInterval::eFilterStatus::INTERSECT_BB;
			}
			else
			{
				return ChunkInterval::eFilterStatus::FAIL;
			}
		}
		return ChunkInterval::eFilterStatus::PASS;
	}

	void CopcLoader::resetIntervalsStatus()
	{
		std::for_each(std::begin(m_chunkIntervalsHierarchy), std::end(m_chunkIntervalsHierarchy), [](auto& kv)
		              {
				kv.second.status = ChunkInterval::eFilterStatus::PASS;
				kv.second.filteredPointCount = 0; });
	}

/* TODO: create LOD
	std::vector<ccGenericPointCloudLOD::Level> CopcLoader::createLOD()
	{
		size_t                                     maxNumLayers = (m_hasMaxLevelConstraint ? m_maxLevelConstraint : m_maxLevel) + 1;
		std::vector<ccGenericPointCloudLOD::Level> lodLevels(maxNumLayers);

		// keep track of the VoxelKey by insterting them in the same order than CC LOD Nodes.
		std::vector<std::vector<VoxelKey>> lodKeys(maxNumLayers);
		lodLevels[0].data.emplace_back(0);
		lodKeys[0].push_back(VoxelKey::Root());

		// First we populate the root level with the root node.
		// root existence in copc hierarchy is tested in the constructor
		// so the indexing operator should not fail.
		const auto& rootInterval = m_chunkIntervalsHierarchy[VoxelKey::Root()];

		// Create the rootNode
		auto& rootNode          = lodLevels[0].data.back();
		rootNode.pointCount     = rootInterval.pointCount - rootInterval.filteredPointCount;
		rootNode.firstCodeIndex = rootInterval.pointOffsetInCCCloud;

		// Compute the enclosing sphere for the root node,
		// sphere could be computed "exactly" by retrieving each point of the root copc entry.
		// we simply use the enclosing sphere of the node/voxel but we should try to left the possibility to compute it
		// elsewhere
		// the sphere center is a float in the LOD struct, so we do not use PointCoordinateType
		// but we shift the center and force it to floating.
		// We do no take into account Clipping.
		CCVector3f rootSphereCenter(static_cast<float>(m_copcInfo.center_x + m_globalShift.x),
		                            static_cast<float>(m_copcInfo.center_y + m_globalShift.y),
		                            static_cast<float>(m_copcInfo.center_z + m_globalShift.z));

		// init the sphere for the rootNode
		rootNode.radius = static_cast<float>(m_extent.getDiagNorm()) / 2.f; // same as halfsize * sqrt(3.0)
		rootNode.center = rootSphereCenter;

		for (size_t levelIndex = 0; levelIndex < maxNumLayers - 1; ++levelIndex)
		{
			auto&  currLevel      = lodLevels[levelIndex];
			size_t nextLevelIndex = levelIndex + 1;

			// test the 8 children for each node of current layer and insert them in the next layer
			for (size_t nodeIndex = 0; nodeIndex < currLevel.data.size(); ++nodeIndex)
			{
				auto&       parentNode = currLevel.data[nodeIndex];
				const auto& parentKey  = lodKeys[levelIndex][nodeIndex];
				for (const auto& childKey : parentKey.childrenKeys())
				{
					const auto& maybeChild = m_chunkIntervalsHierarchy.find(childKey);
					if (maybeChild != std::end(m_chunkIntervalsHierarchy))
					{
						const auto& childInterval = maybeChild->second;
						if (maybeChild->second.status != ChunkInterval::eFilterStatus::FAIL)
						{
							lodKeys[nextLevelIndex].push_back(childKey);
							lodLevels[nextLevelIndex].data.emplace_back(nextLevelIndex);
							auto& newNode          = lodLevels[nextLevelIndex].data.back();
							newNode.firstCodeIndex = childInterval.pointOffsetInCCCloud;
							newNode.pointCount     = childInterval.pointCount - childInterval.filteredPointCount;

							LasDetails::UnscaledExtent voxelExtent;
							childKey.extractExtent(m_extent, voxelExtent);
							newNode.radius                                   = static_cast<float>(voxelExtent.getDiagNorm()) / 2.f;
							const CCVector3d centerd                         = voxelExtent.getCenter();
							newNode.center                                   = CCVector3f(static_cast<float>(centerd.x + m_globalShift.x),
                                                        static_cast<float>(centerd.y + m_globalShift.y),
                                                        static_cast<float>(centerd.z + m_globalShift.z));
							parentNode.childIndexes[parentNode.childCount++] = lodKeys[nextLevelIndex].size() - 1;
						}
					}
				}
			}
		}
		return lodLevels;
	}*/

	void CopcLoader::propagateFailureFlag(const VoxelKey& voxelkey)
	{
		if (voxelkey.level > m_maxLevel)
		{
			return;
		}
		auto maybeNode = m_chunkIntervalsHierarchy.find(voxelkey);
		if (maybeNode != std::end(m_chunkIntervalsHierarchy))
		{
			maybeNode->second.status = ChunkInterval::eFilterStatus::FAIL;
			for (const auto& childKey : voxelkey.childrenKeys())
			{
				propagateFailureFlag(childKey);
			}
		}
	}

	void CopcLoader::recurse(const VoxelKey& voxelkey, std::unordered_set<VoxelKey, std::hash<VoxelKey>>& visitedNodes)
	{
		if (voxelkey.level <= m_maxLevel && m_chunkIntervalsHierarchy.count(voxelkey))
		{
			visitedNodes.insert(voxelkey);
			for (const auto& childKey : voxelkey.childrenKeys())
			{
				recurse(childKey, visitedNodes);
			}
		}
	}

	bool CopcLoader::isTraversable()
	{
		std::unordered_set<VoxelKey> visitedNodes;
		recurse(VoxelKey::Root(), visitedNodes);
		return visitedNodes.size() == m_chunkIntervalsHierarchy.size();
	}

	uint64_t CopcLoader::flagIntervalNodes(const VoxelKey& voxelkey)
	{
		uint64_t pointCount = 0;
		auto     maybeNode  = m_chunkIntervalsHierarchy.find(voxelkey);
		if (maybeNode != std::end(m_chunkIntervalsHierarchy))
		{
			auto& node  = maybeNode->second;
			node.status = checkConstraints(voxelkey);
			if (node.status == ChunkInterval::eFilterStatus::FAIL && voxelkey.level < m_maxLevel)
			{
				propagateFailureFlag(voxelkey);
			}
			else
			{
				pointCount = node.pointCount;
				if (voxelkey.level < m_maxLevel)
				{
					const auto childrenKeys = voxelkey.childrenKeys();
					for (const auto& childKey : childrenKeys)
					{
						pointCount += flagIntervalNodes(childKey);
					}
				}
				// node with 0 point at a a given level are allowed but here
				// the entire traversal at this node does not provides any point,
				// so it's a "failure".
				if (pointCount == 0)
				{
					node.status = ChunkInterval::eFilterStatus::FAIL;
				}
			}
		}
		return pointCount;
	}

	void CopcLoader::getChunkIntervalsSet(std::vector<std::reference_wrapper<ChunkInterval>>& sortedChunkIntervalSet, uint64_t& estimatedPointCount)
	{
		resetIntervalsStatus();
		estimatedPointCount = 0;
		sortedChunkIntervalSet.clear();
		sortedChunkIntervalSet.reserve(m_chunkIntervalsHierarchy.size());

		// start by the root and flag all the hierachy according to constraints;
		estimatedPointCount = flagIntervalNodes(VoxelKey::Root());

		// fill sortedChunkIntervalSet with m_chunkIntervalsHierarchy values
		std::transform(std::begin(m_chunkIntervalsHierarchy), std::end(m_chunkIntervalsHierarchy), std::back_inserter(sortedChunkIntervalSet), [](auto& kv)
		               { return std::ref(kv.second); });

		// sort them by starting point to optimize seeking;
		std::sort(std::begin(sortedChunkIntervalSet), std::end(sortedChunkIntervalSet), [](const ChunkInterval& a, const ChunkInterval& b)
		          { return a.pointOffsetInFile < b.pointOffsetInFile; });
	}

} // namespace copc
