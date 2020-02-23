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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccMinimumSpanningTreeForNormsDirection.h"

//CCLib
#include <ReferenceCloud.h>

//local
#include "ccLog.h"
#include "ccOctree.h"
#include "ccPointCloud.h"
#include "ccProgressDialog.h"
#include "ccScalarField.h"

//system
#include <map>
#include <queue>
#include <set>
#include <vector>

namespace 
{
	//! Weighted graph edge
	class Edge
	{
	public:
		
		//! Unique edge key type
		using Key = std::pair<unsigned, unsigned>;
		
		//! Returns the unique key of an edge (no vertex order)
		inline static Key ConstructKey(unsigned v1, unsigned v2)
		{
			return v1 > v2 ? std::make_pair(v2, v1) : std::make_pair(v1, v2);
		}
		
		//! Default constructor
		Edge(unsigned v1, unsigned v2, float weight)
			: m_key(ConstructKey(v1, v2))
			, m_weight(weight)
		{
			assert(m_weight >= 0);
		}
		
		//! Strict weak ordering operator (required by std::priority_queue)
		inline bool operator < (const Edge& other) const
		{
			return m_weight > other.m_weight;
		}
		
		//! Returns first vertex (index)
		const unsigned& v1() const { return m_key.first; }
		//! Returns second vertex (index)
		const unsigned& v2() const { return m_key.second; }
		
	protected:
		
		//! Unique key
		Key m_key;
		
		//! Associated weight
		float m_weight;
	};
	
	//! Generic graph structure
	class Graph
	{
	public:
		
		//! Default constructor
		Graph() = default;
		
		//! Set of indexes
		using IndexSet = std::set<unsigned int>;
		
		//! Reserves memory for graph
		/** Must be called before using the structure!
		Clears the structure as well.
	**/
		bool reserve(unsigned vertexCount)
		{
			m_edges.clear();
			
			try
			{
				m_vertexNeighbors.resize(vertexCount, IndexSet());
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory
				return false;
			}
			
			return true;
		}
		
		//! Returns the number of vertices
		unsigned vertexCount() const { return static_cast<unsigned>(m_vertexNeighbors.size()); } //the m_vertexNeighbors vector size shouldn't be bigger than a 32 bits integer (see 'reserve')
		
		//! Returns the number of edges
		size_t edgeCount() const { return m_edges.size(); }
		
		//! Returns the weight associated to an edge (if it exists - returns -1 otherwise)
		float weight(unsigned v1, unsigned v2) const
		{
			assert(v1 < m_vertexNeighbors.size() && v2 < m_vertexNeighbors.size());
			
			// try to find the corresponding edge (otherwise return -1)
			std::map<Edge::Key, float>::const_iterator it = m_edges.find(Edge::ConstructKey(v1, v2));
			
			return (it == m_edges.end() ? -1 : it->second);
		}
		
		//! Adds or updates the edge (v1,v2)
		void addEdge(unsigned v1, unsigned v2, float weight = 0.0f)
		{
			assert(v1 < m_vertexNeighbors.size() && v2 < m_vertexNeighbors.size());
			
			m_edges.insert(std::make_pair(Edge::ConstructKey(v1, v2), weight));
			m_vertexNeighbors[v1].insert(v2);
			m_vertexNeighbors[v2].insert(v1);
		}
		
		//! Returns the set of edges connected to a given vertex
		inline const IndexSet& getVertexNeighbors(unsigned index) const
		{
			return m_vertexNeighbors[index];
		}
		
	protected:
		
		//! Graph edges
		std::map<Edge::Key, float> m_edges;
		
		//! Set of neighbors for each vertex
		std::vector<IndexSet> m_vertexNeighbors;
	};
}

static bool ResolveNormalsWithMST(	ccPointCloud* cloud,
#ifdef WITH_GRAPH
									const Graph& graph,
#else
									ccOctree::Shared& octree,
									unsigned char level,
									unsigned kNN,
#endif
									ccProgressDialog* progressCb = nullptr)
{
	assert(cloud && cloud->hasNormals());

//#define COLOR_PATCHES
#ifdef COLOR_PATCHES
	//Test: color patches
	cloud->setColor(ccColor::white);
	cloud->showColors(true);

	//Test: arrival time
	int sfIdx = cloud->getScalarFieldIndexByName("MST arrival time");
	if (sfIdx < 0)
		sfIdx = cloud->addScalarField("MST arrival time");
	ccScalarField* sf = static_cast<ccScalarField*>(cloud->getScalarField(sfIdx));
	sf->fill(NAN_VALUE);
	cloud->setCurrentDisplayedScalarField(sfIdx);
#endif

	//reset
	std::priority_queue<Edge> priorityQueue;
	std::vector<bool> visited;
	unsigned visitedCount = 0;
#ifdef WITH_GRAPH
	unsigned vertexCount = graph.vertexCount();
#else
	unsigned vertexCount = cloud->size();
#endif

	//instantiate the 'visited' table
	try
	{
		visited.resize(vertexCount, false);

		//progress notification
		CCLib::NormalizedProgress nProgress(progressCb, vertexCount);
		if (progressCb)
		{
			progressCb->update(0);
			progressCb->setMethodTitle(QObject::tr("Orient normals (MST)"));
#ifdef WITH_GRAPH
			progressCb->setInfo(QObject::tr("Compute Minimum spanning tree\nPoints: %1\nEdges: %2").arg(vertexCount).arg(graph.edgeCount()));
#else
			progressCb->setInfo(QObject::tr("Compute Minimum spanning tree\nPoints: %1").arg(vertexCount));
#endif
			progressCb->start();
		}

#ifndef WITH_GRAPH
		CCLib::DgmOctree::NearestNeighboursSearchStruct nNSS;
		nNSS.level = level;
		nNSS.minNumberOfNeighbors = kNN + 1; //+1 because we'll get the query point itself!
#endif

		//while unvisited vertices remain...
		unsigned firstUnvisitedIndex = 0;
		size_t patchCount = 0;
		size_t inversionCount = 0;
		while (visitedCount < vertexCount)
		{
			//find the first not-yet-visited vertex
			while (visited[firstUnvisitedIndex])
			{
				++firstUnvisitedIndex;
			}

			//set it as "visited"
			{
				visited[firstUnvisitedIndex] = true;
				++visitedCount;
				//add its neighbors to the priority queue
#ifdef WITH_GRAPH
				const Graph::IndexSet& neighbors = graph.getVertexNeighbors(firstUnvisitedIndex);
				for (Graph::IndexSet::const_iterator it = neighbors.begin(); it != neighbors.end(); ++it)
				{
					priorityQueue.push(Edge(firstUnvisitedIndex, *it, graph.weight(firstUnvisitedIndex, *it)));
				}
#else
				const CCVector3* P = cloud->getPoint(firstUnvisitedIndex);
				nNSS.queryPoint = *P;
				octree->getTheCellPosWhichIncludesThePoint(P, nNSS.cellPos, level);
				octree->computeCellCenter(nNSS.cellPos, level, nNSS.cellCenter);
				nNSS.pointsInNeighbourhood.clear();
				nNSS.alreadyVisitedNeighbourhoodSize = 0;

				//look for neighbors in a sphere
				unsigned neighborCount = octree->findNearestNeighborsStartingFromCell(nNSS, false);
				neighborCount = std::min(neighborCount, kNN + 1);

				//current point index
				const CCVector3& N1 = cloud->getPointNormal(firstUnvisitedIndex);
				for (unsigned j = 0; j < neighborCount; ++j)
				{
					//current neighbor index
					unsigned neighborIndex = nNSS.pointsInNeighbourhood[j].pointIndex;
					if (	firstUnvisitedIndex != neighborIndex
						&&	!visited[neighborIndex])
					{
						const CCVector3& N2 = cloud->getPointNormal(neighborIndex);
						//dot product
						float weight = std::max(0.0f, 1.0f - static_cast<float>(fabs(N1.dot(N2))));

						//distance
						//float weight = sqrt(nNSS.pointsInNeighbourhood[j].squareDistd);

						//mutual dot product
						//const CCVector3* P2 = cloud->getPoint(static_cast<unsigned>(neighborIndex));
						//CCVector3 uAB = *P2 - *P1;
						//uAB.normalize();
						//float weight = (fabs(CCVector3::vdot(uAB.u, N1) + fabs(CCVector3::vdot(uAB.u, N2)))) / 2;

						priorityQueue.emplace( firstUnvisitedIndex, neighborIndex, weight );
					}
				}
#endif

				if (progressCb && !nProgress.oneStep())
				{
					break;
				}
			}

	#ifdef COLOR_PATCHES
			ccColor::Rgb patchCol = ccColor::Generator::Random();
			cloud->setPointColor(static_cast<unsigned>(firstUnvisitedIndex), patchCol.rgb);
			sf->setValue(static_cast<unsigned>(firstUnvisitedIndex),static_cast<ScalarType>(visitedCount));
	#endif

			while (!priorityQueue.empty() && (visitedCount < vertexCount))
			{
				//process next edge (with the lowest 'weight')
				Edge element = priorityQueue.top();
				priorityQueue.pop();

				//shall the normal be inverted?
				const CCVector3& N1 = cloud->getPointNormal(static_cast<unsigned>(element.v1()));
				const CCVector3& N2 = cloud->getPointNormal(static_cast<unsigned>(element.v2()));
				bool inverNormal = (N1.dot(N2) < 0);
				unsigned v = 0;
				//we should change the vertex that has not been visited yet
				if (!visited[element.v1()])
				{
					v = element.v1();
					if (inverNormal)
					{
						cloud->setPointNormal(static_cast<unsigned>(v), -N1);
						++inversionCount;
					}
				}
				else if (!visited[element.v2()])
				{
					v = element.v2();
					if (inverNormal)
					{
						cloud->setPointNormal(static_cast<unsigned>(v), -N2);
						++inversionCount;
					}
				}
				else
				{
					continue;
				}

				//set it as "visited"
				{
					visited[v] = true;
					++visitedCount;
					//add its neighbors to the priority queue
#ifdef WITH_GRAPH
					const Graph::IndexSet& neighbors = graph.getVertexNeighbors(v);
					for (Graph::IndexSet::const_iterator it = neighbors.begin(); it != neighbors.end(); ++it)
						priorityQueue.push(Edge(v, *it, graph.weight(v,*it)));
#else
					const CCVector3* P = cloud->getPoint(v);
					nNSS.queryPoint = *P;
					octree->getTheCellPosWhichIncludesThePoint(P, nNSS.cellPos, level);
					octree->computeCellCenter(nNSS.cellPos, level, nNSS.cellCenter);
					nNSS.pointsInNeighbourhood.clear();
					nNSS.alreadyVisitedNeighbourhoodSize = 0;

					//look for neighbors in a sphere
					unsigned neighborCount = octree->findNearestNeighborsStartingFromCell(nNSS, false);
					neighborCount = std::min(neighborCount, kNN + 1);
					//current point index
					const CCVector3& N1 = cloud->getPointNormal(v);
					for (unsigned j = 0; j < neighborCount; ++j)
					{
						//current neighbor index
						unsigned neighborIndex = nNSS.pointsInNeighbourhood[j].pointIndex;
						if (	v != neighborIndex
							&&	!visited[neighborIndex])
						{
							const CCVector3& N2 = cloud->getPointNormal(neighborIndex);
							//dot product
							float weight = std::max(0.0f, 1.0f - static_cast<float>(fabs(N1.dot(N2))));

							//distance
							//float weight = sqrt(nNSS.pointsInNeighbourhood[j].squareDistd);

							//mutual dot product
							//const CCVector3* P2 = cloud->getPoint(static_cast<unsigned>(neighborIndex));
							//CCVector3 uAB = *P2 - *P1;
							//uAB.normalize();
							//float weight = (fabs(CCVector3::vdot(uAB.u, N1) + fabs(CCVector3::vdot(uAB.u, N2)))) / 2;

							priorityQueue.emplace( v, neighborIndex, weight );
						}
					}
#endif
				}

	#ifdef COLOR_PATCHES
				cloud->setPointColor(static_cast<unsigned>(v), patchCol);
				sf->setValue(static_cast<unsigned>(v),static_cast<ScalarType>(visitedCount));
	#endif
				if (progressCb && !nProgress.oneStep())
				{
					visitedCount = static_cast<unsigned>(vertexCount); //early stop
					break;
				}
			}

			//new patch
			++patchCount;
		}

	#ifdef COLOR_PATCHES
		sf->computeMinAndMax();
		cloud->showSF(true);
	#endif

		if (progressCb)
		{
			progressCb->stop();
		}

		ccLog::Print(QString("[ResolveNormalsWithMST] Patches = %1 / Inversions: %2").arg(patchCount).arg(inversionCount));
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	return true;
}

#ifdef WITH_GRAPH
static bool ComputeMSTGraphAtLevel(	const CCLib::DgmOctree::octreeCell& cell,
									void** additionalParameters,
									CCLib::NormalizedProgress* nProgress/*=0*/)
{
	//parameters
	Graph* graph = static_cast<Graph*>(additionalParameters[0]);
	ccPointCloud* cloud = static_cast<ccPointCloud*>(additionalParameters[1]);

	//structure for the nearest neighbor search
	unsigned kNN = *static_cast<unsigned*>(additionalParameters[2]);

	CCLib::DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level				  = cell.level;
	nNSS.minNumberOfNeighbors = kNN + 1; //+1 because we'll get the query point itself!
	cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);

	unsigned n = cell.points->size(); //number of points in the current cell

	//we already know some of the neighbours: the points in the current cell!
	{
		try
		{
			nNSS.pointsInNeighbourhood.resize(n);
		}
		catch (.../*const std::bad_alloc&*/) //out of memory
		{
			return false;
		}

		CCLib::DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
		for (unsigned i = 0; i < n; ++i, ++it)
		{
			it->point = cell.points->getPointPersistentPtr(i);
			it->pointIndex = cell.points->getPointGlobalIndex(i);
		}
	}
	nNSS.alreadyVisitedNeighbourhoodSize = 1;

	//for each point in the cell
	for (unsigned i = 0; i < n; ++i)
	{
		cell.points->getPoint(i, nNSS.queryPoint);

		//look for neighbors in a sphere
		unsigned neighborCount = cell.parentOctree->findNearestNeighborsStartingFromCell(nNSS, false);
		neighborCount = std::min(neighborCount, kNN + 1);

		//current point index
		unsigned index = cell.points->getPointGlobalIndex(i);
		const CCVector3& N1 = cloud->getPointNormal(index);
		//const CCVector3* P1 = cloud->getPoint(static_cast<unsigned>(index));
		for (unsigned j = 0; j < neighborCount; ++j)
		{
			//current neighbor index
			unsigned neighborIndex = nNSS.pointsInNeighbourhood[j].pointIndex;
			if (index != neighborIndex)
			{
				const CCVector3& N2 = cloud->getPointNormal(neighborIndex);
				//dot product
				float weight = std::max(0.0f, 1.0f - static_cast<float>(fabs(N1.dot(N2))));

				//distance
				//float weight = sqrt(nNSS.pointsInNeighbourhood[j].squareDistd);

				//mutual dot product
				//const CCVector3* P2 = cloud->getPoint(static_cast<unsigned>(neighborIndex));
				//CCVector3 uAB = *P2 - *P1;
				//uAB.normalize();
				//float weight = (fabs(CCVector3::vdot(uAB.u, N1) + fabs(CCVector3::vdot(uAB.u, N2)))) / 2;

				graph->addEdge(index, neighborIndex, weight);
			}
		}

		if (nProgress && !nProgress->oneStep())
			return false;
	}

	return true;
}
#endif

bool ccMinimumSpanningTreeForNormsDirection::OrientNormals(	ccPointCloud* cloud,
															unsigned kNN/*=6*/,
															ccProgressDialog* progressDlg/*=0*/)
{
	assert(cloud);
	if (!cloud->hasNormals())
	{
		ccLog::Warning(QString("Cloud '%1' has no normals!").arg(cloud->getName()));
		return false;
	}

	//we need the octree
	if (!cloud->getOctree())
	{
		if (!cloud->computeOctree(progressDlg))
		{
			ccLog::Warning(QString("[orientNormalsWithMST] Could not compute octree on cloud '%1'").arg(cloud->getName()));
			return false;
		}
	}
	ccOctree::Shared octree = cloud->getOctree();
	assert(octree);

	unsigned char level = octree->findBestLevelForAGivenPopulationPerCell(kNN);

	bool result = true;
	try
	{
#ifdef WITH_GRAPH
		Graph graph;
		if (!graph.reserve(cloud->size()))
		{
			//not enough memory!
			result = false;
		}

		//parameters
		void* additionalParameters[3] = {	reinterpret_cast<void*>(&graph),
											reinterpret_cast<void*>(cloud),
											reinterpret_cast<void*>(&kNN)
										};

		if (octree->executeFunctionForAllCellsAtLevel(	level,
														&ComputeMSTGraphAtLevel,
														additionalParameters,
														false, //not compatible with parallel strategies!
														progressDlg,
														"Build Spanning Tree") == 0)
		{
			//something went wrong
			ccLog::Warning(QString("Failed to compute Spanning Tree on cloud '%1'").arg(cloud->getName()));
			result = false;
		}
		else
		{
			if (!ResolveNormalsWithMST(cloud, graph, progressDlg))
			{
				//something went wrong
				ccLog::Warning(QString("Failed to compute Minimum Spanning Tree on cloud '%1'").arg(cloud->getName()));
				result = false;
			}
		}
#else
		if (!ResolveNormalsWithMST(cloud, octree, level, kNN, progressDlg))
		{
			//something went wrong
			ccLog::Warning(QString("Failed to resolve normals orientation with Minimum Spanning Tree on cloud '%1'").arg(cloud->getName()));
			result = false;
		}
#endif
	}
	catch (...)
	{
		ccLog::Error(QString("Process failed on cloud '%1'").arg(cloud->getName()));
		result = false;
	}

	return result;
}
