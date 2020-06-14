//##########################################################################
//#                                                                        #
//#                    CLOUDCOMPARE PLUGIN: ccCompass                      #
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
//#                     COPYRIGHT: Sam Thiele  2017                        #
//#                                                                        #
//##########################################################################

#include "ccTrace.h"
#include <queue>
#include <bitset>

ccTrace::ccTrace(ccPointCloud* associatedCloud) : ccPolyline(associatedCloud)
{
	init(associatedCloud);
}

ccTrace::ccTrace(ccPolyline* obj)
	: ccPolyline(obj->getAssociatedCloud())
{
	ccPointCloud* cld = dynamic_cast<ccPointCloud*>(obj->getAssociatedCloud());
	assert(cld != nullptr); //should never be null
	init(cld);

	//load waypoints from metadata
	if (obj->hasMetaData("waypoints"))
	{
		QString waypoints = obj->getMetaData("waypoints").toString();
		for (QString str : waypoints.split(","))
		{
			if (str != "")
			{
				int pID = str.toInt();
				m_waypoints.push_back(pID); //add waypoint
			}
		}

		//store waypoints metadata
		QVariantMap* map = new QVariantMap();
		map->insert("waypoints", waypoints);
		setMetaData(*map, true);
	}

	//load cost function from metadata
	if (obj->hasMetaData("cost_function"))
	{
		ccTrace::COST_MODE = obj->getMetaData("cost_function").toInt();
	}

	setName(obj->getName());

	//copy polyline into trace points
	std::deque<int> seg;
	for (unsigned i = 0; i < obj->size(); i++)
	{
		//copy into "trace" object
		int pId = obj->getPointGlobalIndex(i); //get global point ID
		seg.push_back(pId);

		//also copy into polyline object
		addPointIndex(pId);
	}
	m_trace.push_back(seg);

	//recalculate trace if polyline data somehow lost [redundancy thing..]
	if (obj->size() == 0)
	{
		m_trace.clear();
		optimizePath(); //[slooooow...!]
	}

	//load SNE data from metadata (TODO)

	invalidateBoundingBox(); //update bounding box (for picking)
}

void ccTrace::init(ccPointCloud* associatedCloud)
{
	setAssociatedCloud(associatedCloud); //the ccPolyline c'tor should do this, but just to be sure...
	m_cloud = associatedCloud; //store pointer ourselves also
	m_search_r = calculateOptimumSearchRadius(); //estimate the search radius we want to use

	//store these info as object attributes
	updateMetadata();
}

void ccTrace::updateMetadata()
{
	QVariantMap* map = new QVariantMap();
	map->insert("ccCompassType", "Trace");
	map->insert("search_r", m_search_r);
	map->insert("cost_function", ccTrace::COST_MODE);

	//TODO - write metadata for structure normal estimates

	setMetaData(*map, true);
}

int ccTrace::insertWaypoint(int pointId)
{
	if (m_waypoints.size() >= 2)
	{
		//get location of point to add
		//const CCVector3* Q = m_cloud->getPoint(pointId);
		CCVector3 Q = *m_cloud->getPoint(pointId);
		CCVector3 start;
		CCVector3 end;
		//check if point is "inside" any segments
		for (int i = 1; i < m_waypoints.size(); i++)
		{
			//get start and end points
			m_cloud->getPoint(m_waypoints[i - 1], start);
			m_cloud->getPoint(m_waypoints[i], end);

			//are we are "inside" this segment
			if (inCircle(&start, &end, &Q))
			{
				//insert waypoint
				m_waypoints.insert(m_waypoints.begin() + i, pointId);
				m_previous.push_back(i);
				return i;
			}
		}

		//check if the point is closer to the start than the end -> i.e. the point should be 'pre-pended'
		CCVector3 sp = Q - start;
		CCVector3 ep = Q - end;
		if (sp.norm2() < ep.norm2())
		{
			m_waypoints.insert(m_waypoints.begin(), pointId);
			m_previous.push_back(0);
			return 0;
		}
	}

	//add point to end of the trace
	m_waypoints.push_back(pointId);
	int id = static_cast<int>(m_waypoints.size()) - 1;
	m_previous.push_back(id); //store for undo options
	return id;
}

//test if the query point is within a circle bound by segStart & segEnd
bool ccTrace::inCircle(const CCVector3* segStart, const CCVector3* segEnd, const CCVector3* query)
{
	//calculate vector Query->Start and Query->End
	CCVector3 QS(segStart->x - query->x, segStart->y - query->y, segStart->z - query->z);
	CCVector3 QE(segEnd->x - query->x, segEnd->y - query->y, segEnd->z - query->z);
	
	//is angle between these vectors obtuce (i.e. QS dot QE) < 0)? If so we are inside a circle between start&end, otherwise we are not
	QS.normalize();QE.normalize();

	return QS.dot(QE) < 0;
}

bool ccTrace::optimizePath(int maxIterations)
{
	bool success = true;

	if (m_waypoints.size() < 2)
	{
		m_trace.clear();
		return false; //no segments...
	}

	#ifdef DEBUG_PATH
	int idx = m_cloud->getScalarFieldIndexByName("Search"); //look for scalar field to write search to
	if (idx == -1) //doesn't exist - create
		idx=m_cloud->addScalarField("Search");
	m_cloud->setCurrentScalarField(idx);
	#endif

	//update stored cost function etc.
	updateMetadata();

	//update internal vars
	m_maxIterations = maxIterations;

	//loop through segments and build/rebuild trace
	int start = 0;
	int end = 0;
	int tID = 0;
	for (unsigned i = 1; i < m_waypoints.size(); i++)
	{
		//calculate indices
		start = m_waypoints[i - 1]; //global point id for the start waypoint
		end = m_waypoints[i]; //global point id for the end waypoint
		tID = i - 1; //id of the trace segment id (in m_trace vector)

		//are we adding to the end of the trace?
		if (tID >= m_trace.size()) 
		{
			std::deque<int> segment = optimizeSegment(start, end, m_search_r); //calculate segment
			m_trace.push_back(segment); //store segment
			success = success && !segment.empty(); //if the queue is empty, we failed
		} else //no... we're somewhere in the middle - update segment if necessary
		{
			if (!m_trace[tID].empty() && (m_trace[tID][0] == start) &&  (m_trace[tID][m_trace[tID].size() - 1] == end)) //valid trace and start/end match
				continue; //this trace has already been calculated - we can skip! :)
			else
			{
				//calculate segment
				std::deque<int> segment = optimizeSegment(start, end, m_search_r); //calculate segment
				success = success && !segment.empty(); //if the queue is empty, we failed

				//add trace
				if (m_trace[tID][m_trace[tID].size() - 1] == end) //end matches - we can replace the current trace & things should be sweet (all prior traces will have been updated already)
					m_trace[tID] = segment; //end is correct - overwrite this block, then hopefully we will match in the next one
				else //end doesn't match - we need to insert
					m_trace.insert(m_trace.begin()+tID, segment);
			}
		}
	}

	#ifdef DEBUG_PATH
	CCLib::ScalarField * f = m_cloud->getScalarField(idx);
	f->computeMinAndMax();
	#endif

	//write control points to property (for reloading)
	QVariantMap* map = new QVariantMap();
	QString waypoints = "";

	for (unsigned i = 0; i < m_waypoints.size(); i++)
	{
		waypoints += QString::number(m_waypoints[i]) + ",";
	}

	map->insert("waypoints", waypoints);
	setMetaData(*map, true);

	//push points onto underlying polyline object (for picking & save/load)
	finalizePath();

	return success;
}

void ccTrace::finalizePath()
{
	//clear existing points in background "polyline"
	clear();

	//push trace buffer to said polyline (for save/export etc.)
	for (std::deque<int> seg : m_trace)
	{
		for (int p : seg)
		{
			addPointIndex(p);
		}
	}

	//invalidate bb
	invalidateBoundingBox();
}

void ccTrace::recalculatePath()
{
	m_trace.clear();
	optimizePath();
}

int ccTrace::COST_MODE = ccTrace::MODE::DARK; //set default cost mode
std::deque<int> ccTrace::optimizeSegment(int start, int end, int offset)
{
	//check handle to point cloud
	if (!m_cloud)
	{
		return std::deque<int>(); //error -> no cloud
	}

	//retrieve and store start & end rgb
	if (m_cloud->hasColors())
	{
		const ccColor::Rgb& s = m_cloud->getPointColor(start);
		const ccColor::Rgb& e = m_cloud->getPointColor(end);
		m_start_rgb[0] = s.r; m_start_rgb[1] = s.g; m_start_rgb[2] = s.b;
		m_end_rgb[0]   = e.r; m_end_rgb[1]   = e.g; m_end_rgb[2]   = e.b;
	}
	else
	{	//no colour... set to 0 just in case something tries to use these vars
		m_start_rgb[0] = 0; m_start_rgb[1] = 0; m_start_rgb[2] = 0;
		m_end_rgb[0]   = 0; m_end_rgb[1]   = 0; m_end_rgb[2]   = 0;
	}

	//get location of target node - used to optimise algorithm to stop searching paths leading away from the target
	const CCVector3* end_v = m_cloud->getPoint(end);

	//code essentially taken from wikipedia page for Djikstra: https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
	std::vector<bool> visited; //an array of bits to check if node has been visited
	std::priority_queue<Node*,std::vector<Node*>,Compare> openQueue; //priority queue that stores nodes that haven't yet been explored/opened
	std::vector<Node*> nodes; //list of visited nodes. Used to cleanup memory after re-constructing shortest path.

	//set size of visited such that there is one bit per point in the cloud
	visited.resize(m_cloud->size(), false); //n.b. for 400 million points, this will still only be ~50Mb =)

	//declare variables used in the loop
	Node* current = nullptr;
	int current_idx = 0;
	int cost = 0;
	int smallest_cost = 999999;
	int iter_count = 0;
	float cur_d2 = 0.0f;
	float next_d2 = 0.0f;

	//setup buffer for storing nodes
	int bufferSize = 500000; //500k node buffer (~1.5-2Mb). This should be enough for most small-medium size traces. More buffers will be created for bigger ones.
	Node* node_buffer = new Node[bufferSize]; 
	nodes.push_back(node_buffer); //store buffer in nodes list (for cleanup)
	int nodeCount = 1; //start node will be added shortly

	//setup octree & values for nearest neighbour searches
	ccOctree::Shared oct = m_cloud->getOctree();
	if (!oct)
	{
		oct = m_cloud->computeOctree(); //if the user clicked "no" when asked to compute the octree then tough....
	}
	unsigned char level = oct->findBestLevelForAGivenNeighbourhoodSizeExtraction(m_search_r);

	//initialize start node on node_buffer and add to openQueue
	node_buffer[0].set(start, 0, nullptr);
	openQueue.push(&node_buffer[0]);

	//mark start node as visited
	visited[start] = true;

	while (openQueue.size() > 0) //while unvisited nodes exist
	{
		//check if we excede max iterations
		if (iter_count > m_maxIterations)
		{
			//cleanup buffers
			for (Node* n : nodes)
			{
				delete[] n;
			}

			//bail
			return std::deque<int>(); //bail
		}

		iter_count++;

		//get lowest cost node for expansion
		current = openQueue.top();
		current_idx = current->index;

		//remove node from open set
		openQueue.pop(); //remove node from open set (queue)

		if (current_idx == end) //we've found it!
		{
			std::deque<int> path;
			path.push_back(end); //add end node

			//traverse backwards to reconstruct path
			while (current->index != start)
			{
				current = current->previous;
				path.push_front(current->index);
			}

			path.push_front(start);

			//cleanup node buffers
			for (Node* n : nodes)
			{
				delete[] n;
			}

			//return
			return path;
		}

		//calculate distance from current nodes parent to end -> avoid going backwards (in euclidean space) [essentially stops fracture turning > 90 degrees)
		const CCVector3* cur = m_cloud->getPoint(current_idx);
		cur_d2 =	(cur->x - end_v->x)*(cur->x - end_v->x) +
					(cur->y - end_v->y)*(cur->y - end_v->y) +
					(cur->z - end_v->z)*(cur->z - end_v->z);

		//fill "neighbours" with nodes - essentially get results of a "sphere" search around active current point
		m_neighbours.clear();
    
		oct->getPointsInSphericalNeighbourhood(*cur, PointCoordinateType(m_search_r), m_neighbours, level);

		//loop through neighbours
		for (size_t i = 0; i < m_neighbours.size(); i++)
		{
			m_p = m_neighbours[i];
			
			if (visited[m_p.pointIndex]) //Has this node been visited before? If so then bail.
				continue;

			//calculate (squared) distance from this neighbour to the end
			next_d2 =	(m_p.point->x - end_v->x)*(m_p.point->x - end_v->x) +
						(m_p.point->y - end_v->y)*(m_p.point->y - end_v->y) +
						(m_p.point->z - end_v->z)*(m_p.point->z - end_v->z);

			if (next_d2 >= cur_d2) //Bigger than the original distance? If so then bail.
				continue;

			//calculate cost to this neighbour
			cost = getSegmentCost(current_idx, m_p.pointIndex);

			#ifdef DEBUG_PATH
			m_cloud->setPointScalarValue(m_p.pointIndex, static_cast<ScalarType>(cost)); //STORE VISITED NODES (AND COST) FOR DEBUG VISUALISATIONS
			#endif

			//transform into cost from start node
			cost += current->total_cost;

			//check that the node buffer isn't full
			if (nodeCount == bufferSize)
			{
				//buffer is full - create a new one
				node_buffer = new Node[bufferSize];
				nodes.push_back(node_buffer);
				nodeCount = 0;
			}

			//initialize node on node buffer
			node_buffer[nodeCount].set(m_p.pointIndex, cost, current);

			//push node to open set
			openQueue.push(&node_buffer[nodeCount]);

			//we now have one more node
			nodeCount++;

			//mark node as visited
			visited[m_p.pointIndex] = true;
		}
	}

	assert(false);
	return std::deque<int>(); //shouldn't come here?
}

int ccTrace::getSegmentCost(int p1, int p2)
{
	int cost=1; //n.b. default value is 1 so that if no cost functions are used, the function doesn't crash (and returns the unweighted shortest path)
	if (m_cloud->hasColors()) //check cloud has colour data
	{
		if (COST_MODE & MODE::RGB)
			cost += getSegmentCostRGB(p1, p2);
		if (COST_MODE & MODE::DARK)
			cost += getSegmentCostDark(p1, p2);
		if (COST_MODE & MODE::LIGHT)
			cost += getSegmentCostLight(p1, p2);
		if (COST_MODE & MODE::GRADIENT)
			cost += getSegmentCostGrad(p1, p2, m_search_r);
	}
	if (m_cloud->hasDisplayedScalarField()) //check cloud has scalar field data
	{
		if (COST_MODE & MODE::SCALAR)
			cost += getSegmentCostScalar(p1, p2);
		if (COST_MODE & MODE::INV_SCALAR)
			cost += getSegmentCostScalarInv(p1, p2);
	}

	//these cost functions can be used regardless
	if (COST_MODE & MODE::CURVE)
		cost += getSegmentCostCurve(p1, p2);
	if (COST_MODE & MODE::DISTANCE)
		cost += getSegmentCostDist(p1, p2);

	return cost;
}

int ccTrace::getSegmentCostRGB(int p1, int p2)
{
	//get colors
	const ccColor::Rgb& p1_rgb = m_cloud->getPointColor(p1);
	const ccColor::Rgb& p2_rgb = m_cloud->getPointColor(p2);

	//cost function: cost = |c1-c2| + 0.25 ( |c1-start| + |c1-end| + |c2-start| + |c2-end| )
	//IDEA: can we somehow optimize all the square roots here (for speed)? 
	return sqrt(
		//|c1-c2|
		(p1_rgb.r - p2_rgb.r) * (p1_rgb.r - p2_rgb.r) +
		(p1_rgb.g - p2_rgb.g) * (p1_rgb.g - p2_rgb.g) +
		(p1_rgb.b - p2_rgb.b) * (p1_rgb.b - p2_rgb.b)) + 0.25 * (
		//|c1-start|
		sqrt((p1_rgb.r - m_start_rgb[0]) * (p1_rgb.r - m_start_rgb[0]) +
		(p1_rgb.g - m_start_rgb[1]) * (p1_rgb.g - m_start_rgb[1]) +
		(p1_rgb.b - m_start_rgb[2]) * (p1_rgb.b - m_start_rgb[2])) +
		//|c1-end|
		sqrt((p1_rgb.r - m_end_rgb[0]) * (p1_rgb.r - m_end_rgb[0]) +
		(p1_rgb.g - m_end_rgb[1]) * (p1_rgb.g - m_end_rgb[1]) +
		(p1_rgb.b - m_end_rgb[2]) * (p1_rgb.b - m_end_rgb[2])) +
		//|c2-start|
		sqrt((p2_rgb.r - m_start_rgb[0]) * (p2_rgb.r - m_start_rgb[0]) +
		(p2_rgb.g - m_start_rgb[1]) * (p2_rgb.g - m_start_rgb[1]) +
		(p2_rgb.b - m_start_rgb[2]) * (p2_rgb.b - m_start_rgb[2])) +
		//|c2-end|
		sqrt((p2_rgb.r - m_end_rgb[0]) * (p2_rgb.r - m_end_rgb[0]) +
		(p2_rgb.g - m_end_rgb[1]) * (p2_rgb.g - m_end_rgb[1]) +
		(p2_rgb.b - m_end_rgb[2]) * (p2_rgb.b - m_end_rgb[2]))) / 3.5; //N.B. the divide by 3.5 scales this cost function to range between 0 & 255
}

int ccTrace::getSegmentCostDark(int p1, int p2)
{
	//return magnitude of the point p2
	//const ColorCompType* p1_rgb = m_cloud->getPointColor(p1);
	const ccColor::Rgb& p2_rgb = m_cloud->getPointColor(p2);

	//return "taxi-cab" length
	return (p2_rgb.r + p2_rgb.g + p2_rgb.b); //note: this will naturally give a maximum of 765 (=255 + 255 + 255)
}

int ccTrace::getSegmentCostLight(int p1, int p2)
{
	//return the opposite of getCostDark
	return 765 - getSegmentCostDark(p1, p2);
}

int ccTrace::getSegmentCostCurve(int p1, int p2)
{
	int idx = m_cloud->getScalarFieldIndexByName("Curvature"); //look for pre-existing gradient SF
	if (isCurvaturePrecomputed()) //scalar field found - return from precomputed cost]
	{
		//activate SF
		m_cloud->setCurrentScalarField(idx);

		//return inverse of p2 value
		return m_cloud->getScalarField(idx)->getMax() - m_cloud->getPointScalarValue(p2);
	}
	else //scalar field not found - do slow calculation...
	{
		//put neighbourhood in a CCLib::Neighbourhood structure
		if (m_neighbours.size() > 4) //need at least 4 points to calculate curvature....
		{
			m_neighbours.push_back(m_p); //add center point onto end of neighbourhood

			//compute curvature
			CCLib::DgmOctreeReferenceCloud nCloud(&m_neighbours, static_cast<unsigned>(m_neighbours.size()));
			CCLib::Neighbourhood Z(&nCloud);
			float c = Z.computeCurvature(*nCloud.getPoint(0), CCLib::Neighbourhood::CurvatureType::MEAN_CURV);

			m_neighbours.erase(m_neighbours.end() - 1); //remove center point from neighbourhood (so as not to screw up loops)

			//curvature tends to range between 0 (high cost) and 10 (low cost), though it can be greater than 10 in extreme cases
			//hence we need to map to domain 0 - 10 and then transform that to the (integer) domain 0 - 884 to meet the cost function spec
			if (c > 10)
				c = 10;

			//scale curvature to range 0, 765
			c *= 76.5;

			//note that high curvature = low cost, hence subtract curvature from 765
			return 765 - c;

		}
		return 765; //unknown curvature - this point is high cost.
	}
}

int ccTrace::getSegmentCostGrad(int p1, int p2, float search_r)
{
	int idx = m_cloud->getScalarFieldIndexByName("Gradient"); //look for pre-existing gradient SF
	if (idx != -1) //found precomputed gradient
	{
		//activate SF
		m_cloud->setCurrentScalarField(idx);

		//return inverse of p2 value
		return m_cloud->getScalarField(idx)->getMax() - m_cloud->getPointScalarValue(p2);
	}
	else //not found... do expensive calculation
	{

		CCVector3 p = *m_cloud->getPoint(p2);
		const ccColor::Rgb p2_rgb = m_cloud->getPointColor(p2);
		int p_value = p2_rgb.r + p2_rgb.g + p2_rgb.b;

		if (m_neighbours.size() > 2) //need at least 2 points to calculate gradient....
		{
			//N.B. The following code is mostly stolen from the computeGradient function in CloudCompare
			CCVector3d sum(0, 0, 0);
			CCLib::DgmOctree::PointDescriptor n;
			for (int i = 0; i < m_neighbours.size(); i++)
			{
				n = m_neighbours[i];

				//vector from p1 to m_p
				CCVector3 deltaPos = *n.point - p;
				double norm2 = deltaPos.norm2d();

				//colour
				const ccColor::Rgb& c = m_cloud->getPointColor(n.pointIndex);
				int c_value = (static_cast<int>(c.r) + c.g) + c.b;

				//calculate gradient weighted by distance to the point (i.e. divide by distance^2)
				if (norm2 > ZERO_TOLERANCE)
				{
					//color magnitude difference
					int deltaValue = p_value - c_value;
					//divide by norm^2 to get distance-weighted gradient
					deltaValue /= norm2; //we divide by 'norm' to get the normalized direction, and by 'norm' again to get the gradient (hence we use the squared norm)
					//sum stuff
					sum.x += deltaPos.x * deltaValue; //warning: here 'deltaValue'= deltaValue / squaredNorm(deltaPos) ;)
					sum.y += deltaPos.y * deltaValue;
					sum.z += deltaPos.z * deltaValue;
				}
			}

			float gradient = sum.norm() / m_neighbours.size();

			//ensure gradient is lass than a case-specific maximum gradient (colour change from white to black across a distance or search_r,
			//                                                                                  giving a gradient of (255+255+255) / search_r)
			gradient = std::min(gradient, 765 / search_r);
			gradient *= search_r; //scale between 0 and 765
			return 765 - gradient; //return inverse gradient (high gradient = low cost)
		}
		return 765; //no gradient = high cost
	}
}

int ccTrace::getSegmentCostDist(int p1, int p2)
{
	return 255;
}

int ccTrace::getSegmentCostScalar(int p1, int p2)
{
	//m_cloud->getCurrentDisplayedScalarFieldIndex();
	ccScalarField* sf = static_cast<ccScalarField*>(m_cloud->getCurrentDisplayedScalarField());
	return (sf->getValue(p2)-sf->getMin()) * (765 / (sf->getMax()-sf->getMin())); //return scalar field value mapped to range 0 - 765
}

int ccTrace::getSegmentCostScalarInv(int p1, int p2)
{
	ccScalarField* sf = static_cast<ccScalarField*>(m_cloud->getCurrentDisplayedScalarField());
	return (sf->getMax() - sf->getValue(p2)) * (765 / (sf->getMax() - sf->getMin())); //return inverted scalar field value mapped to range 0 - 765
}


//functions for calculating cost SFs
void ccTrace::buildGradientCost(QWidget* parent)
{
	//is there already a gradient SF?
	if (isGradientPrecomputed()) //already done :)
		return;

	//create new SF for greyscale
	int idx = m_cloud->addScalarField("Greyscale");
	m_cloud->setCurrentScalarField(idx);

	//make colours greyscale and push to SF (otherwise copy active SF)
	for (unsigned i = 0; i < m_cloud->size(); i++)
	{
		const ccColor::Rgb& col = m_cloud->getPointColor(i);
		m_cloud->setPointScalarValue(i, static_cast<ScalarType>((static_cast<int>(col.r) + col.g) + col.b));
	}
	//compute min/max
	m_cloud->getScalarField(idx)->computeMinAndMax();

	//calculate new SF with gradient of the old one
	float roughnessKernelSize = m_search_r;
	m_cloud->setCurrentOutScalarField(idx); //set out gradient field - values are read from here for calc

	//make gradient sf
	int gIdx = m_cloud->addScalarField("Gradient");
	m_cloud->setCurrentInScalarField(gIdx); //set in scalar field - gradient is written here

	//get/build octree
	ccProgressDialog pDlg(true, parent);
	pDlg.show();

	ccOctree::Shared octree = m_cloud->getOctree();
	if (!octree)
	{
		octree = m_cloud->computeOctree(&pDlg);
		if (!octree)
		{
			ccLog::Error("Failed to compute octree");
			return;
		}
	}

	//calculate gradient
	int result = CCLib::ScalarFieldTools::computeScalarFieldGradient(m_cloud,
		m_search_r, //auto --> FIXME: should be properly set by the user!
		false,
		false,
		&pDlg,
		octree.data());

	pDlg.close();

	if (result != 0)
	{
		m_cloud->deleteScalarField(gIdx);
		ccLog::Warning("Failed to compute the scalar field gradient");
		return;
	}

	//calculate bounds
	m_cloud->getScalarField(gIdx)->computeMinAndMax();
	
	//normalize and log-transform
	m_cloud->setCurrentScalarField(gIdx);
	float logMax = log(m_cloud->getScalarField(gIdx)->getMax() + 10);
	for (unsigned i = 0; i < m_cloud->size(); i++)
	{
		int nVal = 765 * log(m_cloud->getPointScalarValue(i) + 10) / logMax;
		if (nVal < 0) //this is caused by isolated points that were assigned "null" value gradients
			nVal = 1; //set to low gradient by default
		m_cloud->setPointScalarValue(i, nVal);
	}

	//recompute min-max...
	m_cloud->getScalarField(gIdx)->computeMinAndMax();
}

void ccTrace::buildCurvatureCost(QWidget* parent)
{
	if (isCurvaturePrecomputed()) //already done
		return;

	//create curvature SF
	//make gradient sf
	int idx = m_cloud->addScalarField("Curvature");
	m_cloud->setCurrentInScalarField(idx); //set in scalar field - curvature is written here
	m_cloud->setCurrentScalarField(idx);

	//get/build octree
	ccProgressDialog pDlg(true, parent);
	pDlg.show();

	ccOctree::Shared octree = m_cloud->getOctree();
	if (!octree)
	{
		octree = m_cloud->computeOctree(&pDlg);
	}

	//calculate curvature
	CCLib::GeometricalAnalysisTools::ErrorCode result = CCLib::GeometricalAnalysisTools::ComputeCharactersitic(
		CCLib::GeometricalAnalysisTools::Curvature,
		CCLib::Neighbourhood::CurvatureType::MEAN_CURV,
		m_cloud,
		m_search_r,
		&pDlg,
		octree.data());

	pDlg.close();

	if (result != CCLib::GeometricalAnalysisTools::NoError)
	{
		m_cloud->deleteScalarField(idx);
		ccLog::Warning("Failed to compute the curvature");
		return;
	}

	//calculate minmax
	m_cloud->getScalarField(idx)->computeMinAndMax();

	//normalize and log-transform
	float logMax = log(m_cloud->getScalarField(idx)->getMax() + 10);
	for (unsigned i = 0; i < m_cloud->size(); i++)
	{
		int nVal = 765 * log(m_cloud->getPointScalarValue(i) + 10) / logMax;
		if (nVal < 0) //this is caused by isolated points that were assigned "null" value curvatures
			nVal = 1; //set to low gradient by default
		m_cloud->setPointScalarValue(i, nVal);
	}

	//recompute min-max...
	m_cloud->getScalarField(idx)->computeMinAndMax();
}

bool ccTrace::isGradientPrecomputed()
{
	int idx = m_cloud->getScalarFieldIndexByName("Gradient"); //look for pre-existing gradient SF
	return idx != -1; //was something found?
}
bool ccTrace::isCurvaturePrecomputed()
{
	int idx = m_cloud->getScalarFieldIndexByName("Curvature"); //look for pre-existing gradient SF
	return idx != -1; //was something found?
}

ccFitPlane* ccTrace::fitPlane(int surface_effect_tolerance, float min_planarity)
{
	//put all "trace" points into the cloud
	finalizePath();
	
	if (size() < 3)
		return nullptr; //need three points to fit a plane

	//check we are not trying to fit a plane to a line
	CCLib::Neighbourhood Z(this);

	//calculate eigenvalues of neighbourhood
	CCLib::SquareMatrixd cov = Z.computeCovarianceMatrix();
	CCLib::SquareMatrixd eigVectors; std::vector<double> eigValues;
	if (Jacobi<double>::ComputeEigenValuesAndVectors(cov, eigVectors, eigValues, true))
	{
		//sort eigenvalues into decending order
		std::sort(eigValues.rbegin(), eigValues.rend());

		float y = eigValues[1]; //middle eigen
		float z = eigValues[2]; //smallest eigen (parallel to plane normal)

		//calculate planarity (0 = line or random, 1 = plane)
		float planarity = 1.0f - z / y;
		if (planarity < min_planarity)
		{
			return nullptr;
		}
	}

	//fit plane
	double rms = 0.0; //output for rms
	ccFitPlane* p = ccFitPlane::Fit(this, &rms);

	if (!p)
	{
		return nullptr; //return null for invalid planes
	}

	p->updateAttributes(rms, m_search_r);

	//test for 'surface effect'
	if (m_cloud->hasNormals())
	{
		//calculate average normal of points on trace
		CCVector3 n_avg;
		for (unsigned i = 0; i < size(); i++)
		{
			//get normal vector
			CCVector3 n = ccNormalVectors::GetNormal(m_cloud->getPointNormalIndex(this->getPointGlobalIndex(i)));
			n_avg += n;
		}
		n_avg *= (PC_ONE / size()); //turn sum into average


		//compare to plane normal
		CCVector3 n = p->getNormal();
		if (acos(n_avg.dot(n)) < 0.01745329251*surface_effect_tolerance) //0.01745329251 converts deg to rad
		{
			//this is a false (surface) plane - reject
			return nullptr; //don't return plane
		}
	}

	//all is good! Return the plane :)
	return p;
}

void ccTrace::bakePathToScalarField()
{
	//bake points
	int vertexCount = static_cast<int>(m_cloud->size());
	for (const std::deque<int>& seg : m_trace)
	{
		for (int p : seg)
		{
			if (p >= 0 && p < vertexCount)
			{
				m_cloud->setPointScalarValue(static_cast<unsigned>(p), getUniqueID());
			}
		}
	}
}

float ccTrace::calculateOptimumSearchRadius()
{
	CCLib::DgmOctree::NeighboursSet neighbours;

	//setup octree & values for nearest neighbour searches
	ccOctree::Shared oct = m_cloud->getOctree();
	if (!oct)
	{
		oct = m_cloud->computeOctree(); //if the user clicked "no" when asked to compute the octree then tough....
	}

	//init vars needed for nearest neighbour search
	unsigned char level = oct->findBestLevelForAGivenPopulationPerCell(2);
	CCLib::ReferenceCloud* nCloud = new  CCLib::ReferenceCloud(m_cloud);

	//pick 15 random points
	unsigned int npoints = m_cloud->size();
	double dsum = 0;
	srand(npoints); //set seed as n for repeatability
	for (unsigned int i = 0; i < 30; i++)
	{
		//int rn = rand() * rand(); //need to make bigger than rand max...
		int r = (rand()*rand()) % npoints; //random(ish) number between 0 and n

		//find nearest neighbour for point
		nCloud->clear(false);
		double d = -1.0;
		oct->findPointNeighbourhood(m_cloud->getPoint(r), nCloud, 2, level, d);

		if (d != -1.0) //if a point was found
		{
			dsum += sqrt(d);
		}
	}
	
	//average nearest-neighbour distances
	double d = dsum / 30;
	
	//return a number slightly larger than the average distance
	return d * 1.5;
}

static QSharedPointer<ccSphere> c_unitPointMarker(nullptr);
void ccTrace::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (!MACRO_Foreground(context)) //2D foreground only
		return; //do nothing

	if (MACRO_Draw3D(context))
	{
		if (m_waypoints.empty()) //no points -> bail!
			return;

		//get the set of OpenGL functions (version 2.1)
		QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
		if (glFunc == nullptr) {
			assert(false);
			return;
		}

		//check sphere exists
		if (!c_unitPointMarker)
		{
			c_unitPointMarker = QSharedPointer<ccSphere>(new ccSphere(1.0f, nullptr, "PointMarker", 6));

			c_unitPointMarker->showColors(true);
			c_unitPointMarker->setVisible(true);
			c_unitPointMarker->setEnabled(true);
			c_unitPointMarker->showNormals(false);
		}

		glDrawParams glParams;
		getDrawingParameters(glParams);

		//not sure what this does, but it looks like fun
		CC_DRAW_CONTEXT markerContext = context; //build-up point maker own 'context'
		markerContext.display = nullptr;
		markerContext.drawingFlags &= (~CC_DRAW_ENTITY_NAMES); //we must remove the 'push name flag' so that the sphere doesn't push its own!

		//get camera info
		ccGLCameraParameters camera;
		glFunc->glGetIntegerv(GL_VIEWPORT, camera.viewport);
		glFunc->glGetDoublev(GL_PROJECTION_MATRIX, camera.projectionMat.data());
		glFunc->glGetDoublev(GL_MODELVIEW_MATRIX, camera.modelViewMat.data());

		const ccViewportParameters& viewportParams = context.display->getViewportParameters();

		//push name for picking
		bool pushName = MACRO_DrawEntityNames(context);
		if (pushName)
		{
			glFunc->glPushName(getUniqueIDForDisplay());
			//minimal display for picking mode!
			glParams.showNorms = false;
			glParams.showColors = false;
		}

		//set draw colour
		ccColor::Rgb color = getMeasurementColour();
		c_unitPointMarker->setTempColor(color);

		//get point size for drawing
		float pSize;
		glFunc->glGetFloatv(GL_POINT_SIZE, &pSize);

		//draw key-points and structure normals (if assigned)
		if (m_isActive)
		{
			for (size_t i = 0; i < m_waypoints.size(); i++)
			{
				glFunc->glMatrixMode(GL_MODELVIEW);
				glFunc->glPushMatrix();

				const CCVector3* P = m_cloud->getPoint(m_waypoints[i]);
				ccGL::Translate(glFunc, P->x, P->y, P->z);
				float scale = context.labelMarkerSize * m_relMarkerScale * 0.3 * fmin(pSize, 4);
				if (viewportParams.perspectiveView && viewportParams.zFar > 0)
				{
					//in perspective view, the actual scale depends on the distance to the camera!
					double d = (camera.modelViewMat * CCVector3d::fromArray(P->u)).norm();
					double unitD = viewportParams.zFar / 2; //we consider that the 'standard' scale is at half the depth
					scale = static_cast<float>(scale * sqrt(d / unitD)); //sqrt = empirical (probably because the marker size is already partly compensated by ccGLWindow::computeActualPixelSize())
				}
				glFunc->glScalef(scale, scale, scale);
				c_unitPointMarker->draw(markerContext);
				glFunc->glPopMatrix();
			}
		}
		else //just draw lines
		{
			//draw lines
			for (const std::deque<int>& seg : m_trace)
			{
				if (m_width != 0)
				{
					glFunc->glPushAttrib(GL_LINE_BIT);
					glFunc->glLineWidth(static_cast<GLfloat>(m_width));
				}
				glFunc->glBegin(GL_LINE_STRIP);
				glFunc->glColor3f(color.r, color.g, color.b);
				for (int p : seg)
				{
					ccGL::Vertex3v(glFunc, m_cloud->getPoint(p)->u);
				}
				glFunc->glEnd();
				if (m_width != 0)
				{
					glFunc->glPopAttrib();
				}
			}
		}

		//draw trace points if trace is active OR point size is large (otherwise line gets hidden)
		if (m_isActive || pSize > 8)
		{

			for (const std::deque<int>& seg : m_trace)
			{
				for (int p : seg)
				{
					glFunc->glMatrixMode(GL_MODELVIEW);
					glFunc->glPushMatrix();

					const CCVector3* P = m_cloud->getPoint(p);
					ccGL::Translate(glFunc, P->x, P->y, P->z);
					float scale = context.labelMarkerSize * m_relMarkerScale * fmin(pSize, 4) * 0.2;
					if (viewportParams.perspectiveView && viewportParams.zFar > 0)
					{
						//in perspective view, the actual scale depends on the distance to the camera!
						const double* M = camera.modelViewMat.data();
						double d = (camera.modelViewMat * CCVector3d::fromArray(P->u)).norm();
						double unitD = viewportParams.zFar / 2; //we consider that the 'standard' scale is at half the depth
						scale = static_cast<float>(scale * sqrt(d / unitD)); //sqrt = empirical (probably because the marker size is already partly compensated by ccGLWindow::computeActualPixelSize())
					}

					glFunc->glScalef(scale, scale, scale);
					c_unitPointMarker->draw(markerContext);
					glFunc->glPopMatrix();
				}
			}
		}

		//finish picking name
		if (pushName)
			glFunc->glPopName();
	}
}

bool ccTrace::isTrace(ccHObject* object) //return true if object is a valid trace [regardless of it's class type]
{
	if (object->hasMetaData("ccCompassType"))
	{
		return object->getMetaData("ccCompassType").toString().contains("Trace");
	}
	return false;
}
