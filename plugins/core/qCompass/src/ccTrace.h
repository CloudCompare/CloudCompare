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

#ifndef CC_TRACE_HEADER
#define CC_TRACE_HEADER

//#define DEBUG_PATH   //n.b. uncomment this to write point-costs to a scalar field for debug purposes

#include <ccHObject.h>
#include <ccPolyline.h>

#include "ccMeasurement.h"

#include <ccSphere.h>
#include <DgmOctree.h>
#include <DgmOctreeReferenceCloud.h>
#include <GenericIndexedCloudPersist.h>
#include <ccPointCloud.h>
#include <ccColorTypes.h>
#include <Neighbourhood.h>
#include <ccPlane.h>
#include <Jacobi.h>
#include <ccScalarField.h>
#include <ccProgressDialog.h>
#include <ScalarFieldTools.h>

#include "ccFitPlane.h"

#include <vector>
#include <algorithm>
#include <unordered_map>
#include <deque>
#include <qmessagebox.h>

/*
A ccTrace object is essentially a ccPolyline that is controlled/created by "waypoints" and a least-cost path algorithm
designe to pick fracture traces and lithological contacts (i.e. follow edges or linear features in RGB space). 

If treated as a ccPolyline, it will behave as a conventional polyline going through a series of points (including intermediate
points between the waypoints). 

If treated as a ccTrace object, then the waypoints can be manipulated and the underlying ccPolyline recalculated. For convenience, 
the waypoints are also drawn as bubbles.

*/
class ccTrace : 
	public ccPolyline, 
	public ccMeasurement
{
public:
	ccTrace(ccPointCloud* associatedCloud);
	ccTrace(ccPolyline* obj); //used for constructing from polylines with the correct metadata
	virtual ~ccTrace() {}

	//inherited from ccHObject
	inline virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::POLY_LINE; }

	/*
	Adds waypoint to the end of this trace.
	@Args
	 *pointIndex* = the index of the point (in the cloud associated with this object) that is the new waypoint
	*/
	void pushWaypoint(int pointId) { m_waypoints.push_back(pointId); }

	/*
	Deletes the specified waypoint.
	@Args
	*pointIndex* = the index of the point (in the cloud associated with this object) that represents the waypoint to be deleted. If this point is not
	               a waypoint then the function does nothing.
	*/
	void deleteWaypoint(int pointId) { 
		m_waypoints.erase(std::remove(m_waypoints.begin(), m_waypoints.end(), pointId), m_waypoints.end()); 
	}

	/*
	Retrieves the global point index of the n'th waypoint
	*/
	int getWaypoint(int n)
	{
		return m_waypoints[n];
	}

	/*
	Tries to insert/append the given waypoint based on its location. If the point is within a "circle" including the start and end of the closest segment
	then it "inserts" the point. Otherwise the point is appended onto the end of the trace.

	Returns True if the point new waypoint was successfully added/inserted and the path successfully updated.
	*/
	int insertWaypoint(int pointId);

	/*
	Returns the normal of the point at the specified index in this trace. If no normal exists, it returns the vector 0,0,0.
	*/
	CCVector3f getPointNormal(int pointIdx)
	{
		if (!m_cloud->hasNormals())
		{
			return CCVector3f(0, 0, 0);
		}

		return m_cloud->getPointNormal(getPointGlobalIndex(pointIdx));
	}

	/*
	Undo last action
	*/
	void undoLast()
	{
		if (!m_previous.empty())
		{
			m_waypoints.erase(m_waypoints.begin() + m_previous.at(m_previous.size() - 1));
			m_trace.clear(); //need to recalculate whole trace
			m_previous.pop_back(); //remove waypoint
		}
	}

	size_t waypoint_count() const { return m_waypoints.size(); }

	/*
	Calculates the most "structure-like" path between each waypoint using the A* least cost path algorithm. Can be expensive...

	@Args
	*maxIterations* = the maximum number of search iterations that are run before the algorithm gives up. Default is lots.

	@Returns
	  -true if an optimal path was successfully identified
	  -false if the a path could not be found (iterations > maxIterations)
	*/
	bool optimizePath(int maxIterations = 1000000);

	/*
	Applies the optimized path to the underlying polyline object (allows saving etc.).
	*/
	void finalizePath();

	/*
	Recalculates the path from scratch
	*/
	void recalculatePath();


	/*
	Fit a plane to this trace. Returns true if a plane was fit successfully.

	@Args
	*surface_effect_tolerance* = the minimum allowable angle (in degrees) between the average surface normals of the points [if they exist] and the plane. This gets rid of fit planes that simply represent the outcrop surface.
	*min_planarity* = the minimum allowable planarity (where a perfect plane = 1, while a line or random point cloud = 0)

	@Return
	The plane that was fitted, or a null pointer (0) if the plane is not considered valid (by the above criterion)
	*/
	ccFitPlane* fitPlane(int surface_effect_tolerance = 10, float min_planarity = 0.75f);

	/*
	Assigns the ID of this object to active scalar field (for each trace point).
	*/
	void bakePathToScalarField();

	/*
	Get the edge cost of going from p1 to p2 (this containts the "cost function" to define what is "fracture like" and what is not)
	*/
	int getSegmentCost(int p1, int p2);

	//functions for calculating cost SFs
	void buildGradientCost(QWidget* parent);
	void buildCurvatureCost(QWidget* parent);
	bool isGradientPrecomputed();
	bool isCurvaturePrecomputed();

	enum MODE 
	{
		RGB = 1, //default: cost function minimises local colour gradient & difference to start/end points
		LIGHT = 2, //light points are low cost
		DARK = 4, //dark points are low cost
		CURVE = 8, //points with high curvature are low cost
		GRADIENT = 16, //points with high gradient are low cost
		DISTANCE = 32, //cost is euclidean distance
		SCALAR = 64,
		INV_SCALAR = 128
	};

	static int COST_MODE;

protected:
	//overidden from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;

	/*
	Gets the closest waypoint to the point described by pID.
	*/
	int getClosestWaypoint(int pointID);

	//contains grunt of shortest path algorithm. "offset" inserts points at the specified distance from the END of the trace (used for updating)
	std::deque<int> optimizeSegment(int start, int end, int offset=0);

	//specific cost algorithms (getSegmentCost(...) sums combinations of these depending on the COST_MODE flag.
	//NOTE: to ensure each cost function makes an equal contribution to the result (when multiples are being used), each
	//      returns a value between 0 and 765 (the maximum  r + g + bvalue), with the exception of 
	//      getSegmentCostDist(...) which just returns a constant value (255), meaning it will find the least number of points
	//      between start and end (equal to the euclidean shortest path assuming point density is more or less constant).
	int getSegmentCostRGB(int p1, int p2);
	int getSegmentCostDark(int p1, int p2);
	int getSegmentCostLight(int p1, int p2);
	int getSegmentCostCurve(int p1, int p2);
	int getSegmentCostGrad(int p1, int p2, float search_r);
	int getSegmentCostDist(int p1, int p2);
	int getSegmentCostScalar(int p1, int p2);
	int getSegmentCostScalarInv(int p1, int p2);

	//calculate the search radius that should be used for the shortest path calcs
	float calculateOptimumSearchRadius();

	//ccTrace variables
	float m_relMarkerScale = 1.0f;
	ccPointCloud* m_cloud=0; //pointer to ccPointCloud object this is linked to (slightly different to polylines as we know this data is sampled from a real cloud)

	std::vector<std::deque<int>> m_trace; //contains an ordered list of indices which define this trace. Note that indices representing nodes MAY be inserted twice.
	std::vector<int> m_waypoints; //list of waypoint indices
	std::vector<int> m_previous; //for undoing waypoints
private:

	//class for storing point index & path costs (from the path start) in sorted lists
	class Node
	{
	public:

		void set(int node_index, int node_total_cost, Node* prev_node)
		{
			index = node_index;
			total_cost = node_total_cost;
			previous = prev_node;
		}

		int index=-1;
		int total_cost=0;
		Node* previous=nullptr;
	};

	//class for comparing Node pointers in priority_queue
	class Compare
	{
	public:
		bool operator() (Node* t1, Node* t2)
		{
			//n.b. the priority queue puts "higher" priorities at the front of the queue.
			//in this case, lower total_cost = "higher priority"
			//hence we compare total_cost with the > operator
			//i.e. t1 is less important than t2 if this.total_cost > t1.total_cost 
			return t1->total_cost > t2->total_cost; //compare based on cost
		}
	};

	//random vars that we keep to optimise speed
	int m_start_rgb[3];
	int m_end_rgb[3]; //[r,g,b] values for start and end nodes
	CCLib::DgmOctree::NeighboursSet m_neighbours;
	CCLib::DgmOctree::PointDescriptor m_p;
	float m_search_r;
	float m_maxIterations;

	/*
	Test if a point falls within a circle who's diameter equals the line from segStart to segEnd. This is used to test if a newly added point should be
	(1) appended to the end of the trace [falls outside of all segment-circles], or (2) inserted to split a segment [falls into a segment-circle]
	*/
	bool inCircle(const CCVector3* segStart, const CCVector3* segEnd, const CCVector3* query);

	//used by various constructors to do initialization
	void init(ccPointCloud* associatedCloud);

	//used to update metadata flags
	void updateMetadata();

//static functions
public:
	static bool isTrace(ccHObject* object); //return true if object is a valid trace [regardless of it's class type]
};

#endif
