#include "ccFloodTool.h"

int ccFloodTool::COLOUR_THRESHOLD = 32; //n.b. for performance this is distance^2, hence the 32 * 32
int ccFloodTool::DISTANCE_THRESHOLD = 500; //units here are the average point resolution
int ccFloodTool::MAX_ITERATIONS = 10000;

ccFloodTool::ccFloodTool()
	: ccTool()
{
	
}

ccFloodTool::~ccFloodTool()
{

}

void ccFloodTool::toolActivated()
{
	
}

void ccFloodTool::bakeToActiveCloud()
{
	if (!m_cloud)
		return;

	//"bake" all traces to a scalar field
	int idx = m_cloud->getScalarFieldIndexByName("Boundaries"); //find a scalar field called "Boundaries"
	if (idx != -1) //already exists? delete it - we want to start from a clean slate.
	{
		m_cloud->deleteScalarField(idx);
	}

	m_boundary_sf_id = m_cloud->addScalarField("Boundaries"); //create "Boundaries" SF
	m_cloud->setCurrentScalarField(m_boundary_sf_id); //set active

	//recursively bake any ccTrace's in the DB tree
	recurseBake(m_app->dbRootObject());

	//calculate max/min of sf
	CCLib::ScalarField * f = m_cloud->getScalarField(m_boundary_sf_id);
	f->computeMinAndMax();
}
void ccFloodTool::recurseBake(ccHObject* parent)
{
	//is this a ccTrace?
	if (ccTrace::isTrace(parent))
	{
		ccTrace* trc = dynamic_cast<ccTrace*>(parent);
		if (trc)
		{
			trc->bakePathToScalarField();
		}
	}

	//recurse on children
	for (int i = 0; i < parent->getChildrenNumber(); i++)
	{
		recurseBake(parent->getChild(i));
	}
}

//called when a point in a point cloud gets picked while this tool is active
void ccFloodTool::pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P)
{
	//do flood fill
	if (!cloud)
	{
		return; //getta outta 'ere
	}

	//is this a new cloud?
	if (m_cloud != cloud)
	{
		m_cloud = cloud;
		bakeToActiveCloud(); //bake traces to this cloud
		m_search_r = calculateOptimumSearchRadius(); //calculate search radius for this cloud
	}

	//setup scalar field
	int idx = m_cloud->getScalarFieldIndexByName("Flood"); //look for scalar field to write search to
	if (idx == -1) //doesn't exist - create
	{
		idx = m_cloud->addScalarField("Flood");
	}

	//activate scalar field
	m_cloud->setCurrentScalarField(idx);

	//get initial RGB
	const ColorCompType* s = m_cloud->getPointColor(itemIdx);
	m_seedRGB[0] = s[0]; m_seedRGB[1] = s[1]; m_seedRGB[2] = s[2];

	//get octree
	//setup octree & values for nearest neighbour searches
	m_oct = m_cloud->getOctree();
	if (!oct)
	{
		m_oct = m_cloud->computeOctree(); //if the user clicked "no" when asked to compute the octree then tough....
	}

	m_level = m_oct->findBestLevelForAGivenNeighbourhoodSizeExtraction(m_search_r);

	recurseCount = 0;

	//run flood fill
	flood(itemIdx);

	CCLib::ScalarField * f = m_cloud->getScalarField(idx);
	f->computeMinAndMax();
}

void ccFloodTool::flood(int pID)
{
	//emergency bail
	if (recurseCount > ccFloodTool::MAX_ITERATIONS)
		return;
	recurseCount++;

	//add this point
	m_cloud->setPointScalarValue(pID, static_cast<ScalarType>(100)); //store this node

	//get this points location
	const CCVector3* cur = m_cloud->getPoint(pID);
	//const ColorCompType* p1_rgb = m_cloud->getPointColor(pID);

	//find children nodes

	CCLib::DgmOctree::NeighboursSet m_neighbours;
	int n = m_oct->getPointsInSphericalNeighbourhood(*cur, PointCoordinateType(m_search_r), m_neighbours, m_level);

	//is one of the neighbours in the boundaries set?
	for (size_t i = 0; i < m_neighbours.size(); i++)
	{
		if (m_cloud->getPointScalarValue(m_boundary_sf_id) != 0) //i.e. this is in the boundary set
		{
			return; //we stop here.
		}
	}

	//loop through neighbours (again)
	for (size_t i = 0; i < m_neighbours.size(); i++)
	{
		m_p = m_neighbours[i];

		//check neighbour not already visited/set
		if (m_cloud->getPointScalarValue(m_p.pointIndex) != 0)
			continue; //goto next

		//get neighbour colour
		const ColorCompType* p2_rgb = m_cloud->getPointColor(m_p.pointIndex);

		//calculate distance^2 in colour space
		int distance = (m_seedRGB[0] - p2_rgb[0]) * (m_seedRGB[0] - p2_rgb[0]) +
			           (m_seedRGB[1] - p2_rgb[1]) * (m_seedRGB[1] - p2_rgb[1]) +
			           (m_seedRGB[2] - p2_rgb[2]) * (m_seedRGB[2] - p2_rgb[2]);

		//less than min colour distance?
		if (distance < ccFloodTool::COLOUR_THRESHOLD*ccFloodTool::COLOUR_THRESHOLD)
		{
			//less than min euclidean distance?
			if (m_p.squareDistd < (ccFloodTool::DISTANCE_THRESHOLD * m_search_r)*(ccFloodTool::DISTANCE_THRESHOLD * m_search_r))
			{
				flood(m_p.pointIndex); //recurse
			}
		}
	}
}

//called when "Return" or "Space" is pressed, or the "Accept Button" is clicked
void ccFloodTool::accept()
{

}

//called when the "Escape" is pressed, or the "Cancel" button is clicked
void ccFloodTool::cancel()
{

}

float ccFloodTool::calculateOptimumSearchRadius()
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
	int r;
	unsigned int npoints = m_cloud->size();
	double d, dsum = 0;
	srand(npoints); //set seed as n for repeatability
	for (unsigned int i = 0; i < 30; i++)
	{
		//int rn = rand() * rand(); //need to make bigger than rand max...
		r = (rand()*rand()) % npoints; //random(ish) number between 0 and n

		//find nearest neighbour for point
		nCloud->clear(false);
		int n = oct->findPointNeighbourhood(m_cloud->getPoint(r), nCloud, 2, level, d);

		if (d != -1) //if a point was found
		{
			dsum += sqrt(d);
		}
	}

	//average nearest-neighbour distances
	d = dsum / 30;

	//return a number slightly larger than the average distance
	return d*1.5;
}
