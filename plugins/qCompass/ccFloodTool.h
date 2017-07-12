#ifndef CC_PAINTTOOL_HEADER
#define CC_PAINTTOOL_HEADER

#include <ccHObject.h>
#include <ccPointCloud.h>
#include <ccMainAppInterface.h>
#include <ccGLWindow.h>

#include "ccTrace.h"
#include "ccTool.h"

class ccFloodTool :
	public ccTool
{
public:

	ccFloodTool();
	~ccFloodTool();

	//called when a point in a point cloud gets picked while this tool is active
	virtual void pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P);

	//called when "Return" or "Space" is pressed, or the "Accept Button" is clicked
	virtual void accept();

	//called when the "Escape" is pressed, or the "Cancel" button is clicked
	virtual void cancel();

private:
	void bakeToActiveCloud();
	void recurseBake(ccHObject* parent);
	void flood(int pID);

	//calculate the search radius that should be used for the shortest path calcs
	float calculateOptimumSearchRadius();

	ccPointCloud* m_cloud;
	int m_seedRGB[3];
	float m_search_r;
	ccOctree::Shared m_oct;
	unsigned char m_level;
	CCLib::DgmOctree::PointDescriptor m_p;
	int recurseCount=0;
	int m_boundary_sf_id;

//static consts
public:
	static int DISTANCE_THRESHOLD;
	static int COLOUR_THRESHOLD;
	static int MAX_ITERATIONS;

	//called when the tool is set to active (for initialization)
	virtual void toolActivated();

	//called when the tool is set to disactive (for cleanup)
	//virtual void toolDisactivated() { };

	//called when a point in a point cloud gets picked while this tool is active
	//virtual void pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccHObject* pickedObject, const CCVector3& P);



	//called when the selection is changed while this tool is active
	//virtual void onNewSelection(const ccHObject::Container& selectedEntities) { };
};

#endif