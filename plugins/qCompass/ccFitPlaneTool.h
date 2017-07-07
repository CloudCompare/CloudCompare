#ifndef CC_FITPLANETOOL_HEADER
#define CC_FITPLANETOOL_HEADER

#include <DgmOctreeReferenceCloud.h>

#include "cctool.h"
#include "ccMouseCircle.h"
#include "ccFitPlane.h"

class ccFitPlaneTool :
	public ccTool
{
public:
	ccFitPlaneTool();
	~ccFitPlaneTool();

	//called when the tool is set to active (for initialization)
	void toolActivated() override;

	//called when the tool is set to disactive (for cleanup)
	void toolDisactivated() override;

	//called when a point in a point cloud gets picked while this tool is active
	void pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P) override;

	//mouse circle element used for the selection
	ccMouseCircle* m_mouseCircle = nullptr;
};

#endif
