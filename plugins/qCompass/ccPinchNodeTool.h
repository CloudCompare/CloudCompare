#ifndef CC_NODETOOL_HEADER
#define CC_NODETOOL_HEADER

#include "cctool.h"
#include "ccGeoObject.h"
#include "ccPinchNode.h"

#include <qinputdialog.h>
#include <qmainwindow.h>

class ccPinchNodeTool :
	public ccTool
{
public:
	ccPinchNodeTool();
	~ccPinchNodeTool();

	//called when the tool is set to active (for initialization)
	virtual void toolActivated() override;

	//called when the tool is set to disactive (for cleanup)
	virtual void toolDisactivated() override;

	//called when a point in a point cloud gets picked while this tool is active
	void pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P) override;
};

#endif
