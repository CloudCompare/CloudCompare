#ifndef CC_LINEATIONTOOL_HEADER
#define CC_LINEATIONTOOL_HEADER

#include "cctool.h"
#include "ccLineation.h"

class ccLineationTool :
	public ccTool
{
public:
	ccLineationTool();
	~ccLineationTool();

	//called when a point in a point cloud gets picked while this tool is active
	void pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P) override;

	//called when "Return" or "Space" is pressed, or the "Accept Button" is clicked
	void accept() override; //do nothing

	//called when the "Escape" is pressed, or the "Cancel" button is clicked
	void cancel() override; //do nothing
protected:
	ccLineation* m_lineation = nullptr; //active lineation object
	int m_lineation_id = -1; //used to check if m_lineation has been deleted

};

#endif
