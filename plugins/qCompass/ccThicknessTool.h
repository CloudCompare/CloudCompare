#ifndef CC_THICKNESSTOOL_HEADER
#define CC_THICKNESSTOOL_HEADER

#include "cctool.h"
#include "ccLineation.h"
#include "ccPlane.h"

#include <ccColorTypes.h>
#include <DistanceComputationTools.h>

class ccThicknessTool :
	public ccTool
{
public:
	ccThicknessTool();
	~ccThicknessTool();

	//called when the tool is set to active (for initialization)
	virtual void toolActivated() override;

	//called when the tool is set to disactive (for cleanup)
	virtual void toolDisactivated() override;

	//called when the selection is changed while this tool is active
	virtual void onNewSelection(const ccHObject::Container& selectedEntities) override;

	//called when a point in a point cloud gets picked while this tool is active
	virtual void pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccHObject* pickedObject, const CCVector3& P) override;

	//called when a point in a point cloud gets picked while this tool is active
	void pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P) override;

	//called when "Return" or "Space" is pressed, or the "Accept Button" is clicked
	void accept() override; //do nothing

	//called when the "Escape" is pressed, or the "Cancel" button is clicked
	void cancel() override; //do nothing
protected:
	ccPlane* m_referencePlane = nullptr; //plane used to calculate thickness
	int m_endPoint = -1; //point used to calculate thickness
	int m_endPoint2 = -1; //point used in 2-point + plane thickness calculations
	ccLineation* m_graphic = nullptr;
	std::vector<int> m_hiddenPointClouds;

private:
	float planeToPointDistance(ccPlane* plane, CCVector3 P);

	//recurses children looking for point clouds & making them invisible
	void recurseChildren(ccHObject* obj);

public:
	static ccColor::Rgb ACTIVE_COLOR;

};

#endif
