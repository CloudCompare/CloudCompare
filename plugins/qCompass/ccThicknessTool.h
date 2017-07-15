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
	CCVector3* m_startPoint = nullptr; //first point used to calculate thickness in two-point mode
	std::vector<int> m_hiddenObjects; //used to hide all point clouds (first), then all planes (second).

private:
	float planeToPointDistance(ccPlane* plane, CCVector3 P);
	ccHObject* buildGraphic(CCVector3 endPoint, float thickness);

	//recurses children looking for point clouds & making them invisible
	void recurseChildren(ccHObject* obj, bool hidePointClouds, bool hidePlanes);

	//gets the interior part of the currently selected GeoObject (or returns the insertPoint if it's not a GeoObject)
	ccHObject* getInsertInterior(ccHObject* insertPoint);
public:
	static ccColor::Rgb ACTIVE_COLOR;
	static bool TWO_POINT_MODE; //if true, two points + planar orientation used to calculate thickness.
};

#endif
