#include "ccPinchNodeTool.h"

ccPinchNodeTool::ccPinchNodeTool()
	: ccTool()
{
}

ccPinchNodeTool::~ccPinchNodeTool()
{
}


//called when a point in a point cloud gets picked while this tool is active
void ccPinchNodeTool::pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P)
{

	//create a 1-point lineation object (highlights node-location)
	ccPointPair* l = new ccPinchNode(cloud);
	l->setName("tip");
	l->showNameIn3D(false);
	l->addPointIndex(itemIdx);
	l->setDefaultColor(ccColor::blue);
	l->setActiveColor(ccColor::orange);
	l->setHighlightColor(ccColor::orange);
	l->setAlternateColor(ccColor::orange);

	//update insert-point if there is an active GeoObject
	ccGeoObject* geoObj = ccGeoObject::getGeoObjectParent(insertPoint);
	if (geoObj) //there is an active GeoObject
	{
		insertPoint = geoObj->getRegion(ccGeoObject::INTERIOR); //add pinch-points to GeoObject interior
	}

	//add to scene graph
	insertPoint->addChild(l);
	m_app->addToDB(l);
}


//called when the tool is set to active (for initialization)
void ccPinchNodeTool::toolActivated()
{ 
	//donothing
}

//called when the tool is set to disactive (for cleanup)
void ccPinchNodeTool::toolDisactivated()
{
	//donothing
}