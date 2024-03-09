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

#ifndef CC_THICKNESSTOOL_HEADER
#define CC_THICKNESSTOOL_HEADER

#include "ccTool.h"
#include "ccThickness.h"
#include "ccPlane.h"

#include <ccColorTypes.h>
#include <DistanceComputationTools.h>

//! Tool used to create thickness measurements in qCompass
class ccThicknessTool :	public ccTool
{
public:
	ccThicknessTool();

	//! Called when the tool is set to active (for initialization)
	void toolActivated() override;

	//! Called when the tool is set to disactive (for cleanup)
	void toolDisactivated() override;

	//! Called when the selection is changed while this tool is active
	void onNewSelection(const ccHObject::Container& selectedEntities) override;

	//! Called when a point in a point cloud gets picked while this tool is active
	bool pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccHObject* pickedObject, const CCVector3& P) override;

	//! Called when a point in a point cloud gets picked while this tool is active
	void pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P) override;

	//! Called when "Return" or "Space" is pressed, or the "Accept Button" is clicked
	void accept() override; //do nothing

	//! Called when the "Escape" is pressed, or the "Cancel" button is clicked
	void cancel() override; //do nothing

protected:
	ccPlane* m_referencePlane = nullptr; //!< Plane used to calculate thickness
	CCVector3* m_startPoint = nullptr; //!< First point used to calculate thickness in two-point mode
	std::vector<int> m_hiddenObjects; //!< Used to hide all point clouds (first), then all planes (second).
	int m_graphic_id = -1; //!< Used to store partially completed lineation graphics

private:
	//! Calculates point-to-plane distances
	float planeToPointDistance(ccPlane* plane, CCVector3 P);

	//! Builds a "thickness" graphic
	ccHObject* buildGraphic(CCVector3 endPoint, float thickness);

	//! Recurses children looking for point clouds & making them invisible
	void recurseChildren(ccHObject* obj, bool hidePointClouds, bool hidePlanes);

	//! Gets the interior part of the currently selected GeoObject (or returns the insertPoint if it's not a GeoObject)
	ccHObject* getInsertInterior(ccHObject* insertPoint);

public:
	static bool TWO_POINT_MODE; //if true, two points + planar orientation used to calculate thickness. If false, then point-to-plane distance
	                            //is calculated for each point
};

#endif
