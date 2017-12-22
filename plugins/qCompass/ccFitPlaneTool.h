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

#ifndef CC_FITPLANETOOL_HEADER
#define CC_FITPLANETOOL_HEADER

#include <DgmOctreeReferenceCloud.h>

#include "ccTool.h"
#include "ccMouseCircle.h"
#include "ccFitPlane.h"

/*
Tool that is activated during "Plane Mode", generating fit planes from point-picks
*/
class ccFitPlaneTool :
	public ccTool
{
public:
	ccFitPlaneTool();
	virtual ~ccFitPlaneTool();

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
