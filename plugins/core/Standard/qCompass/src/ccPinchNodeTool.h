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

#ifndef CC_NODETOOL_HEADER
#define CC_NODETOOL_HEADER

#include "ccTool.h"
#include "ccGeoObject.h"
#include "ccPinchNode.h"

#include <qinputdialog.h>
#include <qmainwindow.h>

/*
Tool used to create PinchNodes.
*/
class ccPinchNodeTool :
	public ccTool
{
public:
	ccPinchNodeTool();
	virtual ~ccPinchNodeTool() = default;

	//called when the tool is set to active (for initialization)
	virtual void toolActivated() override;

	//called when the tool is set to disactive (for cleanup)
	virtual void toolDisactivated() override;

	//called when a point in a point cloud gets picked while this tool is active
	void pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P) override;
};

#endif
