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

#ifndef CC_NOTETOOL_HEADER
#define CC_NOTETOOL_HEADER

#include "ccTool.h"
#include "ccNote.h"

#include <QInputDialog>
#include <QMainWindow>

//! Tool used to create notes and associated them with points in a cloud.
class ccNoteTool : public ccTool
{
public:
	ccNoteTool();

	//! Called when the tool is set to active (for initialization)
	void toolActivated() override;

	//! Called when the tool is set to disactive (for cleanup)
	void toolDisactivated() override;

	//! Called when a point in a point cloud gets picked while this tool is active
	void pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P) override;
};

#endif
