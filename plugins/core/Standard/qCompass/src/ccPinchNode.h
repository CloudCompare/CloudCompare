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

#ifndef CC_NODE_HEADER
#define CC_NODE_HEADER

#include "ccPointPair.h"

#include <ccPointCloud.h>

/*
Simple class used to create/represent/draw pinch-nodes created using qCompass
*/
class ccPinchNode : public ccPointPair
{
public:
	//ctors
	ccPinchNode(ccPointCloud* associatedCloud);
	ccPinchNode(ccPolyline* obj);

	//write metadata specific to this object
	void updateMetadata() override;

	//returns true if obj is/was a pinchNode (as recorded by its metadata)
	static bool isPinchNode(ccHObject* obj);
};

#endif
