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

#ifndef CC_NOTE_HEADER
#define CC_NOTE_HEADER

#include "ccPointPair.h"

#include <ccPointCloud.h>

class ccPinchNode : public ccPointPair
{
public:
	//ctors
	ccPinchNode(ccPointCloud* associatedCloud);
	ccPinchNode(ccPolyline* obj);

	//write metadata specific to this object
	void updateMetadata() override;

	static bool isPinchNode(ccHObject* obj);
};

#endif
