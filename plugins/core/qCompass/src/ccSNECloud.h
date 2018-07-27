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

#ifndef CC_SNE_HEADER
#define CC_SNE_HEADER

#include <ccPointCloud.h>
#include <ccMeasurement.h>

/*
Class for representing/drawing lineations measured with qCompass.
*/
class ccSNECloud : 
	public ccPointCloud,
	public ccMeasurement
{
public:
	//ctors
	ccSNECloud();
	ccSNECloud(ccPointCloud* obj);

	//write metadata specific to this object
	void updateMetadata();

	//returns true if the given ccHObject is/was a ccLineation (as defined by the objects metadata)
	static bool isSNECloud(ccHObject* obj);

protected:
	//overidden from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;
};
#endif
