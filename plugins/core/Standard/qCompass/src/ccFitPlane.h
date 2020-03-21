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

#ifndef CC_FITPLANE_HEADER
#define CC_FITPLANE_HEADER

#include <ccPlane.h>
#include <ccNormalVectors.h>

#include "ccMeasurement.h"

/*
ccFitPlane is a class that wraps around ccPlane and is used for storing the planes-of-best-fit created using qCompass.
*/
class ccFitPlane :
	public ccPlane, 
	public ccMeasurement
{
public:
	ccFitPlane(ccPlane* p);
	~ccFitPlane() = default;

	//update the metadata attributes of this plane
	void updateAttributes(float rms, float search_r);

	//create a FitPlane object from a point cloud
	static ccFitPlane* Fit(CCLib::GenericIndexedCloudPersist* cloud, double *rms);

	//returns true if object is a plane created by ccCompass (has the associated data)
	static bool isFitPlane(ccHObject* object);
};

#endif