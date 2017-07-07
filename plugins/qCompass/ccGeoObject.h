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


#ifndef CC_GEOOBJECT_HEADER
#define CC_GEOOBJECT_HEADER

#include <ccHObject.h>
#include <ccPointCloud.h>

class ccGeoObject : public ccHObject
{
public:
	ccGeoObject(ccPointCloud* associatedCloud);

	void setType(QString type);
	QString getType();

	ccPointCloud* getAssociatedCloud();

protected:
	ccPointCloud* m_associatedCloud; //the dataset this object "belongs" too

};


#endif
