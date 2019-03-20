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

#ifndef CC_TOPOLOGY_HEADER
#define CC_TOPOLOGY_HEADER

#include "ccPointPair.h"

#include <ccPointCloud.h>

class ccGeoObject;

/*
Class/object used to define topological (timing) relationships between different ccGeoObjects [WIP]
*/
class ccTopologyRelation : public ccPointPair
{
public:
	//ctors
	ccTopologyRelation(ccPointCloud* associatedCloud, int older_id, int younger_id, int type);
	ccTopologyRelation(ccPolyline* obj);

	//call this function to build a visible representation of this relationship
	void constructGraphic(ccGeoObject* older, ccGeoObject* younger);

	//write metadata specific to this object
	void updateMetadata() override;

	//returns the youuger GeoObject
	int getYoungerID();

	//returns the older GeoObject
	int getOlderID();
	
	//return the type of this relationship
	int getType();

	//set the type of this relationship
	void setType(int topologyType);

	static bool isTopologyRelation(ccHObject* obj);

	//inverts a topology relation (i.e older_than becomes younger_than)
	static int invertType(int type);
private:
	int m_older_id = -1; //the older object of this relationship (notation is arbitrary for "equivalence" relations)
	int m_younger_id = -1; //the younger object of this relationship

	int m_type; //the type of this relationship

public:
	//x-cutting relationships and similar
	static const int YOUNGER_THAN = 2;
	static const int 	OLDER_THAN = 4;

	//conformable relationships and similar
	static const int IMMEDIATELY_FOLLOWS = 8 | YOUNGER_THAN; //(younger than bit set also!)
	static const int IMMEDIATELY_PRECEDES = 16 | OLDER_THAN; //(older than bit set also!)

	//equivalence
	static const int EQUIVALENCE = 32;

	//other (rare relations)
	static const int NOT_OLDER_THAN = 64; //equivalent to or younger
	static const int NOT_YOUNGER_THAN = 128; //equivalent to or older

	static const int UNKNOWN = 2048; //unknown...
};

#endif
