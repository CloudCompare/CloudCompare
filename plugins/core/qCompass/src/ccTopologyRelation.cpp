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

#include "ccTopologyRelation.h"

#include "ccGeoObject.h"

//pass ctors straight to PointPair
ccTopologyRelation::ccTopologyRelation(ccPointCloud* associatedCloud, int older_id, int younger_id, int type)
	: ccPointPair(associatedCloud)
{
	m_older_id = older_id;
	m_younger_id = younger_id;
	m_type = type;

	updateMetadata();
}

ccTopologyRelation::ccTopologyRelation(ccPolyline* obj)
	: ccPointPair(obj)
{ 
	//retrive metadata values
	if (obj->hasMetaData("RelationType"))
	{
		m_type = obj->getMetaData("RelationType").toInt();
	}

	if (obj->hasMetaData("Older_ID"))
	{
		m_older_id = obj->getMetaData("Older_ID").toInt();
	}

	if (obj->hasMetaData("Younger_ID"))
	{
		m_older_id = obj->getMetaData("Younger_ID").toInt();
	}

	//update
	updateMetadata();
}

void ccTopologyRelation::updateMetadata()
{
	//add metadata tag defining the ccCompass class type
	QVariantMap* map = new QVariantMap();
	map->insert("ccCompassType", "TopologyRelation");
	map->insert("RelationType", m_type);
	map->insert("Older_ID", m_older_id);
	map->insert("Younger_ID", m_younger_id);
	setMetaData(*map, true);

	showNameIn3D(true);
}

void ccTopologyRelation::constructGraphic(ccGeoObject* older, ccGeoObject* younger)
{
	//get underlying point cloud
	ccPointCloud* c = dynamic_cast<ccPointCloud*> (getAssociatedCloud());
	assert(c);

	//clear points, both in cloud and in our underlying polyline
	clear();
	c->clear();
	c->setEnabled(false);
	c->setVisible(false); //should be hidden

	//make room for start and end points in underlying cloud
	c->reserve(2);

	//calculate centroids for graphics
	CCVector3 old_P = older->getBB_recursive().getCenter(); //first geoObject centroid
	CCVector3 young_P = younger->getBB_recursive().getCenter(); //second geoObject centroid

	//add start/end points too point cloud
	c->addPoint(old_P);
	c->addPoint(young_P);

	//add polyline points (these will then cause the underlying points in the cloud to be drawn)
	addPointIndex(1); //arrow points to older object
	addPointIndex(0);

	//set name
	switch (m_type)
	{
	case ccTopologyRelation::EQUIVALENCE:
		setName(older->getName() + " Equivalent to " + younger->getName());
		break;
	case ccTopologyRelation::IMMEDIATELY_FOLLOWS:
		setName(older->getName() + " Precedes " + younger->getName());
		break;
	case ccTopologyRelation::IMMEDIATELY_PRECEDES:
		setName(older->getName() + " Follows " + younger->getName());
		break;
	case ccTopologyRelation::YOUNGER_THAN:
		setName(older->getName() + " Younger than " + younger->getName());
		break;
	case ccTopologyRelation::OLDER_THAN:
		setName(older->getName() + " Older than " + younger->getName());
		break;
	case ccTopologyRelation::NOT_OLDER_THAN:
		setName(older->getName() + " Not Older than " + younger->getName());
		break;
	case ccTopologyRelation::NOT_YOUNGER_THAN:
			setName(older->getName() + " Not Younger than " + younger->getName());
			break;
	default:
		setName(older->getName() + " Unknown Relationship to " + younger->getName());
	}

	showNameIn3D(true);
}

//returns the youuger GeoObject
int ccTopologyRelation::getYoungerID()
{
	return m_younger_id;
}

//returns the older GeoObjects
int ccTopologyRelation::getOlderID()
{
	return m_older_id;
}

//return the type of this relationship
int ccTopologyRelation::getType()
{
	return m_type;
}

//get the type of this relationship
void ccTopologyRelation::setType(int topologyType)
{
	m_type = topologyType;
}



int ccTopologyRelation::invertType(int type)
{
	switch (type)
	{
	case (ccTopologyRelation::EQUIVALENCE) :
		return ccTopologyRelation::EQUIVALENCE;
	case (ccTopologyRelation::IMMEDIATELY_FOLLOWS) :
		return ccTopologyRelation::IMMEDIATELY_PRECEDES;
	case (ccTopologyRelation::IMMEDIATELY_PRECEDES) :
		return ccTopologyRelation::IMMEDIATELY_FOLLOWS;
	case (ccTopologyRelation::NOT_OLDER_THAN) :
		return ccTopologyRelation::NOT_YOUNGER_THAN;
	case (ccTopologyRelation::NOT_YOUNGER_THAN) :
		return ccTopologyRelation::NOT_OLDER_THAN;
	case (ccTopologyRelation::OLDER_THAN) :
		return ccTopologyRelation::YOUNGER_THAN;
	case (ccTopologyRelation::YOUNGER_THAN) :
		return ccTopologyRelation::OLDER_THAN;
	default:
		return ccTopologyRelation::UNKNOWN;
	}
}

//returns true if object is a lineation
bool ccTopologyRelation::isTopologyRelation(ccHObject* object)
{
	if (object->hasMetaData("ccCompassType"))
	{
		return object->getMetaData("ccCompassType").toString().contains("TopologyRelation");
	}
	return false;
}
