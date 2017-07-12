#include "ccGeoObject.h"

ccGeoObject::ccGeoObject(QString name, ccMainAppInterface* app)
	: ccHObject(name)
{
	init(name, app);

	//add "interior", "upper" and "lower" HObjects
	generateInterior();
	generateUpper();
	generateLower();
}

ccGeoObject::ccGeoObject(ccHObject* obj, ccMainAppInterface* app)
	: ccHObject(obj->getName())
{
	init(obj->getName(), app);

	//n.b. object should already have upper, inner and lower children

}

void ccGeoObject::init(QString name, ccMainAppInterface* app)
{
	setName(name);

	//store reference to app so we can manipulate the database
	m_app = app;

	//add metadata tag defining the ccCompass class type
	QVariantMap* map = new QVariantMap();
	map->insert("ccCompassType", "GeoObject");
	setMetaData(*map, true);
}

ccPointCloud* ccGeoObject::getAssociatedCloud()
{
	return m_associatedCloud;
}

ccHObject* ccGeoObject::getRegion(int mappingRegion)
{
	switch (mappingRegion)
	{
	case ccGeoObject::INTERIOR:
		//check region hasn't been deleted...
		if (!m_app->dbRootObject()->find(m_interior_id))
		{
			//item has been deleted... make a new one
			generateInterior();
		}
		return m_interior;
	case ccGeoObject::UPPER_BOUNDARY:
		//check region hasn't been deleted...
		if (!m_app->dbRootObject()->find(m_upper_id))
		{
			//item has been deleted... make a new one
			generateUpper();
		}
		return m_upper;
	case ccGeoObject::LOWER_BOUNDARY:
		//check region hasn't been deleted...
		if (!m_app->dbRootObject()->find(m_lower_id))
		{
			//item has been deleted... make a new one
			generateLower();
		}
		return  m_lower;
	default:
		return nullptr;
	}
}

void ccGeoObject::setActive(bool active)
{
	for (ccHObject* c : m_children)
	{
		recurseChildren(c,active);
	}
}

void ccGeoObject::recurseChildren(ccHObject* par, bool highlight)
{
	//set if par is a measurement
	ccMeasurement* m = dynamic_cast<ccMeasurement*>(par);
	if (m)
	{
		m->setHighlight(highlight);

		//draw labels (except for trace objects, when the child plane object will hold the useful info)
		if (!ccTrace::isTrace(par))
		{
			par->showNameIn3D(highlight);
		}
	}

	//recurse
	for (int i = 0; i < par->getChildrenNumber(); i++)
	{
		ccHObject* c = par->getChild(i);
		recurseChildren(c, highlight);
	}
}

void ccGeoObject::generateInterior()
{
	//check interior doesn't already exist
	for (ccHObject* c : m_children)
	{
		if (c->hasMetaData("ccCompassType") & (c->getMetaData("ccCompassType").toString() == "GeoInterior"))
		{
			m_interior = c;
			m_interior_id = c->getUniqueID();
			return;
		}
	}

	m_interior = new ccHObject("Interior");
	
	//give them associated property flags
	QVariantMap* map = new QVariantMap();
	map->insert("ccCompassType", "GeoInterior");
	m_interior->setMetaData(*map, true);

	//add these to the scene graph
	addChild(m_interior);
	m_interior_id = m_interior->getUniqueID();
}

void ccGeoObject::generateUpper()
{
	//check upper doesn't already exist
	for (ccHObject* c : m_children)
	{
		if (c->hasMetaData("ccCompassType") & (c->getMetaData("ccCompassType").toString() == "GeoUpperBoundary"))
		{
			m_upper = c;
			m_upper_id = c->getUniqueID();
			return;
		}
	}

	m_upper = new ccHObject("Upper Boundary");

	QVariantMap* map = new QVariantMap();
	map->insert("ccCompassType", "GeoUpperBoundary");
	m_upper->setMetaData(*map, true);

	addChild(m_upper);
	m_upper_id = m_upper->getUniqueID();
}

void ccGeoObject::generateLower()
{
	//check upper doesn't already exist
	for (ccHObject* c : m_children)
	{
		if (c->hasMetaData("ccCompassType") & (c->getMetaData("ccCompassType").toString() == "GeoLowerBoundary"))
		{
			m_lower = c;
			m_lower_id = c->getUniqueID();
			return;
		}
	}

	m_lower = new ccHObject("Lower Boundary");

	QVariantMap* map = new QVariantMap();
	map->insert("ccCompassType", "GeoLowerBoundary");
	m_lower->setMetaData(*map, true);

	addChild(m_lower);
	m_lower_id = m_lower->getUniqueID();
}

bool ccGeoObject::isGeoObject(ccHObject* object)
{
	if (object->hasMetaData("ccCompassType"))
	{
		return object->getMetaData("ccCompassType").toString().contains("GeoObject");
	}
	return false;
}

bool ccGeoObject::isGeoObjectUpper(ccHObject* object)
{
	if (object->hasMetaData("ccCompassType"))
	{
		return object->getMetaData("ccCompassType").toString().contains("GeoUpperBoundary");
	}
	return false;
}

bool ccGeoObject::isGeoObjectLower(ccHObject* object)
{
	if (object->hasMetaData("ccCompassType"))
	{
		return object->getMetaData("ccCompassType").toString().contains("GeoLowerBoundary");
	}
	return false;
}

bool ccGeoObject::isGeoObjectInterior(ccHObject* object)
{
	if (object->hasMetaData("ccCompassType"))
	{
		return object->getMetaData("ccCompassType").toString().contains("GeoInterior");
	}
	return false;
}

/*

bool ccGeoObject::isGeoObjectUpper(ccHObject* object)
{
return object->getName().contains("Upper Boundary"); //n.b. these are just special folders really...
}

bool ccGeoObject::isGeoObjectLower(ccHObject* object)
{
return object->getName().contains("Lower Boundary");
}

bool ccGeoObject::isGeoObjectInterior(ccHObject* object)
{
return object->getName().contains("Interior");
}

*/