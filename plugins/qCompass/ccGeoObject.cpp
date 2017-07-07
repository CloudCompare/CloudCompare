#include "ccGeoObject.h"

ccGeoObject::ccGeoObject(ccPointCloud* associatedCloud)
{
	m_associatedCloud = associatedCloud;

	//add metadata tag defining the ccCompass class type
	QVariantMap* map = new QVariantMap();
	map->insert("ccCompassType", "GeoObject");
	setMetaData(*map, true);
}

void ccGeoObject::setType(QString type)
{
	//store attributes (centroid, strike, dip, RMS) on plane
	QVariantMap* map = new QVariantMap();
	map->insert("Type", type);
	setMetaData(*map, true);
}

QString ccGeoObject::getType()
{
	if (hasMetaData("Type"))
	{
		return getMetaData("Type").toString(); //return type
	}
	else
	{
		return "Unknown";
	}
}

ccPointCloud* ccGeoObject::getAssociatedCloud()
{
	return m_associatedCloud;
}

