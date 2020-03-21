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

#include "ccFitPlane.h"
#include "ccCompass.h"
ccFitPlane::ccFitPlane(ccPlane* p)
	: ccPlane(p->getXWidth(), p->getYWidth(), &p->getTransformation(), p->getName()) //create an identical plane
{
	p->clone();

	//add metadata tag defining the ccCompass class type
	QVariantMap* map = new QVariantMap();
	map->insert("ccCompassType", "FitPlane");
	setMetaData(*map, true);

	//update name
	CCVector3 N(getNormal());
	//We always consider the normal with a positive 'Z' by default!
	if (N.z < 0.0)
		N *= -1.0;
	//calculate strike/dip/dip direction
	float dip = 0.0f;
	float dipdir = 0.0f;
	ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipdir);
	QString dipAndDipDirStr = QString("%1/%2").arg((int)dip, 2, 10, QChar('0')).arg((int)dipdir, 3, 10, QChar('0'));

	setName(dipAndDipDirStr);

	//update metadata
	float rms = -1;
	float search_r = -1;
	if (p->hasMetaData("RMS"))
	{
		rms = p->getMetaData("RMS").toFloat();
	}
	if (p->hasMetaData("Radius"))
	{
		search_r = p->getMetaData("Radius").toFloat();
	}
	updateAttributes(rms,search_r);

	//update drawing properties based on ccCompass state
	enableStippling(ccCompass::drawStippled);
	showNameIn3D(ccCompass::drawName);
	showNormalVector(ccCompass::drawNormals);
}

void ccFitPlane::updateAttributes(float rms, float search_r)
{
	//calculate and store plane attributes
	//get plane normal vector
	CCVector3 N(getNormal());
	//We always consider the normal with a positive 'Z' by default!
	if (N.z < 0.0)
		N *= -1.0;

	//calculate strike/dip/dip direction
	float strike = 0.0f;
	float dip = 0.0f;
	float dipdir = 0.0f;
	ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipdir);
	//ccNormalVectors::ConvertNormalToStrikeAndDip(N, strike, dip); //n.b. this returns result using the british RHR?!?)

	//calculate strike using American RHR
	strike = dipdir - 90;
	while (strike < 0) //ensure strike > 0
	{
		strike += 360;
	}
	while (strike >= 360) //ensure strike < 360
	{
		strike -= 360;
	}

	//calculate centroid
	CCVector3 C = getCenter();

	//store attributes (centroid, strike, dip, RMS) on plane
	QVariantMap* map = new QVariantMap();
	map->insert("Cx", C.x); map->insert("Cy", C.y); map->insert("Cz", C.z); //centroid
	map->insert("Nx", N.x); map->insert("Ny", N.y); map->insert("Nz", N.z); //normal
	map->insert("Strike", strike); map->insert("Dip", dip); map->insert("DipDir", dipdir); //strike & dip
	map->insert("RMS", rms); //rms
	map->insert("Radius", search_r); //search radius
	setMetaData(*map, true);
}

bool ccFitPlane::isFitPlane(ccHObject* object)
{
	if (object->hasMetaData("ccCompassType"))
	{
		return object->getMetaData("ccCompassType").toString().contains("FitPlane");
	}
	return false;
	/*return object->isKindOf(CC_TYPES::PLANE) //ensure object is a plane
	&& object->hasMetaData("Cx") //ensure plane has the correct metadata
	&& object->hasMetaData("Cy")
	&& object->hasMetaData("Cz")
	&& object->hasMetaData("Nx")
	&& object->hasMetaData("Ny")
	&& object->hasMetaData("Nz")
	&& object->hasMetaData("Strike")
	&& object->hasMetaData("Dip")
	&& object->hasMetaData("DipDir")
	&& object->hasMetaData("RMS")
	&& object->hasMetaData("Radius");*/
}

ccFitPlane* ccFitPlane::Fit(CCLib::GenericIndexedCloudPersist* cloud, double *rms)
{
	ccPlane* p = ccPlane::Fit(cloud, rms);
	if (p) //valid plane
	{
		ccFitPlane* fp = new ccFitPlane(p);
		p->transferChildren(*fp);
		return fp;
	}
	else
	{
		return nullptr; //return null
	}
}