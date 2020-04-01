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

#include "ccCompass.h"
#include "ccGeoObject.h"
#include "ccThicknessTool.h"

static ccColor::Rgba ACTIVE_COLOR = ccColor::red;
bool ccThicknessTool::TWO_POINT_MODE = false;

ccThicknessTool::ccThicknessTool()
	: ccTool()
{
}

//called when the selection is changed while this tool is active
void ccThicknessTool::onNewSelection(const ccHObject::Container& selectedEntities)
{
	for (ccHObject* h : selectedEntities)
	{
		ccPlane* p = dynamic_cast<ccPlane*>(h);
		if (p && p->isDisplayed()) //this is a plane? [and it's shown - avoids confusion]
		{
			if (m_referencePlane)
			{
				m_referencePlane->enableTempColor(false); //go back to normal colour
			}

			//store plane
			m_referencePlane = p; //set the reference plane used to calculate the thickness

			//change colour
			m_referencePlane->setTempColor(ACTIVE_COLOR);
			m_referencePlane->enableTempColor(true);

			//make all point clouds visible again
			for (int i : m_hiddenObjects)
			{
				ccHObject* cld = m_app->dbRootObject()->find(i);
				cld->setVisible(true);
			}
			m_hiddenObjects.clear();

			//now hide all visible planes
			recurseChildren(m_app->dbRootObject(), false, true);

			//make the reference plane visible
			m_referencePlane->setVisible(true);

			//display instructions
			m_app->getActiveGLWindow()->displayNewMessage("Select measurement point.", ccGLWindow::LOWER_LEFT_MESSAGE);

			//redraw
			m_app->getActiveGLWindow()->refresh();

			//done
			return;
		}
	}
}

//called when a point in a point cloud gets picked while this tool is active
void ccThicknessTool::pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccHObject* pickedObject, const CCVector3& P)
{
	if (pickedObject->isA(CC_TYPES::PLANE)) //we want to be able to pick planes
	{
		//select the object
		m_app->setSelectedInDB(pickedObject, true);

		//call to update selection
		onNewSelection(m_app->getSelectedEntities());
	}
}

//called when a point in a point cloud gets picked while this tool is active
void ccThicknessTool::pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P)
{
	//no plane, no deal
	if (!m_referencePlane)
	{
		m_app->dispToConsole("[ccCompass] Please select a fit-plane to constrain true-thickness calculations.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//get modified insert point (thicknesses are always added to GeoObject interiors if possible)
	insertPoint = getInsertInterior(insertPoint);

	if (!ccThicknessTool::TWO_POINT_MODE) //one point mode - calculate plane to point distance and finish
	{
		float dist = planeToPointDistance(m_referencePlane, P);

		//build graphic
		ccHObject* g = buildGraphic(P, dist);

		//add to scene graph
		insertPoint->addChild(g);
		m_app->addToDB(g, false, true, false, true);
	} else //two point mode... which points have been defined?
	{
		if (!m_startPoint) //first point not yet defined - store it
		{
			//store point
			m_startPoint = new CCVector3(P);

			//create temporary graphic
			ccPointPair* temp = new ccPointPair(cloud);
			temp->addPointIndex(itemIdx);
			temp->showNameIn3D(true);
			temp->setName("P1");
			m_graphic_id = temp->getUniqueID();
			insertPoint->addChild(temp);
			m_app->addToDB(temp, false, false, false, true);

			//display instructions
			m_app->getActiveGLWindow()->displayNewMessage("Select second measurement point.", ccGLWindow::LOWER_LEFT_MESSAGE);

		}
		else
		{
			//delete temporary graphic
			m_app->removeFromDB(m_app->dbRootObject()->find(m_graphic_id));

			//calculate distance
			float dist = planeToPointDistance(m_referencePlane, P) - planeToPointDistance(m_referencePlane, *m_startPoint);

			ccHObject* g = buildGraphic(P, dist);

			//add to scene graph
			insertPoint->addChild(g);
			m_app->addToDB(g, false, true, false, true);

			//finish
			delete m_startPoint;
			m_startPoint = nullptr;
		}
	} 
}

ccHObject* ccThicknessTool::getInsertInterior(ccHObject* insertPoint)
{
	ccHObject* p = insertPoint;
	while (p != nullptr)
	{
		//object is a geoObject
		if (ccGeoObject::isGeoObject(p))
		{
			ccGeoObject* obj = dynamic_cast<ccGeoObject*> (p);
			if (obj)
			{
				return obj->getRegion(ccGeoObject::INTERIOR); //return the interior
			}
		}

		//try next parent
		p = p->getParent();
	}

	//haven't found a geoObject - use the supplied insertPoint
	return insertPoint; //todo
}

ccHObject* ccThicknessTool::buildGraphic(CCVector3 endPoint, float thickness)
{
	//back calculate the start point
	CCVector3 start = endPoint - m_referencePlane->getNormal() * thickness;

	//create point cloud and add start/end points too it
	ccPointCloud* verts = new ccPointCloud("vertices");
	assert(verts);
	verts->reserve(2);
	verts->addPoint(start);
	verts->addPoint(endPoint);
	verts->invalidateBoundingBox();
	verts->setEnabled(false); //this is used for storage only!
	verts->setVisible(false); //this is used for storage only!

	//create a "thickness" graphic to display
	ccThickness* graphic = new ccThickness(verts);
	graphic->addPointIndex(0);
	graphic->addPointIndex(1);
	graphic->addChild(verts); //store the verts
	graphic->invalidateBoundingBox();
	graphic->updateMetadata();
	graphic->setName(QString::asprintf("%.3fT", std::fabs(thickness)));
	graphic->showNameIn3D(ccCompass::drawName);

	//return
	return graphic;
}

//called when the tool is set to active (for initialization)
void ccThicknessTool::toolActivated() 
{ 
	//hide all visible point clouds
	recurseChildren(m_app->dbRootObject(), true, false);

	//display instructions
	m_app->getActiveGLWindow()->displayNewMessage("Select reference plane for thickness measurement.", ccGLWindow::LOWER_LEFT_MESSAGE);

	//redraw
	m_app->getActiveGLWindow()->redraw(false, false);
}

//called when the tool is set to disactive (for cleanup)
void ccThicknessTool::toolDisactivated()
{
	//delete start point object
	if (m_startPoint)
	{
		delete m_startPoint;
		m_startPoint = nullptr;
	}

	if (m_referencePlane)
	{
		m_referencePlane->enableTempColor(false); //go back to normal colour
		m_referencePlane = nullptr;
	}

	//make all point clouds visible again
	for (int i : m_hiddenObjects)
	{
		ccHObject* cld = m_app->dbRootObject()->find(i);
		cld->setVisible(true);
	}
	m_hiddenObjects.clear();

	//redraw
	m_app->getActiveGLWindow()->refresh();
}

void ccThicknessTool::recurseChildren(ccHObject* obj, bool hidePointClouds, bool hidePlanes)
{
	//is this a point cloud?
	if (hidePointClouds && obj->isA(CC_TYPES::POINT_CLOUD))
	{
		if (obj->isVisible())
		{
			obj->setVisible(false);
			m_hiddenObjects.push_back(obj->getUniqueID());
		}
		return;
	}

	//is this a plane?
	if (hidePlanes && obj->isA(CC_TYPES::PLANE))
	{
		if (obj->isVisible())
		{
			obj->setVisible(false);
			m_hiddenObjects.push_back(obj->getUniqueID());
		}
		return;
	}

	//recurse on children
	for (unsigned i = 0; i < obj->getChildrenNumber(); i++)
	{
		recurseChildren(obj->getChild(i), hidePointClouds, hidePlanes);
	}
}

//called when "Return" or "Space" is pressed, or the "Accept Button" is clicked
void ccThicknessTool::accept()
{
	//Reset the tool
	toolDisactivated();

	//go back to "plane pick mode"
	toolActivated();
}

//called when the "Escape" is pressed, or the "Cancel" button is clicked
void ccThicknessTool::cancel()
{
	toolDisactivated();
}

float ccThicknessTool::planeToPointDistance(ccPlane* plane, CCVector3 P)
{
	//declare array of 4 pointcoordtypes
	PointCoordinateType pEq[4];

	//build equation of plane
	pEq[0] = plane->getNormal().x;
	pEq[1] = plane->getNormal().y;
	pEq[2] = plane->getNormal().z;
	pEq[3]= plane->getCenter().dot(plane->getNormal()); //a point on the plane dot the plane normal

	//return distance
	return CCLib::DistanceComputationTools::computePoint2PlaneDistance(&P, pEq);
}
