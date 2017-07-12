#include "ccThicknessTool.h"
#include "ccCompass.h"

ccColor::Rgb ccThicknessTool::ACTIVE_COLOR = ccColor::red;
ccThicknessTool::ccThicknessTool()
	: ccTool()
{
}

ccThicknessTool::~ccThicknessTool()
{
}

//called when the selection is changed while this tool is active
void ccThicknessTool::onNewSelection(const ccHObject::Container& selectedEntities)
{
	for (ccHObject* h : selectedEntities)
	{
		ccPlane* p = dynamic_cast<ccPlane*>(h);
		if (p) //this is a plane?
		{
			if (m_referencePlane)
			{
				m_referencePlane->enableTempColor(false); //go back to normal colour
			}

			//store plane
			m_referencePlane = p; //set the reference plane used to calculate the thickness

			//change colour
			m_referencePlane->setTempColor(ccThicknessTool::ACTIVE_COLOR);
			m_referencePlane->enableTempColor(true);

			//make all point clouds visible again
			for (int i : m_hiddenPointClouds)
			{
				ccHObject* cld = m_app->dbRootObject()->find(i);
				cld->setVisible(true);
			}

			m_hiddenPointClouds.clear();

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

	//plane to point distance
	if (m_endPoint == -1)
	{
		float dist = planeToPointDistance(m_referencePlane, P);

		//back calculate the start point
		CCVector3 start = P - m_referencePlane->getNormal() * dist;

		//create point cloud and add start/end points too it
		ccPointCloud* verts = new ccPointCloud("vertices");

		assert(verts);

		verts->reserve(2);
		verts->addPoint(start);
		verts->addPoint(P);

		//create a "lineation" graphic to display
		m_graphic = new ccLineation(verts);
		m_graphic->addPointIndex(0);
		m_graphic->addPointIndex(1);
		m_graphic->addChild(verts); //store the verts
		m_graphic->invalidateBoundingBox();
		m_graphic->setName(QString::asprintf("Thickness: %.3f", dist));
		m_graphic->showNameIn3D(ccCompass::drawName);

		//add to scene graph
		insertPoint->addChild(m_graphic);
		m_app->addToDB(m_graphic, false, true, false, true);

		//finish
		accept();
	} else 
	{
		//get ready to make next measurement
		accept();
	}
}

//called when the tool is set to active (for initialization)
void ccThicknessTool::toolActivated() 
{ 
	//hide all visible point clouds
	recurseChildren(m_app->dbRootObject());
	//redraw
	m_app->getActiveGLWindow()->refresh();
}

void ccThicknessTool::recurseChildren(ccHObject* obj)
{
	if (obj->isA(CC_TYPES::POINT_CLOUD))
	{
		if (obj->isVisible())
		{
			obj->setVisible(false);
			m_hiddenPointClouds.push_back(obj->getUniqueID());
		}
		return;
	}

	for (int i = 0; i < obj->getChildrenNumber(); i++)
	{
		recurseChildren(obj->getChild(i));
	}
}

//called when the tool is set to disactive (for cleanup)
void ccThicknessTool::toolDisactivated() 
{
	if (m_referencePlane)
	{
		m_referencePlane->enableTempColor(false); //go back to normal colour
	}

	//make all point clouds visible again
	for (int i : m_hiddenPointClouds)
	{
		ccHObject* cld = m_app->dbRootObject()->find(i);
		cld->setVisible(true);
	}

	//redraw
	m_app->getActiveGLWindow()->refresh();

	m_hiddenPointClouds.clear();

	//reset
	m_referencePlane = nullptr;
	m_endPoint = -1;
	m_endPoint2 = -1;
}

//called when "Return" or "Space" is pressed, or the "Accept Button" is clicked
void ccThicknessTool::accept()
{
	//Reset the tool (but keep the plane so multiple thickness measurements can be made)
	m_endPoint = -1;
	m_endPoint2 = -1;
}

//called when the "Escape" is pressed, or the "Cancel" button is clicked
void ccThicknessTool::cancel()
{
	
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
