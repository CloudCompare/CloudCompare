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

#include "ccFitPlaneTool.h"

ccFitPlaneTool::ccFitPlaneTool()
	: ccTool()
{
}


ccFitPlaneTool::~ccFitPlaneTool()
{
	if (m_mouseCircle)
	{
		assert(false); //we should never end up here...
		m_mouseCircle->ownerIsDead();
		delete m_mouseCircle;
		m_mouseCircle = nullptr;
	}
}

//called when the tool is set to active (for initialization)
void ccFitPlaneTool::toolActivated()
{
	m_mouseCircle = new ccMouseCircle( m_app->getActiveGLWindow() );
	m_mouseCircle->setVisible(true);

	//set orthographic view (as this tool doesn't work in perspective mode)
	m_app->getActiveGLWindow()->setPerspectiveState(false, true);
}

//called when the tool is set to disactive (for cleanup)
void ccFitPlaneTool::toolDisactivated()
{
	if (m_mouseCircle) {
		m_mouseCircle->setVisible(false);
		delete m_mouseCircle;
		m_mouseCircle = nullptr;
	}
}

//called when a point in a point cloud gets picked while this tool is active
void ccFitPlaneTool::pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P)
{
	//get or generate octree
	ccOctree::Shared oct = cloud->getOctree();
	if (!oct)
	{
		oct = cloud->computeOctree(); //if the user clicked "no" when asked to compute the octree then tough....
	}

	//nearest neighbour search
	float r = m_mouseCircle->getRadiusWorld();
	unsigned char level = oct->findBestLevelForAGivenNeighbourhoodSizeExtraction(r);
	CCLib::DgmOctree::NeighboursSet set;
	int n = oct->getPointsInSphericalNeighbourhood(P, PointCoordinateType(r), set, level);
	//Put data in a point cloud class and encapsulate as a "neighbourhood"
	CCLib::DgmOctreeReferenceCloud nCloud(&set, n);
	CCLib::Neighbourhood Z(&nCloud);

	//Fit plane!
	double rms = 0.0; //output for rms
	ccFitPlane* pPlane = ccFitPlane::Fit(&nCloud, &rms);

	if (pPlane) //valid fit
	{
		pPlane->updateAttributes(rms, m_mouseCircle->getRadiusWorld());

		//make plane to add to display
		pPlane->setVisible(true);
		pPlane->setSelectionBehavior(ccHObject::SELECTION_IGNORED);

		//add plane to scene graph
		insertPoint->addChild(pPlane);
		pPlane->setDisplay(m_app->getActiveGLWindow());
		pPlane->prepareDisplayForRefresh_recursive(); //not sure what this does, but it looks like fun

		//add plane to TOC
		m_app->addToDB(pPlane, false, false, false, false);

		//report orientation to console for convenience
		m_app->dispToConsole(QString("[ccCompass] Surface orientation estimate = " + pPlane->getName()), ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}
}
