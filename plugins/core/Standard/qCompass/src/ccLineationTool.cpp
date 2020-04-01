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

#include "ccLineationTool.h"
#include "ccCompass.h"

ccLineationTool::ccLineationTool()
	: ccTool()
{
}

//called when the tool is set to disactive (for cleanup)
void ccLineationTool::toolDisactivated()
{
	cancel();
}

//called when a point in a point cloud gets picked while this tool is active
void ccLineationTool::pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P)
{
	//try retrieve active lineation (will fail if there isn't one)
	ccLineation* l = dynamic_cast<ccLineation*>(m_app->dbRootObject()->find(m_lineation_id));
	if (!l) //make a new one
	{
		//no active trace -> make a new one
		l = new ccLineation(cloud);
		m_lineation_id = l->getUniqueID();

		//set drawing properties
		l->setDisplay(m_window);
		l->setVisible(true);
		l->setName("Lineation");
		l->prepareDisplayForRefresh_recursive();

		//add to DB Tree
		insertPoint->addChild(l);
		m_app->addToDB(l, false, false, false, false);
	} 

	//add point
	int index = l->addPointIndex(itemIdx);

	//is this the end point?
	if (l->size() == 2)
	{
		l->updateMetadata(); //calculate orientation & store. Also changes the name.
		l->showNameIn3D(ccCompass::drawName);

		//report orientation to console for convenience
		m_app->dispToConsole(QString("[ccCompass] Lineation = " + l->getName()), ccMainAppInterface::STD_CONSOLE_MESSAGE);

		//start new lineation
		m_lineation_id = -1;
	}
}

//called when "Return" or "Space" is pressed, or the "Accept Button" is clicked
void ccLineationTool::accept()
{
	cancel(); //removes any incomplete lineations
}

//called when the "Escape" is pressed, or the "Cancel" button is clicked
void ccLineationTool::cancel()
{
	if (m_lineation_id != -1) //there is an active lineation
	{
		ccPointPair* l = dynamic_cast<ccPointPair*>(m_app->dbRootObject()->find(m_lineation_id));
		if (l && l->size() < 2)
		{
			m_app->removeFromDB(l); //remove incomplete lineation
			m_lineation_id = -1;
		}
	}
}

