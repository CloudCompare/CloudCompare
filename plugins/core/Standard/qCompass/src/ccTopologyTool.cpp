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

#include "ccTopologyTool.h"
#include "ccCompass.h"

int ccTopologyTool::RELATIONSHIP = ccTopologyRelation::YOUNGER_THAN;

ccTopologyTool::ccTopologyTool()
	: ccTool()
{
}

//called when the selection is changed while this tool is active
void ccTopologyTool::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (selectedEntities.size() == 0)
		return;

	//is selection a geoObject?
	ccGeoObject* o = ccGeoObject::getGeoObjectParent(selectedEntities[0]);
	if (o)
	{
		//yes it is a GeoObject - have we got a first point?
		ccHObject* first = m_app->dbRootObject()->find(m_firstPick);
		if (!first) //no... this is the first point (m_firstPick is invalid)
		{
			m_firstPick = o->getUniqueID();

			//write instructions to screen
			m_app->getActiveGLWindow()->displayNewMessage("Select second (younger) GeoObject.", ccGLWindow::LOWER_LEFT_MESSAGE);
		}
		else //yes.. this is the second pick
		{
			ccGeoObject* g1 = static_cast<ccGeoObject*>(first); //n.b. this *should* always be a GeoObject....
			
			//add topology relation!
			g1->addRelationTo(o, ccTopologyTool::RELATIONSHIP, m_app);

			//reset...
			accept();

		}
	}
	else
	{
		//no - throw error
		m_app->dispToConsole("[ccCompass] Please select a GeoObject", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	}
}

//called when the tool is set to active (for initialization)
void ccTopologyTool::toolActivated()
{ 
	//display instructions
	m_app->getActiveGLWindow()->displayNewMessage("Select first (older) GeoObject.", ccGLWindow::LOWER_LEFT_MESSAGE);
}

//called when the tool is set to disactive (for cleanup)
void ccTopologyTool::toolDisactivated()
{
	m_firstPick = -1; //reset
}

//called when "Return" or "Space" is pressed, or the "Accept Button" is clicked
void ccTopologyTool::accept()
{
	//Reset the tool
	toolDisactivated();

	//restart picking mode
	toolActivated();
}

//called when the "Escape" is pressed, or the "Cancel" button is clicked
void ccTopologyTool::cancel()
{
	toolDisactivated();
}

