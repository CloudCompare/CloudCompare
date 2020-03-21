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

#include "ccNoteTool.h"

ccNoteTool::ccNoteTool()
	: ccTool()
{
}

//called when a point in a point cloud gets picked while this tool is active
void ccNoteTool::pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P)
{
	//get note text
	QString note = QInputDialog::getText(m_app->getMainWindow(), "Note", "Contents:", QLineEdit::Normal, "Write note here.");

	if (note == "")
	{
		return;
	}

	//create a 1-point lineation object (highlights note-location)
	ccPointPair* l = new ccNote(cloud);
	l->setName(note);
	l->addPointIndex(itemIdx);

	//find insert point
	ccHObject* notesFolder = nullptr;
	for (unsigned i = 0; i < m_app->dbRootObject()->getChildrenNumber(); i++)
	{
		if (m_app->dbRootObject()->getChild(i)->getName() == "notes")
		{
			notesFolder = m_app->dbRootObject()->getChild(i);
		}
		else
		{
			//also search first-level children of root node (when files are re-loaded this is where things will sit)
			for (unsigned c = 0; c < m_app->dbRootObject()->getChild(i)->getChildrenNumber(); c++)
			{
				if (m_app->dbRootObject()->getChild(i)->getChild(c)->getName() == "notes")
				{
					notesFolder = m_app->dbRootObject()->getChild(i)->getChild(c);
					break;
				}
			}
		}
		if (notesFolder) //found one :)
		{
			break;
		}
	}
	if (!notesFolder)
	{
		notesFolder = new ccHObject("notes");
		m_app->dbRootObject()->addChild(notesFolder);
		m_app->addToDB(notesFolder, false, false, false, false);
	}

	//add to scene graph
	notesFolder->addChild(l);
	m_app->addToDB(l);
}


//called when the tool is set to active (for initialization)
void ccNoteTool::toolActivated()
{ 
	//donothing
}

//called when the tool is set to disactive (for cleanup)
void ccNoteTool::toolDisactivated()
{
	//donothing
}