#include "ccNoteTool.h"

ccNoteTool::ccNoteTool()
	: ccTool()
{
}

ccNoteTool::~ccNoteTool()
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
	ccPointPair* l = new ccPointPair(cloud);
	l->setName(note);
	l->showNameIn3D(true);
	l->addPointIndex(itemIdx);
	l->setDefaultColor(ccColor::cyan);
	l->setActiveColor(ccColor::red);

	//add to scene graph
	ccHObject* notesFolder = nullptr;
	for (int i = 0; i < m_app->dbRootObject()->getChildrenNumber(); i++)
	{
		if (m_app->dbRootObject()->getChild(i)->getName() == "Notes")
		{
			notesFolder = m_app->dbRootObject()->getChild(i);
			break;
		}
	}
	if (!notesFolder)
	{
		notesFolder = new ccHObject("Notes");
		m_app->dbRootObject()->addChild(notesFolder);
		m_app->addToDB(notesFolder, false, false, false, false);
	}

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