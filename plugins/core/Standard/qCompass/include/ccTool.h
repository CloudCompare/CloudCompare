#pragma once

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

#include <ccHObject.h>
#include <ccPointCloud.h>
#include <ccMainAppInterface.h>
#include <ccGLWindowInterface.h>

//! Template class that defining the basic functionality of qCompass "tools". 
class ccTool
{
public:

	virtual ~ccTool() {}
	
	void initializeTool(ccMainAppInterface* app)
	{
		m_app = app; //store copy of app
		m_window = m_app->getActiveGLWindow();
	}

	//! Called when the tool is set to active (for initialization)
	virtual void toolActivated() { }

	//! Called when the tool is set to disactive (for cleanup)
	virtual void toolDisactivated() { }

	//! Called when a point in a point cloud gets picked while this tool is active
	/** \return Whether the information was used or not (if not, the other pointPicked method will be called)
	*/
	virtual bool pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccHObject* pickedObject, const CCVector3& P) { return false; }

	//! Called when a point in a point cloud gets picked while this tool is active
	virtual void pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P) { }

	//! Called when the selection is changed while this tool is active
	virtual void onNewSelection(const ccHObject::Container& selectedEntities) { }

	//! Called when "Return" or "Space" is pressed, or the "Accept Button" is clicked
	virtual void accept() { }

	//! Called when the "Escape" is pressed, or the "Cancel" button is clicked
	virtual void cancel() { }

	//! If this returns true, the undo button is enabled in the gui
	virtual bool canUndo() { return false; }

	//! Called when the undo button is clicked
	virtual void undo()	{ }

protected:
	
	ccTool() :
		m_app( nullptr )
	  , m_window( nullptr )
	{
	}

	//! Link to the main plugin interface
	ccMainAppInterface* m_app;

	//! Link to the active openGLWindow
	ccGLWindowInterface* m_window;
};
