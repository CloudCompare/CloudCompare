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

#ifndef CC_TRACETOOL_HEADER
#define CC_TRACETOOL_HEADER

#include "ccTool.h"
#include "ccTrace.h"

/*
Tool used to digitise traces
*/
class ccTraceTool :
	public ccTool
{
public:
	ccTraceTool();
	virtual ~ccTraceTool() = default;

	//called when the tool is set to active (for initialization)
	void toolActivated() override;

	//called when the tool is disactivated by a tool change or similar
	void toolDisactivated() override;

	//called when a point in a point cloud gets picked while this tool is active
	void pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P) override;

	//called when a new selection is made
	void onNewSelection(const ccHObject::Container& selectedEntities) override;

	//called when "Return" or "Space" is pressed, or the "Accept Button" is clicked
	void accept() override;

	//called when the "Escape" is pressed, or the "Cancel" button is clicked
	void cancel() override;

	//if this returns true, the undo button is enabled in the gui
	bool canUndo() override;

	//called when the undo button is clicked
	void undo() override;

protected:
	//finishes and finalises the trace currently being digitised to
	void finishCurrentTrace();
	bool pickupTrace(ccHObject* obj); //if obj is a ccTrace, it becomes the active trace. Returns true if succesfull
	
	//properties of the active trace
	int m_trace_id = -1; //active trace id (stored rather than a pointer to avoid dead pointers after users delete objects in the DB_Tree)
	bool m_preExisting = false; //set to true when a trace is picked up from a selection (so we don't delete it on cancel).
	bool m_changed = false; //becomes true if changes have been made. Used to update fit planes

	bool m_parentPlaneDeleted = false; //true if parent plane was deleted
	bool m_childPlaneDeleted = false; //true if child plane was deleted

	bool m_precompute_gradient = true; //do we want to precompute gradient for cost function?
	bool m_precompute_curvature = true; //do we want to precompute curvature for cost functions?
};

#endif
