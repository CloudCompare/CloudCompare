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

//! Tool used to digitise traces
class ccTraceTool :	public ccTool
{
public:
	ccTraceTool();

	//! Called when the tool is set to active (for initialization)
	void toolActivated() override;

	//! Called when the tool is disactivated by a tool change or similar
	void toolDisactivated() override;

	//! Called when a point in a point cloud gets picked while this tool is active
	void pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P) override;

	//! Called when a new selection is made
	void onNewSelection(const ccHObject::Container& selectedEntities) override;

	//! Called when "Return" or "Space" is pressed, or the "Accept Button" is clicked
	void accept() override;

	//! Called when the "Escape" is pressed, or the "Cancel" button is clicked
	void cancel() override;

	//! IKf this returns true, the undo button is enabled in the gui
	bool canUndo() override;

	//! Called when the undo button is clicked
	void undo() override;

protected:
	//! Finishes and finalises the trace currently being digitised to
	void finishCurrentTrace();
	//! If obj is a ccTrace, it becomes the active trace. Returns true if succesfull
	bool pickupTrace(ccHObject* obj);
	
	//properties of the active trace
	int m_trace_id = -1; //!< Active trace id (stored rather than a pointer to avoid dead pointers after users delete objects in the DB_Tree)
	bool m_preExisting = false; //!< Set to true when a trace is picked up from a selection (so we don't delete it on cancel).
	bool m_changed = false; //!< Becomes true if changes have been made. Used to update fit planes

	bool m_parentPlaneDeleted = false; //!< True if parent plane was deleted
	bool m_childPlaneDeleted = false; //!< True if child plane was deleted

	bool m_precompute_gradient = true; //!< Do we want to precompute gradient for cost function?
	bool m_precompute_curvature = true; //!< Do we want to precompute curvature for cost functions?
};

#endif
