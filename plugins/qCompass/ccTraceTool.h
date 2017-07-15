#ifndef CC_TRACETOOL_HEADER
#define CC_TRACETOOL_HEADER

#include "cctool.h"
#include "ccTrace.h"

class ccTraceTool :
	public ccTool
{
public:
	ccTraceTool();
	~ccTraceTool();

	//called when the tool is set to active (for initialization)
	void toolActivated() override;

	//called when a point in a point cloud gets picked while this tool is active
	void pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P) override;

	//called when a new selection is made
	void onNewSelection(const ccHObject::Container& selectedEntities) override;

	//called when "Return" or "Space" is pressed, or the "Accept Button" is clicked
	void accept() override;

	//called when the "Escape" is pressed, or the "Cancel" button is clicked
	void cancel() override;

	//if this returns true, the undo button is enabled in the gui
	boolean canUndo() override;

	//called when the undo button is clicked
	void undo() override;

protected:
	void finishCurrentTrace();
	bool pickupTrace(ccHObject* obj); //if obj is a ccTrace, it becomes the active trace. Returns true if succesfull
	
	//properties of the active trace
	int m_trace_id = -1; //active trace id (stored rather than a pointer to avoid dead pointers after users delete objects in the DB_Tree)
	bool m_preExisting = false; //set to true when a trace is picked up from a selection (so we don't delete it on cancel).
};

#endif
