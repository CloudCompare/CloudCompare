#ifndef CC_TOOL_HEADER
#define CC_TOOL_HEADER

#include <ccHObject.h>
#include <ccPointCloud.h>
#include <ccMainAppInterface.h>
#include <ccGLWindow.h>

class ccTool
{
public:

	ccTool()
	{
	}

	~ccTool()
	{
	}

	virtual void initializeTool(ccMainAppInterface* app)
	{
		m_app = app; //store copy of app
		m_window = m_app->getActiveGLWindow();
	}
	//called when the tool is set to active (for initialization)
	virtual void toolActivated() { };

	//called when the tool is set to disactive (for cleanup)
	virtual void toolDisactivated() { };

	//called when a point in a point cloud gets picked while this tool is active
	virtual void pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccHObject* pickedObject, const CCVector3& P) { };

	//called when a point in a point cloud gets picked while this tool is active
	virtual void pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P) { };

	//called when the selection is changed while this tool is active
	virtual void onNewSelection(const ccHObject::Container& selectedEntities) { };

	//called when "Return" or "Space" is pressed, or the "Accept Button" is clicked
	virtual void accept() { }

	//called when the "Escape" is pressed, or the "Cancel" button is clicked
	virtual void cancel() { }

	//if this returns true, the undo button is enabled in the gui
	virtual boolean canUndo() { return false; }

	//called when the undo button is clicked
	virtual void undo()	{ }
protected:
	ccMainAppInterface* m_app;
	ccGLWindow*  m_window;
};

#endif