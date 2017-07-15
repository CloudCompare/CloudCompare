#include "ccTraceTool.h"
#include "ccCompass.h"

ccTraceTool::ccTraceTool()
	: ccTool( )
{
}


ccTraceTool::~ccTraceTool()
{
}

//called when the tool is set to active (for initialization)
void ccTraceTool::toolActivated()
{
	//try "pick-up" selected trace
	onNewSelection(m_app->getSelectedEntities());
}

//called when a point in a point cloud gets picked while this tool is active
void ccTraceTool::pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P)
{
	//try and fetch the trace object (returns null if the id is invalid)
	ccTrace* t = dynamic_cast<ccTrace*>(m_app->dbRootObject()->find(m_trace_id));

	//no active trace -> make a new one
	if (!t)
	{
		t = new ccTrace(cloud);
		t->setDisplay(m_window);
		t->setVisible(true);
		t->setName("Trace");
		t->prepareDisplayForRefresh_recursive();
		m_trace_id = t->getUniqueID();
		insertPoint->addChild(t);
		m_app->addToDB(t, false, false, false, false);
		t->setActive(true);
		m_app->setSelectedInDB(t, true);
		m_preExisting = false;
	}

	//add point
	int index = t->insertWaypoint(itemIdx);

	//optimise points
	if (t->waypoint_count() >= 2)
	{
		//check if m_trace has previously fitted planes before optimizing (and delete them if so)
		for (unsigned idx = 0; idx < t->getChildrenNumber(); idx++)
		{
			ccHObject* child = t->getChild(idx);
			if (child->hasMetaData("ccCompassType"))
			{
				if (child->getMetaData("ccCompassType").toString().contains("FitPlane")) //we've found a best-fit-plane -> remove this as it will no longer be valid.
				{
					m_app->removeFromDB(child); //n.b. removeFromDB also deletes the child
				}
			}
		}

		if (!t->optimizePath()) //optimize the path!
		{
			//... problem?
			m_app->dispToConsole(QString("[ccCompass] Failed to optimize trace path... please try again."), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			t->undoLast(); //go back a step

			if (t->size() < 2) //degenerate trace - delete
			{
				m_app->removeFromDB(t);
				m_trace_id = -1; //start from scratch next time
			}
		}
	}
}

//called when "Return" or "Space" is pressed, or the "Accept Button" is clicked
void ccTraceTool::accept()
{
	//finish trace
	finishCurrentTrace();
}

//called when the "Escape" is pressed, or the "Cancel" button is clicked
void ccTraceTool::cancel()
{
	ccTrace* t = dynamic_cast<ccTrace*>(m_app->dbRootObject()->find(m_trace_id));

	if (t)
	{
		t->setActive(false); //disactivate trace
		
		if (!m_preExisting) //delete new traces (i.e. that were "picked-up" by changing the selection)
		{
			m_app->removeFromDB(t); //delete trace object
			m_trace_id = -1;
		}
	}
}

void ccTraceTool::onNewSelection(const ccHObject::Container& selectedEntities)
{
	//can we pick up a new trace?
	if (selectedEntities.size() > 0) //non-empty selection
	{
		m_app->dispToConsole(selectedEntities[0]->getName(), ccMainAppInterface::STD_CONSOLE_MESSAGE);

		//selection is the already active trace?
		if (selectedEntities[0]->getUniqueID() == m_trace_id) 
		{
			return; //we're already on it
		}
		else if (pickupTrace(selectedEntities[0])) //try pick-up the selection
		{
			return; //bail - we've found our new active trace object
		}
		else if (selectedEntities[0]->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			m_app->dispToConsole("can I delete this?");
			return; //entity is a point cloud -> this often happens during standard picking
		}
	}

	// new selection is not a trace - finish the last one
	finishCurrentTrace();
	m_trace_id = -1; 
}

void ccTraceTool::finishCurrentTrace()
{
	//find current trace [if there is one]
	ccTrace* t = dynamic_cast<ccTrace*>(m_app->dbRootObject()->find(m_trace_id));

	if (t)
	{
		t->finalizePath();
		t->setActive(false);

		//fit plane
		if (ccCompass::fitPlanes)
		{
			//check if a fit-plane already exists (and bail if it does)
			bool calculateFitPlane = true;
			for (unsigned idx = 0; idx < t->getChildrenNumber(); idx++)
			{
				ccHObject* child = t->getChild(idx);

				if (child->hasMetaData("ccCompassType"))
				{
					if (child->getMetaData("ccCompassType").toString().contains("FitPlane")) //we've found a best-fit-plane - bail
					{
						m_app->dispToConsole(QString("[ccCompass] Trace orientation estimate already exists ( " + child->getName() + ")"), ccMainAppInterface::STD_CONSOLE_MESSAGE);
						calculateFitPlane = false;
						break;
					}
				}
			}

			if (calculateFitPlane) //plane has not (yet) been calculated - go ahead and do some magic
			{
				ccPlane* p = t->fitPlane();
				if (p)
				{
					p->setVisible(true);
					p->setSelectionBehavior(ccHObject::SELECTION_IGNORED);
					p->showNormals(true);
					t->addChild(p);

					//report orientation to console for convenience
					m_app->dispToConsole(QString("[ccCompass] Trace orientation estimate = " + p->getName()), ccMainAppInterface::STD_CONSOLE_MESSAGE);
				} else
				{
					m_app->dispToConsole(QString("[ccCompass] Not enough topography to fit plane to trace."), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				}
			}
		}
		m_app->setSelectedInDB(t, false); //deselect this trace
		m_app->setSelectedInDB(t->getParent(), true); //select it's parent instead

		m_trace_id = -1;
		m_window->redraw();
	}
}

bool ccTraceTool::pickupTrace(ccHObject* obj)
{
	//is selected object a ccTrace?
	ccTrace* t = dynamic_cast<ccTrace*>(obj); //try casting to ccTrace
	if (t) //the object is a ccTrace!
	{
		//finish the previous trace
		finishCurrentTrace();

		//activate selected trace
		t->setActive(true);
		m_trace_id = t->getUniqueID();
		m_preExisting = true;

		return true;
	}
	return false;
}

//if this returns true, the undo button is enabled in the gui
boolean ccTraceTool::canUndo()
{
	return true; //yes - we can undo!
}

//called when the undo button is clicked
void ccTraceTool::undo()
{
	ccTrace* t = dynamic_cast<ccTrace*>(m_app->dbRootObject()->find(m_trace_id));
	if (t)
	{
		t->undoLast();
		t->optimizePath();
		m_window->redraw();
	}
}