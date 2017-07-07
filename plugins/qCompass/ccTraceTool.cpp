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
	//"pick-up" selected trace
	for (ccHObject* obj : m_app->getSelectedEntities())
	{
		pickupTrace(obj);
		if (m_trace)
			break; //bail as we've found an active trace object
	}
}

//called when a point in a point cloud gets picked while this tool is active
void ccTraceTool::pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P)
{
	//check that m_trace hasn't been deleted...
	if (m_trace && !m_app->dbRootObject()->find(m_trace_id))
	{
		//item has been deleted...
		m_trace = nullptr;
	}

	if (!m_trace)
	{
		//no active trace -> make a new one
		m_trace = new ccTrace(cloud);
		m_trace->setDisplay(m_window);
		m_trace->setVisible(true);
		m_trace->setName("Trace");
		m_trace->prepareDisplayForRefresh_recursive();
		m_trace_id = m_trace->getUniqueID();
		insertPoint->addChild(m_trace);
		m_app->addToDB(m_trace, false, false, false, false);
		m_app->setSelectedInDB(m_trace, true);
	}

	//add point
	int index = m_trace->insertWaypoint(itemIdx);

	//optimise points
	if (m_trace->waypoint_count() >= 2)
	{
		//check if m_trace has previously fitted planes before optimizing (and delete them if so)
		for (unsigned idx = 0; idx < m_trace->getChildrenNumber(); idx++)
		{
			ccHObject* child = m_trace->getChild(idx);
			if (child->hasMetaData("ccCompassType"))
			{
				if (child->getMetaData("ccCompassType").toString().contains("FitPlane")) //we've found a best-fit-plane -> remove this as it will no longer be valid.
				{
					m_app->removeFromDB(child); //n.b. removeFromDB also deletes the child
				}
			}
		}

		if (!m_trace->optimizePath())
		{
			m_app->dispToConsole(QString("[ccCompass] Failed to optimize trace path... please try again."), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			m_app->removeFromDB(m_trace);
			m_trace = nullptr; //kill trace
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
	if (m_trace)
	{
		if (m_app->dbRootObject()->find(m_trace_id))
		{
			m_app->removeFromDB(m_trace); //delete trace object
		}
		m_trace = nullptr;
	}
}

void ccTraceTool::onNewSelection(const ccHObject::Container& selectedEntities)
{
		//can we pick up a new one?
		for (size_t i = 0; i < selectedEntities.size(); i++)
		{
			if (selectedEntities[i] != m_trace) //can't pick up the already active trace...
			{
				pickupTrace(selectedEntities[i]);
				if (m_trace)
					break; //bail as we've found an active trace object
			}
		}
}

void ccTraceTool::finishCurrentTrace()
{
	//finish current trace [if there is one]
	if (m_trace)
	{
		//check that m_trace hasn't been deleted...
		if (!m_app->dbRootObject()->find(m_trace_id))
		{
			//item has been deleted...
			m_trace = nullptr;
			return;
		}

		m_trace->finalizePath();
		m_trace->setWaypointColor(ccColor::red);
		m_trace->setTraceColor(ccColor::red);

		//fit plane
		if (ccCompass::fitPlanes)
		{
			//check if a fit-plane already exists (and bail if it does)
			bool calculateFitPlane = true;
			for (unsigned idx = 0; idx < m_trace->getChildrenNumber(); idx++)
			{
				ccHObject* child = m_trace->getChild(idx);

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
				ccPlane* p = m_trace->fitPlane();
				if (p)
				{
					p->setVisible(true);
					p->setSelectionBehavior(ccHObject::SELECTION_IGNORED);
					p->showNormals(true);
					m_trace->addChild(p);

					//report orientation to console for convenience
					m_app->dispToConsole(QString("[ccCompass] Trace orientation estimate = " + p->getName()), ccMainAppInterface::STD_CONSOLE_MESSAGE);
				} else
				{
					m_app->dispToConsole(QString("[ccCompass] Not enough topography to fit plane to trace."), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				}
			}
		}
		m_app->setSelectedInDB(m_trace, false); //deselect this trace
	}
	m_trace = nullptr;
	m_window->redraw();
}

void ccTraceTool::pickupTrace(ccHObject* obj)
{
	//is selected object a ccTrace?
	ccTrace* t = dynamic_cast<ccTrace*>(obj); //try casting to ccTrace
	if (t) //the object is a ccTrace!
	{
		//finish the previous trace
		if (m_trace)
		{
			finishCurrentTrace();
		}

		//activate selected trace
		m_trace = t;
		m_trace->setTraceColor(ccColor::yellow);
		m_trace->setWaypointColor(ccColor::green);
	}
}

//if this returns true, the undo button is enabled in the gui
boolean ccTraceTool::canUndo()
{
	return true; //yes - we can undo!
}

//called when the undo button is clicked
void ccTraceTool::undo()
{
	//check not deleted
	if (!m_app->dbRootObject()->find(m_trace_id))
	{
		m_trace = 0;
	}

	if (m_trace)
	{
		m_trace->undoLast();
		m_trace->optimizePath();
	}
	m_window->redraw();
}