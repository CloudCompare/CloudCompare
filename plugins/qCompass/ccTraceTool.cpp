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

	//if cost function is gradient/curvature then check appropriate SFs have been defined
	if (ccTrace::COST_MODE & ccTrace::GRADIENT) //gradient cost is active
	{
		if (m_precompute_gradient & !t->isGradientPrecomputed()) //not already computed
		{
			//give user a chance to bail - computations can take looooong
			QMessageBox::StandardButton q;
			q = QMessageBox::question(m_app->getMainWindow(), "Calculate gradient?", "Precompute Gradient? This can be slow, but once complete will greatly decrease future computation times.", QMessageBox::Yes | QMessageBox::No);
			if (q == QMessageBox::Yes) //do compute
			{
				t->buildGradientCost(m_app->getMainWindow());
			}
			else
			{
				m_precompute_gradient = false; //only need to do this once
			}
		}
	}
	if (ccTrace::COST_MODE & ccTrace::CURVE) //curvature cost is active
	{
		if (m_precompute_curvature & !t->isCurvaturePrecomputed()) //not already computed?
		{
			QMessageBox::StandardButton q;
			q = QMessageBox::question(m_app->getMainWindow(), "Calculate curvature?", "Precompute Curvature? This can be slow, but once complte will greatly decrease future computation times.", QMessageBox::Yes | QMessageBox::No);
			if (q == QMessageBox::Yes) //do compute
			{
				t->buildCurvatureCost(m_app->getMainWindow());
			}
			else
			{
				m_precompute_curvature = false; //only need to do once
			}
		}
	}

	//add point
	int index = t->insertWaypoint(itemIdx);

	//optimise points
	if (t->waypoint_count() >= 2)
	{
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

		// new selection is not a trace - finish the last one
		finishCurrentTrace();
	}
}

void ccTraceTool::finishCurrentTrace()
{
	//find current trace [if there is one]
	ccTrace* t = dynamic_cast<ccTrace*>(m_app->dbRootObject()->find(m_trace_id));

	if (t)
	{
		t->finalizePath();
		t->setActive(false);

		//check for shift key modifier (flips the fitPlane modifier)
		bool fitPlane = ccCompass::fitPlanes;
		if (QApplication::keyboardModifiers().testFlag(Qt::ShiftModifier)) //shift is pressed
		{
			fitPlane = !fitPlane;
		}

		//fit plane
		if (fitPlane)
		{
			//check if a fit-plane already exists (and bail if it does)
			bool calculateFitPlane = true;

			//is one of our children the fit plane (planes made in map mode)?
			for (unsigned idx = 0; idx < t->getChildrenNumber(); idx++)
			{
				ccHObject* child = t->getChild(idx);

				if (ccFitPlane::isFitPlane(child)) //we've found a fit plane - bail
				{
						m_app->dispToConsole(QString("[ccCompass] Trace orientation estimate already exists ( " + child->getName() + ")"), ccMainAppInterface::STD_CONSOLE_MESSAGE);
						calculateFitPlane = false;
						break;
				}
			}

			//is our parent the fit plane (planes made in Compass mode)?
			if (ccFitPlane::isFitPlane(t->getParent()))
			{
				m_app->dispToConsole(QString("[ccCompass] Trace orientation estimate already exists ( " + t->getParent()->getName() + ")"), ccMainAppInterface::STD_CONSOLE_MESSAGE);
				calculateFitPlane = false;
			}

			if (calculateFitPlane) //plane has not (yet) been calculated - go ahead and do some magic
			{
				ccPlane* p = t->fitPlane();
				if (p)
				{
					p->setVisible(true);
					p->setSelectionBehavior(ccHObject::SELECTION_IGNORED);
					p->showNormals(true);

					//are we associated with a geoObject and in compass mode?

					if (!ccCompass::mapMode) //compass mode - trace becomes a child of the plane
					{
						//yes - add trace as child of plane
						ccHObject* parent = t->getParent();
						parent->detachChild(t); //remove trace from it's parent
						p->addChild(t); //add trace as child of plane
						t->setVisible(false); //hide trace
						parent->addChild(p); //add plane as child
						m_app->addToDB(p);
					}
					else  //map mode - plane becomes a child of trace
					{
						t->addChild(p);
					}
					
					//report orientation to console for convenience
					m_app->dispToConsole(QString("[ccCompass] Trace orientation estimate = " + p->getName()), ccMainAppInterface::STD_CONSOLE_MESSAGE);
				}
				else
				{
					m_app->dispToConsole(QString("[ccCompass] Not enough topography to fit plane to trace."), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				}
			}
		}

		m_trace_id = -1; //forget previous trace
		m_app->setSelectedInDB(t, false); //deselect this trace
		m_app->setSelectedInDB(t->getParent(), true); //select it's parent instead

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
		t->setVisible(true);
		m_preExisting = true;

		//check if trace has previously fitted planes before optimizing (and delete them if so)
		if (ccFitPlane::isFitPlane(t->getParent()))
		{
			ccHObject* parent = t->getParent();
			parent->detachChild(t); //remove trace from the plane
			parent->getParent()->addChild(t); //add trace to the plane's parent
			m_app->removeFromDB(parent); //delete plane
			m_app->addToDB(t); //add trace to db
		}
		else
		{
			for (unsigned idx = 0; idx < t->getChildrenNumber(); idx++)
			{
				ccHObject* child = t->getChild(idx);
				if (ccFitPlane::isFitPlane(child))
				{
					m_app->removeFromDB(child); //n.b. removeFromDB also deletes the child
				}
			}
		}

		t->setActive(true);
		m_trace_id = t->getUniqueID();
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