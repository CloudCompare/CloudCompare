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

//Qt
#include <QFileDialog>
#include <QFileInfo>

#include <ccPickingHub.h>

#include "ccCompass.h"
#include "ccCompassDlg.h"
#include "ccCompassInfo.h"
#include "ccFitPlaneTool.h"
#include "ccGeoObject.h"
#include "ccLineationTool.h"
#include "ccMapDlg.h"
#include "ccNoteTool.h"
#include "ccPinchNodeTool.h"
#include "ccThicknessTool.h"
#include "ccTopologyTool.h"
#include "ccTraceTool.h"

//initialize default static pars
bool ccCompass::drawName = false;
bool ccCompass::drawStippled = true;
bool ccCompass::drawNormals = true;
bool ccCompass::fitPlanes = true;
int ccCompass::costMode = ccTrace::DARK;
bool ccCompass::mapMode = false;
int ccCompass::mapTo = ccGeoObject::LOWER_BOUNDARY;

ccCompass::ccCompass(QObject* parent/*=0*/)
	: QObject(parent)
	, m_action(0)
{
	//initialize all tools
	m_fitPlaneTool = new ccFitPlaneTool();
	m_traceTool = new ccTraceTool();
	m_lineationTool = new ccLineationTool();
	m_thicknessTool = new ccThicknessTool();
	m_topologyTool = new ccTopologyTool();
	m_noteTool = new ccNoteTool();
	m_pinchNodeTool = new ccPinchNodeTool();
}

//deconstructor
ccCompass::~ccCompass()
{
	//delete all tools
	delete m_fitPlaneTool;
	delete m_traceTool;
	delete m_lineationTool;
	delete m_thicknessTool;
	delete m_topologyTool;
	delete m_noteTool;
	delete m_pinchNodeTool;

	if (m_dlg)
		delete m_dlg;
	if (m_mapDlg)
		delete m_mapDlg;
}

void ccCompass::onNewSelection(const ccHObject::Container& selectedEntities)
{
	//disable the main plugin icon if no entity is loaded
	m_action->setEnabled(m_app && m_app->dbRootObject() && m_app->dbRootObject()->getChildrenNumber() != 0);

	if (!m_dlg | !m_mapDlg)
	{
		return; //not initialized yet - ignore callback
	}

	if (m_activeTool)
	{
		m_activeTool->onNewSelection(selectedEntities); //pass on to the active tool
	}

	//clear GeoObject selection & disable associated GUI
	if (m_geoObject)
	{
		m_geoObject->setActive(false);
	}
	m_geoObject = nullptr;
	m_geoObject_id = -1;
	if (m_mapDlg)
	{
		m_mapDlg->setLowerButton->setEnabled(false);
		m_mapDlg->setUpperButton->setEnabled(false);
		m_mapDlg->setInteriorButton->setEnabled(false);
		m_mapDlg->selectionLabel->setEnabled(false);
		m_mapDlg->selectionLabel->setText("No Selection");
	}
	//has a GeoObject (or a child of one?) been selected?
	for (ccHObject* obj : selectedEntities)
	{
		//recurse upwards looking for geoObject & relevant part (interior, upper, lower)
		ccHObject* o = obj;
		bool interior = false;
		bool upper = false;
		bool lower = false;
		while (o)
		{
			interior = interior || ccGeoObject::isGeoObjectInterior(o);
			upper = upper || ccGeoObject::isGeoObjectUpper(o);
			lower = lower || ccGeoObject::isGeoObjectLower(o);

			//have we found a geoObject?
			if (ccGeoObject::isGeoObject(o))
			{
				//found one!
				m_geoObject = static_cast<ccGeoObject*>(o);
				if (m_geoObject) //cast succeeded
				{
					m_geoObject_id = m_geoObject->getUniqueID(); //store id
					m_geoObject->setActive(true); //display as "active"

					//activate GUI
					m_mapDlg->setLowerButton->setEnabled(true);
					m_mapDlg->setUpperButton->setEnabled(true);
					m_mapDlg->setInteriorButton->setEnabled(true);
					m_mapDlg->selectionLabel->setEnabled(true);
					m_mapDlg->selectionLabel->setText(m_geoObject->getName());

					//set appropriate upper/lower/interior setting on gui
					if (interior)
					{
						writeToInterior();
					}
					else if (upper)
					{
						writeToUpper();
					}
					else if (lower)
					{
						writeToLower();
					}

					//done!
					return; 
				}
			}

			//next parent
			o = o->getParent();
		}
	}
}

//Submit the action to launch ccCompass to CC
void ccCompass::getActions(QActionGroup& group)
{
	//default action (if it has not been already created, it's the moment to do it)
	if (!m_action) //this is the action triggered by clicking the "Compass" button in the plugin menu
	{
		//here we use the default plugin name, description and icon,
		//but each action can have its own!
		m_action = new QAction(getName(), this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());

		//connect appropriate signal
		QObject::connect(m_action, SIGNAL(triggered()), this, SLOT(doAction())); //this binds the m_action to the ccCompass::doAction() function
	}
	group.addAction(m_action);

	m_app->dispToConsole("[ccCompass] ccCompass plugin initialized successfully.", ccMainAppInterface::STD_CONSOLE_MESSAGE);

}

//Called by CC when the plugin should be activated - sets up the plugin and then calls startMeasuring()
void ccCompass::doAction()
{
	//m_app should have already been initialized by CC when plugin is loaded!
	//(--> pure internal check)
	assert(m_app);

	//initialize tools (essentially give them a copy of m_app)
	m_traceTool->initializeTool(m_app);
	m_fitPlaneTool->initializeTool(m_app);
	m_lineationTool->initializeTool(m_app);
	m_thicknessTool->initializeTool(m_app);
	m_topologyTool->initializeTool(m_app);
	m_noteTool->initializeTool(m_app);
	m_pinchNodeTool->initializeTool(m_app);

	//check valid window
	if (!m_app->getActiveGLWindow())
	{
		m_app->dispToConsole("[ccCompass] Could not find valid 3D window.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//bind gui
	if (!m_dlg)
	{
		//bind GUI events
		m_dlg = new ccCompassDlg(m_app->getMainWindow());

		//general
		ccCompassDlg::connect(m_dlg->closeButton, SIGNAL(clicked()), this, SLOT(onClose()));
		ccCompassDlg::connect(m_dlg->acceptButton, SIGNAL(clicked()), this, SLOT(onAccept()));
		ccCompassDlg::connect(m_dlg->saveButton, SIGNAL(clicked()), this, SLOT(onSave()));
		ccCompassDlg::connect(m_dlg->undoButton, SIGNAL(clicked()), this, SLOT(onUndo()));
		ccCompassDlg::connect(m_dlg->infoButton, SIGNAL(clicked()), this, SLOT(showHelp()));

		//modes
		ccCompassDlg::connect(m_dlg->mapMode, SIGNAL(clicked()), this, SLOT(enableMapMode()));
		ccCompassDlg::connect(m_dlg->compassMode, SIGNAL(clicked()), this, SLOT(enableMeasureMode()));

		//tools
		ccCompassDlg::connect(m_dlg->pickModeButton, SIGNAL(clicked()), this, SLOT(setPick()));
		ccCompassDlg::connect(m_dlg->pairModeButton, SIGNAL(clicked()), this, SLOT(setLineation()));
		ccCompassDlg::connect(m_dlg->planeModeButton, SIGNAL(clicked()), this, SLOT(setPlane()));
		ccCompassDlg::connect(m_dlg->traceModeButton, SIGNAL(clicked()), this, SLOT(setTrace()));

		//extra tools
		ccCompassDlg::connect(m_dlg->m_pinchTool, SIGNAL(triggered()), this, SLOT(addPinchNode()));
		ccCompassDlg::connect(m_dlg->m_measure_thickness, SIGNAL(triggered()), this, SLOT(setThickness()));
		ccCompassDlg::connect(m_dlg->m_measure_thickness_twoPoint, SIGNAL(triggered()), this, SLOT(setThickness2()));

		ccCompassDlg::connect(m_dlg->m_youngerThan, SIGNAL(triggered()), this, SLOT(setYoungerThan()));
		ccCompassDlg::connect(m_dlg->m_follows, SIGNAL(triggered()), this, SLOT(setFollows()));
		ccCompassDlg::connect(m_dlg->m_equivalent, SIGNAL(triggered()), this, SLOT(setEquivalent()));

		ccCompassDlg::connect(m_dlg->m_mergeSelected, SIGNAL(triggered()), this, SLOT(mergeGeoObjects()));
		ccCompassDlg::connect(m_dlg->m_fitPlaneToGeoObject, SIGNAL(triggered()), this, SLOT(fitPlaneToGeoObject()));

		ccCompassDlg::connect(m_dlg->m_noteTool, SIGNAL(triggered()), this, SLOT(setNote()));

		ccCompassDlg::connect(m_dlg->m_toSVG, SIGNAL(triggered()), this, SLOT(exportToSVG()));

		//settings menu
		ccCompassDlg::connect(m_dlg->m_showNames, SIGNAL(toggled(bool)), this, SLOT(toggleLabels(bool)));
		ccCompassDlg::connect(m_dlg->m_showStippled, SIGNAL(toggled(bool)), this, SLOT(toggleStipple(bool)));
		ccCompassDlg::connect(m_dlg->m_showNormals, SIGNAL(toggled(bool)), this, SLOT(toggleNormals(bool)));
		ccCompassDlg::connect(m_dlg->m_recalculate, SIGNAL(triggered()), this, SLOT(recalculateSelectedTraces()));
	}

	if (!m_mapDlg)
	{
		m_mapDlg = new ccMapDlg(m_app->getMainWindow());

		ccCompassDlg::connect(m_mapDlg->addObjectButton, SIGNAL(clicked()), this, SLOT(addGeoObject()));
		ccCompassDlg::connect(m_mapDlg->setInteriorButton, SIGNAL(clicked()), this, SLOT(writeToInterior()));
		ccCompassDlg::connect(m_mapDlg->setUpperButton, SIGNAL(clicked()), this, SLOT(writeToUpper()));
		ccCompassDlg::connect(m_mapDlg->setLowerButton, SIGNAL(clicked()), this, SLOT(writeToLower()));
	}

	m_dlg->linkWith(m_app->getActiveGLWindow());
	m_mapDlg->linkWith(m_app->getActiveGLWindow());

	//loop through DB_Tree and find any ccCompass objects
	std::vector<int> originals; //ids of original objects
	std::vector<ccHObject*> replacements; //pointers to objects that will replace the originals
	for (unsigned i = 0; i < m_app->dbRootObject()->getChildrenNumber(); i++)
	{
		ccHObject* c = m_app->dbRootObject()->getChild(i);
		tryLoading(c, &originals, &replacements);
	}

	//replace all "originals" with their corresponding "duplicates"
	for (size_t i = 0; i < originals.size(); i++)
	{
		ccHObject* original = m_app->dbRootObject()->find(originals[i]);
		ccHObject* replacement = replacements[i];

		if (!original) //can't find for some reason?
			continue;
		if (!replacement) //can't find for some reason?
			continue;

		//steal all the children
		for (unsigned c = 0; c < original->getChildrenNumber(); c++)
		{
			replacement->addChild(original->getChild(c));
		}

		//remove them from the orignal parent
		original->detatchAllChildren(); 

		//add new parent to scene graph
		original->getParent()->addChild(replacement);

		//delete originals
		m_app->removeFromDB(original);

		//add replacement to dbTree
		m_app->addToDB(replacement, false, false, false, false);

		//is replacement a GeoObject? If so, "disactivate" it
		if (ccGeoObject::isGeoObject(replacement))
		{
			ccGeoObject* g = static_cast<ccGeoObject*>(replacement);
			g->setActive(false);
		}
	}

	//start in measure mode
	enableMeasureMode();

	//trigger selection changed
	onNewSelection(m_app->getSelectedEntities());

	//begin measuring
	startMeasuring();
}

void ccCompass::tryLoading(ccHObject* obj, std::vector<int>* originals, std::vector<ccHObject*>* replacements)
{
	//is object already represented by a ccCompass class?
	if (dynamic_cast<ccFitPlane*>(obj)
		|| dynamic_cast<ccTrace*>(obj)
		|| dynamic_cast<ccPointPair*>(obj) //n.b. several classes inherit from PointPair, so this cast will still succede for them
		|| dynamic_cast<ccGeoObject*>(obj))
	{
		return; //we need do nothing!
	}

	//recurse on children
	for (unsigned i = 0; i < obj->getChildrenNumber(); i++)
	{
		tryLoading(obj->getChild(i), originals, replacements);
	}

	//store parent of this object
	ccHObject* parent = obj->getParent();

	//are we a geoObject
	if (ccGeoObject::isGeoObject(obj))
	{
		ccHObject* geoObj = new ccGeoObject(obj,m_app);

		//add to originals/duplicates list [these are used later to overwrite the originals]
		originals->push_back(obj->getUniqueID());
		replacements->push_back(geoObj);
		return;

	}

	//are we a fit plane?
	if (ccFitPlane::isFitPlane(obj))
	{
		//cast to plane
		ccPlane* p = dynamic_cast<ccPlane*>(obj);
		if (p)
		{
			//create equivalent fit plane object
			ccHObject* plane = new ccFitPlane(p);

			//add to originals/duplicates list [these are used later to overwrite the originals]
			originals->push_back(obj->getUniqueID());
			replacements->push_back(plane);
			return;
		}
	}

	//is the HObject a polyline? (this will be the case for lineations & traces)
	ccPolyline* p = dynamic_cast<ccPolyline*>(obj);
	if (p)
	{
		//are we a trace?
		if (ccTrace::isTrace(obj))
		{

			ccHObject* trace = new ccTrace(p);
			//add to originals/duplicates list [these are used later to overwrite the originals]
			originals->push_back(obj->getUniqueID());
			replacements->push_back(trace);
			return;
		}

		//are we a lineation?
		if (ccLineation::isLineation(obj))
		{
			ccHObject* lin = new ccLineation(p);
			originals->push_back(obj->getUniqueID());
			replacements->push_back(lin);
			return;
		}

		//are we a thickness?
		if (ccThickness::isThickness(obj))
		{
			ccHObject* t = new ccThickness(p);
			originals->push_back(obj->getUniqueID());
			replacements->push_back(t);
			return;
		}

		//are we a topology relation?
		//todo

		//are we a pinchpiont
		if (ccPinchNode::isPinchNode(obj))
		{
			ccHObject* n = new ccPinchNode(p);
			originals->push_back(obj->getUniqueID());
			replacements->push_back(n);
			return;
		}

		//are we a note?
		if (ccNote::isNote(obj))
		{
			ccHObject* n = new ccNote(p);
			originals->push_back(obj->getUniqueID());
			replacements->push_back(n);
			return;
		}
	}
}

//Begin measuring 
bool ccCompass::startMeasuring()
{
	//check valid gl window
	if (!m_app->getActiveGLWindow())
	{
		//invalid pointer error
		m_app->dispToConsole("Error: ccCompass could not find the Cloud Compare window. Abort!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	//setup listener for mouse events
	m_app->getActiveGLWindow()->installEventFilter(this);

	//refresh window
	m_app->getActiveGLWindow()->redraw(true, false);

	//start GUI
	m_app->registerOverlayDialog(m_dlg, Qt::TopRightCorner);
	m_dlg->start();

	//activate active tool
	if (m_activeTool)
	{
		m_activeTool->toolActivated();
	}
	
	return true;
}

//Exits measuring
bool ccCompass::stopMeasuring()
{
	//remove click listener
	if (m_app->getActiveGLWindow())
	{
		m_app->getActiveGLWindow()->removeEventFilter(this);
	}

	//reset gui
	cleanupBeforeToolChange();

	//stop picking
	stopPicking();

	//set active tool to null (avoids tools "doing stuff" when the gui isn't shown)
	m_activeTool = nullptr;

	//remove overlay GUI
	if (m_dlg)
	{
		m_dlg->stop(true);
		m_app->unregisterOverlayDialog(m_dlg);
	}

	if (m_mapDlg)
	{
		m_mapDlg->stop(true);
		m_app->unregisterOverlayDialog(m_mapDlg);
	}

	//forget last measurement
	if (m_activeTool)
	{
		m_activeTool->cancel();
		m_activeTool->toolDisactivated();
	}

	//redraw
	if (m_app->getActiveGLWindow())
	{
		m_app->getActiveGLWindow()->redraw(true, false);
	}

	return true;
}

//registers this plugin with the picking hub
bool ccCompass::startPicking()
{
	if (m_picking) //already picking... don't need to add again
		return true;

	//activate "point picking mode"
	if (!m_app->pickingHub())  //no valid picking hub
	{
		m_app->dispToConsole("[ccCompass] Could not retrieve valid picking hub. Measurement aborted.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	if (!m_app->pickingHub()->addListener(this, true, true))
	{
		m_app->dispToConsole("Another tool is already using the picking mechanism. Stop it first", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	m_picking = true;
	return true;
}

//removes this plugin from the picking hub
void  ccCompass::stopPicking()
{
	//stop picking
	if (m_app->pickingHub())
	{
		m_app->pickingHub()->removeListener(this);
	}

	m_picking = false;
}

//Get the place/object that new measurements or interpretation should be stored
ccHObject* ccCompass::getInsertPoint()
{

	//check if there is an active GeoObject or we are in mapMode
	if (ccCompass::mapMode || m_geoObject)
	{
		//check there is an active GeoObject
		if (!m_geoObject)
		{
			m_app->dispToConsole("[ccCompass] Error: Please select a GeoObject to digitize to.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		}

		//check it actually exists/hasn't been deleted
		if (!m_app->dbRootObject()->find(m_geoObject_id))
		{
			//object has been deleted
			m_geoObject = nullptr;
			m_geoObject_id = -1;
			m_app->dispToConsole("[ccCompass] Error: Please select a GeoObject to digitize to.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		}
		else
		{
			//object exists - we can use it to find the insert point
			ccHObject* insertPoint = m_geoObject->getRegion(ccCompass::mapTo);
			if (!insertPoint) //something went wrong?
			{
				m_app->dispToConsole("[ccCompass] Warning: Could not retrieve valid mapping region for the active GeoObject.", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			}
			else
			{
				return insertPoint; // :)
			}
		}
	}
	else
	{

		//otherwise, we're in "Compass" mode, so...
		//find/create a group called "measurements"
		ccHObject* measurement_group = nullptr;

		//search for a "measurements" group
		for (unsigned i = 0; i < m_app->dbRootObject()->getChildrenNumber(); i++)
		{
			if (m_app->dbRootObject()->getChild(i)->getName() == "measurements")
			{
				measurement_group = m_app->dbRootObject()->getChild(i);
			}
			else
			{
				//also search first-level children of root node (when files are re-loaded this is where things will sit)
				for (unsigned c = 0; c < m_app->dbRootObject()->getChild(i)->getChildrenNumber(); c++)
				{
					if (m_app->dbRootObject()->getChild(i)->getChild(c)->getName() == "measurements")
					{
						measurement_group = m_app->dbRootObject()->getChild(i)->getChild(c);
						break;
					}
				}
			}

			//found a valid group :)
			if (measurement_group)
			{
				break;
			}
		}

		//didn't find it - create a new one!
		if (!measurement_group)
		{
			measurement_group = new ccHObject("measurements");
			m_app->dbRootObject()->addChild(measurement_group);
			m_app->addToDB(measurement_group, false, true, false, false);
		}

		return measurement_group; //this is the insert point
	}
	return nullptr; //no valid insert point
}

//This function is called when a point is picked (through the picking hub)
void ccCompass::onItemPicked(const ccPickingListener::PickedItem& pi)
{
	pointPicked(pi.entity, pi.itemIndex, pi.clickPoint.x(), pi.clickPoint.y(), pi.P3D); //map straight to pointPicked function
}

//Process point picks
void ccCompass::pointPicked(ccHObject* entity, unsigned itemIdx, int x, int y, const CCVector3& P)
{
	if (!entity) //null pick
	{
		return;
	}

	//no active tool (i.e. picking mode) - set selected object as active
	if (!m_activeTool)
	{
		m_app->setSelectedInDB(entity, true);
		return;
	}

	//find relevant node to add data to
	ccHObject* parentNode = getInsertPoint();
	
	if (parentNode == nullptr) //could not get insert point for some reason
	{
		return; //bail
	}

	//ensure what we are writing too is visible (avoids confusion if it is turned off...)
	parentNode->setEnabled(true); 

	//call generic "point-picked" function of active tool
	m_activeTool->pointPicked(parentNode, itemIdx, entity, P);

	//have we picked a point cloud?
	if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		//get point cloud
		ccPointCloud* cloud = static_cast<ccPointCloud*>(entity); //cast to point cloud

		if (!cloud)
		{
			ccLog::Warning("[Item picking] Shit's fubar (Picked point is not in pickable entities DB?)!");
			return;
		}

		//pass picked point, cloud & insert point to relevant tool
		m_activeTool->pointPicked(parentNode, itemIdx, cloud, P);
	}

	//redraw
	m_app->updateUI();
	m_app->getActiveGLWindow()->redraw();
}

bool ccCompass::eventFilter(QObject* obj, QEvent* event)
{
	//update cost mode (just in case it has changed) & fit plane params
	ccCompass::costMode = m_dlg->getCostMode();
	ccCompass::fitPlanes = m_dlg->planeFitMode();
	ccTrace::COST_MODE = ccCompass::costMode;

	if (event->type() == QEvent::MouseButtonDblClick)
	{
		QMouseEvent* mouseEvent = static_cast<QMouseEvent *>(event);
		if (mouseEvent->buttons() == Qt::RightButton)
		{
			stopMeasuring();
			return true;
		}
	}
	return false;
}

QIcon ccCompass::getIcon() const
{
	//open ccCompass.qrc (text file), update the "prefix" and the
	//icon(s) filename(s). Then save it with the right name (yourPlugin.qrc).
	//(eventually, remove the original qDummyPlugin.qrc file!)
	return QIcon(":/CC/plugin/qCompass/icon.png");
}

//exit this tool
void ccCompass::onClose()
{
	//cancel current action
	if (m_activeTool)
	{
		m_activeTool->cancel();
	}

	//finish measuring
	stopMeasuring();
}

void ccCompass::onAccept()
{
	if (m_activeTool)
	{
		m_activeTool->accept();
	}
}

//returns true if object was created by ccCompass
bool ccCompass::madeByMe(ccHObject* object)
{
	//return isFitPlane(object) | isTrace(object) | isLineation(object);
	return object->hasMetaData("ccCompassType");
}

//undo last plane
void ccCompass::onUndo()
{
	if (m_activeTool)
	{
		m_activeTool->undo();
	}
}

//called to cleanup pointers etc. before changing the active tool
void ccCompass::cleanupBeforeToolChange()
{
	//finish current tool
	if (m_activeTool)
	{
		m_activeTool->toolDisactivated();
	}

	//clear m_hiddenObjects buffer
	if (!m_hiddenObjects.empty())
	{
		for (int i : m_hiddenObjects)
		{
			ccHObject* o = m_app->dbRootObject()->find(i);
			if (o)
			{
				o->setVisible(true);
			}
		}
		m_hiddenObjects.clear();
		m_app->getActiveGLWindow()->redraw(false, false);
	}
	

	//uncheck/disable gui components (the relevant ones will be activated later)
	if (m_dlg)
	{
		m_dlg->pairModeButton->setChecked(false);
		m_dlg->planeModeButton->setChecked(false);
		m_dlg->traceModeButton->setChecked(false);
		m_dlg->pickModeButton->setChecked(false);
		m_dlg->extraModeButton->setChecked(false);
		m_dlg->undoButton->setEnabled(false);
		m_dlg->acceptButton->setEnabled(false);
	}

	//check picking is engaged
	startPicking();
}

//activate lineation mode
void ccCompass::setLineation()
{
	//cleanup
	cleanupBeforeToolChange();

	//activate lineation tool
	m_activeTool = m_lineationTool;
	m_activeTool->toolActivated();

	//trigger selection changed
	onNewSelection(m_app->getSelectedEntities());

	//update GUI
	m_dlg->undoButton->setEnabled(false);
	m_dlg->pairModeButton->setChecked(true);
	m_app->getActiveGLWindow()->redraw(true, false);
}

//activate plane mode
void ccCompass::setPlane()
{
	//cleanup
	cleanupBeforeToolChange();

	//activate plane tool
	m_activeTool = m_fitPlaneTool;
	m_activeTool->toolActivated();

	//trigger selection changed
	onNewSelection(m_app->getSelectedEntities());

	//update GUI
	m_dlg->undoButton->setEnabled(m_fitPlaneTool->canUndo());
	m_dlg->planeModeButton->setChecked(true);
	m_app->getActiveGLWindow()->redraw(true, false);
}

//activate trace mode
void ccCompass::setTrace()
{
	//cleanup
	cleanupBeforeToolChange();

	//activate trace tool
	m_activeTool = m_traceTool;
	m_activeTool->toolActivated();

	//trigger selection changed
	onNewSelection(m_app->getSelectedEntities());

	//update GUI
	m_dlg->traceModeButton->setChecked(true);
	m_dlg->undoButton->setEnabled( m_traceTool->canUndo() );
	m_dlg->acceptButton->setEnabled(true);
	m_app->getActiveGLWindow()->redraw(true, false);
}

//activate the paint tool
void ccCompass::setPick()
{
	cleanupBeforeToolChange();

	m_activeTool = nullptr; //picking tool is default - so no tool class
	stopPicking(); //let CC handle picks now

	//hide point clouds
	hideAllPointClouds(m_app->dbRootObject());

	m_dlg->pickModeButton->setChecked(true);
	m_dlg->undoButton->setEnabled(false);
	m_dlg->acceptButton->setEnabled(false);
	m_app->getActiveGLWindow()->redraw(true, false);
}

//activate the pinch-node tool
void ccCompass::addPinchNode()
{
	cleanupBeforeToolChange();

	//activate thickness tool
	m_activeTool = m_pinchNodeTool;
	m_activeTool->toolActivated();

	//update GUI
	m_dlg->extraModeButton->setChecked(true);
	m_dlg->undoButton->setEnabled(m_activeTool->canUndo());
	m_dlg->acceptButton->setEnabled(false);
	m_app->getActiveGLWindow()->redraw(true, false);
}
//activates the thickness tool
void ccCompass::setThickness() 
{
	cleanupBeforeToolChange();

	//activate thickness tool
	m_activeTool = m_thicknessTool;
	m_activeTool->toolActivated();
	ccThicknessTool::TWO_POINT_MODE = false; //one-point mode (unless changed later)

	//trigger selection changed
	onNewSelection(m_app->getSelectedEntities());

	//update GUI
	m_dlg->extraModeButton->setChecked(true);
	m_dlg->undoButton->setEnabled(m_activeTool->canUndo());
	m_dlg->acceptButton->setEnabled(true);
	m_app->getActiveGLWindow()->redraw(true, false);
}

//activates the thickness tool in two-point mode
void ccCompass::setThickness2()
{
	setThickness();
	ccThicknessTool::TWO_POINT_MODE = true; //now set the tool to operate in two-point mode
}

void ccCompass::setYoungerThan() //activates topology tool in "older-than" mode
{
	cleanupBeforeToolChange();

	m_activeTool = m_topologyTool; //activate topology tool
	stopPicking(); //let CC handle picks now - this tool only needs "selection changed" callbacks

	//hide point clouds
	hideAllPointClouds(m_app->dbRootObject());

	//update gui
	m_dlg->undoButton->setEnabled(false);
	m_dlg->acceptButton->setEnabled(false);
	m_app->getActiveGLWindow()->redraw(true, false);

	//set topology tool mode
	ccTopologyTool::RELATIONSHIP = ccTopologyRelation::YOUNGER_THAN;
}

void ccCompass::setFollows() //activates topology tool in "follows" mode
{
	setYoungerThan();
	//set topology tool mode
	ccTopologyTool::RELATIONSHIP = ccTopologyRelation::IMMEDIATELY_FOLLOWS;
}

void ccCompass::setEquivalent() //activates topology mode in "equivalent" mode
{
	setYoungerThan();
	//set topology tool mode
	ccTopologyTool::RELATIONSHIP = ccTopologyRelation::EQUIVALENCE;
}

//activates note mode
void ccCompass::setNote()
{
	cleanupBeforeToolChange();

	//activate thickness tool
	m_activeTool = m_noteTool;
	m_activeTool->toolActivated();

	//update GUI
	m_dlg->extraModeButton->setChecked(true);
	m_dlg->undoButton->setEnabled(m_activeTool->canUndo());
	m_dlg->acceptButton->setEnabled(false);
	m_app->getActiveGLWindow()->redraw(true, false);
}

//merges the selected GeoObjects
void ccCompass::mergeGeoObjects()
{
	//get selected GeoObjects
	std::vector<ccGeoObject*> objs;

	for (ccHObject* o : m_app->getSelectedEntities())
	{
		if (ccGeoObject::isGeoObject(o))
		{
			ccGeoObject* g = dynamic_cast<ccGeoObject*> (o);
			if (g) //could possibly be null if non-loaded geo-objects exist
			{
				objs.push_back(g);
			}
		}
	}


	if (objs.size() < 2) //not enough geoObjects
	{
		m_app->dispToConsole("[Compass] Select several GeoObjects to merge.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return; //nothing to merge
	}

	//merge geo-objects with first one
	ccGeoObject* dest = objs[0];
	ccHObject* d_interior = dest->getRegion(ccGeoObject::INTERIOR);
	ccHObject* d_upper = dest->getRegion(ccGeoObject::UPPER_BOUNDARY);
	ccHObject* d_lower = dest->getRegion(ccGeoObject::LOWER_BOUNDARY);
	for (int i = 1; i < objs.size(); i++)
	{
		ccHObject* interior = objs[i]->getRegion(ccGeoObject::INTERIOR);
		ccHObject* upper = objs[i]->getRegion(ccGeoObject::UPPER_BOUNDARY);
		ccHObject* lower = objs[i]->getRegion(ccGeoObject::LOWER_BOUNDARY);

		//add children to destination
		upper->transferChildren(*d_upper, true);
		lower->transferChildren(*d_lower, true);
		interior->transferChildren(*d_interior, true);

		//delete un-needed objects
		objs[i]->removeChild(interior);
		objs[i]->removeChild(upper);
		objs[i]->removeChild(lower);
		objs[i]->getParent()->removeChild(objs[i]);
		
		//delete
		m_app->removeFromDB(objs[i]);
		m_app->removeFromDB(upper);
		m_app->removeFromDB(lower);
		m_app->removeFromDB(interior);
	}

	m_app->setSelectedInDB(dest, true);
	m_app->redrawAll(true); //redraw gui + 3D view

	m_app->dispToConsole("[Compass] Merged selected GeoObjects to " + dest->getName(), ccMainAppInterface::STD_CONSOLE_MESSAGE);
}

//calculates best-fit plane for the upper and lower surfaces of the selected GeoObject
void ccCompass::fitPlaneToGeoObject()
{

	m_app->dispToConsole("[Compass] fitPlane", ccMainAppInterface::STD_CONSOLE_MESSAGE);


	//loop selected GeoObject
	ccHObject* o = m_app->dbRootObject()->find(m_geoObject_id);
	if (!o)
	{
		m_geoObject_id = -1;
		return; //invalid id
	}

	ccGeoObject* obj = static_cast<ccGeoObject*>(o); //get as geoObject

	//fit upper plane
	ccHObject* upper = obj->getRegion(ccGeoObject::UPPER_BOUNDARY);
	ccPointCloud* points = new ccPointCloud(); //create point cloud for storing points
	double rms; //float for storing rms values
	for (unsigned i = 0; i < upper->getChildrenNumber(); i++)
	{
		if (ccTrace::isTrace(upper->getChild(i)))
		{
			ccTrace* t = dynamic_cast<ccTrace*> (upper->getChild(i));
			points->reserve(points->size() + t->size()); //make space
			if (t) //can in rare cases be a null ptr (dynamic cast will fail for traces that haven't been converted to ccTrace objects)
			{
				for (unsigned p = 0; p < t->size(); p++)
				{
					points->addPoint(*t->getPoint(p)); //add point to 
				}
			}
		}
	}

	//calculate and store upper fitplane
	if (points->size() > 0)
	{
		ccFitPlane* p = ccFitPlane::Fit(points, &rms);
		if (p)
		{
			QVariantMap map;
			map.insert("RMS", rms);
			p->setMetaData(map, true);
			upper->addChild(p);
			m_app->addToDB(p, false, false, false, false);
		}
		else
		{
			m_app->dispToConsole("[Compass] Not enough 3D information to generate sensible fit plane.", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		}
	}

	//rinse and repeat for lower
	points->clear();
	ccHObject* lower = obj->getRegion(ccGeoObject::LOWER_BOUNDARY);
	for (unsigned i = 0; i < lower->getChildrenNumber(); i++)
	{
		if (ccTrace::isTrace(lower->getChild(i)))
		{
			ccTrace* t = dynamic_cast<ccTrace*> (lower->getChild(i));
			points->reserve(points->size() + t->size()); //make space
			if (t) //can in rare cases be a null ptr (dynamic cast will fail for traces that haven't been converted to ccTrace objects)
			{
				for (unsigned p = 0; p < t->size(); p++)
				{
					points->addPoint(*t->getPoint(p)); //add point to cloud
				}
			}
		}
	}

	//calculate and store lower fitplane
	if (points->size() > 0)
	{
		ccFitPlane* p = ccFitPlane::Fit(points, &rms);
		if (p)
		{
			QVariantMap map;
			map.insert("RMS", rms);
			p->setMetaData(map, true);
			lower->addChild(p);
			m_app->addToDB(p, false, false, false, true);
		}
		else
		{
			m_app->dispToConsole("[Compass] Not enough 3D information to generate sensible fit plane.", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		}
	}

	//clean up point cloud
	delete(points); 

}

//recompute entirely each selected trace (useful if the cost function has changed)
void ccCompass::recalculateSelectedTraces()
{
	ccTrace::COST_MODE = m_dlg->getCostMode(); //update cost mode

	for (ccHObject* obj : m_app->getSelectedEntities())
	{
		if (ccTrace::isTrace(obj))
		{
			ccTrace* trc = static_cast<ccTrace*>(obj);
			trc->recalculatePath();
		}
	}

	m_app->getActiveGLWindow()->redraw(); //repaint window
}

//recurse and hide visisble point clouds
void ccCompass::hideAllPointClouds(ccHObject* o)
{
	if (o->isKindOf(CC_TYPES::POINT_CLOUD) & o->isVisible())
	{
		o->setVisible(false);
		m_hiddenObjects.push_back(o->getUniqueID());
		return;
	}

	for (unsigned i = 0; i < o->getChildrenNumber(); i++)
	{
		hideAllPointClouds(o->getChild(i));
	}
}

//toggle stippling
void ccCompass::toggleStipple(bool checked)
{
	ccCompass::drawStippled = checked; //change stippling for newly created planes
	recurseStipple(m_app->dbRootObject(), checked); //change stippling for existing planes
	m_app->getActiveGLWindow()->redraw(); //redraw
}

void ccCompass::recurseStipple(ccHObject* object,bool checked)
{
	//check this object
	if (ccFitPlane::isFitPlane(object))
	{
		ccPlane* p = static_cast<ccPlane*>(object);
		p->enableStippling(checked);
	}

	//recurse
	for (unsigned i = 0; i < object->getChildrenNumber(); i++)
	{
		ccHObject* o = object->getChild(i);
		recurseStipple(o, checked);
	}
}

//toggle labels
void ccCompass::toggleLabels(bool checked)
{
	recurseLabels(m_app->dbRootObject(), checked); //change labels for existing planes
	ccCompass::drawName = checked; //change labels for newly created planes
	m_app->getActiveGLWindow()->redraw(); //redraw
}

void ccCompass::recurseLabels(ccHObject* object, bool checked)
{
	//check this object
	if (ccFitPlane::isFitPlane(object) | ccPointPair::isPointPair(object))
	{
		object->showNameIn3D(checked);
	}

	//recurse
	for (unsigned i = 0; i < object->getChildrenNumber(); i++)
	{
		ccHObject* o = object->getChild(i);
		recurseLabels(o, checked);
	}
}

//toggle plane normals
void ccCompass::toggleNormals(bool checked)
{
	recurseNormals(m_app->dbRootObject(), checked); //change labels for existing planes
	ccCompass::drawNormals = checked; //change labels for newly created planes
	m_app->getActiveGLWindow()->redraw(); //redraw
}

void ccCompass::recurseNormals(ccHObject* object, bool checked)
{
	//check this object
	if (ccFitPlane::isFitPlane(object))
	{
		ccPlane* p = static_cast<ccPlane*>(object);
		p->showNormalVector(checked);
	}

	//recurse
	for (unsigned i = 0; i < object->getChildrenNumber(); i++)
	{
		ccHObject* o = object->getChild(i);
		recurseNormals(o, checked);
	}
}

//displays the info dialog
void ccCompass::showHelp()
{
	//create new qt window
	ccCompassInfo info(m_app->getMainWindow());
	info.exec();
}

//enter or turn off map mode
void ccCompass::enableMapMode() //turns on/off map mode
{
	//m_app->dispToConsole("ccCompass: Changing to Map mode. Measurements will be associated with GeoObjects.", ccMainAppInterface::STD_CONSOLE_MESSAGE);
	m_dlg->mapMode->setChecked(true);
	m_dlg->compassMode->setChecked(false);

	ccCompass::mapMode = true;

	//start gui
	m_app->registerOverlayDialog(m_mapDlg, Qt::Corner::TopLeftCorner);
	m_mapDlg->start();
	m_app->updateOverlayDialogsPlacement();
	m_app->getActiveGLWindow()->redraw(true, false);
}

//enter or turn off map mode
void ccCompass::enableMeasureMode() //turns on/off map mode
{
	//m_app->dispToConsole("ccCompass: Changing to Compass mode. Measurements will be stored in the \"Measurements\" folder.", ccMainAppInterface::STD_CONSOLE_MESSAGE);
	m_dlg->mapMode->setChecked(false);
	m_dlg->compassMode->setChecked(true);
	ccCompass::mapMode = false;
	m_app->getActiveGLWindow()->redraw(true, false);

	//turn off map mode dialog
	m_mapDlg->stop(true);
	m_app->unregisterOverlayDialog(m_mapDlg);
	m_app->updateOverlayDialogsPlacement();
}

void ccCompass::addGeoObject() //creates a new GeoObject
{
	//calculate default name
	QString name = m_lastGeoObjectName;
	int number = 0;
	if (name.contains("_"))
	{
		number = name.split("_")[1].toInt(); //counter
		name = name.split("_")[0]; //initial part
	}
	number++;
	name += QString::asprintf("_%d", number);

	//get name
	name = QInputDialog::getText(m_app->getMainWindow(), "New GeoObject", "GeoObject Name:", QLineEdit::Normal, name);
	if (name == "") //user clicked cancel
	{
		return;
	}
	m_lastGeoObjectName = name;

	//search for a "interpretation" group [where the new unit will be added]
	ccHObject* interp_group = nullptr;
	for (unsigned i = 0; i < m_app->dbRootObject()->getChildrenNumber(); i++)
	{
		if (m_app->dbRootObject()->getChild(i)->getName() == "interpretation")
		{
			interp_group = m_app->dbRootObject()->getChild(i);
		}
		else
		{
			//also search first-level children of root node (when files are re-loaded this is where things will sit)
			for (unsigned c = 0; c < m_app->dbRootObject()->getChild(i)->getChildrenNumber(); c++)
			{
				if (m_app->dbRootObject()->getChild(i)->getChild(c)->getName() == "interpretation")
				{
					interp_group = m_app->dbRootObject()->getChild(i)->getChild(c);
					break;
				}
			}
		}
		if (interp_group) //found one :)
		{
			break;
		}
	}

	//didn't find it - create a new one!
	if (!interp_group)
	{
		interp_group = new ccHObject("interpretation");
		m_app->dbRootObject()->addChild(interp_group);
		m_app->addToDB(interp_group, false, true, false, false);
	}

	//create the new GeoObject
	ccGeoObject* newGeoObject = new ccGeoObject(name,m_app);
	interp_group->addChild(newGeoObject);
	m_app->addToDB(newGeoObject, false, true, false, false);

	//set it to selected (this will then make it "active" via the selection change callback)
	m_app->setSelectedInDB(newGeoObject, true);
}

void ccCompass::writeToInterior() //new digitization will be added to the GeoObjects interior
{
	ccCompass::mapTo = ccGeoObject::INTERIOR;
	m_mapDlg->setInteriorButton->setChecked(true);
	m_mapDlg->setUpperButton->setChecked(false);
	m_mapDlg->setLowerButton->setChecked(false);
}

void ccCompass::writeToUpper() //new digitization will be added to the GeoObjects upper boundary
{
	ccCompass::mapTo = ccGeoObject::UPPER_BOUNDARY;
	m_mapDlg->setInteriorButton->setChecked(false);
	m_mapDlg->setUpperButton->setChecked(true);
	m_mapDlg->setLowerButton->setChecked(false);
}

void ccCompass::writeToLower() //new digitiziation will be added to the GeoObjects lower boundary
{
	ccCompass::mapTo = ccGeoObject::LOWER_BOUNDARY;
	m_mapDlg->setInteriorButton->setChecked(false);
	m_mapDlg->setUpperButton->setChecked(false);
	m_mapDlg->setLowerButton->setChecked(true);
}

//save the current view to an SVG file
void ccCompass::exportToSVG()
{
	//get output file path
	QString filename = QFileDialog::getSaveFileName(m_dlg, tr("Output file"), "", tr("SVG files (*.svg)"));
	if (filename.isEmpty())
	{
		//process cancelled by the user
		return;
	}

	QFileInfo fi(filename);
	if (fi.suffix() != "svg")
	{
		filename += ".svg";
	}

	//create file
	QFile svg_file(filename);

	//open file & create text stream
	if (svg_file.open(QIODevice::WriteOnly))
	{
		QTextStream svg_stream(&svg_file);

		int width = m_app->getActiveGLWindow()->glWidth();
		int height = m_app->getActiveGLWindow()->glHeight();

		//write svg header
		svg_stream << QString::asprintf("<svg width=\"%d\" height=\"%d\">", width, height) << endl;

		//recursively write traces
		int count = writeTracesSVG(m_app->dbRootObject(), &svg_stream, height);

		//write end tag for svg file
		svg_stream << "</svg>" << endl; 

		//close file
		svg_stream.flush();
		svg_file.close();

		if (count > 0)
		{
			m_app->dispToConsole(QString::asprintf("[ccCompass] Successfully saved %d polylines to .svg file.", count));
		}
		else
		{
			//remove file
			svg_file.remove();
			m_app->dispToConsole("[ccCompass] Could not write polylines to .svg - no polylines found!",ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		}
	}
}

int ccCompass::writeTracesSVG(ccHObject* object, QTextStream* out, int height)
{
	int n = 0;

	//is this a drawable polyline?
	if (object->isA(CC_TYPES::POLY_LINE) || ccTrace::isTrace(object))
	{
		//get polyline object
		ccPolyline* line = static_cast<ccPolyline*>(object);

		if (!line->isVisible())
		{
			return 0; //as soon as something is not visible we bail
		}

		//write polyline header
		*out << "<polyline fill=\"none\" stroke=\"black\" points=\"";

		//get projection params
		ccGLCameraParameters params;
		m_app->getActiveGLWindow()->getGLCameraParameters(params);
		if (params.perspective)
		{
			m_app->getActiveGLWindow()->setPerspectiveState(false, true);
			m_app->getActiveGLWindow()->redraw(false, false); //not sure if this is needed or not?
			m_app->getActiveGLWindow()->getGLCameraParameters(params); //get updated params
		}

		//write point string
		for (unsigned i = 0; i < line->size(); i++)
		{

			//get point in world coordinates
			CCVector3 P = *line->getPoint(i);

			//project 3D point into 2D
			CCVector3d coords2D;
			params.project(P, coords2D);
			
			//write point
			*out << QString::asprintf("%.3f,%.3f ", coords2D.x, height - coords2D.y); //n.b. we need to flip y-axis

		}

		//end polyline
		*out << "\"/>" << endl;

		n++; //a polyline has been written
	}

	//recurse on children
	for (unsigned i = 0; i < object->getChildrenNumber(); i++)
	{
		n += writeTracesSVG(object->getChild(i), out, height);
	}

	return n;
}

//export the selected layer to CSV file
void ccCompass::onSave()
{
	//get output file path
	QString filename = QFileDialog::getSaveFileName(m_dlg, tr("Output file"), "", tr("CSV files (*.csv *.txt)"));
	if (filename.isEmpty())
	{
		//process cancelled by the user
		return;
	}
	int planes = 0; //keep track of how many objects are being written (used to delete empty files)
	int traces = 0;
	int lineations = 0;
	int thicknesses = 0;

	//build filenames
	QFileInfo fi(filename);
	QString baseName = fi.absolutePath() + "/" + fi.completeBaseName();
	QString ext = fi.suffix();
	if (!ext.isEmpty())
	{
		ext.prepend('.');
	}
	QString plane_fn = baseName + "_planes" + ext;
	QString trace_fn = baseName + "_traces" + ext;
	QString lineation_fn = baseName + "_lineations" + ext;
	QString thickness_fn = baseName + "_thickness" + ext;

	//create files
	QFile plane_file(plane_fn);
	QFile trace_file(trace_fn);
	QFile lineation_file(lineation_fn);
	QFile thickness_file(thickness_fn);
	
	//open files
	if (plane_file.open(QIODevice::WriteOnly) && trace_file.open(QIODevice::WriteOnly) && lineation_file.open(QIODevice::WriteOnly) && thickness_file.open(QIODevice::WriteOnly))
	{
		//create text streams for each file
		QTextStream plane_stream(&plane_file);
		QTextStream trace_stream(&trace_file);
		QTextStream lineation_stream(&lineation_file);
		QTextStream thickness_stream(&thickness_file);

		//write headers
		plane_stream << "Name,Strike,Dip,Dip_Dir,Cx,Cy,Cz,Nx,Ny,Nz,Sample_Radius,RMS" << endl;
		trace_stream << "Name,Trace_id,Point_id,Start_x,Start_y,Start_z,End_x,End_y,End_z,Cost,Cost_Mode" << endl;
		lineation_stream << "Name,Sx,Sy,Sz,Ex,Ey,Ez,Trend,Plunge,Length" << endl;
		thickness_stream << "Name,Sx,Sy,Sz,Ex,Ey,Ez,Trend,Plunge,Thickness" << endl;

		//write data for all objects in the db tree (n.b. we loop through the dbRoots children rathern than just passing db_root so the naming is correct)
		for (unsigned i = 0; i < m_app->dbRootObject()->getChildrenNumber(); i++)
		{
			ccHObject* o = m_app->dbRootObject()->getChild(i);
			planes += writePlanes(o, &plane_stream);
			traces += writeTraces(o, &trace_stream);
			lineations += writeLineations(o, &lineation_stream, QString(), false);
			thicknesses += writeLineations(o, &thickness_stream, QString(), true);
		}

		//cleanup
		plane_stream.flush();
		plane_file.close();
		trace_stream.flush();
		trace_file.close();
		lineation_stream.flush();
		lineation_file.close();
		thickness_stream.flush();
		thickness_file.close();

		//ensure data has been written (and if not, delete the file)
		if (planes)
		{
			m_app->dispToConsole("[ccCompass] Successfully exported plane data.", ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}
		else
		{
			m_app->dispToConsole("[ccCompass] No plane data found.", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			plane_file.remove();
		}
		if (traces)
		{
			m_app->dispToConsole("[ccCompass] Successfully exported trace data.", ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}
		else
		{
			m_app->dispToConsole("[ccCompass] No trace data found.", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			trace_file.remove();
		}
		if (lineations)
		{
			m_app->dispToConsole("[ccCompass] Successfully exported lineation data.", ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}
		else
		{
			m_app->dispToConsole("[ccCompass] No lineation data found.", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			lineation_file.remove();
		}
		if (thicknesses)
		{
			m_app->dispToConsole("[ccCompass] Successfully exported thickness data.", ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}
		else
		{
			m_app->dispToConsole("[ccCompass] No thickness data found.", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			thickness_file.remove();
		}
	}
	else
	{
		m_app->dispToConsole("[ccCompass] Could not open output files... ensure CC has write access to this location.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	}
}

//write plane data
int ccCompass::writePlanes(ccHObject* object, QTextStream* out, QString parentName)
{
	//get object name
	QString name;
	if (parentName.isEmpty())
	{
		name = QString("%1").arg(object->getName());
	}
	else
	{
		name = QString("%1.%2").arg(parentName, object->getName());
	}

	//is object a plane made by ccCompass?
	int n = 0;
	if (ccFitPlane::isFitPlane(object))
	{
		//Write object as Name,Strike,Dip,Dip_Dir,Cx,Cy,Cz,Nx,Ny,Nz,Radius,RMS
		*out << name << ",";
		*out << object->getMetaData("Strike").toString() << "," << object->getMetaData("Dip").toString() << "," << object->getMetaData("DipDir").toString() << ",";
		*out << object->getMetaData("Cx").toString() << "," << object->getMetaData("Cy").toString() << "," << object->getMetaData("Cz").toString() << ",";
		*out << object->getMetaData("Nx").toString() << "," << object->getMetaData("Ny").toString() << "," << object->getMetaData("Nz").toString() << ",";
		*out << object->getMetaData("Radius").toString() << "," << object->getMetaData("RMS").toString() << endl;
		n++;
	}
	else if (object->isKindOf(CC_TYPES::PLANE)) //not one of our planes, but a plane anyway (so we'll export it)
	{
		//calculate plane orientation
		//get plane normal vector
		ccPlane* P = static_cast<ccPlane*>(object);
		CCVector3 N(P->getNormal());
		CCVector3 L = P->getTransformation().getTranslationAsVec3D();

		//We always consider the normal with a positive 'Z' by default!
		if (N.z < 0.0)
			N *= -1.0;

		//calculate strike/dip/dip direction
		float strike, dip, dipdir;
		ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipdir);
		ccNormalVectors::ConvertNormalToStrikeAndDip(N, strike, dip);

		//export
		*out << name << ",";
		*out << strike << "," << dip << "," << dipdir << ","; //write orientation
		*out << L.x << "," << L.y << "," << L.z << ","; //write location
		*out << N.x << "," << N.y << "," << N.z << ","; //write normal
		*out << "NA" << "," << "UNK" << endl; //the "radius" and "RMS" are unknown
		n++;
	}

	//write all children
	for (unsigned i = 0; i < object->getChildrenNumber(); i++)
	{
		ccHObject* o = object->getChild(i);
		n += writePlanes(o, out, name);
	}
	return n;
}

//write trace data
int ccCompass::writeTraces(ccHObject* object, QTextStream* out, QString parentName)
{
	//get object name
	QString name;
	if (parentName.isEmpty())
	{
		name = QString("%1").arg(object->getName());
	}
	else
	{
		name = QString("%1.%2").arg(parentName, object->getName());
	}

	//is object a polyline
	int n = 0;
	if (ccTrace::isTrace(object)) //ensure this is a trace
	{
		ccTrace* p = static_cast<ccTrace*>(object);

		//loop through points
		CCVector3 start, end;
		int cost;
		int tID = object->getUniqueID();
		if (p->size() >= 2)
		{
			//set cost function
			ccTrace::COST_MODE = p->getMetaData("cost_function").toInt();

			//loop through segments
			for (unsigned i = 1; i < p->size(); i++)
			{
				//get points
				p->getPoint(i - 1, start);
				p->getPoint(i, end);
				
				//calculate segment cost
				cost = p->getSegmentCost(p->getPointGlobalIndex(i - 1), p->getPointGlobalIndex(i));
				
				//write data
				//n.b. csv columns are name,trace_id,seg_id,start_x,start_y,start_z,end_x,end_y,end_z, cost, cost_mode
				*out << name << ","; //name
				*out << tID << ",";
				*out << i - 1 << ",";
				*out << start.x << ",";
				*out << start.y << ",";
				*out << start.z << ",";
				*out << end.x << ",";
				*out << end.y << ",";
				*out << end.z << ",";
				*out << cost << ",";
				*out << ccTrace::COST_MODE << endl;
			}
		}
		n++;
	}

	//write all children
	for (unsigned i = 0; i < object->getChildrenNumber(); i++)
	{
		ccHObject* o = object->getChild(i);
		n += writeTraces(o, out, name);
	}
	return n;
}

//write lineation data
int ccCompass::writeLineations(ccHObject* object, QTextStream* out, QString parentName, bool thicknesses)
{
	//get object name
	QString name;
	if (parentName.isEmpty())
	{
		name = QString("%1").arg(object->getName());
	}
	else
	{
		name = QString("%1.%2").arg(parentName, object->getName());
	}

	//is object a lineation made by ccCompass?
	int n = 0;
	if (((thicknesses==false) && ccLineation::isLineation(object)) | //lineation measurement
		((thicknesses==true) && ccThickness::isThickness(object)))    //or thickness measurement
	{
		//Write object as Name,Sx,Sy,Sz,Ex,Ey,Ez,Trend,Plunge
		*out << name << ",";
		*out << object->getMetaData("Sx").toString() << "," << object->getMetaData("Sy").toString() << "," << object->getMetaData("Sz").toString() << ",";
		*out << object->getMetaData("Ex").toString() << "," << object->getMetaData("Ey").toString() << "," << object->getMetaData("Ez").toString() << ",";
		*out << object->getMetaData("Trend").toString() << "," << object->getMetaData("Plunge").toString() << "," << object->getMetaData("Length").toString() << endl;
		n++;
	}

	//write all children
	for (unsigned i = 0; i < object->getChildrenNumber(); i++)
	{
		ccHObject* o = object->getChild(i);
		n += writeLineations(o, out, name, thicknesses);
	}
	return n;
}