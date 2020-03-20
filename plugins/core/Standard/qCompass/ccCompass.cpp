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

#include <array>
#include <random>

//Qt
#include <QCheckBox>
#include <QComboBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QIntValidator>
#include <QXmlStreamWriter>

//common
#include <ccPickingHub.h>

//qCC_db
#include <ccProgressDialog.h>

#include "ccBox.h"
#include "ccFacet.h"
#include "ccCompass.h"
#include "ccCompassDlg.h"
#include "ccCompassInfo.h"
#include "ccFitPlaneTool.h"
#include "ccGeoObject.h"
#include "ccLineationTool.h"
#include "ccMapDlg.h"
#include "ccNoteTool.h"
#include "ccPinchNodeTool.h"
#include "ccSNECloud.h"
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

ccCompass::ccCompass(QObject* parent) :
	QObject( parent )
  , ccStdPluginInterface( ":/CC/plugin/qCompass/info.json" )
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
}

void ccCompass::stop()
{
	stopMeasuring(true);
	m_dlg = nullptr;

	ccStdPluginInterface::stop();
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
					if (!ccGeoObject::isSingleSurfaceGeoObject(m_geoObject))
					{
						m_mapDlg->setLowerButton->setEnabled(true);
						m_mapDlg->setUpperButton->setEnabled(true);
						m_mapDlg->setInteriorButton->setEnabled(true);
					}
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
QList<QAction *> ccCompass::getActions()
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
		connect(m_action, &QAction::triggered, this, &ccCompass::doAction); //this binds the m_action to the ccCompass::doAction() function
	}

	return QList<QAction *>{ m_action };
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
		ccCompassDlg::connect(m_dlg->m_recalculateFitPlanes, SIGNAL(triggered()), this, SLOT(recalculateFitPlanes()));
		ccCompassDlg::connect(m_dlg->m_toPointCloud, SIGNAL(triggered()), this, SLOT(convertToPointCloud()));
		ccCompassDlg::connect(m_dlg->m_distributeSelection, SIGNAL(triggered()), this, SLOT(distributeSelection()));
		ccCompassDlg::connect(m_dlg->m_estimateNormals, SIGNAL(triggered()), this, SLOT(estimateStructureNormals()));
		ccCompassDlg::connect(m_dlg->m_estimateP21, SIGNAL(triggered()), this, SLOT(estimateP21()));
		ccCompassDlg::connect(m_dlg->m_estimateStrain, SIGNAL(triggered()), this, SLOT(estimateStrain()));
		ccCompassDlg::connect(m_dlg->m_noteTool, SIGNAL(triggered()), this, SLOT(setNote()));
		ccCompassDlg::connect(m_dlg->m_loadFoliations, SIGNAL(triggered()), this, SLOT(importFoliations()));
		ccCompassDlg::connect(m_dlg->m_loadLineations, SIGNAL(triggered()), this, SLOT(importLineations()));
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

		ccCompassDlg::connect(m_mapDlg->m_create_geoObject, SIGNAL(triggered()), this, SLOT(addGeoObject()));
		ccCompassDlg::connect(m_mapDlg->m_create_geoObjectSS, SIGNAL(triggered()), this, SLOT(addGeoObjectSS()));
		ccCompassDlg::connect(m_mapDlg->setInteriorButton, SIGNAL(clicked()), this, SLOT(writeToInterior()));
		ccCompassDlg::connect(m_mapDlg->setUpperButton, SIGNAL(clicked()), this, SLOT(writeToUpper()));
		ccCompassDlg::connect(m_mapDlg->setLowerButton, SIGNAL(clicked()), this, SLOT(writeToLower()));
	}

	m_dlg->linkWith(m_app->getActiveGLWindow());
	m_mapDlg->linkWith(m_app->getActiveGLWindow());

	//load ccCompass objects
	tryLoading();

	//start in measure mode
	enableMeasureMode();
	
	//begin measuring
	startMeasuring();
}

//loop through DB tree looking for ccCompass objects that
//are not represented by our custom class. If any are found,
//replace them. Assuming not too many objects are found, this should be
//quite fast; hence we call it every time the selection changes.
void ccCompass::tryLoading()
{
	//setup progress window
	ccProgressDialog prg(true, m_app->getMainWindow());
	prg.setMethodTitle("Compass");
	prg.setInfo("Converting Compass types...");
	prg.start();


	//loop through DB_Tree and find any ccCompass objects
	std::vector<int> originals; //ids of original objects
	std::vector<ccHObject*> replacements; //pointers to objects that will replace the originals
	unsigned nChildren = m_app->dbRootObject()->getChildrenNumber();
	for (unsigned i = 0; i < nChildren; i++)
	{
		prg.setValue(static_cast<int>((50 * i) / nChildren));
		ccHObject* c = m_app->dbRootObject()->getChild(i);
		tryLoading(c, &originals, &replacements);
	}

	//replace all "originals" with their corresponding "duplicates"
	for (size_t i = 0; i < originals.size(); i++)
	{
		prg.setValue(50 + static_cast<int>((50 * i) / originals.size()));

		ccHObject* original = m_app->dbRootObject()->find(originals[i]);
		ccHObject* replacement = replacements[i];

		replacement->setVisible(original->isVisible());
		replacement->setEnabled(original->isEnabled());

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

	prg.close();
}

void ccCompass::tryLoading(ccHObject* obj, std::vector<int>* originals, std::vector<ccHObject*>* replacements)
{
	//recurse on children
	for (unsigned i = 0; i < obj->getChildrenNumber(); i++)
	{
		tryLoading(obj->getChild(i), originals, replacements);
	}

	//is object already represented by a ccCompass class?
	if (dynamic_cast<ccFitPlane*>(obj)
		|| dynamic_cast<ccTrace*>(obj)
		|| dynamic_cast<ccPointPair*>(obj) //n.b. several classes inherit from PointPair, so this cast will still succede for them
		|| dynamic_cast<ccGeoObject*>(obj)
		|| dynamic_cast<ccSNECloud*>(obj))
	{
		return; //we need do nothing!
	}

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

	//are we a SNE cloud?
	if (ccSNECloud::isSNECloud(obj))
	{
		ccHObject* sneCloud = new ccSNECloud(static_cast<ccPointCloud*>(obj));
		originals->push_back(obj->getUniqueID());
		replacements->push_back(sneCloud);
		return;
	}

	//is the HObject a polyline? (this will be the case for lineations & traces)
	ccPolyline* p = dynamic_cast<ccPolyline*>(obj);
	if (p)
	{
		//are we a trace?
		if (ccTrace::isTrace(obj))
		{

			ccTrace* trace = new ccTrace(p);
			trace->setWidth(2);
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
	
	m_active = true;
	return true;
}

//Exits measuring
bool ccCompass::stopMeasuring(bool finalStop/*=false*/)
{
	// Check if we were ever loaded. If the plugin wasn't loaded this will be nullptr.
	if ( m_app == nullptr )
	{
		return true;
	}
	
	//remove click listener
	if (m_app->getActiveGLWindow())
	{
		m_app->getActiveGLWindow()->removeEventFilter(this);
	}

	//reset gui
	cleanupBeforeToolChange(!finalStop);

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

	m_active = false;

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
void ccCompass::cleanupBeforeToolChange(bool autoRestartPicking/*=true*/)
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

	if (autoRestartPicking)
	{
		//check picking is engaged
		startPicking();
	}
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
		interior->transferChildren(*d_interior, true);
		upper->transferChildren(*d_upper, true);
		lower->transferChildren(*d_lower, true);
		
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

			if (t != nullptr) //can in rare cases be a null ptr (dynamic cast will fail for traces that haven't been converted to ccTrace objects)
			{
				points->reserve(points->size() + t->size()); //make space

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

	//rinse and repeat for lower (assuming normal GeoObject; skip this step for single-surface object)
	if (!ccGeoObject::isSingleSurfaceGeoObject(obj)) 
	{
		points->clear();
		ccHObject* lower = obj->getRegion(ccGeoObject::LOWER_BOUNDARY);
		for (unsigned i = 0; i < lower->getChildrenNumber(); i++)
		{
			if (ccTrace::isTrace(lower->getChild(i)))
			{
				ccTrace* t = dynamic_cast<ccTrace*> (lower->getChild(i));

				if (t != nullptr) //can in rare cases be a null ptr (dynamic cast will fail for traces that haven't been converted to ccTrace objects)
				{
					points->reserve(points->size() + t->size()); //make space

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
	}
	//clean up point cloud
	delete(points); 

}

//recalculates all fit planes in the DB Tree, except those generated using the Plane Tool
void ccCompass::recalculateFitPlanes()
{
	//get all plane objects
	ccHObject::Container planes;
	m_app->dbRootObject()->filterChildren(planes, true, CC_TYPES::PLANE, true);

	std::vector<ccHObject*> garbage; //planes that need to be deleted
	for (auto & plane : planes)
	{
		if (!ccFitPlane::isFitPlane(plane))
			continue; //only deal with FitPlane objects

		//is parent of the plane a trace object?
		ccHObject* parent = plane->getParent();

		if (ccTrace::isTrace(parent)) //add to recalculate list
		{
			//recalculate the fit plane
			ccTrace* t = static_cast<ccTrace*>(parent);
			ccFitPlane* p = t->fitPlane();
			if (p)
			{
				t->addChild(p); //add the new fit-plane
				m_app->addToDB(p, false, false, false, false);
			}

			//add the old plane to the garbage list (to be deleted later)
			garbage.push_back(plane);

			continue; //next
		}

		//otherwise - does the plane have a child that is a trace object (i.e. it was created in Compass mode)
		for (unsigned c = 0; c < plane->getChildrenNumber(); c++)
		{
			ccHObject* child = plane->getChild(c);
			if (ccTrace::isTrace(child)) //add to recalculate list
			{
				//recalculate the fit plane
				ccTrace* t = static_cast<ccTrace*>(child);
				ccFitPlane* p = t->fitPlane();
				
				if (p)
				{
					//... do some jiggery pokery
					parent->addChild(p); //add fit-plane to the original fit-plane's parent (as we are replacing it)
					m_app->addToDB(p, false, false, false, false);

					//remove the trace from the original fit-plane
					plane->detachChild(t);

					//add it to the new one
					p->addChild(t);

					//add the old plane to the garbage list (to be deleted later)
					garbage.push_back(plane);

					break;
				}
			}
		}
	}

	//delete all the objects in the garbage
	for (int i = 0; i < garbage.size(); i++)
	{
		garbage[i]->getParent()->removeChild(garbage[i]);
	}
}

//prior distribution for orientations (depends on outcrop orientation)
inline double prior(double phi, double theta, double nx, double ny, double nz)
{
	//check normal points down
	if (nz > 0)
	{
		nx *= -1; ny *= -1; nz *= -1;
	}

	//calculate angle between normal vector and the normal estimate(phi, theta)
	double alpha = acos(nx * sin(phi)*cos(theta) + ny * cos(phi) * cos(theta) - nz * sin(theta));
	return sin(alpha) / (2 * M_PI); //n.b. 2pi is normalising factor so that function integrates to one over all phi,theta
}

//calculate log scale-factor for wishart dist. This only needs to be done once per X, so is pulled out of the wish function for performance
inline double logWishSF(CCLib::SquareMatrixd X, int nobserved)
{
	//calculate determinant of X
	double detX = X.m_values[0][0] * ((X.m_values[1][1] * X.m_values[2][2]) - (X.m_values[2][1] * X.m_values[1][2])) -
		X.m_values[0][1] * (X.m_values[1][0] * X.m_values[2][2] - X.m_values[2][0] * X.m_values[1][2]) +
		X.m_values[0][2] * (X.m_values[1][0] * X.m_values[2][1] - X.m_values[2][0] * X.m_values[1][1]);

	return (nobserved - 4.0)*0.5*log(detX) - (nobserved*3. / 2.)*log(2.0) -   // parts of gamma function that do not depend on the scale matrix
		((3.0 / 2.0)*log(M_PI) + lgamma(nobserved / 2.0) + lgamma((nobserved / 2.0) - 0.5) + lgamma((nobserved / 2.0) - 1.0)); // log(gamma3(nobserved/2))
}

//calculate log wishart probability density
inline double logWishart(CCLib::SquareMatrixd X, int nobserved, double phi, double theta, double alpha, double e1, double e2, double e3, double lsf)
{
	//--------------------------------------------------
	//Derive scale matrix eigenvectors (basis matrix)
	//--------------------------------------------------
	double e[3][3];
	double i[3][3];

	//eigenvector 3 (normal to plane defined by theta->phi)
	e[0][2] = sin(phi) * cos(theta);
	e[1][2] = cos(phi) * cos(theta);
	e[2][2] = -sin(theta);
	//eigenvector 2 (normal of theta->phi projected into horizontal plane and rotated by angle alpha)
	e[0][1] = sin(phi) * sin(theta) * sin(alpha) - cos(phi) * cos(alpha);
	e[1][1] = sin(phi) * cos(alpha) + sin(theta) * cos(phi) * sin(alpha);
	e[2][1] = sin(alpha) * cos(theta);
	//eigenvector 1 (calculate using cross product)
	e[0][0] = e[1][2] * e[2][1] - e[2][2] * e[1][1];
	e[1][0] = e[2][2] * e[0][1] - e[0][2] * e[2][1];
	e[2][0] = e[0][2] * e[1][1] - e[1][2] * e[0][1];

	//calculate determinant of the scale matrix by multiplying it's eigens
	double D = e1*e2*e3;

	//calculate the inverse of the scale matrix (we don't actually need to compute the scale matrix)
	e1 = 1.0 / e1; //N.B. Note that by inverting the eigenvalues we compute the inverse scale matrix
	e2 = 1.0 / e2;
	e3 = 1.0 / e3;

	//calculate unique components of I from the eigenvectors and inverted eigenvalues
	i[0][0] = e1*e[0][0] * e[0][0] + e2*e[0][1] * e[0][1] + e3*e[0][2] * e[0][2]; //diagonal component
	i[1][1] = e1*e[1][0] * e[1][0] + e2*e[1][1] * e[1][1] + e3*e[1][2] * e[1][2];
	i[2][2] = e1*e[2][0] * e[2][0] + e2*e[2][1] * e[2][1] + e3*e[2][2] * e[2][2];
	i[0][1] = e1*e[0][0] * e[1][0] + e2*e[0][1] * e[1][1] + e3*e[0][2] * e[1][2]; //off-axis component
	i[0][2] = e1*e[0][0] * e[2][0] + e2*e[0][1] * e[2][1] + e3*e[0][2] * e[2][2];
	i[1][2] = e1*e[1][0] * e[2][0] + e2*e[1][1] * e[2][1] + e3*e[1][2] * e[2][2];

	//compute the trace of I times X
	double trIX = (i[0][0] * X.m_values[0][0] + i[0][1] * X.m_values[1][0] + i[0][2] * X.m_values[2][0]) +
		(i[0][1] * X.m_values[0][1] + i[1][1] * X.m_values[1][1] + i[1][2] * X.m_values[2][1]) +
		(i[0][2] * X.m_values[0][2] + i[1][2] * X.m_values[1][2] + i[2][2] * X.m_values[2][2]);

	//return the log wishart probability density
	return lsf - 0.5 * (trIX + nobserved*log(D));
}

//Estimate the normal vector to the structure this trace represents at each point in this trace.
//declare variables for the dlg used by the below function as statics, so they are remembered between uses (for convenience)
static unsigned int minsize = 500; //these are the defaults
static unsigned int maxsize = 1000;
static double tcDistance = 10.0; //the square of the maximum distance to compute thicknesses for
static unsigned int oversample = 30;
static double likPower = 1.0;
static bool calcThickness = true;
static double stride = 0.025;
static int dof = 10;
void ccCompass::estimateStructureNormals()
{
	//******************************************
	//build dialog to get input properties
	//******************************************
	QDialog dlg(m_app->getMainWindow());
	QVBoxLayout* vbox = new QVBoxLayout();
	QLabel minSizeLabel("Minimum trace size (points):");
	QLineEdit minSizeText(QString::number(minsize)); minSizeText.setValidator(new QIntValidator(5, std::numeric_limits<int>::max()));
	QLabel maxSizeLabel("Maximum trace size (points):");
	QLineEdit maxSizeText(QString::number(maxsize)); maxSizeText.setValidator(new QIntValidator(50, std::numeric_limits<int>::max()));
	QLabel dofLabel("Wishart Degrees of Freedom:");
	QLineEdit dofText(QString::number(dof)); dofText.setValidator(new QIntValidator(3, std::numeric_limits<int>::max()));
	QLabel likPowerLabel("Likelihood power:");
	QLineEdit likPowerText(QString::number(likPower)); likPowerText.setValidator(new QDoubleValidator(0.01, std::numeric_limits<double>::max(), 6));
	QLabel calcThickLabel("Calculate thickness:");
	QCheckBox calcThickChk("Calculate thickness"); calcThickChk.setChecked(calcThickness);
	QLabel distanceLabel("Distance cutoff (m):");
	QLineEdit distanceText(QString::number(tcDistance)); distanceText.setValidator(new QDoubleValidator(0, std::numeric_limits<double>::max(), 6));
	QLabel sampleLabel("Samples:");
	QLineEdit sampleText(QString::number(oversample)); sampleText.setValidator(new QIntValidator(1, 10000)); //>10000 samples per point will break even the best computer!
	QLabel strideLabel("MCMC Stride (radians):");
	QLineEdit strideText(QString::number(stride)); strideText.setValidator(new QDoubleValidator(0.0000001, 0.5, 6));

	//tooltips
	minSizeText.setToolTip("The minimum size of the normal-estimation window.");
	maxSizeText.setToolTip("The maximum size of the normal-estimation window.");
	dofText.setToolTip("Sets the degrees of freedom parameter for the Wishart distribution. Due to non-independent data/errors in traces, this should be low (~10). Higher give more confident results - use with care!");
	distanceText.setToolTip("The furthest distance to search for points on the opposite surface of a GeoObject during thickness calculations.");
	sampleText.setToolTip("Sample n orientation estimates at each point in each trace to quantify uncertainty.");
	likPowerText.setToolTip("Fudge factor to change the balance between the prior and likelihood functions. Advanced use only - see docs for details.");
	strideText.setToolTip("Standard deviation of the normal distribution used to calculate monte-carlo jumps during sampling. Larger numbers sample more widely but are slower to run.");
	
	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dlg, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dlg, SLOT(reject()));

	vbox->addWidget(&minSizeLabel);
	vbox->addWidget(&minSizeText);
	vbox->addWidget(&maxSizeLabel);
	vbox->addWidget(&maxSizeText);
	vbox->addWidget(&dofLabel);
	vbox->addWidget(&dofText);
	vbox->addWidget(&likPowerLabel);
	vbox->addWidget(&likPowerText);
	vbox->addWidget(&sampleLabel);
	vbox->addWidget(&sampleText);
	vbox->addWidget(&strideLabel);
	vbox->addWidget(&strideText);
	vbox->addWidget(&calcThickLabel);
	vbox->addWidget(&calcThickChk);
	vbox->addWidget(&distanceLabel);
	vbox->addWidget(&distanceText);
	vbox->addWidget(&buttonBox);

	dlg.setLayout(vbox);

	//execute dialog and get results
	int result = dlg.exec();
	if (result == QDialog::Rejected) {
		return; //bail!
	}

	//get values
	minsize = minSizeText.text().toInt(); //these are the defaults
	maxsize = maxSizeText.text().toInt();
	dof = dofText.text().toInt();
	tcDistance = distanceText.text().toDouble(); //the square of the maximum distance to compute thicknesses for
	oversample = sampleText.text().toInt();
	likPower = likPowerText.text().toDouble();
	calcThickness = calcThickChk.isChecked();
	stride = strideText.text().toDouble();

	//cleanup
	dlg.close();
	delete vbox;
	
	//someone is an idiot
	if (maxsize < minsize) {
		m_app->dispToConsole("[ccCompass] Error - provided maxsize is less than minsize? Get your shit together...", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	m_app->dispToConsole("[ccCompass] Estimating structure normals. This may take a while...", ccMainAppInterface::STD_CONSOLE_MESSAGE);

	//declare some variables used in the loops
	double d = 0.0;
	double cx = 0.0;
	double cy = 0.0;
	double cz = 0.0;
	int iid = 0;
	CCLib::SquareMatrixd eigVectors;
	std::vector<double> eigValues;
	bool hasNormals = true;
	bool broken = false; //assume normals exist until check later on

	//setup progress dialog
	ccProgressDialog prg(true, m_app->getMainWindow());
	prg.setMethodTitle("Estimating Structure Normals");
	prg.setInfo("Gathering data...");
	prg.start();
	prg.update(0.0);

	//gather objects to process
	std::vector<std::array<ccHObject*,2>> datasets; //upper/lower surfaces will be put into this array 
	std::vector<ccPointCloud*> pinchClouds;
	for (ccHObject* o : m_app->getSelectedEntities())
	{
		//option 1 - selected object is a GeoObject or has GeoObject children
		ccHObject::Container objs;
		if (ccGeoObject::isGeoObject(o)) { //selected object is a geoObject
				objs.push_back(o);
		} else //otherwise search for all GeoObjects
		{
			o->filterChildren(objs, true, CC_TYPES::HIERARCHY_OBJECT); //n.b. geoObjects are simpy considered to be hierarchy objects by CC
		}

		bool foundGeoObject = false;
		for (ccHObject* o2 : objs) {
			if (ccGeoObject::isGeoObject(o2)) {
				ccGeoObject* g = dynamic_cast<ccGeoObject*> (o2);
				if (g) {//could possibly be null if non-loaded geo-objects exist
					foundGeoObject = true; //use to escape to next object later

					//store upper and lower regions
					std::array<ccHObject*, 2> data = { g->getRegion(ccGeoObject::LOWER_BOUNDARY),g->getRegion(ccGeoObject::UPPER_BOUNDARY) };			
					if (ccGeoObject::isSingleSurfaceGeoObject(g)) { //special case - single surface geoboject (upper and lower regions will be the same). Set upper to null
						data[1] = nullptr; }
					datasets.push_back(data);

					//build empty point cloud for pinch nodes to go in
					ccPointCloud* cloud = new ccPointCloud(); //points will be written here if the object is a GeoObject and if it contains pinch nodes
					pinchClouds.push_back(cloud); //store it

					//gather pinch-nodes from GeoObject
					ccHObject::Container objs;
					g->filterChildren(objs, true, CC_TYPES::POLY_LINE); //pinch nodes inherit the polyline clas
					for (ccHObject* c : objs) {
						if (ccPinchNode::isPinchNode(c)) {  //is it a pinch node?
							ccPinchNode* p = dynamic_cast<ccPinchNode*>(c);
							if (p != nullptr) //can in rare cases fail
							{
								cloud->reserve(cloud->size() + 1); //pinch nodes only have one point
								cloud->addPoint(*p->getPoint(0)); //get this point
							}
						}
					}
				}
			}
		}
		if (foundGeoObject) {
			continue; //skip to next object if we found one (or more!) GeoObjects
		}

		//option 2 - selected object is a trace or has children that are traces
		objs.clear();
		if (ccTrace::isTrace(o)) { //selected object is a trace
			objs.push_back(o);
		}
		else {//otherwise search for all GeoObjects
			o->filterChildren(objs, true, CC_TYPES::POLY_LINE); //n.b. geoObjects are simpy considered to be hierarchy objects by CC
		}
		for (ccHObject* o2 : objs) {
			if (ccTrace::isTrace(o2) && o2->isEnabled()) {//is it a trace?
				ccTrace* t = dynamic_cast<ccTrace*> (o2);
				if (t != nullptr) {//can in rare cases be a null ptr (dynamic cast will fail for traces that haven't been converted to ccTrace objects)
					std::array<ccHObject*, 2> data = { t, nullptr };
					datasets.push_back(data); //store data for processing
					pinchClouds.push_back(new ccPointCloud()); //push empty cloud (no pinch nodes).
				}
			}
		}
	}

	if (datasets.empty()) { //no data found
		m_app->dispToConsole("[ccCompass] No GeoObjects or Traces could be found to estimate structure normals for. Please select some!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	}

	//process datasets std::array<ccHObject*, 2> regions : datasets
	for (int _d = 0; _d < datasets.size(); _d++)
	{
		//update progress dialog
		prg.setInfo(QStringLiteral("Processing %1 of %2 datasets: Calculating fit planes...").arg( _d+1 ).arg( datasets.size() ));
		prg.update(0.0f);
		if (prg.isCancelRequested()) {
			break;
		}

		//get regions and pinchNodes to work on this step
		std::array<ccHObject*, 2> regions = datasets[_d];
		ccPointCloud* pinchNodes = pinchClouds[_d];

		//************************************************
		//LOAD POINT DATA FROM TRACESS IN REGIONS
		//************************************************
		ccPointCloud* points[] = { new ccSNECloud(),  //Lower Boundary Points 
			new ccSNECloud() }; //Upper Boundary Points (will remain empty for everything execept multi-surface GeoObjects)
		ccPointCloud* samples[] = { nullptr, nullptr }; //lower and upper boundary samples (will be populated later if samples are generated).

		//for lower,upper in the case of a GeoObject, otherwise regions[1] will be null and will be ignored
		for (unsigned r = 0; r < 2; r++) 
		{
			if (regions[r] == nullptr) {
				delete points[r];
				continue; //skip null regions
			}

			//search for traces in this region
			ccHObject::Container objs;
			if (ccTrace::isTrace(regions[r])) { //given object is a trace
				objs.push_back(regions[r]);
			} else { //otherwise search for child traces (this is a GeoObject region so traces need to be joined together)
				regions[r]->filterChildren(objs, true, CC_TYPES::POLY_LINE);
			}
			for (ccHObject* c : objs)
			{
				if (ccTrace::isTrace(c) && c->isEnabled()) //is it a trace?
				{
					ccTrace* t = dynamic_cast<ccTrace*> (c);
					if (t != nullptr) //can in rare cases be a null ptr (dynamic cast will fail for traces that haven't been converted to ccTrace objects)
					{
						//copy points from this trace across into the relevant point cloud for future access
						points[r]->reserve(points[r]->size() + t->size()); //make space
						points[r]->reserveTheNormsTable(); //make space for normals
						points[r]->setGlobalScale(t->getGlobalScale()); //copy global shift & scale onto new point cloud
						points[r]->setGlobalShift(t->getGlobalShift());
						for (unsigned p = 0; p < t->size(); p++)
						{
							points[r]->addPoint(*t->getPoint(p)); //add point to relevant surface
							points[r]->addNorm(t->getPointNormal(p)); //add point normal
						}
					}
				}
			}

			//skip if there are not enough points!
			if (points[r]->size() < minsize) {
				m_app->dispToConsole(QString::asprintf("[ccCompass] Warning: Region %d contains less than minsize points. Region ignored.", regions[r]->getUniqueID()), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				delete points[r];
				points[r] = nullptr;
				regions[r] = nullptr;
				continue;
			}

			//*********************************************************
			//SORT GATHERED POINTS INTO ORDER ALONG LONG-AXIS OF TRACE
			//*********************************************************
			CCLib::Neighbourhood Z(points[r]); //put points for this surface into a neighbourhood and get the sorting direction (principal eigenvector)
			const CCVector3* longAxis = Z.getLSPlaneX(); //n.b. this is a normal vector
			if (longAxis == nullptr) {
				//fail friendly if eigens could not be computed
				m_app->dispToConsole(QString::asprintf("[ccCompass] Warning: Could not compute eigensystem for region %u. Region ignored.", regions[r]->getUniqueID()), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				continue; //skip to next region
			}

			//now sort points along this vector
			std::vector<unsigned> pid; //store link to point id in original cloud (for later data storage)
			std::vector<double> dist;
			std::vector<double> px;
			std::vector<double> py;
			std::vector<double> pz;
			std::vector<double> nx;
			std::vector<double> ny;
			std::vector<double> nz;
			
			//add first point
			pid.push_back(0); dist.push_back(points[r]->getPoint(0)->dot(*longAxis));
			px.push_back(points[r]->getPoint(0)->x); py.push_back(points[r]->getPoint(0)->y); pz.push_back(points[r]->getPoint(0)->z);
			nx.push_back(points[r]->getPointNormal(0).x); ny.push_back(points[r]->getPointNormal(0).y); nz.push_back(points[r]->getPointNormal(0).z);

			for (unsigned p = 0; p < points[r]->size(); p++) {
				//calculate distance along the longAxis
				d = points[r]->getPoint(p)->dot(*longAxis);

				//quick-check to see if point can just be pushed to end of the list
				if (dist[dist.size() - 1] <= d) {
					pid.push_back(p); dist.push_back(d);
					px.push_back(points[r]->getPoint(p)->x); py.push_back(points[r]->getPoint(p)->y); pz.push_back(points[r]->getPoint(p)->z);
					nx.push_back(points[r]->getPointNormal(p).x); ny.push_back(points[r]->getPointNormal(p).y); nz.push_back(points[r]->getPointNormal(p).z);
				}
				else {
					//find insert point
					for (int n = 0; n < dist.size(); n++)
					{
						//check id = n
						if (dist[n] > d) //found an insert point from the left
						{
							iid = n;
							break;
						} //TODO - could optimise this by searching backwards from the end also? 
					}

					//do inserts
					dist.insert(dist.begin() + iid, d);
					pid.insert(pid.begin() + iid, p);
					px.insert(px.begin() + iid, points[r]->getPoint(p)->x);
					py.insert(py.begin() + iid, points[r]->getPoint(p)->y);
					pz.insert(pz.begin() + iid, points[r]->getPoint(p)->z);
					nx.insert(nx.begin() + iid, points[r]->getPointNormal(p).x);
					ny.insert(ny.begin() + iid, points[r]->getPointNormal(p).y);
					nz.insert(nz.begin() + iid, points[r]->getPointNormal(p).z);
				}
			}

			//**************************************************************************************************
			//CREATE BREAKS AT PINCH NODES (these prevent planes including points from two sides of a pinch node
			//**************************************************************************************************
			std::vector<bool> breaks(px.size(), false); //if point n is a break (closest point to a pinch node), breaks[n] == True.
			CCLib::DgmOctree::NeighboursSet neighbours;

			//build octree over points in combined trace
			ccOctree::Shared oct = points[r]->computeOctree();
			unsigned char level = oct->findBestLevelForAGivenPopulationPerCell(2); //init vars needed for nearest neighbour search
			CCLib::ReferenceCloud* nCloud = new  CCLib::ReferenceCloud(points[r]);
			d = -1.0; //re-use the d variable rather than re-declaring another
			for (unsigned p = 0; p < pinchNodes->size(); p++)
			{
				//get closest point in combined trace to this pinch node
				nCloud->clear(false);
				oct->findPointNeighbourhood(pinchNodes->getPoint(p), nCloud, 1, level, d);
				breaks[nCloud->getPointGlobalIndex(0)] = true; //assign
			}

			//***********************************************************************************************
			//RECURSE THROUGH ALL POSSIBLE COMBINATIONS OF POINTS TO FIND THE BEST STRUCTURE NORMAL ESTIMATE
			//***********************************************************************************************
			//declare variables used in nested loops below
			int n = 0;
			double mnx = 0.0;
			double mny = 0.0;
			double mnz = 0.0;
			double lpd = 0.0;
			double lsf = 0.0;
			double phi = 0.0;
			double theta = 0.0;
			double alpha = 0.0;
			double len = 0.0;
			bool hasValidSNE = false; //becomes true once a valid plane is found
			std::vector<double> bestPd(px.size(), std::numeric_limits<double>::lowest()); //best (log) probability density observed for each point
			std::vector<double> bestPhi(px.size(), 0);
			std::vector<double> bestTheta(px.size(), 0);
			std::vector<double> bestAlpha(px.size(), 0);
			std::vector<double> bestE1(px.size(), 0);
			std::vector<double> bestE2(px.size(), 0);
			std::vector<double> bestE3(px.size(), 0);
			std::vector<CCLib::SquareMatrixd> bestX(px.size()); //keep track of best COV matrix for each trace (for oversampling later)
			std::vector<CCVector3> sne(px.size()); //list of the best surface normal estimates found for each point (corresponds with the MAP above)
			std::vector<int> start(px.size(),0); //index of start point for best planes
			std::vector<int> end(px.size(),0); //index of end point for best planes
			std::vector<int> segmentID(px.size(),-1); //unique id for each point segment.
			std::vector<CCVector3> normal(px.size()); //list of the surface normals (as opposed to structure normals stored in SNE).

			//check if valid normals have been retrieved
			if (hasNormals) {
				if (std::abs(nx[0]) <= 0.000001 && std::abs(ny[0]) <= 0.0000001 && std::abs(nz[0]) <= 0.00000001) //zero normal vector means normals not computed
				{ 
					m_app->dispToConsole("[ccCompass] Warning: Cannot compensate for outcrop-surface bias as point cloud has no normals. Structure normal estimates may be misleading or incorrect.", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
					hasNormals = false; //don't bother checking again - if normals are computed they will exist for all points
				}
			}

			//loop through all possible continuous subsets of the combined trace with minsize < length < maxsize.
			for (unsigned _min = 0; _min < px.size() - minsize; _min++)
			{
				//update progress bar
				prg.update(100 * _min / static_cast<float>(px.size() - minsize));

				if (prg.isCancelRequested()) {

					//cleanup
					delete points[r];
					for (int i = 0; i < pinchClouds.size(); i++) {
						delete pinchClouds[i];
					}
					return; }

				//do inner loop
				for (unsigned _max = _min + minsize; _max < std::min(static_cast<unsigned>(px.size()), _min + maxsize); _max++)
				{
					//size of the current subset
					n = _max - _min + 1;

					//-------------------------------------------------------------------------------------------------------------------------------------
					//compute centroid of points between min and max (and the average normal). Also check if break-point exists (if so skip this subset)
					//-------------------------------------------------------------------------------------------------------------------------------------
					cx = 0.0; cy = 0.0; cz = 0.0;
					mnx = 0.0; mny = 0.0; mnz = 0.0;
					broken = false;
					for (unsigned p = _min; p <= _max; p++) {
						cx += px[p]; cy += py[p]; cz += pz[p]; //average point positions
						if (hasNormals) {
							mnx += nx[p]; mny += ny[p]; mnz += nz[p]; //average point normals
						}
						if (breaks[pid[p]]) { //is this a breakpoint
							broken = true;
							break; //skip to next plane!
						}
					}
					if (broken) {
						break; //skip to next _min point
					}

					cx /= n; cy /= n; cz /= n; //position vector of subset centroid

					if (hasNormals) {
						mnx /= n; mny /= n; mnz /= n; //average normal vector of subset centroid
						len = sqrt(mnx*mnx + mny*mny + mnz*mnz); //normalise
						mnx /= len; mny /= len; mnz /= len;
					}

					hasValidSNE = true; //we have now found at least one valid plane

					//-----------------------------------------------------------------------------
					//compute the scatter and covariance matrices of this section of the trace
					//-----------------------------------------------------------------------------
					CCLib::SquareMatrixd X(3); //scale matrix
					for (unsigned p = _min; p <= _max; p++)
					{
						X.m_values[0][0] += (px[p] - cx) * (px[p] - cx); //mXX
						X.m_values[1][1] += (py[p] - cy) * (py[p] - cy); //mYY
						X.m_values[2][2] += (pz[p] - cz) * (pz[p] - cz); //mZZ
						X.m_values[0][1] += (px[p] - cx) * (py[p] - cy); //mXY
						X.m_values[0][2] += (px[p] - cx) * (pz[p] - cz); //mXZ
						X.m_values[1][2] += (py[p] - cy) * (pz[p] - cz); //mYZ
					}
					
					CCLib::SquareMatrixd cov(3); //convert to covariance matrix
					cov.m_values[0][0] = X.m_values[0][0] / n; cov.m_values[1][1] = X.m_values[1][1] / n; cov.m_values[2][2] = X.m_values[2][2] / n;
					cov.m_values[0][1] = X.m_values[0][1] / n; cov.m_values[0][2] = X.m_values[0][2] / n; cov.m_values[1][2] = X.m_values[1][2] / n;

					//update X to reflect the dof (rather than the true number of samples, as these are not truly independent due to spatial covariance)
					X.m_values[0][0] = cov.m_values[0][0] * dof; X.m_values[1][1] = cov.m_values[1][1] * dof; X.m_values[2][2] = cov.m_values[2][2] * dof;
					X.m_values[0][1] = cov.m_values[0][1] * dof; X.m_values[0][2] = cov.m_values[0][2] * dof; X.m_values[1][2] = cov.m_values[1][2] * dof;

					//fill symmetric parts
					X.m_values[1][0] = X.m_values[0][1]; cov.m_values[1][0] = cov.m_values[0][1];
					X.m_values[2][0] = X.m_values[0][2]; cov.m_values[2][0] = cov.m_values[0][2];
					X.m_values[2][1] = X.m_values[1][2]; cov.m_values[2][1] = cov.m_values[1][2];

					//compute and sort eigens
					Jacobi<double>::ComputeEigenValuesAndVectors(cov, eigVectors, eigValues, true); //get eigens
					Jacobi<double>::SortEigenValuesAndVectors(eigVectors, eigValues); //sort into decreasing order

					//----------------------------------------------------------------------------------------------------
					//Compute the trend and plunge of the best-fit plane (based entirely on the eigensystem).
					//These values will be the maxima of the wishart likelihood distribution and are used to efficiently
					//estimate the maxima a-postiori. This will be incorrect where we are at the low-point in the prior, 
					//but it doesn't matter that much....
					//----------------------------------------------------------------------------------------------------

					//calculate trend and plunge of 3rd eigenvector (this represents the "best-fit-plane").
					phi = atan2(eigVectors.m_values[0][2], eigVectors.m_values[1][2]); //trend of the third eigenvector
					theta = -asin(eigVectors.m_values[2][2]); //plunge of the principal eigenvector

					//ensure phi and theta are in the correct domain
					if (theta < 0) //ensure dip angle is positive
					{
						phi = phi + (M_PI);
						theta = -theta;
					}
					while (phi < 0) //ensure phi ranges between 0 and 2 pi
					{
						phi += 2 * M_PI;
					} while (phi > 2 * M_PI)
					{
						phi -= 2 * M_PI;
					}

					//calculate third angle (alpha) defining the orientation of the eigensystem
					alpha = asin(eigVectors.m_values[2][1] / cos(theta)); //alpha = arcsin(eigVector2.z / cos(theta))

					//map alpha to correct domain (0 to 180 degrees)
					while (alpha < 0) {
						alpha += M_PI;
					}
					while (alpha > M_PI) {
						alpha -= M_PI;
					}

					//compute log-likelihood of this plane estimate
					//dof = _max - _min - 1;
					lsf = logWishSF(X, dof);
					lpd = likPower*logWishart(X, dof, phi, theta, alpha, eigValues[0], eigValues[1], eigValues[2], lsf);

					//multiply by prior 
					if (hasNormals)
					{
						lpd += log(prior(phi, theta, mnx, mny, mnz));
					}

					//----------------------------------------------------------------------------
					//Check if this is the best observed posterior probability
					//----------------------------------------------------------------------------
					for (unsigned p = _min; p <= _max; p++)
					{
						if (lpd > bestPd[p]) //this is a better Pd
						{
							bestPd[p] = lpd;
							bestPhi[p] = phi;
							bestTheta[p] = theta;
							bestAlpha[p] = alpha;
							bestE1[p] = eigValues[0];
							bestE2[p] = eigValues[1];
							bestE3[p] = eigValues[2];
							sne[p] = CCVector3(eigVectors.m_values[0][2], eigVectors.m_values[1][2], eigVectors.m_values[2][2]);
							start[p] = _min;
							end[p] = _max;
							segmentID[p] = static_cast<int>(_max * px.size() + _min);
							bestX[p] = X;
							normal[p] = CCVector3(mnx, mny, mnz);
						}
					}
				}
			}

			if (!hasValidSNE) { //if segments between pinch nodes are too small, then we will not get any valid fit-planes
				m_app->dispToConsole(QString::asprintf("[ccCompass] Warning: Region %d contains no valid points (PinchNodes break the trace into small segments?). Region ignored.", regions[r]->getUniqueID()), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				delete points[r];
				points[r] = nullptr;
				regions[r] = nullptr;
				continue;
			}

			//#################################################################################################################
			//STORE SNE ESTIMATES ON CLOUD
			//##################################################################################################################
			points[r]->setName("SNE");

			//build scalar fields
			CCLib::ScalarField* startSF = points[r]->getScalarField(points[r]->addScalarField(new ccScalarField("StartPoint")));
			CCLib::ScalarField* endSF = points[r]->getScalarField(points[r]->addScalarField(new ccScalarField("EndPoint")));
			CCLib::ScalarField* idSF = points[r]->getScalarField(points[r]->addScalarField(new ccScalarField("SegmentID")));
			CCLib::ScalarField* weightSF = points[r]->getScalarField(points[r]->addScalarField(new ccScalarField("Weight")));
			CCLib::ScalarField* trend = points[r]->getScalarField(points[r]->addScalarField(new ccScalarField("Trend")));
			CCLib::ScalarField* plunge = points[r]->getScalarField(points[r]->addScalarField(new ccScalarField("Plunge")));
			CCLib::ScalarField* pointID = points[r]->getScalarField(points[r]->addScalarField(new ccScalarField("PointID"))); //used for linking samples representing the same point

			weightSF->reserve(px.size());
			startSF->reserve(px.size());
			endSF->reserve(px.size());
			idSF->reserve(px.size());
			trend->reserve(px.size());
			plunge->reserve(px.size());
			pointID->reserve(px.size());

			//store best-guess (maximum a-postiori) surface normal estimates
			for (unsigned p = 0; p < px.size(); p++)
			{
				points[r]->setPointNormal(pid[p], sne[p]);
				weightSF->setValue(pid[p], bestPd[p]);
				startSF->setValue(pid[p], start[p]);
				endSF->setValue(pid[p], end[p]);
				idSF->setValue(pid[p], segmentID[p]);
				trend->setValue(pid[p], bestPhi[p] * 180.0 / M_PI);
				plunge->setValue(pid[p], bestTheta[p] * 180.0 / M_PI);
				pointID->setValue(pid[p], pid[p]);
			}
			
			//compute range
			weightSF->computeMinAndMax();
			startSF->computeMinAndMax();
			endSF->computeMinAndMax();
			idSF->computeMinAndMax();
			trend->computeMinAndMax();
			plunge->computeMinAndMax();
			pointID->computeMinAndMax();

			//set weight to visible
			points[r]->setCurrentDisplayedScalarField(0);
			points[r]->showSF(true);

			//add cloud to object
			regions[r]->addChild(points[r]);
			m_app->addToDB(points[r], false, false, false, false);


			//*************************************************************************************
			//SAMPLE ORIENTATIONS FROM POSTERIORS FOR EACH POINTS SNE TO PROPAGATE UNCERTAINTY
			//*************************************************************************************
			if (oversample > 1)
			{
				//build point cloud to store MCMC samples in and associated scalar fields
				samples[r] = new ccSNECloud();
				samples[r]->setName("SNE_Samples");
				samples[r]->setGlobalScale(points[r]->getGlobalScale()); //copy global shift & scale onto new point cloud
				samples[r]->setGlobalShift(points[r]->getGlobalShift());
				samples[r]->reserve(static_cast<unsigned>(px.size())*oversample);
				samples[r]->reserveTheNormsTable();
				CCLib::ScalarField* startSF = samples[r]->getScalarField(samples[r]->addScalarField(new ccScalarField("StartPoint")));
				CCLib::ScalarField* endSF = samples[r]->getScalarField(samples[r]->addScalarField(new ccScalarField("EndPoint")));
				CCLib::ScalarField* idSF = samples[r]->getScalarField(samples[r]->addScalarField(new ccScalarField("SegmentID")));
				CCLib::ScalarField* weightSF = samples[r]->getScalarField(samples[r]->addScalarField(new ccScalarField("Weight")));
				CCLib::ScalarField* trend = samples[r]->getScalarField(samples[r]->addScalarField(new ccScalarField("Trend")));
				CCLib::ScalarField* plunge = samples[r]->getScalarField(samples[r]->addScalarField(new ccScalarField("Plunge")));
				CCLib::ScalarField* pointID = samples[r]->getScalarField(samples[r]->addScalarField(new ccScalarField("PointID")));
				weightSF->reserve(px.size()*oversample);
				startSF->reserve(px.size()*oversample);
				endSF->reserve(px.size()*oversample);
				idSF->reserve(px.size()*oversample);
				trend->reserve(px.size()*oversample);
				plunge->reserve(px.size()*oversample);
				pointID->reserve(px.size()*oversample);

				//init random number generators 
				std::random_device rd;
				std::default_random_engine generator(rd());
				std::normal_distribution<double> N(0.0, stride); //construct random samplers
				std::uniform_real_distribution<double> U(0.0, 1.0);

				//loop through points
				for (unsigned p = 0; p < px.size(); p++) 
				{
					//update progress dialog
					prg.setInfo(QStringLiteral("Processing %1 of %2 datasets: Sampling points...").arg( _d + 1 ).arg( datasets.size() ));
					prg.update(100.0f * p / static_cast<float>(px.size()));
					if (prg.isCancelRequested()) {
						//cleanup
						delete points[r];
						for (int i = 0; i < pinchClouds.size(); i++)
						{
							delete pinchClouds[i];
							if (samples[0] != nullptr)
							{
								delete samples[0];
							}
							if (samples[1] != nullptr)
							{
								delete samples[1];
							}
						}
						return;
					}

					//skip stand-alone points (with no co-variance matrix)
					if (bestX[p].m_values == nullptr)
					{
						continue; 
					}

					//calculate log scale factor for wish distribution
					lsf = logWishSF(bestX[p], dof);

					//initialise MCMC sampler at likelihood maxima (to avoid need for burn-in period)
					double lpdProposed;
					double lpdCurrent = bestPd[p];
					double phi = bestPhi[p];
					double theta = bestTheta[p];
					double alpha = bestAlpha[p];
					double e1 = bestE1[p];
					double e2 = bestE2[p];
					double e3 = bestE3[p];
					
					//proposals
					double _phi = 0.0;
					double _theta = 0.0;
					double _alpha = 0.0;

					//generate chain
					unsigned count = 0;
					unsigned iter = 0;
					while (count < oversample)
					{
						//generate proposed sample
						_phi = phi + N(generator);
						_theta = theta + N(generator);
						_alpha = alpha + N(generator);

						//evaluate log-likelihood of proposal
						lpdProposed = likPower*logWishart(bestX[p], dof, _phi, _theta, _alpha, e1, e2, e3, lsf);

						//apply prior?
						if (hasNormals)
						{
							lpdProposed += log(prior(_phi, _theta, normal[p].x, normal[p].y, normal[p].z));
						}

						//accept or reject
						if (log(U(generator)) <= lpdProposed - lpdCurrent)
						{
							//accept - update chain
							phi = _phi;
							theta = _theta;
							alpha = _alpha;

							//calculate normal vector
							CCVector3 norm(sin(phi)*cos(theta), cos(phi)*cos(theta), -sin(theta));

							//store sample in point cloud
							samples[r]->addPoint(CCVector3(px[p], py[p], pz[p]));
							samples[r]->addNorm(norm);
							weightSF->addElement(lpdProposed);
							startSF->addElement(start[p]);
							endSF->addElement(end[p]);
							idSF->addElement(segmentID[p]);
							trend->addElement(phi * 180.0 / M_PI);
							plunge->addElement(theta * 180.0 / M_PI);
							pointID->addElement(pid[p]);

							//move to next point
							lpdCurrent = lpdProposed;
							count++;
						}

						if (iter > 1000000 * oversample)
						{
							m_app->dispToConsole("[ccCompass] Warning - MCMC sampler failed so sampling will be incomplete.", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
							break;
						}
						iter++;
					}
				}

				//get rid of excess space in arrays (if samples were not generated in sections)
				samples[r]->shrinkToFit();
				samples[r]->normalsHaveChanged();
				weightSF->shrink_to_fit();
				startSF->shrink_to_fit();
				idSF->shrink_to_fit();
				trend->shrink_to_fit();
				plunge->shrink_to_fit();
				pointID->shrink_to_fit();

				//compute range
				weightSF->computeMinAndMax();
				startSF->computeMinAndMax();
				endSF->computeMinAndMax();
				idSF->computeMinAndMax();
				trend->computeMinAndMax();
				plunge->computeMinAndMax();
				pointID->computeMinAndMax();

				//set weight to visible
				samples[r]->setCurrentDisplayedScalarField(0);
				samples[r]->showSF(true);

				//add cloud to object
				regions[r]->addChild(samples[r]);
				m_app->addToDB(samples[r], false, false, false, false);
				//samples[r]->setEnabled(false); //disable by default
			}
		}

		//compute thicknesses if upper + lower surfaces are defined
		if (regions[0] != nullptr && regions[1] != nullptr && calcThickness) //have both surfaces been defined?
		{
			if (points[0]->size() > 0 && points[1]->size() > 0) { //do both surfaces have points in them?
				prg.setInfo(QStringLiteral("Processing %1 of %2 datasets: Estimating thickness...").arg( _d + 1 ).arg( datasets.size() ));
				for (int r = 0; r < 2; r++)
				{
					//make scalar field
					CCLib::ScalarField* thickSF = points[r]->getScalarField(points[r]->addScalarField(new ccScalarField("Thickness")));
					thickSF->reserve(points[r]->size());
					
					//set thickness to visible scalar field
					points[r]->setCurrentDisplayedScalarField(points[r]->getScalarFieldIndexByName("Thickness"));
					points[r]->showSF(true);

					//create scalar field in samples point cloud
					CCLib::ScalarField* thickSF_sample = nullptr;
					CCLib::ScalarField* idSF_sample = nullptr;
					if (samples[r] != nullptr)
					{
						thickSF_sample = samples[r]->getScalarField(samples[r]->addScalarField(new ccScalarField("Thickness")));
						thickSF_sample->reserve(samples[r]->size());
						idSF_sample = samples[r]->getScalarField(samples[r]->getScalarFieldIndexByName("PointID"));
						samples[r]->setCurrentDisplayedScalarField(samples[r]->getScalarFieldIndexByName("Thickness"));
						samples[r]->showSF(true);
					}

					//figure out id of the compared surface (opposite to the current one)
					int compID = 0;
					if (r == 0) {
						compID = 1;
					}

					//get octree for the picking and build picking data structures
					ccOctree::Shared oct = points[compID]->getOctree();
					CCLib::ReferenceCloud* nCloud = new  CCLib::ReferenceCloud(points[compID]);
					unsigned char level = oct->findBestLevelForAGivenNeighbourhoodSizeExtraction(tcDistance/2);
					
					CCLib::DgmOctree::NeighboursSet neighbours;
					d = -1.0;

					//loop through points in this surface
					for (unsigned p = 0; p < points[r]->size(); p++)
					{

						//keep progress bar up to date
						if (r == 0)
						{
							prg.update((50.0f * p) / points[r]->size()); //first 50% from lower surface
						} else
						{
							prg.update(50.0f + (50.0f * p) / points[r]->size()); //second 50% from upper surface
						}
						if (prg.isCancelRequested())
						{
							//cleanup
							for (int i = 0; i < pinchClouds.size(); i++)
							{
								delete pinchClouds[i];
							}
							return;
						}

						//pick nearest point in opposite surface closest to this one
						nCloud->clear();
						oct->findPointNeighbourhood(points[r]->getPoint(p), nCloud, 1, level, d);

						//skip points that are a long way from their opposite neighbours
						if (d > tcDistance*tcDistance)
						{
							thickSF->setValue(p, -1.0);

							if (samples[r] != nullptr)
							{
								for (unsigned s = 0; s < samples[r]->size(); s++)
								{
									if (idSF_sample->getValue(s) == p) //find samples matching this point
									{
										thickSF_sample->setValue(s, -1.0);
									}
								}
							}

							continue; 
						}

						//calculate thickness for this point pair in sne cloud
						//build equation of the plane
						PointCoordinateType pEq[4];
						pEq[0] = points[r]->getPointNormal(p).x;
						pEq[1] = points[r]->getPointNormal(p).y;
						pEq[2] = points[r]->getPointNormal(p).z;
						pEq[3] = points[r]->getPoint(p)->dot(points[r]->getPointNormal(p));

						//calculate point to plane distance
						d = CCLib::DistanceComputationTools::computePoint2PlaneDistance(nCloud->getPoint(0), pEq);

						//write thickness scalar field
						thickSF->setValue(p, std::abs(d));

						//flip normals so that it points in the correct direction
						points[r]->setPointNormal(p, points[r]->getPointNormal(p) * (d / std::abs(d)));

						//if samples have been generated, also calculate thicknesses for matching sets of points
						if (samples[r] != nullptr)
						{
							for (unsigned s = 0; s < samples[r]->size(); s++)
							{
								if (idSF_sample->getValue(s) == p) //find samples matching this point
								{
									//calculate and store thickness
									PointCoordinateType pEq[4];
									pEq[0] = samples[r]->getPointNormal(s).x;
									pEq[1] = samples[r]->getPointNormal(s).y;
									pEq[2] = samples[r]->getPointNormal(s).z;
									pEq[3] = samples[r]->getPoint(s)->dot(samples[r]->getPointNormal(s));
									d = CCLib::DistanceComputationTools::computePoint2PlaneDistance(nCloud->getPoint(0), pEq);
									thickSF_sample->setValue(s, std::abs(d));
									samples[r]->setPointNormal(s, samples[r]->getPointNormal(s) * (d / std::abs(d)));
								}
							}
						}
					}

					//compute min and max of thickness scalar fields
					thickSF->computeMinAndMax();
					if (thickSF_sample != nullptr)
					{
						thickSF_sample->computeMinAndMax();
					}
				}
			}
		}
	}

	//cleanup
	for (int i = 0; i < pinchClouds.size(); i++)
	{
		delete pinchClouds[i];
	}

	//notify finish
	prg.stop();
	m_app->dispToConsole("[ccCompass] Structure normal estimation complete.", ccMainAppInterface::STD_CONSOLE_MESSAGE);

	//redraw
	m_app->redrawAll();
}

//Estimate strain from Mode-I dykes and veins
static double binSize = 50;
static bool useExternalSNE = true;
static bool buildGraphics = true;
static double exag = 2.0f;
void ccCompass::estimateStrain()
{
	//******************************
	//gather structure traces
	//******************************
	std::vector<ccPolyline*> lines;
	for (ccHObject* o : m_app->getSelectedEntities())
	{
		//Is selected object a trace?
		if (ccTrace::isTrace(o) && o->isEnabled()) {
			lines.push_back(static_cast<ccPolyline*>(o));
			continue;
		}

		//Clearly not... what about it's children?
		ccHObject::Container objs;
		o->filterChildren(objs, true, CC_TYPES::POLY_LINE); //look for SNE
		for (ccHObject* c : objs)
		{
			if (ccTrace::isTrace(c) && c->isEnabled())
			{
				lines.push_back(static_cast<ccPolyline*>(c));
			}
		}
	}

	//calculate bounding box of all traces
	float minx = std::numeric_limits<float>::max();
	float maxx = std::numeric_limits<float>::lowest();
	float miny = std::numeric_limits<float>::max();
	float maxy = std::numeric_limits<float>::lowest();
	float minz = std::numeric_limits<float>::max();
	float maxz = std::numeric_limits<float>::lowest();

	if (lines.empty())
	{
		m_app->dispToConsole("[ccCompass] Error - no traces or SNEs found to compute estimate strain with.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//check bounds
	for (ccPolyline* poly : lines)
	{
		CCVector3 bbMin;
		CCVector3 bbMax;
		if (poly->size() > 0) //avoid (0,0,0),(0,0,0) bounding boxes...
		{
			poly->getBoundingBox(bbMin, bbMax);
			minx = std::min(bbMin.x, minx); maxx = std::max(bbMax.x, maxx);
			miny = std::min(bbMin.y, miny); maxy = std::max(bbMax.y, maxy);
			minz = std::min(bbMin.z, minz); maxz = std::max(bbMax.z, maxz);
		}
	}

	//******************************
	//get bin-size from user
	//******************************
	QDialog dlg(m_app->getMainWindow());
	QVBoxLayout* vbox = new QVBoxLayout();
	QLabel boxSizeLabel("Voxel Size:");
	QLineEdit boxSizeText(QString::number(binSize)); boxSizeText.setValidator(new QDoubleValidator(0.00001, std::numeric_limits<double>::max(), 6));
	QCheckBox externalSNEChk("Use external SNE:"); externalSNEChk.setChecked(useExternalSNE);
	QCheckBox buildBlocksChk("Build graphics:"); buildBlocksChk.setChecked(buildGraphics);
	QLabel exagLabel("Shape exaggeration factor:");
	QLineEdit exagText(QString::number(exag)); boxSizeText.setValidator(new QDoubleValidator(0.00001, std::numeric_limits<double>::max(), 6));


	boxSizeText.setToolTip("The voxel size for computing strain. This should be large enough that most boxes contain SNEs.");
	externalSNEChk.setToolTip("Use SNE orientation estimates for outside the current cell if none are avaliable within it.");
	buildBlocksChk.setToolTip("Build graphic strain ellipses and grid domains. Useful for validation.");
	exagText.setToolTip("Exaggerate the shape of strain ellipses for easier visualisation.");

	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dlg, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dlg, SLOT(reject()));
	vbox->addWidget(&boxSizeLabel);
	vbox->addWidget(&boxSizeText);
	vbox->addWidget(&buildBlocksChk);
	vbox->addWidget(&exagLabel);
	vbox->addWidget(&exagText);
	vbox->addWidget(&externalSNEChk);
	vbox->addWidget(&buttonBox);
	dlg.setLayout(vbox);

	//execute dialog and get results
	int result = dlg.exec();
	if (result == QDialog::Rejected) {
		return; //bail!
	}

	//get values
	binSize = boxSizeText.text().toDouble();
	useExternalSNE = externalSNEChk.isChecked();
	buildGraphics = buildBlocksChk.isChecked();
	exag = exagText.text().toDouble();

	//cleanup
	dlg.close();
	delete vbox;

	//setup progress window
	ccProgressDialog prg(true, m_app->getMainWindow());
	prg.setMethodTitle("Computing strain estimates");
	prg.start();
	
	//***************************************
	//build grid for evaluating strain within
	//***************************************
	//pad out by a bin size on each side to avoid gaps due to rounding
	minx -= binSize;
	miny -= binSize;
	minz -= binSize;
	maxx += binSize;
	maxy += binSize;
	maxz += binSize;

	int nx = (maxx - minx) / binSize;
	int ny = (maxy - miny) / binSize;
	int nz = (maxz - minz) / binSize;

	//*********************************************
	//Map geo-objects onto cells
	//*********************************************
	prg.setInfo("Gathering GeoObjects...");
	std::vector< std::unordered_set<ccGeoObject*> > geoObjectBins(nx*ny*nz, std::unordered_set<ccGeoObject*>());
	for (int i = 0; i < lines.size(); i++)
	{
		prg.update(100.0 * i / float(lines.size()));
		if (prg.isCancelRequested())
		{
			return;
		}

		ccGeoObject* g = ccGeoObject::getGeoObjectParent(lines[i]);
		if (g != nullptr)
		{
			for (unsigned p = 0; p < lines[i]->size(); p++)
			{
				CCVector3 V = *lines[i]->getPoint(p);

				//compute cell this point falls in
				int x = (V.x - minx) / binSize;
				int y = (V.y - miny) / binSize;
				int z = (V.z - minz) / binSize;
				int idx = x + nx * (y + ny * z);

				//store reference to this geoobject
				if (idx < geoObjectBins.size())
				{
					geoObjectBins[idx].insert(g);
				}
				else
				{
					//n.b. this *should* never happen!
					const QString message = QStringLiteral( "[ccCompass] Error: cell %1 is outside of mesh bounds (with total size = %2 [%3,%4,%5])." )
											.arg( idx )
											.arg( geoObjectBins.size() )
											.arg( nx ).arg( ny ).arg( nz );
					
					m_app->dispToConsole( message, ccMainAppInterface::ERR_CONSOLE_MESSAGE );
					return;
				}
			}
		}
	}

	//*********************************************
	//Loop through cells and compute strain tensor
	//*********************************************
	std::vector<int> nStructures(nx*ny*nz, 0); //number of structures used to compute the strain tensor per cell
	std::vector<int> nIgnored(nx*ny*nz, 0); //number of structured ignored during the above calculation (as they did not have orientation/thickness estimates)
	std::vector<ccPointCloud*> dataInCell(nx*ny*nz, nullptr);
	
	//init object to store blocks in
	ccHObject* blocks = nullptr;
	if (buildGraphics)
	{
		blocks = new ccHObject("Blocks");
	}

	//init strain tensors
	CCLib::SquareMatrixd I(3); I.toIdentity();
	std::vector<CCLib::SquareMatrixd> F(nx*ny*nz, CCLib::SquareMatrixd(I)); //deformation gradient tensors

	int validCells = 0;
	prg.setInfo("Calculating strain tensors...");
	for (int x = 0; x < nx; x++)
	{
		for (int y = 0; y < ny; y++)
		{
			for (int z = 0; z < nz; z++)
			{
				int idx = x + nx * (y + ny * z);
				prg.update((100.0f * idx) / (nx*ny*nz));
				if (prg.isCancelRequested())
				{
					delete blocks;
					return;
				}

				//build graphics objects. These are deleted later if no graphics were built.
				dataInCell[idx] = new ccSNECloud();
				dataInCell[idx]->setName("DataInCell");
				ccScalarField* thickness = new ccScalarField("Thickness");
				dataInCell[idx]->addScalarField(thickness);

				for (ccGeoObject* g : geoObjectBins[idx])
				{
					//calculate average dilation vector for this structure (based on the SNEs within this cell)
					CCVector3 average_direction; //dilation direction 
					double average_thickness = 0.0; //amount of dilation

					//TODO - write code that also tracks error/variability in orientation/length of dilation vectors?

					int n_lower = 0;
					int n_upper = 0;
					ccHObject::Container objs;
					g->filterChildren(objs, true, CC_TYPES::POINT_CLOUD,true); //look for SNE
					for (ccHObject* c : objs)
					{
						if (ccSNECloud::isSNECloud(c))
						{
							ccSNECloud* s = dynamic_cast<ccSNECloud*>(c);
							if (s != nullptr)
							{
								//check that a thickness scalar field exists
								int thickSF = s->getScalarFieldIndexByName("Thickness");
								if (thickSF != -1)
								{
									s->setCurrentOutScalarField(thickSF);
									int region = ccGeoObject::getGeoObjectRegion(s);
									if (!(region == ccGeoObject::LOWER_BOUNDARY || region == ccGeoObject::UPPER_BOUNDARY))
									{
										continue;
									}

									//loop through points and only pick those that fall in this bin
									for (unsigned p = 0; p < s->size(); p++)
									{
										CCVector3 V = *s->getPoint(p);

										//compute voxel that last vertex of this segment falls in
										int _x = (V.x - minx) / binSize;
										int _y = (V.y - miny) / binSize;
										int _z = (V.z - minz) / binSize;
										int _idx = _x + nx * (_y + ny * _z);

										if (_idx == idx)
										{										
											//compute averages
											CCVector3 normal = s->getPointNormal(p);
											if (average_direction.norm2() == 0.0)
											{
												average_direction = normal;
											}else if (normal.dot(average_direction) < 0)
											{
												//avoid vectors pointing in opposite directions (as opposite normal directions are equivalent)
												average_direction += -1 * normal; 
											} else
											{
												average_direction += normal;
											}
											average_thickness += s->getPointScalarValue(p);

											//increment counters
											if (region == ccGeoObject::UPPER_BOUNDARY)
											{
												n_upper++;
											}
											else
											{
												n_lower++;
											}

											//write to point cloud
											if (buildGraphics)
											{
												dataInCell[idx]->reserve(1);
												thickness->reserve(1);
												thickness->addElement(s->getPointScalarValue(p));
												dataInCell[idx]->addPoint(V);
												dataInCell[idx]->reserveTheNormsTable();
												dataInCell[idx]->addNorm(s->getPointNormal(p));
											}

										}
									}

									if (buildGraphics)
									{
										thickness->computeMinAndMax();
										dataInCell[idx]->setCurrentDisplayedScalarField(0);
										dataInCell[idx]->showSF(true);
									}
								}
							}
						}
					}

					//check that an upper SNE has been observed, otherwise we ignore this structure
					if (n_upper == 0 && n_lower == 0)
					{
						nIgnored[idx]++;
						continue; //skip this structure
					}

					//compute average dilation vector
					average_direction.normalize();
					average_thickness /= (n_lower + n_upper);

					//increment number of structures that have contributed to the strain in this cell and number of valid cells for which
					//strain can be estimated
					validCells++; 
					nStructures[idx]++;

					//build local coordinate system relative to the dyke opening vector (opening vector N, strike vector S, dip vector D)
					CCVector3 N = average_direction;
					CCVector3 S = N.cross(CCVector3(0.0f, 0.0f, 1.0f));
					CCVector3 D = N.cross(S);

					//define basis matrix
					CCLib::SquareMatrixd B(3);
					B.setValue(0, 0, N.x); B.setValue(1, 0, N.y); B.setValue(2, 0, N.z);
					B.setValue(0, 1, S.x); B.setValue(1, 1, S.y); B.setValue(2, 1, S.z);
					B.setValue(0, 2, D.x); B.setValue(1, 2, D.y); B.setValue(2, 2, D.z);

					//compute transform matrix that apply's a scaling/stretching equal to the dyke thickness and in the direction of its normal
					CCLib::SquareMatrixd e(3); e.toIdentity();
					e.setValue(0, 0, (binSize + average_thickness) / binSize); //stretch matrix in local coordinates
					CCLib::SquareMatrixd F_increment = B*(e*B.transposed());// transform to global coords
					
					//apply this (multiply with) the deformation gradient tensor
					//N.B. The order here is important, but we don't know the timing!
					//Hence we need to somehow bootstrap this to try  all possibilites. 
					F[idx] = F_increment*F[idx];

					//build blocks for visualisation?
					if (buildGraphics)
					{
						double gl16[16]; B.toGlMatrix(gl16);
						gl16[12] = minx + (x+0.5) * binSize; gl16[13] = miny + (y+0.5) * binSize; gl16[14] = minz + (z+0.5)*binSize; gl16[15] = 1.0;
						ccGLMatrix gl(gl16);
						ccGenericPrimitive* box = new ccBox(CCVector3(average_thickness, binSize/2, binSize/2), &gl, "BlockStrain");
						dataInCell[idx]->addChild(box);
					}
				}
			}
		}
	}

	//remove progress bar
	prg.close();

	//**************************************************************
	//store strain tensors on point cloud and build graphics
	//**************************************************************
	ccPointCloud* points = new ccPointCloud("Strain");
	points->setGlobalScale(lines[0]->getGlobalScale()); //copy global shift & scale from one of the polylines (N.B. we assume here that all features have the same shift/scale)
	points->setGlobalShift(lines[0]->getGlobalShift());

	points->reserve(validCells);
	ccScalarField* nValidSF = new ccScalarField("nValid");
	ccScalarField* nIgnoredSF = new ccScalarField("nIgnored");
	ccScalarField* JSF = new ccScalarField("J");
	points->addScalarField(nValidSF);
	points->addScalarField(nIgnoredSF);
	points->addScalarField(JSF);
	nValidSF->reserve(validCells);
	nIgnoredSF->reserve(validCells);
	JSF->reserve(validCells);
	ccScalarField* eSF[3][3];
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			eSF[i][j] = new ccScalarField(QString::asprintf("E%d%d", i + 1, j + 1).toStdString().c_str());
			eSF[i][j]->reserve(validCells);
			points->addScalarField(eSF[i][j]);
		}
	}

	ccHObject* ellipses = nullptr;
	ccHObject* grid = nullptr;
	if (buildGraphics)
	{
		ellipses = new ccHObject("Ellipses");
		grid = new ccHObject("Grid");
		points->addChild(ellipses);
		points->addChild(grid);
		points->addChild(blocks);
	}

	//loop through bins and build points/graphics where strain has been estimated
	for (int x = 0; x < nx; x++)
	{
		for (int y = 0; y < ny; y++)
		{
			for (int z = 0; z < nz; z++)
			{
				int idx = x + nx * (y + ny * z);
				if (nStructures[idx] > 0)
				{
					//build point
					CCVector3 p(minx + ((x + 0.5) * binSize), miny + ((y + 0.5) * binSize), minz + ((z + 0.5) * binSize));
					points->addPoint(p);
					nValidSF->addElement(nStructures[idx]);
					nIgnoredSF->addElement(nIgnored[idx]);


					//decompose into the rotation and right-stretch 
					CCLib::SquareMatrixd eigVectors; std::vector<double> eigValues;
					CCLib::SquareMatrixd B = F[idx] * F[idx].transposed();
					Jacobi<double>::ComputeEigenValuesAndVectors(B, eigVectors, eigValues, true); //get eigens

					CCLib::SquareMatrixd U_local(3); U_local.toIdentity();  //calculate stretch matrix in local (un-rotated coordinates)
					U_local.setValue(0, 0, sqrt(eigValues[0])); U_local.setValue(1, 1, sqrt(eigValues[1])); U_local.setValue(2, 2, sqrt(eigValues[2]));
					CCLib::SquareMatrixd U = eigVectors.transposed() * (U_local * eigVectors); //transform back into global coordinates

					//compute jacobian (volumetric strain)
					double J = eigValues[0] * eigValues[1] * eigValues[2]; //F[idx].computeDet();
					JSF->addElement(J);

					//store strain tensor
					for (int i = 0; i < 3; i++)
					{
						for (int j = 0; j < 3; j++)
						{
							eSF[i][j]->addElement(U.getValue(i, j));
						}
					}

					if (buildGraphics)
					{
						//compute eigens of F
						eigVectors.clear(); eigValues.clear();
						Jacobi<double>::ComputeEigenValuesAndVectors(F[idx], eigVectors, eigValues, true); //get eigens
						Jacobi<double>::SortEigenValuesAndVectors(eigVectors, eigValues);
						
						//apply exaggeration to eigenvalues (exaggerate shape of the strain ellipse)
						CCLib::SquareMatrixd transMat(3);
						transMat.setValue(0, 0, pow(eigValues[0] / eigValues[1], exag));
						transMat.setValue(1, 1, pow(eigValues[1] / eigValues[1], exag));
						transMat.setValue(2, 2, pow(eigValues[2] / eigValues[1], exag));

						//transform back into global coords
						transMat = eigVectors * (transMat * eigVectors.transposed());
						double gl16[16]; transMat.toGlMatrix(gl16);
						gl16[12] = p.x; gl16[13] = p.y; gl16[14] = p.z; gl16[15] = 1.0; //add translation to GL matrix
						ccGLMatrix gl(gl16);
						ccGenericPrimitive* ellipse = new ccSphere(binSize / 3, &gl, "StrainEllipse");
						ellipse->setColor(ccColor::blue);
						ellipse->showColors(true);
						ellipses->addChild(ellipse);

						//store strain tensor on the graphic for reference
						QVariantMap* map = new QVariantMap();
						map->insert("Exx", U.getValue(0, 0) - 1.0); map->insert("Exy", U.getValue(0, 1)); map->insert("Exz", U.getValue(0, 2));
						map->insert("Eyx", U.getValue(1, 0)); map->insert("Eyy", U.getValue(1, 1) - 1.0); map->insert("Eyz", U.getValue(1, 2));
						map->insert("Ezx", U.getValue(2, 0)); map->insert("Ezy", U.getValue(2, 1)); map->insert("Ezz", U.getValue(2, 2) - 1.0);
						map->insert("J", J);
						ellipse->setMetaData(*map, true);

						//create cubes to highlight gridding
						ccGLMatrix T;
						T.setTranslation(p);
						ccGenericPrimitive* box = new ccBox(CCVector3(binSize, binSize, binSize), &T, "GridCell");
						grid->addChild(box);
						box->showWired(true);

						//add points to this grid cell
						box->addChild(dataInCell[idx]);
					}
				}
				else //no strain estimate here - cleanup
				{
					delete dataInCell[idx];
				}
			}
		}
	}

	//finalize scalar fields
	nValidSF->computeMinAndMax();
	nIgnoredSF->computeMinAndMax();
	JSF->computeMinAndMax();
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			eSF[i][j]->computeMinAndMax();
		}
	}

	//store & display mesh
	m_app->dbRootObject()->addChild(points);
	m_app->addToDB(points);
	points->setCurrentDisplayedScalarField(0);
	points->showSF(true);
}

//Estimate P21 intensity of selected structures
static double searchR = 10;
static unsigned subsample = 25;
void ccCompass::estimateP21()
{
	//setup point cloud to store data in
	ccPointCloud* cloud = new ccPointCloud();
	ccScalarField* weight = new ccScalarField("weight");
	cloud->addScalarField(weight);
	cloud->setCurrentScalarField(0);

	//******************************
	//gather polylines and SNEs
	//******************************
	std::vector<ccPolyline*> lines;
	std::vector<ccSNECloud*> sne;
	for (ccHObject* o : m_app->getSelectedEntities())
	{
		//Is selected object a trace?
		if (ccTrace::isTrace(o))
		{ 
			lines.push_back(static_cast<ccPolyline*>(o));
			continue;
		}
		//What about an SNE cloud?
		else if (ccSNECloud::isSNECloud(o))
		{
			ccSNECloud* s = dynamic_cast<ccSNECloud*>(o);
			if (s != nullptr)
			{
				sne.push_back(s);
			}
			continue;
		}

		//Clearly not... what about it's children?
		ccHObject::Container objs;
		o->filterChildren(objs, true, CC_TYPES::POINT_CLOUD); //look for SNE
		for (ccHObject* c : objs)
		{
			if (ccSNECloud::isSNECloud(c))
			{
				ccSNECloud* s = dynamic_cast<ccSNECloud*>(c);
				if (s != nullptr)
				{
					sne.push_back(s);
				}
			}
		}

		objs.clear();
		o->filterChildren(objs, true, CC_TYPES::POLY_LINE); //look for SNE
		for (ccHObject* c : objs)
		{
			if (ccTrace::isTrace(c))
			{
				lines.push_back(static_cast<ccPolyline*>(c));
			}
		}
	}

	if (lines.empty())
	{
		m_app->dispToConsole("[ccCompass] Error - no polylines or traces found to compute P21.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//get points from polylines
	ccPointCloud* outcrop = nullptr;
	for (ccPolyline* p : lines)
	{
		//if unknown, find the point cloud features have been digitised on
		if (outcrop == nullptr)
		{
			outcrop = dynamic_cast<ccPointCloud*> (p->getAssociatedCloud());
		}

		//check that all features have been digitised on the same feature...
		if (outcrop != p->getAssociatedCloud())
		{
			m_app->dispToConsole("[ccCompass] Error - cannot calculate P21 intensity for structures digitised from different point clouds.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		int sID = ccGeoObject::getGeoObjectRegion(p); //get the region of any associated geo-object this polyline relates too.
		double w = 1.0;
		if (sID == ccGeoObject::UPPER_BOUNDARY || ccGeoObject::LOWER_BOUNDARY)
		{
			w = 0.5; //upper/lower boundaries only count for 0.5 as they should be represented/counted twice.
		}

		cloud->reserve(p->size());
		weight->reserve(p->size());
		for (unsigned i = 0; i < p->size(); i++)
		{
			cloud->addPoint(*p->getPoint(i));
			weight->addElement(w);
		}
	}

	//compute octree for this cloud (for future picking)
	cloud->computeOctree();

	//******************************
	//get search radius from user
	//******************************
	QDialog dlg(m_app->getMainWindow());
	QVBoxLayout* vbox = new QVBoxLayout();
	QLabel boxSizeLabel("Search Radius:");
	QLineEdit boxSizeText(QString::number(searchR)); boxSizeText.setValidator(new QDoubleValidator(0.00001, std::numeric_limits<double>::max(), 6));
	QLabel subsampleLabel("Subsample:");
	QLineEdit subsampleText(QString::number(subsample)); boxSizeText.setValidator(new QIntValidator(1, std::numeric_limits<int>::max()));

	boxSizeText.setToolTip("The search radius used to define the region to compute P21 within.");
	subsampleText.setToolTip("Only sample P21 on the each n'th point in the original outcrop model (decreases calculation time).");

	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dlg, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dlg, SLOT(reject()));
	vbox->addWidget(&boxSizeLabel);
	vbox->addWidget(&boxSizeText);
	vbox->addWidget(&subsampleLabel);
	vbox->addWidget(&subsampleText);
	vbox->addWidget(&buttonBox);
	dlg.setLayout(vbox);

	//execute dialog and get results
	int result = dlg.exec();
	if (result == QDialog::Rejected) {
		return; //bail!
	}

	//get values
	searchR = boxSizeText.text().toDouble();
	subsample = subsampleText.text().toUInt();
	m_app->dispToConsole(QString::asprintf("[ccCompass] Estimating P21 Intensity using a search radius of of %f.",searchR), ccMainAppInterface::STD_CONSOLE_MESSAGE);

	//cleanup
	dlg.close();
	delete vbox;

	//***************************************************************************
	//Setup output cloud
	//***************************************************************************
	//subsample outcrop cloud
	ccPointCloud* outputCloud = new ccPointCloud("P21 Intensity");
	outputCloud->reserve(outcrop->size() / subsample);
	for (unsigned p = 0; p < outcrop->size(); p+= subsample)
	{
		outputCloud->addPoint(*outcrop->getPoint(p));
	}
	//copy global shift
	outputCloud->setGlobalScale(outcrop->getGlobalScale()); //copy global scale
	outputCloud->setGlobalShift(outcrop->getGlobalShift()); //copy global shift
	
	//setup scalar fields etc
	ccScalarField* P21 = new ccScalarField("P21");
	outputCloud->addScalarField(P21);
	P21->reserve(outputCloud->size());

	//*****************************************************************************
	//Loop through points on outcrop and calculate trace points / outcrop points
	//*****************************************************************************

	//get octree for the picking and build picking data structures
	ccOctree::Shared trace_oct = cloud->computeOctree();
	unsigned char trace_level = trace_oct->findBestLevelForAGivenNeighbourhoodSizeExtraction(searchR);

	//structure for nearest neighbors search
	CCLib::DgmOctree::NeighboursSet region;

	//setup progress dialog
	ccProgressDialog prg(true, m_app->getMainWindow());
	prg.setMethodTitle("Estimating P21 Intensity");
	prg.setInfo("Sampling structures...");
	prg.start();
	prg.update(0.0);

	//loop through points in the output cloud
	for (unsigned p = 0; p < outputCloud->size(); p++)
	{
		//keep progress bar up to date
		prg.update(100.0 * p / static_cast<float>(outputCloud->size()));
		if (prg.isCancelRequested())
		{
			//cleanup
			delete cloud;
			return;
		}

		//get number of structure points in this neighbourhood
		region.clear();
		trace_oct->getPointsInSphericalNeighbourhood(*outputCloud->getPoint(p), searchR, region, trace_level);

		//calculate total weight (think length) of structure points by summing weights
		float sum = 0;
		for (int i = 0; i < region.size(); i++)
		{
			sum += weight->getValue(region[i].pointIndex);
		}

		//calculate and store number of trace points within this domain
		P21->setValue(p, sum);
	}

	//loop through points in output cloud again, but this time calculate surface area in regions where n_trace wasn't zero
	prg.setInfo("Calculating patch areas...");
	ccOctree::Shared outcrop_oct = outputCloud->computeOctree();
	int n_outcrop = 0;
	for (unsigned p = 0; p < outputCloud->size(); p++)
	{
		float sum = P21->getValue(p);
		if (sum > 0) //this domain has at least some structures, so is worth computing patch area
		{

			//keep progress bar up to date
			prg.update(100.0 * p / static_cast<float>(outputCloud->size()));
			if (prg.isCancelRequested())
			{
				//cleanup
				delete cloud;
				return;
			}

			//get number of structure points in this neighbourhood
			region.clear();
			n_outcrop = outcrop_oct->getPointsInSphericalNeighbourhood(*outputCloud->getPoint(p), searchR, region, trace_level);

			//update scalar field
			P21->setValue(p, sum / (n_outcrop*subsample));
		}
	}

	delete cloud;

	//finish
	P21->computeMinAndMax();
	outputCloud->setCurrentDisplayedScalarField(0);
	outputCloud->showSF(true);
	m_app->dbRootObject()->addChild(outputCloud);
	m_app->addToDB(outputCloud);
}

//converts selected traces or geoObjects to point clouds
void ccCompass::convertToPointCloud()
{
	//get selected objects
	std::vector<ccGeoObject*> objs;
	std::vector<ccPolyline*> lines;

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
		else if (o->isA(CC_TYPES::POLY_LINE))
		{
			lines.push_back(static_cast<ccPolyline*> (o));
		}
		else
		{
			//search children for geo-objects and polylines
			ccHObject::Container objs;
			o->filterChildren(objs, true, CC_TYPES::POLY_LINE | CC_TYPES::HIERARCHY_OBJECT);
			for (ccHObject* c : objs)
			{
				if (ccGeoObject::isGeoObject(c))
				{
					ccGeoObject* g = dynamic_cast<ccGeoObject*> (c);
					if (g) //could possibly be null if non-loaded geo-objects exist
					{
						objs.push_back(g);
					}
				}
				if (c->isA(CC_TYPES::POLY_LINE))
				{
					lines.push_back(static_cast<ccPolyline*>(c));
				}
			}
		}
	}

	//convert GeoObjects
	for (ccGeoObject* o : objs)
	{
		//get regions
		ccHObject* regions[3] = { o->getRegion(ccGeoObject::INTERIOR), 
								  o->getRegion(ccGeoObject::LOWER_BOUNDARY), 
								  o->getRegion(ccGeoObject::UPPER_BOUNDARY)};
		
		//make point cloud
		ccPointCloud* points = new ccPointCloud("ConvertedLines"); //create point cloud for storing points
		int sfid = points->addScalarField(new ccScalarField("Region")); //add scalar field containing region info
		CCLib::ScalarField* sf = points->getScalarField(sfid);

		//convert traces in each region
		int nRegions = 3;
		if (ccGeoObject::isSingleSurfaceGeoObject(o))
		{
			nRegions = 1; //single surface objects only have one region
		}
		for (int i = 0; i < nRegions; i++)
		{
			ccHObject* region = regions[i];
			
			//get polylines/traces
			ccHObject::Container poly;
			region->filterChildren(poly, true, CC_TYPES::POLY_LINE);
			
			for (ccHObject::Container::const_iterator it = poly.begin(); it != poly.end(); it++)
			{
				ccPolyline* t = static_cast<ccPolyline*>(*it);
				points->setGlobalScale(t->getGlobalScale()); //copy global scale
				points->setGlobalShift(t->getGlobalShift()); //copy global shift
				points->reserve(points->size() + t->size()); //make space
				sf->reserve(points->size() + t->size());
				for (unsigned int p = 0; p < t->size(); p++)
				{
					points->addPoint(*t->getPoint(p)); //add point to cloud
					sf->addElement(i);
				}
			}
		}

		//save 
		if (points->size() > 0)
		{
			sf->computeMinAndMax();
			points->setCurrentDisplayedScalarField(sfid);
			points->showSF(true);

			regions[2]->addChild(points);
			m_app->addToDB(points, false, true, false, false);
		}
		else
		{
			m_app->dispToConsole("[Compass] No polylines or traces converted - none found.", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			delete points;
		}
	}

	//convert traces not associated with a GeoObject
	if (objs.empty())
	{
		//make point cloud
		ccPointCloud* points = new ccPointCloud("ConvertedLines"); //create point cloud for storing points
		int sfid = points->addScalarField(new ccScalarField("Region")); //add scalar field containing region info
		CCLib::ScalarField* sf = points->getScalarField(sfid);
		int number = 0;
		for (ccPolyline* t : lines)
		{
			number++;
			points->reserve(points->size() + t->size()); //make space
			sf->reserve(points->size() + t->size());
			for (unsigned p = 0; p < t->size(); p++)
			{
				points->addPoint(*t->getPoint(p)); //add point to cloud
				sf->addElement(number);
			}
		}
		if (points->size() > 0)
		{

			sf->computeMinAndMax();
			points->setCurrentDisplayedScalarField(sfid);
			points->showSF(true);

			m_app->dbRootObject()->addChild(points);
			m_app->addToDB(points, false, true, false, true);
		}
		else
		{
			delete points;
		}
	}
}

//distributes selected objects into GeoObjects with the same name
void ccCompass::distributeSelection()
{

	//get selection
	ccHObject::Container selection = m_app->getSelectedEntities();
	if (selection.empty())
	{
		m_app->dispToConsole("[Compass] No objects selected.", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
	}

	//build list of GeoObjects
	std::vector<ccGeoObject*> geoObjs;
	ccHObject::Container search;
	m_app->dbRootObject()->filterChildren(search, true, CC_TYPES::HIERARCHY_OBJECT, false);
	for (ccHObject* obj : search)
	{
		if (ccGeoObject::isGeoObject(obj))
		{
			ccGeoObject* g = dynamic_cast<ccGeoObject*>(obj);
			if (g)
			{
				geoObjs.push_back(g);
			}
		}
	}

	//loop through selection and try to match with a GeoObject
	for (ccHObject* obj : selection)
	{
		//try to match name
		ccGeoObject* bestMatch = nullptr;
		int matchingChars = 0; //size of match
		for (ccGeoObject* g : geoObjs)
		{
			//find geoObject with biggest matching name (this avoids issues with Object_1 and Object_11 matching)
			if (obj->getName().contains(g->getName())) //object name contains a GeoObject name
			{
				if (g->getName().size() > matchingChars)
				{
					matchingChars = g->getName().size();
					bestMatch = g;
				}
			}
		}

		//was a match found?
		if (bestMatch)
		{
			//detach child from parent and DB Tree
			m_app->removeFromDB(obj, false);

			//look for upper or low (otherwise put in interior)
			if (obj->getName().contains("upper"))
			{
				bestMatch->getRegion(ccGeoObject::UPPER_BOUNDARY)->addChild(obj); //add to GeoObject upper
			}
			else if (obj->getName().contains("lower"))
			{
				bestMatch->getRegion(ccGeoObject::LOWER_BOUNDARY)->addChild(obj); //add to GeoObject lower
			}
			else
			{
				bestMatch->getRegion(ccGeoObject::INTERIOR)->addChild(obj); //add to GeoObject interior
			}

			//deselect and update
			obj->setSelected(false);
			m_app->addToDB(obj, false, true, false, false);
		}
		else //a best match was not found...
		{
			m_app->dispToConsole(QString::asprintf("[Compass] Warning: No GeoObject could be found that matches %s.",obj->getName().toLatin1().data()), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		}
	}
	
	m_app->updateUI();
	m_app->redrawAll();
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

void ccCompass::addGeoObject(bool singleSurface) //creates a new GeoObject
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
	ccGeoObject* newGeoObject = new ccGeoObject(name, m_app, singleSurface);
	interp_group->addChild(newGeoObject);
	m_app->addToDB(newGeoObject, false, true, false, false);

	//set it to selected (this will then make it "active" via the selection change callback)
	m_app->setSelectedInDB(newGeoObject, true);
}

void ccCompass::addGeoObjectSS()
{
	addGeoObject(true);
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

//convert a point cloud containing field points (x,y,z) and dip+dip-direction scalar fields to planes for visualisation.
void ccCompass::importFoliations()
{
	//get selected point cloud
	std::vector<ccHObject*> sel = m_app->getSelectedEntities();
	if (sel.empty())
	{
		m_app->dispToConsole("Please select a point cloud containing your field data (this can be loaded from a text file)",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	if (!sel[0]->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Please select a point cloud containing your field data (this can be loaded from a text file)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//get point cloud object
	ccPointCloud* cld = static_cast<ccPointCloud*>(sel[0]);

	//get user to select scalar field for dip & dip-directon
	QDialog dlg(m_app->getMainWindow());
	QVBoxLayout* vbox = new QVBoxLayout();
	QLabel dipLabel("Dip Field:");
	QLabel dipDirLabel("Dip-Direction Field:");
	QLabel sizeLabel("Plane Size");
	QComboBox dipDirCombo(m_app->getMainWindow());
	QComboBox dipCombo(m_app->getMainWindow());
	QLineEdit planeSize("2.0"); planeSize.setValidator(new QDoubleValidator(0.01, std::numeric_limits<double>::max(), 6));

	//fill combo boxes with field names
	//std::vector<QString> fields;
	//std::vector<int> idx;
	for (unsigned i = 0; i < cld->getNumberOfScalarFields(); i++)
	{
		dipDirCombo.addItem(cld->getScalarFieldName(i));
		dipCombo.addItem(cld->getScalarFieldName(i));
		//fields.push_back(cld->getScalarFieldName(i));
		//idx.push_back(i);
	}

	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dlg, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dlg, SLOT(reject()));

	vbox->addWidget(&dipLabel);
	vbox->addWidget(&dipCombo);
	vbox->addWidget(&dipDirLabel);
	vbox->addWidget(&dipDirCombo);
	vbox->addWidget(&buttonBox);
	vbox->addWidget(&sizeLabel);
	vbox->addWidget(&planeSize);
	dlg.setLayout(vbox);

	//execute dialog and get results
	int result = dlg.exec();
	if (result == QDialog::Rejected) {
		return; //bail!
	}


	//get values
	int dipSF = cld->getScalarFieldIndexByName(dipCombo.currentText().toStdString().c_str());
	int dipDirSF = cld->getScalarFieldIndexByName(dipDirCombo.currentText().toStdString().c_str());
	double size = planeSize.text().toDouble();
	if (dipSF == dipDirSF)
	{
		m_app->dispToConsole("Error: Dip and Dip-Direction scalar fields must be different!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//loop through points
	for (unsigned p = 0; p < cld->size(); p++)
	{
		float dip = cld->getScalarField(dipSF)->at(p);
		float dipdir = cld->getScalarField(dipDirSF)->at(p);
		CCVector3 Cd = *cld->getPoint(p);

		//build plane and get its orientation 
		ccPlane* plane = new ccPlane(QString("%1/%2").arg(static_cast<int>(dip), 2, 10, QChar('0')).arg(static_cast<int>(dipdir), 3, 10, QChar('0')));
		plane->showNameIn3D(true);
		cld->addChild(plane);
		m_app->addToDB(plane, false, true, false, false);
		CCVector3 N = plane->getNormal();
		CCVector3 C = plane->getCenter();

		//figure out transform (blatantly stolen from ccPlaneEditDlg::updatePlane())
		CCVector3 Nd = ccNormalVectors::ConvertDipAndDipDirToNormal(dip, dipdir, true);
		ccGLMatrix trans;
		bool needToApplyTrans = false;
		bool needToApplyRot = false;

		needToApplyRot = (std::abs(N.dot(Nd) - PC_ONE) > std::numeric_limits<PointCoordinateType>::epsilon());
		needToApplyTrans = needToApplyRot || ((C - Cd).norm2d() != 0);

		if (needToApplyTrans)
		{
			trans.setTranslation(-C);
			needToApplyTrans = true;
		}
		if (needToApplyRot)
		{
			ccGLMatrix rotation;
			//special case: plane parallel to XY
			if (std::abs(N.z) > PC_ONE - std::numeric_limits<PointCoordinateType>::epsilon())
			{
				ccGLMatrix rotX; rotX.initFromParameters(-dip * CC_DEG_TO_RAD, CCVector3(1, 0, 0), CCVector3(0, 0, 0)); //plunge
				ccGLMatrix rotZ; rotZ.initFromParameters(dipdir * CC_DEG_TO_RAD, CCVector3(0, 0, -1), CCVector3(0, 0, 0));
				rotation = rotZ * rotX;
			}
			else //general case
			{
				rotation = ccGLMatrix::FromToRotation(N, Nd);
			}
			trans = rotation * trans;
		}
		if (needToApplyTrans)
		{
			trans.setTranslation(trans.getTranslationAsVec3D() + Cd);
		}
		if (needToApplyRot || needToApplyTrans)
		{
			plane->applyGLTransformation_recursive(&trans);

			//ccLog::Print("[Plane edit] Applied transformation matrix:");
			//ccLog::Print(trans.toString(12, ' ')); //full precision
		}
		plane->setXWidth(size, false);
		plane->setYWidth(size, true);
	}
}

//convert a point cloud containing field points (x,y,z) and trend + plunge scalar fields to lineation vectors for visualisation.
void ccCompass::importLineations()
{
	//get selected point cloud
	std::vector<ccHObject*> sel = m_app->getSelectedEntities();
	if (sel.empty())
	{
		m_app->dispToConsole("Please select a point cloud containing your field data (this can be loaded from a text file)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	if (!sel[0]->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Please select a point cloud containing your field data (this can be loaded from a text file)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//get point cloud object
	ccPointCloud* cld = static_cast<ccPointCloud*>(sel[0]);

	//get user to select scalar field for dip & dip-directon
	QDialog dlg(m_app->getMainWindow());
	QVBoxLayout* vbox = new QVBoxLayout();
	QLabel dipLabel("Trend Field:");
	QLabel dipDirLabel("Plunge Field:");
	QLabel sizeLabel("Display Length");
	QComboBox dipDirCombo(m_app->getMainWindow());
	QComboBox dipCombo(m_app->getMainWindow());
	QLineEdit planeSize("2.0"); planeSize.setValidator(new QDoubleValidator(0.01, std::numeric_limits<double>::max(), 6));

	//fill combo boxes with field names
	for (unsigned i = 0; i < cld->getNumberOfScalarFields(); i++)
	{
		dipDirCombo.addItem(cld->getScalarFieldName(i));
		dipCombo.addItem(cld->getScalarFieldName(i));
	}

	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dlg, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dlg, SLOT(reject()));

	vbox->addWidget(&dipLabel);
	vbox->addWidget(&dipCombo);
	vbox->addWidget(&dipDirLabel);
	vbox->addWidget(&dipDirCombo);
	vbox->addWidget(&buttonBox);
	vbox->addWidget(&sizeLabel);
	vbox->addWidget(&planeSize);
	dlg.setLayout(vbox);

	//execute dialog and get results
	int result = dlg.exec();
	if (result == QDialog::Rejected) {
		return; //bail!
	}

	//get values
	int dipSF = cld->getScalarFieldIndexByName(dipCombo.currentText().toStdString().c_str());
	int dipDirSF = cld->getScalarFieldIndexByName(dipDirCombo.currentText().toStdString().c_str());
	double size = planeSize.text().toDouble();
	if (dipSF == dipDirSF)
	{
		m_app->dispToConsole("Error: Trend and plunge scalar fields must be different!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//loop through points
	for (unsigned p = 0; p < cld->size(); p++)
	{
		float trend = cld->getScalarField(dipSF)->at(p);
		float plunge = cld->getScalarField(dipDirSF)->at(p);
		CCVector3 Cd = *cld->getPoint(p);
		
		//build lineation vector
		CCVector3 l(sin(trend * CC_DEG_TO_RAD) * cos(plunge * CC_DEG_TO_RAD), cos(trend * CC_DEG_TO_RAD)*cos(plunge * CC_DEG_TO_RAD), -sin(plunge * CC_DEG_TO_RAD));

		//create new point cloud to associate with lineation graphic
		ccPointCloud* points = new ccPointCloud();
		points->setGlobalScale(cld->getGlobalScale()); //copy global shift & scale onto new point cloud
		points->setGlobalShift(cld->getGlobalShift());
		points->reserve(2);
		points->addPoint(Cd);
		points->addPoint(Cd + l*size);
		points->setName("verts");

		//create lineation graphic
		ccLineation* lineation = new ccLineation(points);
		lineation->addChild(points);
		lineation->addPointIndex(0);
		lineation->addPointIndex(1);
		lineation->updateMetadata();
		lineation->setName(QStringLiteral("%1->%2").arg( qRound( plunge ) ).arg( qRound( trend ) ));
		cld->addChild(lineation);
		m_app->addToDB(lineation, false, true, false, false);
	}
}

//save the current view to an SVG file
void ccCompass::exportToSVG()
{
	float zoom = 2.0f; //TODO: create popup box

	//get filename for the svg file
	QString filename = QFileDialog::getSaveFileName(m_dlg, tr("SVG Output file"), "", tr("SVG files (*.svg)"));
	if (filename.isEmpty())
	{
		//process cancelled by the user
		return;
	}

	if (QFileInfo(filename).suffix() != "svg")
	{
		filename += ".svg";
	}


	//set all objects except the point clouds invisible
	std::vector<ccHObject*> hidden; //store objects we hide so we can turn them back on after!
	ccHObject::Container objects;
	m_app->dbRootObject()->filterChildren(objects, true, CC_TYPES::OBJECT, false); //get list of all children!
	for (ccHObject* o : objects)
	{
		if (!o->isA(CC_TYPES::POINT_CLOUD))
		{
			if (o->isVisible())
			{
				hidden.push_back(o);
				o->setVisible(false);
			}
		}
	}

	//render the scene
	QImage img = m_app->getActiveGLWindow()->renderToImage(zoom);

	//restore visibility
	for (ccHObject* o : hidden)
	{
		o->setVisible(true);
	}

	//convert image to base64 (png format) to write in svg file
	QByteArray ba;
	QBuffer bu(&ba);
	bu.open(QIODevice::WriteOnly);
	img.save(&bu, "PNG");
	bu.close();

	//create .svg file
	QFile svg_file(filename);
	//open file & create text stream
	if (svg_file.open(QIODevice::WriteOnly))
	{
		QTextStream svg_stream(&svg_file);

		int width  = std::abs(static_cast<int>(m_app->getActiveGLWindow()->glWidth()  * zoom)); //glWidth and glHeight are negative on some machines??
		int height = std::abs(static_cast<int>(m_app->getActiveGLWindow()->glHeight() * zoom));

		//write svg header
		svg_stream << QString::asprintf("<svg width=\"%d\" height=\"%d\">", width, height) << endl;

		//write the image
		svg_stream << QString::asprintf("<image height = \"%d\" width = \"%d\" xlink:href = \"data:image/png;base64,", height, width) << ba.toBase64() << "\"/>" << endl;

		//recursively write traces
		int count = writeTracesSVG(m_app->dbRootObject(), &svg_stream, height,zoom);

		//TODO: write scale bar

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

int ccCompass::writeTracesSVG(ccHObject* object, QTextStream* out, int height, float zoom)
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
			//m_app->getActiveGLWindow()->redraw(false, false); //not sure if this is needed or not?
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
			*out << QString::asprintf("%.3f,%.3f ", coords2D.x*zoom, height - (coords2D.y*zoom)); //n.b. we need to flip y-axis

		}

		//end polyline
		*out << "\"/>" << endl;

		n++; //a polyline has been written
	}

	//recurse on children
	for (unsigned i = 0; i < object->getChildrenNumber(); i++)
	{
		n += writeTracesSVG(object->getChild(i), out, height, zoom);
	}

	return n;
}

//export interpretations to csv or xml
void ccCompass::onSave()
{
	//get output file path
	QString filename = QFileDialog::getSaveFileName(m_dlg, tr("Output file"), "", tr("CSV files (*.csv *.txt);;XML (*.xml)"));
	if (filename.isEmpty())
	{
		//process cancelled by the user
		return;
	}

	//is this an xml file?
	QFileInfo fi(filename);
	if (fi.suffix() == "xml")
	{
		writeToXML(filename); //write xml file
		return;
	}

	//otherwise write a whole bunch of .csv files

	int planes = 0; //keep track of how many objects are being written (used to delete empty files)
	int traces = 0;
	int lineations = 0;
	int thicknesses = 0;

	/*
	QString filename = QFileDialog::getSaveFileName(m_dlg, tr("Output file"), "", tr("XML files (*.xml *.txt)"));
	*/
	//build filenames

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
		plane_stream << "Name,Strike,Dip,Dip_Dir,Cx,Cy,Cz,Nx,Ny,Nz,Sample_Radius,RMS,Gx,Gy,Gz,Length" << endl;
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
int ccCompass::writePlanes(ccHObject* object, QTextStream* out, const QString &parentName)
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

	//find point cloud (biggest in project) to pull global shift & scale from
	//n.b. ccPlanes do not store a global shift/scale like point clouds do, hence this hack.
	//Will only cause issues if CC is being used with multiple point clouds that have different underlying
	//coordinate systems (and hence shift and scales) - which I can't see happening too often.
	//(in any case, 99% of the time the biggest point cloud in the project will be the model being interpreted)
	ccPointCloud* ss = nullptr; 
	if (object->isKindOf(CC_TYPES::PLANE) | ccFitPlane::isFitPlane(object))
	{
		std::vector<ccHObject*> clouds;
		m_app->dbRootObject()->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, false);
		unsigned int npoints = 0;
		for (ccHObject* o : clouds)
		{
			ccPointCloud* p = static_cast<ccPointCloud*>(o);
			if (npoints <= p->size())
			{
				npoints = p->size();
				ss = p;
			}
		}
	}

	//is object a plane made by ccCompass?
	int n = 0;
	if (ccFitPlane::isFitPlane(object))
	{
		//write global position
		ccPlane* P = static_cast<ccPlane*>(object);

		//Write object as Name,Strike,Dip,Dip_Dir,Cx,Cy,Cz,Nx,Ny,Nz,Radius,RMS,Gx,Gy,Gz,Length
		*out << name << ",";
		*out << object->getMetaData("Strike").toString() << "," << object->getMetaData("Dip").toString() << "," << object->getMetaData("DipDir").toString() << ",";
		*out << object->getMetaData("Cx").toString() << "," << object->getMetaData("Cy").toString() << "," << object->getMetaData("Cz").toString() << ",";
		*out << object->getMetaData("Nx").toString() << "," << object->getMetaData("Ny").toString() << "," << object->getMetaData("Nz").toString() << ",";
		*out << object->getMetaData("Radius").toString() << "," << object->getMetaData("RMS").toString() << ",";

		if (ss != nullptr)
		{
			CCVector3 L = P->getTransformation().getTranslationAsVec3D();
			CCVector3d G = ss->toGlobal3d(L);

			*out << G.x << "," << G.y << "," << G.z << ",";
		}

		//write length of trace associated with this plane
		*out << std::max(P->getXWidth(), P->getYWidth()) << endl;
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
		float strike = 0.0f;
		float dip = 0.0f;
		float dipdir = 0.0f;
		ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipdir);
		ccNormalVectors::ConvertNormalToStrikeAndDip(N, strike, dip);

		//export
		*out << name << ",";
		*out << strike << "," << dip << "," << dipdir << ","; //write orientation
		*out << L.x << "," << L.y << "," << L.z << ","; //write location
		*out << N.x << "," << N.y << "," << N.z << ","; //write normal
		*out << "NA" << "," << "UNK" << ","; //the "radius" and "RMS" are unknown

	    //write global position
		if (ss != nullptr)
		{
			CCVector3d G = ss->toGlobal3d(L);
			*out << G.x << "," << G.y << "," << G.z << endl;
		}
		else
		{
			*out << endl;
		}

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
int ccCompass::writeTraces(ccHObject* object, QTextStream* out, const QString &parentName)
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
		CCVector3 start;
		CCVector3 end;
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
				*out << p->toGlobal3d(start).x << ",";
				*out << p->toGlobal3d(start).y << ",";
				*out << p->toGlobal3d(start).z << ",";
				*out << p->toGlobal3d(end).x << ",";
				*out << p->toGlobal3d(end).y << ",";
				*out << p->toGlobal3d(end).z << ",";
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
int ccCompass::writeLineations(ccHObject* object, QTextStream* out, const QString &parentName, bool thicknesses)
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


int ccCompass::writeToXML(const QString &filename)
{
	int n = 0;

	//open output stream
	QFile file(filename);
	if (file.open(QIODevice::WriteOnly)) //open the file
	{
		//QTextStream plane_stream(&plane_file);
		QXmlStreamWriter xmlWriter(&file); //open xml stream;

		xmlWriter.setAutoFormatting(true);
		xmlWriter.writeStartDocument();

		//find root node
		ccHObject* root = m_app->dbRootObject();
		if (root->getChildrenNumber() == 1)
		{
			root = root->getChild(0); //HACK - often the root only has one child (a .bin file); if so, move down a level
		}

		/*ccHObject::Container pointClouds;
		m_app->dbRootObject()->filterChildren(&pointClouds, true, CC_TYPES::POINT_CLOUD, true);*/

		//write data tree
		n += writeObjectXML(root, &xmlWriter);

		//write end of document
		xmlWriter.writeEndDocument();

		//close
		file.flush();
		file.close();

		m_app->dispToConsole("[ccCompass] Successfully exported data-tree to xml.", ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}
	else
	{
		m_app->dispToConsole("[ccCompass] Could not open output files... ensure CC has write access to this location.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	}

	return n;
}

//recursively write the provided ccHObject and its children
int ccCompass::writeObjectXML(ccHObject* object, QXmlStreamWriter* out)
{
	int n = 1;
	//write object header based on type
	if (ccGeoObject::isGeoObject(object))
	{
		//write GeoObject
		out->writeStartElement("GEO_OBJECT");
	}
	else if (object->isA(CC_TYPES::PLANE))
	{
		//write fitPlane
		out->writeStartElement("PLANE");
	}
	else if (ccTrace::isTrace(object))
	{
		//write trace
		out->writeStartElement("TRACE");
	}
	else if (ccThickness::isThickness(object))
	{
		//write thickness
		out->writeStartElement("THICKNESS");
	}
	else if (ccSNECloud::isSNECloud(object))
	{
		out->writeStartElement("SNE");
	}
	else if (ccLineation::isLineation(object))
	{
		//write lineation
		out->writeStartElement("LINEATION");
	}
	else if (object->isA(CC_TYPES::POINT_CLOUD))
	{
		out->writeStartElement("CLOUD");
	}
	else if (object->isA(CC_TYPES::POLY_LINE))
	{
		//write polyline (note that this will ignore "trace" polylines as they have been grabbed earlier)
		out->writeStartElement("POLYLINE");
	}
	else if (object->isA(CC_TYPES::HIERARCHY_OBJECT))
	{
		//write container
		out->writeStartElement("CONTAINER"); //QString::asprintf("CONTAINER name = '%s' id = %d", object->getName(), object->getUniqueID())
	}
	else if (object->isA(CC_TYPES::FACET))
	{
		//write container
		out->writeStartElement("FACET");
	}
	else //just write name,id and metadata for unknown objects
	{
		out->writeStartElement("OBJECT");
	}

	//write name and oid attributes
	out->writeAttribute("name", object->getName());
	out->writeAttribute("id", QString::asprintf("%d", object->getUniqueID()));

	//write metadata tags (these contain the data)
	for (QMap<QString, QVariant>::const_iterator it = object->metaData().begin(); it != object->metaData().end(); it++)
	{
		out->writeTextElement(it.key(), it.value().toString());
	}

	//special case - we can calculate all metadata from a plane
	if (object->isA(CC_TYPES::PLANE))
	{
		ccPlane* P = static_cast<ccPlane*> (object);

		//write length
		out->writeTextElement("Length", QString::asprintf("%f", std::max(P->getXWidth(), P->getYWidth())));

		//if this is just an ordinary plane, make a corresponding fitplane object and then steal metadata
		if (!ccFitPlane::isFitPlane(P))
		{
			//build fitplane object
			ccFitPlane* temp = new ccFitPlane(P);

			//write metadata
			for (QMap<QString, QVariant>::const_iterator it = temp->metaData().begin(); it != temp->metaData().end(); it++)
			{
				out->writeTextElement(it.key(), it.value().toString());
			}

			//cleanup
			delete temp;
		}
	}

	//if object is a polyline object (or a trace) write trace points and normals
	if (object->isA(CC_TYPES::POLY_LINE))
	{
		ccPolyline* poly = static_cast<ccPolyline*>(object);
		ccTrace* trace = nullptr;
		if (ccTrace::isTrace(object))
		{
			trace = static_cast<ccTrace*>(object);
		}

		QString x;
		QString y;
		QString z;
		QString nx;
		QString ny;
		QString nz;
		QString cost;
		QString wIDs;
		QString w_local_ids;

		//loop through points
		//position
		CCVector3 p1;
		CCVector3 p2;
		
		//normal vector (if defined)
		CCVector3 n1;
		CCVector3 n2;

		//becomes true if any valid normals are recieved
		bool hasNormals = false;

		if (poly->size() >= 2)
		{
			//loop through segments
			for (unsigned i = 1; i < poly->size(); i++)
			{
				//get points
				poly->getPoint(i - 1, p1); //segment start point
				poly->getPoint(i, p2); //segment end point

				//store data to buffers
				x += QString::asprintf("%f,", p1.x);
				y += QString::asprintf("%f,", p1.y);
				z += QString::asprintf("%f,", p1.z);

				//write data specific to traces
				if (trace)
				{
					int c = trace->getSegmentCost(trace->getPointGlobalIndex(i - 1), trace->getPointGlobalIndex(i));
					cost += QString::asprintf("%d,", c);

					//write point normals (if this is a trace)
					n2 = trace->getPointNormal(i);
					nx += QString::asprintf("%f,", n1.x);
					ny += QString::asprintf("%f,", n1.y);
					nz += QString::asprintf("%f,", n1.z);
					if (!hasNormals && !(n1.x == 0 && n1.y == 0 && n1.z == 0))
					{
						hasNormals = true; //this was a non-null normal estimate - we will write normals now
					}
				}

			}

			//store last point
			x += QString::asprintf("%f", p2.x);
			y += QString::asprintf("%f", p2.y);
			z += QString::asprintf("%f", p2.z);
			if (hasNormals) //normal
			{
				nx += QString::asprintf("%f", n2.x);
				ny += QString::asprintf("%f", n2.y);
				nz += QString::asprintf("%f", n2.z);
			}
			if (trace) //cost
			{
				cost += "0";
			}

			//if this is a trace also write the waypoints
			if (trace)
			{
				//get ids (on the cloud) for waypoints
				for (int w = 0; w < trace->waypoint_count(); w++)
				{
					wIDs += QString::asprintf("%d,", trace->getWaypoint(w));
				}

				//get ids (vertex # in polyline) for waypoints
				for (int w = 0; w < trace->waypoint_count(); w++)
				{
					
					//get id of waypoint in cloud
					int globalID = trace->getWaypoint(w);

					//find corresponding point in trace
					unsigned i = 0;
					for (; i < trace->size(); i++)
					{
						if (trace->getPointGlobalIndex(i) == globalID)
						{
							break; //found it!;
						}
					}

					//write this points local index
					w_local_ids += QString::asprintf("%d,", i);
				}

			}
			//write points
			out->writeStartElement("POINTS");
			out->writeAttribute("count", QString::asprintf("%d", poly->size()));

			if (hasNormals)
			{
				out->writeAttribute("normals", "True");
			}
			else
			{
				out->writeAttribute("normals", "False");
			}

			out->writeTextElement("x", x);
			out->writeTextElement("y", y);
			out->writeTextElement("z", z);

			if (hasNormals)
			{
				out->writeTextElement("nx", nx);
				out->writeTextElement("ny", ny);
				out->writeTextElement("nz", nz);
			}

			if (trace)
			{
				//write waypoints
				out->writeTextElement("cost", cost);
				out->writeTextElement("control_point_cloud_ids", wIDs);
				out->writeTextElement("control_point_local_ids", w_local_ids);
			}

			//fin!
			out->writeEndElement();
		}
	}

	//if object is a point cloud write global shift and scale
	if (object->isA(CC_TYPES::POINT_CLOUD))
	{
		ccPointCloud* cloud = static_cast<ccPointCloud*>(object);
		out->writeTextElement("GLOBAL_SCALE", QString::asprintf("%f", cloud->getGlobalScale()));
		out->writeTextElement("GLOBAL_X", QString::asprintf("%f", cloud->getGlobalShift().x));
		out->writeTextElement("GLOBAL_Y", QString::asprintf("%f", cloud->getGlobalShift().y));
		out->writeTextElement("GLOBAL_Z", QString::asprintf("%f", cloud->getGlobalShift().z));

		//for SNE clouds write all points, point normals and scalar fields
		if (ccSNECloud::isSNECloud(object))
		{
			//write header for point data
			out->writeStartElement("POINTS");
			out->writeAttribute("count", QString::asprintf("%d", cloud->size()));

			//gather data strings
			QString x;
			QString y;
			QString z;
			QString nx;
			QString ny;
			QString nz;
			QString thickness;
			QString weight;
			QString trend;
			QString plunge;
			CCLib::ScalarField* wSF = cloud->getScalarField(cloud->getScalarFieldIndexByName("Weight"));
			CCLib::ScalarField* trendSF = cloud->getScalarField(cloud->getScalarFieldIndexByName("Trend"));
			CCLib::ScalarField* plungeSF = cloud->getScalarField(cloud->getScalarFieldIndexByName("Plunge"));

			CCLib::ScalarField* tSF = cloud->getScalarField(cloud->getScalarFieldIndexByName("Thickness"));
			for (unsigned p = 0; p < cloud->size(); p++)
			{
				x += QString::asprintf("%f,", cloud->getPoint(p)->x);
				y += QString::asprintf("%f,", cloud->getPoint(p)->y);
				z += QString::asprintf("%f,", cloud->getPoint(p)->z);
				nx += QString::asprintf("%f,", cloud->getPointNormal(p).x);
				ny += QString::asprintf("%f,", cloud->getPointNormal(p).y);
				nz += QString::asprintf("%f,", cloud->getPointNormal(p).z);
				weight += QString::asprintf("%f,", wSF->getValue(p));
				trend += QString::asprintf("%f,", trendSF->getValue(p));
				plunge += QString::asprintf("%f,", plungeSF->getValue(p));


				if (tSF != nullptr) //can be null if no thickness was estimated!
				{
					thickness += QString::asprintf("%f,", tSF->getValue(p));
				}
			}

			//write
			out->writeTextElement("x", x);
			out->writeTextElement("y", y);
			out->writeTextElement("z", z);
			out->writeTextElement("nx", nx);
			out->writeTextElement("ny", ny);
			out->writeTextElement("nz", nz);
			out->writeTextElement("weight", weight);
			out->writeTextElement("trend", trend);
			out->writeTextElement("plunge", plunge);
			if (tSF != nullptr)
			{
				out->writeTextElement("thickness", thickness);
			}

			//fin
			out->writeEndElement();
		}
	}

	//write facet data
	if (object->isA(CC_TYPES::FACET))
	{
		//write orientation
		ccFacet* f = static_cast<ccFacet*>(object);
		out->writeTextElement("Nx", QString::asprintf("%f", f->getNormal().x));
		out->writeTextElement("Ny", QString::asprintf("%f", f->getNormal().y));
		out->writeTextElement("Nz", QString::asprintf("%f", f->getNormal().z));
		out->writeTextElement("Cx", QString::asprintf("%f", f->getCenter().x));
		out->writeTextElement("Cy", QString::asprintf("%f", f->getCenter().y));
		out->writeTextElement("Cz", QString::asprintf("%f", f->getCenter().z));
		out->writeTextElement("rms", QString::asprintf("%f", f->getRMS()));
		out->writeTextElement("surface", QString::asprintf("%f", f->getSurface()));
	}

	//write children
	for (unsigned i = 0; i < object->getChildrenNumber(); i++)
	{
		n += writeObjectXML(object->getChild(i), out);
	}

	//close this object
	out->writeEndElement();

	return n;
}

