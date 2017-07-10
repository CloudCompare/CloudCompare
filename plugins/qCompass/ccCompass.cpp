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

#include "ccCompass.h"

//Qt
#include <QtGui>
#include <QFileInfo>

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

	//activate plane tool by default
	m_activeTool = m_fitPlaneTool;
}

//deconstructor
ccCompass::~ccCompass()
{
	//delete all tools
	delete m_fitPlaneTool;
	delete m_traceTool;
	delete m_lineationTool;
}

void ccCompass::onNewSelection(const ccHObject::Container& selectedEntities)
{
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
				if (m_geoObject) //cast succeded
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

	m_app->dispToConsole("[ccCompass] ccCompass plugin initialized succesfully.", ccMainAppInterface::STD_CONSOLE_MESSAGE);

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

	//Get handle to ccGLWindow
	m_window = m_app->getActiveGLWindow();

	//check valid window
	if (!m_window)
	{
		m_app->dispToConsole("[ccCompass] Could not find valid 3D window.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//bind gui
	if (!m_dlg)
	{
		//bind GUI events
		m_dlg = new ccCompassDlg(m_app->getMainWindow());

		ccCompassDlg::connect(m_dlg->closeButton, SIGNAL(clicked()), this, SLOT(onClose()));
		ccCompassDlg::connect(m_dlg->acceptButton, SIGNAL(clicked()), this, SLOT(onAccept()));
		ccCompassDlg::connect(m_dlg->saveButton, SIGNAL(clicked()), this, SLOT(onSave()));
		ccCompassDlg::connect(m_dlg->undoButton, SIGNAL(clicked()), this, SLOT(onUndo()));
		ccCompassDlg::connect(m_dlg->pairModeButton, SIGNAL(clicked()), this, SLOT(setLineationMode()));
		ccCompassDlg::connect(m_dlg->planeModeButton, SIGNAL(clicked()), this, SLOT(setPlaneMode()));
		ccCompassDlg::connect(m_dlg->traceModeButton, SIGNAL(clicked()), this, SLOT(setTraceMode()));
		ccCompassDlg::connect(m_dlg->categoryBox, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(changeType()));
		ccCompassDlg::connect(m_dlg->infoButton, SIGNAL(clicked()), this, SLOT(showHelp()));

		ccCompassDlg::connect(m_dlg->m_showNames, SIGNAL(toggled(bool)), this, SLOT(toggleLabels(bool)));
		ccCompassDlg::connect(m_dlg->m_showStippled, SIGNAL(toggled(bool)), this, SLOT(toggleStipple(bool)));
		ccCompassDlg::connect(m_dlg->m_showNormals, SIGNAL(toggled(bool)), this, SLOT(toggleNormals(bool)));

		ccCompassDlg::connect(m_dlg->mapMode, SIGNAL(clicked()), this, SLOT(enableMapMode()));
		ccCompassDlg::connect(m_dlg->compassMode, SIGNAL(clicked()), this, SLOT(enableMeasureMode()));
	}

	if (!m_mapDlg)
	{
		m_mapDlg = new ccMapDlg(m_app->getMainWindow());

		ccCompassDlg::connect(m_mapDlg->addObjectButton, SIGNAL(clicked()), this, SLOT(addGeoObject()));
		ccCompassDlg::connect(m_mapDlg->pickObjectButton, SIGNAL(clicked()), this, SLOT(pickGeoObject()));
		ccCompassDlg::connect(m_mapDlg->clearObjectButton, SIGNAL(clicked()), this, SLOT(clearGeoObject()));
		ccCompassDlg::connect(m_mapDlg->setInteriorButton, SIGNAL(clicked()), this, SLOT(writeToInterior()));
		ccCompassDlg::connect(m_mapDlg->setUpperButton, SIGNAL(clicked()), this, SLOT(writeToUpper()));
		ccCompassDlg::connect(m_mapDlg->setLowerButton, SIGNAL(clicked()), this, SLOT(writeToLower()));
	}

	m_dlg->linkWith(m_window);
	m_mapDlg->linkWith(m_window);

	//loop through DB_Tree and find any ccCompass objects
	std::vector<ccHObject*> originals;
	std::vector<ccHObject*> replacements;
	for (int i = 0; i < m_app->dbRootObject()->getChildrenNumber(); i++)
	{
		ccHObject* c = m_app->dbRootObject()->getChild(i);
		tryLoading(c,&originals,&replacements);
	}

	//replace all "originals" with their corresponding "duplicates"
	for (int i = 0; i < originals.size(); i++)
	{
		//originals[i]->transferChildren(*replacements[i]); 
		//steal all the children!
		for (int c = 0; c < originals[i]->getChildrenNumber(); c++)
		{
			replacements[i]->addChild(originals[i]->getChild(c));
		}

		//add replacement
		originals[i]->getParent()->addChild(replacements[i]);

		//remove original from scene graph/db tree
		originals[i]->getParent()->detachChild(originals[i]);

		//m_app->dbRootObject()->addChild(replacements[i]);
		//m_app->addToDB(replacements[i], false, true, false, false);
	}

	//start in measure mode
	enableMeasureMode();

	//trigger selection changed
	onNewSelection(m_app->getSelectedEntities());

	//begin measuring
	startMeasuring();
}

void ccCompass::tryLoading(ccHObject* obj, std::vector<ccHObject*>* originals, std::vector<ccHObject*>* replacements)
{
	//is object already represented by a ccCompass class?
	if (dynamic_cast<ccFitPlane*>(obj)
		|| dynamic_cast<ccTrace*>(obj)
		|| dynamic_cast<ccLineation*>(obj)
		|| dynamic_cast<ccGeoObject*>(obj))
	{
		return; //we need do nothing!
	}

	//recurse on children
	for (int i = 0; i < obj->getChildrenNumber(); i++)
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
		originals->push_back(obj);
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
			originals->push_back(obj);
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
			originals->push_back(obj);
			replacements->push_back(trace);
			return;
		}

		//are we a lineation?
		if (ccLineation::isLineation(obj))
		{
			ccHObject* lin = new ccLineation(p);
			originals->push_back(obj);
			replacements->push_back(lin);
			return;
		}
	}
}

//Begin measuring 
bool ccCompass::startMeasuring()
{
	//check valid gl window
	if (!m_window)
	{
		//invalid pointer error
		m_app->dispToConsole("Error: ccCompass could not find the Cloud Compare window. Abort!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

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

	//setup listener for mouse events
	m_window->installEventFilter(this);

	//refresh window
	m_window->redraw(true, false);

	//start GUI
	m_app->registerOverlayDialog(m_dlg, Qt::TopRightCorner);
	m_dlg->start();

	//activate active tool
	if (m_activeTool)
	{
		m_activeTool->toolActivated();
	}
	

	//safety: auto stop if the window is deleted
	connect(m_window, &QObject::destroyed, [&]() { m_window = 0; stopMeasuring(); });

	return true;
}

//Exits measuring
bool ccCompass::stopMeasuring()
{
	//remove click listener
	if (m_window)
	{
		m_window->removeEventFilter(this);
	}

	//stop picking
	if (m_app->pickingHub())
	{
		m_app->pickingHub()->removeListener(this);
	}

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
	if (m_window)
	{
		m_window->redraw(true, false);
	}

	return true;
}

//Get the place/object that new measurements or interpretation should be stored
ccHObject* ccCompass::getInsertPoint()
{

	//check if there is an active GeoObject and we are in mapMode
	if (ccCompass::mapMode)
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

		//search for relevant category group within this
		ccHObject* category_group = nullptr;
		for (unsigned i = 0; i < measurement_group->getChildrenNumber(); i++) //check if a category group exists
		{
			if (measurement_group->getChild(i)->getName() == m_category)
			{
				category_group = measurement_group->getChild(i);
				break;
			}
		}

		//didn't find it... create it!
		if (!category_group)
		{
			category_group = new ccHObject(m_category);
			measurement_group->addChild(category_group);
			m_app->addToDB(category_group, false, true, false, false);
		}

		return category_group; //this is the insert point
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
		return;

	//no active tool - disregard pick
	if (!m_activeTool)
		return;

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
	m_window->redraw();
}

bool ccCompass::eventFilter(QObject* obj, QEvent* event)
{
	//update cost mode (just in case it has changed) & fit plane params
	ccCompass::costMode = m_dlg->getCostMode();
	ccCompass::fitPlanes = m_dlg->planeFitMode();
	ccTrace::COST_MODE = ccCompass::costMode;

	//reorder menu's (sloppy, but works for now?)
	m_app->updateOverlayDialogsPlacement();

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
	m_activeTool->toolDisactivated();

	//uncheck/disable gui components (the relevant ones will be activated later)
	m_dlg->pairModeButton->setChecked(false);
	m_dlg->planeModeButton->setChecked(false);
	m_dlg->traceModeButton->setChecked(false);
	m_dlg->paintModeButton->setChecked(false);
	m_dlg->undoButton->setEnabled(false);
	m_dlg->acceptButton->setEnabled(false);
}

//activate lineation mode
void ccCompass::setLineationMode()
{
	//cleanup
	cleanupBeforeToolChange();

	//activate lineation tool
	m_activeTool = m_lineationTool;
	m_activeTool->toolActivated();

	//trigger selection changed
	onNewSelection(m_app->getSelectedEntities());

	//update GUI
	m_dlg->undoButton->setEnabled(m_lineationTool->canUndo());
	m_dlg->pairModeButton->setChecked(true);
	m_window->redraw(true, false);
}

//activate plane mode
void ccCompass::setPlaneMode()
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
	m_window->redraw(true, false);
}

//activate trace mode
void ccCompass::setTraceMode()
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
	//m_dlg->m_cost_algorithm_menu->setEnabled(true); //add algorithm menu
	m_window->redraw(true, false);
}

void ccCompass::setPaintMode()
{
	m_dlg->paintModeButton->setChecked(true);
	
	//do nothing more for now (paint mode not yet availiable

}

//toggle stippling
void ccCompass::toggleStipple(bool checked)
{
	ccCompass::drawStippled = checked; //change stippling for newly created planes
	recurseStipple(m_app->dbRootObject(), checked); //change stippling for existing planes
	m_window->redraw(); //redraw
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
	m_window->redraw(); //redraw
}

void ccCompass::recurseLabels(ccHObject* object, bool checked)
{
	//check this object
	if (ccFitPlane::isFitPlane(object) | ccLineation::isLineation(object))
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
	m_window->redraw(); //redraw
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

//called when the "structure type" combo is changed
void ccCompass::changeType()
{
	if (m_dlg->categoryBox->currentText().contains("Custom"))
	{
		m_dlg->categoryBox->blockSignals(true);

		//get name
		QString name = QInputDialog::getText(m_dlg, "Custom type", "Structure type:");

		//add to category box & set active
		if (name != "")
			m_dlg->categoryBox->insertItem(0, name);

		//set first category (normally the new one) to active
		m_dlg->categoryBox->setCurrentIndex(0);

		m_dlg->categoryBox->blockSignals(false);
	}
	m_category = m_dlg->categoryBox->currentText();
}

//displays the info dialog
void ccCompass::showHelp()
{
	//create new qt window
	ccCompassInfo info(m_app->getMainWindow());
	info.exec();
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

	//create files
	QFile plane_file(plane_fn);
	QFile trace_file(trace_fn);
	QFile lineation_file(lineation_fn);

	//open files
	if (plane_file.open(QIODevice::WriteOnly) && trace_file.open(QIODevice::WriteOnly) && lineation_file.open(QIODevice::WriteOnly))
	{
		//create text streams for each file
		QTextStream plane_stream(&plane_file);
		QTextStream trace_stream(&trace_file);
		QTextStream lineation_stream(&lineation_file);

		//write headers
		plane_stream << "Name,Strike,Dip,Dip_Dir,Cx,Cy,Cz,Nx,Ny,Nz,Sample_Radius,RMS" << endl;
		trace_stream << "name,trace_id,point_id,start_x,start_y,start_z,end_x,end_y,end_z" << endl;
		lineation_stream << "name,Sx,Sy,Sz,Ex,Ey,Ez,Trend,Plunge" << endl;

		//write data for all objects in the db tree (n.b. we loop through the dbRoots children rathern than just passing db_root so the naming is correct)
		for (unsigned i = 0; i < m_app->dbRootObject()->getChildrenNumber(); i++)
		{
			ccHObject* o = m_app->dbRootObject()->getChild(i);
			planes += writePlanes(o, &plane_stream);
			traces += writeTraces(o, &trace_stream);
			lineations += writeLineations(o, &lineation_stream);
		}

		//cleanup
		plane_stream.flush();
		plane_file.close();
		trace_stream.flush();
		trace_file.close();
		lineation_stream.flush();
		lineation_file.close();

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
	}
	else
	{
		m_app->dispToConsole("[ccCompass] Could not open output files... ensure CC has write access to this location.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	}
}

//enter or turn off map mode
void ccCompass::enableMapMode() //turns on/off map mode
{
	//m_app->dispToConsole("ccCompass: Changing to Map mode. Measurements will be associated with GeoObjects.", ccMainAppInterface::STD_CONSOLE_MESSAGE);
	m_dlg->mapMode->setChecked(true);
	m_dlg->compassMode->setChecked(false);
	m_dlg->paintModeButton->setEnabled(true);

	ccCompass::mapMode = true;

	//start gui
	m_app->registerOverlayDialog(m_mapDlg, Qt::Corner::TopLeftCorner);
	m_mapDlg->start();
	m_app->updateOverlayDialogsPlacement();
	m_window->redraw(true, false);
}

//enter or turn off map mode
void ccCompass::enableMeasureMode() //turns on/off map mode
{
	//m_app->dispToConsole("ccCompass: Changing to Compass mode. Measurements will be stored in the \"Measurements\" folder.", ccMainAppInterface::STD_CONSOLE_MESSAGE);
	m_dlg->mapMode->setChecked(false);
	m_dlg->compassMode->setChecked(true);
	ccCompass::mapMode = false;
	m_window->redraw(true, false);
	m_dlg->paintModeButton->setEnabled(false);

	//turn off map mode dialog
	m_mapDlg->stop(true);
	m_app->unregisterOverlayDialog(m_mapDlg);
	m_app->updateOverlayDialogsPlacement();
}

void ccCompass::addGeoObject() //creates a new GeoObject
{
	//calculate default name
	QString name = m_lastGeoObjectName;

	//get name
	name = QInputDialog::getText(m_dlg, "New GeoObject", "GeoObject Name:", QLineEdit::Normal, name);
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

void ccCompass::pickGeoObject() //uses a "picking tool" to select GeoObjects
{
	//todo
}

void ccCompass::clearGeoObject()  //clears the selected GeoObject
{
	//todo
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
	int tID = object->getUniqueID();
	int n = 0;
	if (object->isKindOf(CC_TYPES::POLY_LINE)) //ensure this is a polyline
	{
		ccPolyline* p = static_cast<ccPolyline*>(object);

		//loop through points
		CCVector3 start, end;
		int tID = object->getUniqueID();
		if (p->size() >= 2)
		{
			for (unsigned i = 1; i < p->size(); i++)
			{
				p->getPoint(i - 1, start);
				p->getPoint(i, end);
				//write data
				//n.b. csv columns are name,trace_id,seg_id,start_x,start_y,start_z,end_x,end_y,end_z
				*out << name << ","; //name
				*out << tID << ",";
				*out << i - 1 << ",";
				*out << start.x << ",";
				*out << start.y << ",";
				*out << start.z << ",";
				*out << end.x << ",";
				*out << end.y << ",";
				*out << end.z << endl;
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
int ccCompass::writeLineations(ccHObject* object, QTextStream* out, QString parentName)
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
	if (ccLineation::isLineation(object))
	{
		//Write object as Name,Sx,Sy,Sz,Ex,Ey,Ez,Trend,Plunge
		*out << name << ",";
		*out << object->getMetaData("Sx").toString() << "," << object->getMetaData("Sy").toString() << "," << object->getMetaData("Sz").toString() << ",";
		*out << object->getMetaData("Ex").toString() << "," << object->getMetaData("Ey").toString() << "," << object->getMetaData("Ez").toString() << ",";
		*out << object->getMetaData("Trend").toString() << "," << object->getMetaData("Plunge").toString() << endl;
		n++;
	}

	//write all children
	for (unsigned i = 0; i < object->getChildrenNumber(); i++)
	{
		ccHObject* o = object->getChild(i);
		n += writeLineations(o, out, name);
	}
	return n;
}