//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: ccCompass                   #
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
//#                             COPYRIGHT: XXX                             #
//#                                                                        #
//##########################################################################

#include "ccCompass.h"


//Qt
#include <QtGui>

//potentially useful code snippets

//get main app interface
//ccMainAppInterface* app = ccStdPluginInterface::getMainAppInterface();

//Default constructor: should mainly be used to initialize
//actions (pointers) and other members
ccCompass::ccCompass(QObject* parent/*=0*/)
	: QObject(parent)
	, m_action(0)
{
	//bind GUI events
	m_dlg = new ccCompassDlg();
	//m_dlg->setWindowFlags(Qt::SubWindow);

	ccCompassDlg::connect(m_dlg->closeButton, SIGNAL(clicked()), this, SLOT(onClose()));
	//ccCompassDlg::connect(m_dlg, SIGNAL(closed()), this, SLOT(onClose()));
	ccCompassDlg::connect(m_dlg->acceptButton, SIGNAL(clicked()), this, SLOT(onAccept()));
	ccCompassDlg::connect(m_dlg->saveButton, SIGNAL(clicked()), this, SLOT(onSave()));
	ccCompassDlg::connect(m_dlg->undoButton, SIGNAL(clicked()), this, SLOT(onUndo()));
	ccCompassDlg::connect(m_dlg->lineationModeButton, SIGNAL(clicked()), this, SLOT(setLineationMode()));
	ccCompassDlg::connect(m_dlg->planeModeButton, SIGNAL(clicked()), this, SLOT(setPlaneMode()));
	ccCompassDlg::connect(m_dlg->traceModeButton, SIGNAL(clicked()), this, SLOT(setTraceMode()));
	ccCompassDlg::connect(m_dlg->showNameToggle, SIGNAL(toggled(bool)), this, SLOT(toggleLabels(bool)));
	ccCompassDlg::connect(m_dlg->showStippledToggle, SIGNAL(toggled(bool)), this, SLOT(toggleStipple(bool)));
	ccCompassDlg::connect(m_dlg->showNormalsToggle, SIGNAL(toggled(bool)), this, SLOT(toggleNormals(bool)));
	ccCompassDlg::connect(m_dlg->categoryBox, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(changeType()));
	ccCompassDlg::connect(m_dlg->infoButton, SIGNAL(clicked()), this, SLOT(showHelp()));
}

//deconstructor
ccCompass::~ccCompass()
{
	if (m_mouseCircle)
	{
		delete m_mouseCircle;
	}
	m_mouseCircle = 0;
}

//Generally we ignore new selections... except when trace mode is activated, when the selected object is set as the active trace (if it is a trace)
void ccCompass::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_pickingMode == MODE::TRACE_MODE)
	{
		for (int i = 0; i < selectedEntities.size(); i++)
		{
			if (isTrace(selectedEntities[i]))
			{
				if (m_trace)
					onAccept(); //accept current trace

				//activate newly selected trace
				pickupTrace(selectedEntities[i]);
				//done
				return;
			}
		}
	}
}

void ccCompass::pickupTrace(ccHObject* obj)
{
	//"pick-up" selected trace
	if (m_trace = dynamic_cast<ccTrace*>(obj)) //try casting to ccTrace - if succesfull ccTrace is updated. If not, ccTrace becomes null.
	{
		//change color
		m_trace->setTraceColor(ccColor::yellow);
		m_trace->setWaypointColor(ccColor::green);

		//remove any fit planes
		m_trace->removeAllChildren();
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

	m_app->dispToConsole("[ccCompass] ccCompas plugin initialized succesfully.", ccMainAppInterface::STD_CONSOLE_MESSAGE);

}

//Called by CC when the plugin should be activated - sets up the plugin and then calls startMeasuring()
void ccCompass::doAction()
{
	//m_app should have already been initialized by CC when plugin is loaded!
	//(--> pure internal check)
	assert(m_app);

	/*** HERE STARTS THE ACTION ***/

	//Get handle to ccGLWindow
	m_window = m_app->getActiveGLWindow();

	//check valid window
	if (!m_window)
	{
		m_app->dispToConsole("[ccCompass] Could not find valid 3D window.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//setup picking callback
	//QObject::connect(m_window, SIGNAL(itemPicked(ccHObject*, unsigned, int, int, const CCVector3&)), this, SLOT(pointPicked(ccHObject*, unsigned, int, int, const CCVector3&)));

	//bind gui
	m_dlg->linkWith(m_window);

	//begin measuring
	startMeasuring();
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

	//setup mouse circle
	m_mouseCircle = new ccMouseCircle(m_window);
	m_window->addToOwnDB(m_mouseCircle, true);

	//start GUI
	m_dlg->setParent(m_app->getMainWindow()->parentWidget());
	m_dlg->start();
	m_app->registerOverlayDialog(m_dlg, Qt::TopRightCorner);

	//mode specific GUI setup
	switch (m_pickingMode)
	{
		case MODE::PLANE_MODE:
			setPlaneMode();
			break;
		case MODE::LINEATION_MODE:
			setLineationMode();
			break;
		case MODE::TRACE_MODE:
			setTraceMode();
			break;
	}

	//"pick-up" selected trace
	for (ccHObject* obj : m_app->getSelectedEntities())
	{
		if (obj->isKindOf(CC_TYPES::POLY_LINE))
			pickupTrace(obj);
			/*if (m_trace = dynamic_cast<ccTrace*>(obj)) //try casting to ccTrace
			{
				//change color
				m_trace->setTraceColor(ccColor::yellow);
				m_trace->setWaypointColor(ccColor::green);
				//remove any fit planes
				m_trace->removeAllChildren(); 
			}*/
			if (m_trace)
				break; //bail now we've found it
	}

	//setup listener for mouse events
	m_window->installEventFilter(this);

	//activate "point picking mode"
	m_pickingHub = m_app->pickingHub();
	if (!m_pickingHub)  //no valid picking hub
	{
		m_app->dispToConsole("[ccCompass] Could not retrieve valid picking hub. Measurement aborted.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	m_pickingHub->addListener(this, true, true);

	//refresh and return
	m_window->redraw(true, false);
	return true;
}

//Exits measuring
bool ccCompass::stopMeasuring()
{
	//stop picking
	if (m_pickingHub)
		m_pickingHub->removeListener(this);

	//remove click listener
		m_window->removeEventFilter(this);

	//remove overlay GUI
	delete ccCompass::m_mouseCircle;
	m_mouseCircle = 0;

	m_dlg->stop(true);
	m_app->unregisterOverlayDialog(m_dlg);

	//forget last measurement
	m_trace = 0; //n.b. we don't finalize this one, as this is akin to "cancelling".
	m_lineation = 0;

	//redraw
	m_window->redraw(true, false);

	return true;
}

//Process point picks
void ccCompass::pointPicked(ccHObject* entity, unsigned itemIdx, int x, int y, const CCVector3& P)
{
	if (!entity) //null pick
		return;

	//if we have picked a point cloud
	if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		//get point cloud
		ccPointCloud* cloud = static_cast<ccPointCloud*>(entity); //cast to point cloud

		//expand cloud by removing & re-adding it (hack)
		m_app->removeFromDB(cloud, false);
		m_app->addToDB(cloud, false, true, false, false);

		if (!cloud)
		{
			ccLog::Warning("[Item picking] Shit's fucked (Picked point is not in pickable entities DB?)!");
			return;
		}

		//SETUP DATA STRUCTURE/FIND RELEVANT NODE TO ADD DATA TO
		//add measurements group if necessary
		ccHObject* measurement_group = 0;
		for (int i = 0; i < cloud->getChildrenNumber(); i++) //check if a "measurements" group exists
		{
			if (cloud->getChild(i)->getName() == "measurements")
			{
				measurement_group = cloud->getChild(i);
				break;
			}
		}
		if (!measurement_group)
		{
			measurement_group = new ccHObject("measurements");
			cloud->addChild(measurement_group);
			m_app->addToDB(measurement_group, false, true, false, false);
		}
		//add category group if necessary
		ccHObject* category_group = 0;
		for (int i = 0; i < measurement_group->getChildrenNumber(); i++) //check if a category group exists
		{
			if (measurement_group->getChild(i)->getName() == m_category)
			{
				category_group = measurement_group->getChild(i);
				break;
			}
		}
		if (!category_group)
		{
			category_group = new ccHObject(m_category);
			measurement_group->addChild(category_group);
			m_app->addToDB(category_group, false, true, false, false);
		}


		if (m_pickingMode == MODE::PLANE_MODE) //DIRECT PLANE FITTING MODE
		{
			//get or generate octree
			ccOctree::Shared oct = cloud->getOctree();
			if (!oct)
			{
				oct = cloud->computeOctree(); //if the user clicked "no" when asked to compute the octree then tough....
			}

			//nearest neighbour search
			float r = m_mouseCircle->getRadiusWorld();
			unsigned char level = oct->findBestLevelForAGivenNeighbourhoodSizeExtraction(r);
			CCLib::DgmOctree::NeighboursSet set;
			int n = oct->getPointsInSphericalNeighbourhood(P, PointCoordinateType(r), set, level);
			//Put data in a point cloud class and encapsulate as a "neighbourhood"
			CCLib::DgmOctreeReferenceCloud nCloud(&set, n);
			CCLib::Neighbourhood Z(&nCloud);

			//Fit plane!
			double rms = 0.0; //output for rms
			ccPlane* pPlane = ccPlane::Fit(&nCloud, &rms);
			if (pPlane) //valid fit
			{
				//get plane normal vector
				CCVector3 N(pPlane->getNormal());
				//We always consider the normal with a positive 'Z' by default!
				if (N.z < 0.0)
					N *= -1.0;
				//calculate dip/dip direction
				float strike, dip;
				ccNormalVectors::ConvertNormalToStrikeAndDip(N, strike, dip);
				//QString dipAndDipDirStr = ccNormalVectors::ConvertStrikeAndDipToString(s, d);
				QString dipAndDipDirStr = QString("%1/%2").arg((int) strike, 3, 10, QChar('0')).arg((int) dip, 2, 10, QChar('0'));

				//calculate centroid
				CCVector3 C = *Z.getGravityCenter();
				//store attributes (centroid, strike, dip, RMS) on plane
				QVariantMap* map = new QVariantMap();
				map->insert("Cx", C.x); map->insert("Cy", C.y); map->insert("Cz", C.z);
				map->insert("Nx", N.x); map->insert("Ny", N.y); map->insert("Nz", N.z);
				map->insert("Strike", strike); map->insert("Dip", dip);
				map->insert("RMS", rms);
				map->insert("Radius", m_mouseCircle->getRadiusWorld());
				pPlane->setMetaData(*map, true);

				//make plane to add to display
				pPlane->setVisible(true);
				pPlane->setName(dipAndDipDirStr);
				pPlane->setSelectionBehavior(ccHObject::SELECTION_FIT_BBOX);
				pPlane->showNormals(true);
				pPlane->enableStippling(m_drawStippled);
				pPlane->showNameIn3D(m_drawName);
				pPlane->showNormalVector(m_drawNormals);
				//add plane to scene graph
				category_group->addChild(pPlane);
				pPlane->setDisplay(m_window);
				pPlane->prepareDisplayForRefresh_recursive(); //not sure what this does, but it looks like fun
				//add plane to TOC
				m_app->addToDB(pPlane, false, false, false, false);
				//write answer
				m_app->dispToConsole(QString("[ccCompass] Sampled %d points in radius %f. Fitted plane %s").arg(QString(n), QString("%.3f").arg(r), QString::fromStdString(dipAndDipDirStr.toStdString())), ccMainAppInterface::STD_CONSOLE_MESSAGE);
			} else
			{
				m_app->dispToConsole(QString("[ccCompass] Sampled %d points in radius %.3f. Could not fit a plane.").arg(n, r), ccMainAppInterface::STD_CONSOLE_MESSAGE);
			}
		} else if (m_pickingMode == MODE::TRACE_MODE) //TRACE PICKING MODE
		{
			if (m_trace)
			{
				//check that m_trace hasn't been deleted...
				if (!m_app->dbRootObject()->find(m_trace_id))
				{
					//item has been deleted...
					m_trace = 0;
				}
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
				category_group->addChild(m_trace);
				m_app->addToDB(m_trace, false, false, false, false);
				m_app->dispToConsole(QString("[ccCompass] Added trace waypoint."), ccMainAppInterface::STD_CONSOLE_MESSAGE);
			}

			//update cost function
			ccTrace::COST_MODE = m_dlg->getCostMode();

			//add point
			int index = m_trace->insertWaypoint(itemIdx);
			m_app->dispToConsole(QString("[ccCompass] Added point to active trace."), ccMainAppInterface::STD_CONSOLE_MESSAGE);

			//optimise points
			if (m_trace->waypoint_count() >= 2) {
				m_app->dispToConsole(QString("[ccCompass] Optimising path..."), ccMainAppInterface::STD_CONSOLE_MESSAGE);
				if (m_trace->optimizePath(100000000))
					m_app->dispToConsole(QString("[ccCompass] Success :)"), ccMainAppInterface::STD_CONSOLE_MESSAGE);
				else
					m_app->dispToConsole(QString("[ccCompass] Fail :("), ccMainAppInterface::STD_CONSOLE_MESSAGE);
			}
		}
		else if (m_pickingMode == MODE::LINEATION_MODE)
		{
			if (!m_lineation)
			{
				//no active trace -> make a new one
				m_lineation = new ccLineation(cloud);
				m_lineation->setDisplay(m_window);
				m_lineation->setVisible(true);
				m_lineation->setName("Lineation");
				m_lineation->prepareDisplayForRefresh_recursive();
				category_group->addChild(m_lineation);
				m_app->addToDB(m_lineation, false, false, false, false);
				m_app->dispToConsole(QString("[ccCompass] Measuring new lineation."), ccMainAppInterface::STD_CONSOLE_MESSAGE);
			}

			//add point
			int index = m_lineation->addPointIndex(itemIdx);

			//is this the end point?
			if (m_lineation->size()==2)
			{
				//calculate trace orientation (trend/plunge)
				CCVector3f dir = m_lineation->getDirection(); dir.normalize();
				float trend, plunge;
				//special case: dir is vertical
				if (dir.z > 0.9999999) //vector = 0,0,1
				{
					trend = 0;
					if (dir.z < 0)
						plunge = 90;
					else
						plunge = -90;
				}
				else //normal cases...
				{
					CCVector3f hzComp = CCVector3f(dir.x, dir.y, 0); hzComp.normalize();

					//calculate plunge: plunge = angle between vector & vector projected onto horizontal (x,y) plane
					plunge = std::acos(dir.dot(hzComp)) * (180 / M_PI); //plunge measured from horizontal (in degrees)
					if (dir.z > 0) //lineations pointing towards the sky have negative plunges
						plunge *= -1;

					//calculate trend (N.B. I have very little idea how exactly this code work, it's kinda magic)
					//[c.f. http://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors ]
					CCVector3f N(0, 1, 0); //north vector
					float dot = hzComp.dot(N);
					float det = CCVector3f(0, 0, 1).dot(hzComp.cross(N));
					trend = std::atan2(det, dot) * (180 / M_PI); //heading measured clockwise from north (in degrees)
					if (trend < 0)
						trend += 360;
				}

				//store trend and plunge info
				QVariantMap* map = new QVariantMap();
				CCVector3 s = *m_lineation->getPoint(0);
				CCVector3 e = *m_lineation->getPoint(1);
				map->insert("Sx", s.x); map->insert("Sy", s.y); map->insert("Sz", s.z);
				map->insert("Ex", e.x); map->insert("Ey", e.y); map->insert("Ez", e.z);
				map->insert("Trend", trend); map->insert("Plunge", plunge);
				m_lineation->setMetaData(*map, true);

				//rename lineation
				QString trendAndPlungeStr = QString("%1->%2").arg((int)plunge, 2, 10, QChar('0')).arg((int)trend, 3, 10, QChar('0'));
				m_lineation->setName(trendAndPlungeStr);
				m_lineation->showNameIn3D(m_drawName);

				//start new one
				m_lineation = 0;
			}

		}

		//redraw
		m_window->redraw();
	}
}
//This function is called when a point is picked (through the picking hub)
void ccCompass::onItemPicked(const ccPickingListener::PickedItem& pi)
{
	pointPicked(pi.entity, pi.itemIndex, pi.clickPoint.x(), pi.clickPoint.y(), pi.P3D); //map straight to pointPicked function
}

bool ccCompass::eventFilter(QObject* obj, QEvent* event)
{

	m_dlg->raise(); //keep gui on top
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
	return QIcon(":/CC/plugin/qDummyPlugin/icon.png");
}

//exit this tool
void ccCompass::onClose()
{
	stopMeasuring();
}

//called during trace mode when an active trace is complete
void ccCompass::onAccept()
{
	//finish current trace
	if (m_trace)
	{
		if (m_trace)
		{
			//check that m_trace hasn't been deleted...
			if (!m_app->dbRootObject()->find(m_trace_id))
			{
				//item has been deleted...
				m_trace = 0;
				return;
			}
		}

		m_trace->finalizePath();
		m_trace->setWaypointColor(ccColor::red);
		m_trace->setTraceColor(ccColor::red);

		//fit plane
		if (m_dlg->planeFitMode())
		{
			ccPlane* p = m_trace->fitPlane();
			if (p)
			{
				p ->setVisible(true);
				p->setSelectionBehavior(ccHObject::SELECTION_FIT_BBOX);
				p->showNormals(true);
				p->enableStippling(m_drawStippled);
				p->showNameIn3D(m_drawName);
				p->showNormalVector(m_drawNormals);
				m_trace->addChild(p);
			}
			else
			{
				m_app->dispToConsole(QString("[ccCompass] Not enough topography to fit plane to trace!"), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			}
		}
	}
	m_trace = 0;
	m_window->redraw();
}

//export the selected layer to CSV file
void ccCompass::onSave()
{
	//get output file path
	QString filename = QFileDialog::getSaveFileName(m_dlg, tr("Output file"), "", tr("CSV files (*.csv *.txt)"));
	int planes = 0, traces = 0, lineations = 0; //keept track of how many objects are being written (used to delete empty files)

	//build filenames
	QString plane_fn = filename, trace_fn = filename, lineation_fn = filename;
	plane_fn.insert(plane_fn.lastIndexOf("\."), "_planes");
	trace_fn.insert(trace_fn.lastIndexOf("\."), "_traces");
	lineation_fn.insert(lineation_fn.lastIndexOf("\."), "_lineations");

	QFile file(plane_fn);
	if (file.open(QIODevice::WriteOnly)) {
		//create text stream
		QTextStream stream(&file);

		//write header
		stream << "Name,Strike,Dip,Dip_Dir,Cx,Cy,Cz,Nx,Ny,Nz,Sample_Radius,RMS" << endl;

		//write data (n.b. we use a loop here rather than calling writePlanes(...) on dbRoot to avoid including dbRoot in the object name
		for (int i = 0; i < m_app->dbRootObject()->getChildrenNumber(); i++)
		{
			ccHObject* o = m_app->dbRootObject()->getChild(i);
			planes += writePlanes(o, &stream);
		}

		//cleanup
		stream.flush();
		file.close();

		m_app->dispToConsole("[ccCompass] Successfully exported plane data.", ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}
	else
	{
		m_app->dispToConsole("[ccCompass] Could not write plane data...", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
	}

	if (planes == 0) //delete if nothing written
		file.remove();

	//write trace data
	QFile t_file(trace_fn);
	if (t_file.open(QIODevice::WriteOnly)) {
		//create text stream
		QTextStream stream(&t_file);

		//write header
		stream << "name,trace_id,point_id,start_x,start_y,start_z,end_x,end_y,end_z" << endl;

		//write data
		for (int i = 0; i < m_app->dbRootObject()->getChildrenNumber(); i++)
		{
			ccHObject* o = m_app->dbRootObject()->getChild(i);
			traces += writeTraces(o, &stream);
		}

		//cleanup
		stream.flush();
		t_file.close();

		m_app->dispToConsole("[ccCompass] Successfully exported trace data.", ccMainAppInterface::STD_CONSOLE_MESSAGE);
	} else
	{
		m_app->dispToConsole("[ccCompass] Could not write trace data...", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
	}

	if (traces == 0) //delete if nothing written
		t_file.remove();

	//write lineation data
	QFile l_file(lineation_fn);
	if (l_file.open(QIODevice::WriteOnly)) {
		//create text stream
		QTextStream stream(&l_file);

		//write header
		stream << "name,Sx,Sy,Sz,Ex,Ey,Ez,Trend,Plunge" << endl;

		//write data
		for (int i = 0; i < m_app->dbRootObject()->getChildrenNumber(); i++)
		{
			ccHObject* o = m_app->dbRootObject()->getChild(i);
			lineations += writeLineations(o, &stream);
		}

		//cleanup
		stream.flush();
		l_file.close();

		m_app->dispToConsole("[ccCompass] Successfully exported lineation data.", ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}
	else
	{
		m_app->dispToConsole("[ccCompass] Could not write lineation data...", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
	}

	if (lineations == 0) //delete if nothing written
		l_file.remove();
}
//write plane data
int ccCompass::writePlanes(ccHObject* object, QTextStream* out, QString parentName)
{
	//get object name
	QString name;
	if (parentName.isEmpty()) {
		name = QString("%1").arg(object->getName());
	}
	else
	{
		name = QString("%1.%2").arg(parentName,object->getName());
	}

	//is object a plane made by ccCompass?
	int n = 0;
	if (isFitPlane(object))
	{
		//Write object as Name,Strike,Dip,Dip_Dir,Cx,Cy,Cz,Nx,Ny,Nz,Radius,RMS
		*out << name << ",";
		*out << object->getMetaData("Strike").toString() << "," << object->getMetaData("Dip").toString() << "," << object->getMetaData("Strike").toFloat() + 90;
		*out << object->getMetaData("Cx").toString() << "," << object->getMetaData("Cy").toString() << "," << object->getMetaData("Cz").toString() << ",";
		*out << object->getMetaData("Nx").toString() << "," << object->getMetaData("Ny").toString() << "," << object->getMetaData("Nz").toString() << ",";
		*out << object->getMetaData("Radius").toString() << "," << object->getMetaData("RMS").toString() << endl;
		n++;
	}

	//write all children
	for (int i = 0; i < object->getChildrenNumber(); i++)
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
	if (parentName.isEmpty()) {
		name = QString("%1").arg(object->getName());
	}
	else
	{
		name = QString("%1.%2").arg(parentName, object->getName());
	}

	//is object a polyline
	int tID = object->getUniqueID();
	int n = 0;
	if (isTrace(object)) //ensure this is a ccTrace
	{
		ccPolyline* p = static_cast<ccPolyline*>(object);

		//loop through points
		CCVector3 start, end;
		int tID = object->getUniqueID();
		if (p->size() >= 2)
			for (int i = 1; i < p->size(); i++)
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
		n++;
	}

	//write all children
	for (int i = 0; i < object->getChildrenNumber(); i++)
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
	if (parentName.isEmpty()) {
		name = QString("%1").arg(object->getName());
	}
	else
	{
		name = QString("%1.%2").arg(parentName, object->getName());
	}

	//is object a lineation made by ccCompass?
	int n = 0;
	if (isLineation(object))
	{
		//Write object as Name,Sx,Sy,Sz,Ex,Ey,Ez,Trend,Plunge
		*out << name << ",";
		*out << object->getMetaData("Sx").toString() << "," << object->getMetaData("Sy").toString() << "," << object->getMetaData("Sz").toString() << ",";
		*out << object->getMetaData("Ex").toString() << "," << object->getMetaData("Ey").toString() << "," << object->getMetaData("Ez").toString() << ",";
		*out << object->getMetaData("Trend").toString() << "," << object->getMetaData("Plunge").toString() << endl;
		n++;
	}

	//write all children
	for (int i = 0; i < object->getChildrenNumber(); i++)
	{
		ccHObject* o = object->getChild(i);
		n+=writeLineations(o, out, name);
	}
	return n;
}
//returns true if object was created by ccCompass
bool ccCompass::madeByMe(ccHObject* object)
{
	return isFitPlane(object) | isTrace(object) | isLineation(object);
}
//returns true if object is a fitPlane
bool ccCompass::isFitPlane(ccHObject* object)
{
	if (!object->isKindOf(CC_TYPES::PLANE)) //ensure object is a plane
		return false;
	else									//ensure plane has the correct metadata
		return object->hasMetaData("Cx") && object->hasMetaData("Cy") && object->hasMetaData("Cz") &&
			object->hasMetaData("Nx") && object->hasMetaData("Ny") && object->hasMetaData("Nz") &&
			object->hasMetaData("Strike") && object->hasMetaData("Dip") &&
			object->hasMetaData("RMS") && object->hasMetaData("Radius");
}
//returns true if object is a lineation
bool ccCompass::isLineation(ccHObject* object)
{
	if (!object->isKindOf(CC_TYPES::POLY_LINE)) //lineations are polylines
		return false;
	else										//ensure polyline has correct metadata for lineation
		return object->hasMetaData("Sx") && object->hasMetaData("Sy") && object->hasMetaData("Sz") &&
		object->hasMetaData("Ex") && object->hasMetaData("Ey") && object->hasMetaData("Ez") &&
		object->hasMetaData("Trend") && object->hasMetaData("Plunge");
}
//returns true if object is a trace
bool ccCompass::isTrace(ccHObject* object)
{
	if (!object->isKindOf(CC_TYPES::POLY_LINE)) //lineations are polylines
		return false;
	else										//ensure polyline has correct metadata for trace
		return object->hasMetaData("search_r") && object->hasMetaData("cost_function");
}

//undo last plane
void ccCompass::onUndo()
{
	if (m_pickingMode == MODE::TRACE_MODE)
		if (m_trace) {
			m_trace->undoLast();
			m_trace->optimizePath();
		}
	m_window->redraw();
}

//activate lineation mode
void ccCompass::setLineationMode()
{
	if (m_trace) //cleanup after trace mode if necessary
		onAccept(); //finish last trace
		m_trace = 0;
	m_pickingMode = ccCompass::MODE::LINEATION_MODE;
	m_dlg->lineationModeButton->setChecked(true);
	m_dlg->planeModeButton->setChecked(false);
	m_dlg->traceModeButton->setChecked(false);
	m_dlg->undoButton->setEnabled(false);
	m_dlg->acceptButton->setEnabled(false);
	m_mouseCircle->setVisible(false);
	m_window->redraw(true, false);
	m_dlg->algorithmButton->setEnabled(false);
}
//activate plane mode
void ccCompass::setPlaneMode()
{
	if (m_trace) //cleanup after trace mode if necessary
		onAccept(); //finish last trace
		m_trace = 0;
	m_pickingMode = ccCompass::MODE::PLANE_MODE;
	m_dlg->lineationModeButton->setChecked(false);
	m_dlg->planeModeButton->setChecked(true);
	m_dlg->traceModeButton->setChecked(false);
	m_dlg->undoButton->setEnabled(false);
	m_dlg->acceptButton->setEnabled(false);
	m_mouseCircle->setVisible(true);
	m_window->redraw(true, false);
	m_dlg->algorithmButton->setEnabled(false); //disable. IDEA: Could add functionality here to allow RANSAC or Least Squares plane fitting.
}
//activate trace mode
void ccCompass::setTraceMode()
{
	m_pickingMode = ccCompass::MODE::TRACE_MODE;
	m_dlg->lineationModeButton->setChecked(false);
	m_dlg->planeModeButton->setChecked(false);
	m_dlg->traceModeButton->setChecked(true);
	m_dlg->undoButton->setEnabled(true);
	m_dlg->acceptButton->setEnabled(true);
	m_mouseCircle->setVisible(false);
	m_dlg->algorithmButton->setMenu(m_dlg->m_cost_algorithm_menu); //add algorithm menu
	m_dlg->algorithmButton->setEnabled(true); //add algorithm menu
	m_window->redraw(true, false);
}

//toggle stippling
void ccCompass::toggleStipple(bool checked)
{
	m_drawStippled = checked; //change stippling for newly created planes
	recurseStipple(m_app->dbRootObject(), checked); //change stippling for existing planes
	m_window->redraw(); //redraw
}
void ccCompass::recurseStipple(ccHObject* object,bool checked)
{
	//check this object
	if (isFitPlane(object))
	{
		ccPlane* p = static_cast<ccPlane*>(object);
		p->enableStippling(checked);
	}

	//recurse
	for (int i = 0; i < object->getChildrenNumber(); i++)
	{
		ccHObject* o = object->getChild(i);
		recurseStipple(o, checked);
	}
}

//toggle labels
void ccCompass::toggleLabels(bool checked)
{
	recurseLabels(m_app->dbRootObject(), checked); //change labels for existing planes
	m_drawName = checked; //change labels for newly created planes
	m_window->redraw(); //redraw
}
void ccCompass::recurseLabels(ccHObject* object, bool checked)
{
	//check this object
	if (isFitPlane(object) | isLineation(object))
	{
		object->showNameIn3D(checked);
	}

	//recurse
	for (int i = 0; i < object->getChildrenNumber(); i++)
	{
		ccHObject* o = object->getChild(i);
		recurseLabels(o, checked);
	}
}
//toggle plane normals
void ccCompass::toggleNormals(bool checked)
{
	recurseNormals(m_app->dbRootObject(), checked); //change labels for existing planes
	m_drawNormals = checked; //change labels for newly created planes
	m_window->redraw(); //redraw
}
void ccCompass::recurseNormals(ccHObject* object, bool checked)
{
	//check this object
	if (isFitPlane(object))
	{
		ccPlane* p = static_cast<ccPlane*>(object);
		p->showNormalVector(checked);
	}

	//recurse
	for (int i = 0; i < object->getChildrenNumber(); i++)
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
