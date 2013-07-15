//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE LIGHT VIEWER                            #
//#                                                                        #
//#  This project has been initated under funding from ANR/CIFRE           #
//#  This program is free software; you can redistribute it and/or modify  #
//#                                                                        #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#      +++ COPYRIGHT: EDF R&D + TELECOM ParisTech (ENST-TSI) +++         #
//#                                                                        #
//##########################################################################

#include "ccviewer.h"

//Qt
#include <QVBoxLayout>
#include <QMessageBox>

//qCC
#include <ccGLWindow.h>
#include <fileIO/FileIOFilter.h>
#include <ccGuiParameters.h>

//dialogs
#include <ccDisplayOptionsDlg.h>
#include <ccCameraParamEditDlg.h>

//qCC_db
#include <ccHObjectCaster.h>
#include <ccHObject.h>
#include <ccPointCloud.h>
#include <ccGenericMesh.h>

//3D mouse handler
#ifdef CC_3DXWARE_SUPPORT
#include <devices/3dConnexion/Mouse3DInput.h>
#endif
#include <ccMouse3DContextMenu.h>

//system
#include <assert.h>

//! Current version
const double CC_VIEWER_VERSION = 1.26;

//Camera parameters dialog
ccCameraParamEditDlg* s_cpeDlg = 0;

ccViewer::ccViewer(QWidget *parent, Qt::WindowFlags flags)
	: QMainWindow(parent, flags)
	, m_glWindow(0)
	, m_selectedObject(0)
	, m_3dMouseInput(0)
{
	ui.setupUi(this);

	//insert GL window in a vertical layout
	QVBoxLayout* verticalLayout_2 = new QVBoxLayout(ui.GLframe);
	verticalLayout_2->setSpacing(0);
	const int margin = 10;
    verticalLayout_2->setContentsMargins(margin,margin,margin,margin);
    QGLFormat format;
    format.setSwapInterval(0);
	m_glWindow = new ccGLWindow(ui.GLframe,format);
	verticalLayout_2->addWidget(m_glWindow);

	updateGLFrameGradient();

	m_glWindow->setRectangularPickingAllowed(false); //multiple entities picking not supported

	//UI/display synchronization
	ui.actionFullScreen->setChecked(false);
	ui.menuSelected->setEnabled(false);
	reflectLightsState();
	reflectPerspectiveState();
	reflectPivotVisibilityState();

#ifdef CC_3DXWARE_SUPPORT
	enable3DMouse(true,true);
#else
	ui.actionEnable3DMouse->setEnabled(false);
#endif

	//Signals & slots connection
	connect(m_glWindow,								SIGNAL(filesDropped(const QStringList&)),			this,		SLOT(addToDB(const QStringList&)));
	connect(m_glWindow,								SIGNAL(entitySelectionChanged(int)),				this,		SLOT(selectEntity(int)));
    //connect(m_glWindow,								SIGNAL(entitiesSelectionChanged(std::set<int>)),	this,		SLOT(selectEntities(std::set<int>))); //not supported!
	//connect(m_glWindow,								SIGNAL(newLabel(ccHObject*),					this,		SLOT(handleNewEntity(ccHObject*))); //nothing to do in ccViewer!

	//"Options" menu
	connect(ui.actionDisplayParameters,				SIGNAL(triggered()),						this,	SLOT(showDisplayParameters()));
    connect(ui.actionEditCamera,					SIGNAL(triggered()),    					this,	SLOT(doActionEditCamera()));
    //"Display > Standard views" menu
    connect(ui.actionSetViewTop,					SIGNAL(triggered()),						this,	SLOT(setTopView()));
    connect(ui.actionSetViewBottom,					SIGNAL(triggered()),						this,	SLOT(setBottomView()));
    connect(ui.actionSetViewFront,					SIGNAL(triggered()),						this,	SLOT(setFrontView()));
    connect(ui.actionSetViewBack,					SIGNAL(triggered()),						this,	SLOT(setBackView()));
    connect(ui.actionSetViewLeft,					SIGNAL(triggered()),						this,	SLOT(setLeftView()));
    connect(ui.actionSetViewRight,					SIGNAL(triggered()),						this,	SLOT(setRightView()));
    connect(ui.actionSetViewIso1,					SIGNAL(triggered()),						this,	SLOT(setIsoView1()));
    connect(ui.actionSetViewIso2,					SIGNAL(triggered()),						this,	SLOT(setIsoView2()));

	//"Options > Perspective" menu
	connect(ui.actionSetOrthoView,					SIGNAL(triggered()),						this,	SLOT(setOrthoView()));
	connect(ui.actionSetCenteredPerspectiveView,	SIGNAL(triggered()),						this,	SLOT(setCenteredPerspectiveView()));
	connect(ui.actionSetViewerPerspectiveView,		SIGNAL(triggered()),						this,	SLOT(setViewerPerspectiveView()));
	//"Options > Rotation symbol" menu
	connect(ui.actionSetPivotAlwaysOn,				SIGNAL(triggered()),						this,	SLOT(setPivotAlwaysOn()));
	connect(ui.actionSetPivotRotationOnly,			SIGNAL(triggered()),						this,	SLOT(setPivotRotationOnly()));
	connect(ui.actionSetPivotOff,					SIGNAL(triggered()),						this,	SLOT(setPivotOff()));
	//"Options > 3D mouse" menu
	connect(ui.actionEnable3DMouse,					SIGNAL(toggled(bool)),						this,	SLOT(setup3DMouse(bool)));
    //"Display > Lights & Materials" menu
    connect(ui.actionToggleSunLight,				SIGNAL(toggled(bool)),    					this,	SLOT(toggleSunLight(bool)));
    connect(ui.actionToggleCustomLight,				SIGNAL(toggled(bool)),    					this,	SLOT(toggleCustomLight(bool)));
	//"Options" menu
	connect(ui.actionGlobalZoom,					SIGNAL(triggered()),						this,	SLOT(setGlobalZoom()));
	connect(ui.actionFullScreen,					SIGNAL(toggled(bool)),						this,	SLOT(toggleFullScreen(bool)));

	//"Options > Selected" menu
	connect(ui.actionShowColors,					SIGNAL(toggled(bool)),    					this,	SLOT(toggleColorsShown(bool)));
	connect(ui.actionShowNormals,					SIGNAL(toggled(bool)),    					this,	SLOT(toggleNormalsShown(bool)));
	connect(ui.actionShowScalarField,				SIGNAL(toggled(bool)),    					this,	SLOT(toggleScalarShown(bool)));
	connect(ui.actionShowColorRamp,					SIGNAL(toggled(bool)),    					this,	SLOT(toggleColorbarShown(bool)));
	connect(ui.actionZoomOnSelectedEntity,			SIGNAL(triggered()),						this,	SLOT(zoomOnSelectedEntity()));
    connect(ui.actionDelete,						SIGNAL(triggered()),						this,	SLOT(doActionDeleteSelectedEntity()));

	//"Help" menu
    connect(ui.actionAbout,							SIGNAL(triggered()),						this,	SLOT(doActionAbout()));
    connect(ui.actionHelpShortctus,					SIGNAL(triggered()),						this,	SLOT(doActionDisplayShortcuts()));
}

ccViewer::~ccViewer()
{
	release3DMouse();

	if (s_cpeDlg)
	{
		delete s_cpeDlg;
		s_cpeDlg=0;
	}

	ccHObject* currentRoot = m_glWindow->getSceneDB();
	if (currentRoot)
	{
		m_glWindow->setSceneDB(0);
		m_glWindow->redraw();
		delete currentRoot;
	}
}

void ccViewer::doActionDeleteSelectedEntity()
{
	ccHObject* currentRoot = m_glWindow->getSceneDB();
	if (!currentRoot)
		return;

	ccHObject::Container toCheck;
	toCheck.push_back(currentRoot);

	while (!toCheck.empty())
	{
		ccHObject* obj = toCheck.back();
		toCheck.pop_back();
		
		if (obj->isSelected())
		{
			bool fatherDependant = false;
			if (obj->getParent())
			{
				//Warning: we must ask the object if it is fatehr dependant BEFORE removing it ;)
				fatherDependant = obj->getFlagState(CC_FATHER_DEPENDANT);
				obj->getParent()->removeChild(obj);
			}
			if (!fatherDependant)
				delete obj;
		}
		else
		{
			for (unsigned i=0; i<obj->getChildrenNumber(); ++i)
				toCheck.push_back(obj->getChild(i));
		}
	}

	m_glWindow->redraw();
}

void ccViewer::selectEntity(int uniqueID)
{
	ccHObject* currentRoot = m_glWindow->getSceneDB();
	if (!currentRoot)
		return;

	currentRoot->setSelected_recursive(false);
	ui.menuSelectSF->clear();
	ui.menuSelected->setEnabled(false);

	ccHObject* toSelect = currentRoot->find(uniqueID);
	if (toSelect)
	{
		toSelect->setSelected(true);

		ui.actionShowColors->blockSignals(true);
		ui.actionShowNormals->blockSignals(true);
		ui.actionShowScalarField->blockSignals(true);
		ui.actionShowColorRamp->blockSignals(true);

		ui.actionShowColors->setEnabled(toSelect->hasColors());
		ui.actionShowColors->setChecked(toSelect->colorsShown());
		ui.actionShowNormals->setEnabled(toSelect->hasNormals());
		ui.actionShowNormals->setChecked(toSelect->normalsShown());

		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(toSelect);
		bool hasSF = (cloud ? cloud->hasScalarFields() : false);
		ui.actionShowScalarField->setEnabled(hasSF);
		ui.actionShowScalarField->setChecked(toSelect->sfShown());
		ui.actionShowColorRamp->setEnabled(hasSF);
		ui.actionShowColorRamp->setChecked(cloud ? cloud->sfColorScaleShown() && cloud->sfShown() : false);

		unsigned sfCount = (cloud ? cloud->getNumberOfScalarFields() : 0);
		ui.menuSelectSF->setEnabled(hasSF && sfCount>1);
		if (hasSF && sfCount>1)
		{
			int currentSFIndex = cloud->getCurrentDisplayedScalarFieldIndex();
			//ui.menuSelectSF->clear();
			for (unsigned i=0;i<sfCount;++i)
			{
				QAction* action = ui.menuSelectSF->addAction(cloud->getScalarFieldName(i));
				action->setData(i);
				action->setCheckable(true);
				if (currentSFIndex == (int)i)
					action->setChecked(true);
				connect(action, SIGNAL(toggled(bool)), this, SLOT(changeCurrentScalarField(bool)));
			}
		}

		ui.menuSelected->setEnabled(true);

		ui.actionShowColors->blockSignals(false);
		ui.actionShowNormals->blockSignals(false);
		ui.actionShowScalarField->blockSignals(false);
		ui.actionShowColorRamp->blockSignals(false);

		m_selectedObject = toSelect;
	}

	m_glWindow->redraw();
}

//not supported!
//void ccViewer::selectEntities(std::set<int> entIDs)
//{
//	ccHObject* currentRoot = m_glWindow->getSceneDB();
//	if (!currentRoot)
//		return;
//
//	//convert input list of IDs to proper entities
//	ccHObject::Container entities;
//	size_t labelCount = 0;
//	{
//		try
//		{
//			entities.reserve(entIDs.size());
//		}
//		catch(std::bad_alloc)
//		{
//			ccLog::Error("[ccViewer::selectEntities] Not enough memory!");
//			return;
//		}
//
//		for (std::set<int>::const_iterator it = entIDs.begin(); it != entIDs.end(); ++it)
//		{
//			ccHObject* obj = currentRoot->find(*it);
//			if (obj)
//			{
//				entities.push_back(obj);
//				if (obj->isA(CC_2D_LABEL))
//					++labelCount;
//			}
//		}
//	}
//
//}

bool ccViewer::checkForLoadedEntities()
{
    bool loadedEntities = true;
	m_glWindow->displayNewMessage(QString(),ccGLWindow::SCREEN_CENTER_MESSAGE); //clear (any) message in the middle area

	if (!m_glWindow->getSceneDB())
    {
		m_glWindow->displayNewMessage("Drag & drop files on the 3D window to load them!",ccGLWindow::SCREEN_CENTER_MESSAGE,true,3600);
        loadedEntities = false;
    }

    if (ccGui::Parameters().displayCross != loadedEntities)
    {
        ccGui::ParamStruct params = ccGui::Parameters();
        params.displayCross = loadedEntities;
        ccGui::Set(params);
    }

    return loadedEntities;
}

void ccViewer::updateDisplay()
{
	updateGLFrameGradient();

	m_glWindow->redraw();
}

void ccViewer::updateGLFrameGradient()
{
	//display parameters
    const unsigned char* bkgCol = ccGui::Parameters().backgroundCol;
    const unsigned char* forCol = ccGui::Parameters().pointsDefaultCol;

    glColor3ubv(bkgCol);
    glColor3ub(255-forCol[0],255-forCol[1],255-forCol[2]);

	QString styleSheet = QString("QFrame{border: 2px solid white; border-radius: 10px; background: qlineargradient(x1:0, y1:0, x2:0, y2:1,stop:0 rgb(%1,%2,%3), stop:1 rgb(%4,%5,%6));}").arg(bkgCol[0]).arg(bkgCol[1]).arg(bkgCol[2]).arg(255-forCol[0]).arg(255-forCol[1]).arg(255-forCol[2]);
	ui.GLframe->setStyleSheet(styleSheet);
}

void ccViewer::addToDB(const QStringList& filenames)
{
	ccHObject* currentRoot = m_glWindow->getSceneDB();
	if (currentRoot)
	{
		m_selectedObject = 0;
		m_glWindow->setSceneDB(0);
		m_glWindow->redraw();
		delete currentRoot;
		currentRoot=0;
	}

	bool scaleAlreadyDisplayed=false;

	for (int i=0;i<filenames.size();++i)
	{
		ccHObject* newEntities = FileIOFilter::LoadFromFile(filenames[i],UNKNOWN_FILE,false);

		if (newEntities)
		{
			addToDB(newEntities);

			if (!scaleAlreadyDisplayed)
			{
				for (unsigned i=0;i<newEntities->getChildrenNumber();++i)
				{
					ccHObject* ent = newEntities->getChild(i);
					if (ent->isA(CC_POINT_CLOUD))
					{
						ccPointCloud* pc = static_cast<ccPointCloud*>(ent);
						if (pc->hasScalarFields())
						{
							pc->setCurrentDisplayedScalarField(0);
							pc->showSFColorsScale(true);
							scaleAlreadyDisplayed=true;
						}
					}
					else if (ent->isKindOf(CC_MESH))
					{
						ccGenericMesh* mesh = static_cast<ccGenericMesh*>(ent);
						if (mesh->hasScalarFields())
						{
							mesh->showSF(true);
							scaleAlreadyDisplayed=true;
							ccPointCloud* pc = static_cast<ccPointCloud*>(mesh->getAssociatedCloud());
							pc->showSFColorsScale(true);
						}
					}
				}
			}
		}
	}

	checkForLoadedEntities();
}

void ccViewer::addToDB(ccHObject* entity)
{
	assert(entity && m_glWindow);

	entity->setDisplay_recursive(m_glWindow);

	ccHObject* currentRoot = m_glWindow->getSceneDB();
	if (currentRoot)
	{
		//already a pure 'root'
		if (currentRoot->isA(CC_HIERARCHY_OBJECT))
		{
			currentRoot->addChild(entity);
		}
		else
		{
			ccHObject* root = new ccHObject("root");
			root->addChild(currentRoot);
			root->addChild(entity);
			m_glWindow->setSceneDB(root);
		}
	}
	else
	{
		m_glWindow->setSceneDB(entity);
	}

	checkForLoadedEntities();
}

void ccViewer::showDisplayParameters()
{
	ccDisplayOptionsDlg clmDlg(this);

	connect(&clmDlg, SIGNAL(aspectHasChanged()), this, SLOT(updateDisplay()));

	clmDlg.exec();

	disconnect(&clmDlg,0,0,0);
}

void ccViewer::doActionEditCamera()
{
	if (!s_cpeDlg)
	{
		s_cpeDlg = new ccCameraParamEditDlg(0);
		s_cpeDlg->linkWith(m_glWindow);
	}
	s_cpeDlg->show();
}

void ccViewer::reflectPerspectiveState()
{
	bool objectCentered;
	bool perspectiveEnabled = m_glWindow->getPerspectiveState(objectCentered);
	
	ui.actionSetOrthoView->setChecked(!perspectiveEnabled);
	ui.actionSetCenteredPerspectiveView->setChecked(perspectiveEnabled && objectCentered);
	ui.actionSetViewerPerspectiveView->setChecked(perspectiveEnabled && !objectCentered);
}

void ccViewer::setOrthoView()
{
    if (m_glWindow)
    {
        m_glWindow->setPerspectiveState(false,true);
        m_glWindow->redraw();
    }
	reflectPerspectiveState();
}

void ccViewer::setCenteredPerspectiveView()
{
    if (m_glWindow)
    {
        m_glWindow->setPerspectiveState(true,true);
        m_glWindow->redraw();
    }
	reflectPerspectiveState();
}

void ccViewer::setViewerPerspectiveView()
{
    if (m_glWindow)
    {
        m_glWindow->setPerspectiveState(true,false);
        m_glWindow->redraw();
    }
	reflectPerspectiveState();
}

void ccViewer::reflectPivotVisibilityState()
{
	ccGLWindow::PivotVisibility vis = m_glWindow->getPivotVisibility();

	ui.actionSetPivotAlwaysOn->setChecked(vis == ccGLWindow::PIVOT_ALWAYS_SHOW);
	ui.actionSetPivotRotationOnly->setChecked(vis == ccGLWindow::PIVOT_SHOW_ON_MOVE);
	ui.actionSetPivotOff->setChecked(vis == ccGLWindow::PIVOT_HIDE);
}

void ccViewer::setPivotAlwaysOn()
{
    if (m_glWindow)
    {
		m_glWindow->setPivotVisibility(ccGLWindow::PIVOT_ALWAYS_SHOW);
        m_glWindow->redraw();
    }
	reflectPivotVisibilityState();
}

void ccViewer::setPivotRotationOnly()
{
    if (m_glWindow)
    {
		m_glWindow->setPivotVisibility(ccGLWindow::PIVOT_SHOW_ON_MOVE);
        m_glWindow->redraw();
    }
	reflectPivotVisibilityState();
}

void ccViewer::setPivotOff()
{
    if (m_glWindow)
    {
		m_glWindow->setPivotVisibility(ccGLWindow::PIVOT_HIDE);
        m_glWindow->redraw();
    }
	reflectPivotVisibilityState();
}

void ccViewer::reflectLightsState()
{
	ui.actionToggleSunLight->blockSignals(true);
	ui.actionToggleCustomLight->blockSignals(true);

	ui.actionToggleSunLight->setChecked(m_glWindow->sunLightEnabled());
	ui.actionToggleCustomLight->setChecked(m_glWindow->customLightEnabled());

	ui.actionToggleSunLight->blockSignals(false);
	ui.actionToggleCustomLight->blockSignals(false);
}

void ccViewer::toggleSunLight(bool state)
{
    if (m_glWindow)
		m_glWindow->setSunLight(state);
	reflectLightsState();
}

void ccViewer::toggleCustomLight(bool state)
{
    if (m_glWindow)
		m_glWindow->setCustomLight(state);
	reflectLightsState();
}

void ccViewer::toggleFullScreen(bool state)
{
    if (state)
        showFullScreen();
    else
        showNormal();

	if (m_glWindow)
		m_glWindow->redraw();
}

void ccViewer::doActionDisplayShortcuts()
{
	QMessageBox msgBox;
	QString text;
	text += "Shortcuts:\n\n";
	text += "F2 : Set orthographic view\n";
	text += "F3 : Set object-centered perspective\n";
	text += "F4 : Set viewer-based perspective\n";
	text += "F6 : Toggle sun light\n";
	text += "F7 : Toggle custom light\n";
	text += "F11: Toggle full screen\n";
	text += "Z: Zoom on selected entity\n";
	text += "DEL: Delete selected entity\n";
	text += "\n";
	text += "Ctrl+D: Display parameters\n";
	text += "Ctrl+C: Camera parameters\n";
	text += "\n";
	text += "Left click: Select entity\n";
	//text += "Ctrl + left click: Select multiple entities (toggle)\n";
	//text += "Alt + left button hold: Select multiple entities (rectangular area)\n";
	text += "Shift + left click (on a point): create and display label\n";
	text += "Right click (on a label): expand/collapse\n";
	msgBox.setText(text);
	msgBox.exec();
}

void ccViewer::setTopView()
{
	m_glWindow->setView(CC_TOP_VIEW);
}

void ccViewer::setBottomView()
{
    m_glWindow->setView(CC_BOTTOM_VIEW);
}

void ccViewer::setFrontView()
{
    m_glWindow->setView(CC_FRONT_VIEW);
}

void ccViewer::setBackView()
{
    m_glWindow->setView(CC_BACK_VIEW);
}

void ccViewer::setLeftView()
{
    m_glWindow->setView(CC_LEFT_VIEW);
}

void ccViewer::setRightView()
{
    m_glWindow->setView(CC_RIGHT_VIEW);
}

void ccViewer::setIsoView1()
{
    m_glWindow->setView(CC_ISO_VIEW_1);
}

void ccViewer::setIsoView2()
{
    m_glWindow->setView(CC_ISO_VIEW_2);
}

void ccViewer::toggleColorsShown(bool state)
{
	if (!m_selectedObject)
		return;

	m_selectedObject->showColors(state);
	m_glWindow->redraw();
}

void ccViewer::toggleNormalsShown(bool state)
{
	if (!m_selectedObject)
		return;

	m_selectedObject->showNormals(state);
	m_glWindow->redraw();
}

void ccViewer::toggleScalarShown(bool state)
{
	if (!m_selectedObject)
		return;

	m_selectedObject->showSF(state);
	m_glWindow->redraw();
}

void ccViewer::toggleColorbarShown(bool state)
{
	if (!m_selectedObject)
		return;

	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_selectedObject);
	if (!cloud)
		return;
	cloud->showSFColorsScale(state);
	m_glWindow->updateGL();
}

void ccViewer::changeCurrentScalarField(bool state)
{
	if (!m_selectedObject)
		return;

	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_selectedObject);
	if (!cloud)
		return;

	QAction* action = qobject_cast<QAction*>(QObject::sender());
	if (!action)
		return;

	//disable all other actions
	const QObjectList& children = ui.menuSelectSF->children();
	for (int i=0;i<children.size();++i)
	{
		QAction* act = static_cast<QAction*>(children[i]);
		act->blockSignals(true);
		act->setChecked(act == action);
		act->blockSignals(false);
	}

	int sfIndex = action->data().toInt();
	if (sfIndex < (int)cloud->getNumberOfScalarFields())
	{
		cloud->setCurrentDisplayedScalarField(sfIndex);
		//when 'setCurrentDisplayedScalarField' is called, scalar field is automatically shown!
		ui.actionShowScalarField->blockSignals(true);
		ui.actionShowScalarField->setChecked(true);
		ui.actionShowScalarField->blockSignals(false);
		m_glWindow->redraw();
	}
}

void ccViewer::setGlobalZoom()
{
    if (m_glWindow)
        m_glWindow->zoomGlobal();
}

void ccViewer::zoomOnSelectedEntity()
{
    if (!m_glWindow || !m_selectedObject)
        return;

	ccBBox box = m_selectedObject->getBB(false, false, m_glWindow);
    m_glWindow->updateConstellationCenterAndZoom(&box);
	m_glWindow->redraw();
}

#include <ui_ccviewerAbout.h>

void ccViewer::doActionAbout()
{
	QDialog aboutDialog(this);

	Ui::AboutDialog ui;
	ui.setupUi(&aboutDialog);
	ui.textEdit->setHtml(ui.textEdit->toHtml().arg(CC_VIEWER_VERSION,0,'f',2));

	aboutDialog.exec();
}

/*** 3D MOUSE SUPPORT ***/

void ccViewer::release3DMouse()
{
#ifdef CC_3DXWARE_SUPPORT
	if (m_3dMouseInput)
	{
		disconnect(m_3dMouseInput);
		delete m_3dMouseInput;
		m_3dMouseInput=0;
	}
#endif
}

void ccViewer::setup3DMouse(bool state)
{
	enable3DMouse(state,false);
}

void ccViewer::enable3DMouse(bool state, bool silent)
{
#ifdef CC_3DXWARE_SUPPORT
	if (m_3dMouseInput)
		release3DMouse();

	if (state)
	{
		if (Mouse3DInput::DeviceAvailable())
		{
			ccLog::Warning("[3D Mouse] Device has been detected!");
			m_3dMouseInput = new Mouse3DInput(this);
			QObject::connect(m_3dMouseInput, SIGNAL(sigMove3d(std::vector<float>&)), this, SLOT(on3DMouseMove(std::vector<float>&)));
			QObject::connect(m_3dMouseInput, SIGNAL(sigOn3dmouseKeyDown(int)), this, SLOT(on3DMouseKeyDown(int)));
			QObject::connect(m_3dMouseInput, SIGNAL(sigOn3dmouseKeyUp(int)), this, SLOT(on3DMouseKeyUp(int)));
		}
		else
		{
			if (silent)
				ccLog::Print("[3D Mouse] No device found");
			else
				ccLog::Error("[3D Mouse] No device found");
			state = false;
		}
	}
	else
	{
		ccLog::Warning("[3D Mouse] Device has been disabled");
	}
#else
	state = false;
#endif

	ui.actionEnable3DMouse->blockSignals(true);
	ui.actionEnable3DMouse->setChecked(state);
	ui.actionEnable3DMouse->blockSignals(false);
}

void ccViewer::on3DMouseKeyUp(int)
{
	//nothing right now
}

#ifdef CC_3DXWARE_SUPPORT
static bool s_3dMouseContextMenuAlreadyShown = false; //DGM: to prevent multiple instances at once
#endif
// ANY CHANGE/BUG FIX SHOULD BE REFLECTED TO THE EQUIVALENT METHODS IN QCC "MainWindow.cpp" FILE!
void ccViewer::on3DMouseKeyDown(int key)
{
	if (!m_3dMouseInput)
		return;

#ifdef CC_3DXWARE_SUPPORT

	switch(key)
	{
	case Mouse3DInput::V3DK_MENU:
		if (!s_3dMouseContextMenuAlreadyShown)
		{
				s_3dMouseContextMenuAlreadyShown = true;

				//is there a currently active window?
				if (m_glWindow)
				{
					ccMouse3DContextMenu(&m_3dMouseInput->getMouseParams(),m_glWindow,this).exec(QCursor::pos());
					//in case of...
					reflectPivotVisibilityState();
				}
				else
				{
					ccLog::Error("No active 3D view?!");
					return;
				}

				s_3dMouseContextMenuAlreadyShown = false;
		}
		break;
	case Mouse3DInput::V3DK_FIT:
		{
			if (m_selectedObject)
				zoomOnSelectedEntity();
			else
				setGlobalZoom();
		}
		break;
	case Mouse3DInput::V3DK_TOP:
		setTopView();
		break;
	case Mouse3DInput::V3DK_LEFT:
		setLeftView();
		break;
	case Mouse3DInput::V3DK_RIGHT:
		setRightView();
		break;
	case Mouse3DInput::V3DK_FRONT:
		setFrontView();
		break;
	case Mouse3DInput::V3DK_BOTTOM:
		setBottomView();
		break;
	case Mouse3DInput::V3DK_BACK:
		setBackView();
		break;
	case Mouse3DInput::V3DK_ROTATE:
		m_3dMouseInput->getMouseParams().enableRotation(!m_3dMouseInput->getMouseParams().rotationEnabled());
		break;
	case Mouse3DInput::V3DK_PANZOOM:
		m_3dMouseInput->getMouseParams().enablePanZoom(!m_3dMouseInput->getMouseParams().panZoomEnabled());
		break;
	case Mouse3DInput::V3DK_ISO1:
		setIsoView1();
		break;
	case Mouse3DInput::V3DK_ISO2:
		setIsoView2();
		break;
	case Mouse3DInput::V3DK_PLUS:
		m_3dMouseInput->getMouseParams().accelerate();
		break;
	case Mouse3DInput::V3DK_MINUS:
		m_3dMouseInput->getMouseParams().slowDown();
		break;
	case Mouse3DInput::V3DK_DOMINANT:
		m_3dMouseInput->getMouseParams().toggleDominantMode();
		break;
	case Mouse3DInput::V3DK_CW:
	case Mouse3DInput::V3DK_CCW:
		{
			if (m_glWindow)
			{
				CCVector3 axis(0.0f,0.0f,-1.0f);
				CCVector3 trans(0.0f);
				ccGLMatrix mat;
				float angle = (float)(M_PI/2.0);
				if (key == Mouse3DInput::V3DK_CCW)
					angle = -angle;
				mat.initFromParameters(angle,axis,trans);
				m_glWindow->rotateBaseViewMat(mat);
				m_glWindow->redraw();
			}
		}
		break;
	case Mouse3DInput::V3DK_ESC:
	case Mouse3DInput::V3DK_ALT:
	case Mouse3DInput::V3DK_SHIFT:
	case Mouse3DInput::V3DK_CTRL:
	default:
		ccLog::Warning("[3D mouse] This button is not handled (yet)");
		//TODO
		break;
	}

#endif
}

// ANY CHANGE/BUG FIX SHOULD BE REFLECTED TO THE EQUIVALENT METHODS IN QCC "mainwindow.cpp" FILE!
void ccViewer::on3DMouseMove(std::vector<float>& vec)
{
	//ccLog::PrintDebug(QString("[3D mouse] %1 %2 %3 %4 %5 %6").arg(vec[0]).arg(vec[1]).arg(vec[2]).arg(vec[3]).arg(vec[4]).arg(vec[5]));

#ifdef CC_3DXWARE_SUPPORT

	//no active device?
	if (!m_3dMouseInput)
		return;

    ccGLWindow* win = m_glWindow; //to keep the same code as in qCC's "mainwindow.cpp" file!
	//no active window?
	if (!win)
		return;

	//mouse parameters
	const Mouse3DParameters& params = m_3dMouseInput->getMouseParams();
	bool panZoom = params.panZoomEnabled();
	bool rotate = params.rotationEnabled();
	if (!panZoom && !rotate)
		return;

	//view parameters
	bool objectMode = true;
	bool perspectiveView = win->getPerspectiveState(objectMode);

	//Viewer based perspective IS 'camera mode'
	{
		//Mouse3DParameters::NavigationMode navigationMode = objectMode ? Mouse3DParameters::ObjectMode : Mouse3DParameters::CameraMode;
		//if (params.navigationMode() != navigationMode)
		//{
		//	m_3dMouseInput->getMouseParams().setNavigationMode(navigationMode);
		//	ccLog::Warning("[3D mouse] Navigation mode has been changed to fit current viewing mode");
		//}
	}

	//dominant mode: dominant mode is intended to limit movement to a single direction
	if (params.dominantModeEnabled())
	{
		unsigned dominantDim = 0;
		for (unsigned i=1; i<6; ++i)
			if (fabs(vec[i]) > fabs(vec[dominantDim]))
				dominantDim = i;
		for (unsigned i=0; i<6; ++i)
			if (i != dominantDim)
				vec[i] = 0.0;
	}
	if (panZoom)
	{
		//Zoom: object moves closer/away (only for ortho. mode)
		if (!perspectiveView && fabs(vec[1])>ZERO_TOLERANCE)
		{
			win->updateZoom(1.0f + vec[1]);
			vec[1] = 0.0f;
		}
		
		//Zoom & Panning: camera moves right/left + up/down + backward/forward (only for perspective mode)
		if (fabs(vec[0])>ZERO_TOLERANCE || fabs(vec[1])>ZERO_TOLERANCE || fabs(vec[2])>ZERO_TOLERANCE)
		{
			const ccViewportParameters& viewParams = win->getViewportParameters();

			float scale = (float)std::min(win->width(),win->height()) * viewParams.pixelSize;
			if (perspectiveView)
			{
				float tanFOV = tan(viewParams.fov*CC_DEG_TO_RAD/**0.5f*/);
				vec[0] *= tanFOV;
				vec[2] *= tanFOV;
				scale /= win->computePerspectiveZoom();
			}
			else
			{
				scale /= win->getViewportParameters().zoom;
			}
			
			if (objectMode)
				scale = -scale;
			win->moveCamera(vec[0]*scale,-vec[2]*scale,vec[1]*scale);
		}
	}

	if (rotate)
	{
		if (fabs(vec[3])>ZERO_TOLERANCE
			|| fabs(vec[4])>ZERO_TOLERANCE
			|| fabs(vec[5])>ZERO_TOLERANCE)
		{
			//get corresponding quaternion
			float q[4];
			Mouse3DInput::GetQuaternion(vec,q);
			ccGLMatrix rotMat = ccGLMatrix::FromQuaternion(q);

			//horizon locked?
			if (params.horizonLocked())
			{
				rotMat = rotMat.yRotation();
			}

			win->rotateBaseViewMat(objectMode ? rotMat : rotMat.inverse());
			win->showPivotSymbol(true);
		}
		else
		{
			win->showPivotSymbol(false);
		}
	}

	win->redraw();

#endif
}