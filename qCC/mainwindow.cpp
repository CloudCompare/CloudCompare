//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "mainwindow.h"
#include <iostream>
//CCLib Includes
#include <GenericChunkedArray.h>
#include <CloudSamplingTools.h>
#include <MeshSamplingTools.h>
#include <ScalarFieldTools.h>
#include <StatisticalTestingTools.h>
#include <WeibullDistribution.h>
#include <NormalDistribution.h>
#include <GenericIndexedCloud.h>
#include <Neighbourhood.h>
#include <AutoSegmentationTools.h>
#include <DistanceComputationTools.h>
#include <PointProjectionTools.h>
#include <GeometricalAnalysisTools.h>
#include <SimpleCloud.h>
#include <RegistrationTools.h>  //Aurelien BEY
#include <CloudSamplingTools.h> //Aurelien BEY

//qCC_db
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccMeshGroup.h>
#include <ccOctree.h>
#include <ccKdTree.h>
#include <ccGBLSensor.h>
#include <ccNormalVectors.h>
#include <ccProgressDialog.h>
#include <ccPlane.h>
#include <ccImage.h>
#include <cc2DLabel.h>
#include <cc2DViewportObject.h>
#include <ccColorScale.h>
#include <ccColorScalesManager.h>

//qCC includes
#include "ccHeightGridGeneration.h"
#include "ccRenderingTools.h"
#include "ccFastMarchingForNormsDirection.h"
#include "ccCommon.h"

//sub-windows
#include "ccGLWindow.h"
#include "ccConsole.h"
#include "ccHistogramWindow.h"

//plugins handling
#include "plugins/ccPluginInterface.h"
#include "plugins/ccStdPluginInterface.h"
#include "plugins/ccGLFilterPluginInterface.h"
#include "ccPluginDlg.h"

//shaders & Filters
#include <ccShader.h>
#include <ccGlFilter.h>

//dialogs
#include "ccDisplayOptionsDlg.h"
#include "ccGraphicalSegmentationTool.h"
#include "ccGraphicalTransformationTool.h"
#include "ccClippingBoxTool.h"
#include "ccOrderChoiceDlg.h"
#include "ccComparisonDlg.h"
#include "ccTwoColorsDlg.h"
#include "ccAskOneIntValueDlg.h"
#include "ccAskOneStringDlg.h"
#include "ccAskOneDoubleValueDlg.h"
#include "ccAskTwoDoubleValuesDlg.h"
#include "ccAskThreeDoubleValuesDlg.h"
#include "ccPtsSamplingDlg.h"
#include "ccPickOneElementDlg.h"
#include "ccStatisticalTestDlg.h"
#include "ccLabelingDlg.h"
#include "ccSensorProjectionDlg.h"
#include "ccHeightGridGenerationDlg.h"
#include "ccUnrollDlg.h"
#include "ccAlignDlg.h" //Aurelien BEY
#include "ccRegistrationDlg.h" //Aurelien BEY
#include "ccSubsamplingDlg.h" //Aurelien BEY
#include "ccRenderToFileDlg.h"
#include "ccPointPropertiesDlg.h" //Aurelien BEY
#include "ccPointListPickingDlg.h"
#include "ccNormalComputationDlg.h"
#include "ccCameraParamEditDlg.h"
#include "ccScalarFieldArithmeticDlg.h"
#include "ccScalarFieldFromColorDlg.h"
#include "ccSensorComputeDistancesDlg.h"
#include "ccSensorComputeScatteringAnglesDlg.h"
#include "ccCurvatureDlg.h"
#include "ccApplyTransformationDlg.h"
#include "ccCoordinatesShiftManager.h"
#include "ccPointPairRegistrationDlg.h"
#include "ccExportCoordToSFDlg.h"
#include "ccPrimitiveFactoryDlg.h"
#include "ccMouse3DContextMenu.h"
#include "ccColorScaleEditorDlg.h"
#include "ccComputeOctreeDlg.h"
#include <ui_aboutDlg.h>

//3D mouse handler
#ifdef CC_3DXWARE_SUPPORT
#include "devices/3dConnexion/Mouse3DInput.h"
#endif

//Qt Includes
#include <QtGui>
#include <QMdiArea>
#include <QSignalMapper>
#include <QMdiSubWindow>
#include <QLCDNumber>
#include <QFileDialog>
#include <QActionGroup>
#include <QProcess>
#include <QSettings>
#include <QMessageBox>
#include <QElapsedTimer>
#include <QInputDialog>

//System
#include <string.h>
#include <math.h>
#include <assert.h>
#include <cfloat>

//global static pointer (as there should only be one instance of MainWindow!)
static MainWindow* s_instance = 0;

//standard message in case of locked vertices
void DisplayLockedVerticesWarning()
{
	ccConsole::Error("Mesh vertices are 'locked' (they may be shared by mutliple meshes for instance).\nYou should call this method directly on the vertices cloud (but all meshes will be impacted!).");
}

MainWindow::MainWindow()
	: m_ccRoot(0)
	, m_uiFrozen(false)
	, m_3dMouseInput(0)
	, m_viewModePopupButton(0)
	, m_pivotVisibilityPopupButton(0)
    , m_cpeDlg(0)
    , m_gsTool(0)
    , m_transTool(0)
	, m_clipTool(0)
    , m_compDlg(0)
    , m_ppDlg(0)
    , m_plpDlg(0)
	, m_pprDlg(0)
	, m_pfDlg(0)
	, m_glFilterActions(this)
{
    //Dialog "auto-construction"
    setupUi(this);

	//advanced widgets not handled by QDesigner
	{
		//view mode pop-up menu
		{
			QMenu* menu = new QMenu();
			menu->addAction(actionSetOrthoView);
			menu->addAction(actionSetCenteredPerspectiveView);
			menu->addAction(actionSetViewerPerspectiveView);

			m_viewModePopupButton = new QToolButton();
			m_viewModePopupButton->setMenu(menu);
			m_viewModePopupButton->setPopupMode(QToolButton::InstantPopup);
			m_viewModePopupButton->setToolTip("Set current view mode");
			m_viewModePopupButton->setStatusTip(m_viewModePopupButton->toolTip());
			toolBarView->insertWidget(actionZoomAndCenter,m_viewModePopupButton);
			m_viewModePopupButton->setEnabled(false);
		}

		//pivot center pop-up menu
		{
			QMenu* menu = new QMenu();
			menu->addAction(actionSetPivotAlwaysOn);
			menu->addAction(actionSetPivotRotationOnly);
			menu->addAction(actionSetPivotOff);

			m_pivotVisibilityPopupButton = new QToolButton();
			m_pivotVisibilityPopupButton->setMenu(menu);
			m_pivotVisibilityPopupButton->setPopupMode(QToolButton::InstantPopup);
			m_pivotVisibilityPopupButton->setToolTip("Set pivot visibility");
			m_pivotVisibilityPopupButton->setStatusTip(m_pivotVisibilityPopupButton->toolTip());
			toolBarView->insertWidget(actionZoomAndCenter,m_pivotVisibilityPopupButton);
			m_pivotVisibilityPopupButton->setEnabled(false);
		}
	}

    //tabifyDockWidget(DockableDBTree,DockableProperties);

    //Console
    ccConsole::Init(consoleWidget,this);

    //db-tree link
    m_ccRoot = new ccDBRoot(dbTreeView, propertiesTreeView, this);
    connect(m_ccRoot, SIGNAL(selectionChanged()), this, SLOT(updateUIWithSelection()));

    //MDI Area
    m_mdiArea = new QMdiArea(this);
    setCentralWidget(m_mdiArea);
    connect(m_mdiArea, SIGNAL(subWindowActivated(QMdiSubWindow*)), this, SLOT(updateMenus()));
    connect(m_mdiArea, SIGNAL(subWindowActivated(QMdiSubWindow*)), this, SLOT(on3DViewActivated(QMdiSubWindow*)));

    //Window Mapper
    m_windowMapper = new QSignalMapper(this);
    connect(m_windowMapper, SIGNAL(mapped(QWidget*)), this, SLOT(setActiveSubWindow(QWidget*)));

	//Keyboard shortcuts
	connect(actionToggleVisibility,	SIGNAL(triggered()), this, SLOT(toggleSelectedEntitiesVisibility()));	//'V': toggles selected items visibility
	connect(actionToggleNormals,	SIGNAL(triggered()), this, SLOT(toggleSelectedEntitiesNormals()));		//'N': toggles selected items normals visibility
	connect(actionToggleColors,		SIGNAL(triggered()), this, SLOT(toggleSelectedEntitiesColors()));		//'C': toggles selected items colors visibility
	connect(actionToggleSF,			SIGNAL(triggered()), this, SLOT(toggleSelectedEntitiesSF()));			//'S': toggles selected items SF visibility
	connect(actionToggleShowName,	SIGNAL(triggered()), this, SLOT(toggleSelectedEntities3DName()));		//'D': toggles selected items '3D name' visibility
	connect(actionToggleMaterials,	SIGNAL(triggered()), this, SLOT(toggleSelectedEntitiesMaterials()));	//'M': toggles selected items materials/textures visibility

    connectActions();

    loadPlugins();

#ifdef CC_3DXWARE_SUPPORT
	enable3DMouse(true,true);
#else
	actionEnable3DMouse->setEnabled(false);
#endif

    new3DView();

    freezeUI(false);

    //updateMenus(); //the calls to 'new3DView' and 'freezeUI' already did that
    updateUIWithSelection();

    showMaximized();

    QMainWindow::statusBar()->showMessage(QString("Ready"));
    ccConsole::Print("CloudCompare started!");
}

MainWindow::~MainWindow()
{
	release3DMouse();

	assert(m_ccRoot && m_mdiArea && m_windowMapper);
    m_ccRoot->disconnect();
    m_mdiArea->disconnect();
    m_windowMapper->disconnect();

	//we don't want any other dialog/function to use the following structures
	ccDBRoot* ccRoot = m_ccRoot;
	m_ccRoot = 0;
	if (m_mdiArea)
	{
		QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
		for (int i=0;i<subWindowList.size();++i)
			static_cast<ccGLWindow*>(subWindowList[i]->widget())->setSceneDB(0);
    }
    m_cpeDlg = 0;
    m_gsTool = 0;
    m_transTool = 0;
	m_clipTool = 0;
    m_compDlg=0;
    m_ppDlg = 0;
    m_plpDlg = 0;
	m_pprDlg = 0;
	m_pfDlg = 0;

	//release all 'overlay' dialogs
	while (!m_mdiDialogs.empty())
	{
		m_mdiDialogs.back().dialog->stop(false);
		m_mdiDialogs.back().dialog->setParent(0);
		delete m_mdiDialogs.back().dialog;
		m_mdiDialogs.pop_back();
	}
    //m_mdiDialogs.clear();
    m_mdiArea->closeAllSubWindows();

	if (ccRoot)
		delete ccRoot;
}

ccPluginInterface* MainWindow::getValidPlugin(QObject* plugin)
{
	if (plugin)
	{
		//standard plugin?
		ccStdPluginInterface* ccStdPlugin = qobject_cast<ccStdPluginInterface*>(plugin);
		if (ccStdPlugin)
			return static_cast<ccPluginInterface*>(ccStdPlugin);

		ccGLFilterPluginInterface* ccGLPlugin = qobject_cast<ccGLFilterPluginInterface*>(plugin);
		if (ccGLPlugin)
			return static_cast<ccPluginInterface*>(ccGLPlugin);
	}

    return 0;
}

void MainWindow::loadPlugins()
{
	menuPlugins->setEnabled(false);
	menuShadersAndFilters->setEnabled(false);
	toolBarPluginTools->setVisible(false);
	toolBarGLFilters->setVisible(false);

	//"static" plugins
    foreach (QObject *plugin, QPluginLoader::staticInstances())
		dispatchPlugin(plugin);

    ccConsole::Print(QString("Application path: ")+QCoreApplication::applicationDirPath());

#if defined(Q_OS_MAC)
    // plugins are in the bundle
    QString  path = QCoreApplication::applicationDirPath();
    path.remove( "MacOS" );
    m_pluginsPath = path + "Plugins/ccPlugins";
#else
    //plugins are in bin/plugins
    m_pluginsPath = QCoreApplication::applicationDirPath()+QString("/plugins");
#endif

    ccConsole::Print(QString("Plugins lookup dir.: %1").arg(m_pluginsPath));

    QStringList filters;
#if defined(Q_OS_WIN)
    filters << "*.dll";
#elif defined(Q_OS_LINUX)
    filters << "*.so";
#elif defined(Q_OS_MAC)
    filters << "*.dylib";
#endif
    QDir pluginsDir(m_pluginsPath);
    pluginsDir.setNameFilters(filters);
    foreach (QString filename, pluginsDir.entryList(filters))
    {
        QPluginLoader loader(pluginsDir.absoluteFilePath(filename));
        QObject* plugin = loader.instance();
        if (plugin)
        {
            ccConsole::Print(QString("Found new plugin! ('%1')").arg(filename));
            if (dispatchPlugin(plugin))
            {
                m_pluginFileNames += filename;
            }
            else
            {
                ccConsole::Warning("Unsupported or invalid plugin type");
            }
        }
        else
        {
            ccConsole::Warning(QString("[%1] %2").arg(pluginsDir.absoluteFilePath(filename)).arg(loader.errorString()));
        }
    }

	if (toolBarPluginTools->isEnabled())
	{
		actionDisplayPluginTools->setEnabled(true);
		actionDisplayPluginTools->setChecked(true);
    }
}

bool MainWindow::dispatchPlugin(QObject *plugin)
{
    ccPluginInterface* ccPlugin = getValidPlugin(plugin);
    if (!ccPlugin)
        return false;
	plugin->setParent(this);

	QString pluginName = ccPlugin->getName();
	if (pluginName.isEmpty())
	{
		ccLog::Warning("Plugin has an invalid (empty) name!");
		return false;
	}
    ccConsole::Print("Plugin name: [%s]",qPrintable(pluginName));

	switch(ccPlugin->getType())
	{
	
	case CC_STD_PLUGIN: //standard plugin
		{
			ccStdPluginInterface* stdPlugin = static_cast<ccStdPluginInterface*>(ccPlugin);
			stdPlugin->setMainAppInterface(this);

			QMenu* destMenu=0;
			QToolBar* destToolBar=0;

			QActionGroup actions(this);
			stdPlugin->getActions(actions);
			if (actions.actions().size()>1) //more than one action? We create it's own menu and toolbar
			{
				destMenu = (menuPlugins ? menuPlugins->addMenu(pluginName) : 0);
				if (destMenu)
					destMenu->setIcon(stdPlugin->getIcon());
				destToolBar = addToolBar(pluginName+QString(" toolbar"));
			}
			else //default destination
			{
				destMenu = menuPlugins;
				destToolBar = toolBarPluginTools;
			}

			//add actions
			foreach(QAction* action,actions.actions())
			{
				//add to menu (if any)
				if (destMenu)
				{
					destMenu->addAction(action);
					destMenu->setEnabled(true);
				}
				//add to toolbar
				if (destToolBar)
				{
					destToolBar->addAction(action);
					destToolBar->setVisible(true);
					destToolBar->setEnabled(true);
				}
			}

			//add to std. plugins list
			m_stdPlugins.push_back(stdPlugin);
		}
		break;

	case CC_GL_FILTER_PLUGIN:  //GL filter
		{
			//(auto)create action
			QAction* action = new QAction(pluginName,plugin);
			action->setToolTip(ccPlugin->getDescription());
			action->setIcon(ccPlugin->getIcon());
			//connect default signal
			connect(action, SIGNAL(triggered()), this, SLOT(doEnableGLFilter()));

			menuShadersAndFilters->addAction(action);
			menuShadersAndFilters->setEnabled(true);
			toolBarGLFilters->addAction(action);
			toolBarGLFilters->setVisible(true);
			toolBarGLFilters->setEnabled(true);

			//add to GL filter (actions) list
			m_glFilterActions.addAction(action);

		}
		break;

	default:
		assert(false);
		ccLog::Print("Unhandled plugin type!");
		return false;
	}

    return true;
}

void MainWindow::aboutPlugins()
{
    ccPluginDlg ccpDlg(m_pluginsPath, m_pluginFileNames, this);
    ccpDlg.exec();
}

void MainWindow::doEnableGLFilter()
{
	ccGLWindow* win = getActiveGLWindow();
	if (!win)
	{
		ccLog::Warning("[GL filter] No active 3D view!");
		return;
	}

    QAction *action = qobject_cast<QAction*>(sender());
    ccPluginInterface *ccPlugin = getValidPlugin(action ? action->parent() : 0);
    if (!ccPlugin)
        return;

    if (ccPlugin->getType() != CC_GL_FILTER_PLUGIN)
		return;

	ccGlFilter* filter = static_cast<ccGLFilterPluginInterface*>(ccPlugin)->getFilter();
	if (filter)
	{
		if (win->areGLFiltersEnabled())
		{
			win->setGlFilter(filter);
			ccConsole::Print("Note: go to << Display > Shaders & Filters > No filter >> to disable GL filter");
		}
		else
			ccConsole::Error("GL filters not supported!");
	}
	else
	{
		ccConsole::Error("Can't load GL filter (an error occured)!");
	}
}

void MainWindow::release3DMouse()
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

void MainWindow::setup3DMouse(bool state)
{
	enable3DMouse(state,false);
}

void MainWindow::enable3DMouse(bool state, bool silent)
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

	actionEnable3DMouse->blockSignals(true);
	actionEnable3DMouse->setChecked(state);
	actionEnable3DMouse->blockSignals(false);
}

void MainWindow::on3DMouseKeyUp(int)
{
	//nothing right now
}

#ifdef CC_3DXWARE_SUPPORT
static bool s_3dMouseContextMenuAlreadyShown = false; //DGM: to prevent multiple instances at once
#endif
// ANY CHANGE/BUG FIX SHOULD BE REFLECTED TO THE EQUIVALENT METHODS IN QCC "MainWindow.cpp" FILE!
void MainWindow::on3DMouseKeyDown(int key)
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
				ccGLWindow* activeWin = getActiveGLWindow();
				if (activeWin)
				{
					ccMouse3DContextMenu(&m_3dMouseInput->getMouseParams(),activeWin,this).exec(QCursor::pos());
					//in case of...
					updateViewModePopUpMenu(activeWin);
					updatePivotVisibilityPopUpMenu(activeWin);
				}
				else
				{
					ccLog::Error("No active 3D view! Select a 3D view first...");
					return;
				}

				s_3dMouseContextMenuAlreadyShown = false;
		}
		break;
	case Mouse3DInput::V3DK_FIT:
		{
			if (m_selectedEntities.empty())
				setGlobalZoom();
			else
				zoomOnSelectedEntities();
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
			ccGLWindow* activeWin = getActiveGLWindow();
			if (activeWin)
			{
				CCVector3 axis(0.0f,0.0f,-1.0f);
				CCVector3 trans(0.0f);
				ccGLMatrix mat;
				float angle = (float)(M_PI/2.0);
				if (key == Mouse3DInput::V3DK_CCW)
					angle = -angle;
				mat.initFromParameters(angle,axis,trans);
				activeWin->rotateBaseViewMat(mat);
				activeWin->redraw();
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

// ANY CHANGE/BUG FIX SHOULD BE REFLECTED TO THE EQUIVALENT METHODS IN QCC "MainWindow.cpp" FILE!
void MainWindow::on3DMouseMove(std::vector<float>& vec)
{
	//ccLog::PrintDebug(QString("[3D mouse] %1 %2 %3 %4 %5 %6").arg(vec[0]).arg(vec[1]).arg(vec[2]).arg(vec[3]).arg(vec[4]).arg(vec[5]));

#ifdef CC_3DXWARE_SUPPORT

	//no active device?
	if (!m_3dMouseInput)
		return;

    ccGLWindow* win = getActiveGLWindow();
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

void MainWindow::connectActions()
{
    assert(m_ccRoot);
    assert(m_mdiArea);

	//TODO... but not ready yet ;)
	actionLoadShader->setVisible(false);
	actionKMeans->setVisible(false);
	actionFrontPropagation->setVisible(false);

    /*** MAIN MENU ***/

    //"File" menu
    connect(actionOpen,                         SIGNAL(triggered()),    this,       SLOT(loadFile()));
    connect(actionSave,                         SIGNAL(triggered()),    this,       SLOT(saveFile()));
	connect(actionPrimitiveFactory,				SIGNAL(triggered()),    this,       SLOT(doShowPrimitiveFactory()));
	connect(actionEnable3DMouse,				SIGNAL(toggled(bool)),	this,		SLOT(setup3DMouse(bool)));
    connect(actionQuit,                         SIGNAL(triggered()),    this,       SLOT(close()));

    //"Edit > Colors" menu
    connect(actionSetUniqueColor,               SIGNAL(triggered()),    this,       SLOT(doActionSetUniqueColor()));
    connect(actionSetColorGradient,             SIGNAL(triggered()),    this,       SLOT(doActionSetColorGradient()));
    connect(actionColorize,                     SIGNAL(triggered()),    this,       SLOT(doActionColorize()));
    connect(actionClearColor,                   SIGNAL(triggered()),    this,       SLOT(doActionClearColor()));
    //"Edit > Normals" menu
    connect(actionComputeNormals,               SIGNAL(triggered()),    this,       SLOT(doActionComputeNormals()));
    connect(actionInvertNormals,                SIGNAL(triggered()),    this,       SLOT(doActionInvertNormals()));
	connect(actionConvertNormalToHSV,			SIGNAL(triggered()),    this,       SLOT(doActionConvertNormalsToHSV()));
    connect(actionResolveNormalsDirection,      SIGNAL(triggered()),    this,       SLOT(doActionResolveNormalsDirection()));
    connect(actionClearNormals,                 SIGNAL(triggered()),    this,       SLOT(doActionClearNormals()));
    //"Edit > Octree" menu
    connect(actionComputeOctree,                SIGNAL(triggered()),    this,       SLOT(doActionComputeOctree()));
    connect(actionResampleWithOctree,           SIGNAL(triggered()),    this,       SLOT(doActionResampleWithOctree()));
    //"Edit > Mesh" menu
    connect(actionComputeMeshAA,                SIGNAL(triggered()),    this,       SLOT(doActionComputeMeshAA()));
    connect(actionComputeMeshLS,                SIGNAL(triggered()),    this,       SLOT(doActionComputeMeshLS()));
	connect(actionConvertTextureToColor,		SIGNAL(triggered()),    this,       SLOT(doActionConvertTextureToColor()));
    connect(actionSamplePoints,                 SIGNAL(triggered()),    this,       SLOT(doActionSamplePoints()));
    connect(actionSmoothMeshLaplacian,			SIGNAL(triggered()),    this,       SLOT(doActionSmoothMeshLaplacian()));
	connect(actionSubdivideMesh,				SIGNAL(triggered()),    this,       SLOT(doActionSubdivideMesh()));
    connect(actionMeasureMeshSurface,           SIGNAL(triggered()),    this,       SLOT(doActionMeasureMeshSurface()));
    //"Edit > Mesh > Scalar Field" menu
    connect(actionSmoothMeshSF,                 SIGNAL(triggered()),    this,       SLOT(doActionSmoothMeshSF()));
    connect(actionEnhanceMeshSF,                SIGNAL(triggered()),    this,       SLOT(doActionEnhanceMeshSF()));
    //"Edit > Sensor > Ground-Based lidar" menu
    connect(actionShowDepthBuffer,              SIGNAL(triggered()),    this,       SLOT(doActionShowDepthBuffer()));
    connect(actionExportDepthBuffer,            SIGNAL(triggered()),    this,       SLOT(doActionExportDepthBuffer()));
    //"Edit > Sensor" menu
    connect(actionProjectSensor,                SIGNAL(triggered()),    this,       SLOT(doActionProjectSensor()));
    connect(actionModifySensor,                 SIGNAL(triggered()),    this,       SLOT(doActionModifySensor()));
    connect(actionComputeDistancesFromSensor,   SIGNAL(triggered()),    this,       SLOT(doActionComputeDistancesFromSensor()));
    connect(actionComputeScatteringAngles,      SIGNAL(triggered()),    this,       SLOT(doActionComputeScatteringAngles()));
    //"Edit > Scalar fields" menu
    connect(actionSFGradient,                   SIGNAL(triggered()),    this,       SLOT(doActionSFGradient()));
    connect(actionGaussianFilter,               SIGNAL(triggered()),    this,       SLOT(doActionSFGaussianFilter()));
    connect(actionBilateralFilter,              SIGNAL(triggered()),    this,       SLOT(doActionSFBilateralFilter()));
    connect(actionFilterByValue,                SIGNAL(triggered()),    this,       SLOT(doActionFilterByValue()));
	connect(actionAddConstantSF,				SIGNAL(triggered()),    this,       SLOT(doActionAddConstantSF()));
    connect(actionScalarFieldArithmetic,        SIGNAL(triggered()),    this,       SLOT(doActionScalarFieldArithmetic()));
    connect(actionScalarFieldFromColor,         SIGNAL(triggered()),    this,       SLOT(doActionScalarFieldFromColor()));
    connect(actionConvertToRGB,                 SIGNAL(triggered()),    this,       SLOT(doActionSFConvertToRGB()));
	connect(actionRenameSF,						SIGNAL(triggered()),    this,       SLOT(doActionRenameSF()));
	connect(actionOpenColorScalesManager,		SIGNAL(triggered()),    this,       SLOT(doActionOpenColorScalesManager()));
    connect(actionAddIdField,                   SIGNAL(triggered()),    this,       SLOT(doActionAddIdField()));
    connect(actionDeleteScalarField,            SIGNAL(triggered()),    this,       SLOT(doActionDeleteScalarField()));
    connect(actionDeleteAllSF,                  SIGNAL(triggered()),    this,       SLOT(doActionDeleteAllSF()));
    //"Edit" menu
    connect(actionClone,                        SIGNAL(triggered()),    this,       SLOT(doActionClone()));
    connect(actionFuse,                         SIGNAL(triggered()),    this,       SLOT(doActionFuse()));
    connect(actionApplyTransformation,			SIGNAL(triggered()),    this,       SLOT(doActionApplyTransformation()));
    connect(actionMultiply,                     SIGNAL(triggered()),    this,       SLOT(doActionMultiply()));
    connect(actionEditGlobalShift,				SIGNAL(triggered()),    this,       SLOT(doActionEditGlobalShift()));
    connect(actionSubsample,                    SIGNAL(triggered()),    this,       SLOT(doActionSubsample())); //Aurelien BEY le 13/11/2008
    connect(actionSynchronize,                  SIGNAL(triggered()),    this,       SLOT(doActionSynchronize()));
    connect(actionDelete,                       SIGNAL(triggered()),    m_ccRoot,	SLOT(deleteSelectedEntities()));

    //"Tools > Projection" menu
    connect(actionUnroll,                       SIGNAL(triggered()),    this,       SLOT(doActionUnroll()));
    connect(actionHeightGridGeneration,         SIGNAL(triggered()),    this,       SLOT(doActionHeightGridGeneration()));
	connect(actionExportCoordToSF,				SIGNAL(triggered()),    this,       SLOT(doActionExportCoordToSF()));
    //"Tools > Registration" menu
    connect(actionRegister,                     SIGNAL(triggered()),    this,       SLOT(doActionRegister()));
    connect(actionPointPairsAlign,				SIGNAL(triggered()),    this,       SLOT(activateRegisterPointPairTool()));
    //"Tools > Distances" menu
    connect(actionCloudCloudDist,               SIGNAL(triggered()),    this,       SLOT(doActionCloudCloudDist()));
    connect(actionCloudMeshDist,                SIGNAL(triggered()),    this,       SLOT(doActionCloudMeshDist()));
    connect(actionCPS,                          SIGNAL(triggered()),    this,       SLOT(doActionComputeCPS()));
    //"Tools > Statistics" menu
    connect(actionComputeStatParams,            SIGNAL(triggered()),    this,       SLOT(doActionComputeStatParams()));
    connect(actionStatisticalTest,              SIGNAL(triggered()),    this,       SLOT(doActionStatisticalTest()));
    //"Tools > Segmentation" menu
    connect(actionLabelConnectedComponents,     SIGNAL(triggered()),    this,       SLOT(doActionLabelConnectedComponents()));
    connect(actionKMeans,                       SIGNAL(triggered()),    this,       SLOT(doActionKMeans()));
    connect(actionFrontPropagation,             SIGNAL(triggered()),    this,       SLOT(doActionFrontPropagation()));
	connect(actionCrossSection,					SIGNAL(triggered()),	this,		SLOT(activateClippingBoxMode()));
    //"Tools > Other" menu
    connect(actionDensity,                      SIGNAL(triggered()),    this,       SLOT(doComputeDensity()));
    connect(actionCurvature,                    SIGNAL(triggered()),    this,       SLOT(doComputeCurvature()));
    connect(actionRoughness,                    SIGNAL(triggered()),    this,       SLOT(doComputeRoughness()));
    connect(actionPlaneOrientation,				SIGNAL(triggered()),    this,       SLOT(doComputePlaneOrientation()));
	//"Tools"
    connect(actionPointListPicking,             SIGNAL(triggered()),    this,       SLOT(activatePointListPickingMode()));
    connect(actionPointPicking,                 SIGNAL(triggered()),    this,       SLOT(activatePointsPropertiesMode()));

	//"Tools > Sand box (research)" menu
    connect(actionComputeKdTree,                SIGNAL(triggered()),    this,       SLOT(doActionComputeKdTree()));
	connect(actionMeshBestFittingQuadric,		SIGNAL(triggered()),    this,       SLOT(doActionComputeQuadric3D()));
    connect(actionComputeBestFitBB,             SIGNAL(triggered()),    this,       SLOT(doComputeBestFitBB()));
    connect(actionAlign,                        SIGNAL(triggered()),    this,       SLOT(doAction4pcsRegister())); //Aurelien BEY le 13/11/2008
    connect(actionSNETest,						SIGNAL(triggered()),    this,       SLOT(doSphericalNeighbourhoodExtractionTest()));

    //"Display" menu
    connect(actionFullScreen,                   SIGNAL(toggled(bool)),  this,       SLOT(toggleFullScreen(bool)));
    connect(actionRefresh,                      SIGNAL(triggered()),    this,       SLOT(refreshAll()));
    connect(actionTestFrameRate,                SIGNAL(triggered()),    this,       SLOT(testFrameRate()));
    connect(actionToggleCenteredPerspective,    SIGNAL(triggered()),    this,       SLOT(toggleActiveWindowCenteredPerspective()));
    connect(actionToggleViewerBasedPerspective, SIGNAL(triggered()),    this,       SLOT(toggleActiveWindowViewerBasedPerspective()));
    connect(actionEditCamera,                   SIGNAL(triggered()),    this,       SLOT(doActionEditCamera()));
	connect(actionSaveViewportAsObject,			SIGNAL(triggered()),    this,       SLOT(doActionSaveViewportAsCamera()));

    //"Display > Lights & Materials" menu
    connect(actionDisplayOptions,				SIGNAL(triggered()),    this,       SLOT(setLightsAndMaterials()));
    connect(actionToggleSunLight,               SIGNAL(triggered()),    this,       SLOT(toggleActiveWindowSunLight()));
    connect(actionToggleCustomLight,            SIGNAL(triggered()),    this,       SLOT(toggleActiveWindowCustomLight()));
    connect(actionRenderToFile,                 SIGNAL(triggered()),    this,       SLOT(doActionRenderToFile()));
    //"Display > Shaders & filters" menu
    connect(actionLoadShader,                   SIGNAL(triggered()),    this,       SLOT(doActionLoadShader()));
    connect(actionDeleteShader,                 SIGNAL(triggered()),    this,       SLOT(doActionDeleteShader()));
    connect(actionNoFilter,                     SIGNAL(triggered()),    this,       SLOT(doActionDeactivateGlFilter()));

    //"Display > Active SF" menu
    connect(actionToggleActiveSFColorScale,		SIGNAL(triggered()),    this,       SLOT(doActionToggleActiveSFColorScale()));
    connect(actionShowActiveSFPrevious,			SIGNAL(triggered()),    this,       SLOT(doActionShowActiveSFPrevious()));
    connect(actionShowActiveSFNext,				SIGNAL(triggered()),    this,       SLOT(doActionShowActiveSFNext()));

    //"3D Views" menu
    connect(menu3DViews,                        SIGNAL(aboutToShow()),  this,       SLOT(update3DViewsMenu()));
    connect(actionNew3DView,                    SIGNAL(triggered()),    this,       SLOT(new3DView()));
    connect(actionClose3DView,                  SIGNAL(triggered()),    m_mdiArea,  SLOT(closeActiveSubWindow()));
    connect(actionCloseAll3DViews,              SIGNAL(triggered()),    m_mdiArea,  SLOT(closeAllSubWindows()));
    connect(actionTile3DViews,                  SIGNAL(triggered()),    m_mdiArea,  SLOT(tileSubWindows()));
    connect(actionCascade3DViews,               SIGNAL(triggered()),    m_mdiArea,  SLOT(cascadeSubWindows()));
    connect(actionNext3DView,                   SIGNAL(triggered()),    m_mdiArea,  SLOT(activateNextSubWindow()));
    connect(actionPrevious3DView,               SIGNAL(triggered()),    m_mdiArea,  SLOT(activatePreviousSubWindow()));

    //"About" menu entry
    connect(actionHelp,                         SIGNAL(triggered()),    this,       SLOT(help()));
    connect(actionAbout,                        SIGNAL(triggered()),    this,       SLOT(about()));
    connect(actionAboutPlugins,                 SIGNAL(triggered()),    this,       SLOT(aboutPlugins()));

    /*** Toolbars ***/

    //View toolbar
    connect(actionGlobalZoom,                   SIGNAL(triggered()),    this,       SLOT(setGlobalZoom()));
    connect(actionPickRotationCenter,           SIGNAL(triggered()),    this,       SLOT(doPickRotationCenter()));
    connect(actionZoomAndCenter,                SIGNAL(triggered()),    this,       SLOT(zoomOnSelectedEntities()));
	connect(actionSetPivotAlwaysOn,				SIGNAL(triggered()),    this,       SLOT(setPivotAlwaysOn()));
	connect(actionSetPivotRotationOnly,			SIGNAL(triggered()),    this,       SLOT(setPivotRotationOnly()));
	connect(actionSetPivotOff,					SIGNAL(triggered()),    this,       SLOT(setPivotOff()));
	connect(actionSetOrthoView,					SIGNAL(triggered()),    this,       SLOT(setOrthoView()));
	connect(actionSetCenteredPerspectiveView,	SIGNAL(triggered()),    this,       SLOT(setCenteredPerspectiveView()));
	connect(actionSetViewerPerspectiveView,		SIGNAL(triggered()),    this,       SLOT(setViewerPerspectiveView()));
    connect(actionSetViewTop,                   SIGNAL(triggered()),    this,       SLOT(setTopView()));
    connect(actionSetViewBottom,				SIGNAL(triggered()),    this,       SLOT(setBottomView()));
    connect(actionSetViewFront,                 SIGNAL(triggered()),    this,       SLOT(setFrontView()));
    connect(actionSetViewBack,                  SIGNAL(triggered()),    this,       SLOT(setBackView()));
    connect(actionSetViewLeft,					SIGNAL(triggered()),    this,       SLOT(setLeftView()));
    connect(actionSetViewRight,		            SIGNAL(triggered()),    this,       SLOT(setRightView()));
    connect(actionSetViewIso1,		            SIGNAL(triggered()),    this,       SLOT(setIsoView1()));
    connect(actionSetViewIso2,		            SIGNAL(triggered()),    this,       SLOT(setIsoView2()));

    //"Main tools" toolbar
    connect(actionTranslateRotate,              SIGNAL(triggered()),    this,       SLOT(activateTranslateRotateMode()));
    connect(actionSegment,                      SIGNAL(triggered()),    this,       SLOT(activateSegmentationMode()));
    connect(actionShowHistogram,                SIGNAL(triggered()),    this,       SLOT(showSelectedEntitiesHistogram()));
}

void MainWindow::doActionColorize()
{
    doActionSetColor(true);
}

void MainWindow::doActionSetUniqueColor()
{
    doActionSetColor(false);
}

void MainWindow::doActionSetColor(bool colorize)
{
    //ask for color choice
    QColor newCol = QColorDialog::getColor(Qt::white, this);

    if (!newCol.isValid())
        return;

	ccHObject::Container selectedEntities = m_selectedEntities;
	while (!selectedEntities.empty())
	{
        ccHObject* ent = selectedEntities.back();
		selectedEntities.pop_back();
		if (ent->isA(CC_HIERARCHY_OBJECT))
		{
			//automatically parse a group's children set
			for (unsigned i=0;i<ent->getChildrenNumber();++i)
				selectedEntities.push_back(ent->getChild(i));
		}
		else
		{
			bool lockedVertices;
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
			if (lockedVertices && !ent->isA(CC_MESH_GROUP))
			{
				DisplayLockedVerticesWarning();
				continue;
			}

			if (cloud && cloud->isA(CC_POINT_CLOUD)) // TODO
			{
				if (colorize)
					static_cast<ccPointCloud*>(cloud)->colorize(newCol.redF(), newCol.greenF(), newCol.blueF());
				else
					static_cast<ccPointCloud*>(cloud)->setRGBColor(newCol.red(), newCol.green(), newCol.blue());
				ent->showColors(true);
				ent->prepareDisplayForRefresh();

				if (ent->getParent() && ent->getParent()->isKindOf(CC_MESH))
					ent->getParent()->showColors(true);
			}
		}
    }

    refreshAll();
	updateUI();
}

void MainWindow::doActionSetColorGradient()
{
    ccTwoColorsDlg dlg(this);
    if (!dlg.exec())
        return;

    unsigned char dim = (unsigned char)dlg.directionComboBox->currentIndex();
    bool useDefaultRamp = (dlg.defaultRampCheckBox->checkState() == Qt::Checked);

	ccColorScale::Shared colorScale(0);
	if (useDefaultRamp)
	{
		colorScale = ccColorScalesManager::GetDefaultScale();
	}
	else
	{
		colorScale = ccColorScale::Create("Temp scale");
		colorScale->insert(ccColorScaleElement(0.0,dlg.s_firstColor),false);
		colorScale->insert(ccColorScaleElement(1.0,dlg.s_secondColor),true);
	}
	assert(colorScale);

    size_t selNum = m_selectedEntities.size();
    for (size_t i=0; i<selNum; ++i)
    {
        ccHObject* ent = m_selectedEntities[i];
		
		bool lockedVertices;
        ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
		if (lockedVertices)
		{
			DisplayLockedVerticesWarning();
			continue;
		}

        if (cloud && cloud->isA(CC_POINT_CLOUD)) // TODO
        {
			if (static_cast<ccPointCloud*>(cloud)->setRGBColorByHeight(dim, colorScale))
			{
				ent->showColors(true);
				ent->prepareDisplayForRefresh();
			}
        }
    }

    refreshAll();
	updateUI();
}

void MainWindow::doActionInvertNormals()
{
    size_t i,selNum = m_selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = m_selectedEntities[i];
		bool lockedVertices;
        ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
		if (lockedVertices)
		{
			DisplayLockedVerticesWarning();
			continue;
		}

        if (cloud && cloud->isA(CC_POINT_CLOUD)) // TODO
        {
            ccPointCloud* ccCloud = static_cast<ccPointCloud*>(cloud);
			if (ccCloud->hasNormals())
			{
				ccCloud->invertNormals();
				ccCloud->showNormals(true);
				ccCloud->prepareDisplayForRefresh_recursive();
			}
		}
    }

    refreshAll();
}

void MainWindow::doActionConvertNormalsToHSV()
{
    size_t i,selNum = m_selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = m_selectedEntities[i];
		bool lockedVertices;
        ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
		if (lockedVertices)
		{
			DisplayLockedVerticesWarning();
			continue;
		}

        if (cloud && cloud->isA(CC_POINT_CLOUD)) // TODO
        {
            ccPointCloud* ccCloud = static_cast<ccPointCloud*>(cloud);
			if (ccCloud->hasNormals())
			{
				ccCloud->convertNormalToRGB();
				ccCloud->showColors(true);
				ccCloud->showNormals(false);
				ccCloud->showSF(false);
				ccCloud->prepareDisplayForRefresh_recursive();
			}
        }
    }

    refreshAll();
	updateUI();
}

static double s_kdTreeMaxRMSPerCell = 0.1;
void MainWindow::doActionComputeKdTree()
{
	ccGenericPointCloud* cloud = 0;

	if (m_selectedEntities.size() == 1)
	{
		ccHObject* ent = m_selectedEntities.back();
		bool lockedVertices;
		cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
		if (lockedVertices)
		{
			DisplayLockedVerticesWarning();
			return;
		}
	}

	if (!cloud)
	{
		ccLog::Error("Selected one and only one point cloud or mesh!");
		return;
	}

	bool ok;
	s_kdTreeMaxRMSPerCell = QInputDialog::getDouble(this, "Kd-tree", "Max RMS per leaf cell:", s_kdTreeMaxRMSPerCell, 1.0e-6, 1.0e6, 6, &ok);
	if (!ok)
		return;

    ccProgressDialog pDlg(true,this);
	
	//computation
	QElapsedTimer eTimer;
	eTimer.start();
	ccKdTree* kdtree = new ccKdTree(cloud);

	if (kdtree->build(s_kdTreeMaxRMSPerCell,&pDlg))
	{
		int elapsedTime_ms = eTimer.elapsed();

		ccConsole::Print("[doActionComputeKdTree] Timing: %2.3f s",elapsedTime_ms/1.0e3);
		cloud->setEnabled(true); //for mesh vertices!
		cloud->addChild(kdtree);
		kdtree->setDisplay(cloud->getDisplay());
		kdtree->setVisible(true);
		kdtree->prepareDisplayForRefresh();
#ifdef _DEBUG
		kdtree->convertCellIndexToSF();
#endif

		addToDB(kdtree,true,0,false,false);
		
		refreshAll();
		updateUI();
	}
	else
	{
		ccLog::Error("An error occured!");
		delete kdtree;
		kdtree = 0;
	}
}

void MainWindow::doActionComputeOctree()
{
	ccBBox bbox;
	std::set<ccGenericPointCloud*> clouds;
    size_t selNum = m_selectedEntities.size();
	PointCoordinateType maxBoxSize = -1.0;
    for (size_t i=0; i<selNum; ++i)
    {
        ccHObject* ent = m_selectedEntities[i];

		//specific test for locked vertices
		bool lockedVertices;
        ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
		if (cloud && lockedVertices)
		{
			DisplayLockedVerticesWarning();
			continue;
		}
		clouds.insert(cloud);

		//we look for the biggest box so as to define the "minimum cell size"
		ccBBox thisBBox = cloud->getMyOwnBB();
		if (thisBBox.isValid())
		{
			CCVector3 dd = thisBBox.maxCorner()-thisBBox.minCorner();
			PointCoordinateType maxd = std::max(dd.x,std::max(dd.y,dd.z));
			if (maxBoxSize < 0.0 || maxd > maxBoxSize)
				maxBoxSize = maxd;
		}

		bbox += cloud->getBB();
	}

	if (clouds.empty() || maxBoxSize < 0.0)
	{
		ccLog::Warning("[doActionComputeOctree] No elligible entities in selection!");
		return;
	}

	//min(cellSize) = max(dim)/2^N with N = max subidivision level
	double minCellSize = (double)maxBoxSize/(double)(1 << ccOctree::MAX_OCTREE_LEVEL);

	ccComputeOctreeDlg coDlg(bbox,minCellSize,this);
	if (!coDlg.exec())
		return;

    ccProgressDialog pDlg(true,this);
	
	//if we must use a custom bounding box, we update 'bbox'
	if (coDlg.getMode() == ccComputeOctreeDlg::CUSTOM_BBOX)
		bbox = coDlg.getCustomBBox();

    for (std::set<ccGenericPointCloud*>::iterator it = clouds.begin(); it != clouds.end(); ++it)
    {
        ccGenericPointCloud* cloud = *it;

		//we temporarily detach entity, as it may undergo
		//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::computeOctree
		ccHObject* parent=0;
		removeObjectTemporarilyFromDBTree(cloud,parent);
		
		//computation
		QElapsedTimer eTimer;
		eTimer.start();
		ccOctree* octree = 0;
		switch(coDlg.getMode())
		{
		case ccComputeOctreeDlg::DEFAULT:
			octree = cloud->computeOctree(&pDlg);
			break;
		case ccComputeOctreeDlg::MIN_CELL_SIZE:
		case ccComputeOctreeDlg::CUSTOM_BBOX:
			{
				//for a cell-size based custom box, we must update it for each cloud!
				if (coDlg.getMode() == ccComputeOctreeDlg::MIN_CELL_SIZE)
				{
					double cellSize = coDlg.getMinCellSize();
					PointCoordinateType halfBoxWidth = (PointCoordinateType)(cellSize * (double)(1 << ccOctree::MAX_OCTREE_LEVEL) / 2.0);
					CCVector3 C = cloud->getBB().getCenter();
					bbox = ccBBox(	C-CCVector3(halfBoxWidth,halfBoxWidth,halfBoxWidth),
									C+CCVector3(halfBoxWidth,halfBoxWidth,halfBoxWidth));
				}
				cloud->deleteOctree();
				octree = new ccOctree(cloud);
				if (octree->build(bbox.minCorner(),bbox.maxCorner(),0,0,&pDlg)>0)
				{
					octree->setDisplay(cloud->getDisplay());
					cloud->addChild(octree);
				}
				else
				{
					delete octree;
					octree = 0;
				}
			}
			break;
		default:
			assert(false);
			return;
		}
		int elapsedTime_ms = eTimer.elapsed();
		
		//put object back in tree
		putObjectBackIntoDBTree(cloud,parent);
		
		if (octree)
		{
			ccConsole::Print("[doActionComputeOctree] Timing: %2.3f s",elapsedTime_ms/1.0e3);
			cloud->setEnabled(true); //for mesh vertices!
			octree->setVisible(true);
			octree->prepareDisplayForRefresh();
		}
		else
		{
			ccConsole::Warning(QString("Octree computation on cloud '%1' failed!").arg(cloud->getName()));
		}
    }

    refreshAll();
	updateUI();
}

void MainWindow::doActionResampleWithOctree()
{
    ccAskOneIntValueDlg dlg("Points (approx)",1,INT_MAX,1000000,"Resample with octree",this);
    if (!dlg.exec())
        return;

    ccProgressDialog pDlg(false,this);
    unsigned aimedPoints = (unsigned)dlg.getValue();

	ccHObject::Container selectedEntities = m_selectedEntities;
    size_t i,selNum = selectedEntities.size();
	bool errors = false;
    for (i=0; i<selNum; ++i)
    {
        ccPointCloud* cloud = 0;
        ccHObject* ent = selectedEntities[i];
        /*if (ent->isKindOf(CC_MESH)) //TODO
            cloud = ccHObjectCaster::ToGenericMesh(ent)->getAssociatedCloud();
        else */
        if (ent->isKindOf(CC_POINT_CLOUD))
            cloud = static_cast<ccPointCloud*>(ent);

        if (cloud)
        {
            ccOctree* octree = cloud->getOctree();
            if (!octree)
			{
                octree = cloud->computeOctree(&pDlg);
				if (!octree)
				{
					ccConsole::Error(QString("Could not compute octree for cloud '%1'").arg(cloud->getName()));
					continue;
				}
			}

			cloud->setEnabled(false);
			QElapsedTimer eTimer;
			eTimer.start();
			CCLib::GenericIndexedCloud* result = CCLib::CloudSamplingTools::resampleCloudWithOctree(cloud,
				aimedPoints,
				CCLib::CloudSamplingTools::CELL_GRAVITY_CENTER,
				&pDlg,
				octree);

			if (result)
			{
				ccConsole::Print("[ResampleWithOctree] Timing: %3.2f s.",eTimer.elapsed()/1.0e3);
				ccPointCloud* newCloud = ccPointCloud::From(result);

				delete result;
				result = 0;

				if (newCloud)
				{
					const double* shift = cloud->getOriginalShift();
					if (shift)
						newCloud->setOriginalShift(shift[0],shift[1],shift[2]);
					addToDB(newCloud, true, 0, false, false);
					newCloud->setDisplay(cloud->getDisplay());
					newCloud->prepareDisplayForRefresh();
				}
				else
				{
					errors = true;
				}
			}
        }
    }

	if (errors)
		ccLog::Error("[ResampleWithOctree] Errors occured during the process! Result may be incomplete!");

    refreshAll();
}

void MainWindow::doActionApplyTransformation()
{
	ccApplyTransformationDlg dlg(this);
	if (!dlg.exec())
		return;

	ccGLMatrix transMat = dlg.getTransformation();

	//we must backup 'm_selectedEntities' as removeObjectTemporarilyFromDBTree can modify it!
	ccHObject::Container selectedEntities = m_selectedEntities;
    size_t i,selNum = selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = selectedEntities[i];
		
		//specific test for locked vertices
		bool lockedVertices;
        ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
		if (cloud && lockedVertices)
		{
			DisplayLockedVerticesWarning();
			continue;
		}

		//we temporarily detach entity, as it may undergo
		//"severe" modifications (octree deletion, etc.) --> see ccHObject::applyRigidTransformation
		ccHObject* parent=0;
		removeObjectTemporarilyFromDBTree(ent,parent);
		ent->setGLTransformation(transMat);
		ent->applyGLTransformation_recursive();
		ent->prepareDisplayForRefresh_recursive();
		putObjectBackIntoDBTree(ent,parent);
    }

    refreshAll();
}

static double s_lastMultFactorX = 1.0;
static double s_lastMultFactorY = 1.0;
static double s_lastMultFactorZ = 1.0;
void MainWindow::doActionMultiply()
{
    ccAskThreeDoubleValuesDlg dlg("fx","fy","fz",-1.0e6,1.0e6,s_lastMultFactorX,s_lastMultFactorY,s_lastMultFactorZ,8,"Scaling",this);
    if (!dlg.exec())
        return;

	//save values for next time
	s_lastMultFactorX = dlg.doubleSpinBox1->value();
	s_lastMultFactorY = dlg.doubleSpinBox2->value();
	s_lastMultFactorZ = dlg.doubleSpinBox3->value();

	//we must backup 'm_selectedEntities' as removeObjectTemporarilyFromDBTree can modify it!
	ccHObject::Container selectedEntities = m_selectedEntities;
    size_t i,selNum = selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = selectedEntities[i];
		bool lockedVertices;
        ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
		if (lockedVertices)
		{
			DisplayLockedVerticesWarning();
			continue;
		}

        if (cloud && cloud->isA(CC_POINT_CLOUD)) //TODO
        {
			//we temporarily detach entity, as it may undergo
			//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::multiply
			ccHObject* parent=0;
			removeObjectTemporarilyFromDBTree(cloud,parent);
            static_cast<ccPointCloud*>(cloud)->multiply((PointCoordinateType)s_lastMultFactorX,
														(PointCoordinateType)s_lastMultFactorY,
														(PointCoordinateType)s_lastMultFactorZ);
			putObjectBackIntoDBTree(cloud,parent);
            cloud->prepareDisplayForRefresh_recursive();

			//don't forget shift on load!
			const double* shift = cloud->getOriginalShift();
			if (shift)
			{
				cloud->setOriginalShift(shift[0]*s_lastMultFactorX,
										shift[1]*s_lastMultFactorY,
										shift[2]*s_lastMultFactorZ);
			}
        }
    }

    refreshAll();
	updateUI();
}

void MainWindow::doActionEditGlobalShift()
{
    size_t selNum = m_selectedEntities.size();
    if (selNum!=1)
    {
		if (selNum>1)
			ccConsole::Error("Select only one point cloud or mesh!");
        return;
    }
	ccHObject* ent = m_selectedEntities[0];

	bool lockedVertices;
	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);

	//for "real" point clouds only
	if (!cloud)
		return;
	if (lockedVertices)
	{
		//see ccPropertiesTreeDelegate::fillWithMesh
		if (!ent->isA(CC_MESH_GROUP) && !ent->isAncestorOf(cloud))
		{
			DisplayLockedVerticesWarning();
			return;
		}
	}

	assert(cloud);
	const double* shift = cloud->getOriginalShift();

	ccAskThreeDoubleValuesDlg dlg("x","y","z",-DBL_MAX,DBL_MAX,shift[0],shift[1],shift[2],2,"Global shift",this);
    if (!dlg.exec())
        return;

	double x = dlg.doubleSpinBox1->value();
	double y = dlg.doubleSpinBox2->value();
	double z = dlg.doubleSpinBox3->value();

	//apply new shift
	cloud->setOriginalShift(x,y,z);
	ccLog::Print("[doActionEditGlobalShift] New shift: (%f, %f, %f)",x,y,z);
    
	updateUI();
}

void MainWindow::doComputeBestFitBB()
{
    if (QMessageBox::warning(	this,
								"This method is for test purpose only",
								"Cloud(s) are going to be rotated while still displayed in their previous position! Proceed?",
								QMessageBox::Yes | QMessageBox::No,
								QMessageBox::No ) != QMessageBox::Yes)
	{
        return;
	}

	//we must backup 'm_selectedEntities' as removeObjectTemporarilyFromDBTree can modify it!
	ccHObject::Container selectedEntities = m_selectedEntities;
	size_t i,j,selNum = selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = selectedEntities[i];
        ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);

        if (cloud && cloud->isA(CC_POINT_CLOUD)) // TODO
        {
            CCLib::Neighbourhood Yk(cloud);

            CCLib::SquareMatrixd covMat = Yk.computeCovarianceMatrix();
            if (covMat.isValid())
            {
                CCLib::SquareMatrixd eig = covMat.computeJacobianEigenValuesAndVectors();
                if (eig.isValid())
                {
                    eig.sortEigenValuesAndVectors();

                    ccGLMatrix trans;
                    GLfloat* rotMat = trans.data();
                    for (j=0;j<3;++j)
                    {
						double u[3];
                        eig.getEigenValueAndVector((unsigned)j,u);
                        CCVector3 v(u[0],u[1],u[2]);
                        v.normalize();
                        rotMat[j*4] = v.x;
                        rotMat[j*4+1] = v.y;
                        rotMat[j*4+2] = v.z;
                    }

					const CCVector3* G = Yk.getGravityCenter();
					assert(G);
                    trans.shiftRotationCenter(*G);

                    cloud->setGLTransformation(trans);
                    trans.invert();

					//we temporarily detach entity, as it may undergo
					//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::applyRigidTransformation
					ccHObject* parent=0;
					removeObjectTemporarilyFromDBTree(cloud,parent);
					static_cast<ccPointCloud*>(cloud)->applyRigidTransformation(trans);
					putObjectBackIntoDBTree(cloud,parent);

                    ent->prepareDisplayForRefresh_recursive();
                }
            }
        }
    }

    refreshAll();
}

void MainWindow::doActionClearColor()
{
    doActionClearProperty(0);
}

void MainWindow::doActionClearNormals()
{
    doActionClearProperty(1);
}

void MainWindow::doActionDeleteScalarField()
{
    doActionClearProperty(2);
}

void MainWindow::doActionDeleteAllSF()
{
    doActionClearProperty(3);
}

void MainWindow::doActionClearProperty(int prop)
{
	//we must backup 'm_selectedEntities' as removeObjectTemporarilyFromDBTree can modify it!
	ccHObject::Container selectedEntities = m_selectedEntities;

    size_t i,selNum = selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = selectedEntities[i];

		//specific case: clear normals on a mesh
		if (prop == 1 && ent->isKindOf(CC_MESH))
		{
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(ent);
			if (mesh->hasTriNormals())
			{
				mesh->showNormals(false);
				ccHObject* parent=0;
				removeObjectTemporarilyFromDBTree(mesh,parent);
				mesh->clearTriNormals();
				putObjectBackIntoDBTree(mesh,parent);
				ent->prepareDisplayForRefresh();
				continue;
			}
			else if (mesh->hasNormals()) //per-vertex normals?
			{
				if (mesh->getParent()
					&& mesh->getParent()->isKindOf(CC_MESH)
					&& ccHObjectCaster::ToGenericMesh(mesh->getParent())->getAssociatedCloud() == mesh->getAssociatedCloud())
				{
					ccLog::Warning("[doActionClearNormals] Can't remove per-vertex normals on a sub mesh!");
				}
				else //mesh is alone, we can freely remove normals
				{
					if (mesh->getAssociatedCloud() && mesh->getAssociatedCloud()->isA(CC_POINT_CLOUD))
					{
						mesh->showNormals(false);
						static_cast<ccPointCloud*>(mesh->getAssociatedCloud())->unallocateNorms();
						mesh->prepareDisplayForRefresh();
						continue;
					}
				}
			}
		}

		bool lockedVertices;
        ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
		if (lockedVertices)
		{
			DisplayLockedVerticesWarning();
			continue;
		}

        if (cloud && cloud->isA(CC_POINT_CLOUD)) // TODO
        {
            switch (prop)
            {
            case 0: //colors
                if (cloud->hasColors())
                {
                    static_cast<ccPointCloud*>(cloud)->unallocateColors();
                    ent->prepareDisplayForRefresh();
                }
                break;
            case 1: //normals
                if (cloud->hasNormals())
                {
                    static_cast<ccPointCloud*>(cloud)->unallocateNorms();
                    ent->prepareDisplayForRefresh();
                }
                break;
            case 2: //current sf
                if (cloud->hasDisplayedScalarField())
                {
                    ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
                    pc->deleteScalarField(pc->getCurrentDisplayedScalarFieldIndex());
                    ent->prepareDisplayForRefresh();
                }
                break;
            case 3: //all sf
                if (cloud->hasScalarFields())
                {
                    static_cast<ccPointCloud*>(cloud)->deleteAllScalarFields();
                    ent->prepareDisplayForRefresh();
                }
                break;
            }
        }
    }

    refreshAll();
	updateUI();
}

void MainWindow::doActionMeasureMeshSurface()
{
    size_t i,selNum = m_selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = m_selectedEntities[i];
        if (ent->isKindOf(CC_MESH))
        {
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(ent);
            double S = CCLib::MeshSamplingTools::computeMeshArea(mesh);
            //we force the console to display itself
            forceConsoleDisplay();
            ccConsole::Print(QString("[Mesh Surface Measurer] Mesh %1: S=%2 (square units)").arg(ent->getName()).arg(S));
			if (mesh->size())
				ccConsole::Print(QString("[Mesh Surface Measurer] Mean triangle surface: %1 (square units)").arg(S/double(mesh->size())));
        }
    }
}

void displaySensorProjectErrorString(int errorCode)
{
    switch (errorCode)
    {
    case -1:
        ccConsole::Error("Internal error: bad input!");
        break;
    case -2:
        ccConsole::Error("Error: depth buffer is too big (try to reduce angular steps)");
        break;
    case -3:
        ccConsole::Error("Error: depth buffer is too small (try to increase angular steps)");
        break;
    case -4:
        ccConsole::Error("Error: not enough memory! (try to reduce angular steps)");
        break;
    default:
        ccConsole::Error("An unknown error occured while creating sensor (code: %i)",errorCode);
    }
}

void MainWindow::doActionComputeDistancesFromSensor()
{
    //there should be only one sensor in current selection!
    if (m_selectedEntities.empty() || m_selectedEntities.size()>1 || !m_selectedEntities[0]->isKindOf(CC_SENSOR))
    {
        ccConsole::Error("Select one and only one sensor!");
        return;
    }

	ccSensor* sensor = ccHObjectCaster::ToSensor(m_selectedEntities[0]);
	assert(sensor);

    //sensor must have a parent cloud -> this error probably could not happen
    //If in a future cc will permits to have not-cloud-associated sensors this will
    //ensure to not have bugs
    if (!sensor->getParent() || !sensor->getParent()->isKindOf(CC_POINT_CLOUD))
    {
        ccConsole::Error("Sensor must be associated with a point cloud!");
        return;
    }

    //get associated cloud
	ccPointCloud * cloud = ccHObjectCaster::ToPointCloud(sensor->getParent());
	assert(cloud);

    //start dialog
    ccSensorComputeDistancesDlg cdDlg(this);
    if (!cdDlg.exec())
        return;

    //squared required?
    bool squared = cdDlg.computeSquaredDistances();

	//set up a new scalar field
	const char* defaultRangesSFname = squared ? CC_DEFAULT_SQUARED_RANGES_SF_NAME : CC_DEFAULT_RANGES_SF_NAME;
	int sfIdx = cloud->getScalarFieldIndexByName(defaultRangesSFname);
	if (sfIdx<0)
	{
		sfIdx = cloud->addScalarField(defaultRangesSFname);
		if (sfIdx<0)
		{
			ccConsole::Error("Not enough memory!");
			return;
		}
	}
	CCLib::ScalarField* distances = cloud->getScalarField(sfIdx);

    //sensor center
    CCVector3 center = sensor->getCenter();

	for (unsigned i=0; i<cloud->size(); ++i)
	{
		const CCVector3* P = cloud->getPoint(i);
		distances->setValue(i, squared ? (*P-center).norm2() :  (*P-center).norm());
	}

	distances->computeMinAndMax();
	cloud->setCurrentDisplayedScalarField(sfIdx);
	cloud->showSF(true);
    cloud->prepareDisplayForRefresh_recursive();

	refreshAll();
	updateUI();
}

void MainWindow::doActionComputeScatteringAngles()
{
    //there should be only one sensor in current selection!
    if (m_selectedEntities.size() != 1 || !m_selectedEntities[0]->isKindOf(CC_SENSOR))
    {
        ccConsole::Error("Select one and only one sensor!");
        return;
    }

	ccSensor* sensor = ccHObjectCaster::ToSensor(m_selectedEntities[0]);
	assert(sensor);

    //sensor must have a parent cloud with normal
    if (!sensor->getParent() || !sensor->getParent()->isKindOf(CC_POINT_CLOUD) || !sensor->getParent()->hasNormals())
    {
        ccConsole::Error("Sensor must be associated to a point cloud with normals! (compute normals first)");
        return;
    }

    //get associated cloud
	ccPointCloud * cloud = ccHObjectCaster::ToPointCloud(sensor->getParent());
	assert(cloud);

    ccSensorComputeScatteringAnglesDlg cdDlg(this);
    if (!cdDlg.exec())
        return;

    bool toDegreeFlag = cdDlg.anglesInDegrees();

    //prepare a new scalar field
	const char* defaultScatAnglesSFname = toDegreeFlag ? CC_DEFAULT_DEG_SCATTERING_ANGLES_SF_NAME : CC_DEFAULT_RAD_SCATTERING_ANGLES_SF_NAME;
	int sfIdx = cloud->getScalarFieldIndexByName(defaultScatAnglesSFname);
	if (sfIdx<0)
	{
		sfIdx = cloud->addScalarField(defaultScatAnglesSFname);
		if (sfIdx<0)
		{
			ccConsole::Error("Not enough memory!");
			return;
		}
	}
	CCLib::ScalarField* angles = cloud->getScalarField(sfIdx);

	//Sensor center
	CCVector3 sensorCenter = sensor->getCenter();

	//perform computations
	for (unsigned i=0; i<cloud->size(); ++i)
	{
		//the point position
		const CCVector3* P = cloud->getPoint(i);

		//build the ray
		CCVector3 ray = *P - sensorCenter;
		ray.normalize();

		//get the current normal
		CCVector3 normal(cloud->getPointNormal(i));
		//normal.normalize(); //should already be the case!

		//compute the angle
		float cosTheta = ray.dot(normal);
		float theta = acos(std::min<float>(fabs(cosTheta),1.0f));

		if (toDegreeFlag)
			theta *= (float)CC_RAD_TO_DEG;

		angles->setValue(i,theta);
	}

	angles->computeMinAndMax();
	cloud->setCurrentDisplayedScalarField(sfIdx);
	cloud->showSF(true);
	cloud->prepareDisplayForRefresh_recursive();

	refreshAll();
	updateUI();
}

void MainWindow::doActionProjectSensor()
{
    ccSensorProjectionDlg spDlg(this);
    if (!spDlg.exec())
        return;

    //We create the corresponding sensor for each input cloud (in a perfect world, there should be only one ;)
	ccHObject::Container selectedEntities = m_selectedEntities;
    size_t i,selNum = selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = selectedEntities[i];
        if (ent->isKindOf(CC_POINT_CLOUD))
        {
            ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);

            //we create a new sensor
            ccGBLSensor* sensor = new ccGBLSensor();

            //we init its parameters with dialog
            spDlg.updateGBLSensor(sensor);

            //we compute projection
            int errorCode;
            CCLib::GenericIndexedCloud* projectedPoints = sensor->project(cloud,errorCode,true);

            if (projectedPoints)
            {
                cloud->addChild(sensor);

                //we try to guess the sensor relative size (dirty)
                ccBBox bb = cloud->getBB();
                double diag = bb.getDiagNorm();
                if (diag < 1.0)
                    sensor->setGraphicScale(1e-3);
                else if (diag > 10000.0)
                    sensor->setGraphicScale(1e3);

                //we update sensor graphic representation
                sensor->updateGraphicRepresentation();

                //we display depth buffer
                ccRenderingTools::ShowDepthBuffer(sensor,this);

                ccGLWindow* win = static_cast<ccGLWindow*>(cloud->getDisplay());
                if (win)
                {
                    sensor->setDisplay(win);
                    sensor->setVisible(true);
                    ccBBox box = cloud->getBB();
                    win->updateConstellationCenterAndZoom(&box);
                }
                delete projectedPoints;

				addToDB(sensor,true,0,false,false);
            }
            else
            {
                displaySensorProjectErrorString(errorCode);
                delete sensor;
            }
        }
    }

    updateUI();
}

void MainWindow::doActionModifySensor()
{
    //there should be only one point cloud with sensor in current selection!
    if (m_selectedEntities.empty() || m_selectedEntities.size()>1 || !m_selectedEntities[0]->isKindOf(CC_SENSOR))
    {
        ccConsole::Error("Select one and only one sensor!");
        return;
    }

    ccSensor* sensor = static_cast<ccSensor*>(m_selectedEntities[0]);

    if (sensor->isA(CC_GBL_SENSOR))
    {
        ccGBLSensor* gbl = static_cast<ccGBLSensor*>(sensor);
        ccSensorProjectionDlg spDlg(this);

        spDlg.initWithGBLSensor(gbl);
        if (spDlg.exec())
        {
            //we init its parameters with dialog
            spDlg.updateGBLSensor(gbl);

            //we re-project cloud
            if (gbl->getParent()->isKindOf(CC_POINT_CLOUD))
            {
                int errorCode;
                ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(gbl->getParent());
                CCLib::GenericIndexedCloud* projectedPoints = gbl->project(cloud,errorCode,true);

                if (projectedPoints)
                {
                    //we don't need the projected points
                    delete projectedPoints;

                    //we update sensor graphic representation
                    gbl->updateGraphicRepresentation();

                    //we display depth buffer
                    ccRenderingTools::ShowDepthBuffer(gbl,this);

                    //in case the sensor position has changed
                    ccGLWindow* win = static_cast<ccGLWindow*>(cloud->getDisplay());
                    if (win)
                    {
                        ccBBox box = cloud->getBB();
                        win->updateConstellationCenterAndZoom(&box);
                    }

                    if (sensor->isVisible() && sensor->isEnabled())
                    {
                        sensor->prepareDisplayForRefresh();
                        refreshAll();
                    }

                    updateUI();
                }
                else
                {
                    displaySensorProjectErrorString(errorCode);
                }
            }
            else
            {
                ccConsole::Error(QString("Internal error: sensor ('%1') parent is not a point cloud!").arg(sensor->getName()));
            }
        }
    }
    else
    {
        ccConsole::Error("Can't modify this kind of sensor!");
    }
}

void MainWindow::doActionShowDepthBuffer()
{
    if (m_selectedEntities.empty())
        return;

    size_t i,selNum = m_selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = m_selectedEntities[i];
        if (ent->isKindOf(CC_GBL_SENSOR))
        {
            ccGBLSensor* sensor = static_cast<ccGBLSensor*>(m_selectedEntities[0]);

            ccRenderingTools::ShowDepthBuffer(sensor,this);
        }
    }
}

static CC_FILE_TYPES currentDBSaveDlgFilter = DM_ASCII;
void MainWindow::doActionExportDepthBuffer()
{
    if (m_selectedEntities.empty())
        return;

    //we set up file filters
    QStringList filters;
    filters << CC_FILE_TYPE_FILTERS[DM_ASCII];

    QFileDialog dialog(this);
    dialog.setFilters(filters);
    dialog.setViewMode(QFileDialog::Detail);
    dialog.setConfirmOverwrite(true);
    dialog.setAcceptMode(QFileDialog::AcceptSave);

    dialog.selectFilter(CC_FILE_TYPE_FILTERS[currentDBSaveDlgFilter]);

    QString filename = m_selectedEntities[0]->getName()+QString(".")+QString(CC_FILE_TYPE_DEFAULT_EXTENSION[currentDBSaveDlgFilter]);
    dialog.selectFile(filename);

    if (dialog.exec())
    {
        QStringList fileNames = dialog.selectedFiles();
        if (fileNames.empty())
            return;

        assert(fileNames.size()==1);

        //we try to find the selected file format
        QString filter = dialog.selectedFilter();
        CC_FILE_TYPES fType = UNKNOWN_FILE;
        for (unsigned i=0;i<(unsigned)FILE_TYPES_COUNT;++i)
        {
            if (filter == QString(CC_FILE_TYPE_FILTERS[i]))
            {
                fType = CC_FILE_TYPES_ENUMS[i];
                break;
            }
        }
        currentDBSaveDlgFilter = fType;

        ccHObject* toSave = 0;
        bool multEntities = false;
        if (m_selectedEntities.size()>1)
        {
            toSave = new ccHObject("Temp Group");
            for (unsigned i=0;i<m_selectedEntities.size();++i)
                toSave->addChild(m_selectedEntities[i],false);
            multEntities=true;
        }
        else
        {
            toSave = m_selectedEntities[0];
        }

        CC_FILE_ERROR result = FileIOFilter::SaveToFile(toSave,
														qPrintable(fileNames.at(0)),
														fType);

        if (result!=CC_FERR_NO_ERROR)
            FileIOFilter::DisplayErrorMessage(result,"saving depth buffer",fileNames.at(0));
		else
			ccLog::Print(QString("[doActionExportDepthBuffer] File '%1' successfully exported").arg(fileNames.at(0)));

        if (multEntities)
            delete toSave;
    }
}

void MainWindow::doActionConvertTextureToColor()
{
	ccHObject::Container selectedEntities = m_selectedEntities;

	for (unsigned i=0;i<selectedEntities.size();++i)
    {
        ccHObject* ent = selectedEntities[i];
        if (ent->isKindOf(CC_MESH))
        {
            ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(ent);
			assert(mesh);

			if (!mesh->hasMaterials())
			{
				ccLog::Warning(QString("[doActionConvertTextureToColor] Mesh '%1' has no material/texture!").arg(mesh->getName()));
				continue;
			}
			else
			{
				if (QMessageBox::warning(	this,
											"Mesh already has colors",
											QString("Mesh '%1' already has colors! Overwrite them?").arg(mesh->getName()),
											QMessageBox::Yes | QMessageBox::No,
											QMessageBox::No ) != QMessageBox::Yes)
				{
					continue;
				}
			

								//colorType C[3]={MAX_COLOR_COMP,MAX_COLOR_COMP,MAX_COLOR_COMP};
								//mesh->getColorFromMaterial(triIndex,*P,C,withRGB);
								//cloud->addRGBColor(C);
				if (mesh->convertMaterialsToVertexColors())
				{
					mesh->showColors(true);
					mesh->showMaterials(false);
					mesh->prepareDisplayForRefresh_recursive();
				}
				else
				{
					ccLog::Warning(QString("[doActionConvertTextureToColor] Failed to convert texture on mesh '%1'!").arg(mesh->getName()));
				}
			}
		}
	}

    refreshAll();
	updateUI();
}

static unsigned s_ptsSamplingCount = 1000000;
static double s_ptsSamplingDensity = 10.0;
void MainWindow::doActionSamplePoints()
{
	ccPtsSamplingDlg dlg(this);
	//restore last parameters
	dlg.setPointsNumber(s_ptsSamplingCount);
	dlg.setDensityValue(s_ptsSamplingDensity);
    if (!dlg.exec())
        return;

    ccProgressDialog pDlg(false,this);

    bool withNormals = dlg.generateNormals();
    bool withRGB = dlg.interpolateRGB();
    bool withTexture = dlg.interpolateTexture();
	bool useDensity = dlg.useDensity();
	assert(dlg.getPointsNumber() >= 0);
	s_ptsSamplingCount = (unsigned)dlg.getPointsNumber();
	s_ptsSamplingDensity = dlg.getDensityValue();

	bool withFeatures = (withNormals || withRGB || withTexture);
	bool errors = false;

	ccHObject::Container selectedEntities = m_selectedEntities;

	for (size_t i=0; i<selectedEntities.size() ;++i)
    {
        ccHObject* ent = selectedEntities[i];
        if (ent->isKindOf(CC_MESH))
        {
            ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(ent);
			assert(mesh);

            CCLib::GenericIndexedCloud* sampledCloud = 0;
			GenericChunkedArray<1,unsigned>* triIndices = (withFeatures ? new GenericChunkedArray<1,unsigned> : 0);

            if (useDensity)
            {
				sampledCloud = CCLib::MeshSamplingTools::samplePointsOnMesh(mesh,s_ptsSamplingDensity,&pDlg,triIndices);
            }
            else
            {
				sampledCloud = CCLib::MeshSamplingTools::samplePointsOnMesh(mesh,s_ptsSamplingCount,&pDlg,triIndices);
            }

            if (sampledCloud)
            {
				//convert to real point cloud
                ccPointCloud* cloud = ccPointCloud::From(sampledCloud);

				delete sampledCloud;
				sampledCloud=0;

				if (!cloud)
				{
					if (triIndices)
						triIndices->release();
					errors = true;
					continue;
				}

				if (withFeatures && triIndices && triIndices->currentSize() >= cloud->size())
				{
					//generate normals
					if (withNormals && mesh->hasNormals())
					{
						if (cloud->reserveTheNormsTable())
						{
							for (unsigned i=0;i<cloud->size();++i)
							{
								unsigned triIndex = triIndices->getValue(i);
								const CCVector3* P = cloud->getPoint(i);

								CCVector3 N(0.0,0.0,1.0);
								mesh->interpolateNormals(triIndex,*P,N);
								cloud->addNorm(N.u);
							}

							cloud->showNormals(true);
						}
						else
						{
							ccConsole::Error("Failed to interpolate normals (not enough memory?)");
						}
					}
					
					//generate colors
					if (withTexture && mesh->hasMaterials())
					{
						if (cloud->reserveTheRGBTable())
						{
							for (unsigned i=0;i<cloud->size();++i)
							{
								unsigned triIndex = triIndices->getValue(i);
								const CCVector3* P = cloud->getPoint(i);

								colorType C[3]={MAX_COLOR_COMP,MAX_COLOR_COMP,MAX_COLOR_COMP};
								mesh->getColorFromMaterial(triIndex,*P,C,withRGB);
								cloud->addRGBColor(C);
							}

							cloud->showColors(true);
						}
						else
						{
							ccConsole::Error("Failed to export texture colors (not enough memory?)");
						}
					}
					else if (withRGB && mesh->hasColors())
					{
						if (cloud->reserveTheRGBTable())
						{
							for (unsigned i=0;i<cloud->size();++i)
							{
								unsigned triIndex = triIndices->getValue(i);
								const CCVector3* P = cloud->getPoint(i);

								colorType C[3]={MAX_COLOR_COMP,MAX_COLOR_COMP,MAX_COLOR_COMP};
								mesh->interpolateColors(triIndex,*P,C);
								cloud->addRGBColor(C);
							}

							cloud->showColors(true);
						}
						else
						{
							ccConsole::Error("Failed to interpolate colors (not enough memory?)");
						}
					}
                }

                //we rename the resulting cloud
                cloud->setName(mesh->getName()+QString(".sampled"));
                cloud->setDisplay(mesh->getDisplay());
                cloud->prepareDisplayForRefresh();
				//copy 'shift on load' information
				if (mesh->getAssociatedCloud())
				{
					const double* shift = mesh->getAssociatedCloud()->getOriginalShift();
					if (shift)
						cloud->setOriginalShift(shift[0],shift[1],shift[2]);
				}
                addToDB(cloud,true,0,false,false);
            }

			if (triIndices)
				triIndices->release();
        }
    }

	if (errors)
		ccLog::Error("[doActionSamplePoints] Errors occured during the process! Result may be incomplete!");

    refreshAll();
}

void MainWindow::doActionFilterByValue()
{
	ccHObject::Container selectedEntities = m_selectedEntities;
    size_t i,selNum = selectedEntities.size();

    typedef std::pair<ccHObject*,ccPointCloud*> entityAndVerticesType;
    std::vector<entityAndVerticesType> toFilter;
    for (i=0;i<selNum;++i)
    {
        ccGenericPointCloud* cloud = 0;
        ccHObject* ent = selectedEntities[i];

        cloud = ccHObjectCaster::ToGenericPointCloud(ent);
        if (cloud && cloud->isA(CC_POINT_CLOUD)) // TODO
        {
            ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
            //la methode est activee sur le champ scalaire affiche
            CCLib::ScalarField* sf = pc->getCurrentDisplayedScalarField();
            if (sf)
            {
                toFilter.push_back(entityAndVerticesType(ent,pc));
            }
            else
            {
                ccConsole::Warning(QString("Entity [%1] has no active scalar field !").arg(ent->getName()));
            }
        }
    }

    double minVald = 0.0;
    double maxVald = 1.0;

    if (toFilter.empty())
        return;

    //compute min and max "displayed" scalar values of currently selected
    //entities (the ones with an active scalar field only!)
    for (i=0;i<toFilter.size();++i)
    {
        ccScalarField* sf = toFilter[i].second->getCurrentDisplayedScalarField();
        assert(sf);

        if (i==0)
        {
			minVald = (double)sf->displayRange().start();
            maxVald = (double)sf->displayRange().stop();
        }
        else
        {
            if (minVald > (double)sf->displayRange().start())
                minVald = (double)sf->displayRange().start();
            if (maxVald < (double)sf->displayRange().stop())
                maxVald = (double)sf->displayRange().stop();
        }
    }

    ccAskTwoDoubleValuesDlg dlg("Min","Max",-DBL_MAX,DBL_MAX,minVald,maxVald,8,"Filter by scalar value",this);
    if (!dlg.exec())
        return;

    ScalarType minVal = (ScalarType)dlg.doubleSpinBox1->value();
    ScalarType maxVal = (ScalarType)dlg.doubleSpinBox2->value();

    ccHObject* firstResult = 0;
    for (i=0;i<toFilter.size();++i)
    {
        ccHObject* ent = toFilter[i].first;
        ccPointCloud* pc = toFilter[i].second;
        //CCLib::ScalarField* sf = pc->getCurrentDisplayedScalarField();
        //assert(sf);

        //on met en lecture (OUT) le champ scalaire actuellement affiche
        int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
        assert(outSfIdx>=0);
        pc->setCurrentOutScalarField(outSfIdx);
        //pc->setCurrentScalarField(outSfIdx);

        ccHObject* result = 0;
        if (ent->isKindOf(CC_MESH))
        {
            pc->hidePointsByScalarValue(minVal,maxVal);
            result = ccHObjectCaster::ToGenericMesh(ent)->createNewMeshFromSelection(false);
            pc->unallocateVisibilityArray();
        }
        else if (ent->isKindOf(CC_POINT_CLOUD))
        {
            //pc->hidePointsByScalarValue(minVal,maxVal);
            //result = ccHObjectCaster::ToGenericPointCloud(ent)->hidePointsByScalarValue(false);
            //pc->unallocateVisibilityArray();

            //shortcut, as we now here that the point cloud is a "ccPointCloud"
            result = pc->filterPointsByScalarValue(minVal,maxVal);
        }

        if (result)
        {
            ent->setEnabled(false);
            result->setDisplay(ent->getDisplay());
            result->prepareDisplayForRefresh();
            addToDB(result,true,0,false,false);

            if (!firstResult)
                firstResult = result;
        }
        //*/
    }

    if (firstResult)
    {
        ccConsole::Warning("Previously selected entities (sources) have been hidden!");
		if (m_ccRoot)
			m_ccRoot->selectEntity(firstResult);
    }

    refreshAll();
}

void MainWindow::doActionSFConvertToRGB()
{
    //we first ask the user if the SF colors should be mixed with existing colors
    bool mixWithExistingColors=false;
	{
		QMessageBox::StandardButton answer = QMessageBox::warning(	this,
																	"Scalar Field to RGB",
																	"Mix with existing colors (if relevant)?",
																	QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel,
																	QMessageBox::Yes );
		if (answer == QMessageBox::Yes)
			mixWithExistingColors = true;
		else if (answer == QMessageBox::Cancel)
			return;
	}

    size_t i,selNum = m_selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccGenericPointCloud* cloud = 0;
        ccHObject* ent = m_selectedEntities[i];

		bool lockedVertices;
		cloud = ccHObjectCaster::ToPointCloud(ent,&lockedVertices);
		if (lockedVertices)
		{
			DisplayLockedVerticesWarning();
			continue;
		}
        if (cloud) //TODO
        {
            ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
            //if there is no displayed SF --> nothing to do!
            if (pc->getCurrentDisplayedScalarField())
            {
				if (pc->setRGBColorWithCurrentScalarField(mixWithExistingColors))
				{
					ent->showColors(true);
					ent->showSF(false);
				}
            }

            cloud->prepareDisplayForRefresh_recursive();
        }
    }

    refreshAll();
	updateUI();
}

void MainWindow::doActionToggleActiveSFColorScale()
{
	doApplyActiveSFAction(0);
}

void MainWindow::doActionShowActiveSFPrevious()
{
	doApplyActiveSFAction(1);
}

void MainWindow::doActionShowActiveSFNext()
{
	doApplyActiveSFAction(2);
}

void MainWindow::doApplyActiveSFAction(int action)
{
    size_t selNum = m_selectedEntities.size();
    if (selNum!=1)
    {
		if (selNum>1)
			ccConsole::Error("Select only one point cloud or mesh!");
        return;
    }
	ccHObject* ent = m_selectedEntities[0];

	bool lockedVertices;
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent,&lockedVertices);

	//for "real" point clouds only
	if (!cloud)
		return;
	if (lockedVertices)
	{
		//see ccPropertiesTreeDelegate::fillWithMesh
		if (!ent->isA(CC_MESH_GROUP) && !ent->isAncestorOf(cloud))
		{
			DisplayLockedVerticesWarning();
			return;
		}
	}

	assert(cloud);
	int sfIdx = cloud->getCurrentDisplayedScalarFieldIndex();
	switch (action)
	{
		case 0: //Toggle SF color scale
			if (sfIdx>=0)
			{
				cloud->showSFColorsScale(!cloud->sfColorScaleShown());
				cloud->prepareDisplayForRefresh();
			}
			else
				ccConsole::Warning(QString("No active scalar field on entity '%1'").arg(ent->getName()));
			break;
		case 1: //Activate previous SF
			if (sfIdx>=0)
			{
				cloud->setCurrentDisplayedScalarField(sfIdx-1);
				cloud->prepareDisplayForRefresh();
			}
			break;
		case 2: //Activate next SF
			if (sfIdx+1<(int)cloud->getNumberOfScalarFields())
			{
				cloud->setCurrentDisplayedScalarField(sfIdx+1);
				cloud->prepareDisplayForRefresh();
			}
			break;
	}

    refreshAll();
	updateUI();
}

void MainWindow::doActionRenameSF()
{
    size_t i,selNum = m_selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
		ccGenericPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_selectedEntities[i]);
        if (cloud) //TODO
        {
            ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
			ccScalarField* sf = pc->getCurrentDisplayedScalarField();
            //if there is no displayed SF --> nothing to do!
            if (!sf)
            {
				ccConsole::Warning(QString("Cloud %1 has no displayed scalar field!").arg(pc->getName()));
			}
			else
			{
				const char* sfName = sf->getName();
				ccAskOneStringDlg casd("Name",sfName ? QString(sfName) : "unknown","S.F. name",this);

				if (casd.exec())
					sf->setName(qPrintable(casd.result()));
            }
        }
    }

	updateUI();
}

void MainWindow::doActionOpenColorScalesManager()
{
	ccColorScaleEditorDialog cseDlg(ccColorScale::Shared(0), this);
	cseDlg.exec();

	updateUI();

}

void MainWindow::doActionAddIdField()
{
    size_t i,selNum = m_selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccGenericPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_selectedEntities[i]);
        if (cloud) //TODO
        {
            ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
            ccScalarField* sf = new ccScalarField;

            sf->resize(pc->size());
            sf->setName("Id");

            for (int j = 0 ; j < cloud->size(); j ++)
                sf->setValue(j, j);


            sf->computeMinAndMax();
            int id = pc->addScalarField(sf);

            pc->setCurrentScalarField(id);

        }
    }

    updateUI();
}

PointCoordinateType MainWindow::GetDefaultCloudKernelSize(const ccHObject::Container& entities)
{
	PointCoordinateType sigma = -1.0;

	size_t i,selNum = entities.size();
	//computation of a first sigma guess
	for (i=0;i<selNum;++i)
	{
		ccPointCloud* pc = ccHObjectCaster::ToPointCloud(entities[i]);
		if (pc && pc->size()>0)
		{
			//we get 1% of the cloud bounding box
			//and we divide by the number of points / 10e6 (so that the kernel for a 20 M. points cloud is half the one of a 10 M. cloud)
			PointCoordinateType sigmaCloud = pc->getBB().getDiagNorm()*(PointCoordinateType)(0.01/std::max(1.0,1.0e-7*(double)pc->size()));

			//we keep the smallest value
			if (sigma<0.0 || sigmaCloud<sigma)
				sigma=sigmaCloud;
		}
	}

	return sigma;
}

void MainWindow::doActionSFGaussianFilter()
{
    size_t i,selNum = m_selectedEntities.size();
    if (selNum==0)
        return;

	double sigma = GetDefaultCloudKernelSize(m_selectedEntities);
	if (sigma<0.0)
	{
		ccConsole::Error("No elligible point cloud in selection!");
		return;
	}

	ccAskOneDoubleValueDlg dlg("Sigma", DBL_MIN, DBL_MAX, sigma, 8, 0, this);
    dlg.dValueSpinBox->setStatusTip("3*sigma = 98% attenuation");
    if (!dlg.exec())
        return;

    sigma = dlg.dValueSpinBox->value();

    for (i=0;i<selNum;++i)
    {
		bool lockedVertices;
		ccPointCloud* pc = ccHObjectCaster::ToPointCloud(m_selectedEntities[i],&lockedVertices);
		if (!pc || lockedVertices)
		{
			DisplayLockedVerticesWarning();
			continue;
		}

        //la methode est activee sur le champ scalaire affiche
        CCLib::ScalarField* sf = pc->getCurrentDisplayedScalarField();
        if (sf)
        {
            //on met en lecture (OUT) le champ scalaire actuellement affiche
            int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
            assert(outSfIdx>=0);

            pc->setCurrentOutScalarField(outSfIdx);
            CCLib::ScalarField* outSF = pc->getCurrentOutScalarField();
            assert(sf);

            QString sfName = QString("%1.smooth(%2)").arg(outSF->getName()).arg(sigma);
            int sfIdx = pc->getScalarFieldIndexByName(qPrintable(sfName));
            if (sfIdx<0)
                sfIdx = pc->addScalarField(qPrintable(sfName)); //output SF has same type as input SF
            if (sfIdx>=0)
                pc->setCurrentInScalarField(sfIdx);
            else
            {
                ccConsole::Error(QString("Failed to create scalar field for cloud '%1' (not enough memory?)").arg(pc->getName()));
                continue;
            }

            ccOctree* octree = pc->getOctree();
            if (!octree)
			{
				ccProgressDialog pDlg(true,this);
                octree = pc->computeOctree(&pDlg);
				if (!octree)
				{
					ccConsole::Error(QString("Couldn't compute octree for cloud '%1'!").arg(pc->getName()));
					continue;
				}
			}

            if (octree)
            {
                ccProgressDialog pDlg(true,this);
				QElapsedTimer eTimer;
				eTimer.start();
                CCLib::ScalarFieldTools::applyScalarFieldGaussianFilter(sigma,pc,-1,&pDlg, octree);
				ccConsole::Print("[GaussianFilter] Timing: %3.2f s.",eTimer.elapsed()/1.0e3);
                pc->setCurrentDisplayedScalarField(sfIdx);
				pc->showSF(sfIdx>=0);
                sf = pc->getCurrentDisplayedScalarField();
                if (sf)
                    sf->computeMinAndMax();
                pc->prepareDisplayForRefresh_recursive();
            }
            else
            {
                ccConsole::Error(QString("Failed to compute entity [%1] octree! (not enough memory?)").arg(pc->getName()));
            }
        }
        else
        {
            ccConsole::Warning(QString("Entity [%1] has no active scalar field!").arg(pc->getName()));
        }
    }

    refreshAll();
	updateUI();
}

void MainWindow::doActionSFBilateralFilter()
{
    size_t i,selNum = m_selectedEntities.size();
    if (selNum==0)
        return;

    double sigma = GetDefaultCloudKernelSize(m_selectedEntities);
    if (sigma<0.0)
    {
        ccConsole::Error("No elligible point cloud in selection!");
        return;
    }

    //estimate a good value for scalar field sigma, based on the first cloud
    //and its displayed scalar field
    ccPointCloud* pc_test = ccHObjectCaster::ToPointCloud(m_selectedEntities[0]);
    CCLib::ScalarField* sf_test = pc_test->getCurrentDisplayedScalarField();
    ScalarType range = sf_test->getMax() - sf_test->getMin();
    ScalarType scalarFieldSigma = range / 4.0f; // using 1/4 of total range


    ccAskTwoDoubleValuesDlg dlg("Spatial sigma", "Scalar sigma", DBL_MIN, DBL_MAX, sigma, scalarFieldSigma , 8, 0, this);
    dlg.doubleSpinBox1->setStatusTip("3*sigma = 98% attenuation");
    dlg.doubleSpinBox2->setStatusTip("Scalar field's sigma controls how much the filter behaves as a Gaussian Filter\n sigma at +inf uses the whole range of scalars ");
    if (!dlg.exec())
        return;

    //get values
    sigma = dlg.doubleSpinBox1->value();
    scalarFieldSigma = dlg.doubleSpinBox2->value();

    for (i=0;i<selNum;++i)
    {
        bool lockedVertices;
        ccPointCloud* pc = ccHObjectCaster::ToPointCloud(m_selectedEntities[i],&lockedVertices);
        if (!pc || lockedVertices)
        {
            DisplayLockedVerticesWarning();
            continue;
        }

        //the algorithm will use the currently displayed SF
        CCLib::ScalarField* sf = pc->getCurrentDisplayedScalarField();
        if (sf)
        {
            //we set the displayed SF as "OUT" SF
            int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
            assert(outSfIdx>=0);

            pc->setCurrentOutScalarField(outSfIdx);
            CCLib::ScalarField* outSF = pc->getCurrentOutScalarField();
            assert(sf);

            QString sfName = QString("%1.bilsmooth(%2,%3)").arg(outSF->getName()).arg(sigma).arg(scalarFieldSigma);
            int sfIdx = pc->getScalarFieldIndexByName(qPrintable(sfName));
            if (sfIdx<0)
                sfIdx = pc->addScalarField(qPrintable(sfName)); //output SF has same type as input SF
            if (sfIdx>=0)
                pc->setCurrentInScalarField(sfIdx);
            else
            {
                ccConsole::Error(QString("Failed to create scalar field for cloud '%1' (not enough memory?)").arg(pc->getName()));
                continue;
            }

            ccOctree* octree = pc->getOctree();
            if (!octree)
            {
                ccProgressDialog pDlg(true,this);
                octree = pc->computeOctree(&pDlg);
                if (!octree)
                {
                    ccConsole::Error(QString("Couldn't compute octree for cloud '%1'!").arg(pc->getName()));
                    continue;
                }
            }

			assert(octree);
            {
                ccProgressDialog pDlg(true,this);
                QElapsedTimer eTimer;
                eTimer.start();

                CCLib::ScalarFieldTools::applyScalarFieldGaussianFilter(sigma,pc, scalarFieldSigma,&pDlg,octree);
                ccConsole::Print("[BilateralFilter] Timing: %3.2f s.",eTimer.elapsed()/1.0e3);
                pc->setCurrentDisplayedScalarField(sfIdx);
                pc->showSF(sfIdx>=0);
                sf = pc->getCurrentDisplayedScalarField();
                if (sf)
                    sf->computeMinAndMax();
                pc->prepareDisplayForRefresh_recursive();
            }
        }
        else
        {
            ccConsole::Warning(QString("Entity [%1] has no active scalar field!").arg(pc->getName()));
        }
    }

    refreshAll();
	updateUI();
}

void MainWindow::doActionSmoothMeshSF()
{
    doMeshSFAction(ccGenericMesh::SMOOTH_MESH_SF);
}

void MainWindow::doActionEnhanceMeshSF()
{
    doMeshSFAction(ccGenericMesh::ENHANCE_MESH_SF);
}

void MainWindow::doMeshSFAction(ccGenericMesh::MESH_SCALAR_FIELD_PROCESS process)
{
    ccProgressDialog pDlg(false,this);

    size_t i,selNum = m_selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = m_selectedEntities[i];
        if (ent->isKindOf(CC_MESH))
        {
            ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(ent);
            ccGenericPointCloud* cloud = mesh->getAssociatedCloud();

            if (cloud && cloud->isA(CC_POINT_CLOUD)) //TODO
            {
                ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

                //on active le champ scalaire actuellement affiche
                int sfIdx = pc->getCurrentDisplayedScalarFieldIndex();
                if (sfIdx>=0)
                {
                    pc->setCurrentScalarField(sfIdx);
                    mesh->processScalarField(process);
                    pc->getCurrentInScalarField()->computeMinAndMax();
                    mesh->prepareDisplayForRefresh_recursive();
                }
                else
                {
                    ccConsole::Warning(QString("Mesh [%1] vertices have no activated scalar field!").arg(mesh->getName()));
                }
            }
        }
    }

    refreshAll();
	updateUI();
}

static float s_subdivideMaxArea = 1.0f;
void MainWindow::doActionSubdivideMesh()
{
	bool ok;
	s_subdivideMaxArea = QInputDialog::getDouble(this, "Subdivide mesh", "Max area per triangle:", s_subdivideMaxArea, 1e-6, 1e6, 8, &ok);
	if (!ok)
		return;

	//ccProgressDialog pDlg(true,this);

    size_t i,selNum = m_selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = m_selectedEntities[i];
        if (ent->isKindOf(CC_MESH))
        {
			//single mesh?
			if (ent->isA(CC_MESH))
			{
				ccMesh* mesh = static_cast<ccMesh*>(ent);

				ccMesh* subdividedMesh = 0;
				try
				{
					subdividedMesh = mesh->subdivide(s_subdivideMaxArea);
				}
				catch(...)
				{
					ccLog::Error(QString("[Subdivide] An error occured while trying to subdivide mesh '%1' (not enough memory?)").arg(mesh->getName()));
				}

				if (subdividedMesh)
				{
					subdividedMesh->setName(QString("%1.subdivided(S<%2)").arg(mesh->getName()).arg(s_subdivideMaxArea));
					mesh->setEnabled(false);
					mesh->refreshDisplay_recursive();
					addToDB(subdividedMesh, true, 0, true, false);
				}
				else
				{
					ccConsole::Warning(QString("[Subdivide] Failed to subdivide mesh '%1' (not enough memory?)").arg(mesh->getName()));
				}
			}
			else
			{
				ccLog::Warning("[Subdivide] Works only on single meshes!");
			}
		}
    }

    refreshAll();
	updateUI();
}

static unsigned s_laplacianSmooth_nbIter = 20;
static float    s_laplacianSmooth_factor = 0.2f;
void MainWindow::doActionSmoothMeshLaplacian()
{
	bool ok;
	s_laplacianSmooth_nbIter = QInputDialog::getInt(this, "Smooth mesh", "Iterations:", s_laplacianSmooth_nbIter, 1, 1000, 1, &ok);
	if (!ok)
		return;
	s_laplacianSmooth_factor = QInputDialog::getDouble(this, "Smooth mesh", "Smoothing factor:", s_laplacianSmooth_factor, 0, 100, 3, &ok);
	if (!ok)
		return;

	ccProgressDialog pDlg(true,this);

    size_t i,selNum = m_selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = m_selectedEntities[i];
        if (ent->isKindOf(CC_MESH))
        {
            ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(ent);

			if (mesh->laplacianSmooth(s_laplacianSmooth_nbIter, s_laplacianSmooth_factor,&pDlg))
			{
				mesh->prepareDisplayForRefresh_recursive();
			}
			else
			{
				ccConsole::Warning(QString("Failed to apply Laplacian smoothing to mesh '%1'").arg(mesh->getName()));
            }
        }
    }

    refreshAll();
	updateUI();
}

void MainWindow::RemoveSiblingsFromCCObjectList(ccHObject::Container& ccObjects)
{
    ccHObject::Container keptObjects;
    size_t count=ccObjects.size();
    keptObjects.reserve(count);

    for (size_t i=0;i<count;++i)
    {
        ccHObject* obj = ccObjects[i];
        assert(obj);
        for (size_t j=0;j<count;++j)
        {
            if (i!=j)
            {
                if (ccObjects[j]->isAncestorOf(obj))
                {
                    obj=0;
                    break;
                }
            }
        }

        if (obj)
            keptObjects.push_back(obj);
        else
            ccObjects[i]=0;
    }

    ccObjects = keptObjects;
}

void MainWindow::doActionFuse()
{
    ccPointCloud* firstCloud=0;
	ccHObject* firstCloudParent=0;

    //we deselect all selected entities (as they are going to disappear)
    ccHObject::Container _selectedEntities = m_selectedEntities;
	if (m_ccRoot)
		m_ccRoot->selectEntity(0);

	//we will remove the useless clouds later
	ccHObject::Container toBeRemoved;

    size_t i,selNum = _selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = _selectedEntities[i];
		if (!ent)
			continue;

        //point clouds are simply added to the first selected ones
        //and then removed
        if (ent->isKindOf(CC_POINT_CLOUD))
        {
            ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);

            if (cloud && cloud->isA(CC_POINT_CLOUD)) //TODO
            {
                ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

                if (!firstCloud)
				{
                    firstCloud=pc;
					//we temporarily detach the first cloud, as it may undergo
					//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::operator +=
					removeObjectTemporarilyFromDBTree(firstCloud,firstCloudParent);
				}
                else
                {
                    unsigned beforePts = firstCloud->size();
                    unsigned newPts = pc->size();
                    *firstCloud += pc;

                    //success?
                    if (firstCloud->size() == beforePts + newPts)
					{
						firstCloud->prepareDisplayForRefresh_recursive();

						ccHObject* toRemove = 0;
						//if the entity to remove is a group with a unique child, we can remove it as well
						ccHObject* parent = pc->getParent();
						if (parent && parent->isA(CC_HIERARCHY_OBJECT) && parent->getChildrenNumber()==1 && parent!=firstCloudParent)
							toRemove = parent;
						else
							toRemove = pc;

						//is a parent or sibling already in the "toBeRemoved" list?
						int j=0,count=(int)toBeRemoved.size();
						while (j<count)
						{
							if (toBeRemoved[j]->isAncestorOf(toRemove))
							{
								toRemove=0;
								break;
							}
							else if (toRemove->isAncestorOf(toBeRemoved[j]))
							{
								toBeRemoved[j]=toBeRemoved.back();
								toBeRemoved.pop_back();
								count--;
								j++;
							}
							else
							{
								//forward
								j++;
							}
						}
						if (toRemove)
							toBeRemoved.push_back(toRemove);
					}
					else
					{
						ccConsole::Error("Fusion failed! (not enough memory?)");
						break;
					}
					pc=0;
				}
            }
        }
        //meshes are placed in a common mesh group
        else if (ent->isKindOf(CC_MESH))
        {
            ccConsole::Warning("Can't fuse meshes yet! Sorry ...");
        }
		else
		{
			ccConsole::Warning(QString("Entity '%1' is neither a cloud nor a mesh, can't fuse it!").arg(ent->getName()));
		}

		//security (we don't want to encounter it again)
		_selectedEntities[i]=0;
    }

	//something to remove?
	while (!toBeRemoved.empty())
	{
		if (toBeRemoved.back() && m_ccRoot)
			m_ccRoot->removeElement(toBeRemoved.back());
		toBeRemoved.pop_back();
	}

	if (firstCloud)
		putObjectBackIntoDBTree(firstCloud,firstCloudParent);

    refreshAll();
	updateUI();
}

void MainWindow::zoomOn(ccDrawableObject* object)
{
    ccGLWindow* win = static_cast<ccGLWindow*>(object->getDisplay());
    if (win)
    {
        ccBBox box = object->getBB(true,false,win);
        win->updateConstellationCenterAndZoom(&box);
    }
}

void MainWindow::doActionRegister()
{
    if (m_selectedEntities.size() != 2 ||
            (!m_selectedEntities[0]->isKindOf(CC_POINT_CLOUD) && !m_selectedEntities[0]->isKindOf(CC_MESH)) ||
            (!m_selectedEntities[1]->isKindOf(CC_POINT_CLOUD) && !m_selectedEntities[1]->isKindOf(CC_MESH)))
    {
        ccConsole::Error("Select 2 point clouds or meshes!");
        return;
    }

    ccHObject *data = static_cast<ccHObject*>(m_selectedEntities[1]);
    ccHObject *model = static_cast<ccHObject*>(m_selectedEntities[0]);

    ccRegistrationDlg rDlg(data,model,this);
    if (!rDlg.exec())
        return;

    //DGM (23/01/09): model and data order may have changed!
    model = rDlg.getModelEntity();
    data = rDlg.getDataEntity();

    //progress bar
    ccProgressDialog pDlg(false,this);

    //if the 'model' entity is a mesh, we need to sample points on it
    CCLib::GenericIndexedCloudPersist* modelCloud = 0;
    if (model->isKindOf(CC_MESH))
    {
        modelCloud = CCLib::MeshSamplingTools::samplePointsOnMesh(ccHObjectCaster::ToGenericMesh(model),(unsigned)100000,&pDlg);
        if (!modelCloud)
        {
            ccConsole::Error("Failed to sample points on 'model' mesh!");
            return;
        }
    }
    else
    {
        modelCloud = ccHObjectCaster::ToGenericPointCloud(model);
    }

    //if the 'data' entity is a mesh, we need to sample points on it
    CCLib::GenericIndexedCloudPersist* dataCloud = 0;
    if (data->isKindOf(CC_MESH))
    {
        dataCloud = CCLib::MeshSamplingTools::samplePointsOnMesh(ccHObjectCaster::ToGenericMesh(data),(unsigned)50000,&pDlg);
        if (!dataCloud)
        {
            ccConsole::Error("Failed to sample points on 'data' mesh!");
            return;
        }
    }
    else
    {
        dataCloud = ccHObjectCaster::ToGenericPointCloud(data);
    }

    //we activate a temporary scalar field for registration distances computation
	CCLib::ScalarField* dataDisplayedSF = 0;
    int oldDataSfIdx=-1, dataSfIdx=-1;

    //if the 'data' entity is a real ccPointCloud, we can even create a temporary SF for registration distances
    if (data->isA(CC_POINT_CLOUD))
    {
        ccPointCloud* pc = static_cast<ccPointCloud*>(data);
		dataDisplayedSF = pc->getCurrentDisplayedScalarField();
        oldDataSfIdx = pc->getCurrentInScalarFieldIndex();
        dataSfIdx = pc->getScalarFieldIndexByName("RegistrationDistances");
        if (dataSfIdx<0)
            dataSfIdx=pc->addScalarField("RegistrationDistances");
        if (dataSfIdx>=0)
            pc->setCurrentScalarField(dataSfIdx);
        else
            ccConsole::Warning("Couldn't create temporary scalar field! Not enough memory?");
    }
    else
    {
        dataCloud->enableScalarField();
    }

    //parameters
    CCLib::PointProjectionTools::Transformation transform;

    double minErrorDecrease			= rDlg.getMinErrorDecrease();
    unsigned maxIterationCount		= rDlg.getMaxIterationCount();
	unsigned randomSamplingLimit	= rDlg.randomSamplingLimit();
    bool removeFarthestPoints		= rDlg.removeFarthestPoints();
    ConvergenceMethod method		= rDlg.getConvergenceMethod();
	bool useDataSFAsWeights			= rDlg.useDataSFAsWeights();
    bool useModelSFAsWeights		= rDlg.useModelSFAsWeights();
    bool useScaleFree               = rDlg.useFreeScaleParameter();
    double finalError				= 0.0;

	CCLib::ScalarField* modelWeights = 0;
	if (useModelSFAsWeights)
	{
		if (modelCloud==(CCLib::GenericIndexedCloudPersist*)model && model->isA(CC_POINT_CLOUD))
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(model);
			modelWeights = pc->getCurrentDisplayedScalarField();
			if (!modelWeights)
				ccConsole::Warning("[MainWindow::doActionRegister] Model has no displayed scalar field!");
		}
		else
		{
			ccConsole::Warning("[MainWindow::doActionRegister] Only point clouds scalar fields can be used as weights!");
		}
	}

	CCLib::ScalarField* dataWeights = 0;
	if (useDataSFAsWeights)
	{
		if (!dataDisplayedSF)
		{
			if (dataCloud==(CCLib::GenericIndexedCloudPersist*)data && data->isA(CC_POINT_CLOUD))
				ccConsole::Warning("[MainWindow::doActionRegister] Data has no displayed scalar field!");
			else
				ccConsole::Warning("[MainWindow::doActionRegister] Only point clouds scalar fields can be used as weights!");
		}
		else
			dataWeights = dataDisplayedSF;

	}

    CCLib::ICPRegistrationTools::CC_ICP_RESULT result;
    result = CCLib::ICPRegistrationTools::RegisterClouds(modelCloud,
             dataCloud,
             transform,
             method,
             minErrorDecrease,
             maxIterationCount,
             finalError,
             useScaleFree,
             (CCLib::GenericProgressCallback*)&pDlg,
             removeFarthestPoints,
			 randomSamplingLimit,
			 modelWeights,
			 dataWeights);

    if (result >= CCLib::ICPRegistrationTools::ICP_ERROR)
    {
        ccConsole::Error("Registration failed: an error occured (code %i)",result);
    }
    else if (result == CCLib::ICPRegistrationTools::ICP_APPLY_TRANSFO)
    {
        ccConsole::Print("[Register] Convergence reached (RMS at last step: %f)",finalError);

        ccGLMatrix transMat(transform.R,transform.T, transform.s);

		forceConsoleDisplay();

		if (useScaleFree)
			ccConsole::Print("[Register] Scale: %f",transform.s);

		ccConsole::Print("[Register] Resulting matrix:");
		{
			const float* mat = transMat.data();
			ccConsole::Print("%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f",mat[0],mat[4],mat[8],mat[12],mat[1],mat[5],mat[9],mat[13],mat[2],mat[6],mat[10],mat[14],mat[3],mat[7],mat[11],mat[15]);
			ccConsole::Print("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool");
		}

        //cloud to move
        ccGenericPointCloud* pc = 0;

        if (data->isKindOf(CC_POINT_CLOUD))
        {
            pc = ccHObjectCaster::ToGenericPointCloud(data);
        }
        else if (data->isKindOf(CC_MESH))
        {
            ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(data);
            pc = mesh->getAssociatedCloud();

            //warning: point cloud is locked!
            if (pc->isLocked())
            {
                pc = 0;
                //we ask the user about cloning the 'data' mesh
                QMessageBox::StandardButton result = QMessageBox::question(this,
                                                     "Registration",
                                                     "Data mesh vertices are locked (they may be shared with other meshes): Do you wish to clone this mesh to apply transformation?",
                                                     QMessageBox::Ok | QMessageBox::Cancel,
                                                     QMessageBox::Ok);

                //continue process?
                if (result == QMessageBox::Ok)
                {
                    ccGenericMesh* newMesh = mesh->clone();

                    if (newMesh)
                    {
                        addToDB(newMesh, true, 0, true, false);
                        data = newMesh;
                        pc = newMesh->getAssociatedCloud();
                    }
                    else
                    {
                        ccConsole::Error("Failed to clone 'data' mesh! (not enough memory?)");
                    }
                }
            }
        }

        //if we managed to get a point cloud to move!
        if (pc)
        {
			//we temporarily detach cloud, as it may undergo
			//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::applyRigidTransformation
			ccHObject* parent=0;
			removeObjectTemporarilyFromDBTree(pc,parent);
			pc->applyRigidTransformation(transMat);
			putObjectBackIntoDBTree(pc,parent);

            //don't forget to update mesh bounding box also!
            if (data->isKindOf(CC_MESH))
                ccHObjectCaster::ToGenericMesh(data)->refreshBB();

            data->prepareDisplayForRefresh_recursive();
            data->setName(data->getName()+QString(".registered"));
            zoomOn(data);
        }
    }

    //if we had to sample points an the data mesh
    if (!data->isKindOf(CC_POINT_CLOUD))
    {
        delete dataCloud;
    }
    else
    {
        if (data->isA(CC_POINT_CLOUD))
        {
            ccPointCloud* pc = static_cast<ccPointCloud*>(data);
            pc->setCurrentScalarField(oldDataSfIdx);
            if (dataSfIdx>=0)
                pc->deleteScalarField(dataSfIdx);
            dataSfIdx=-1;
        }
    }

    //if we had to sample points an the model mesh
    if (!model->isKindOf(CC_POINT_CLOUD))
        delete modelCloud;

    refreshAll();
	updateUI();
}

//Aurelien BEY le 13/11/2008 : ajout de la fonction permettant de traiter la fonctionnalite de recalage grossier
void MainWindow::doAction4pcsRegister()
{
    if (QMessageBox::warning(	this,
								"Work in progress",
								"This method is still under development: are you sure you want to use it? (a crash may likely happen)",
								QMessageBox::Yes,QMessageBox::No) == QMessageBox::No )
        return;

    if (m_selectedEntities.size() != 2)
    {
        ccConsole::Error("Select 2 point clouds!");
        return;
    }

    if (!m_selectedEntities[0]->isKindOf(CC_POINT_CLOUD) ||
            !m_selectedEntities[1]->isKindOf(CC_POINT_CLOUD))
    {
        ccConsole::Error("Select 2 point clouds!");
        return;
    }

    ccGenericPointCloud *model = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities[0]);
    ccGenericPointCloud *data = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities[1]);

    ccAlignDlg aDlg(model, data);
    if (!aDlg.exec())
        return;

    model = aDlg.getModelObject();
    data = aDlg.getDataObject();

    //Take the correct number of points among the clouds
    CCLib::ReferenceCloud *subModel = aDlg.getSampledModel();
    CCLib::ReferenceCloud *subData = aDlg.getSampledData();

    unsigned nbMaxCandidates = aDlg.isNumberOfCandidatesLimited() ? aDlg.getMaxNumberOfCandidates() : 0;

    ccProgressDialog pDlg(true,this);

    CCLib::PointProjectionTools::Transformation transform;
    if (CCLib::FPCSRegistrationTools::RegisterClouds(subModel, subData, transform, aDlg.getDelta(), aDlg.getDelta()/2, aDlg.getOverlap(), aDlg.getNbTries(), 5000, &pDlg, nbMaxCandidates))
    {
		//output resulting transformation matrix
		{
			ccGLMatrix transMat(transform.R,transform.T);
			forceConsoleDisplay();
			ccConsole::Print("[Align] Resulting matrix:");
			const float* mat = transMat.data();
			ccConsole::Print("%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f",mat[0],mat[4],mat[8],mat[12],mat[1],mat[5],mat[9],mat[13],mat[2],mat[6],mat[10],mat[14],mat[3],mat[7],mat[11],mat[15]);
			ccConsole::Print("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool");
		}

		ccPointCloud *newDataCloud=0;
		if (data->isA(CC_POINT_CLOUD))
		{
            newDataCloud = static_cast<ccPointCloud*>(data)->cloneThis();
		}
        else
		{
            newDataCloud = ccPointCloud::From(data);
			const double* shift = data->getOriginalShift();
			if (shift)
				newDataCloud->setOriginalShift(shift[0],shift[1],shift[2]);
		}

        if (data->getParent())
            data->getParent()->addChild(newDataCloud);
        newDataCloud->setName(data->getName()+QString(".registered"));
        newDataCloud->applyTransformation(transform);
        newDataCloud->setDisplay(data->getDisplay());
        newDataCloud->prepareDisplayForRefresh();
        zoomOn(newDataCloud);
        addToDB(newDataCloud, true, 0, true, false);

        data->setEnabled(false);
        data->prepareDisplayForRefresh_recursive();
    }
	else
	{
		ccConsole::Warning("[Align] Registration failed!");
	}

	if (subModel)
		delete subModel;
	if (subData)
		delete subData;

    refreshAll();
	updateUI();
}

//Aurelien BEY le 4/12/2008 : ajout de la fonction de sous echantillonage de nuages de points
void MainWindow::doActionSubsample()
{
    if (m_selectedEntities.size() != 1 || !m_selectedEntities[0]->isA(CC_POINT_CLOUD)) //TODO
    {
        ccConsole::Error("Select 1 point cloud!");
        return;
    }

    ccPointCloud *pointCloud = static_cast<ccPointCloud*>(m_selectedEntities[0]);
    ccSubsamplingDlg sDlg(pointCloud);

    if (!sDlg.exec())
        return;

    ccProgressDialog pDlg(false,this);
    pDlg.setMethodTitle("Subsampling");

    CCLib::ReferenceCloud *sampledCloud = sDlg.getSampledCloud(&pDlg);
    if (!sampledCloud)
    {
        ccConsole::Error("An internal error occured: failed to sample cloud!");
        return;
    }

	int warnings = 0;
    ccPointCloud *newPointCloud = pointCloud->partialClone(sampledCloud,&warnings);
	if (newPointCloud)
	{
		newPointCloud->setName(pointCloud->getName()+QString(".subsampled"));
		newPointCloud->setDisplay(pointCloud->getDisplay());
		const double* shift = pointCloud->getOriginalShift();
		if (shift)
			newPointCloud->setOriginalShift(shift[0],shift[1],shift[2]);
		newPointCloud->prepareDisplayForRefresh();
		if (pointCloud->getParent())
			pointCloud->getParent()->addChild(newPointCloud);
		newPointCloud->setDisplay(pointCloud->getDisplay());
		pointCloud->setVisible(false);
		addToDB(newPointCloud, true, 0, false, false);

		newPointCloud->refreshDisplay();

		if (warnings)
			ccLog::Error("Not enough memory: colors, normals or scalar fields may be missing!");
	}
	else
	{
		ccLog::Error("Not enough memory!");
	}

    delete sampledCloud;

    refreshAll();
	updateUI();
}

void MainWindow::doActionStatisticalTest()
{
    ccPickOneElementDlg pDlg("Distribution","Choose distribution",this);
    pDlg.addElement("Gauss");
    pDlg.addElement("Weibull");
    pDlg.setDefaultIndex(0);
    if (!pDlg.exec())
        return;

    int distribIndex = pDlg.getSelectedIndex();

    ccStatisticalTestDlg* sDlg=0;
    switch (distribIndex)
    {
    case 0: //Gauss
        sDlg = new ccStatisticalTestDlg("mu","sigma",QString(),"Local Statistical Test (Gauss)",this);
        break;
    case 1: //Weibull
        sDlg = new ccStatisticalTestDlg("a","b","shift","Local Statistical Test (Weibull)",this);
        break;
    default:
        ccConsole::Error("Invalid distribution!");
		return;
    }

    if (!sDlg->exec())
    {
        delete sDlg;
        return;
    }

	//build up corresponding distribution
    CCLib::GenericDistribution* distrib=0;
	{
		double a = sDlg->getParam1();
		double b = sDlg->getParam2();
		double c = sDlg->getParam3();

		switch (distribIndex)
		{
		case 0: //Gauss
		{
			CCLib::NormalDistribution* N = new CCLib::NormalDistribution();
			N->setParameters(a,b*b); //warning: we input sigma2 here (not sigma)
			distrib = static_cast<CCLib::GenericDistribution*>(N);
			break;
		}
		case 1: //Weibull
			CCLib::WeibullDistribution* W = new CCLib::WeibullDistribution();
			W->setParameters(a,b,c);
			distrib = static_cast<CCLib::GenericDistribution*>(W);
			break;
		}
	}

    size_t i,selNum = m_selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccPointCloud* pc = ccHObjectCaster::ToPointCloud(m_selectedEntities[i]); //TODO
        if (pc)
        {
            //we apply method on currently displayed SF
            ccScalarField* inSF = pc->getCurrentDisplayedScalarField();
            if (inSF)
            {
                assert(inSF->isAllocated());

                //force SF as 'OUT' field (in case of)
				int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
				pc->setCurrentOutScalarField(outSfIdx);

				//force Chi2 Distances field as 'IN' field (create it by the way if necessary)
				int chi2SfIdx = pc->getScalarFieldIndexByName(CC_CHI2_DISTANCES_DEFAULT_SF_NAME);
				if (chi2SfIdx<0)
					chi2SfIdx=pc->addScalarField(CC_CHI2_DISTANCES_DEFAULT_SF_NAME);
				if (chi2SfIdx<0)
				{
					ccConsole::Error("Couldn't allocate a new scalar field for computing chi2 distances! Try to free some memory ...");
					break;
				}
				pc->setCurrentInScalarField(chi2SfIdx);

				//compute octree if necessary
				ccOctree* theOctree=pc->getOctree();
				if (!theOctree)
				{
					ccProgressDialog pDlg(true,this);
					theOctree = pc->computeOctree(&pDlg);
					if (!theOctree)
					{
						ccConsole::Error(QString("Couldn't compute octree for cloud '%1'!").arg(pc->getName()));
						break;
					}
				}

				ccProgressDialog pDlg(true,this);

				double pChi2 = sDlg->getProba();
				int nn = sDlg->getNeighborsNumber();

				QElapsedTimer eTimer;
				eTimer.start();

				double chi2dist = CCLib::StatisticalTestingTools::testCloudWithStatisticalModel(distrib,pc,nn,pChi2,&pDlg,theOctree);
				
				ccConsole::Print("[Chi2 Test] Timing: %3.2f ms.",eTimer.elapsed()/1.0e3);
				ccConsole::Print("[Chi2 Test] %s test result = %f",distrib->getName(),chi2dist);

				//we set the theoretical Chi2 distance limit as the minimumed displayed SF value so that all points below are grayed
				{
					ccScalarField* chi2SF = static_cast<ccScalarField*>(pc->getCurrentInScalarField());
					chi2SF->computeMinAndMax();
					chi2dist *= chi2dist;
					chi2SF->setMinDisplayed(chi2dist);
					chi2SF->setSymmetricalScale(false);
					chi2SF->setSaturationStart(chi2dist);
					//chi2SF->setSaturationStop(chi2dist);
					pc->setCurrentDisplayedScalarField(chi2SfIdx);
					pc->showSF(true);
					pc->prepareDisplayForRefresh_recursive();
				}
			}
		}
    }

    delete distrib;
    delete sDlg;

    refreshAll();
	updateUI();
}

void MainWindow::doActionComputeStatParams()
{
    ccPickOneElementDlg pDlg("Distribution","Distribution Fitting",this);
    pDlg.addElement("Gauss");
    pDlg.addElement("Weibull");
    pDlg.setDefaultIndex(0);
    if (!pDlg.exec())
        return;

    CCLib::GenericDistribution* distrib=0;
	{
		switch (pDlg.getSelectedIndex())
		{
		case 0: //GAUSS
			distrib = new CCLib::NormalDistribution();
			break;
		case 1: //WEIBULL
			distrib = new CCLib::WeibullDistribution();
			break;
		default:
			assert(false);
			return;
		}
	}
    assert(distrib);

    size_t i,selNum = m_selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccPointCloud* pc = ccHObjectCaster::ToPointCloud(m_selectedEntities[i]); //TODO
        if (pc)
        {
            //we apply method on currently displayed SF
            ccScalarField* sf = pc->getCurrentDisplayedScalarField();
            if (sf)
            {
                assert(sf->isAllocated());

                //force SF as 'OUT' field (in case of)
                int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
				assert(outSfIdx>=0);
                pc->setCurrentOutScalarField(outSfIdx);

                if (distrib->computeParameters(pc))
                {
					QString description;

					unsigned precision = ccGui::Parameters().displayedNumPrecision;
					switch (pDlg.getSelectedIndex())
					{
					case 0: //GAUSS
						{
							CCLib::NormalDistribution* normal = static_cast<CCLib::NormalDistribution*>(distrib);
							description = QString("mean = %1 / std.dev. = %2").arg(normal->getMu(),0,'f',precision).arg(sqrt(normal->getSigma2()),0,'f',precision);
						}
						break;
					case 1: //WEIBULL
						{
							CCLib::WeibullDistribution* weibull = static_cast<CCLib::WeibullDistribution*>(distrib);
							ScalarType a,b;
							weibull->getParameters(a,b);
							description = QString("a = %1 / b = %2 / shift = %3").arg(a,0,'f',precision).arg(b,0,'f',precision).arg(weibull->getValueShift(),0,'f',precision);
						}
						break;
					default:
						assert(false);
						return;
					}
					description.prepend(QString("%1: ").arg(distrib->getName()));
					ccConsole::Print(QString("[Distribution fitting] %1").arg(description));

                    //Auto Chi2
                    unsigned numberOfClasses = (unsigned)ceil(sqrt((double)pc->size()));
                    unsigned* histo = new unsigned[numberOfClasses];
                    double* npis = new double[numberOfClasses];
					{
						unsigned finalNumberOfClasses = 0;
						double chi2dist = CCLib::StatisticalTestingTools::computeAdaptativeChi2Dist(distrib,pc,0,finalNumberOfClasses,false,0,0,histo,npis);

						if (chi2dist>=0.0)
						{
							ccConsole::Print("[Distribution fitting] %s: Chi2 Distance = %f",distrib->getName(),chi2dist);
						}
						else
						{
							ccConsole::Warning("[Distribution fitting] Failed to compute Chi2 distance?!");
							delete[] histo;
							histo=0;
							delete[] npis;
							npis=0;
						}
					}

					ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);
					hDlg->setWindowTitle("[Distribution fitting]");
					{
						ccHistogramWindow* histogram = hDlg->window();
						histogram->setInfoStr(description);
						if (histo && npis)
						{
							histogram->fromBinArray(histo,numberOfClasses,sf->getMin(),sf->getMax(),true);
							histo=0;
							histogram->setCurveValues(npis,numberOfClasses,true);
							npis=0;
						}
						else
						{
							histogram->fromSF(sf,numberOfClasses);
						}
					}
					hDlg->show();
				}
                else
				{
                    ccConsole::Warning(QString("[Entity: %1]-[SF: %2] Couldn't compute distribution parameters!").arg(pc->getName()).arg(pc->getScalarFieldName(outSfIdx)));
				}
            }
        }
    }

    delete distrib;
}

void MainWindow::createComponentsClouds(ccGenericPointCloud* cloud, CCLib::ReferenceCloudContainer& components, unsigned minPointsPerComponent, bool randomColors)
{
	if (!cloud || components.empty())
		return;

	//we create "real" point clouds for all input components
	{
		ccPointCloud* pc = cloud->isA(CC_POINT_CLOUD) ? static_cast<ccPointCloud*>(cloud) : 0;

		//we create a new group to store all CCs
		ccHObject* ccGroup = new ccHObject(cloud->getName()+QString(" [CCs]"));

		//'shift on load' information
		const double* shift = (pc ? pc->getOriginalShift() : 0);

		//for each component
		while (!components.empty())
		{
			CCLib::ReferenceCloud* compIndexes = components.back();
			components.pop_back();

			//if it has enough points
			if (compIndexes->size() >= minPointsPerComponent)
			{
				//we create a new entity
				ccPointCloud* compCloud = (pc ? pc->partialClone(compIndexes) : ccPointCloud::From(compIndexes));
				if (compCloud)
				{
					//shall we colorize it with random color?
					if (randomColors)
					{
						colorType col[3] = {colorType(float(MAX_COLOR_COMP)*float(rand())/float(RAND_MAX)),
											colorType(float(MAX_COLOR_COMP)*float(rand())/float(RAND_MAX)),
											colorType(float(MAX_COLOR_COMP)*float(rand())/float(RAND_MAX))};
						compCloud->setRGBColor(col);
						compCloud->showColors(true);
						compCloud->showSF(false);
					}

					if (shift)
						compCloud->setOriginalShift(shift[0],shift[1],shift[2]);
					compCloud->setVisible(true);
					compCloud->setName(QString("CC#%1").arg(ccGroup->getChildrenNumber()));

					//we add new CC to group
					ccGroup->addChild(compCloud);
				}
				else
				{
					ccConsole::Warning("[createComponentsClouds] Failed to create component #%i! (not enough memory)",ccGroup->getChildrenNumber()+1);
				}
			}

			delete compIndexes;
			compIndexes = 0;
		}

		if (ccGroup->getChildrenNumber() == 0)
		{
			ccConsole::Error("No component was created! Check the minimum size...");
			delete ccGroup;
		}
		else
		{
			addToDB(ccGroup,true,0,true,false);
		}

		ccConsole::Print(QString("[createComponentsClouds] %1 component(s) where created from cloud '%2'").arg(ccGroup->getChildrenNumber()).arg(cloud->getName()));

		cloud->prepareDisplayForRefresh();
		cloud->setEnabled(ccGroup != 0);
	}
}

void MainWindow::doActionLabelConnectedComponents()
{
    ccLabelingDlg dlg(this);
    if (!dlg.exec())
        return;

    int octreeLevel = dlg.getOctreeLevel();
    int minComponentSize = dlg.getMinPointsNb();
    bool randColors = dlg.randomColors();

    ccProgressDialog pDlg(false,this);

	ccHObject::Container selectedEntities = m_selectedEntities;
    size_t i,selNum = selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = selectedEntities[i];
        if (ent->isKindOf(CC_POINT_CLOUD))
        {
            ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);

            if (cloud && cloud->isA(CC_POINT_CLOUD)) //TODO
            {
                ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

                ccOctree* theOctree = cloud->getOctree();
                if (!theOctree)
                {
					ccProgressDialog pDlg(true,this);
                    theOctree = cloud->computeOctree(&pDlg);
                    if (!theOctree)
                    {
                        ccConsole::Error(QString("Couldn't compute octree for cloud '%s'!").arg(cloud->getName()));
                        break;
                    }
                }

                //we create/activate CCs label scalar field
                int sfIdx = pc->getScalarFieldIndexByName(CC_CONNECTED_COMPONENTS_DEFAULT_LABEL_NAME);
                if (sfIdx<0)
                    sfIdx=pc->addScalarField(CC_CONNECTED_COMPONENTS_DEFAULT_LABEL_NAME);
                if (sfIdx<0)
                {
                    ccConsole::Error("Couldn't allocate a new scalar field for computing CC labels! Try to free some memory ...");
                    break;
                }
                pc->setCurrentScalarField(sfIdx);

                //we try to label all CCs
                CCLib::ReferenceCloudContainer components;
                if (CCLib::AutoSegmentationTools::labelConnectedComponents(cloud,octreeLevel,false,&pDlg,theOctree)>=0)
                {
                    //if successful, we extract each CC (stored in "components")
                    pc->getCurrentInScalarField()->computeMinAndMax();
                    if (!CCLib::AutoSegmentationTools::extractConnectedComponents(cloud,components))
					{
						ccConsole::Warning(QString("[doActionLabelConnectedComponents] Something went wrong while extracting CCs from cloud %1...").arg(cloud->getName()));
					}
                }
                else
                {
                    ccConsole::Warning(QString("[doActionLabelConnectedComponents] Something went wrong while extracting CCs from cloud %1...").arg(cloud->getName()));
                }

                //we delete the CCs label scalar field (we don't need it anymore)
                pc->deleteScalarField(sfIdx);
                sfIdx=-1;

                //we create "real" point clouds for all CCs
                if (!components.empty())
				{
					createComponentsClouds(cloud, components, minComponentSize, randColors);
				}
            }
        }
    }

    refreshAll();
	updateUI();
}

void MainWindow::doActionExportCoordToSF()
{
	ccExportCoordToSFDlg ectsDlg(this);

	if (!ectsDlg.exec())
		return;

	bool exportDim[3] = {ectsDlg.exportX(), ectsDlg.exportY(), ectsDlg.exportZ()};
	QString defaultSFName[3] = {"coordX", "coordY", "coordZ"};
	
	if (!exportDim[0] && !exportDim[1] && !exportDim[2]) //nothing to do?!
		return;

	//for each selected cloud (or vertices set)
	size_t selNum = m_selectedEntities.size();
	for (size_t i=0;i<selNum;++i)
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities[i]);
		if (cloud && cloud->isA(CC_POINT_CLOUD))
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
			unsigned ptsCount = pc->size();

			//test each dimension
			for (unsigned d=0;d<3;++d)
			{
				if (exportDim[d])
				{
					int sfIndex = pc->getScalarFieldIndexByName(qPrintable(defaultSFName[d]));
					if (sfIndex<0)
					{
						sfIndex = pc->addScalarField(qPrintable(defaultSFName[d]));
						if (sfIndex<0)
						{
							ccLog::Error("Not enough memory!");
							i=selNum;
							break;
						}
					}

					CCLib::ScalarField* sf = pc->getScalarField(sfIndex);
					sf->resize(ptsCount);
					assert(sf && sf->currentSize() >= ptsCount);
					if (sf)
					{
						for (unsigned k=0;k<ptsCount;++k)
							sf->setValue(k,pc->getPoint(k)->u[d]);
						sf->computeMinAndMax();
						pc->setCurrentDisplayedScalarField(sfIndex);
						m_selectedEntities[i]->showSF(true);
						m_selectedEntities[i]->refreshDisplay_recursive();
					}
				}
			}
		}

	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionHeightGridGeneration()
{
    size_t selNum = m_selectedEntities.size();
    if (selNum!=1)
    {
        ccConsole::Error("Select only one point cloud!");
        return;
    }

    ccHObject* ent = m_selectedEntities[0];
    if (!ent->isKindOf(CC_POINT_CLOUD) )
    {
        ccConsole::Error("Select a point cloud!");
        return;
    }

	ccHeightGridGenerationDlg dlg(ent->getMyOwnBB(),this);
    if (!dlg.exec())
        return;

    bool generateCloud = dlg.generateCloud();
	bool generateCountSF = dlg.generateCountSF();
    bool generateImage = dlg.generateImage();
    bool generateASCII = dlg.generateASCII();

    if (!generateCloud && !generateImage && !generateASCII)
    {
        ccConsole::Error("Nothing to do?! Mind the 'Generate' checkboxes...");
        return;
    }

    //Grid step must be > 0
    double gridStep = dlg.getGridStep();
    assert(gridStep>0);
	//Custom bundig box
	ccBBox box = dlg.getCustomBBox();

    ccProgressDialog pDlg(true,this);
    ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);

    //let's rock
	ccPointCloud* outputGrid = ccHeightGridGeneration::Compute(
		cloud,
		gridStep,
		box,
		dlg.getProjectionDimension(),
		dlg.getTypeOfProjection(),
		dlg.getFillEmptyCellsStrategy(),
		dlg.getTypeOfSFInterpolation(),
		dlg.getCustomHeightForEmptyCells(),
		generateCloud,
		generateImage,
		generateASCII,
		generateCountSF,
		&pDlg);

	//a cloud was demanded as output?
	if (outputGrid)
	{
		if (outputGrid->size() != 0)
		{
			if (cloud->getParent())
				cloud->getParent()->addChild(outputGrid);

			outputGrid->setName(QString("%1.heightGrid(%2)").arg(cloud->getName()).arg(gridStep,0,'g',3));
			ccGenericGLDisplay* win = cloud->getDisplay();
			outputGrid->setDisplay(win);
			//zoomOn(outputGrid);
			addToDB(outputGrid, true, 0, true, false);
			if (m_ccRoot)
				m_ccRoot->selectEntity(outputGrid);

			//don't forget original shift
			const double* shift = cloud->getOriginalShift();
			if (shift)
				outputGrid->setOriginalShift(shift[0],shift[1],shift[2]);
			cloud->prepareDisplayForRefresh_recursive();
			cloud->setEnabled(false);
			ccConsole::Warning("Previously selected entity (source) has been hidden!");

			if (win)
				win->refresh();
			updateUI();
		}
		else
		{
			ccConsole::Warning("[doActionHeightGridGeneration] Output cloud was empty!");
			delete outputGrid;
			outputGrid = 0;
		}
	}
}

void MainWindow::doActionComputeMeshAA()
{
    doActionComputeMesh(GENERIC);
}

void MainWindow::doActionComputeMeshLS()
{
    doActionComputeMesh(GENERIC_BEST_LS_PLANE);
}

void MainWindow::doActionComputeMesh(CC_TRIANGULATION_TYPES type)
{
	QProgressDialog pDlg("Triangulation in progress...", QString(), 0, 0, this);
	pDlg.show();

	QApplication::processEvents();

	ccHObject::Container selectedEntities = m_selectedEntities;
    size_t i,selNum = selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = selectedEntities[i];
        if (ent->isKindOf(CC_POINT_CLOUD))
        {
            ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);
			bool hadNormals = cloud->hasNormals();

            CCLib::GenericIndexedMesh* dummyMesh = CCLib::PointProjectionTools::computeTriangulation(cloud,type);

            if (dummyMesh)
            {
                ccMesh* mesh = new ccMesh(dummyMesh, cloud);
                if (mesh)
                {
                    mesh->setName(cloud->getName()+QString(".mesh"));
                    mesh->setDisplay(cloud->getDisplay());
					if (hadNormals && ent->isA(CC_POINT_CLOUD))
					{
						//if the cloud already had normals, they might not be concordant with the mesh!
						if (QMessageBox::question(	this,
													"Keep old normals?",
													"Cloud already had normals. Do you want to update them (yes) or keep the old ones (no)?",
													QMessageBox::Yes,QMessageBox::No ) != QMessageBox::No)
						{
							static_cast<ccPointCloud*>(ent)->unallocateNorms();
							mesh->computeNormals();
						}
					}
					if (cloud->hasColors() && !cloud->hasNormals())
						mesh->showNormals(false);
                    cloud->setVisible(false);
                    cloud->addChild(mesh);
                    cloud->prepareDisplayForRefresh();
					if (mesh->getAssociatedCloud() && mesh->getAssociatedCloud() != cloud)
					{
						const double* shift = cloud->getOriginalShift();
						if (shift)
							mesh->getAssociatedCloud()->setOriginalShift(shift[0],shift[1],shift[2]);
					}
                    addToDB(mesh,true,0,false,false);
					if (i==0)
						m_ccRoot->selectEntity(mesh); //auto-select first element
                }
                else
                {
                    ccConsole::Error("An error occured while computing mesh! (not enough memory?)");
                }
            }
            else
            {
                ccConsole::Error("An error occured while computing mesh! (not enough memory?)");
            }
        }
    }

    refreshAll();
	updateUI();
}

void MainWindow::doActionComputeQuadric3D()
{
	ccHObject::Container selectedEntities = m_selectedEntities;
    size_t i,selNum = selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = selectedEntities[i];
        if (ent->isKindOf(CC_POINT_CLOUD))
        {
            ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);
            CCLib::Neighbourhood Yk(cloud);

//#define USE_QUADRIC_3D
#ifdef USE_QUADRIC_3D
            /*void* R = */Yk.getGeometricalElement(CCLib::Neighbourhood::QUADRIC_3D);
#else
			uchar hfdims[3];
            const PointCoordinateType* Q = Yk.getHeightFunction(hfdims);
#endif
            if (Q)
            {
#ifdef USE_QUADRIC_3D
                const PointCoordinateType a=Q[0];
                const PointCoordinateType b=Q[1];
                const PointCoordinateType c=Q[2];
                const PointCoordinateType e=Q[3];
                const PointCoordinateType f=Q[4];
                const PointCoordinateType g=Q[5];
                const PointCoordinateType l=Q[6];
                const PointCoordinateType m=Q[7];
                const PointCoordinateType n=Q[8];
                const PointCoordinateType d=Q[9];
#else
                const PointCoordinateType a=Q[0];
                const PointCoordinateType b=Q[1];
                const PointCoordinateType c=Q[2];
                const PointCoordinateType d=Q[3];
                const PointCoordinateType e=Q[4];
                const PointCoordinateType f=Q[5];

                const uchar hfX = hfdims[0];
                const uchar hfY = hfdims[1];
                const uchar hfZ = hfdims[2];
				const char dimChars[3]={'x','y','z'};

				ccConsole::Print("Resulting quadric (Z=a+b*X+c*Y+d*X^2+e*XY+f*Y^2):");
				ccConsole::Print("a=%f",a);
				ccConsole::Print("b=%f",b);
				ccConsole::Print("c=%f",c);
				ccConsole::Print("d=%f",d);
				ccConsole::Print("e=%f",e);
				ccConsole::Print("f=%f",f);
				ccConsole::Print("Dimensions: X=%c Y=%c Z=%c",dimChars[hfX],dimChars[hfY],dimChars[hfZ]);
#endif

                //gravity center
                const CCVector3* G = Yk.getGravityCenter();
                if (!G)
                {
                    ccConsole::Warning(QString("Failed to get gravity center of cloud '%1'!").arg(cloud->getName()));
                    continue;
                }

                const ccBBox bbox = cloud->getBB();
				CCVector3 bboxDiag = bbox.getDiagVec();
                CCVector3 P, Pc;

#ifndef USE_QUADRIC_3D
				//Sample points on Quadric and triangulate them!
				float spanX = bboxDiag.u[hfX];
				float spanY = bboxDiag.u[hfY];

				const unsigned nStepX = 20;
				const unsigned nStepY = 20;
				float stepX = spanX/(float)(nStepX-1);
				float stepY = spanY/(float)(nStepY-1);

                ccPointCloud* vertices = new ccPointCloud();
				const double* shift = cloud->getOriginalShift();
				if (shift)
					vertices->setOriginalShift(shift[0],shift[1],shift[2]);
				vertices->setName("vertices");
                vertices->reserve(nStepX*nStepY);

				ccMesh* quadMesh = new ccMesh(vertices);
				quadMesh->reserve((nStepX-1)*(nStepY-1)*2);

				for (unsigned x=0;x<nStepX;++x)
				{
					P.x = bbox.minCorner().u[hfX] + stepX*(float)x - G->u[hfX];
					for (unsigned y=0;y<nStepY;++y)
					{
						P.y = bbox.minCorner().u[hfY] + stepY*(float)y - G->u[hfY];
						P.z = a+b*P.x+c*P.y+d*P.x*P.x+e*P.x*P.y+f*P.y*P.y;

						Pc.u[hfX] = P.x;
						Pc.u[hfY] = P.y;
						Pc.u[hfZ] = P.z;
						Pc += *G;

						vertices->addPoint(Pc);

						if (x>0 && y>0)
						{
							unsigned iA = (x-1) * nStepY + y-1;
							unsigned iB = iA+1;
							unsigned iC = iA+nStepY;
							unsigned iD = iB+nStepY;

							quadMesh->addTriangle(iA,iC,iB);
							quadMesh->addTriangle(iB,iC,iD);
						}
					}
				}

				quadMesh->computeNormals();
				quadMesh->addChild(vertices);
				quadMesh->setName(QString("Quadric(%1)").arg(cloud->getName()));
                addToDB(quadMesh, true, 0, false, false);
                quadMesh->setDisplay(cloud->getDisplay());
                quadMesh->prepareDisplayForRefresh();
#endif


				/* //Sample points on a cube and compute for each of them the distance to the Quadric
                const unsigned steps = 50;
                ccPointCloud* newCloud = new ccPointCloud();
                int sfIdx = newCloud->getScalarFieldIndexByName("Dist2Quadric");
                if (sfIdx<0) sfIdx=newCloud->addScalarField("Dist2Quadric");
                if (sfIdx<0)
                {
                    ccConsole::Error("Couldn't allocate a new scalar field for computing distances! Try to free some memory ...");
                    delete newCloud;
                    continue;
                }
                newCloud->setCurrentScalarField(sfIdx);
                newCloud->reserve(steps*steps*steps);

                //FILE* fp = fopen("doActionComputeQuadric3D_trace.txt","wt");
                count=0;
                unsigned x,y,z;
                PointCoordinateType maxDim = std::max(std::max(bboxDiag.x,bboxDiag.y),bboxDiag.z);
                CCVector3 C = bbox.getCenter();
                for (x=0;x<steps;++x)
                {
                    P.x = C.x - maxDim * 0.5 + maxDim * (PointCoordinateType)x / (PointCoordinateType)(steps-1);
                    for (y=0;y<steps;++y)
                    {
                        P.y = C.y - maxDim * 0.5 + maxDim * (PointCoordinateType)y / (PointCoordinateType)(steps-1);
                        for (z=0;z<steps;++z)
                        {
                            P.z = C.z - maxDim * 0.5  + maxDim * (PointCoordinateType)z / (PointCoordinateType)(steps-1);
                            newCloud->addPoint(P);

                            Pc = P-Gc;

#ifdef USE_QUADRIC_3D
                            ScalarType dist = (ScalarType)(a*Pc.x*Pc.x + b*Pc.y*Pc.y + c*Pc.z*Pc.z + e*Pc.x*Pc.y + f*Pc.y*Pc.z + g*Pc.x*Pc.z + l*Pc.x + m*Pc.y + n*Pc.z + d);
#else
                            ScalarType dist = Pc.u[hfZ] - (a+b*Pc.u[hfX]+c*Pc.u[hfY]+d*Pc.u[hfX]*Pc.u[hfX]+e*Pc.u[hfX]*Pc.u[hfY]+f*Pc.u[hfY]*Pc.u[hfY]);
#endif
                            newCloud->setPointScalarValue(count++,dist);
                            //fprintf(fp,"%f %f %f %f\n",Pc.x,Pc.y,Pc.z,dist);
                        }
                    }
                }
                //fclose(fp);

                //Yk.computeCurvature2(0,CCLib::Neighbourhood::GAUSSIAN_CURV);
                newCloud->getScalarField(sfIdx)->computeMinAndMax();
                newCloud->setCurrentDisplayedScalarField(sfIdx);
				newCloud->showSF(sfIdx>=0);
                newCloud->setName("Distance to quadric");

                addToDB(newCloud, true, 0, false, false);
                newCloud->setDisplay(cloud->getDisplay());
                newCloud->prepareDisplayForRefresh();
				//*/
            }
            else
            {
                ccConsole::Warning(QString("Failed to compute quadric on cloud '%1'").arg(cloud->getName()));
            }
        }
    }

    refreshAll();
}
//*/

void MainWindow::doActionComputeCPS()
{
    size_t selNum = m_selectedEntities.size();
    if (selNum!=2)
    {
        ccConsole::Error("Select 2 point clouds!");
        return;
    }

    if (!m_selectedEntities[0]->isKindOf(CC_POINT_CLOUD) ||
            !m_selectedEntities[1]->isKindOf(CC_POINT_CLOUD))
    {
        ccConsole::Error("Select 2 point clouds!");
        return;
    }

    ccOrderChoiceDlg dlg(m_selectedEntities[0], "Reference", m_selectedEntities[1], "Source", this);
    if (!dlg.exec())
        return;

    ccGenericPointCloud* compCloud = ccHObjectCaster::ToGenericPointCloud(dlg.getFirstEntity());
    ccGenericPointCloud* srcCloud = ccHObjectCaster::ToGenericPointCloud(dlg.getSecondEntity());

    if (!compCloud->isA(CC_POINT_CLOUD)) //TODO
    {
        ccConsole::Error("Reference cloud must be a real point cloud!");
        return;
    }
    ccPointCloud* cmpPC = static_cast<ccPointCloud*>(compCloud);

    int sfIdx = cmpPC->getScalarFieldIndexByName("tempScalarField");
    if (sfIdx<0) sfIdx=cmpPC->addScalarField("tempScalarField");
    if (sfIdx<0)
    {
        ccConsole::Error("Couldn't allocate a new scalar field for computing distances! Try to free some memory ...");
        return;
    }
    cmpPC->setCurrentScalarField(sfIdx);
    cmpPC->enableScalarField();
	cmpPC->forEach(CCLib::ScalarFieldTools::SetScalarValueToNaN);

    CCLib::ReferenceCloud CPSet(srcCloud);
    ccProgressDialog pDlg(true,this);
	CCLib::DistanceComputationTools::Cloud2CloudDistanceComputationParams params;
	params.CPSet = &CPSet;
    int result=CCLib::DistanceComputationTools::computeHausdorffDistance(compCloud,srcCloud,params,&pDlg);
    cmpPC->deleteScalarField(sfIdx);

    if (result>=0)
    {
        ccPointCloud* newCloud = 0;
        //if the source cloud is a "true" cloud, the extracted CPS
        //will also get its attributes
        if (srcCloud->isA(CC_POINT_CLOUD))
			newCloud = static_cast<ccPointCloud*>(srcCloud)->partialClone(&CPSet);
        else
		{
            newCloud = ccPointCloud::From(&CPSet);
			const double* shift = srcCloud->getOriginalShift();
			if (shift)
				newCloud->setOriginalShift(shift[0],shift[1],shift[2]);
		}

		newCloud->setName(QString("[%1]->CPSet(%2)").arg(srcCloud->getName()).arg(compCloud->getName()));
        addToDB(newCloud, true, 0, false, false);
        newCloud->setDisplay(compCloud->getDisplay());
        newCloud->prepareDisplayForRefresh();
        //we hide the source cloud (for a clearer display)
        srcCloud->setEnabled(false);
        srcCloud->prepareDisplayForRefresh();
    }

    refreshAll();
}

void MainWindow::doActionComputeNormals()
{
    if (m_selectedEntities.empty())
    {
        ccConsole::Error("Select at least one point cloud");
        return;
    }

    size_t i,count = m_selectedEntities.size();
	float defaultRadius = 0.0;
	bool onlyMeshes = true;
	for (i=0;i<count;++i)
		if (!m_selectedEntities[i]->isKindOf(CC_MESH))
		{
			if (defaultRadius == 0.0 && m_selectedEntities[i]->isA(CC_POINT_CLOUD))
			{
				ccPointCloud* cloud = static_cast<ccPointCloud*>(m_selectedEntities[i]);
				defaultRadius = cloud->getBB().getDiagNorm()*0.02; //diameter=2% of the bounding box diagonal
			}
			onlyMeshes = false;
			break;
		}

    CC_LOCAL_MODEL_TYPES model = NO_MODEL;
    int preferedOrientation = -1;

	//We display dialog only for point clouds
	if (!onlyMeshes)
	{
		ccNormalComputationDlg ncDlg(this);
		ncDlg.setRadius(defaultRadius);

		if (!ncDlg.exec())
			return;

		model = ncDlg.getLocalModel();
		preferedOrientation = ncDlg.getPreferedOrientation();
		defaultRadius = ncDlg.radiusDoubleSpinBox->value();
	}

    //Compute normals for each selected cloud
    for (unsigned i=0; i<m_selectedEntities.size(); i++)
    {
        if (m_selectedEntities[i]->isA(CC_POINT_CLOUD))
		{
			ccPointCloud* cloud = static_cast<ccPointCloud*>(m_selectedEntities[i]);

			ccProgressDialog pDlg(true,this);

			if (!cloud->getOctree())
				if (!cloud->computeOctree(&pDlg))
				{
					ccConsole::Error(QString("Could not compute octree for cloud '%1'").arg(cloud->getName()));
					continue;
				}

			//computes cloud normals
			QElapsedTimer eTimer;
			eTimer.start();
			NormsIndexesTableType* normsIndexes = new NormsIndexesTableType;
			if (!ccNormalVectors::ComputeCloudNormals(cloud, *normsIndexes, model, defaultRadius, preferedOrientation, (CCLib::GenericProgressCallback*)&pDlg, cloud->getOctree()))
			{
				ccConsole::Error(QString("Failed to compute normals on cloud '%1'").arg(cloud->getName()));
				continue;
			}
			ccConsole::Print("[ComputeCloudNormals] Timing: %3.2f s.",eTimer.elapsed()/1.0e3);

			if (!cloud->hasNormals())
			{
                if (!cloud->resizeTheNormsTable())
				{
					ccConsole::Error(QString("Failed to instantiate normals array on cloud '%1'").arg(cloud->getName()));
					continue;
				}
			}
			else
			{
				//we hide normals during process
				cloud->showNormals(false);
			}

			for (unsigned j=0; j<normsIndexes->currentSize(); j++)
				cloud->setPointNormalIndex(j, normsIndexes->getValue(j));

			normsIndexes->release();
			normsIndexes=0;

			cloud->showNormals(true);
			cloud->prepareDisplayForRefresh();
		}
		else if (m_selectedEntities[i]->isKindOf(CC_MESH))
		{
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(m_selectedEntities[i]);
			if (!mesh->computeNormals())
			{
				ccConsole::Error(QString("Failed to compute normals on mesh '%1'").arg(mesh->getName()));
				continue;
			}
			mesh->showNormals(true);
			mesh->prepareDisplayForRefresh_recursive();
		}
    }

    refreshAll();
	updateUI();
}

void MainWindow::doActionResolveNormalsDirection()
{
    if (m_selectedEntities.size() < 1)
    {
        ccConsole::Error("Select at least one point cloud");
        return;
    }

    ccAskOneIntValueDlg vDlg("Octree level", 1, CCLib::DgmOctree::MAX_OCTREE_LEVEL, 7, "Resolve normal directions");
    if (!vDlg.exec())
        return;
	assert(vDlg.getValue() && vDlg.getValue()<=255);
    uchar level = (uchar)vDlg.getValue();

    for (unsigned i=0; i<m_selectedEntities.size(); i++)
    {
        if (!m_selectedEntities[i]->isA(CC_POINT_CLOUD))
            continue;

        ccPointCloud* cloud = static_cast<ccPointCloud*>(m_selectedEntities[i]);
        ccProgressDialog pDlg(false,this);

        if (!cloud->getOctree())
		{
            if (!cloud->computeOctree((CCLib::GenericProgressCallback*)&pDlg))
            {
                ccConsole::Error(QString("Could not compute octree for cloud '%1'").arg(cloud->getName()));
                continue;
            }
		}

		unsigned pointCount = cloud->size();

        NormsIndexesTableType* normsIndexes = new NormsIndexesTableType;
        if (!normsIndexes->reserve(pointCount))
		{
			ccConsole::Error(QString("Not engouh memory! (cloud '%1')").arg(cloud->getName()));
			continue;
		}

		//init array with current normals
        for (unsigned j=0; j<pointCount; j++)
        {
            const normsType& index = cloud->getPointNormalIndex(j);
			normsIndexes->addElement(index);
        }
        
		//apply algorithm
		ccFastMarchingForNormsDirection::ResolveNormsDirectionByFrontPropagation(cloud, normsIndexes, level, (CCLib::GenericProgressCallback*)&pDlg, cloud->getOctree());
        
		//compress resulting normals and transfer them to the cloud
		for (unsigned j=0; j<pointCount; j++)
            cloud->setPointNormalIndex(j, normsIndexes->getValue(j));

		normsIndexes->release();
		normsIndexes=0;

        cloud->prepareDisplayForRefresh();
    }

    refreshAll();
	updateUI();
}

void MainWindow::doActionSynchronize()
{
    size_t i,selNum = m_selectedEntities.size();

    //we need at least 2 entities
    if (selNum<2)
        return;

	//we must backup 'm_selectedEntities' as removeObjectTemporarilyFromDBTree can modify it!
	ccHObject::Container selectedEntities = m_selectedEntities;

	//by default, we take the first entity as reference
    //TODO: maybe the user would like to select the reference himself ;)
    ccHObject* refEnt = selectedEntities[0];
    CCVector3 refCenter = refEnt->getCenter();

    for (i=1;i<selNum;++i)
    {
        ccHObject* ent = selectedEntities[i];
        CCVector3 center = ent->getCenter();

        CCVector3 T = refCenter-center;

        //transformation (used only for translation)
        ccGLMatrix glTrans;
        glTrans += T;

        //ccConsole::Print("[Synchronize] Translation: (%f,%f,%f)",T.x,T.y,T.z);
		forceConsoleDisplay();
		ccConsole::Print(QString("[Synchronize] Transformation matrix (%1 --> %2):").arg(ent->getName()).arg(selectedEntities[0]->getName()));
		const float* mat = glTrans.data();
		ccConsole::Print("%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f",mat[0],mat[4],mat[8],mat[12],mat[1],mat[5],mat[9],mat[13],mat[2],mat[6],mat[10],mat[14],mat[3],mat[7],mat[11],mat[15]);
		ccConsole::Print("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool");

		//we temporarily detach entity, as it may undergo
		//"severe" modifications (octree deletion, etc.) --> see ccHObject::applyGLTransformation
		ccHObject* parent=0;
		removeObjectTemporarilyFromDBTree(ent,parent);
		ent->applyGLTransformation_recursive(&glTrans);
		putObjectBackIntoDBTree(ent,parent);

		ent->prepareDisplayForRefresh_recursive();
    }

    zoomOnSelectedEntities();

    updateUI();
}

void MainWindow::doActionUnroll()
{
    //there should be only one point cloud with sensor in current selection!
    if (m_selectedEntities.empty() || m_selectedEntities.size()>1)
    {
        ccConsole::Error("Select one and only one entity!");
        return;
    }

    //if selected entity is a mesh, the method will be applied to its vertices
	bool lockedVertices;
	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities[0],&lockedVertices);
	if (lockedVertices)
	{
		DisplayLockedVerticesWarning();
		return;
	}

    //for "real" point clouds only
    if (!cloud || !cloud->isA(CC_POINT_CLOUD))
    {
        ccConsole::Error("Method can't be applied on virtual point clouds!");
        return;
    }
    ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

    ccUnrollDlg unrollDlg(this);
    if (!unrollDlg.exec())
        return;

    int mode = unrollDlg.getType();
    double radius = unrollDlg.getRadius();
    double angle = unrollDlg.getAngle();
    unsigned char dim = (unsigned char)unrollDlg.getAxisDimension();
    CCVector3* pCenter = 0;
    CCVector3 center;
    if (mode==1 || !unrollDlg.isAxisPositionAuto())
    {
        center = unrollDlg.getAxisPosition();
        pCenter = &center;
    }

    //We apply unrolling method
    ccProgressDialog pDlg(true,this);

    if (mode==0)
        pc->unrollOnCylinder(radius,pCenter,dim,(CCLib::GenericProgressCallback*)&pDlg);
    else if (mode==1)
        pc->unrollOnCone(radius,angle,center,dim,(CCLib::GenericProgressCallback*)&pDlg);
    else
        assert(false);

    ccGLWindow* win = static_cast<ccGLWindow*>(cloud->getDisplay());
    if (win)
        win->updateConstellationCenterAndZoom();
    updateUI();
}

ccGLWindow *MainWindow::getActiveGLWindow()
{
	if (!m_mdiArea)
		return 0;

    QMdiSubWindow *activeSubWindow = m_mdiArea->activeSubWindow();
    if (activeSubWindow)
        return static_cast<ccGLWindow*>(activeSubWindow->widget());
    else
    {
        QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
        if (!subWindowList.isEmpty())
            return static_cast<ccGLWindow*>(subWindowList[0]->widget());
    }

    return 0;
}

QMdiSubWindow* MainWindow::getMDISubWindow(ccGLWindow* win)
{
	QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
	for (int i=0;i<subWindowList.size();++i)
		if (static_cast<ccGLWindow*>(subWindowList[i]->widget()) == win)
			return subWindowList[i];

	//not found!
	return 0;
}

ccGLWindow* MainWindow::new3DView()
{
	assert(m_ccRoot && m_mdiArea);

	//already existing window?
	QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
	ccGLWindow* otherWin=0;
	if (!subWindowList.isEmpty())
		otherWin=static_cast<ccGLWindow*>(subWindowList[0]->widget());

    QGLFormat format;
    format.setSwapInterval(0);
	ccGLWindow *view3D = new ccGLWindow(this,format,otherWin); //We share OpenGL contexts between windows!
    view3D->setMinimumSize(400,300);
    view3D->resize(500,400);

    m_mdiArea->addSubWindow(view3D);

    connect(view3D,	SIGNAL(entitySelectionChanged(int)),				m_ccRoot,	SLOT(selectEntity(int)));
    connect(view3D,	SIGNAL(entitiesSelectionChanged(std::set<int>)),	m_ccRoot,	SLOT(selectEntities(std::set<int>)));

	//'echo' mode
    connect(view3D,	SIGNAL(mouseWheelRotated(float)),			this,       SLOT(echoMouseWheelRotate(float)));
    connect(view3D,	SIGNAL(cameraDisplaced(float,float)),		this,       SLOT(echoCameraDisplaced(float,float)));
    connect(view3D,	SIGNAL(viewMatRotated(const ccGLMatrix&)),	this,       SLOT(echoBaseViewMatRotation(const ccGLMatrix&)));

    connect(view3D,	SIGNAL(destroyed(QObject*)),				this,       SLOT(prepareWindowDeletion(QObject*)));
    connect(view3D,	SIGNAL(filesDropped(const QStringList&)),	this,       SLOT(addToDBAuto(const QStringList&)));
    connect(view3D,	SIGNAL(newLabel(ccHObject*)),				this,       SLOT(handleNewEntity(ccHObject*)));

	view3D->setSceneDB(m_ccRoot->getRootEntity());
    view3D->setAttribute(Qt::WA_DeleteOnClose);
    m_ccRoot->updatePropertiesView();

    QMainWindow::statusBar()->showMessage(QString("New 3D View"), 2000);

    view3D->showMaximized();

	return view3D;
}

void MainWindow::prepareWindowDeletion(QObject* glWindow)
{
    if (!m_ccRoot)
        return;

    //we assume only ccGLWindow can be connected to this slot!
    ccGLWindow* win = qobject_cast<ccGLWindow*>(glWindow);

    m_ccRoot->hidePropertiesView();
    m_ccRoot->getRootEntity()->removeFromDisplay_recursive(win);
    m_ccRoot->updatePropertiesView();
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    //if (m_uiFrozen)
    //{
    //    ccConsole::Error("Close current dialog/interactor first!");
    //    event->ignore();
    //}
    //else
    {
		if (m_ccRoot && m_ccRoot->getRootEntity()->getChildrenNumber()==0
			|| QMessageBox::question(	this,
										"Quit",
										"Are you sure you want to quit?",
										QMessageBox::Ok,QMessageBox::Cancel ) != QMessageBox::Cancel)
		{
            event->accept();
        }
        else
        {
            event->ignore();
        }
    }
}

void MainWindow::moveEvent(QMoveEvent* event)
{
    QMainWindow::moveEvent(event);

    updateMDIDialogsPlacement();
}

void MainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);

    updateMDIDialogsPlacement();
}

void MainWindow::registerMDIDialog(ccOverlayDialog* dlg, Qt::Corner pos)
{
    //check for existence
    for (unsigned i=0; i<m_mdiDialogs.size(); ++i)
    {
        if (m_mdiDialogs[i].dialog == dlg)
        {
            //we only update position in this case
            m_mdiDialogs[i].position = pos;
            return;
        }
    }

    //otherwise we add it to DB
    m_mdiDialogs.push_back(ccMDIDialogs(dlg,pos));
}

void MainWindow::placeMDIDialog(ccMDIDialogs& mdiDlg)
{
    if (!mdiDlg.dialog || !mdiDlg.dialog->isVisible() || !m_mdiArea)
        return;

    int dx=0,dy=0;
    switch (mdiDlg.position)
    {
    case Qt::TopLeftCorner:
        dx = 5;
        dy = 5;
        break;
    case Qt::TopRightCorner:
        dx = std::max(5,m_mdiArea->width() - mdiDlg.dialog->width() - 5);
        dy = 5;
        break;
    case Qt::BottomLeftCorner:
        dx = 5;
        dy = std::max(5,m_mdiArea->height() - mdiDlg.dialog->height() - 5);
        break;
    case Qt::BottomRightCorner:
        dx = std::max(5,m_mdiArea->width() - mdiDlg.dialog->width() - 5);
        dy = std::max(5,m_mdiArea->height() - mdiDlg.dialog->height() - 5);
        break;
    }

	//show();
   mdiDlg.dialog->move(m_mdiArea->mapToGlobal(QPoint(dx,dy)));
   mdiDlg.dialog->raise();
}

void MainWindow::updateMDIDialogsPlacement()
{
    for (unsigned i=0; i<m_mdiDialogs.size(); ++i)
        placeMDIDialog(m_mdiDialogs[i]);
}

void MainWindow::toggleFullScreen(bool state)
{
    if (state)
        showFullScreen();
    else
        showNormal();
}

void MainWindow::about()
{
	QDialog* aboutDialog = new QDialog(this);

	Ui::AboutDialog ui;
	ui.setupUi(aboutDialog);

	QString ccVer = ccCommon::GetCCVersion();
	QString htmlText = ui.textEdit->toHtml();
	QString enrichedHtmlText = htmlText.arg(ccVer);
	//ccLog::PrintDebug(htmlText);
	//ccLog::PrintDebug(ccVer);
	//ccLog::PrintDebug(enrichedHtmlText);

	ui.textEdit->setHtml(enrichedHtmlText);

	aboutDialog->exec();

	//delete aboutDialog; //Qt will take care of it? Anyway CC crash if we try to delete it now!
}

void MainWindow::help()
{
	QFile doc(QApplication::applicationDirPath()+QString("/user_guide_CloudCompare.pdf"));
    if (!doc.open(QIODevice::ReadOnly))
	{
        QMessageBox::warning(	this, 
								QString("User guide not found"),
								QString("Goto http://www.danielgm.net/cc/doc/qCC") );
	}
    else
    {
        QString program = "AcroRd32.exe";
        QStringList arguments;
        arguments << "user_guide_CloudCompare.pdf";
        QProcess *myProcess = new QProcess();
        myProcess->start(program, arguments);
    }
}

void MainWindow::freezeUI(bool state)
{
    toolBarMainTools->setDisabled(state);
    toolBarSFTools->setDisabled(state);
    toolBarPluginTools->setDisabled(state);
	toolBarGLFilters->setDisabled(state);

    //toolBarView->setDisabled(state);
    DockableDBTree->setDisabled(state);
    menubar->setDisabled(state);

    if (state)
    {
        menuEdit->setDisabled(true);
        menuTools->setDisabled(true);
    }
    else
    {
        updateMenus();
    }

    m_uiFrozen = state;
}

void MainWindow::activateRegisterPointPairTool()
{
    size_t selNum = m_selectedEntities.size();
    if (selNum==0 || selNum>2)
    {
        ccConsole::Error("Select one or two entities (point cloud or mesh)!");
        return;
    }

	bool lockedVertices1=false;
	ccGenericPointCloud* cloud1 = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities[0],&lockedVertices1);
	bool lockedVertices2=false;
	ccGenericPointCloud* cloud2 = (m_selectedEntities.size()>1 ? ccHObjectCaster::ToGenericPointCloud(m_selectedEntities[1],&lockedVertices2) : 0);
    if (!cloud1 || (m_selectedEntities.size()>1 && !cloud2))
    {
        ccConsole::Error("Select point clouds or meshes only!");
        return;
    }
	if (lockedVertices1 || lockedVertices2)
	{
		DisplayLockedVerticesWarning();
        //ccConsole::Error("At least one vertex set is locked (you should select the 'vertices' entity directly!)");
        return;
	}

	ccGenericPointCloud* aligned = cloud1;
	ccGenericPointCloud* reference = 0;

	//if we have 2 clouds, we must ask the user which one is the 'aligned' one and which one is the 'reference' one
	if (cloud2)
	{
		ccOrderChoiceDlg dlg(cloud1, "Aligned", cloud2, "Reference", this);
		if (!dlg.exec())
			return;

		aligned = ccHObjectCaster::ToGenericPointCloud(dlg.getFirstEntity());
		reference = ccHObjectCaster::ToGenericPointCloud(dlg.getSecondEntity());
	}

	//we disable all windows
    disableAllBut(0);

	if (!m_pprDlg)
    {
        m_pprDlg = new ccPointPairRegistrationDlg(this);
        connect(m_pprDlg, SIGNAL(processFinished(bool)), this, SLOT(deactivateRegisterPointPairTool(bool)));
        registerMDIDialog(m_pprDlg,Qt::TopRightCorner);
    }

	if (!m_pprDlg->init(aligned,reference))
		deactivateRegisterPointPairTool(false);

    freezeUI(true);

    if (!m_pprDlg->start())
        deactivateRegisterPointPairTool(false);
    else
        updateMDIDialogsPlacement();
}

void MainWindow::deactivateRegisterPointPairTool(bool state)
{
	if (m_pprDlg)
		m_pprDlg->clear();

    //we enable all GL windows
    enableAll();

	QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
	if (!subWindowList.isEmpty())
		subWindowList[0]->showMaximized();

	freezeUI(false);

    updateUI();

	ccGLWindow* win = getActiveGLWindow();
	if (win)
		win->zoomGlobal();
}

void MainWindow::activateSegmentationMode()
{
    ccGLWindow* win = getActiveGLWindow();
    if (!win)
        return;

    size_t i,selNum = m_selectedEntities.size();
    if (selNum==0)
        return;

    if (!m_gsTool)
    {
        m_gsTool = new ccGraphicalSegmentationTool(this);
        connect(m_gsTool, SIGNAL(processFinished(bool)), this, SLOT(deactivateSegmentationMode(bool)));

        registerMDIDialog(m_gsTool,Qt::TopRightCorner);
    }

    m_gsTool->linkWith(win);

    for (i=0; i<selNum; ++i)
        m_gsTool->addEntity(m_selectedEntities[i]);

    if (m_gsTool->getNumberOfValidEntities()==0)
    {
        ccConsole::Error("No segmentable entity in active window!");
        return;
    }

    freezeUI(true);
    toolBarView->setDisabled(false);

    //we disable all other windows
    disableAllBut(win);

    if (!m_gsTool->start())
        deactivateSegmentationMode(false);
    else
        updateMDIDialogsPlacement();
}

void MainWindow::deactivateSegmentationMode(bool state)
{
	bool deleteHiddenPoints = false;

    //shall we apply segmentation?
    if (state)
    {
        ccHObject* firstResult = 0;

        unsigned n = m_gsTool->getNumberOfValidEntities();
		deleteHiddenPoints = m_gsTool->deleteHiddenPoints();

        for (unsigned i=0;i<n;++i)
        {
            ccHObject* entity = m_gsTool->getEntity(i);
			
            if (entity->isKindOf(CC_POINT_CLOUD) || entity->isKindOf(CC_MESH))
			{
				//Special case: labels (do this before temporarily removing 'entity' from DB!)
				bool lockedVertices;
				ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity,&lockedVertices);
				if (cloud)
				{
					//assert(!lockedVertices); //in some cases we accept to segment meshes with locked vertices!
					ccHObject::Container labels;
					if (m_ccRoot)
						m_ccRoot->getRootEntity()->filterChildren(labels,true,CC_2D_LABEL);
					for (ccHObject::Container::iterator it=labels.begin(); it!=labels.end(); ++it)
					{
						//we must check all dependent labels and remove them!!!
						//TODO: couldn't we be more clever and update the label instead?
						cc2DLabel* label = static_cast<cc2DLabel*>(*it);
						bool removeLabel = false;
						for (unsigned i=0;i<label->size();++i)
							if (label->getPoint(i).cloud == entity)
							{
								removeLabel = true;
								break;
							}

						if (removeLabel && label->getParent())
						{
							ccLog::Warning(QString("[Segmentation] Label %1 is dependent on cloud %2 and will be removed").arg(label->getName()).arg(cloud->getName()));
							ccHObject* labelParent = label->getParent();
							ccHObject* labelParentParent = 0;
							removeObjectTemporarilyFromDBTree(labelParent,labelParentParent);
							labelParent->removeChild(label);
							label=0;
							putObjectBackIntoDBTree(labelParent,labelParentParent);
						}
					}
				}

				//we temporarily detach entity, as it may undergo
				//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::createNewCloudFromVisibilitySelection
				ccHObject* parent=0;
				removeObjectTemporarilyFromDBTree(entity,parent);

				ccHObject* segmentationResult = 0;
				if (entity->isKindOf(CC_POINT_CLOUD))
				{
					segmentationResult = ccHObjectCaster::ToGenericPointCloud(entity)->createNewCloudFromVisibilitySelection(true);
				}
				else if (entity->isKindOf(CC_MESH))
				{
					segmentationResult = ccHObjectCaster::ToGenericMesh(entity)->createNewMeshFromSelection(true);
				}

				if (!deleteHiddenPoints) //no need to put it back if we delete it afterwards!
				{
					entity->setName(entity->getName()+QString(".remaining"));
					putObjectBackIntoDBTree(entity,parent);
				}
				else
				{
					//keep original name(s)
					segmentationResult->setName(entity->getName());
					if (entity->isKindOf(CC_MESH) && segmentationResult->isKindOf(CC_MESH))
						ccHObjectCaster::ToGenericMesh(segmentationResult)->getAssociatedCloud()->setName(ccHObjectCaster::ToGenericMesh(entity)->getAssociatedCloud()->getName());

					delete entity;
					entity=0;
				}

				if (segmentationResult)
				{
					//we look for first non-mesh or non-cloud parent
					while (parent && (parent->isKindOf(CC_MESH) || parent->isKindOf(CC_POINT_CLOUD)))
						parent=parent->getParent();

					if (parent)
						parent->addChild(segmentationResult);

					addToDB(segmentationResult, true, 0, true, false);
					segmentationResult->prepareDisplayForRefresh_recursive();

					if (!firstResult)
						firstResult = segmentationResult;
				}
				//else
				//{
				//	ccConsole::Error("An error occured! (not enough memory?)");
				//}
			}
        }

        if (firstResult && m_ccRoot)
            m_ccRoot->selectEntity(firstResult);
    }

    if (m_gsTool)
		m_gsTool->removeAllEntities(!deleteHiddenPoints);

    //we enable all GL windows
    enableAll();

    freezeUI(false);

    updateUI();

	ccGLWindow* win = getActiveGLWindow();
    if (win)
		win->redraw();
}

void MainWindow::activatePointListPickingMode()
{
    ccGLWindow* win = getActiveGLWindow();
    if (!win)
        return;

    //there should be only one point cloud in current selection!
    if (m_selectedEntities.empty() || m_selectedEntities.size()>1)
    {
        ccConsole::Error("Select one and only one entity!");
        return;
    }

	ccPointCloud* pc = ccHObjectCaster::ToPointCloud(m_selectedEntities[0]);
	if (!pc)
    {
        ccConsole::Error("Wrong type of entity");
        return;
    }

	if (!pc->isVisible() || !pc->isEnabled())
	{
        ccConsole::Error("Points must be visible!");
        return;
	}

    if (!m_plpDlg)
    {
        m_plpDlg = new ccPointListPickingDlg(this);
        connect(m_plpDlg, SIGNAL(processFinished(bool)),  this, SLOT(deactivatePointListPickingMode(bool)));

        registerMDIDialog(m_plpDlg,Qt::TopRightCorner);
    }

	//DGM: we must update marke size spin box value (as it may have changed by the user with the "display dialog")
	m_plpDlg->markerSizeSpinBox->setValue(ccGui::Parameters().pickedPointsSize);

    m_plpDlg->linkWith(win);
	m_plpDlg->linkWithCloud(pc);

    freezeUI(true);

    //we disable all other windows
    disableAllBut(win);

    if (!m_plpDlg->start())
        deactivatePointListPickingMode(false);
    else
        updateMDIDialogsPlacement();
}

void MainWindow::deactivatePointListPickingMode(bool state)
{
    if (m_plpDlg)
	{
        //m_plpDlg->linkWith(0);
		m_plpDlg->linkWithCloud(0);
	}

    //we enable all GL windows
    enableAll();

    freezeUI(false);

    updateUI();
}

void MainWindow::activatePointsPropertiesMode()
{
    ccGLWindow* win = getActiveGLWindow();
    if (!win)
        return;

	if (m_ccRoot)
		m_ccRoot->selectEntity(0); //we don't want any entity selected (especially existing labels!)

    if (!m_ppDlg)
    {
        m_ppDlg = new ccPointPropertiesDlg(this);
        connect(m_ppDlg, SIGNAL(processFinished(bool)),	this, SLOT(deactivatePointsPropertiesMode(bool)));
        connect(m_ppDlg, SIGNAL(newLabel(ccHObject*)),	this, SLOT(handleNewEntity(ccHObject*)));

        registerMDIDialog(m_ppDlg,Qt::TopRightCorner);
    }

    m_ppDlg->linkWith(win);

    freezeUI(true);

    //we disable all other windows
    disableAllBut(win);

    if (!m_ppDlg->start())
        deactivatePointsPropertiesMode(false);
    else
        updateMDIDialogsPlacement();
}

void MainWindow::deactivatePointsPropertiesMode(bool state)
{
    //if (m_ppDlg)
    //    m_ppDlg->linkWith(0);

    //we enable all GL windows
    enableAll();

    freezeUI(false);

    updateUI();
}

void MainWindow::activateClippingBoxMode()
{
    size_t selNum=m_selectedEntities.size();
    if (selNum==0)
        return;

    ccGLWindow* win = getActiveGLWindow();
    if (!win)
        return;

    if (!m_clipTool)
        m_clipTool = new ccClippingBoxTool(this);
	m_clipTool->linkWith(win);

	ccHObject* entity = m_selectedEntities[0];
	if (!m_clipTool->setAssociatedEntity(entity))
    {
        ccConsole::Error("Select a point cloud!");
        return;
    }

    if (m_clipTool->start())
	{
		//automatically deselect the entity (to avoid seeing its bounding box ;)
		m_ccRoot->unselectEntity(entity);
		connect(m_clipTool, SIGNAL(processFinished(bool)), this, SLOT(deactivateClippingBoxMode(bool)));
		registerMDIDialog(m_clipTool,Qt::TopRightCorner);
		freezeUI(true);
        updateMDIDialogsPlacement();
		//deactivate all other GL windows
		disableAllBut(win);
	}
	else
	{
		ccConsole::Error("Unexpected error!"); //indeed...
	}
}

void MainWindow::deactivateClippingBoxMode(bool state)
{
    //we reactivate all GL windows
    enableAll();

    freezeUI(false);

    updateUI();
}

void MainWindow::activateTranslateRotateMode()
{
    size_t i,selNum=m_selectedEntities.size();
    if (selNum==0)
        return;

    ccGLWindow* win = getActiveGLWindow();
    if (!win)
        return;

    if (!m_transTool)
        m_transTool = new ccGraphicalTransformationTool(this);
	assert(m_transTool->getNumberOfValidEntities()==0);
	m_transTool->linkWith(win);

	bool rejectedEntities = false;
    for (i=0; i<selNum;++i)
    {
        ccHObject* entity = m_selectedEntities[i];
		if (!m_transTool->addEntity(entity))
			rejectedEntities = true;
    }

    if (m_transTool->getNumberOfValidEntities()==0)
    {
        ccConsole::Error("No entity elligible for manual transformation! (see console)");
        return;
    }
	else if (rejectedEntities)
	{
        ccConsole::Error("Some entities were ingored! (see console)");
	}

    //try to activate "moving mode" in current GL window
    if (m_transTool->start())
	{
		connect(m_transTool, SIGNAL(processFinished(bool)), this, SLOT(deactivateTranslateRotateMode(bool)));
		registerMDIDialog(m_transTool,Qt::TopRightCorner);
		freezeUI(true);
        updateMDIDialogsPlacement();
		//deactivate all other GL windows
		disableAllBut(win);
	}
	else
	{
		ccConsole::Error("Unexpected error!"); //indeed...
	}
}

void MainWindow::deactivateTranslateRotateMode(bool state)
{
	//if (m_transTool)
	//	m_transTool->close();

    //we reactivate all GL windows
    enableAll();

    freezeUI(false);

    updateUI();
}

void MainWindow::testFrameRate()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
        win->testFrameRate();
}

void MainWindow::setTopView()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
        win->setView(CC_TOP_VIEW);
}

void MainWindow::setBottomView()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
        win->setView(CC_BOTTOM_VIEW);
}

void MainWindow::setFrontView()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
        win->setView(CC_FRONT_VIEW);
}

void MainWindow::setBackView()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
        win->setView(CC_BACK_VIEW);
}

void MainWindow::setLeftView()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
        win->setView(CC_LEFT_VIEW);
}

void MainWindow::setRightView()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
        win->setView(CC_RIGHT_VIEW);
}

void MainWindow::setIsoView1()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
        win->setView(CC_ISO_VIEW_1);
}

void MainWindow::setIsoView2()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
        win->setView(CC_ISO_VIEW_2);
}

void MainWindow::setLightsAndMaterials()
{
	ccDisplayOptionsDlg colorsDlg(this);
	connect(&colorsDlg, SIGNAL(aspectHasChanged()), this, SLOT(redrawAll()));

	colorsDlg.exec();

	disconnect(&colorsDlg, 0, 0, 0);
}

void MainWindow::doActionRenderToFile()
{
    ccGLWindow* win = getActiveGLWindow();
    if (!win)
        return;

    ccRenderToFileDlg rtfDlg(win->width(),win->height(),this);

    if (rtfDlg.exec())
    {
        QApplication::processEvents();
        win->renderToFile(qPrintable(rtfDlg.getFilename()),rtfDlg.getZoom(),rtfDlg.dontScalePoints());
    }
}

void MainWindow::doActionEditCamera()
{
    //current active MDI area
    QMdiSubWindow* qWin = m_mdiArea->activeSubWindow();
    if (!qWin)
        return;

    if (!m_cpeDlg)
    {
        m_cpeDlg = new ccCameraParamEditDlg(qWin);
		//m_cpeDlg->makeFrameless(); //does not work on linux
        m_cpeDlg->linkWith(qWin);
        connect(m_mdiArea, SIGNAL(subWindowActivated(QMdiSubWindow*)), m_cpeDlg, SLOT(linkWith(QMdiSubWindow*)));

        registerMDIDialog(m_cpeDlg,Qt::BottomLeftCorner);
    }

    m_cpeDlg->start();

    updateMDIDialogsPlacement();
}

static unsigned s_viewportIndex = 0;
void MainWindow::doActionSaveViewportAsCamera()
{
    ccGLWindow* win = getActiveGLWindow();
    if (!win)
        return;
 
	cc2DViewportObject* viewportObject = new cc2DViewportObject(QString("Viewport #%1").arg(++s_viewportIndex));
	viewportObject->setParameters(win->getViewportParameters());

	addToDB(viewportObject,true,0,false,false);
}

void MainWindow::setGlobalZoom()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
        win->zoomGlobal();
}

void MainWindow::zoomOnSelectedEntities()
{
    ccGLWindow* win = getActiveGLWindow();
    if (!win)
        return;

    ccHObject tempGroup("TempGroup");
    size_t selNum = m_selectedEntities.size();
    for (size_t i=0; i<selNum; ++i)
    {
        if (m_selectedEntities[i]->getDisplay()==win)
            tempGroup.addChild(m_selectedEntities[i],false);
    }

    if (tempGroup.getChildrenNumber()>0)
    {
        ccBBox box = tempGroup.getBB(false, false, win);
        win->updateConstellationCenterAndZoom(&box);
    }

    refreshAll();
}

void MainWindow::setPivotAlwaysOn()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
    {
		win->setPivotVisibility(ccGLWindow::PIVOT_ALWAYS_SHOW);
        win->redraw();

		//update pop-up menu 'top' icon
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setIcon(actionSetPivotAlwaysOn->icon());
    }
}

void MainWindow::setPivotRotationOnly()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
    {
		win->setPivotVisibility(ccGLWindow::PIVOT_SHOW_ON_MOVE);
        win->redraw();

		//update pop-up menu 'top' icon
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setIcon(actionSetPivotRotationOnly->icon());
    }
}

void MainWindow::setPivotOff()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
    {
		win->setPivotVisibility(ccGLWindow::PIVOT_HIDE);
        win->redraw();

		//update pop-up menu 'top' icon
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setIcon(actionSetPivotOff->icon());
    }
}

void MainWindow::setOrthoView()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
    {
        win->setPerspectiveState(false,true);
        win->redraw();

		//update pop-up menu 'top' icon
		if (m_viewModePopupButton)
			m_viewModePopupButton->setIcon(actionSetOrthoView->icon());
    }
}

void MainWindow::setCenteredPerspectiveView()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
    {
        win->setPerspectiveState(true,true);
        win->redraw();

		//update pop-up menu 'top' icon
		if (m_viewModePopupButton)
			m_viewModePopupButton->setIcon(actionSetCenteredPerspectiveView->icon());
    }
}

void MainWindow::setViewerPerspectiveView()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
    {
        win->setPerspectiveState(true,false);
        win->redraw();

		//update pop-up menu 'top' icon
		if (m_viewModePopupButton)
			m_viewModePopupButton->setIcon(actionSetViewerPerspectiveView->icon());
    }
}

static ccGLWindow* s_pickingWindow = 0;
void MainWindow::doPickRotationCenter()
{
	if (s_pickingWindow)
	{
		s_pickingWindow->displayNewMessage(QString(),ccGLWindow::LOWER_LEFT_MESSAGE); //clear previous messages
		s_pickingWindow->displayNewMessage("Rotation center picking aborted",ccGLWindow::LOWER_LEFT_MESSAGE);
		s_pickingWindow->redraw();
		cancelPickRotationCenter();
		return;
	}

    ccGLWindow* win = getActiveGLWindow();
    if (!win)
	{
		ccConsole::Error("No active 3D view!");
        return;
	}

	bool objectCentered = true;
	bool perspectiveEnabled = win->getPerspectiveState(objectCentered);
	if (perspectiveEnabled && !objectCentered)
	{
		ccLog::Error("Perspective mode is viewer-centered: can't use a point as rotation center!");
		return;
	}

	//we need at least one cloud
	//bool atLeastOneCloudVisible = false;
	//{
	//	assert(m_ccRoot && m_ccRoot->getRootEntity());
	//	ccHObject::Container clouds;
	//	m_ccRoot->getRootEntity()->filterChildren(clouds,true,CC_POINT_CLOUD);
	//	for (unsigned i=0;i<clouds.size();++i)
	//		if (clouds[i]->isVisible() && clouds[i]->isEnabled())
	//		{
	//			//we must check that the cloud is really visible: i.e. it's parent are not deactivated!
	//			atLeastOneCloudVisible = true;
	//			ccHObject* parent = clouds[i]->getParent();
	//			while (parent)
	//			{
	//				if (!parent->isEnabled())
	//				{
	//					atLeastOneCloudVisible = false;
	//					break;
	//				}
	//				parent = parent->getParent();
	//			}
	//			if (atLeastOneCloudVisible)
	//				break;
	//		}
	//}
			
	//if (!atLeastOneCloudVisible)
	//{
	//	ccConsole::Error("No visible cloud in active 3D view!");
	//	return;
	//}

	connect(win, SIGNAL(pointPicked(int, unsigned, int, int)), this, SLOT(processPickedRotationCenter(int, unsigned, int, int)));
	win->setPickingMode(ccGLWindow::POINT_PICKING);
	win->displayNewMessage("Pick a point to be used as rotation center (click on icon again to cancel)",ccGLWindow::LOWER_LEFT_MESSAGE,true,3600);
	win->redraw();
	s_pickingWindow = win;

	freezeUI(true);
}

void MainWindow::processPickedRotationCenter(int cloudUniqueID, unsigned pointIndex, int, int)
{
	if (s_pickingWindow)
	{
		ccHObject* obj = 0;
		ccHObject* db = s_pickingWindow->getSceneDB();
		if (db)
			obj = db->find(cloudUniqueID);
		if (obj && obj->isKindOf(CC_POINT_CLOUD))
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(obj);
			const CCVector3* P = cloud->getPoint(pointIndex);
			if (P)
			{
				unsigned precision = ccGui::Parameters().displayedNumPrecision;
				s_pickingWindow->displayNewMessage(QString(),ccGLWindow::LOWER_LEFT_MESSAGE,false); //clear precedent message
				s_pickingWindow->displayNewMessage(QString("Point (%1,%2,%3) set as rotation center").arg(P->x,0,'f',precision).arg(P->y,0,'f',precision).arg(P->z,0,'f',precision),ccGLWindow::LOWER_LEFT_MESSAGE,true);

				const ccViewportParameters& params = s_pickingWindow->getViewportParameters();
				if (!params.perspectiveView || params.objectCenteredView)
				{
					CCVector3 newPivot = *P;
					//compute the equivalent camera center
					CCVector3 dP = params.pivotPoint - newPivot;
					CCVector3 MdP = dP; params.viewMat.applyRotation(MdP);
					CCVector3 newCameraPos = params.cameraCenter + MdP - dP;
					s_pickingWindow->setCameraPos(newCameraPos);
					s_pickingWindow->setPivotPoint(newPivot);
				}
				s_pickingWindow->redraw();
			}
		}
	}

	cancelPickRotationCenter();
}

void MainWindow::cancelPickRotationCenter()
{
	if (s_pickingWindow)
	{
		disconnect(s_pickingWindow, SIGNAL(pointPicked(int, unsigned, int, int)), this, SLOT(processPickedRotationCenter(int, unsigned, int, int)));
		s_pickingWindow->setPickingMode(ccGLWindow::DEFAULT_PICKING);
		s_pickingWindow=0;
	}

	freezeUI(false);
}

void MainWindow::toggleSelectedEntitiesVisibility()
{
	toggleSelectedEntitiesProp(0);
}

void MainWindow::toggleSelectedEntitiesColors()
{
	toggleSelectedEntitiesProp(1);
}

void MainWindow::toggleSelectedEntitiesNormals()
{
	toggleSelectedEntitiesProp(2);
}

void MainWindow::toggleSelectedEntitiesSF()
{
	toggleSelectedEntitiesProp(3);
}

void MainWindow::toggleSelectedEntitiesMaterials()
{
	toggleSelectedEntitiesProp(4);
}

void MainWindow::toggleSelectedEntities3DName()
{
	toggleSelectedEntitiesProp(5);
}

void MainWindow::toggleSelectedEntitiesProp(int prop)
{
	ccHObject::Container baseEntities;
	RemoveSiblings(m_selectedEntities,baseEntities);
    for (unsigned i=0; i<baseEntities.size(); ++i)
    {
		switch(prop)
		{
		case 0: //visibility
			baseEntities[i]->toggleVisibility_recursive();
			break;
		case 1: //colors
			baseEntities[i]->toggleColors_recursive();
			break;
		case 2: //normals
			baseEntities[i]->toggleNormals_recursive();
			break;
		case 3: //sf
			baseEntities[i]->toggleSF_recursive();
			break;
		case 4: //material/texture
			baseEntities[i]->toggleMaterials_recursive();
			break;
		case 5: //name in 3D
			baseEntities[i]->toggleShowName_recursive();
			break;
		default:
			assert(false);
		}
        baseEntities[i]->prepareDisplayForRefresh_recursive();
    }

    refreshAll();
	updateUI();
}

void MainWindow::showSelectedEntitiesHistogram()
{
	size_t selNum = m_selectedEntities.size();
	for (unsigned i=0; i<selNum; ++i)
	{
		ccGenericPointCloud* gc = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities[i]);

		//for "real" point clouds only
		if (gc->isA(CC_POINT_CLOUD))
		{
			ccPointCloud* cloud = static_cast<ccPointCloud*>(gc);
			//on affiche l'histogramme du champ scalaire courant
			ccScalarField* sf = static_cast<ccScalarField*>(cloud->getCurrentDisplayedScalarField());
			if (sf)
			{
				ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);

				const QString& cloudName = cloud->getName();
				hDlg->setWindowTitle(QString("Histogram [%1]").arg(cloudName));

				ccHistogramWindow* histogram = hDlg->window();
				{
					unsigned numberOfPoints = cloud->size();
					unsigned numberOfClasses = (unsigned)sqrt((double)numberOfPoints);
					//we take the 'nearest' multiple of 4
					numberOfClasses &= (~3);
					numberOfClasses = std::max<unsigned>(4,numberOfClasses);
					numberOfClasses = std::min<unsigned>(256,numberOfClasses);

					histogram->setInfoStr(QString("%1 (%2 values) ").arg(sf->getName()).arg(numberOfPoints));
					histogram->fromSF(sf,numberOfClasses);
				}
				hDlg->show();
			}
		}
	}
}

void MainWindow::doActionClone()
{
	ccHObject::Container selectedEntities = m_selectedEntities;
    size_t i,selNum=selectedEntities.size();

	ccHObject* lastClone=0;
    for (i=0;i<selNum;++i)
    {
        if (selectedEntities[i]->isKindOf(CC_POINT_CLOUD))
        {
            ccGenericPointCloud* clone = ccHObjectCaster::ToGenericPointCloud(selectedEntities[i])->clone();
            if (clone)
            {
                addToDB(clone, true, 0, true, false);
				lastClone=clone;
            }
            else
            {
                ccConsole::Error(QString("An error occured while cloning cloud %1").arg(selectedEntities[i]->getName()));
            }
        }
        else if (selectedEntities[i]->isKindOf(CC_PRIMITIVE))
        {
            ccGenericPrimitive* clone = static_cast<ccGenericPrimitive*>(selectedEntities[i])->clone();
            if (clone)
            {
                addToDB(clone, true, 0, true, false);
				lastClone=clone;
            }
            else
            {
				ccConsole::Error(QString("An error occured while cloning primitive %1").arg(selectedEntities[i]->getName()));
            }
        }
        else if (selectedEntities[i]->isKindOf(CC_MESH))
        {
            ccGenericMesh* clone = ccHObjectCaster::ToGenericMesh(selectedEntities[i])->clone();
            if (clone)
            {
                addToDB(clone, true, 0, true, false);
				lastClone=clone;
            }
            else
            {
				ccConsole::Error(QString("An error occured while cloning mesh %1").arg(selectedEntities[i]->getName()));
            }
        }
    }

	if (lastClone && m_ccRoot)
		m_ccRoot->selectEntity(lastClone->getUniqueID());

    updateUI();
}

static double s_constantSFValue = 0.0;
void MainWindow::doActionAddConstantSF()
{
    size_t selNum = m_selectedEntities.size();
    if (selNum!=1)
    {
		if (selNum>1)
			ccConsole::Error("Select only one point cloud or mesh!");
        return;
    }

	ccHObject* ent = m_selectedEntities[0];

	bool lockedVertices;
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent,&lockedVertices);

	//for "real" point clouds only
	if (!cloud)
		return;
	if (lockedVertices)
	{
		//see ccPropertiesTreeDelegate::fillWithMesh
		if (!ent->isA(CC_MESH_GROUP) && !ent->isAncestorOf(cloud))
		{
			DisplayLockedVerticesWarning();
			return;
		}
	}

	QString defaultName = "Constant";
	unsigned trys = 1;
	while (cloud->getScalarFieldIndexByName(qPrintable(defaultName))>=0 || trys>99)
		defaultName = QString("Constant #%1").arg(++trys);

	//ask for a name
	bool ok;
	QString sfName = QInputDialog::getText(this,"New SF name", "SF name (must be unique)", QLineEdit::Normal, defaultName, &ok);
	if (!ok)
		return;
	if (sfName.isNull())
	{
		ccLog::Error("Invalid name");
		return;
	}
	if (cloud->getScalarFieldIndexByName(qPrintable(sfName))>=0)
	{
		ccLog::Error("Name already exists!");
		return;
	}

	double sfValue = QInputDialog::getDouble(this,"SF value", "value", s_constantSFValue, -DBL_MAX, DBL_MAX, 8, &ok);
	if (!ok)
		return;

	int pos = cloud->addScalarField(qPrintable(sfName));
	if (pos<0)
	{
		ccLog::Error("An error occured! (see console)");
		return;
	}
	
	CCLib::ScalarField* sf = cloud->getScalarField(pos);
	assert(sf);
	if (sf)
	{
		sf->fill(sfValue);
		sf->computeMinAndMax();
		cloud->setCurrentDisplayedScalarField(pos);
		cloud->showSF(true);
		updateUI();
		if (cloud->getDisplay())
			cloud->getDisplay()->redraw();
	}

	ccLog::Print(QString("New scalar field added to %1 (constant value: %2)").arg(cloud->getName()).arg(sfValue));
}

QString GetFirstAvailableSFName(ccPointCloud* cloud, const QString& baseName)
{
	if (!cloud)
	{
		assert(false);
		return QString();
	}

	QString name = baseName;
	unsigned trys = 0;
	while (cloud->getScalarFieldIndexByName(qPrintable(name)) >= 0 || trys > 99)
		name = QString("%1 #%2").arg(baseName).arg(++trys);

	if (trys > 99)
		return QString();
	return name;
}

void MainWindow::doActionScalarFieldFromColor()
{
	//candidates
    std::set<ccPointCloud*> clouds;
	{
		for (size_t i = 0; i < m_selectedEntities.size(); ++i)
		{
			ccHObject* ent = m_selectedEntities[i];
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
			if (cloud && ent->hasColors()) //only for clouds (or vertices)
				clouds.insert( cloud );
		}
	}

    if (clouds.empty())
        return;

    ccScalarFieldFromColorDlg dialog(this);
    if (!dialog.exec())
        return;

    bool exportR = dialog.getRStatus();
    bool exportG = dialog.getGStatus();
    bool exportB = dialog.getBStatus();
    bool exportC = dialog.getCompositeStatus();

	for (std::set<ccPointCloud*>::const_iterator it = clouds.begin(); it != clouds.end(); ++it)
    {
		ccPointCloud* cloud = *it;
		
        std::vector<ccScalarField*> fields(4);
		fields[0] = (exportR ? new ccScalarField(qPrintable(GetFirstAvailableSFName(cloud,"R"))) : 0);
		fields[1] = (exportG ? new ccScalarField(qPrintable(GetFirstAvailableSFName(cloud,"G"))) : 0);
		fields[2] = (exportB ? new ccScalarField(qPrintable(GetFirstAvailableSFName(cloud,"B"))) : 0);
		fields[3] = (exportC ? new ccScalarField(qPrintable(GetFirstAvailableSFName(cloud,"Composite"))) : 0);

		//try to instantiate memory for each field
		{
			unsigned count = cloud->size();
			for (size_t i=0; i<fields.size(); ++i)
			{
				if (fields[i] && !fields[i]->reserve(count))
				{
					ccLog::Warning(QString("[doActionScalarFieldFromColor] Not enough memory to instantiate SF '%1' on cloud '%2'").arg(fields[i]->getName()).arg(cloud->getName()));
					fields[i]->release();
					fields[i]=0;
				}
			}
		}

		//export points
		for (unsigned j=0; j<cloud->size(); ++j)
		{
			const colorType* rgb = cloud->getPointColor(j);

			if (fields[0])
				fields[0]->setValue(j, rgb[0]);
			if (fields[1])
				fields[1]->setValue(j, rgb[1]);
			if (fields[2])
				fields[2]->setValue(j, rgb[2]);
			if (fields[3])
				fields[3]->setValue(j, (ScalarType)(rgb[0] + rgb[1] + rgb[2])/(ScalarType)3.0 );
		}

		QString fieldsStr;
		{
			for (size_t i=0; i<fields.size(); ++i)
			{
				if (fields[i])
				{
					fields[i]->computeMinAndMax();
					
					int sfIdx = cloud->addScalarField(fields[i]);
					cloud->setCurrentDisplayedScalarField(sfIdx);
					cloud->showSF(true);
					cloud->refreshDisplay();

					//mesh vertices?
					if (cloud->getParent() && cloud->getParent()->isKindOf(CC_MESH))
					{
						cloud->getParent()->showSF(true);
						cloud->getParent()->refreshDisplay();
					}
					
					if (!fieldsStr.isEmpty())
						fieldsStr.append(", ");
					fieldsStr.append(fields[i]->getName());
				}
			}
		}

		if (!fieldsStr.isEmpty())
			ccLog::Print(QString("[doActionScalarFieldFromColor] New scalar fields (%1) added to '%2'").arg(fieldsStr).arg(cloud->getName()));
	}

	refreshAll();
    updateUI();
}

void MainWindow::doActionScalarFieldArithmetic()
{
    assert(!m_selectedEntities.empty());

    ccHObject* entity = m_selectedEntities[0];
	bool lockedVertices;
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity,&lockedVertices);
	if (lockedVertices)
	{
		DisplayLockedVerticesWarning();
		return;
	}
    if (!cloud)
        return;

    ccScalarFieldArithmeticDlg sfaDlg(cloud,this);

    if (!sfaDlg.exec())
        return;

    int sf1Idx = sfaDlg.getSF1Index();
    int sf2Idx = sfaDlg.getSF2Index();
    ccScalarFieldArithmeticDlg::Operation op = sfaDlg.getOperation();

    if (sf1Idx<0 || sf2Idx<0)
        return;

    CCLib::ScalarField* sf1 = cloud->getScalarField(sf1Idx);
    CCLib::ScalarField* sf2 = cloud->getScalarField(sf2Idx);

    if (!sf1 || !sf2)
        return;

    QString opStr;
    switch (op)
    {
    case ccScalarFieldArithmeticDlg::PLUS:
        opStr = "+";
        break;
    case ccScalarFieldArithmeticDlg::MINUS:
        opStr = "-";
        break;
    case ccScalarFieldArithmeticDlg::MULTIPLY:
        opStr = "*";
        break;
    case ccScalarFieldArithmeticDlg::DIVIDE:
        opStr = "/";
        break;
    }
    QString sfName = QString("(SF#%1").arg(sf1Idx)+opStr+QString("SF#%1)").arg(sf2Idx);

	int sfIdx = cloud->getScalarFieldIndexByName(qPrintable(sfName));
	if (sfIdx>=0)
	{
		if (sfIdx==sf1Idx || sfIdx==sf2Idx)
		{
			ccConsole::Error(QString("Resulting scalar field will have the same name\nas one of the operand (%1)! Rename it first...").arg(sfName));
			return;
		}

		if (QMessageBox::warning(	this,
									"Same scalar field name",
									"Resulting scalar field already exists! Overwrite it?",
									QMessageBox::Ok | QMessageBox::Cancel,
									QMessageBox::Ok ) != QMessageBox::Ok)
		{
			return;
		}

		cloud->deleteScalarField(sfIdx);
	}

    sfIdx = cloud->addScalarField(qPrintable(sfName));
    if (sfIdx<0)
    {
        ccConsole::Error("Failed to create destination scalar field! (not enough memory?)");
        return;
    }
    CCLib::ScalarField* sfDest = cloud->getScalarField(sfIdx);

	unsigned valCount = sf1->currentSize();
    if (!sfDest->resize(valCount))
	{
		ccConsole::Error("Not enough memory!");
		sfDest->release();
		return;
	}

    assert(valCount == sf2->currentSize() && valCount == sfDest->currentSize());

    for (unsigned i=0; i<valCount; ++i)
    {
		ScalarType val = NAN_VALUE;

        //we must handle 'invalid' values
		const ScalarType& val1 = sf1->getValue(i);
		if (ccScalarField::ValidValue(val1))
        {
            const ScalarType& val2 = sf2->getValue(i);
            if (ccScalarField::ValidValue(val2))
            {
                switch (op)
                {
                case ccScalarFieldArithmeticDlg::PLUS:
                    val = val1 + val2;
                    break;
                case ccScalarFieldArithmeticDlg::MINUS:
                    val = val1 - val2;
                    break;
                case ccScalarFieldArithmeticDlg::MULTIPLY:
                    val = val1 * val2;
                    break;
                case ccScalarFieldArithmeticDlg::DIVIDE:
                    val = val1 / val2;
                    break;
                }
            }
        }

		sfDest->setValue(i,val);
    }

    sfDest->computeMinAndMax();
    cloud->setCurrentDisplayedScalarField(sfIdx);
	cloud->showSF(sfIdx>=0);
    cloud->prepareDisplayForRefresh_recursive();

    refreshAll();
	updateUI();
}

void MainWindow::doComputePlaneOrientation()
{
	ccHObject::Container selectedEntities = m_selectedEntities;
    size_t i,selNum=selectedEntities.size();
    if (selNum<1)
        return;

    for (i=0;i<selNum;++i)
    {
        //is the ith selected data is elligible for processing?
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(selectedEntities[i]);
        if (cloud)
        {
			double rms=0.0;
			ccPlane* pPlane = cloud->fitPlane(&rms);

			if (!pPlane)
			{
				ccConsole::Warning(QString("\tWarning: failed to fit a plane on cloud '%1'").arg(cloud->getName()));
			}
			else
			{
				//as all information appears in Console...
				forceConsoleDisplay();

				ccConsole::Print(QString("[Orientation] cloud '%1'").arg(cloud->getName()));
				ccConsole::Print("\t- plane fitting RMS: %f",rms);

				const ccGLMatrix& planteTrans = pPlane->getTransformation();
				CCVector3 N = pPlane->getNormal(); //plane normal

				//We always consider the normal with a positive 'Z' by default!
				if(N.z < 0.0)
					N *= -1.0;
				ccConsole::Print("\t- normal: (%f,%f,%f)",N.x,N.y,N.z);

				//we compute strike & dip by the way
				double strike,dip;
				ccNormalVectors::ConvertNormalToStrikeAndDip(N,strike,dip);

                int iStrike = (int)strike;
                int iDip = (int)dip;
				QString strikeAndDipStr = QString("N%1°E - %2°SE").arg(iStrike,3,10,QChar('0')).arg(iDip,3,10,QChar('0'));
				ccConsole::Print(QString("\t- strike/dip: %1").arg(strikeAndDipStr));

				//hack: output the transformation matrix that would make this normal points towards +Z
				ccGLMatrix makeZPosMatrix = ccGLMatrix::FromToRotation(N,CCVector3(0,0,1.0f));
				CCVector3 G = *CCLib::Neighbourhood(cloud).getGravityCenter();
				CCVector3 Gt = G;
				makeZPosMatrix.applyRotation(Gt.u);
				makeZPosMatrix.setTranslation(G-Gt);
				makeZPosMatrix.invert();
				ccConsole::Print("[Orientation] A matrix that would make this plane horizontal (normal towards Z+) is:");
				const float* mat = makeZPosMatrix.data();
				ccConsole::Print("%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f",mat[0],mat[4],mat[8],mat[12],mat[1],mat[5],mat[9],mat[13],mat[2],mat[6],mat[10],mat[14],mat[3],mat[7],mat[11],mat[15]);
				ccConsole::Print("[Orientation] You can copy this matrix values (CTRL+C) and paste them in the 'Apply transformation tool' dialog");

				pPlane->setName(QString("Strike plane ")+strikeAndDipStr);
				pPlane->applyGLTransformation_recursive(); //not yet in DB
				cloud->addChild(pPlane);
				pPlane->setDisplay(cloud->getDisplay());
				pPlane->setVisible(true);
				pPlane->enableStippling(true);
				pPlane->setSelectionBehavior(ccHObject::SELECTION_FIT_BBOX);
				cloud->prepareDisplayForRefresh_recursive();
				addToDB(pPlane);
			}
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doShowPrimitiveFactory()
{
	if (!m_pfDlg)
		m_pfDlg = new ccPrimitiveFactoryDlg(this);

	m_pfDlg->setModal(false);
	m_pfDlg->setWindowModality(Qt::NonModal);
	m_pfDlg->show();
}

void MainWindow::doComputeDensity()
{
	if (!ApplyCCLibAlgortihm(CCLIB_ALGO_DENSITY,m_selectedEntities,this))
		return;
    refreshAll();
	updateUI();
}

void MainWindow::doComputeCurvature()
{
    if (!ApplyCCLibAlgortihm(CCLIB_ALGO_CURVATURE,m_selectedEntities,this))
		return;
    refreshAll();
	updateUI();
}

void MainWindow::doActionSFGradient()
{
    if (!ApplyCCLibAlgortihm(CCLIB_ALGO_SF_GRADIENT,m_selectedEntities,this))
		return;
    refreshAll();
	updateUI();
}

void MainWindow::doComputeRoughness()
{
    if (!ApplyCCLibAlgortihm(CCLIB_ALGO_ROUGHNESS,m_selectedEntities,this))
		return;
    refreshAll();
	updateUI();
}

void MainWindow::doSphericalNeighbourhoodExtractionTest()
{
    if (!ApplyCCLibAlgortihm(CCLIB_SPHERICAL_NEIGHBOURHOOD_EXTRACTION_TEST,m_selectedEntities,this))
		return;
    refreshAll();
	updateUI();
}

bool MainWindow::ApplyCCLibAlgortihm(CC_LIB_ALGORITHM algo, ccHObject::Container& entities, QWidget *parent/*=0*/, void** additionalParameters/*=0*/)
{
    size_t i,selNum=entities.size();
    if (selNum<1)
        return false;

    //generic parameters
    QString sfName;

    //curvature parameters
	double curvKernelSize = -1.0;
	CCLib::Neighbourhood::CC_CURVATURE_TYPE curvType = CCLib::Neighbourhood::GAUSSIAN_CURV;

    //computeScalarFieldGradient parameters
    bool euclidian=false;

    //computeRoughness parameters
    float roughnessKernelSize = 1.0;

    switch (algo)
    {
        case CCLIB_ALGO_DENSITY:
			{
				sfName = CC_LOCAL_DENSITY_FIELD_NAME;
			}
			break;
        
		case CCLIB_ALGO_CURVATURE:
			{
				//parameters already provided?
				if (additionalParameters)
				{
					curvType = *(CCLib::Neighbourhood::CC_CURVATURE_TYPE*)additionalParameters[0];
					curvKernelSize = *(double*)additionalParameters[1];
				}
				else //ask the user!
				{
					curvKernelSize = GetDefaultCloudKernelSize(entities);
					if (curvKernelSize<0.0)
					{
						ccConsole::Error("No elligible point cloud in selection!");
						return false;
					}
					ccCurvatureDlg curvDlg(0);
					if (selNum==1)
						curvDlg.setKernelSize(curvKernelSize);
					if (!curvDlg.exec())
						return false;

					curvType = curvDlg.getCurvatureType();
					curvKernelSize = curvDlg.getKernelSize();
				}

				if (curvType == CCLib::Neighbourhood::MEAN_CURV)
					sfName = QString(CC_MEAN_CURVATURE_FIELD_NAME);
				else //if (curvType == CCLib::Neighbourhood::GAUSSIAN_CURV)
					sfName = QString(CC_GAUSSIAN_CURVATURE_FIELD_NAME);
				sfName += QString("(%1)").arg(curvKernelSize);
			}
            break;

        case CCLIB_ALGO_SF_GRADIENT:
			{
				sfName = CC_GRADIENT_NORMS_FIELD_NAME;
				//parameters already provided?
				if (additionalParameters)
				{
					euclidian = *(bool*)additionalParameters[0];
				}
				else //ask the user!
				{
					euclidian = ( QMessageBox::question(parent,
														"Gradient",
														"Is the scalar field composed of (euclidian) distances?",
														QMessageBox::Yes | QMessageBox::No,
														QMessageBox::No ) == QMessageBox::Yes );
				}
			}
            break;

        case CCLIB_ALGO_ROUGHNESS:
		case CCLIB_SPHERICAL_NEIGHBOURHOOD_EXTRACTION_TEST:
            {
				//parameters already provided?
				if (additionalParameters)
				{
					roughnessKernelSize = *(float*)additionalParameters[0];
				}
				else //ask the user!
				{
					roughnessKernelSize = GetDefaultCloudKernelSize(entities);
					if (roughnessKernelSize<0.0)
					{
						ccConsole::Error("No elligible point cloud in selection!");
						return false;
					}
					ccAskOneDoubleValueDlg dlg("Kernel size", DBL_MIN, DBL_MAX, roughnessKernelSize, 8, 0, 0);
					if (!dlg.exec())
						return false;
					roughnessKernelSize = (float)dlg.dValueSpinBox->value();
				}
				
				sfName = QString(CC_ROUGHNESS_FIELD_NAME)+QString("(%1)").arg(roughnessKernelSize);
            }
            break;
        
		default:
            assert(false);
            return false;
    }

    for (i=0;i<selNum;++i)
    {
        //is the ith selected data is elligible for processing?
        ccGenericPointCloud* cloud =0;
        switch(algo)
        {
            case CCLIB_ALGO_SF_GRADIENT:
                //for scalar field gradient, we can apply it directly on meshes
				bool lockedVertices;
				cloud = ccHObjectCaster::ToGenericPointCloud(entities[i],&lockedVertices);
				if (lockedVertices)
				{
					DisplayLockedVerticesWarning();
					cloud=0;
				}
                if (cloud)
                {
                    //but we need an already displayed SF!
                    if (cloud->isA(CC_POINT_CLOUD))
                    {
                        ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
                        //on met en lecture (OUT) le champ scalaire actuellement affiche
                        int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
                        if (outSfIdx<0)
                        {
                            cloud=0;
                        }
                        else
                        {
                            pc->setCurrentOutScalarField(outSfIdx);
                            sfName = QString("%1(%2)").arg(CC_GRADIENT_NORMS_FIELD_NAME).arg(pc->getScalarFieldName(outSfIdx));
                        }
                    }
                    else //if (!cloud->hasDisplayedScalarField()) //TODO: displayed but not necessarily set as OUTPUT!
                    {
                        cloud=0;
                    }
                }
                break;
            //by default, we apply processings on clouds only
            default:
                if (entities[i]->isKindOf(CC_POINT_CLOUD))
					cloud = ccHObjectCaster::ToGenericPointCloud(entities[i]);
                break;
        }

        if (cloud)
        {
            ccPointCloud* pc = 0;
            int sfIdx = -1;
            if (cloud->isA(CC_POINT_CLOUD))
            {
                pc = static_cast<ccPointCloud*>(cloud);

                sfIdx = pc->getScalarFieldIndexByName(qPrintable(sfName));
                if (sfIdx<0)
                    sfIdx = pc->addScalarField(qPrintable(sfName));
                if (sfIdx>=0)
                    pc->setCurrentInScalarField(sfIdx);
                else
                {
                    ccConsole::Error(QString("Failed to create scalar field on cloud '%1' (not enough memory?)").arg(pc->getName()));
                    continue;
                }
            }

			ccProgressDialog pDlg(true,parent);

            ccOctree* octree = cloud->getOctree();
            if (!octree)
			{
				pDlg.show();
                octree = cloud->computeOctree(&pDlg);
				if (!octree)
				{
					ccConsole::Error(QString("Couldn't compute octree for cloud '%1'!").arg(cloud->getName()));
					break;
				}
			}

            int result = 0;
			QElapsedTimer eTimer;
			eTimer.start();
            switch(algo)
            {
                case CCLIB_ALGO_DENSITY:
                    result = CCLib::GeometricalAnalysisTools::computeLocalDensity(cloud,&pDlg,octree);
                    break;
                case CCLIB_ALGO_CURVATURE:
                    result = CCLib::GeometricalAnalysisTools::computeCurvature(cloud,
                                                                                curvType,
                                                                                curvKernelSize,
                                                                                &pDlg,
                                                                                octree);
                    break;
                case CCLIB_ALGO_SF_GRADIENT:
                    result = CCLib::ScalarFieldTools::computeScalarFieldGradient(cloud,euclidian,false,&pDlg,octree);

                    if (result == 0)
                    {
                        int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
                        assert(outSfIdx>=0);
                        sfName = QString("%1.gradient").arg(pc->getScalarFieldName(outSfIdx));
                        pc->renameScalarField(outSfIdx,qPrintable(sfName));
                    }
                    //*/
                    break;
                case CCLIB_ALGO_ROUGHNESS:
                    result = CCLib::GeometricalAnalysisTools::computeRoughness(cloud,
                                                                                roughnessKernelSize,
                                                                                &pDlg,
                                                                                octree);
                    break;

				//TEST
				case CCLIB_SPHERICAL_NEIGHBOURHOOD_EXTRACTION_TEST:
				{
					unsigned i,count = cloud->size();
					if (count<3)
					{
						result = -2;
						break;
					}

					cloud->enableScalarField();
					for (i=0;i<count;++i)
						cloud->setPointScalarValue(i,-1.0);

					QElapsedTimer subTimer;
					subTimer.start();
					double extractedPoints=0.0;
					for (i=0;i<1000;++i)
					{
						unsigned randIndex = ((unsigned)((float)rand()*(float)count/(float)RAND_MAX))%count;
						CCLib::DgmOctree::NeighboursSet neighbours;
						octree->getPointsInSphericalNeighbourhood(*cloud->getPoint(randIndex),roughnessKernelSize,neighbours);
						size_t neihgboursCount=neighbours.size();
						extractedPoints += (double)neihgboursCount;
						for (size_t k=0;k<neihgboursCount;++k)
							cloud->setPointScalarValue(neighbours[k].pointIndex,sqrt(neighbours[k].squareDist));
					}
					ccConsole::Print("[CCLIB_SPHERICAL_NEIGHBOURHOOD_EXTRACTION_TEST] Mean extraction time = %3.3f ms (radius = %f, mean(neighbours)=%3.1f)",subTimer.elapsed(),roughnessKernelSize,extractedPoints/1000.0);

					result = 0;
				}
				break;

                default:
                    //missed something?
                    assert(false);
            }
			int elapsedTime_ms = eTimer.elapsed();

            if (result == 0)
            {
                if (pc && sfIdx>=0)
                {
                    pc->setCurrentDisplayedScalarField(sfIdx);
					pc->showSF(sfIdx>=0);
                    pc->getCurrentInScalarField()->computeMinAndMax();
                }
                cloud->prepareDisplayForRefresh();
				ccConsole::Print("[Algortihm] Timing: %3.2f s.",elapsedTime_ms/1.0e3);
            }
            else
            {
                ccConsole::Warning(QString("Failed to apply processing to cloud '%1'").arg(cloud->getName()));
                if (pc && sfIdx>=0)
                {
                    pc->deleteScalarField(sfIdx);
                    sfIdx = -1;
                }
            }
        }
    }

	return true;
}

void MainWindow::doActionCloudCloudDist()
{
    size_t selNum = m_selectedEntities.size();
    if (selNum!=2)
    {
        ccConsole::Error("Select 2 point clouds!");
        return;
    }

    if (!m_selectedEntities[0]->isKindOf(CC_POINT_CLOUD) ||
            !m_selectedEntities[1]->isKindOf(CC_POINT_CLOUD))
    {
        ccConsole::Error("Select 2 point clouds!");
        return;
    }

    ccOrderChoiceDlg dlg(m_selectedEntities[0], "Compared", m_selectedEntities[1], "Reference", this);
    if (!dlg.exec())
        return;

    ccGenericPointCloud* compCloud = ccHObjectCaster::ToGenericPointCloud(dlg.getFirstEntity());
    ccGenericPointCloud* refCloud = ccHObjectCaster::ToGenericPointCloud(dlg.getSecondEntity());

    //assert(!m_compDlg);
	if (m_compDlg)
		delete m_compDlg;
    m_compDlg = new ccComparisonDlg(compCloud, refCloud, ccComparisonDlg::CLOUDCLOUD_DIST, this);
    connect(m_compDlg, SIGNAL(finished(int)), this, SLOT(deactivateComparisonMode(int)));
    m_compDlg->show();
    //cDlg.setModal(false);
    //cDlg.exec();
    freezeUI(true);
}

void MainWindow::doActionCloudMeshDist()
{
    size_t selNum = m_selectedEntities.size();
    if (selNum!=2)
    {
        ccConsole::Error("Select 2 entities!");
        return;
    }

    bool isMesh[2]={false,false};
    int meshNum=0;
    int cloudNum=0;
    for (int i=0;i<2;++i)
    {
        if (m_selectedEntities[i]->isKindOf(CC_MESH))
        {
            ++meshNum;
            isMesh[i]=true;
        }
        else if (m_selectedEntities[i]->isKindOf(CC_POINT_CLOUD))
        {
            ++cloudNum;
        }
    }

    if (meshNum==0)
    {
        ccConsole::Error("Select at least one mesh!");
        return;
    }
    else if (meshNum+cloudNum<2)
    {
        ccConsole::Error("Select one mesh and one cloud or two meshes!");
        return;
    }

    ccHObject* compEnt=0;
    ccGenericMesh* refMesh = 0;

    if (meshNum==1)
    {
        compEnt = m_selectedEntities[isMesh[0] ? 1 : 0];
        refMesh = ccHObjectCaster::ToGenericMesh(m_selectedEntities[isMesh[0] ? 0 : 1]);
    }
    else
    {
        ccOrderChoiceDlg dlg(m_selectedEntities[0], "Compared",
                             m_selectedEntities[1], "Reference",
                             this);
        if (!dlg.exec())
            return;

        compEnt = dlg.getFirstEntity();
        refMesh = ccHObjectCaster::ToGenericMesh(dlg.getSecondEntity());
    }

    //assert(!m_compDlg);
	if (m_compDlg)
		delete m_compDlg;
    m_compDlg = new ccComparisonDlg(compEnt, refMesh, ccComparisonDlg::CLOUDMESH_DIST, this);
    connect(m_compDlg, SIGNAL(finished(int)), this, SLOT(deactivateComparisonMode(int)));
    m_compDlg->show();

    freezeUI(true);
}

void MainWindow::deactivateComparisonMode(int)
{
	//DGM: a bug apperead with recent changes (from CC or QT?)
	//which prevent us from deleting the dialog right away...
	//(it seems that QT has not yet finished the dialog closing
	//when the 'finished' signal is sent).
    //if(m_compDlg)
    //    delete m_compDlg;
    //m_compDlg = 0;

    freezeUI(false);

    updateUI();
}

void MainWindow::toggleActiveWindowSunLight()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
    {
        win->toggleSunLight();
        win->redraw();
    }
}

void MainWindow::toggleActiveWindowCustomLight()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
    {
        win->toggleCustomLight();
        win->redraw();
    }
}

void MainWindow::toggleActiveWindowCenteredPerspective()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
    {
        win->togglePerspective(true);
        win->redraw();
		updateViewModePopUpMenu(win);
    }
}

void MainWindow::toggleActiveWindowViewerBasedPerspective()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
    {
        win->togglePerspective(false);
        win->redraw();
		updateViewModePopUpMenu(win);
    }
}

void MainWindow::doActionDeleteShader()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
        win->setShader(0);
}

void MainWindow::doActionDeactivateGlFilter()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
    {
        win->setGlFilter(0);
        win->redraw();
    }
}

void MainWindow::removeFromDB(ccHObject* obj, bool autoDelete/*=true*/)
{
	if (!obj)
		return;

	if (!autoDelete && obj->getParent())
		obj->setFlagState(CC_FATHER_DEPENDANT,false);

	if (m_ccRoot)
		m_ccRoot->removeElement(obj);
}

void MainWindow::setSelectedInDB(ccHObject* obj, bool selected)
{
	if (obj && m_ccRoot)
	{
		if (selected)
			m_ccRoot->selectEntity(obj);
		else
			m_ccRoot->unselectEntity(obj);
	}
}

void MainWindow::addToDB(ccHObject* obj,
						 bool autoExpandDBTree/*=true*/,
						 const char* statusMessage/*=0*/,
						 bool addToDisplay/*=true*/,
						 bool updateZoom/*=true*/,
						 ccGLWindow* winDest/*=0*/,
						 bool* coordinatesTransEnabled/*=0*/,
						 double* coordinatesShift/*=0*/,
						 double* coordinatesScale/*=0*/)
{
    if (statusMessage)
        QMainWindow::statusBar()->showMessage(QString(statusMessage), 2000);

	//we handle display issues before inserting objects in the DB tree!
    if (addToDisplay)
    {
        //let's check that the new entity is not too big nor too far from scene center!
		//if (!coordinatesTransEnabled || !(*coordinatesTransEnabled)	)
		{
			//get entity bounding box
			ccBBox bBox = obj->getBB();

			CCVector3 center = bBox.getCenter();
			PointCoordinateType diag = bBox.getDiagNorm();

			bool shiftAlreadyEnabled = (coordinatesTransEnabled && *coordinatesTransEnabled && coordinatesShift);
			double P[3]={center[0],center[1],center[2]};
			double Pshift[3]={0};
			if (shiftAlreadyEnabled)
				memcpy(Pshift,coordinatesShift,sizeof(double)*3);
			double scale = (coordinatesScale ? *coordinatesScale : 1.0);
			bool applyAll=false;
			if (ccCoordinatesShiftManager::Handle(P,diag,true,shiftAlreadyEnabled,Pshift,&scale,applyAll))
			{
				ccGLMatrix mat;
				mat.toIdentity();
				mat.data()[12] = (float)Pshift[0];
				mat.data()[13] = (float)Pshift[1];
				mat.data()[14] = (float)Pshift[2];
				mat.data()[0] = mat.data()[5] = mat.data()[10] = scale;
				obj->applyGLTransformation_recursive(&mat);
				ccConsole::Warning(QString("Entity '%1' will be translated: (%2,%3,%4)").arg(obj->getName()).arg(Pshift[0],0,'f',2).arg(Pshift[1],0,'f',2).arg(Pshift[2],0,'f',2));
				if (scale != 1.0)
					ccConsole::Warning(QString("Entity '%1' will be rescaled: X%2").arg(obj->getName().arg(scale)));

				//update 'original shift' for ALL clouds
				ccHObject::Container children;
				children.push_back(obj);
				while (!children.empty())
				{
					ccHObject* child = children.back();
					children.pop_back();

					if (child->isKindOf(CC_POINT_CLOUD))
					{
						ccGenericPointCloud* pc = ccHObjectCaster::ToGenericPointCloud(child);
						const double* oShift = pc->getOriginalShift();
						assert(oShift);
						pc->setOriginalShift(Pshift[0]+oShift[0],Pshift[1]+oShift[1],Pshift[2]+oShift[2]);
					}

					for (unsigned i=0;i<child->getChildrenNumber();++i)
						children.push_back(child->getChild(i));
				}

				//we save coordinates shift information
				if (applyAll && coordinatesTransEnabled && coordinatesShift)
				{
					*coordinatesTransEnabled = true;
					coordinatesShift[0] = Pshift[0];
					coordinatesShift[1] = Pshift[1];
					coordinatesShift[2] = Pshift[2];
					if (coordinatesScale)
						*coordinatesScale = scale;
				}
			}
		}
	}

	//add object to DB root
	if (m_ccRoot)
		m_ccRoot->addElement(obj,autoExpandDBTree);

	//we can now update the current display, as GL windows use DB root as scene graph!
	if (addToDisplay)
    {
		ccGLWindow* activeWin = (winDest ? winDest : getActiveGLWindow());
        if (activeWin)
        {
            obj->setDisplay_recursive(activeWin);
            activeWin->invalidateViewport();
            if (updateZoom)
                activeWin->zoomGlobal();
            else
                activeWin->redraw();

        }
    }
}

void MainWindow::addToDBAuto(const QStringList& filenames)
{
	ccGLWindow* win = qobject_cast<ccGLWindow*>(QObject::sender());

	addToDB(filenames, UNKNOWN_FILE, win);
}

void MainWindow::addToDB(const QStringList& filenames, CC_FILE_TYPES fType, ccGLWindow* destWin/*=0*/)
{
	//to handle same 'shift on load' for multiple files
	double loadCoordinatesShift[3]={0,0,0};
	bool loadCoordinatesTransEnabled=false;

	//the same for 'addToDB' (if the first one is not supported, or if the scale remains too big)
	double addCoordinatesShift[3]={0,0,0};
	bool addCoordinatesTransEnabled=false;
	double addCoordinatesScale = 1.0;

	for (int i=0; i<filenames.size(); ++i)
	{
		ccHObject* newGroup = FileIOFilter::LoadFromFile(filenames[i],fType,true,&loadCoordinatesTransEnabled,loadCoordinatesShift);

		if (newGroup)
			addToDB(newGroup,true,"File loaded",true,true,destWin,&addCoordinatesTransEnabled,addCoordinatesShift,&addCoordinatesScale);
	}
}

void MainWindow::handleNewEntity(ccHObject* entity)
{
	assert(entity);
	if (entity)
		addToDB(entity,true,0,false,false,0);
}

void MainWindow::AddToDB(const QString& filename, CC_FILE_TYPES fType)
{
    TheInstance()->addToDB(QStringList(filename), fType);
}

void MainWindow::forceConsoleDisplay()
{
	//if the console is hidden, we autoamtically display it!
	if (DockableConsole && DockableConsole->isHidden())
	{
		DockableConsole->show();
		QApplication::processEvents();
	}
}

void MainWindow::loadFile()
{
    QSettings settings;
    settings.beginGroup("LoadFile");
    QString currentPath = settings.value("currentPath",QApplication::applicationDirPath()).toString();
    int currentOpenDlgFilter = settings.value("selectedFilter",BIN).toInt();

    //available filters
    QString filters;
    filters.append(CC_FILE_TYPE_FILTERS[UNKNOWN_FILE]);
    filters.append("\n");
    filters.append(CC_FILE_TYPE_FILTERS[BIN]);
    filters.append("\n");
    filters.append(CC_FILE_TYPE_FILTERS[ASCII]);
    filters.append("\n");
    filters.append(CC_FILE_TYPE_FILTERS[PLY]);
    filters.append("\n");
    filters.append(CC_FILE_TYPE_FILTERS[OBJ]);
    filters.append("\n");
    filters.append(CC_FILE_TYPE_FILTERS[VTK]);
    filters.append("\n");
    filters.append(CC_FILE_TYPE_FILTERS[STL]);
    filters.append("\n");
    filters.append(CC_FILE_TYPE_FILTERS[PCD]);
    filters.append("\n");
#ifdef CC_X3D_SUPPORT
    filters.append(CC_FILE_TYPE_FILTERS[X3D]);
    filters.append("\n");
#endif
#ifdef CC_LAS_SUPPORT
    filters.append(CC_FILE_TYPE_FILTERS[LAS]);
    filters.append("\n");
#endif
#ifdef CC_E57_SUPPORT
    filters.append(CC_FILE_TYPE_FILTERS[E57]);
    filters.append("\n");
#endif
#ifdef CC_PDMS_SUPPORT
    filters.append(CC_FILE_TYPE_FILTERS[PDMS]);
    filters.append("\n");
#endif
    filters.append(CC_FILE_TYPE_FILTERS[SOI]);
    filters.append("\n");
    filters.append(CC_FILE_TYPE_FILTERS[PN]);
    filters.append("\n");
    filters.append(CC_FILE_TYPE_FILTERS[PV]);
    filters.append("\n");
    filters.append(CC_FILE_TYPE_FILTERS[POV]);
    filters.append("\n");
    filters.append(CC_FILE_TYPE_FILTERS[ICM]);
    filters.append("\n");
    filters.append(CC_FILE_TYPE_FILTERS[BUNDLER]);
    filters.append("\n");

    //currently selected filter
    QString selectedFilter = CC_FILE_TYPE_FILTERS[currentOpenDlgFilter];

    //file choosing dialog
    QStringList selectedFiles = QFileDialog::getOpenFileNames(this,
                                tr("Open file(s)"),
                                currentPath,
                                filters,
                                &selectedFilter
                                /*QFileDialog::DontUseNativeDialog*/); //Windows has a limitation on the returned string size
    if (selectedFiles.isEmpty())
        return;

    CC_FILE_TYPES fType = UNKNOWN_FILE;
    for (unsigned i=0;i<(unsigned)FILE_TYPES_COUNT;++i)
    {
        if (selectedFilter == QString(CC_FILE_TYPE_FILTERS[i]))
        {
            fType = CC_FILE_TYPES_ENUMS[i];
            break;
        }
    }

	//load files
	addToDB(selectedFiles,fType);

    //we update current file path
    currentPath = QFileInfo(selectedFiles[0]).absolutePath();
    currentOpenDlgFilter = fType;

	//save last loading location
    settings.setValue("currentPath",currentPath);
    settings.setValue("selectedFilter",currentOpenDlgFilter);
    settings.endGroup();
}

void MainWindow::saveFile()
{
    size_t selNum = m_selectedEntities.size();
    if (selNum==0)
        return;

    //persistent settings
    QSettings settings;
    settings.beginGroup("SaveFile");
    QString currentPath = settings.value("currentPath",QApplication::applicationDirPath()).toString();
    int currentCloudSaveDlgFilter = settings.value("selectedFilterCloud",BIN).toInt();
    int currentMeshSaveDlgFilter = settings.value("selectedFilterMesh",PLY).toInt();

	ccHObject clouds("clouds");
	ccHObject meshes("meshes");
	ccHObject images("images");
	ccHObject other("other");
	ccHObject otherSerializable("serializable");
	ccHObject::Container entitiesToSave;
	entitiesToSave.insert(entitiesToSave.begin(),m_selectedEntities.begin(),m_selectedEntities.end());
	while (!entitiesToSave.empty())
    {
		ccHObject* child = entitiesToSave.back();
		entitiesToSave.pop_back();
        
        if (child->isA(CC_HIERARCHY_OBJECT))
        {
            for (unsigned j=0;j<child->getChildrenNumber();++j)
				entitiesToSave.push_back(child->getChild(j));
		}
		else
		{
			//we put entity in the container corresponding to its type
			ccHObject* dest = 0;
			if (child->isA(CC_POINT_CLOUD))
				dest = &clouds;
			else if (child->isKindOf(CC_MESH))
				dest = &meshes;
			else if (child->isKindOf(CC_IMAGE))
				dest = &images;
			else if (child->isSerializable())
				dest = &otherSerializable;
			else
				dest = &other;

			assert(dest);

			//we don't want double insertions if the user has clicked both the father and child
			if (!dest->find(child->getUniqueID()))
				dest->addChild(child,false);
        }
	}

	bool hasCloud = (clouds.getChildrenNumber()!=0);
	bool hasMesh = (meshes.getChildrenNumber()!=0);
	bool hasImage = (images.getChildrenNumber()!=0);
	bool hasSerializable = (otherSerializable.getChildrenNumber()!=0);
	bool hasOther = (other.getChildrenNumber()!=0);
	
	int stdSaveTypes = (int)hasCloud + (int)hasMesh + (int)hasImage + (int)hasSerializable;
	if (stdSaveTypes == 0)
	{
		ccConsole::Error("Can't save this entity(ies) this way!");
		return;
	}
	
    //we set up the right file filters, depending on the selected
    //entities shared type (cloud, mesh, etc.).
    QString filters;

	//From now on, BIN format handles about anyhting!
    filters.append(CC_FILE_TYPE_FILTERS[BIN]);
	filters.append("\n");

	ccHObject* toSave = 0;
    QString selectedFilter = CC_FILE_TYPE_FILTERS[BIN];

	//if we only have one type of entity selected, then we can let the user choose specific formats
	if (stdSaveTypes == 1)
	{
		if (hasCloud)
		{
			toSave = &clouds;
			selectedFilter = CC_FILE_TYPE_FILTERS[currentCloudSaveDlgFilter];

			//add cloud output file filters
			filters.append(CC_FILE_TYPE_FILTERS[ASCII]);
			filters.append("\n");
	#ifdef CC_E57_SUPPORT
			filters.append(CC_FILE_TYPE_FILTERS[E57]);
			filters.append("\n");
	#endif
			if (clouds.getChildrenNumber()==1)
			{
				filters.append(CC_FILE_TYPE_FILTERS[PLY]);
				filters.append("\n");
				filters.append(CC_FILE_TYPE_FILTERS[VTK]);
				filters.append("\n");
	#ifdef CC_LAS_SUPPORT
				filters.append(CC_FILE_TYPE_FILTERS[LAS]);
				filters.append("\n");
	#endif
				filters.append(CC_FILE_TYPE_FILTERS[PN]);
				filters.append("\n");
				filters.append(CC_FILE_TYPE_FILTERS[PV]);
				filters.append("\n");
			}
			//TODO: POV files handling!
			//filters.append(CC_FILE_TYPE_FILTERS[POV]);
			//filters.append("\n");
		}
		else if (hasMesh)
		{
			if (meshes.getChildrenNumber()==1)
			{
				toSave = &meshes;
				selectedFilter = CC_FILE_TYPE_FILTERS[currentMeshSaveDlgFilter];

				//add meshes output file filters
				filters.append(CC_FILE_TYPE_FILTERS[OBJ]);
				filters.append("\n");
				filters.append(CC_FILE_TYPE_FILTERS[PLY]);
				filters.append("\n");
				filters.append(CC_FILE_TYPE_FILTERS[VTK]);
				filters.append("\n");
				filters.append(CC_FILE_TYPE_FILTERS[STL]);
				filters.append("\n");
		#ifdef CC_X3D_SUPPORT
				filters.append(CC_FILE_TYPE_FILTERS[X3D]);
				filters.append("\n");
		#endif
				filters.append(CC_FILE_TYPE_FILTERS[MA]);
				filters.append("\n");
			}
		}
		else if (hasImage)
		{
			if (images.getChildrenNumber()>1)
			{
				ccConsole::Warning("[MainWindow::saveFile] Only BIN format is able to store multiple images at once");
			}
			else
			{
				toSave = &images;

				//add images output file filters
				//we grab the list of supported image file formats (writing)
				QList<QByteArray> formats = QImageWriter::supportedImageFormats();
				//we convert this list into a proper "filters" string
				for (int i=0;i<formats.size();++i)
					filters.append(QString("%1 image (*.%2)\n").arg(QString(formats[i].data()).toUpper()).arg(formats[i].data()));
			}
		}
	}


    QString dir = currentPath+QString("/");
    if (selNum==1)
	{
		//hierarchy objects have generally as name: 'filename.ext (fullpath)'
		//so we must only take the first part! (otherwise this type of name
		//with a path inside perturbs the QFileDialog a lot ;))
		QString defaultFileName(m_selectedEntities[0]->getName());
		if (m_selectedEntities[0]->isA(CC_HIERARCHY_OBJECT))
		{
			QStringList parts = defaultFileName.split(' ',QString::SkipEmptyParts);
			if (parts.size()>0)
				defaultFileName = parts[0];
		}

		if (!QFileInfo(defaultFileName).suffix().isEmpty()) //we remove extension
			defaultFileName = QFileInfo(defaultFileName).baseName();

		dir += defaultFileName;
	}

    QString selectedFilename = QFileDialog::getSaveFileName(this,
                               tr("Save file"),
                               dir,
                               filters,
                               &selectedFilter);

    if (selectedFilename.isEmpty())
        return;

	//ignored items
    if (hasOther)
	{
		ccConsole::Warning("[MainWindow::saveFile] The following selected entites won't be saved:");
		for (unsigned i=0;i<other.getChildrenNumber();++i)
			ccConsole::Warning(QString("\t- %1s").arg(other.getChild(i)->getName()));
	}

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	//bin format
	if (selectedFilter == QString(CC_FILE_TYPE_FILTERS[BIN]))
	{
		if (selNum==1)
			result = FileIOFilter::SaveToFile(m_selectedEntities[0],qPrintable(selectedFilename),BIN);
		else
		{
			ccHObject::Container tempContainer;
			RemoveSiblings(m_selectedEntities,tempContainer);

			if (tempContainer.size())
			{
				ccHObject root;
				for (unsigned i=0;i<tempContainer.size();++i)
					root.addChild(tempContainer[i],false);
				result = FileIOFilter::SaveToFile(&root,qPrintable(selectedFilename),BIN);
			}
			else
			{
				ccLog::Warning("[MainWindow::saveFile] No selected entity could be saved!");
				result = CC_FERR_NO_SAVE;
			}
		}

		currentCloudSaveDlgFilter = BIN;
	}
	else if (toSave)
	{
		//ignored items
		if (hasSerializable)
		{
			if (!hasOther)
				ccConsole::Warning("[MainWindow::saveFile] The following selected entites won't be saved:"); //display this warning only if not already done
			for (unsigned i=0;i<otherSerializable.getChildrenNumber();++i)
				ccConsole::Warning(QString("\t- %1").arg(otherSerializable.getChild(i)->getName()));
		}

		if (hasCloud || hasMesh)
		{
			CC_FILE_TYPES fType = UNKNOWN_FILE;
			for (unsigned i=0;i<(unsigned)FILE_TYPES_COUNT;++i)
			{
				if (selectedFilter == QString(CC_FILE_TYPE_FILTERS[i]))
				{
					fType = CC_FILE_TYPES_ENUMS[i];
					break;
				}
			}

			if (hasCloud)
				currentCloudSaveDlgFilter = fType;
			else if (hasMesh)
				currentMeshSaveDlgFilter = fType;

			result = FileIOFilter::SaveToFile(toSave->getChildrenNumber()>1 ? toSave : toSave->getChild(0),
											   qPrintable(selectedFilename),
											   fType);
		}
		else if (hasImage)
		{
			assert(images.getChildrenNumber()==1);
			ccImage* image = static_cast<ccImage*>(images.getChild(0));
			if (!image->data().save(selectedFilename))
				result = CC_FERR_WRITING;
		}
	}

	if (result!=CC_FERR_NO_ERROR)
		FileIOFilter::DisplayErrorMessage(result,"saving",selectedFilename);

    //we update current file path
    currentPath = QFileInfo(selectedFilename).absolutePath();

    settings.setValue("currentPath",currentPath);
    settings.setValue("selectedFilterCloud",(int)currentCloudSaveDlgFilter);
    settings.setValue("selectedFilterMesh",(int)currentMeshSaveDlgFilter);
    settings.endGroup();
}

void MainWindow::on3DViewActivated(QMdiSubWindow* mdiWin)
{
	ccGLWindow* win = mdiWin ? static_cast<ccGLWindow*>(mdiWin->widget()) : 0;
	if (win)
	{
		updateViewModePopUpMenu(win);
		updatePivotVisibilityPopUpMenu(win);


	}
	else
	{
		if (m_viewModePopupButton)
		{
		}
	}
}

void MainWindow::updateViewModePopUpMenu(ccGLWindow* win)
{
	if (!m_viewModePopupButton)
		return;

	//update the view mode pop-up 'top' icon
	if (win)
	{
		bool objectCentered = true;
		bool perspectiveEnabled = win->getPerspectiveState(objectCentered);

		QAction* currentModeAction = 0;
		if (!perspectiveEnabled)
			currentModeAction = actionSetOrthoView;
		else if (objectCentered)
			currentModeAction = actionSetCenteredPerspectiveView;
		else
			currentModeAction = actionSetViewerPerspectiveView;

		assert(currentModeAction);
		m_viewModePopupButton->setIcon(currentModeAction->icon());
		m_viewModePopupButton->setEnabled(true);
	}
	else
	{
		m_viewModePopupButton->setIcon(QIcon());
		m_viewModePopupButton->setEnabled(false);
	}
}

void MainWindow::updatePivotVisibilityPopUpMenu(ccGLWindow* win)
{
	if (!m_pivotVisibilityPopupButton)
		return;

	//update the pivot visibility pop-up 'top' icon
	if (win)
	{
		QAction* visibilityAction = 0;
		switch(win->getPivotVisibility())
		{
		case ccGLWindow::PIVOT_HIDE:
			visibilityAction = actionSetPivotOff;
			break;
		case ccGLWindow::PIVOT_SHOW_ON_MOVE:
			visibilityAction = actionSetPivotRotationOnly;
			break;
		case ccGLWindow::PIVOT_ALWAYS_SHOW:
			visibilityAction = actionSetPivotAlwaysOn;
			break;
		default:
			assert(false);
		}

		if (visibilityAction)
			m_pivotVisibilityPopupButton->setIcon(visibilityAction->icon());

		//pivot is not available in viewer-based perspective!
		bool objectCentered = true;
		win->getPerspectiveState(objectCentered);
		m_pivotVisibilityPopupButton->setEnabled(objectCentered);
	}
	else
	{
		m_pivotVisibilityPopupButton->setIcon(QIcon());
		m_pivotVisibilityPopupButton->setEnabled(false);
	}
}


void MainWindow::updateMenus()
{
    ccGLWindow* win = getActiveGLWindow();
    bool hasMdiChild = (win != 0);
    bool hasSelectedEntities = (m_ccRoot && m_ccRoot->countSelectedEntities()>0);

    //General Menu
    menuEdit->setEnabled(true/*hasSelectedEntities*/);
    menuTools->setEnabled(true/*hasSelectedEntities*/);

    //3D Views Menu
    actionClose3DView->setEnabled(hasMdiChild);
    actionCloseAll3DViews->setEnabled(hasMdiChild);
    actionTile3DViews->setEnabled(hasMdiChild);
    actionCascade3DViews->setEnabled(hasMdiChild);
    actionNext3DView->setEnabled(hasMdiChild);
    actionPrevious3DView->setEnabled(hasMdiChild);

    //Shaders & Filters display Menu
    bool shadersEnabled = (win ? win->areShadersEnabled() : false);
    actionLoadShader->setEnabled(shadersEnabled);
    actionDeleteShader->setEnabled(shadersEnabled);

    bool filtersEnabled = (win ? win->areGLFiltersEnabled() : false);
    actionNoFilter->setEnabled(filtersEnabled);

    //View Menu
    toolBarView->setEnabled(hasMdiChild);

    //oher actions
    actionSegment->setEnabled(hasMdiChild && hasSelectedEntities);
    actionTranslateRotate->setEnabled(hasMdiChild && hasSelectedEntities);
	actionPointPicking->setEnabled(hasMdiChild);
    //actionPointListPicking->setEnabled(hasMdiChild);
    actionTestFrameRate->setEnabled(hasMdiChild);
    actionRenderToFile->setEnabled(hasMdiChild);
    actionToggleSunLight->setEnabled(hasMdiChild);
    actionToggleCustomLight->setEnabled(hasMdiChild);
    actionToggleCenteredPerspective->setEnabled(hasMdiChild);
    actionToggleViewerBasedPerspective->setEnabled(hasMdiChild);

    //plugins
	foreach (QAction* act, m_glFilterActions.actions())
		act->setEnabled(hasMdiChild);
}

void MainWindow::update3DViewsMenu()
{
    menu3DViews->clear();
    menu3DViews->addAction(actionNew3DView);
    menu3DViews->addSeparator();
    menu3DViews->addAction(actionClose3DView);
    menu3DViews->addAction(actionCloseAll3DViews);
    menu3DViews->addSeparator();
    menu3DViews->addAction(actionTile3DViews);
    menu3DViews->addAction(actionCascade3DViews);
    menu3DViews->addSeparator();
    menu3DViews->addAction(actionNext3DView);
    menu3DViews->addAction(actionPrevious3DView);

    QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
    if (!windows.isEmpty())
    {
        //Dynamic Separator
        QAction* separator = new QAction(this);
        separator->setSeparator(true);
        menu3DViews->addAction(separator);

        for (int i = 0; i < windows.size(); ++i)
        {
            QWidget *child = windows.at(i)->widget();

            QString text = QString("&%1 %2").arg(i + 1).arg(child->windowTitle());
            QAction *action  = menu3DViews->addAction(text);
            action->setCheckable(true);
            action ->setChecked(child == getActiveGLWindow());
            connect(action, SIGNAL(triggered()), m_windowMapper, SLOT(map()));
            m_windowMapper->setMapping(action, windows.at(i));
        }
    }
}

void MainWindow::setActiveSubWindow(QWidget *window)
{
    if (!window || !m_mdiArea)
        return;
    m_mdiArea->setActiveSubWindow(qobject_cast<QMdiSubWindow *>(window));
}

void MainWindow::redrawAll()
{
    QList<QMdiSubWindow*> windows = m_mdiArea->subWindowList();
    for (int i = 0; i < windows.size(); ++i)
        static_cast<ccGLWindow*>(windows.at(i)->widget())->redraw();
}

void MainWindow::refreshAll()
{
    QList<QMdiSubWindow*> windows = m_mdiArea->subWindowList();
    for (int i = 0; i < windows.size(); ++i)
        static_cast<ccGLWindow*>(windows.at(i)->widget())->refresh();
}

void MainWindow::updateUI()
{
    updateUIWithSelection();
	updateMenus();
    if (m_ccRoot)
        m_ccRoot->updatePropertiesView();
}

void MainWindow::updateUIWithSelection()
{
    dbTreeSelectionInfo selInfo;

    m_selectedEntities.clear();

	if (m_ccRoot)
		m_ccRoot->getSelectedEntities(m_selectedEntities,CC_OBJECT,&selInfo);
    expandDBTreeWithSelection(m_selectedEntities);

    enableUIItems(selInfo);
}

void MainWindow::expandDBTreeWithSelection(ccHObject::Container& selection)
{
	if (!m_ccRoot)
		return;
/*
    size_t i,selNum = selection.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = selection[i];
		//if (ent->isA(CC_MESH_GROUP)) //DGM: we don't expand mesh groups anymore!
		//	m_ccRoot->expandElement(ent,true);
	}
*/
}

void MainWindow::enableAll()
{
    QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
    for (int i = 0; i < windows.size(); ++i)
        windows.at(i)->setEnabled(true);
}

void MainWindow::disableAll()
{
    QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
    for (int i = 0; i < windows.size(); ++i)
        windows.at(i)->setEnabled(false);
}

void MainWindow::disableAllBut(ccGLWindow* win)
{
    //we disable all other windows
    QList<QMdiSubWindow*> windows = m_mdiArea->subWindowList();
    for (int i = 0; i < windows.size(); ++i)
        if (static_cast<ccGLWindow*>(windows.at(i)->widget()) != win)
            windows.at(i)->setEnabled(false);
}

void MainWindow::enableUIItems(dbTreeSelectionInfo& selInfo)
{
    //>0
    bool atLeastOneEntity = (selInfo.selCount>0);
    bool atLeastOneCloud = (selInfo.cloudCount>0);
    bool atLeastOneMesh = (selInfo.meshCount>0);
    //bool atLeastOneOctree = (selInfo.octreeCount>0);
    bool atLeastOneNormal = (selInfo.normalsCount>0);
    bool atLeastOneColor = (selInfo.colorCount>0);
    bool atLeastOneSF = (selInfo.sfCount>0);
    //bool atLeastOneSensor = (selInfo.sensorCount>0);
    bool atLeastOneGDBSensor = (selInfo.gblSensorCount>0);
    bool activeWindow = (getActiveGLWindow() != 0);

    //menuEdit->setEnabled(atLeastOneEntity);
    //menuTools->setEnabled(atLeastOneEntity);
    menuGroundBasedLidar->setEnabled(atLeastOneGDBSensor);

    actionZoomAndCenter->setEnabled(atLeastOneEntity && activeWindow);
    actionSave->setEnabled(atLeastOneEntity);
    actionClone->setEnabled(atLeastOneCloud || atLeastOneMesh);
    actionDelete->setEnabled(atLeastOneEntity);
	actionExportCoordToSF->setEnabled(atLeastOneEntity);
    actionSegment->setEnabled(atLeastOneEntity && activeWindow);
    actionTranslateRotate->setEnabled(atLeastOneEntity && activeWindow);
    actionShowDepthBuffer->setEnabled(atLeastOneGDBSensor);
    actionExportDepthBuffer->setEnabled(atLeastOneGDBSensor);
    actionResampleWithOctree->setEnabled(atLeastOneCloud);
    actionMultiply->setEnabled(atLeastOneEntity);
    actionApplyTransformation->setEnabled(atLeastOneEntity);
    actionComputeOctree->setEnabled(atLeastOneCloud || atLeastOneMesh);
    actionComputeNormals->setEnabled(atLeastOneCloud || atLeastOneMesh);
    actionSetColorGradient->setEnabled(atLeastOneCloud || atLeastOneMesh);
    actionSetUniqueColor->setEnabled(atLeastOneEntity/*atLeastOneCloud || atLeastOneMesh*/); //DGM: we can set color to a group now!
    actionColorize->setEnabled(atLeastOneEntity/*atLeastOneCloud || atLeastOneMesh*/); //DGM: we can set color to a group now!
    actionScalarFieldFromColor->setEnabled(atLeastOneEntity && atLeastOneColor);
    actionComputeMeshAA->setEnabled(atLeastOneCloud);
    actionComputeMeshLS->setEnabled(atLeastOneCloud);
    //actionComputeQuadric3D->setEnabled(atLeastOneCloud);
    actionComputeBestFitBB->setEnabled(atLeastOneEntity);
    actionDensity->setEnabled(atLeastOneCloud);
    actionCurvature->setEnabled(atLeastOneCloud);
    actionRoughness->setEnabled(atLeastOneCloud);
	actionPlaneOrientation->setEnabled(atLeastOneCloud);
	actionSNETest->setEnabled(atLeastOneCloud);

    actionFilterByValue->setEnabled(atLeastOneSF);
    actionConvertToRGB->setEnabled(atLeastOneSF);
	actionRenameSF->setEnabled(atLeastOneSF);
	actionAddIdField->setEnabled(atLeastOneCloud);
    actionComputeStatParams->setEnabled(atLeastOneSF);
    actionShowHistogram->setEnabled(atLeastOneSF);
    actionGaussianFilter->setEnabled(atLeastOneSF);
    actionBilateralFilter->setEnabled(atLeastOneSF);
    actionDeleteScalarField->setEnabled(atLeastOneSF);
    actionDeleteAllSF->setEnabled(atLeastOneSF);
    actionMultiplySF->setEnabled(/*TODO: atLeastOneSF*/false);
    actionSFGradient->setEnabled(atLeastOneSF);

    actionSamplePoints->setEnabled(atLeastOneMesh);
    actionMeasureMeshSurface->setEnabled(atLeastOneMesh);
	actionSmoothMeshLaplacian->setEnabled(atLeastOneMesh);
	actionConvertTextureToColor->setEnabled(atLeastOneMesh);
	actionSubdivideMesh->setEnabled(atLeastOneMesh);
	actionMeshBestFittingQuadric->setEnabled(atLeastOneMesh);

    menuMeshScalarField->setEnabled(atLeastOneSF && atLeastOneMesh);
    //actionSmoothMeshSF->setEnabled(atLeastOneSF && atLeastOneMesh);
    //actionEnhanceMeshSF->setEnabled(atLeastOneSF && atLeastOneMesh);

    actionResolveNormalsDirection->setEnabled(atLeastOneCloud && atLeastOneNormal);
    actionClearNormals->setEnabled(atLeastOneNormal);
    actionInvertNormals->setEnabled(atLeastOneNormal);
    actionConvertNormalToHSV->setEnabled(atLeastOneNormal);
    actionClearColor->setEnabled(atLeastOneColor);

    //==1
    bool exactlyOneEntity = (selInfo.selCount==1);
    bool exactlyOneCloud = (selInfo.cloudCount==1);
    bool exactlyOneMesh = (selInfo.meshCount==1);
    bool exactlyOneSF = (selInfo.sfCount==1);
    bool exactlyOneSensor = (selInfo.sensorCount==1);

    actionModifySensor->setEnabled(exactlyOneSensor);
    actionComputeDistancesFromSensor->setEnabled(exactlyOneSensor);
    actionComputeScatteringAngles->setEnabled(exactlyOneSensor);
    actionProjectSensor->setEnabled(atLeastOneCloud);
    actionLabelConnectedComponents->setEnabled(atLeastOneCloud);
    actionUnroll->setEnabled(exactlyOneEntity);
    actionStatisticalTest->setEnabled(exactlyOneEntity && exactlyOneSF);
	actionAddConstantSF->setEnabled(exactlyOneCloud || exactlyOneMesh);
	actionEditGlobalShift->setEnabled(exactlyOneCloud || exactlyOneMesh);
	actionComputeKdTree->setEnabled(exactlyOneCloud || exactlyOneMesh);
	
	actionKMeans->setEnabled(/*TODO: exactlyOneEntity && exactlyOneSF*/false);
    actionFrontPropagation->setEnabled(/*TODO: exactlyOneEntity && exactlyOneSF*/false);
	
	menuActiveScalarField->setEnabled((exactlyOneCloud || exactlyOneMesh) && selInfo.sfCount>0);
	actionCrossSection->setEnabled(exactlyOneCloud);
	actionHeightGridGeneration->setEnabled(exactlyOneCloud);

	actionPointListPicking->setEnabled(exactlyOneEntity);

	//==2
    bool exactlyTwoEntities = (selInfo.selCount==2);
    bool exactlyTwoClouds = (selInfo.cloudCount==2);
    //bool exactlyTwoSF = (selInfo.sfCount==2);

    actionRegister->setEnabled(exactlyTwoEntities);
	actionPointPairsAlign->setEnabled(exactlyOneEntity || exactlyTwoEntities);
    actionAlign->setEnabled(exactlyTwoEntities); //Aurelien BEY le 13/11/2008
    actionSubsample->setEnabled(exactlyOneCloud); //Aurelien BEY le 4/12/2008
    actionCloudCloudDist->setEnabled(exactlyTwoClouds);
    actionCloudMeshDist->setEnabled(exactlyTwoEntities && atLeastOneMesh);
    actionCPS->setEnabled(exactlyTwoClouds);
    actionScalarFieldArithmetic->setEnabled(exactlyOneEntity && atLeastOneSF);

    //>1
    bool atLeastTwoEntities = (selInfo.selCount>1);

    actionFuse->setEnabled(atLeastTwoEntities);
    actionSynchronize->setEnabled(atLeastTwoEntities);

    //standard plugins
	foreach (ccStdPluginInterface* plugin, m_stdPlugins)
		plugin->onNewSelection(m_selectedEntities);
}

void MainWindow::echoMouseWheelRotate(float wheelDelta_deg)
{
    if (checkBoxCameraLink->checkState() != Qt::Checked)
        return;

    ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

    QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
    for (int i = 0; i < windows.size(); ++i)
    {
        ccGLWindow *child = static_cast<ccGLWindow*>(windows.at(i)->widget());
		if (child != sendingWindow)
        {
			child->onWheelEvent(wheelDelta_deg);
            child->redraw();
        }
    }
}

void MainWindow::echoCameraDisplaced(float ddx, float ddy)
{
    if (checkBoxCameraLink->checkState() != Qt::Checked)
        return;

    ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

    QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
    for (int i = 0; i < windows.size(); ++i)
    {
        ccGLWindow *child = static_cast<ccGLWindow*>(windows.at(i)->widget());
        if (child != sendingWindow)
        {
			child->moveCamera(ddx,ddy,0.0f);
            child->redraw();
        }
    }
}

void MainWindow::echoBaseViewMatRotation(const ccGLMatrix& rotMat)
{
    if (checkBoxCameraLink->checkState() != Qt::Checked)
        return;

    ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

    QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
    for (int i = 0; i < windows.size(); ++i)
    {
        ccGLWindow *child = static_cast<ccGLWindow*>(windows.at(i)->widget());
        if (child != sendingWindow)
        {
            child->rotateBaseViewMat(rotMat);
            child->redraw();
        }
    }
}

void MainWindow::dispToConsole(QString message, ConsoleMessageLevel level/*=STD_CONSOLE_MESSAGE*/)
{
	switch(level)
	{
	case STD_CONSOLE_MESSAGE:
		ccConsole::Print(message);
		break;
	case WRN_CONSOLE_MESSAGE:
        ccConsole::Warning(message);
		break;
	case ERR_CONSOLE_MESSAGE:
		ccConsole::Error(message);
		break;
	}
}

void MainWindow::doActionLoadShader() //TODO
{
    ccConsole::Error("Not yet implemented! Sorry ...");
}

void MainWindow::doActionKMeans()  //TODO
{
    ccConsole::Error("Not yet implemented! Sorry ...");
}

void MainWindow::doActionFrontPropagation() //TODO
{
    ccConsole::Error("Not yet implemented! Sorry ...");
}

/************** STATIC METHODS ******************/

MainWindow* MainWindow::TheInstance()
{
    if (!s_instance)
        s_instance = new MainWindow();
    return s_instance;
}

void MainWindow::DestroyInstance()
{
    if (s_instance)
        delete s_instance;
    s_instance=0;
}

void MainWindow::GetGLWindows(std::vector<ccGLWindow*>& glWindows)
{
    QList<QMdiSubWindow *> windows = TheInstance()->m_mdiArea->subWindowList();
    unsigned winNum = windows.size();

    if (winNum==0)
        return;

    glWindows.clear();
    glWindows.reserve(winNum);

    for (unsigned i=0; i<winNum; ++i)
        glWindows.push_back(static_cast<ccGLWindow*>(windows.at(i)->widget()));
}

ccGLWindow* MainWindow::GetActiveGLWindow()
{
    return TheInstance()->getActiveGLWindow();
}

ccGLWindow* MainWindow::GetGLWindow(const QString& title)
{
    QList<QMdiSubWindow *> windows = TheInstance()->m_mdiArea->subWindowList();
    unsigned winNum = windows.size();

    if (winNum==0)
        return 0;

    for (unsigned i=0; i<winNum; ++i)
    {
        ccGLWindow* win = static_cast<ccGLWindow*>(windows.at(i)->widget());
        if (win->windowTitle() == title)
            return win;
    }

    return 0;
}

void MainWindow::RefreshAllGLWindow()
{
    TheInstance()->refreshAll();
}

void MainWindow::UpdateUI()
{
    TheInstance()->updateUI();
}

ccDBRoot* MainWindow::db()
{
	return m_ccRoot;
}

ccHObject* MainWindow::dbRootObject()
{
	return (m_ccRoot ? m_ccRoot->getRootEntity() : 0);
}

void MainWindow::removeObjectTemporarilyFromDBTree(ccHObject* obj, ccHObject* &parent)
{
	parent = 0;

	assert(obj);
	if (!m_ccRoot || !obj)
		return;

	//mandatory (to call putObjectBackIntoDBTree)
	parent = obj->getParent();

	//detach the object (in case its children undergo "severe" modifications)
	obj->setFlagState(CC_FATHER_DEPENDANT,false);
	m_ccRoot->removeElement(obj);
}

void MainWindow::putObjectBackIntoDBTree(ccHObject* obj, ccHObject* parent)
{
	assert(obj);
	if (!obj || !m_ccRoot)
		return;

	obj->setFlagState(CC_FATHER_DEPENDANT,true);
	if (parent)
		parent->addChild(obj);
	m_ccRoot->addElement(obj,false);
}


//For primitives test
/*#include <ccBox.h>
#include <ccCone.h>
#include <ccCylinder.h>
#include <ccTorus.h>
#include <ccSphere.h>
#include <ccDish.h>
#include <ccExtru.h>

void doTestPrimitives()
{
	//PRIMITIVES TEST
	addToDB(new ccBox(CCVector3(10,20,30)));
	addToDB(new ccCone(10,20,30));
	addToDB(new ccCylinder(20,30));
	addToDB(new ccCone(10,20,30,5,10));
	addToDB(new ccTorus(50,60,M_PI/3,false));
	addToDB(new ccTorus(50,60,M_PI/3,true,20));
	addToDB(new ccSphere(35));
	addToDB(new ccDish(35,15,0));
	addToDB(new ccDish(35,25,0));
	addToDB(new ccDish(35,35,0));
	addToDB(new ccDish(35,15,15));

	std::vector<CCVector2> contour;
	contour.push_back(CCVector2(10,00));
	contour.push_back(CCVector2(00,20));
	contour.push_back(CCVector2(15,25));
	contour.push_back(CCVector2(20,10));
	contour.push_back(CCVector2(25,27));
	contour.push_back(CCVector2(18,35));
	contour.push_back(CCVector2(22,40));
	contour.push_back(CCVector2(30,30));
	contour.push_back(CCVector2(27,05));
	addToDB(new ccExtru(contour,10));
}
//*/
