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
#include <RegistrationTools.h> //Aurelien BEY
#include <Delaunay2dMesh.h>

//for tests
#include <ChamferDistanceTransform.h>
#include <SaitoSquaredDistanceTransform.h>

//qCC_db
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccPolyline.h>
#include <ccSubMesh.h>
#include <ccOctree.h>
#include <ccKdTree.h>
#include <ccGBLSensor.h>
#include <ccCameraSensor.h>
#include <ccNormalVectors.h>
#include <ccProgressDialog.h>
#include <ccPlane.h>
#include <ccImage.h>
#include <cc2DLabel.h>
#include <cc2DViewportObject.h>
#include <ccColorScale.h>
#include <ccColorScalesManager.h>
#include <ccFacet.h>
#include <ccQuadric.h>
#include <ccExternalFactory.h>
#include <ccSphere.h>

//qCC_io
#include <ccGlobalShiftManager.h>
#include <ccShiftAndScaleCloudDlg.h>
#include <BinFilter.h>
#include <DepthMapFileFilter.h>

//QCC_glWindow
#include <ccRenderingTools.h>
#include <ccGLWindow.h>

//local includes
#include "ccCommon.h"
#include "ccConsole.h"
#include "ccInnerRect2DFinder.h"
#include "ccHistogramWindow.h"

//plugins handling
#include <ccPluginInterface.h>
#include <ccStdPluginInterface.h>
#include <ccGLFilterPluginInterface.h>
#include <ccIOFilterPluginInterface.h>
#include "ccPluginDlg.h"

//shaders & Filters
#include <ccShader.h>
#include <ccGlFilter.h>

//dialogs
#include "ccDisplayOptionsDlg.h"
#include "ccGraphicalSegmentationTool.h"
#include "ccGraphicalTransformationTool.h"
#include "ccSectionExtractionTool.h"
#include "ccClippingBoxTool.h"
#include "ccOrderChoiceDlg.h"
#include "ccComparisonDlg.h"
#include "ccColorGradientDlg.h"
#include "ccAskTwoDoubleValuesDlg.h"
#include "ccAskThreeDoubleValuesDlg.h"
#include "ccPtsSamplingDlg.h"
#include "ccPickOneElementDlg.h"
#include "ccStatisticalTestDlg.h"
#include "ccLabelingDlg.h"
#include "ccGBLSensorProjectionDlg.h"
#include "ccCamSensorProjectionDlg.h"
#include "ccUnrollDlg.h"
#include "ccAlignDlg.h" //Aurelien BEY
#include "ccRegistrationDlg.h" //Aurelien BEY
#include "ccSubsamplingDlg.h" //Aurelien BEY
#include "ccRenderToFileDlg.h"
#include "ccPointPropertiesDlg.h" //Aurelien BEY
#include "ccPointListPickingDlg.h"
#include "ccNormalComputationDlg.h"
#include "ccCameraParamEditDlg.h"
#include "ccScalarFieldArithmeticsDlg.h"
#include "ccScalarFieldFromColorDlg.h"
#include "ccSensorComputeDistancesDlg.h"
#include "ccSensorComputeScatteringAnglesDlg.h"
#include "ccCurvatureDlg.h"
#include "ccApplyTransformationDlg.h"
#include "ccPointPairRegistrationDlg.h"
#include "ccExportCoordToSFDlg.h"
#include "ccPrimitiveFactoryDlg.h"
#include "ccColorScaleEditorDlg.h"
#include "ccComputeOctreeDlg.h"
#include "ccAdjustZoomDlg.h"
#include "ccBoundingBoxEditorDlg.h"
#include "ccColorLevelsDlg.h"
#include "ccSORFilterDlg.h"
#include "ccNoiseFilterDlg.h"
#include "ccDensityDlg.h"
#include "ccEntityPickerDlg.h"
#include "ccRasterizeTool.h"
#include "ccVolumeCalcTool.h"
#include "ccMatchScalesDlg.h"
#include "ccStereoModeDlg.h"
#include <ui_aboutDlg.h>

//other
#include "ccRegistrationTools.h"
#include "ccPersistentSettings.h"
#include "ccCropTool.h"

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
#include <QSettings>
#include <QMessageBox>
#include <QElapsedTimer>
#include <QInputDialog>
#include <QTextStream>
#include <QColorDialog>

//System
#include <string.h>
#include <math.h>
#include <assert.h>
#include <cfloat>
#include <iostream>

//global static pointer (as there should only be one instance of MainWindow!)
static MainWindow* s_instance = 0;

//default 'All files' file filter
static const QString s_allFilesFilter("All (*.*)");
//default file filter separator
static const QString s_fileFilterSeparator(";;");

//standard message in case of locked vertices
void DisplayLockedVerticesWarning(QString meshName, bool displayAsError)
{
	QString message = QString("Vertices of mesh '%1' are locked (they may be shared by multiple entities for instance).\nYou should call this method directly on the vertices cloud.\n(warning: all entities depending on this cloud will be impacted!)").arg(meshName);
	if (displayAsError)
		ccConsole::Error(message);
	else
		ccConsole::Warning(message);
}

MainWindow::MainWindow()
	: m_ccRoot(0)
	, m_uiFrozen(false)
	, m_3dMouseInput(0)
	, m_viewModePopupButton(0)
	, m_pivotVisibilityPopupButton(0)
	, m_cpeDlg(0)
	, m_gsTool(0)
	, m_seTool(0)
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
	QSettings settings;
	restoreGeometry(settings.value(ccPS::MainWinGeom()).toByteArray());

	setWindowTitle(QString("CloudCompare v")+ccCommon::GetCCVersion(false));

	//advanced widgets not handled by QDesigner
	{
		//view mode pop-up menu
		{
			m_viewModePopupButton = new QToolButton();
			QMenu* menu = new QMenu(m_viewModePopupButton);
			menu->addAction(actionSetOrthoView);
			menu->addAction(actionSetCenteredPerspectiveView);
			menu->addAction(actionSetViewerPerspectiveView);

			m_viewModePopupButton->setMenu(menu);
			m_viewModePopupButton->setPopupMode(QToolButton::InstantPopup);
			m_viewModePopupButton->setToolTip("Set current view mode");
			m_viewModePopupButton->setStatusTip(m_viewModePopupButton->toolTip());
			toolBarView->insertWidget(actionZoomAndCenter,m_viewModePopupButton);
			m_viewModePopupButton->setEnabled(false);
		}

		//pivot center pop-up menu
		{
			m_pivotVisibilityPopupButton = new QToolButton();
			QMenu* menu = new QMenu(m_pivotVisibilityPopupButton);
			menu->addAction(actionSetPivotAlwaysOn);
			menu->addAction(actionSetPivotRotationOnly);
			menu->addAction(actionSetPivotOff);

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
	ccConsole::Init(consoleWidget,this,this);

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
	connect(actionToggleActivation,	SIGNAL(triggered()), this, SLOT(toggleSelectedEntitiesActivation()));	//'A': toggles selected items activation
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

#ifndef Q_OS_MAC
	restoreState(settings.value(ccPS::MainWinState()).toByteArray());
#endif
}

MainWindow::~MainWindow()
{
	release3DMouse();
	cancelPreviousPickingOperation(false); //just in case

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
		for (int i=0; i<subWindowList.size(); ++i)
			static_cast<ccGLWindow*>(subWindowList[i]->widget())->setSceneDB(0);
	}
	m_cpeDlg = 0;
	m_gsTool = 0;
	m_seTool = 0;
	m_transTool = 0;
	m_clipTool = 0;
	m_compDlg = 0;
	m_ppDlg = 0;
	m_plpDlg = 0;
	m_pprDlg = 0;
	m_pfDlg = 0;

	//release all 'overlay' dialogs
	while (!m_mdiDialogs.empty())
	{
		m_mdiDialogs.back().dialog->disconnect();
		m_mdiDialogs.back().dialog->stop(false);
		m_mdiDialogs.back().dialog->setParent(0);
		delete m_mdiDialogs.back().dialog;
		m_mdiDialogs.pop_back();
	}
	//m_mdiDialogs.clear();
	m_mdiArea->closeAllSubWindows();

	if (ccRoot)
		delete ccRoot;

	ccConsole::ReleaseInstance();
}

ccPluginInterface* MainWindow::getValidPlugin(QObject* plugin)
{
	if (plugin)
	{
		//standard plugin?
		ccStdPluginInterface* ccStdPlugin = qobject_cast<ccStdPluginInterface*>(plugin);
		if (ccStdPlugin)
			return static_cast<ccPluginInterface*>(ccStdPlugin);

		//GL (shader) plugin
		ccGLFilterPluginInterface* ccGLPlugin = qobject_cast<ccGLFilterPluginInterface*>(plugin);
		if (ccGLPlugin)
			return static_cast<ccPluginInterface*>(ccGLPlugin);

		//I/O filter plugin
		ccIOFilterPluginInterface* ccIOPlugin = qobject_cast<ccIOFilterPluginInterface*>(plugin);
		if (ccIOPlugin)
			return static_cast<ccPluginInterface*>(ccIOPlugin);
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
	QString path = QCoreApplication::applicationDirPath();
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
			ccConsole::Print(QString("Found new plugin: '%1'").arg(filename));
			if (dispatchPlugin(plugin))
			{
				m_pluginFileNames += filename;
			}
			else
			{
				delete plugin;
				plugin = 0;
				ccConsole::Warning("\tUnsupported or invalid plugin type");
			}
		}
		else
		{
			delete plugin;
			plugin = 0;
			ccConsole::Warning(QString("[Plugin] %1")/*.arg(pluginsDir.absoluteFilePath(filename))*/.arg(loader.errorString()));
		}
	}

	if (menuPlugins)
	{
		menuPlugins->setEnabled(!m_stdPlugins.empty());
	}

	if (toolBarPluginTools->isEnabled())
	{
		actionDisplayPluginTools->setEnabled(true);
		actionDisplayPluginTools->setChecked(true);
	}
	else
	{
		//DGM: doesn't work :(
		//actionDisplayPluginTools->setChecked(false);
	}

	if (toolBarGLFilters->isEnabled())
	{
		actionDisplayGLFiltersTools->setEnabled(true);
		actionDisplayGLFiltersTools->setChecked(true);
	}
	else
	{
		//DGM: doesn't work :(
		//actionDisplayGLFiltersTools->setChecked(false);
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
		ccLog::Warning("\tplugin has an invalid (empty) name!");
		return false;
	}
	ccConsole::Print(QString("\tplugin name: [%1]").arg(pluginName));

	CC_PLUGIN_TYPE type = ccPlugin->getType();

	switch(type)
	{

	case CC_STD_PLUGIN: //standard plugin
		{
			ccStdPluginInterface* stdPlugin = static_cast<ccStdPluginInterface*>(ccPlugin);
			stdPlugin->setMainAppInterface(this);

			QMenu* destMenu = 0;
			QToolBar* destToolBar = 0;

			QActionGroup actions(this);
			stdPlugin->getActions(actions);
			if (actions.actions().size() > 1) //more than one action? We create it's own menu and toolbar
			{
				destMenu = (menuPlugins ? menuPlugins->addMenu(pluginName) : 0);
				if (destMenu)
					destMenu->setIcon(stdPlugin->getIcon());
				destToolBar = addToolBar(pluginName+QString(" toolbar"));

				if (destToolBar)
				{
					m_stdPluginsToolbars.push_back(destToolBar);
					//not sure why but it seems that we must specifically set the object name.
					//if not the QSettings thing will complain about a not-setted name
					//when saving settings of qCC mainwindow
					destToolBar->setObjectName(pluginName);
				}
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

			// see if this plugin can give back an additional factory for objects
			ccExternalFactory* factory = stdPlugin->getCustomObjectsFactory();
			if (factory) // if it is valid add to the plugin_factories
			{
				assert(ccExternalFactory::Container::GetUniqueInstance());
				ccExternalFactory::Container::GetUniqueInstance()->addFactory(factory);
			}
		}
		break;

	case CC_GL_FILTER_PLUGIN: //GL filter
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

	case CC_IO_FILTER_PLUGIN: //I/O filter
		{
			ccIOFilterPluginInterface* ioPlugin = static_cast<ccIOFilterPluginInterface*>(ccPlugin);
			FileIOFilter::Shared filter = ioPlugin->getFilter(this);
			if (filter)
			{
				FileIOFilter::Register(filter);
				ccConsole::Print(QString("\tfile extension: %1").arg(filter->getDefaultExtension().toUpper()));
			}
		}
		break;

	default:
		assert(false);
		ccLog::Warning("\tunhandled plugin type!");
		return false;
	}

	return true;
}

void MainWindow::doActionShowAboutPluginsDialog()
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

	if (win->areGLFiltersEnabled())
	{
		ccGlFilter* filter = static_cast<ccGLFilterPluginInterface*>(ccPlugin)->getFilter();
		if (filter)
		{
			win->setGlFilter(filter);
			ccConsole::Print("Note: go to << Display > Shaders & Filters > No filter >> to disable GL filter");
		}
		else
		{
			ccConsole::Error("Can't load GL filter (an error occurred)!");
		}
	}
	else
	{
		ccConsole::Error("GL filters not supported!");
	}
}

void MainWindow::release3DMouse()
{
#ifdef CC_3DXWARE_SUPPORT
	if (m_3dMouseInput)
	{
		m_3dMouseInput->disconnectDriver(); //disconnect from the driver
		m_3dMouseInput->disconnect(this); //disconnect from Qt ;)
		
		delete m_3dMouseInput;
		m_3dMouseInput = 0;
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
	{
		release3DMouse();
	}

	if (state)
	{
		m_3dMouseInput = new Mouse3DInput(this);
		if (m_3dMouseInput->connect(this,"CloudCompare"))
		{
			QObject::connect(m_3dMouseInput, SIGNAL(sigMove3d(std::vector<float>&)),	this,	SLOT(on3DMouseMove(std::vector<float>&)));
			QObject::connect(m_3dMouseInput, SIGNAL(sigReleased()),						this,	SLOT(on3DMouseReleased()));
			QObject::connect(m_3dMouseInput, SIGNAL(sigOn3dmouseKeyDown(int)),			this,	SLOT(on3DMouseKeyDown(int)));
			QObject::connect(m_3dMouseInput, SIGNAL(sigOn3dmouseKeyUp(int)),			this,	SLOT(on3DMouseKeyUp(int)));
		}
		else
		{
			delete m_3dMouseInput;
			m_3dMouseInput = 0;
			
			if (!silent)
			{
				ccLog::Error("[3D Mouse] No device found"); //warning message has already been issued by Mouse3DInput::connect
			}
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

// ANY CHANGE/BUG FIX SHOULD BE REFLECTED TO THE EQUIVALENT METHODS IN QCC "MainWindow.cpp" FILE!
void MainWindow::on3DMouseKeyDown(int key)
{
#ifdef CC_3DXWARE_SUPPORT

	switch(key)
	{
	case Mouse3DInput::V3DK_MENU:
		//should be handled by the driver now!
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
		//should be handled by the driver now!
		break;
	case Mouse3DInput::V3DK_PANZOOM:
		//should be handled by the driver now!
		break;
	case Mouse3DInput::V3DK_ISO1:
		setIsoView1();
		break;
	case Mouse3DInput::V3DK_ISO2:
		setIsoView2();
		break;
	case Mouse3DInput::V3DK_PLUS:
		//should be handled by the driver now!
		break;
	case Mouse3DInput::V3DK_MINUS:
		//should be handled by the driver now!
		break;
	case Mouse3DInput::V3DK_DOMINANT:
		//should be handled by the driver now!
		break;
	case Mouse3DInput::V3DK_CW:
	case Mouse3DInput::V3DK_CCW:
		{
			ccGLWindow* activeWin = getActiveGLWindow();
			if (activeWin)
			{
				CCVector3d axis(0,0,-1);
				CCVector3d trans(0,0,0);
				ccGLMatrixd mat;
				double angle = M_PI/2;
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

void MainWindow::on3DMouseMove(std::vector<float>& vec)
{
#ifdef CC_3DXWARE_SUPPORT

	ccGLWindow* win = getActiveGLWindow();

	//active window?
	if (win)
		Mouse3DInput::Apply(vec,win);

#endif
}

void MainWindow::on3DMouseReleased()
{
	//active window?
	ccGLWindow* win = getActiveGLWindow();
	if (win && win->getPivotVisibility() == ccGLWindow::PIVOT_SHOW_ON_MOVE)
	{
		//we have to hide the pivot symbol!
		win->showPivotSymbol(false);
		win->redraw();
	}
}

void MainWindow::connectActions()
{
	assert(m_ccRoot);
	assert(m_mdiArea);

	//TODO... but not ready yet ;)
	actionLoadShader->setVisible(false);
	actionDeleteShader->setVisible(false);
	actionKMeans->setVisible(false);
	actionFrontPropagation->setVisible(false);

	/*** MAIN MENU ***/

	//"File" menu
	connect(actionOpen,							SIGNAL(triggered()),	this,		SLOT(doActionLoadFile()));
	connect(actionSave,							SIGNAL(triggered()),	this,		SLOT(doActionSaveFile()));
	connect(actionPrimitiveFactory,				SIGNAL(triggered()),	this,		SLOT(doShowPrimitiveFactory()));
	connect(actionEnable3DMouse,				SIGNAL(toggled(bool)),	this,		SLOT(setup3DMouse(bool)));
	connect(actionCloseAll,						SIGNAL(triggered()),	this,		SLOT(closeAll()));
	connect(actionQuit,							SIGNAL(triggered()),	this,		SLOT(close()));

	//"Edit > Colors" menu
	connect(actionSetUniqueColor,				SIGNAL(triggered()),	this,		SLOT(doActionSetUniqueColor()));
	connect(actionSetColorGradient,				SIGNAL(triggered()),	this,		SLOT(doActionSetColorGradient()));
	connect(actionChangeColorLevels,			SIGNAL(triggered()),	this,		SLOT(doActionChangeColorLevels()));
	connect(actionColorize,						SIGNAL(triggered()),	this,		SLOT(doActionColorize()));
	connect(actionClearColor,					SIGNAL(triggered()),	this,		SLOT(doActionClearColor()));
	connect(actionInterpolateColors,			SIGNAL(triggered()),	this,		SLOT(doActionInterpolateColors()));

	//"Edit > Normals" menu
	connect(actionComputeNormals,				SIGNAL(triggered()),	this,		SLOT(doActionComputeNormals()));
	connect(actionInvertNormals,				SIGNAL(triggered()),	this,		SLOT(doActionInvertNormals()));
	connect(actionConvertNormalToHSV,			SIGNAL(triggered()),	this,		SLOT(doActionConvertNormalsToHSV()));
	connect(actionConvertNormalToDipDir,		SIGNAL(triggered()),	this,		SLOT(doActionConvertNormalsToDipDir()));
	connect(actionOrientNormalsMST,				SIGNAL(triggered()),	this,		SLOT(doActionOrientNormalsMST()));
	connect(actionOrientNormalsFM,				SIGNAL(triggered()),	this,		SLOT(doActionOrientNormalsFM()));
	connect(actionClearNormals,					SIGNAL(triggered()),	this,		SLOT(doActionClearNormals()));
	//"Edit > Octree" menu
	connect(actionComputeOctree,				SIGNAL(triggered()),	this,		SLOT(doActionComputeOctree()));
	connect(actionResampleWithOctree,			SIGNAL(triggered()),	this,		SLOT(doActionResampleWithOctree()));
	//"Edit > Mesh" menu
	connect(actionComputeMeshAA,				SIGNAL(triggered()),	this,		SLOT(doActionComputeMeshAA()));
	connect(actionComputeMeshLS,				SIGNAL(triggered()),	this,		SLOT(doActionComputeMeshLS()));
	connect(actionConvertTextureToColor,		SIGNAL(triggered()),	this,		SLOT(doActionConvertTextureToColor()));
	connect(actionSamplePoints,					SIGNAL(triggered()),	this,		SLOT(doActionSamplePoints()));
	connect(actionSmoothMeshLaplacian,			SIGNAL(triggered()),	this,		SLOT(doActionSmoothMeshLaplacian()));
	connect(actionSubdivideMesh,				SIGNAL(triggered()),	this,		SLOT(doActionSubdivideMesh()));
	connect(actionMeasureMeshSurface,			SIGNAL(triggered()),	this,		SLOT(doActionMeasureMeshSurface()));
	connect(actionMeasureMeshVolume,			SIGNAL(triggered()),	this,		SLOT(doActionMeasureMeshVolume()));
	connect(actionFlagMeshVetices,				SIGNAL(triggered()),	this,		SLOT(doActionFlagMeshVetices()));
	//"Edit > Mesh > Scalar Field" menu
	connect(actionSmoothMeshSF,					SIGNAL(triggered()),	this,		SLOT(doActionSmoothMeshSF()));
	connect(actionEnhanceMeshSF,				SIGNAL(triggered()),	this,		SLOT(doActionEnhanceMeshSF()));
	//"Edit > Sensor > Ground-Based lidar" menu
	connect(actionShowDepthBuffer,				SIGNAL(triggered()),	this,		SLOT(doActionShowDepthBuffer()));
	connect(actionExportDepthBuffer,			SIGNAL(triggered()),	this,		SLOT(doActionExportDepthBuffer()));
	connect(actionComputePointsVisibility,		SIGNAL(triggered()),	this,		SLOT(doActionComputePointsVisibility()));
	
	//"Edit > Sensor" menu
	connect(actionCreateGBLSensor,				SIGNAL(triggered()),	this,		SLOT(doActionCreateGBLSensor()));
	connect(actionCreateCameraSensor,			SIGNAL(triggered()),	this,		SLOT(doActionCreateCameraSensor()));
	connect(actionModifySensor,					SIGNAL(triggered()),	this,		SLOT(doActionModifySensor()));
	connect(actionProjectUncertainty,			SIGNAL(triggered()),	this,		SLOT(doActionProjectUncertainty()));
	connect(actionCheckPointsInsideFrustrum,	SIGNAL(triggered()),	this,		SLOT(doActionCheckPointsInsideFrustrum()));
	connect(actionComputeDistancesFromSensor,	SIGNAL(triggered()),	this,		SLOT(doActionComputeDistancesFromSensor()));
	connect(actionComputeScatteringAngles,		SIGNAL(triggered()),	this,		SLOT(doActionComputeScatteringAngles()));
	connect(actionViewFromSensor,				SIGNAL(triggered()),	this,		SLOT(doActionSetViewFromSensor()));
	//"Edit > Scalar fields" menu
	connect(actionShowHistogram,				SIGNAL(triggered()),	this,		SLOT(showSelectedEntitiesHistogram()));
	connect(actionComputeStatParams,			SIGNAL(triggered()),	this,		SLOT(doActionComputeStatParams()));
	connect(actionSFGradient,					SIGNAL(triggered()),	this,		SLOT(doActionSFGradient()));
	connect(actionGaussianFilter,				SIGNAL(triggered()),	this,		SLOT(doActionSFGaussianFilter()));
	connect(actionBilateralFilter,				SIGNAL(triggered()),	this,		SLOT(doActionSFBilateralFilter()));
	connect(actionFilterByValue,				SIGNAL(triggered()),	this,		SLOT(doActionFilterByValue()));
	connect(actionAddConstantSF,				SIGNAL(triggered()),	this,		SLOT(doActionAddConstantSF()));
	connect(actionScalarFieldArithmetic,		SIGNAL(triggered()),	this,		SLOT(doActionScalarFieldArithmetic()));
	connect(actionScalarFieldFromColor,			SIGNAL(triggered()),	this,		SLOT(doActionScalarFieldFromColor()));
	connect(actionConvertToRGB,					SIGNAL(triggered()),	this,		SLOT(doActionSFConvertToRGB()));
	connect(actionConvertToRandomRGB,			SIGNAL(triggered()),	this,		SLOT(doActionSFConvertToRandomRGB()));
	connect(actionRenameSF,						SIGNAL(triggered()),	this,		SLOT(doActionRenameSF()));
	connect(actionOpenColorScalesManager,		SIGNAL(triggered()),	this,		SLOT(doActionOpenColorScalesManager()));
	connect(actionAddIdField,					SIGNAL(triggered()),	this,		SLOT(doActionAddIdField()));
	connect(actionSetSFAsCoord,					SIGNAL(triggered()),	this,		SLOT(doActionSetSFAsCoord()));
	connect(actionDeleteScalarField,			SIGNAL(triggered()),	this,		SLOT(doActionDeleteScalarField()));
	connect(actionDeleteAllSF,					SIGNAL(triggered()),	this,		SLOT(doActionDeleteAllSF()));
	//"Edit" menu
	connect(actionClone,						SIGNAL(triggered()),	this,		SLOT(doActionClone()));
	connect(actionMerge,						SIGNAL(triggered()),	this,		SLOT(doActionMerge()));
	connect(actionApplyTransformation,			SIGNAL(triggered()),	this,		SLOT(doActionApplyTransformation()));
	connect(actionApplyScale,					SIGNAL(triggered()),	this,		SLOT(doActionApplyScale()));
	connect(actionTranslateRotate,				SIGNAL(triggered()),	this,		SLOT(activateTranslateRotateMode()));
	connect(actionSegment,						SIGNAL(triggered()),	this,		SLOT(activateSegmentationMode()));
	connect(actionCrop,							SIGNAL(triggered()),	this,		SLOT(doActionCrop()));
	connect(actionEditGlobalShiftAndScale,		SIGNAL(triggered()),	this,		SLOT(doActionEditGlobalShiftAndScale()));
	connect(actionSubsample,					SIGNAL(triggered()),	this,		SLOT(doActionSubsample()));
	connect(actionMatchBBCenters,				SIGNAL(triggered()),	this,		SLOT(doActionMatchBBCenters()));
	connect(actionMatchScales,					SIGNAL(triggered()),	this,		SLOT(doActionMatchScales()));
	connect(actionDelete,						SIGNAL(triggered()),	m_ccRoot,	SLOT(deleteSelectedEntities()));

	//"Tools > Clean" menu
	connect(actionSORFilter,					SIGNAL(triggered()),	this,		SLOT(doActionSORFilter()));
	connect(actionNoiseFilter,					SIGNAL(triggered()),	this,		SLOT(doActionFilterNoise()));
	
	//"Tools > Projection" menu
	connect(actionUnroll,						SIGNAL(triggered()),	this,		SLOT(doActionUnroll()));
	connect(actionRasterize,					SIGNAL(triggered()),	this,		SLOT(doActionRasterize()));
	connect(actionConvertPolylinesToMesh,		SIGNAL(triggered()),	this,		SLOT(doConvertPolylinesToMesh()));
	connect(actionExportCoordToSF,				SIGNAL(triggered()),	this,		SLOT(doActionExportCoordToSF()));
	//"Tools > Registration" menu
	connect(actionRegister,						SIGNAL(triggered()),	this,		SLOT(doActionRegister()));
	connect(actionPointPairsAlign,				SIGNAL(triggered()),	this,		SLOT(activateRegisterPointPairTool()));
	//"Tools > Distances" menu
	connect(actionCloudCloudDist,				SIGNAL(triggered()),	this,		SLOT(doActionCloudCloudDist()));
	connect(actionCloudMeshDist,				SIGNAL(triggered()),	this,		SLOT(doActionCloudMeshDist()));
	connect(actionCPS,							SIGNAL(triggered()),	this,		SLOT(doActionComputeCPS()));
	//"Tools > Volume" menu
	connect(actionCompute2HalfDimVolume,		SIGNAL(triggered()),	this,		SLOT(doCompute2HalfDimVolume()));
	//"Tools > Statistics" menu
	connect(actionComputeStatParams2,			SIGNAL(triggered()),	this,		SLOT(doActionComputeStatParams())); //duplicated action --> we can't use the same otherwise we get an ugly console warning on Linux :(
	connect(actionStatisticalTest,				SIGNAL(triggered()),	this,		SLOT(doActionStatisticalTest()));
	//"Tools > Segmentation" menu
	connect(actionLabelConnectedComponents,		SIGNAL(triggered()),	this,		SLOT(doActionLabelConnectedComponents()));
	connect(actionKMeans,						SIGNAL(triggered()),	this,		SLOT(doActionKMeans()));
	connect(actionFrontPropagation,				SIGNAL(triggered()),	this,		SLOT(doActionFrontPropagation()));
	connect(actionCrossSection,					SIGNAL(triggered()),	this,		SLOT(activateClippingBoxMode()));
	connect(actionExtractSections,				SIGNAL(triggered()),	this,		SLOT(activateSectionExtractionMode()));
	//"Tools > Fit" menu
	connect(actionFitPlane,						SIGNAL(triggered()),	this,		SLOT(doActionFitPlane()));
	connect(actionFitSphere,					SIGNAL(triggered()),	this,		SLOT(doActionFitSphere()));
	connect(actionFitFacet,						SIGNAL(triggered()),	this,		SLOT(doActionFitFacet()));
	connect(actionFitQuadric,					SIGNAL(triggered()),	this,		SLOT(doActionFitQuadric()));
	//"Tools > Other" menu
	connect(actionComputeDensity,				SIGNAL(triggered()),	this,		SLOT(doComputeDensity()));
	connect(actionCurvature,					SIGNAL(triggered()),	this,		SLOT(doComputeCurvature()));
	connect(actionRoughness,					SIGNAL(triggered()),	this,		SLOT(doComputeRoughness()));
	connect(actionRemoveDuplicatePoints,		SIGNAL(triggered()),	this,		SLOT(doRemoveDuplicatePoints()));
	//"Tools"
	connect(actionLevel,						SIGNAL(triggered()),	this,		SLOT(doLevel()));
	connect(actionPointListPicking,				SIGNAL(triggered()),	this,		SLOT(activatePointListPickingMode()));
	connect(actionPointPicking,					SIGNAL(triggered()),	this,		SLOT(activatePointPickingMode()));

	//"Tools > Sand box (research)" menu
	connect(actionComputeKdTree,				SIGNAL(triggered()),	this,		SLOT(doActionComputeKdTree()));
	connect(actionDistanceMapToMesh,			SIGNAL(triggered()),	this,		SLOT(doActionComputeDistanceMap()));
	connect(actionDistanceToBestFitQuadric3D,	SIGNAL(triggered()),	this,		SLOT(doActionComputeDistToBestFitQuadric3D()));
	connect(actionComputeBestFitBB,				SIGNAL(triggered()),	this,		SLOT(doComputeBestFitBB()));
	connect(actionAlign,						SIGNAL(triggered()),	this,		SLOT(doAction4pcsRegister())); //Aurelien BEY le 13/11/2008
	connect(actionSNETest,						SIGNAL(triggered()),	this,		SLOT(doSphericalNeighbourhoodExtractionTest()));
	connect(actionCNETest,						SIGNAL(triggered()),	this,		SLOT(doCylindricalNeighbourhoodExtractionTest()));
	connect(actionFindBiggestInnerRectangle,	SIGNAL(triggered()),	this,		SLOT(doActionFindBiggestInnerRectangle()));
	connect(actionExportCloudsInfo,				SIGNAL(triggered()),	this,		SLOT(doActionExportCloudsInfo()));
	connect(actionCreateCloudFromEntCenters,	SIGNAL(triggered()),	this,		SLOT(doActionCreateCloudFromEntCenters()));
	connect(actionComputeBestICPRmsMatrix,		SIGNAL(triggered()),	this,		SLOT(doActionComputeBestICPRmsMatrix()));

	//"Display" menu
	connect(actionFullScreen,					SIGNAL(toggled(bool)),	this,		SLOT(toggleFullScreen(bool)));
	connect(actionExclusiveFullScreen,			SIGNAL(toggled(bool)),	this,		SLOT(toggleExclusiveFullScreen(bool)));
	connect(actionRefresh,						SIGNAL(triggered()),	this,		SLOT(refreshAll()));
	connect(actionTestFrameRate,				SIGNAL(triggered()),	this,		SLOT(testFrameRate()));
	connect(actionToggleCenteredPerspective,	SIGNAL(triggered()),	this,		SLOT(toggleActiveWindowCenteredPerspective()));
	connect(actionToggleViewerBasedPerspective, SIGNAL(triggered()),	this,		SLOT(toggleActiveWindowViewerBasedPerspective()));
	connect(actionLockRotationVertAxis,			SIGNAL(triggered()),	this,		SLOT(toggleRotationAboutVertAxis()));
	connect(actionEnterBubbleViewMode,			SIGNAL(triggered()),	this,		SLOT(doActionEnableBubbleViewMode()));
	connect(actionEditCamera,					SIGNAL(triggered()),	this,		SLOT(doActionEditCamera()));
	connect(actionAdjustZoom,					SIGNAL(triggered()),	this,		SLOT(doActionAdjustZoom()));
	connect(actionSaveViewportAsObject,			SIGNAL(triggered()),	this,		SLOT(doActionSaveViewportAsCamera()));

	//"Display > Lights & Materials" menu
	connect(actionDisplayOptions,				SIGNAL(triggered()),	this,		SLOT(setLightsAndMaterials()));
	connect(actionToggleSunLight,				SIGNAL(triggered()),	this,		SLOT(toggleActiveWindowSunLight()));
	connect(actionToggleCustomLight,			SIGNAL(triggered()),	this,		SLOT(toggleActiveWindowCustomLight()));
	connect(actionRenderToFile,					SIGNAL(triggered()),	this,		SLOT(doActionRenderToFile()));
	//"Display > Shaders & filters" menu
	connect(actionLoadShader,					SIGNAL(triggered()),	this,		SLOT(doActionLoadShader()));
	connect(actionDeleteShader,					SIGNAL(triggered()),	this,		SLOT(doActionDeleteShader()));
	connect(actionNoFilter,						SIGNAL(triggered()),	this,		SLOT(doDisableGLFilter()));

	//"Display > Active SF" menu
	connect(actionToggleActiveSFColorScale,		SIGNAL(triggered()),	this,		SLOT(doActionToggleActiveSFColorScale()));
	connect(actionShowActiveSFPrevious,			SIGNAL(triggered()),	this,		SLOT(doActionShowActiveSFPrevious()));
	connect(actionShowActiveSFNext,				SIGNAL(triggered()),	this,		SLOT(doActionShowActiveSFNext()));

	//"Display" menu
	connect(actionResetGUIElementsPos,			SIGNAL(triggered()),	this,		SLOT(doActionResetGUIElementsPos()));

	//"3D Views" menu
	connect(menu3DViews,						SIGNAL(aboutToShow()),	this,		SLOT(update3DViewsMenu()));
	connect(actionNew3DView,					SIGNAL(triggered()),	this,		SLOT(new3DView()));
	connect(actionZoomIn,						SIGNAL(triggered()),	this,		SLOT(zoomIn()));
	connect(actionZoomOut,						SIGNAL(triggered()),	this,		SLOT(zoomOut()));
	connect(actionClose3DView,					SIGNAL(triggered()),	m_mdiArea,	SLOT(closeActiveSubWindow()));
	connect(actionCloseAll3DViews,				SIGNAL(triggered()),	m_mdiArea,	SLOT(closeAllSubWindows()));
	connect(actionTile3DViews,					SIGNAL(triggered()),	m_mdiArea,	SLOT(tileSubWindows()));
	connect(actionCascade3DViews,				SIGNAL(triggered()),	m_mdiArea,	SLOT(cascadeSubWindows()));
	connect(actionNext3DView,					SIGNAL(triggered()),	m_mdiArea,	SLOT(activateNextSubWindow()));
	connect(actionPrevious3DView,				SIGNAL(triggered()),	m_mdiArea,	SLOT(activatePreviousSubWindow()));

	//"About" menu entry
	connect(actionHelp,							SIGNAL(triggered()),	this,		SLOT(doActionShowHelpDialog()));
	connect(actionAbout,						SIGNAL(triggered()),	this,		SLOT(doActionShawAboutDialog()));
	connect(actionAboutPlugins,					SIGNAL(triggered()),	this,		SLOT(doActionShowAboutPluginsDialog()));

	/*** Toolbars ***/

	//View toolbar
	connect(actionGlobalZoom,					SIGNAL(triggered()),	this,		SLOT(setGlobalZoom()));
	connect(actionPickRotationCenter,			SIGNAL(triggered()),	this,		SLOT(doPickRotationCenter()));
	connect(actionZoomAndCenter,				SIGNAL(triggered()),	this,		SLOT(zoomOnSelectedEntities()));
	connect(actionSetPivotAlwaysOn,				SIGNAL(triggered()),	this,		SLOT(setPivotAlwaysOn()));
	connect(actionSetPivotRotationOnly,			SIGNAL(triggered()),	this,		SLOT(setPivotRotationOnly()));
	connect(actionSetPivotOff,					SIGNAL(triggered()),	this,		SLOT(setPivotOff()));
	connect(actionSetOrthoView,					SIGNAL(triggered()),	this,		SLOT(setOrthoView()));
	connect(actionSetCenteredPerspectiveView,	SIGNAL(triggered()),	this,		SLOT(setCenteredPerspectiveView()));
	connect(actionSetViewerPerspectiveView,		SIGNAL(triggered()),	this,		SLOT(setViewerPerspectiveView()));
	connect(actionSetViewTop,					SIGNAL(triggered()),	this,		SLOT(setTopView()));
	connect(actionSetViewBottom,				SIGNAL(triggered()),	this,		SLOT(setBottomView()));
	connect(actionSetViewFront,					SIGNAL(triggered()),	this,		SLOT(setFrontView()));
	connect(actionSetViewBack,					SIGNAL(triggered()),	this,		SLOT(setBackView()));
	connect(actionSetViewLeft,					SIGNAL(triggered()),	this,		SLOT(setLeftView()));
	connect(actionSetViewRight,					SIGNAL(triggered()),	this,		SLOT(setRightView()));
	connect(actionSetViewIso1,					SIGNAL(triggered()),	this,		SLOT(setIsoView1()));
	connect(actionSetViewIso2,					SIGNAL(triggered()),	this,		SLOT(setIsoView2()));
	connect(actionEnableStereo,					SIGNAL(toggled(bool)),	this,		SLOT(toggleActiveWindowStereoVision(bool)));

	//hidden
	connect(actionEnableVisualDebugTraces,		SIGNAL(triggered()),	this,		SLOT(toggleVisualDebugTraces()));
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
		if (ent->isA(CC_TYPES::HIERARCHY_OBJECT))
		{
			//automatically parse a group's children set
			for (unsigned i=0; i<ent->getChildrenNumber(); ++i)
				selectedEntities.push_back(ent->getChild(i));
		}
		else if (ent->isA(CC_TYPES::POINT_CLOUD) || ent->isA(CC_TYPES::MESH))
		{
			ccPointCloud* cloud = 0;
			if (ent->isA(CC_TYPES::POINT_CLOUD))
			{
				cloud = static_cast<ccPointCloud*>(ent);
			}
			else
			{
				ccMesh* mesh = static_cast<ccMesh*>(ent);
				ccGenericPointCloud* vertices = mesh->getAssociatedCloud();
				if (	!vertices
					||	!vertices->isA(CC_TYPES::POINT_CLOUD)
					||	(vertices->isLocked() && !mesh->isAncestorOf(vertices)) )
				{
					ccLog::Warning(QString("[SetColor] Can't set color for mesh '%1' (vertices are not accessible)").arg(ent->getName()));
					continue;
				}

				cloud = static_cast<ccPointCloud*>(vertices);
			}

			if (colorize)
			{
				cloud->colorize(static_cast<float>(newCol.redF()),
								static_cast<float>(newCol.greenF()),
								static_cast<float>(newCol.blueF()) );
			}
			else
			{
				cloud->setRGBColor(	static_cast<ColorCompType>(newCol.red()),
									static_cast<ColorCompType>(newCol.green()),
									static_cast<ColorCompType>(newCol.blue()) );
			}
			cloud->showColors(true);
			cloud->prepareDisplayForRefresh();

			if (ent != cloud)
				ent->showColors(true);
			else if (cloud->getParent() && cloud->getParent()->isKindOf(CC_TYPES::MESH))
				cloud->getParent()->showColors(true);
		}
		else if (ent->isKindOf(CC_TYPES::PRIMITIVE))
		{
			ccGenericPrimitive* prim = ccHObjectCaster::ToPrimitive(ent);
			ccColor::Rgb col(	static_cast<ColorCompType>(newCol.red()),
								static_cast<ColorCompType>(newCol.green()),
								static_cast<ColorCompType>(newCol.blue()) );
			prim->setColor(col);
			ent->showColors(true);
			ent->prepareDisplayForRefresh();
		}
		else if (ent->isA(CC_TYPES::POLY_LINE))
		{
			ccPolyline* poly = ccHObjectCaster::ToPolyline(ent);
			ccColor::Rgb col(	static_cast<ColorCompType>(newCol.red()),
								static_cast<ColorCompType>(newCol.green()),
								static_cast<ColorCompType>(newCol.blue()) );
			poly->setColor(col);
			ent->showColors(true);
			ent->prepareDisplayForRefresh();
		}
		else if (ent->isA(CC_TYPES::FACET))
		{
			ccFacet* facet = ccHObjectCaster::ToFacet(ent);
			ccColor::Rgb col(	static_cast<ColorCompType>(newCol.red()),
								static_cast<ColorCompType>(newCol.green()),
								static_cast<ColorCompType>(newCol.blue()) );
			facet->setColor(col);
			ent->showColors(true);
			ent->prepareDisplayForRefresh();
		}
		else
		{
			ccLog::Warning(QString("[SetColor] Can't change color of entity '%1'").arg(ent->getName()));
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionSetColorGradient()
{
	ccColorGradientDlg dlg(this);
	if (!dlg.exec())
		return;

	unsigned char dim = dlg.getDimension();
	ccColorGradientDlg::GradientType ramp = dlg.getType();

	ccColorScale::Shared colorScale(0);
	if (ramp == ccColorGradientDlg::Default)
	{
		colorScale = ccColorScalesManager::GetDefaultScale();
	}
	else if (ramp == ccColorGradientDlg::TwoColors)
	{
		colorScale = ccColorScale::Create("Temp scale");
		QColor first,second;
		dlg.getColors(first,second);
		colorScale->insert(ccColorScaleElement(0.0,first),false);
		colorScale->insert(ccColorScaleElement(1.0,second),true);
	}
	assert(colorScale || ramp == ccColorGradientDlg::Banding);

	size_t selNum = m_selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = m_selectedEntities[i];

		bool lockedVertices;
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
		if (lockedVertices)
		{
			DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
			continue;
		}

		if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD)) // TODO
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

			bool success = false;
			if (ramp == ccColorGradientDlg::Banding)
				success = pc->setRGBColorByBanding(dim, dlg.getBandingFrequency());
			else
				success = pc->setRGBColorByHeight(dim, colorScale);

			if (success)
			{
				ent->showColors(true);
				ent->prepareDisplayForRefresh();
			}
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionChangeColorLevels()
{
	if (m_selectedEntities.size() != 1)
	{
		ccConsole::Error("Select one and only one colored cloud or mesh!");
		return;
	}

	bool lockedVertices;
	ccPointCloud* pointCloud = ccHObjectCaster::ToPointCloud(m_selectedEntities[0],&lockedVertices);
	if (!pointCloud || lockedVertices)
	{
		if (lockedVertices)
			DisplayLockedVerticesWarning(pointCloud->getName(),true);
		return;
	}

	if (!pointCloud->hasColors())
	{
		ccConsole::Error("Selected entity has no colors!");
		return;
	}

	ccColorLevelsDlg dlg(this,pointCloud);
	dlg.exec();
}

void MainWindow::doActionInterpolateColors()
{
	if (m_selectedEntities.size() != 2)
	{
		ccConsole::Error("Select 2 entities (clouds or meshes)!");
		return;
	}

	ccHObject* ent1 = m_selectedEntities[0];
	ccHObject* ent2 = m_selectedEntities[1];

	ccGenericPointCloud* cloud1 = ccHObjectCaster::ToGenericPointCloud(ent1);
	ccGenericPointCloud* cloud2 = ccHObjectCaster::ToGenericPointCloud(ent2);

	if (!cloud1 || !cloud2)
	{
		ccConsole::Error("Select 2 entities (clouds or meshes)!");
		return;
	}

	if (!cloud1->hasColors() && !cloud2->hasColors())
	{
		ccConsole::Error("None of the selected entities has per-point or per-vertex colors!");
		return;
	}
	else if (cloud1->hasColors() && cloud2->hasColors())
	{
		ccConsole::Error("Both entities have colors! Remove the colors on the entity you wish to import the colors to!");
		return;
	}

	ccGenericPointCloud* source = cloud1;
	ccGenericPointCloud* dest = cloud2;

	if ( cloud2->hasColors())
	{
		std::swap(source,dest);
		std::swap(cloud1,cloud2);
		std::swap(ent1,ent2);
	}

	if (!dest->isA(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error("Destination cloud (or vertices) must be a real point cloud!");
		return;
	}

	bool ok;
	unsigned char s_defaultLevel = 7;
	int value = QInputDialog::getInt(this,"Interpolate colors", "Octree level", s_defaultLevel, 1, CCLib::DgmOctree::MAX_OCTREE_LEVEL, 1, &ok);
	if (!ok)
		return;
	s_defaultLevel = static_cast<unsigned char>(value);

	ccProgressDialog pDlg(true, this);
	if (static_cast<ccPointCloud*>(dest)->interpolateColorsFrom(source,&pDlg,s_defaultLevel))
	{
		ent2->showColors(true);
	}
	else
	{
		ccConsole::Error("An error occurred! (see console)");
	}

	ent2->prepareDisplayForRefresh_recursive();
	refreshAll();
	updateUI();
}

void MainWindow::doActionInvertNormals()
{
	size_t selNum = m_selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = m_selectedEntities[i];
		bool lockedVertices;
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
		if (lockedVertices)
		{
			DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
			continue;
		}

		if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD)) // TODO
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

void MainWindow::doActionConvertNormalsToDipDir()
{
	doActionConvertNormalsTo(DIP_DIR_SFS);
}

void MainWindow::doActionConvertNormalsToHSV()
{
	doActionConvertNormalsTo(HSV_COLORS);
}

void MainWindow::doActionConvertNormalsTo(NORMAL_CONVERSION_DEST dest)
{
	unsigned errorCount = 0;

	size_t selNum = m_selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = m_selectedEntities[i];
		bool lockedVertices;
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
		if (lockedVertices)
		{
			DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
			continue;
		}

		if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD)) // TODO
		{
			ccPointCloud* ccCloud = static_cast<ccPointCloud*>(cloud);
			if (ccCloud->hasNormals())
			{
				bool success = true;
				switch(dest)
				{
				case HSV_COLORS:
					{
						success = ccCloud->convertNormalToRGB();
						if (success)
						{
							ccCloud->showSF(false);
							ccCloud->showNormals(false);
							ccCloud->showColors(true);
						}
					}
					break;
				case DIP_DIR_SFS:
					{
						//get/create 'dip' scalar field
						int dipSFIndex = ccCloud->getScalarFieldIndexByName(CC_DEFAULT_DIP_SF_NAME);
						if (dipSFIndex < 0)
							dipSFIndex = ccCloud->addScalarField(CC_DEFAULT_DIP_SF_NAME);
						if (dipSFIndex < 0)
						{
							ccLog::Warning("[MainWindow::doActionConvertNormalsTo] Not enough memory!");
							success = false;
							break;
						}

						//get/create 'dip direction' scalar field
						int dipDirSFIndex = ccCloud->getScalarFieldIndexByName(CC_DEFAULT_DIP_DIR_SF_NAME);
						if (dipDirSFIndex < 0)
							dipDirSFIndex = ccCloud->addScalarField(CC_DEFAULT_DIP_DIR_SF_NAME);
						if (dipDirSFIndex < 0)
						{
							ccCloud->deleteScalarField(dipSFIndex);
							ccLog::Warning("[MainWindow::doActionConvertNormalsTo] Not enough memory!");
							success = false;
							break;
						}

						ccScalarField* dipSF = static_cast<ccScalarField*>(ccCloud->getScalarField(dipSFIndex));
						ccScalarField* dipDirSF = static_cast<ccScalarField*>(ccCloud->getScalarField(dipDirSFIndex));
						assert(dipSF && dipDirSF);

						success = ccCloud->convertNormalToDipDirSFs(dipSF, dipDirSF);

						if (success)
						{
							//apply default 360 degrees color scale!
							ccColorScale::Shared scale = ccColorScalesManager::GetDefaultScale(ccColorScalesManager::HSV_360_DEG);
							dipSF->setColorScale(scale);
							dipDirSF->setColorScale(scale);
							ccCloud->setCurrentDisplayedScalarField(dipDirSFIndex); //dip dir. seems more interesting by default
							ccCloud->showSF(true);
						}
						else
						{
							ccCloud->deleteScalarField(dipSFIndex);
							ccCloud->deleteScalarField(dipDirSFIndex);
						}
					}
					break;
				default:
					assert(false);
					ccLog::Warning("[MainWindow::doActionConvertNormalsTo] Internal error: unhandled destination!");
					success = false;
					i = selNum; //no need to process the selected entities anymore!
					break;
				}

				if (success)
				{
					ccCloud->prepareDisplayForRefresh_recursive();
				}
				else
				{
					++errorCount;
				}
			}
		}
	}

	//errors should have been sent to console as warnings
	if (errorCount)
	{
		ccConsole::Error("Error(s) occurred! (see console)");
	}

	refreshAll();
	updateUI();
}

static double s_kdTreeMaxErrorPerCell = 0.1;
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
			DisplayLockedVerticesWarning(ent->getName(),true);
			return;
		}
	}

	if (!cloud)
	{
		ccLog::Error("Selected one and only one point cloud or mesh!");
		return;
	}

	bool ok;
	s_kdTreeMaxErrorPerCell = QInputDialog::getDouble(this, "Compute Kd-tree", "Max error per leaf cell:", s_kdTreeMaxErrorPerCell, 1.0e-6, 1.0e6, 6, &ok);
	if (!ok)
		return;

	ccProgressDialog pDlg(true,this);

	//computation
	QElapsedTimer eTimer;
	eTimer.start();
	ccKdTree* kdtree = new ccKdTree(cloud);

	if (kdtree->build(s_kdTreeMaxErrorPerCell,CCLib::DistanceComputationTools::MAX_DIST_95_PERCENT,4,1000,&pDlg))
	{
		qint64 elapsedTime_ms = eTimer.elapsed();

		ccConsole::Print("[doActionComputeKdTree] Timing: %2.3f s",static_cast<double>(elapsedTime_ms)/1.0e3);
		cloud->setEnabled(true); //for mesh vertices!
		cloud->addChild(kdtree);
		kdtree->setDisplay(cloud->getDisplay());
		kdtree->setVisible(true);
		kdtree->prepareDisplayForRefresh();
#ifdef _DEBUG
		kdtree->convertCellIndexToSF();
#else
		kdtree->convertCellIndexToRandomColor();
#endif

		addToDB(kdtree);

		refreshAll();
		updateUI();
	}
	else
	{
		ccLog::Error("An error occurred!");
		delete kdtree;
		kdtree = 0;
	}
}

void MainWindow::doActionComputeOctree()
{
	ccBBox bbox;
	std::set<ccGenericPointCloud*> clouds;
	size_t selNum = m_selectedEntities.size();
	PointCoordinateType maxBoxSize = -1;
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = m_selectedEntities[i];

		//specific test for locked vertices
		bool lockedVertices;
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
		if (cloud && lockedVertices)
		{
			DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
			continue;
		}
		clouds.insert(cloud);

		//we look for the biggest box so as to define the "minimum cell size"
		ccBBox thisBBox = cloud->getOwnBB();
		if (thisBBox.isValid())
		{
			CCVector3 dd = thisBBox.maxCorner()-thisBBox.minCorner();
			PointCoordinateType maxd = std::max(dd.x,std::max(dd.y,dd.z));
			if (maxBoxSize < 0.0 || maxd > maxBoxSize)
				maxBoxSize = maxd;
		}
		bbox += thisBBox;
	}

	if (clouds.empty() || maxBoxSize < 0.0)
	{
		ccLog::Warning("[doActionComputeOctree] No eligible entities in selection!");
		return;
	}

	//min(cellSize) = max(dim)/2^N with N = max subidivision level
	double minCellSize = static_cast<double>(maxBoxSize)/(1 << ccOctree::MAX_OCTREE_LEVEL);

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
		ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(cloud);

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
					PointCoordinateType halfBoxWidth = (PointCoordinateType)(cellSize * (1 << ccOctree::MAX_OCTREE_LEVEL) / 2.0);
					CCVector3 C = cloud->getOwnBB().getCenter();
					bbox = ccBBox(	C-CCVector3(halfBoxWidth,halfBoxWidth,halfBoxWidth),
									C+CCVector3(halfBoxWidth,halfBoxWidth,halfBoxWidth));
				}
				cloud->deleteOctree();
				octree = new ccOctree(cloud);
				if (octree->build(bbox.minCorner(),bbox.maxCorner(),0,0,&pDlg) > 0)
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
		qint64 elapsedTime_ms = eTimer.elapsed();

		//put object back in tree
		putObjectBackIntoDBTree(cloud,objContext);

		if (octree)
		{
			ccConsole::Print("[doActionComputeOctree] Timing: %2.3f s",static_cast<double>(elapsedTime_ms)/1.0e3);
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
	bool ok;
	int pointCount = QInputDialog::getInt(this,"Resample with octree", "Points (approx.)", 1000000, 1, INT_MAX, 100000, &ok);
	if (!ok)
		return;

	ccProgressDialog pDlg(false,this);
	assert(pointCount > 0);
	unsigned aimedPoints = static_cast<unsigned>(pointCount);

	ccHObject::Container selectedEntities = m_selectedEntities;
	size_t selNum = selectedEntities.size();
	bool errors = false;
	for (size_t i=0; i<selNum; ++i)
	{
		ccPointCloud* cloud = 0;
		ccHObject* ent = selectedEntities[i];
		/*if (ent->isKindOf(CC_TYPES::MESH)) //TODO
			cloud = ccHObjectCaster::ToGenericMesh(ent)->getAssociatedCloud();
		else */
		if (ent->isKindOf(CC_TYPES::POINT_CLOUD))
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
				ccPointCloud* newCloud = ccPointCloud::From(result,cloud);

				delete result;
				result = 0;

				if (newCloud)
				{
					addToDB(newCloud);
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
		ccLog::Error("[ResampleWithOctree] Errors occurred during the process! Result may be incomplete!");

	refreshAll();
}

void MainWindow::doActionApplyTransformation()
{
	ccApplyTransformationDlg dlg(this);
	if (!dlg.exec())
		return;

	ccGLMatrixd transMat = dlg.getTransformation();
	applyTransformation(transMat);
}

void MainWindow::applyTransformation(const ccGLMatrixd& mat)
{
	//if the transformation is partly converted to global shift/scale
	bool updateGlobalShiftAndScale = false;
	double scaleChange = 1.0;
	CCVector3d shiftChange(0,0,0);
	ccGLMatrixd transMat = mat;

	//we must backup 'm_selectedEntities' as removeObjectTemporarilyFromDBTree can modify it!
	ccHObject::Container selectedEntities = m_selectedEntities;

	//special case: the selected entity is a group
	//if (selectedEntities.size() == 1 && selectedEntities.front()->isA(CC_TYPES::HIERARCHY_OBJECT))
	//{
	//	ccHObject* ent = selectedEntities.front();
	//	m_selectedEntities.clear();
	//	for (unsigned i=0; i<ent->getChildrenNumber(); ++i)
	//	{
	//		selectedEntities.push_back(ent->getChild(i));
	//	}
	//}

	size_t selNum = selectedEntities.size();
	bool firstCloud = true;
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = selectedEntities[i];

		//we don't test primitives (it's always ok while the 'vertices lock' test would fail)
		if (!ent->isKindOf(CC_TYPES::PRIMITIVE))
		{
			//specific test for locked vertices
			bool lockedVertices;
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
			if (cloud)
			{
				if (lockedVertices)
				{
					DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
					continue;
				}

				if (firstCloud)
				{
					//test if the translated cloud was already "too big"
					//(in which case we won't bother the user about the fact
					//that the transformed cloud will be too big...)
					ccBBox localBBox = ent->getOwnBB();
					CCVector3d Pl = CCVector3d::fromArray(localBBox.minCorner().u);
					double Dl = localBBox.getDiagNormd();

					//the cloud was alright
					if (	!ccGlobalShiftManager::NeedShift(Pl)
						&&	!ccGlobalShiftManager::NeedRescale(Dl))
					{
						//test if the translated cloud is not "too big" (in local coordinate space)
						ccBBox rotatedBox = ent->getOwnBB() * transMat;
						double Dl2 = rotatedBox.getDiagNorm();
						CCVector3d Pl2 = CCVector3d::fromArray(rotatedBox.getCenter().u);

						bool needShift = ccGlobalShiftManager::NeedShift(Pl2);
						bool needRescale = ccGlobalShiftManager::NeedRescale(Dl2);

						if (needShift || needRescale)
						{
							//existing shift information
							CCVector3d globalShift = cloud->getGlobalShift();
							double globalScale = cloud->getGlobalScale();
			
							//we compute the transformation matrix in the global coordinate space
							ccGLMatrixd globalTransMat = transMat;
							globalTransMat.scale(1.0/globalScale);
							globalTransMat.setTranslation(globalTransMat.getTranslationAsVec3D() - globalShift);
							//and we apply it to the cloud bounding-box
							ccBBox rotatedBox = cloud->getOwnBB() * globalTransMat;
							double Dg = rotatedBox.getDiagNorm();
							CCVector3d Pg = CCVector3d::fromArray(rotatedBox.getCenter().u);

							//ask the user the right values!
							ccShiftAndScaleCloudDlg sasDlg(Pl2,Dl2,Pg,Dg,this);
							sasDlg.showApplyAllButton(false);
							sasDlg.showTitle(true);
							sasDlg.setKeepGlobalPos(true);
							sasDlg.showKeepGlobalPosCheckbox(false); //we don't want the user to mess with this!

							//add "original" entry
							int index = sasDlg.addShiftInfo(ccShiftAndScaleCloudDlg::ShiftInfo("Original",globalShift,globalScale));
							//sasDlg.setCurrentProfile(index);
							//add "suggested" entry
							CCVector3d suggestedShift = ccGlobalShiftManager::BestShift(Pg);
							double suggestedScale = ccGlobalShiftManager::BestScale(Dg);
							index = sasDlg.addShiftInfo(ccShiftAndScaleCloudDlg::ShiftInfo("Suggested",suggestedShift,suggestedScale));
							sasDlg.setCurrentProfile(index);
							//add "last" entry (if available)
							ccShiftAndScaleCloudDlg::ShiftInfo lastInfo;
							if (sasDlg.getLast(lastInfo))
								sasDlg.addShiftInfo(lastInfo);
							//add entries from file (if any)
							sasDlg.addFileInfo();

							if (sasDlg.exec())
							{
								//get the relative modification to existing global shift/scale info
								assert(cloud->getGlobalScale() != 0);
								scaleChange = sasDlg.getScale() / cloud->getGlobalScale();
								shiftChange =  (sasDlg.getShift() - cloud->getGlobalShift());

								updateGlobalShiftAndScale = (scaleChange != 1.0 || shiftChange.norm2() != 0);

								//update transformation matrix accordingly
								if (updateGlobalShiftAndScale)
								{
									transMat.scale(scaleChange);
									transMat.setTranslation(transMat.getTranslationAsVec3D()+shiftChange*scaleChange);
								}
							}
							else if (sasDlg.cancelled())
							{
								ccLog::Warning("[ApplyTransformation] Process cancelled by user");
								return;
							}
						}
					}

					firstCloud = false;
				}

				if (updateGlobalShiftAndScale)
				{
					//apply translation as global shift
					cloud->setGlobalShift(cloud->getGlobalShift() + shiftChange);
					cloud->setGlobalScale(cloud->getGlobalScale() * scaleChange);
					const CCVector3d& T = cloud->getGlobalShift();
					double scale = cloud->getGlobalScale();
					ccLog::Warning(QString("[ApplyTransformation] Cloud '%1' global shift/scale information has been updated: shift = (%2,%3,%4) / scale = %5").arg(cloud->getName()).arg(T.x).arg(T.y).arg(T.z).arg(scale));
				}
			}
		}

		//we temporarily detach entity, as it may undergo
		//"severe" modifications (octree deletion, etc.) --> see ccHObject::applyRigidTransformation
		ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(ent);
		ent->setGLTransformation(ccGLMatrix(transMat.data()));
		//DGM FIXME: we only test the entity own bounding box (and we update its shift & scale info) but we apply the transformation to all its children?!
		ent->applyGLTransformation_recursive();
		ent->prepareDisplayForRefresh_recursive();
		putObjectBackIntoDBTree(ent,objContext);
	}

	//reselect previously selected entities!
	if (m_ccRoot)
		m_ccRoot->selectEntities(selectedEntities);
	
	ccLog::Print("[ApplyTransformation] Applied transformation matrix:");
	ccLog::Print(transMat.toString(12,' ')); //full precision
	ccLog::Print("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool");

	//reselect previously selected entities!
	if (m_ccRoot)
		m_ccRoot->selectEntities(selectedEntities);

	refreshAll();
}

static CCVector3d s_lastMultFactors(1.0,1.0,1.0);
static bool s_lastMultKeepInPlace = true;
typedef std::pair<ccHObject*,ccGenericPointCloud*> EntityCloudAssociation;
void MainWindow::doActionApplyScale()
{
	ccAskThreeDoubleValuesDlg dlg("fx","fy","fz",-1.0e6,1.0e6,s_lastMultFactors.x,s_lastMultFactors.y,s_lastMultFactors.z,8,"Scaling",this);
	dlg.showCheckbox("Keep in place",s_lastMultKeepInPlace,"Whether the cloud (center) should stay at the same place or not (i.e. coordinates are multiplied directly)");
	if (!dlg.exec())
		return;

	//save values for next time
	double sX = s_lastMultFactors.x = dlg.doubleSpinBox1->value();
	double sY = s_lastMultFactors.y = dlg.doubleSpinBox2->value();
	double sZ = s_lastMultFactors.z = dlg.doubleSpinBox3->value();
	bool keepInPlace = s_lastMultKeepInPlace = dlg.getCheckboxState();

	//we must backup 'm_selectedEntities' as removeObjectTemporarilyFromDBTree can modify it!
	ccHObject::Container selectedEntities = m_selectedEntities;
	size_t selNum = selectedEntities.size();

	//first check that all coordinates are kept 'small'
	std::vector< EntityCloudAssociation > candidates;
	{
		bool testBigCoordinates = true;
		//size_t processNum = 0;
		for (size_t i=0; i<selNum; ++i)
		{
			ccHObject* ent = selectedEntities[i];
			bool lockedVertices;
			//try to get the underlying cloud (or the vertices set for a mesh)
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
			//otherwise we can look if the selected entity is a polyline
			if (!cloud && ent->isA(CC_TYPES::POLY_LINE))
			{
				cloud = dynamic_cast<ccGenericPointCloud*>(static_cast<ccPolyline*>(ent)->getAssociatedCloud());
				if (!cloud || cloud->isAncestorOf(ent))
					lockedVertices = true;
			}
			if (!cloud || !cloud->isKindOf(CC_TYPES::POINT_CLOUD))
			{
				ccLog::Warning(QString("[Apply scale] Entity '%1' can't be scaled this way").arg(ent->getName()));
				continue;
			}
			if (lockedVertices)
			{
				DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
				//++processNum;
				continue;
			}

			CCVector3 C(0,0,0);
			if (keepInPlace)
				C = cloud->getOwnBB().getCenter();

			//we must check that the resulting cloud coordinates are not too big
			if (testBigCoordinates)
			{
				ccBBox bbox = cloud->getOwnBB();
				CCVector3 bbMin = bbox.minCorner();
				CCVector3 bbMax = bbox.maxCorner();

				double maxx = std::max(fabs(bbMin.x), fabs(bbMax.x));
				double maxy = std::max(fabs(bbMin.y), fabs(bbMax.y));
				double maxz = std::max(fabs(bbMin.z), fabs(bbMax.z));

				const double maxCoord = ccGlobalShiftManager::MaxCoordinateAbsValue();
				bool oldCoordsWereTooBig = (	maxx > maxCoord
					||	maxy > maxCoord
					||	maxz > maxCoord );

				if (!oldCoordsWereTooBig)
				{
					maxx = std::max( fabs((bbMin.x - C.x) * sX + C.x), fabs( (bbMax.x - C.x) * sX + C.x) );
					maxy = std::max( fabs((bbMin.y - C.y) * sY + C.y), fabs( (bbMax.y - C.y) * sY + C.y) );
					maxz = std::max( fabs((bbMin.z - C.z) * sZ + C.z), fabs( (bbMax.z - C.z) * sZ + C.z) );

					bool newCoordsAreTooBig = (	maxx > maxCoord
						||	maxy > maxCoord
						||	maxz > maxCoord );

					if (newCoordsAreTooBig)
					{
						if (QMessageBox::question(	this,
							"Big coordinates",
							"Resutling coordinates will be too big (original precision may be lost!). Proceeed anyway?",
							QMessageBox::Yes,
							QMessageBox::No) == QMessageBox::Yes)
						{
							//ok, we won't test anymore and proceed
							testBigCoordinates = false;
						}
						else
						{
							//we stop the process
							return;
						}
					}
				}
			}

			assert(cloud);
			candidates.push_back(EntityCloudAssociation(ent,cloud));
		}
	}

	if (candidates.empty())
	{
		ccConsole::Warning("[Apply scale] No eligible entities (point clouds or meshes) were selected!");
		return;
	}

	//now do the real scaling work
	{
		for (size_t i=0; i<candidates.size(); ++i)
		{
			ccHObject* ent = candidates[i].first;
			ccGenericPointCloud* cloud = candidates[i].second;

			CCVector3 C(0,0,0);
			if (keepInPlace)
				C = cloud->getOwnBB().getCenter();

			//we temporarily detach entity, as it may undergo
			//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::scale
			ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(cloud);
			
			cloud->scale(	static_cast<PointCoordinateType>(sX),
							static_cast<PointCoordinateType>(sY),
							static_cast<PointCoordinateType>(sZ),
							C );

			putObjectBackIntoDBTree(cloud,objContext);
			cloud->prepareDisplayForRefresh_recursive();

			//don't forget the 'global shift'!
			//DGM: but not the global scale!
			if (!keepInPlace)
			{
				const CCVector3d& shift = cloud->getGlobalShift();
				cloud->setGlobalShift( CCVector3d(	shift.x*sX,
													shift.y*sY,
													shift.z*sZ) );
			}

			ent->prepareDisplayForRefresh_recursive();
		}
	}

	//reselect previously selected entities!
	if (m_ccRoot)
		m_ccRoot->selectEntities(selectedEntities);

	if (!keepInPlace)
		zoomOnSelectedEntities();

	refreshAll();
	updateUI();
}

void MainWindow::doActionEditGlobalShiftAndScale()
{
	size_t selNum = m_selectedEntities.size();

	//get the global shift/scale info and bounding box of all selected clouds
	std::vector< std::pair<ccShiftedObject*,ccHObject*> > shiftedEntities;
	CCVector3d Pl(0,0,0);
	double Dl = 1.0;
	CCVector3d Pg(0,0,0);
	double Dg = 1.0;
	//shift and scale (if unique)
	CCVector3d shift(0,0,0);
	double scale = 1.0;
	{
		bool uniqueShift = true;
		bool uniqueScale = true;
		ccBBox localBB;
		//sadly we don't have a double-typed bounding box class yet ;)
		CCVector3d globalBBmin(0,0,0), globalBBmax(0,0,0);

		for (size_t i=0; i<selNum; ++i)
		{
			ccHObject* ent = m_selectedEntities[i];
			bool lockedVertices;
			ccShiftedObject* shifted = ccHObjectCaster::ToShifted(ent,&lockedVertices);
			//for (unlocked) entities only
			if (lockedVertices)
			{
				//get the vertices
				assert(ent->isKindOf(CC_TYPES::MESH));
				ccGenericPointCloud* vertices = static_cast<ccGenericMesh*>(ent)->getAssociatedCloud();
				if (!vertices || !ent->isAncestorOf(vertices))
				{
					DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
					continue;
				}
				ent = vertices;
			}

			CCVector3 Al = ent->getOwnBB().minCorner();
			CCVector3 Bl = ent->getOwnBB().maxCorner();
			CCVector3d Ag = shifted->toGlobal3d<PointCoordinateType>(Al);
			CCVector3d Bg = shifted->toGlobal3d<PointCoordinateType>(Bl);

			//update local BB
			localBB.add(Al);
			localBB.add(Bl);

			//update global BB
			if (shiftedEntities.empty())
			{
				globalBBmin = Ag;
				globalBBmax = Bg;
				shift = shifted->getGlobalShift();
				uniqueScale = shifted->getGlobalScale();
			}
			else
			{
				globalBBmin = CCVector3d(	std::min(globalBBmin.x,Ag.x),
											std::min(globalBBmin.y,Ag.y),
											std::min(globalBBmin.z,Ag.z) );
				globalBBmax = CCVector3d(	std::max(globalBBmax.x,Bg.x),
											std::max(globalBBmax.y,Bg.y),
											std::max(globalBBmax.z,Bg.z) );

				if (uniqueShift)
					uniqueShift = ((shifted->getGlobalShift() - shift).norm() < ZERO_TOLERANCE);
				if (uniqueScale)
					uniqueScale = (fabs(shifted->getGlobalScale() - scale) < ZERO_TOLERANCE);
			}

			shiftedEntities.push_back( std::pair<ccShiftedObject*,ccHObject*>(shifted,ent) );
		}

		Pg = globalBBmin;
		Dg = (globalBBmax-globalBBmin).norm();

		Pl = CCVector3d::fromArray(localBB.minCorner().u);
		Dl = (localBB.maxCorner()-localBB.minCorner()).normd();

		if (!uniqueShift)
			shift = Pl - Pg;
		if (!uniqueScale)
			scale = Dg / Dl;
	}

	if (shiftedEntities.empty())
		return;

	ccShiftAndScaleCloudDlg sasDlg(Pl,Dl,Pg,Dg,this);
	sasDlg.showApplyAllButton(shiftedEntities.size() > 1);
	sasDlg.showApplyButton(shiftedEntities.size() == 1);
	sasDlg.showNoButton(false);
	//add "original" entry
	int index = sasDlg.addShiftInfo(ccShiftAndScaleCloudDlg::ShiftInfo("Original",shift,scale));
	sasDlg.setCurrentProfile(index);
	//add "last" entry (if available)
	ccShiftAndScaleCloudDlg::ShiftInfo lastInfo;
	if (sasDlg.getLast(lastInfo))
		sasDlg.addShiftInfo(lastInfo);
	//add entries from file (if any)
	sasDlg.addFileInfo();

	if (!sasDlg.exec())
		return;

	shift = sasDlg.getShift();
	scale = sasDlg.getScale();
	bool preserveGlobalPos = sasDlg.keepGlobalPos();

	ccLog::Print("[Global Shift/Scale] New shift: (%f, %f, %f)",shift.x,shift.y,shift.z);
	ccLog::Print("[Global Shift/Scale] New scale: %f",scale);

	//apply new shift
	{
		for (size_t i=0; i<shiftedEntities.size(); ++i)
		{
			ccShiftedObject* shifted = shiftedEntities[i].first;
			ccHObject* ent = shiftedEntities[i].second;
			if (preserveGlobalPos)
			{
				//to preserve the global position of the cloud, we may have to translate and/or rescale the cloud
				CCVector3d Ql = CCVector3d::fromArray(ent->getOwnBB().minCorner().u);
				CCVector3d Qg = shifted->toGlobal3d(Ql);
				CCVector3d Ql2 = Qg * scale + shift;
				CCVector3d T = Ql2 - Ql;

				assert(shifted->getGlobalScale() > 0);
				double scaleCoef = scale / shifted->getGlobalScale();

				if (T.norm() > ZERO_TOLERANCE || fabs(scaleCoef-1.0) > ZERO_TOLERANCE)
				{
					ccGLMatrix transMat;
					transMat.toIdentity();
					transMat.scale(static_cast<float>(scaleCoef));
					transMat.setTranslation(T);

					//DGM FIXME: we only test the entity own bounding box (and we update its shift & scale info) but we apply the transformation to all its children?!
					ent->applyGLTransformation_recursive(&transMat);
					ent->prepareDisplayForRefresh_recursive();

					ccLog::Warning(QString("[Global Shift/Scale] To preserve its original position, the entity '%1' has been translated of (%2,%3,%4) and rescaled of a factor %5")
									.arg(ent->getName())
									.arg(T.x)
									.arg(T.y)
									.arg(T.z)
									.arg(scaleCoef));
				}
			}
			shifted->setGlobalShift(shift);
			shifted->setGlobalScale(scale);
		}
	}

	refreshAll();
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
	size_t selNum = selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = selectedEntities[i];
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);

		if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD)) // TODO
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
					for (unsigned j=0; j<3; ++j)
					{
						double u[3];
						eig.getEigenValueAndVector(j,u);
						CCVector3 v(static_cast<PointCoordinateType>(u[0]),
									static_cast<PointCoordinateType>(u[1]),
									static_cast<PointCoordinateType>(u[2]));
						v.normalize();
						rotMat[j*4]		= static_cast<float>(v.x);
						rotMat[j*4+1]	= static_cast<float>(v.y);
						rotMat[j*4+2]	= static_cast<float>(v.z);
					}

					const CCVector3* G = Yk.getGravityCenter();
					assert(G);
					trans.shiftRotationCenter(*G);

					cloud->setGLTransformation(trans);
					trans.invert();

					//we temporarily detach entity, as it may undergo
					//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::applyRigidTransformation
					ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(cloud);
					static_cast<ccPointCloud*>(cloud)->applyRigidTransformation(trans);
					putObjectBackIntoDBTree(cloud,objContext);

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

	size_t selNum = selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = selectedEntities[i];

		//specific case: clear normals on a mesh
		if (prop == 1 && ( ent->isA(CC_TYPES::MESH) /*|| ent->isKindOf(CC_TYPES::PRIMITIVE)*/ )) //TODO
		{
			ccMesh* mesh = ccHObjectCaster::ToMesh(ent);
			if (mesh->hasTriNormals())
			{
				mesh->showNormals(false);
				ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(mesh);
				mesh->clearTriNormals();
				putObjectBackIntoDBTree(mesh,objContext);
				ent->prepareDisplayForRefresh();
				continue;
			}
			else if (mesh->hasNormals()) //per-vertex normals?
			{
				if (mesh->getParent()
					&& (mesh->getParent()->isA(CC_TYPES::MESH)/*|| mesh->getParent()->isKindOf(CC_TYPES::PRIMITIVE)*/) //TODO
					&& ccHObjectCaster::ToMesh(mesh->getParent())->getAssociatedCloud() == mesh->getAssociatedCloud())
				{
					ccLog::Warning("[doActionClearNormals] Can't remove per-vertex normals on a sub mesh!");
				}
				else //mesh is alone, we can freely remove normals
				{
					if (mesh->getAssociatedCloud() && mesh->getAssociatedCloud()->isA(CC_TYPES::POINT_CLOUD))
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
			DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
			continue;
		}

		if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD)) // TODO
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

void MainWindow::doActionMeasureMeshVolume()
{
	size_t selNum = m_selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = m_selectedEntities[i];
		if (ent->isKindOf(CC_TYPES::MESH))
		{
			ccMesh* mesh = ccHObjectCaster::ToMesh(ent);
			if (mesh)
			{
				//first check that the mesh is closed
				CCLib::MeshSamplingTools::EdgeConnectivityStats stats;
				if (CCLib::MeshSamplingTools::computeMeshEdgesConnectivity(mesh,stats))
				{
					if (stats.edgesNotShared != 0)
					{
						ccConsole::Warning(QString("[Mesh Volume Measurer] The computed volume might be invalid (mesh '%1' has holes)").arg(ent->getName()));
					}
					else if (stats.edgesSharedByMore != 0)
					{
						ccConsole::Warning(QString("[Mesh Volume Measurer] The computed volume might be invalid (mesh '%1' has non-manifold edges)").arg(ent->getName()));
					}
				}
				else
				{
					ccConsole::Warning(QString("[Mesh Volume Measurer] The computed volume might be invalid (not enough memory to check if mesh '%1' is closed)").arg(ent->getName()));
				}
				//then we compute the mesh volume
				double V = CCLib::MeshSamplingTools::computeMeshVolume(mesh);
				//we force the console to display itself
				forceConsoleDisplay();
				ccConsole::Print(QString("[Mesh Volume Measurer] Mesh '%1': V=%2 (cube units)").arg(ent->getName()).arg(V));
			}
			else
			{
				assert(false);
			}
		}
	}
}

void MainWindow::doActionFlagMeshVetices()
{
	size_t selNum = m_selectedEntities.size();
	bool errors = false;
	bool success = false;
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = m_selectedEntities[i];
		if (ent->isKindOf(CC_TYPES::MESH))
		{
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(ent);
			ccPointCloud* vertices = ccHObjectCaster::ToPointCloud(mesh ? mesh->getAssociatedCloud() : 0);
			if (mesh && vertices)
			{
				//prepare a new scalar field
				int sfIdx = vertices->getScalarFieldIndexByName(CC_DEFAULT_MESH_VERT_FLAGS_SF_NAME);
				if (sfIdx < 0)
				{
					sfIdx = vertices->addScalarField(CC_DEFAULT_MESH_VERT_FLAGS_SF_NAME);
					if (sfIdx < 0)
					{
						ccConsole::Warning(QString("Not enough memory to flag the vertices of mesh '%1'!").arg(mesh->getName()));
						errors = true;
						continue;
					}
				}
				CCLib::ScalarField* flags = vertices->getScalarField(sfIdx);

				CCLib::MeshSamplingTools::EdgeConnectivityStats stats;
				if (CCLib::MeshSamplingTools::flagMeshVerticesByType(mesh,flags,&stats))
				{
					vertices->setCurrentDisplayedScalarField(sfIdx);
					ccScalarField* sf = vertices->getCurrentDisplayedScalarField();
					if (sf)
					{
						sf->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::VERTEX_QUALITY));
						//sf->setColorRampSteps(3); //ugly :(
					}
					vertices->showSF(true);
					mesh->showSF(true);
					mesh->prepareDisplayForRefresh_recursive();
					success = true;

					//display stats in the Console as well
					ccConsole::Print(QString("[Mesh Quality] Mesh '%1' edges: %2 total (normal: %3 / on hole borders: %4 / non-manifold: %5)").arg(ent->getName()).arg(stats.edgesCount).arg(stats.edgesSharedByTwo).arg(stats.edgesNotShared).arg(stats.edgesSharedByMore));
				}
				else
				{
					vertices->deleteScalarField(sfIdx);
					sfIdx = -1;
					ccConsole::Warning(QString("Not enough memory to flag the vertices of mesh '%1'!").arg(mesh->getName()));
					errors = true;
				}
			}
			else
			{
				assert(false);
			}
		}
	}

	refreshAll();
	updateUI();
	
	if (success)
	{
		//display reminder
		forceConsoleDisplay();
		ccConsole::Print(QString("[Mesh Quality] SF flags: %1 (NORMAL) / %2 (BORDER) / (%3) NON-MANIFOLD").arg(CCLib::MeshSamplingTools::VERTEX_NORMAL).arg(CCLib::MeshSamplingTools::VERTEX_BORDER).arg(CCLib::MeshSamplingTools::VERTEX_NON_MANIFOLD));
	}

	if (errors)
	{
		ccConsole::Error("Error(s) occurred! Check the console...");
	}
}

void MainWindow::doActionMeasureMeshSurface()
{
	size_t selNum = m_selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = m_selectedEntities[i];
		if (ent->isKindOf(CC_TYPES::MESH))
		{
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(ent);
			if (mesh)
			{
				double S = CCLib::MeshSamplingTools::computeMeshArea(mesh);
				//we force the console to display itself
				forceConsoleDisplay();
				ccConsole::Print(QString("[Mesh Surface Measurer] Mesh %1: S=%2 (square units)").arg(ent->getName()).arg(S));
				if (mesh->size())
					ccConsole::Print(QString("[Mesh Surface Measurer] Average triangle surface: %1 (square units)").arg(S/double(mesh->size())));
			}
			else
			{
				assert(false);
			}
		}
	}
}

void MainWindow::doActionComputeDistancesFromSensor()
{
	//we support more than just one sensor in selection
	if (m_selectedEntities.empty())
	{
		ccConsole::Error("Select at least a sensor.");
		return;
	}

	//start dialog
	ccSensorComputeDistancesDlg cdDlg(this);
	if (!cdDlg.exec())
		return;

	for (size_t i=0; i<m_selectedEntities.size(); ++i)
	{
		ccSensor* sensor = ccHObjectCaster::ToSensor(m_selectedEntities[i]);
		assert(sensor);
		if (!sensor)
			continue; //skip this entity

		//get associated cloud
		ccHObject* defaultCloud = sensor->getParent() && sensor->getParent()->isA(CC_TYPES::POINT_CLOUD) ? sensor->getParent() : 0;
		ccPointCloud* cloud = askUserToSelectACloud(defaultCloud, "Select a cloud on which to project the uncertainty:");
		if (!cloud)
		{
			return;
		}

		//sensor center
		CCVector3 sensorCenter;
		if (!sensor->getActiveAbsoluteCenter(sensorCenter))
			return;

		//squared required?
		bool squared = cdDlg.computeSquaredDistances();

		//set up a new scalar field
		const char* defaultRangesSFname = squared ? CC_DEFAULT_SQUARED_RANGES_SF_NAME : CC_DEFAULT_RANGES_SF_NAME;
		int sfIdx = cloud->getScalarFieldIndexByName(defaultRangesSFname);
		if (sfIdx < 0)
		{
			sfIdx = cloud->addScalarField(defaultRangesSFname);
			if (sfIdx < 0)
			{
				ccConsole::Error("Not enough memory!");
				return;
			}
		}
		CCLib::ScalarField* distances = cloud->getScalarField(sfIdx);

		for (unsigned i=0; i<cloud->size(); ++i)
		{
			const CCVector3* P = cloud->getPoint(i);
			ScalarType s = static_cast<ScalarType>(squared ? (*P-sensorCenter).norm2() : (*P-sensorCenter).norm());
			distances->setValue(i, s);
		}

		distances->computeMinAndMax();
		cloud->setCurrentDisplayedScalarField(sfIdx);
		cloud->showSF(true);
		cloud->prepareDisplayForRefresh_recursive();
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionComputeScatteringAngles()
{
	//there should be only one sensor in current selection!
	if (m_selectedEntities.size() != 1 || !m_selectedEntities[0]->isKindOf(CC_TYPES::GBL_SENSOR))
	{
		ccConsole::Error("Select one and only one GBL sensor!");
		return;
	}

	ccSensor* sensor = ccHObjectCaster::ToSensor(m_selectedEntities[0]);
	assert(sensor);

	//sensor center
	CCVector3 sensorCenter;
	if (!sensor->getActiveAbsoluteCenter(sensorCenter))
		return;

	//get associated cloud
	ccHObject* defaultCloud = sensor->getParent() && sensor->getParent()->isA(CC_TYPES::POINT_CLOUD) ? sensor->getParent() : 0;
	ccPointCloud* cloud = askUserToSelectACloud(defaultCloud, "Select a cloud on which to project the uncertainty:");
	if (!cloud)
	{
		return;
	}
	if (!cloud->hasNormals())
	{
		ccConsole::Error("The cloud must have normals!");
		return;
	}

	ccSensorComputeScatteringAnglesDlg cdDlg(this);
	if (!cdDlg.exec())
		return;

	bool toDegreeFlag = cdDlg.anglesInDegrees();

	//prepare a new scalar field
	const char* defaultScatAnglesSFname = toDegreeFlag ? CC_DEFAULT_DEG_SCATTERING_ANGLES_SF_NAME : CC_DEFAULT_RAD_SCATTERING_ANGLES_SF_NAME;
	int sfIdx = cloud->getScalarFieldIndexByName(defaultScatAnglesSFname);
	if (sfIdx < 0)
	{
		sfIdx = cloud->addScalarField(defaultScatAnglesSFname);
		if (sfIdx < 0)
		{
			ccConsole::Error("Not enough memory!");
			return;
		}
	}
	CCLib::ScalarField* angles = cloud->getScalarField(sfIdx);

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
		PointCoordinateType cosTheta = ray.dot(normal);
		ScalarType theta = static_cast<ScalarType>( acos(std::min<PointCoordinateType>(fabs(cosTheta),1)) );

		if (toDegreeFlag)
			theta *= static_cast<ScalarType>(CC_RAD_TO_DEG);

		angles->setValue(i,theta);
	}

	angles->computeMinAndMax();
	cloud->setCurrentDisplayedScalarField(sfIdx);
	cloud->showSF(true);
	cloud->prepareDisplayForRefresh_recursive();

	refreshAll();
	updateUI();
}

void MainWindow::doActionSetViewFromSensor()
{
	//there should be only one sensor in current selection!
	if (m_selectedEntities.size() != 1 || !m_selectedEntities[0]->isKindOf(CC_TYPES::SENSOR))
	{
		ccConsole::Error("Select one and only one sensor!");
		return;
	}

	ccSensor* sensor = ccHObjectCaster::ToSensor(m_selectedEntities[0]);
	assert(sensor);

	//try to find the associated window
	ccGenericGLDisplay* win = sensor->getDisplay();
	if (!win)
	{
		//get associated cloud
		ccPointCloud * cloud = ccHObjectCaster::ToPointCloud(sensor->getParent());
		if (cloud)
			win = cloud->getDisplay();
	}

	if (sensor->applyViewport(win))
	{
		ccConsole::Print("[doActionSetViewFromSensor] Viewport applied");
	}
}

void MainWindow::doActionCreateGBLSensor()
{
	ccGBLSensorProjectionDlg spDlg(this);
	if (!spDlg.exec())
		return;

	//We create the corresponding sensor for each input cloud (in a perfect world, there should be only one ;)
	ccHObject::Container selectedEntities = m_selectedEntities;
	size_t selNum = selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = selectedEntities[i];
		if (ent->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);

			//we create a new sensor
			ccGBLSensor* sensor = new ccGBLSensor();

			//we init its parameters with the dialog
			spDlg.updateGBLSensor(sensor);

			//we compute projection
			if (sensor->computeAutoParameters(cloud))
			{
				cloud->addChild(sensor);

				//we try to guess the sensor relative size (dirty)
				ccBBox bb = cloud->getOwnBB();
				double diag = bb.getDiagNorm();
				if (diag < 1.0)
					sensor->setGraphicScale(static_cast<PointCoordinateType>(1.0e-3));
				else if (diag > 10000.0)
					sensor->setGraphicScale(static_cast<PointCoordinateType>(1.0e3));

				//we display depth buffer
				int errorCode;
				if (sensor->computeDepthBuffer(cloud,errorCode))
				{
					ccRenderingTools::ShowDepthBuffer(sensor,this);
				}
				else
				{
					ccConsole::Error(ccGBLSensor::GetErrorString(errorCode));
				}

				////DGM: test
				//{
				//	//add positions
				//	const unsigned count = 1000;
				//	const PointCoordinateType R = 100;
				//	const PointCoordinateType dh = 100;
				//	for (unsigned i=0; i<1000; ++i)
				//	{
				//		float angle = (float)i/(float)count * 6 * M_PI;
				//		float X = R * cos(angle);
				//		float Y = R * sin(angle);
				//		float Z = (float)i/(float)count * dh;

				//		ccIndexedTransformation trans;
				//		trans.initFromParameters(-angle,CCVector3(0,0,1),CCVector3(X,Y,Z));
				//		sensor->addPosition(trans,i);
				//	}
				//}

				//set position
				//ccIndexedTransformation trans;
				//sensor->addPosition(trans,0);

				ccGLWindow* win = static_cast<ccGLWindow*>(cloud->getDisplay());
				if (win)
				{
					sensor->setDisplay_recursive(win);
					sensor->setVisible(true);
					ccBBox box = cloud->getOwnBB();
					win->updateConstellationCenterAndZoom(&box);
				}

				addToDB(sensor);
			}
			else
			{
				ccLog::Error("Failed to create sensor");
				delete sensor;
				sensor = 0;
			}
		}
	}

	updateUI();
}

void MainWindow::doActionCreateCameraSensor()
{
	ccCamSensorProjectionDlg spDlg(this);
	if (!spDlg.exec())
		return;

	//We create the corresponding sensor for each input cloud
	ccHObject::Container selectedEntities = m_selectedEntities;
	size_t selNum = selectedEntities.size();

	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = selectedEntities[i];

		if (ent->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);

			//we create a new sensor
			ccCameraSensor* sensor = new ccCameraSensor();
			cloud->addChild(sensor);

			//we init its parameters with the dialog
			spDlg.updateCamSensor(sensor);

			//we try to guess the sensor relative size (dirty)
			ccBBox bb = cloud->getOwnBB();
			double diag = bb.getDiagNorm();
			if (diag < 1.0)
				sensor->setGraphicScale(static_cast<PointCoordinateType>(1.0e-3));
			else if (diag > 10000.0)
				sensor->setGraphicScale(static_cast<PointCoordinateType>(1.0e3));

			//set position
			ccIndexedTransformation trans;
			sensor->addPosition(trans,0);

			ccGLWindow* win = static_cast<ccGLWindow*>(cloud->getDisplay());
			if (win)
			{
				sensor->setDisplay(win);
				sensor->setVisible(true);
				ccBBox box = cloud->getOwnBB();
				win->updateConstellationCenterAndZoom(&box);
			}

			addToDB(sensor);
		}
	}

	updateUI();
}

void MainWindow::doActionModifySensor()
{
	//there should be only one point cloud with sensor in current selection!
	if (m_selectedEntities.empty() || m_selectedEntities.size()>1 || !m_selectedEntities[0]->isKindOf(CC_TYPES::SENSOR))
	{
		ccConsole::Error("Select one and only one sensor!");
		return;
	}

	ccSensor* sensor = static_cast<ccSensor*>(m_selectedEntities[0]);

	//Ground based laser sensors
	if (sensor->isA(CC_TYPES::GBL_SENSOR))
	{
		ccGBLSensor* gbl = static_cast<ccGBLSensor*>(sensor);

		ccGBLSensorProjectionDlg spDlg(this);
		spDlg.initWithGBLSensor(gbl);
		
		if (!spDlg.exec())
			return;

		//we update its parameters
		spDlg.updateGBLSensor(gbl);

		//we re-project the associated cloud (if any)
		if (gbl->getParent() && gbl->getParent()->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(gbl->getParent());

			int errorCode;
			if (gbl->computeDepthBuffer(cloud,errorCode))
			{
				//we display depth buffer
				ccRenderingTools::ShowDepthBuffer(gbl,this);
			}
			else
			{
				ccConsole::Error(ccGBLSensor::GetErrorString(errorCode));
			}
		}
		else
		{
			//ccConsole::Warning(QString("Internal error: sensor ('%1') parent is not a point cloud!").arg(sensor->getName()));
		}
	}
	//Camera sensors
	else if (sensor->isA(CC_TYPES::CAMERA_SENSOR))
	{
		ccCameraSensor* cam = static_cast<ccCameraSensor*>(sensor);

		ccCamSensorProjectionDlg spDlg(this);
		spDlg.initWithCamSensor(cam);
		
		if (!spDlg.exec())
			return;

		//we update its parameters
		spDlg.updateCamSensor(cam);
	}
	else
	{
		ccConsole::Error("Can't modify this kind of sensor!");
		return;
	}

	if (sensor->isVisible() && sensor->isEnabled())
	{
		sensor->prepareDisplayForRefresh();
		refreshAll();
	}

	updateUI();
}

ccPointCloud* MainWindow::askUserToSelectACloud(ccHObject* defaultCloudEntity/*=0*/, QString inviteMessage/*=QString()*/)
{
	ccHObject::Container clouds;
	m_ccRoot->getRootEntity()->filterChildren(clouds,true,CC_TYPES::POINT_CLOUD,true);
	if (clouds.empty())
	{
		ccConsole::Error("No cloud in database!");
		return 0;
	}
	//default selected index
	int selectedIndex = 0;
	if (defaultCloudEntity)
	{
		for (size_t i=1; i<clouds.size(); ++i)
		{
			if (clouds[i] == defaultCloudEntity)
			{
				selectedIndex = static_cast<int>(i);
				break;
			}
		}
	}
	//ask the user to choose a cloud
	{
		selectedIndex = ccEntityPickerDlg::SelectEntity(clouds,selectedIndex,this,inviteMessage);
		if (selectedIndex < 0)
			return 0;
	}

	assert(selectedIndex >= 0 && static_cast<size_t>(selectedIndex) < clouds.size());
	return ccHObjectCaster::ToPointCloud(clouds[selectedIndex]);
}

void MainWindow::doActionProjectUncertainty()
{
	//there should only be one sensor in the current selection!
	if (m_selectedEntities.size() != 1 || !m_selectedEntities[0]->isKindOf(CC_TYPES::CAMERA_SENSOR))
	{
		ccConsole::Error("Select one and only one camera (projective) sensor!");
		return;
	}

	ccCameraSensor* sensor = ccHObjectCaster::ToCameraSensor(m_selectedEntities[0]);
	if (!sensor)
	{
		assert(false);
		return;
	}

	const ccCameraSensor::LensDistortionParameters::Shared& distParams = sensor->getDistortionParameters();
	if (!distParams || distParams->getModel() != ccCameraSensor::BROWN_DISTORTION)
	{
		ccLog::Error("Sensor has no associated uncertainty model! (Brown, etc.)");
		return;
	}

	//we need a cloud to project the uncertainty on!
	ccHObject* defaultCloud = sensor->getParent() && sensor->getParent()->isA(CC_TYPES::POINT_CLOUD) ? sensor->getParent() : 0;
	ccPointCloud* pointCloud = askUserToSelectACloud(defaultCloud, "Select a cloud on which to project the uncertainty:");
	if (!pointCloud)
	{
		return;
	}

	CCLib::ReferenceCloud points(pointCloud);
	if (!points.reserve(pointCloud->size()))
	{
		ccConsole::Error("Not enough memory!");
		return;
	}
	points.addPointIndex(0,pointCloud->size());

	// compute uncertainty
	std::vector< Vector3Tpl<ScalarType> > accuracy;
	if (!sensor->computeUncertainty(&points, accuracy/*, false*/))
	{
		ccConsole::Error("Not enough memory!");
		return;
	}

	/////////////
	// SIGMA D //
	/////////////
	const char dimChar[3] = {'x','y','z'};
	for (unsigned d=0; d<3; ++d)
	{
		// add scalar field
		QString sfName = QString("[%1] Uncertainty (%2)").arg(sensor->getName()).arg(dimChar[d]);
		int sfIdx = pointCloud->getScalarFieldIndexByName(qPrintable(sfName));
		if (sfIdx < 0)
			sfIdx = pointCloud->addScalarField(qPrintable(sfName));
		if (sfIdx < 0)
		{
			ccLog::Error("An error occured! (see console)");
			return;
		}

		// fill scalar field
		CCLib::ScalarField* sf = pointCloud->getScalarField(sfIdx);
		assert(sf);
		if (sf)
		{
			unsigned count = static_cast<unsigned>(accuracy.size());
			assert(count == pointCloud->size());
			for (unsigned i=0; i<count; i++)
				sf->setValue(i, accuracy[i].u[d]);
			sf->computeMinAndMax();
		}
	}

	/////////////////
	// SIGMA TOTAL //
	/////////////////

	// add scalar field
	{
		QString sfName = QString("[%1] Uncertainty (3D)").arg(sensor->getName());
		int sfIdx = pointCloud->getScalarFieldIndexByName(qPrintable(sfName));
		if (sfIdx < 0)
			sfIdx = pointCloud->addScalarField(qPrintable(sfName));
		if (sfIdx < 0)
		{
			ccLog::Error("An error occured! (see console)");
			return;
		}

		// fill scalar field
		CCLib::ScalarField* sf = pointCloud->getScalarField(sfIdx);
		assert(sf);
		if (sf)
		{
			unsigned count = static_cast<unsigned>(accuracy.size());
			assert(count == pointCloud->size());
			for (unsigned i=0; i<count; i++)
				sf->setValue(i, accuracy[i].norm());
			sf->computeMinAndMax();
		}

		pointCloud->showSF(true);
		pointCloud->setCurrentDisplayedScalarField(sfIdx);
		pointCloud->prepareDisplayForRefresh();
	}

	refreshAll();
}

void MainWindow::doActionCheckPointsInsideFrustrum()
{
	//there should be only one camera sensor in the current selection!
	if (m_selectedEntities.size() != 1 || !m_selectedEntities[0]->isKindOf(CC_TYPES::CAMERA_SENSOR))
	{
		ccConsole::Error("Select one and only one camera sensor!");
		return;
	}

	ccCameraSensor* sensor = ccHObjectCaster::ToCameraSensor(m_selectedEntities[0]);
	if (!sensor)
		return;

	//we need a cloud to filter!
	ccHObject* defaultCloud = sensor->getParent() && sensor->getParent()->isA(CC_TYPES::POINT_CLOUD) ? sensor->getParent() : 0;
	ccPointCloud* pointCloud = askUserToSelectACloud(defaultCloud, "Select a cloud to filter:");
	if (!pointCloud)
	{
		return;
	}

	//comupte/get the point cloud's octree
	ccOctree* octree = pointCloud->getOctree();
	if (!octree)
	{
		octree = pointCloud->computeOctree();
		if (!octree)
		{
			ccConsole::Error("Failed to compute the octree!");
			return;
		}
	}
	assert(octree);

	// filter octree then project the points
	std::vector<unsigned> inCameraFrustrum;
	if (!octree->intersectWithFrustrum(sensor,inCameraFrustrum))
	{
		ccConsole::Error("Failed to intersect sensor frustrum with octree!");
	}
	else
	{
		// scalar field
		const char sfName[] = "Frustrum visibility";
		int sfIdx = pointCloud->getScalarFieldIndexByName(sfName);

		if (inCameraFrustrum.empty())
		{
			ccConsole::Error("No point fell inside the frustrum!");
			if (sfIdx >= 0)
				pointCloud->deleteScalarField(sfIdx);
		}
		else
		{
			if (sfIdx < 0)
				sfIdx = pointCloud->addScalarField(sfName);
			if (sfIdx < 0)
			{
				ccLog::Error("Failed to allocate memory for output scalar field!");
				return;
			}

			CCLib::ScalarField* sf = pointCloud->getScalarField(sfIdx);
			assert(sf);
			if (sf)
			{
				sf->fill(0);

				const ScalarType c_insideValue = static_cast<ScalarType>(1);

				for (size_t i=0; i<inCameraFrustrum.size(); i++)
				{
					sf->setValue(inCameraFrustrum[i], c_insideValue);
				}

				sf->computeMinAndMax();
				pointCloud->setCurrentDisplayedScalarField(sfIdx);
				pointCloud->showSF(true);

				pointCloud->redrawDisplay();
			}
		}
	}

	updateUI();
}

void MainWindow::doActionShowDepthBuffer()
{
	if (m_selectedEntities.empty())
		return;

	size_t selNum = m_selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = m_selectedEntities[i];
		if (ent->isKindOf(CC_TYPES::GBL_SENSOR))
		{
			ccGBLSensor* sensor = static_cast<ccGBLSensor*>(m_selectedEntities[0]);
			if (sensor->getDepthBuffer().zBuff.empty())
			{
				//look for depending cloud
				ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent->getParent());
				if (cloud)
				{
					//force depth buffer computation
					int errorCode;
					if (!sensor->computeDepthBuffer(cloud,errorCode))
					{
						ccConsole::Error(ccGBLSensor::GetErrorString(errorCode));
					}
				}
				else
				{
					ccConsole::Error(QString("Internal error: sensor ('%1') parent is not a point cloud!").arg(sensor->getName()));
					return;
				}
			}

			ccRenderingTools::ShowDepthBuffer(sensor,this);
		}
	}
}

void MainWindow::doActionExportDepthBuffer()
{
	if (m_selectedEntities.empty())
		return;

	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::SaveFile());
	QString currentPath = settings.value(ccPS::CurrentPath(),QApplication::applicationDirPath()).toString();

	QString filename = QFileDialog::getSaveFileName(this,"Select output file",currentPath,DepthMapFileFilter::GetFileFilter());
	if (filename.isEmpty())
	{
		//process cancelled by user
		return;
	}

	//save last saving location
	settings.setValue(ccPS::CurrentPath(),QFileInfo(filename).absolutePath());
	settings.endGroup();

	ccHObject* toSave = 0;
	bool multEntities = false;
	if (m_selectedEntities.size() == 1)
	{
		toSave = m_selectedEntities.front();
	}
	else
	{
		toSave = new ccHObject("Temp Group");
		for (size_t i=0; i<m_selectedEntities.size(); ++i)
			toSave->addChild(m_selectedEntities[i],ccHObject::DP_NONE);
		multEntities = true;
	}

	DepthMapFileFilter::SaveParameters parameters;
	{
		parameters.alwaysDisplaySaveDialog = true;
	}
	CC_FILE_ERROR result = DepthMapFileFilter().saveToFile(toSave,filename,parameters);

	if (result != CC_FERR_NO_ERROR)
	{
		FileIOFilter::DisplayErrorMessage(result,"saving",filename);
	}
	else
	{
		ccLog::Print(QString("[I/O] File '%1' saved successfully").arg(filename));
	}

	if (multEntities)
	{
		delete toSave;
		toSave = 0;
	}
}

void MainWindow::doActionComputePointsVisibility()
{
	//there should be only one camera sensor in the current selection!
	if (m_selectedEntities.size() != 1 || !m_selectedEntities[0]->isKindOf(CC_TYPES::GBL_SENSOR))
	{
		ccConsole::Error("Select one and only one GBL/TLS sensor!");
		return;
	}

	ccGBLSensor* sensor = ccHObjectCaster::ToGBLSensor(m_selectedEntities[0]);
	if (!sensor)
		return;

	//we need a cloud to filter!
	ccHObject* defaultCloud = sensor->getParent() && sensor->getParent()->isA(CC_TYPES::POINT_CLOUD) ? sensor->getParent() : 0;
	ccPointCloud* pointCloud = askUserToSelectACloud(defaultCloud, "Select a cloud to filter:");
	if (!pointCloud)
	{
		return;
	}

	if (sensor->getDepthBuffer().zBuff.empty())
	{
		if (defaultCloud)
		{
			//the sensor has no depth buffer, we'll ask the user if he wants to compute it first
			if (QMessageBox::warning(	this,
										"Depth buffer.",
										"Sensor has no depth buffer: do you want to compute it now?",
										QMessageBox::Yes | QMessageBox::No,
										QMessageBox::Yes ) == QMessageBox::No)
			{
				//we can stop then...
				return;
			}
		
			int errorCode;
			if (sensor->computeDepthBuffer(static_cast<ccPointCloud*>(defaultCloud),errorCode))
			{
				ccRenderingTools::ShowDepthBuffer(sensor,this);
			}
			else
			{
				ccConsole::Error(ccGBLSensor::GetErrorString(errorCode));
				return;
			}
		}
		else
		{
			ccConsole::Error("Sensor has no depth buffer (and no associated cloud?)");
			return;
		}
	}

	// scalar field
	const char sfName[] = "Sensor visibility";
	int sfIdx = pointCloud->getScalarFieldIndexByName(sfName);
	if (sfIdx < 0)
		sfIdx = pointCloud->addScalarField(sfName);
	if (sfIdx < 0)
	{
		ccLog::Error("Failed to allocate memory for output scalar field!");
		return;
	}

	CCLib::ScalarField* sf = pointCloud->getScalarField(sfIdx);
	assert(sf);
	if (sf)
	{
		sf->fill(0);

		//progress bar
		ccProgressDialog pdlg(true);
		CCLib::NormalizedProgress nprogress(&pdlg,pointCloud->size());
		pdlg.setMethodTitle("Compute visibility");
		pdlg.setInfo(qPrintable(QString("Points: %1").arg(pointCloud->size())));
		pdlg.start();
		QApplication::processEvents();

		for (unsigned i=0; i<pointCloud->size(); i++)
		{
			const CCVector3* P = pointCloud->getPoint(i);
			unsigned char visibility = sensor->checkVisibility(*P);
			ScalarType visValue = static_cast<ScalarType>(visibility);

			sf->setValue(i, visValue);

			if (!nprogress.oneStep())
			{
				//cancelled by user
				pointCloud->deleteScalarField(sfIdx);
				sf = 0;
				break;
			}
		}
		
		if (sf)
		{
			sf->computeMinAndMax();
			pointCloud->setCurrentDisplayedScalarField(sfIdx);
			pointCloud->showSF(true);

			ccConsole::Print(QString("Visibility computed for cloud '%1'").arg(pointCloud->getName()));
			ccConsole::Print(QString("\tVisible = %1").arg(POINT_VISIBLE));
			ccConsole::Print(QString("\tHidden = %1").arg(POINT_HIDDEN));
			ccConsole::Print(QString("\tOut of range = %1").arg(POINT_OUT_OF_RANGE));
			ccConsole::Print(QString("\tOut of fov = %1").arg(POINT_OUT_OF_FOV));
		}
		pointCloud->redrawDisplay();
	}

	updateUI();
}

void MainWindow::doActionConvertTextureToColor()
{
	ccHObject::Container selectedEntities = m_selectedEntities;

	for (size_t i=0; i<selectedEntities.size(); ++i)
	{
		ccHObject* ent = selectedEntities[i];
		if (ent->isA(CC_TYPES::MESH)/*|| ent->isKindOf(CC_TYPES::PRIMITIVE)*/) //TODO
		{
			ccMesh* mesh = ccHObjectCaster::ToMesh(ent);
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


				//ColorCompType C[3]={MAX_COLOR_COMP,MAX_COLOR_COMP,MAX_COLOR_COMP};
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

void MainWindow::doActionSamplePoints()
{
	static unsigned s_ptsSamplingCount = 1000000;
	static double s_ptsSamplingDensity = 10.0;
	static bool s_ptsSampleNormals = true;
	static bool s_useDensity = false;
	
	ccPtsSamplingDlg dlg(this);
	//restore last parameters
	dlg.setPointsNumber(s_ptsSamplingCount);
	dlg.setDensityValue(s_ptsSamplingDensity);
	dlg.setGenerateNormals(s_ptsSampleNormals);
	dlg.setUseDensity(s_useDensity);
	if (!dlg.exec())
		return;

	ccProgressDialog pDlg(false,this);

	bool withNormals = dlg.generateNormals();
	bool withRGB = dlg.interpolateRGB();
	bool withTexture = dlg.interpolateTexture();
	bool useDensity = dlg.useDensity();
	assert(dlg.getPointsNumber() >= 0);
	s_ptsSamplingCount = static_cast<unsigned>(dlg.getPointsNumber());
	s_ptsSamplingDensity = dlg.getDensityValue();
	s_ptsSampleNormals = withNormals;
	s_useDensity = useDensity;

	bool errors = false;

	ccHObject::Container selectedEntities = m_selectedEntities;

	for (size_t i=0; i<selectedEntities.size() ;++i)
	{
		ccHObject* ent = selectedEntities[i];
		if (ent->isKindOf(CC_TYPES::MESH))
		{
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(ent);
			assert(mesh);

			ccPointCloud* cloud = mesh->samplePoints(	useDensity,
														useDensity ? s_ptsSamplingDensity : s_ptsSamplingCount,
														withNormals,
														withRGB,
														withTexture,
														&pDlg );

			if (cloud)
			{
				addToDB(cloud);
			}
			else
			{
				errors = true;
			}
		}
	}

	if (errors)
		ccLog::Error("[doActionSamplePoints] Errors occurred during the process! Result may be incomplete!");

	refreshAll();
}

void MainWindow::doRemoveDuplicatePoints()
{
	if (m_selectedEntities.empty())
		return;

	ccHObject::Container selectedEntities = m_selectedEntities;
	size_t selNum = selectedEntities.size();
	bool first = true;

	//persistent setting(s)
	QSettings settings;
	settings.beginGroup(ccPS::DuplicatePointsGroup());
	double minDistanceBetweenPoints = settings.value(ccPS::DuplicatePointsMinDist(),1.0e-12).toDouble();

	bool ok;
	minDistanceBetweenPoints = QInputDialog::getDouble(this, "Remove duplicate points", "Min distance between points:", minDistanceBetweenPoints, 0, 1.0e8, 12, &ok);
	if (!ok)
		return;

	//save parameter
	settings.setValue(ccPS::DuplicatePointsMinDist(),minDistanceBetweenPoints);

	static const char DEFAULT_DUPLICATE_TEMP_SF_NAME[] = "DuplicateFlags";

	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = selectedEntities[i];

		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
		if (cloud)
		{
			//create temporary SF for 'duplicate flags'
			int sfIdx = cloud->getScalarFieldIndexByName(DEFAULT_DUPLICATE_TEMP_SF_NAME);
			if (sfIdx < 0)
				sfIdx = cloud->addScalarField(DEFAULT_DUPLICATE_TEMP_SF_NAME);
			if (sfIdx >= 0)
				cloud->setCurrentScalarField(sfIdx);
			else
			{
				ccConsole::Error("Couldn't create temporary scalar field! Not enough memory?");
				break;
			}

			ccOctree* octree = cloud->getOctree();
			ccProgressDialog pDlg(true,this);

			int result = CCLib::GeometricalAnalysisTools::flagDuplicatePoints(cloud,minDistanceBetweenPoints,&pDlg,octree);

			if (result >= 0)
			{
				//count the number of duplicate points!
				CCLib::ScalarField* flagSF = cloud->getScalarField(sfIdx);
				unsigned duplicateCount = 0;
				assert(flagSF);
				if (flagSF)
				{
					for (unsigned j=0; j<flagSF->currentSize(); ++j)
						if (flagSF->getValue(j) != 0)
							++duplicateCount;
				}

				if (duplicateCount == 0)
				{
					ccConsole::Print(QString("Cloud '%1' has no duplicate points").arg(cloud->getName()));
				}
				else
				{
					ccConsole::Warning(QString("Cloud '%1' has %2 duplicate point(s)").arg(cloud->getName()).arg(duplicateCount));

					ccPointCloud* filteredCloud = cloud->filterPointsByScalarValue(0,0);
					if (filteredCloud)
					{
						int sfIdx2 = filteredCloud->getScalarFieldIndexByName(DEFAULT_DUPLICATE_TEMP_SF_NAME);
						assert(sfIdx2 >= 0);
						filteredCloud->deleteScalarField(sfIdx2);
						filteredCloud->setName(QString("%1.clean").arg(cloud->getName()));
						filteredCloud->setDisplay(cloud->getDisplay());
						filteredCloud->prepareDisplayForRefresh();
						addToDB(filteredCloud);
						if (first)
						{
							m_ccRoot->unselectAllEntities();
							first = false;
						}
						cloud->setEnabled(false);
						m_ccRoot->selectEntity(filteredCloud,true);
					}
				}
			}
			else
			{
				ccConsole::Error("An error occurred! (Not enough memory?)");
			}

			cloud->deleteScalarField(sfIdx);
		}
	}

	if (!first)
		ccConsole::Warning("Previously selected entities (sources) have been hidden!");

	refreshAll();
}

void MainWindow::doActionFilterByValue()
{
	ccHObject::Container selectedEntities = m_selectedEntities;
	size_t selNum = selectedEntities.size();

	typedef std::pair<ccHObject*,ccPointCloud*> entityAndVerticesType;
	std::vector<entityAndVerticesType> toFilter;
	{
		for (size_t i=0; i<selNum; ++i)
		{
			ccGenericPointCloud* cloud = 0;
			ccHObject* ent = selectedEntities[i];

			cloud = ccHObjectCaster::ToGenericPointCloud(ent);
			if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD)) // TODO
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
	}

	double minVald = 0.0;
	double maxVald = 1.0;

	if (toFilter.empty())
		return;

	//compute min and max "displayed" scalar values of currently selected
	//entities (the ones with an active scalar field only!)
	{
		for (size_t i=0; i<toFilter.size(); ++i)
		{
			ccScalarField* sf = toFilter[i].second->getCurrentDisplayedScalarField();
			assert(sf);

			if (i == 0)
			{
				minVald = static_cast<double>(sf->displayRange().start());
				maxVald = static_cast<double>(sf->displayRange().stop());
			}
			else
			{
				if (minVald > static_cast<double>(sf->displayRange().start()))
					minVald = static_cast<double>(sf->displayRange().start());
				if (maxVald < static_cast<double>(sf->displayRange().stop()))
					maxVald = static_cast<double>(sf->displayRange().stop());
			}
		}
	}

	ccAskTwoDoubleValuesDlg dlg("Min","Max",-1.0e9,1.0e9,minVald,maxVald,8,"Filter by scalar value",this);
	if (!dlg.exec())
		return;

	ScalarType minVal = (ScalarType)dlg.doubleSpinBox1->value();
	ScalarType maxVal = (ScalarType)dlg.doubleSpinBox2->value();

	ccHObject* firstResult = 0;
	{
		for (size_t i=0; i<toFilter.size(); ++i)
		{
			ccHObject* ent = toFilter[i].first;
			ccPointCloud* pc = toFilter[i].second;
			//CCLib::ScalarField* sf = pc->getCurrentDisplayedScalarField();
			//assert(sf);

			//we set as output (OUT) the currently displayed scalar field
			int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
			assert(outSfIdx >= 0);
			pc->setCurrentOutScalarField(outSfIdx);
			//pc->setCurrentScalarField(outSfIdx);

			ccHObject* result = 0;
			if (ent->isKindOf(CC_TYPES::MESH))
			{
				pc->hidePointsByScalarValue(minVal,maxVal);
				if (ent->isA(CC_TYPES::MESH)/*|| ent->isKindOf(CC_TYPES::PRIMITIVE)*/) //TODO
					result = ccHObjectCaster::ToMesh(ent)->createNewMeshFromSelection(false);
				else if (ent->isA(CC_TYPES::SUB_MESH))
					result = ccHObjectCaster::ToSubMesh(ent)->createNewSubMeshFromSelection(false);
				pc->unallocateVisibilityArray();
			}
			else if (ent->isKindOf(CC_TYPES::POINT_CLOUD))
			{
				//pc->hidePointsByScalarValue(minVal,maxVal);
				//result = ccHObjectCaster::ToGenericPointCloud(ent)->hidePointsByScalarValue(false);
				//pc->unallocateVisibilityArray();

				//shortcut, as we know here that the point cloud is a "ccPointCloud"
				result = pc->filterPointsByScalarValue(minVal,maxVal);
			}

			if (result)
			{
				ent->setEnabled(false);
				result->setDisplay(ent->getDisplay());
				result->prepareDisplayForRefresh();
				addToDB(result);

				if (!firstResult)
					firstResult = result;
			}
			//*/
		}
	}

	if (firstResult)
	{
		ccConsole::Warning("Previously selected entities (sources) have been hidden!");
		if (m_ccRoot)
			m_ccRoot->selectEntity(firstResult);
	}

	refreshAll();
}

static int s_randomColorsNumber = 256;
void MainWindow::doActionSFConvertToRandomRGB()
{
	bool ok;
	s_randomColorsNumber = QInputDialog::getInt(this, "Random colors", "Number of random colors (will be regularly sampled over the SF interval):", s_randomColorsNumber, 2, 2147483647, 16, &ok);
	if (!ok)
		return;
	assert(s_randomColorsNumber > 1);

	ColorsTableType* randomColors = new ColorsTableType;
	if (!randomColors->reserve(static_cast<unsigned>(s_randomColorsNumber)))
	{
		ccConsole::Error("Not enough memory!");
		return;
	}

	//generate random colors
	{
		for (int i=0; i<s_randomColorsNumber; ++i)
		{
			ccColor::Rgb col = ccColor::Generator::Random();
			randomColors->addElement(col.rgb);
		}
	}

	//apply random colors
	size_t selNum = m_selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		ccGenericPointCloud* cloud = 0;
		ccHObject* ent = m_selectedEntities[i];

		bool lockedVertices;
		cloud = ccHObjectCaster::ToPointCloud(ent,&lockedVertices);
		if (lockedVertices)
		{
			DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
			continue;
		}
		if (cloud) //TODO
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
			ccScalarField* sf = pc->getCurrentDisplayedScalarField();
			//if there is no displayed SF --> nothing to do!
			if (sf && sf->currentSize() >= pc->size())
			{
				if (!pc->resizeTheRGBTable(false))
				{
					ccConsole::Error("Not enough memory!");
					break;
				}
				else
				{
					ScalarType minSF = sf->getMin();
					ScalarType maxSF = sf->getMax();

					ScalarType step = (maxSF-minSF)/(s_randomColorsNumber-1);
					if (step == 0)
						step = static_cast<ScalarType>(1.0);

					for (unsigned i=0; i<pc->size(); ++i)
					{
						ScalarType val = sf->getValue(i);
						unsigned colIndex = static_cast<unsigned>((val-minSF)/step);
						if (colIndex == s_randomColorsNumber)
							--colIndex;

						pc->setPointColor(i,randomColors->getValue(colIndex));
					}

					pc->showColors(true);
					pc->showSF(false);
				}
			}

			cloud->prepareDisplayForRefresh_recursive();
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionSFConvertToRGB()
{
	//we first ask the user if the SF colors should be mixed with existing colors
	bool mixWithExistingColors = false;
	{
		QMessageBox::StandardButton answer = QMessageBox::warning(	this,
																	"Scalar Field to RGB",
																	"Mix with existing colors (if any)?",
																	QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel,
																	QMessageBox::Yes );
		if (answer == QMessageBox::Yes)
			mixWithExistingColors = true;
		else if (answer == QMessageBox::Cancel)
			return;
	}

	size_t selNum = m_selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		ccGenericPointCloud* cloud = 0;
		ccHObject* ent = m_selectedEntities[i];

		bool lockedVertices;
		cloud = ccHObjectCaster::ToPointCloud(ent,&lockedVertices);
		if (lockedVertices)
		{
			DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
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
	if (selNum != 1)
	{
		if (selNum > 1)
			ccConsole::Error("Select only one cloud or one mesh!");
		return;
	}
	ccHObject* ent = m_selectedEntities[0];

	bool lockedVertices;
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent,&lockedVertices);

	//for "real" point clouds only
	if (!cloud)
		return;
	if (lockedVertices && !ent->isAncestorOf(cloud))
	{
		//see ccPropertiesTreeDelegate::fillWithMesh
		DisplayLockedVerticesWarning(ent->getName(),true);
		return;
	}

	assert(cloud);
	int sfIdx = cloud->getCurrentDisplayedScalarFieldIndex();
	switch (action)
	{
		case 0: //Toggle SF color scale
			if (sfIdx >= 0)
			{
				cloud->showSFColorsScale(!cloud->sfColorScaleShown());
				cloud->prepareDisplayForRefresh();
			}
			else
				ccConsole::Warning(QString("No active scalar field on entity '%1'").arg(ent->getName()));
			break;
		case 1: //Activate previous SF
			if (sfIdx >= 0)
			{
				cloud->setCurrentDisplayedScalarField(sfIdx-1);
				cloud->prepareDisplayForRefresh();
			}
			break;
		case 2: //Activate next SF
			if (sfIdx+1 < static_cast<int>(cloud->getNumberOfScalarFields()))
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
	size_t selNum = m_selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
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
				bool ok;
				QString newName = QInputDialog::getText(this,"SF name","name:",QLineEdit::Normal, QString(sfName ? sfName : "unknown"), &ok);
				if (ok)
					sf->setName(qPrintable(newName));
			}
		}
	}

	updateUI();
}

void MainWindow::doActionOpenColorScalesManager()
{
	ccColorScaleEditorDialog cseDlg(ccColorScalesManager::GetUniqueInstance(), this, ccColorScale::Shared(0), this);

	if (cseDlg.exec())
	{
		//save current scale manager state to persistent settings
		ccColorScalesManager::GetUniqueInstance()->toPersistentSettings();
	}

	updateUI();
}

void MainWindow::doActionAddIdField()
{
	size_t selNum = m_selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_selectedEntities[i]);
		if (cloud) //TODO
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

			int sfIdx = pc->getScalarFieldIndexByName(CC_DEFAULT_ID_SF_NAME);
			if (sfIdx < 0)
				sfIdx = pc->addScalarField(CC_DEFAULT_ID_SF_NAME);
			if (sfIdx < 0)
			{
				ccLog::Warning("Not enough memory!");
				return;
			}

			CCLib::ScalarField* sf = pc->getScalarField(sfIdx);
			assert(sf->currentSize() == pc->size());

			for (unsigned j=0 ; j<cloud->size(); j++)
			{
				ScalarType idValue = static_cast<ScalarType>(j);
				sf->setValue(j, idValue);
			}

			sf->computeMinAndMax();
			pc->setCurrentDisplayedScalarField(sfIdx);
			pc->showSF(true);
			pc->prepareDisplayForRefresh();
		}
	}


	refreshAll();
	updateUI();
}

PointCoordinateType MainWindow::GetDefaultCloudKernelSize(ccGenericPointCloud* cloud, unsigned knn/*=12*/)
{
	assert(cloud);
	if (cloud && cloud->size() != 0)
	{
		//we get 1% of the cloud bounding box
		//and we divide by the number of points / 10e6 (so that the kernel for a 20 M. points cloud is half the one of a 10 M. cloud)
		ccBBox box = cloud->getOwnBB();

		//old way
		//PointCoordinateType radius = box.getDiagNorm() * static_cast<PointCoordinateType>(0.01/std::max(1.0,1.0e-7*static_cast<double>(cloud->size())));

		//new way
		CCVector3 d = box.getDiagVec();
		PointCoordinateType volume = d[0] * d[1] * d[2];
		PointCoordinateType surface = pow(volume, static_cast<PointCoordinateType>(2.0/3.0));
		PointCoordinateType surfacePerPoint = surface / cloud->size();
		return sqrt(surfacePerPoint * knn);
	}

	return -PC_ONE;
}

PointCoordinateType MainWindow::GetDefaultCloudKernelSize(const ccHObject::Container& entities, unsigned knn/*=12*/)
{
	PointCoordinateType sigma = -PC_ONE;

	size_t selNum = entities.size();
	//computation of a first sigma guess
	for (size_t i=0; i<selNum; ++i)
	{
		ccPointCloud* pc = ccHObjectCaster::ToPointCloud(entities[i]);
		PointCoordinateType sigmaCloud = GetDefaultCloudKernelSize(pc);

		//we keep the smallest value
		if (sigma < 0 || sigmaCloud < sigma)
			sigma = sigmaCloud;
	}

	return sigma;
}

void MainWindow::doActionSFGaussianFilter()
{
	size_t selNum = m_selectedEntities.size();
	if (selNum == 0)
		return;

	double sigma = GetDefaultCloudKernelSize(m_selectedEntities);
	if (sigma < 0.0)
	{
		ccConsole::Error("No eligible point cloud in selection!");
		return;
	}

	bool ok;
	sigma = QInputDialog::getDouble(this,"Gaussian filter","sigma:",sigma,DBL_MIN,1.0e9,8,&ok);
	if (!ok)
		return;

	for (size_t i=0; i<selNum; ++i)
	{
		bool lockedVertices;
		ccHObject* ent = m_selectedEntities[i];
		ccPointCloud* pc = ccHObjectCaster::ToPointCloud(ent,&lockedVertices);
		if (!pc || lockedVertices)
		{
			DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
			continue;
		}

		//la methode est activee sur le champ scalaire affiche
		CCLib::ScalarField* sf = pc->getCurrentDisplayedScalarField();
		if (sf)
		{
			//on met en lecture (OUT) le champ scalaire actuellement affiche
			int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
			assert(outSfIdx >= 0);

			pc->setCurrentOutScalarField(outSfIdx);
			CCLib::ScalarField* outSF = pc->getCurrentOutScalarField();
			assert(sf);

			QString sfName = QString("%1.smooth(%2)").arg(outSF->getName()).arg(sigma);
			int sfIdx = pc->getScalarFieldIndexByName(qPrintable(sfName));
			if (sfIdx < 0)
				sfIdx = pc->addScalarField(qPrintable(sfName)); //output SF has same type as input SF
			if (sfIdx >= 0)
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
				CCLib::ScalarFieldTools::applyScalarFieldGaussianFilter(static_cast<PointCoordinateType>(sigma),
																		pc,
																		-1,
																		&pDlg,
																		octree);
				ccConsole::Print("[GaussianFilter] Timing: %3.2f s.",static_cast<double>(eTimer.elapsed())/1.0e3);
				pc->setCurrentDisplayedScalarField(sfIdx);
				pc->showSF(sfIdx >= 0);
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
	size_t selNum = m_selectedEntities.size();
	if (selNum == 0)
		return;

	double sigma = GetDefaultCloudKernelSize(m_selectedEntities);
	if (sigma < 0.0)
	{
		ccConsole::Error("No eligible point cloud in selection!");
		return;
	}

	//estimate a good value for scalar field sigma, based on the first cloud
	//and its displayed scalar field
	ccPointCloud* pc_test = ccHObjectCaster::ToPointCloud(m_selectedEntities[0]);
	CCLib::ScalarField* sf_test = pc_test->getCurrentDisplayedScalarField();
	ScalarType range = sf_test->getMax() - sf_test->getMin();
	double scalarFieldSigma = range / 4; // using 1/4 of total range


	ccAskTwoDoubleValuesDlg dlg("Spatial sigma", "Scalar sigma", DBL_MIN, 1.0e9, sigma, scalarFieldSigma , 8, 0, this);
	dlg.doubleSpinBox1->setStatusTip("3*sigma = 98% attenuation");
	dlg.doubleSpinBox2->setStatusTip("Scalar field's sigma controls how much the filter behaves as a Gaussian Filter\n sigma at +inf uses the whole range of scalars ");
	if (!dlg.exec())
		return;

	//get values
	sigma = dlg.doubleSpinBox1->value();
	scalarFieldSigma = dlg.doubleSpinBox2->value();

	for (size_t i=0; i<selNum; ++i)
	{
		bool lockedVertices;
		ccHObject* ent = m_selectedEntities[i];
		ccPointCloud* pc = ccHObjectCaster::ToPointCloud(ent,&lockedVertices);
		if (!pc || lockedVertices)
		{
			DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
			continue;
		}

		//the algorithm will use the currently displayed SF
		CCLib::ScalarField* sf = pc->getCurrentDisplayedScalarField();
		if (sf)
		{
			//we set the displayed SF as "OUT" SF
			int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
			assert(outSfIdx >= 0);

			pc->setCurrentOutScalarField(outSfIdx);
			CCLib::ScalarField* outSF = pc->getCurrentOutScalarField();
			assert(sf);

			QString sfName = QString("%1.bilsmooth(%2,%3)").arg(outSF->getName()).arg(sigma).arg(scalarFieldSigma);
			int sfIdx = pc->getScalarFieldIndexByName(qPrintable(sfName));
			if (sfIdx < 0)
				sfIdx = pc->addScalarField(qPrintable(sfName)); //output SF has same type as input SF
			if (sfIdx >= 0)
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

				CCLib::ScalarFieldTools::applyScalarFieldGaussianFilter(static_cast<PointCoordinateType>(sigma),
																		pc,
																		static_cast<PointCoordinateType>(scalarFieldSigma),
																		&pDlg,
																		octree);
				ccConsole::Print("[BilateralFilter] Timing: %3.2f s.",eTimer.elapsed()/1.0e3);
				pc->setCurrentDisplayedScalarField(sfIdx);
				pc->showSF(sfIdx >= 0);
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
	doMeshSFAction(ccMesh::SMOOTH_MESH_SF);
}

void MainWindow::doActionEnhanceMeshSF()
{
	doMeshSFAction(ccMesh::ENHANCE_MESH_SF);
}

void MainWindow::doMeshSFAction(ccMesh::MESH_SCALAR_FIELD_PROCESS process)
{
	ccProgressDialog pDlg(false,this);

	size_t selNum = m_selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = m_selectedEntities[i];
		if (ent->isKindOf(CC_TYPES::MESH) || ent->isKindOf(CC_TYPES::PRIMITIVE)) //TODO
		{
			ccMesh* mesh = ccHObjectCaster::ToMesh(ent);
			if (mesh)
			{
				ccGenericPointCloud* cloud = mesh->getAssociatedCloud();

				if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD)) //TODO
				{
					ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

					//on active le champ scalaire actuellement affiche
					int sfIdx = pc->getCurrentDisplayedScalarFieldIndex();
					if (sfIdx >= 0)
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
	}

	refreshAll();
	updateUI();
}

static double s_subdivideMaxArea = 1.0;
void MainWindow::doActionSubdivideMesh()
{
	bool ok;
	s_subdivideMaxArea = QInputDialog::getDouble(this, "Subdivide mesh", "Max area per triangle:", s_subdivideMaxArea, 1e-6, 1e6, 8, &ok);
	if (!ok)
		return;

	//ccProgressDialog pDlg(true,this);

	size_t selNum = m_selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = m_selectedEntities[i];
		if (ent->isKindOf(CC_TYPES::MESH))
		{
			//single mesh?
			if (ent->isA(CC_TYPES::MESH))
			{
				ccMesh* mesh = static_cast<ccMesh*>(ent);

				ccMesh* subdividedMesh = 0;
				try
				{
					subdividedMesh = mesh->subdivide(static_cast<PointCoordinateType>(s_subdivideMaxArea));
				}
				catch(...)
				{
					ccLog::Error(QString("[Subdivide] An error occurred while trying to subdivide mesh '%1' (not enough memory?)").arg(mesh->getName()));
				}

				if (subdividedMesh)
				{
					subdividedMesh->setName(QString("%1.subdivided(S<%2)").arg(mesh->getName()).arg(s_subdivideMaxArea));
					subdividedMesh->setDisplay(mesh->getDisplay());
					mesh->refreshDisplay_recursive();
					mesh->setEnabled(false);
					addToDB(subdividedMesh);
				}
				else
				{
					ccConsole::Warning(QString("[Subdivide] Failed to subdivide mesh '%1' (not enough memory?)").arg(mesh->getName()));
				}
			}
			else
			{
				ccLog::Warning("[Subdivide] Works only on real meshes!");
			}
		}
	}

	refreshAll();
	updateUI();
}

static unsigned	s_laplacianSmooth_nbIter = 20;
static double	s_laplacianSmooth_factor = 0.2;
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

	size_t selNum = m_selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = m_selectedEntities[i];
		if (ent->isA(CC_TYPES::MESH) || ent->isA(CC_TYPES::PRIMITIVE)) //FIXME: can we really do this with primitives?
		{
			ccMesh* mesh = ccHObjectCaster::ToMesh(ent);

			if (mesh->laplacianSmooth(	s_laplacianSmooth_nbIter,
										static_cast<PointCoordinateType>(s_laplacianSmooth_factor),
										&pDlg) )
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
	size_t count = ccObjects.size();
	keptObjects.reserve(count);

	for (size_t i=0; i<count; ++i)
	{
		ccHObject* obj = ccObjects[i];
		assert(obj);
		for (size_t j=0; j<count; ++j)
		{
			if (i != j)
			{
				if (ccObjects[j]->isAncestorOf(obj))
				{
					obj = 0;
					break;
				}
			}
		}

		if (obj)
			keptObjects.push_back(obj);
		else
			ccObjects[i] = 0;
	}

	ccObjects = keptObjects;
}

// helper for doActionMerge
void AddToRemoveList(ccHObject* toRemove, ccHObject::Container& toBeRemovedList)
{
	//is a parent or sibling already in the "toBeRemoved" list?
	int j = 0;
	int count = static_cast<int>(toBeRemovedList.size());
	while (j < count)
	{
		if (toBeRemovedList[j]->isAncestorOf(toRemove))
		{
			toRemove = 0;
			break;
		}
		else if (toRemove->isAncestorOf(toBeRemovedList[j]))
		{
			toBeRemovedList[j] = toBeRemovedList.back();
			toBeRemovedList.pop_back();
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
		toBeRemovedList.push_back(toRemove);
}

void MainWindow::doActionMerge()
{
	//let's look for clouds or meshes (warning: we don't mix them)
	std::vector<ccPointCloud*> clouds;
	std::vector<ccMesh*> meshes;

	try
	{
		size_t selNum = m_selectedEntities.size();
		for (size_t i=0; i<selNum; ++i)
		{
			ccHObject* ent = m_selectedEntities[i];
			if (!ent)
				continue;

			if (ent->isA(CC_TYPES::POINT_CLOUD))
			{
				ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
				clouds.push_back(cloud);
			}
			else if (ent->isKindOf(CC_TYPES::MESH))
			{
				ccMesh* mesh = ccHObjectCaster::ToMesh(ent);
				//this is a purely theoretical test for now!
				if (mesh && mesh->getAssociatedCloud() && mesh->getAssociatedCloud()->isA(CC_TYPES::POINT_CLOUD))
				{
					meshes.push_back(mesh);
				}
				else
				{
					ccConsole::Warning(QString("Only meshes with standard vertices are handled for now! Can't merge entity '%1'...").arg(ent->getName()));
				}
			}
			else
			{
				ccConsole::Warning(QString("Entity '%1' is neither a cloud nor a mesh, can't merge it!").arg(ent->getName()));
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("Not enough memory!");
		return;
	}

	if (clouds.empty() && meshes.empty())
	{
		ccLog::Error("Select only clouds or meshes!");
		return;
	}
	if (!clouds.empty() && !meshes.empty())
	{
		ccLog::Error("Can't mix point clouds and meshes!");
	}

	//merge clouds?
	if (!clouds.empty())
	{
		//we deselect all selected entities (as most of them are going to disappear)
		if (m_ccRoot)
		{
			m_ccRoot->unselectAllEntities();
			assert(m_selectedEntities.empty());
			//m_selectedEntities.clear();
		}

		//we will remove the useless clouds/meshes later
		ccHObject::Container toBeRemoved;

		ccPointCloud* firstCloud = 0;
		ccHObjectContext firstCloudContext;

		for (size_t i=0; i<clouds.size(); ++i)
		{
			ccPointCloud* pc = clouds[i];
			if (!firstCloud)
			{
				//we don't delete the first cloud (we'll merge the other one 'inside' it
				firstCloud = pc;
				//we still have to temporarily detach the first cloud, as it may undergo
				//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::operator +=
				firstCloudContext = removeObjectTemporarilyFromDBTree(firstCloud);
			}
			else
			{
				unsigned countBefore = firstCloud->size();
				unsigned countAdded = pc->size();
				*firstCloud += pc;

				//success?
				if (firstCloud->size() == countBefore + countAdded)
				{
					firstCloud->prepareDisplayForRefresh_recursive();

					ccHObject* toRemove = 0;
					//if the entity to remove is a group with a unique child, we can remove it as well
					ccHObject* parent = pc->getParent();
					if (parent && parent->isA(CC_TYPES::HIERARCHY_OBJECT) && parent->getChildrenNumber() == 1 && parent != firstCloudContext.parent)
						toRemove = parent;
					else
						toRemove = pc;

					AddToRemoveList(toRemove, toBeRemoved);
				}
				else
				{
					ccConsole::Error("Fusion failed! (not enough memory?)");
					break;
				}
				pc = 0;
			}
		}

		//something to remove?
		while (!toBeRemoved.empty())
		{
			if (toBeRemoved.back() && m_ccRoot)
			{
				m_ccRoot->removeElement(toBeRemoved.back());
			}
			toBeRemoved.pop_back();
		}

		//put back first cloud in DB
		if (firstCloud)
		{
			putObjectBackIntoDBTree(firstCloud,firstCloudContext);
			if (m_ccRoot)
				m_ccRoot->selectEntity(firstCloud);
		}
	}
	//merge meshes?
	else if (!meshes.empty())
	{
		bool createSubMeshes = true;
		//createSubMeshes = (QMessageBox::question(this,"Create sub-meshes","Do you want to create sub-mesh entities corresponding to each source mesh? (requires more memory)",QMessageBox::Yes,QMessageBox::No) == QMessageBox::Yes);

		//meshes are merged
		ccPointCloud* baseVertices = new ccPointCloud("vertices");
		ccMesh* baseMesh = new ccMesh(baseVertices);
		baseMesh->setName("Merged mesh");
		baseMesh->addChild(baseVertices);
		baseVertices->setEnabled(false);
		for (size_t i=0; i<meshes.size(); ++i)
		{
			ccMesh* mesh = meshes[i];

			unsigned sizeBefore = baseMesh->size();
			if (!baseMesh->merge(mesh))
			{
				ccConsole::Error("Fusion failed! (not enough memory?)");
				break;
			}
			unsigned sizeAfter = baseMesh->size();

			//create corresponding sub-mesh
			if (createSubMeshes)
			{
				ccSubMesh* subMesh = new ccSubMesh(baseMesh);
				if (subMesh->reserve(sizeAfter-sizeBefore))
				{
					subMesh->addTriangleIndex(sizeBefore,sizeAfter);
					subMesh->setName(mesh->getName());
					subMesh->showMaterials(baseMesh->materialsShown());
					subMesh->showNormals(baseMesh->normalsShown());
					subMesh->showTriNorms(baseMesh->triNormsShown());
					subMesh->showColors(baseMesh->colorsShown());
					subMesh->showWired(baseMesh->isShownAsWire());
					subMesh->enableStippling(baseMesh->stipplingEnabled());
					subMesh->setEnabled(false);
					baseMesh->addChild(subMesh);
				}
				else
				{
					ccConsole::Warning(QString("[Merge] Not enough memory to create the sub-mesh corresponding to mesh '%1'!").arg(mesh->getName()));
					delete subMesh;
					subMesh = 0;
				}
			}
		}
		
		baseMesh->setDisplay_recursive(meshes.front()->getDisplay());
		baseMesh->setVisible(true);
		addToDB(baseMesh);

		if (m_ccRoot)
			m_ccRoot->selectEntity(baseMesh);
	}

	refreshAll();
	updateUI();
}

void MainWindow::zoomOn(ccHObject* object)
{
	ccGLWindow* win = static_cast<ccGLWindow*>(object->getDisplay());
	if (win)
	{
		ccBBox box = object->getDisplayBB_recursive(false,win);
		win->updateConstellationCenterAndZoom(&box);
	}
}

void MainWindow::doActionRegister()
{
	if (	m_selectedEntities.size() != 2
		||	(!m_selectedEntities[0]->isKindOf(CC_TYPES::POINT_CLOUD) && !m_selectedEntities[0]->isKindOf(CC_TYPES::MESH))
		||	(!m_selectedEntities[1]->isKindOf(CC_TYPES::POINT_CLOUD) && !m_selectedEntities[1]->isKindOf(CC_TYPES::MESH)) )
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

	double minRMSDecrease										= rDlg.getMinRMSDecrease();
	unsigned maxIterationCount									= rDlg.getMaxIterationCount();
	unsigned randomSamplingLimit								= rDlg.randomSamplingLimit();
	bool removeFarthestPoints									= rDlg.removeFarthestPoints();
	bool useDataSFAsWeights										= rDlg.useDataSFAsWeights();
	bool useModelSFAsWeights									= rDlg.useModelSFAsWeights();
	bool adjustScale											= rDlg.adjustScale();
	int transformationFilters									= rDlg.getTransformationFilters();
	unsigned finalOverlap										= rDlg.getFinalOverlap();
	CCLib::ICPRegistrationTools::CONVERGENCE_TYPE method		= rDlg.getConvergenceMethod();

	//semi-persistent storage (for next call)
	rDlg.saveParameters();

	ccGLMatrix transMat;
	double finalError = 0.0;
	double finalScale = 1.0;
	unsigned finalPointCount = 0;

	if (ccRegistrationTools::ICP(	data,
									model,
									transMat,
									finalScale,
									finalError,
									finalPointCount,
									minRMSDecrease,
									maxIterationCount,
									randomSamplingLimit,
									removeFarthestPoints,
									method,
									adjustScale,
									finalOverlap/100.0,
									useDataSFAsWeights,
									useModelSFAsWeights,
									transformationFilters,
									this))
	{
		QString rmsString = QString("Final RMS: %1 (computed on %2 points)").arg(finalError).arg(finalPointCount);
		ccLog::Print(QString("[Register] ") + rmsString);
		
		QStringList summary;
		summary << rmsString;
		summary << "----------------";

		//transformation matrix
		{
			QString matString = transMat.toString();
			summary << "Transformation matrix";
			summary << transMat.toString(3,'\t'); //low precision, just for display
			summary << "----------------";

			ccLog::Print("[Register] Applied transformation matrix:");
			ccLog::Print(transMat.toString(12,' ')); //full precision
			ccLog::Print("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool");
		}

		if (adjustScale)
		{
			QString scaleString = QString("Scale: %1 (already integrated in above matrix!)").arg(finalScale);
			ccLog::Warning(QString("[Register] ")+scaleString);
			summary << scaleString;
		}
		else
		{
			ccLog::Print(QString("[Register] Scale: fixed (1.0)"));
			summary << "Scale: fixed (1.0)";
		}

		//overlap
		summary << "----------------";
		QString overlapString = QString("Theorical overlap: %1%").arg(finalOverlap);
		ccLog::Print(QString("[Register] ")+overlapString);
		summary << overlapString;

		summary << "----------------";
		summary << "This report has been output to Console (F8)";

		//cloud to move
		ccGenericPointCloud* pc = 0;

		if (data->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			pc = ccHObjectCaster::ToGenericPointCloud(data);
		}
		else if (data->isKindOf(CC_TYPES::MESH))
		{
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(data);
			pc = mesh->getAssociatedCloud();

			//warning: point cloud is locked!
			if (pc->isLocked())
			{
				pc = 0;
				//we ask the user about cloning the 'data' mesh
				QMessageBox::StandardButton result = QMessageBox::question(	this,
																			"Registration",
																			"Data mesh vertices are locked (they may be shared with other meshes): Do you wish to clone this mesh to apply transformation?",
																			QMessageBox::Ok | QMessageBox::Cancel,
																			QMessageBox::Ok);

				//continue process?
				if (result == QMessageBox::Ok)
				{
					ccGenericMesh* newMesh = 0;
					if (mesh->isA(CC_TYPES::MESH))
						newMesh = static_cast<ccMesh*>(mesh)->cloneMesh();
					else
					{
						//FIXME TODO
						ccLog::Error("Doesn't work on sub-meshes yet!");
					}

					if (newMesh)
					{
						newMesh->setDisplay(data->getDisplay());
						addToDB(newMesh);
						data = newMesh;
						pc = newMesh->getAssociatedCloud();
					}
					else
					{
						ccLog::Error("Failed to clone 'data' mesh! (not enough memory?)");
					}
				}
			}
		}

		//if we managed to get a point cloud to move!
		if (pc)
		{
			//we temporarily detach cloud, as it may undergo
			//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::applyRigidTransformation
			ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(pc);
			pc->applyRigidTransformation(transMat);
			putObjectBackIntoDBTree(pc,objContext);

			//don't forget to update mesh bounding box also!
			if (data->isKindOf(CC_TYPES::MESH))
				ccHObjectCaster::ToGenericMesh(data)->refreshBB();

			//don't forget global shift
			ccGenericPointCloud* refPc = ccHObjectCaster::ToGenericPointCloud(model);
			if (refPc)
			{
				if (refPc->isShifted())
				{
					const CCVector3d& Pshift = refPc->getGlobalShift();
					const double& scale = refPc->getGlobalScale();
					pc->setGlobalShift(Pshift);
					pc->setGlobalScale(scale);
					ccLog::Warning(QString("[ICP] Aligned entity global shift has been updated to match the reference: (%1,%2,%3) [x%4]").arg(Pshift.x).arg(Pshift.y).arg(Pshift.z).arg(scale));
				}
				else if (pc->isShifted()) //we'll ask the user first before dropping the shift information on the aligned cloud
				{
					if (QMessageBox::question(this, "Drop shift information?", "Aligned entity is shifted but reference cloud is not: drop global shift information?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes)
					{
						pc->setGlobalShift(0,0,0);
						pc->setGlobalScale(1.0);
						ccLog::Warning(QString("[ICP] Aligned entity global shift has been reset to match the reference!"));
					}
				}
			}

			data->prepareDisplayForRefresh_recursive();
			data->setName(data->getName()+QString(".registered"));
			zoomOn(data);
		}

		//pop-up summary
		QMessageBox::information(this,"Register info",summary.join("\n"));
		forceConsoleDisplay();
	}

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

	if (!m_selectedEntities[0]->isKindOf(CC_TYPES::POINT_CLOUD) ||
		!m_selectedEntities[1]->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error("Select 2 point clouds!");
		return;
	}

	ccGenericPointCloud *model = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities[0]);
	ccGenericPointCloud *data = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities[1]);

	ccAlignDlg aDlg(model, data);
	if (!aDlg.exec())
		return;

	// model = aDlg.getModelObject();
	data = aDlg.getDataObject();

	//Take the correct number of points among the clouds
	CCLib::ReferenceCloud *subModel = aDlg.getSampledModel();
	CCLib::ReferenceCloud *subData = aDlg.getSampledData();

	unsigned nbMaxCandidates = aDlg.isNumberOfCandidatesLimited() ? aDlg.getMaxNumberOfCandidates() : 0;

	ccProgressDialog pDlg(true,this);

	CCLib::PointProjectionTools::Transformation transform;
	if (CCLib::FPCSRegistrationTools::RegisterClouds(	subModel,
														subData,
														transform,
														static_cast<ScalarType>(aDlg.getDelta()),
														static_cast<ScalarType>(aDlg.getDelta()/2),
														static_cast<PointCoordinateType>(aDlg.getOverlap()),
														aDlg.getNbTries(),
														5000,
														&pDlg,
														nbMaxCandidates))
	{
		//output resulting transformation matrix
		{
			ccGLMatrix transMat = FromCCLibMatrix<PointCoordinateType,float>(transform.R,transform.T);
			forceConsoleDisplay();
			ccConsole::Print("[Align] Resulting matrix:");
			ccConsole::Print(transMat.toString(12,' ')); //full precision
			ccConsole::Print("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool");
		}

		ccPointCloud *newDataCloud = data->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(data)->cloneThis() : ccPointCloud::From(data,data);

		if (data->getParent())
			data->getParent()->addChild(newDataCloud);
		newDataCloud->setName(data->getName()+QString(".registered"));
		newDataCloud->applyTransformation(transform);
		newDataCloud->setDisplay(data->getDisplay());
		newDataCloud->prepareDisplayForRefresh();
		zoomOn(newDataCloud);
		addToDB(newDataCloud);

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

void MainWindow::doActionSubsample()
{
	//find candidates
	std::vector<ccPointCloud*> clouds;
	unsigned maxPointCount = 0;
	double maxCloudRadius = 0;
	ScalarType sfMin = NAN_VALUE;
	ScalarType sfMax = NAN_VALUE;
	{
		for (size_t i=0; i<m_selectedEntities.size(); ++i)
		{
			if (m_selectedEntities[i]->isA(CC_TYPES::POINT_CLOUD))
			{
				ccPointCloud* cloud = static_cast<ccPointCloud*>(m_selectedEntities[i]);
				clouds.push_back(cloud);

				maxPointCount = std::max<unsigned>(maxPointCount, cloud->size());
				maxCloudRadius = std::max<double>(maxCloudRadius, cloud->getOwnBB().getDiagNorm());

				//we also look for the min and max sf values
				ccScalarField* sf = cloud->getCurrentDisplayedScalarField();
				if (sf)
				{
					if (!ccScalarField::ValidValue(sfMin) || sfMin > sf->getMin())
						sfMin = sf->getMin();
					if (!ccScalarField::ValidValue(sfMax) || sfMax < sf->getMax())
						sfMax = sf->getMax();
				}
			}
		}
	}

	if (clouds.empty())
	{
		ccConsole::Error("Select at least one point cloud!");
		return;
	}

	//Display dialog
	ccSubsamplingDlg sDlg(maxPointCount, maxCloudRadius, this);
	bool hasValidSF = ccScalarField::ValidValue(sfMin) && ccScalarField::ValidValue(sfMax);
	if (hasValidSF)
		sDlg.enableSFModulation(sfMin,sfMax);
	if (!sDlg.exec())
		return;

	//process clouds
	ccHObject::Container resultingClouds;
	{
		ccProgressDialog pDlg(false,this);
		pDlg.setMethodTitle("Subsampling");

		bool errors = false;

		QElapsedTimer eTimer;
		eTimer.start();

		for (size_t i=0; i<clouds.size(); ++i)
		{
			ccPointCloud* cloud = clouds[i];
			CCLib::ReferenceCloud *sampledCloud = sDlg.getSampledCloud(cloud,&pDlg);
			if (!sampledCloud)
			{
				ccConsole::Warning(QString("[Subsampling] Failed to subsample cloud '%1'!").arg(cloud->getName()));
				errors = true;
				continue;
			}

			int warnings = 0;
			ccPointCloud *newPointCloud = cloud->partialClone(sampledCloud,&warnings);
			
			delete sampledCloud;
			sampledCloud = 0;
			
			if (newPointCloud)
			{
				newPointCloud->setName(cloud->getName() + QString(".subsampled"));
				newPointCloud->setGlobalShift(cloud->getGlobalShift());
				newPointCloud->setGlobalScale(cloud->getGlobalScale());
				newPointCloud->setDisplay(cloud->getDisplay());
				newPointCloud->prepareDisplayForRefresh();
				if (cloud->getParent())
					cloud->getParent()->addChild(newPointCloud);
				cloud->setEnabled(false);
				addToDB(newPointCloud);

				newPointCloud->refreshDisplay();
				resultingClouds.push_back(newPointCloud);

				if (warnings)
				{
					ccLog::Warning("[Subsampling] Not enough memory: colors, normals or scalar fields may be missing!");
					errors = true;
				}
			}
			else
			{
				ccLog::Error("Not enough memory!");
				break;
			}
		}

		ccLog::Print("[Subsampling] Timing: %3.3f s.",eTimer.elapsed()/1000.0);

		if (errors)
		{
			ccLog::Error("Errors occurred (see console)");
		}
	}

	if (m_ccRoot)
		m_ccRoot->selectEntities(resultingClouds);

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

	ccStatisticalTestDlg* sDlg = 0;
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

	if (sDlg->exec())
	{
		//build up corresponding distribution
		CCLib::GenericDistribution* distrib = 0;
		{
			ScalarType a = static_cast<ScalarType>(sDlg->getParam1());
			ScalarType b = static_cast<ScalarType>(sDlg->getParam2());
			ScalarType c = static_cast<ScalarType>(sDlg->getParam3());

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

		double pChi2 = sDlg->getProba();
		int nn = sDlg->getNeighborsNumber();

		size_t selNum = m_selectedEntities.size();
		for (size_t i=0; i<selNum; ++i)
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
					if (chi2SfIdx < 0)
						chi2SfIdx = pc->addScalarField(CC_CHI2_DISTANCES_DEFAULT_SF_NAME);
					if (chi2SfIdx < 0)
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

					QElapsedTimer eTimer;
					eTimer.start();

					double chi2dist = CCLib::StatisticalTestingTools::testCloudWithStatisticalModel(distrib,pc,nn,pChi2,&pDlg,theOctree);

					ccConsole::Print("[Chi2 Test] Timing: %3.2f ms.",eTimer.elapsed()/1.0e3);
					ccConsole::Print("[Chi2 Test] %s test result = %f",distrib->getName(),chi2dist);

					//we set the theoretical Chi2 distance limit as the minimum displayed SF value so that all points below are grayed
					{
						ccScalarField* chi2SF = static_cast<ccScalarField*>(pc->getCurrentInScalarField());
						assert(chi2SF);
						chi2SF->computeMinAndMax();
						chi2dist *= chi2dist;
						chi2SF->setMinDisplayed(static_cast<ScalarType>(chi2dist));
						chi2SF->setSymmetricalScale(false);
						chi2SF->setSaturationStart(static_cast<ScalarType>(chi2dist));
						//chi2SF->setSaturationStop(chi2dist);
						pc->setCurrentDisplayedScalarField(chi2SfIdx);
						pc->showSF(true);
						pc->prepareDisplayForRefresh_recursive();
					}
				}
			}
		}

		delete distrib;
		distrib = 0;
	}

	delete sDlg;
	sDlg = 0;

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

	CCLib::GenericDistribution* distrib = 0;
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

	size_t selNum = m_selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
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
				assert(outSfIdx >= 0);
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
					unsigned numberOfClasses = static_cast<unsigned>(ceil(sqrt(static_cast<double>(pc->size()))));
					std::vector<unsigned> histo;
					std::vector<double> npis;
					try
					{
						histo.resize(numberOfClasses,0);
						npis.resize(numberOfClasses,0.0);
					}
					catch (const std::bad_alloc&)
					{
						ccConsole::Warning("[Distribution fitting] Not enough memory!");
						continue;
					}

					unsigned finalNumberOfClasses = 0;
					double chi2dist = CCLib::StatisticalTestingTools::computeAdaptativeChi2Dist(distrib,pc,0,finalNumberOfClasses,false,0,0,&(histo[0]),&(npis[0]));

					if (chi2dist >= 0.0)
					{
						ccConsole::Print("[Distribution fitting] %s: Chi2 Distance = %f",distrib->getName(),chi2dist);
					}
					else
					{
						ccConsole::Warning("[Distribution fitting] Failed to compute Chi2 distance?!");
						continue;
					}

					//show histogram
					ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);
					hDlg->setWindowTitle("[Distribution fitting]");
					{
						ccHistogramWindow* histogram = hDlg->window();
						histogram->fromBinArray(histo,sf->getMin(),sf->getMax());
						histo.clear();
						histogram->setCurveValues(npis);
						npis.clear();
						histogram->setTitle(description);
						histogram->setColorScheme(ccHistogramWindow::USE_CUSTOM_COLOR_SCALE);
						histogram->setColorScale(sf->getColorScale());
						histogram->setAxisLabels(sf->getName(),"Count");
						histogram->refresh();
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
	distrib = 0;
}

struct ComponentIndexAndSize
{
	unsigned index;
	unsigned size;

	ComponentIndexAndSize(unsigned i, unsigned s) : index(i), size(s) {}

	static bool DescendingCompOperator(const ComponentIndexAndSize& a, const ComponentIndexAndSize& b)
	{
		return a.size > b.size;
	}
};

void MainWindow::createComponentsClouds(ccGenericPointCloud* cloud,
										CCLib::ReferenceCloudContainer& components,
										unsigned minPointsPerComponent,
										bool randomColors,
										bool selectComponents,
										bool sortBysize/*=true*/)
{
	if (!cloud || components.empty())
		return;

	std::vector<ComponentIndexAndSize> sortedIndexes;
	std::vector<ComponentIndexAndSize>* _sortedIndexes = 0;
	if (sortBysize)
	{
		try
		{
			sortedIndexes.reserve(components.size());
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Warning("[CreateComponentsClouds] Not enough memory to sort components by size!");
			sortBysize = false;
		}

		if (sortBysize) //still ok?
		{
			unsigned compCount = static_cast<unsigned>(components.size());
			for (unsigned i=0; i<compCount; ++i)
			{
				sortedIndexes.push_back(ComponentIndexAndSize(i,components[i]->size()));
			}

			std::sort(sortedIndexes.begin(), sortedIndexes.end(), ComponentIndexAndSize::DescendingCompOperator);
			_sortedIndexes = &sortedIndexes;
		}
	}

	//we create "real" point clouds for all input components
	{
		ccPointCloud* pc = cloud->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(cloud) : 0;

		//we create a new group to store all CCs
		ccHObject* ccGroup = new ccHObject(cloud->getName()+QString(" [CCs]"));

		//for each component
		for (unsigned i=0; i<components.size(); ++i)
		{
			CCLib::ReferenceCloud* compIndexes = _sortedIndexes ? components[_sortedIndexes->at(i).index] : components[i];

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
						ccColor::Rgb col = ccColor::Generator::Random();
						compCloud->setRGBColor(col);
						compCloud->showColors(true);
						compCloud->showSF(false);
					}

					//'shift on load' information
					if (pc)
					{
						compCloud->setGlobalShift(pc->getGlobalShift());
						compCloud->setGlobalScale(pc->getGlobalScale());
					}
					compCloud->setVisible(true);
					compCloud->setName(QString("CC#%1").arg(ccGroup->getChildrenNumber()));

					//we add new CC to group
					ccGroup->addChild(compCloud);

					if (selectComponents && m_ccRoot)
						m_ccRoot->selectEntity(compCloud,true);
				}
				else
				{
					ccConsole::Warning("[createComponentsClouds] Failed to create component #%i! (not enough memory)",ccGroup->getChildrenNumber()+1);
				}
			}

			delete compIndexes;
			compIndexes = 0;
		}

		components.clear();

		if (ccGroup->getChildrenNumber() == 0)
		{
			ccConsole::Error("No component was created! Check the minimum size...");
			delete ccGroup;
			ccGroup = 0;
		}
		else
		{
			ccGroup->setDisplay(cloud->getDisplay());
			addToDB(ccGroup);
		}

		ccConsole::Print(QString("[createComponentsClouds] %1 component(s) were created from cloud '%2'").arg(ccGroup->getChildrenNumber()).arg(cloud->getName()));

		cloud->prepareDisplayForRefresh();

		//auto-hide original cloud
		if (ccGroup)
		{
			cloud->setEnabled(false);
			ccConsole::Warning("[createComponentsClouds] Original cloud has been automatically hidden");
		}
	}
}

void MainWindow::doActionLabelConnectedComponents()
{
	ccHObject::Container selectedEntities = m_selectedEntities;
	//keep only the point clouds!
	std::vector<ccGenericPointCloud*> clouds;
	{
		size_t selNum = selectedEntities.size();
		for (size_t i=0; i<selNum; ++i)
		{
			ccHObject* ent = selectedEntities[i];
			if (ent->isKindOf(CC_TYPES::POINT_CLOUD))
				clouds.push_back(ccHObjectCaster::ToGenericPointCloud(ent));
		}
	}

	size_t count = clouds.size();
	if (count == 0)
		return;

	ccLabelingDlg dlg(this);
	if (count == 1)
		dlg.octreeLevelSpinBox->setCloud(clouds.front());
	if (!dlg.exec())
		return;

	int octreeLevel = dlg.getOctreeLevel();
	int minComponentSize = dlg.getMinPointsNb();
	bool randColors = dlg.randomColors();

	ccProgressDialog pDlg(false,this);

	//we unselect all entities as we are going to automatically select the created components
	//(otherwise the user won't percieve the change!)
	if (m_ccRoot)
		m_ccRoot->unselectAllEntities();

	for (size_t i=0; i<count; ++i)
	{
		ccGenericPointCloud* cloud = clouds[i];

		if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD)) //TODO
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
			if (sfIdx < 0)
				sfIdx = pc->addScalarField(CC_CONNECTED_COMPONENTS_DEFAULT_LABEL_NAME);
			if (sfIdx < 0)
			{
				ccConsole::Error("Couldn't allocate a new scalar field for computing CC labels! Try to free some memory ...");
				break;
			}
			pc->setCurrentScalarField(sfIdx);

			//we try to label all CCs
			CCLib::ReferenceCloudContainer components;
			if (CCLib::AutoSegmentationTools::labelConnectedComponents(	cloud,
																		static_cast<unsigned char>(octreeLevel),
																		false,
																		&pDlg,
																		theOctree) >= 0)
			{
				//if successfull, we extract each CC (stored in "components")
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
			sfIdx = -1;

			//we create "real" point clouds for all CCs
			if (!components.empty())
			{
				createComponentsClouds(cloud, components, minComponentSize, randColors, true);
			}
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionSetSFAsCoord()
{
	ccExportCoordToSFDlg ectsDlg(this);
	ectsDlg.warningLabel->setVisible(false);
	ectsDlg.setWindowTitle("Export SF to coordinate(s)");

	if (!ectsDlg.exec())
		return;

	bool exportDim[3] = {ectsDlg.exportX(), ectsDlg.exportY(), ectsDlg.exportZ()};
	if (!exportDim[0] && !exportDim[1] && !exportDim[2]) //nothing to do?!
		return;

	//for each selected cloud (or vertices set)
	size_t selNum = m_selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities[i]);
		if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD))
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

			ccScalarField* sf = pc->getCurrentDisplayedScalarField();
			if (sf)
			{
				unsigned ptsCount = pc->size();
				bool hasDefaultValueForNaN = false;
				ScalarType defaultValueForNaN = sf->getMin();

				for (unsigned i=0; i<ptsCount; ++i)
				{
					ScalarType s = sf->getValue(i);

					//handle NaN values
					if (!CCLib::ScalarField::ValidValue(s))
					{
						if (!hasDefaultValueForNaN)
						{
							bool ok;
							double out = QInputDialog::getDouble(this,"SF --> coordinate","Enter the coordinate equivalent for NaN values:",defaultValueForNaN,-1.0e9,1.0e9,6,&ok);
							if (ok)
								defaultValueForNaN = static_cast<ScalarType>(out);
							else
								ccLog::Warning("[SetSFAsCoord] By default the coordinate equivalent for NaN values will be the minimum SF value");
							hasDefaultValueForNaN = true;
						}
						s = defaultValueForNaN;
					}

					CCVector3* P = const_cast<CCVector3*>(pc->getPoint(i));

					//test each dimension
					if (exportDim[0])
						P->x = s;
					if (exportDim[1])
						P->y = s;
					if (exportDim[2])
						P->z = s;
				}

				pc->invalidateBoundingBox();
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
	const QString defaultSFName[3] = {"Coord. X", "Coord. Y", "Coord. Z"};

	if (!exportDim[0] && !exportDim[1] && !exportDim[2]) //nothing to do?!
		return;

	//for each selected cloud (or vertices set)
	size_t selNum = m_selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		ccPointCloud* pc = ccHObjectCaster::ToPointCloud(m_selectedEntities[i]);
		if (pc)
		{
			unsigned ptsCount = pc->size();

			//test each dimension
			for (unsigned d=0; d<3; ++d)
			{
				if (exportDim[d])
				{
					int sfIndex = pc->getScalarFieldIndexByName(qPrintable(defaultSFName[d]));
					if (sfIndex < 0)
						sfIndex = pc->addScalarField(qPrintable(defaultSFName[d]));
					if (sfIndex < 0)
					{
						ccLog::Error("Not enough memory!");
						i = selNum;
						break;
					}

					CCLib::ScalarField* sf = pc->getScalarField(sfIndex);
					assert(sf && sf->currentSize() == ptsCount);
					if (sf)
					{
						for (unsigned k=0; k<ptsCount; ++k)
						{
							ScalarType s = static_cast<ScalarType>(pc->getPoint(k)->u[d]);
							sf->setValue(k,s);
						}
						sf->computeMinAndMax();
						pc->setCurrentDisplayedScalarField(sfIndex);
						m_selectedEntities[i]->showSF(true);
						m_selectedEntities[i]->prepareDisplayForRefresh_recursive();
					}
				}
			}
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doConvertPolylinesToMesh()
{
	size_t selNum = m_selectedEntities.size();
	if (selNum == 0)
		return;

	std::vector<ccPolyline*> polylines;
	try
	{
		if (selNum == 1 && m_selectedEntities.back()->isA(CC_TYPES::HIERARCHY_OBJECT))
		{
			ccHObject* obj = m_selectedEntities.back();
			for (unsigned i=0; i<obj->getChildrenNumber(); ++i)
			{
				if (obj->getChild(i)->isA(CC_TYPES::POLY_LINE))
					polylines.push_back(static_cast<ccPolyline*>(obj->getChild(i)));
			}
		}
		else
		{
			for (size_t i=0; i<selNum; ++i)
			{
				if (m_selectedEntities[i]->isA(CC_TYPES::POLY_LINE))
					polylines.push_back(static_cast<ccPolyline*>(m_selectedEntities[i]));
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccConsole::Error("Not enough memory!");
		return;
	}

	if (polylines.empty())
	{
		ccConsole::Error("Select a group of polylines or multiple polylines (contour plot)!");
		return;
	}

	ccPickOneElementDlg poeDlg("Projection dimension","Contour plot to mesh",this);
	poeDlg.addElement("X");
	poeDlg.addElement("Y");
	poeDlg.addElement("Z");
	poeDlg.setDefaultIndex(2);
	if (!poeDlg.exec())
		return;

	int dim = poeDlg.getSelectedIndex();
	assert(dim >= 0 && dim < 3);

	const unsigned char Z = static_cast<unsigned char>(dim);
	const unsigned char X = Z == 2 ? 0 : Z+1;
	const unsigned char Y = X == 2 ? 0 : X+1;

	//number of segments
	unsigned segmentCount = 0;
	unsigned vertexCount = 0;
	{
		for (size_t i=0; i<polylines.size(); ++i)
		{
			ccPolyline* poly = polylines[i];
			assert(poly);
			if (poly)
			{
				//count the total number of vertices and segments
				unsigned vertCount = poly->size();
				unsigned maxVertCount = poly->isClosed() ? vertCount : vertCount-1;
				if (vertCount != 0)
				{
					vertexCount += vertCount;
					segmentCount += maxVertCount;
				}
			}
		}
	}
	
	if (segmentCount < 2)
	{
		//not enough points/segments
		ccLog::Error("Not enough segments!");
		return;
	}
#define USE_CGAL_LIB
#if defined(USE_TRIANGLE_LIB) || defined(USE_CGAL_LIB)
	std::vector<CCVector2> points2D;
	std::vector<int> segments2D;
	try
	{
		points2D.reserve(vertexCount);
		segments2D.reserve(segmentCount * 2);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		ccLog::Error("Not enough memory");
		return;
	}

	//fill arrays
	{
		for (size_t i=0; i<polylines.size(); ++i)
		{
			ccPolyline* poly = polylines[i];
			if (poly)
			{
				unsigned vertCount = poly->size();
				int vertIndex0 = static_cast<int>(points2D.size());
				bool closed = poly->isClosed();
				for (unsigned v=0; v<vertCount; ++v)
				{
					const CCVector3* P = poly->getPoint(v);
					int vertIndex = static_cast<int>(points2D.size());
					points2D.push_back(CCVector2(P->u[X],P->u[Y]));
					
					if (v+1 < vertCount)
					{
						segments2D.push_back(vertIndex);
						segments2D.push_back(vertIndex+1);
					}
					else if (closed)
					{
						segments2D.push_back(vertIndex);
						segments2D.push_back(vertIndex0);
					}
				}
			}
		}
		assert(points2D.size() == vertexCount);
		assert(segments2D.size() == segmentCount*2);
	}

	CCLib::Delaunay2dMesh* delaunayMesh = new CCLib::Delaunay2dMesh;
	char errorStr[1024];
	if (!delaunayMesh->buildMesh(points2D,segments2D,errorStr))
	{
		ccLog::Error(QString("Triangle lib error: %1").arg(errorStr));
		delete delaunayMesh;
		return;
	}

	ccPointCloud* vertices = new ccPointCloud("vertices");
	if (!vertices->reserve(vertexCount))
	{
		//not enough memory
		ccLog::Error("Not enough memory");
		delete vertices;
		delete delaunayMesh;
		return;
	}

	//fill vertices cloud
	{
		for (size_t i=0; i<polylines.size(); ++i)
		{
			ccPolyline* poly = polylines[i];
			unsigned vertCount = poly->size();
			for (unsigned v=0; v<vertCount; ++v)
			{
				const CCVector3* P = poly->getPoint(v);
				vertices->addPoint(*P);
			}
		}
		delaunayMesh->linkMeshWith(vertices,false);
	}

#else
	double totalLength = 0.0;
	{
		for (size_t i=0; i<polylines.size(); ++i)
		{
			ccPolyline* poly = polylines[i];
			assert(poly);
			if (poly)
			{
				//compute total length
				totalLength += poly->computeLength();
			}
		}
	}
	//sample points on the polylines
	double step = QInputDialog::getDouble(this,"Contour plot meshing","Sampling step",totalLength/1000.0,1.0e-6,1.0e6,6);
	unsigned approxCount = static_cast<unsigned>(totalLength/step) + vertexCount;

	ccPointCloud* vertices = new ccPointCloud("vertices");
	if (!vertices->reserve(approxCount))
	{
		ccLog::Error("Not enough memory!");
		delete vertices;
		return;
	}

	//now let sample points on the polylines
	for (size_t i=0; i<polylines.size(); ++i)
	{
		ccPolyline* poly = polylines[i];
		if (poly)
		{
			bool closed = poly->isClosed();
			unsigned vertCount = poly->size();
			unsigned maxVertCount = closed ? vertCount : vertCount-1;
			for (unsigned v=0; v<vertCount; ++v)
			{
				const CCVector3* A = poly->getPoint(v);
				const CCVector3* B = poly->getPoint((v+1)%vertCount);

				CCVector3 AB = *B-*A;
				double l = AB.norm();
				double s = 0.0;
				while (s < l)
				{
					CCVector3 P = *A + AB * (s/l);
					vertices->addPoint(P);
					s += step;
				}

				//add the last point if the polyline is not closed!
				if (!closed && v+1 == maxVertCount)
					vertices->addPoint(*B);
			}
		}

		if (vertices->size() < 3)
		{
			ccLog::Error("Not enough vertices (reduce the step size)!");
			delete vertices;
			return;
		}
	}

	char errorStr[1024];
	CCLib::GenericIndexedMesh* delaunayMesh = CCLib::PointProjectionTools::computeTriangulation(vertices,DELAUNAY_2D_AXIS_ALIGNED,0,dim,errorStr);
	if (!delaunayMesh)
	{
		ccLog::Error(QString("Triangle lib error: %1").arg(errorStr));
		delete delaunayMesh;
		delete vertices;
		return;
	}

#endif

#ifdef _DEBUG
	//Test delaunay output
	{
		unsigned vertCount = vertices->size();
		for (unsigned i=0; i<delaunayMesh->size(); ++i)
		{
			const CCLib::VerticesIndexes* tsi = delaunayMesh->getTriangleVertIndexes(i);
			assert(tsi->i1 < vertCount && tsi->i2 < vertCount && tsi->i3 < vertCount);
		}
	}
#endif
	
	ccMesh* mesh = new ccMesh(delaunayMesh,vertices);
	if (mesh->size() != delaunayMesh->size())
	{
		//not enough memory (error will be issued later)
		delete mesh;
		mesh = 0;
	}

	//don't need this anymore
	delete delaunayMesh;
	delaunayMesh = 0;

	if (mesh)
	{
		mesh->addChild(vertices);
		mesh->setVisible(true);
		vertices->setEnabled(false);
		addToDB(mesh);
		if (mesh->computePerVertexNormals())
			mesh->showNormals(true);
		else
			ccLog::Warning("[Contour plot to mesh] Failed to compute normals!");

		if (mesh->getDisplay())
			mesh->getDisplay()->redraw();
	}
	else
	{
		ccLog::Error("Not enough memory!");
		if (vertices)
			delete vertices;
		vertices = 0;
	}
}

void MainWindow::doCompute2HalfDimVolume()
{
	size_t selNum = m_selectedEntities.size();
	if (selNum < 1 || selNum > 2)
	{
		ccConsole::Error("Select only one point cloud!");
		return;
	}

	ccGenericPointCloud* cloud1 = 0;
	{
		ccHObject* ent = m_selectedEntities[0];
		if (!ent->isKindOf(CC_TYPES::POINT_CLOUD) )
		{
			ccConsole::Error("Select point clouds only!");
			return;
		}
		else
		{
			cloud1 = ccHObjectCaster::ToGenericPointCloud(ent);
		}
	}

	ccGenericPointCloud* cloud2 = 0;
	if (m_selectedEntities.size() > 1)
	{
		ccHObject* ent = m_selectedEntities[1];
		if (!ent->isKindOf(CC_TYPES::POINT_CLOUD) )
		{
			ccConsole::Error("Select point clouds only!");
			return;
		}
		else
		{
			cloud2 = ccHObjectCaster::ToGenericPointCloud(ent);
		}
	}

	ccVolumeCalcTool calcVolumeTool(cloud1, cloud2, this);
	calcVolumeTool.exec();
}

void MainWindow::doActionRasterize()
{
	size_t selNum = m_selectedEntities.size();
	if (selNum != 1)
	{
		ccConsole::Error("Select only one point cloud!");
		return;
	}

	ccHObject* ent = m_selectedEntities[0];
	if (!ent->isKindOf(CC_TYPES::POINT_CLOUD) )
	{
		ccConsole::Error("Select a point cloud!");
		return;
	}

	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);
	ccRasterizeTool rasterizeTool(cloud,this);
	rasterizeTool.exec();
}

void MainWindow::doActionComputeMeshAA()
{
	doActionComputeMesh(DELAUNAY_2D_AXIS_ALIGNED);
}

void MainWindow::doActionComputeMeshLS()
{
	doActionComputeMesh(DELAUNAY_2D_BEST_LS_PLANE);
}

static double s_meshMaxEdgeLength = 0;
void MainWindow::doActionComputeMesh(CC_TRIANGULATION_TYPES type)
{
	bool ok = true;
	double maxEdgeLength = QInputDialog::getDouble(this,"Triangulate", "Max edge length (0 = no limit)", s_meshMaxEdgeLength, 0, 1.0e9, 8, &ok);
	if (!ok)
		return;
	s_meshMaxEdgeLength = maxEdgeLength;

	//select candidates
	ccHObject::Container clouds;
	bool hadNormals = false;
	{
		for (size_t i=0; i<m_selectedEntities.size(); ++i)
		{
			ccHObject* ent = m_selectedEntities[i];
			if (ent->isKindOf(CC_TYPES::POINT_CLOUD))
			{
				clouds.push_back(ent);
				if (ent->isA(CC_TYPES::POINT_CLOUD))
					hadNormals |= static_cast<ccPointCloud*>(ent)->hasNormals();
			}
		}
	}

	//if the cloud(s) already had normals, ask the use if wants to update them or keep them as is (can look strange!)
	bool updateNormals = false;
	if (hadNormals)
	{
		updateNormals = (QMessageBox::question(	this,
												"Keep old normals?",
												"Cloud(s) already have normals. Do you want to update them (yes) or keep the old ones (no)?",
												QMessageBox::Yes,
												QMessageBox::No ) == QMessageBox::Yes);
	}

	ccProgressDialog pDlg(false,this);
	pDlg.setWindowTitle("Triangulation");
	pDlg.setInfo("Triangulation in progress...");
	pDlg.setRange(0,0);
	pDlg.show();
	QApplication::processEvents();

	bool errors = false;
	for (size_t i=0; i<clouds.size(); ++i)
	{
		ccHObject* ent = clouds[i];
		assert(ent->isKindOf(CC_TYPES::POINT_CLOUD));

		//compute mesh
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);
		ccMesh* mesh = ccMesh::Triangulate(	cloud,
											type,
											updateNormals,
											static_cast<PointCoordinateType>(maxEdgeLength),
											2 //XY plane by default
											);
		if (mesh)
		{
			cloud->setVisible(false); //can't disable the cloud as the resulting mesh will be its child!
			cloud->addChild(mesh);
			cloud->prepareDisplayForRefresh_recursive();
			addToDB(mesh);
			if (i == 0)
				m_ccRoot->selectEntity(mesh); //auto-select first element
		}
		else
		{
			errors = true;
		}
	}

	if (errors)
	{
		ccConsole::Error("Error(s) occurred! See the Console messages");
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionFitQuadric()
{
	ccHObject::Container selectedEntities = m_selectedEntities;
	size_t selNum = selectedEntities.size();

	bool errors = false;
	//for all selected entities
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = selectedEntities[i];
		//look for clouds
		if (ent->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);

			double rms = 0;
			ccQuadric* quadric = ccQuadric::Fit(cloud,&rms);
			if (quadric)
			{
				cloud->addChild(quadric);
				quadric->setName(QString("Quadric (%1)").arg(cloud->getName()));
				quadric->setDisplay(cloud->getDisplay());
				quadric->prepareDisplayForRefresh();
				addToDB(quadric);

				ccConsole::Print(QString("[doActionFitQuadric] Quadric local coordinate system:"));
				ccConsole::Print(quadric->getTransformation().toString(12,' ')); //full precision
				ccConsole::Print(QString("[doActionFitQuadric] Quadric equation (in local coordinate system): ") + quadric->getEquationString());
				ccConsole::Print(QString("[doActionFitQuadric] RMS: %1").arg(rms));

#if 0
				//test: project the input cloud on the quadric
				if (cloud->isA(CC_TYPES::POINT_CLOUD))
				{
					ccPointCloud* newCloud = static_cast<ccPointCloud*>(cloud)->cloneThis();
					if (newCloud)
					{
						const PointCoordinateType* eq = quadric->getEquationCoefs();
						const Tuple3ub& dims = quadric->getEquationDims();
					
						const unsigned char dX = dims.x;
						const unsigned char dY = dims.y;
						const unsigned char dZ = dims.z;

						const ccGLMatrix& trans = quadric->getTransformation();
						ccGLMatrix invTrans = trans.inverse();
						for (unsigned i=0; i<newCloud->size(); ++i)
						{
							CCVector3* P = const_cast<CCVector3*>(newCloud->getPoint(i));
							CCVector3 Q = invTrans * (*P);
							Q.u[dZ] = eq[0] + eq[1]*Q.u[dX] + eq[2]*Q.u[dY] + eq[3]*Q.u[dX]*Q.u[dX] + eq[4]*Q.u[dX]*Q.u[dY] + eq[5]*Q.u[dY]*Q.u[dY];
							*P = trans * Q;
						}
						newCloud->invalidateBoundingBox();
						newCloud->setName(newCloud->getName() + ".projection_on_quadric");
						addToDB(newCloud);
					}
				}
#endif
			}
			else
			{
				ccConsole::Warning(QString("Failed to compute quadric on cloud '%1'").arg(cloud->getName()));
				errors = true;
			}
		}
	}

	if (errors)
	{
		ccConsole::Error("Error(s) occurred: see console");
	}

	refreshAll();
}

void MainWindow::doActionComputeDistanceMap()
{
	ccHObject::Container selectedEntities = m_selectedEntities;

	bool ok = true;
	unsigned steps = static_cast<unsigned>(QInputDialog::getInt(this, "Distance map", "Distance map resolution", 128, 16, 1024, 16, &ok));
	if (!ok)
		return;

	size_t selNum = selectedEntities.size();
	for (size_t i = 0; i < selNum; ++i)
	{
		ccHObject* ent = selectedEntities[i];
		if (ent->isKindOf(CC_TYPES::MESH))
		{
			//CCLib::ChamferDistanceTransform cdt;

			CCLib::SaitoSquaredDistanceTransform cdt;
			if (!cdt.initGrid(Tuple3ui(steps, steps, steps)))
			{
				//not enough memory
				ccLog::Error("Not enough memory!");
				return;
			}

			ccMesh* mesh = static_cast<ccMesh*>(ent);
			ccBBox box = mesh->getOwnBB();
			PointCoordinateType largestDim = box.getMaxBoxDim();
			PointCoordinateType cellDim = largestDim / steps;
			CCVector3 minCorner = box.getCenter() - CCVector3(1, 1, 1) * (largestDim / 2);

			ccProgressDialog pDlg(true, this);
			if (cdt.initDT(mesh, cellDim, minCorner, &pDlg))
			{
				//cdt.propagateDistance(CHAMFER_345, &pDlg);
				cdt.propagateDistance(&pDlg);

				//convert the grid to a cloud
				ccPointCloud* gridCloud = new ccPointCloud(mesh->getName() + QString(".distance_grid(%1)").arg(steps));
				{
					unsigned pointCount = steps*steps*steps;
					if (!gridCloud->reserve(pointCount))
					{
						ccLog::Error("Not enough memory!");
						delete gridCloud;
						return;
					}

					ccScalarField* sf = new ccScalarField("DT values");
					if (!sf->reserve(pointCount))
					{
						ccLog::Error("Not enough memory!");
						delete gridCloud;
						sf->release();
						return;
					}

					for (unsigned i = 0; i < steps; ++i)
					{
						for (unsigned j = 0; j < steps; ++j)
						{
							for (unsigned k = 0; k < steps; ++k)
							{
								gridCloud->addPoint(minCorner + CCVector3(i + 0.5, j + 0.5, k + 0.5) * cellDim);
								ScalarType s = static_cast<ScalarType>(cdt.getValue(i, j, k));
								//sf->addElement(s < maxDist ? s : NAN_VALUE);
								sf->addElement(sqrt(s));
							}
						}
					}

					sf->computeMinAndMax();
					int sfIdx = gridCloud->addScalarField(sf);
					gridCloud->setCurrentDisplayedScalarField(sfIdx);
					gridCloud->showSF(true);
					gridCloud->setDisplay(mesh->getDisplay());
					addToDB(gridCloud);
				}
			}
			else
			{
				ccLog::Error("Not enough memory!");
				return;
			}
		}
	}
}

void MainWindow::doActionComputeDistToBestFitQuadric3D()
{
	ccHObject::Container selectedEntities = m_selectedEntities;

	bool ok = true;
	int steps = QInputDialog::getInt(this,"Distance to best fit quadric (3D)","Steps (per dim.)",50,10,10000,10,&ok);
	if (!ok)
		return;

	size_t selNum = selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = selectedEntities[i];
		if (ent->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);
			CCLib::Neighbourhood Yk(cloud);

			double Q[10];
			if (Yk.compute3DQuadric(Q))
			{
				const double& a = Q[0];
				const double& b = Q[1];
				const double& c = Q[2];
				const double& e = Q[3];
				const double& f = Q[4];
				const double& g = Q[5];
				const double& l = Q[6];
				const double& m = Q[7];
				const double& n = Q[8];
				const double& d = Q[9];

				//gravity center
				const CCVector3* G = Yk.getGravityCenter();
				if (!G)
				{
					ccConsole::Warning(QString("Failed to get gravity center of cloud '%1'!").arg(cloud->getName()));
					continue;
				}

				const ccBBox bbox = cloud->getOwnBB();
				PointCoordinateType maxDim = bbox.getMaxBoxDim();
				CCVector3 C = bbox.getCenter();

				//Sample points on a cube and compute for each of them the distance to the Quadric
				ccPointCloud* newCloud = new ccPointCloud();
				if (!newCloud->reserve(steps*steps*steps))
				{
					ccConsole::Error("Not enough memory!");
				}

				const char defaultSFName[] = "Dist. to 3D quadric";
				int sfIdx = newCloud->getScalarFieldIndexByName(defaultSFName);
				if (sfIdx < 0)
					sfIdx = newCloud->addScalarField(defaultSFName);
				if (sfIdx < 0)
				{
					ccConsole::Error("Couldn't allocate a new scalar field for computing distances! Try to free some memory ...");
					delete newCloud;
					continue;
				}

				ccScalarField* sf = static_cast<ccScalarField*>(newCloud->getScalarField(sfIdx));
				assert(sf);

				//FILE* fp = fopen("doActionComputeQuadric3D_trace.txt","wt");
				unsigned count = 0;
				for (int x=0; x<steps; ++x)
				{
					CCVector3 P;
					P.x = C.x + maxDim * (static_cast<PointCoordinateType>(x) / static_cast<PointCoordinateType>(steps-1) - PC_ONE/2);
					for (int y=0; y<steps; ++y)
					{
						P.y = C.y + maxDim * (static_cast<PointCoordinateType>(y) / static_cast<PointCoordinateType>(steps-1) - PC_ONE/2);
						for (int z=0; z<steps; ++z)
						{
							P.z = C.z + maxDim * (static_cast<PointCoordinateType>(z) / static_cast<PointCoordinateType>(steps-1) - PC_ONE/2);
							newCloud->addPoint(P);

							//compute distance to quadric
							CCVector3 Pc = P-*G;
							ScalarType dist = static_cast<ScalarType>(	a*Pc.x*Pc.x + b*Pc.y*Pc.y + c*Pc.z*Pc.z
																	+	e*Pc.x*Pc.y + f*Pc.y*Pc.z + g*Pc.x*Pc.z
																	+	l*Pc.x + m*Pc.y + n*Pc.z + d);

							sf->setValue(count++,dist);
							//fprintf(fp,"%f %f %f %f\n",Pc.x,Pc.y,Pc.z,dist);
						}
					}
				}
				//fclose(fp);

				if (sf)
				{
					sf->computeMinAndMax();
					newCloud->setCurrentDisplayedScalarField(sfIdx);
					newCloud->showSF(true);
				}
				newCloud->setName("Distance map to 3D quadric");
				newCloud->setDisplay(cloud->getDisplay());
				newCloud->prepareDisplayForRefresh();

				addToDB(newCloud);
			}
			else
			{
				ccConsole::Warning(QString("Failed to compute 3D quadric on cloud '%1'").arg(cloud->getName()));
			}
		}
	}

	refreshAll();
}

void MainWindow::doActionComputeCPS()
{
	size_t selNum = m_selectedEntities.size();
	if (selNum != 2)
	{
		ccConsole::Error("Select 2 point clouds!");
		return;
	}

	if (!m_selectedEntities[0]->isKindOf(CC_TYPES::POINT_CLOUD) ||
		!m_selectedEntities[1]->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error("Select 2 point clouds!");
		return;
	}

	ccOrderChoiceDlg dlg(	m_selectedEntities[0], "Compared",
							m_selectedEntities[1], "Reference",
							this );
	if (!dlg.exec())
		return;

	ccGenericPointCloud* compCloud = ccHObjectCaster::ToGenericPointCloud(dlg.getFirstEntity());
	ccGenericPointCloud* srcCloud = ccHObjectCaster::ToGenericPointCloud(dlg.getSecondEntity());

	if (!compCloud->isA(CC_TYPES::POINT_CLOUD)) //TODO
	{
		ccConsole::Error("Compared cloud must be a real point cloud!");
		return;
	}
	ccPointCloud* cmpPC = static_cast<ccPointCloud*>(compCloud);

	static const char DEFAULT_CPS_TEMP_SF_NAME[] = "CPS temporary";
	int sfIdx = cmpPC->getScalarFieldIndexByName(DEFAULT_CPS_TEMP_SF_NAME);
	if (sfIdx < 0)
		sfIdx = cmpPC->addScalarField(DEFAULT_CPS_TEMP_SF_NAME);
	if (sfIdx < 0)
	{
		ccConsole::Error("Couldn't allocate a new scalar field for computing distances! Try to free some memory ...");
		return;
	}
	cmpPC->setCurrentScalarField(sfIdx);
	cmpPC->enableScalarField();
	//cmpPC->forEach(CCLib::ScalarFieldTools::SetScalarValueToNaN); //now done by default by computeCloud2CloudDistance

	CCLib::ReferenceCloud CPSet(srcCloud);
	ccProgressDialog pDlg(true,this);
	CCLib::DistanceComputationTools::Cloud2CloudDistanceComputationParams params;
	params.CPSet = &CPSet;
	int result = CCLib::DistanceComputationTools::computeCloud2CloudDistance(compCloud,srcCloud,params,&pDlg);
	cmpPC->deleteScalarField(sfIdx);

	if (result >= 0)
	{
		ccPointCloud* newCloud = 0;
		//if the source cloud is a "true" cloud, the extracted CPS
		//will also get its attributes
		newCloud = srcCloud->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(srcCloud)->partialClone(&CPSet) : ccPointCloud::From(&CPSet,srcCloud);

		newCloud->setName(QString("[%1]->CPSet(%2)").arg(srcCloud->getName()).arg(compCloud->getName()));
		newCloud->setDisplay(compCloud->getDisplay());
		newCloud->prepareDisplayForRefresh();
		addToDB(newCloud);

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

	//look for clouds and meshes
	std::vector<ccPointCloud*> clouds;
	size_t cloudsWithScanGrids = 0;
	std::vector<ccMesh*> meshes;
	PointCoordinateType defaultRadius = 0;
	try
	{
		for (size_t i=0; i<m_selectedEntities.size(); ++i)
		{
			if (m_selectedEntities[i]->isA(CC_TYPES::POINT_CLOUD))
			{
				ccPointCloud* cloud = static_cast<ccPointCloud*>(m_selectedEntities[i]);
				clouds.push_back(cloud);

				if (cloud->gridCount() != 0)
					++cloudsWithScanGrids;

				if (defaultRadius == 0)
				{
					//default radius
					defaultRadius = ccNormalVectors::GuessNaiveRadius(cloud);
				}
			}
			else if (m_selectedEntities[i]->isKindOf(CC_TYPES::MESH))
			{
				if (m_selectedEntities[i]->isA(CC_TYPES::MESH))
				{
					ccMesh* mesh = ccHObjectCaster::ToMesh(m_selectedEntities[i]);
					meshes.push_back(mesh);
				}
				else
				{
					ccConsole::Error(QString("Can't compute normals on sub-meshes! Select the parent mesh instead"));
					return;
				}
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccConsole::Error("Not enough memory!");
		return;
	}

	//compute normals for each selected cloud
	if (!clouds.empty())
	{
		ccNormalComputationDlg::SelectionMode selectionMode = ccNormalComputationDlg::WITHOUT_SCAN_GRIDS;
		if (cloudsWithScanGrids)
		{
			if (clouds.size() == cloudsWithScanGrids)
			{
				//all clouds have an associated grid
				selectionMode = ccNormalComputationDlg::WITH_SCAN_GRIDS;
			}
			else
			{
				//only a part of the clouds have an associated grid
				selectionMode = ccNormalComputationDlg::MIXED;
			}
		}

		static CC_LOCAL_MODEL_TYPES s_lastModelType = LS;
		static ccNormalVectors::Orientation s_lastNormalOrientation = ccNormalVectors::UNDEFINED;
		static int s_lastMSTNeighborCount = 6;
		static int s_lastKernelSize = 2;

		ccNormalComputationDlg ncDlg(selectionMode, this);
		ncDlg.setLocalModel(s_lastModelType);
		ncDlg.setRadius(defaultRadius);
		ncDlg.setPreferredOrientation(s_lastNormalOrientation);
		ncDlg.setMSTNeighborCount(s_lastMSTNeighborCount);
		ncDlg.setGridKernelSize(s_lastKernelSize);
		if (clouds.size() == 1)
		{
			ncDlg.setCloud(clouds.front());
		}

		if (!ncDlg.exec())
			return;

		//normals computation
		CC_LOCAL_MODEL_TYPES model = s_lastModelType = ncDlg.getLocalModel();
		bool useGridStructure = cloudsWithScanGrids && ncDlg.useScanGridsForComputation();
		defaultRadius = ncDlg.getRadius();
		int kernelSize = s_lastKernelSize = ncDlg.getGridKernelSize();

		//normals orientation
		bool orientNormals = ncDlg.orientNormals();
		bool orientNormalsWithGrids = cloudsWithScanGrids && ncDlg.useScanGridsForOrientation();
		bool orientNormalsPreferred = ncDlg.usePreferredOrientation();
		ccNormalVectors::Orientation preferredOrientation = s_lastNormalOrientation = ncDlg.getPreferredOrientation();
		bool orientNormalsMST = ncDlg.useMSTOrientation();
		int mstNeighbors = s_lastMSTNeighborCount = ncDlg.getMSTNeighborCount();
		
		size_t errors = 0;
		for (size_t i=0; i<clouds.size(); i++)
		{
			ccPointCloud* cloud = clouds[i];
			assert(cloud);

			ccProgressDialog pDlg(true,this);

			bool result = false;
			bool orientNormalsForThisCloud = false;
			if (useGridStructure && cloud->gridCount())
			{
#if 0
				ccPointCloud* newCloud = new ccPointCloud("temp");
				newCloud->reserve(cloud->size());
				for (size_t gi=0; gi<cloud->gridCount(); ++gi)
				{
					const ccPointCloud::Grid::Shared& scanGrid = cloud->grid(gi);
					if (scanGrid && scanGrid->indexes.empty())
					{
						//empty grid, we skip it
						continue;
					}
					ccGLMatrixd toSensor = scanGrid->sensorPosition.inverse();

					const int* _indexGrid = &(scanGrid->indexes[0]);
					for (int j=0; j<static_cast<int>(scanGrid->h); ++j)
					{
						for (int i=0; i<static_cast<int>(scanGrid->w); ++i, ++_indexGrid)
						{
							if (*_indexGrid >= 0)
							{
								unsigned pointIndex = static_cast<unsigned>(*_indexGrid);
								const CCVector3* P = cloud->getPoint(pointIndex);
								CCVector3 Q = toSensor * (*P);
								newCloud->addPoint(Q);
							}
						}
					}

					addToDB(newCloud);
				}
#endif


				//compute normals with the associated scan grid(s)
				orientNormalsForThisCloud = orientNormals && orientNormalsWithGrids;
				result = cloud->computeNormalsWithGrids(model, kernelSize, orientNormalsForThisCloud, &pDlg);
			}
			else
			{
				//compute normals with the octree
				orientNormalsForThisCloud = orientNormals && (preferredOrientation != ccNormalVectors::UNDEFINED);
				result = cloud->computeNormalsWithOctree(model, orientNormals ? preferredOrientation : ccNormalVectors::UNDEFINED, defaultRadius, &pDlg);
			}

			//do we need to orient the normals? (this may have been already done if 'orientNormalsForThisCloud' is true)
			if (result && orientNormals && !orientNormalsForThisCloud)
			{
				if (cloud->gridCount() && orientNormalsWithGrids)
				{
					//we can still use the grid structure(s) to orient the normals!
					result = cloud->orientNormalsWithGrids();
				}
				else if (orientNormalsMST)
				{
					//use Minimum Spanning Tree to resolve normals direction
					result = cloud->orientNormalsWithMST(mstNeighbors, &pDlg);
				}
			}

			if (!result)
			{
				++errors;
			}

			cloud->prepareDisplayForRefresh();
		}

		if (errors != 0)
		{
			if (errors < clouds.size())
				ccConsole::Error("Failed to compute or orient the normals on some clouds! (see console)");
			else
				ccConsole::Error("Failed to compute or orient the normals! (see console)");
		}
	}

	//compute normals for each selected mesh
	if (!meshes.empty())
	{
		QMessageBox question(	QMessageBox::Question,
								"Mesh normals",
								"Compute per-vertex normals (smooth) or per-triangle (faceted)?",
								QMessageBox::NoButton,
								this);

		QPushButton* perVertexButton   = question.addButton("Per-vertex", QMessageBox::YesRole);
		QPushButton* perTriangleButton = question.addButton("Per-triangle", QMessageBox::NoRole);

		question.exec();
		
		bool computePerVertexNormals = (question.clickedButton() == perVertexButton);

		for (size_t i=0; i<meshes.size(); i++)
		{
			ccMesh* mesh = meshes[i];
			assert(mesh);
			
			//we remove temporarily the mesh as its normals may be removed (and they can be a child object)
			ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(mesh);
			mesh->clearTriNormals();
			mesh->showNormals(false);
			bool result = mesh->computeNormals(computePerVertexNormals);
			putObjectBackIntoDBTree(mesh,objContext);

			if (!result)
			{
				ccConsole::Error(QString("Failed to compute normals on mesh '%1'").arg(mesh->getName()));
				continue;
			}
			mesh->prepareDisplayForRefresh_recursive();
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionOrientNormalsMST()
{
	if (m_selectedEntities.empty())
	{
		ccConsole::Error("Select at least one point cloud");
		return;
	}

	bool ok;
	static unsigned s_defaultKNN = 6;
	unsigned kNN = static_cast<unsigned>(QInputDialog::getInt(0,"Neighborhood size", "Neighbors", s_defaultKNN , 1, 1000, 1, &ok));
	if (!ok)
		return;
	s_defaultKNN = kNN;

	ccProgressDialog pDlg(true,this);
	
	size_t errors = 0;
	for (size_t i=0; i<m_selectedEntities.size(); i++)
	{
		if (!m_selectedEntities[i]->isA(CC_TYPES::POINT_CLOUD))
			continue;

		ccPointCloud* cloud = static_cast<ccPointCloud*>(m_selectedEntities[i]);
		if (!cloud->hasNormals())
		{
			ccConsole::Warning(QString("Cloud '%1' has no normals!").arg(cloud->getName()));
			continue;
		}

		//use Minimum Spanning Tree to resolve normals direction
		if (cloud->orientNormalsWithMST(kNN, &pDlg))
		{
			cloud->prepareDisplayForRefresh();
		}
		else
		{
			ccConsole::Warning(QString("Process failed on cloud '%1'").arg(cloud->getName()));
			++errors;
		}
	}

	if (errors)
	{
		ccConsole::Error(QString("Process failed (check console)"));
	}
	else
	{
		ccLog::Warning("Normals have been oriented: you may still have to globally invert the cloud normals however (Edit > Normals > Invert).");
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionOrientNormalsFM()
{
	if (m_selectedEntities.empty())
	{
		ccConsole::Error("Select at least one point cloud");
		return;
	}

	bool ok;
	unsigned char s_defaultLevel = 6;
	int value = QInputDialog::getInt(this,"Orient normals (FM)", "Octree level", s_defaultLevel, 1, CCLib::DgmOctree::MAX_OCTREE_LEVEL, 1, &ok);
	if (!ok)
		return;

	assert(value >= 0 && value <= 255);
	unsigned char level = static_cast<unsigned char>(value);
	s_defaultLevel = level;

	ccProgressDialog pDlg(false,this);

	size_t errors = 0;
	for (size_t i=0; i<m_selectedEntities.size(); i++)
	{
		if (!m_selectedEntities[i]->isA(CC_TYPES::POINT_CLOUD))
			continue;

		ccPointCloud* cloud = static_cast<ccPointCloud*>(m_selectedEntities[i]);
		if (!cloud->hasNormals())
		{
			ccConsole::Warning(QString("Cloud '%1' has no normals!").arg(cloud->getName()));
			continue;
		}

		//orient normals with Fast Marching
		if (cloud->orientNormalsWithFM(level, &pDlg))
		{
			cloud->prepareDisplayForRefresh();
		}
		else
		{
			++errors;
		}
	}

	if (errors)
	{
		ccConsole::Error(QString("Process failed (check console)"));
	}
	else
	{
		ccLog::Warning("Normals have been oriented: you may still have to globally invert the cloud normals however (Edit > Normals > Invert).");
	}

	refreshAll();
	updateUI();
}

static int s_innerRectDim = 2;
void MainWindow::doActionFindBiggestInnerRectangle()
{
	size_t selNum = m_selectedEntities.size();
	if (selNum == 0)
		return;

	ccHObject* entity = m_selectedEntities.size() == 1 ? m_selectedEntities[0] : 0;
	if (!entity || !entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error("Select one point cloud!");
		return;
	}

	bool ok;
	int dim = QInputDialog::getInt(this,"Dimension","Orthogonal dim (X=0 / Y=1 / Z=2)",s_innerRectDim,0,2,1,&ok);
	if (!ok)
		return;
	s_innerRectDim = dim;

	ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(entity);
	ccBox* box = ccInnerRect2DFinder().process(cloud,static_cast<unsigned char>(dim));

	if (box)
	{
		cloud->addChild(box);
		box->setVisible(true);
		box->setDisplay(cloud->getDisplay());
		box->setDisplay(cloud->getDisplay());
		addToDB(box);
	}

	updateUI();
}

void MainWindow::doActionMatchBBCenters()
{
	size_t selNum = m_selectedEntities.size();

	//we need at least 2 entities
	if (selNum < 2)
		return;

	//we must backup 'm_selectedEntities' as removeObjectTemporarilyFromDBTree can modify it!
	ccHObject::Container selectedEntities = m_selectedEntities;

	//by default, we take the first entity as reference
	//TODO: maybe the user would like to select the reference himself ;)
	ccHObject* refEnt = selectedEntities[0];
	CCVector3 refCenter = refEnt->getBB_recursive().getCenter();

	for (size_t i=1; i<selNum; ++i)
	{
		ccHObject* ent = selectedEntities[i];
		CCVector3 center = ent->getBB_recursive().getCenter();

		CCVector3 T = refCenter-center;

		//transformation (used only for translation)
		ccGLMatrix glTrans;
		glTrans += T;

		forceConsoleDisplay();
		ccConsole::Print(QString("[Synchronize] Transformation matrix (%1 --> %2):").arg(ent->getName()).arg(selectedEntities[0]->getName()));
		ccConsole::Print(glTrans.toString(12,' ')); //full precision
		ccConsole::Print("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool");

		//we temporarily detach entity, as it may undergo
		//"severe" modifications (octree deletion, etc.) --> see ccHObject::applyGLTransformation
		ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(ent);
		ent->applyGLTransformation_recursive(&glTrans);
		putObjectBackIntoDBTree(ent,objContext);

		ent->prepareDisplayForRefresh_recursive();
	}

	//reselect previously selected entities!
	if (m_ccRoot)
		m_ccRoot->selectEntities(selectedEntities);

	zoomOnSelectedEntities();

	updateUI();
}

//semi-persistent parameters
static MainWindow::ScaleMatchingAlgorithm s_msAlgorithm = MainWindow::PCA_MAX_DIM;
static double s_msRmsDiff = 1.0e-5;
static int s_msFinalOverlap = 100;

void MainWindow::doActionMatchScales()
{
	size_t selNum = m_selectedEntities.size();

	//we need at least 2 entities
	if (selNum < 2)
		return;

	//we must select the point clouds and meshes
	ccHObject::Container selectedEntities;
	try
	{
		for (unsigned i=0; i<m_selectedEntities.size(); ++i)
		{
			ccHObject* ent = m_selectedEntities[i];
			if (	ent->isKindOf(CC_TYPES::POINT_CLOUD)
				||	ent->isKindOf(CC_TYPES::MESH))
			{
				selectedEntities.push_back(ent);
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccConsole::Error("Not enough memory!");
		return;
	}

	ccMatchScalesDlg msDlg(selectedEntities, 0, this);
	msDlg.setSelectedAlgorithm(s_msAlgorithm);
	msDlg.rmsDifferenceLineEdit->setText(QString::number(s_msRmsDiff,'e',1));
	msDlg.overlapSpinBox->setValue(s_msFinalOverlap);

	if (!msDlg.exec())
		return;

	//save semi-persistent parameters
	s_msAlgorithm = msDlg.getSelectedAlgorithm();
	if (s_msAlgorithm == MainWindow::ICP_SCALE)
	{
		s_msRmsDiff = msDlg.rmsDifferenceLineEdit->text().toDouble();
		s_msFinalOverlap = msDlg.overlapSpinBox->value();
	}

	ApplyScaleMatchingAlgortihm(s_msAlgorithm,
								selectedEntities,
								s_msRmsDiff,
								s_msFinalOverlap,
								msDlg.getSelectedIndex(),
								this);

	//reselect previously selected entities!
	if (m_ccRoot)
		m_ccRoot->selectEntities(selectedEntities);

	refreshAll();
	updateUI();
}

bool MainWindow::ApplyScaleMatchingAlgortihm(ScaleMatchingAlgorithm algo,
											ccHObject::Container& entities,
											double icpRmsDiff,
											int icpFinalOverlap,
											unsigned refEntityIndex/*=0*/,
											QWidget* parent/*=0*/)
{
	if (	entities.size() < 2
		||	refEntityIndex >= entities.size())
	{
		ccLog::Error("[ApplyScaleMatchingAlgortihm] Invalid input parameter(s)");
		return false;
	}
	
	std::vector<double> scales;
	try
	{
		scales.resize(entities.size(),-1.0);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("Not enough memory!");
		return false;
	}

	//check the reference entity
	ccHObject* refEntity = entities[refEntityIndex];
	if (	!refEntity->isKindOf(CC_TYPES::POINT_CLOUD)
		&&	!refEntity->isKindOf(CC_TYPES::MESH))
	{
		ccLog::Warning("[Scale Matching] The reference entity must be a cloud or a mesh!");
		return false;
	}

	unsigned count = static_cast<unsigned>(entities.size());

	//now compute the scales
	ccProgressDialog pDlg(true,parent);
	pDlg.setMethodTitle("Computing entities scales");
	pDlg.setInfo(qPrintable(QString("Entities: %1").arg(count)));
	CCLib::NormalizedProgress nProgress(&pDlg,2*count-1);
	pDlg.start();
	QApplication::processEvents();

	for (unsigned i=0; i<count; ++i)
	{
		ccHObject* ent = entities[i];
		//try to get the underlying cloud (or the vertices set for a mesh)
		bool lockedVertices;
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
		if (cloud && !lockedVertices)
		{
			switch (algo)
			{
			case BB_MAX_DIM:
			case BB_VOLUME:
				{
					ccBBox box = ent->getOwnBB();
					if (box.isValid())
						scales[i] = algo == BB_MAX_DIM ? box.getMaxBoxDim() : box.computeVolume();
					else
						ccLog::Warning(QString("[Scale Matching] Entity '%1' has an invalid bounding-box!").arg(ent->getName()));
				}
				break;

			case PCA_MAX_DIM:
				{
					CCLib::Neighbourhood Yk(cloud);
					if (!Yk.getLSPlane())
					{
						ccLog::Warning(QString("[Scale Matching] Failed to perform PCA on entity '%1'!").arg(ent->getName()));
						break;
					}
					//deduce the scale
					{
						const CCVector3* X = Yk.getLSPlaneX();
						const CCVector3* O = Yk.getGravityCenter();
						double minX = 0,maxX = 0;
						for (unsigned j=0; j<cloud->size(); ++j)
						{
							double x = (*cloud->getPoint(j) - *O).dot(*X);
							if (j != 0)
							{
								minX = std::min(x,minX);
								maxX = std::max(x,maxX);
							}
							else
							{
								minX = maxX = x;
							}
						}
						scales[i] = maxX-minX;
					}
				}
				break;

			case ICP_SCALE:
				{
					ccGLMatrix transMat;
					double finalError = 0.0;
					double finalScale = 1.0;
					unsigned finalPointCount = 0;
					int transformationFilters = 0; //CCLib::RegistrationTools::SKIP_ROTATION;

					if (ccRegistrationTools::ICP(
						ent,
						refEntity,
						transMat,
						finalScale,
						finalError,
						finalPointCount,
						icpRmsDiff,
						0,
						50000,
						false,
						CCLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE,
						true,
						icpFinalOverlap/100.0,
						false,
						false,
						transformationFilters,
						parent))
					{
						scales[i] = finalScale;
					}
					else
					{
						ccLog::Warning(QString("[Scale Matching] Failed to register entity '%1'!").arg(ent->getName()));
					}

				}
				break;

			default:
				assert(false);
				break;
			}
		}
		else if (cloud && lockedVertices)
		{
			//locked entities
			DisplayLockedVerticesWarning(ent->getName(),false);
		}
		else
		{
			//we need a cloud or a mesh
			ccLog::Warning(QString("[Scale Matching] Entity '%1' can't be rescaled this way!").arg(ent->getName()));
		}

		//if the reference entity is invalid!
		if (scales[i] <= 0 &&  i == refEntityIndex)
		{
			ccLog::Error("Reference entity has an invalid scale! Can't proceed.");
			return false;
		}

		if (!nProgress.oneStep())
		{
			//process cancelled by user
			return false;
		}
	}

	MainWindow* instance = dynamic_cast<MainWindow*>(parent);
	ccLog::Print(QString("[Scale Matching] Reference entity scale: %1").arg(scales[refEntityIndex]));

	//now we can rescale
	pDlg.setMethodTitle("Rescaling entities");
	{
		for (unsigned i=0; i<count; ++i)
		{
			if (i == refEntityIndex)
				continue;
			if (scales[i] < 0)
				continue;

			ccLog::Print(QString("[Scale Matching] Entity '%1' scale: %2").arg(entities[i]->getName()).arg(scales[i]));
			if (scales[i] <= ZERO_TOLERANCE)
			{
				ccLog::Warning("[Scale Matching] Entity scale is too small!");
				continue;
			}
			
			ccHObject* ent = entities[i];
		
			bool lockedVertices;
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
			if (!cloud || lockedVertices)
				continue;

			double scaled = 1.0;
			if (algo == ICP_SCALE)
				scaled = scales[i];
			else
				scaled = scales[refEntityIndex]/scales[i];
			PointCoordinateType scale_pc = static_cast<PointCoordinateType>(scaled);

			//we temporarily detach entity, as it may undergo
			//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::scale
			ccHObjectContext objContext;
			if (instance)
				objContext = instance->removeObjectTemporarilyFromDBTree(cloud);

			CCVector3 C = cloud->getOwnBB().getCenter();
			
			cloud->scale(	scale_pc,
							scale_pc,
							scale_pc,
							C );
			
			if (instance)
				instance->putObjectBackIntoDBTree(cloud,objContext);
			cloud->prepareDisplayForRefresh_recursive();

			//don't forget the 'global shift'!
			const CCVector3d& shift = cloud->getGlobalShift();
			cloud->setGlobalShift(shift*scaled);
			//DGM: nope! Not the global scale!
		}

		if (!nProgress.oneStep())
		{
			//process cancelled by user
			return false;
		}
	}

	return true;
}

static int s_sorFilterKnn = 6;
static double s_sorFilterNSigma = 1.0;
void MainWindow::doActionSORFilter()
{
	ccSORFilterDlg sorDlg(this);
	
	//set semi-persistent/dynamic parameters
	sorDlg.knnSpinBox->setValue(s_sorFilterKnn);
	sorDlg.nSigmaDoubleSpinBox->setValue(s_sorFilterNSigma);
	if (!sorDlg.exec())
		return;

	//update semi-persistent/dynamic parameters
	s_sorFilterKnn = sorDlg.knnSpinBox->value();
	s_sorFilterNSigma = sorDlg.nSigmaDoubleSpinBox->value();

	ccProgressDialog pDlg(true,this);

	size_t selNum = m_selectedEntities.size();
	bool firstCloud = true;
	
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = m_selectedEntities[i];

		//specific test for locked vertices
		bool lockedVertices;
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent,&lockedVertices);
		if (cloud && lockedVertices)
		{
			DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
			continue;
		}

		//computation
		CCLib::ReferenceCloud* selection = CCLib::CloudSamplingTools::sorFilter(cloud,
																				s_sorFilterKnn,
																				s_sorFilterNSigma,
																				0,
																				&pDlg);

		if (selection)
		{
			if (selection->size() == cloud->size())
			{
				ccLog::Warning(QString("[doActionFilterNoise] No points were removed from cloud '%1'").arg(cloud->getName()));
			}
			else
			{
				ccPointCloud* cleanCloud = cloud->partialClone(selection);
				if (cleanCloud)
				{
					cleanCloud->setName(cloud->getName()+QString(".clean"));
					cleanCloud->setDisplay(cloud->getDisplay());
					if (cloud->getParent())
						cloud->getParent()->addChild(cleanCloud);
					addToDB(cleanCloud);

					cloud->setEnabled(false);
					if (firstCloud)
					{
						ccConsole::Warning("Previously selected entities (sources) have been hidden!");
						firstCloud = false;
						m_ccRoot->selectEntity(cleanCloud,true);
					}
				}
				else
				{
					ccConsole::Warning(QString("[doActionFilterNoise] Not enough memory to create a clean version of cloud '%1'!").arg(cloud->getName()));
				}
			}
			
			delete selection;
			selection = 0;
		}
		else
		{
			//no points fall inside selection!
			ccConsole::Warning(QString("[doActionFilterNoise] Failed to apply the noise filter to cloud '%1'! (not enough memory?)").arg(cloud->getName()));
		}
	}

	refreshAll();
	updateUI();
}

static bool s_noiseFilterUseKnn = false;
static int s_noiseFilterKnn = 6;
static bool s_noiseFilterUseAbsError = false;
static double s_noiseFilterAbsError = 1.0;
static double s_noiseFilterNSigma = 1.0;
static bool s_noiseFilterRemoveIsolatedPoints = false;
void MainWindow::doActionFilterNoise()
{
	PointCoordinateType kernelRadius = GetDefaultCloudKernelSize(m_selectedEntities);

	ccNoiseFilterDlg noiseDlg(this);
	
	//set semi-persistent/dynamic parameters
	noiseDlg.radiusDoubleSpinBox->setValue(kernelRadius);
	noiseDlg.knnSpinBox->setValue(s_noiseFilterKnn);
	noiseDlg.nSigmaDoubleSpinBox->setValue(s_noiseFilterNSigma);
	noiseDlg.absErrorDoubleSpinBox->setValue(s_noiseFilterAbsError);
	noiseDlg.removeIsolatedPointsCheckBox->setChecked(s_noiseFilterRemoveIsolatedPoints);
	if (s_noiseFilterUseAbsError)
		noiseDlg.absErrorRadioButton->setChecked(true);
	else
		noiseDlg.relativeRadioButton->setChecked(true);
	if (s_noiseFilterUseKnn)
		noiseDlg.knnRadioButton->setChecked(true);
	else
		noiseDlg.radiusRadioButton->setChecked(true);

	if (!noiseDlg.exec())
		return;

	//update semi-persistent/dynamic parameters
	kernelRadius = static_cast<PointCoordinateType>(noiseDlg.radiusDoubleSpinBox->value());
	s_noiseFilterUseKnn = noiseDlg.knnRadioButton->isChecked();
	s_noiseFilterKnn = noiseDlg.knnSpinBox->value();
	s_noiseFilterUseAbsError = noiseDlg.absErrorRadioButton->isChecked();
	s_noiseFilterNSigma = noiseDlg.nSigmaDoubleSpinBox->value();
	s_noiseFilterAbsError = noiseDlg.absErrorDoubleSpinBox->value();
	s_noiseFilterRemoveIsolatedPoints = noiseDlg.removeIsolatedPointsCheckBox->isChecked();

	ccProgressDialog pDlg(true,this);

	size_t selNum = m_selectedEntities.size();
	bool firstCloud = true;
	
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = m_selectedEntities[i];

		//specific test for locked vertices
		bool lockedVertices;
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent,&lockedVertices);
		if (cloud && lockedVertices)
		{
			DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
			continue;
		}

		//computation
		CCLib::ReferenceCloud* selection = CCLib::CloudSamplingTools::noiseFilter(	cloud,
																					kernelRadius,
																					s_noiseFilterNSigma,
																					s_noiseFilterRemoveIsolatedPoints,
																					s_noiseFilterUseKnn,
																					s_noiseFilterKnn,
																					s_noiseFilterUseAbsError,
																					s_noiseFilterAbsError,
																					0,
																					&pDlg);

		if (selection)
		{
			if (selection->size() == cloud->size())
			{
				ccLog::Warning(QString("[doActionFilterNoise] No points were removed from cloud '%1'").arg(cloud->getName()));
			}
			else
			{
				ccPointCloud* cleanCloud = cloud->partialClone(selection);
				if (cleanCloud)
				{
					cleanCloud->setName(cloud->getName()+QString(".clean"));
					cleanCloud->setDisplay(cloud->getDisplay());
					if (cloud->getParent())
						cloud->getParent()->addChild(cleanCloud);
					addToDB(cleanCloud);

					cloud->setEnabled(false);
					if (firstCloud)
					{
						ccConsole::Warning("Previously selected entities (sources) have been hidden!");
						firstCloud = false;
						m_ccRoot->selectEntity(cleanCloud,true);
					}
				}
				else
				{
					ccConsole::Warning(QString("[doActionFilterNoise] Not enough memory to create a clean version of cloud '%1'!").arg(cloud->getName()));
				}
			}
			
			delete selection;
			selection = 0;
		}
		else
		{
			//no points fall inside selection!
			ccConsole::Warning(QString("[doActionFilterNoise] Failed to apply the noise filter to cloud '%1'! (not enough memory?)").arg(cloud->getName()));
		}
	}

	refreshAll();
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
		DisplayLockedVerticesWarning(m_selectedEntities[0]->getName(),true);
		return;
	}

	//for "real" point clouds only
	if (!cloud || !cloud->isA(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error("Method can't be applied on locked vertices or virtual point clouds!");
		return;
	}
	ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

	ccUnrollDlg unrollDlg(this);
	if (!unrollDlg.exec())
		return;

	int mode = unrollDlg.getType();
	PointCoordinateType radius = static_cast<PointCoordinateType>(unrollDlg.getRadius());
	double angle = unrollDlg.getAngle();
	unsigned char dim = (unsigned char)unrollDlg.getAxisDimension();
	CCVector3* pCenter = 0;
	CCVector3 center;
	if (mode == 1 || !unrollDlg.isAxisPositionAuto())
	{
		center = unrollDlg.getAxisPosition();
		pCenter = &center;
	}

	//We apply unrolling method
	ccProgressDialog pDlg(true,this);

	if (mode == 0)
		pc->unrollOnCylinder(radius,pCenter,dim,(CCLib::GenericProgressCallback*)&pDlg);
	else if (mode == 1)
		pc->unrollOnCone(radius,angle,center,dim,(CCLib::GenericProgressCallback*)&pDlg);
	else
		assert(false);

	ccGLWindow* win = static_cast<ccGLWindow*>(cloud->getDisplay());
	if (win)
		win->updateConstellationCenterAndZoom();
	updateUI();
}

ccGLWindow* MainWindow::getActiveGLWindow()
{
	if (!m_mdiArea)
	{
		return 0;
	}

	QMdiSubWindow *activeSubWindow = m_mdiArea->activeSubWindow();
	if (activeSubWindow)
	{
		return static_cast<ccGLWindow*>(activeSubWindow->widget());
	}
	else
	{
		QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
		if (!subWindowList.isEmpty())
		{
			return static_cast<ccGLWindow*>(subWindowList[0]->widget());
		}
	}

	return 0;
}

QMdiSubWindow* MainWindow::getMDISubWindow(ccGLWindow* win)
{
	QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
	for (int i=0; i<subWindowList.size(); ++i)
		if (static_cast<ccGLWindow*>(subWindowList[i]->widget()) == win)
			return subWindowList[i];

	//not found!
	return 0;
}

void MainWindow::zoomIn()
{
	ccGLWindow* win = MainWindow::getActiveGLWindow();
	if (win)
	{
		//we simulate a real wheel event
		win->onWheelEvent(15.0f);
	}
}

void MainWindow::zoomOut()
{
	ccGLWindow* win = MainWindow::getActiveGLWindow();
	if (win)
	{
		//we simulate a real wheel event
		win->onWheelEvent(-15.0f);
	}
}

ccGLWindow* MainWindow::new3DView()
{
	assert(m_ccRoot && m_mdiArea);

	//already existing window?
	QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
	ccGLWindow* otherWin = 0;
	if (!subWindowList.isEmpty())
		otherWin = static_cast<ccGLWindow*>(subWindowList[0]->widget());

	QGLFormat format = QGLFormat::defaultFormat();
	format.setStencil(false);
	format.setDoubleBuffer(true);
	format.setStereo(true);
	//format.setSwapInterval(1);
	ccGLWindow *view3D = new ccGLWindow(this,format,otherWin); //We share OpenGL contexts between windows!

	view3D->setMinimumSize(400,300);
	view3D->resize(500,400);

	m_mdiArea->addSubWindow(view3D);

	connect(view3D,	SIGNAL(entitySelectionChanged(int)),				m_ccRoot,	SLOT(selectEntity(int)));
	connect(view3D,	SIGNAL(entitiesSelectionChanged(std::set<int>)),	m_ccRoot,	SLOT(selectEntities(std::set<int>)));

	//'echo' mode
	connect(view3D,	SIGNAL(mouseWheelRotated(float)),					this,		SLOT(echoMouseWheelRotate(float)));
	connect(view3D,	SIGNAL(cameraDisplaced(float,float)),				this,		SLOT(echoCameraDisplaced(float,float)));
	connect(view3D,	SIGNAL(viewMatRotated(const ccGLMatrixd&)),			this,		SLOT(echoBaseViewMatRotation(const ccGLMatrixd&)));
	connect(view3D,	SIGNAL(cameraPosChanged(const CCVector3d&)),		this,		SLOT(echoCameraPosChanged(const CCVector3d&)));
	connect(view3D,	SIGNAL(pivotPointChanged(const CCVector3d&)),		this,		SLOT(echoPivotPointChanged(const CCVector3d&)));
	connect(view3D,	SIGNAL(pixelSizeChanged(float)),					this,		SLOT(echoPixelSizeChanged(float)));

	connect(view3D,	SIGNAL(destroyed(QObject*)),						this,		SLOT(prepareWindowDeletion(QObject*)));
	connect(view3D,	SIGNAL(filesDropped(const QStringList&)),			this,		SLOT(addToDBAuto(QStringList)), Qt::QueuedConnection); //DGM: we don't want to block the 'dropEvent' method of ccGLWindow instances!
	connect(view3D,	SIGNAL(newLabel(ccHObject*)),						this,		SLOT(handleNewLabel(ccHObject*)));
	connect(view3D,	SIGNAL(exclusiveFullScreenToggled(bool)),			this,		SLOT(onExclusiveFullScreenToggled(bool)));

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

static bool s_autoSaveGuiElementPos = true;
void MainWindow::doActionResetGUIElementsPos()
{
	QSettings settings;
	settings.remove(ccPS::MainWinGeom());
	settings.remove(ccPS::MainWinState());

	QMessageBox::information(this,"Restart","To finish the process, you'll have to close and restart CloudCompare");

	//to avoid saving them right away!
	s_autoSaveGuiElementPos = false;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	//if (m_uiFrozen)
	//{
	//	ccConsole::Error("Close current dialog/interactor first!");
	//	event->ignore();
	//}
	//else
	{
		if ((m_ccRoot && m_ccRoot->getRootEntity()->getChildrenNumber() == 0)
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

	if (s_autoSaveGuiElementPos)
		saveGUIElementsPos();
}

void MainWindow::saveGUIElementsPos()
{
	//save the state as settings
	QSettings settings;
	settings.setValue(ccPS::MainWinGeom(), saveGeometry());
	settings.setValue(ccPS::MainWinState(), saveState());
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
	for (size_t i=0; i<m_mdiDialogs.size(); ++i)
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

void MainWindow::unregisterMDIDialog(ccOverlayDialog* dlg)
{
	if (dlg)
	{
		std::vector<ccMDIDialogs>::iterator it = m_mdiDialogs.begin();
		while (it != m_mdiDialogs.end())
			if (it->dialog == dlg)
				break;
		if (it != m_mdiDialogs.end())
			m_mdiDialogs.erase(it);
		dlg->disconnect();
		dlg->stop(false);
		dlg->deleteLater();
	}
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
	for (size_t i=0; i<m_mdiDialogs.size(); ++i)
		placeMDIDialog(m_mdiDialogs[i]);
}

void MainWindow::toggleVisualDebugTraces()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->toggleDebugTrace();
		win->redraw(true, false);
	}
}

void MainWindow::toggleFullScreen(bool state)
{
	if (state)
		showFullScreen();
	else
		showNormal();
}

void MainWindow::toggleExclusiveFullScreen(bool state)
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->toggleExclusiveFullScreen(state);
	}
}

void MainWindow::doActionShawAboutDialog()
{
	QDialog* aboutDialog = new QDialog(this);

	Ui::AboutDialog ui;
	ui.setupUi(aboutDialog);

	QString ccVer = ccCommon::GetCCVersion();
	//add compilation info
	ccVer += QString("<br><i>Compiled with");
#if defined(_MSC_VER)
	ccVer += QString(" MSVC %1 and").arg(_MSC_VER);
#endif
	ccVer += QString(" Qt %1").arg(QT_VERSION_STR);
	ccVer += QString("</i>");
	QString htmlText = ui.textEdit->toHtml();
	QString enrichedHtmlText = htmlText.arg(ccVer);
	//ccLog::PrintDebug(htmlText);
	//ccLog::PrintDebug(ccVer);
	//ccLog::PrintDebug(enrichedHtmlText);

	ui.textEdit->setHtml(enrichedHtmlText);

	aboutDialog->exec();

	//delete aboutDialog; //Qt will take care of it? Anyway CC crash if we try to delete it now!
}

void MainWindow::doActionShowHelpDialog()
{
	QMessageBox::information(this, "Documentation", "Please visit http://www.cloudcompare.org/doc");
}

void MainWindow::freezeUI(bool state)
{
	//freeze standard plugins
	toolBarMainTools->setDisabled(state);
	toolBarSFTools->setDisabled(state);
	toolBarPluginTools->setDisabled(state);
	//toolBarGLFilters->setDisabled(state);
	//toolBarView->setDisabled(state);
	
	//freeze plugin toolbars
	{
		for (int i=0; i<m_stdPluginsToolbars.size(); ++i)
			m_stdPluginsToolbars[i]->setDisabled(state);
	}

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
	if (selNum == 0 || selNum > 2)
	{
		ccConsole::Error("Select one or two entities (point cloud or mesh)!");
		return;
	}

	ccHObject* aligned = m_selectedEntities[0];
	ccHObject* reference = m_selectedEntities.size() > 1 ? m_selectedEntities[1] : 0;

	ccGenericPointCloud* cloud1 = ccHObjectCaster::ToGenericPointCloud(aligned);
	ccGenericPointCloud* cloud2 = (reference ? ccHObjectCaster::ToGenericPointCloud(reference) : 0);
	if (!cloud1 || (m_selectedEntities.size() > 1 && !cloud2))
	{
		ccConsole::Error("Select point clouds or meshes only!");
		return;
	}
	
	//if we have 2 entities, we must ask the user which one is the 'aligned' one and which one is the 'reference' one
	if (reference)
	{
		ccOrderChoiceDlg dlg(	m_selectedEntities[0], "Aligned",
								m_selectedEntities[1], "Reference",
								this );
		if (!dlg.exec())
			return;

		aligned = dlg.getFirstEntity();
		reference = dlg.getSecondEntity();
	}

	//we disable all windows
	disableAllBut(0);

	if (!m_pprDlg)
	{
		m_pprDlg = new ccPointPairRegistrationDlg(this);
		connect(m_pprDlg, SIGNAL(processFinished(bool)), this, SLOT(deactivateRegisterPointPairTool(bool)));
		registerMDIDialog(m_pprDlg,Qt::TopRightCorner);
	}

	ccGLWindow* win = new3DView();
	if (!win)
	{
		ccLog::Error("[PointPairRegistration] Failed to create dedicated 3D view!");
		return;
	}

	if (!m_pprDlg->init(win,aligned,reference))
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

void MainWindow::activateSectionExtractionMode()
{
	size_t selNum = m_selectedEntities.size();
	if (selNum == 0)
		return;

	if (!m_seTool)
	{
		m_seTool = new ccSectionExtractionTool(this);
		connect(m_seTool, SIGNAL(processFinished(bool)), this, SLOT(deactivateSectionExtractionMode(bool)));

		registerMDIDialog(m_seTool,Qt::TopRightCorner);
	}

	//add clouds
	unsigned validCount = 0;
	ccGLWindow* firstDisplay = 0;
	for (size_t i=0; i<selNum; ++i)
		if (m_selectedEntities[i]->isKindOf(CC_TYPES::POINT_CLOUD))
			if (m_seTool->addCloud(static_cast<ccGenericPointCloud*>(m_selectedEntities[i])))
			{
				if (!firstDisplay && m_selectedEntities[i]->getDisplay())
					firstDisplay = static_cast<ccGLWindow*>(m_selectedEntities[i]->getDisplay());
				++validCount;
			}

	if (validCount == 0)
	{
		ccConsole::Error("No cloud in selection!");
		return;
	}

	//deselect all entities
	if (m_ccRoot)
		m_ccRoot->unselectAllEntities();

	ccGLWindow* win = new3DView();
	if (!win)
	{
		ccLog::Error("[PointPairRegistration] Failed to create dedicated 3D view!");
		return;
	}
	if (firstDisplay && firstDisplay->getGlFilter())
	{
		win->setGlFilter(firstDisplay->getGlFilter()->clone());
	}
	m_seTool->linkWith(win);

	freezeUI(true);
	toolBarView->setDisabled(true);

	//we disable all other windows
	disableAllBut(win);

	if (!m_seTool->start())
		deactivateSectionExtractionMode(false);
	else
		updateMDIDialogsPlacement();
}

void MainWindow::deactivateSectionExtractionMode(bool state)
{
	if (m_seTool)
		m_seTool->removeAllEntities();

	//we enable all GL windows
	enableAll();

	QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
	if (!subWindowList.isEmpty())
		subWindowList[0]->showMaximized();

	freezeUI(false);
	toolBarView->setDisabled(false);

	updateUI();

	ccGLWindow* win = getActiveGLWindow();
	if (win)
		win->redraw();
}

void MainWindow::activateSegmentationMode()
{
	ccGLWindow* win = getActiveGLWindow();
	if (!win)
		return;

	size_t selNum = m_selectedEntities.size();
	if (selNum == 0)
		return;

	if (!m_gsTool)
	{
		m_gsTool = new ccGraphicalSegmentationTool(this);
		connect(m_gsTool, SIGNAL(processFinished(bool)), this, SLOT(deactivateSegmentationMode(bool)));

		registerMDIDialog(m_gsTool,Qt::TopRightCorner);
	}

	m_gsTool->linkWith(win);

	for (size_t i=0; i<selNum; ++i)
		m_gsTool->addEntity(m_selectedEntities[i]);

	if (m_gsTool->getNumberOfValidEntities() == 0)
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
	bool deleteHiddenParts = false;

	//shall we apply segmentation?
	if (state)
	{
		ccHObject* firstResult = 0;

		deleteHiddenParts = m_gsTool->deleteHiddenParts();

		//aditional vertices of which visibility array should be manually reset
		std::set<ccGenericPointCloud*> verticesToReset;

		for (std::set<ccHObject*>::const_iterator p = m_gsTool->entities().begin(); p != m_gsTool->entities().end(); ++p)
		{
			ccHObject* entity = *p;

			if (entity->isKindOf(CC_TYPES::POINT_CLOUD) || entity->isKindOf(CC_TYPES::MESH))
			{
				//first, do the things that must absolutely be done BEFORE removing the entity from DB (even temporarily)
				//bool lockedVertices;
				ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity/*,&lockedVertices*/);
				assert(cloud);
				if (cloud)
				{
					//assert(!lockedVertices); //in some cases we accept to segment meshes with locked vertices!
					
					//specific case: labels (do this before temporarily removing 'entity' from DB!)
					ccHObject::Container labels;
					if (m_ccRoot)
						m_ccRoot->getRootEntity()->filterChildren(labels,true,CC_TYPES::LABEL_2D);
					for (ccHObject::Container::iterator it=labels.begin(); it!=labels.end(); ++it)
					{
						if ((*it)->isA(CC_TYPES::LABEL_2D)) //Warning: cc2DViewportLabel is also a kind of 'CC_TYPES::LABEL_2D'!
						{
							//we must search for all dependent labels and remove them!!!
							//TODO: couldn't we be more clever and update the label instead?
							cc2DLabel* label = static_cast<cc2DLabel*>(*it);
							bool removeLabel = false;
							for (unsigned i=0; i<label->size(); ++i)
							{
								if (label->getPoint(i).cloud == entity)
								{
									removeLabel = true;
									break;
								}
							}

							if (removeLabel && label->getParent())
							{
								ccLog::Warning(QString("[Segmentation] Label %1 is dependent on cloud %2 and will be removed").arg(label->getName()).arg(cloud->getName()));
								ccHObject* labelParent = label->getParent();
								ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(labelParent);
								labelParent->removeChild(label);
								label = 0;
								putObjectBackIntoDBTree(labelParent,objContext);
							}
						}
					} //for each label
				} // if (cloud)

				//we temporarily detach the entity, as it may undergo
				//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::createNewCloudFromVisibilitySelection
				ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(entity);

				//apply segmentation
				ccHObject* segmentationResult = 0;
				bool deleteOriginalEntity = deleteHiddenParts;
				if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
				{
					ccGenericPointCloud* genCloud = ccHObjectCaster::ToGenericPointCloud(entity);
					segmentationResult = genCloud->createNewCloudFromVisibilitySelection(!deleteHiddenParts);

					deleteOriginalEntity |= (genCloud->size() == 0);
				}
				else if (entity->isKindOf(CC_TYPES::MESH)/*|| entity->isA(CC_TYPES::PRIMITIVE)*/) //TODO
				{
					if (entity->isA(CC_TYPES::MESH))
					{
						segmentationResult = ccHObjectCaster::ToMesh(entity)->createNewMeshFromSelection(!deleteHiddenParts);
					}
					else if (entity->isA(CC_TYPES::SUB_MESH))
					{
						segmentationResult = ccHObjectCaster::ToSubMesh(entity)->createNewSubMeshFromSelection(!deleteHiddenParts);
					}

					deleteOriginalEntity |=  (ccHObjectCaster::ToGenericMesh(entity)->size() == 0);
				}

				if (segmentationResult)
				{
					assert(cloud);
					if (cloud)
					{
						//another specific case: sensors (on clouds)
						for (unsigned i=0; i<entity->getChildrenNumber(); ++i)
						{
							ccHObject* child = entity->getChild(i);
							assert(child);
							if (child && child->isKindOf(CC_TYPES::SENSOR))
							{
								if (child->isA(CC_TYPES::GBL_SENSOR))
								{
									ccGBLSensor* sensor = ccHObjectCaster::ToGBLSensor(entity->getChild(i));
									//remove the associated depth buffer of the original sensor (derpecated)
									sensor->clearDepthBuffer();
									if (deleteOriginalEntity)
										//either transfer
											entity->transferChild(sensor,*segmentationResult);
									else
										//or copy
										segmentationResult->addChild(new ccGBLSensor(*sensor));
								}
								else if (child->isA(CC_TYPES::CAMERA_SENSOR))
								{
									ccCameraSensor* sensor = ccHObjectCaster::ToCameraSensor(entity->getChild(i));
									if (deleteOriginalEntity)
										//either transfer
											entity->transferChild(sensor,*segmentationResult);
									else
										//or copy
										segmentationResult->addChild(new ccCameraSensor(*sensor));
								}
								else
								{
									//unhandled sensor?!
									assert(false);
								}
							}
						} //for each child
					}

					//we must take care of the remaining part
					if (!deleteHiddenParts)
					{
						//no need to put back the entity in DB if we delete it afterwards!
						if (!deleteOriginalEntity)
						{
							entity->setName(entity->getName() + QString(".remaining"));
							putObjectBackIntoDBTree(entity,objContext);
						}
					}
					else
					{
						//keep original name(s)
						segmentationResult->setName(entity->getName());
						if (entity->isKindOf(CC_TYPES::MESH) && segmentationResult->isKindOf(CC_TYPES::MESH))
						{
							ccGenericMesh* meshEntity = ccHObjectCaster::ToGenericMesh(entity);
							ccHObjectCaster::ToGenericMesh(segmentationResult)->getAssociatedCloud()->setName(meshEntity->getAssociatedCloud()->getName());

							//specific case: if the sub mesh is deleted afterwards (see below)
							//then its associated vertices won't be 'reset' by the segmentation tool!
							if (deleteHiddenParts && meshEntity->isA(CC_TYPES::SUB_MESH))
								verticesToReset.insert(meshEntity->getAssociatedCloud());
						}
						assert(deleteOriginalEntity);
						//deleteOriginalEntity = true;
					}

					if (segmentationResult->isA(CC_TYPES::SUB_MESH))
					{
						//for sub-meshes, we have no choice but to use its parent mesh!
						objContext.parent = static_cast<ccSubMesh*>(segmentationResult)->getAssociatedMesh();
					}
					else
					{
						//otherwise we look for first non-mesh or non-cloud parent
						while (objContext.parent && (objContext.parent->isKindOf(CC_TYPES::MESH) || objContext.parent->isKindOf(CC_TYPES::POINT_CLOUD)))
							objContext.parent = objContext.parent->getParent();
					}

					if (objContext.parent)
						objContext.parent->addChild(segmentationResult); //FiXME: objContext.parentFlags?

					segmentationResult->setDisplay_recursive(entity->getDisplay());
					segmentationResult->prepareDisplayForRefresh_recursive();

					addToDB(segmentationResult);

					if (!firstResult)
						firstResult = segmentationResult;
				}
				else if (!deleteOriginalEntity)
				{
					//ccConsole::Error("An error occurred! (not enough memory?)");
					putObjectBackIntoDBTree(entity,objContext);
				}
				
				if (deleteOriginalEntity)
				{
					delete entity;
					entity = 0;
				}
			}
		}

		//specific actions
		{
			for (std::set<ccGenericPointCloud*>::iterator p = verticesToReset.begin(); p != verticesToReset.end(); ++p)
			{
				(*p)->resetVisibilityArray();
			}
		}

		if (firstResult && m_ccRoot)
			m_ccRoot->selectEntity(firstResult);
	}

	if (m_gsTool)
		m_gsTool->removeAllEntities(!deleteHiddenParts);

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
		connect(m_plpDlg, SIGNAL(processFinished(bool)), this, SLOT(deactivatePointListPickingMode(bool)));

		registerMDIDialog(m_plpDlg,Qt::TopRightCorner);
	}

	//DGM: we must update marker size spin box value (as it may have changed by the user with the "display dialog")
	m_plpDlg->markerSizeSpinBox->setValue(win->getDisplayParameters().labelMarkerSize);

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

void MainWindow::activatePointPickingMode()
{
	ccGLWindow* win = getActiveGLWindow();
	if (!win)
		return;

	if (m_ccRoot)
		m_ccRoot->unselectAllEntities(); //we don't want any entity selected (especially existing labels!)

	if (!m_ppDlg)
	{
		m_ppDlg = new ccPointPropertiesDlg(this);
		connect(m_ppDlg, SIGNAL(processFinished(bool)),	this, SLOT(deactivatePointPickingMode(bool)));
		connect(m_ppDlg, SIGNAL(newLabel(ccHObject*)),	this, SLOT(handleNewLabel(ccHObject*)));

		registerMDIDialog(m_ppDlg,Qt::TopRightCorner);
	}

	m_ppDlg->linkWith(win);

	freezeUI(true);

	//we disable all other windows
	disableAllBut(win);

	if (!m_ppDlg->start())
		deactivatePointPickingMode(false);
	else
		updateMDIDialogsPlacement();
}

void MainWindow::deactivatePointPickingMode(bool state)
{
	//if (m_ppDlg)
	//	m_ppDlg->linkWith(0);

	//we enable all GL windows
	enableAll();

	freezeUI(false);

	updateUI();
}

void MainWindow::activateClippingBoxMode()
{
	size_t selNum = m_selectedEntities.size();
	if (selNum == 0)
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
	size_t selNum = m_selectedEntities.size();
	if (selNum == 0)
		return;

	ccGLWindow* win = getActiveGLWindow();
	if (!win)
		return;

	if (!m_transTool)
		m_transTool = new ccGraphicalTransformationTool(this);
	assert(m_transTool->getNumberOfValidEntities() == 0);
	m_transTool->linkWith(win);

	bool rejectedEntities = false;
	for (size_t i=0; i<selNum;++i)
	{
		ccHObject* entity = m_selectedEntities[i];
		if (!m_transTool->addEntity(entity))
			rejectedEntities = true;
	}

	if (m_transTool->getNumberOfValidEntities() == 0)
	{
		ccConsole::Error("No entity eligible for manual transformation! (see console)");
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
	if (m_transTool)
	{
		//reselect previously selected entities!
		if (state && m_ccRoot)
		{
			const ccHObject& transformedSet = m_transTool->getValidEntities();
			try
			{
				ccHObject::Container transformedEntities;
				transformedEntities.resize(transformedSet.getChildrenNumber());
				for (unsigned i=0; i<transformedSet.getChildrenNumber(); ++i)
				{
					transformedEntities[i] = transformedSet.getChild(i);
				}
				m_ccRoot->selectEntities(transformedEntities);
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory (nothing to do)
			}
		}
		//m_transTool->close();
	}

	//we reactivate all GL windows
	enableAll();

	freezeUI(false);

	updateUI();
}

void MainWindow::testFrameRate()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
		win->startFrameRateTest();
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
		win->renderToFile(qPrintable(rtfDlg.getFilename()),rtfDlg.getZoom(),rtfDlg.dontScalePoints(),rtfDlg.renderOverlayItems());
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
		
		connect(m_mdiArea, SIGNAL(subWindowActivated(QMdiSubWindow*)), m_cpeDlg, SLOT(linkWith(QMdiSubWindow*)));

		registerMDIDialog(m_cpeDlg,Qt::BottomLeftCorner);
	}

	m_cpeDlg->linkWith(qWin);
	m_cpeDlg->start();

	updateMDIDialogsPlacement();
}

void MainWindow::doActionAdjustZoom()
{
	//current active MDI area
	ccGLWindow* win = getActiveGLWindow();
	if (!win)
		return;

	const ccViewportParameters& params = win->getViewportParameters();
	if (params.perspectiveView)
	{
		ccConsole::Error("Orthographic mode only!");
		return;
	}

	ccAdjustZoomDlg azDlg(win,this);

	if (!azDlg.exec())
		return;

	//apply zoom
	double zoom = azDlg.getZoom();
	win->setZoom(static_cast<float>(zoom));
	win->redraw();
}

static unsigned s_viewportIndex = 0;
void MainWindow::doActionSaveViewportAsCamera()
{
	ccGLWindow* win = getActiveGLWindow();
	if (!win)
		return;

	cc2DViewportObject* viewportObject = new cc2DViewportObject(QString("Viewport #%1").arg(++s_viewportIndex));
	viewportObject->setParameters(win->getViewportParameters());
	viewportObject->setDisplay(win);

	addToDB(viewportObject);
}

void MainWindow::setGlobalZoom()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
		win->zoomGlobal();
}

void MainWindow::zoomOnSelectedEntities()
{
	ccGLWindow* win = 0;

	ccHObject tempGroup("TempGroup");
	size_t selNum = m_selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		if (i == 0 || !win)
		{
			//take the first valid window as reference
			win = static_cast<ccGLWindow*>(m_selectedEntities[i]->getDisplay());
		}

		if (win)
		{
			if (m_selectedEntities[i]->getDisplay() == win)
			{
				tempGroup.addChild(m_selectedEntities[i],ccHObject::DP_NONE);
			}
			else if (m_selectedEntities[i]->getDisplay() != 0)
			{
				ccLog::Error("All selected entities must be displayed in the same 3D view!");
				return;
			}
		}
	}

	if (tempGroup.getChildrenNumber() != 0)
	{
		ccBBox box = tempGroup.getDisplayBB_recursive(false, win);
		if (!box.isValid())
		{
			ccLog::Warning("Selected entities have no valid bounding-box!");
		}
		else
		{
			win->updateConstellationCenterAndZoom(&box);
		}
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
	setOrthoView(getActiveGLWindow());
}

void MainWindow::setOrthoView(ccGLWindow* win)
{
	if (win)
	{
		if (!checkStereoMode(win))
		{
			return;
		}
		win->setPerspectiveState(false,true);
		win->redraw();

		//update pop-up menu 'top' icon
		if (m_viewModePopupButton)
			m_viewModePopupButton->setIcon(actionSetOrthoView->icon());
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setEnabled(true);
	}
}

void MainWindow::setCenteredPerspectiveView()
{
	setCenteredPerspectiveView(getActiveGLWindow());
}

void MainWindow::setCenteredPerspectiveView(ccGLWindow* win, bool autoRedraw/*=true*/)
{
	if (win)
	{
		win->setPerspectiveState(true,true);
		if (autoRedraw)
			win->redraw();

		//update pop-up menu 'top' icon
		if (m_viewModePopupButton)
			m_viewModePopupButton->setIcon(actionSetCenteredPerspectiveView->icon());
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setEnabled(true);
	}
}

void MainWindow::setViewerPerspectiveView()
{
	setViewerPerspectiveView(getActiveGLWindow());
}

void MainWindow::setViewerPerspectiveView(ccGLWindow* win)
{
	if (win)
	{
		win->setPerspectiveState(true,false);
		win->redraw();

		//update pop-up menu 'top' icon
		if (m_viewModePopupButton)
			m_viewModePopupButton->setIcon(actionSetViewerPerspectiveView->icon());
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setEnabled(false);
	}
}

//globals for point picking operations
enum PickingOperation {	NO_PICKING_OPERATION,
						PICKING_ROTATION_CENTER,
						PICKING_LEVEL_POINTS,
};
static ccGLWindow* s_pickingWindow = 0;
static ccGLWindow::PICKING_MODE s_previousPickingMode = ccGLWindow::NO_PICKING;
static PickingOperation s_previousPickingOperation = NO_PICKING_OPERATION;
static std::vector<cc2DLabel*> s_levelLabels;
static ccPointCloud* s_levelMarkersCloud = 0;
static ccHObject* s_levelEntity = 0;

void MainWindow::enablePickingOperation(ccGLWindow* win, QString message)
{
	if (!win)
	{
		assert(false);
		return;
	}

	//specific case: we prevent the 'point-pair based alignment' tool to process the picked point!
	if (m_pprDlg)
		m_pprDlg->pause(true);

	connect(win, SIGNAL(itemPicked(int, unsigned, int, int)), this, SLOT(processPickedPoint(int, unsigned, int, int)));
	s_pickingWindow = win;
	s_previousPickingMode = win->getPickingMode();
	win->setPickingMode(ccGLWindow::POINT_OR_TRIANGLE_PICKING); //points or triangles
	win->displayNewMessage(message,ccGLWindow::LOWER_LEFT_MESSAGE,true,24*3600);
	win->redraw(true, false);

	freezeUI(true);
}

void MainWindow::cancelPreviousPickingOperation(bool aborted)
{
	if (!s_pickingWindow)
		return;

	switch(s_previousPickingOperation)
	{
	case PICKING_ROTATION_CENTER:
		//nothing to do
		break;
	case PICKING_LEVEL_POINTS:
		if (s_levelMarkersCloud)
		{
			s_pickingWindow->removeFromOwnDB(s_levelMarkersCloud);
			delete s_levelMarkersCloud;
			s_levelMarkersCloud = 0;
		}
		break;
	default:
		assert(false);
		break;
	}

	if (aborted)
	{
		s_pickingWindow->displayNewMessage(QString(),ccGLWindow::LOWER_LEFT_MESSAGE); //clear previous messages
		s_pickingWindow->displayNewMessage("Picking operation aborted",ccGLWindow::LOWER_LEFT_MESSAGE);
	}
	s_pickingWindow->redraw(false);

	//specific case: we allow the 'point-pair based alignment' tool to process the picked point!
	if (m_pprDlg)
		m_pprDlg->pause(false);

	freezeUI(false);

	disconnect(s_pickingWindow, SIGNAL(itemPicked(int, unsigned, int, int)), this, SLOT(processPickedPoint(int, unsigned, int, int)));
	//restore previous picking mode
	s_pickingWindow->setPickingMode(s_previousPickingMode);
	s_pickingWindow = 0;
	s_previousPickingOperation = NO_PICKING_OPERATION;
}

void MainWindow::processPickedPoint(int cloudUniqueID, unsigned itemIndex, int x, int y)
{
	if (!s_pickingWindow)
		return;

	ccHObject* db = s_pickingWindow->getSceneDB();
	ccHObject* obj = db ? db->find(cloudUniqueID) : 0;
	if (!obj)
		return;

	CCVector3 pickedPoint;
	if (obj->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(obj);
		if (!cloud)
		{
			assert(false);
			return;
		}
		pickedPoint = *cloud->getPoint(itemIndex);
	}
	else if (obj->isKindOf(CC_TYPES::MESH))
	{
		ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(obj);
		if (!mesh)
		{
			assert(false);
			return;
		}
		CCLib::GenericTriangle* tri = mesh->_getTriangle(itemIndex);
		pickedPoint = s_pickingWindow->backprojectPointOnTriangle(CCVector2i(x,y),*tri->_getA(),*tri->_getB(),*tri->_getC());
	}
	else
	{
		//unhandled entity
		ccLog::Warning(QString("[Picking] Can't use points picked on this entity ('%1')!").arg(obj->getName()));
		assert(false);
		return;
	}
	
	switch(s_previousPickingOperation)
	{
	case PICKING_LEVEL_POINTS:
		{
			//we only accept points picked on the right entity!
			//if (obj != s_levelEntity)
			//{
			//	ccLog::Warning(QString("[Level] Only points picked on '%1' are considered!").arg(s_levelEntity->getName()));
			//	return;
			//}

			if (!s_levelMarkersCloud)
			{
				assert(false);
				cancelPreviousPickingOperation(true);
			}

			for (unsigned i=0; i<s_levelMarkersCloud->size(); ++i)
			{
				const CCVector3* P = s_levelMarkersCloud->getPoint(i);
				if ((pickedPoint - *P).norm() < 1.0e-6)
				{
					ccLog::Warning("[Level] Point is too close from the others!");
					return;
				}
			}

			//add the corresponding marker
			s_levelMarkersCloud->addPoint(pickedPoint);
			unsigned markerCount = s_levelMarkersCloud->size();
			cc2DLabel* label = new cc2DLabel();
			label->addPoint(s_levelMarkersCloud,markerCount-1);
			label->setName(QString("P#%1").arg(markerCount));
			label->setDisplayedIn2D(false);
			label->setDisplay(s_pickingWindow);
			label->setVisible(true);
			s_levelMarkersCloud->addChild(label);
			s_pickingWindow->redraw();

			if (markerCount == 3)
			{
				//we have enough points!
				const CCVector3* A = s_levelMarkersCloud->getPoint(0);
				const CCVector3* B = s_levelMarkersCloud->getPoint(1);
				const CCVector3* C = s_levelMarkersCloud->getPoint(2);
				CCVector3 X = *B-*A;
				CCVector3 Y = *C-*A;
				CCVector3 Z = X.cross(Y);
				//we choose 'Z' so that it points 'upward' relatively to the camera (assuming the user will be looking from the top)
				CCVector3d viewDir = s_pickingWindow->getCurrentViewDir();
				if (CCVector3d::fromArray(Z.u).dot(viewDir) > 0)
				{
					Z = -Z;
				}
				Y = Z.cross(X);
				X.normalize();
				Y.normalize();
				Z.normalize();
			
				ccGLMatrixd trans;
				double* mat = trans.data();
				mat[0] = X.x; mat[4] = X.y; mat[8]  = X.z; mat[12] = 0;
				mat[1] = Y.x; mat[5] = Y.y; mat[9]  = Y.z; mat[13] = 0;
				mat[2] = Z.x; mat[6] = Z.y; mat[10] = Z.z; mat[14] = 0;
				mat[3] = 0  ; mat[7] = 0  ; mat[11] = 0  ; mat[15] = 1;

				CCVector3d T = -CCVector3d::fromArray(A->u);
				trans/*.inverse()*/.apply(T);
				T += CCVector3d::fromArray(A->u);
				trans.setTranslation(T);

				assert(m_selectedEntities.size() == 1 && m_selectedEntities.front() == s_levelEntity);
				applyTransformation(trans);

				//clear message
				s_pickingWindow->displayNewMessage(QString(),ccGLWindow::LOWER_LEFT_MESSAGE,false); //clear previous message
				s_pickingWindow->setView(CC_TOP_VIEW);
			}
			else
			{
				//we need more points!
				return;
			}
		}
		//we use the next 'case' entry (PICKING_ROTATION_CENTER) to redefine the rotation center as well!
		assert(s_levelMarkersCloud && s_levelMarkersCloud->size() != 0);
		pickedPoint = *s_levelMarkersCloud->getPoint(0);
		//break;
	
	case PICKING_ROTATION_CENTER:
		{
			CCVector3d newPivot = CCVector3d::fromArray(pickedPoint.u);
			//specific case: transformation tool is enabled
			if (m_transTool && m_transTool->started())
			{
				m_transTool->setRotationCenter(newPivot);
				const unsigned& precision = s_pickingWindow->getDisplayParameters().displayedNumPrecision;
				s_pickingWindow->displayNewMessage(QString(),ccGLWindow::LOWER_LEFT_MESSAGE,false); //clear previous message
				s_pickingWindow->displayNewMessage(	QString("Point (%1,%2,%3) set as rotation center for interactive transformation")
														.arg(pickedPoint.x,0,'f',precision)
														.arg(pickedPoint.y,0,'f',precision)
														.arg(pickedPoint.z,0,'f',precision),
													ccGLWindow::LOWER_LEFT_MESSAGE,true);
			}
			else
			{
				const ccViewportParameters& params = s_pickingWindow->getViewportParameters();
				if (!params.perspectiveView || params.objectCenteredView)
				{
					//apply current GL transformation (if any)
					obj->getGLTransformation().apply(newPivot);
					//compute the equivalent camera center
					CCVector3d dP = params.pivotPoint - newPivot;
					CCVector3d MdP = dP; params.viewMat.applyRotation(MdP);
					CCVector3d newCameraPos = params.cameraCenter + MdP - dP;
					s_pickingWindow->setCameraPos(newCameraPos);
					s_pickingWindow->setPivotPoint(newPivot);

					const unsigned& precision = s_pickingWindow->getDisplayParameters().displayedNumPrecision;
					s_pickingWindow->displayNewMessage(QString(),ccGLWindow::LOWER_LEFT_MESSAGE,false); //clear previous message
					s_pickingWindow->displayNewMessage(	QString("Point (%1,%2,%3) set as rotation center")
															.arg(pickedPoint.x,0,'f',precision)
															.arg(pickedPoint.y,0,'f',precision)
															.arg(pickedPoint.z,0,'f',precision),
														ccGLWindow::LOWER_LEFT_MESSAGE,true);
				}
			}
			//s_pickingWindow->redraw(); //already called by 'cancelPreviousPickingOperation' (see below)
		}
		break;
	
	default:
		assert(false);
		break;
	}

	cancelPreviousPickingOperation(false);
}

void MainWindow::doLevel()
{
	//picking operation already in progress
	if (s_pickingWindow)
	{
		if (s_previousPickingOperation == PICKING_LEVEL_POINTS)
		{
			cancelPreviousPickingOperation(true);
		}
		else
		{
			ccConsole::Error("Stop the other picking operation first!");
		}
		return;
	}

	ccGLWindow* win = getActiveGLWindow();
	if (!win)
	{
		ccConsole::Error("No active 3D view!");
		return;
	}

	size_t selNum = m_selectedEntities.size();
	if (selNum != 1)
	{
		ccConsole::Error("Select an entity!");
		return;
	}

	//create markers cloud
	assert(!s_levelMarkersCloud);
	{
		s_levelMarkersCloud = new ccPointCloud("Level points");
		if (!s_levelMarkersCloud->reserve(3))
		{
			ccConsole::Error("Not enough memory!");
			return;
		}
		win->addToOwnDB(s_levelMarkersCloud);
	}

	s_levelEntity = m_selectedEntities[0];
	s_levelLabels.clear();
	s_previousPickingOperation = PICKING_LEVEL_POINTS;

	enablePickingOperation(win,"Pick three points on the floor plane (click on icon/menu entry again to cancel)");
}

void MainWindow::doPickRotationCenter()
{
	//picking operation already in progress
	if (s_pickingWindow)
	{
		if (s_previousPickingOperation == PICKING_ROTATION_CENTER)
		{
			cancelPreviousPickingOperation(true);
		}
		else
		{
			ccConsole::Error("Stop the other picking operation first!");
		}
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

	s_previousPickingOperation = PICKING_ROTATION_CENTER;
	enablePickingOperation(win,"Pick a point to be used as rotation center (click on icon again to cancel)");
}

enum ToggleEntityState
{
	TOGGLE_ENT_ACTIVATION = 0,
	TOGGLE_ENT_VISIBILITY,
	TOGGLE_ENT_COLORS,
	TOGGLE_ENT_NORMALS,
	TOGGLE_ENT_SF,
	TOGGLE_ENT_MAT,
	TOGGLE_ENT_3D_NAME,
};

void MainWindow::toggleSelectedEntitiesActivation()
{
	toggleSelectedEntitiesProp(TOGGLE_ENT_ACTIVATION);
}

void MainWindow::toggleSelectedEntitiesVisibility()
{
	toggleSelectedEntitiesProp(TOGGLE_ENT_VISIBILITY);
}

void MainWindow::toggleSelectedEntitiesColors()
{
	toggleSelectedEntitiesProp(TOGGLE_ENT_COLORS);
}

void MainWindow::toggleSelectedEntitiesNormals()
{
	toggleSelectedEntitiesProp(TOGGLE_ENT_NORMALS);
}

void MainWindow::toggleSelectedEntitiesSF()
{
	toggleSelectedEntitiesProp(TOGGLE_ENT_SF);
}

void MainWindow::toggleSelectedEntitiesMaterials()
{
	toggleSelectedEntitiesProp(TOGGLE_ENT_MAT);
}

void MainWindow::toggleSelectedEntities3DName()
{
	toggleSelectedEntitiesProp(TOGGLE_ENT_3D_NAME);
}

void MainWindow::toggleSelectedEntitiesProp(int prop)
{
	ccHObject baseEntities;
	ConvertToGroup(m_selectedEntities,baseEntities,ccHObject::DP_NONE);
	for (unsigned i=0; i<baseEntities.getChildrenNumber(); ++i)
	{
		ccHObject* child = baseEntities.getChild(i);
		switch(prop)
		{
		case TOGGLE_ENT_ACTIVATION:
			child->toggleActivation/*_recursive*/();
			break;
		case TOGGLE_ENT_VISIBILITY:
			child->toggleVisibility_recursive();
			break;
		case TOGGLE_ENT_COLORS:
			child->toggleColors_recursive();
			break;
		case TOGGLE_ENT_NORMALS:
			child->toggleNormals_recursive();
			break;
		case TOGGLE_ENT_SF:
			child->toggleSF_recursive();
			break;
		case TOGGLE_ENT_MAT:
			child->toggleMaterials_recursive();
			break;
		case TOGGLE_ENT_3D_NAME:
			child->toggleShowName_recursive();
			break;
		default:
			assert(false);
		}
		child->prepareDisplayForRefresh_recursive();
	}

	refreshAll();
	updateUI();
}

void MainWindow::spawnHistogramDialog(const std::vector<unsigned>& histoValues, double minVal, double maxVal, QString title, QString xAxisLabel)
{
	ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);
	hDlg->setWindowTitle("Histogram");

	ccHistogramWindow* histogram = hDlg->window();
	{
		histogram->setTitle(title);
		histogram->fromBinArray(histoValues,minVal,maxVal);
		histogram->setAxisLabels(xAxisLabel,"Count");
		histogram->refresh();
	}

	hDlg->show();
}

void MainWindow::showSelectedEntitiesHistogram()
{
	size_t selNum = m_selectedEntities.size();
	for (size_t i=0; i<selNum; ++i)
	{
		//for "real" point clouds only
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_selectedEntities[i]);
		if (cloud)
		{
			//we display the histogram of the current scalar field
			ccScalarField* sf = static_cast<ccScalarField*>(cloud->getCurrentDisplayedScalarField());
			if (sf)
			{
				ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);
				hDlg->setWindowTitle(QString("Histogram [%1]").arg(cloud->getName()));

				ccHistogramWindow* histogram = hDlg->window();
				{
					unsigned numberOfPoints = cloud->size();
					unsigned numberOfClasses = static_cast<unsigned>(sqrt(static_cast<double>(numberOfPoints)));
					//we take the 'nearest' multiple of 4
					numberOfClasses &= (~3);
					numberOfClasses = std::max<unsigned>(4,numberOfClasses);
					numberOfClasses = std::min<unsigned>(256,numberOfClasses);

					histogram->setTitle(QString("%1 (%2 values) ").arg(sf->getName()).arg(numberOfPoints));
					bool showNaNValuesInGrey = sf->areNaNValuesShownInGrey();
					histogram->fromSF(sf,numberOfClasses,true,showNaNValuesInGrey);
					histogram->setAxisLabels(sf->getName(),"Count");
					histogram->refresh();
				}
				hDlg->show();
			}
		}
	}
}

void MainWindow::doActionCrop()
{
	ccHObject::Container selectedEntities = m_selectedEntities;
	size_t selNum = selectedEntities.size();

	//find candidates
	std::vector<ccHObject*> candidates;
	ccBBox baseBB;
	{
		for (size_t i=0; i<selNum; ++i)
		{
			ccHObject* ent = selectedEntities[i];
			if (	ent->isA(CC_TYPES::POINT_CLOUD)
				||	ent->isKindOf(CC_TYPES::MESH) )
			{
				candidates.push_back(ent);
				baseBB += ent->getOwnBB();
			}
		}
	}

	if (candidates.empty())
	{
		ccConsole::Warning("[Crop] No eligible candidate found!");
		return;
	}

	ccBoundingBoxEditorDlg bbeDlg(this);
	bbeDlg.setBaseBBox(baseBB,false);
	bbeDlg.showInclusionWarning(false);
	bbeDlg.setWindowTitle("Crop");

	if (!bbeDlg.exec())
		return;

	//deselect all entities
	if (m_ccRoot)
		m_ccRoot->unselectAllEntities();

	//cropping box
	ccBBox box = bbeDlg.getBox();

	//process cloud/meshes
	bool errors = false;
	bool successes = false;
	{
		for (size_t i=0; i<candidates.size(); ++i)
		{
			ccHObject* ent = candidates[i];
			ccHObject* croppedEnt = ccCropTool::Crop(ent,box,true);
			if (croppedEnt)
			{
				croppedEnt->setName(ent->getName() + QString(".cropped"));
				croppedEnt->setDisplay(ent->getDisplay());
				croppedEnt->prepareDisplayForRefresh();
				if (ent->getParent())
					ent->getParent()->addChild(croppedEnt);
				ent->setEnabled(false);
				addToDB(croppedEnt);
				//select output entity
				m_ccRoot->selectEntity(croppedEnt, true);
				successes = true;
			}
			else
			{
				errors = true;
			}
		}
	}

	if (successes)
		ccLog::Warning("[Crop] Selected entities have been hidden");
	if (errors)
		ccLog::Error("Error(s) occurred! See the Console");

	refreshAll();
	updateUI();
}

void MainWindow::doActionClone()
{
	ccHObject::Container selectedEntities = m_selectedEntities;
	size_t selNum = selectedEntities.size();

	ccHObject* lastClone = 0;
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* clone = 0;
		if (selectedEntities[i]->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			clone = ccHObjectCaster::ToGenericPointCloud(selectedEntities[i])->clone();
			if (!clone)
			{
				ccConsole::Error(QString("An error occurred while cloning cloud %1").arg(selectedEntities[i]->getName()));
			}
		}
		else if (selectedEntities[i]->isKindOf(CC_TYPES::PRIMITIVE))
		{
			clone = static_cast<ccGenericPrimitive*>(selectedEntities[i])->clone();
			if (!clone)
			{
				ccConsole::Error(QString("An error occurred while cloning primitive %1").arg(selectedEntities[i]->getName()));
			}
		}
		else if (selectedEntities[i]->isA(CC_TYPES::MESH))
		{
			clone = ccHObjectCaster::ToMesh(selectedEntities[i])->cloneMesh();
			if (!clone)
			{
				ccConsole::Error(QString("An error occurred while cloning mesh %1").arg(selectedEntities[i]->getName()));
			}
		}
		else if (selectedEntities[i]->isA(CC_TYPES::POLY_LINE))
		{
			ccPolyline* poly = ccHObjectCaster::ToPolyline(selectedEntities[i]);
			clone = (poly ? new ccPolyline(*poly) : 0);
			if (!clone)
			{
				ccConsole::Error(QString("An error occurred while cloning polyline %1").arg(selectedEntities[i]->getName()));
			}
		}
		else if (selectedEntities[i]->isA(CC_TYPES::FACET))
		{
			ccFacet* facet = ccHObjectCaster::ToFacet(selectedEntities[i]);
			clone = (facet ? facet->clone() : 0);
			if (!clone)
			{
				ccConsole::Error(QString("An error occurred while cloning facet %1").arg(selectedEntities[i]->getName()));
			}
		}
		else
		{
			ccLog::Warning(QString("Entity '%1' can't be cloned (type not supported yet!)").arg(selectedEntities[i]->getName()));
		}

		if (clone)
		{
			//copy GL transformation history
			clone->setGLTransformationHistory(selectedEntities[i]->getGLTransformationHistory());
			//copy display
			clone->setDisplay(selectedEntities[i]->getDisplay());
			
			addToDB(clone);
			lastClone = clone;
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
	if (selNum != 1)
	{
		if (selNum > 1)
			ccConsole::Error("Select only one cloud or one mesh!");
		return;
	}

	ccHObject* ent = m_selectedEntities[0];

	bool lockedVertices;
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent,&lockedVertices);

	//for "real" point clouds only
	if (!cloud)
		return;
	if (lockedVertices && !ent->isAncestorOf(cloud))
	{
		DisplayLockedVerticesWarning(ent->getName(),true);
		return;
	}

	QString defaultName = "Constant";
	unsigned trys = 1;
	while (cloud->getScalarFieldIndexByName(qPrintable(defaultName)) >= 0 || trys > 99)
	{
		defaultName = QString("Constant #%1").arg(++trys);
	}

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
	if (cloud->getScalarFieldIndexByName(qPrintable(sfName)) >= 0)
	{
		ccLog::Error("Name already exists!");
		return;
	}

	ScalarType sfValue = static_cast<ScalarType>(QInputDialog::getDouble(this,"Add constant value", "value", s_constantSFValue, -1.0e9, 1.0e9, 8, &ok));
	if (!ok)
		return;

	int sfIdx = cloud->getScalarFieldIndexByName(qPrintable(sfName));
	if (sfIdx < 0)
		sfIdx = cloud->addScalarField(qPrintable(sfName));
	if (sfIdx < 0)
	{
		ccLog::Error("An error occurred! (see console)");
		return;
	}

	CCLib::ScalarField* sf = cloud->getScalarField(sfIdx);
	assert(sf);
	if (sf)
	{
		sf->fill(sfValue);
		sf->computeMinAndMax();
		cloud->setCurrentDisplayedScalarField(sfIdx);
		cloud->showSF(true);
		updateUI();
		if (cloud->getDisplay())
			cloud->getDisplay()->redraw(false);
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
		for (size_t i=0; i<m_selectedEntities.size(); ++i)
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
					fields[i] = 0;
				}
			}
		}

		//export points
		for (unsigned j=0; j<cloud->size(); ++j)
		{
			const ColorCompType* rgb = cloud->getPointColor(j);

			if (fields[0])
				fields[0]->addElement(rgb[0]);
			if (fields[1])
				fields[1]->addElement(rgb[1]);
			if (fields[2])
				fields[2]->addElement(rgb[2]);
			if (fields[3])
				fields[3]->addElement(static_cast<ScalarType>(rgb[0] + rgb[1] + rgb[2])/3);
		}

		QString fieldsStr;
		{
			for (size_t i=0; i<fields.size(); ++i)
			{
				if (fields[i])
				{
					fields[i]->computeMinAndMax();

					int sfIdx = cloud->getScalarFieldIndexByName(fields[i]->getName());
					if (sfIdx >= 0)
						cloud->deleteScalarField(sfIdx);
					sfIdx = cloud->addScalarField(fields[i]);
					assert(sfIdx >= 0);
					if (sfIdx >= 0)
					{
						cloud->setCurrentDisplayedScalarField(sfIdx);
						cloud->showSF(true);
						cloud->refreshDisplay();

						//mesh vertices?
						if (cloud->getParent() && cloud->getParent()->isKindOf(CC_TYPES::MESH))
						{
							cloud->getParent()->showSF(true);
							cloud->getParent()->refreshDisplay();
						}

						if (!fieldsStr.isEmpty())
							fieldsStr.append(", ");
						fieldsStr.append(fields[i]->getName());
					}
					else
					{
						ccConsole::Warning(QString("[doActionScalarFieldFromColor] Failed to add scalar field '%1' to cloud '%2'?!").arg(fields[i]->getName()).arg(cloud->getName()));
						fields[i]->release();
						fields[i] = 0;
					}
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
		DisplayLockedVerticesWarning(entity->getName(),true);
		return;
	}
	if (!cloud)
		return;

	ccScalarFieldArithmeticsDlg sfaDlg(cloud,this);

	if (!sfaDlg.exec())
		return;

	if (!sfaDlg.apply(cloud))
	{
		ccConsole::Error("An error occurred (see Console for more details)");
	}

	cloud->showSF(true);
	cloud->prepareDisplayForRefresh_recursive();

	refreshAll();
	updateUI();
}

void MainWindow::doActionFitSphere()
{
	size_t selNum = m_selectedEntities.size();

	double outliersRatio = 0.5;
	double confidence = 0.99;

	ccProgressDialog pDlg(true,this);
	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = m_selectedEntities[i];

		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
		if (!cloud)
			continue;

		CCVector3 center;
		PointCoordinateType radius;
		double rms;
		if (!CCLib::GeometricalAnalysisTools::detectSphereRobust(cloud,
			outliersRatio,
			center,
			radius,
			rms,
			&pDlg,
			confidence))
		{
			ccLog::Warning(QString("[Fit sphere] Failed to fit a sphere on cloud '%1'").arg(cloud->getName()));
			continue;
		}

		ccLog::Print(QString("[Fit sphere] Cloud '%1': center (%2,%3,%4) - radius = %5 [RMS = %6]")
			.arg(cloud->getName())
			.arg(center.x)
			.arg(center.y)
			.arg(center.z)
			.arg(radius)
			.arg(rms));

		ccGLMatrix trans;
		trans.setTranslation(center);
		ccSphere* sphere = new ccSphere(radius,&trans,QString("Sphere r=%1 [rms %2]").arg(radius).arg(rms));
		cloud->addChild(sphere);
		//sphere->setDisplay(cloud->getDisplay());
		sphere->prepareDisplayForRefresh();
		addToDB(sphere,false,false,false);
	}
	
	refreshAll();
}

void MainWindow::doActionFitPlane()
{
	doComputePlaneOrientation(false);
}

void MainWindow::doActionFitFacet()
{
	doComputePlaneOrientation(true);
}

static double s_polygonMaxEdgeLength = 0;
void MainWindow::doComputePlaneOrientation(bool fitFacet)
{
	ccHObject::Container selectedEntities = m_selectedEntities;
	size_t selNum = selectedEntities.size();
	if (selNum < 1)
		return;

	double maxEdgeLength = 0;
	if (fitFacet)
	{
		bool ok = true;
		maxEdgeLength = QInputDialog::getDouble(this,"Fit facet", "Max edge length (0 = no limit)", s_polygonMaxEdgeLength, 0, 1.0e9, 8, &ok);
		if (!ok)
			return;
		s_polygonMaxEdgeLength = maxEdgeLength;
	}

	for (size_t i=0; i<selNum; ++i)
	{
		ccHObject* ent = selectedEntities[i];
		ccShiftedObject* shifted = 0;
		CCLib::GenericIndexedCloudPersist* cloud = 0;

		if (ent->isKindOf(CC_TYPES::POLY_LINE))
		{
			ccPolyline* poly = ccHObjectCaster::ToPolyline(ent);
			cloud = static_cast<CCLib::GenericIndexedCloudPersist*>(poly);
			shifted = poly;
		}
		else
		{
			ccGenericPointCloud* gencloud = ccHObjectCaster::ToGenericPointCloud(ent);
			if (gencloud)
			{
				cloud = static_cast<CCLib::GenericIndexedCloudPersist*>(gencloud);
				shifted = gencloud;
			}
		}

		if (cloud)
		{
			double rms = 0.0;
			CCVector3 C,N;

			ccHObject* plane = 0;
			if (fitFacet)
			{
				ccFacet* facet = ccFacet::Create(cloud, static_cast<PointCoordinateType>(maxEdgeLength));
				if (facet)
				{
					plane = static_cast<ccHObject*>(facet);
					N = facet->getNormal();
					C = facet->getCenter();
					rms = facet->getRMS();

					//manually copy shift & scale info!
					if (shifted)
					{
						ccPolyline* contour = facet->getContour();
						if (contour)
						{
							contour->setGlobalScale(shifted->getGlobalScale());
							contour->setGlobalShift(shifted->getGlobalShift());
						}
					}
				}
			}
			else
			{
				ccPlane* pPlane = ccPlane::Fit(cloud, &rms);
				if (pPlane)
				{
					plane = static_cast<ccHObject*>(pPlane);
					N = pPlane->getNormal();
					C = *CCLib::Neighbourhood(cloud).getGravityCenter();
					pPlane->enableStippling(true);
				}
			}

			//as all information appears in Console...
			forceConsoleDisplay();

			if (plane)
			{
				ccConsole::Print(QString("[Orientation] Entity '%1'").arg(ent->getName()));
				ccConsole::Print("\t- plane fitting RMS: %f",rms);

				//We always consider the normal with a positive 'Z' by default!
				if (N.z < 0.0)
					N *= -1.0;
				ccConsole::Print("\t- normal: (%f,%f,%f)",N.x,N.y,N.z);

				//we compute strike & dip by the way
				PointCoordinateType dip = 0, dipDir = 0;
				ccNormalVectors::ConvertNormalToDipAndDipDir(N,dip,dipDir);
				QString dipAndDipDirStr = ccNormalVectors::ConvertDipAndDipDirToString(dip,dipDir);
				ccConsole::Print(QString("\t- %1").arg(dipAndDipDirStr));

				//hack: output the transformation matrix that would make this normal points towards +Z
				ccGLMatrix makeZPosMatrix = ccGLMatrix::FromToRotation(N,CCVector3(0,0,PC_ONE));
				CCVector3 Gt = C;
				makeZPosMatrix.applyRotation(Gt);
				makeZPosMatrix.setTranslation(C-Gt);
				ccConsole::Print("[Orientation] A matrix that would make this plane horizontal (normal towards Z+) is:");
				ccConsole::Print(makeZPosMatrix.toString(12,' ')); //full precision
				ccConsole::Print("[Orientation] You can copy this matrix values (CTRL+C) and paste them in the 'Apply transformation tool' dialog");

				plane->setName(dipAndDipDirStr);
				plane->applyGLTransformation_recursive(); //not yet in DB
				plane->setVisible(true);
				plane->setSelectionBehavior(ccHObject::SELECTION_FIT_BBOX);

				ent->addChild(plane);
				plane->setDisplay(ent->getDisplay());
				plane->prepareDisplayForRefresh_recursive();
				addToDB(plane);
			}
			else
			{
				ccConsole::Warning(QString("Failed to fit a plane/facet on entity '%1'").arg(ent->getName()));
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
	//we use CCLIB_ALGO_ACCURATE_DENSITY by default (will be modified if necessary)
	if (!ApplyCCLibAlgortihm(CCLIB_ALGO_ACCURATE_DENSITY,m_selectedEntities,this))
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

void MainWindow::doCylindricalNeighbourhoodExtractionTest()
{
	bool ok;
	double radius = QInputDialog::getDouble(this,"CNE Test","radius",0.02,1.0e-6,1.0e6,6,&ok);
	if (!ok)
		return;

	double height = QInputDialog::getDouble(this,"CNE Test","height",0.05,1.0e-6,1.0e6,6,&ok);
	if (!ok)
		return;

	ccPointCloud* cloud = new ccPointCloud("cube");
	const unsigned ptsCount = 1000000;
	if (!cloud->reserve(ptsCount))
	{
		ccConsole::Error("Not enough memory!");
		delete cloud;
		return;
	}

	//fill a unit cube with random points
	{
		for (unsigned i=0; i<ptsCount; ++i)
		{
			CCVector3 P(	static_cast<PointCoordinateType>(rand())/static_cast<PointCoordinateType>(RAND_MAX),
							static_cast<PointCoordinateType>(rand())/static_cast<PointCoordinateType>(RAND_MAX),
							static_cast<PointCoordinateType>(rand())/static_cast<PointCoordinateType>(RAND_MAX) );

			cloud->addPoint(P);
		}
	}

	//get/Add scalar field
	static const char DEFAULT_CNE_TEST_TEMP_SF_NAME[] = "CNE test";
	int sfIdx = cloud->getScalarFieldIndexByName(DEFAULT_CNE_TEST_TEMP_SF_NAME);
	if (sfIdx < 0)
		sfIdx = cloud->addScalarField(DEFAULT_CNE_TEST_TEMP_SF_NAME);
	if (sfIdx < 0)
	{
		ccConsole::Error("Not enough memory!");
		delete cloud;
		return;
	}
	cloud->setCurrentScalarField(sfIdx);

	//reset scalar field
	cloud->getScalarField(sfIdx)->fill(NAN_VALUE);

	ccProgressDialog pDlg(true,this);
	ccOctree* octree = cloud->computeOctree(&pDlg);
	if (octree)
	{
		QElapsedTimer subTimer;
		subTimer.start();
		unsigned long long extractedPoints = 0;
		unsigned char level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(static_cast<PointCoordinateType>(2.5*radius)); //2.5 = empirical
		const unsigned samples = 1000;
		for (unsigned j=0; j<samples; ++j)
		{
			//generate random normal vector
			CCVector3 dir(0,0,1);
			{
				ccGLMatrix rot;
				rot.initFromParameters(	static_cast<PointCoordinateType>( static_cast<double>(rand())/static_cast<double>(RAND_MAX) * 2.0*M_PI ),
										static_cast<PointCoordinateType>( static_cast<double>(rand())/static_cast<double>(RAND_MAX) * 2.0*M_PI ),
										static_cast<PointCoordinateType>( static_cast<double>(rand())/static_cast<double>(RAND_MAX) * 2.0*M_PI ),
										CCVector3(0,0,0) );
				rot.applyRotation(dir);
			}
			unsigned randIndex = (static_cast<unsigned>(static_cast<double>(rand())*static_cast<double>(ptsCount)/static_cast<double>(RAND_MAX)) % ptsCount);

			CCLib::DgmOctree::CylindricalNeighbourhood cn;
			cn.center = *cloud->getPoint(randIndex);
			cn.dir = dir;
			cn.level = level;
			cn.radius = static_cast<PointCoordinateType>(radius);
			cn.maxHalfLength = static_cast<PointCoordinateType>(height/2);

			octree->getPointsInCylindricalNeighbourhood(cn);
			//octree->getPointsInSphericalNeighbourhood(*cloud->getPoint(randIndex),radius,neighbours,level);
			size_t neihgboursCount = cn.neighbours.size();
			extractedPoints += static_cast<unsigned long long>(neihgboursCount);
			for (size_t k=0; k<neihgboursCount; ++k)
				cloud->setPointScalarValue(cn.neighbours[k].pointIndex,static_cast<ScalarType>(sqrt(cn.neighbours[k].squareDistd)));
		}
		ccConsole::Print("[CNE_TEST] Mean extraction time = %i ms (radius = %f, height = %f, mean(neighbours) = %3.1f)",subTimer.elapsed(),radius,height,static_cast<double>(extractedPoints)/static_cast<double>(samples));
	}
	else
	{
		ccConsole::Error("Failed to compute octree!");
	}

	ccScalarField* sf = static_cast<ccScalarField*>(cloud->getScalarField(sfIdx));
	sf->computeMinAndMax();
	sf->showNaNValuesInGrey(false);
	cloud->setCurrentDisplayedScalarField(sfIdx);
	cloud->showSF(true);

	addToDB(cloud);

	refreshAll();
	updateUI();
}

void MainWindow::doActionCreateCloudFromEntCenters()
{
	size_t selNum = m_selectedEntities.size();

	ccPointCloud* centers = new ccPointCloud("centers");
	if (!centers->reserve(static_cast<unsigned>(selNum)))
	{
		ccLog::Error("Not enough memory!");
		delete centers;
		centers = 0;
		return;
	}

	//look for clouds
	{
		for (size_t i=0; i<selNum; ++i)
		{
			ccHObject* ent = m_selectedEntities[i];
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
			if (cloud)
			{
				centers->addPoint(cloud->getOwnBB().getCenter());
				//we display the cloud in the same window as the first (selected) cloud we encounter
				if (!centers->getDisplay())
					centers->setDisplay(cloud->getDisplay());
			}
		}
	}

	if (centers->size() == 0)
	{
		ccLog::Error("Not cloud in selection?!");
		delete centers;
		centers = 0;
	}
	else
	{
		centers->resize(centers->size());
		centers->setPointSize(10);
		centers->setVisible(true);
		addToDB(centers);
	}
}

void MainWindow::doActionComputeBestICPRmsMatrix()
{
	size_t selNum = m_selectedEntities.size();

	//look for clouds
	std::vector<ccPointCloud*> clouds;
	try
	{
		for (size_t i=0; i<selNum; ++i)
		{
			ccHObject* ent = m_selectedEntities[i];
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
			if (cloud)
			{
				clouds.push_back(cloud);
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("Not enough memory!");
		return;
	}

	size_t cloudCount = clouds.size();
	if (cloudCount < 2)
	{
		ccLog::Error("Need at least two clouds!");
		return;
	}

	//init matrices
	std::vector<double> rmsMatrix;
	std::vector<ccGLMatrix> matrices;
	std::vector< std::pair<double,double> > matrixAngles;
	try
	{
		rmsMatrix.resize(cloudCount*cloudCount,0);

		//init all possible transformations
		static const double angularStep_deg = 45.0;
		unsigned phiSteps = static_cast<unsigned>(360.0/angularStep_deg);
		assert(fabs(360.0 - phiSteps * angularStep_deg) < ZERO_TOLERANCE);
		unsigned thetaSteps = static_cast<unsigned>(180.0/angularStep_deg);
		assert(fabs(180.0 - thetaSteps * angularStep_deg) < ZERO_TOLERANCE);
		unsigned rotCount = phiSteps * (thetaSteps-1) + 2;
		matrices.reserve(rotCount);
		matrixAngles.reserve(rotCount);

		for (unsigned j=0; j<=thetaSteps; ++j)
		{
			//we want to cover the full [0-180] interval! ([-90;90] in fact)
			double theta_deg = j * angularStep_deg - 90.0;
			for (unsigned i=0; i<phiSteps; ++i)
			{
				double phi_deg = i * angularStep_deg;
				ccGLMatrix trans;
				trans.initFromParameters(	static_cast<float>(phi_deg * CC_DEG_TO_RAD),
											static_cast<float>(theta_deg * CC_DEG_TO_RAD),
											0,
											CCVector3(0,0,0) );
				matrices.push_back(trans);
				matrixAngles.push_back( std::pair<double,double>(phi_deg,theta_deg) );

				//for poles, no need to rotate!
				if (j == 0 || j == thetaSteps)
					break;
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("Not enough memory!");
		return;
	}

	//let's start!
	{
		ccProgressDialog pDlg(true,this);
		pDlg.setMethodTitle("Testing all possible positions");
		pDlg.setInfo(qPrintable(QString("%1 clouds and %2 positions").arg(cloudCount).arg(matrices.size())));
		CCLib::NormalizedProgress nProgress(&pDlg,static_cast<unsigned>(((cloudCount*(cloudCount-1))/2)*matrices.size()));
		pDlg.start();
		QApplication::processEvents();

//#define TEST_GENERATION
#ifdef TEST_GENERATION
		ccPointCloud* testSphere = new ccPointCloud();
		testSphere->reserve(matrices.size());
#endif

		for (size_t i=0; i<cloudCount-1; ++i)
		{
			ccPointCloud* A = clouds[i];
			A->computeOctree();

			for (size_t j=i+1; j<cloudCount; ++j)
			{
				ccGLMatrix transBToZero;
				transBToZero.toIdentity();
				transBToZero.setTranslation(-clouds[j]->getOwnBB().getCenter());
				
				ccGLMatrix transFromZeroToA;
				transFromZeroToA.toIdentity();
				transFromZeroToA.setTranslation(A->getOwnBB().getCenter());

#ifndef TEST_GENERATION
				double minRMS = -1.0;
				int bestMatrixIndex = -1;
				ccPointCloud* bestB = 0;
#endif
				for (size_t k=0; k<matrices.size(); ++k)
				{
					ccPointCloud* B = clouds[j]->cloneThis();
					if (!B)
					{
						ccLog::Error("Not enough memory!");
						return;
					}

					ccGLMatrix BtoA = transFromZeroToA * matrices[k] * transBToZero;
					B->applyRigidTransformation(BtoA);

#ifndef TEST_GENERATION
					double finalRMS = 0;
					unsigned finalPointCount = 0;
					CCLib::ICPRegistrationTools::RESULT_TYPE result;
					CCLib::ICPRegistrationTools::ScaledTransformation registerTrans;
					result = CCLib::ICPRegistrationTools::Register(A,0,B,registerTrans,CCLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE,1.0e-6,0,finalRMS,finalPointCount);

					if (result == CCLib::ICPRegistrationTools::ICP_ERROR)
					{
						delete B;
						if (bestB)
							delete bestB;
						ccLog::Error("An error occurred while performing ICP!");
						return;
					}

					if (minRMS < 0 || finalRMS < minRMS)
					{
						minRMS = finalRMS;
						bestMatrixIndex = static_cast<int>(k);
						std::swap(bestB,B);
					}

					if (B)
					{
						delete B;
						B = 0;
					}
#else
					addToDB(B);

					//Test sphere
					CCVector3 Y(0,1,0);
					matrices[k].apply(Y);
					testSphere->addPoint(Y);
#endif

					if (!nProgress.oneStep())
					{
						//process cancelled by user
						return;
					}
				}

#ifndef TEST_GENERATION
				if (bestMatrixIndex >= 0)
				{
					assert(bestB);
					ccHObject* group = new ccHObject(QString("Best case #%1 / #%2 - RMS = %3").arg(i+1).arg(j+1).arg(minRMS));
					group->addChild(bestB);
					group->setDisplay_recursive(A->getDisplay());
					addToDB(group);
					ccLog::Print(QString("[doActionComputeBestICPRmsMatrix] Comparison #%1 / #%2: min RMS = %3 (phi = %4 / theta = %5 deg.)").arg(i+1).arg(j+1).arg(minRMS).arg(matrixAngles[bestMatrixIndex].first).arg(matrixAngles[bestMatrixIndex].second));
				}
				else
				{
					assert(!bestB);
					ccLog::Warning(QString("[doActionComputeBestICPRmsMatrix] Comparison #%1 / #%2: INVALID").arg(i+1).arg(j+1));
				}

				rmsMatrix[i*cloudCount + j] = minRMS;
#else
				addToDB(testSphere);
				i = cloudCount;
				break;
#endif
			}
		}
	}

	//export result as a CSV file
#ifdef TEST_GENERATION
	if (false)
#endif
	{
		//persistent settings
		QSettings settings;
		settings.beginGroup(ccPS::SaveFile());
		QString currentPath = settings.value(ccPS::CurrentPath(),QApplication::applicationDirPath()).toString();

		QString outputFilename = QFileDialog::getSaveFileName(this, "Select output file", currentPath, "*.csv");
		if (outputFilename.isEmpty())
			return;

		QFile fp(outputFilename);
		if (fp.open(QFile::Text | QFile::WriteOnly))
		{
			QTextStream stream(&fp);
			//header
			{
				stream << "RMS";
				for (size_t i=0; i<cloudCount; ++i)
				{
					stream << ";";
					stream << clouds[i]->getName();
				}
				stream << endl;
			}

			//rows
			for (size_t j=0; j<cloudCount; ++j)
			{
				stream << clouds[j]->getName();
				stream << ";";
				for (size_t i=0; i<cloudCount; ++i)
				{
					stream << rmsMatrix[j*cloudCount+i];
					stream << ";";
				}
				stream << endl;
			}
			
			ccLog::Print("[doActionComputeBestICPRmsMatrix] Job done");
		}
		else
		{
			ccLog::Error("Failed to save output file?!");
		}
	}
}

void MainWindow::doActionExportCloudsInfo()
{
	size_t selNum = m_selectedEntities.size();

	//look for clouds
	std::vector<ccPointCloud*> clouds;
	unsigned maxSFCount = 0;
	{
		for (size_t i=0; i<selNum; ++i)
		{
			ccHObject* ent = m_selectedEntities[i];
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
			if (cloud)
			{
				clouds.push_back(cloud);
				maxSFCount = std::max<unsigned>(maxSFCount,cloud->getNumberOfScalarFields());
			}
		}
	}

	if (clouds.empty())
	{
		ccConsole::Error("Select at least one point cloud!");
		return;
	}

	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::SaveFile());
	QString currentPath = settings.value(ccPS::CurrentPath(),QApplication::applicationDirPath()).toString();

	QString outputFilename = QFileDialog::getSaveFileName(this, "Select output file", currentPath, "*.csv");
	if (outputFilename.isEmpty())
		return;

	QFile csvFile(outputFilename);
	if (!csvFile.open(QFile::WriteOnly))
	{
		ccConsole::Error("Failed to open file for writing! (check file permissions)");
		return;
	}

	//save last saving location
	settings.setValue(ccPS::CurrentPath(),QFileInfo(outputFilename).absolutePath());
	settings.endGroup();

	//write CSV header
	QTextStream csvStream(&csvFile);
	csvStream << "Name;";
	csvStream << "Points;";
	csvStream << "meanX;";
	csvStream << "meanY;";
	csvStream << "meanZ;";
	{
		for (unsigned i=0; i<maxSFCount; ++i)
		{
			QString sfIndex = QString("SF#%1").arg(i+1);
			csvStream << sfIndex << " name;";
			csvStream << sfIndex << " valid values;";
			csvStream << sfIndex << " mean;";
			csvStream << sfIndex << " std.dev.;";
			csvStream << sfIndex << " sum;";
		}
	}
	csvStream << endl;

	//write one line per cloud
	{
		for (size_t i=0; i<clouds.size(); ++i)
		{
			ccPointCloud* cloud = clouds[i];
			{
				CCVector3 G = *CCLib::Neighbourhood(cloud).getGravityCenter();
				csvStream << cloud->getName() << ";" /*"Name;"*/;
				csvStream << cloud->size() << ";" /*"Points;"*/;
				csvStream << G.x << ";" /*"meanX;"*/;
				csvStream << G.y << ";" /*"meanY;"*/;
				csvStream << G.z << ";" /*"meanZ;"*/;
				for (unsigned j=0; j<cloud->getNumberOfScalarFields(); ++j)
				{
					CCLib::ScalarField* sf = cloud->getScalarField(j);
					csvStream << sf->getName() << ";" /*"SF name;"*/;

					unsigned validCount = 0;
					double sfSum = 0;
					double sfSum2 = 0;
					for (unsigned k=0; k<sf->currentSize(); ++k)
					{
						const ScalarType& val = sf->getValue(k);
						if (CCLib::ScalarField::ValidValue(val))
						{
							++validCount;
							sfSum += val;
							sfSum2 += val*val;
						}
					}
					csvStream << validCount << ";" /*"SF valid values;"*/;
					double mean = sfSum/validCount;
					csvStream << mean << ";" /*"SF mean;"*/;
					csvStream << sqrt(fabs(sfSum2/validCount - mean*mean)) << ";" /*"SF std.dev.;"*/;
					csvStream << sfSum << ";" /*"SF sum;"*/;
				}
				csvStream << endl;
			}
		}
	}

	ccConsole::Print(QString("[I/O] File '%1' successfully saved (%2 cloud(s))").arg(outputFilename).arg(clouds.size()));
	csvFile.close();
}

QString GetDensitySFName(CCLib::GeometricalAnalysisTools::Density densityType, bool approx, double densityKernelSize = 0.0)
{
	QString sfName;

	//update the name with the density type
	switch (densityType)
	{
	case CCLib::GeometricalAnalysisTools::DENSITY_KNN:
		sfName = CC_LOCAL_KNN_DENSITY_FIELD_NAME;
		break;
	case CCLib::GeometricalAnalysisTools::DENSITY_2D:
		sfName = CC_LOCAL_SURF_DENSITY_FIELD_NAME;
		break;
	case CCLib::GeometricalAnalysisTools::DENSITY_3D:
		sfName = CC_LOCAL_VOL_DENSITY_FIELD_NAME;
		break;
	default:
		assert(false);
		break;
	}

	if (densityType != CCLib::GeometricalAnalysisTools::DENSITY_KNN)
		sfName += QString(" (r=%2)").arg(densityKernelSize);

	if (approx)
		sfName += " [approx]";

	return sfName;
}


bool MainWindow::ApplyCCLibAlgortihm(CC_LIB_ALGORITHM algo, ccHObject::Container& entities, QWidget* parent/*=0*/, void** additionalParameters/*=0*/)
{
	size_t selNum = entities.size();
	if (selNum < 1)
		return false;

	//generic parameters
	QString sfName;

	//computeDensity parameters
	PointCoordinateType densityKernelSize = PC_ONE;
	CCLib::GeometricalAnalysisTools::Density densityType = CCLib::GeometricalAnalysisTools::DENSITY_3D;

	//curvature parameters
	PointCoordinateType curvKernelSize = -PC_ONE;
	CCLib::Neighbourhood::CC_CURVATURE_TYPE curvType = CCLib::Neighbourhood::GAUSSIAN_CURV;

	//computeScalarFieldGradient parameters
	bool euclidean = false;

	//computeRoughness parameters
	PointCoordinateType roughnessKernelSize = PC_ONE;

	switch (algo)
	{
	case CCLIB_ALGO_APPROX_DENSITY:
		{
			//parameters already provided?
			if (additionalParameters)
			{
				densityType = *static_cast<CCLib::GeometricalAnalysisTools::Density*>(additionalParameters[0]);
			}
			else
			{
				//shouldn't happen
				assert(false);
			}
			sfName = GetDensitySFName(densityType,true);
		}
		break;

	case CCLIB_ALGO_ACCURATE_DENSITY:
		{
			//parameters already provided?
			if (additionalParameters)
			{
				densityKernelSize = *static_cast<PointCoordinateType*>(additionalParameters[0]);
				densityType = *static_cast<CCLib::GeometricalAnalysisTools::Density*>(additionalParameters[1]);
			}
			else //ask the user!
			{
				densityKernelSize = GetDefaultCloudKernelSize(entities);
				if (densityKernelSize < 0)
				{
					ccConsole::Error("Invalid kernel size!");
					return false;
				}

				ccDensityDlg dDlg(parent);
				dDlg.setRadius(densityKernelSize);

				//process cancelled by user
				if (!dDlg.exec())
					return false;

				if (!dDlg.isPrecise())
				{
					//Switch to approx density if necessary
					algo = CCLIB_ALGO_APPROX_DENSITY;
				}
				else
				{
					//otherwise read kernel size
					densityKernelSize = static_cast<PointCoordinateType>(dDlg.getRadius());
				}
				//and in both cases we need the density type
				densityType = dDlg.getDensityType();
			}

			sfName = GetDensitySFName(densityType,algo == CCLIB_ALGO_APPROX_DENSITY,densityKernelSize);
		}
		break;

	case CCLIB_ALGO_CURVATURE:
		{
			//parameters already provided?
			if (additionalParameters)
			{
				curvType = *static_cast<CCLib::Neighbourhood::CC_CURVATURE_TYPE*>(additionalParameters[0]);
				curvKernelSize = *static_cast<PointCoordinateType*>(additionalParameters[1]);
			}
			else //ask the user!
			{
				curvKernelSize = GetDefaultCloudKernelSize(entities);
				if (curvKernelSize < 0)
				{
					ccConsole::Error("Invalid kernel size!");
					return false;
				}
				ccCurvatureDlg curvDlg(0);
				if (selNum == 1)
					curvDlg.setKernelSize(curvKernelSize);
				if (!curvDlg.exec())
					return false;

				curvType = curvDlg.getCurvatureType();
				curvKernelSize = static_cast<PointCoordinateType>(curvDlg.getKernelSize());
			}

			QString fieldName;
			switch (curvType)
			{
			case CCLib::Neighbourhood::GAUSSIAN_CURV:
				fieldName = CC_CURVATURE_GAUSSIAN_FIELD_NAME;
				break;
			case CCLib::Neighbourhood::MEAN_CURV:
				fieldName = CC_CURVATURE_MEAN_FIELD_NAME;
				break;
			case CCLib::Neighbourhood::NORMAL_CHANGE_RATE:
				fieldName = CC_CURVATURE_NORM_CHANGE_RATE_FIELD_NAME;
				break;
			default:
				assert(false);
				break;
			}
			sfName = QString("%1 (%2)").arg(fieldName).arg(curvKernelSize);
		}
		break;

	case CCLIB_ALGO_SF_GRADIENT:
		{
			sfName = CC_GRADIENT_NORMS_FIELD_NAME;
			//parameters already provided?
			if (additionalParameters)
			{
				euclidean = *static_cast<bool*>(additionalParameters[0]);
			}
			else //ask the user!
			{
				euclidean = (	QMessageBox::question(parent,
								"Gradient",
								"Is the scalar field composed of (euclidean) distances?",
								QMessageBox::Yes | QMessageBox::No,
								QMessageBox::No ) == QMessageBox::Yes );
			}
		}
		break;

	case CCLIB_ALGO_ROUGHNESS:
	case CCLIB_SPHERICAL_NEIGHBOURHOOD_EXTRACTION_TEST: //for tests: we'll use the roughness kernel for SNE
			{
				//parameters already provided?
				if (additionalParameters)
				{
					roughnessKernelSize = *static_cast<PointCoordinateType*>(additionalParameters[0]);
				}
				else //ask the user!
				{
					roughnessKernelSize = GetDefaultCloudKernelSize(entities);
					if (roughnessKernelSize < 0)
					{
						ccConsole::Error("Invalid kernel size!");
						return false;
					}
					bool ok;
					double val = QInputDialog::getDouble(parent, "Roughness", "Kernel size:", static_cast<double>(roughnessKernelSize), DBL_MIN, 1.0e9, 8, &ok);
					if (!ok)
						return false;
					roughnessKernelSize = static_cast<PointCoordinateType>(val);
				}

				sfName = QString(CC_ROUGHNESS_FIELD_NAME) + QString("(%1)").arg(roughnessKernelSize);
			}
			break;

		default:
			assert(false);
			return false;
	}

	for (size_t i=0; i<selNum; ++i)
	{
		//is the ith selected data is eligible for processing?
		ccGenericPointCloud* cloud = 0;
		switch(algo)
		{
		case CCLIB_ALGO_SF_GRADIENT:
			//for scalar field gradient, we can apply it directly on meshes
			bool lockedVertices;
			cloud = ccHObjectCaster::ToGenericPointCloud(entities[i],&lockedVertices);
			if (lockedVertices)
			{
				DisplayLockedVerticesWarning(entities[i]->getName(),selNum == 1);
				cloud = 0;
			}
			if (cloud)
			{
				//but we need an already displayed SF!
				if (cloud->isA(CC_TYPES::POINT_CLOUD))
				{
					ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
					//on met en lecture (OUT) le champ scalaire actuellement affiche
					int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
					if (outSfIdx < 0)
					{
						cloud = 0;
					}
					else
					{
						pc->setCurrentOutScalarField(outSfIdx);
						sfName = QString("%1(%2)").arg(CC_GRADIENT_NORMS_FIELD_NAME).arg(pc->getScalarFieldName(outSfIdx));
					}
				}
				else //if (!cloud->hasDisplayedScalarField()) //TODO: displayed but not necessarily set as OUTPUT!
				{
					cloud = 0;
				}
			}
			break;

			//by default, we apply processings on clouds only
		default:
			if (entities[i]->isKindOf(CC_TYPES::POINT_CLOUD))
				cloud = ccHObjectCaster::ToGenericPointCloud(entities[i]);
			break;
		}

		if (cloud)
		{
			ccPointCloud* pc = 0;
			int sfIdx = -1;
			if (cloud->isA(CC_TYPES::POINT_CLOUD))
			{
				pc = static_cast<ccPointCloud*>(cloud);

				sfIdx = pc->getScalarFieldIndexByName(qPrintable(sfName));
				if (sfIdx < 0)
					sfIdx = pc->addScalarField(qPrintable(sfName));
				if (sfIdx >= 0)
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
			case CCLIB_ALGO_APPROX_DENSITY:
				result = CCLib::GeometricalAnalysisTools::computeLocalDensityApprox(cloud,
																					densityType,
																					&pDlg,
																					octree);
				break;

			case CCLIB_ALGO_ACCURATE_DENSITY:
				result = CCLib::GeometricalAnalysisTools::computeLocalDensity(	cloud,
																				densityType,
																				densityKernelSize,
																				&pDlg,
																				octree);
				break;

			case CCLIB_ALGO_CURVATURE:
				result = CCLib::GeometricalAnalysisTools::computeCurvature(	cloud,
																			curvType,
																			curvKernelSize,
																			&pDlg,
																			octree);
				break;

			case CCLIB_ALGO_SF_GRADIENT:
				result = CCLib::ScalarFieldTools::computeScalarFieldGradient(	cloud,
																				0, //auto --> FIXME: should be properly set by the user!
																				euclidean,
																				false,
																				&pDlg,
																				octree);

				//rename output scalar field
				if (result == 0)
				{
					int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
					assert(outSfIdx >= 0);
					sfName = QString("%1.gradient").arg(pc->getScalarFieldName(outSfIdx));
					pc->renameScalarField(outSfIdx,qPrintable(sfName));
				}
				//*/
				break;

			case CCLIB_ALGO_ROUGHNESS:
				result = CCLib::GeometricalAnalysisTools::computeRoughness(	cloud,
																			roughnessKernelSize,
																			&pDlg,
																			octree);
				break;

				//TEST
			case CCLIB_SPHERICAL_NEIGHBOURHOOD_EXTRACTION_TEST:
				{
					unsigned count = cloud->size();
					cloud->enableScalarField();
					{
						for (unsigned j=0; j<count; ++j)
							cloud->setPointScalarValue(j,NAN_VALUE);
					}

					QElapsedTimer subTimer;
					subTimer.start();
					unsigned long long extractedPoints = 0;
					unsigned char level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(roughnessKernelSize);;
					const unsigned samples = 1000;
					for (unsigned j=0; j<samples; ++j)
					{
						unsigned randIndex = (static_cast<unsigned>(static_cast<double>(rand())*static_cast<double>(count)/static_cast<double>(RAND_MAX)) % count);
						CCLib::DgmOctree::NeighboursSet neighbours;
						octree->getPointsInSphericalNeighbourhood(*cloud->getPoint(randIndex),roughnessKernelSize,neighbours,level);
						size_t neihgboursCount = neighbours.size();
						extractedPoints += static_cast<unsigned long long>(neihgboursCount);
						for (size_t k=0; k<neihgboursCount; ++k)
							cloud->setPointScalarValue(neighbours[k].pointIndex,static_cast<ScalarType>(sqrt(neighbours[k].squareDistd)));
					}
					ccConsole::Print("[SNE_TEST] Mean extraction time = %i ms (radius = %f, mean(neighbours) = %3.1f)",subTimer.elapsed(),roughnessKernelSize,static_cast<double>(extractedPoints)/static_cast<double>(samples));

					result = 0;
				}
				break;

			default:
				//missed something?
				assert(false);
			}
			qint64 elapsedTime_ms = eTimer.elapsed();

			if (result == 0)
			{
				if (pc && sfIdx >= 0)
				{
					pc->setCurrentDisplayedScalarField(sfIdx);
					pc->showSF(sfIdx >= 0);
					pc->getCurrentInScalarField()->computeMinAndMax();
				}
				cloud->prepareDisplayForRefresh();
				ccConsole::Print("[Algortihm] Timing: %3.2f s.",static_cast<double>(elapsedTime_ms)/1.0e3);
			}
			else
			{
				ccConsole::Warning(QString("Failed to apply processing to cloud '%1'").arg(cloud->getName()));
				if (pc && sfIdx >= 0)
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
	if (selNum != 2)
	{
		ccConsole::Error("Select 2 point clouds!");
		return;
	}

	if (!m_selectedEntities[0]->isKindOf(CC_TYPES::POINT_CLOUD) ||
		!m_selectedEntities[1]->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error("Select 2 point clouds!");
		return;
	}

	ccOrderChoiceDlg dlg(	m_selectedEntities[0], "Compared",
							m_selectedEntities[1], "Reference",
							this );
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
	if (selNum != 2)
	{
		ccConsole::Error("Select 2 entities!");
		return;
	}

	bool isMesh[2] = {false,false};
	unsigned meshNum = 0;
	unsigned cloudNum = 0;
	for (unsigned i=0; i<2; ++i)
	{
		if (m_selectedEntities[i]->isKindOf(CC_TYPES::MESH))
		{
			++meshNum;
			isMesh[i] = true;
		}
		else if (m_selectedEntities[i]->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			++cloudNum;
		}
	}

	if (meshNum == 0)
	{
		ccConsole::Error("Select at least one mesh!");
		return;
	}
	else if (meshNum+cloudNum < 2)
	{
		ccConsole::Error("Select one mesh and one cloud or two meshes!");
		return;
	}

	ccHObject* compEnt = 0;
	ccGenericMesh* refMesh = 0;

	if (meshNum == 1)
	{
		compEnt = m_selectedEntities[isMesh[0] ? 1 : 0];
		refMesh = ccHObjectCaster::ToGenericMesh(m_selectedEntities[isMesh[0] ? 0 : 1]);
	}
	else
	{
		ccOrderChoiceDlg dlg(	m_selectedEntities[0], "Compared",
								m_selectedEntities[1], "Reference",
								this );
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

void MainWindow::deactivateComparisonMode(int result)
{
	//DGM: a bug apperead with recent changes (from CC or QT?)
	//which prevent us from deleting the dialog right away...
	//(it seems that QT has not yet finished the dialog closing
	//when the 'finished' signal is sent).
	//if(m_compDlg)
	//	delete m_compDlg;
	//m_compDlg = 0;

	//if the comparison is a success, we select only the compared entity
	if (m_compDlg && result == QDialog::Accepted && m_ccRoot)
	{
		ccHObject* compEntity = m_compDlg->getComparedEntity();
		if (compEntity)
			m_ccRoot->selectEntity(compEntity);
	}

	freezeUI(false);

	updateUI();
}

void MainWindow::toggleActiveWindowSunLight()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->toggleSunLight();
		win->redraw(false);
	}
}

void MainWindow::toggleActiveWindowCustomLight()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->toggleCustomLight();
		win->redraw(false);
	}
}

void MainWindow::toggleActiveWindowStereoVision(bool state)
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		bool isActive = win->stereoModeIsEnabled();
		if (isActive == state)
		{
			//nothing to do
			return;
		}

		if (isActive)
		{
			win->disableStereoMode();
		}
		else
		{
			//display a parameters dialog
			ccStereoModeDlg smDlg(this);
			smDlg.setParameters(win->getStereoParams());
			if (!smDlg.exec())
			{
				//cancelled by the user
				actionEnableStereo->blockSignals(true);
				actionEnableStereo->setChecked(false);
				actionEnableStereo->blockSignals(false);
				return;
			}

			//force perspective state!
			if (!win->getViewportParameters().perspectiveView)
			{
				setCenteredPerspectiveView(win,false);
			}

			ccGLWindow::StereoParams params = smDlg.getParameters();

			if (params.glassType == ccGLWindow::StereoParams::NVIDIA_VISION)
			{
				//force (exclusive) full screen
				actionExclusiveFullScreen->setChecked(true);
			}

			if (!win->enableStereoMode(params))
			{
				//activation of the stereo mode failed: cancel selection
				actionEnableStereo->blockSignals(true);
				actionEnableStereo->setChecked(false);
				actionEnableStereo->blockSignals(false);
			}
		}
		win->redraw();
	}
}

bool MainWindow::checkStereoMode(ccGLWindow* win)
{
	assert(win);
		
	if (win && win->getViewportParameters().perspectiveView && win->stereoModeIsEnabled())
	{
		if (QMessageBox::question(this,"Stereo mode", "Stereo-mode only works in perspective mode. Do you want to disable it?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
		{
			return false;
		}
		else
		{
			win->disableStereoMode();
			actionEnableStereo->blockSignals(true);
			actionEnableStereo->setChecked(false);
			actionEnableStereo->blockSignals(false);
		}
	}

	return true;
}

void MainWindow::toggleActiveWindowCenteredPerspective()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		const ccViewportParameters& params = win->getViewportParameters();
		if (params.perspectiveView && params.objectCenteredView && !checkStereoMode(win)) //we need to check this only if we are already in object-centered perspective mode
		{
			return;
		}
		win->togglePerspective(true);
		win->redraw(false);
		updateViewModePopUpMenu(win);
		updatePivotVisibilityPopUpMenu(win);
	}
}

void MainWindow::toggleActiveWindowViewerBasedPerspective()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		const ccViewportParameters& params = win->getViewportParameters();
		if (params.perspectiveView && !params.objectCenteredView && !checkStereoMode(win)) //we need to check this only if we are already in viewer-based perspective mode
		{
			return;
		}
		win->togglePerspective(false);
		win->redraw(false);
		updateViewModePopUpMenu(win);
		updatePivotVisibilityPopUpMenu(win);
	}
}

void MainWindow::toggleRotationAboutVertAxis()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		bool wasLocked = win->isVerticalRotationLocked();
		bool isLocked = !wasLocked;
		
		win->lockVerticalRotation(isLocked);
		
		actionLockRotationVertAxis->blockSignals(true);
		actionLockRotationVertAxis->setChecked(isLocked);
		actionLockRotationVertAxis->blockSignals(false);

		if (isLocked)
		{
			win->displayNewMessage(QString("[ROTATION LOCKED]"),ccGLWindow::UPPER_CENTER_MESSAGE,false,24*3600,ccGLWindow::ROTAION_LOCK_MESSAGE);
		}
		else
		{
			win->displayNewMessage(QString(),ccGLWindow::UPPER_CENTER_MESSAGE,false,0,ccGLWindow::ROTAION_LOCK_MESSAGE);
		}
		win->redraw(true, false);
	}
}

void MainWindow::doActionEnableBubbleViewMode()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->setBubbleViewMode(true);
		win->redraw(false);
	}
}

void MainWindow::doActionDeleteShader()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
		win->setShader(0);
}

void MainWindow::doDisableGLFilter()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->setGlFilter(0);
		win->redraw(false);
	}
}

void MainWindow::removeFromDB(ccHObject* obj, bool autoDelete/*=true*/)
{
	if (!obj)
		return;

	//remove dependency to avoid deleting the object when removing it from DB tree
	if (!autoDelete && obj->getParent())
		obj->getParent()->removeDependencyWith(obj);

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

void MainWindow::addToDB(	ccHObject* obj,
							bool updateZoom/*=true*/,
							bool autoExpandDBTree/*=true*/,
							bool checkDimensions/*=true*/,
							bool autoRedraw/*=true*/)
{
	//let's check that the new entity is not too big nor too far from scene center!
	if (checkDimensions)
	{
		//get entity bounding box
		ccBBox bBox = obj->getBB_recursive();

		CCVector3 center = bBox.getCenter();
		PointCoordinateType diag = bBox.getDiagNorm();

		CCVector3d P = CCVector3d::fromArray(center.u);
		CCVector3d Pshift(0,0,0);
		double scale = 1.0;
		//here we must test that coordinates are not too big whatever the case because OpenGL
		//really doesn't like big ones (even if we work with GLdoubles :( ).
		if (ccGlobalShiftManager::Handle(P,diag,ccGlobalShiftManager::DIALOG_IF_NECESSARY,false,Pshift,&scale))
		{
			bool needRescale = (scale != 1.0);
			bool needShift = (Pshift.norm2() > 0);

			if (needRescale || needShift)
			{
				ccGLMatrix mat;
				mat.toIdentity();
				mat.data()[0] = mat.data()[5] = mat.data()[10] = static_cast<float>(scale);
				mat.setTranslation(Pshift);
				obj->applyGLTransformation_recursive(&mat);
				ccConsole::Warning(QString("Entity '%1' has been translated: (%2,%3,%4) and rescaled of a factor %5 [original position will be restored when saving]").arg(obj->getName()).arg(Pshift.x,0,'f',2).arg(Pshift.y,0,'f',2).arg(Pshift.z,0,'f',2).arg(scale,0,'f',6));
			}

			//update 'global shift' and 'global scale' for ALL clouds recursively
			//FIXME: why don't we do that all the time by the way?!
			ccHObject::Container children;
			children.push_back(obj);
			while (!children.empty())
			{
				ccHObject* child = children.back();
				children.pop_back();

				if (child->isKindOf(CC_TYPES::POINT_CLOUD))
				{
					ccGenericPointCloud* pc = ccHObjectCaster::ToGenericPointCloud(child);
					pc->setGlobalShift(pc->getGlobalShift() + Pshift);
					pc->setGlobalScale(pc->getGlobalScale() * scale);
				}

				for (unsigned i=0; i<child->getChildrenNumber(); ++i)
					children.push_back(child->getChild(i));
			}
		}
	}

	//add object to DB root
	if (m_ccRoot)
	{
		//force a 'global zoom' if the DB was emtpy!
		if (!m_ccRoot->getRootEntity() || m_ccRoot->getRootEntity()->getChildrenNumber() == 0)
			updateZoom = true;
		m_ccRoot->addElement(obj,autoExpandDBTree);
	}
	else
	{
		ccLog::Warning("[MainWindow::addToDB] Internal error: no associated db?!");
		assert(false);
	}

	//we can now set destination display (if none already)
	if (!obj->getDisplay())
	{
		ccGLWindow* activeWin = getActiveGLWindow();
		if (!activeWin)
		{
			//no active GL window?!
			return;
		}
		obj->setDisplay_recursive(activeWin);
	}

	//eventually we update the corresponding display
	assert(obj->getDisplay());
	if (updateZoom)
	{
		static_cast<ccGLWindow*>(obj->getDisplay())->zoomGlobal(); //automatically calls ccGLWindow::redraw
	}
	else if (autoRedraw)
	{
		obj->redrawDisplay();
	}
}

void MainWindow::onExclusiveFullScreenToggled(bool state)
{
	//we simply update the fullscreen action method icon (whatever the window)
	ccGLWindow* win = getActiveGLWindow();

	actionExclusiveFullScreen->blockSignals(true);
	actionExclusiveFullScreen->setChecked(win ? win->exclusiveFullScreen() : false);
	actionExclusiveFullScreen->blockSignals(false);

	if (!state && win->stereoModeIsEnabled() && win->getStereoParams().glassType == ccGLWindow::StereoParams::NVIDIA_VISION)
	{
		//auto disable stereo mode as NVidia Vision only works in full screen mode!
		actionEnableStereo->setChecked(false);
	}
}

void MainWindow::addToDBAuto(QStringList filenames)
{
	ccGLWindow* win = qobject_cast<ccGLWindow*>(QObject::sender());

	addToDB(filenames, QString(), win);
}

void MainWindow::addToDB(	const QStringList& filenames,
							QString fileFilter/*=QString()*/,
							ccGLWindow* destWin/*=0*/)
{
	//to handle same 'shift on load' for multiple files
	CCVector3d loadCoordinatesShift(0,0,0);
	bool loadCoordinatesTransEnabled = false;
	
	FileIOFilter::LoadParameters parameters;
	parameters.alwaysDisplayLoadDialog = true;
	parameters.shiftHandlingMode = ccGlobalShiftManager::DIALOG_IF_NECESSARY;
	parameters.coordinatesShift = &loadCoordinatesShift;
	parameters.coordinatesShiftEnabled = &loadCoordinatesTransEnabled;

	//the same for 'addToDB' (if the first one is not supported, or if the scale remains too big)
	CCVector3d addCoordinatesShift(0,0,0);

	for (int i=0; i<filenames.size(); ++i)
	{
		ccHObject* newGroup = FileIOFilter::LoadFromFile(filenames[i],parameters,fileFilter);

		if (newGroup)
		{
			if (destWin)
				newGroup->setDisplay_recursive(destWin);
			addToDB(newGroup,true,true,false);
		}
	}

	QMainWindow::statusBar()->showMessage(QString("%1 file(s) loaded").arg(filenames.size()),2000);
}

void MainWindow::handleNewLabel(ccHObject* entity)
{
	if (entity)
	{
		addToDB(entity);
	}
	else
	{
		assert(false);
	}
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

ccColorScalesManager* MainWindow::getColorScalesManager()
{
	return ccColorScalesManager::GetUniqueInstance();
}

void MainWindow::closeAll()
{
	if (!m_ccRoot)
		return;

	if (QMessageBox::question(	this, "Close all", "Are you sure you want to remove all loaded entities?", QMessageBox::Yes, QMessageBox::No ) != QMessageBox::Yes)
		return;

	ccHObject* root = m_ccRoot->getRootEntity();
	if (root)
	{
		m_ccRoot->unloadAll();
	}

	redrawAll(false);
}

void MainWindow::doActionLoadFile()
{
	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::LoadFile());
	QString currentPath = settings.value(ccPS::CurrentPath(),QApplication::applicationDirPath()).toString();
	QString currentOpenDlgFilter = settings.value(ccPS::SelectedInputFilter(),BinFilter::GetFileFilter()).toString();

	// Add all available file I/O filters (with import capabilities)
	QStringList fileFilters;
	fileFilters.append(s_allFilesFilter);
	bool defaultFilterFound = false;
	{
		const FileIOFilter::FilterContainer& filters = FileIOFilter::GetFilters();
		for (size_t i=0; i<filters.size(); ++i)
		{
			if (filters[i]->importSupported())
			{
				QStringList ff = filters[i]->getFileFilters(true);
				for (int j=0; j<ff.size(); ++j)
				{
					fileFilters.append(ff[j]);
					//is it the (last) default filter?
					if (!defaultFilterFound && currentOpenDlgFilter == ff[j])
					{
						defaultFilterFound = true;
					}
				}
			}
		}
	}

	//default filter is still valid?
	if (!defaultFilterFound)
		currentOpenDlgFilter = s_allFilesFilter;

	//file choosing dialog
	QStringList selectedFiles = QFileDialog::getOpenFileNames(	this,
																"Open file(s)",
																currentPath,
																fileFilters.join(s_fileFilterSeparator),
																&currentOpenDlgFilter
#ifdef _DEBUG
																,QFileDialog::DontUseNativeDialog
#endif
															);
	if (selectedFiles.isEmpty())
		return;

	//save last loading parameters
	currentPath = QFileInfo(selectedFiles[0]).absolutePath();
	settings.setValue(ccPS::CurrentPath(),currentPath);
	settings.setValue(ccPS::SelectedInputFilter(),currentOpenDlgFilter);
	settings.endGroup();

	if (currentOpenDlgFilter == s_allFilesFilter)
		currentOpenDlgFilter.clear(); //this way FileIOFilter will try to guess the file type automatically!

	//load files
	addToDB(selectedFiles,currentOpenDlgFilter);
}

//Helper: check for a filename validity
static bool IsValidFileName(QString filename)
{
#ifdef CC_WINDOWS
	QString sPattern("^(?!^(PRN|AUX|CLOCK\\$|NUL|CON|COM\\d|LPT\\d|\\..*)(\\..+)?$)[^\\x00-\\x1f\\\\?*:\\"";|/]+$");
#else
	QString sPattern("^(([a-zA-Z]:|\\\\)\\\\)?(((\\.)|(\\.\\.)|([^\\\\/:\\*\\?""\\|<>\\. ](([^\\\\/:\\*\\?""\\|<>\\. ])|([^\\\\/:\\*\\?""\\|<>]*[^\\\\/:\\*\\?""\\|<>\\. ]))?))\\\\)*[^\\\\/:\\*\\?""\\|<>\\. ](([^\\\\/:\\*\\?""\\|<>\\. ])|([^\\\\/:\\*\\?""\\|<>]*[^\\\\/:\\*\\?""\\|<>\\. ]))?$");
#endif

	return QRegExp(sPattern).exactMatch(filename);
}

void MainWindow::doActionSaveFile()
{
	size_t selNum = m_selectedEntities.size();
	if (selNum == 0)
		return;

	ccHObject clouds("clouds");
	ccHObject meshes("meshes");
	ccHObject images("images");
	ccHObject polylines("polylines");
	ccHObject other("other");
	ccHObject otherSerializable("serializable");
	ccHObject::Container entitiesToDispatch;
	entitiesToDispatch.insert(entitiesToDispatch.begin(),m_selectedEntities.begin(),m_selectedEntities.end());
	ccHObject entitiesToSave;
	while (!entitiesToDispatch.empty())
	{
		ccHObject* child = entitiesToDispatch.back();
		entitiesToDispatch.pop_back();

		if (child->isA(CC_TYPES::HIERARCHY_OBJECT))
		{
			for (unsigned j=0; j<child->getChildrenNumber(); ++j)
				entitiesToDispatch.push_back(child->getChild(j));
		}
		else
		{
			//we put the entity in the container corresponding to its type
			ccHObject* dest = 0;
			if (child->isA(CC_TYPES::POINT_CLOUD))
				dest = &clouds;
			else if (child->isKindOf(CC_TYPES::MESH))
				dest = &meshes;
			else if (child->isKindOf(CC_TYPES::IMAGE))
				dest = &images;
			else if (child->isKindOf(CC_TYPES::POLY_LINE))
				dest = &polylines;
			else if (child->isSerializable())
				dest = &otherSerializable;
			else
				dest = &other;

			assert(dest);

			//we don't want double insertions if the user has clicked both the father and child
			if (!dest->find(child->getUniqueID()))
			{
				dest->addChild(child,ccHObject::DP_NONE);
				entitiesToSave.addChild(child,ccHObject::DP_NONE);
			}
		}
	}

	bool hasCloud = (clouds.getChildrenNumber() != 0);
	bool hasMesh = (meshes.getChildrenNumber() != 0);
	bool hasImages = (images.getChildrenNumber() != 0);
	bool hasPolylines = (polylines.getChildrenNumber() != 0);
	bool hasSerializable = (otherSerializable.getChildrenNumber() != 0);
	bool hasOther = (other.getChildrenNumber() != 0);

	int stdSaveTypes =		static_cast<int>(hasCloud)
						+	static_cast<int>(hasMesh)
						+	static_cast<int>(hasImages)
						+	static_cast<int>(hasPolylines)
						+	static_cast<int>(hasSerializable);
	if (stdSaveTypes == 0)
	{
		ccConsole::Error("Can't save selected entity(ies) this way!");
		return;
	}

	//we set up the right file filters, depending on the selected
	//entities type (cloud, mesh, etc.).
	QStringList fileFilters;
	{
		const FileIOFilter::FilterContainer& filters = FileIOFilter::GetFilters();
		for (size_t i=0; i<filters.size(); ++i)
		{
			bool atLeastOneExclusive = false;

			//current I/O filter
			const FileIOFilter::Shared filter = filters[i];

			//does this filter can export one or several clouds?
			bool canExportClouds = true;
			if (hasCloud)
			{
				bool isExclusive = true;
				bool multiple = false;
				canExportClouds = (		filter->canSave(CC_TYPES::POINT_CLOUD,multiple,isExclusive)
									&&	(multiple || clouds.getChildrenNumber() == 1) );
				atLeastOneExclusive |= isExclusive;
			}

			//does this filter can export one or several meshes?
			bool canExportMeshes = true;
			if (hasMesh)
			{
				bool isExclusive = true;
				bool multiple = false;
				canExportMeshes = (		filter->canSave(CC_TYPES::MESH,multiple,isExclusive)
									&&	(multiple || meshes.getChildrenNumber() == 1) );
				atLeastOneExclusive |= isExclusive;
			}

			//does this filter can export one or several polylines?
			bool canExportPolylines = true;
			if (hasPolylines)
			{
				bool isExclusive = true;
				bool multiple = false;
				canExportPolylines = (	filter->canSave(CC_TYPES::POLY_LINE,multiple,isExclusive)
									&&	(multiple || polylines.getChildrenNumber() == 1) );
				atLeastOneExclusive |= isExclusive;
			}

			//does this filter can export one or several images?
			bool canExportImages = true;
			if (hasImages)
			{
				bool isExclusive = true;
				bool multiple = false;
				canExportImages = (		filter->canSave(CC_TYPES::IMAGE,multiple,isExclusive)
									&&	(multiple || images.getChildrenNumber() == 1) );
				atLeastOneExclusive |= isExclusive;
			}

			//does this filter can export one or several other serializable entities?
			bool canExportSerializables = true;
			if (hasSerializable)
			{
				//check if all entities have the same type
				{
					CC_CLASS_ENUM firstClassID = otherSerializable.getChild(0)->getUniqueID();
					for (unsigned j=1; j<otherSerializable.getChildrenNumber(); ++j)
					{
						if (otherSerializable.getChild(j)->getUniqueID() != firstClassID)
						{
							//we add a virtual second 'stdSaveType' so as to properly handle exlusivity
							++stdSaveTypes;
							break;
						}
					}
				}

				for (unsigned j=0; j<otherSerializable.getChildrenNumber(); ++j)
				{
					ccHObject* child = otherSerializable.getChild(j);
					bool isExclusive = true;
					bool multiple = false;
					canExportSerializables &= (		filter->canSave(child->getUniqueID(),multiple,isExclusive)
												&&	(multiple || otherSerializable.getChildrenNumber() == 1) );
					atLeastOneExclusive |= isExclusive;
				}
			}

			bool useThisFilter =	canExportClouds
								&&	canExportMeshes
								&&	canExportImages
								&&	canExportPolylines
								&&	canExportSerializables
								&&	(!atLeastOneExclusive || stdSaveTypes == 1);

			if (useThisFilter)
			{
				QStringList ff = filter->getFileFilters(false);
				for (int j=0; j<ff.size(); ++j)
					fileFilters.append(ff[j]);
			}
		}
	}

	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::SaveFile());

	//default filter
	QString selectedFilter = fileFilters.first();
	if (hasCloud)
		selectedFilter = settings.value(ccPS::SelectedOutputFilterCloud(),selectedFilter).toString();
	else if (hasMesh)
		selectedFilter = settings.value(ccPS::SelectedOutputFilterMesh(), selectedFilter).toString();
	else if (hasImages)
		selectedFilter = settings.value(ccPS::SelectedOutputFilterImage(), selectedFilter).toString();
	else if (hasPolylines)
		selectedFilter = settings.value(ccPS::SelectedOutputFilterPoly(), selectedFilter).toString();
	
	//default output path (+ filename)
	QString currentPath = settings.value(ccPS::CurrentPath(),QApplication::applicationDirPath()).toString();
	QString fullPathName = currentPath;
	if (selNum == 1)
	{
		//hierarchy objects have generally as name: 'filename.ext (fullpath)'
		//so we must only take the first part! (otherwise this type of name
		//with a path inside perturbs the QFileDialog a lot ;))
		QString defaultFileName(m_selectedEntities.front()->getName());
		if (m_selectedEntities.front()->isA(CC_TYPES::HIERARCHY_OBJECT))
		{
			QStringList parts = defaultFileName.split(' ',QString::SkipEmptyParts);
			if (parts.size() > 0)
				defaultFileName = parts[0];
		}

		//we remove the extension
		defaultFileName = QFileInfo(defaultFileName).baseName();

		if (!IsValidFileName(defaultFileName))
		{
			ccLog::Warning("[I/O] First entity's name would make an invalid filename! Can't use it...");
			defaultFileName = "project";
		}
		
		fullPathName += QString("/") + defaultFileName;
	}

	//ask the user for the output filename
	QString selectedFilename = QFileDialog::getSaveFileName(this,
															"Save file",
															fullPathName,
															fileFilters.join(s_fileFilterSeparator),
															&selectedFilter);

	if (selectedFilename.isEmpty())
	{
		//process cancelled by the user
		return;
	}

	//ignored items
	if (hasOther)
	{
		ccConsole::Warning("[I/O] The following selected entites won't be saved:");
		for (unsigned i=0; i<other.getChildrenNumber(); ++i)
			ccConsole::Warning(QString("\t- %1s").arg(other.getChild(i)->getName()));
	}

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	FileIOFilter::SaveParameters parameters;
	{
		parameters.alwaysDisplaySaveDialog = true;
	}
	
	//specific case: BIN format
	if (selectedFilter == BinFilter::GetFileFilter())
	{
		if (selNum == 1)
		{
			result = FileIOFilter::SaveToFile(m_selectedEntities.front(),selectedFilename,parameters,selectedFilter);
		}
		else
		{
			//we'll regroup all selected entities in a temporary group
			ccHObject tempContainer;
			ConvertToGroup(m_selectedEntities,tempContainer,ccHObject::DP_NONE);
			if (tempContainer.getChildrenNumber())
			{
				result = FileIOFilter::SaveToFile(&tempContainer,selectedFilename,parameters,selectedFilter);
			}
			else
			{
				ccLog::Warning("[I/O] None of the selected entities can be saved this way...");
				result = CC_FERR_NO_SAVE;
			}
		}
	}
	else if (entitiesToSave.getChildrenNumber() != 0)
	{
		//ignored items
		/*if (hasSerializable)
		{
			if (!hasOther)
				ccConsole::Warning("[I/O] The following selected entites won't be saved:"); //display this warning only if not already done
			for (unsigned i=0; i<otherSerializable.getChildrenNumber(); ++i)
				ccConsole::Warning(QString("\t- %1").arg(otherSerializable.getChild(i)->getName()));
		}
		//*/

		result = FileIOFilter::SaveToFile(	entitiesToSave.getChildrenNumber() > 1 ? &entitiesToSave : entitiesToSave.getChild(0),
											selectedFilename,
											parameters,
											selectedFilter);
	}

	//update default filters
	if (hasCloud)
		settings.setValue(ccPS::SelectedOutputFilterCloud(),selectedFilter);
	if (hasMesh)
		settings.setValue(ccPS::SelectedOutputFilterMesh(), selectedFilter);
	if (hasImages)
		settings.setValue(ccPS::SelectedOutputFilterImage(),selectedFilter);
	if (hasPolylines)
		settings.setValue(ccPS::SelectedOutputFilterPoly(), selectedFilter);

	//we update current file path
	currentPath = QFileInfo(selectedFilename).absolutePath();
	settings.setValue(ccPS::CurrentPath(),currentPath);
	settings.endGroup();
}

void MainWindow::on3DViewActivated(QMdiSubWindow* mdiWin)
{
	ccGLWindow* win = mdiWin ? static_cast<ccGLWindow*>(mdiWin->widget()) : 0;
	if (win)
	{
		updateViewModePopUpMenu(win);
		updatePivotVisibilityPopUpMenu(win);

		actionLockRotationVertAxis->blockSignals(true);
		actionLockRotationVertAxis->setChecked(win->isVerticalRotationLocked());
		actionLockRotationVertAxis->blockSignals(false);

		actionEnableStereo->blockSignals(true);
		actionEnableStereo->setChecked(win->stereoModeIsEnabled());
		actionEnableStereo->blockSignals(false);

		actionExclusiveFullScreen->blockSignals(true);
		actionExclusiveFullScreen->setChecked(win->exclusiveFullScreen());
		actionExclusiveFullScreen->blockSignals(false);
	}

	actionLockRotationVertAxis->setEnabled(win != 0);
	actionEnableStereo->setEnabled(win != 0);
	actionExclusiveFullScreen->setEnabled(win != 0);
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
	int mdiChildCount = m_mdiArea->subWindowList().size();
	bool hasSelectedEntities = (m_ccRoot && m_ccRoot->countSelectedEntities() > 0);

	//General Menu
	menuEdit->setEnabled(true/*hasSelectedEntities*/);
	menuTools->setEnabled(true/*hasSelectedEntities*/);

	//3D Views Menu
	actionClose3DView    ->setEnabled(hasMdiChild);
	actionCloseAll3DViews->setEnabled(mdiChildCount != 0);
	actionTile3DViews    ->setEnabled(mdiChildCount > 1);
	actionCascade3DViews ->setEnabled(mdiChildCount > 1);
	actionNext3DView     ->setEnabled(mdiChildCount > 1);
	actionPrevious3DView ->setEnabled(mdiChildCount > 1);

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
	menu3DViews->addAction(actionZoomIn);
	menu3DViews->addAction(actionZoomOut);
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

		for (int i=0; i<windows.size(); ++i)
		{
			QWidget *child = windows.at(i)->widget();

			QString text = QString("&%1 %2").arg(i + 1).arg(child->windowTitle());
			QAction *action = menu3DViews->addAction(text);
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

void MainWindow::redrawAll(bool only2D/*=false*/)
{
	QList<QMdiSubWindow*> windows = m_mdiArea->subWindowList();
	for (int i=0; i<windows.size(); ++i)
		static_cast<ccGLWindow*>(windows.at(i)->widget())->redraw(only2D);
}

void MainWindow::refreshAll(bool only2D/*=false*/)
{
	QList<QMdiSubWindow*> windows = m_mdiArea->subWindowList();
	for (int i=0; i<windows.size(); ++i)
		static_cast<ccGLWindow*>(windows.at(i)->widget())->refresh(only2D);
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
		m_ccRoot->getSelectedEntities(m_selectedEntities,CC_TYPES::OBJECT,&selInfo);
	//expandDBTreeWithSelection(m_selectedEntities);

	enableUIItems(selInfo);
}

void MainWindow::expandDBTreeWithSelection(ccHObject::Container& selection)
{
	if (!m_ccRoot)
		return;

	size_t selNum = selection.size();
	for (size_t i=0; i<selNum; ++i)
		m_ccRoot->expandElement(selection[i],true);
}

void MainWindow::enableAll()
{
	QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
	for (int i=0; i<windows.size(); ++i)
		windows.at(i)->setEnabled(true);
}

void MainWindow::disableAll()
{
	QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
	for (int i=0; i<windows.size(); ++i)
		windows.at(i)->setEnabled(false);
}

void MainWindow::disableAllBut(ccGLWindow* win)
{
	//we disable all other windows
	QList<QMdiSubWindow*> windows = m_mdiArea->subWindowList();
	for (int i=0; i<windows.size(); ++i)
		if (static_cast<ccGLWindow*>(windows.at(i)->widget()) != win)
			windows.at(i)->setEnabled(false);
}

void MainWindow::enableUIItems(dbTreeSelectionInfo& selInfo)
{
	bool atLeastOneEntity = (selInfo.selCount > 0);
	bool atLeastOneCloud = (selInfo.cloudCount > 0);
	bool atLeastOneMesh = (selInfo.meshCount > 0);
	//bool atLeastOneOctree = (selInfo.octreeCount > 0);
	bool atLeastOneNormal = (selInfo.normalsCount > 0);
	bool atLeastOneColor = (selInfo.colorCount > 0);
	bool atLeastOneSF = (selInfo.sfCount > 0);
	//bool atLeastOneSensor = (selInfo.sensorCount > 0);
	bool atLeastOneGBLSensor = (selInfo.gblSensorCount > 0);
	bool atLeastOneCameraSensor = (selInfo.cameraSensorCount > 0);
	bool atLeastOnePolyline = (selInfo.polylineCount > 0);
	bool activeWindow = (getActiveGLWindow() != 0);

	//menuEdit->setEnabled(atLeastOneEntity);
	//menuTools->setEnabled(atLeastOneEntity);

	actionZoomAndCenter->setEnabled(atLeastOneEntity && activeWindow);
	actionSave->setEnabled(atLeastOneEntity);
	actionClone->setEnabled(atLeastOneEntity);
	actionDelete->setEnabled(atLeastOneEntity);
	actionExportCoordToSF->setEnabled(atLeastOneEntity);
	actionSegment->setEnabled(atLeastOneEntity && activeWindow);
	actionTranslateRotate->setEnabled(atLeastOneEntity && activeWindow);
	actionShowDepthBuffer->setEnabled(atLeastOneGBLSensor);
	actionExportDepthBuffer->setEnabled(atLeastOneGBLSensor);
	actionComputePointsVisibility->setEnabled(atLeastOneGBLSensor);
	actionResampleWithOctree->setEnabled(atLeastOneCloud);
	actionApplyScale->setEnabled(atLeastOneCloud || atLeastOneMesh || atLeastOnePolyline);
	actionApplyTransformation->setEnabled(atLeastOneEntity);
	actionComputeOctree->setEnabled(atLeastOneCloud || atLeastOneMesh);
	actionComputeNormals->setEnabled(atLeastOneCloud || atLeastOneMesh);
	actionSetColorGradient->setEnabled(atLeastOneCloud || atLeastOneMesh);
	actionChangeColorLevels->setEnabled(atLeastOneCloud || atLeastOneMesh);
	actionEditGlobalShiftAndScale->setEnabled(atLeastOneCloud || atLeastOneMesh || atLeastOnePolyline);
	actionCrop->setEnabled(atLeastOneCloud || atLeastOneMesh);
	actionSetUniqueColor->setEnabled(atLeastOneEntity/*atLeastOneCloud || atLeastOneMesh*/); //DGM: we can set color to a group now!
	actionColorize->setEnabled(atLeastOneEntity/*atLeastOneCloud || atLeastOneMesh*/); //DGM: we can set color to a group now!
	actionScalarFieldFromColor->setEnabled(atLeastOneEntity && atLeastOneColor);
	actionComputeMeshAA->setEnabled(atLeastOneCloud);
	actionComputeMeshLS->setEnabled(atLeastOneCloud);
	//actionComputeQuadric3D->setEnabled(atLeastOneCloud);
	actionComputeBestFitBB->setEnabled(atLeastOneEntity);
	actionComputeDensity->setEnabled(atLeastOneCloud);
	actionCurvature->setEnabled(atLeastOneCloud);
	actionRoughness->setEnabled(atLeastOneCloud);
	actionRemoveDuplicatePoints->setEnabled(atLeastOneCloud);
	actionFitPlane->setEnabled(atLeastOneEntity);
	actionFitSphere->setEnabled(atLeastOneCloud);
	actionLevel->setEnabled(atLeastOneEntity);
	actionFitFacet->setEnabled(atLeastOneEntity);
	actionFitQuadric->setEnabled(atLeastOneCloud);
	actionSubsample->setEnabled(atLeastOneCloud);

	actionSNETest->setEnabled(atLeastOneCloud);
	actionExportCloudsInfo->setEnabled(atLeastOneCloud);

	actionFilterByValue->setEnabled(atLeastOneSF);
	actionConvertToRGB->setEnabled(atLeastOneSF);
	actionConvertToRandomRGB->setEnabled(atLeastOneSF);
	actionRenameSF->setEnabled(atLeastOneSF);
	actionAddIdField->setEnabled(atLeastOneCloud);
	actionComputeStatParams->setEnabled(atLeastOneSF);
	actionComputeStatParams2->setEnabled(atLeastOneSF);
	actionShowHistogram->setEnabled(atLeastOneSF);
	actionGaussianFilter->setEnabled(atLeastOneSF);
	actionBilateralFilter->setEnabled(atLeastOneSF);
	actionDeleteScalarField->setEnabled(atLeastOneSF);
	actionDeleteAllSF->setEnabled(atLeastOneSF);
	actionMultiplySF->setEnabled(/*TODO: atLeastOneSF*/false);
	actionSFGradient->setEnabled(atLeastOneSF);
	actionSetSFAsCoord->setEnabled(atLeastOneSF && atLeastOneCloud);

	actionSamplePoints->setEnabled(atLeastOneMesh);
	actionMeasureMeshSurface->setEnabled(atLeastOneMesh);
	actionMeasureMeshVolume->setEnabled(atLeastOneMesh);
	actionFlagMeshVetices->setEnabled(atLeastOneMesh);
	actionSmoothMeshLaplacian->setEnabled(atLeastOneMesh);
	actionConvertTextureToColor->setEnabled(atLeastOneMesh);
	actionSubdivideMesh->setEnabled(atLeastOneMesh);
	actionDistanceToBestFitQuadric3D->setEnabled(atLeastOneCloud);
	actionDistanceMapToMesh->setEnabled(atLeastOneMesh);

	menuMeshScalarField->setEnabled(atLeastOneSF && atLeastOneMesh);
	//actionSmoothMeshSF->setEnabled(atLeastOneSF && atLeastOneMesh);
	//actionEnhanceMeshSF->setEnabled(atLeastOneSF && atLeastOneMesh);

	actionOrientNormalsMST->setEnabled(atLeastOneCloud && atLeastOneNormal);
	actionOrientNormalsFM->setEnabled(atLeastOneCloud && atLeastOneNormal);
	actionClearNormals->setEnabled(atLeastOneNormal);
	actionInvertNormals->setEnabled(atLeastOneNormal);
	actionConvertNormalToHSV->setEnabled(atLeastOneNormal);
	actionConvertNormalToDipDir->setEnabled(atLeastOneNormal);
	actionClearColor->setEnabled(atLeastOneColor);

	// == 1
	bool exactlyOneEntity = (selInfo.selCount == 1);
	bool exactlyOneGroup = (selInfo.groupCount == 1);
	bool exactlyOneCloud = (selInfo.cloudCount == 1);
	bool exactlyOneMesh = (selInfo.meshCount == 1);
	bool exactlyOneSF = (selInfo.sfCount == 1);
	bool exactlyOneSensor = (selInfo.sensorCount == 1);
	bool exactlyOneCameraSensor = (selInfo.cameraSensorCount == 1);

	actionConvertPolylinesToMesh->setEnabled(atLeastOnePolyline || exactlyOneGroup);
	actionModifySensor->setEnabled(exactlyOneSensor);
	actionComputeDistancesFromSensor->setEnabled(atLeastOneCameraSensor || atLeastOneGBLSensor);
	actionComputeScatteringAngles->setEnabled(exactlyOneSensor);
	actionViewFromSensor->setEnabled(exactlyOneSensor);
	actionCreateGBLSensor->setEnabled(atLeastOneCloud);
	actionCreateCameraSensor->setEnabled(atLeastOneCloud);
	actionProjectUncertainty->setEnabled(exactlyOneCameraSensor);
	actionCheckPointsInsideFrustrum->setEnabled(exactlyOneCameraSensor);
	actionLabelConnectedComponents->setEnabled(atLeastOneCloud);
	actionSORFilter->setEnabled(atLeastOneCloud);
	actionNoiseFilter->setEnabled(atLeastOneCloud);
	actionUnroll->setEnabled(exactlyOneEntity);
	actionStatisticalTest->setEnabled(exactlyOneEntity && exactlyOneSF);
	actionAddConstantSF->setEnabled(exactlyOneCloud || exactlyOneMesh);
	actionEditGlobalScale->setEnabled(exactlyOneCloud || exactlyOneMesh);
	actionComputeKdTree->setEnabled(exactlyOneCloud || exactlyOneMesh);

	actionKMeans->setEnabled(/*TODO: exactlyOneEntity && exactlyOneSF*/false);
	actionFrontPropagation->setEnabled(/*TODO: exactlyOneEntity && exactlyOneSF*/false);

	actionFindBiggestInnerRectangle->setEnabled(exactlyOneCloud);

	menuActiveScalarField->setEnabled((exactlyOneCloud || exactlyOneMesh) && selInfo.sfCount > 0);
	actionCrossSection->setEnabled(exactlyOneCloud || exactlyOneMesh);
	actionExtractSections->setEnabled(atLeastOneCloud);
	actionRasterize->setEnabled(exactlyOneCloud);
	actionCompute2HalfDimVolume->setEnabled(selInfo.cloudCount == selInfo.selCount && selInfo.cloudCount >= 1 && selInfo.cloudCount <= 2); //one or two clouds!

	actionPointListPicking->setEnabled(exactlyOneEntity);

	// == 2
	bool exactlyTwoEntities = (selInfo.selCount == 2);
	bool exactlyTwoClouds = (selInfo.cloudCount == 2);
	//bool exactlyTwoSF = (selInfo.sfCount == 2);

	actionRegister->setEnabled(exactlyTwoEntities);
	actionInterpolateColors->setEnabled(exactlyTwoEntities && atLeastOneColor);
	actionPointPairsAlign->setEnabled(exactlyOneEntity || exactlyTwoEntities);
	actionAlign->setEnabled(exactlyTwoEntities); //Aurelien BEY le 13/11/2008
	actionCloudCloudDist->setEnabled(exactlyTwoClouds);
	actionCloudMeshDist->setEnabled(exactlyTwoEntities && atLeastOneMesh);
	actionCPS->setEnabled(exactlyTwoClouds);
	actionScalarFieldArithmetic->setEnabled(exactlyOneEntity && atLeastOneSF);

	//>1
	bool atLeastTwoEntities = (selInfo.selCount > 1);

	actionMerge->setEnabled(atLeastTwoEntities);
	actionMatchBBCenters->setEnabled(atLeastTwoEntities);
	actionMatchScales->setEnabled(atLeastTwoEntities);

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
	for (int i=0; i<windows.size(); ++i)
	{
		ccGLWindow *child = static_cast<ccGLWindow*>(windows.at(i)->widget());
		if (child != sendingWindow)
		{
			child->blockSignals(true);
			child->onWheelEvent(wheelDelta_deg);
			child->blockSignals(false);
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
	for (int i=0; i<windows.size(); ++i)
	{
		ccGLWindow *child = static_cast<ccGLWindow*>(windows.at(i)->widget());
		if (child != sendingWindow)
		{
			child->blockSignals(true);
			child->moveCamera(ddx,ddy,0.0f);
			child->blockSignals(false);
			child->redraw();
		}
	}
}

void MainWindow::echoBaseViewMatRotation(const ccGLMatrixd& rotMat)
{
	if (checkBoxCameraLink->checkState() != Qt::Checked)
		return;

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

	QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
	for (int i=0; i<windows.size(); ++i)
	{
		ccGLWindow *child = static_cast<ccGLWindow*>(windows.at(i)->widget());
		if (child != sendingWindow)
		{
			child->blockSignals(true);
			child->rotateBaseViewMat(rotMat);
			child->blockSignals(false);
			child->redraw();
		}
	}
}

 void MainWindow::echoCameraPosChanged(const CCVector3d& P)
 {
	 if (checkBoxCameraLink->checkState() != Qt::Checked)
		 return;

	 ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	 if (!sendingWindow)
		 return;

	 QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
	 for (int i=0; i<windows.size(); ++i)
	 {
		 ccGLWindow *child = static_cast<ccGLWindow*>(windows.at(i)->widget());
		 if (child != sendingWindow)
		 {
			 child->blockSignals(true);
			 child->setCameraPos(P);
			 child->blockSignals(false);
			 child->redraw();
		 }
	 }
 }

 void MainWindow::echoPivotPointChanged(const CCVector3d& P)
 {
	 if (checkBoxCameraLink->checkState() != Qt::Checked)
		 return;

	 ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	 if (!sendingWindow)
		 return;

	 QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
	 for (int i=0; i<windows.size(); ++i)
	 {
		 ccGLWindow *child = static_cast<ccGLWindow*>(windows.at(i)->widget());
		 if (child != sendingWindow)
		 {
			 child->blockSignals(true);
			 child->setPivotPoint(P);
			 child->blockSignals(false);
			 child->redraw();
		 }
	 }
 }

 void MainWindow::echoPixelSizeChanged(float pixelSize)
 {
	 if (checkBoxCameraLink->checkState() != Qt::Checked)
		 return;

	 ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	 if (!sendingWindow)
		 return;

	 QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
	 for (int i=0; i<windows.size(); ++i)
	 {
		 ccGLWindow *child = static_cast<ccGLWindow*>(windows.at(i)->widget());
		 if (child != sendingWindow)
		 {
			 child->blockSignals(true);
			 child->setPixelSize(pixelSize);
			 child->blockSignals(false);
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

void MainWindow::doActionKMeans()//TODO
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
	QList<QMdiSubWindow*> windows = TheInstance()->m_mdiArea->subWindowList();
	int winNum = windows.size();

	if (winNum == 0)
		return;

	glWindows.clear();
	glWindows.reserve(winNum);

	for (int i=0; i<winNum; ++i)
		glWindows.push_back(static_cast<ccGLWindow*>(windows.at(i)->widget()));
}

ccGLWindow* MainWindow::GetActiveGLWindow()
{
	return TheInstance()->getActiveGLWindow();
}

ccGLWindow* MainWindow::GetGLWindow(const QString& title)
{
	QList<QMdiSubWindow *> windows = TheInstance()->m_mdiArea->subWindowList();
	int winNum = windows.size();

	if (winNum == 0)
		return 0;

	for (int i=0; i<winNum; ++i)
	{
		ccGLWindow* win = static_cast<ccGLWindow*>(windows.at(i)->widget());
		if (win->windowTitle() == title)
			return win;
	}

	return 0;
}

void MainWindow::RefreshAllGLWindow(bool only2D/*=false*/)
{
	TheInstance()->refreshAll(only2D);
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

ccUniqueIDGenerator::Shared MainWindow::getUniqueIDGenerator()
{
	return ccObject::GetUniqueIDGenerator();
}

MainWindow::ccHObjectContext MainWindow::removeObjectTemporarilyFromDBTree(ccHObject* obj)
{
	ccHObjectContext context;

	assert(obj);
	if (!m_ccRoot || !obj)
		return context;

	//mandatory (to call putObjectBackIntoDBTree)
	context.parent = obj->getParent();

	//remove the object's dependency to its father (in case it undergoes "severe" modifications)
	if (context.parent)
	{
		context.parentFlags = context.parent->getDependencyFlagsWith(obj);
		context.childFlags = obj->getDependencyFlagsWith(context.parent);

		context.parent->removeDependencyWith(obj);
		obj->removeDependencyWith(context.parent);
	}

	m_ccRoot->removeElement(obj);

	return context;
}

void MainWindow::putObjectBackIntoDBTree(ccHObject* obj, const ccHObjectContext& context)
{
	assert(obj);
	if (!obj || !m_ccRoot)
		return;

	if (context.parent)
	{
		context.parent->addChild(obj,context.parentFlags);
		obj->addDependency(context.parent,context.childFlags);
	}

	//DGM: we must call 'notifyGeometryUpdate' as any call to this method
	//while the object was temporarily 'cut' from the DB tree were
	//ineffective!
	obj->notifyGeometryUpdate();

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
