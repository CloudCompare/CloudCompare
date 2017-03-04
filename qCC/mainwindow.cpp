//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "mainwindow.h"

//CCLib Includes
#include <CloudSamplingTools.h>
#include <MeshSamplingTools.h>
#include <ScalarFieldTools.h>
#include <StatisticalTestingTools.h>
#include <WeibullDistribution.h>
#include <NormalDistribution.h>
#include <SimpleCloud.h>
#include <Delaunay2dMesh.h>
#include <Jacobi.h>
#include <SortAlgo.h>

//for tests
#include <ChamferDistanceTransform.h>
#include <SaitoSquaredDistanceTransform.h>

//qCC_db
#include <ccSubMesh.h>
#include <ccKdTree.h>
#include <ccGBLSensor.h>
#include <ccCameraSensor.h>
#include <ccProgressDialog.h>
#include <ccPlane.h>
#include <ccImage.h>
#include <cc2DLabel.h>
#include <cc2DViewportObject.h>
#include <ccColorScalesManager.h>
#include <ccFacet.h>
#include <ccFileUtils.h>
#include <ccQuadric.h>
#include <ccSphere.h>

//qCC_io
#include <ccShiftAndScaleCloudDlg.h>
#include <BinFilter.h>
#include <DepthMapFileFilter.h>

//QCC_glWindow
#include <ccRenderingTools.h>
#include <ccGLWidget.h>

//local includes
#include "ccConsole.h"
#include "ccEntityAction.h"
#include "ccInnerRect2DFinder.h"
#include "ccHistogramWindow.h"

//shaders & Filters
#include <ccShader.h>

//dialogs
#include "ccAboutDialog.h"
#include "ccAdjustZoomDlg.h"
#include "ccAlignDlg.h" //Aurelien BEY
#include "ccApplyTransformationDlg.h"
#include "ccAskThreeDoubleValuesDlg.h"
#include "ccFilterByValueDlg.h"
#include "ccBoundingBoxEditorDlg.h"
#include "ccCameraParamEditDlg.h"
#include "ccCamSensorProjectionDlg.h"
#include "ccClippingBoxTool.h"
#include "ccColorScaleEditorDlg.h"
#include "ccComparisonDlg.h"
#include "ccDisplayOptionsDlg.h"
#include "ccEntityPickerDlg.h"
#include "ccGBLSensorProjectionDlg.h"
#include "ccGraphicalSegmentationTool.h"
#include "ccGraphicalTransformationTool.h"
#include "ccLabelingDlg.h"
#include "ccMatchScalesDlg.h"
#include "ccNoiseFilterDlg.h"
#include "ccOrderChoiceDlg.h"
#include "ccPickOneElementDlg.h"
#include "ccPointListPickingDlg.h"
#include "ccPointPairRegistrationDlg.h"
#include "ccPointPropertiesDlg.h" //Aurelien BEY
#include "ccPrimitiveFactoryDlg.h"
#include "ccPtsSamplingDlg.h"
#include "ccRasterizeTool.h"
#include "ccRegistrationDlg.h" //Aurelien BEY
#include "ccRenderToFileDlg.h"
#include "ccSectionExtractionTool.h"
#include "ccSensorComputeDistancesDlg.h"
#include "ccSensorComputeScatteringAnglesDlg.h"
#include "ccSORFilterDlg.h"
#include "ccStereoModeDlg.h"
#include "ccSubsamplingDlg.h" //Aurelien BEY
#include "ccTracePolylineTool.h"
#include "ccUnrollDlg.h"
#include "ccVolumeCalcTool.h"
#include "ccPluginDlg.h"
#include "ccWaveformDialog.h"
#include "ccPlaneEditDlg.h"
#include "ccPickingHub.h"

//other
#include "ccCropTool.h"
#include "ccPersistentSettings.h"
#include "ccRecentFiles.h"
#include "ccRegistrationTools.h"
#include "ccUtils.h"

//3D mouse handler
#ifdef CC_3DXWARE_SUPPORT
#include "devices/3dConnexion/cc3DMouseManager.h"
#endif

//Gamepads
#ifdef CC_GAMEPADS_SUPPORT
#include "devices/gamepad/ccGamepadManager.h"
#endif

//Qt UI files
#include <ui_distanceMapDlg.h>
#include <ui_globalShiftSettingsDlg.h>

//System
#include <iostream>
#include <random>

//global static pointer (as there should only be one instance of MainWindow!)
static MainWindow* s_instance = 0;

//default 'All files' file filter
static const QString s_allFilesFilter("All (*.*)");
//default file filter separator
static const QString s_fileFilterSeparator(";;");

MainWindow::MainWindow()
	: m_ccRoot(0)
	, m_uiFrozen(false)
	, m_recentFiles(new ccRecentFiles(this))
	, m_3DMouseManager(nullptr)
	, m_gamepadManager(nullptr)
	, m_viewModePopupButton(0)
	, m_pivotVisibilityPopupButton(0)
	, m_FirstShow(true)
	, m_pickingHub(nullptr)
	, m_cpeDlg(0)
	, m_gsTool(0)
	, m_tplTool(0)
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
	setupUi(this);

#ifdef Q_OS_MAC
	actionAbout->setMenuRole( QAction::AboutRole );
	actionAboutPlugins->setMenuRole( QAction::NoRole );

	actionFullScreen->setText( tr( "Enter Full Screen" ) );
	actionFullScreen->setShortcut( QKeySequence( Qt::CTRL + Qt::META + Qt::Key_F ) );
#endif

	menuFile->insertMenu(actionSave, m_recentFiles->menu());

	//Console
	ccConsole::Init(consoleWidget, this, this);
	actionEnableQtWarnings->setChecked(ccConsole::QtMessagesEnabled());

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
			toolBarView->insertWidget(actionZoomAndCenter, m_viewModePopupButton);
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

	//db-tree
	{
		m_ccRoot = new ccDBRoot(dbTreeView, propertiesTreeView, this);
		connect(m_ccRoot, SIGNAL(selectionChanged()), this, SLOT(updateUIWithSelection()));
	}

	//MDI Area
	{
		m_mdiArea = new QMdiArea(this);
		setCentralWidget(m_mdiArea);
		connect(m_mdiArea, SIGNAL(subWindowActivated(QMdiSubWindow*)), this, SLOT(updateMenus()));
		connect(m_mdiArea, SIGNAL(subWindowActivated(QMdiSubWindow*)), this, SLOT(on3DViewActivated(QMdiSubWindow*)));
	}

	//picking hub
	{
		m_pickingHub = new ccPickingHub(this, this);
		connect(m_mdiArea, SIGNAL(subWindowActivated(QMdiSubWindow*)), m_pickingHub, SLOT(onActiveWindowChanged(QMdiSubWindow*)));
	}

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

	new3DView();

	setupInputDevices();

	freezeUI(false);

	updateUIWithSelection();

	QMainWindow::statusBar()->showMessage(QString("Ready"));
	ccConsole::Print("CloudCompare started!");
}

MainWindow::~MainWindow()
{
	destroyInputDevices();

	cancelPreviousPickingOperation(false); //just in case

	assert(m_ccRoot && m_mdiArea && m_windowMapper);
	m_ccRoot->disconnect();
	m_mdiArea->disconnect();
	m_windowMapper->disconnect();

	//we don't want any other dialog/function to use the following structures
	ccDBRoot* ccRoot = m_ccRoot;
	m_ccRoot = 0;
	for (int i = 0; i < getGLWindowCount(); ++i)
	{
		getGLWindow(i)->setSceneDB(0);
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
		ccMDIDialogs mdiDialog = m_mdiDialogs.back();
		m_mdiDialogs.pop_back();

		mdiDialog.dialog->disconnect();
		mdiDialog.dialog->stop(false);
		mdiDialog.dialog->setParent(0);
		delete mdiDialog.dialog;
	}
	//m_mdiDialogs.clear();
	m_mdiArea->closeAllSubWindows();

	if (ccRoot)
		delete ccRoot;

	ccConsole::ReleaseInstance();
}

void MainWindow::dispatchPlugins(const tPluginInfoList& plugins, const QStringList& pluginPaths)
{
	menuPlugins->setEnabled(false);
	menuShadersAndFilters->setEnabled(false);
	toolBarPluginTools->setVisible(false);
	toolBarGLFilters->setVisible(false);

	m_pluginInfoList = plugins;
	m_pluginPaths = pluginPaths;

	for ( const tPluginInfo &plugin : plugins )
	{
		if (!plugin.object)
		{
			assert(false);
			continue;
		}

		QString pluginName = plugin.object->getName();
		if (pluginName.isEmpty())
		{
			ccLog::Warning(QString("[Plugin] Plugin '%1' has an invalid (empty) name!").arg(plugin.filename));
			continue;
		}

		CC_PLUGIN_TYPE type = plugin.object->getType();
		switch (type)
		{

		case CC_STD_PLUGIN: //standard plugin
		{
			plugin.qObject->setParent(this);
			ccStdPluginInterface* stdPlugin = static_cast<ccStdPluginInterface*>(plugin.object);
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
				destToolBar = addToolBar(pluginName + QString(" toolbar"));

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
			foreach (QAction* action, actions.actions())
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

			//last but not least: update the current plugin state
			stdPlugin->onNewSelection(m_selectedEntities);
		}
		break;

		case CC_GL_FILTER_PLUGIN: //GL filter
		{
			//(auto)create action
			plugin.qObject->setParent(this);
			QAction* action = new QAction(pluginName, plugin.qObject);
			action->setToolTip(plugin.object->getDescription());
			action->setIcon(plugin.object->getIcon());
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
			//not handled by the MainWindow instance
			continue;
		}
		break;

		default:
			assert(false);
			continue;
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

void MainWindow::doActionShowAboutPluginsDialog()
{
	ccPluginDlg ccpDlg(m_pluginPaths, m_pluginInfoList, this);
	ccpDlg.exec();
}

void MainWindow::doEnableQtWarnings(bool state)
{
	ccConsole::EnableQtMessages(state);
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
	ccPluginInterface* ccPlugin = ccPlugins::ToValidPlugin(action ? action->parent() : 0);
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

void MainWindow::increasePointSize()
{
	//active window?
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->setPointSize(win->getViewportParameters().defaultPointSize + 1);
		win->redraw();
	}
}

void MainWindow::decreasePointSize()
{
	//active window?
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->setPointSize(win->getViewportParameters().defaultPointSize - 1);
		win->redraw();
	}
}

void MainWindow::setupInputDevices()
{
#ifdef CC_3DXWARE_SUPPORT
	m_3DMouseManager = new cc3DMouseManager( this, this );
	menuFile->insertMenu(actionCloseAll, m_3DMouseManager->menu());
#endif

#ifdef CC_GAMEPADS_SUPPORT
	m_gamepadManager = new ccGamepadManager( this, this );
	menuFile->insertMenu(actionCloseAll, m_gamepadManager->menu());
#endif

#if defined(CC_3DXWARE_SUPPORT) || defined(CC_GAMEPADS_SUPPORT)
	menuFile->insertSeparator(actionCloseAll);
#endif
}

void MainWindow::destroyInputDevices()
{
#ifdef CC_GAMEPADS_SUPPORT
	delete m_gamepadManager;
	m_gamepadManager = nullptr;
#endif

#ifdef CC_3DXWARE_SUPPORT
	delete m_3DMouseManager;
	m_3DMouseManager = nullptr;
#endif
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
	connect(actionGlobalShiftSettings,			SIGNAL(triggered()),	this,		SLOT(doActionGlobalShiftSeetings()));
	connect(actionPrimitiveFactory,				SIGNAL(triggered()),	this,		SLOT(doShowPrimitiveFactory()));
	connect(actionCloseAll,						SIGNAL(triggered()),	this,		SLOT(closeAll()));
	connect(actionQuit,							SIGNAL(triggered()),	this,		SLOT(close()));

	//"Edit > Colors" menu
	connect(actionSetUniqueColor,				SIGNAL(triggered()),	this,		SLOT(doActionSetUniqueColor()));
	connect(actionSetColorGradient,				SIGNAL(triggered()),	this,		SLOT(doActionSetColorGradient()));
	connect(actionChangeColorLevels,			SIGNAL(triggered()),	this,		SLOT(doActionChangeColorLevels()));
	connect(actionColorize,						SIGNAL(triggered()),	this,		SLOT(doActionColorize()));
	connect(actionRGBToGreyScale,				SIGNAL(triggered()),	this,		SLOT(doActionRGBToGreyScale()));
	connect(actionClearColor,					SIGNAL(triggered()),	this,		SLOT(doActionClearColor()));
	connect(actionInterpolateColors,			SIGNAL(triggered()),	this,		SLOT(doActionInterpolateColors()));
	connect(actionEnhanceRGBWithIntensities,	SIGNAL(triggered()),	this,		SLOT(doActionEnhanceRGBWithIntensities()));

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
	connect(actionMeshScanGrids,				SIGNAL(triggered()),	this,		SLOT(doActionMeshScanGrids()));
	connect(actionConvertTextureToColor,		SIGNAL(triggered()),	this,		SLOT(doActionConvertTextureToColor()));
	connect(actionSamplePoints,					SIGNAL(triggered()),	this,		SLOT(doActionSamplePoints()));
	connect(actionSmoothMeshLaplacian,			SIGNAL(triggered()),	this,		SLOT(doActionSmoothMeshLaplacian()));
	connect(actionSubdivideMesh,				SIGNAL(triggered()),	this,		SLOT(doActionSubdivideMesh()));
	connect(actionMeasureMeshSurface,			SIGNAL(triggered()),	this,		SLOT(doActionMeasureMeshSurface()));
	connect(actionMeasureMeshVolume,			SIGNAL(triggered()),	this,		SLOT(doActionMeasureMeshVolume()));
	connect(actionFlagMeshVertices,				SIGNAL(triggered()),	this,		SLOT(doActionFlagMeshVertices()));
	//"Edit > Mesh > Scalar Field" menu
	connect(actionSmoothMeshSF,					SIGNAL(triggered()),	this,		SLOT(doActionSmoothMeshSF()));
	connect(actionEnhanceMeshSF,				SIGNAL(triggered()),	this,		SLOT(doActionEnhanceMeshSF()));
	//"Edit > Plane" menu
	connect(actionCreatePlane,					SIGNAL(triggered()),	this,		SLOT(doActionCreatePlane()));
	connect(actionEditPlane,					SIGNAL(triggered()),	this,		SLOT(doActionEditPlane()));
	//"Edit > Sensor > Ground-Based lidar" menu
	connect(actionShowDepthBuffer,				SIGNAL(triggered()),	this,		SLOT(doActionShowDepthBuffer()));
	connect(actionExportDepthBuffer,			SIGNAL(triggered()),	this,		SLOT(doActionExportDepthBuffer()));
	connect(actionComputePointsVisibility,		SIGNAL(triggered()),	this,		SLOT(doActionComputePointsVisibility()));
	//"Edit > Sensor" menu
	connect(actionCreateGBLSensor,				SIGNAL(triggered()),	this,		SLOT(doActionCreateGBLSensor()));
	connect(actionCreateCameraSensor,			SIGNAL(triggered()),	this,		SLOT(doActionCreateCameraSensor()));
	connect(actionModifySensor,					SIGNAL(triggered()),	this,		SLOT(doActionModifySensor()));
	connect(actionProjectUncertainty,			SIGNAL(triggered()),	this,		SLOT(doActionProjectUncertainty()));
	connect(actionCheckPointsInsideFrustum,		SIGNAL(triggered()),	this,		SLOT(doActionCheckPointsInsideFrustum()));
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
	//"Edit > Waveform" menu
	connect(actionShowWaveDialog,				SIGNAL(triggered()),	this,		SLOT(doActionShowWaveDialog()));
	//"Edit" menu
	connect(actionClone,						SIGNAL(triggered()),	this,		SLOT(doActionClone()));
	connect(actionMerge,						SIGNAL(triggered()),	this,		SLOT(doActionMerge()));
	connect(actionApplyTransformation,			SIGNAL(triggered()),	this,		SLOT(doActionApplyTransformation()));
	connect(actionApplyScale,					SIGNAL(triggered()),	this,		SLOT(doActionApplyScale()));
	connect(actionTranslateRotate,				SIGNAL(triggered()),	this,		SLOT(activateTranslateRotateMode()));
	connect(actionSegment,						SIGNAL(triggered()),	this,		SLOT(activateSegmentationMode()));
    connect(actionTracePolyline,				SIGNAL(triggered()),	this,		SLOT(activateTracePolylineMode()));

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
	connect(actionMeshTwoPolylines,				SIGNAL(triggered()),	this,		SLOT(doMeshTwoPolylines()));
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
	connect(actionDistanceMap,					SIGNAL(triggered()),	this,		SLOT(doActionComputeDistanceMap()));
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
	connect(actionAboutPlugins,					SIGNAL(triggered()),	this,		SLOT(doActionShowAboutPluginsDialog()));
	connect(actionEnableQtWarnings,				SIGNAL(toggled(bool)),	this,		SLOT(doEnableQtWarnings(bool)));

	connect(actionAbout,	&QAction::triggered, [this] () {
		ccAboutDialog* aboutDialog = new ccAboutDialog(this);
		aboutDialog->exec();
	});

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
	if ( !ccEntityAction::setColor(m_selectedEntities, colorize, this) )
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionRGBToGreyScale()
{
	if ( !ccEntityAction::rgbToGreyScale( m_selectedEntities ) )
		return;

	refreshAll();
}

void MainWindow::doActionSetColorGradient()
{
	if ( !ccEntityAction::setColorGradient(m_selectedEntities, this) )
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionChangeColorLevels()
{
	ccEntityAction::changeColorLevels(m_selectedEntities, this);
}

void MainWindow::doActionInterpolateColors()
{
	if ( !ccEntityAction::interpolateColors(m_selectedEntities, this) )
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionEnhanceRGBWithIntensities()
{
	if (!ccEntityAction::enhanceRGBWithIntensities(m_selectedEntities, this))
		return;

	refreshAll();
}


void MainWindow::doActionInvertNormals()
{
	if ( !ccEntityAction::invertNormals(m_selectedEntities) )
		return;

	refreshAll();
}

void MainWindow::doActionConvertNormalsToDipDir()
{
	if ( !ccEntityAction::convertNormalsTo(	m_selectedEntities,
											ccEntityAction::NORMAL_CONVERSION_DEST::DIP_DIR_SFS) )
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionConvertNormalsToHSV()
{
	if ( !ccEntityAction::convertNormalsTo(	m_selectedEntities,
											ccEntityAction::NORMAL_CONVERSION_DEST::HSV_COLORS) )
	{
		return;
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
			ccUtils::DisplayLockedVerticesWarning(ent->getName(),true);
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

	ccProgressDialog pDlg(true, this);

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
#ifdef QT_DEBUG
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
	if ( !ccEntityAction::computeOctree(m_selectedEntities, this) )
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionResampleWithOctree()
{
	bool ok;
	int pointCount = QInputDialog::getInt(this,"Resample with octree", "Points (approx.)", 1000000, 1, INT_MAX, 100000, &ok);
	if (!ok)
		return;

	ccProgressDialog pDlg(false, this);
	pDlg.setAutoClose(false);

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
			ccOctree::Shared octree = cloud->getOctree();
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
			CCLib::GenericIndexedCloud* result = CCLib::CloudSamplingTools::resampleCloudWithOctree
			(
				cloud,
				aimedPoints,
				CCLib::CloudSamplingTools::CELL_GRAVITY_CENTER,
				&pDlg,
				octree.data()
			);

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
	CCVector3d shiftChange(0, 0, 0);
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
	for (size_t i = 0; i < selNum; ++i)
	{
		ccHObject* ent = selectedEntities[i];

		//we don't test primitives (it's always ok while the 'vertices lock' test would fail)
		if (!ent->isKindOf(CC_TYPES::PRIMITIVE))
		{
			//specific test for locked vertices
			bool lockedVertices;
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent, &lockedVertices);
			if (cloud)
			{
				if (lockedVertices)
				{
					ccUtils::DisplayLockedVerticesWarning(ent->getName(), selNum == 1);
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
							int index = sasDlg.addShiftInfo(ccShiftAndScaleCloudDlg::ShiftInfo("Original", globalShift, globalScale));
							//sasDlg.setCurrentProfile(index);
							//add "suggested" entry
							CCVector3d suggestedShift = ccGlobalShiftManager::BestShift(Pg);
							double suggestedScale = ccGlobalShiftManager::BestScale(Dg);
							index = sasDlg.addShiftInfo(ccShiftAndScaleCloudDlg::ShiftInfo("Suggested", suggestedShift, suggestedScale));
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
								shiftChange = (sasDlg.getShift() - cloud->getGlobalShift());

								updateGlobalShiftAndScale = (scaleChange != 1.0 || shiftChange.norm2() != 0);

								//update transformation matrix accordingly
								if (updateGlobalShiftAndScale)
								{
									transMat.scale(scaleChange);
									transMat.setTranslation(transMat.getTranslationAsVec3D() + shiftChange*scaleChange);
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

static CCVector3d s_lastMultFactors(1.0, 1.0, 1.0);
static bool s_lastMultKeepInPlace = true;
typedef std::pair<ccHObject*, ccGenericPointCloud*> EntityCloudAssociation;
void MainWindow::doActionApplyScale()
{
	ccAskThreeDoubleValuesDlg dlg("fx", "fy", "fz", -1.0e6, 1.0e6, s_lastMultFactors.x, s_lastMultFactors.y, s_lastMultFactors.z, 8, "Scaling", this);
	dlg.showCheckbox("Keep in place", s_lastMultKeepInPlace, "Whether the cloud (center) should stay at the same place or not (i.e. coordinates are multiplied directly)");
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
		for (size_t i = 0; i < selNum; ++i)
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
				ccUtils::DisplayLockedVerticesWarning(ent->getName(), selNum == 1);
				//++processNum;
				continue;
			}

			CCVector3 C(0, 0, 0);
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
						if (QMessageBox::question(
							this,
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
			candidates.push_back(EntityCloudAssociation(ent, cloud));
		}
	}

	if (candidates.empty())
	{
		ccConsole::Warning("[Apply scale] No eligible entities (point clouds or meshes) were selected!");
		return;
	}

	//now do the real scaling work
	{
		for (size_t i = 0; i < candidates.size(); ++i)
		{
			ccHObject* ent = candidates[i].first;
			ccGenericPointCloud* cloud = candidates[i].second;

			CCVector3 C(0, 0, 0);
			if (keepInPlace)
			{
				C = cloud->getOwnBB().getCenter();
			}

			//we temporarily detach entity, as it may undergo
			//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::scale
			ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(cloud);

			cloud->scale(	static_cast<PointCoordinateType>(sX),
							static_cast<PointCoordinateType>(sY),
							static_cast<PointCoordinateType>(sZ),
							C );

			putObjectBackIntoDBTree(cloud, objContext);
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
					ccUtils::DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
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
				CCLib::SquareMatrixd eigVectors;
				std::vector<double> eigValues;
				if (Jacobi<double>::ComputeEigenValuesAndVectors(covMat, eigVectors, eigValues, true))
				{
					Jacobi<double>::SortEigenValuesAndVectors(eigVectors, eigValues);

					ccGLMatrix trans;
					GLfloat* rotMat = trans.data();
					for (unsigned j=0; j<3; ++j)
					{
						double u[3];
						Jacobi<double>::GetEigenVector(eigVectors, j, u);
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
	if ( !ccEntityAction::clearProperty( m_selectedEntities,
													 ccEntityAction::CLEAR_PROPERTY::COLORS,
													 this) )
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionClearNormals()
{
	if ( !ccEntityAction::clearProperty( m_selectedEntities,
													 ccEntityAction::CLEAR_PROPERTY::NORMALS,
													 this) )
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionDeleteScalarField()
{
	if ( !ccEntityAction::clearProperty( m_selectedEntities,
													 ccEntityAction::CLEAR_PROPERTY::CURRENT_SCALAR_FIELD,
													 this) )
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionDeleteAllSF()
{
	if ( !ccEntityAction::clearProperty( m_selectedEntities,
													 ccEntityAction::CLEAR_PROPERTY::ALL_SCALAR_FIELDS,
													 this) )
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionFlagMeshVertices()
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

void MainWindow::doActionMeasureMeshVolume()
{
	size_t selNum = m_selectedEntities.size();
	for (size_t i = 0; i<selNum; ++i)
	{
		ccHObject* ent = m_selectedEntities[i];
		if (ent->isKindOf(CC_TYPES::MESH))
		{
			ccMesh* mesh = ccHObjectCaster::ToMesh(ent);
			if (mesh)
			{
				//we compute the mesh volume
				double V = CCLib::MeshSamplingTools::computeMeshVolume(mesh);
				//we force the console to display itself
				forceConsoleDisplay();
				ccConsole::Print(QString("[Mesh Volume] Mesh '%1': V=%2 (cube units)").arg(ent->getName()).arg(V));

				//check that the mesh is closed
				CCLib::MeshSamplingTools::EdgeConnectivityStats stats;
				if (CCLib::MeshSamplingTools::computeMeshEdgesConnectivity(mesh, stats))
				{
					if (stats.edgesNotShared != 0)
					{
						ccConsole::Warning(QString("[Mesh Volume] The above volume might be invalid (mesh has holes)"));
					}
					else if (stats.edgesSharedByMore != 0)
					{
						ccConsole::Warning(QString("[Mesh Volume] The above volume might be invalid (mesh has non-manifold edges)"));
					}
				}
				else
				{
					ccConsole::Warning(QString("[Mesh Volume] The above volume might be invalid (not enough memory to check if the mesh is closed)"));
				}
			}
			else
			{
				assert(false);
			}
		}
	}
}

void MainWindow::doActionMeasureMeshSurface()
{
	size_t selNum = m_selectedEntities.size();
	for (size_t i = 0; i < selNum; ++i)
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
				ccConsole::Print(QString("[Mesh Surface] Mesh '%1': S=%2 (square units)").arg(ent->getName()).arg(S));
				if (mesh->size())
				{
					ccConsole::Print(QString("[Mesh Surface] Average triangle surface: %1 (square units)").arg(S / double(mesh->size())));
				}
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
	//we create the camera sensor
	ccCameraSensor* sensor = new ccCameraSensor();

	ccHObject* ent = 0;
	if (!m_selectedEntities.empty())
	{
		assert(m_selectedEntities.size() == 1);
		ent = m_selectedEntities.front();
	}

	//we try to guess the sensor relative size (dirty)
	if (ent && ent->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);
		ccBBox bb = cloud->getOwnBB();
		double diag = bb.getDiagNorm();
		if (diag < 1.0)
			sensor->setGraphicScale(static_cast<PointCoordinateType>(1.0e-3));
		else if (diag > 10000.0)
			sensor->setGraphicScale(static_cast<PointCoordinateType>(1.0e3));

		//set position
		ccIndexedTransformation trans;
		sensor->addPosition(trans, 0);
	}

	ccCamSensorProjectionDlg spDlg(this);
	spDlg.updateCamSensor(sensor);
	if (!spDlg.exec())
	{
		delete sensor;
		return;
	}

	ccGLWindow* win = 0;
	if (ent)
	{
		ent->addChild(sensor);
		win = static_cast<ccGLWindow*>(ent->getDisplay());
	}
	else
	{
		win = getActiveGLWindow();
	}

	if (win)
	{
		sensor->setDisplay(win);
		sensor->setVisible(true);
		if (ent)
		{
			ccBBox box = ent->getOwnBB();
			win->updateConstellationCenterAndZoom(&box);
		}
	}

	addToDB(sensor);

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
			ccLog::Error("An error occurred! (see console)");
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
			ccLog::Error("An error occurred! (see console)");
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

void MainWindow::doActionCheckPointsInsideFrustum()
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
	ccOctree::Shared octree = pointCloud->getOctree();
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
	std::vector<unsigned> inCameraFrustum;
	if (!octree->intersectWithFrustum(sensor, inCameraFrustum))
	{
		ccConsole::Error("Failed to intersect sensor frustum with octree!");
	}
	else
	{
		// scalar field
		const char sfName[] = "Frustum visibility";
		int sfIdx = pointCloud->getScalarFieldIndexByName(sfName);

		if (inCameraFrustum.empty())
		{
			ccConsole::Error("No point fell inside the frustum!");
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

				for (size_t i = 0; i < inCameraFrustum.size(); i++)
				{
					sf->setValue(inCameraFrustum[i], c_insideValue);
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
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	QString filename = QFileDialog::getSaveFileName(this, "Select output file", currentPath, DepthMapFileFilter::GetFileFilter());
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
		pdlg.setMethodTitle(tr("Compute visibility"));
		pdlg.setInfo(tr("Points: %1").arg(pointCloud->size()));
		pdlg.start();
		QApplication::processEvents();

		for (unsigned i = 0; i < pointCloud->size(); i++)
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
	if ( !ccEntityAction::convertTextureToColor(m_selectedEntities, this) )
		return;

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

	ccProgressDialog pDlg(false, this);
	pDlg.setAutoClose(false);

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

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	for (size_t i = 0; i<selNum; ++i)
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

			ccOctree::Shared octree = cloud->getOctree();

			int result = CCLib::GeometricalAnalysisTools::flagDuplicatePoints(	cloud,
																				minDistanceBetweenPoints,
																				&pDlg,
																				octree.data());

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
						m_ccRoot->selectEntity(filteredCloud, true);
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

	ccFilterByValueDlg dlg(minVald, maxVald, -1.0e9, 1.0e9, this);
	if (!dlg.exec())
		return;

	ccFilterByValueDlg::Mode mode = dlg.mode();
	assert(mode != ccFilterByValueDlg::CANCEL);

	ScalarType minVal = static_cast<ScalarType>(dlg.minDoubleSpinBox->value());
	ScalarType maxVal = static_cast<ScalarType>(dlg.maxDoubleSpinBox->value());

	ccHObject::Container results;
	{
		for (size_t i = 0; i < toFilter.size(); ++i)
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

			ccHObject* resultInside = 0;
			ccHObject* resultOutside = 0;
			if (ent->isKindOf(CC_TYPES::MESH))
			{
				pc->hidePointsByScalarValue(minVal, maxVal);
				if (ent->isA(CC_TYPES::MESH)/*|| ent->isKindOf(CC_TYPES::PRIMITIVE)*/) //TODO
					resultInside = ccHObjectCaster::ToMesh(ent)->createNewMeshFromSelection(false);
				else if (ent->isA(CC_TYPES::SUB_MESH))
					resultInside = ccHObjectCaster::ToSubMesh(ent)->createNewSubMeshFromSelection(false);

				if (mode == ccFilterByValueDlg::SPLIT)
				{
					pc->invertVisibilityArray();
					if (ent->isA(CC_TYPES::MESH)/*|| ent->isKindOf(CC_TYPES::PRIMITIVE)*/) //TODO
						resultOutside = ccHObjectCaster::ToMesh(ent)->createNewMeshFromSelection(false);
					else if (ent->isA(CC_TYPES::SUB_MESH))
						resultOutside = ccHObjectCaster::ToSubMesh(ent)->createNewSubMeshFromSelection(false);
				}

				pc->unallocateVisibilityArray();
			}
			else if (ent->isKindOf(CC_TYPES::POINT_CLOUD))
			{
				//pc->hidePointsByScalarValue(minVal,maxVal);
				//result = ccHObjectCaster::ToGenericPointCloud(ent)->hidePointsByScalarValue(false);
				//pc->unallocateVisibilityArray();

				//shortcut, as we know here that the point cloud is a "ccPointCloud"
				resultInside = pc->filterPointsByScalarValue(minVal, maxVal, false);

				if (mode == ccFilterByValueDlg::SPLIT)
				{
					resultOutside = pc->filterPointsByScalarValue(minVal, maxVal, true);
				}
			}

			if (resultInside)
			{
				ent->setEnabled(false);
				resultInside->setDisplay(ent->getDisplay());
				resultInside->prepareDisplayForRefresh();
				addToDB(resultInside);

				results.push_back(resultInside);
			}
			if (resultOutside)
			{
				ent->setEnabled(false);
				resultOutside->setDisplay(ent->getDisplay());
				resultOutside->prepareDisplayForRefresh();
				resultOutside->setName(resultOutside->getName() + ".outside");
				addToDB(resultOutside);

				results.push_back(resultOutside);
			}
			//*/
		}
	}

	if (!results.empty())
	{
		ccConsole::Warning("Previously selected entities (sources) have been hidden!");
		if (m_ccRoot)
		{
			m_ccRoot->selectEntities(results);
		}
	}

	refreshAll();
}

void MainWindow::doActionSFConvertToRandomRGB()
{
	if ( !ccEntityAction::sfConvertToRandomRGB(m_selectedEntities, this) )
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionSFConvertToRGB()
{
	if ( !ccEntityAction::sfConvertToRGB(m_selectedEntities, this) )
		return;

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
		ccUtils::DisplayLockedVerticesWarning(ent->getName(),true);
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
	if ( !ccEntityAction::sfRename(m_selectedEntities, this) )
		return;

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
	if ( !ccEntityAction::sfAddIdField(m_selectedEntities) )
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionSFGaussianFilter()
{
	if ( !ccEntityAction::sfGaussianFilter(m_selectedEntities, this) )
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionSFBilateralFilter()
{
	if ( !ccEntityAction::sfBilateralFilter(m_selectedEntities, this) )
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionSmoothMeshSF()
{
	if ( !ccEntityAction::processMeshSF(m_selectedEntities, ccMesh::SMOOTH_MESH_SF, this) )
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionEnhanceMeshSF()
{
	if ( !ccEntityAction::processMeshSF(m_selectedEntities, ccMesh::ENHANCE_MESH_SF, this) )
		return;

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

	//ccProgressDialog pDlg(true, this);
	//pDlg.setAutoClose(false);

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
					mesh->redrawDisplay();
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

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

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

		for (size_t i = 0; i < clouds.size(); ++i)
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
		for (size_t i = 0; i < meshes.size(); ++i)
		{
			ccMesh* mesh = meshes[i];

			//if (mesh->isA(CC_TYPES::PRIMITIVE))
			//{
			//	mesh = mesh->ccMesh::cloneMesh(); //we want a clone of the mesh part, not the primitive!
			//}

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
	int maxThreadCount											= rDlg.getMaxThreadCount();

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
									finalOverlap / 100.0,
									useDataSFAsWeights,
									useModelSFAsWeights,
									transformationFilters,
									maxThreadCount,
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
		QString overlapString = QString("Theoretical overlap: %1%").arg(finalOverlap);
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

	ccProgressDialog pDlg(true, this);

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
		ccProgressDialog pDlg(false, this);
		pDlg.setAutoClose(false);

		pDlg.setMethodTitle(tr("Subsampling"));

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

				newPointCloud->prepareDisplayForRefresh();
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
	if ( !ccEntityAction::statisticalTest(m_selectedEntities, this ) )
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionComputeStatParams()
{
	ccEntityAction::computeStatParams(m_selectedEntities, this );
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
			for (unsigned i = 0; i < compCount; ++i)
			{
				sortedIndexes.push_back(ComponentIndexAndSize(i, components[i]->size()));
			}

			SortAlgo(sortedIndexes.begin(), sortedIndexes.end(), ComponentIndexAndSize::DescendingCompOperator);
			_sortedIndexes = &sortedIndexes;
		}
	}

	//we create "real" point clouds for all input components
	{
		ccPointCloud* pc = cloud->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(cloud) : 0;

		//we create a new group to store all CCs
		ccHObject* ccGroup = new ccHObject(cloud->getName()+QString(" [CCs]"));

		//for each component
		for (size_t i = 0; i < components.size(); ++i)
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
						m_ccRoot->selectEntity(compCloud, true);
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

			ccConsole::Print(QString("[createComponentsClouds] %1 component(s) were created from cloud '%2'").arg(ccGroup->getChildrenNumber()).arg(cloud->getName()));
		}

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
	unsigned minComponentSize = static_cast<unsigned>(std::max(0, dlg.getMinPointsNb()));
	bool randColors = dlg.randomColors();

	ccProgressDialog pDlg(false, this);
	pDlg.setAutoClose(false);

	//we unselect all entities as we are going to automatically select the created components
	//(otherwise the user won't perceive the change!)
	if (m_ccRoot)
	{
		m_ccRoot->unselectAllEntities();
	}

	for (size_t i = 0; i < count; ++i)
	{
		ccGenericPointCloud* cloud = clouds[i];

		if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD)) //TODO
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

			ccOctree::Shared theOctree = cloud->getOctree();
			if (!theOctree)
			{
				ccProgressDialog pOctreeDlg(true, this);
				theOctree = cloud->computeOctree(&pOctreeDlg);
				if (!theOctree)
				{
					ccConsole::Error(QString("Couldn't compute octree for cloud '%s'!").arg(cloud->getName()));
					break;
				}
			}

			//we create/activate CCs label scalar field
			int sfIdx = pc->getScalarFieldIndexByName(CC_CONNECTED_COMPONENTS_DEFAULT_LABEL_NAME);
			if (sfIdx < 0)
			{
				sfIdx = pc->addScalarField(CC_CONNECTED_COMPONENTS_DEFAULT_LABEL_NAME);
			}
			if (sfIdx < 0)
			{
				ccConsole::Error("Couldn't allocate a new scalar field for computing CC labels! Try to free some memory ...");
				break;
			}
			pc->setCurrentScalarField(sfIdx);

			//we try to label all CCs
			CCLib::ReferenceCloudContainer components;
			int componentCount = CCLib::AutoSegmentationTools::labelConnectedComponents(cloud,
																						static_cast<unsigned char>(octreeLevel),
																						false,
																						&pDlg,
																						theOctree.data());

			if (componentCount >= 0)
			{
				//if successful, we extract each CC (stored in "components")

				//safety test
				int realComponentCount = 0;
				{
					for (size_t i = 0; i < components.size(); ++i)
					{
						if (components[i]->size() >= minComponentSize)
						{
							++realComponentCount;
						}
					}
				}

				if (realComponentCount > 500)
				{
					//too many components
					if (QMessageBox::warning(this, "Many components", QString("Do you really expect up to %1 components?\n(this may take a lot of time to process and display)").arg(realComponentCount), QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
					{
						//cancel
						pc->deleteScalarField(sfIdx);
						if (pc->getNumberOfScalarFields() != 0)
						{
							pc->setCurrentDisplayedScalarField(static_cast<int>(pc->getNumberOfScalarFields()) - 1);
						}
						else
						{
							pc->showSF(false);
						}
						pc->prepareDisplayForRefresh();
						continue;
					}
				}

				pc->getCurrentInScalarField()->computeMinAndMax();
				if (!CCLib::AutoSegmentationTools::extractConnectedComponents(cloud, components))
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
	if ( !ccEntityAction::sfSetAsCoord(m_selectedEntities, this) )
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionExportCoordToSF()
{
	if ( !ccEntityAction::exportCoordToSF(m_selectedEntities, this) )
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doMeshTwoPolylines()
{
	size_t selNum = m_selectedEntities.size();
	if (selNum != 2)
		return;

	ccPolyline* p1 = ccHObjectCaster::ToPolyline(m_selectedEntities[0]);
	ccPolyline* p2 = ccHObjectCaster::ToPolyline(m_selectedEntities[1]);
	if (!p1 || !p2)
	{
		ccConsole::Error("Select 2 and only 2 polylines");
		return;
	}

	//Ask the user how the 2D projection should be computed
	bool useViewingDir = false;
	CCVector3 viewingDir(0, 0, 0);
	if (p1->getDisplay())
	{
		useViewingDir = (QMessageBox::question(this, "Projection method", "Use best fit plane (yes) or the current viewing direction (no)", QMessageBox::Yes, QMessageBox::No) == QMessageBox::No);
		if (useViewingDir)
		{
			viewingDir = -CCVector3::fromArray(static_cast<ccGLWindow*>(p1->getDisplay())->getCurrentViewDir().u);
		}
	}

	ccMesh* mesh = ccMesh::TriangulateTwoPolylines(p1, p2, useViewingDir ? &viewingDir : 0);
	if (mesh)
	{
		addToDB(mesh);
		if (mesh->computePerVertexNormals())
		{
			mesh->showNormals(true);
		}
		else
		{
			ccLog::Warning("[Mesh two polylines] Failed to compute normals!");
		}

		if (mesh->getDisplay())
		{
			mesh->getDisplay()->redraw();
		}
	}
	else
	{
		ccLog::Error("Failed to create mesh (see Console)");
		forceConsoleDisplay();
	}
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
			for (unsigned i = 0; i < obj->getChildrenNumber(); ++i)
			{
				if (obj->getChild(i)->isA(CC_TYPES::POLY_LINE))
					polylines.push_back(static_cast<ccPolyline*>(obj->getChild(i)));
			}
		}
		else
		{
			for (size_t i = 0; i < selNum; ++i)
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

	ccPickOneElementDlg poeDlg("Projection dimension", "Contour plot to mesh", this);
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
		for (size_t i = 0; i < polylines.size(); ++i)
		{
			ccPolyline* poly = polylines[i];
			if (poly)
			{
				//count the total number of vertices and segments
				vertexCount += poly->size();
				segmentCount += poly->segmentCount();
			}
		}
	}

	if (segmentCount < 2)
	{
		//not enough points/segments
		ccLog::Error("Not enough segments!");
		return;
	}

	//we assume we link with CGAL now (if not the call to Delaunay2dMesh::buildMesh will fail anyway)
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
		for (size_t i = 0; i < polylines.size(); ++i)
		{
			ccPolyline* poly = polylines[i];
			if (poly)
			{
				unsigned vertCount = poly->size();
				int vertIndex0 = static_cast<int>(points2D.size());
				bool closed = poly->isClosed();
				for (unsigned v = 0; v < vertCount; ++v)
				{
					const CCVector3* P = poly->getPoint(v);
					int vertIndex = static_cast<int>(points2D.size());
					points2D.push_back(CCVector2(P->u[X], P->u[Y]));

					if (v + 1 < vertCount)
					{
						segments2D.push_back(vertIndex);
						segments2D.push_back(vertIndex + 1);
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
		assert(segments2D.size() == segmentCount * 2);
	}

	CCLib::Delaunay2dMesh* delaunayMesh = new CCLib::Delaunay2dMesh;
	char errorStr[1024];
	if (!delaunayMesh->buildMesh(points2D, segments2D, errorStr))
	{
		ccLog::Error(QString("Third party library error: %1").arg(errorStr));
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
		for (size_t i = 0; i < polylines.size(); ++i)
		{
			ccPolyline* poly = polylines[i];
			unsigned vertCount = poly->size();
			for (unsigned v = 0; v < vertCount; ++v)
			{
				const CCVector3* P = poly->getPoint(v);
				vertices->addPoint(*P);
			}
		}
		delaunayMesh->linkMeshWith(vertices, false);
	}

#ifdef QT_DEBUG
	//Test delaunay output
	{
		unsigned vertCount = vertices->size();
		for (unsigned i = 0; i < delaunayMesh->size(); ++i)
		{
			const CCLib::VerticesIndexes* tsi = delaunayMesh->getTriangleVertIndexes(i);
			assert(tsi->i1 < vertCount && tsi->i2 < vertCount && tsi->i3 < vertCount);
		}
	}
#endif

	ccMesh* mesh = new ccMesh(delaunayMesh, vertices);
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
		{
			mesh->showNormals(true);
		}
		else
		{
			ccLog::Warning("[Contour plot to mesh] Failed to compute normals!");
		}

		if (mesh->getDisplay())
		{
			mesh->getDisplay()->redraw();
		}

		//global shift & scale (we copy it from the first polyline by default)
		vertices->setGlobalShift(polylines.front()->getGlobalShift());
		vertices->setGlobalScale(polylines.front()->getGlobalScale());
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

void MainWindow::doActionMeshScanGrids()
{
	//ask the user for the min angle (inside triangles)
	static double s_meshMinTriangleAngle_deg = 1.0;
	{
		bool ok = true;
		double minAngle_deg = QInputDialog::getDouble(this, "Triangulate", "Min triangle angle (in degrees)", s_meshMinTriangleAngle_deg, 0, 90.0, 3, &ok);
		if (!ok)
			return;
		s_meshMinTriangleAngle_deg = minAngle_deg;
	}

	//look for clouds with scan grids
	for (ccHObject* ent : m_selectedEntities)
	{
		if (!ent || !ent->isA(CC_TYPES::POINT_CLOUD))
		{
			continue;
		}

		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
		assert(cloud);

		for (size_t i = 0; i < cloud->gridCount(); ++i)
		{
			ccPointCloud::Grid::Shared grid = cloud->grid(i);
			if (!grid)
			{
				assert(false);
				continue;
			}

			ccMesh* gridMesh = cloud->triangulateGrid(*grid, s_meshMinTriangleAngle_deg);
			if (gridMesh)
			{
				cloud->addChild(gridMesh);
				cloud->setVisible(false); //hide the cloud
				gridMesh->setDisplay(cloud->getDisplay());
				addToDB(gridMesh, false, true, false, false);
				gridMesh->prepareDisplayForRefresh();
			}
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionComputeMeshAA()
{
	doActionComputeMesh(DELAUNAY_2D_AXIS_ALIGNED);
}

void MainWindow::doActionComputeMeshLS()
{
	doActionComputeMesh(DELAUNAY_2D_BEST_LS_PLANE);
}

void MainWindow::doActionComputeMesh(CC_TRIANGULATION_TYPES type)
{
	//ask the user for the max edge length
	static double s_meshMaxEdgeLength = 0;
	{
		bool ok = true;
		double maxEdgeLength = QInputDialog::getDouble(this, "Triangulate", "Max edge length (0 = no limit)", s_meshMaxEdgeLength, 0, 1.0e9, 8, &ok);
		if (!ok)
			return;
		s_meshMaxEdgeLength = maxEdgeLength;
	}

	//select candidates
	ccHObject::Container clouds;
	bool hadNormals = false;
	{
		for (size_t i = 0; i < m_selectedEntities.size(); ++i)
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

	ccProgressDialog pDlg(false, this);
	pDlg.setAutoClose(false);
	pDlg.setWindowTitle(tr("Triangulation"));
	pDlg.setInfo(tr("Triangulation in progress..."));
	pDlg.setRange(0, 0);
	pDlg.show();
	QApplication::processEvents();

	bool errors = false;
	for (size_t i = 0; i < clouds.size(); ++i)
	{
		ccHObject* ent = clouds[i];
		assert(ent->isKindOf(CC_TYPES::POINT_CLOUD));

		//compute mesh
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);
		ccMesh* mesh = ccMesh::Triangulate(	cloud,
											type,
											updateNormals,
											static_cast<PointCoordinateType>(s_meshMaxEdgeLength),
											2 //XY plane by default
											);
		if (mesh)
		{
			cloud->setVisible(false); //can't disable the cloud as the resulting mesh will be its child!
			cloud->addChild(mesh);
			cloud->prepareDisplayForRefresh_recursive();
			addToDB(mesh);
			if (i == 0)
			{
				m_ccRoot->selectEntity(mesh); //auto-select first element
			}
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
	static unsigned steps = 128;
	static double margin = 0.0;
	static bool filterRange = false;
	static double range[2] = { 0.0, 1.0 };

	//show dialog
	{
		QDialog dialog(this);
		Ui_DistanceMapDialog ui;
		ui.setupUi(&dialog);

		ui.stepsSpinBox->setValue(static_cast<int>(steps));
		ui.marginDoubleSpinBox->setValue(margin);
		ui.rangeCheckBox->setChecked(filterRange);
		ui.minDistDoubleSpinBox->setValue(range[0]);
		ui.maxDistDoubleSpinBox->setValue(range[1]);

		if (!dialog.exec())
		{
			return;
		}

		steps = static_cast<unsigned>(ui.stepsSpinBox->value());
		margin = ui.marginDoubleSpinBox->value();
		filterRange = ui.rangeCheckBox->isChecked();
		range[0] = ui.minDistDoubleSpinBox->value();
		range[1] = ui.maxDistDoubleSpinBox->value();
	}

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	size_t selNum = m_selectedEntities.size();
	for (size_t i = 0; i < selNum; ++i)
	{
		ccHObject* ent = m_selectedEntities[i];
		if (!ent->isKindOf(CC_TYPES::MESH) && !ent->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			//non handled entity type
			continue;
		}

		//CCLib::ChamferDistanceTransform cdt;
		CCLib::SaitoSquaredDistanceTransform cdt;
		if (!cdt.initGrid(Tuple3ui(steps, steps, steps)))
		{
			//not enough memory
			ccLog::Error("Not enough memory!");
			return;
		}

		ccBBox box = ent->getOwnBB();
		PointCoordinateType largestDim = box.getMaxBoxDim() + static_cast<PointCoordinateType>(margin);
		PointCoordinateType cellDim = largestDim / steps;
		CCVector3 minCorner = box.getCenter() - CCVector3(1, 1, 1) * (largestDim / 2);

		bool result = false;
		if (ent->isKindOf(CC_TYPES::MESH))
		{
			ccMesh* mesh = static_cast<ccMesh*>(ent);
			result = cdt.initDT(mesh, cellDim, minCorner, &pDlg);
		}
		else
		{
			ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(ent);
			result = cdt.initDT(cloud, cellDim, minCorner, &pDlg);
		}

		if (!result)
		{
			ccLog::Error("Not enough memory!");
			return;
		}

		//cdt.propagateDistance(CHAMFER_345, &pDlg);
		cdt.propagateDistance(&pDlg);

		//convert the grid to a cloud
		ccPointCloud* gridCloud = new ccPointCloud(ent->getName() + QString(".distance_grid(%1)").arg(steps));
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
						ScalarType d = sqrt(static_cast<ScalarType>(cdt.getValue(i, j, k))) * cellDim;

						if (!filterRange || (d >= range[0] && d <= range[1]))
						{
							gridCloud->addPoint(minCorner + CCVector3(i + 0.5, j + 0.5, k + 0.5) * cellDim);
							sf->addElement(d);
						}
					}
				}
			}

			sf->computeMinAndMax();
			int sfIdx = gridCloud->addScalarField(sf);

			if (gridCloud->size() == 0)
			{
				ccLog::Warning(QString("[DistanceMap] Cloud '%1': no point falls inside the specified range").arg(ent->getName()));
				delete gridCloud;
				gridCloud = 0;
			}
			else
			{
				gridCloud->setCurrentDisplayedScalarField(sfIdx);
				gridCloud->showSF(true);
				gridCloud->setDisplay(ent->getDisplay());
				gridCloud->shrinkToFit();
				ent->prepareDisplayForRefresh();
				addToDB(gridCloud);
			}
		}
	}

	refreshAll();
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
					ccConsole::Warning(QString("Failed to get the center of gravity of cloud '%1'!").arg(cloud->getName()));
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

							sf->addElement(dist);
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
	ccProgressDialog pDlg(true, this);
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
	if ( !ccEntityAction::computeNormals(m_selectedEntities, this) )
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionOrientNormalsMST()
{
	if ( !ccEntityAction::orientNormalsMST(m_selectedEntities, this) )
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionOrientNormalsFM()
{
	if ( !ccEntityAction::orientNormalsFM(m_selectedEntities, this) )
		return;

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
static ccLibAlgorithms::ScaleMatchingAlgorithm s_msAlgorithm = ccLibAlgorithms::PCA_MAX_DIM;
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
	if (s_msAlgorithm == ccLibAlgorithms::ICP_SCALE)
	{
		s_msRmsDiff = msDlg.rmsDifferenceLineEdit->text().toDouble();
		s_msFinalOverlap = msDlg.overlapSpinBox->value();
	}

	ccLibAlgorithms::ApplyScaleMatchingAlgorithm(	s_msAlgorithm,
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

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	size_t selNum = m_selectedEntities.size();
	bool firstCloud = true;

	for (size_t i = 0; i < selNum; ++i)
	{
		ccHObject* ent = m_selectedEntities[i];

		//specific test for locked vertices
		bool lockedVertices;
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent, &lockedVertices);
		if (cloud && lockedVertices)
		{
			ccUtils::DisplayLockedVerticesWarning(ent->getName(), selNum == 1);
			continue;
		}

		//computation
		CCLib::ReferenceCloud* selection = CCLib::CloudSamplingTools::sorFilter(cloud,
																				s_sorFilterKnn,
																				s_sorFilterNSigma,
																				0,
																				&pDlg);

		if (selection && cloud)
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
					cleanCloud->setName(cloud->getName() + QString(".clean"));
					cleanCloud->setDisplay(cloud->getDisplay());
					if (cloud->getParent())
						cloud->getParent()->addChild(cleanCloud);
					addToDB(cleanCloud);

					cloud->setEnabled(false);
					if (firstCloud)
					{
						ccConsole::Warning("Previously selected entities (sources) have been hidden!");
						firstCloud = false;
						m_ccRoot->selectEntity(cleanCloud, true);
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
	PointCoordinateType kernelRadius = ccLibAlgorithms::GetDefaultCloudKernelSize(m_selectedEntities);

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

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

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
			ccUtils::DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
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

		if (selection && cloud)
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
						m_ccRoot->selectEntity(cleanCloud, true);
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
	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities[0], &lockedVertices);
	if (lockedVertices)
	{
		ccUtils::DisplayLockedVerticesWarning(m_selectedEntities[0]->getName(), true);
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
	unrollDlg.fromPersistentSettings();
	if (!unrollDlg.exec())
		return;
	unrollDlg.toPersistentSettings();

	ccUnrollDlg::Type mode = unrollDlg.getType();
	PointCoordinateType radius = static_cast<PointCoordinateType>(unrollDlg.getRadius());
	double angle_deg = unrollDlg.getAngle();
	unsigned char dim = static_cast<unsigned char>(unrollDlg.getAxisDimension());
	bool exportDeviationSF = unrollDlg.exportDeviationSF();
	CCVector3* pCenter = 0;
	CCVector3 center;
	if (mode != ccUnrollDlg::CYLINDER || !unrollDlg.isAxisPositionAuto())
	{
		//we need the axis center point
		center = unrollDlg.getAxisPosition();
		pCenter = &center;
	}

	//let's rock unroll ;)
	ccProgressDialog pDlg(true, this);

	ccPointCloud* output = 0;
	switch (mode)
	{
	case ccUnrollDlg::CYLINDER:
		output = pc->unrollOnCylinder(radius, dim, pCenter, exportDeviationSF, &pDlg);
		break;
	case ccUnrollDlg::CONE:
		output = pc->unrollOnCone(angle_deg, center, dim, false, 0, exportDeviationSF, &pDlg);
		break;
	case ccUnrollDlg::STRAIGHTENED_CONE:
		output = pc->unrollOnCone(angle_deg, center, dim, true, radius, exportDeviationSF, &pDlg);
		break;
	default:
		assert(false);
		break;
	}

	if (output)
	{
		pc->setEnabled(false);
		ccConsole::Warning("[Unroll] Original cloud has been automatically hidden");

		if (pc->getParent())
		{
			pc->getParent()->addChild(output);
		}
		addToDB(output, true, true, false, true);

		updateUI();
	}
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
		return GLWindowFromWidget(activeSubWindow->widget());
	}
	else
	{
		QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
		if (!subWindowList.isEmpty())
		{
			return GLWindowFromWidget(subWindowList[0]->widget());
		}
	}

	return 0;
}

int MainWindow::getGLWindowCount() const
{
	return m_mdiArea ? m_mdiArea->subWindowList().size() : 0;
}

ccGLWindow* MainWindow::getGLWindow(int index) const
{
	QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
	if (index >= 0 && index < subWindowList.size())
	{
		ccGLWindow* win = GLWindowFromWidget(subWindowList[index]->widget());
		assert(win);
		return win;
	}
	else
	{
		assert(false);
		return 0;
	}
}

QMdiSubWindow* MainWindow::getMDISubWindow(ccGLWindow* win)
{
	QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
	for (int i = 0; i < subWindowList.size(); ++i)
	{
		if (GLWindowFromWidget(subWindowList[i]->widget()) == win)
			return subWindowList[i];
	}

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

	QWidget* viewWidget = nullptr;
	ccGLWindow* view3D = nullptr;
	createGLWindow(view3D, viewWidget);
	assert(viewWidget && view3D);

	viewWidget->setMinimumSize(400, 300);

	m_mdiArea->addSubWindow(viewWidget);

	connect(view3D,	SIGNAL(entitySelectionChanged(ccHObject*)),					m_ccRoot,	SLOT(selectEntity(ccHObject*)));
	connect(view3D,	SIGNAL(entitiesSelectionChanged(std::unordered_set<int>)),	m_ccRoot,	SLOT(selectEntities(std::unordered_set<int>)));

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

	if (m_pickingHub)
	{
		//we must notify the picking hub as well if the window is destroyed
		connect(view3D, SIGNAL(destroyed(QObject*)), m_pickingHub, SLOT(onActiveWindowDeleted(QObject*)));
	}

	view3D->setSceneDB(m_ccRoot->getRootEntity());
	viewWidget->setAttribute(Qt::WA_DeleteOnClose);
	m_ccRoot->updatePropertiesView();

	QMainWindow::statusBar()->showMessage(QString("New 3D View"), 2000);

	viewWidget->showMaximized();
	viewWidget->update();

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
	// show the user it will be maximized
	showMaximized();

	QSettings settings;
	settings.remove(ccPS::MainWinGeom());
	settings.remove(ccPS::MainWinState());

	QMessageBox::information(this,"Restart","To finish the process, you'll have to close and restart CloudCompare");

	//to avoid saving them right away!
	s_autoSaveGuiElementPos = false;
}

void MainWindow::showEvent(QShowEvent* event)
{
	QMainWindow::showEvent( event );

	if ( m_FirstShow )
	{
		QSettings settings;
		QVariant  geometry = settings.value(ccPS::MainWinGeom());

		if ( geometry.isValid() )
		{
			restoreGeometry(geometry.toByteArray());
			restoreState(settings.value(ccPS::MainWinState()).toByteArray());
		}

		m_FirstShow = false;

		if ( !geometry.isValid() )
		{
			showMaximized();
		}

		if ( isFullScreen() )
		{
			actionFullScreen->setChecked( true );
		}

#ifdef Q_OS_MAC
		if ( isFullScreen() )
		{
			actionFullScreen->setText( tr( "Exit Full Screen" ) );
		}
		else
		{
			actionFullScreen->setText( tr( "Enter Full Screen" ) );
		}
#endif

		//special warning message for high DPI (Retina) displays
		if (devicePixelRatio() != 1.0)
		{
			//we always show a warning in the console
			ccLog::Warning("High pixel density screen detected: point picking and label display won't work properly!");
			//but the first time we also show an annoying warning popup ;)
			QMetaObject::invokeMethod(this, "showHighDPIScreenWarning", Qt::QueuedConnection);
		}
	}
}

void MainWindow::showHighDPIScreenWarning()
{
	QSettings settings;
	if (settings.value("HighDPIScreenWarningIssued", false).toBool())
	{
		//message already issued
		return;
	}

	QMessageBox* msgBox = new QMessageBox(this);
	msgBox->setWindowTitle("High DPI screen detected");
	msgBox->setIcon(QMessageBox::Warning);
	msgBox->setText("Your screen (Apple Retina?) seems to have a high pixel density: point picking and label display won't work properly!");
	msgBox->setInformativeText("(Note from Daniel: if you want this issue to be solved fast, you should send a Retina screen to Andy ;)");
	msgBox->setStandardButtons(QMessageBox::Ignore | QMessageBox::Ok);
	msgBox->setDefaultButton(QMessageBox::Ignore);

	//Man, I love lambdas!
	connect(msgBox->button(QMessageBox::Ok), &QAbstractButton::clicked, []()
		{
			//don't annoy the user ever again
			QSettings settings;
			settings.setValue("HighDPIScreenWarningIssued", true);
		}
	);

	//DGM: don't 'exec' this dialog, otherwise it will block the main loop
	//and it will prevent the spash screen from disappearing!
	msgBox->show();
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

	updateOverlayDialogsPlacement();
}

void MainWindow::resizeEvent(QResizeEvent* event)
{
	QMainWindow::resizeEvent(event);

	updateOverlayDialogsPlacement();
}

void MainWindow::registerOverlayDialog(ccOverlayDialog* dlg, Qt::Corner pos)
{
	//check for existence
	for (ccMDIDialogs& mdi : m_mdiDialogs)
	{
		if (mdi.dialog == dlg)
		{
			//we only update its position in this case
			mdi.position = pos;
			repositionOverlayDialog(mdi);
			return;
		}
	}

	//otherwise we add it to DB
	m_mdiDialogs.push_back(ccMDIDialogs(dlg, pos));

	//automatically update the dialog placement when its shown
	connect(dlg, &ccOverlayDialog::shown, [&]() { updateOverlayDialogPlacement(dlg); });

	repositionOverlayDialog(m_mdiDialogs.back());
}

void MainWindow::updateOverlayDialogPlacement(ccOverlayDialog* dlg)
{
	//check for existence
	for (ccMDIDialogs& mdi : m_mdiDialogs)
	{
		if (mdi.dialog == dlg)
		{
			repositionOverlayDialog(mdi);
			break;
		}
	}
}

void MainWindow::unregisterOverlayDialog(ccOverlayDialog* dialog)
{
	for (std::vector<ccMDIDialogs>::iterator it = m_mdiDialogs.begin(); it != m_mdiDialogs.end(); ++it)
	{
		if (it->dialog == dialog)
		{
			m_mdiDialogs.erase(it);
			break;
		}
	}
}

void MainWindow::updateOverlayDialogsPlacement()
{
	for (ccMDIDialogs& mdiDlg : m_mdiDialogs)
	{
		repositionOverlayDialog(mdiDlg);
	}
}

void MainWindow::repositionOverlayDialog(ccMDIDialogs& mdiDlg)
{
	if (!mdiDlg.dialog || !mdiDlg.dialog->isVisible() || !m_mdiArea)
		return;

	int dx = 0, dy = 0;
	static const int margin = 5;
	switch (mdiDlg.position)
	{
	case Qt::TopLeftCorner:
		dx = margin;
		dy = margin;
		break;
	case Qt::TopRightCorner:
		dx = std::max(margin, m_mdiArea->width() - mdiDlg.dialog->width() - margin);
		dy = margin;
		break;
	case Qt::BottomLeftCorner:
		dx = margin;
		dy = std::max(margin, m_mdiArea->height() - mdiDlg.dialog->height() - margin);
		break;
	case Qt::BottomRightCorner:
		dx = std::max(margin, m_mdiArea->width() - mdiDlg.dialog->width() - margin);
		dy = std::max(margin, m_mdiArea->height() - mdiDlg.dialog->height() - margin);
		break;
	}

	//show();
	mdiDlg.dialog->move(m_mdiArea->mapToGlobal(QPoint(dx, dy)));
	mdiDlg.dialog->raise();
}

void MainWindow::toggleVisualDebugTraces()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->toggleDebugTrace();
		win->redraw(false, false);
	}
}

void MainWindow::toggleFullScreen(bool state)
{
	if (state)
		showFullScreen();
	else
		showNormal();

#ifdef Q_OS_MAC
	if ( state )
	{
		actionFullScreen->setText( tr( "Exit Full Screen" ) );
	}
	else
	{
		actionFullScreen->setText( tr( "Enter Full Screen" ) );
	}
#endif
}

void MainWindow::toggleExclusiveFullScreen(bool state)
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->toggleExclusiveFullScreen(state);
	}
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
		m_pprDlg = new ccPointPairRegistrationDlg(m_pickingHub, this);
		connect(m_pprDlg, SIGNAL(processFinished(bool)), this, SLOT(deactivateRegisterPointPairTool(bool)));
		registerOverlayDialog(m_pprDlg, Qt::TopRightCorner);
	}

	ccGLWindow* win = new3DView();
	if (!win)
	{
		ccLog::Error("[PointPairRegistration] Failed to create dedicated 3D view!");
		return;
	}

	if (!m_pprDlg->init(win, aligned, reference))
		deactivateRegisterPointPairTool(false);

	freezeUI(true);

	if (!m_pprDlg->start())
		deactivateRegisterPointPairTool(false);
	else
		updateOverlayDialogsPlacement();
}

void MainWindow::deactivateRegisterPointPairTool(bool state)
{
	if (m_pprDlg)
		m_pprDlg->clear();

	//we enable all GL windows
	enableAll();

	QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
	if (!subWindowList.isEmpty())
		subWindowList.first()->showMaximized();

	freezeUI(false);

	updateUI();

	setGlobalZoom();
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

		registerOverlayDialog(m_seTool, Qt::TopRightCorner);
	}

	//add clouds
	ccGLWindow* firstDisplay = 0;
	{
		unsigned validCount = 0;
		for (size_t i = 0; i < selNum; ++i)
		{
			if (m_selectedEntities[i]->isKindOf(CC_TYPES::POINT_CLOUD))
			{
				if (m_seTool->addCloud(static_cast<ccGenericPointCloud*>(m_selectedEntities[i])))
				{
					if (!firstDisplay && m_selectedEntities[i]->getDisplay())
						firstDisplay = static_cast<ccGLWindow*>(m_selectedEntities[i]->getDisplay());
					++validCount;
				}
			}
		}

		if (validCount == 0)
		{
			ccConsole::Error("No cloud in selection!");
			return;
		}
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

	//warning: we must disable the entity picking signal!
	win->disconnect(m_ccRoot, SLOT(selectEntity(ccHObject*)));

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
		updateOverlayDialogsPlacement();
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

		registerOverlayDialog(m_gsTool, Qt::TopRightCorner);
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
		updateOverlayDialogsPlacement();
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
		std::unordered_set<ccGenericPointCloud*> verticesToReset;

		QSet<ccHObject*>& segmentedEntities = m_gsTool->entities();
		for (QSet<ccHObject*>::iterator p = segmentedEntities.begin(); p != segmentedEntities.end(); )
		{
			ccHObject* entity = (*p);

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
					{
						m_ccRoot->getRootEntity()->filterChildren(labels,true,CC_TYPES::LABEL_2D);
					}
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
								ccLog::Warning(QString("[Segmentation] Label %1 depends on cloud %2 and will be removed").arg(label->getName()).arg(cloud->getName()));
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
									{
										//either transfer
										entity->transferChild(sensor,*segmentationResult);
									}
									else
									{
										//or copy
										segmentationResult->addChild(new ccGBLSensor(*sensor));
									}
								}
								else if (child->isA(CC_TYPES::CAMERA_SENSOR))
								{
									ccCameraSensor* sensor = ccHObjectCaster::ToCameraSensor(entity->getChild(i));
									if (deleteOriginalEntity)
									{
										//either transfer
										entity->transferChild(sensor,*segmentationResult);
									}
									else
									{
										//or copy
										segmentationResult->addChild(new ccCameraSensor(*sensor));
									}
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
							putObjectBackIntoDBTree(entity, objContext);
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
							{
								verticesToReset.insert(meshEntity->getAssociatedCloud());
							}
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
						{
							objContext.parent = objContext.parent->getParent();
						}
					}

					if (objContext.parent)
					{
						objContext.parent->addChild(segmentationResult); //FiXME: objContext.parentFlags?
					}

					segmentationResult->setDisplay_recursive(entity->getDisplay());
					segmentationResult->prepareDisplayForRefresh_recursive();

					addToDB(segmentationResult);

					if (!firstResult)
					{
						firstResult = segmentationResult;
					}
				}
				else if (!deleteOriginalEntity)
				{
					//ccConsole::Error("An error occurred! (not enough memory?)");
					putObjectBackIntoDBTree(entity,objContext);
				}

				if (deleteOriginalEntity)
				{
					p = segmentedEntities.erase(p);

					delete entity;
					entity = 0;
				}
				else
				{
					++p;
				}
			}
		}

		//specific actions
		{
			for (std::unordered_set<ccGenericPointCloud*>::const_iterator p = verticesToReset.begin(); p != verticesToReset.end(); ++p)
			{
				(*p)->resetVisibilityArray();
			}
		}

		if (firstResult && m_ccRoot)
		{
			m_ccRoot->selectEntity(firstResult);
		}
	}

	if (m_gsTool)
	{
		m_gsTool->removeAllEntities(!deleteHiddenParts);
	}

	//we enable all GL windows
	enableAll();

	freezeUI(false);

	updateUI();

	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->redraw();
	}
}

void MainWindow::activateTracePolylineMode()
{
	ccGLWindow* win = getActiveGLWindow();
	if (!win)
	{
		return;
	}

	if (!m_tplTool)
	{
		m_tplTool = new ccTracePolylineTool(m_pickingHub, this);
		connect(m_tplTool, SIGNAL(processFinished(bool)), this, SLOT(deactivateTracePolylineMode(bool)));
		registerOverlayDialog(m_tplTool, Qt::TopRightCorner);
	}

	m_tplTool->linkWith(win);

	freezeUI(true);
	toolBarView->setDisabled(false);

	//we disable all other windows
	disableAllBut(win);

	if (!m_tplTool->start())
		deactivateTracePolylineMode(false);
	else
		updateOverlayDialogsPlacement();
}

void MainWindow::deactivateTracePolylineMode(bool)
{
	//we enable all GL windows
	enableAll();

	freezeUI(false);

	updateUI();

	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->redraw();
	}
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
		m_plpDlg = new ccPointListPickingDlg(m_pickingHub, this);
		connect(m_plpDlg, SIGNAL(processFinished(bool)), this, SLOT(deactivatePointListPickingMode(bool)));

		registerOverlayDialog(m_plpDlg, Qt::TopRightCorner);
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
		updateOverlayDialogsPlacement();
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
		m_ppDlg = new ccPointPropertiesDlg(m_pickingHub, this);
		connect(m_ppDlg, SIGNAL(processFinished(bool)),	this, SLOT(deactivatePointPickingMode(bool)));
		connect(m_ppDlg, SIGNAL(newLabel(ccHObject*)),	this, SLOT(handleNewLabel(ccHObject*)));

		registerOverlayDialog(m_ppDlg, Qt::TopRightCorner);
	}

	m_ppDlg->linkWith(win);

	freezeUI(true);

	//we disable all other windows
	disableAllBut(win);

	if (!m_ppDlg->start())
		deactivatePointPickingMode(false);
	else
		updateOverlayDialogsPlacement();
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
	{
		return;
	}

	ccGLWindow* win = getActiveGLWindow();
	if (!win)
	{
		return;
	}

	if (!m_clipTool)
	{
		m_clipTool = new ccClippingBoxTool(this);
		connect(m_clipTool, SIGNAL(processFinished(bool)), this, SLOT(deactivateClippingBoxMode(bool)));
	}
	m_clipTool->linkWith(win);

	ccHObject::Container selectedEntities = m_selectedEntities;
	for (ccHObject* entity : selectedEntities)
	{
		if (m_clipTool->addAssociatedEntity(entity))
		{
			//automatically deselect the entity (to avoid seeing its bounding box ;)
			m_ccRoot->unselectEntity(entity);
		}
	}

	if (m_clipTool->getNumberOfAssociatedEntity() == 0)
	{
		m_clipTool->close();
		return;
	}

	if (m_clipTool->start())
	{
		registerOverlayDialog(m_clipTool, Qt::TopRightCorner);
		freezeUI(true);
		updateOverlayDialogsPlacement();
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
		registerOverlayDialog(m_transTool, Qt::TopRightCorner);
		freezeUI(true);
		updateOverlayDialogsPlacement();
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

	ccRenderToFileDlg rtfDlg(win->glWidth(), win->glHeight(), this);

	if (rtfDlg.exec())
	{
		QApplication::processEvents();
		win->renderToFile(rtfDlg.getFilename(), rtfDlg.getZoom(), rtfDlg.dontScalePoints(), rtfDlg.renderOverlayItems());
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
		m_cpeDlg = new ccCameraParamEditDlg(qWin, m_pickingHub);
		//m_cpeDlg->makeFrameless(); //does not work on linux

		connect(m_mdiArea, SIGNAL(subWindowActivated(QMdiSubWindow*)), m_cpeDlg, SLOT(linkWith(QMdiSubWindow*)));

		registerOverlayDialog(m_cpeDlg, Qt::BottomLeftCorner);
	}

	m_cpeDlg->linkWith(qWin);
	m_cpeDlg->start();

	updateOverlayDialogsPlacement();
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

void MainWindow::setGlobalZoom()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
		win->zoomGlobal();
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
		win->setPerspectiveState(false, true);
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
		win->setPerspectiveState(true, true);
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

enum PickingOperation {	NO_PICKING_OPERATION,
						PICKING_ROTATION_CENTER,
						PICKING_LEVEL_POINTS,
};
static ccGLWindow* s_pickingWindow = 0;
static PickingOperation s_currentPickingOperation = NO_PICKING_OPERATION;
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

	assert(m_pickingHub);
	if (!m_pickingHub->addListener(this))
	{
		ccLog::Error("Can't start the picking mechanism (another tool is already using it)");
		return;
	}

	//specific case: we prevent the 'point-pair based alignment' tool to process the picked point!
	//if (m_pprDlg)
	//	m_pprDlg->pause(true);

	s_pickingWindow = win;
	win->displayNewMessage(message, ccGLWindow::LOWER_LEFT_MESSAGE, true, 24 * 3600);
	win->redraw(true, false);

	freezeUI(true);
}

void MainWindow::cancelPreviousPickingOperation(bool aborted)
{
	if (!s_pickingWindow)
		return;

	switch(s_currentPickingOperation)
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
		s_pickingWindow->displayNewMessage(QString(), ccGLWindow::LOWER_LEFT_MESSAGE); //clear previous messages
		s_pickingWindow->displayNewMessage("Picking operation aborted", ccGLWindow::LOWER_LEFT_MESSAGE);
	}
	s_pickingWindow->redraw(false);

	//specific case: we allow the 'point-pair based alignment' tool to process the picked point!
	if (m_pprDlg)
		m_pprDlg->pause(false);

	freezeUI(false);

	m_pickingHub->removeListener(this);

	s_pickingWindow = 0;
	s_currentPickingOperation = NO_PICKING_OPERATION;
}

void MainWindow::onItemPicked(const PickedItem& pi)
{
	if (!s_pickingWindow || !m_pickingHub)
	{
		return;
	}

	if (!pi.entity)
	{
		return;
	}

	if (m_pickingHub->activeWindow() != s_pickingWindow)
	{
		ccLog::Warning("The point picked was picked in the wrong window");
		return;
	}

	CCVector3 pickedPoint = pi.P3D;
	switch(s_currentPickingOperation)
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

			for (unsigned i = 0; i < s_levelMarkersCloud->size(); ++i)
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
			label->addPoint(s_levelMarkersCloud, markerCount - 1);
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
				CCVector3 X = *B - *A;
				CCVector3 Y = *C - *A;
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
				trans.apply(T);
				T += CCVector3d::fromArray(A->u);
				trans.setTranslation(T);

				assert(m_selectedEntities.size() == 1 && m_selectedEntities.front() == s_levelEntity);
				applyTransformation(trans);

				//clear message
				s_pickingWindow->displayNewMessage(QString(), ccGLWindow::LOWER_LEFT_MESSAGE, false); //clear previous message
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
					pi.entity->getGLTransformation().apply(newPivot);
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
		if (s_currentPickingOperation == PICKING_LEVEL_POINTS)
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
	s_currentPickingOperation = PICKING_LEVEL_POINTS;

	enablePickingOperation(win,"Pick three points on the floor plane (click on icon/menu entry again to cancel)");
}

void MainWindow::doPickRotationCenter()
{
	//picking operation already in progress
	if (s_pickingWindow)
	{
		if (s_currentPickingOperation == PICKING_ROTATION_CENTER)
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

	s_currentPickingOperation = PICKING_ROTATION_CENTER;
	enablePickingOperation(win,"Pick a point to be used as rotation center (click on icon again to cancel)");
}

void MainWindow::toggleSelectedEntitiesActivation()
{
	if ( !ccEntityAction::toggleProperty(	m_selectedEntities,
											ccEntityAction::TOGGLE_PROPERTY::ACTIVE) )
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::toggleSelectedEntitiesVisibility()
{
	if ( !ccEntityAction::toggleProperty(	m_selectedEntities,
											ccEntityAction::TOGGLE_PROPERTY::VISIBLE) )
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::toggleSelectedEntitiesColors()
{
	if ( !ccEntityAction::toggleProperty(	m_selectedEntities,
											ccEntityAction::TOGGLE_PROPERTY::COLOR) )
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::toggleSelectedEntitiesNormals()
{
	if ( !ccEntityAction::toggleProperty(	m_selectedEntities,
											ccEntityAction::TOGGLE_PROPERTY::NORMALS) )
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::toggleSelectedEntitiesSF()
{
	if ( !ccEntityAction::toggleProperty(	m_selectedEntities,
											ccEntityAction::TOGGLE_PROPERTY::SCALAR_FIELD) )
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::toggleSelectedEntitiesMaterials()
{
	if ( !ccEntityAction::toggleProperty(	m_selectedEntities,
											ccEntityAction::TOGGLE_PROPERTY::MATERIAL) )
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::toggleSelectedEntities3DName()
{
	if ( !ccEntityAction::toggleProperty(	m_selectedEntities,
											ccEntityAction::TOGGLE_PROPERTY::NAME) )
	{
		return;
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
	{
		m_ccRoot->selectEntity(lastClone);
	}

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
		ccUtils::DisplayLockedVerticesWarning(ent->getName(),true);
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

	ScalarType sfValue = static_cast<ScalarType>(QInputDialog::getDouble(this, "Add constant value", "value", s_constantSFValue, -1.0e9, 1.0e9, 8, &ok));
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

void MainWindow::doActionScalarFieldFromColor()
{
	if ( !ccEntityAction::sfFromColor(m_selectedEntities, this) )
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionScalarFieldArithmetic()
{
	if ( !ccEntityAction::sfArithmetic(m_selectedEntities, this) )
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionFitSphere()
{
	size_t selNum = m_selectedEntities.size();

	double outliersRatio = 0.5;
	double confidence = 0.99;

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	for (size_t i = 0; i<selNum; ++i)
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
		maxEdgeLength = QInputDialog::getDouble(this, "Fit facet", "Max edge length (0 = no limit)", s_polygonMaxEdgeLength, 0, 1.0e9, 8, &ok);
		if (!ok)
			return;
		s_polygonMaxEdgeLength = maxEdgeLength;
	}

	for (size_t i = 0; i < selNum; ++i)
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
			CCVector3 C, N;

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
				ccConsole::Print("\t- plane fitting RMS: %f", rms);

				//We always consider the normal with a positive 'Z' by default!
				if (N.z < 0.0)
					N *= -1.0;
				ccConsole::Print("\t- normal: (%f,%f,%f)", N.x, N.y, N.z);

				//we compute strike & dip by the way
				PointCoordinateType dip = 0, dipDir = 0;
				ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);
				QString dipAndDipDirStr = ccNormalVectors::ConvertDipAndDipDirToString(dip, dipDir);
				ccConsole::Print(QString("\t- %1").arg(dipAndDipDirStr));

				//hack: output the transformation matrix that would make this normal points towards +Z
				ccGLMatrix makeZPosMatrix = ccGLMatrix::FromToRotation(N, CCVector3(0, 0, PC_ONE));
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
	if (!ccLibAlgorithms::ApplyCCLibAlgorithm(ccLibAlgorithms::CCLIB_ALGO_ACCURATE_DENSITY,m_selectedEntities,this))
		return;
	refreshAll();
	updateUI();
}

void MainWindow::doComputeCurvature()
{
	if (!ccLibAlgorithms::ApplyCCLibAlgorithm(ccLibAlgorithms::CCLIB_ALGO_CURVATURE, m_selectedEntities, this))
		return;
	refreshAll();
	updateUI();
}

void MainWindow::doActionSFGradient()
{
	if (!ccLibAlgorithms::ApplyCCLibAlgorithm(ccLibAlgorithms::CCLIB_ALGO_SF_GRADIENT, m_selectedEntities, this))
		return;
	refreshAll();
	updateUI();
}

void MainWindow::doComputeRoughness()
{
	if (!ccLibAlgorithms::ApplyCCLibAlgorithm(ccLibAlgorithms::CCLIB_ALGO_ROUGHNESS, m_selectedEntities, this))
		return;
	refreshAll();
	updateUI();
}

void MainWindow::doSphericalNeighbourhoodExtractionTest()
{
	if (!ccLibAlgorithms::ApplyCCLibAlgorithm(ccLibAlgorithms::CCLIB_SPHERICAL_NEIGHBOURHOOD_EXTRACTION_TEST,m_selectedEntities,this))
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
		std::random_device rd;   // non-deterministic generator
		std::mt19937 gen(rd());  // to seed mersenne twister.
		std::uniform_real_distribution<double> dist(0, 1);

		for (unsigned i=0; i<ptsCount; ++i)
		{
			CCVector3 P(	dist(gen),
							dist(gen),
							dist(gen) );

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

	ccProgressDialog pDlg(true, this);
	ccOctree::Shared octree = cloud->computeOctree(&pDlg);
	if (octree)
	{
		QElapsedTimer subTimer;
		subTimer.start();
		unsigned long long extractedPoints = 0;
		unsigned char level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(static_cast<PointCoordinateType>(2.5*radius)); //2.5 = empirical
		const unsigned samples = 1000;
		std::random_device rd;   // non-deterministic generator
		std::mt19937 gen(rd());  // to seed mersenne twister.
		std::uniform_real_distribution<PointCoordinateType> distAngle(0, static_cast<PointCoordinateType>(2 * M_PI));
		std::uniform_int_distribution<unsigned> distIndex(0, ptsCount - 1);

		for (unsigned j=0; j<samples; ++j)
		{
			//generate random normal vector
			CCVector3 dir(0,0,1);
			{
				ccGLMatrix rot;
				rot.initFromParameters(	distAngle(gen),
										distAngle(gen),
										distAngle(gen),
										CCVector3(0,0,0) );
				rot.applyRotation(dir);
			}
			unsigned randIndex = distIndex(gen);

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
			{
				cloud->setPointScalarValue(cn.neighbours[k].pointIndex, static_cast<ScalarType>(sqrt(cn.neighbours[k].squareDistd)));
			}
		}
		ccConsole::Print("[CNE_TEST] Mean extraction time = %i ms (radius = %f, height = %f, mean(neighbours) = %3.1f)", subTimer.elapsed(), radius, height, static_cast<double>(extractedPoints) / samples);
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
	std::vector< std::pair<double, double> > matrixAngles;
	try
	{
		rmsMatrix.resize(cloudCount*cloudCount,0);

		//init all possible transformations
		static const double angularStep_deg = 45.0;
		unsigned phiSteps = static_cast<unsigned>(360.0 / angularStep_deg);
		assert(fabs(360.0 - phiSteps * angularStep_deg) < ZERO_TOLERANCE);
		unsigned thetaSteps = static_cast<unsigned>(180.0 / angularStep_deg);
		assert(fabs(180.0 - thetaSteps * angularStep_deg) < ZERO_TOLERANCE);
		unsigned rotCount = phiSteps * (thetaSteps - 1) + 2;
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
		ccProgressDialog pDlg(true, this);
		pDlg.setMethodTitle(tr("Testing all possible positions"));
		pDlg.setInfo(tr("%1 clouds and %2 positions").arg(cloudCount).arg(matrices.size()));
		CCLib::NormalizedProgress nProgress(&pDlg, static_cast<unsigned>(((cloudCount*(cloudCount - 1)) / 2)*matrices.size()));
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
					CCLib::ICPRegistrationTools::Parameters params;
					{
						params.convType = CCLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE;
						params.minRMSDecrease = 1.0e-6;
					}

					result = CCLib::ICPRegistrationTools::Register(A, 0, B, params, registerTrans, finalRMS, finalPointCount);

					if (result >= CCLib::ICPRegistrationTools::ICP_ERROR)
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
		QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

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
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

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
		{
			m_ccRoot->selectEntity(compEntity);
		}
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

			if (win->getStereoParams().glassType == ccGLWindow::StereoParams::NVIDIA_VISION)
			{
				//disable (exclusive) full screen
				actionExclusiveFullScreen->setChecked(false);
			}
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

			ccGLWindow::StereoParams params = smDlg.getParameters();
#ifndef CC_GL_WINDOW_USE_QWINDOW
			if (!params.isAnaglyph())
			{
				ccLog::Error("This version doesn't handle stereo glasses and headsets.\nUse the 'Stereo' version instead.");
				//activation of the stereo mode failed: cancel selection
				actionEnableStereo->blockSignals(true);
				actionEnableStereo->setChecked(false);
				actionEnableStereo->blockSignals(false);
				return;
			}
#endif

			//force perspective state!
			if (!win->getViewportParameters().perspectiveView)
			{
				setCenteredPerspectiveView(win, false);
			}

			if (params.glassType == ccGLWindow::StereoParams::NVIDIA_VISION)
			{
				//force (exclusive) full screen
				actionExclusiveFullScreen->setChecked(true);
			}

			if (!win->enableStereoMode(params))
			{
				if (params.glassType == ccGLWindow::StereoParams::NVIDIA_VISION)
				{
					//disable (exclusive) full screen
					actionExclusiveFullScreen->setChecked(false);
				}

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
		ccGLWindow::StereoParams params = win->getStereoParams();
		bool wasExclusiveFullScreen = win->exclusiveFullScreen();
		if (wasExclusiveFullScreen)
		{
			win->toggleExclusiveFullScreen(false);
		}
		win->disableStereoMode();

		if (QMessageBox::question(	this,
									"Stereo mode",
									"Stereo-mode only works in perspective mode. Do you want to disable it?",
									QMessageBox::Yes,
									QMessageBox::No) == QMessageBox::No )
		{
			if (wasExclusiveFullScreen)
			{
				win->toggleExclusiveFullScreen(true);
				win->enableStereoMode(params);
			}
			return false;
		}
		else
		{
			if (win == getActiveGLWindow())
			{
				actionEnableStereo->setChecked(false);
			}
			else
			{
				assert(false);
				actionEnableStereo->blockSignals(true);
				actionEnableStereo->setChecked(false);
				actionEnableStereo->blockSignals(false);
			}
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
			win->displayNewMessage(QString("[ROTATION LOCKED]"), ccGLWindow::UPPER_CENTER_MESSAGE, false, 24 * 3600, ccGLWindow::ROTAION_LOCK_MESSAGE);
		}
		else
		{
			win->displayNewMessage(QString(), ccGLWindow::UPPER_CENTER_MESSAGE, false, 0, ccGLWindow::ROTAION_LOCK_MESSAGE);
		}
		win->redraw(true, false);
	}
}

void MainWindow::doActionEnableBubbleViewMode()
{
	//special case: the selected entity is a TLS sensor or a cloud with a TLS sensor
	if (m_ccRoot)
	{
		ccHObject::Container selectedEntities;
		m_ccRoot->getSelectedEntities(selectedEntities);

		if (selectedEntities.size() == 1)
		{
			ccHObject* ent = selectedEntities.front();
			ccGBLSensor* sensor = 0;
			if (ent->isA(CC_TYPES::GBL_SENSOR))
			{
				sensor = static_cast<ccGBLSensor*>(ent);
			}
			else if (ent->isA(CC_TYPES::POINT_CLOUD))
			{
				ccHObject::Container sensors;
				ent->filterChildren(sensors, false, CC_TYPES::GBL_SENSOR, true);
				if (sensors.size() >= 1)
				{
					sensor = static_cast<ccGBLSensor*>(sensors.front());
				}
			}

			if (sensor)
			{
				sensor->applyViewport();
				return;
			}
		}
	}

	//otherwise we simply enable the bubble view mode in the active 3D view
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
		CCVector3d Pshift(0, 0, 0);
		double scale = 1.0;
		//here we must test that coordinates are not too big whatever the case because OpenGL
		//really doesn't like big ones (even if we work with GLdoubles :( ).
		if (ccGlobalShiftManager::Handle(P, diag, ccGlobalShiftManager::DIALOG_IF_NECESSARY, false, Pshift, &scale))
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

				for (unsigned i = 0; i < child->getChildrenNumber(); ++i)
				{
					children.push_back(child->getChild(i));
				}
			}
		}
	}

	//add object to DB root
	if (m_ccRoot)
	{
		//force a 'global zoom' if the DB was emtpy!
		if (!m_ccRoot->getRootEntity() || m_ccRoot->getRootEntity()->getChildrenNumber() == 0)
		{
			updateZoom = true;
		}
		m_ccRoot->addElement(obj, autoExpandDBTree);
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
	//to use the same 'global shift' for multiple files
	CCVector3d loadCoordinatesShift(0,0,0);
	bool loadCoordinatesTransEnabled = false;

	FileIOFilter::LoadParameters parameters;
	{
		parameters.alwaysDisplayLoadDialog = true;
		parameters.shiftHandlingMode = ccGlobalShiftManager::DIALOG_IF_NECESSARY;
		parameters.coordinatesShift = &loadCoordinatesShift;
		parameters.coordinatesShiftEnabled = &loadCoordinatesTransEnabled;
		parameters.parentWidget = this;
	}

	//the same for 'addToDB' (if the first one is not supported, or if the scale remains too big)
	CCVector3d addCoordinatesShift(0, 0, 0);

	for (int i = 0; i < filenames.size(); ++i)
	{
		CC_FILE_ERROR result = CC_FERR_NO_ERROR;
		ccHObject* newGroup = FileIOFilter::LoadFromFile(filenames[i], parameters, result, fileFilter);

		if (newGroup)
		{
			if (destWin)
			{
				newGroup->setDisplay_recursive(destWin);
			}
			addToDB(newGroup, true, true, false);

			m_recentFiles->addFilePath( filenames[i] );
		}

		if (result == CC_FERR_CANCELED_BY_USER)
		{
			//stop importing the file if the user has cancelled the current process!
			break;
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

	m_ccRoot->unloadAll();

	redrawAll(false);
}

void MainWindow::doActionLoadFile()
{
	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::LoadFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
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
#ifdef Q_OS_WIN
//#ifdef QT_DEBUG
																, QFileDialog::DontUseNativeDialog
//#endif
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
				canExportClouds = (		filter->canSave(CC_TYPES::POINT_CLOUD, multiple, isExclusive)
									&&	(multiple || clouds.getChildrenNumber() == 1) );
				atLeastOneExclusive |= isExclusive;
			}

			//does this filter can export one or several meshes?
			bool canExportMeshes = true;
			if (hasMesh)
			{
				bool isExclusive = true;
				bool multiple = false;
				canExportMeshes = (		filter->canSave(CC_TYPES::MESH, multiple, isExclusive)
									&&	(multiple || meshes.getChildrenNumber() == 1) );
				atLeastOneExclusive |= isExclusive;
			}

			//does this filter can export one or several polylines?
			bool canExportPolylines = true;
			if (hasPolylines)
			{
				bool isExclusive = true;
				bool multiple = false;
				canExportPolylines = (	filter->canSave(CC_TYPES::POLY_LINE, multiple, isExclusive)
									&&	(multiple || polylines.getChildrenNumber() == 1) );
				atLeastOneExclusive |= isExclusive;
			}

			//does this filter can export one or several images?
			bool canExportImages = true;
			if (hasImages)
			{
				bool isExclusive = true;
				bool multiple = false;
				canExportImages = (		filter->canSave(CC_TYPES::IMAGE, multiple, isExclusive)
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
					canExportSerializables &= (		filter->canSave(child->getUniqueID(), multiple, isExclusive)
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
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
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
		for (unsigned i = 0; i < other.getChildrenNumber(); ++i)
		{
			ccConsole::Warning(QString("\t- %1s").arg(other.getChild(i)->getName()));
		}
	}

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	FileIOFilter::SaveParameters parameters;
	{
		parameters.alwaysDisplaySaveDialog = true;
		parameters.parentWidget = this;
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
			ConvertToGroup(m_selectedEntities, tempContainer, ccHObject::DP_NONE);
			if (tempContainer.getChildrenNumber())
			{
				result = FileIOFilter::SaveToFile(&tempContainer, selectedFilename, parameters, selectedFilter);
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
		//if (hasSerializable)
		//{
		//	if (!hasOther)
		//		ccConsole::Warning("[I/O] The following selected entites won't be saved:"); //display this warning only if not already done
		//	for (unsigned i = 0; i < otherSerializable.getChildrenNumber(); ++i)
		//		ccConsole::Warning(QString("\t- %1").arg(otherSerializable.getChild(i)->getName()));
		//}

		result = FileIOFilter::SaveToFile(	entitiesToSave.getChildrenNumber() > 1 ? &entitiesToSave : entitiesToSave.getChild(0),
											selectedFilename,
											parameters,
											selectedFilter);

		if (result == CC_FERR_NO_ERROR && m_ccRoot)
		{
			m_ccRoot->unselectAllEntities();
		}
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
	ccGLWindow* win = mdiWin ? GLWindowFromWidget(mdiWin->widget()) : 0;
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
	int mdiChildCount = getGLWindowCount();
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
			ccGLWindow *child = GLWindowFromWidget(windows.at(i)->widget());

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
		GLWindowFromWidget(windows.at(i)->widget())->redraw(only2D);
}

void MainWindow::refreshAll(bool only2D/*=false*/)
{
	QList<QMdiSubWindow*> windows = m_mdiArea->subWindowList();
	for (int i = 0; i < windows.size(); ++i)
		GLWindowFromWidget(windows.at(i)->widget())->refresh(only2D);
}

void MainWindow::updateUI()
{
	updateUIWithSelection();
	updateMenus();
	updatePropertiesView();
}

void MainWindow::updatePropertiesView()
{
	if (m_ccRoot)
	{
		m_ccRoot->updatePropertiesView();
	}
}
void MainWindow::updateUIWithSelection()
{
	dbTreeSelectionInfo selInfo;

	m_selectedEntities.clear();

	if (m_ccRoot)
		m_ccRoot->getSelectedEntities(m_selectedEntities, CC_TYPES::OBJECT, &selInfo);
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
		if (GLWindowFromWidget(windows.at(i)->widget()) != win)
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
	actionMeshScanGrids->setEnabled(atLeastOneCloud);
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
	actionFlagMeshVertices->setEnabled(atLeastOneMesh);
	actionSmoothMeshLaplacian->setEnabled(atLeastOneMesh);
	actionConvertTextureToColor->setEnabled(atLeastOneMesh);
	actionSubdivideMesh->setEnabled(atLeastOneMesh);
	actionDistanceToBestFitQuadric3D->setEnabled(atLeastOneCloud);
	actionDistanceMap->setEnabled(atLeastOneMesh || atLeastOneCloud);

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
	actionRGBToGreyScale->setEnabled(atLeastOneColor);
	actionEnhanceRGBWithIntensities->setEnabled(atLeastOneColor);

	// == 1
	bool exactlyOneEntity = (selInfo.selCount == 1);
	bool exactlyOneGroup = (selInfo.groupCount == 1);
	bool exactlyOneCloud = (selInfo.cloudCount == 1);
	bool exactlyOneMesh = (selInfo.meshCount == 1);
	bool exactlyOneSF = (selInfo.sfCount == 1);
	bool exactlyOneSensor = (selInfo.sensorCount == 1);
	bool exactlyOneCameraSensor = (selInfo.cameraSensorCount == 1);

	actionConvertPolylinesToMesh->setEnabled(atLeastOnePolyline || exactlyOneGroup);
	actionMeshTwoPolylines->setEnabled(selInfo.selCount == 2 && selInfo.polylineCount == 2);
	actionModifySensor->setEnabled(exactlyOneSensor);
	actionComputeDistancesFromSensor->setEnabled(atLeastOneCameraSensor || atLeastOneGBLSensor);
	actionComputeScatteringAngles->setEnabled(exactlyOneSensor);
	actionViewFromSensor->setEnabled(exactlyOneSensor);
	actionCreateGBLSensor->setEnabled(atLeastOneCloud);
	actionCreateCameraSensor->setEnabled(selInfo.selCount <= 1); //free now
	actionProjectUncertainty->setEnabled(exactlyOneCameraSensor);
	actionCheckPointsInsideFrustum->setEnabled(exactlyOneCameraSensor);
	actionLabelConnectedComponents->setEnabled(atLeastOneCloud);
	actionSORFilter->setEnabled(atLeastOneCloud);
	actionNoiseFilter->setEnabled(atLeastOneCloud);
	actionUnroll->setEnabled(exactlyOneEntity);
	actionStatisticalTest->setEnabled(exactlyOneEntity && exactlyOneSF);
	actionAddConstantSF->setEnabled(exactlyOneCloud || exactlyOneMesh);
	actionEditGlobalScale->setEnabled(exactlyOneCloud || exactlyOneMesh);
	actionComputeKdTree->setEnabled(exactlyOneCloud || exactlyOneMesh);
	actionShowWaveDialog->setEnabled(exactlyOneCloud);

	actionKMeans->setEnabled(/*TODO: exactlyOneEntity && exactlyOneSF*/false);
	actionFrontPropagation->setEnabled(/*TODO: exactlyOneEntity && exactlyOneSF*/false);

	//actionCreatePlane->setEnabled(true);
	actionEditPlane->setEnabled(selInfo.planeCount == 1);

	actionFindBiggestInnerRectangle->setEnabled(exactlyOneCloud);

	menuActiveScalarField->setEnabled((exactlyOneCloud || exactlyOneMesh) && selInfo.sfCount > 0);
	actionCrossSection->setEnabled(atLeastOneCloud || atLeastOneMesh);
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
	{
		plugin->onNewSelection(m_selectedEntities);
	}
}

void MainWindow::echoMouseWheelRotate(float wheelDelta_deg)
{
	if (!actionEnableCameraLink->isChecked())
		return;

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

	QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
	for (int i=0; i<windows.size(); ++i)
	{
		ccGLWindow *child = GLWindowFromWidget(windows.at(i)->widget());
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
	if (!actionEnableCameraLink->isChecked())
		return;

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

	QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
	for (int i=0; i<windows.size(); ++i)
	{
		ccGLWindow *child = GLWindowFromWidget(windows.at(i)->widget());
		if (child != sendingWindow)
		{
			child->blockSignals(true);
			child->moveCamera(ddx, ddy, 0.0f);
			child->blockSignals(false);
			child->redraw();
		}
	}
}

void MainWindow::echoBaseViewMatRotation(const ccGLMatrixd& rotMat)
{
	if (!actionEnableCameraLink->isChecked())
		return;

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

	QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
	for (int i=0; i<windows.size(); ++i)
	{
		ccGLWindow *child = GLWindowFromWidget(windows.at(i)->widget());
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
	 if (!actionEnableCameraLink->isChecked())
		 return;

	 ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	 if (!sendingWindow)
		 return;

	 QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
	 for (int i=0; i<windows.size(); ++i)
	 {
		 ccGLWindow *child = GLWindowFromWidget(windows.at(i)->widget());
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
	 if (!actionEnableCameraLink->isChecked())
		 return;

	 ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	 if (!sendingWindow)
		 return;

	 QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
	 for (int i=0; i<windows.size(); ++i)
	 {
		 ccGLWindow *child = GLWindowFromWidget(windows.at(i)->widget());
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
	 if (!actionEnableCameraLink->isChecked())
		 return;

	 ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	 if (!sendingWindow)
		 return;

	 QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
	 for (int i=0; i<windows.size(); ++i)
	 {
		 ccGLWindow *child = GLWindowFromWidget(windows.at(i)->widget());
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

	for (int i = 0; i < winNum; ++i)
	{
		glWindows.push_back(GLWindowFromWidget(windows.at(i)->widget()));
	}
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
		ccGLWindow* win = GLWindowFromWidget(windows.at(i)->widget());
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

void MainWindow::createGLWindow(ccGLWindow*& window, QWidget*& widget) const
{
	bool stereoMode = QSurfaceFormat::defaultFormat().stereo();

	CreateGLWindow(window, widget, stereoMode);
	assert(window && widget);
}

void MainWindow::destroyGLWindow(ccGLWindow* view3D) const
{
	if (view3D)
	{
		view3D->setParent(0);
		delete view3D;
	}
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

void MainWindow::doActionGlobalShiftSeetings()
{
	QDialog dialog(this);
	Ui_GlobalShiftSettingsDialog ui;
	ui.setupUi(&dialog);

	ui.maxAbsCoordSpinBox->setValue(static_cast<int>(log10(ccGlobalShiftManager::MaxCoordinateAbsValue())));
	ui.maxAbsDiagSpinBox->setValue(static_cast<int>(log10(ccGlobalShiftManager::MaxBoundgBoxDiagonal())));

	if (!dialog.exec())
	{
		return;
	}

	double maxAbsCoord = pow(10.0, static_cast<double>(ui.maxAbsCoordSpinBox->value()));
	double maxAbsDiag = pow(10.0, static_cast<double>(ui.maxAbsDiagSpinBox->value()));

	ccGlobalShiftManager::SetMaxCoordinateAbsValue(maxAbsCoord);
	ccGlobalShiftManager::SetMaxBoundgBoxDiagonal(maxAbsDiag);

	ccLog::Print(QString("[Global Shift] Max abs. coord = %1 / max abs. diag = %2")
		.arg(ccGlobalShiftManager::MaxCoordinateAbsValue(), 0, 'e', 0)
		.arg(ccGlobalShiftManager::MaxBoundgBoxDiagonal(), 0, 'e', 0));

	//save to persistent settings
	{
		QSettings settings;
		settings.beginGroup(ccPS::GlobalShift());
		settings.setValue(ccPS::MaxAbsCoord(), maxAbsCoord);
		settings.setValue(ccPS::MaxAbsDiag(), maxAbsDiag);
		settings.endGroup();
	}
}

void MainWindow::doActionShowWaveDialog()
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

	ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);
	if (!cloud->hasFWF())
	{
		ccConsole::Error("Cloud has no associated waveform information");
		return;
	}

	ccWaveDialog* wDlg = new ccWaveDialog(cloud, m_pickingHub, this);
	wDlg->setAttribute(Qt::WA_DeleteOnClose);
	wDlg->setModal(false);
	wDlg->show();
}

void MainWindow::doActionCreatePlane()
{
	ccPlaneEditDlg* peDlg = new ccPlaneEditDlg(m_pickingHub, this);
	peDlg->show();
}

void MainWindow::doActionEditPlane()
{
	if (m_selectedEntities.empty())
	{
		assert(false);
		return;
	}

	ccPlane* plane = ccHObjectCaster::ToPlane(m_selectedEntities.front());
	if (!plane)
	{
		assert(false);
		return;
	}

	ccPlaneEditDlg* peDlg = new ccPlaneEditDlg(m_pickingHub, this);
	peDlg->initWithPlane(plane);
	peDlg->show();
}
