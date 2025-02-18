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

//CCCoreLib Includes
#include <CloudSamplingTools.h>
#include <Delaunay2dMesh.h>
#include <Jacobi.h>
#include <MeshSamplingTools.h>
#include <NormalDistribution.h>
#include <ParallelSort.h>
#include <PointCloud.h>
#include <ScalarFieldTools.h>
#include <StatisticalTestingTools.h>
#include <WeibullDistribution.h>

//for tests
#include <ChamferDistanceTransform.h>
#include <SaitoSquaredDistanceTransform.h>

//qCC_db
#include <cc2DLabel.h>
#include <cc2DViewportObject.h>
#include <cc2DViewportLabel.h>
#include <ccCameraSensor.h>
#include <ccCircle.h>
#include <ccColorScalesManager.h>
#include <ccCylinder.h>
#include <ccFacet.h>
#include <ccFileUtils.h>
#include <ccGBLSensor.h>
#include <ccImage.h>
#include <ccKdTree.h>
#include <ccPlane.h>
#include <ccProgressDialog.h>
#include <ccQuadric.h>
#include <ccSphere.h>
#include <ccSubMesh.h>

//qCC_io
#include <ccShiftAndScaleCloudDlg.h>
#include <BinFilter.h>
#include <AsciiFilter.h>
#include <DepthMapFileFilter.h>

//QCC_glWindow
#include <ccGLWindowInterface.h>
#include <ccRenderingTools.h>

//local includes
#include "ccConsole.h"
#include "ccEntityAction.h"
#include "ccHistogramWindow.h"
#include "ccInnerRect2DFinder.h"

//common
#include <ccPickingHub.h>
//common dialogs
#include <ccCameraParamEditDlg.h>
#include <ccDisplaySettingsDlg.h>
#include <ccPickOneElementDlg.h>
#include <ccStereoModeDlg.h>

//dialogs
#include "ccAboutDialog.h"
#include "ccAdjustZoomDlg.h"
#include "ccAlignDlg.h"
#include "ccApplication.h"
#include "ccApplyTransformationDlg.h"
#include "ccAskThreeDoubleValuesDlg.h"
#include "ccBoundingBoxEditorDlg.h"
#include "ccCamSensorProjectionDlg.h"
#include "ccClippingBoxTool.h"
#include "ccColorFromScalarDlg.h"
#include "ccColorScaleEditorDlg.h"
#include "ccComparisonDlg.h"
#include "ccPrimitiveDistanceDlg.h"
#include "ccFilterByValueDlg.h"
#include "ccGBLSensorProjectionDlg.h"
#include "ccGeomFeaturesDlg.h"
#include "ccGraphicalSegmentationTool.h"
#include "ccGraphicalTransformationTool.h"
#include "ccItemSelectionDlg.h"
#include "ccLabelingDlg.h"
#include "ccMatchScalesDlg.h"
#include "ccNoiseFilterDlg.h"
#include "ccOrderChoiceDlg.h"
#include "ccPlaneEditDlg.h"
#include "ccPointListPickingDlg.h"
#include "ccPointPairRegistrationDlg.h"
#include "ccPointPropertiesDlg.h"
#include "ccPrimitiveFactoryDlg.h"
#include "ccPtsSamplingDlg.h"
#include "ccRasterizeTool.h"
#include "ccRegistrationDlg.h"
#include "ccRenderToFileDlg.h"
#include "ccScaleDlg.h"
#include "ccSectionExtractionTool.h"
#include "ccSensorComputeDistancesDlg.h"
#include "ccSensorComputeScatteringAnglesDlg.h"
#include "ccSORFilterDlg.h"
#include "ccSubsamplingDlg.h"
#include "ccTracePolylineTool.h"
#include "ccTranslationManager.h"
#include "ccUnrollDlg.h"
#include "ccVolumeCalcTool.h"
#include "ccWaveformDialog.h"
#include "ccEntitySelectionDlg.h"
#include "ccSmoothPolylineDlg.h"

//other
#include "ccCropTool.h"
#include "ccPersistentSettings.h"
#include "ccRecentFiles.h"
#include "ccRegistrationTools.h"
#include "ccUtils.h"
#include "db_tree/ccDBRoot.h"
#include "pluginManager/ccPluginUIManager.h"

#include "ccGlFilter.h"

//3D mouse handler
#ifdef CC_3DXWARE_SUPPORT
#include "cc3DMouseManager.h"
#endif

//Gamepads
#ifdef CC_GAMEPAD_SUPPORT
#include "ccGamepadManager.h"
#endif

//Qt
#include <QClipboard>
#include <QGLShader>

//Qt UI files
#include <ui_distanceMapDlg.h>
#include <ui_globalShiftSettingsDlg.h>
#include <ui_mainWindow.h>

//System
#include <iostream>
#include <random>

//global static pointer (as there should only be one instance of MainWindow!)
static MainWindow* s_instance  = nullptr;

//default file filter separator
static const QString s_fileFilterSeparator(";;");

enum PickingOperation {	NO_PICKING_OPERATION,
						PICKING_ROTATION_CENTER,
						PICKING_LEVEL_POINTS,
					  };
static ccGLWindowInterface* s_pickingWindow = nullptr;
static PickingOperation s_currentPickingOperation = NO_PICKING_OPERATION;
static std::vector<cc2DLabel*> s_levelLabels;
static ccPointCloud* s_levelMarkersCloud = nullptr;
static ccHObject* s_levelEntity = nullptr;

static QFileDialog::Options CCFileDialogOptions()
{
	//dialog options
	QFileDialog::Options dialogOptions = QFileDialog::Options();
	dialogOptions |= QFileDialog::DontResolveSymlinks;
	if (!ccOptions::Instance().useNativeDialogs)
	{
		dialogOptions |= QFileDialog::DontUseNativeDialog;
	}
	return dialogOptions;
}

MainWindow::MainWindow()
	: m_UI( new Ui::MainWindow )
	, m_ccRoot(nullptr)
	, m_uiFrozen(false)
	, m_recentFiles(new ccRecentFiles(this))
	, m_3DMouseManager(nullptr)
	, m_gamepadManager(nullptr)
	, m_viewModePopupButton(nullptr)
	, m_pivotVisibilityPopupButton(nullptr)
	, m_firstShow(true)
	, m_pickingHub(nullptr)
	, m_cpeDlg(nullptr)
	, m_gsTool(nullptr)
	, m_tplTool(nullptr)
	, m_seTool(nullptr)
	, m_transTool(nullptr)
	, m_clipTool(nullptr)
	, m_compDlg(nullptr)
	, m_ppDlg(nullptr)
	, m_plpDlg(nullptr)
	, m_pprDlg(nullptr)
	, m_pfDlg(nullptr)
{
	m_UI->setupUi( this );

	setWindowTitle(QStringLiteral("CloudCompare v") + ccApp->versionLongStr(false));
	
	m_pluginUIManager = new ccPluginUIManager( this, this );
	
	ccTranslationManager::Get().populateMenu( m_UI->menuLanguage, ccApp->translationPath() );
	
#ifdef Q_OS_MAC
	m_UI->actionAbout->setMenuRole( QAction::AboutRole );
	m_UI->actionAboutPlugins->setMenuRole( QAction::ApplicationSpecificRole );

	m_UI->actionFullScreen->setText( tr( "Enter Full Screen" ) );
	m_UI->actionFullScreen->setShortcut( QKeySequence( Qt::CTRL + Qt::META + Qt::Key_F ) );
#endif

	// Set up dynamic menus
	m_UI->menuFile->insertMenu(m_UI->actionSave, m_recentFiles->menu());
	
	//Console
	ccConsole::Init(m_UI->consoleWidget, this, this);
	m_UI->actionEnableQtWarnings->setChecked(ccConsole::QtMessagesEnabled());

	//advanced widgets not handled by QDesigner
	{
		//view mode pop-up menu
		{
			m_viewModePopupButton = new QToolButton();
			QMenu* menu = new QMenu(m_viewModePopupButton);
			menu->addAction(m_UI->actionSetOrthoView);
			menu->addAction(m_UI->actionSetCenteredPerspectiveView);
			menu->addAction(m_UI->actionSetViewerPerspectiveView);

			m_viewModePopupButton->setMenu(menu);
			m_viewModePopupButton->setPopupMode(QToolButton::InstantPopup);
			m_viewModePopupButton->setToolTip(tr("Set current view mode"));
			m_viewModePopupButton->setStatusTip(m_viewModePopupButton->toolTip());
			m_UI->toolBarView->insertWidget(m_UI->actionZoomAndCenter, m_viewModePopupButton);
			m_viewModePopupButton->setEnabled(false);
		}

		//pivot center pop-up menu
		{
			m_pivotVisibilityPopupButton = new QToolButton();
			QMenu* menu = new QMenu(m_pivotVisibilityPopupButton);
			menu->addAction(m_UI->actionSetPivotAlwaysOn);
			menu->addAction(m_UI->actionSetPivotRotationOnly);
			menu->addAction(m_UI->actionSetPivotOff);

			m_pivotVisibilityPopupButton->setMenu(menu);
			m_pivotVisibilityPopupButton->setPopupMode(QToolButton::InstantPopup);
			m_pivotVisibilityPopupButton->setToolTip(tr("Set pivot visibility"));
			m_pivotVisibilityPopupButton->setStatusTip(m_pivotVisibilityPopupButton->toolTip());
			m_UI->toolBarView->insertWidget(m_UI->actionZoomAndCenter,m_pivotVisibilityPopupButton);
			m_pivotVisibilityPopupButton->setEnabled(false);
		}
	}

	//tabifyDockWidget(DockableDBTree,DockableProperties);

	//db-tree
	{
		m_ccRoot = new ccDBRoot(m_UI->dbTreeView, m_UI->propertiesTreeView, this);
		connect(m_ccRoot, &ccDBRoot::selectionChanged,    this, &MainWindow::updateUIWithSelection, Qt::QueuedConnection);
		connect(m_ccRoot, &ccDBRoot::dbIsEmpty,           this, [=]() { updateUIWithSelection(); updateMenus(); }, Qt::QueuedConnection); //we don't call updateUI because there's no need to update the properties dialog
		connect(m_ccRoot, &ccDBRoot::dbIsNotEmptyAnymore, this, [=]() { updateUIWithSelection(); updateMenus(); }, Qt::QueuedConnection); //we don't call updateUI because there's no need to update the properties dialog
	}

	//MDI Area
	{
		m_mdiArea = new QMdiArea(this);
		setCentralWidget(m_mdiArea);
		connect(m_mdiArea, &QMdiArea::subWindowActivated, this, &MainWindow::updateMenus);
		connect(m_mdiArea, &QMdiArea::subWindowActivated, this, &MainWindow::on3DViewActivated);
		m_mdiArea->installEventFilter(this);
	}

	//picking hub
	{
		m_pickingHub = new ccPickingHub(this, this);
		connect(m_mdiArea, &QMdiArea::subWindowActivated, m_pickingHub, &ccPickingHub::onActiveWindowChanged);
	}

	// restore the state of the 'auto-restore' menu entry
	// (do that before connecting the actions)
	{
		QSettings settings;
		bool doNotAutoRestoreGeometry = settings.value(ccPS::DoNotRestoreWindowGeometry(), !m_UI->actionRestoreWindowOnStartup->isChecked()).toBool();
		m_UI->actionRestoreWindowOnStartup->setChecked(!doNotAutoRestoreGeometry);
	}

	connectActions();

	new3DView();

	setupInputDevices();

	freezeUI(false);

	updateUI();

	QMainWindow::statusBar()->showMessage(tr("Ready"));
	
#ifdef CC_CORE_LIB_USES_TBB
	ccConsole::Print( QStringLiteral( "[TBB] Using Intel's Threading Building Blocks %1" )
					  .arg( QString( TBB_VERSION ) ) );
#endif
	
	ccConsole::Print(tr("CloudCompare started!"));
}

MainWindow::~MainWindow()
{
	destroyInputDevices();

	cancelPreviousPickingOperation(false); //just in case

	assert(m_ccRoot && m_mdiArea);
	m_ccRoot->disconnect();
	m_mdiArea->disconnect();

	//we don't want any other dialog/function to use the following structures
	ccDBRoot* ccRoot = m_ccRoot;
	m_ccRoot = nullptr;

	//remove all entities from 3D views before quitting to avoid any side-effect
	//(this won't be done automatically since we've just reset m_ccRoot)
	ccRoot->getRootEntity()->setDisplay_recursive(nullptr);
	for (int i = 0; i < getGLWindowCount(); ++i)
	{
		getGLWindow(i)->setSceneDB(nullptr);
	}
	m_cpeDlg = nullptr;
	m_gsTool = nullptr;
	m_seTool = nullptr;
	m_transTool = nullptr;
	m_clipTool = nullptr;
	m_compDlg = nullptr;
	m_ppDlg = nullptr;
	m_plpDlg = nullptr;
	m_pprDlg = nullptr;
	m_pfDlg = nullptr;

	//release all 'overlay' dialogs
	while (!m_mdiDialogs.empty())
	{
		ccMDIDialogs mdiDialog = m_mdiDialogs.back();
		m_mdiDialogs.pop_back();

		mdiDialog.dialog->disconnect();
		mdiDialog.dialog->stop(false);
		mdiDialog.dialog->setParent(nullptr);
		delete mdiDialog.dialog;
	}
	//m_mdiDialogs.clear();
	m_mdiArea->closeAllSubWindows();

	if (ccRoot)
	{
		delete ccRoot;
		ccRoot = nullptr;
	}

	delete m_UI;
	m_UI = nullptr;
	
	ccConsole::ReleaseInstance(false); //if we flush the console, it will try to display the console window while we are destroying everything!
}

void MainWindow::initPlugins( )
{
	m_pluginUIManager->init();
	
	// Set up dynamic tool bars
	addToolBar( Qt::RightToolBarArea, m_pluginUIManager->glFiltersToolbar() );
	addToolBar( Qt::RightToolBarArea, m_pluginUIManager->mainPluginToolbar() );
	
	for ( QToolBar *toolbar : m_pluginUIManager->additionalPluginToolbars() )
	{
		addToolBar( Qt::TopToolBarArea, toolbar );
	}
	
	// Set up dynamic menus
	m_UI->menubar->insertMenu( m_UI->menu3DViews->menuAction(), m_pluginUIManager->pluginMenu() );
	m_UI->menuDisplay->insertMenu( m_UI->menuActiveScalarField->menuAction(), m_pluginUIManager->shaderAndFilterMenu() );

	m_UI->menuToolbars->addAction( m_pluginUIManager->actionShowMainPluginToolbar() );
	m_UI->menuToolbars->addAction( m_pluginUIManager->actionShowGLFilterToolbar() );
}

void MainWindow::doEnableQtWarnings(bool state)
{
	ccConsole::EnableQtMessages(state);
}

void MainWindow::increasePointSize()
{
	//active window?
	ccGLWindowInterface* win = getActiveGLWindow();
	if (win)
	{
		win->setPointSize(win->getViewportParameters().defaultPointSize + 1);
		win->redraw();
	}
}

void MainWindow::decreasePointSize()
{
	//active window?
	ccGLWindowInterface* win = getActiveGLWindow();
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
	m_UI->menuFile->insertMenu(m_UI->actionCloseAll, m_3DMouseManager->menu());
#endif

#ifdef CC_GAMEPAD_SUPPORT
	m_gamepadManager = new ccGamepadManager( this, this );
	m_UI->menuFile->insertMenu(m_UI->actionCloseAll, m_gamepadManager->menu());
#endif

#if defined(CC_3DXWARE_SUPPORT) || defined(CC_GAMEPAD_SUPPORT)
	m_UI->menuFile->insertSeparator(m_UI->actionCloseAll);
#endif
}

void MainWindow::destroyInputDevices()
{
#ifdef CC_GAMEPAD_SUPPORT
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
	
	//Keyboard shortcuts
	
	//'A': toggles selected items activation
	connect(m_UI->actionToggleActivation, &QAction::triggered, this, [=]() {
		toggleSelectedEntitiesProperty( ccEntityAction::TOGGLE_PROPERTY::ACTIVE );
	});

	//'V': toggles selected items visibility
	connect(m_UI->actionToggleVisibility, &QAction::triggered, this, [=]() {
		toggleSelectedEntitiesProperty( ccEntityAction::TOGGLE_PROPERTY::VISIBLE );
	});

	//'N': toggles selected items normals visibility
	connect(m_UI->actionToggleNormals, &QAction::triggered, this, [=]() {
		toggleSelectedEntitiesProperty( ccEntityAction::TOGGLE_PROPERTY::NORMALS );
	});

	//'C': toggles selected items colors visibility
	connect(m_UI->actionToggleColors,	&QAction::triggered, this, [=]() {
		toggleSelectedEntitiesProperty( ccEntityAction::TOGGLE_PROPERTY::COLOR );
	});

	//'S': toggles selected items SF visibility
	connect(m_UI->actionToggleSF, &QAction::triggered, this, [=]() {
		toggleSelectedEntitiesProperty( ccEntityAction::TOGGLE_PROPERTY::SCALAR_FIELD );
	});

	//'D': toggles selected items '3D name' visibility
	connect(m_UI->actionToggleShowName, &QAction::triggered, this, [=]() {
		toggleSelectedEntitiesProperty( ccEntityAction::TOGGLE_PROPERTY::NAME );
	});

	//'M': toggles selected items materials/textures visibility
	connect(m_UI->actionToggleMaterials, &QAction::triggered, this, [=]() {
		toggleSelectedEntitiesProperty( ccEntityAction::TOGGLE_PROPERTY::MATERIAL );
	});

	//TODO... but not ready yet ;)
	m_UI->actionLoadShader->setVisible(false);
	m_UI->actionDeleteShader->setVisible(false);
	m_UI->actionKMeans->setVisible(false);
	m_UI->actionFrontPropagation->setVisible(false);

	/*** MAIN MENU ***/

	//"File" menu
	connect(m_UI->actionOpen,						&QAction::triggered, this, &MainWindow::doActionLoadFile);
	connect(m_UI->actionSave,						&QAction::triggered, this, &MainWindow::doActionSaveFile);
	connect(m_UI->actionSaveProject,				&QAction::triggered, this, &MainWindow::doActionSaveProject);
	connect(m_UI->actionGlobalShiftSettings,		&QAction::triggered, this, &MainWindow::doActionGlobalShiftSeetings);
	connect(m_UI->actionPrimitiveFactory,			&QAction::triggered, this, &MainWindow::doShowPrimitiveFactory);
	connect(m_UI->actionCloseAll,					&QAction::triggered, this, &MainWindow::closeAll);
	connect(m_UI->actionQuit,						&QAction::triggered, this, &QWidget::close);

	//"Edit > Colors" menu
	connect(m_UI->actionSetUniqueColor,				&QAction::triggered, this, &MainWindow::doActionSetUniqueColor);
	connect(m_UI->actionSetColorGradient,			&QAction::triggered, this, &MainWindow::doActionSetColorGradient);
	connect(m_UI->actionChangeColorLevels,			&QAction::triggered, this, &MainWindow::doActionChangeColorLevels);
	connect(m_UI->actionColorize,					&QAction::triggered, this, &MainWindow::doActionColorize);
	connect(m_UI->actionRGBToGreyScale,				&QAction::triggered, this, &MainWindow::doActionRGBToGreyScale);
	connect(m_UI->actionInterpolateColors,			&QAction::triggered, this, &MainWindow::doActionInterpolateColors);
	connect(m_UI->actionEnhanceRGBWithIntensities,	&QAction::triggered, this, &MainWindow::doActionEnhanceRGBWithIntensities);
	connect(m_UI->actionColorFromScalarField,       &QAction::triggered, this, &MainWindow::doActionColorFromScalars);
	connect(m_UI->actionClearColor,					&QAction::triggered, this, [=]() {
		clearSelectedEntitiesProperty( ccEntityAction::CLEAR_PROPERTY::COLORS );
	});
	connect(m_UI->actionRGBGaussianFilter,			&QAction::triggered, this, &MainWindow::doActionRGBGaussianFilter);
	connect(m_UI->actionRGBBilateralFilter,			&QAction::triggered, this, &MainWindow::doActionRGBBilateralFilter);
	connect(m_UI->actionRGBMeanFilter,				&QAction::triggered, this, &MainWindow::doActionRGBMeanFilter);
	connect(m_UI->actionRGBMedianFilter,			&QAction::triggered, this, &MainWindow::doActionRGBMedianFilter);

	//"Edit > Normals" menu
	connect(m_UI->actionComputeNormals,				&QAction::triggered, this, &MainWindow::doActionComputeNormals);
	connect(m_UI->actionInvertNormals,				&QAction::triggered, this, &MainWindow::doActionInvertNormals);
	connect(m_UI->actionConvertNormalToHSV,			&QAction::triggered, this, &MainWindow::doActionConvertNormalsToHSV);
	connect(m_UI->actionConvertNormalToDipDir,		&QAction::triggered, this, &MainWindow::doActionConvertNormalsToDipDir);
	connect(m_UI->actionExportNormalToSF,			&QAction::triggered, this, &MainWindow::doActionExportNormalToSF);
	connect(m_UI->actionSetSFsAsNormal,				&QAction::triggered, this, &MainWindow::doActionSetSFsAsNormal);
	connect(m_UI->actionOrientNormalsMST,			&QAction::triggered, this, &MainWindow::doActionOrientNormalsMST);
	connect(m_UI->actionOrientNormalsFM,			&QAction::triggered, this, &MainWindow::doActionOrientNormalsFM);
	connect(m_UI->actionShiftPointsAlongNormals,	&QAction::triggered, this, &MainWindow::doActionShiftPointsAlongNormals);
	connect(m_UI->actionClearNormals,				&QAction::triggered, this, [=]() {
		clearSelectedEntitiesProperty( ccEntityAction::CLEAR_PROPERTY::NORMALS );
	});

	//"Edit > Octree" menu
	connect(m_UI->actionComputeOctree,				&QAction::triggered, this, &MainWindow::doActionComputeOctree);
	connect(m_UI->actionResampleWithOctree,			&QAction::triggered, this, &MainWindow::doActionResampleWithOctree);

	//"Edit > Grid" menu
	connect(m_UI->actionDeleteScanGrid,				&QAction::triggered, this, &MainWindow::doActionDeleteScanGrids);

	//"Edit > Cloud" menu
	connect(m_UI->actionCreateSinglePointCloud,		&QAction::triggered, this, &MainWindow::createSinglePointCloud);
	connect(m_UI->actionPasteCloudFromClipboard,	&QAction::triggered, this, &MainWindow::createPointCloudFromClipboard);
	//the 'Paste from clipboard' tool depends on the clipboard state
	{
		const QClipboard* clipboard = QApplication::clipboard();
		assert(clipboard);
		m_UI->actionPasteCloudFromClipboard->setEnabled(clipboard->mimeData()->hasText());
		connect(clipboard, &QClipboard::dataChanged, [&]() { m_UI->actionPasteCloudFromClipboard->setEnabled(clipboard->mimeData()->hasText()); });
	}


	//"Edit > Mesh" menu
	connect(m_UI->actionComputeMeshAA,				&QAction::triggered, this, &MainWindow::doActionComputeMeshAA);
	connect(m_UI->actionComputeMeshLS,				&QAction::triggered, this, &MainWindow::doActionComputeMeshLS);
	connect(m_UI->actionMeshTwoPolylines,			&QAction::triggered, this, &MainWindow::doMeshTwoPolylines);
	connect(m_UI->actionMeshScanGrids,				&QAction::triggered, this, &MainWindow::doActionMeshScanGrids);
	connect(m_UI->actionConvertTextureToColor,		&QAction::triggered, this, &MainWindow::doActionConvertTextureToColor);
	connect(m_UI->actionSamplePointsOnMesh,			&QAction::triggered, this, &MainWindow::doActionSamplePointsOnMesh);
	connect(m_UI->actionSmoothMeshLaplacian,		&QAction::triggered, this, &MainWindow::doActionSmoothMeshLaplacian);
	connect(m_UI->actionSubdivideMesh,				&QAction::triggered, this, &MainWindow::doActionSubdivideMesh);
	connect(m_UI->actionFlipMeshTriangles,			&QAction::triggered, this, &MainWindow::doActionFlipMeshTriangles);
	connect(m_UI->actionMeasureMeshSurface,			&QAction::triggered, this, &MainWindow::doActionMeasureMeshSurface);
	connect(m_UI->actionMeasureMeshVolume,			&QAction::triggered, this, &MainWindow::doActionMeasureMeshVolume);
	connect(m_UI->actionFlagMeshVertices,			&QAction::triggered, this, &MainWindow::doActionFlagMeshVertices);
	//"Edit > Mesh > Scalar Field" menu
	connect(m_UI->actionSmoothMeshSF,				&QAction::triggered, this, &MainWindow::doActionSmoothMeshSF);
	connect(m_UI->actionEnhanceMeshSF,				&QAction::triggered, this, &MainWindow::doActionEnhanceMeshSF);
	//"Edit > Polyline" menu
	connect(m_UI->actionSamplePointsOnPolyline,		&QAction::triggered, this, &MainWindow::doActionSamplePointsOnPolyline);
	connect(m_UI->actionSmoothPolyline,				&QAction::triggered, this, &MainWindow::doActionSmoohPolyline);
	
	//"Edit > Plane" menu
	connect(m_UI->actionCreatePlane,				&QAction::triggered, this, &MainWindow::doActionCreatePlane);
	connect(m_UI->actionEditPlane,					&QAction::triggered, this, &MainWindow::doActionEditPlane);
	connect(m_UI->actionFlipPlane,					&QAction::triggered, this, &MainWindow::doActionFlipPlane);
	connect(m_UI->actionComparePlanes,				&QAction::triggered, this, &MainWindow::doActionComparePlanes);

	//"Edit > Circle" menu
	connect(m_UI->actionPromoteCircleToCylinder,	&QAction::triggered, this, &MainWindow::doActionPromoteCircleToCylinder);

	//"Edit > Sensor > Ground-Based lidar" menu
	connect(m_UI->actionShowDepthBuffer,			&QAction::triggered, this, &MainWindow::doActionShowDepthBuffer);
	connect(m_UI->actionExportDepthBuffer,			&QAction::triggered, this, &MainWindow::doActionExportDepthBuffer);
	connect(m_UI->actionComputePointsVisibility,	&QAction::triggered, this, &MainWindow::doActionComputePointsVisibility);
	//"Edit > Sensor" menu
	connect(m_UI->actionCreateGBLSensor,			&QAction::triggered, this, &MainWindow::doActionCreateGBLSensor);
	connect(m_UI->actionCreateCameraSensor,			&QAction::triggered, this, &MainWindow::doActionCreateCameraSensor);
	connect(m_UI->actionModifySensor,				&QAction::triggered, this, &MainWindow::doActionModifySensor);
	connect(m_UI->actionProjectUncertainty,			&QAction::triggered, this, &MainWindow::doActionProjectUncertainty);
	connect(m_UI->actionCheckPointsInsideFrustum,	&QAction::triggered, this, &MainWindow::doActionCheckPointsInsideFrustum);
	connect(m_UI->actionComputeDistancesFromSensor,	&QAction::triggered, this, &MainWindow::doActionComputeDistancesFromSensor);
	connect(m_UI->actionComputeScatteringAngles,	&QAction::triggered, this, &MainWindow::doActionComputeScatteringAngles);
	connect(m_UI->actionViewFromSensor,				&QAction::triggered, this, &MainWindow::doActionSetViewFromSensor);
	//"Edit > Scalar fields" menu
	connect(m_UI->actionShowHistogram,				&QAction::triggered, this, &MainWindow::showSelectedEntitiesHistogram);
	connect(m_UI->actionComputeStatParams,			&QAction::triggered, this, &MainWindow::doActionComputeStatParams);
	connect(m_UI->actionSFGradient,					&QAction::triggered, this, &MainWindow::doActionSFGradient);
	connect(m_UI->actionGaussianFilter,				&QAction::triggered, this, &MainWindow::doActionSFGaussianFilter);
	connect(m_UI->actionBilateralFilter,			&QAction::triggered, this, &MainWindow::doActionSFBilateralFilter);
	connect(m_UI->actionFilterByValue,				&QAction::triggered, this, &MainWindow::doActionFilterByValue);
	connect(m_UI->actionAddConstantSF,				&QAction::triggered, this, &MainWindow::doActionAddConstantSF);
    connect(m_UI->actionAddClassificationSF,        &QAction::triggered, this, &MainWindow::doActionAddClassificationSF);
	connect(m_UI->actionScalarFieldArithmetic,		&QAction::triggered, this, &MainWindow::doActionScalarFieldArithmetic);
	connect(m_UI->actionScalarFieldFromColor,		&QAction::triggered, this, &MainWindow::doActionScalarFieldFromColor);
	connect(m_UI->actionConvertToRGB,				&QAction::triggered, this, &MainWindow::doActionSFConvertToRGB);
	connect(m_UI->actionConvertToRandomRGB,			&QAction::triggered, this, &MainWindow::doActionSFConvertToRandomRGB);
	connect(m_UI->actionRenameSF,					&QAction::triggered, this, &MainWindow::doActionRenameSF);
	connect(m_UI->actionOpenColorScalesManager,		&QAction::triggered, this, &MainWindow::doActionOpenColorScalesManager);
	connect(m_UI->actionAddIdField,					&QAction::triggered, this, &MainWindow::doActionAddIdField);
    connect(m_UI->actionSplitCloudUsingSF,          &QAction::triggered, this, &MainWindow::doActionSplitCloudUsingSF);
	connect(m_UI->actionSetSFAsCoord,				&QAction::triggered, this, &MainWindow::doActionSetSFAsCoord);
	connect(m_UI->actionInterpolateSFs,				&QAction::triggered, this, &MainWindow::doActionInterpolateScalarFields);
	connect(m_UI->actionDeleteScalarField,			&QAction::triggered, this, [=]() {
		clearSelectedEntitiesProperty( ccEntityAction::CLEAR_PROPERTY::CURRENT_SCALAR_FIELD );
	});
	connect(m_UI->actionDeleteAllSF,				&QAction::triggered, this, [=]() {
		clearSelectedEntitiesProperty( ccEntityAction::CLEAR_PROPERTY::ALL_SCALAR_FIELDS );
	});
	
	//"Edit > Waveform" menu
	connect(m_UI->actionShowWaveDialog,				&QAction::triggered, this, &MainWindow::doActionShowWaveDialog);
	connect(m_UI->actionCompressFWFData,			&QAction::triggered, this, &MainWindow::doActionCompressFWFData);
	//"Edit" menu
	connect(m_UI->actionClone,						&QAction::triggered, this, &MainWindow::doActionClone);
	connect(m_UI->actionMerge,						&QAction::triggered, this, &MainWindow::doActionMerge);
	connect(m_UI->actionApplyTransformation,		&QAction::triggered, this, &MainWindow::doActionApplyTransformation);
	connect(m_UI->actionApplyScale,					&QAction::triggered, this, &MainWindow::doActionApplyScale);
	connect(m_UI->actionTranslateRotate,			&QAction::triggered, this, &MainWindow::activateTranslateRotateMode);
	connect(m_UI->actionSegment,					&QAction::triggered, this, &MainWindow::activateSegmentationMode);
    connect(m_UI->actionTracePolyline,				&QAction::triggered, this, &MainWindow::activateTracePolylineMode);

	connect(m_UI->actionCrop,						&QAction::triggered, this, &MainWindow::doActionCrop);
	connect(m_UI->actionEditGlobalShiftAndScale,	&QAction::triggered, this, &MainWindow::doActionEditGlobalShiftAndScale);
	connect(m_UI->actionSubsample,					&QAction::triggered, this, &MainWindow::doActionSubsample);
	connect(m_UI->actionDelete,						&QAction::triggered,	m_ccRoot,	&ccDBRoot::deleteSelectedEntities);

	//"Tools > Clean" menu
	connect(m_UI->actionSORFilter,					&QAction::triggered, this, &MainWindow::doActionSORFilter);
	connect(m_UI->actionNoiseFilter,				&QAction::triggered, this, &MainWindow::doActionFilterNoise);

	//"Tools > Projection" menu
	connect(m_UI->actionUnroll,						&QAction::triggered, this, &MainWindow::doActionUnroll);
	connect(m_UI->actionRasterize,					&QAction::triggered, this, &MainWindow::doActionRasterize);
	connect(m_UI->actionConvertPolylinesToMesh,		&QAction::triggered, this, &MainWindow::doConvertPolylinesToMesh);
	//connect(m_UI->actionCreateSurfaceBetweenTwoPolylines, &QAction::triggered, this, &MainWindow::doMeshTwoPolylines); //DGM: already connected to actionMeshTwoPolylines
	connect(m_UI->actionExportCoordToSF,			&QAction::triggered, this, &MainWindow::doActionExportCoordToSF);
	
	//"Tools > Registration" menu
	connect(m_UI->actionMatchBBCenters,				&QAction::triggered, this, &MainWindow::doActionMatchBBCenters);
	connect(m_UI->actionMatchScales,				&QAction::triggered, this, &MainWindow::doActionMatchScales);
	connect(m_UI->actionRegister,					&QAction::triggered, this, &MainWindow::doActionRegister);
	connect(m_UI->actionPointPairsAlign,			&QAction::triggered, this, &MainWindow::activateRegisterPointPairTool);
	connect(m_UI->actionBBCenterToOrigin,			&QAction::triggered, this, &MainWindow::doActionMoveBBCenterToOrigin);
	connect(m_UI->actionBBMinCornerToOrigin,		&QAction::triggered, this, &MainWindow::doActionMoveBBMinCornerToOrigin);
	connect(m_UI->actionBBMaxCornerToOrigin,		&QAction::triggered, this, &MainWindow::doActionMoveBBMaxCornerToOrigin);
	//"Tools > Distances" menu
	connect(m_UI->actionCloudCloudDist,				&QAction::triggered, this, &MainWindow::doActionCloudCloudDist);
	connect(m_UI->actionCloudMeshDist,				&QAction::triggered, this, &MainWindow::doActionCloudMeshDist);
	connect(m_UI->actionCloudPrimitiveDist,			&QAction::triggered, this, &MainWindow::doActionCloudPrimitiveDist);
	connect(m_UI->actionCPS,						&QAction::triggered, this, &MainWindow::doActionComputeCPS);
	//"Tools > Volume" menu
	connect(m_UI->actionCompute2HalfDimVolume,		&QAction::triggered, this, &MainWindow::doCompute2HalfDimVolume);
	//"Tools > Statistics" menu
	connect(m_UI->actionComputeStatParams2,			&QAction::triggered, this, &MainWindow::doActionComputeStatParams); //duplicated action --> we can't use the same otherwise we get an ugly console warning on Linux :(
	connect(m_UI->actionStatisticalTest,			&QAction::triggered, this, &MainWindow::doActionStatisticalTest);
	//"Tools > Segmentation" menu
	connect(m_UI->actionLabelConnectedComponents,	&QAction::triggered, this, &MainWindow::doActionLabelConnectedComponents);
	connect(m_UI->actionKMeans,						&QAction::triggered, this, &MainWindow::doActionKMeans);
	connect(m_UI->actionFrontPropagation,			&QAction::triggered, this, &MainWindow::doActionFrontPropagation);
	connect(m_UI->actionCrossSection,				&QAction::triggered, this, &MainWindow::activateClippingBoxMode);
	connect(m_UI->actionExtractSections,			&QAction::triggered, this, &MainWindow::activateSectionExtractionMode);
	//"Tools > Fit" menu
	connect(m_UI->actionFitPlane,					&QAction::triggered, this, &MainWindow::doActionFitPlane);
	connect(m_UI->actionFitSphere,					&QAction::triggered, this, &MainWindow::doActionFitSphere);
	connect(m_UI->actionFitCircle,					&QAction::triggered, this, &MainWindow::doActionFitCircle);
	connect(m_UI->actionFitFacet,					&QAction::triggered, this, &MainWindow::doActionFitFacet);
	connect(m_UI->actionFitQuadric,					&QAction::triggered, this, &MainWindow::doActionFitQuadric);
	//"Tools > Batch export" menu
	connect(m_UI->actionExportCloudInfo,			&QAction::triggered, this, &MainWindow::doActionExportCloudInfo);
	connect(m_UI->actionExportPlaneInfo,			&QAction::triggered, this, &MainWindow::doActionExportPlaneInfo);
	//"Tools > Other" menu
	connect(m_UI->actionComputeGeometricFeature,	&QAction::triggered, this, &MainWindow::doComputeGeometricFeature);
	connect(m_UI->actionRemoveDuplicatePoints,		&QAction::triggered, this, &MainWindow::doRemoveDuplicatePoints);
	//"Tools"
	connect(m_UI->actionLevel,						&QAction::triggered, this, &MainWindow::doLevel);
	connect(m_UI->actionPointListPicking,			&QAction::triggered, this, &MainWindow::activatePointListPickingMode);
	connect(m_UI->actionPointPicking,				&QAction::triggered, this, &MainWindow::activatePointPickingMode);

	//"Tools > Sand box (research)" menu
	connect(m_UI->actionComputeKdTree,				&QAction::triggered, this, &MainWindow::doActionComputeKdTree);
	connect(m_UI->actionDistanceMap,				&QAction::triggered, this, &MainWindow::doActionComputeDistanceMap);
	connect(m_UI->actionDistanceToBestFitQuadric3D,	&QAction::triggered, this, &MainWindow::doActionComputeDistToBestFitQuadric3D);
	connect(m_UI->actionComputeBestFitBB,			&QAction::triggered, this, &MainWindow::doComputeBestFitBB);
	connect(m_UI->actionAlign,						&QAction::triggered, this, &MainWindow::doAction4pcsRegister); //Aurelien BEY le 13/11/2008
	connect(m_UI->actionSNETest,					&QAction::triggered, this, &MainWindow::doSphericalNeighbourhoodExtractionTest);
	connect(m_UI->actionCNETest,					&QAction::triggered, this, &MainWindow::doCylindricalNeighbourhoodExtractionTest);
	connect(m_UI->actionFindBiggestInnerRectangle,	&QAction::triggered, this, &MainWindow::doActionFindBiggestInnerRectangle);
	connect(m_UI->actionCreateCloudFromEntCenters,	&QAction::triggered, this, &MainWindow::doActionCreateCloudFromEntCenters);
	connect(m_UI->actionComputeBestICPRmsMatrix,	&QAction::triggered, this, &MainWindow::doActionComputeBestICPRmsMatrix);

	//"Display" menu
	connect(m_UI->actionFullScreen,						&QAction::toggled, this, &MainWindow::toggleFullScreen);
	connect(m_UI->actionExclusiveFullScreen,			&QAction::toggled, this, &MainWindow::toggleExclusiveFullScreen);
	connect(m_UI->actionRefresh,						&QAction::triggered, this, &MainWindow::refreshAll);
	connect(m_UI->actionTestFrameRate,					&QAction::triggered, this, &MainWindow::testFrameRate);
	connect(m_UI->actionToggleCenteredPerspective,		&QAction::triggered, this, &MainWindow::toggleActiveWindowCenteredPerspective);
	connect(m_UI->actionToggleViewerBasedPerspective,	&QAction::triggered, this, &MainWindow::toggleActiveWindowViewerBasedPerspective);
	connect(m_UI->actionShowCursor3DCoordinates,		&QAction::toggled, this, &MainWindow::toggleActiveWindowShowCursorCoords);
	connect(m_UI->actionLockRotationAxis,				&QAction::triggered, this, &MainWindow::toggleLockRotationAxis);
	connect(m_UI->actionEnterBubbleViewMode,			&QAction::triggered, this, &MainWindow::doActionEnableBubbleViewMode);
	connect(m_UI->actionEditCamera,						&QAction::triggered, this, &MainWindow::doActionEditCamera);
	connect(m_UI->actionAdjustZoom,						&QAction::triggered, this, &MainWindow::doActionAdjustZoom);
	connect(m_UI->actionSaveViewportAsObject,			&QAction::triggered, this, &MainWindow::doActionSaveViewportAsCamera);

	//"Display > Lights & Materials" menu
	connect(m_UI->actionDisplaySettings,			&QAction::triggered, this, &MainWindow::showDisplaySettings);
	connect(m_UI->actionToggleSunLight,				&QAction::triggered, this, &MainWindow::toggleActiveWindowSunLight);
	connect(m_UI->actionToggleCustomLight,			&QAction::triggered, this, &MainWindow::toggleActiveWindowCustomLight);
	connect(m_UI->actionRenderToFile,				&QAction::triggered, this, &MainWindow::doActionRenderToFile);
	//"Display > Shaders & filters" menu
	connect(m_UI->actionLoadShader,					&QAction::triggered, this, &MainWindow::doActionLoadShader);
	connect(m_UI->actionDeleteShader,				&QAction::triggered, this, &MainWindow::doActionDeleteShader);

	//"Display > Active SF" menu
	connect(m_UI->actionToggleActiveSFColorScale,	&QAction::triggered, this, &MainWindow::doActionToggleActiveSFColorScale);
	connect(m_UI->actionShowActiveSFPrevious,		&QAction::triggered, this, &MainWindow::doActionShowActiveSFPrevious);
	connect(m_UI->actionShowActiveSFNext,			&QAction::triggered, this, &MainWindow::doActionShowActiveSFNext);

	//"Display" menu
	connect(m_UI->actionResetGUIElementsPos,		&QAction::triggered, this, &MainWindow::doActionResetGUIElementsPos);
	connect(m_UI->actionRestoreWindowOnStartup,		&QAction::toggled,   this, &MainWindow::doActionToggleRestoreWindowOnStartup);
	connect(m_UI->actionResetAllVBOs,				&QAction::triggered, this, &MainWindow::doActionResetAllVBOs);

	//"3D Views" menu
	connect(m_UI->menu3DViews,						&QMenu::aboutToShow, this, &MainWindow::update3DViewsMenu);
	connect(m_UI->actionNew3DView,					&QAction::triggered, this, &MainWindow::new3DView);
	connect(m_UI->actionZoomIn,						&QAction::triggered, this, &MainWindow::zoomIn);
	connect(m_UI->actionZoomOut,					&QAction::triggered, this, &MainWindow::zoomOut);
	connect(m_UI->actionClose3DView,				&QAction::triggered, m_mdiArea, &QMdiArea::closeActiveSubWindow);
	connect(m_UI->actionCloseAll3DViews,			&QAction::triggered, m_mdiArea, &QMdiArea::closeAllSubWindows);
	connect(m_UI->actionTile3DViews,				&QAction::triggered, m_mdiArea, &QMdiArea::tileSubWindows);
	connect(m_UI->actionCascade3DViews,				&QAction::triggered, m_mdiArea, &QMdiArea::cascadeSubWindows);
	connect(m_UI->actionNext3DView,					&QAction::triggered, m_mdiArea, &QMdiArea::activateNextSubWindow);
	connect(m_UI->actionPrevious3DView,				&QAction::triggered, m_mdiArea, &QMdiArea::activatePreviousSubWindow);

	//"About" menu entry
	connect(m_UI->actionHelp,						&QAction::triggered, this, &MainWindow::doActionShowHelpDialog);
	connect(m_UI->actionAboutPlugins,				&QAction::triggered, m_pluginUIManager, &ccPluginUIManager::showAboutDialog);
	connect(m_UI->actionEnableQtWarnings,			&QAction::toggled, this, &MainWindow::doEnableQtWarnings);

	connect(m_UI->actionAbout,						&QAction::triggered, this, [this] () {
		ccAboutDialog* aboutDialog = new ccAboutDialog(this);
		aboutDialog->exec();
	});

	/*** Toolbars ***/

	//View toolbar
	connect(m_UI->actionGlobalZoom,					&QAction::triggered, this, &MainWindow::setGlobalZoom);
	connect(m_UI->actionPickRotationCenter,			&QAction::triggered, this, &MainWindow::doPickRotationCenter);
	connect(m_UI->actionZoomAndCenter,				&QAction::triggered, this, &MainWindow::zoomOnSelectedEntities);
	connect(m_UI->actionSetPivotAlwaysOn,			&QAction::triggered, this, &MainWindow::setPivotAlwaysOn);
	connect(m_UI->actionSetPivotRotationOnly,		&QAction::triggered, this, &MainWindow::setPivotRotationOnly);
	connect(m_UI->actionSetPivotOff,				&QAction::triggered, this, &MainWindow::setPivotOff);
	
	connect(m_UI->actionSetOrthoView,               &QAction::triggered, this, [this] () {
		setOrthoView( getActiveGLWindow() );
	});
	connect(m_UI->actionSetCenteredPerspectiveView, &QAction::triggered, this, [this] () {
		setCenteredPerspectiveView( getActiveGLWindow() );
	});
	connect(m_UI->actionSetViewerPerspectiveView,   &QAction::triggered, this, [this] () {
		setViewerPerspectiveView( getActiveGLWindow() );
	});
	
	connect(m_UI->actionEnableStereo,				&QAction::toggled, this, &MainWindow::toggleActiveWindowStereoVision);
	connect(m_UI->actionAutoPickRotationCenter,		&QAction::toggled, this, &MainWindow::toggleActiveWindowAutoPickRotCenter);
	
	connect(m_UI->actionSetViewTop,                 &QAction::triggered, this, [=]() { setView( CC_TOP_VIEW ); });
	connect(m_UI->actionSetViewBottom,              &QAction::triggered, this, [=]() { setView( CC_BOTTOM_VIEW ); });
	connect(m_UI->actionSetViewFront,               &QAction::triggered, this, [=]() { setView( CC_FRONT_VIEW ); });
	connect(m_UI->actionSetViewBack,                &QAction::triggered, this, [=]() { setView( CC_BACK_VIEW ); });
	connect(m_UI->actionSetViewLeft,                &QAction::triggered, this, [=]() { setView( CC_LEFT_VIEW ); });
	connect(m_UI->actionSetViewRight,               &QAction::triggered, this, [=]() { setView( CC_RIGHT_VIEW ); });
	connect(m_UI->actionSetViewIso1,                &QAction::triggered, this, [=]() { setView( CC_ISO_VIEW_1 ); });
	connect(m_UI->actionSetViewIso2,                &QAction::triggered, this, [=]() { setView( CC_ISO_VIEW_2 ); });
	
	//hidden
	connect(m_UI->actionEnableVisualDebugTraces,	&QAction::triggered, this, &MainWindow::toggleVisualDebugTraces);
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

void MainWindow::doActionInterpolateScalarFields()
{
	if (!ccEntityAction::interpolateSFs(m_selectedEntities, this))
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

void MainWindow::doActionColorFromScalars()
{
	for (ccHObject *entity : getSelectedEntities())
	{
		//for "real" point clouds only
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (cloud)
		{
			//create color from scalar dialogue
			ccColorFromScalarDlg* cfsDlg = new ccColorFromScalarDlg(this, cloud);
			cfsDlg->setAttribute(Qt::WA_DeleteOnClose, true);
			cfsDlg->show();
		}
	}
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
	ccGenericPointCloud* cloud = nullptr;

	if ( haveOneSelection() )
	{
		ccHObject* ent = m_selectedEntities.front();
		bool lockedVertices;
		cloud = ccHObjectCaster::ToGenericPointCloud(ent, &lockedVertices);
		if (lockedVertices)
		{
			ccUtils::DisplayLockedVerticesWarning(ent->getName(), true);
			return;
		}
	}

	if (!cloud)
	{
		ccLog::Error(tr("Selected one and only one point cloud or mesh!"));
		return;
	}

	bool ok;
	s_kdTreeMaxErrorPerCell = QInputDialog::getDouble(this, tr("Compute Kd-tree"), tr("Max error per leaf cell:"), s_kdTreeMaxErrorPerCell, 1.0e-6, 1.0e6, 6, &ok);
	if (!ok)
		return;

	ccProgressDialog pDlg(true, this);

	//computation
	QElapsedTimer eTimer;
	eTimer.start();
	ccKdTree* kdtree = new ccKdTree(cloud);

	if (kdtree->build(s_kdTreeMaxErrorPerCell, CCCoreLib::DistanceComputationTools::MAX_DIST_95_PERCENT, 4, 1000, &pDlg))
	{
		qint64 elapsedTime_ms = eTimer.elapsed();

		ccConsole::Print("[doActionComputeKdTree] Timing: %2.3f s", elapsedTime_ms / 1.0e3);
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
		ccLog::Error(tr("An error occurred"));
		delete kdtree;
		kdtree = nullptr;
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
	int pointCount = QInputDialog::getInt(this, tr("Resample with octree"), tr("Points (approx.)"), 1000000, 1, INT_MAX, 100000, &ok);
	if (!ok)
		return;

	ccProgressDialog pDlg(false, this);
	pDlg.setAutoClose(false);

	assert(pointCount > 0);
	unsigned aimedPoints = static_cast<unsigned>(pointCount);

	bool errors = false;

	for ( ccHObject *entity : getSelectedEntities() )
	{
		ccPointCloud* cloud = nullptr;

		/*if (ent->isKindOf(CC_TYPES::MESH)) //TODO
			cloud = ccHObjectCaster::ToGenericMesh(ent)->getAssociatedCloud();
		else */
		if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			cloud = static_cast<ccPointCloud*>(entity);
		}

		if (cloud)
		{
			ccOctree::Shared octree = cloud->getOctree();
			if (!octree)
			{
				octree = cloud->computeOctree(&pDlg);
				if (!octree)
				{
					ccConsole::Error(tr("Could not compute octree for cloud '%1'").arg(cloud->getName()));
					continue;
				}
			}

			cloud->setEnabled(false);
			QElapsedTimer eTimer;
			eTimer.start();
			CCCoreLib::GenericIndexedCloud* result = CCCoreLib::CloudSamplingTools::resampleCloudWithOctree
			(
				cloud,
				aimedPoints,
				CCCoreLib::CloudSamplingTools::CELL_GRAVITY_CENTER,
				&pDlg,
				octree.data()
			);

			if (result)
			{
				ccConsole::Print("[ResampleWithOctree] Timing: %3.2f s.", eTimer.elapsed() / 1.0e3);
				ccPointCloud* newCloud = ccPointCloud::From(result, cloud);

				delete result;
				result = nullptr;

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
		ccLog::Error(tr("[ResampleWithOctree] Errors occurred during the process, result may be incomplete"));

	refreshAll();
}

void MainWindow::doActionApplyTransformation()
{
	ccApplyTransformationDlg dlg(this);
	if (!dlg.exec())
		return;

	bool applyToGlobal = false;
	ccGLMatrixd transMat = dlg.getTransformation(applyToGlobal);
	applyTransformation(transMat, applyToGlobal);
}

ccHObject::Container MainWindow::getTopLevelSelectedEntities() const
{
	ccHObject::Container topLevelSelectedEntities;
	for (size_t i = 0; i < m_selectedEntities.size(); ++i)
	{
		ccHObject* entity = m_selectedEntities[i];
		bool hasParentsInselection = false;
		for (size_t j = 0; j < m_selectedEntities.size(); ++j)
		{
			if (i == j)
				continue;

			ccHObject* otherEntity = m_selectedEntities[j];
			if (otherEntity->isAncestorOf(entity))
			{
				hasParentsInselection = true;
				break;
			}
		}

		if (!hasParentsInselection)
		{
			topLevelSelectedEntities.push_back(entity);
		}
	}

	return topLevelSelectedEntities;
}

void MainWindow::applyTransformation(const ccGLMatrixd& mat, bool applyToGlobal)
{
	//if the transformation is partly converted to global shift/scale
	bool autoApplyPreviousGlobalShiftAndScale = false;
	double previousScale = 1.0;
	CCVector3d previousShift(0, 0, 0);

	//we don't want any entity that would be the children of other selected entities
	ccHObject::Container selectedEntities = getTopLevelSelectedEntities();

	for (ccHObject* entity : selectedEntities) //warning, getSelectedEntites may change during this loop!
	{
		ccGLMatrixd transMat = mat;

		if (applyToGlobal && nullptr != dynamic_cast<ccShiftedObject*>(entity))
		{
			// the user wants to apply the transformation to the global coordinates
			ccShiftedObject* shiftedEntity = static_cast<ccShiftedObject*>(entity);
			CCVector3d globalShift = shiftedEntity->getGlobalShift();
			double globalScale = shiftedEntity->getGlobalScale();

			// we compute the impact to the local coordinate system without changing the
			// current Global Shift & Scale parameters (for now)
			// Here is the formula, assuming:
			// - the Global Shift is Ts
			// - the Global scale is Sc
			// - the transformation is (R, T)
			// - the Global point coordinates Pg are derived from the local ones Pl with: Pg = Pl/Sc - Ts
			// Therefore, Pg' = R * Pg + T
			// i.e.       Pg' = R.(Pl/Sc - Ts) + T
			// i.e.       Pg' = (R.Pl)/Sc - R.Ts + T
			// i.e.       Pg' = (R.Pl)/Sc - R.Ts + T + Ts - Ts
			// i.e.       Pg' = (R.Pl + Sc.[Ts - R.Ts + T])/Sc - Ts
			// i.e.       Pl' = Sc.[Ts -R.Ts + T]
			// i.e. the translation of the 'local' coordinate system is: Sc.[T + Ts - R.Ts]
			CCVector3d rotatedGlobalShift = globalShift;
			mat.applyRotation(rotatedGlobalShift);
			CCVector3d localTranslation = globalScale * (globalShift - rotatedGlobalShift + mat.getTranslationAsVec3D());

			// we switch to a local transformation matrix
			transMat.setTranslation(localTranslation);
		}

		//we don't test primitives (it's always ok while the 'vertices lock' test would fail)
		if (!entity->isKindOf(CC_TYPES::PRIMITIVE))
		{
			//specific test for locked vertices
			bool lockedVertices;
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity, &lockedVertices);
			if (cloud)
			{
				if (lockedVertices)
				{
					ccUtils::DisplayLockedVerticesWarning(entity->getName(), haveOneSelection());
					continue;
				}

				//test if the translated cloud coordinates were already "too large"
				//(in which case we won't bother the user about the fact that the transformed cloud coordinates will be too large...)
				ccBBox localBBox = entity->getOwnBB();
				CCVector3d Pl = localBBox.minCorner();
				double Dl = localBBox.getDiagNormd();

				//if the cloud is alright
				if (	!ccGlobalShiftManager::NeedShift(Pl)
					&&	!ccGlobalShiftManager::NeedRescale(Dl) )
				{
					//test if the translated cloud (local) coordinates are too large
					ccBBox transformedLocalBox = entity->getOwnBB() * transMat;
					CCVector3d transformedPl = transformedLocalBox.minCorner();
					double transformedDl = transformedLocalBox.getDiagNormd();

					bool needShift = ccGlobalShiftManager::NeedShift(transformedPl) || ccGlobalShiftManager::NeedRescale(transformedDl);
					if (needShift)
					{
						//existing shift information
						CCVector3d globalShift = cloud->getGlobalShift();
						double globalScale = cloud->getGlobalScale();

						//we compute the global coordinates and scale of the reference point (= the min corner of the bounding-box)
						CCVector3d Pg = cloud->toGlobal3d(transformedPl);
						double Dg = transformedDl / globalScale;

						//let's try to find better Global Shift and Scale values
						CCVector3d newShift(0.0, 0.0, 0.0);
						double newScale = 1.0;
						bool updateShiftAndscale = false;

						//should we try to use the previous Global Shift and Scale values?
						if (autoApplyPreviousGlobalShiftAndScale)
						{
							if (	!ccGlobalShiftManager::NeedShift(Pg + previousShift)
								&&	!ccGlobalShiftManager::NeedRescale(Dg * previousScale))
							{
								newScale = previousScale;
								newShift = previousShift;
								needShift = false;
								updateShiftAndscale = true;
							}
						}

						//if we still need to define new Global Shift and Scale
						if (needShift)
						{
							//ask the user the right values!
							ccShiftAndScaleCloudDlg sasDlg(transformedPl, transformedDl, Pg, Dg, this);
							sasDlg.showApplyAllButton(selectedEntities.size() > 1);
							sasDlg.showTitle(true);
							sasDlg.setKeepGlobalPos(true);
							sasDlg.showKeepGlobalPosCheckbox(false); //we don't want the user to mess with this!
							sasDlg.showPreserveShiftOnSave(true);

							//add "original" entry
							int index = sasDlg.addShiftInfo(ccGlobalShiftManager::ShiftInfo(tr("Original"), globalShift, globalScale));
							//add "previous" entry (if any)
							if (autoApplyPreviousGlobalShiftAndScale)
							{
								index = sasDlg.addShiftInfo(ccGlobalShiftManager::ShiftInfo(tr("Previous"), previousShift, previousScale));
							}
							//add "last" entries (if any)
							int matchingIndex = -1;
							const auto& previousEntries = ccGlobalShiftManager::GetLast();
							for (const ccGlobalShiftManager::ShiftInfo& shiftInfo : previousEntries)
							{
								index = sasDlg.addShiftInfo(shiftInfo);

								if (matchingIndex < 0)
								{
									if (	!ccGlobalShiftManager::NeedShift(Pg + shiftInfo.shift)
										&&	!ccGlobalShiftManager::NeedRescale(Dg * shiftInfo.scale) )
									{
										matchingIndex = index;
									}
								}
							}

							//if no good solution was found...
							if (matchingIndex < 0)
							{
								//add a "suggested" entry
								CCVector3d suggestedShift = ccGlobalShiftManager::BestShift(Pg);
								double suggestedScale = ccGlobalShiftManager::BestScale(Dg);
								matchingIndex = sasDlg.addShiftInfo(ccGlobalShiftManager::ShiftInfo(tr("Suggested"), suggestedShift, suggestedScale));
							}

							sasDlg.setCurrentProfile(matchingIndex);
							if (sasDlg.exec())
							{
								newScale = sasDlg.getScale();
								newShift = sasDlg.getShift();
								needShift = false;
								updateShiftAndscale = true;

								//store the shift for next time!
								ccGlobalShiftManager::StoreShift(newShift, newScale);

								if (sasDlg.applyAll())
								{
									autoApplyPreviousGlobalShiftAndScale = true;
									previousScale = newScale;
									previousShift = newShift;
								}
							}
							else if (sasDlg.cancelled())
							{
								ccLog::Warning(tr("[ApplyTransformation] Process cancelled by user"));
								return;
							}
							else
							{
								// the user did not want to change the shift & scale
							}
						}

						if (updateShiftAndscale)
						{
							assert(!needShift);

							//get the relative modification to existing global shift/scale info
							assert(globalScale != 0);
							double scaleChange = newScale / globalScale;
							CCVector3d shiftChange = newShift - globalShift;

							if (scaleChange != 1.0 || shiftChange.norm2() != 0)
							{
								//apply translation as global shift
								cloud->setGlobalShift(newShift);
								cloud->setGlobalScale(newScale);
								ccLog::Warning(tr("[ApplyTransformation] Cloud '%1' global shift/scale information has been updated: shift = (%2,%3,%4) / scale = %5").arg(cloud->getName()).arg(newShift.x).arg(newShift.y).arg(newShift.z).arg(newScale));

								transMat.scaleRotation(scaleChange);
								transMat.setTranslation(transMat.getTranslationAsVec3D() + newScale * shiftChange);
							}
						}
					}
				}
			}
		}

		//we temporarily detach the entity, as it may undergo
		//'severe' modifications (octree deletion, etc.) --> see ccHObject::applyRigidTransformation
		ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(entity);
		entity->setGLTransformation(ccGLMatrix(transMat.data()));

		//DGM FIXME: we only test the entity own bounding box (and we update its shift & scale info) but we apply the transformation to all its children?!
		entity->applyGLTransformation_recursive();
		entity->prepareDisplayForRefresh_recursive();
		putObjectBackIntoDBTree(entity, objContext);

		if (applyToGlobal)
		{
			ccLog::Print(tr("[ApplyTransformation] Transformation matrix applied to the local coordinates of %1:").arg(entity->getName()));
			ccLog::Print(transMat.toString(12, ' ')); //full precision
		}
	}

	if (!applyToGlobal)
	{
		ccLog::Print(tr("[ApplyTransformation] Applied transformation matrix:"));
	}
	else
	{
		ccLog::Print(tr("[ApplyTransformation] Global transformation matrix:"));
	}
	ccLog::Print(mat.toString(12, ' ')); //full precision
	ccLog::Print(tr("Hint: you can copy a transformation matrix (CTRL+C) and apply it - or its inverse - to another entity with the 'Edit > Apply transformation' tool"));

	//reselect previously selected entities!
	if (m_ccRoot)
		m_ccRoot->selectEntities(selectedEntities);

	zoomOnSelectedEntities();

	refreshAll();
}

void MainWindow::doActionApplyScale()
{
	ccScaleDlg dlg(this);
	if (!dlg.exec())
		return;
	dlg.saveState();

	//save values for next time
	CCVector3d scales = dlg.getScales();
	bool keepInPlace = dlg.keepInPlace();
	bool rescaleGlobalShift = dlg.rescaleGlobalShift();

	//we must backup 'm_selectedEntities' as removeObjectTemporarilyFromDBTree can modify it!
	ccHObject::Container selectedEntities = m_selectedEntities;

	//first check that all coordinates are kept 'small'
	std::vector< std::pair<ccHObject*, ccGenericPointCloud*> > candidates;
	{
		bool testBigCoordinates = true;

		for (ccHObject *entity : selectedEntities) //warning, getSelectedEntites may change during this loop!
		{
			bool lockedVertices;
			//try to get the underlying cloud (or the vertices set for a mesh)
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity, &lockedVertices);
			//otherwise we can look if the selected entity is a polyline
			if (!cloud && entity->isA(CC_TYPES::POLY_LINE))
			{
				cloud = dynamic_cast<ccGenericPointCloud*>(static_cast<ccPolyline*>(entity)->getAssociatedCloud());
				if (!cloud || cloud->isAncestorOf(entity))
				{
					lockedVertices = true;
				}
			}
			if (!cloud || !cloud->isKindOf(CC_TYPES::POINT_CLOUD))
			{
				ccLog::Warning(tr("[Apply scale] Entity '%1' can't be scaled this way").arg(entity->getName()));
				continue;
			}
			if (lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(entity->getName(), haveOneSelection());
				continue;
			}

			CCVector3 C(0, 0, 0);
			if (keepInPlace)
			{
				C = cloud->getOwnBB().getCenter();
			}

			//we must check that the resulting cloud coordinates are not too big
			if (testBigCoordinates)
			{
				ccBBox bbox = cloud->getOwnBB();
				CCVector3 bbMin = bbox.minCorner();
				CCVector3 bbMax = bbox.maxCorner();

				double maxx = std::max(std::abs(bbMin.x), std::abs(bbMax.x));
				double maxy = std::max(std::abs(bbMin.y), std::abs(bbMax.y));
				double maxz = std::max(std::abs(bbMin.z), std::abs(bbMax.z));

				const double maxCoord = ccGlobalShiftManager::MaxCoordinateAbsValue();
				bool oldCoordsWereTooBig = (	maxx > maxCoord
											||	maxy > maxCoord
											||	maxz > maxCoord );

				if (!oldCoordsWereTooBig)
				{
					maxx = std::max(std::abs((bbMin.x - C.x) * scales.x + C.x), std::abs((bbMax.x - C.x) * scales.x + C.x));
					maxy = std::max(std::abs((bbMin.y - C.y) * scales.y + C.y), std::abs((bbMax.y - C.y) * scales.y + C.y));
					maxz = std::max(std::abs((bbMin.z - C.z) * scales.z + C.z), std::abs((bbMax.z - C.z) * scales.z + C.z));

					bool newCoordsAreTooBig = (	maxx > maxCoord
											||	maxy > maxCoord
											||	maxz > maxCoord );

					if (newCoordsAreTooBig)
					{
						if (QMessageBox::question(
							this,
							tr("Big coordinates"),
							tr("Resutling coordinates will be too big (original precision may be lost!). Proceed anyway?"),
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
			candidates.emplace_back(entity, cloud);
		}
	}

	if (candidates.empty())
	{
		ccConsole::Warning(tr("[Apply scale] No eligible entities (point clouds or meshes) were selected!"));
		return;
	}

	//now do the real scaling work
	{
		for ( auto& candidate : candidates )
		{
			ccHObject* ent = candidate.first;
			ccGenericPointCloud* cloud = candidate.second;

			CCVector3 C(0, 0, 0);
			if (keepInPlace)
			{
				C = cloud->getOwnBB().getCenter();
			}

			//we temporarily detach the entity, as it may undergo
			//'severe' modifications (octree deletion, etc.) --> see ccPointCloud::scale
			ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(cloud);

			cloud->scale(	static_cast<PointCoordinateType>(scales.x),
							static_cast<PointCoordinateType>(scales.y),
							static_cast<PointCoordinateType>(scales.z),
							C );

			putObjectBackIntoDBTree(cloud, objContext);
			cloud->prepareDisplayForRefresh_recursive();

			//don't forget the 'global shift'!
			//DGM: but not the global scale!
			if (rescaleGlobalShift)
			{
				const CCVector3d& shift = cloud->getGlobalShift();
				cloud->setGlobalShift( CCVector3d(	shift.x*scales.x,
													shift.y*scales.y,
													shift.z*scales.z) );
			}

			//specific case for polyline vertices
			if (cloud->getParent() && cloud->getParent()->isA(CC_TYPES::POLY_LINE))
			{
				ccPolyline* poly = static_cast<ccPolyline*>(cloud->getParent());
				if (poly->getAssociatedCloud() == cloud)
				{
					poly->invalidateBoundingBox();
				}
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
	//get the global shift/scale info and bounding box of all selected clouds
	std::vector< std::pair<ccShiftedObject*, ccHObject*> > shiftedEntities;
	CCVector3d Pl(0, 0, 0);
	double Dl = 1.0;
	CCVector3d Pg(0, 0, 0);
	double Dg = 1.0;
	//shift and scale (if unique)
	CCVector3d shift(0, 0, 0);
	double scale = 1.0;
	{
		bool uniqueShift = true;
		bool uniqueScale = true;
		ccBBox localBB;
		//sadly we don't have a double-typed bounding box class yet ;)
		CCVector3d globalBBmin(0, 0, 0);
		CCVector3d globalBBmax(0, 0, 0);

		for ( ccHObject *entity : getSelectedEntities() )
		{
			bool lockedVertices;
			ccShiftedObject* shifted = ccHObjectCaster::ToShifted(entity, &lockedVertices);
			if (!shifted)
			{
				continue;
			}
			//for (unlocked) entities only
			if (lockedVertices)
			{
				//get the vertices
				ccGenericPointCloud* vertices = nullptr;
				//if it's a mesh
				if (entity->isKindOf(CC_TYPES::MESH))
				{
					vertices = static_cast<ccGenericMesh*>(entity)->getAssociatedCloud();
				}
				else if (entity->isKindOf(CC_TYPES::POLY_LINE))
				{
					vertices = dynamic_cast<ccGenericPointCloud*>(static_cast<ccPolyline*>(entity)->getAssociatedCloud());
				}
				if (!vertices || !entity->isAncestorOf(vertices))
				{
					ccUtils::DisplayLockedVerticesWarning(entity->getName(), haveOneSelection());
					continue;
				}
				entity = vertices;
			}

			CCVector3 Al = entity->getOwnBB().minCorner();
			CCVector3 Bl = entity->getOwnBB().maxCorner();
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
				globalBBmin = CCVector3d(	std::min(globalBBmin.x, Ag.x),
											std::min(globalBBmin.y, Ag.y),
											std::min(globalBBmin.z, Ag.z) );
				globalBBmax = CCVector3d(	std::max(globalBBmax.x, Bg.x),
											std::max(globalBBmax.y, Bg.y),
											std::max(globalBBmax.z, Bg.z) );

				if (uniqueShift)
				{
					uniqueShift = CCCoreLib::LessThanEpsilon((shifted->getGlobalShift() - shift).norm());
				}
				if (uniqueScale)
				{
					uniqueScale = CCCoreLib::LessThanEpsilon(std::abs(shifted->getGlobalScale() - scale));
				}
			}

			shiftedEntities.emplace_back(shifted, entity);
		}

		Pg = globalBBmin;
		Dg = (globalBBmax - globalBBmin).norm();

		Pl = localBB.minCorner();
		Dl = (localBB.maxCorner() - localBB.minCorner()).normd();

		if (!uniqueShift)
			shift = Pl - Pg;
		if (!uniqueScale)
			scale = Dg / Dl;
	}

	if (shiftedEntities.empty())
	{
		return;
	}

	ccShiftAndScaleCloudDlg sasDlg(Pl, Dl, Pg, Dg, this);
	sasDlg.showApplyAllButton(shiftedEntities.size() > 1);
	sasDlg.showApplyButton(shiftedEntities.size() == 1);
	sasDlg.showNoButton(false);
	sasDlg.setShiftFieldsPrecision(6);
	//add "original" entry
	int index = sasDlg.addShiftInfo(ccGlobalShiftManager::ShiftInfo(tr("Original"), shift, scale));
	sasDlg.setCurrentProfile(index);
	//add "last" entries (if any)
	sasDlg.addShiftInfo(ccGlobalShiftManager::GetLast());

	if (!sasDlg.exec())
		return;

	shift = sasDlg.getShift();
	scale = sasDlg.getScale();
	bool preserveGlobalPos = sasDlg.keepGlobalPos();

	ccLog::Print(tr("[Global Shift/Scale] New shift: (%1, %2, %3)").arg(shift.x).arg(shift.y).arg(shift.z));
	ccLog::Print(tr("[Global Shift/Scale] New scale: %1").arg(scale));

	//apply new shift
	{
		for ( auto &entity : shiftedEntities )
		{
			ccShiftedObject* shifted = entity.first;
			ccHObject* ent = entity.second;
			if (preserveGlobalPos)
			{
				//to preserve the global position of the cloud, we may have to translate and/or rescale the cloud
				CCVector3d Ql = ent->getOwnBB().minCorner();
				CCVector3d Qg = shifted->toGlobal3d(Ql);
				CCVector3d Ql2 = Qg * scale + shift;
				CCVector3d T = Ql2 - Ql;

				assert(shifted->getGlobalScale() > 0);
				double scaleCoef = scale / shifted->getGlobalScale();

				if ( CCCoreLib::GreaterThanEpsilon( T.norm() )
					 || CCCoreLib::GreaterThanEpsilon( std::abs(scaleCoef - 1.0) ) )
				{
					ccGLMatrix transMat;
					transMat.toIdentity();
					transMat.scaleRotation(static_cast<float>(scaleCoef));
					transMat.setTranslation(T);

					//DGM FIXME: we only test the entity own bounding box (and we update its shift & scale info) but we apply the transformation to all its children?!
					ent->applyGLTransformation_recursive(&transMat);
					ent->prepareDisplayForRefresh_recursive();

					ccLog::Warning(tr("[Global Shift/Scale] To preserve its original position, the entity '%1' has been translated of (%2 ; %3 ; %4) and rescaled of a factor %5")
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
								tr("This method is for test purpose only"),
								tr("Cloud(s) are going to be rotated while still displayed in their previous position! Proceed?"),
								QMessageBox::Yes | QMessageBox::No,
								QMessageBox::No ) != QMessageBox::Yes)
	{
		return;
	}

	//backup selected entities as removeObjectTemporarilyFromDBTree can modify them
	ccHObject::Container selectedEntities = getSelectedEntities();

	for (ccHObject *entity : selectedEntities) //warning, getSelectedEntites may change during this loop!
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);

		if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD)) // TODO
		{
			CCCoreLib::Neighbourhood Yk(cloud);

			CCCoreLib::SquareMatrixd covMat = Yk.computeCovarianceMatrix();
			if (covMat.isValid())
			{
				CCCoreLib::SquareMatrixd eigVectors;
				std::vector<double> eigValues;
				if (CCCoreLib::Jacobi<double>::ComputeEigenValuesAndVectors(covMat, eigVectors, eigValues, true))
				{
					CCCoreLib::Jacobi<double>::SortEigenValuesAndVectors(eigVectors, eigValues);

					ccGLMatrix trans;
					GLfloat* rotMat = trans.data();
					for (unsigned j = 0; j < 3; ++j)
					{
						double u[3];
						CCCoreLib::Jacobi<double>::GetEigenVector(eigVectors, j, u);
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

					//we temporarily detach the entity, as it may undergo
					//'severe' modifications (octree deletion, etc.) --> see ccPointCloud::applyRigidTransformation
					ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(cloud);
					static_cast<ccPointCloud*>(cloud)->applyRigidTransformation(trans);
					putObjectBackIntoDBTree(cloud,objContext);

					entity->prepareDisplayForRefresh_recursive();
				}
			}
		}
	}

	refreshAll();
}

void MainWindow::doActionFlagMeshVertices()
{
	bool errors = false;
	bool success = false;

	for ( ccHObject *entity : getSelectedEntities() )
	{
		if (entity->isKindOf(CC_TYPES::MESH))
		{
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(entity);
			ccPointCloud* vertices = ccHObjectCaster::ToPointCloud(mesh ? mesh->getAssociatedCloud() : nullptr);
			if (mesh && vertices)
			{
				//prepare a new scalar field
				int sfIdx = vertices->getScalarFieldIndexByName(CC_DEFAULT_MESH_VERT_FLAGS_SF_NAME);
				if (sfIdx < 0)
				{
					sfIdx = vertices->addScalarField(CC_DEFAULT_MESH_VERT_FLAGS_SF_NAME);
					if (sfIdx < 0)
					{
						ccConsole::Warning(tr("Not enough memory to flag the vertices of mesh '%1'!").arg(mesh->getName()));
						errors = true;
						continue;
					}
				}
				CCCoreLib::ScalarField* flags = vertices->getScalarField(sfIdx);

				CCCoreLib::MeshSamplingTools::EdgeConnectivityStats stats;
				if (CCCoreLib::MeshSamplingTools::flagMeshVerticesByType(mesh,flags,&stats))
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
					ccConsole::Print(tr("[Mesh Quality] Mesh '%1' edges: %2 total (normal: %3 / on hole borders: %4 / non-manifold: %5)").arg(entity->getName()).arg(stats.edgesCount).arg(stats.edgesSharedByTwo).arg(stats.edgesNotShared).arg(stats.edgesSharedByMore));
				}
				else
				{
					vertices->deleteScalarField(sfIdx);
					sfIdx = -1;
					ccConsole::Warning(tr("Not enough memory to flag the vertices of mesh '%1'!").arg(mesh->getName()));
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
		ccConsole::Print(tr("[Mesh Quality] SF flags: %1 (NORMAL) / %2 (BORDER) / (%3) NON-MANIFOLD").arg(CCCoreLib::MeshSamplingTools::VERTEX_NORMAL).arg(CCCoreLib::MeshSamplingTools::VERTEX_BORDER).arg(CCCoreLib::MeshSamplingTools::VERTEX_NON_MANIFOLD));
	}

	if (errors)
	{
		ccConsole::Error(tr("Error(s) occurred! Check the console..."));
	}
}

void MainWindow::doActionMeasureMeshVolume()
{
	for ( ccHObject *entity : getSelectedEntities() )
	{
		if (entity->isKindOf(CC_TYPES::MESH))
		{
			ccMesh* mesh = ccHObjectCaster::ToMesh(entity);
			if (mesh)
			{
				//we compute the mesh volume
				double V = CCCoreLib::MeshSamplingTools::computeMeshVolume(mesh);
				//we force the console to display itself
				forceConsoleDisplay();
				ccConsole::Print(tr("[Mesh Volume] Mesh '%1': V=%2 (cube units)").arg(entity->getName()).arg(V));

				//check that the mesh is closed
				CCCoreLib::MeshSamplingTools::EdgeConnectivityStats stats;
				if (CCCoreLib::MeshSamplingTools::computeMeshEdgesConnectivity(mesh, stats))
				{
					if (stats.edgesNotShared != 0)
					{
						ccConsole::Warning(tr("[Mesh Volume] The above volume might be invalid (mesh has holes)"));
					}
					else if (stats.edgesSharedByMore != 0)
					{
						ccConsole::Warning(tr("[Mesh Volume] The above volume might be invalid (mesh has non-manifold edges)"));
					}
				}
				else
				{
					ccConsole::Warning(tr("[Mesh Volume] The above volume might be invalid (not enough memory to check if the mesh is closed)"));
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
	for ( ccHObject *entity : getSelectedEntities() )
	{
		if (entity->isKindOf(CC_TYPES::MESH))
		{
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(entity);
			if (mesh)
			{
				double S = CCCoreLib::MeshSamplingTools::computeMeshArea(mesh);
				//we force the console to display itself
				forceConsoleDisplay();
				ccConsole::Print(tr("[Mesh Surface] Mesh '%1': S=%2 (square units)").arg(entity->getName()).arg(S));
				if (mesh->size())
				{
					ccConsole::Print(tr("[Mesh Surface] Average triangle surface: %1 (square units)").arg(S / double(mesh->size())));
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
	if (!haveSelection())
	{
		ccConsole::Error(tr("Select at least one sensor"));
		return;
	}

	//start dialog
	ccSensorComputeDistancesDlg cdDlg(this);
	if (!cdDlg.exec())
		return;

	for ( ccHObject *entity : getSelectedEntities() )
	{
		ccSensor* sensor = ccHObjectCaster::ToSensor( entity );
		assert(sensor);
		if (!sensor)
			continue; //skip this entity

		//get associated cloud
		ccHObject* defaultCloud = sensor->getParent() && sensor->getParent()->isA(CC_TYPES::POINT_CLOUD) ? sensor->getParent() : nullptr;
		ccPointCloud* cloud = askUserToSelectACloud(defaultCloud, tr("Select a cloud on which to project the uncertainty:"));
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
				ccConsole::Error(tr("Not enough memory!"));
				return;
			}
		}
		CCCoreLib::ScalarField* distances = cloud->getScalarField(sfIdx);

		for (unsigned i = 0; i < cloud->size(); ++i)
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
	if (!haveOneSelection() || !m_selectedEntities.front()->isKindOf(CC_TYPES::GBL_SENSOR))
	{
		ccConsole::Error(tr("Select one and only one GBL sensor!"));
		return;
	}

	ccSensor* sensor = ccHObjectCaster::ToSensor(m_selectedEntities.front());
	assert(sensor);

	//sensor center
	CCVector3 sensorCenter;
	if (!sensor->getActiveAbsoluteCenter(sensorCenter))
		return;

	//get associated cloud
	ccHObject* defaultCloud = sensor->getParent() && sensor->getParent()->isA(CC_TYPES::POINT_CLOUD) ? sensor->getParent() : nullptr;
	ccPointCloud* cloud = askUserToSelectACloud(defaultCloud, tr("Select a cloud on which to project the uncertainty:"));
	if (!cloud)
	{
		return;
	}
	if (!cloud->hasNormals())
	{
		ccConsole::Error(tr("The cloud must have normals!"));
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
			ccConsole::Error(tr("Not enough memory!"));
			return;
		}
	}
	CCCoreLib::ScalarField* angles = cloud->getScalarField(sfIdx);

	//perform computations
	for (unsigned i = 0; i < cloud->size(); ++i)
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
		ScalarType theta = std::acos(std::min(std::abs(cosTheta), 1.0f));

		if (toDegreeFlag)
		{
			theta = CCCoreLib::RadiansToDegrees( theta );
		}
		
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
	if (!haveOneSelection() || !m_selectedEntities.front()->isKindOf(CC_TYPES::SENSOR))
	{
		ccConsole::Error(tr("Select one and only one sensor!"));
		return;
	}

	ccSensor* sensor = ccHObjectCaster::ToSensor(m_selectedEntities.front());
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
		ccConsole::Print(tr("[DoActionSetViewFromSensor] Viewport applied"));
	}
}

void MainWindow::doActionCreateGBLSensor()
{
	ccGBLSensorProjectionDlg spDlg(this);
	spDlg.initWithPrevious();
	if (!spDlg.exec())
		return;
	spDlg.saveForNextTime();

	//We create the corresponding sensor for each input cloud (in a perfect world, there should be only one ;)
	for ( ccHObject *entity : getSelectedEntities() )
	{
		if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);

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

				ccGLWindowInterface* win = static_cast<ccGLWindowInterface*>(cloud->getDisplay());
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
				ccLog::Error(tr("Failed to create sensor"));
				delete sensor;
				sensor = nullptr;
			}
		}
	}

	updateUI();
}

void MainWindow::doActionCreateCameraSensor()
{
	//we create the camera sensor
	ccCameraSensor* sensor = new ccCameraSensor();

	ccHObject* ent = nullptr;
	if (haveSelection())
	{
		assert(haveOneSelection());
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
	//spDlg.initWithCamSensor(sensor); //DGM: we'd better leave the default parameters of the dialog!
	if (!spDlg.exec())
	{
		delete sensor;
		return;
	}
	spDlg.updateCamSensor(sensor);

	ccGLWindowInterface* win = nullptr;
	if (ent)
	{
		ent->addChild(sensor);
		win = static_cast<ccGLWindowInterface*>(ent->getDisplay());
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
	if (!haveOneSelection() || !m_selectedEntities.front()->isKindOf(CC_TYPES::SENSOR))
	{
		ccConsole::Error(tr("Select one and only one sensor!"));
		return;
	}

	ccSensor* sensor = static_cast<ccSensor*>(m_selectedEntities.front());

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
			//ccConsole::Warning(tr("Internal error: sensor ('%1') parent is not a point cloud!").arg(sensor->getName()));
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
		ccConsole::Error(tr("Can't modify this kind of sensor!"));
		return;
	}

	if (sensor->isVisible() && sensor->isEnabled())
	{
		sensor->prepareDisplayForRefresh();
		refreshAll();
	}

	updateUI();
}

void MainWindow::doActionProjectUncertainty()
{
	//there should only be one sensor in the current selection!
	if (!haveOneSelection() || !m_selectedEntities.front()->isKindOf(CC_TYPES::CAMERA_SENSOR))
	{
		ccConsole::Error(tr("Select one and only one camera (projective) sensor!"));
		return;
	}

	ccCameraSensor* sensor = ccHObjectCaster::ToCameraSensor(m_selectedEntities.front());
	if (!sensor)
	{
		assert(false);
		return;
	}

	const ccCameraSensor::LensDistortionParameters::Shared& distParams = sensor->getDistortionParameters();
	if (!distParams || distParams->getModel() != ccCameraSensor::BROWN_DISTORTION)
	{
		ccLog::Error(tr("Sensor has no associated uncertainty model! (Brown, etc.)"));
		return;
	}

	//we need a cloud to project the uncertainty on!
	ccHObject* defaultCloud = sensor->getParent() && sensor->getParent()->isA(CC_TYPES::POINT_CLOUD) ? sensor->getParent() : nullptr;
	ccPointCloud* pointCloud = askUserToSelectACloud(defaultCloud, tr("Select a cloud on which to project the uncertainty:"));
	if (!pointCloud)
	{
		return;
	}

	CCCoreLib::ReferenceCloud points(pointCloud);
	if (!points.reserve(pointCloud->size()))
	{
		ccConsole::Error(tr("Not enough memory!"));
		return;
	}
	points.addPointIndex(0,pointCloud->size());

	// compute uncertainty
	std::vector< Vector3Tpl<ScalarType> > accuracy;
	if (!sensor->computeUncertainty(&points, accuracy/*, false*/))
	{
		ccConsole::Error(tr("Not enough memory!"));
		return;
	}

	/////////////
	// SIGMA D //
	/////////////
	const char dimChar[3] = {'x','y','z'};
	for (unsigned d = 0; d < 3; ++d)
	{
		// add scalar field
		QString sfName = tr("[%1] Uncertainty (%2)").arg(sensor->getName()).arg(dimChar[d]);
		int sfIdx = pointCloud->getScalarFieldIndexByName(sfName.toStdString());
		if (sfIdx < 0)
			sfIdx = pointCloud->addScalarField(sfName.toStdString());
		if (sfIdx < 0)
		{
			ccLog::Error(tr("An error occurred! (see console)"));
			return;
		}

		// fill scalar field
		CCCoreLib::ScalarField* sf = pointCloud->getScalarField(sfIdx);
		assert(sf);
		if (sf)
		{
			unsigned count = static_cast<unsigned>(accuracy.size());
			assert(count == pointCloud->size());
			for (unsigned i = 0; i < count; i++)
				sf->setValue(i, accuracy[i].u[d]);
			sf->computeMinAndMax();
		}
	}

	/////////////////
	// SIGMA TOTAL //
	/////////////////

	// add scalar field
	{
		QString sfName = tr("[%1] Uncertainty (3D)").arg(sensor->getName());
		int sfIdx = pointCloud->getScalarFieldIndexByName(sfName.toStdString());
		if (sfIdx < 0)
			sfIdx = pointCloud->addScalarField(sfName.toStdString());
		if (sfIdx < 0)
		{
			ccLog::Error(tr("An error occurred! (see console)"));
			return;
		}

		// fill scalar field
		CCCoreLib::ScalarField* sf = pointCloud->getScalarField(sfIdx);
		assert(sf);
		if (sf)
		{
			unsigned count = static_cast<unsigned>(accuracy.size());
			assert(count == pointCloud->size());
			for (unsigned i = 0; i < count; i++)
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
	if (!haveOneSelection() || !m_selectedEntities.front()->isKindOf(CC_TYPES::CAMERA_SENSOR))
	{
		ccConsole::Error(tr("Select one and only one camera sensor!"));
		return;
	}

	ccCameraSensor* sensor = ccHObjectCaster::ToCameraSensor(m_selectedEntities.front());
	if (!sensor)
		return;

	//we need a cloud to filter!
	ccHObject* defaultCloud = sensor->getParent() && sensor->getParent()->isA(CC_TYPES::POINT_CLOUD) ? sensor->getParent() : nullptr;
	ccPointCloud* pointCloud = askUserToSelectACloud(defaultCloud, tr("Select a cloud to filter:"));
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
			ccConsole::Error(tr("Failed to compute the octree!"));
			return;
		}
	}
	assert(octree);

	// filter octree then project the points
	std::vector<unsigned> inCameraFrustum;
	if (!octree->intersectWithFrustum(sensor, inCameraFrustum))
	{
		ccConsole::Error(tr("Failed to intersect sensor frustum with octree!"));
	}
	else
	{
		// scalar field
		const char sfName[] = "Frustum visibility";
		int sfIdx = pointCloud->getScalarFieldIndexByName(sfName);

		if (inCameraFrustum.empty())
		{
			ccConsole::Error(tr("No point fell inside the frustum!"));
			if (sfIdx >= 0)
				pointCloud->deleteScalarField(sfIdx);
		}
		else
		{
			if (sfIdx < 0)
				sfIdx = pointCloud->addScalarField(sfName);
			if (sfIdx < 0)
			{
				ccLog::Error(tr("Failed to allocate memory for output scalar field!"));
				return;
			}

			CCCoreLib::ScalarField* sf = pointCloud->getScalarField(sfIdx);
			assert(sf);
			if (sf)
			{
				sf->fill(0);

				const ScalarType c_insideValue = static_cast<ScalarType>(1);

				for ( unsigned index : inCameraFrustum )
				{
					sf->setValue(index, c_insideValue);
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
	if (!haveSelection())
		return;

	for ( ccHObject *entity : getSelectedEntities() )
	{
		if (entity->isKindOf(CC_TYPES::GBL_SENSOR))
		{
			ccGBLSensor* sensor = static_cast<ccGBLSensor*>(m_selectedEntities.front());
			if (sensor->getDepthBuffer().zBuff.empty())
			{
				//look for depending cloud
				ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity->getParent());
				if (cloud)
				{
					//force depth buffer computation
					int errorCode;
					if (!sensor->computeDepthBuffer(cloud, errorCode))
					{
						ccConsole::Error(ccGBLSensor::GetErrorString(errorCode));
					}
				}
				else
				{
					ccConsole::Error(tr("Internal error: sensor ('%1') parent is not a point cloud!").arg(sensor->getName()));
					return;
				}
			}

			ccRenderingTools::ShowDepthBuffer(sensor, this);
		}
	}
}

void MainWindow::doActionExportDepthBuffer()
{
	if (!haveSelection())
		return;

	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::SaveFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	QString filename = QFileDialog::getSaveFileName(this,
													tr("Select output file"),
													currentPath,
													DepthMapFileFilter::GetFileFilter(),
													nullptr,
													CCFileDialogOptions()
	);
	if (filename.isEmpty())
	{
		//process cancelled by user
		return;
	}

	//save last saving location
	settings.setValue(ccPS::CurrentPath(),QFileInfo(filename).absolutePath());
	settings.endGroup();

	ccHObject* toSave = nullptr;
	bool multEntities = false;
	if (haveOneSelection())
	{
		toSave = m_selectedEntities.front();
	}
	else
	{
		toSave = new ccHObject("Temp Group");

		for ( ccHObject *entity : getSelectedEntities() )
		{
			toSave->addChild(entity,ccHObject::DP_NONE);
		}
		multEntities = true;
	}

	DepthMapFileFilter::SaveParameters parameters;
	{
		parameters.alwaysDisplaySaveDialog = true;
	}
	CC_FILE_ERROR result = DepthMapFileFilter().saveToFile(toSave, filename, parameters);

	if (result != CC_FERR_NO_ERROR)
	{
		FileIOFilter::DisplayErrorMessage(result, tr("saving"), filename);
	}
	else
	{
		ccLog::Print(tr("[I/O] File '%1' saved successfully").arg(filename));
	}

	if (multEntities)
	{
		delete toSave;
		toSave = nullptr;
	}
}

void MainWindow::doActionComputePointsVisibility()
{
	//there should be only one camera sensor in the current selection!
	if (!haveOneSelection() || !m_selectedEntities.front()->isKindOf(CC_TYPES::GBL_SENSOR))
	{
		ccConsole::Error(tr("Select one and only one GBL/TLS sensor!"));
		return;
	}

	ccGBLSensor* sensor = ccHObjectCaster::ToGBLSensor(m_selectedEntities.front());
	if (!sensor)
		return;

	//we need a cloud to filter!
	ccHObject* defaultCloud = sensor->getParent() && sensor->getParent()->isA(CC_TYPES::POINT_CLOUD) ? sensor->getParent() : nullptr;
	ccPointCloud* pointCloud = askUserToSelectACloud(defaultCloud, tr("Select a cloud to filter:"));
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
										tr("Depth buffer"),
										tr("Sensor has no depth buffer: do you want to compute it now?"),
										QMessageBox::Yes | QMessageBox::No,
										QMessageBox::Yes ) == QMessageBox::No)
			{
				//we can stop then...
				return;
			}

			int errorCode;
			if (sensor->computeDepthBuffer(static_cast<ccPointCloud*>(defaultCloud), errorCode))
			{
				ccRenderingTools::ShowDepthBuffer(sensor, this);
			}
			else
			{
				ccConsole::Error(ccGBLSensor::GetErrorString(errorCode));
				return;
			}
		}
		else
		{
			ccConsole::Error(tr("Sensor has no depth buffer (and no associated cloud?)"));
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
		ccLog::Error(tr("Failed to allocate memory for output scalar field!"));
		return;
	}

	CCCoreLib::ScalarField* sf = pointCloud->getScalarField(sfIdx);
	assert(sf);
	if (sf)
	{
		sf->fill(0);

		//progress bar
		ccProgressDialog pdlg(true);
		CCCoreLib::NormalizedProgress nprogress(&pdlg,pointCloud->size());
		pdlg.setMethodTitle(tr("Compute visibility"));
		pdlg.setInfo(tr("Points: %L1").arg( pointCloud->size() ));
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
				sf = nullptr;
				break;
			}
		}

		if (sf)
		{
			sf->computeMinAndMax();
			pointCloud->setCurrentDisplayedScalarField(sfIdx);
			pointCloud->showSF(true);

			ccConsole::Print(tr("Visibility computed for cloud '%1'").arg(pointCloud->getName()));
			ccConsole::Print(tr("\tVisible = %1").arg(CCCoreLib::POINT_VISIBLE));
			ccConsole::Print(tr("\tHidden = %1").arg(CCCoreLib::POINT_HIDDEN));
			ccConsole::Print(tr("\tOut of range = %1").arg(CCCoreLib::POINT_OUT_OF_RANGE));
			ccConsole::Print(tr("\tOut of fov = %1").arg(CCCoreLib::POINT_OUT_OF_FOV));
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

void MainWindow::doActionSamplePointsOnMesh()
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
	s_useDensity = dlg.useDensity();
	s_ptsSamplingCount = dlg.getPointsNumber();
	s_ptsSamplingDensity = dlg.getDensityValue();
	s_ptsSampleNormals = withNormals;

	bool errors = false;

	for ( ccHObject *entity : getSelectedEntities() )
	{
		if (!entity->isKindOf(CC_TYPES::MESH))
			continue;
		
		ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(entity);
		assert(mesh);
		
		ccPointCloud* cloud = mesh->samplePoints(	s_useDensity,
													s_useDensity ? s_ptsSamplingDensity : s_ptsSamplingCount,
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

	if (errors)
		ccLog::Error(tr("[doActionSamplePointsOnMesh] Errors occurred during the process! Result may be incomplete!"));

	refreshAll();
}

void MainWindow::doActionSamplePointsOnPolyline()
{
	static unsigned s_ptsSamplingCount = 1000;
	static double s_ptsSamplingDensity = 10.0;
	static bool s_useDensity = false;

	ccPtsSamplingDlg dlg(this);
	//restore last parameters
	dlg.setPointsNumber(s_ptsSamplingCount);
	dlg.setDensityValue(s_ptsSamplingDensity);
	dlg.setUseDensity(s_useDensity);
	dlg.optionsFrame->setVisible(false);
	if (!dlg.exec())
		return;

	s_ptsSamplingCount = dlg.getPointsNumber();
	s_ptsSamplingDensity = dlg.getDensityValue();
	s_useDensity = dlg.useDensity();

	bool errors = false;

	for (ccHObject *entity : getSelectedEntities())
	{
		if (!entity->isKindOf(CC_TYPES::POLY_LINE))
			continue;

		ccPolyline* poly = ccHObjectCaster::ToPolyline(entity);
		assert(poly);

		ccPointCloud* cloud = poly->samplePoints(	s_useDensity,
													s_useDensity ? s_ptsSamplingDensity : s_ptsSamplingCount,
													true);

		if (cloud)
		{
			addToDB(cloud);
		}
		else
		{
			errors = true;
		}
	}

	if (errors)
	{
		ccLog::Error(tr("[DoActionSamplePointsOnPolyline] Errors occurred during the process! Result may be incomplete!"));
	}

	refreshAll();
}

void MainWindow::doActionSmoohPolyline()
{
	static int s_iterationCount = 5;
	static double s_ratio = 0.25;

	ccSmoothPolylineDialog dlg(this);
	//restore last parameters
	dlg.setIerationCount(s_iterationCount);
	dlg.setRatio(s_ratio);
	if (!dlg.exec())
		return;

	s_iterationCount = dlg.getIerationCount();
	s_ratio = dlg.getRatio();

	bool errors = false;

	ccHObject::Container selectedEntities = getSelectedEntities();
	m_ccRoot->unselectAllEntities();

	for (ccHObject *entity : selectedEntities)
	{
		if (!entity->isKindOf(CC_TYPES::POLY_LINE))
			continue;

		ccPolyline* poly = ccHObjectCaster::ToPolyline(entity);
		assert(poly);

		ccPolyline* smoothPoly = poly->smoothChaikin(s_ratio, static_cast<unsigned>(s_iterationCount));
		if (smoothPoly)
		{
			if (poly->getParent())
			{
				poly->getParent()->addChild(smoothPoly);
			}
			poly->setEnabled(false);
			addToDB(smoothPoly);

			m_ccRoot->selectEntity(smoothPoly, true);
		}
		else
		{
			errors = true;
		}
	}

	if (errors)
	{
		ccLog::Error(tr("[DoActionSmoohPolyline] Errors occurred during the process! Result may be incomplete!"));
	}

	refreshAll();
}

void MainWindow::doRemoveDuplicatePoints()
{
	if (!haveSelection())
		return;

	bool first = true;

	//persistent setting(s)
	QSettings settings;
	settings.beginGroup(ccPS::DuplicatePointsGroup());
	double minDistanceBetweenPoints = settings.value(ccPS::DuplicatePointsMinDist(),1.0e-12).toDouble();

	bool ok;
	minDistanceBetweenPoints = QInputDialog::getDouble(this, tr("Remove duplicate points"), tr("Min distance between points:"), minDistanceBetweenPoints, 0, 1.0e8, 12, &ok);
	if (!ok)
		return;

	//save parameter
	settings.setValue(ccPS::DuplicatePointsMinDist(), minDistanceBetweenPoints);

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	ccHObject::Container selectedEntities = getSelectedEntities(); //we have to use a local copy: 'unselectAllEntities' and 'selectEntity' will change the set of currently selected entities!

	for (ccHObject* entity : selectedEntities)
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (cloud)
		{
			ccPointCloud* filteredCloud = cloud->removeDuplicatePoints(minDistanceBetweenPoints, &pDlg);
			if (!filteredCloud)
			{
				ccConsole::Error(tr("Process failed (see Console)"));
				break;
			}

			if (filteredCloud != cloud) // otherwise the cloud has no duplicate point
			{
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

	if (!first)
	{
		ccConsole::Warning(tr("Previously selected entities (sources) have been hidden!"));
	}

	refreshAll();
}

void MainWindow::doActionFilterByValue()
{
	typedef std::pair<ccHObject*, ccPointCloud*> EntityAndVerticesType;
	std::vector<EntityAndVerticesType> toFilter;
	
	for ( ccHObject *entity : getSelectedEntities() )
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);
		if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD))
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
			//la methode est activee sur le champ scalaire affiche
			CCCoreLib::ScalarField* sf = pc->getCurrentDisplayedScalarField();
			if (sf)
			{
				toFilter.emplace_back(entity,pc);
			}
			else
			{
				ccConsole::Warning(tr("Entity [%1] has no active scalar field!").arg(entity->getName()));
			}
		}
	}

	if (toFilter.empty())
		return;
	
	double minVald = 0.0;
	double maxVald = 1.0;

	//compute min and max "displayed" scalar values of currently selected
	//entities (the ones with an active scalar field only!)
	{
		for (size_t i = 0; i < toFilter.size(); ++i)
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
		for ( auto &item : toFilter )
		{
			ccHObject* ent = item.first;
			ccPointCloud* pc = item.second;

			//we set as output (OUT) the currently displayed scalar field
			int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
			assert(outSfIdx >= 0);
			pc->setCurrentOutScalarField(outSfIdx);

			ccHObject* resultInside = nullptr;
			ccHObject* resultOutside = nullptr;
			if (ent->isKindOf(CC_TYPES::MESH))
			{
				pc->hidePointsByScalarValue(minVal, maxVal);
				if (ent->isA(CC_TYPES::MESH)/*|| ent->isKindOf(CC_TYPES::PRIMITIVE)*/) //TODO
					resultInside = ccHObjectCaster::ToMesh(ent)->createNewMeshFromSelection(false, nullptr, true);
				else if (ent->isA(CC_TYPES::SUB_MESH))
					resultInside = ccHObjectCaster::ToSubMesh(ent)->createNewSubMeshFromSelection(false);

				if (resultInside == ent)
				{
					//specific case: all triangles were selected, nothing to do
					ccLog::Warning(QString("Mesh %1 is fully inside the specified range").arg(ent->getName()));
					resultInside = nullptr;
				}
				else if (mode == ccFilterByValueDlg::SPLIT)
				{
					pc->invertVisibilityArray();
					if (ent->isA(CC_TYPES::MESH)/*|| ent->isKindOf(CC_TYPES::PRIMITIVE)*/) //TODO
						resultOutside = ccHObjectCaster::ToMesh(ent)->createNewMeshFromSelection(false, nullptr, true);
					else if (ent->isA(CC_TYPES::SUB_MESH))
						resultOutside = ccHObjectCaster::ToSubMesh(ent)->createNewSubMeshFromSelection(false);

					if (resultOutside == ent)
					{
						//specific case: all triangles were selected, nothing to do
						ccLog::Warning(QString("Mesh %1 is fully outside the specified range").arg(ent->getName()));
						ent->setEnabled(false);
						ent->prepareDisplayForRefresh();

						delete resultInside; // we don't need it
						resultInside = nullptr;
						resultOutside = nullptr;
					}
				}

				pc->unallocateVisibilityArray();
			}
			else if (ent->isKindOf(CC_TYPES::POINT_CLOUD))
			{
				//shortcut, as we know here that the point cloud is a "ccPointCloud"
				resultInside = pc->filterPointsByScalarValue(minVal, maxVal, false);
				
				if (resultInside == ent)
				{
					//specific case: all points were selected, nothing to do
					ccLog::Warning(QString("Cloud %1 is fully inside the specified range").arg(ent->getName()));
					resultInside = nullptr;
				}
				else if (mode == ccFilterByValueDlg::SPLIT)
				{
					resultOutside = pc->filterPointsByScalarValue(minVal, maxVal, true);
					if (resultOutside == ent)
					{
						//specific case: all points were selected, nothing to do
						ccLog::Warning(QString("Cloud %1 is fully outside the specified range").arg(ent->getName()));
						ent->setEnabled(false);
						ent->prepareDisplayForRefresh();

						delete resultInside;
						resultInside = nullptr;
						resultOutside = nullptr;
					}
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
		}
	}

	if (!results.empty())
	{
		ccConsole::Warning(tr("Previously selected entities (sources) have been hidden!"));
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
	if (!haveOneSelection())
	{
		if (haveSelection())
		{
			ccConsole::Error(tr("Select only one cloud or one mesh!"));
		}
		return;
	}
	ccHObject* ent = m_selectedEntities.front();

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
				ccConsole::Warning(tr("No active scalar field on entity '%1'").arg(ent->getName()));
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
	ccColorScaleEditorDialog cseDlg(ccColorScalesManager::GetUniqueInstance(), this, ccColorScale::Shared(nullptr), this);

	if (cseDlg.exec())
	{
		//save current scale manager state to persistent settings
		ccColorScalesManager::GetUniqueInstance()->toPersistentSettings();
	}

	updateUI();
}

void MainWindow::doActionAddIdField()
{
	if (!ccEntityAction::sfAddIdField(m_selectedEntities))
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionSplitCloudUsingSF()
{
    if (!ccEntityAction::sfSplitCloud(m_selectedEntities, this))
        return;

    refreshAll();
    updateUI();
}


void MainWindow::doActionRGBGaussianFilter()
{
	ccPointCloud::RgbFilterOptions(filterParams);
	filterParams.filterType = ccPointCloud::RGB_FILTER_TYPES::GAUSSIAN;
	if (!ccEntityAction::rgbGaussianFilter(m_selectedEntities, filterParams, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionRGBBilateralFilter()
{
	ccPointCloud::RgbFilterOptions(filterParams);
	filterParams.filterType = ccPointCloud::RGB_FILTER_TYPES::BILATERAL;
	if (!ccEntityAction::rgbGaussianFilter(m_selectedEntities, filterParams, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionRGBMeanFilter()
{
	ccPointCloud::RgbFilterOptions(filterParams);
	filterParams.filterType = ccPointCloud::RGB_FILTER_TYPES::MEAN;
	if (!ccEntityAction::rgbGaussianFilter(m_selectedEntities, filterParams, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionRGBMedianFilter()
{
	ccPointCloud::RgbFilterOptions(filterParams);
	filterParams.filterType = ccPointCloud::RGB_FILTER_TYPES::MEDIAN;
	if (!ccEntityAction::rgbGaussianFilter(m_selectedEntities, filterParams, this))
		return;

	refreshAll();
	updateUI();
}


void MainWindow::doActionSFGaussianFilter()
{
	ccPointCloud::RgbFilterOptions(filterParams);
	filterParams.filterType = ccPointCloud::RGB_FILTER_TYPES::GAUSSIAN;
	if ( !ccEntityAction::sfGaussianFilter(m_selectedEntities, filterParams, this) )
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionSFBilateralFilter()
{
	ccPointCloud::RgbFilterOptions(filterParams);
	filterParams.filterType = ccPointCloud::RGB_FILTER_TYPES::BILATERAL;
	if ( !ccEntityAction::sfGaussianFilter(m_selectedEntities, filterParams, this) )
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
	s_subdivideMaxArea = QInputDialog::getDouble(this, tr("Subdivide mesh"), tr("Max area per triangle:"), s_subdivideMaxArea, 1e-6, 1e6, 8, &ok);
	if (!ok)
		return;

	//ccProgressDialog pDlg(true, this);
	//pDlg.setAutoClose(false);
	bool warningIssued = false;

	for ( ccHObject *entity : getSelectedEntities() )
	{
		if (entity->isKindOf(CC_TYPES::MESH))
		{
			//single mesh?
			if (entity->isA(CC_TYPES::MESH))
			{
				ccMesh* mesh = static_cast<ccMesh*>(entity);

				ccMesh* subdividedMesh = nullptr;
				try
				{
					subdividedMesh = mesh->subdivide(static_cast<PointCoordinateType>(s_subdivideMaxArea));
				}
				catch(...)
				{
					ccLog::Error(tr("[Subdivide] An error occurred while trying to subdivide mesh '%1' (not enough memory?)").arg(mesh->getName()));
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
					ccConsole::Warning(tr("[Subdivide] Failed to subdivide mesh '%1' (not enough memory?)").arg(mesh->getName()));
				}
			}
			else if (!warningIssued)
			{
				ccLog::Warning(tr("[Subdivide] Works only on real meshes!"));
				warningIssued = true;
			}
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionFlipMeshTriangles()
{
	bool warningIssued = false;
	for (ccHObject *entity : getSelectedEntities())
	{
		if (entity->isKindOf(CC_TYPES::MESH))
		{
			//single mesh?
			if (entity->isA(CC_TYPES::MESH))
			{
				ccMesh* mesh = static_cast<ccMesh*>(entity);
				mesh->flipTriangles();
				mesh->prepareDisplayForRefresh();
			}
			else if (!warningIssued)
			{
				ccLog::Warning(tr("[Flip triangles] Works only on real meshes!"));
				warningIssued = true;
			}
		}
	}

	refreshAll();
}

void MainWindow::doActionSmoothMeshLaplacian()
{
	static unsigned	s_laplacianSmooth_nbIter = 20;
	static double	s_laplacianSmooth_factor = 0.2;

	bool ok;
	s_laplacianSmooth_nbIter = QInputDialog::getInt(this, tr("Smooth mesh"), tr("Iterations:"), s_laplacianSmooth_nbIter, 1, 1000, 1, &ok);
	if (!ok)
		return;
	s_laplacianSmooth_factor = QInputDialog::getDouble(this, tr("Smooth mesh"), tr("Smoothing factor:"), s_laplacianSmooth_factor, 0, 100, 3, &ok);
	if (!ok)
		return;

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	for ( ccHObject *entity : getSelectedEntities() )
	{
		if (entity->isA(CC_TYPES::MESH) || entity->isA(CC_TYPES::PRIMITIVE)) //FIXME: can we really do this with primitives?
		{
			ccMesh* mesh = ccHObjectCaster::ToMesh(entity);

			if (mesh->laplacianSmooth(	s_laplacianSmooth_nbIter,
										static_cast<PointCoordinateType>(s_laplacianSmooth_factor),
										&pDlg) )
			{
				mesh->prepareDisplayForRefresh_recursive();
			}
			else
			{
				ccConsole::Warning(tr("Failed to apply Laplacian smoothing to mesh '%1'").arg(mesh->getName()));
			}
		}
	}

	refreshAll();
	updateUI();
}

// helper for doActionMerge
void AddToRemoveList(ccHObject* toRemove, ccHObject::Container& toBeRemovedList)
{
	// is a parent or sibling already in the "toBeRemoved" list?
	size_t count = toBeRemovedList.size();
	for (size_t j = 0; j < count;)
	{
		if (toBeRemovedList[j]->isAncestorOf(toRemove))
		{
			// nothing to do, we already have an ancestor
			return;
		}
		else if (toRemove->isAncestorOf(toBeRemovedList[j]))
		{
			// we don't need to keep the children
			toBeRemovedList[j] = toBeRemovedList.back();
			toBeRemovedList.pop_back();
			count--;
		}
		else
		{
			// forward
			++j;
		}
	}

	toBeRemovedList.push_back(toRemove);
}

void MainWindow::doActionMerge()
{
	//let's look for clouds or meshes (warning: we don't mix them)
	std::vector<ccPointCloud*> clouds;
	std::vector<ccMesh*> meshes;

	try
	{
		for (ccHObject* entity : getSelectedEntities())
		{
			if (!entity)
			{
				continue;
			}

			if (entity->isA(CC_TYPES::POINT_CLOUD))
			{
				ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
				clouds.push_back(cloud);

				// check whether this cloud is an ancestor of the first cloud in the selection
				if (clouds.size() > 1)
				{
					if (clouds.back()->isAncestorOf(clouds.front()))
					{
						// this way we are sure that the first cloud is not below any other cloud
						std::swap(clouds.front(), clouds.back());
					}
				}
			}
			else if (entity->isKindOf(CC_TYPES::MESH))
			{
				ccMesh* mesh = ccHObjectCaster::ToMesh(entity);
				//this is a purely theoretical test for now!
				if (mesh && mesh->getAssociatedCloud() && mesh->getAssociatedCloud()->isA(CC_TYPES::POINT_CLOUD))
				{
					meshes.push_back(mesh);
				}
				else
				{
					ccConsole::Warning(tr("Only meshes with standard vertices are handled for now! Can't merge entity '%1'...").arg(entity->getName()));
				}
			}
			else
			{
				ccConsole::Warning(tr("Entity '%1' is neither a cloud nor a mesh, can't merge it!").arg(entity->getName()));
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error(tr("Not enough memory!"));
		return;
	}

	if (clouds.empty() && meshes.empty())
	{
		ccLog::Error(tr("Select only clouds or meshes!"));
		return;
	}
	if (!clouds.empty() && !meshes.empty())
	{
		ccLog::Error(tr("Can't mix point clouds and meshes!"));
	}

	//merge clouds?
	if (!clouds.empty())
	{
		//we deselect all selected entities (as most of them are going to disappear)
		if (m_ccRoot)
		{
			m_ccRoot->unselectAllEntities();
			assert(!haveSelection());
			//m_selectedEntities.clear();
		}

		//we will remove the useless clouds/meshes later
		ccHObject::Container toBeRemoved;

		ccPointCloud* firstCloud = nullptr;
		ccHObjectContext firstCloudContext;

		//whether to generate the 'original cloud index' scalar field or not
		CCCoreLib::ScalarField* ocIndexSF = nullptr;
		size_t cloudIndex = 0;

		//compute total size of the final cloud
		unsigned totalSize = 0;
		for (size_t i = 0; i < clouds.size(); ++i)
		{
			totalSize += clouds[i]->size();
		}

		for (size_t i = 0; i < clouds.size(); ++i)
		{
			ccPointCloud* pc = clouds[i];
			if (!firstCloud)
			{
				//we don't delete the first cloud (we'll merge the other one 'inside' it
				firstCloud = pc;
				//we still have to temporarily detach the first cloud, as it may undergo
				//'severe' modifications (octree deletion, etc.) --> see ccPointCloud::operator +=
				firstCloudContext = removeObjectTemporarilyFromDBTree(firstCloud);

				//reserve the final required number of points
				if (!firstCloud->reserve(totalSize))
				{
					ccConsole::Error(tr("Not enough memory!"));
					break;
				}

				if (QMessageBox::question(this, tr("Original cloud index"), tr("Do you want to generate a scalar field with the original cloud index?")) == QMessageBox::Yes)
				{
					int sfIdx = pc->getScalarFieldIndexByName(CC_ORIGINAL_CLOUD_INDEX_SF_NAME);
					if (sfIdx < 0)
					{
						sfIdx = pc->addScalarField(CC_ORIGINAL_CLOUD_INDEX_SF_NAME);
					}
					if (sfIdx < 0)
					{
						ccConsole::Error(tr("Couldn't allocate a new scalar field for storing the original cloud index! Try to free some memory ..."));
						return;
					}
					else
					{
						ocIndexSF = pc->getScalarField(sfIdx);
						if (ocIndexSF)
						{
							ocIndexSF->fill(0);
							firstCloud->setCurrentDisplayedScalarField(sfIdx);
						}
					}
				}
			}
			else
			{
				unsigned countBefore = firstCloud->size();
				unsigned countAdded = pc->size();
				firstCloud->append(pc, countBefore, false, false); //append without recalculating SF min/max

				//success?
				if (firstCloud->size() == countBefore + countAdded)
				{
					firstCloud->prepareDisplayForRefresh_recursive();

					ccHObject* toRemove = nullptr;
					//if the entity to remove is inside a group with a unique child, we can remove the group as well
					ccHObject* parent = pc->getParent();
					if (parent && parent->isA(CC_TYPES::HIERARCHY_OBJECT) && parent->getChildrenNumber() == 1 && parent != firstCloudContext.parent)
						toRemove = parent;
					else
						toRemove = pc;

					AddToRemoveList(toRemove, toBeRemoved);

					if (ocIndexSF)
					{
						ScalarType index = static_cast<ScalarType>(++cloudIndex);
						for (unsigned i = 0; i < countAdded; ++i)
						{
							ocIndexSF->setValue(countBefore + i, index);
						}
					}
				}
				else
				{
					ccConsole::Error(tr("Fusion failed! (not enough memory?)"));
					break;
				}
				pc = nullptr;
			}
		}

		//compute min and max once after all appends are done
		if (firstCloud)
		{
			for (unsigned i = 0; i < firstCloud->getNumberOfScalarFields(); i++)
			{
				firstCloud->getScalarField(i)->computeMinAndMax();
			}

			if (ocIndexSF)
			{
				firstCloud->showSF(true);
			}
		}

		//something to remove?
		for (ccHObject* toRemove : toBeRemoved)
		{
			if (firstCloud->isAncestorOf(toRemove))
			{
				// we cannot call 'removeElement' on a child of the first cloud, as it's temporarily detached from the DB tree!
				if (toRemove->getParent())
					toRemove->getParent()->removeChild(toRemove);
				else
					delete toRemove;
			}
			else
			{
				m_ccRoot->removeElement(toRemove);
			}
		}
		toBeRemoved.clear();

		//eventually we can put back the first cloud in DB
		if (firstCloud)
		{
			putObjectBackIntoDBTree(firstCloud, firstCloudContext);
			if (m_ccRoot)
				m_ccRoot->selectEntity(firstCloud);
		}
	}
	//merge meshes?
	else if (!meshes.empty())
	{
		bool createSubMeshes = true;
		//createSubMeshes = (QMessageBox::question(this, tr("Create sub-meshes"), tr("Do you want to create sub-mesh entities corresponding to each source mesh? (requires more memory)"), QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes);

		//meshes are merged
		ccPointCloud* baseVertices = new ccPointCloud("vertices");
		ccMesh* baseMesh = new ccMesh(baseVertices);
		baseMesh->setName("Merged mesh");
		baseMesh->addChild(baseVertices);
		baseVertices->setEnabled(false);

		for (ccMesh *mesh : meshes)
		{
			//if (mesh->isA(CC_TYPES::PRIMITIVE))
			//{
			//	mesh = mesh->ccMesh::cloneMesh(); //we want a clone of the mesh part, not the primitive!
			//}

			if (!baseMesh->merge(mesh, createSubMeshes))
			{
				ccConsole::Error(tr("Fusion failed! (not enough memory?)"));
				break;
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
	ccGLWindowInterface* win = static_cast<ccGLWindowInterface*>(object->getDisplay());
	if (win)
	{
		ccBBox box = object->getDisplayBB_recursive(false,win);
		win->updateConstellationCenterAndZoom(&box);
	}
}

void MainWindow::doActionRegister()
{
	if (	m_selectedEntities.size() != 2
		||	(!m_selectedEntities.front()->isKindOf(CC_TYPES::POINT_CLOUD) && !m_selectedEntities.front()->isKindOf(CC_TYPES::MESH))
		||	(!m_selectedEntities.back()->isKindOf(CC_TYPES::POINT_CLOUD) && !m_selectedEntities.back()->isKindOf(CC_TYPES::MESH)) )
	{
		ccConsole::Error(tr("Select 2 point clouds or meshes!"));
		return;
	}

	ccHObject* data = static_cast<ccHObject*>(m_selectedEntities.front());
	ccHObject* model = static_cast<ccHObject*>(m_selectedEntities.back());
	if (data->isKindOf(CC_TYPES::MESH) && model->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		//by default, prefer the mesh as the reference
		std::swap(data, model);
	}

	ccRegistrationDlg rDlg(data, model, this);
	if (!rDlg.exec())
		return;

	//model and data order may have changed!
	model = rDlg.getModelEntity();
	data = rDlg.getDataEntity();

	double minRMSDecrease = rDlg.getMinRMSDecrease();
	if (std::isnan(minRMSDecrease))
	{
		ccLog::Error(tr("Invalid minimum RMS decrease value"));
		return;
	}
	if (minRMSDecrease < ccRegistrationDlg::GetAbsoluteMinRMSDecrease())
	{
		minRMSDecrease = ccRegistrationDlg::GetAbsoluteMinRMSDecrease();
		ccLog::Error(tr("Minimum RMS decrease value is too small.\n%1 will be used instead (numerical accuracy limit).").arg(minRMSDecrease, 0, 'E', 1));
		rDlg.setMinRMSDecrease(minRMSDecrease);
	}

	CCCoreLib::ICPRegistrationTools::Parameters parameters;
	{
		parameters.convType					= rDlg.getConvergenceMethod();
		parameters.minRMSDecrease			= minRMSDecrease;
		parameters.nbMaxIterations			= rDlg.getMaxIterationCount();
		parameters.adjustScale				= rDlg.adjustScale();
		parameters.filterOutFarthestPoints	= rDlg.removeFarthestPoints();
		parameters.samplingLimit			= rDlg.randomSamplingLimit();
		parameters.finalOverlapRatio		= rDlg.getFinalOverlap() / 100.0;
		parameters.transformationFilters	= rDlg.getTransformationFilters();
		parameters.maxThreadCount			= rDlg.getMaxThreadCount();
		parameters.useC2MSignedDistances	= rDlg.useC2MSignedDistances(parameters.robustC2MSignedDistances);
		parameters.normalsMatching			= rDlg.normalsMatchingOption();
	}
	bool useDataSFAsWeights		= rDlg.useDataSFAsWeights();
	bool useModelSFAsWeights	= rDlg.useModelSFAsWeights();

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
									parameters,
									useDataSFAsWeights,
									useModelSFAsWeights,
									this))
	{
		QString rmsString = tr("Final RMS*: %1 (computed on %2 points)").arg(finalError).arg(finalPointCount);
		QString rmsDisclaimerString = tr("(* RMS is potentially weighted, depending on the selected options)");
		ccLog::Print(QString("[Register] ") + rmsString);
		ccLog::Print(QString("[Register] ") + rmsDisclaimerString);

		QStringList summary;
		summary << rmsString;
		summary << rmsDisclaimerString;
		summary << "----------------";

		//transformation matrix
		{
			summary << "Transformation matrix";
			summary << transMat.toString(3, '\t'); //low precision, just for display
			summary << "----------------";

			ccLog::Print(tr("[Register] Applied transformation matrix:"));
			ccLog::Print(transMat.toString(12, ' ')); //full precision
			ccLog::Print(tr("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool"));
		}

		if (parameters.adjustScale)
		{
			QString scaleString = tr("Scale: %1 (already integrated in above matrix!)").arg(finalScale);
			ccLog::Warning(QString("[Register] ") + scaleString);
			summary << scaleString;
		}
		else
		{
			ccLog::Print(tr("[Register] Scale: fixed (1.0)"));
			summary << tr("Scale: fixed (1.0)");
		}

		//overlap
		summary << "----------------";
		QString overlapString = tr("Theoretical overlap: %1%").arg(static_cast<int>(parameters.finalOverlapRatio * 100));
		ccLog::Print(QString("[Register] ") + overlapString);
		summary << overlapString;

		summary << "----------------";
		summary << tr("This report has been output to Console (F8)");

		//cloud to move
		ccGenericPointCloud* pc = nullptr;

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
				pc = nullptr;
				//we ask the user about cloning the 'data' mesh
				QMessageBox::StandardButton result = QMessageBox::question(	this,
																			tr("Registration"),
																			tr("Data mesh vertices are locked (they may be shared with other meshes): Do you wish to clone this mesh to apply transformation?"),
																			QMessageBox::Ok | QMessageBox::Cancel,
																			QMessageBox::Ok);

				//continue process?
				if (result == QMessageBox::Ok)
				{
					ccGenericMesh* newMesh = nullptr;
					if (mesh->isA(CC_TYPES::MESH))
						newMesh = static_cast<ccMesh*>(mesh)->cloneMesh();
					else
					{
						//FIXME TODO
						ccLog::Error(tr("Doesn't work on sub-meshes yet!"));
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
						ccLog::Error(tr("Failed to clone 'data' mesh! (not enough memory?)"));
					}
				}
			}
		}

		//if we managed to get a point cloud to move!
		if (pc)
		{
			//we temporarily detach cloud, as it may undergo
			//'severe' modifications (octree deletion, etc.) --> see ccPointCloud::applyRigidTransformation
			ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(pc);

			// check if the 'model' entity is a child of the 'data' one (would be strange, but can still happen!)
			bool modelIsChildOfData = data->isAncestorOf(model);
			if (modelIsChildOfData)
			{
				if (pc->getChildrenNumber() != 0)
				{
					ccLog::Warning(tr("[ICP] The reference entity is a child of the aligned one! CC will move only the aligned entity, and not its children"));
				}
				pc->applyRigidTransformation(transMat);
			}
			else
			{
				// we can safely apply the transformation to all the children
				pc->applyGLTransformation_recursive(&transMat);
			}
			putObjectBackIntoDBTree(pc, objContext);

			//don't forget to update mesh bounding box also!
			if (data->isKindOf(CC_TYPES::MESH))
			{
				ccHObjectCaster::ToGenericMesh(data)->refreshBB();
			}

			//don't forget global shift
			ccGenericPointCloud* refPC = ccHObjectCaster::ToGenericPointCloud(model);
			if (refPC)
			{
				if (refPC->isShifted())
				{
					const CCVector3d& Pshift = refPC->getGlobalShift();
					double scale = refPC->getGlobalScale();
					pc->setGlobalShift(Pshift);
					pc->setGlobalScale(scale);
					ccLog::Warning(tr("[ICP] Aligned entity global shift has been updated to match the reference: (%1,%2,%3) [x%4]").arg(Pshift.x).arg(Pshift.y).arg(Pshift.z).arg(scale));

					ccGLMatrixd transMatD(transMat.data());
					transMatD.scale(1.0 / scale);
					transMatD.setTranslation(transMatD.getTranslationAsVec3D() - Pshift);
					ccLog::Print("[ICP] Transformation to global coordinates:");
					ccLog::Print(transMatD.toString(12, ' ')); //full precision

				}
				else if (pc->isShifted()) //we'll ask the user first before dropping the shift information on the aligned cloud
				{
					if (QMessageBox::question(this, tr("Drop shift information?"), tr("Aligned entity is shifted but reference cloud is not: drop global shift information?"), QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes)
					{
						pc->setGlobalShift(0, 0, 0);
						pc->setGlobalScale(1.0);
						ccLog::Warning(tr("[ICP] Aligned entity global shift has been reset to match the reference!"));
					}
				}
			}

			data->prepareDisplayForRefresh_recursive();
			data->setName(data->getName() + QString(".registered"));
			zoomOn(data);
		}

		//pop-up summary
		QMessageBox::information(this, tr("Registration info"), summary.join("\n"));
		forceConsoleDisplay();
	}

	refreshAll();
	updateUI();
}

//Aurelien BEY le 13/11/2008 : ajout de la fonction permettant de traiter la fonctionnalite de recalage grossier
void MainWindow::doAction4pcsRegister()
{
	if (QMessageBox::warning(	this,
								tr("Work in progress"),
								tr("This method is still under development: are you sure you want to use it? (a crash may likely happen)"),
								QMessageBox::Yes,QMessageBox::No) == QMessageBox::No )
								return;

	if (m_selectedEntities.size() != 2)
	{
		ccConsole::Error(tr("Select 2 point clouds!"));
		return;
	}

	if (!m_selectedEntities.front()->isKindOf(CC_TYPES::POINT_CLOUD) ||
		!m_selectedEntities.back()->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error(tr("Select 2 point clouds!"));
		return;
	}

	ccGenericPointCloud* data = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities.back());
	ccGenericPointCloud* model = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities.front());

	ccAlignDlg aDlg(data, model);
	if (!aDlg.exec())
	{
		return;
	}

	// model = aDlg.getModelObject();
	data = aDlg.getDataObject();

	//Take the correct number of points among the clouds
	CCCoreLib::ReferenceCloud* subModel = aDlg.getSampledModel();
	CCCoreLib::ReferenceCloud* subData = aDlg.getSampledData();

	if (!subModel || !subData)
	{
		delete subModel;
		delete subData;
		assert(false);
		return;
	}

	unsigned nbMaxCandidates = aDlg.isNumberOfCandidatesLimited() ? aDlg.getMaxNumberOfCandidates() : 0;

	ccProgressDialog pDlg(true, this);

	CCCoreLib::PointProjectionTools::Transformation transform;
	if (CCCoreLib::FPCSRegistrationTools::RegisterClouds(	subModel,
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
			ccGLMatrix transMat = FromCCLibMatrix<double, float>(transform.R, transform.T);
			forceConsoleDisplay();
			ccConsole::Print(tr("[Align] Resulting matrix:"));
			ccConsole::Print(transMat.toString(12, ' ')); //full precision
			ccConsole::Print(tr("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool"));
		}

		ccPointCloud* newDataCloud = data->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(data)->cloneThis() : ccPointCloud::From(data, data);

		if (data->getParent())
		{
			data->getParent()->addChild(newDataCloud);
		}
		newDataCloud->setName(data->getName() + QString(".registered"));
		transform.apply(*newDataCloud);
		newDataCloud->invalidateBoundingBox(); //invalidate bb
		newDataCloud->setDisplay(data->getDisplay());
		newDataCloud->prepareDisplayForRefresh();
		zoomOn(newDataCloud);
		addToDB(newDataCloud);

		data->setEnabled(false);
		data->prepareDisplayForRefresh_recursive();
	}
	else
	{
		ccConsole::Warning(tr("[Align] Registration failed!"));
	}

	delete subModel;
	subModel = nullptr;
	delete subData;
	subData = nullptr;

	refreshAll();
	updateUI();
}

void MainWindow::doActionSubsample()
{
	//find candidates
	std::vector<ccPointCloud*> clouds;
	unsigned maxPointCount = 0;
	double maxCloudRadius = 0;
	ScalarType sfMin = CCCoreLib::NAN_VALUE;
	ScalarType sfMax = CCCoreLib::NAN_VALUE;
	{
		for ( ccHObject* entity : getSelectedEntities() )
		{
			if (entity->isA(CC_TYPES::POINT_CLOUD))
			{
				ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);
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
		ccConsole::Error(tr("Select at least one point cloud!"));
		return;
	}

	//Display dialog
	ccSubsamplingDlg sDlg(maxPointCount, maxCloudRadius, this);
	sDlg.loadFromPersistentSettings();

	bool hasValidSF = ccScalarField::ValidValue(sfMin) && ccScalarField::ValidValue(sfMax);
	if (hasValidSF)
	{
		sDlg.enableSFModulation(sfMin, sfMax);
	}
	if (!sDlg.exec())
	{
		return;
	}

	sDlg.saveToPersistentSettings();

	//process clouds
	ccHObject::Container resultingClouds;
	{
		ccProgressDialog pDlg(false, this);
		pDlg.setAutoClose(false);

		pDlg.setMethodTitle(tr("Subsampling"));

		bool errors = false;

		QElapsedTimer eTimer;
		eTimer.start();

		for (size_t i = 0; i < clouds.size(); ++i)
		{
			ccPointCloud* cloud = clouds[i];
			CCCoreLib::ReferenceCloud* sampledCloud = sDlg.getSampledCloud(cloud,&pDlg);
			if (!sampledCloud)
			{
				ccConsole::Warning(tr("[Subsampling] Failed to subsample cloud '%1'!").arg(cloud->getName()));
				errors = true;
				continue;
			}

			int warnings = 0;
			ccPointCloud* newPointCloud = cloud->partialClone(sampledCloud, &warnings);

			delete sampledCloud;
			sampledCloud = nullptr;

			if (newPointCloud)
			{
				newPointCloud->setName(cloud->getName() + QString(".subsampled"));
				newPointCloud->copyGlobalShiftAndScale(*cloud);
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
					ccLog::Warning(tr("[Subsampling] Not enough memory: colors, normals or scalar fields may be missing!"));
					errors = true;
				}
			}
			else
			{
				ccLog::Error(tr("Not enough memory!"));
				break;
			}
		}

		ccLog::Print("[Subsampling] Timing: %3.3f s.",eTimer.elapsed()/1000.0);

		if (errors)
		{
			ccLog::Error(tr("Errors occurred (see console)"));
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
										CCCoreLib::ReferenceCloudContainer& components,
										unsigned minPointsPerComponent,
										bool randomColors,
										bool selectComponents,
										bool sortBysize/*=true*/)
{
	if (!cloud || components.empty())
		return;

	std::vector<ComponentIndexAndSize> sortedIndexes;
	std::vector<ComponentIndexAndSize>* _sortedIndexes = nullptr;
	if (sortBysize)
	{
		try
		{
			sortedIndexes.reserve(components.size());
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Warning(tr("[CreateComponentsClouds] Not enough memory to sort components by size!"));
			sortBysize = false;
		}

		if (sortBysize) //still ok?
		{
			unsigned compCount = static_cast<unsigned>(components.size());
			for (unsigned i = 0; i < compCount; ++i)
			{
				sortedIndexes.emplace_back(i, components[i]->size());
			}

			ParallelSort(sortedIndexes.begin(), sortedIndexes.end(), ComponentIndexAndSize::DescendingCompOperator);
			
			_sortedIndexes = &sortedIndexes;
		}
	}

	//we create "real" point clouds for all input components
	{
		ccPointCloud* pc = cloud->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(cloud) : nullptr;

		//we create a new group to store all CCs
		ccHObject* ccGroup = new ccHObject(cloud->getName() + QString(" [CCs]"));

		//for each component
		for (size_t i = 0; i < components.size(); ++i)
		{
			CCCoreLib::ReferenceCloud* compIndexes = _sortedIndexes ? components[_sortedIndexes->at(i).index] : components[i];

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
						compCloud->setColor(col);
						compCloud->showColors(true);
						compCloud->showSF(false);
					}

					//'shift on load' information
					if (pc)
					{
						compCloud->copyGlobalShiftAndScale(*pc);
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
					ccConsole::Warning(tr("[CreateComponentsClouds] Failed to create component #%1! (not enough memory)").arg(ccGroup->getChildrenNumber() + 1));
				}
			}

			delete compIndexes;
			compIndexes = nullptr;
		}

		components.clear();

		if (ccGroup->getChildrenNumber() == 0)
		{
			ccConsole::Error(tr("No component was created! Check the minimum size..."));
			delete ccGroup;
			ccGroup = nullptr;
		}
		else
		{
			ccGroup->setDisplay(cloud->getDisplay());
			addToDB(ccGroup);

			ccConsole::Print(tr("[CreateComponentsClouds] %1 component(s) were created from cloud '%2'").arg(ccGroup->getChildrenNumber()).arg(cloud->getName()));
		}

		cloud->prepareDisplayForRefresh();

		//auto-hide original cloud
		if (ccGroup)
		{
			cloud->setEnabled(false);
			ccConsole::Warning(tr("[CreateComponentsClouds] Original cloud has been automatically hidden"));
		}
	}
}

void MainWindow::doActionLabelConnectedComponents()
{
	//keep only the point clouds!
	std::vector<ccGenericPointCloud*> clouds;
	{
		for ( ccHObject *entity : getSelectedEntities() )
		{
			if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
				clouds.push_back(ccHObjectCaster::ToGenericPointCloud(entity));
		}
	}

	size_t count = clouds.size();
	if (count == 0)
		return;

	static int s_octreeLevel = 8;
	static unsigned s_minComponentSize = 10;
	static bool s_randomColors = false;

	ccLabelingDlg dlg(this);
	if (count == 1)
		dlg.octreeLevelSpinBox->setCloud(clouds.front());

	dlg.setOctreeLevel(s_octreeLevel);
	dlg.setMinPointsNb(s_minComponentSize);
	dlg.setRandomColors(s_randomColors);

	if (!dlg.exec())
		return;

	s_octreeLevel = dlg.getOctreeLevel();
	s_minComponentSize = static_cast<unsigned>(std::max(0, dlg.getMinPointsNb()));
	s_randomColors = dlg.randomColors();

	ccProgressDialog pDlg(false, this);
	pDlg.setAutoClose(false);

	//we unselect all entities as we are going to automatically select the created components
	//(otherwise the user won't perceive the change!)
	if (m_ccRoot)
	{
		m_ccRoot->unselectAllEntities();
	}

	for ( ccGenericPointCloud *cloud : clouds )
	{
		if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD))
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

			ccOctree::Shared theOctree = cloud->getOctree();
			if (!theOctree)
			{
				ccProgressDialog pOctreeDlg(true, this);
				theOctree = cloud->computeOctree(&pOctreeDlg);
				if (!theOctree)
				{
					ccConsole::Error(tr("Couldn't compute octree for cloud '%1'!").arg(cloud->getName()));
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
				ccConsole::Error(tr("Couldn't allocate a new scalar field for computing CC labels! Try to free some memory ..."));
				break;
			}
			pc->setCurrentScalarField(sfIdx);

			//we try to label all CCs
			CCCoreLib::ReferenceCloudContainer components;
			int componentCount = CCCoreLib::AutoSegmentationTools::labelConnectedComponents(cloud,
																						static_cast<unsigned char>(s_octreeLevel),
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
						if (components[i]->size() >= s_minComponentSize)
						{
							++realComponentCount;
						}
					}
				}

				if (realComponentCount > 500)
				{
					//too many components
					if (QMessageBox::warning(this, tr("Many components"), tr("Do you really expect up to %1 components?\n(this may take a lot of time to process and display)").arg(realComponentCount), QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
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
				if (!CCCoreLib::AutoSegmentationTools::extractConnectedComponents(cloud, components))
				{
					ccConsole::Warning(tr("[DoActionLabelConnectedComponents] Something went wrong while extracting CCs from cloud %1...").arg(cloud->getName()));
				}
			}
			else
			{
				ccConsole::Warning(tr("[DoActionLabelConnectedComponents] Something went wrong while extracting CCs from cloud %1...").arg(cloud->getName()));
			}

			//we delete the CCs label scalar field (we don't need it anymore)
			pc->deleteScalarField(sfIdx);
			sfIdx = -1;

			//we create "real" point clouds for all CCs
			if (!components.empty())
			{
				createComponentsClouds(cloud, components, s_minComponentSize, s_randomColors, true);
			}
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionSetSFAsCoord()
{
	if (!ccEntityAction::sfSetAsCoord(m_selectedEntities, this))
	{
		return;
	}

	zoomOnSelectedEntities();

	updateUI();
}

void MainWindow::doActionExportCoordToSF()
{
	if (!ccEntityAction::exportCoordToSF(m_selectedEntities, this))
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionExportNormalToSF()
{
	if (!ccEntityAction::exportNormalToSF(m_selectedEntities, this))
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionSetSFsAsNormal()
{
	if (!haveOneSelection())
	{
		if (haveSelection())
			ccConsole::Error(tr("Select only one cloud or one mesh!"));
		return;
	}

	ccHObject* ent = m_selectedEntities.front();

	if (!ccEntityAction::setSFsAsNormal(ent, this))
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::doMeshTwoPolylines()
{
	if (m_selectedEntities.size() != 2)
		return;

	ccPolyline* p1 = ccHObjectCaster::ToPolyline(m_selectedEntities.front());
	ccPolyline* p2 = ccHObjectCaster::ToPolyline(m_selectedEntities.back());
	if (!p1 || !p2)
	{
		ccConsole::Error(tr("Select 2 and only 2 polylines"));
		return;
	}

	//Ask the user how the 2D projection should be computed
	bool useViewingDir = false;
	CCVector3 viewingDir(0, 0, 0);
	if (p1->getDisplay())
	{
		useViewingDir = (QMessageBox::question(this, tr("Projection method"), tr("Use best fit plane (yes) or the current viewing direction (no)"), QMessageBox::Yes, QMessageBox::No) == QMessageBox::No);
		if (useViewingDir)
		{
			viewingDir = -p1->getDisplay()->getViewportParameters().getViewDir().toPC();
		}
	}

	ccMesh* mesh = ccMesh::TriangulateTwoPolylines(p1, p2, useViewingDir ? &viewingDir : nullptr);
	if (mesh)
	{
		addToDB(mesh);
		if (mesh->computePerVertexNormals())
		{
			mesh->showNormals(true);
		}
		else
		{
			ccLog::Warning(tr("[Mesh two polylines] Failed to compute normals!"));
		}

		if (mesh->getDisplay())
		{
			mesh->getDisplay()->redraw();
		}
	}
	else
	{
		ccLog::Error(tr("Failed to create mesh (see Console)"));
		forceConsoleDisplay();
	}
}

void MainWindow::doConvertPolylinesToMesh()
{
	if (!haveSelection())
		return;

	std::vector<ccPolyline*> polylines;
	try
	{
		if (haveOneSelection() && m_selectedEntities.back()->isA(CC_TYPES::HIERARCHY_OBJECT))
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
			for ( ccHObject *entity : getSelectedEntities() )
			{
				if (entity->isA(CC_TYPES::POLY_LINE))
				{
					polylines.push_back(static_cast<ccPolyline*>(entity));
				}
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccConsole::Error(tr("Not enough memory!"));
		return;
	}

	if (polylines.empty())
	{
		ccConsole::Error(tr("Select a group of polylines or multiple polylines (contour plot)!"));
		return;
	}

	ccPickOneElementDlg poeDlg(tr("Projection dimension"), tr("Contour plot to mesh"), this);
	poeDlg.addElement("X");
	poeDlg.addElement("Y");
	poeDlg.addElement("Z");
	poeDlg.setDefaultIndex(2);
	if (!poeDlg.exec())
		return;

	int dim = poeDlg.getSelectedIndex();
	assert(dim >= 0 && dim < 3);

	const unsigned char Z = static_cast<unsigned char>(dim);
	const unsigned char X = (Z == 2 ? 0 : Z + 1);
	const unsigned char Y = (X == 2 ? 0 : X + 1);

	//number of segments
	unsigned segmentCount = 0;
	unsigned vertexCount = 0;
	{
		for ( ccPolyline *poly : polylines )
		{
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
		ccLog::Error(tr("Not enough segments!"));
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
		ccLog::Error(tr("Not enough memory!"));
		return;
	}

	//fill arrays
	{
		for ( ccPolyline *poly : polylines )
		{
			if (poly == nullptr)
				continue;
			
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
		assert(points2D.size() == vertexCount);
		assert(segments2D.size() == segmentCount * 2);
	}

	CCCoreLib::Delaunay2dMesh* delaunayMesh = new CCCoreLib::Delaunay2dMesh;
	std::string errorStr;
	if (!delaunayMesh->buildMesh(points2D, segments2D, errorStr))
	{
		ccLog::Error( tr("Third party library error: %1").arg( QString::fromStdString( errorStr ) ) );
		delete delaunayMesh;
		return;
	}

	ccPointCloud* vertices = new ccPointCloud("vertices");
	if (!vertices->reserve(vertexCount))
	{
		//not enough memory
		ccLog::Error(tr("Not enough memory!"));
		delete vertices;
		delete delaunayMesh;
		return;
	}

	//fill vertices cloud
	{
		for ( ccPolyline *poly : polylines )
		{
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
			const CCCoreLib::VerticesIndexes* tsi = delaunayMesh->getTriangleVertIndexes(i);
			assert(tsi->i1 < vertCount && tsi->i2 < vertCount && tsi->i3 < vertCount);
		}
	}
#endif

	ccMesh* mesh = new ccMesh(delaunayMesh, vertices);
	if (mesh->size() != delaunayMesh->size())
	{
		//not enough memory (error will be issued later)
		delete mesh;
		mesh = nullptr;
	}

	//don't need this anymore
	delete delaunayMesh;
	delaunayMesh = nullptr;

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
			ccLog::Warning(tr("[Contour plot to mesh] Failed to compute normals!"));
		}

		if (mesh->getDisplay())
		{
			mesh->getDisplay()->redraw();
		}

		//global shift & scale (we copy it from the first polyline by default)
		mesh->copyGlobalShiftAndScale(*polylines.front());
	}
	else
	{
		ccLog::Error(tr("Not enough memory!"));
		delete vertices;
		vertices = nullptr;
	}
}

void MainWindow::doCompute2HalfDimVolume()
{
	if (m_selectedEntities.empty() || m_selectedEntities.size() > 2)
	{
		ccConsole::Error(tr("Select one or two point clouds!"));
		return;
	}

	ccGenericPointCloud* cloud1 = nullptr;
	{
		ccHObject* ent = m_selectedEntities.front();
		if (!ent->isKindOf(CC_TYPES::POINT_CLOUD) )
		{
			ccConsole::Error(tr("Select point clouds only!"));
			return;
		}
		else
		{
			cloud1 = ccHObjectCaster::ToGenericPointCloud(ent);
		}
	}

	ccGenericPointCloud* cloud2 = nullptr;
	if (m_selectedEntities.size() > 1)
	{
		ccHObject* ent = m_selectedEntities[1];
		if (!ent->isKindOf(CC_TYPES::POINT_CLOUD) )
		{
			ccConsole::Error(tr("Select point clouds only!"));
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
	if (!haveOneSelection())
	{
		ccConsole::Error(tr("Select only one point cloud!"));
		return;
	}

	ccHObject* ent = m_selectedEntities.front();
	if (!ent->isKindOf(CC_TYPES::POINT_CLOUD) )
	{
		ccConsole::Error(tr("Select a point cloud!"));
		return;
	}

	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);
	ccRasterizeTool rasterizeTool(cloud, this);
	rasterizeTool.exec();
}

void MainWindow::doActionDeleteScanGrids()
{
	//look for clouds with scan grids
	for ( ccHObject *entity : getSelectedEntities() )
	{
		if (!entity || !entity->isA(CC_TYPES::POINT_CLOUD))
		{
			continue;
		}

		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		assert(cloud);

		if(cloud->gridCount() > 0)
		{
			cloud->removeGrids();
		}
	}
	refreshAll();
	updateUI();
}


void MainWindow::doActionMeshScanGrids()
{
	//ask the user for the min angle (inside triangles)
	static double s_meshMinTriangleAngle_deg = 1.0;
	{
		bool ok = true;
		double minAngle_deg = QInputDialog::getDouble(this, tr("Triangulate"), tr("Min triangle angle (in degrees)"), s_meshMinTriangleAngle_deg, 0, 90.0, 3, &ok);
		if (!ok)
			return;
		s_meshMinTriangleAngle_deg = minAngle_deg;
	}

	//look for clouds with scan grids
	for ( ccHObject *entity : getSelectedEntities() )
	{
		if (!entity || !entity->isA(CC_TYPES::POINT_CLOUD))
		{
			continue;
		}

		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
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
	doActionComputeMesh(CCCoreLib::DELAUNAY_2D_AXIS_ALIGNED);
}

void MainWindow::doActionComputeMeshLS()
{
	doActionComputeMesh(CCCoreLib::DELAUNAY_2D_BEST_LS_PLANE);
}

void MainWindow::doActionComputeMesh(CCCoreLib::TRIANGULATION_TYPES type)
{
	//ask the user for the max edge length
	static double s_meshMaxEdgeLength = 0.0;
	{
		bool ok = true;
		double maxEdgeLength = QInputDialog::getDouble(this, tr("Triangulate"), tr("Max edge length (0 = no limit)"), s_meshMaxEdgeLength, 0, 1.0e9, 8, &ok);
		if (!ok)
			return;
		s_meshMaxEdgeLength = maxEdgeLength;
	}

	//select candidates
	ccHObject::Container clouds;
	bool hadNormals = false;
	{
		for ( ccHObject *entity : getSelectedEntities() )
		{
			if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
			{
				clouds.push_back(entity);
				if (entity->isA(CC_TYPES::POINT_CLOUD))
				{
					hadNormals |= static_cast<ccPointCloud*>(entity)->hasNormals();
				}
			}
		}
	}

	//if the cloud(s) already had normals, ask the use if wants to update them or keep them as is (can look strange!)
	bool updateNormals = false;
	if (hadNormals)
	{
		updateNormals = (QMessageBox::question(	this,
												tr("Keep old normals?"),
												tr("Cloud(s) already have normals. Do you want to update them (yes) or keep the old ones (no)?"),
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
		ccConsole::Error(tr("Error(s) occurred! See the Console messages"));
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionFitQuadric()
{
	bool errors = false;
	
	//for all selected entities
	for ( ccHObject *entity : getSelectedEntities() )
	{
		//look for clouds
		if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);

			double rms = 0.0;
			ccQuadric* quadric = ccQuadric::Fit(cloud, &rms);
			if (quadric)
			{
				cloud->addChild(quadric);
				quadric->setName(QString("Quadric (%1)").arg(cloud->getName()));
				quadric->setDisplay(cloud->getDisplay());
				quadric->prepareDisplayForRefresh();
				quadric->copyGlobalShiftAndScale(*cloud);
				addToDB(quadric);

				ccConsole::Print(tr("[DoActionFitQuadric] Quadric local coordinate system:"));
				ccConsole::Print(quadric->getTransformation().toString(12,' ')); //full precision
				ccConsole::Print(tr("[DoActionFitQuadric] Quadric equation (in local coordinate system): ") + quadric->getEquationString());
				ccConsole::Print(QString("[DoActionFitQuadric] RMS: %1").arg(rms));

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
				ccConsole::Warning(tr("Failed to compute quadric on cloud '%1'").arg(cloud->getName()));
				errors = true;
			}
		}
	}

	if (errors)
	{
		ccConsole::Error(tr("Error(s) occurred: see console"));
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

	for ( ccHObject *entity : getSelectedEntities() )
	{
		if (!entity->isKindOf(CC_TYPES::MESH) && !entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			//non handled entity type
			continue;
		}

		//CCCoreLib::ChamferDistanceTransform cdt;
		CCCoreLib::SaitoSquaredDistanceTransform cdt;
		if (!cdt.initGrid(Tuple3ui(steps, steps, steps)))
		{
			//not enough memory
			ccLog::Error(tr("Not enough memory!"));
			return;
		}

		ccBBox box = entity->getOwnBB();
		PointCoordinateType largestDim = box.getMaxBoxDim() + static_cast<PointCoordinateType>(margin);
		CCVector3 minCorner = box.getCenter() - CCVector3(1, 1, 1) * (largestDim / 2);
		PointCoordinateType cellDim = largestDim / steps + std::numeric_limits<PointCoordinateType>::epsilon(); //to avoid rounding issues when projecting triangles or points inside the grid

		bool result = false;
		if (entity->isKindOf(CC_TYPES::MESH))
		{
			ccMesh* mesh = static_cast<ccMesh*>(entity);
			result = cdt.initDT(mesh, cellDim, minCorner, &pDlg);
		}
		else
		{
			ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(entity);
			result = cdt.initDT(cloud, cellDim, minCorner, &pDlg);
		}

		if (!result)
		{
			ccLog::Error(tr("Not enough memory!"));
			return;
		}

		//cdt.propagateDistance(CHAMFER_345, &pDlg);
		cdt.propagateDistance(&pDlg);

		//convert the grid to a cloud
		ccPointCloud* gridCloud = new ccPointCloud(entity->getName() + QString(".distance_grid(%1)").arg(steps));
		{
			unsigned pointCount = steps*steps*steps;
			if (!gridCloud->reserve(pointCount))
			{
				ccLog::Error(tr("Not enough memory!"));
				delete gridCloud;
				return;
			}

			ccScalarField* sf = new ccScalarField("DT values");
			if (!sf->reserveSafe(pointCount))
			{
				ccLog::Error(tr("Not enough memory!"));
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
						ScalarType d = std::sqrt(static_cast<ScalarType>(cdt.getValue(i, j, k))) * cellDim;

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
				ccLog::Warning(tr("[DistanceMap] Cloud '%1': no point falls inside the specified range").arg(entity->getName()));
				delete gridCloud;
				gridCloud = nullptr;
			}
			else
			{
				gridCloud->setCurrentDisplayedScalarField(sfIdx);
				gridCloud->showSF(true);
				gridCloud->setDisplay(entity->getDisplay());
				gridCloud->shrinkToFit();
				entity->prepareDisplayForRefresh();
				addToDB(gridCloud);
			}
		}
	}

	refreshAll();
}

void MainWindow::doActionComputeDistToBestFitQuadric3D()
{
	bool ok = true;
	int steps = QInputDialog::getInt(this, tr("Distance to best fit quadric (3D)"), tr("Steps (per dim.)"), 50, 10, 10000, 10, &ok);
	if (!ok)
		return;

	for ( ccHObject *entity : getSelectedEntities() )
	{
		if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);
			CCCoreLib::Neighbourhood Yk(cloud);

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
					ccConsole::Warning(tr("Failed to get the center of gravity of cloud '%1'!").arg(cloud->getName()));
					continue;
				}

				const ccBBox bbox = cloud->getOwnBB();
				PointCoordinateType maxDim = bbox.getMaxBoxDim();
				CCVector3 C = bbox.getCenter();

				//Sample points on a cube and compute for each of them the distance to the Quadric
				ccPointCloud* newCloud = new ccPointCloud();
				if (!newCloud->reserve(steps*steps*steps))
				{
					ccConsole::Error(tr("Not enough memory!"));
				}

				const char defaultSFName[] = "Dist. to 3D quadric";
				int sfIdx = newCloud->getScalarFieldIndexByName(defaultSFName);
				if (sfIdx < 0)
					sfIdx = newCloud->addScalarField(defaultSFName);
				if (sfIdx < 0)
				{
					ccConsole::Error(tr("Couldn't allocate a new scalar field for computing distances! Try to free some memory ..."));
					delete newCloud;
					continue;
				}

				ccScalarField* sf = static_cast<ccScalarField*>(newCloud->getScalarField(sfIdx));
				assert(sf);

				for (int x = 0; x < steps; ++x)
				{
					CCVector3 P;
					P.x = C.x + maxDim * (static_cast<PointCoordinateType>(x) / static_cast<PointCoordinateType>(steps - 1) - CCCoreLib::PC_ONE / 2);
					for (int y = 0; y < steps; ++y)
					{
						P.y = C.y + maxDim * (static_cast<PointCoordinateType>(y) / static_cast<PointCoordinateType>(steps - 1) - CCCoreLib::PC_ONE / 2);
						for (int z = 0; z < steps; ++z)
						{
							P.z = C.z + maxDim * (static_cast<PointCoordinateType>(z) / static_cast<PointCoordinateType>(steps - 1) - CCCoreLib::PC_ONE / 2);
							newCloud->addPoint(P);

							//compute distance to quadric
							CCVector3 Pc = P-*G;
							ScalarType dist = static_cast<ScalarType>(	a*Pc.x*Pc.x + b*Pc.y*Pc.y + c*Pc.z*Pc.z
																	+	e*Pc.x*Pc.y + f*Pc.y*Pc.z + g*Pc.x*Pc.z
																	+	l*Pc.x + m*Pc.y + n*Pc.z + d);

							sf->addElement(dist);
						}
					}
				}

				if (sf)
				{
					sf->computeMinAndMax();
					newCloud->setCurrentDisplayedScalarField(sfIdx);
					newCloud->showSF(true);
				}
				newCloud->setName(tr("Distance map to 3D quadric"));
				newCloud->setDisplay(cloud->getDisplay());
				newCloud->prepareDisplayForRefresh();

				addToDB(newCloud);
			}
			else
			{
				ccConsole::Warning(tr("Failed to compute 3D quadric on cloud '%1'").arg(cloud->getName()));
			}
		}
	}

	refreshAll();
}

void MainWindow::doActionComputeCPS()
{
	if (m_selectedEntities.size() != 2)
	{
		ccConsole::Error(tr("Select 2 point clouds!"));
		return;
	}

	if (!m_selectedEntities.front()->isKindOf(CC_TYPES::POINT_CLOUD) ||
		!m_selectedEntities.back()->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error(tr("Select 2 point clouds!"));
		return;
	}

	ccOrderChoiceDlg dlg(	m_selectedEntities.front(), tr("Compared"),
							m_selectedEntities.back(), tr("Reference"),
							this );
	if (!dlg.exec())
		return;

	ccGenericPointCloud* compCloud = ccHObjectCaster::ToGenericPointCloud(dlg.getFirstEntity());
	ccGenericPointCloud* srcCloud = ccHObjectCaster::ToGenericPointCloud(dlg.getSecondEntity());

	if (!compCloud->isA(CC_TYPES::POINT_CLOUD)) //TODO
	{
		ccConsole::Error(tr("Compared cloud must be a real point cloud!"));
		return;
	}
	ccPointCloud* cmpPC = static_cast<ccPointCloud*>(compCloud);

	static const char DEFAULT_CPS_TEMP_SF_NAME[] = "CPS temporary";
	int sfIdx = cmpPC->getScalarFieldIndexByName(DEFAULT_CPS_TEMP_SF_NAME);
	if (sfIdx < 0)
		sfIdx = cmpPC->addScalarField(DEFAULT_CPS_TEMP_SF_NAME);
	if (sfIdx < 0)
	{
		ccConsole::Error(tr("Couldn't allocate a new scalar field for computing distances! Try to free some memory ..."));
		return;
	}
	cmpPC->setCurrentScalarField(sfIdx);
	if (!cmpPC->enableScalarField())
	{
		ccConsole::Error(tr("Not enough memory!"));
		return;
	}
	//cmpPC->forEach(CCCoreLib::ScalarFieldTools::SetScalarValueToNaN); //now done by default by computeCloud2CloudDistances

	CCCoreLib::ReferenceCloud CPSet(srcCloud);
	ccProgressDialog pDlg(true, this);
	CCCoreLib::DistanceComputationTools::Cloud2CloudDistancesComputationParams params;
	params.CPSet = &CPSet;
	int result = CCCoreLib::DistanceComputationTools::computeCloud2CloudDistances(compCloud, srcCloud, params, &pDlg);
	cmpPC->deleteScalarField(sfIdx);

	if (result >= 0)
	{
		ccPointCloud* newCloud = nullptr;
		//if the source cloud is a "true" cloud, the extracted CPS
		//will also get its attributes
		newCloud = srcCloud->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(srcCloud)->partialClone(&CPSet) : ccPointCloud::From(&CPSet, srcCloud);

		newCloud->setName(QString("[%1]->CPSet(%2)").arg(srcCloud->getName(), compCloud->getName()));
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

void MainWindow::doActionShiftPointsAlongNormals()
{
	std::vector<ccPointCloud*> candidates;
	for (ccHObject* entity : m_selectedEntities)
	{
		if (entity && entity->isA(CC_TYPES::POINT_CLOUD))
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(entity);
			if (pc->hasNormals())
			{
				candidates.push_back(pc);
			}
		}
	}

	if (candidates.empty())
	{
		ccConsole::Error(QObject::tr("Select at least one point cloud with normals"));
		return;
	}

	// ask the user to define the shift value
	static double s_shift = 0.0;
	bool ok = false;
	double shift = QInputDialog::getDouble(this, tr("Shift along normals"), tr("Shift quantity"), s_shift, -1.0e6, 1.0e6, 6, &ok);
	if (!ok)
	{
		return;
	}
	// remember for next time
	s_shift = shift;

	for (ccPointCloud* pc : candidates)
	{
		pc->shiftPointsAlongNormals(static_cast<PointCoordinateType>(shift));
		pc->prepareDisplayForRefresh();
	}

	refreshAll();
}

void MainWindow::doActionFindBiggestInnerRectangle()
{
	if (!haveSelection())
		return;

	ccHObject* entity = haveOneSelection() ? m_selectedEntities.front() : nullptr;
	if (!entity || !entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error(tr("Select one point cloud!"));
		return;
	}

	bool ok;
	static int s_innerRectDim = 2;
	int dim = QInputDialog::getInt(this, tr("Dimension"), tr("Orthogonal dim (X=0 / Y=1 / Z=2)"), s_innerRectDim, 0, 2, 1, &ok);
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

void MainWindow::doActionFastRegistration(FastRegistrationMode mode)
{
	//we need at least 1 entity
	if (m_selectedEntities.empty())
		return;

	//we must backup 'm_selectedEntities' as removeObjectTemporarilyFromDBTree can modify it!
	ccHObject::Container selectedEntities = m_selectedEntities;

	for (ccHObject *entity : selectedEntities)
	{
		ccBBox box = entity->getBB_recursive();

		CCVector3 T; //translation

		switch (mode)
		{
		case MoveBBCenterToOrigin:
			T = -box.getCenter();
			break;
		case MoveBBMinCornerToOrigin:
			T = -box.minCorner();
			break;
		case MoveBBMaxCornerToOrigin:
			T = -box.maxCorner();
			break;
		default:
			assert(false);
			return;
		}

		//transformation (used only for translation)
		ccGLMatrix glTrans;
		glTrans.setTranslation(T);

		forceConsoleDisplay();
		ccConsole::Print(tr("[Synchronize] Transformation matrix (%1):").arg(entity->getName()));
		ccConsole::Print(glTrans.toString(12, ' ')); //full precision
		ccConsole::Print(tr("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool"));

		//we temporarily detach the entity, as it may undergo
		//'severe' modifications (octree deletion, etc.) --> see ccHObject::applyGLTransformation
		ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(entity);
		entity->applyGLTransformation_recursive(&glTrans);
		putObjectBackIntoDBTree(entity, objContext);

		entity->prepareDisplayForRefresh_recursive();
	}

	//reselect previously selected entities!
	if (m_ccRoot)
		m_ccRoot->selectEntities(selectedEntities);

	zoomOnSelectedEntities();

	updateUI();
}

void MainWindow::doActionMatchBBCenters()
{
	//we need at least 2 entities
	if (m_selectedEntities.size() < 2)
		return;

	//we must backup 'm_selectedEntities' as removeObjectTemporarilyFromDBTree can modify it!
	ccHObject::Container selectedEntities = m_selectedEntities;

	//by default, we take the first entity as reference
	//TODO: maybe the user would like to select the reference himself ;)
	ccHObject* refEnt = selectedEntities[0];
	CCVector3 refCenter = refEnt->getBB_recursive().getCenter();

	for (ccHObject *entity : selectedEntities) //warning, getSelectedEntites may change during this loop!
	{
		CCVector3 center = entity->getBB_recursive().getCenter();

		CCVector3 T = refCenter-center;

		//transformation (used only for translation)
		ccGLMatrix glTrans;
		glTrans += T;

		forceConsoleDisplay();
		ccConsole::Print(tr("[Synchronize] Transformation matrix (%1 --> %2):").arg(entity->getName(),selectedEntities[0]->getName()));
		ccConsole::Print(glTrans.toString(12,' ')); //full precision
		ccConsole::Print(tr("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool"));

		//we temporarily detach the entity, as it may undergo
		//'severe' modifications (octree deletion, etc.) --> see ccHObject::applyGLTransformation
		ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(entity);
		entity->applyGLTransformation_recursive(&glTrans);
		putObjectBackIntoDBTree(entity, objContext);

		entity->prepareDisplayForRefresh_recursive();
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
	//we need at least 2 entities
	if (m_selectedEntities.size() < 2)
		return;

	//we must select the point clouds and meshes
	ccHObject::Container selectedEntities;
	try
	{
		for ( ccHObject *entity : getSelectedEntities() )
		{
			if (	entity->isKindOf(CC_TYPES::POINT_CLOUD)
				||	entity->isKindOf(CC_TYPES::MESH))
			{
				selectedEntities.push_back(entity);
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccConsole::Error(tr("Not enough memory!"));
		return;
	}

	ccMatchScalesDlg msDlg(selectedEntities, 0, this);
	msDlg.setSelectedAlgorithm(s_msAlgorithm);
	msDlg.rmsDifferenceLineEdit->setText(QString::number(s_msRmsDiff, 'e', 1));
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

void MainWindow::doActionSORFilter()
{
	ccSORFilterDlg sorDlg(this);

	//set semi-persistent/dynamic parameters
	static int s_sorFilterKnn = 6;
	static double s_sorFilterNSigma = 1.0;
	sorDlg.setKNN(s_sorFilterKnn);
	sorDlg.setNSigma(s_sorFilterNSigma);
	if (!sorDlg.exec())
	{
		return;
	}

	//update semi-persistent/dynamic parameters
	s_sorFilterKnn = sorDlg.KNN();
	s_sorFilterNSigma = sorDlg.nSigma();

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	bool firstCloud = true;

	ccHObject::Container selectedEntities = getSelectedEntities(); //we have to use a local copy: 'selectEntity' will change the set of currently selected entities!

	for (ccHObject* entity : selectedEntities)
	{
		//specific test for locked vertices
		bool lockedVertices;
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity, &lockedVertices);
		if (!cloud)
		{
			continue;
		}
		if (lockedVertices)
		{
			ccUtils::DisplayLockedVerticesWarning(entity->getName(), haveOneSelection());
			continue;
		}

		//computation
		CCCoreLib::ReferenceCloud* selection = CCCoreLib::CloudSamplingTools::sorFilter(cloud,
																						s_sorFilterKnn,
																						s_sorFilterNSigma,
																						cloud->getOctree().data(),
																						&pDlg);

		if (selection)
		{
			if (selection->size() == cloud->size())
			{
				ccLog::Warning(tr("[DoActionSORFilter] No points were removed from cloud '%1'").arg(cloud->getName()));
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
						ccConsole::Warning(tr("Previously selected entities (sources) have been hidden!"));
						firstCloud = false;
						m_ccRoot->selectEntity(cleanCloud, true);
					}
				}
				else
				{
					ccConsole::Warning(tr("[DoActionSORFilter] Not enough memory to create a clean version of cloud '%1'!").arg(cloud->getName()));
				}
			}

			delete selection;
			selection = nullptr;
		}
		else
		{
			//no points fall inside selection!
			ccConsole::Warning(tr("[DoActionSORFilter] Failed to apply the noise filter to cloud '%1'! (not enough memory?)").arg(cloud->getName()));
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionFilterNoise()
{
	double kernelRadius = ccLibAlgorithms::GetDefaultCloudKernelSize(m_selectedEntities);

	ccNoiseFilterDlg noiseDlg(this);

	//set semi-persistent/dynamic parameters
	static bool s_noiseFilterUseKnn = false;
	static int s_noiseFilterKnn = 6;
	static bool s_noiseFilterUseAbsError = false;
	static double s_noiseFilterAbsError = 1.0;
	static double s_noiseFilterNSigma = 1.0;
	static bool s_noiseFilterRemoveIsolatedPoints = false;
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
	kernelRadius = noiseDlg.radiusDoubleSpinBox->value();
	s_noiseFilterUseKnn = noiseDlg.knnRadioButton->isChecked();
	s_noiseFilterKnn = noiseDlg.knnSpinBox->value();
	s_noiseFilterUseAbsError = noiseDlg.absErrorRadioButton->isChecked();
	s_noiseFilterNSigma = noiseDlg.nSigmaDoubleSpinBox->value();
	s_noiseFilterAbsError = noiseDlg.absErrorDoubleSpinBox->value();
	s_noiseFilterRemoveIsolatedPoints = noiseDlg.removeIsolatedPointsCheckBox->isChecked();

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	bool firstCloud = true;
	
	ccHObject::Container selectedEntities = getSelectedEntities(); //we have to use a local copy: and 'selectEntity' will change the set of currently selected entities!
	
	for ( ccHObject *entity : selectedEntities )
	{
		//specific test for locked vertices
		bool lockedVertices;
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity, &lockedVertices);
		if (!cloud)
		{
			continue;
		}
		if (lockedVertices)
		{
			ccUtils::DisplayLockedVerticesWarning(entity->getName(), haveOneSelection());
			continue;
		}

		//computation
		CCCoreLib::ReferenceCloud* selection = CCCoreLib::CloudSamplingTools::noiseFilter(	cloud,
																							static_cast<PointCoordinateType>(kernelRadius),
																							s_noiseFilterNSigma,
																							s_noiseFilterRemoveIsolatedPoints,
																							s_noiseFilterUseKnn,
																							s_noiseFilterKnn,
																							s_noiseFilterUseAbsError,
																							s_noiseFilterAbsError,
																							cloud->getOctree().data(),
																							&pDlg);

		if (selection)
		{
			if (selection->size() == cloud->size())
			{
				ccLog::Warning(tr("[DoActionFilterNoise] No points were removed from cloud '%1'").arg(cloud->getName()));
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
						ccConsole::Warning(tr("Previously selected entities (sources) have been hidden!"));
						firstCloud = false;
						m_ccRoot->selectEntity(cleanCloud, true);
					}
				}
				else
				{
					ccConsole::Warning(tr("[DoActionFilterNoise] Not enough memory to create a clean version of cloud '%1'!").arg(cloud->getName()));
				}
			}

			delete selection;
			selection = nullptr;
		}
		else
		{
			//no points fall inside selection!
			ccConsole::Warning(tr("[DoActionFilterNoise] Failed to apply the noise filter to cloud '%1'! (not enough memory?)").arg(cloud->getName()));
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionUnroll()
{
	//there should be only one point cloud or one mesh!
	if (!haveOneSelection())
	{
		ccConsole::Error(tr("Select one and only one entity!"));
		return;
	}

	//if selected entity is a mesh, the method will be applied to its vertices
	bool lockedVertices;
	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities.front(), &lockedVertices);
	if (lockedVertices)
	{
		ccUtils::DisplayLockedVerticesWarning(m_selectedEntities.front()->getName(), true);
		return;
	}

	//for "real" point clouds only
	if (!cloud || !cloud->isA(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error(tr("Method can't be applied on locked vertices or virtual point clouds!"));
		return;
	}
	ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

	ccUnrollDlg unrollDlg(m_ccRoot ? m_ccRoot->getRootEntity() : nullptr, this);
	unrollDlg.fromPersistentSettings();
	if (!unrollDlg.exec())
		return;
	unrollDlg.toPersistentSettings();

	ccPointCloud::UnrollMode mode = unrollDlg.getType();
	PointCoordinateType radius = static_cast<PointCoordinateType>(unrollDlg.getRadius());
	CCVector3d axisDir = unrollDlg.getAxis();
	bool exportDeviationSF = unrollDlg.exportDeviationSF();
	bool arbitraryOutputCS = unrollDlg.useArbitraryOutputCS();
	CCVector3 center = unrollDlg.getAxisPosition();

	//let's rock unroll ;)
	ccProgressDialog pDlg(true, this);

	double startAngle_deg = 0.0;
	double stopAngle_deg = 360.0;
	unrollDlg.getAngleRange(startAngle_deg, stopAngle_deg);
	if (startAngle_deg >= stopAngle_deg)
	{
		QMessageBox::critical(this, tr("Error"), tr("Invalid angular range"));
		return;
	}

	ccPointCloud* output = nullptr;
	switch (mode)
	{
	case ccPointCloud::CYLINDER:
	{
		ccPointCloud::UnrollCylinderParams params;
		params.radius = radius;
		params.axisDir = CCVector3::fromArray(axisDir.u);
		if (unrollDlg.isAxisPositionAuto())
		{
			center = pc->getOwnBB().getCenter();
		}
		params.center = center;
		output = pc->unroll(mode, &params, exportDeviationSF, startAngle_deg, stopAngle_deg, arbitraryOutputCS, &pDlg);
	}
	break;

	case ccPointCloud::CONE_CONICAL:
	case ccPointCloud::CONE_CYLINDRICAL_FIXED_RADIUS:
	case ccPointCloud::CONE_CYLINDRICAL_ADAPTIVE_RADIUS:
	{
		ccPointCloud::UnrollConeParams params;
		params.radius = (mode == ccPointCloud::CONE_CYLINDRICAL_FIXED_RADIUS ? radius : 0);
		params.apex = center;
		params.coneAngle_deg = unrollDlg.getConeHalfAngle();
		params.axisDir = CCVector3::fromArray(axisDir.u);
		params.spanRatio = unrollDlg.getConicalProjSpanRatio();
		output = pc->unroll(mode, &params, exportDeviationSF, startAngle_deg, stopAngle_deg, arbitraryOutputCS, &pDlg);
	}
	break;
	
	default:
		assert(false);
		break;
	}

	if (output)
	{
		if (m_selectedEntities.front()->isA(CC_TYPES::MESH))
		{
			ccMesh* mesh = ccHObjectCaster::ToMesh(m_selectedEntities.front());
			mesh->setEnabled(false);
			ccConsole::Warning(tr("[Unroll] Original mesh has been automatically hidden"));
			ccMesh* outputMesh = mesh->cloneMesh(output);
			outputMesh->addChild(output);
			addToDB(outputMesh, true, true, false, true);
			outputMesh->setEnabled(true);
			outputMesh->setVisible(true);
		}
		else
		{
			pc->setEnabled(false);
			ccConsole::Warning(tr("[Unroll] Original cloud has been automatically hidden"));
			if (pc->getParent())
			{
				pc->getParent()->addChild(output);
			}
			addToDB(output, true, true, false, true);
		}
		updateUI();
	}
}

ccGLWindowInterface* MainWindow::getActiveGLWindow()
{
	if (!m_mdiArea)
	{
		return nullptr;
	}

	QMdiSubWindow *activeSubWindow = m_mdiArea->activeSubWindow();
	if (activeSubWindow)
	{
		return ccGLWindowInterface::FromWidget(activeSubWindow->widget());
	}
	else
	{
		QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
		if (!subWindowList.isEmpty())
		{
			return ccGLWindowInterface::FromWidget(subWindowList[0]->widget());
		}
	}

	return nullptr;
}

QMdiSubWindow* MainWindow::getMDISubWindow(ccGLWindowInterface* win)
{
	QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
	for (int i = 0; i < subWindowList.size(); ++i)
	{
		if (ccGLWindowInterface::FromWidget(subWindowList[i]->widget()) == win)
			return subWindowList[i];
	}

	//not found!
	return nullptr;
}

ccGLWindowInterface* MainWindow::getGLWindow(int index) const
{
	QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();	
	if (index >= 0 && index < subWindowList.size())
	{
		ccGLWindowInterface* win = ccGLWindowInterface::FromWidget(subWindowList[index]->widget());
		assert(win);
		return win;
	}
	else
	{
		assert(false);
		return nullptr;
	}
}

int MainWindow::getGLWindowCount() const
{
	return m_mdiArea ? m_mdiArea->subWindowList().size() : 0;
}

void MainWindow::zoomIn()
{
	ccGLWindowInterface* win = MainWindow::getActiveGLWindow();
	if (win)
	{
		//we simulate a real wheel event
		win->onWheelEvent(15.0f);
	}
}

void MainWindow::zoomOut()
{
	ccGLWindowInterface* win = MainWindow::getActiveGLWindow();
	if (win)
	{
		//we simulate a real wheel event
		win->onWheelEvent(-15.0f);
	}
}

ccGLWindowInterface* MainWindow::new3DViewInternal( bool allowEntitySelection )
{
	assert(m_ccRoot && m_mdiArea);

	QWidget* viewWidget = nullptr;
	ccGLWindowInterface* view3D = nullptr;
	
	createGLWindow(view3D, viewWidget);
	if (!viewWidget || !view3D)
	{
		ccLog::Error(tr("Failed to create the 3D view"));
		assert(false);
		return nullptr;
	}

	//restore options
	{
		QSettings settings;
		bool autoPickRotationCenter = settings.value(ccPS::AutoPickRotationCenter(), true).toBool();
		view3D->setAutoPickPivotAtCenter(autoPickRotationCenter);
	}

	viewWidget->setMinimumSize(400, 300);

	m_mdiArea->addSubWindow(viewWidget);

	if ( allowEntitySelection )
	{
		connect(view3D->signalEmitter(), &ccGLWindowSignalEmitter::entitySelectionChanged, this, [=] (ccHObject *entity) {
			m_ccRoot->selectEntity( entity );
		});
		
		connect(view3D->signalEmitter(), &ccGLWindowSignalEmitter::entitiesSelectionChanged, this, [=] (std::unordered_set<int> entities){
			m_ccRoot->selectEntities( entities );
		});
	}

	//'echo' mode
	connect(view3D->signalEmitter(),	&ccGLWindowSignalEmitter::mouseWheelRotated,			this, &MainWindow::echoMouseWheelRotate);
	connect(view3D->signalEmitter(),	&ccGLWindowSignalEmitter::viewMatRotated,				this, &MainWindow::echoBaseViewMatRotation);
	connect(view3D->signalEmitter(),	&ccGLWindowSignalEmitter::cameraPosChanged,				this, &MainWindow::echoCameraPosChanged);
	connect(view3D->signalEmitter(),	&ccGLWindowSignalEmitter::pivotPointChanged,			this, &MainWindow::echoPivotPointChanged);

	connect(view3D->signalEmitter(),	&ccGLWindowSignalEmitter::aboutToClose,					this, &MainWindow::prepareWindowDeletion);
	connect(view3D->signalEmitter(),	&ccGLWindowSignalEmitter::filesDropped,					this, &MainWindow::addToDBAuto, Qt::QueuedConnection); //DGM: we don't want to block the 'dropEvent' method of ccGLWindow instances!
	connect(view3D->signalEmitter(),	&ccGLWindowSignalEmitter::newLabel,						this, &MainWindow::handleNewLabel);
	connect(view3D->signalEmitter(),	&ccGLWindowSignalEmitter::exclusiveFullScreenToggled,	this, &MainWindow::onExclusiveFullScreenToggled);

	if (m_pickingHub)
	{
		//we must notify the picking hub as well if the window is destroyed
		connect(view3D->signalEmitter(), &ccGLWindowSignalEmitter::aboutToClose, m_pickingHub, &ccPickingHub::onActiveWindowDeleted);
	}

	view3D->setSceneDB(m_ccRoot->getRootEntity());
	viewWidget->setAttribute(Qt::WA_DeleteOnClose);
	m_ccRoot->updatePropertiesView();

	QMainWindow::statusBar()->showMessage(tr("New 3D View"), 2000);

	viewWidget->showMaximized();
	viewWidget->update();

	return view3D;
}

void MainWindow::prepareWindowDeletion(ccGLWindowInterface* glWindow)
{
	if (!m_ccRoot)
		return;

	if (glWindow)
	{
		m_ccRoot->hidePropertiesView();
		m_ccRoot->getRootEntity()->removeFromDisplay_recursive(glWindow);
		m_ccRoot->updatePropertiesView();
	}
	else
	{
		assert(false);
	}
}

static bool s_autoSaveGuiElementPos = true;
void MainWindow::doActionResetGUIElementsPos()
{
	//show the user it will be maximized
	showMaximized();

	QSettings settings;
	settings.remove(ccPS::MainWinGeom());
	settings.remove(ccPS::MainWinState());

	QMessageBox::information( this,
							  tr("Restart"),
							  tr("To finish the process, you'll have to close and restart CloudCompare") );
	
	//to avoid saving them right away!
	s_autoSaveGuiElementPos = false;
}

void MainWindow::doActionToggleRestoreWindowOnStartup(bool state)
{
	QSettings settings;
	settings.setValue(ccPS::DoNotRestoreWindowGeometry(), !state);
}

void MainWindow::doActionResetAllVBOs()
{
	ccHObject::Container clouds;
	m_ccRoot->getRootEntity()->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, true);

	size_t releasedSize = 0;
	for (ccHObject* entity : clouds)
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (cloud)
		{
			releasedSize += cloud->vboSize();
			cloud->releaseVBOs();
		}
	}

	if (releasedSize != 0)
	{
		ccLog::Print(tr("All VBOs have been released (%1 Mb)").arg(releasedSize / static_cast<double>(1 << 20), 0, 'f', 2));
		if (ccGui::Parameters().useVBOs)
		{
			ccLog::Warning(tr("You might want to disable the 'use VBOs' option in the Display Settings to keep the GPU memory empty"));
		}
	}
	else
	{
		ccLog::Print(tr("No VBO allocated"));
	}
}

void MainWindow::restoreGUIElementsPos()
{
	QSettings settings;

	QVariant previousState = settings.value(ccPS::MainWinState());
	if (previousState.isValid())
	{
		restoreState(previousState.toByteArray());
	}

	QVariant previousGeometry;
	if (!settings.value(ccPS::DoNotRestoreWindowGeometry(), false).toBool())
	{
		previousGeometry = settings.value(ccPS::MainWinGeom());
	}

	if (previousGeometry.isValid())
	{
		restoreGeometry(previousGeometry.toByteArray()); 
	}
	else
	{
		showMaximized();
	}

	if (isFullScreen())
	{
		m_UI->actionFullScreen->blockSignals(true);
		m_UI->actionFullScreen->setChecked(true);
		m_UI->actionFullScreen->blockSignals(false);
	}
}

void MainWindow::showEvent(QShowEvent* event)
{
	if (m_firstShow)
	{
		restoreGUIElementsPos();

		m_firstShow = false;
	}

	QMainWindow::showEvent(event);
}

void MainWindow::closeEvent(QCloseEvent* event)
{
	const ccOptions &opts =  ccOptions::Instance();
	// If we don't have anything displayed, then just close...
	if (!opts.confirmQuit || (m_ccRoot && (m_ccRoot->getRootEntity()->getChildrenNumber() == 0)))
	{
		event->accept();
	}
	else // ...otherwise confirm
	{
		QMessageBox message_box( QMessageBox::Question,
								 tr("Quit"),
								 tr("Are you sure you want to quit?"),
								 QMessageBox::NoButton,
								 this);
		message_box.addButton(QMessageBox::Button::Yes);
		message_box.addButton(tr("Yes, don't ask again"), QMessageBox::ButtonRole::YesRole);
		message_box.addButton(QMessageBox::Button::No);

		message_box.exec();

		switch (message_box.buttons().indexOf(message_box.clickedButton())) {
			case 1: // Yes, don't ask again
			{
				ccOptions optsCopied = opts;
				optsCopied.confirmQuit = false;
				optsCopied.toPersistentSettings();
				Q_FALLTHROUGH(); // Fallthrough to the yes case, to accept the event
			}
			case 0: // Yes
				event->accept();
				break;
			case 2: // No
			default:
				event->ignore();
		}
	}

	if (s_autoSaveGuiElementPos)
	{
		saveGUIElementsPos();
	}
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
	connect(dlg, &ccOverlayDialog::shown, this, [=]()
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
	});

	repositionOverlayDialog(m_mdiDialogs.back());
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

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
	switch (event->type())
	{
	case QEvent::Resize:
	case QEvent::Move:
		updateOverlayDialogsPlacement();
		break;
	default:
		//nothing to do
		break;
	}

	// standard event processing
	return QObject::eventFilter(obj, event);
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
	switch (event->key())
	{
		case Qt::Key_Escape:
		{
			if ( s_pickingWindow != nullptr )
			{
				cancelPreviousPickingOperation( true );
			}
			break;
		}
			
		default:
			QMainWindow::keyPressEvent(event);
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

	int dx = 0;
	int dy = 0;
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
	ccGLWindowInterface* win = getActiveGLWindow();
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
		m_UI->actionFullScreen->setText( tr( "Exit Full Screen" ) );
	}
	else
	{
		m_UI->actionFullScreen->setText( tr( "Enter Full Screen" ) );
	}
#endif
}

void MainWindow::toggleExclusiveFullScreen(bool state)
{
	ccGLWindowInterface* win = getActiveGLWindow();
	if (win)
	{
		win->toggleExclusiveFullScreen(state);
	}
}

void MainWindow::doActionShowHelpDialog()
{
	QMessageBox messageBox;
	messageBox.setTextFormat(Qt::RichText);
	messageBox.setWindowTitle("Documentation");
	messageBox.setText("Please look at the <a href='http://www.cloudcompare.org/doc/wiki'>wiki</a>");
	messageBox.setStandardButtons(QMessageBox::Ok);
	messageBox.exec();
}

void MainWindow::freezeUI(bool state)
{
	//freeze standard plugins
	m_UI->toolBarMainTools->setDisabled(state);
	m_UI->toolBarSFTools->setDisabled(state);
	
	m_pluginUIManager->mainPluginToolbar()->setDisabled(state);

	//freeze plugin toolbars
	for ( QToolBar *toolbar : m_pluginUIManager->additionalPluginToolbars() )
	{
		toolbar->setDisabled(state);
	}

	m_UI->DockableDBTree->setDisabled(state);
	m_UI->menubar->setDisabled(state);

	if (state)
	{
		m_UI->menuEdit->setDisabled(true);
		m_UI->menuTools->setDisabled(true);
	}
	else
	{
		updateMenus();
	}

	m_uiFrozen = state;
}

void MainWindow::activateRegisterPointPairTool()
{
	if (!haveSelection())
	{
		ccConsole::Error(tr("Select at least one entity (point cloud or mesh)!"));
		return;
	}

	ccHObject::Container alignedEntities;
	ccHObject::Container refEntities;
	try
	{
		ccHObject::Container entities;
		entities.reserve(m_selectedEntities.size());

		for (ccHObject* entity : m_selectedEntities)
		{
			//for now, we only handle clouds or meshes
			if (entity->isKindOf(CC_TYPES::POINT_CLOUD) || entity->isKindOf(CC_TYPES::MESH))
			{
				entities.push_back(entity);
			}
		}

		if (entities.empty())
		{
			ccConsole::Error("Select at least one entity (point cloud or mesh)!");
			return;
		}
		else if (entities.size() == 1)
		{
			alignedEntities = entities;
		}
		else
		{
			std::vector<int> indexes;
			if (!ccEntitySelectionDialog::SelectEntities(entities, indexes, this, tr("Select to-be-aligned entities")))
			{
				//process cancelled by the user
				return;
			}

			//add the selected indexes as 'aligned' entities
			alignedEntities.reserve(indexes.size());
			for (size_t i = 0; i < indexes.size(); ++i)
			{
				alignedEntities.push_back(entities[indexes[i]]);
			}

			//add the others as 'reference' entities
			assert(indexes.size() <= entities.size());
			refEntities.reserve(entities.size() - indexes.size());
			for (size_t i = 0; i < entities.size(); ++i)
			{
				if (std::find(indexes.begin(), indexes.end(), i) == indexes.end())
				{
					refEntities.push_back(entities[i]);
				}
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error(tr("Not enough memory"));
		return;
	}

	if (alignedEntities.empty())
	{
		ccLog::Error(tr("No to-be-aligned entity selected"));
		return;
	}

	//deselect all entities
	if (m_ccRoot)
	{
		m_ccRoot->unselectAllEntities();
	}

	ccGLWindowInterface* win = new3DView();
	if (!win)
	{
		ccLog::Error(tr("[PointPairRegistration] Failed to create dedicated 3D view!"));
		return;
	}

	//we disable all windows
	disableAllBut(win);

	if (!m_pprDlg)
	{
		m_pprDlg = new ccPointPairRegistrationDlg(m_pickingHub, this, this);
		connect(m_pprDlg, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateRegisterPointPairTool);
		registerOverlayDialog(m_pprDlg, Qt::TopRightCorner);
	}

	if (!m_pprDlg->init(win, alignedEntities, &refEntities))
	{
		deactivateRegisterPointPairTool(false);
	}

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
	if (!haveSelection())
		return;

	if (!m_seTool)
	{
		m_seTool = new ccSectionExtractionTool(this);
		connect(m_seTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateSectionExtractionMode);

		registerOverlayDialog(m_seTool, Qt::TopRightCorner);
	}

	//add clouds
	ccGLWindowInterface* firstDisplay = nullptr;
	{
		unsigned validCount = 0;
		for (ccHObject *entity : getSelectedEntities())
		{
			if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
			{
				if (m_seTool->addCloud(static_cast<ccGenericPointCloud*>(entity)))
				{
					if (!firstDisplay && entity->getDisplay())
					{
						firstDisplay = static_cast<ccGLWindowInterface*>(entity->getDisplay());
					}
					
					++validCount;
				}
			}
		}

		if (validCount == 0)
		{
			ccConsole::Error(tr("No cloud in selection!"));
			return;
		}
	}

	//deselect all entities
	if (m_ccRoot)
	{
		m_ccRoot->unselectAllEntities();
	}

	ccGLWindowInterface* win = new3DViewInternal(false);
	if (!win)
	{
		ccLog::Error(tr("[SectionExtraction] Failed to create dedicated 3D view!"));
		return;
	}

	if (firstDisplay && firstDisplay->getGlFilter())
	{
		win->setGlFilter(firstDisplay->getGlFilter()->clone());
	}
	m_seTool->linkWith(win);

	freezeUI(true);
	m_UI->toolBarView->setDisabled(true);

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
	m_UI->toolBarView->setDisabled(false);

	updateUI();

	ccGLWindowInterface* win = getActiveGLWindow();
	if (win)
		win->redraw();
}

void MainWindow::activateSegmentationMode()
{
	ccGLWindowInterface* win = getActiveGLWindow();
	if (!win)
		return;

	if (!haveSelection())
		return;

	if (!m_gsTool)
	{
		m_gsTool = new ccGraphicalSegmentationTool(this);
		connect(m_gsTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateSegmentationMode);

		registerOverlayDialog(m_gsTool, Qt::TopRightCorner);
	}

	m_gsTool->linkWith(win);

	for ( ccHObject *entity : getSelectedEntities() )
	{
		m_gsTool->addEntity(entity);
	}

	if (m_gsTool->getNumberOfValidEntities() == 0)
	{
		ccConsole::Error(tr("No segmentable entity in active window!"));
		return;
	}

	freezeUI(true);
	m_UI->toolBarView->setDisabled(false);

	//we disable all other windows
	disableAllBut(win);

	if (!m_gsTool->start())
		deactivateSegmentationMode(false);
	else
		updateOverlayDialogsPlacement();
}

void MainWindow::deactivateSegmentationMode(bool state)
{
	if (!m_gsTool)
	{
		assert(false);
		return;
	}

	bool deleteHiddenParts = false;

	//shall we apply segmentation?
	if (state)
	{
		ccHObject::Container result;

		m_gsTool->applySegmentation(this, result);

		if (m_ccRoot)
		{
			m_ccRoot->selectEntities(result);
		}
	}
	else
	{
		m_gsTool->removeAllEntities();
	}

	//we enable all GL windows
	enableAll();

	freezeUI(false);

	updateUI();

	ccGLWindowInterface* win = getActiveGLWindow();
	if (win)
	{
		win->redraw();
	}
}

void MainWindow::activateTracePolylineMode()
{
	ccGLWindowInterface* win = getActiveGLWindow();
	if (!win)
	{
		return;
	}

	if (!m_tplTool)
	{
		m_tplTool = new ccTracePolylineTool(m_pickingHub, this);
		connect(m_tplTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateTracePolylineMode);
		registerOverlayDialog(m_tplTool, Qt::TopRightCorner);
	}

	m_tplTool->linkWith(win);

	freezeUI(true);
	m_UI->toolBarView->setDisabled(false);

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

	ccGLWindowInterface* win = getActiveGLWindow();
	if (win)
	{
		win->redraw();
	}
}

void MainWindow::activatePointListPickingMode()
{
	ccGLWindowInterface* win = getActiveGLWindow();
	if (!win)
		return;

	//there should be only one point cloud in current selection!
	if (!haveOneSelection())
	{
		ccConsole::Error(tr("Select one and only one entity!"));
		return;
	}

	ccHObject* entity = m_selectedEntities.front();
	if (!entity->isKindOf(CC_TYPES::POINT_CLOUD) && !entity->isKindOf(CC_TYPES::MESH))
	{
		ccConsole::Error(tr("Select a cloud or a mesh"));
		return;
	}

	if (!entity->isVisible() || !entity->isEnabled())
	{
		ccConsole::Error(tr("Entity must be visible!"));
		return;
	}

	if (!m_plpDlg)
	{
		m_plpDlg = new ccPointListPickingDlg(m_pickingHub, this);
		connect(m_plpDlg, &ccOverlayDialog::processFinished, this, &MainWindow::deactivatePointListPickingMode);

		registerOverlayDialog(m_plpDlg, Qt::TopRightCorner);
	}

	//DGM: we must update marker size spin box value (as it may have changed by the user with the "display dialog")
	m_plpDlg->markerSizeSpinBox->setValue(win->getDisplayParameters().labelMarkerSize);

	m_plpDlg->linkWith(win);
	m_plpDlg->linkWithEntity(entity);

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
		m_plpDlg->linkWithEntity(nullptr);
	}

	//we enable all GL windows
	enableAll();

	freezeUI(false);

	updateUI();
}

void MainWindow::activatePointPickingMode()
{
	ccGLWindowInterface* win = getActiveGLWindow();
	if (!win)
	{
		return;
	}

	if (m_ccRoot)
	{
		m_ccRoot->unselectAllEntities(); //we don't want any entity selected (especially existing labels!)
	}

	if (!m_ppDlg)
	{
		m_ppDlg = new ccPointPropertiesDlg(m_pickingHub, this);
		connect(m_ppDlg, &ccOverlayDialog::processFinished,	this, &MainWindow::deactivatePointPickingMode);
		connect(m_ppDlg, &ccPointPropertiesDlg::newLabel,	this, &MainWindow::handleNewLabel);

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
	//	m_ppDlg->linkWith(nullptr);

	//we enable all GL windows
	enableAll();

	freezeUI(false);

	updateUI();
	redrawAll();
}

void MainWindow::activateClippingBoxMode()
{
	if ( !haveSelection() )
	{
		return;
	}

	ccGLWindowInterface* win = getActiveGLWindow();
	if (!win)
	{
		return;
	}

	if (!m_clipTool)
	{
		m_clipTool = new ccClippingBoxTool(this);
		connect(m_clipTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateClippingBoxMode);
	}
	m_clipTool->linkWith(win);

	ccHObject::Container selectedEntities = getSelectedEntities(); //we have to use a local copy: 'unselectEntity' will change the set of currently selected entities!
	for (ccHObject *entity : selectedEntities)
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
		ccConsole::Error(tr("Unexpected error!")); //indeed...
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
	if (!haveSelection())
		return;

	ccGLWindowInterface* win = getActiveGLWindow();
	if (!win)
		return;

	if (!m_transTool)
		m_transTool = new ccGraphicalTransformationTool(this);
	assert(m_transTool->getNumberOfValidEntities() == 0);
	m_transTool->linkWith(win);

	bool rejectedEntities = false;
	for ( ccHObject *entity : getSelectedEntities() )
	{
		if (!m_transTool->addEntity(entity))
			rejectedEntities = true;
	}

	if (m_transTool->getNumberOfValidEntities() == 0)
	{
		ccConsole::Error(tr("No entity eligible for manual transformation! (see console)"));
		return;
	}
	else if (rejectedEntities)
	{
		ccConsole::Error(tr("Some entities were ignored! (see console)"));
	}

	//try to activate "moving mode" in current GL window
	if (m_transTool->start())
	{
		connect(m_transTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateTranslateRotateMode);
		registerOverlayDialog(m_transTool, Qt::TopRightCorner);
		freezeUI(true);
		updateOverlayDialogsPlacement();
		//deactivate all other GL windows
		disableAllBut(win);
	}
	else
	{
		ccConsole::Error(tr("Unexpected error!")); //indeed...
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
				for (unsigned i = 0; i < transformedSet.getChildrenNumber(); ++i)
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
	ccGLWindowInterface* win = getActiveGLWindow();
	if (win)
		win->startFrameRateTest();
}

void MainWindow::showDisplaySettings()
{
	ccDisplaySettingsDlg displayOptionsDlg(this);
	connect(&displayOptionsDlg, &ccDisplaySettingsDlg::aspectHasChanged, this, [=] () { redrawAll(); });
			
	displayOptionsDlg.exec();

	disconnect(&displayOptionsDlg);
}

void MainWindow::doActionRenderToFile()
{
	ccGLWindowInterface* win = getActiveGLWindow();
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

		connect(m_mdiArea, &QMdiArea::subWindowActivated,
				m_cpeDlg, qOverload<QMdiSubWindow*>(&ccCameraParamEditDlg::linkWith));

		registerOverlayDialog(m_cpeDlg, Qt::BottomLeftCorner);
	}

	m_cpeDlg->linkWith(qWin);
	m_cpeDlg->start();

	updateOverlayDialogsPlacement();
}

void MainWindow::doActionAdjustZoom()
{
	//current active MDI area
	ccGLWindowInterface* win = getActiveGLWindow();
	if (!win)
		return;

	const ccViewportParameters& params = win->getViewportParameters();
	if (params.perspectiveView)
	{
		ccConsole::Error(tr("Orthographic mode only!"));
		return;
	}

	ccAdjustZoomDlg azDlg(win, this);

	if (!azDlg.exec())
		return;

	//apply new focal
	double focalDist = azDlg.getFocalDistance();
	win->setFocalDistance(focalDist);
	win->redraw();
}

static unsigned s_viewportIndex = 0;
void MainWindow::doActionSaveViewportAsCamera()
{
	ccGLWindowInterface* win = getActiveGLWindow();
	if (!win)
		return;

	cc2DViewportObject* viewportObject = new cc2DViewportObject(QString("Viewport #%1").arg(++s_viewportIndex));
	viewportObject->setParameters(win->getViewportParameters());
	viewportObject->setDisplay(win);

	// Save the custom light position as well
	{
		bool customLightEnabled = win->customLightEnabled();
		CCVector3f customLightPos = win->getCustomLightPosition();

		viewportObject->setMetaData("CustomLightEnabled", customLightEnabled);
		viewportObject->setMetaData("CustomLightPosX", customLightPos.x);
		viewportObject->setMetaData("CustomLightPosY", customLightPos.y);
		viewportObject->setMetaData("CustomLightPosZ", customLightPos.z);
	}

	addToDB(viewportObject);
}

void MainWindow::zoomOnSelectedEntities()
{
	ccGLWindowInterface* win = nullptr;

	ccHObject tempGroup("TempGroup");
	size_t selNum = m_selectedEntities.size();
	for (size_t i = 0; i < selNum; ++i)
	{
		ccHObject *entity = m_selectedEntities[i];
		
		if (i == 0 || !win)
		{
			//take the first valid window as reference
			win = static_cast<ccGLWindowInterface*>(entity->getDisplay());
		}

		if (win)
		{
			if (entity->getDisplay() == win)
			{
				tempGroup.addChild(entity,ccHObject::DP_NONE);
			}
			else if (entity->getDisplay() != nullptr)
			{
				ccLog::Error(tr("All selected entities must be displayed in the same 3D view!"));
				return;
			}
		}
	}

	if (tempGroup.getChildrenNumber() != 0)
	{
		ccBBox box = tempGroup.getDisplayBB_recursive(false, win);
		if (!box.isValid())
		{
			ccLog::Warning(tr("Selected entities have no valid bounding-box!"));
		}
		else
		{
			if ( win != nullptr )
			{
				win->updateConstellationCenterAndZoom(&box);
			}
		}
	}

	refreshAll();
}

void MainWindow::setGlobalZoom()
{
	ccGLWindowInterface* win = getActiveGLWindow();
	if (win)
		win->zoomGlobal();
}

void MainWindow::setPivotAlwaysOn()
{
	ccGLWindowInterface* win = getActiveGLWindow();
	if (win)
	{
		win->setPivotVisibility(ccGLWindowInterface::PIVOT_ALWAYS_SHOW);
		win->redraw();

		//update pop-up menu 'top' icon
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setIcon(m_UI->actionSetPivotAlwaysOn->icon());
	}
}

void MainWindow::setPivotRotationOnly()
{
	ccGLWindowInterface* win = getActiveGLWindow();
	if (win)
	{
		win->setPivotVisibility(ccGLWindowInterface::PIVOT_SHOW_ON_MOVE);
		win->redraw();

		//update pop-up menu 'top' icon
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setIcon(m_UI->actionSetPivotRotationOnly->icon());
	}
}

void MainWindow::setPivotOff()
{
	ccGLWindowInterface* win = getActiveGLWindow();
	if (win)
	{
		win->setPivotVisibility(ccGLWindowInterface::PIVOT_HIDE);
		win->redraw();

		//update pop-up menu 'top' icon
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setIcon(m_UI->actionSetPivotOff->icon());
	}
}

void MainWindow::setOrthoView(ccGLWindowInterface* win)
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
			m_viewModePopupButton->setIcon(m_UI->actionSetOrthoView->icon());
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setEnabled(true);
	}
}

void MainWindow::setCenteredPerspectiveView(ccGLWindowInterface* win, bool autoRedraw/*=true*/)
{
	if (win)
	{
		win->setPerspectiveState(true, true);
		if (autoRedraw)
			win->redraw();

		//update pop-up menu 'top' icon
		if (m_viewModePopupButton)
			m_viewModePopupButton->setIcon(m_UI->actionSetCenteredPerspectiveView->icon());
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setEnabled(true);
	}
}

void MainWindow::setViewerPerspectiveView(ccGLWindowInterface* win)
{
	if (win)
	{
		win->setPerspectiveState(true,false);
		win->redraw();

		//update pop-up menu 'top' icon
		if (m_viewModePopupButton)
			m_viewModePopupButton->setIcon(m_UI->actionSetViewerPerspectiveView->icon());
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setEnabled(false);
	}
}

void MainWindow::enablePickingOperation(ccGLWindowInterface* win, QString message)
{
	if (!win)
	{
		assert(false);
		return;
	}

	assert(m_pickingHub);
	if (!m_pickingHub->addListener(this))
	{
		ccLog::Error(tr("Can't start the picking mechanism (another tool is already using it)"));
		return;
	}

	//specific case: we prevent the 'point-pair based alignment' tool to process the picked point!
	//if (m_pprDlg)
	//	m_pprDlg->pause(true);

	s_pickingWindow = win;
	win->displayNewMessage(message, ccGLWindowInterface::LOWER_LEFT_MESSAGE, true, 24 * 3600);
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
			s_levelMarkersCloud = nullptr;
		}
		break;
	default:
		assert(false);
		break;
	}

	if (aborted)
	{
		s_pickingWindow->displayNewMessage(QString(), ccGLWindowInterface::LOWER_LEFT_MESSAGE); //clear previous messages
		s_pickingWindow->displayNewMessage(tr("Picking operation aborted"), ccGLWindowInterface::LOWER_LEFT_MESSAGE);
	}
	s_pickingWindow->redraw(false);

	//specific case: we allow the 'point-pair based alignment' tool to process the picked point!
	if (m_pprDlg)
		m_pprDlg->pause(false);

	freezeUI(false);

	m_pickingHub->removeListener(this);

	s_pickingWindow = nullptr;
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
		ccLog::Warning(tr("The point was picked in the wrong window"));
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
			//	ccLog::Warning(tr("[Level] Only points picked on '%1' are considered!").arg(s_levelEntity->getName()));
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
					ccLog::Warning(tr("[Level] Point is too close from the others!"));
					return;
				}
			}

			//add the corresponding marker
			s_levelMarkersCloud->addPoint(pickedPoint);
			unsigned markerCount = s_levelMarkersCloud->size();
			cc2DLabel* label = new cc2DLabel();
			label->addPickedPoint(s_levelMarkersCloud, markerCount - 1);
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
				CCVector3d viewDir = s_pickingWindow->getViewportParameters().getViewDir();
				if (Z.toDouble().dot(viewDir) > 0)
				{
					Z = -Z;
				}
				Y = Z.cross(X);
				X.normalize();
				Y.normalize();
				Z.normalize();

				ccGLMatrixd trans;
				{
					double* mat = trans.data();
					mat[0] = X.x; mat[4] = X.y; mat[8]  = X.z; mat[12] = 0.0;
					mat[1] = Y.x; mat[5] = Y.y; mat[9]  = Y.z; mat[13] = 0.0;
					mat[2] = Z.x; mat[6] = Z.y; mat[10] = Z.z; mat[14] = 0.0;
					mat[3] = 0.0; mat[7] = 0.0; mat[11] = 0.0; mat[15] = 1.0;
				}

				CCVector3d T = -A->toDouble();
				trans.apply(T);
				T += *A;
				trans.setTranslation(T);

				assert(haveOneSelection() && m_selectedEntities.front() == s_levelEntity);
				applyTransformation(trans, false);

				//clear message
				s_pickingWindow->displayNewMessage(QString(), ccGLWindowInterface::LOWER_LEFT_MESSAGE, false); //clear previous message
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
			CCVector3d newPivot = pickedPoint;
			//specific case: transformation tool is enabled
			if (m_transTool && m_transTool->started())
			{
				m_transTool->setRotationCenter(newPivot);
				const unsigned& precision = s_pickingWindow->getDisplayParameters().displayedNumPrecision;
				s_pickingWindow->displayNewMessage(QString(), ccGLWindowInterface::LOWER_LEFT_MESSAGE, false); //clear previous message
				s_pickingWindow->displayNewMessage(QString("Point (%1 ; %2 ; %3) set as rotation center for interactive transformation")
					.arg(pickedPoint.x, 0, 'f', precision)
					.arg(pickedPoint.y, 0, 'f', precision)
					.arg(pickedPoint.z, 0, 'f', precision),
					ccGLWindowInterface::LOWER_LEFT_MESSAGE, true);
			}
			else
			{
				const ccViewportParameters& params = s_pickingWindow->getViewportParameters();
				if (!params.perspectiveView || params.objectCenteredView)
				{
					//apply current GL transformation (if any)
					pi.entity->getGLTransformation().apply(newPivot);
					s_pickingWindow->setPivotPoint(newPivot, true, true);
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
			ccConsole::Error(tr("Stop the other picking operation first!"));
		}
		return;
	}

	ccGLWindowInterface* win = getActiveGLWindow();
	if (!win)
	{
		ccConsole::Error(tr("No active 3D view!"));
		return;
	}

	if (!haveOneSelection())
	{
		ccConsole::Error(tr("Select an entity!"));
		return;
	}

	//create markers cloud
	assert(!s_levelMarkersCloud);
	{
		s_levelMarkersCloud = new ccPointCloud("Level points");
		if (!s_levelMarkersCloud->reserve(3))
		{
			ccConsole::Error(tr("Not enough memory!"));
			return;
		}
		win->addToOwnDB(s_levelMarkersCloud);
	}

	s_levelEntity = m_selectedEntities.front();
	s_levelLabels.clear();
	s_currentPickingOperation = PICKING_LEVEL_POINTS;

	enablePickingOperation(win, tr("Pick three points on the floor plane (click the Level button or press Escape to cancel)"));
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
			ccConsole::Error(tr("Stop the other picking operation first!"));
		}
		return;
	}

	ccGLWindowInterface* win = getActiveGLWindow();
	if (!win)
	{
		ccConsole::Error(tr("No active 3D view!"));
		return;
	}

	bool objectCentered = true;
	bool perspectiveEnabled = win->getPerspectiveState(objectCentered);
	if (perspectiveEnabled && !objectCentered)
	{
		ccLog::Error(tr("Perspective mode is viewer-centered: can't use a point as rotation center!"));
		return;
	}

	s_currentPickingOperation = PICKING_ROTATION_CENTER;
	enablePickingOperation(win, tr("Pick a point to be used as rotation center (click on icon again to cancel)"));
}

ccPointCloud* MainWindow::askUserToSelectACloud(ccHObject* defaultCloudEntity/*=nullptr*/, QString inviteMessage/*=QString()*/)
{
	ccHObject::Container clouds;
	m_ccRoot->getRootEntity()->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, true);
	if (clouds.empty())
	{
		ccConsole::Error(tr("No cloud in database!"));
		return nullptr;
	}
	//default selected index
	int selectedIndex = 0;
	if (defaultCloudEntity)
	{
		for (size_t i = 1; i < clouds.size(); ++i)
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
		selectedIndex = ccItemSelectionDlg::SelectEntity(clouds, selectedIndex, this, inviteMessage);
		if (selectedIndex < 0)
			return nullptr;
	}

	assert(selectedIndex >= 0 && static_cast<size_t>(selectedIndex) < clouds.size());
	return ccHObjectCaster::ToPointCloud(clouds[selectedIndex]);
}

void MainWindow::toggleSelectedEntitiesProperty( ccEntityAction::TOGGLE_PROPERTY property )
{
	if ( !ccEntityAction::toggleProperty( m_selectedEntities, property ) )
	{
		return;
	}
	
	refreshAll();
	updateUI();
}

void MainWindow::clearSelectedEntitiesProperty( ccEntityAction::CLEAR_PROPERTY property )
{
	if ( !ccEntityAction::clearProperty( m_selectedEntities, property, this ) )
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::setView( CC_VIEW_ORIENTATION view )
{
	ccGLWindowInterface* win = getActiveGLWindow();
	if (win)
	{
		win->setView(view);
	}
}

void MainWindow::spawnHistogramDialog(const std::vector<unsigned>& histoValues, double minVal, double maxVal, QString title, QString xAxisLabel)
{
	ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);
	hDlg->setAttribute(Qt::WA_DeleteOnClose, true);
	hDlg->setWindowTitle(tr("Histogram"));

	ccHistogramWindow* histogram = hDlg->window();
	{
		histogram->setTitle(title);
		histogram->fromBinArray(histoValues, minVal, maxVal);
		histogram->setAxisLabels(xAxisLabel, tr("Count"));
		histogram->refresh();
	}

	hDlg->show();
}

void MainWindow::showSelectedEntitiesHistogram()
{
	for ( ccHObject *entity : getSelectedEntities() )
	{
		//for "real" point clouds only
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud( entity );
		if (cloud)
		{
			//we display the histogram of the current scalar field
			ccScalarField* sf = static_cast<ccScalarField*>(cloud->getCurrentDisplayedScalarField());
			if (sf)
			{
				ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);
				hDlg->setAttribute(Qt::WA_DeleteOnClose, true);
				hDlg->setWindowTitle(tr("Histogram [%1]").arg(cloud->getName()));

				ccHistogramWindow* histogram = hDlg->window();
				{
					unsigned numberOfPoints = cloud->size();
					unsigned numberOfClasses = static_cast<unsigned>(sqrt(static_cast<double>(numberOfPoints)));
					//we take the 'nearest' multiple of 4
					numberOfClasses &= (~3);
					numberOfClasses = std::max<unsigned>(4, numberOfClasses);
					numberOfClasses = std::min<unsigned>(256, numberOfClasses);

					histogram->setTitle(tr("%1 (%2 values) ").arg(QString::fromStdString(sf->getName())).arg(numberOfPoints));
					bool showNaNValuesInGrey = sf->areNaNValuesShownInGrey();
					histogram->fromSF(sf, numberOfClasses, true, showNaNValuesInGrey);
					histogram->setAxisLabels(QString::fromStdString(sf->getName()), tr("Count"));
					histogram->refresh();
				}
				hDlg->show();
			}
		}
	}
}

void MainWindow::doActionCrop()
{
	//find candidates
	std::vector<ccHObject*> candidates;
	ccBBox baseBB;
	{
		const ccHObject::Container& selectedEntities = getSelectedEntities();
		for ( ccHObject *entity : selectedEntities )
		{
			if (	entity->isA(CC_TYPES::POINT_CLOUD)
				||	entity->isKindOf(CC_TYPES::MESH) )
			{
				candidates.push_back(entity);
				baseBB += entity->getOwnBB();
			}
		}
	}

	if (candidates.empty())
	{
		ccConsole::Warning(tr("[Crop] No eligible candidate found!"));
		return;
	}

	ccBoundingBoxEditorDlg bbeDlg(false, false, this);
	bbeDlg.setBaseBBox(baseBB, false);
	bbeDlg.showInclusionWarning(false);
	bbeDlg.setWindowTitle("Crop");

	if (!bbeDlg.exec())
	{
		//process cancelled by user
		return;
	}

	//deselect all entities
	if (m_ccRoot)
	{
		m_ccRoot->unselectAllEntities();
	}

	//cropping box
	ccBBox box = bbeDlg.getBox();

	//process cloud/meshes
	bool errors = false;
	bool successes = false;
	{
		for ( ccHObject *entity : candidates )
		{
			ccHObject* croppedEnt = ccCropTool::Crop(entity, box, true);
			if (croppedEnt)
			{
				croppedEnt->setName(entity->getName() + QString(".cropped"));
				croppedEnt->setDisplay(entity->getDisplay());
				croppedEnt->prepareDisplayForRefresh();
				if (entity->getParent())
					entity->getParent()->addChild(croppedEnt);
				entity->setEnabled(false);
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
		ccLog::Warning(tr("[Crop] Selected entities have been hidden"));
	if (errors)
		ccLog::Error(tr("Error(s) occurred! See the Console"));

	refreshAll();
	updateUI();
}

void MainWindow::doActionClone()
{
	ccHObject* lastClone = nullptr;
	
	for ( ccHObject *entity : getSelectedEntities() )
	{
		ccHObject* clone = nullptr;
		
		if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			clone = ccHObjectCaster::ToGenericPointCloud(entity)->clone();
			if (!clone)
			{
				ccConsole::Error(tr("An error occurred while cloning cloud %1").arg(entity->getName()));
			}
		}
		else if (entity->isKindOf(CC_TYPES::PRIMITIVE))
		{
			clone = static_cast<ccGenericPrimitive*>(entity)->clone();
			if (!clone)
			{
				ccConsole::Error(tr("An error occurred while cloning primitive %1").arg(entity->getName()));
			}
		}
		else if (entity->isA(CC_TYPES::MESH))
		{
			clone = ccHObjectCaster::ToMesh(entity)->cloneMesh();
			if (!clone)
			{
				ccConsole::Error(tr("An error occurred while cloning mesh %1").arg(entity->getName()));
			}
		}
		else if (entity->isA(CC_TYPES::POLY_LINE))
		{
			clone = ccHObjectCaster::ToPolyline(entity)->clone();
			if (!clone)
			{
				ccConsole::Error(tr("An error occurred while cloning polyline %1").arg(entity->getName()));
			}
		}
		else if (entity->isA(CC_TYPES::CIRCLE))
		{
			clone = ccHObjectCaster::ToCircle(entity)->clone();
			if (!clone)
			{
				ccConsole::Error(tr("An error occurred while cloning polyline %1").arg(entity->getName()));
			}
		}
		else if (entity->isA(CC_TYPES::FACET))
		{
			clone = ccHObjectCaster::ToFacet(entity);
			if (!clone)
			{
				ccConsole::Error(tr("An error occurred while cloning facet %1").arg(entity->getName()));
			}
		}
		else if (entity->isA(CC_TYPES::CAMERA_SENSOR))
		{
			ccCameraSensor* camera = ccHObjectCaster::ToCameraSensor(entity);
			if (camera)
			{
				clone = new ccCameraSensor(*camera);
				if (camera->getParent())
				{
					camera->getParent()->addChild(clone);
				}
			}
			if (!clone)
			{
				ccConsole::Error(tr("An error occurred while cloning camera sensor %1").arg(entity->getName()));
			}
		}
		else if (entity->isA(CC_TYPES::GBL_SENSOR))
		{
			ccGBLSensor* sensor = ccHObjectCaster::ToGBLSensor(entity);
			if (sensor)
			{
				clone = new ccGBLSensor(*sensor);
				if (sensor->getParent())
				{
					sensor->getParent()->addChild(clone);
				}
			}
			if (!clone)
			{
				ccConsole::Error(tr("An error occurred while cloning GBL sensor %1").arg(entity->getName()));
			}
		}
		else if (entity->isA(CC_TYPES::IMAGE))
		{
			ccImage* image = ccHObjectCaster::ToImage(entity);
			if (image)
			{
				clone = new ccImage(*image);
				if (image->getParent())
				{
					image->getParent()->addChild(clone);
				}
			}
			if (!clone)
			{
				ccConsole::Error(tr("An error occurred while cloning image %1").arg(entity->getName()));
			}
		}
		else if (entity->isA(CC_TYPES::LABEL_2D))
		{
			cc2DLabel* label = ccHObjectCaster::To2DLabel(entity);
			if (label)
			{
				clone = new cc2DLabel(*label, true);
				if (label->getParent())
				{
					label->getParent()->addChild(clone);
				}
			}
			if (!clone)
			{
				ccConsole::Error(tr("An error occurred while cloning label %1").arg(entity->getName()));
			}
		}
		else if (entity->isA(CC_TYPES::VIEWPORT_2D_OBJECT))
		{
		cc2DViewportObject* viewport = ccHObjectCaster::To2DViewportObject(entity);
		if (viewport)
		{
			clone = new cc2DViewportObject(*viewport);
			if (viewport->getParent())
			{
				viewport->getParent()->addChild(clone);
			}
		}
		if (!clone)
		{
			ccConsole::Error(tr("An error occurred while cloning viewport %1").arg(entity->getName()));
		}
		}
		else if (entity->isA(CC_TYPES::VIEWPORT_2D_LABEL))
		{
		cc2DViewportLabel* viewportLabel = ccHObjectCaster::To2DViewportLabel(entity);
		if (viewportLabel)
		{
			clone = new cc2DViewportLabel(*viewportLabel);
			if (viewportLabel->getParent())
			{
				viewportLabel->getParent()->addChild(clone);
			}
		}
		if (!clone)
		{
			ccConsole::Error(tr("An error occurred while cloning viewport %1").arg(entity->getName()));
		}
		}
		else
		{
			ccLog::Warning(tr("Entity '%1' can't be cloned (type not supported yet!)").arg(entity->getName()));
		}

		if (clone)
		{
			//copy GL transformation history
			clone->setGLTransformationHistory(entity->getGLTransformationHistory());
			//copy display
			clone->setDisplay(entity->getDisplay());

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

void MainWindow::doActionAddConstantSF()
{
	if (!haveOneSelection())
	{
		if (haveSelection())
			ccConsole::Error(tr("Select only one cloud or one mesh!"));
		return;
	}

	ccHObject* ent = m_selectedEntities.front();

	bool lockedVertices;
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent, &lockedVertices);

	//for "real" point clouds only
	if (!cloud)
	{
		return;
	}
	if (lockedVertices && !ent->isAncestorOf(cloud))
	{
		ccUtils::DisplayLockedVerticesWarning(ent->getName(),true);
		return;
	}

	QString defaultName = "Constant";
	unsigned trys = 1;
	while (cloud->getScalarFieldIndexByName(defaultName.toStdString()) >= 0 || trys > 99)
	{
		defaultName = tr("Constant #%1").arg(++trys);
	}

	//ask for a name
	bool ok;
	QString sfName = QInputDialog::getText(this, tr("New SF name"), tr("SF name (must be unique)"), QLineEdit::Normal, defaultName, &ok);
	if (!ok)
		return;

	if (ccEntityAction::sfAddConstant(cloud, sfName, false, this))
	{
		updateUI();
		cloud->redrawDisplay();
	}
}

void MainWindow::doActionAddClassificationSF()
{
	if (!haveOneSelection())
	{
		if (haveSelection())
			ccConsole::Error(tr("Select only one cloud or one mesh!"));
		return;
	}

	ccHObject* ent = m_selectedEntities.front();

	bool lockedVertices;
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent, &lockedVertices);

	//for "real" point clouds only
	if (!cloud)
	{
		return;
	}

	if (lockedVertices && !ent->isAncestorOf(cloud))
	{
		ccUtils::DisplayLockedVerticesWarning(ent->getName(), true);
		return;
	}

	if (ccEntityAction::sfAddConstant(cloud, "Classification", true, this))
	{
		updateUI();
		cloud->redrawDisplay();
	}
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
	double outliersRatio = 0.5;
	double confidence = 0.99;

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	for ( ccHObject *entity : getSelectedEntities() )
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (!cloud)
			continue;

		CCVector3 center;
		PointCoordinateType radius = 0;
		double rms = std::numeric_limits<double>::quiet_NaN();
		if (CCCoreLib::GeometricalAnalysisTools::DetectSphereRobust(cloud,
			outliersRatio,
			center,
			radius,
			rms,
			&pDlg,
			confidence) != CCCoreLib::GeometricalAnalysisTools::NoError)
		{
			ccLog::Warning(tr("[Fit sphere] Failed to fit a sphere on cloud '%1'").arg(cloud->getName()));
			continue;
		}

		ccLog::Print(tr("[Fit sphere] Cloud '%1': center (%2,%3,%4) - radius = %5 [RMS = %6]")
			.arg(cloud->getName())
			.arg(center.x)
			.arg(center.y)
			.arg(center.z)
			.arg(radius)
			.arg(rms));

		ccGLMatrix trans;
		trans.setTranslation(center);
		ccSphere* sphere = new ccSphere(radius, &trans, tr("Sphere r=%1").arg(radius));
		cloud->addChild(sphere);
		//sphere->setDisplay(cloud->getDisplay());
		sphere->prepareDisplayForRefresh();
		sphere->copyGlobalShiftAndScale(*cloud);
		sphere->setMetaData("RMS", rms);
		addToDB(sphere, false, false, false);
	}

	refreshAll();
}

void MainWindow::doActionFitCircle()
{
	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	for (ccHObject* entity : getSelectedEntities())
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (!cloud)
			continue;

		CCVector3 center;
		CCVector3 normal;
		PointCoordinateType radius = 0;
		double rms = std::numeric_limits<double>::quiet_NaN();
		if (CCCoreLib::GeometricalAnalysisTools::DetectCircle(	cloud,
																center,
																normal,
																radius,
																rms,
																&pDlg) != CCCoreLib::GeometricalAnalysisTools::NoError)
		{
			ccLog::Warning(tr("[Fit circle] Failed to fit a circle on cloud '%1'").arg(cloud->getName()));
			continue;
		}

		ccLog::Print(tr("[Fit circle] Cloud '%1': center (%2,%3,%4) - radius = %5 [RMS = %6]")
						.arg(cloud->getName())
						.arg(center.x)
						.arg(center.y)
						.arg(center.z)
						.arg(radius)
						.arg(rms));

		ccLog::Print(tr("[Fit circle] Normal (%1,%2,%3)")
			.arg(normal.x)
			.arg(normal.y)
			.arg(normal.z));

		// create the circle representation as a polyline
		ccCircle* circle = new ccCircle(radius, 128);
		if (circle)
		{
			circle->setName(QObject::tr("Circle r=%1").arg(radius));
			cloud->addChild(circle);
			circle->prepareDisplayForRefresh();
			circle->copyGlobalShiftAndScale(*cloud);
			circle->setMetaData("RMS", rms);

			ccGLMatrix trans = ccGLMatrix::FromToRotation(CCVector3(0, 0, 1), normal);
			trans.setTranslation(center);
			circle->applyGLTransformation(trans);

			addToDB(circle, false, false, false);
		}
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

void MainWindow::doComputePlaneOrientation(bool fitFacet)
{
	if (!haveSelection())
		return;

	double maxEdgeLength = 0.0;
	if (fitFacet)
	{
		bool ok = true;
		static double s_polygonMaxEdgeLength = 0.0;
		maxEdgeLength = QInputDialog::getDouble(this, tr("Fit facet"), tr("Max edge length (0 = no limit)"), s_polygonMaxEdgeLength, 0, 1.0e9, 8, &ok);
		if (!ok)
			return;
		s_polygonMaxEdgeLength = maxEdgeLength;
	}

	ccHObject::Container selectedEntities = getSelectedEntities(); //warning, getSelectedEntites may change during this loop!
	bool firstEntity = true;
	
	for (ccHObject *entity : selectedEntities) 
	{
		ccShiftedObject* shifted = nullptr;
		CCCoreLib::GenericIndexedCloudPersist* cloud = nullptr;

		if (entity->isKindOf(CC_TYPES::POLY_LINE))
		{
			ccPolyline* poly = ccHObjectCaster::ToPolyline(entity);
			cloud = static_cast<CCCoreLib::GenericIndexedCloudPersist*>(poly);
			shifted = poly;
		}
		else
		{
			ccGenericPointCloud* gencloud = ccHObjectCaster::ToGenericPointCloud(entity);
			if (gencloud)
			{
				cloud = static_cast<CCCoreLib::GenericIndexedCloudPersist*>(gencloud);
				shifted = gencloud;
			}
		}

		if (cloud)
		{
			double rms = 0.0;
			CCVector3 C;
			CCVector3 N;

			ccHObject* plane = nullptr;
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
							contour->copyGlobalShiftAndScale(*shifted);
						}

						ccMesh* polygon = facet->getPolygon();
						if (polygon)
						{
							polygon->copyGlobalShiftAndScale(*shifted);
						}

						ccPointCloud* points = facet->getOriginPoints();
						if (points)
						{
							points->copyGlobalShiftAndScale(*shifted);
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
					C = *CCCoreLib::Neighbourhood(cloud).getGravityCenter();
					pPlane->enableStippling(true);

					if (shifted)
					{
						pPlane->copyGlobalShiftAndScale(*shifted);
					}
				}
			}

			//as all information appears in Console...
			forceConsoleDisplay();

			if (plane)
			{
				ccConsole::Print(tr("[Orientation] Entity '%1'").arg(entity->getName()));
				ccConsole::Print(tr("\t- plane fitting RMS: %1").arg(rms));

				//We always consider the normal with a positive 'Z' by default!
				if (N.z < 0.0)
					N *= -1.0;
				ccConsole::Print(tr("\t- normal: (%1, %2, %3)").arg(N.x).arg(N.y).arg(N.z));

				//we compute strike & dip by the way
				PointCoordinateType dip = 0.0f;
				PointCoordinateType dipDir = 0.0f;
				ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);
				QString dipAndDipDirStr = ccNormalVectors::ConvertDipAndDipDirToString(dip, dipDir);
				ccConsole::Print(QString("\t- %1").arg(dipAndDipDirStr));

				//hack: output the transformation matrix that would make this normal points towards +Z
				ccGLMatrix makeZPosMatrix = ccGLMatrix::FromToRotation(N, CCVector3(0, 0, CCCoreLib::PC_ONE));
				CCVector3 Gt = C;
				makeZPosMatrix.applyRotation(Gt);
				makeZPosMatrix.setTranslation(C-Gt);
				ccConsole::Print(tr("[Orientation] A matrix that would make this plane horizontal (normal towards Z+) is:"));
				ccConsole::Print(makeZPosMatrix.toString(12,' ')); //full precision
				ccConsole::Print(tr("[Orientation] You can copy this matrix values (CTRL+C) and paste them in the 'Apply transformation tool' dialog"));

				plane->setName(dipAndDipDirStr);
				plane->applyGLTransformation_recursive(); //not yet in DB
				plane->setVisible(true);
				plane->setSelectionBehavior(ccHObject::SELECTION_FIT_BBOX);

				entity->addChild(plane);
				plane->setDisplay(entity->getDisplay());
				plane->prepareDisplayForRefresh_recursive();
				addToDB(plane);

				if (firstEntity)
				{
					m_ccRoot->unselectAllEntities();
					m_ccRoot->selectEntity(plane);
				}
			}
			else
			{
				ccConsole::Warning(tr("Failed to fit a plane/facet on entity '%1'").arg(entity->getName()));
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

void MainWindow::doComputeGeometricFeature()
{
	static ccLibAlgorithms::GeomCharacteristicSet s_selectedCharacteristics;
	static CCVector3 s_upDir(0, 0, 1);
	static bool s_upDirDefined = false;

	ccGeomFeaturesDlg gfDlg(this);
	double radius = ccLibAlgorithms::GetDefaultCloudKernelSize(m_selectedEntities);
	gfDlg.setRadius(radius);

	// restore semi-persistent parameters
	gfDlg.setSelectedFeatures(s_selectedCharacteristics);
	if (s_upDirDefined)
	{
		gfDlg.setUpDirection(s_upDir);
	}

	if (!gfDlg.exec())
		return;

	radius = gfDlg.getRadius();
	if (!gfDlg.getSelectedFeatures(s_selectedCharacteristics))
	{
		ccLog::Error(tr("Not enough memory!"));
		return;
	}

	CCVector3* upDir = gfDlg.getUpDirection();

	// remember semi-persistent parameters
	s_upDirDefined = (upDir != nullptr);
	if (s_upDirDefined)
	{
		s_upDir = *upDir;
	}

	ccLibAlgorithms::ComputeGeomCharacteristics(s_selectedCharacteristics, static_cast<PointCoordinateType>(radius), m_selectedEntities, upDir, this);

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

void MainWindow::doSphericalNeighbourhoodExtractionTest()
{
	size_t selNum = m_selectedEntities.size();
	if (selNum < 1)
		return;

	//spherical neighborhood extraction radius
	double sphereRadius = ccLibAlgorithms::GetDefaultCloudKernelSize(m_selectedEntities);
	if (sphereRadius < 0)
	{
		ccConsole::Error(tr("Invalid kernel size!"));
		return;
	}

	bool ok;
	double val = QInputDialog::getDouble(this, tr("SNE test"), tr("Radius:"), sphereRadius, DBL_MIN, 1.0e9, 8, &ok);
	if (!ok)
		return;
	sphereRadius = val;

	QString sfName = tr("Spherical extraction test (%1)").arg(sphereRadius);

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	for (size_t i = 0; i < selNum; ++i)
	{
		//we only process clouds
		if (!m_selectedEntities[i]->isA(CC_TYPES::POINT_CLOUD))
		{
			continue;
		}
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_selectedEntities[i]);

		int sfIdx = cloud->getScalarFieldIndexByName(sfName.toStdString());
		if (sfIdx < 0)
			sfIdx = cloud->addScalarField(sfName.toStdString());
		if (sfIdx < 0)
		{
			ccConsole::Error(tr("Failed to create scalar field on cloud '%1' (not enough memory?)").arg(cloud->getName()));
			return;
		}
			
		ccOctree::Shared octree = cloud->getOctree();
		if (!octree)
		{
			pDlg.reset();
			pDlg.show();
			octree = cloud->computeOctree(&pDlg);
			if (!octree)
			{
				ccConsole::Error(tr("Couldn't compute octree for cloud '%1'!").arg(cloud->getName()));
				return;
			}
		}

		CCCoreLib::ScalarField* sf = cloud->getScalarField(sfIdx);
		sf->fill(CCCoreLib::NAN_VALUE);
		cloud->setCurrentScalarField(sfIdx);

		QElapsedTimer eTimer;
		eTimer.start();

		size_t extractedPoints = 0;
		unsigned char level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(static_cast<PointCoordinateType>(sphereRadius));
		std::random_device rd;   // non-deterministic generator
		std::mt19937 gen(rd());  // to seed mersenne twister.
		std::uniform_int_distribution<unsigned> dist(0, cloud->size() - 1);

		const unsigned samples = 1000;
		for (unsigned j = 0; j < samples; ++j)
		{
			unsigned randIndex = dist(gen);
			CCCoreLib::DgmOctree::NeighboursSet neighbours;
			octree->getPointsInSphericalNeighbourhood(	*cloud->getPoint(randIndex),
														static_cast<PointCoordinateType>(sphereRadius),
														neighbours,
														level );
			size_t neihgboursCount = neighbours.size();
			extractedPoints += neihgboursCount;
			for (size_t k = 0; k < neihgboursCount; ++k)
			{
				cloud->setPointScalarValue(neighbours[k].pointIndex, static_cast<ScalarType>(sqrt(neighbours[k].squareDistd)));
			}
		}
		ccConsole::Print(tr("[SNE_TEST] Mean extraction time = %1 ms (radius = %2, mean (neighbours) = %3)").arg(eTimer.elapsed()).arg(sphereRadius).arg(extractedPoints / static_cast<double>(samples), 0, 'f', 1));

		sf->computeMinAndMax();
		cloud->setCurrentDisplayedScalarField(sfIdx);
		cloud->showSF(true);
		cloud->prepareDisplayForRefresh();
	}

	refreshAll();
	updateUI();
}

void MainWindow::doCylindricalNeighbourhoodExtractionTest()
{
	bool ok;
	double radius = QInputDialog::getDouble(this, tr("CNE Test"), tr("radius"), 0.02, 1.0e-6, 1.0e6, 6, &ok);
	if (!ok)
		return;

	double height = QInputDialog::getDouble(this, tr("CNE Test"), tr("height"), 0.05, 1.0e-6, 1.0e6, 6, &ok);
	if (!ok)
		return;

	ccPointCloud* cloud = new ccPointCloud(tr("cube"));
	const unsigned ptsCount = 1000000;
	if (!cloud->reserve(ptsCount))
	{
		ccConsole::Error(tr("Not enough memory!"));
		delete cloud;
		return;
	}

	//fill a unit cube with random points
	{
		std::random_device rd;   // non-deterministic generator
		std::mt19937 gen(rd());  // to seed mersenne twister.
		std::uniform_real_distribution<double> dist(0, 1);

		for (unsigned i = 0; i < ptsCount; ++i)
		{
			CCVector3 P(dist(gen),
				dist(gen),
				dist(gen));

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
		ccConsole::Error(tr("Not enough memory!"));
		delete cloud;
		return;
	}
	cloud->setCurrentScalarField(sfIdx);

	//reset scalar field
	cloud->getScalarField(sfIdx)->fill(CCCoreLib::NAN_VALUE);

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

		for (unsigned j = 0; j < samples; ++j)
		{
			//generate random normal vector
			CCVector3 dir(0, 0, 1);
			{
				ccGLMatrix rot;
				rot.initFromParameters(distAngle(gen),
					distAngle(gen),
					distAngle(gen),
					CCVector3(0, 0, 0));
				rot.applyRotation(dir);
			}
			unsigned randIndex = distIndex(gen);

			CCCoreLib::DgmOctree::CylindricalNeighbourhood cn;
			cn.center = *cloud->getPoint(randIndex);
			cn.dir = dir;
			cn.level = level;
			cn.radius = static_cast<PointCoordinateType>(radius);
			cn.maxHalfLength = static_cast<PointCoordinateType>(height / 2);

			octree->getPointsInCylindricalNeighbourhood(cn);
			//octree->getPointsInSphericalNeighbourhood(*cloud->getPoint(randIndex),radius,neighbours,level);
			size_t neihgboursCount = cn.neighbours.size();
			extractedPoints += static_cast<unsigned long long>(neihgboursCount);
			for (size_t k = 0; k < neihgboursCount; ++k)
			{
				cloud->setPointScalarValue(cn.neighbours[k].pointIndex, static_cast<ScalarType>(sqrt(cn.neighbours[k].squareDistd)));
			}
		}
		ccConsole::Print(tr("[CNE_TEST] Mean extraction time = %1 ms (radius = %2, height = %3, mean (neighbours) = %4))").arg(subTimer.elapsed()).arg(radius).arg(height).arg(static_cast<double>(extractedPoints) / samples, 0, 'f', 1));
	}
	else
	{
		ccConsole::Error(tr("Failed to compute octree!"));
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
	size_t selNum = getSelectedEntities().size();

	ccPointCloud* centers = new ccPointCloud(tr("centers"));
	if (!centers->reserve(static_cast<unsigned>(selNum)))
	{
		ccLog::Error(tr("Not enough memory!"));
		delete centers;
		centers = nullptr;
		return;
	}

	//look for clouds
	{
		for ( ccHObject *entity : getSelectedEntities() )
		{
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
			
			if (cloud == nullptr)
			{
				continue;
			}
			
			centers->addPoint(cloud->getOwnBB().getCenter());
			
			//we display the cloud in the same window as the first (selected) cloud we encounter
			if (!centers->getDisplay())
			{
				centers->setDisplay(cloud->getDisplay());
			}
		}
	}

	if (centers->size() == 0)
	{
		ccLog::Error(tr("No cloud in selection?!"));
		delete centers;
		centers = nullptr;
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
	//look for clouds
	std::vector<ccPointCloud*> clouds;
	try
	{
		for ( ccHObject *entity : getSelectedEntities() )
		{
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
			if (cloud)
			{
				clouds.push_back(cloud);
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error(tr("Not enough memory!"));
		return;
	}

	size_t cloudCount = clouds.size();
	if (cloudCount < 2)
	{
		ccLog::Error(tr("Need at least two clouds!"));
		return;
	}

	//init matrices
	std::vector<double> rmsMatrix;
	std::vector<ccGLMatrix> matrices;
	std::vector< std::pair<double, double> > matrixAngles;
	try
	{
		rmsMatrix.resize(cloudCount*cloudCount, 0);

		//init all possible transformations
		static const double angularStep_deg = 45.0;
		unsigned phiSteps = static_cast<unsigned>(360.0 / angularStep_deg);
		assert(CCCoreLib::LessThanEpsilon(std::abs(360.0 - phiSteps * angularStep_deg)));
		unsigned thetaSteps = static_cast<unsigned>(180.0 / angularStep_deg);
		assert(CCCoreLib::LessThanEpsilon(std::abs(180.0 - thetaSteps * angularStep_deg)));
		unsigned rotCount = phiSteps * (thetaSteps - 1) + 2;
		matrices.reserve(rotCount);
		matrixAngles.reserve(rotCount);

		for (unsigned j = 0; j <= thetaSteps; ++j)
		{
			//we want to cover the full [0-180] interval! ([-90;90] in fact)
			double theta_deg = j * angularStep_deg - 90.0;
			for (unsigned i = 0; i < phiSteps; ++i)
			{
				double phi_deg = i * angularStep_deg;
				ccGLMatrix trans;
				trans.initFromParameters(	static_cast<float>( CCCoreLib::DegreesToRadians( phi_deg ) ),
											static_cast<float>( CCCoreLib::DegreesToRadians( theta_deg ) ),
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
		ccLog::Error(tr("Not enough memory!"));
		return;
	}

	//let's start!
	{
		ccProgressDialog pDlg(true, this);
		pDlg.setMethodTitle(tr("Testing all possible positions"));
		pDlg.setInfo(tr("%1 clouds and %2 positions").arg(cloudCount).arg(matrices.size()));
		CCCoreLib::NormalizedProgress nProgress(&pDlg, static_cast<unsigned>(((cloudCount*(cloudCount - 1)) / 2)*matrices.size()));
		pDlg.start();
		QApplication::processEvents();

//#define TEST_GENERATION
#ifdef TEST_GENERATION
		ccPointCloud* testSphere = new ccPointCloud();
		testSphere->reserve(matrices.size());
#endif

		for (size_t i = 0; i < cloudCount - 1; ++i)
		{
			ccPointCloud* A = clouds[i];
			A->computeOctree();

			for (size_t j = i + 1; j < cloudCount; ++j)
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
				ccPointCloud* bestB = nullptr;
#endif
				for (size_t k = 0; k < matrices.size(); ++k)
				{
					ccPointCloud* B = clouds[j]->cloneThis();
					if (!B)
					{
						ccLog::Error(tr("Not enough memory!"));
						return;
					}

					ccGLMatrix BtoA = transFromZeroToA * matrices[k] * transBToZero;
					B->applyRigidTransformation(BtoA);

#ifndef TEST_GENERATION
					double finalRMS = 0.0;
					unsigned finalPointCount = 0;
					CCCoreLib::ICPRegistrationTools::RESULT_TYPE result;
					CCCoreLib::ICPRegistrationTools::ScaledTransformation registerTrans;
					CCCoreLib::ICPRegistrationTools::Parameters params;
					{
						params.convType = CCCoreLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE;
						params.minRMSDecrease = 1.0e-6;
					}

					result = CCCoreLib::ICPRegistrationTools::Register(A, nullptr, B, params, registerTrans, finalRMS, finalPointCount);

					if (result >= CCCoreLib::ICPRegistrationTools::ICP_ERROR)
					{
						delete B;
						if (bestB)
							delete bestB;
						ccLog::Error(tr("An error occurred while performing ICP!"));
						return;
					}

					if (minRMS < 0 || finalRMS < minRMS)
					{
						minRMS = finalRMS;
						bestMatrixIndex = static_cast<int>(k);
						std::swap(bestB, B);
					}

					if (B)
					{
						delete B;
						B = nullptr;
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
					ccHObject* group = new ccHObject(tr("Best case #%1 / #%2 - RMS = %3").arg(i+1).arg(j+1).arg(minRMS));
					group->addChild(bestB);
					group->setDisplay_recursive(A->getDisplay());
					addToDB(group);
					ccLog::Print(tr("[DoActionComputeBestICPRmsMatrix] Comparison #%1 / #%2: min RMS = %3 (phi = %4 / theta = %5 deg.)").arg(i+1).arg(j+1).arg(minRMS).arg(matrixAngles[bestMatrixIndex].first).arg(matrixAngles[bestMatrixIndex].second));
				}
				else
				{
					assert(!bestB);
					ccLog::Warning(tr("[DoActionComputeBestICPRmsMatrix] Comparison #%1 / #%2: INVALID").arg(i+1).arg(j+1));
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

		QString outputFilename = QFileDialog::getSaveFileName(	this,
																tr("Select output file"),
																currentPath,
																"*.csv",
																nullptr,
																CCFileDialogOptions());

		if (outputFilename.isEmpty())
			return;

		QFile fp(outputFilename);
		if (fp.open(QFile::Text | QFile::WriteOnly))
		{
			QTextStream stream(&fp);
			//header
			{
				stream << "RMS";
				for ( ccPointCloud *cloud : clouds )
				{
					stream << ';';
					stream << cloud->getName();
				}
				stream << endl;
			}

			//rows
			for (size_t j = 0; j < cloudCount; ++j)
			{
				stream << clouds[j]->getName();
				stream << ';';
				for (size_t i = 0; i < cloudCount; ++i)
				{
					stream << rmsMatrix[j*cloudCount+i];
					stream << ';';
				}
				stream << endl;
			}

			ccLog::Print(tr("[DoActionComputeBestICPRmsMatrix] Job done"));
		}
		else
		{
			ccLog::Error(tr("Failed to save output file?!"));
		}
	}
}

void MainWindow::doActionExportPlaneInfo()
{
	ccHObject::Container planes;

	const ccHObject::Container& selectedEntities = getSelectedEntities();
	if (selectedEntities.size() == 1 && selectedEntities.front()->isA(CC_TYPES::HIERARCHY_OBJECT))
	{
		//a group
		selectedEntities.front()->filterChildren(planes, true, CC_TYPES::PLANE, false);
	}
	else
	{
		for (ccHObject* ent : selectedEntities)
		{
			if (ent->isKindOf(CC_TYPES::PLANE))
			{
				//a single plane
				planes.push_back(static_cast<ccPlane*>(ent));
			}
		}
	}

	if (planes.size() == 0)
	{
		ccLog::Error(tr("No plane in selection"));
		return;
	}

	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::SaveFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	QString outputFilename = QFileDialog::getSaveFileName(	this,
															tr("Select output file"),
															currentPath,
															"*.csv",
															nullptr,
															CCFileDialogOptions());

	if (outputFilename.isEmpty())
	{
		//process cancelled by the user
		return;
	}

	QFile csvFile(outputFilename);
	if (!csvFile.open(QFile::WriteOnly | QFile::Text))
	{
		ccConsole::Error(tr("Failed to open file for writing! (check file permissions)"));
		return;
	}

	//save last saving location
	settings.setValue(ccPS::CurrentPath(), QFileInfo(outputFilename).absolutePath());
	settings.endGroup();

	//write CSV header
	QTextStream csvStream(&csvFile);
	csvStream << "Name;";
	csvStream << "Width;";
	csvStream << "Height;";
	csvStream << "Cx;";
	csvStream << "Cy;";
	csvStream << "Cz;";
	csvStream << "Cx_global;";
	csvStream << "Cy_global;";
	csvStream << "Cz_global;";
	csvStream << "Nx;";
	csvStream << "Ny;";
	csvStream << "Nz;";
	csvStream << "Dip;";
	csvStream << "Dip dir;";
	csvStream << endl;

	QChar separator(';');

	//write one line per plane
	for (ccHObject* ent : planes)
	{
		ccPlane* plane = static_cast<ccPlane*>(ent);
			
		CCVector3 C = plane->getOwnBB().getCenter();
		CCVector3d Cg = plane->toGlobal3d(C);
		CCVector3 N = plane->getNormal();
		PointCoordinateType dip_deg = 0;
		PointCoordinateType dipDir_deg = 0;
		ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip_deg, dipDir_deg);

		csvStream << plane->getName() << separator;		//Name
		csvStream << plane->getXWidth() << separator;	//Width
		csvStream << plane->getYWidth() << separator;	//Height
		csvStream << C.x << separator;					//Cx
		csvStream << C.y << separator;					//Cy
		csvStream << C.z << separator;					//Cz
		csvStream << Cg.x << separator;					//Cx
		csvStream << Cg.y << separator;					//Cy
		csvStream << Cg.z << separator;					//Cz
		csvStream << N.x << separator;					//Nx
		csvStream << N.y << separator;					//Ny
		csvStream << N.z << separator;					//Nz
		csvStream << dip_deg << separator;				//Dip
		csvStream << dipDir_deg << separator;			//Dip direction
		csvStream << endl;
	}

	ccConsole::Print(tr("[I/O] File '%1' successfully saved (%2 plane(s))").arg(outputFilename).arg(planes.size()));
	csvFile.close();
}

void MainWindow::doActionExportCloudInfo()
{
	//look for clouds
	ccHObject::Container clouds;

	const ccHObject::Container& selectedEntities = getSelectedEntities();
	if (selectedEntities.size() == 1 && selectedEntities.front()->isA(CC_TYPES::HIERARCHY_OBJECT))
	{
		//a group
		selectedEntities.front()->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, true);
	}
	else
	{
		for (ccHObject* entity : selectedEntities)
		{
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
			if (cloud)
			{
				clouds.push_back(cloud);
			}
		}
	}

	if (clouds.empty())
	{
		ccConsole::Error(tr("Select at least one point cloud!"));
		return;
	}

	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::SaveFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	QString outputFilename = QFileDialog::getSaveFileName(	this,
															tr("Select output file"),
															currentPath,
															"*.csv",
															nullptr,
															CCFileDialogOptions());
	if (outputFilename.isEmpty())
	{
		//process cancelled by the user
		return;
	}

	QFile csvFile(outputFilename);
	if (!csvFile.open(QFile::WriteOnly | QFile::Text))
	{
		ccConsole::Error(tr("Failed to open file for writing! (check file permissions)"));
		return;
	}

	//save last saving location
	settings.setValue(ccPS::CurrentPath(), QFileInfo(outputFilename).absolutePath());
	settings.endGroup();

	//determine the maximum number of SFs
	unsigned maxSFCount = 0;
	for (ccHObject* entity : clouds)
	{
		maxSFCount = std::max<unsigned>(maxSFCount, static_cast<ccPointCloud*>(entity)->getNumberOfScalarFields());
	}

	//write CSV header
	QTextStream csvStream(&csvFile);
	csvStream << "Name;";
	csvStream << "Points;";
	csvStream << "meanX;";
	csvStream << "meanY;";
	csvStream << "meanZ;";
	csvStream << "meanX_global;";
	csvStream << "meanY_global;";
	csvStream << "meanZ_global;";
	{
		for (unsigned i = 0; i < maxSFCount; ++i)
		{
			QString sfIndex = QString("SF#%1").arg(i + 1);
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
		for (ccHObject* entity : clouds)
		{
			ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);

			CCVector3 G = *CCCoreLib::Neighbourhood(cloud).getGravityCenter();
			CCVector3d Gg = cloud->toGlobal3d(G);
			csvStream << cloud->getName() << ';' /*"Name;"*/;
			csvStream << cloud->size() << ';' /*"Points;"*/;
			csvStream << G.x << ';' /*"meanX;"*/;
			csvStream << G.y << ';' /*"meanY;"*/;
			csvStream << G.z << ';' /*"meanZ;"*/;
			csvStream << Gg.x << ';' /*"meanX_global;"*/;
			csvStream << Gg.y << ';' /*"meanY_global;"*/;
			csvStream << Gg.z << ';' /*"meanZ_global;"*/;
			for (unsigned j = 0; j < cloud->getNumberOfScalarFields(); ++j)
			{
				CCCoreLib::ScalarField* sf = cloud->getScalarField(j);
				csvStream << QString::fromStdString(sf->getName()) << ';' /*"SF name;"*/;

				unsigned validCount = 0;
				double sfSum = 0.0;
				double sfSum2 = 0.0;
				for (unsigned k = 0; k < sf->currentSize(); ++k)
				{
					const ScalarType& val = sf->getValue(k);
					if (CCCoreLib::ScalarField::ValidValue(val))
					{
						++validCount;
						sfSum += val;
						sfSum2 += static_cast<double>(val)*val;
					}
				}
				csvStream << validCount << ';' /*"SF valid values;"*/;
				if (validCount)
				{
					double mean = sfSum / validCount;
					csvStream << mean << ';' /*"SF mean;"*/;
					csvStream << sqrt(std::abs(sfSum2 / validCount - mean * mean)) << ';' /*"SF std.dev.;"*/;
				}
				else
				{
					csvStream << "N/A;" /*"SF mean;"*/;
					csvStream << "N/A;" /*"SF std.dev.;"*/;
				}
				csvStream << sfSum << ';' /*"SF sum;"*/;
			}
			csvStream << endl;
		}
	}

	ccConsole::Print(tr("[I/O] File '%1' successfully saved (%2 cloud(s))").arg(outputFilename).arg(clouds.size()));
	csvFile.close();
}

void MainWindow::doActionCloudCloudDist()
{
	if (getSelectedEntities().size() != 2)
	{
		ccConsole::Error(tr("Select 2 point clouds!"));
		return;
	}

	if (!m_selectedEntities.front()->isKindOf(CC_TYPES::POINT_CLOUD) ||
		!m_selectedEntities.back()->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error(tr("Select 2 point clouds!"));
		return;
	}

	ccOrderChoiceDlg dlg(	m_selectedEntities.front(), tr("Compared"),
							m_selectedEntities.back(), tr("Reference"),
							this );
	if (!dlg.exec())
		return;

	ccGenericPointCloud* compCloud = ccHObjectCaster::ToGenericPointCloud(dlg.getFirstEntity());
	ccGenericPointCloud* refCloud = ccHObjectCaster::ToGenericPointCloud(dlg.getSecondEntity());

	//assert(!m_compDlg);
	if (m_compDlg)
		delete m_compDlg;

	m_compDlg = new ccComparisonDlg(compCloud, refCloud, ccComparisonDlg::CLOUDCLOUD_DIST, this);
	if (!m_compDlg->initDialog())
	{
		ccConsole::Error(tr("Failed to initialize comparison dialog"));
		delete m_compDlg;
		m_compDlg = nullptr;
		return;
	}

	connect(m_compDlg, &QDialog::finished, this, &MainWindow::deactivateComparisonMode);
	m_compDlg->show();
	//cDlg.setModal(false);
	//cDlg.exec();
	freezeUI(true);
}

void MainWindow::doActionCloudMeshDist()
{
	if (getSelectedEntities().size() != 2)
	{
		ccConsole::Error(tr("Select 2 entities!"));
		return;
	}

	bool isMesh[2]{ false,false };
	unsigned meshNum = 0;
	unsigned cloudNum = 0;
	for (unsigned i = 0; i < 2; ++i)
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
		ccConsole::Error(tr("Select at least one mesh!"));
		return;
	}
	else if (meshNum + cloudNum < 2)
	{
		ccConsole::Error(tr("Select one mesh and one cloud or two meshes!"));
		return;
	}

	ccHObject* compEnt = nullptr;
	ccGenericMesh* refMesh = nullptr;

	if (meshNum == 1)
	{
		compEnt = m_selectedEntities[isMesh[0] ? 1 : 0];
		refMesh = ccHObjectCaster::ToGenericMesh(m_selectedEntities[isMesh[0] ? 0 : 1]);

		if (refMesh->isKindOf(CC_TYPES::PRIMITIVE))
		{
			static bool DontShowPrimitiveDistWarning = false;
			if (!DontShowPrimitiveDistWarning)
			{
				QMessageBox::StandardButton answer = QMessageBox::warning(
					this,
					tr("Distance to primitive"),
					tr("Computing distances to a primitive is faster and more accurate with the 'Tools > Distances > Cloud / Primitive Dist.' tool.\nDo you want to use this other tool instead?"),
					QMessageBox::Yes | QMessageBox::No | QMessageBox::NoToAll,
					QMessageBox::Yes
				);

				if (answer == QMessageBox::Yes)
				{
					return doActionCloudPrimitiveDist();
				}
				else if (answer == QMessageBox::NoToAll)
				{
					DontShowPrimitiveDistWarning = true;
				}
			}
		}
	}
	else
	{
		ccOrderChoiceDlg dlg(	m_selectedEntities.front(), tr("Compared"),
								m_selectedEntities.back(), tr("Reference"),
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
	if (!m_compDlg->initDialog())
	{
		ccConsole::Error(tr("Failed to initialize comparison dialog"));
		delete m_compDlg;
		m_compDlg = nullptr;
		return;
	}

	connect(m_compDlg, &QDialog::finished, this, &MainWindow::deactivateComparisonMode);
	m_compDlg->show();

	freezeUI(true);
}

void MainWindow::doActionCloudPrimitiveDist()
{
	ccHObject::Container clouds;
	ccHObject* refEntity = nullptr;

	for (ccHObject* entity : getSelectedEntities())
	{
		if (entity->isKindOf(CC_TYPES::PRIMITIVE) || entity->isA(CC_TYPES::POLY_LINE))
		{
			if (entity->isA(CC_TYPES::PLANE) ||
				entity->isA(CC_TYPES::SPHERE) ||
				entity->isA(CC_TYPES::CYLINDER) ||
				entity->isA(CC_TYPES::CONE) ||
				entity->isA(CC_TYPES::BOX) ||
				entity->isA(CC_TYPES::POLY_LINE))
			{
				if (!refEntity)
				{
					// first primitive encountered
					refEntity = entity;
				}
				else
				{
					ccConsole::Error(tr("Select only one primitive (Plane/Box/Sphere/Cylinder/Cone) or polyline"));
					return;
				}
			}
		}
		else if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			clouds.push_back(entity);
		}
	}

	if (!refEntity)
	{
		ccConsole::Error(tr("Select one prmitive (Plane/Box/Sphere/Cylinder/Cone) or a polyline"));
		return;
	}

	if (clouds.empty())
	{
		ccConsole::Error(tr("Select at least one cloud"));
		return;
	}
		
	ccPrimitiveDistanceDlg pDD{ this };

	static bool s_treatPlanesAsBounded = false;
	static bool s_signedDist = true;
	static bool s_flipNormals = false;

	if (refEntity->isA(CC_TYPES::PLANE))
	{
		pDD.treatPlanesAsBoundedCheckBox->setEnabled(true);
		pDD.treatPlanesAsBoundedCheckBox->setChecked(s_treatPlanesAsBounded);
	}
	else
	{
		pDD.treatPlanesAsBoundedCheckBox->setEnabled(false);
	}

	pDD.flipNormalsCheckBox->setChecked(s_flipNormals);
	pDD.signedDistCheckBox->setChecked(s_signedDist);

	if (!refEntity->isA(CC_TYPES::POLY_LINE))
	{
		if (!pDD.exec())
		{
			return;
		}
	}

	s_signedDist = pDD.signedDistances();
	s_flipNormals = pDD.flipNormals();
	s_treatPlanesAsBounded = pDD.treatPlanesAsBounded();

	size_t errorCount = 0;
	for (auto &cloud : clouds)
	{
		ccPointCloud* compEnt = ccHObjectCaster::ToPointCloud(cloud);
		int sfIdx = compEnt->getScalarFieldIndexByName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
		if (sfIdx < 0)
		{
			//we need to create a new scalar field
			sfIdx = compEnt->addScalarField(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
			if (sfIdx < 0)
			{
				ccLog::Warning(tr("[Compute Primitive Distances] [Cloud: %1] Couldn't allocate a new scalar field for computing distances! Try to free some memory ...").arg(compEnt->getName()));
				++errorCount;
				continue;
			}
		}
		compEnt->setCurrentScalarField(sfIdx);
		if (!compEnt->enableScalarField())
		{
			ccLog::Warning(tr("[Compute Primitive Distances] [Cloud: %1] Not enough memory").arg(compEnt->getName()));
			++errorCount;
			continue;
		}
		compEnt->forEach(CCCoreLib::ScalarFieldTools::SetScalarValueToNaN);

		int returnCode = 0;
		QString errString = tr("[Compute Primitive Distances] Cloud to %1 distance computation failed (error code = %2)");

		switch (refEntity->getClassID())
		{
		case CC_TYPES::SPHERE:
		{
			if (!(returnCode = CCCoreLib::DistanceComputationTools::computeCloud2SphereEquation(compEnt, refEntity->getOwnBB().getCenter(), static_cast<ccSphere*>(refEntity)->getRadius(), s_signedDist)))
				ccConsole::Error(errString.arg(tr("Sphere")).arg(returnCode));
			break;
		}

		case CC_TYPES::PLANE:
		{
			ccPlane* plane = static_cast<ccPlane*>(refEntity);
			if (s_treatPlanesAsBounded)
			{
				CCCoreLib::SquareMatrix rotationTransform(plane->getTransformation().data(), true);
				if (!(returnCode = CCCoreLib::DistanceComputationTools::computeCloud2RectangleEquation(compEnt, plane->getXWidth(), plane->getYWidth(), rotationTransform, plane->getCenter(), s_signedDist)))
				{
					ccConsole::Warning(errString.arg(tr("Bounded Plane")).arg(returnCode));
					++errorCount;
				}
			}
			else
			{
				if (!(returnCode = CCCoreLib::DistanceComputationTools::computeCloud2PlaneEquation(compEnt, static_cast<ccPlane*>(refEntity)->getEquation(), s_signedDist)))
				{
					ccConsole::Warning(errString.arg(tr("Infinite Plane")).arg(returnCode));
					++errorCount;
				}
			}
			break;
		}

		case CC_TYPES::CYLINDER:
		{
			if (!(returnCode = CCCoreLib::DistanceComputationTools::computeCloud2CylinderEquation(compEnt, static_cast<ccCylinder*>(refEntity)->getBottomCenter(), static_cast<ccCylinder*>(refEntity)->getTopCenter(), static_cast<ccCylinder*>(refEntity)->getBottomRadius(), s_signedDist)))
			{
				ccConsole::Warning(errString.arg(tr("Cylinder")).arg(returnCode));
				++errorCount;
			}
			break;
		}

		case CC_TYPES::CONE:
		{
			if (!(returnCode = CCCoreLib::DistanceComputationTools::computeCloud2ConeEquation(compEnt, static_cast<ccCone*>(refEntity)->getLargeCenter(), static_cast<ccCone*>(refEntity)->getSmallCenter(), static_cast<ccCone*>(refEntity)->getLargeRadius(), static_cast<ccCone*>(refEntity)->getSmallRadius(), s_signedDist)))
			{
				ccConsole::Warning(errString.arg(tr("Cone")).arg(returnCode));
				++errorCount;
			}
			break;
		}

		case CC_TYPES::BOX:
		{
			const ccGLMatrix& glTransform = refEntity->getGLTransformationHistory();
			CCCoreLib::SquareMatrix rotationTransform(glTransform.data(), true);
			CCVector3 boxCenter = glTransform.getColumnAsVec3D(3);
			if (!(returnCode = CCCoreLib::DistanceComputationTools::computeCloud2BoxEquation(compEnt, static_cast<ccBox*>(refEntity)->getDimensions(), rotationTransform, boxCenter, s_signedDist)))
			{
				ccConsole::Warning(errString.arg(tr("Box")).arg(returnCode));
				++errorCount;
			}
			break;
		}

		case CC_TYPES::POLY_LINE:
		{
			ccPolyline* line = static_cast<ccPolyline*>(refEntity);
			returnCode = CCCoreLib::DistanceComputationTools::computeCloud2PolylineEquation(compEnt, line);
			if (!returnCode)
			{
				ccConsole::Warning(errString.arg(tr("Polyline")).arg(returnCode));
				++errorCount;
			}
			break;
		}

		default:
		{
			ccConsole::Error(tr("Unsupported primitive type")); //Shouldn't ever reach here...
			return;
		}
		}

		QString sfName(CC_CLOUD2PRIMITIVE_DISTANCES_DEFAULT_SF_NAME);
		if (!refEntity->isKindOf(CC_TYPES::POLY_LINE))
		{
			if (s_signedDist)
			{
				sfName = CC_CLOUD2PRIMITIVE_SIGNED_DISTANCES_DEFAULT_SF_NAME;
			}
			if (s_flipNormals)
			{
				compEnt->forEach(CCCoreLib::ScalarFieldTools::SetScalarValueInverted);
				sfName += "[-]";
			}
		}

		int _sfIdx = compEnt->getScalarFieldIndexByName(sfName.toStdString());
		if (_sfIdx >= 0)
		{
			compEnt->deleteScalarField(_sfIdx);
			//we update sfIdx because indexes are all messed up after deletion
			sfIdx = compEnt->getScalarFieldIndexByName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
		}
		compEnt->renameScalarField(sfIdx, sfName.toStdString());

		ccScalarField* sf = static_cast<ccScalarField*>(compEnt->getScalarField(sfIdx));
		if (sf)
		{
			ScalarType mean;
			ScalarType variance;
			sf->computeMinAndMax();
			sf->computeMeanAndVariance(mean, &variance);
			ccLog::Print(tr("[Compute Primitive Distances] [Primitive: %1] [Cloud: %2] [%3] Mean distance = %4 / std deviation = %5")
				.arg(refEntity->getName())
				.arg(compEnt->getName())
				.arg(sfName)
				.arg(mean)
				.arg(sqrt(variance)));
		}
		compEnt->setCurrentDisplayedScalarField(sfIdx);
		compEnt->showSF(sfIdx >= 0);
		compEnt->prepareDisplayForRefresh_recursive();
	}

	if (errorCount != 0)
	{
		ccLog::Error(tr("%1 error(s) occurred: refer to the Console (F8)").arg(errorCount));
	}

	MainWindow::UpdateUI();

	MainWindow::RefreshAllGLWindow(false);
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
	ccGLWindowInterface* win = getActiveGLWindow();
	if (win)
	{
		win->toggleSunLight();
		win->redraw(false);
	}
}

void MainWindow::toggleActiveWindowCustomLight()
{
	ccGLWindowInterface* win = getActiveGLWindow();
	if (win)
	{
		win->toggleCustomLight();
		win->redraw(false);
	}
}

void MainWindow::toggleActiveWindowAutoPickRotCenter(bool state)
{
	ccGLWindowInterface* win = getActiveGLWindow();
	if (win)
	{
		win->setAutoPickPivotAtCenter(state);

		//save the option
		{
			QSettings settings;
			settings.setValue(ccPS::AutoPickRotationCenter(), state);
		}
	}
}

void MainWindow::toggleActiveWindowShowCursorCoords(bool state)
{
	ccGLWindowInterface* win = getActiveGLWindow();
	if (win)
	{
		win->showCursorCoordinates(state);
	}
}

void MainWindow::toggleActiveWindowStereoVision(bool state)
{
	ccGLWindowInterface* win = getActiveGLWindow();
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

			if (	win->getStereoParams().glassType == ccGLWindowInterface::StereoParams::NVIDIA_VISION
				||	win->getStereoParams().glassType == ccGLWindowInterface::StereoParams::GENERIC_STEREO_DISPLAY)
			{
				//disable (exclusive) full screen
				m_UI->actionExclusiveFullScreen->setChecked(false);
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
				m_UI->actionEnableStereo->blockSignals(true);
				m_UI->actionEnableStereo->setChecked(false);
				m_UI->actionEnableStereo->blockSignals(false);
				return;
			}

			ccGLWindowInterface::StereoParams params = smDlg.getParameters();
			ccLog::WarningDebug("Stereo strength: " + QString::number(params.stereoStrength));

			if (!ccGLWindowInterface::StereoSupported() && !params.isAnaglyph())
			{
				ccLog::Error(tr("It seems your graphic card doesn't support Quad Buffered Stereo rendering"));
				//activation of the stereo mode failed: cancel selection
				m_UI->actionEnableStereo->blockSignals(true);
				m_UI->actionEnableStereo->setChecked(false);
				m_UI->actionEnableStereo->blockSignals(false);
				return;
			}

			//force perspective state!
			if (!win->getViewportParameters().perspectiveView)
			{
				setCenteredPerspectiveView(win, false);
			}

			if (	params.glassType == ccGLWindowInterface::StereoParams::NVIDIA_VISION
				||	params.glassType == ccGLWindowInterface::StereoParams::GENERIC_STEREO_DISPLAY)
			{
				//force (exclusive) full screen
				m_UI->actionExclusiveFullScreen->setChecked(true);
			}

			if (smDlg.updateFOV())
			{
				//set the right FOV
				double fov_deg = 2 * CCCoreLib::RadiansToDegrees( std::atan(params.screenWidth_mm / (2.0 * params.screenDistance_mm)) );
				ccLog::Print(tr("[Stereo] F.O.V. forced to %1 deg.").arg(fov_deg));
				win->setFov(fov_deg);
			}

			if (!win->enableStereoMode(params))
			{
				if (	params.glassType == ccGLWindowInterface::StereoParams::NVIDIA_VISION
					||	params.glassType == ccGLWindowInterface::StereoParams::GENERIC_STEREO_DISPLAY)
				{
					//disable (exclusive) full screen
					m_UI->actionExclusiveFullScreen->setChecked(false);
				}

				//activation of the stereo mode failed: cancel selection
				m_UI->actionEnableStereo->blockSignals(true);
				m_UI->actionEnableStereo->setChecked(false);
				m_UI->actionEnableStereo->blockSignals(false);
			}
		}
		win->redraw();
	}
}

bool MainWindow::checkStereoMode(ccGLWindowInterface* win)
{
	assert(win);

	if (win && win->getViewportParameters().perspectiveView && win->stereoModeIsEnabled())
	{
		ccGLWindowInterface::StereoParams params = win->getStereoParams();
		bool wasExclusiveFullScreen = win->exclusiveFullScreen();
		if (wasExclusiveFullScreen)
		{
			win->toggleExclusiveFullScreen(false);
		}
		win->disableStereoMode();

		if (QMessageBox::question(	this,
									tr("Stereo mode"),
									tr("Stereo-mode only works in perspective mode. Do you want to disable it?"),
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
				m_UI->actionEnableStereo->setChecked(false);
			}
			else
			{
				assert(false);
				m_UI->actionEnableStereo->blockSignals(true);
				m_UI->actionEnableStereo->setChecked(false);
				m_UI->actionEnableStereo->blockSignals(false);
			}
		}
	}

	return true;
}

void MainWindow::toggleActiveWindowCenteredPerspective()
{
	ccGLWindowInterface* win = getActiveGLWindow();
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
	ccGLWindowInterface* win = getActiveGLWindow();
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

void MainWindow::createSinglePointCloud()
{
	// ask the user to input the point coordinates
	static CCVector3d s_lastPoint(0, 0, 0);
	static size_t s_lastPointIndex = 0;
	ccAskThreeDoubleValuesDlg axisDlg("x", "y", "z", -1.0e12, 1.0e12, s_lastPoint.x, s_lastPoint.y, s_lastPoint.z, 4, tr("Point coordinates"), this);
	if (axisDlg.buttonBox->button(QDialogButtonBox::Ok))
		axisDlg.buttonBox->button(QDialogButtonBox::Ok)->setFocus();
	if (!axisDlg.exec())
		return;
	s_lastPoint.x = axisDlg.doubleSpinBox1->value();
	s_lastPoint.y = axisDlg.doubleSpinBox2->value();
	s_lastPoint.z = axisDlg.doubleSpinBox3->value();

	// create the cloud
	ccPointCloud* cloud = new ccPointCloud();
	if (!cloud->reserve(1))
	{
		delete cloud;
		ccLog::Error(tr("Not enough memory!"));
		return;
	}
	cloud->setName(tr("Point #%1").arg(++s_lastPointIndex));
	cloud->addPoint(s_lastPoint.toPC());
	cloud->setPointSize(5);

	// add it to the DB tree
	addToDB(cloud, true, true, true, true);

	// select it
	m_ccRoot->unselectAllEntities();
	setSelectedInDB(cloud, true);
}

void MainWindow::createPointCloudFromClipboard()
{
	const QClipboard* clipboard = QApplication::clipboard();
	assert(clipboard);
	const QMimeData* mimeData = clipboard->mimeData();
	if (!mimeData)
	{
		ccLog::Warning(tr("Clipboard is empty"));
		return;
	}

	if (!mimeData->hasText())
	{
		ccLog::Error("ASCII/text data expected");
		return;
	}

	// try to convert the data to a point cloud
	FileIOFilter::LoadParameters parameters;
	{
		parameters.alwaysDisplayLoadDialog = true;
		parameters.shiftHandlingMode = ccGlobalShiftManager::DIALOG_IF_NECESSARY;
		parameters.parentWidget = this;
	}

	ccHObject container;
	QByteArray data = mimeData->data("text/plain");
	CC_FILE_ERROR result = AsciiFilter().loadAsciiData(data, tr("Clipboard"), container, parameters);
	if (result != CC_FERR_NO_ERROR)
	{
		FileIOFilter::DisplayErrorMessage(result, tr("loading"), tr("from the clipboard"));
		return;
	}

	// we only expect clouds
	ccHObject::Container clouds;
	if (container.filterChildren(clouds, true, CC_TYPES::POINT_CLOUD) == 0)
	{
		assert(false);
		ccLog::Error(tr("No cloud loaded"));
		return;
	}

	// detach the clouds from the loading container
	for (ccHObject* cloud : clouds)
	{
		if (cloud)
		{
			container.removeDependencyWith(cloud);
		}
	}
	container.removeAllChildren();

	// retrieve or create the group to store the 'clipboard' clouds
	ccHObject* clipboardGroup = nullptr;
	{
		static unsigned s_clipboardGroupID = 0;

		if (s_clipboardGroupID != 0)
		{
			clipboardGroup = dbRootObject()->find(s_clipboardGroupID);
			if (nullptr == clipboardGroup)
			{
				// can't find the previous group
				s_clipboardGroupID = 0;
			}
		}

		if (s_clipboardGroupID == 0)
		{
			clipboardGroup = new ccHObject(tr("Clipboard"));
			s_clipboardGroupID = clipboardGroup->getUniqueID();
			addToDB(clipboardGroup, false, false, false, false);
		}
	}
	assert(clipboardGroup);

	bool normalsDisplayedByDefault = ccOptions::Instance().normalsDisplayedByDefault;
	for (ccHObject* cloud : clouds)
	{
		if (cloud)
		{
			clipboardGroup->addChild(cloud);
			cloud->setName(tr("Cloud #%1").arg(clipboardGroup->getChildrenNumber()));

			if (!normalsDisplayedByDefault)
			{
				// disable the normals on all loaded clouds!
				static_cast<ccGenericPointCloud*>(cloud)->showNormals(false);
			}
		}
	}

	// eventually, we can add the clouds to the DB tree
	for (size_t i = 0; i < clouds.size(); ++i)
	{
		ccHObject* cloud = clouds[i];
		if (cloud)
		{
			bool lastCloud = (i + 1 == clouds.size());
			addToDB(cloud, lastCloud, lastCloud, true, lastCloud);
		}
	}

	QMainWindow::statusBar()->showMessage(tr("%1 cloud(s) loaded from the clipboard").arg(clouds.size()), 2000);
}

void MainWindow::toggleLockRotationAxis()
{
	ccGLWindowInterface* win = getActiveGLWindow();
	if (win)
	{
		bool wasLocked = win->isRotationAxisLocked();
		bool isLocked = !wasLocked;

		static CCVector3d s_lastAxis(0.0, 0.0, 1.0);
		if (isLocked)
		{
			ccAskThreeDoubleValuesDlg axisDlg("x", "y", "z", -1.0e12, 1.0e12, s_lastAxis.x, s_lastAxis.y, s_lastAxis.z, 4, tr("Lock rotation axis"), this);
			if (axisDlg.buttonBox->button(QDialogButtonBox::Ok))
				axisDlg.buttonBox->button(QDialogButtonBox::Ok)->setFocus();
			if (!axisDlg.exec())
				return;
			s_lastAxis.x = axisDlg.doubleSpinBox1->value();
			s_lastAxis.y = axisDlg.doubleSpinBox2->value();
			s_lastAxis.z = axisDlg.doubleSpinBox3->value();
		}
		win->lockRotationAxis(isLocked, s_lastAxis);

		m_UI->actionLockRotationAxis->blockSignals(true);
		m_UI->actionLockRotationAxis->setChecked(isLocked);
		m_UI->actionLockRotationAxis->blockSignals(false);

		if (isLocked)
		{
			win->displayNewMessage(tr("[ROTATION LOCKED]"), ccGLWindowInterface::UPPER_CENTER_MESSAGE, false, 24 * 3600, ccGLWindowInterface::ROTAION_LOCK_MESSAGE);
		}
		else
		{
			win->displayNewMessage(QString(), ccGLWindowInterface::UPPER_CENTER_MESSAGE, false, 0, ccGLWindowInterface::ROTAION_LOCK_MESSAGE);
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
			ccGBLSensor* sensor = nullptr;
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
	ccGLWindowInterface* win = getActiveGLWindow();
	if (win)
	{
		win->setBubbleViewMode(true);
		win->redraw(false);
	}
}

void MainWindow::doActionDeleteShader()
{
	ccGLWindowInterface* win = getActiveGLWindow();
	if (win)
	{
		win->setShader(nullptr);
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

		CCVector3d P = center;
		CCVector3d Pshift(0, 0, 0);
		double scale = 1.0;
		bool preserveCoordinateShift = true;
		//here we must test that coordinates are not too big whatever the case because OpenGL
		//really doesn't like big ones (even if we work with GLdoubles :( ).
		if (ccGlobalShiftManager::Handle(	P,
											diag,
											ccGlobalShiftManager::DIALOG_IF_NECESSARY,
											false,
											Pshift,
											&preserveCoordinateShift,
											&scale)
			)
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
				ccConsole::Warning(tr("Entity '%1' has been translated: (%2,%3,%4) and rescaled of a factor %5 [original position will be restored when saving]").arg(obj->getName()).arg(Pshift.x,0,'f',2).arg(Pshift.y,0,'f',2).arg(Pshift.z,0,'f',2).arg(scale,0,'f',6));
			}

			//update 'global shift' and 'global scale' for ALL clouds recursively
			if (preserveCoordinateShift)
			{
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
		ccLog::Warning(tr("[MainWindow::addToDB] Internal error: no associated DB?!"));
		assert(false);
	}

	//we can now set destination display (if none already)
	if (!obj->getDisplay())
	{
		ccGLWindowInterface* activeWin = getActiveGLWindow();
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
		static_cast<ccGLWindowInterface*>(obj->getDisplay())->zoomGlobal(); //automatically calls ccGLWindowInterface::redraw
	}
	else if (autoRedraw)
	{
		obj->redrawDisplay();
	}
}

void MainWindow::onExclusiveFullScreenToggled(bool state)
{
	//we simply update the fullscreen action method icon (whatever the window)
	ccGLWindowInterface* win = getActiveGLWindow();
	
	if (win == nullptr)
		return;

	m_UI->actionExclusiveFullScreen->blockSignals(true);
	m_UI->actionExclusiveFullScreen->setChecked(win->exclusiveFullScreen());
	m_UI->actionExclusiveFullScreen->blockSignals(false);

	if (	!state
		&&	win->stereoModeIsEnabled()
		&&	(	win->getStereoParams().glassType == ccGLWindowInterface::StereoParams::NVIDIA_VISION
			||	win->getStereoParams().glassType == ccGLWindowInterface::StereoParams::GENERIC_STEREO_DISPLAY ))
	{
		//auto disable stereo mode as NVidia Vision only works in full screen mode!
		m_UI->actionEnableStereo->setChecked(false);
	}
}

ccHObject* MainWindow::loadFile(QString filename, bool silent)
{
	FileIOFilter::LoadParameters parameters;
	{
		parameters.alwaysDisplayLoadDialog = silent ? false : true;
		parameters.shiftHandlingMode = ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT;
		parameters.parentWidget = silent ? nullptr : this;
	}

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	ccHObject* newGroup = FileIOFilter::LoadFromFile(filename, parameters, result);

	return newGroup;
}

void MainWindow::addToDBAuto(const QStringList& filenames)
{
	ccGLWindowInterface* win = ccGLWindowInterface::FromEmitter(sender());
	if (win)
	{
		addToDB(filenames, QString(), win);
	}
	else
	{
		assert(false);
	}
}

void MainWindow::addToDB(	const QStringList& filenames,
							QString fileFilter/*=QString()*/,
							ccGLWindowInterface* destWin/*=nullptr*/)
{
	//to use the same 'global shift' for multiple files
	CCVector3d loadCoordinatesShift(0, 0, 0);
	bool loadCoordinatesTransEnabled = false;
	bool loadCoordinatesTransForced = false;

	FileIOFilter::LoadParameters parameters;
	{
		parameters.alwaysDisplayLoadDialog = true;
		parameters.shiftHandlingMode = ccGlobalShiftManager::DIALOG_IF_NECESSARY;
		parameters._coordinatesShift = &loadCoordinatesShift;
		parameters._coordinatesShiftEnabled = &loadCoordinatesTransEnabled;
		parameters._coordinatesShiftForced = &loadCoordinatesTransForced;
		parameters.parentWidget = this;
	}

	bool normalsDisplayedByDefault = ccOptions::Instance().normalsDisplayedByDefault;
	FileIOFilter::ResetSesionCounter();

	for ( const QString &filename : filenames )
	{
		CC_FILE_ERROR result = CC_FERR_NO_ERROR;
		ccHObject* newGroup = FileIOFilter::LoadFromFile(filename, parameters, result, fileFilter);

		if (newGroup)
		{
			if (!normalsDisplayedByDefault)
			{
				//disable the normals on all loaded clouds!
				ccHObject::Container clouds;
				newGroup->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD);
				for (ccHObject* cloud : clouds)
				{
					if (cloud)
					{
						static_cast<ccGenericPointCloud*>(cloud)->showNormals(false);
					}
				}
			}
			
			if (destWin)
			{
				newGroup->setDisplay_recursive(destWin);
			}
			addToDB(newGroup, true, true, false);

			m_recentFiles->addFilePath( filename );
		}

		if (result == CC_FERR_CANCELED_BY_USER)
		{
			//stop importing the file if the user has cancelled the current process!
			break;
		}
	}

	QMainWindow::statusBar()->showMessage(tr("%1 file(s) loaded").arg(filenames.size()),2000);
}

void MainWindow::handleNewLabel(ccHObject* entity)
{
	if (entity)
	{
		addToDB(entity, false, true, false, false);
	}
	else
	{
		assert(false);
	}
}

void MainWindow::forceConsoleDisplay()
{
	//if the console is hidden, we autoamtically display it!
	if (m_UI->DockableConsole && m_UI->DockableConsole->isHidden())
	{
		m_UI->DockableConsole->show();
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
	{
		return;
	}
	
	QMessageBox message_box( QMessageBox::Question,
							 tr("Close all"),
							 tr("Are you sure you want to remove all loaded entities?"),
							 QMessageBox::Yes | QMessageBox::No,
							 this );
	
	if (message_box.exec() == QMessageBox::No)
	{
		return;
	}
	
	m_ccRoot->unloadAll();

	redrawAll(false);
}

void MainWindow::doActionLoadFile()
{
	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::LoadFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
	QString currentOpenDlgFilter = settings.value(ccPS::SelectedInputFilter(), BinFilter::GetFileFilter()).toString();

	// Add all available file I/O filters (with import capabilities)
	const QStringList filterStrings = FileIOFilter::ImportFilterList();
	const QString &allFilter = filterStrings.at( 0 );
	
	if ( !filterStrings.contains( currentOpenDlgFilter ) )
	{
		currentOpenDlgFilter = allFilter;
	}
	
	//file choosing dialog
	QStringList selectedFiles = QFileDialog::getOpenFileNames(	this,
																tr("Open file(s)"),
																currentPath,
																filterStrings.join(s_fileFilterSeparator),
																&currentOpenDlgFilter,
																CCFileDialogOptions());
	if (selectedFiles.isEmpty())
		return;

	//save last loading parameters
	currentPath = QFileInfo(selectedFiles[0]).absolutePath();
	settings.setValue(ccPS::CurrentPath(),currentPath);
	settings.setValue(ccPS::SelectedInputFilter(),currentOpenDlgFilter);
	settings.endGroup();

	if (currentOpenDlgFilter == allFilter)
	{
		currentOpenDlgFilter.clear(); //this way FileIOFilter will try to guess the file type automatically!
	}
	
	//load files
	addToDB(selectedFiles, currentOpenDlgFilter);
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
	if (!haveSelection())
		return;

	ccHObject clouds("clouds");
	ccHObject meshes("meshes");
	ccHObject images("images");
	ccHObject polylines("polylines");
	ccHObject other("other");
	ccHObject otherSerializable("serializable");
	ccHObject::Container entitiesToDispatch;
	entitiesToDispatch.insert(entitiesToDispatch.begin(), m_selectedEntities.begin(), m_selectedEntities.end());
	ccHObject entitiesToSave;
	while (!entitiesToDispatch.empty())
	{
		ccHObject* child = entitiesToDispatch.back();
		entitiesToDispatch.pop_back();

		if (child->isA(CC_TYPES::HIERARCHY_OBJECT))
		{
			for (unsigned j = 0; j < child->getChildrenNumber(); ++j)
				entitiesToDispatch.push_back(child->getChild(j));
		}
		else
		{
			//we put the entity in the container corresponding to its type
			ccHObject* dest = nullptr;
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

			//we don't want double insertions if the user has highlighted both the father and the child
			if (!dest->find(child->getUniqueID()))
			{
				dest->addChild(child, ccHObject::DP_NONE);
				entitiesToSave.addChild(child, ccHObject::DP_NONE);
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
		ccConsole::Error(tr("Can't save selected entity(ies) this way!"));
		return;
	}

	//we set up the right file filters, depending on the selected
	//entities type (cloud, mesh, etc.).
	QStringList fileFilters;
	{
		for ( const FileIOFilter::Shared& filter : FileIOFilter::GetFilters() )
		{
			bool atLeastOneExclusive = false;

			//can this filter export one or several clouds?
			bool canExportClouds = true;
			if (hasCloud)
			{
				bool isExclusive = true;
				bool multiple = false;
				canExportClouds = (		filter->canSave(CC_TYPES::POINT_CLOUD, multiple, isExclusive)
									&&	(multiple || clouds.getChildrenNumber() == 1) );
				atLeastOneExclusive |= isExclusive;
			}

			//can this filter export one or several meshes?
			bool canExportMeshes = true;
			if (hasMesh)
			{
				bool isExclusive = true;
				bool multiple = false;
				canExportMeshes = (		filter->canSave(CC_TYPES::MESH, multiple, isExclusive)
									&&	(multiple || meshes.getChildrenNumber() == 1) );
				atLeastOneExclusive |= isExclusive;
			}

			//can this filter export one or several polylines?
			bool canExportPolylines = true;
			if (hasPolylines)
			{
				bool isExclusive = true;
				bool multiple = false;
				canExportPolylines = (	filter->canSave(CC_TYPES::POLY_LINE, multiple, isExclusive)
									&&	(multiple || polylines.getChildrenNumber() == 1) );
				atLeastOneExclusive |= isExclusive;
			}

			//can this filter export one or several images?
			bool canExportImages = true;
			if (hasImages)
			{
				bool isExclusive = true;
				bool multiple = false;
				canExportImages = (		filter->canSave(CC_TYPES::IMAGE, multiple, isExclusive)
									&&	(multiple || images.getChildrenNumber() == 1) );
				atLeastOneExclusive |= isExclusive;
			}

			//can this filter export one or several other serializable entities?
			bool canExportSerializables = true;
			if (hasSerializable)
			{
				//check if all entities have the same type
				{
					CC_CLASS_ENUM firstClassID = otherSerializable.getChild(0)->getUniqueID();
					for (unsigned j = 1; j < otherSerializable.getChildrenNumber(); ++j)
					{
						if (otherSerializable.getChild(j)->getUniqueID() != firstClassID)
						{
							//we add a virtual second 'stdSaveType' so as to properly handle exlusivity
							++stdSaveTypes;
							break;
						}
					}
				}

				for (unsigned j = 0; j < otherSerializable.getChildrenNumber(); ++j)
				{
					ccHObject* child = otherSerializable.getChild(j);
					bool isExclusive = true;
					bool multiple = false;
					canExportSerializables &= (		filter->canSave(child->getClassID(), multiple, isExclusive)
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
				for (int j = 0; j < ff.size(); ++j)
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
		selectedFilter = settings.value(ccPS::SelectedOutputFilterCloud(), selectedFilter).toString();
	else if (hasMesh)
		selectedFilter = settings.value(ccPS::SelectedOutputFilterMesh(), selectedFilter).toString();
	else if (hasImages)
		selectedFilter = settings.value(ccPS::SelectedOutputFilterImage(), selectedFilter).toString();
	else if (hasPolylines)
		selectedFilter = settings.value(ccPS::SelectedOutputFilterPoly(), selectedFilter).toString();

	//default output path (+ filename)
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
	QString fullPathName = currentPath;
	
	if (haveOneSelection())
	{
		//hierarchy objects have generally as name: 'filename.ext (fullpath)'
		//so we must only take the first part! (otherwise this type of name
		//with a path inside disturbs QFileDialog a lot ;))
		QString defaultFileName(m_selectedEntities.front()->getName());
		if (m_selectedEntities.front()->isA(CC_TYPES::HIERARCHY_OBJECT))
		{
			QStringList parts = defaultFileName.split(' ', QString::SkipEmptyParts);
			if (!parts.empty())
			{
				defaultFileName = parts[0];
			}
		}

		//we remove the extension
		defaultFileName = QFileInfo(defaultFileName).completeBaseName();

		if (!IsValidFileName(defaultFileName))
		{
			ccLog::Warning(tr("[I/O] First entity's name would make an invalid filename! Can't use it..."));
			defaultFileName = "project";
		}

		fullPathName += QString("/") + defaultFileName;
	}

	//ask the user for the output filename
	QString selectedFilename = QFileDialog::getSaveFileName(this,
															tr("Save file"),
															fullPathName,
															fileFilters.join(s_fileFilterSeparator),
															&selectedFilter,
															CCFileDialogOptions());

	if (selectedFilename.isEmpty())
	{
		//process cancelled by the user
		return;
	}

	//ignored items
	if (hasOther)
	{
		ccConsole::Warning(tr("[I/O] The following selected entities won't be saved:"));
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
		if ( haveOneSelection() )
		{
			result = FileIOFilter::SaveToFile(m_selectedEntities.front(), selectedFilename, parameters, selectedFilter);
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
				ccLog::Warning(tr("[I/O] None of the selected entities can be saved this way..."));
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
		//		ccConsole::Warning(tr("[I/O] The following selected entites won't be saved:")); //display this warning only if not already done
		//	for (unsigned i = 0; i < otherSerializable.getChildrenNumber(); ++i)
		//		ccConsole::Warning(tr("\t- %1").arg(otherSerializable.getChild(i)->getName()));
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

	if (result == CC_FERR_NO_ERROR && selectedFilter == BinFilter::GetFileFilter())
	{
		//only for BIN files: display the compatible CC version
		short fileVersion = BinFilter::GetLastSavedFileVersion();
		if (0 != fileVersion)
		{
			QString minCCVersion = ccApplication::GetMinCCVersionForFileVersion(fileVersion);
			ccLog::Print(QString("This file can be loaded by CloudCompare version %1 and later").arg(minCCVersion));
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

void MainWindow::doActionSaveProject()
{
	if (!m_ccRoot || !m_ccRoot->getRootEntity())
	{
		assert(false);
		return;
	}

	ccHObject* rootEntity = m_ccRoot->getRootEntity();
	if (rootEntity->getChildrenNumber() == 0)
	{
		return;
	}

	//default output path (+ filename)
	QSettings settings;
	settings.beginGroup(ccPS::SaveFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
	ccLog::PrintDebug(currentPath);
	QString fullPathName = currentPath;

	static QString s_previousProjectName{ "project" };
	QString defaultFileName = s_previousProjectName;
	if (rootEntity->getChildrenNumber() == 1)
	{
		// If there's only on top entity, we can try to use its name as the project name.
		ccHObject* topEntity = rootEntity->getChild(0);
		defaultFileName = topEntity->getName();
		if (topEntity->isA(CC_TYPES::HIERARCHY_OBJECT))
		{
			// Hierarchy objects have generally as name: 'filename.ext (fullpath)'
			// so we must only take the first part! (otherwise this type of name
			// with a path inside disturbs the QFileDialog a lot ;))
			QStringList parts = defaultFileName.split(' ', QString::SkipEmptyParts);
			if (!parts.empty())
			{
				defaultFileName = parts[0];
			}
		}

		//we remove the extension
		defaultFileName = QFileInfo(defaultFileName).completeBaseName();

		if (!IsValidFileName(defaultFileName))
		{
			ccLog::Warning(tr("[I/O] Top entity's name would make an invalid filename! Can't use it..."));
			defaultFileName = "project";
		}
	}
	fullPathName += QString("/") + defaultFileName;

	QString binFilter = BinFilter::GetFileFilter();

	//ask the user for the output filename
	QString selectedFilename = QFileDialog::getSaveFileName(this,
		tr("Save file"),
		fullPathName,
		binFilter,
		&binFilter,
		CCFileDialogOptions());

	if (selectedFilename.isEmpty())
	{
		//process cancelled by the user
		return;
	}

	FileIOFilter::SaveParameters parameters;
	{
		parameters.alwaysDisplaySaveDialog = true;
		parameters.parentWidget = this;
	}

	CC_FILE_ERROR result = FileIOFilter::SaveToFile(rootEntity->getChildrenNumber() == 1 ? rootEntity->getChild(0) : rootEntity, selectedFilename, parameters, binFilter);

	if (result == CC_FERR_NO_ERROR)
	{
		//only for BIN files: display the compatible CC version
		short fileVersion = BinFilter::GetLastSavedFileVersion();
		if (0 != fileVersion)
		{
			QString minCCVersion = ccApplication::GetMinCCVersionForFileVersion(fileVersion);
			ccLog::Print(QString("This file can be loaded by CloudCompare version %1 and later").arg(minCCVersion));
		}
	}

	//we update the current 'save' path
	QFileInfo fi(selectedFilename);
	s_previousProjectName = fi.fileName();
	currentPath = fi.absolutePath();
	settings.setValue(ccPS::CurrentPath(), currentPath);
	settings.endGroup();
}

void MainWindow::on3DViewActivated(QMdiSubWindow* mdiWin)
{
	ccGLWindowInterface* win = mdiWin ? ccGLWindowInterface::FromWidget(mdiWin->widget()) : nullptr;
	if (win)
	{
		updateViewModePopUpMenu(win);
		updatePivotVisibilityPopUpMenu(win);

		m_UI->actionLockRotationAxis->blockSignals(true);
		m_UI->actionLockRotationAxis->setChecked(win->isRotationAxisLocked());
		m_UI->actionLockRotationAxis->blockSignals(false);

		m_UI->actionEnableStereo->blockSignals(true);
		m_UI->actionEnableStereo->setChecked(win->stereoModeIsEnabled());
		m_UI->actionEnableStereo->blockSignals(false);

		m_UI->actionExclusiveFullScreen->blockSignals(true);
		m_UI->actionExclusiveFullScreen->setChecked(win->exclusiveFullScreen());
		m_UI->actionExclusiveFullScreen->blockSignals(false);

		m_UI->actionShowCursor3DCoordinates->blockSignals(true);
		m_UI->actionShowCursor3DCoordinates->setChecked(win->cursorCoordinatesShown());
		m_UI->actionShowCursor3DCoordinates->blockSignals(false);

		m_UI->actionAutoPickRotationCenter->blockSignals(true);
		m_UI->actionAutoPickRotationCenter->setChecked(win->autoPickPivotAtCenter());
		m_UI->actionAutoPickRotationCenter->blockSignals(false);
	}

	m_UI->actionLockRotationAxis->setEnabled(win != nullptr);
	m_UI->actionEnableStereo->setEnabled(win != nullptr);
	m_UI->actionExclusiveFullScreen->setEnabled(win != nullptr);
}

void MainWindow::updateViewModePopUpMenu(ccGLWindowInterface* win)
{
	if (!m_viewModePopupButton)
		return;

	//update the view mode pop-up 'top' icon
	if (win)
	{
		bool objectCentered = true;
		bool perspectiveEnabled = win->getPerspectiveState(objectCentered);

		QAction* currentModeAction = nullptr;
		if (!perspectiveEnabled)
		{
			currentModeAction = m_UI->actionSetOrthoView;
		}
		else if (objectCentered)
		{
			currentModeAction = m_UI->actionSetCenteredPerspectiveView;
		}
		else
		{
			currentModeAction = m_UI->actionSetViewerPerspectiveView;
		}

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

void MainWindow::updatePivotVisibilityPopUpMenu(ccGLWindowInterface* win)
{
	if (!m_pivotVisibilityPopupButton)
		return;

	//update the pivot visibility pop-up 'top' icon
	if (win)
	{
		QAction* visibilityAction = nullptr;
		switch(win->getPivotVisibility())
		{
		case ccGLWindowInterface::PIVOT_HIDE:
			visibilityAction = m_UI->actionSetPivotOff;
			break;
		case ccGLWindowInterface::PIVOT_SHOW_ON_MOVE:
			visibilityAction = m_UI->actionSetPivotRotationOnly;
			break;
		case ccGLWindowInterface::PIVOT_ALWAYS_SHOW:
			visibilityAction = m_UI->actionSetPivotAlwaysOn;
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
	ccGLWindowInterface* active3DView = getActiveGLWindow();
	bool hasMdiChild = (active3DView != nullptr);
	int mdiChildCount = getGLWindowCount();
	bool hasLoadedEntities = (m_ccRoot && m_ccRoot->getRootEntity() && m_ccRoot->getRootEntity()->getChildrenNumber() != 0);
	bool hasSelectedEntities = (m_ccRoot && m_ccRoot->countSelectedEntities() > 0);

	//General Menu
	m_UI->menuEdit->setEnabled(true/*hasSelectedEntities*/);
	m_UI->menuTools->setEnabled(true/*hasSelectedEntities*/);

	//3D Views Menu
	m_UI->actionClose3DView    ->setEnabled(hasMdiChild);
	m_UI->actionCloseAll3DViews->setEnabled(mdiChildCount != 0);
	m_UI->actionTile3DViews    ->setEnabled(mdiChildCount > 1);
	m_UI->actionCascade3DViews ->setEnabled(mdiChildCount > 1);
	m_UI->actionNext3DView     ->setEnabled(mdiChildCount > 1);
	m_UI->actionPrevious3DView ->setEnabled(mdiChildCount > 1);

	//Shaders & Filters display Menu
	bool shadersEnabled = (active3DView ? active3DView->areShadersEnabled() : false);
	m_UI->actionLoadShader->setEnabled(shadersEnabled);
	m_UI->actionDeleteShader->setEnabled(shadersEnabled);

	//View Menu
	m_UI->toolBarView->setEnabled(hasMdiChild);

	//oher actions
	m_UI->actionSegment->setEnabled(hasMdiChild && hasSelectedEntities);
	m_UI->actionTranslateRotate->setEnabled(hasMdiChild && hasSelectedEntities);
	m_UI->actionPointPicking->setEnabled(hasMdiChild && hasLoadedEntities);
	m_UI->actionTestFrameRate->setEnabled(hasMdiChild);
	m_UI->actionRenderToFile->setEnabled(hasMdiChild);
	m_UI->actionToggleSunLight->setEnabled(hasMdiChild);
	m_UI->actionToggleCustomLight->setEnabled(hasMdiChild);
	m_UI->actionToggleCenteredPerspective->setEnabled(hasMdiChild);
	m_UI->actionToggleViewerBasedPerspective->setEnabled(hasMdiChild);

	//plugins
	m_pluginUIManager->updateMenus();
}

void MainWindow::update3DViewsMenu()
{
	m_UI->menu3DViews->clear();
	m_UI->menu3DViews->addAction(m_UI->actionNew3DView);
	m_UI->menu3DViews->addSeparator();
	m_UI->menu3DViews->addAction(m_UI->actionZoomIn);
	m_UI->menu3DViews->addAction(m_UI->actionZoomOut);
	m_UI->menu3DViews->addSeparator();
	m_UI->menu3DViews->addAction(m_UI->actionClose3DView);
	m_UI->menu3DViews->addAction(m_UI->actionCloseAll3DViews);
	m_UI->menu3DViews->addSeparator();
	m_UI->menu3DViews->addAction(m_UI->actionTile3DViews);
	m_UI->menu3DViews->addAction(m_UI->actionCascade3DViews);
	m_UI->menu3DViews->addSeparator();
	m_UI->menu3DViews->addAction(m_UI->actionNext3DView);
	m_UI->menu3DViews->addAction(m_UI->actionPrevious3DView);

	QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
	if (!windows.isEmpty())
	{
		//Dynamic Separator
		QAction* separator = new QAction(this);
		separator->setSeparator(true);
		m_UI->menu3DViews->addAction(separator);

		int i = 0;
		
		for ( QMdiSubWindow* window : windows )
		{
			ccGLWindowInterface *child = ccGLWindowInterface::FromWidget(window->widget());

			QString text = QString("&%1 %2").arg(++i).arg(child->getWindowTitle());
			QAction *action = m_UI->menu3DViews->addAction(text);
			
			action->setCheckable(true);
			action->setChecked(child == getActiveGLWindow());
			
			connect(action, &QAction::triggered, this, [=] () {
				setActiveSubWindow( window );
			} );
		}
	}
}

void MainWindow::setActiveSubWindow(QWidget *window)
{
	if (!window || !m_mdiArea)
		return;
	m_mdiArea->setActiveSubWindow(qobject_cast<QMdiSubWindow*>(window));
}

void MainWindow::redrawAll(bool only2D/*=false*/)
{
	for (QMdiSubWindow* window : m_mdiArea->subWindowList())
	{
		ccGLWindowInterface::FromWidget(window->widget())->redraw(only2D);
	}
}

void MainWindow::refreshAll(bool only2D/*=false*/)
{
	for (QMdiSubWindow* window : m_mdiArea->subWindowList())
	{
		ccGLWindowInterface::FromWidget(window->widget())->refresh(only2D);
	}
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
	{
		m_ccRoot->getSelectedEntities(m_selectedEntities, CC_TYPES::OBJECT, &selInfo);
	}

	enableUIItems(selInfo);
}

void MainWindow::enableAll()
{
	for ( QMdiSubWindow* window : m_mdiArea->subWindowList() )
	{
		window->setEnabled( true );
	}
}

void MainWindow::disableAll()
{
	for ( QMdiSubWindow* window : m_mdiArea->subWindowList() )
	{
		window->setEnabled( false );
	}
}

void MainWindow::disableAllBut(ccGLWindowInterface* win)
{
	//we disable all other windows
	for ( QMdiSubWindow* window : m_mdiArea->subWindowList() )
	{
		if (ccGLWindowInterface::FromWidget(window->widget()) != win)
		{
			window->setEnabled(false);
		}
	}
}

void MainWindow::enableUIItems(dbTreeSelectionInfo& selInfo)
{
	bool dbIsEmpty = (!m_ccRoot || !m_ccRoot->getRootEntity() || m_ccRoot->getRootEntity()->getChildrenNumber() == 0);
	bool atLeastOneEntity = (selInfo.selCount > 0);
	bool atLeastOneCloud = (selInfo.cloudCount > 0);
	bool atLeastOneMesh = (selInfo.meshCount > 0);
	bool atLeastOnePrimitive = (selInfo.primitiveCount > 0);
	//bool atLeastOneOctree = (selInfo.octreeCount > 0);
	bool atLeastOneNormal = (selInfo.normalsCount > 0);
	bool atLeastOneColor = (selInfo.colorCount > 0);
	bool atLeastOneSF = (selInfo.sfCount > 0);
	bool atLeastOneGrid = (selInfo.gridCound > 0);

	//bool atLeastOneSensor = (selInfo.sensorCount > 0);
	bool atLeastOneGBLSensor = (selInfo.gblSensorCount > 0);
	bool atLeastOneCameraSensor = (selInfo.cameraSensorCount > 0);
	bool atLeastOnePolyline = (selInfo.polylineCount > 0);
	bool activeWindow = (getActiveGLWindow() != nullptr);

	//menuEdit->setEnabled(atLeastOneEntity);
	//menuTools->setEnabled(atLeastOneEntity);

	m_UI->actionTracePolyline->setEnabled(!dbIsEmpty);
	m_UI->actionZoomAndCenter->setEnabled(atLeastOneEntity && activeWindow);
	m_UI->actionSave->setEnabled(atLeastOneEntity);
	m_UI->actionSaveProject->setEnabled(!dbIsEmpty);
	m_UI->actionClone->setEnabled(atLeastOneEntity);
	m_UI->actionDelete->setEnabled(atLeastOneEntity);
	m_UI->actionExportCoordToSF->setEnabled(atLeastOneEntity);
	m_UI->actionExportNormalToSF->setEnabled(atLeastOneNormal);
	m_UI->actionSegment->setEnabled(atLeastOneEntity && activeWindow);
	m_UI->actionTranslateRotate->setEnabled(atLeastOneEntity && activeWindow);
	m_UI->actionShowDepthBuffer->setEnabled(atLeastOneGBLSensor);
	m_UI->actionExportDepthBuffer->setEnabled(atLeastOneGBLSensor);
	m_UI->actionComputePointsVisibility->setEnabled(atLeastOneGBLSensor);
	m_UI->actionResampleWithOctree->setEnabled(atLeastOneCloud);
	m_UI->actionApplyScale->setEnabled(atLeastOneCloud || atLeastOneMesh || atLeastOnePolyline);
	m_UI->actionApplyTransformation->setEnabled(atLeastOneEntity);
	m_UI->actionComputeOctree->setEnabled(atLeastOneCloud || atLeastOneMesh);
	m_UI->actionComputeNormals->setEnabled(atLeastOneCloud || atLeastOneMesh);
	m_UI->actionChangeColorLevels->setEnabled(atLeastOneCloud || atLeastOneMesh);
	m_UI->actionEditGlobalShiftAndScale->setEnabled(atLeastOneCloud || atLeastOneMesh || atLeastOnePolyline);
	m_UI->actionCrop->setEnabled(atLeastOneCloud || atLeastOneMesh);
	m_UI->actionSetUniqueColor->setEnabled(atLeastOneEntity/*atLeastOneCloud || atLeastOneMesh*/); //DGM: we can set color to a group now!
	m_UI->actionSetColorGradient->setEnabled(atLeastOneCloud || atLeastOneMesh);
	m_UI->actionColorize->setEnabled(atLeastOneEntity/*atLeastOneCloud || atLeastOneMesh*/); //DGM: we can set color to a group now!
	m_UI->actionDeleteScanGrid->setEnabled(atLeastOneGrid);

	m_UI->actionScalarFieldFromColor->setEnabled(atLeastOneEntity && atLeastOneColor);
	m_UI->actionComputeMeshAA->setEnabled(atLeastOneCloud);
	m_UI->actionComputeMeshLS->setEnabled(atLeastOneCloud);
	m_UI->actionMeshScanGrids->setEnabled(atLeastOneGrid);
	//actionComputeQuadric3D->setEnabled(atLeastOneCloud);
	m_UI->actionComputeBestFitBB->setEnabled(atLeastOneEntity);
	m_UI->actionComputeGeometricFeature->setEnabled(atLeastOneCloud);
	m_UI->actionRemoveDuplicatePoints->setEnabled(atLeastOneCloud);
	m_UI->actionFitPlane->setEnabled(atLeastOneEntity);
	m_UI->actionFitPlaneProxy->setEnabled(atLeastOneEntity);
	m_UI->actionFitSphere->setEnabled(atLeastOneCloud);
	m_UI->actionFitCircle->setEnabled(atLeastOneCloud);
	m_UI->actionLevel->setEnabled(atLeastOneEntity);
	m_UI->actionFitFacet->setEnabled(atLeastOneEntity);
	m_UI->actionFitQuadric->setEnabled(atLeastOneCloud);
	m_UI->actionSubsample->setEnabled(atLeastOneCloud);

	m_UI->actionSNETest->setEnabled(atLeastOneCloud);
	m_UI->actionExportCloudInfo->setEnabled(atLeastOneEntity);
	m_UI->actionExportPlaneInfo->setEnabled(atLeastOneEntity);

	m_UI->actionFilterByValue->setEnabled(atLeastOneSF);
	m_UI->actionConvertToRGB->setEnabled(atLeastOneSF);
	m_UI->actionConvertToRandomRGB->setEnabled(atLeastOneSF);
	m_UI->actionRenameSF->setEnabled(atLeastOneSF);
	m_UI->actionAddIdField->setEnabled(atLeastOneCloud);
	m_UI->actionSplitCloudUsingSF->setEnabled(atLeastOneSF);
	m_UI->actionComputeStatParams->setEnabled(atLeastOneSF);
	m_UI->actionComputeStatParams2->setEnabled(atLeastOneSF);
	m_UI->actionShowHistogram->setEnabled(atLeastOneSF);
	m_UI->actionGaussianFilter->setEnabled(atLeastOneSF);
	m_UI->actionBilateralFilter->setEnabled(atLeastOneSF);
	m_UI->actionDeleteScalarField->setEnabled(atLeastOneSF);
	m_UI->actionDeleteAllSF->setEnabled(atLeastOneSF);
	m_UI->actionMultiplySF->setEnabled(/*TODO: atLeastOneSF*/false);
	m_UI->actionSFGradient->setEnabled(atLeastOneSF);
	m_UI->actionSetSFAsCoord->setEnabled(atLeastOneSF && atLeastOneCloud);
	m_UI->actionInterpolateSFs->setEnabled(atLeastOneCloud || atLeastOneMesh);

	m_UI->actionSamplePointsOnMesh->setEnabled(atLeastOneMesh);
	m_UI->actionMeasureMeshSurface->setEnabled(atLeastOneMesh);
	m_UI->actionMeasureMeshVolume->setEnabled(atLeastOneMesh);
	m_UI->actionFlagMeshVertices->setEnabled(atLeastOneMesh);
	m_UI->actionSmoothMeshLaplacian->setEnabled(atLeastOneMesh);
	m_UI->actionConvertTextureToColor->setEnabled(atLeastOneMesh);
	m_UI->actionSubdivideMesh->setEnabled(atLeastOneMesh);
	m_UI->actionFlipMeshTriangles->setEnabled(atLeastOneMesh);
	m_UI->actionDistanceToBestFitQuadric3D->setEnabled(atLeastOneCloud);
	m_UI->actionDistanceMap->setEnabled(atLeastOneMesh || atLeastOneCloud);

	m_UI->menuMeshScalarField->setEnabled(atLeastOneSF && atLeastOneMesh);
	//actionSmoothMeshSF->setEnabled(atLeastOneSF && atLeastOneMesh);
	//actionEnhanceMeshSF->setEnabled(atLeastOneSF && atLeastOneMesh);

	m_UI->actionOrientNormalsMST->setEnabled(atLeastOneCloud && atLeastOneNormal);
	m_UI->actionOrientNormalsFM->setEnabled(atLeastOneCloud && atLeastOneNormal);
	m_UI->actionShiftPointsAlongNormals->setEnabled(atLeastOneCloud && atLeastOneNormal);
	m_UI->actionClearNormals->setEnabled(atLeastOneNormal);
	m_UI->actionInvertNormals->setEnabled(atLeastOneNormal);
	m_UI->actionConvertNormalToHSV->setEnabled(atLeastOneNormal);
	m_UI->actionConvertNormalToDipDir->setEnabled(atLeastOneNormal);
	m_UI->actionClearColor->setEnabled(atLeastOneColor);
	m_UI->actionRGBToGreyScale->setEnabled(atLeastOneColor);
	m_UI->actionEnhanceRGBWithIntensities->setEnabled(atLeastOneColor);
	m_UI->actionRGBGaussianFilter->setEnabled(atLeastOneColor);
	m_UI->actionRGBBilateralFilter->setEnabled(atLeastOneColor);
	m_UI->actionRGBMeanFilter->setEnabled(atLeastOneColor);
	m_UI->actionRGBMedianFilter->setEnabled(atLeastOneColor);
	m_UI->actionColorFromScalarField->setEnabled(atLeastOneSF);
	// == 1
	bool exactlyOneEntity = (selInfo.selCount == 1);
	bool exactlyOneGroup = (selInfo.groupCount == 1);
	bool exactlyOneCloud = (selInfo.cloudCount == 1);
	bool exactlyOneMesh = (selInfo.meshCount == 1);
	bool exactlyOneSF = (selInfo.sfCount == 1);
	bool exactlyOneSensor = (selInfo.sensorCount == 1);
	bool exactlyOneCameraSensor = (selInfo.cameraSensorCount == 1);

	m_UI->actionConvertPolylinesToMesh->setEnabled(atLeastOnePolyline || exactlyOneGroup);
	m_UI->actionSamplePointsOnPolyline->setEnabled(atLeastOnePolyline);
	m_UI->actionSmoothPolyline->setEnabled(atLeastOnePolyline);

	m_UI->actionMeshTwoPolylines->setEnabled(selInfo.selCount == 2 && selInfo.polylineCount == 2);
	m_UI->actionCreateSurfaceBetweenTwoPolylines->setEnabled(m_UI->actionMeshTwoPolylines->isEnabled()); //clone of actionMeshTwoPolylines
	m_UI->actionModifySensor->setEnabled(exactlyOneSensor);
	m_UI->actionComputeDistancesFromSensor->setEnabled(atLeastOneCameraSensor || atLeastOneGBLSensor);
	m_UI->actionComputeScatteringAngles->setEnabled(exactlyOneSensor);
	m_UI->actionViewFromSensor->setEnabled(exactlyOneSensor);
	m_UI->actionCreateGBLSensor->setEnabled(atLeastOneCloud);
	m_UI->actionCreateCameraSensor->setEnabled(selInfo.selCount <= 1); //free now
	m_UI->actionProjectUncertainty->setEnabled(exactlyOneCameraSensor);
	m_UI->actionCheckPointsInsideFrustum->setEnabled(exactlyOneCameraSensor);
	m_UI->actionLabelConnectedComponents->setEnabled(atLeastOneCloud);
	m_UI->actionSORFilter->setEnabled(atLeastOneCloud);
	m_UI->actionNoiseFilter->setEnabled(atLeastOneCloud);
	m_UI->actionUnroll->setEnabled(exactlyOneEntity);
	m_UI->actionStatisticalTest->setEnabled(exactlyOneEntity && exactlyOneSF);
	m_UI->actionAddConstantSF->setEnabled(exactlyOneCloud || exactlyOneMesh);
	m_UI->actionAddClassificationSF->setEnabled(exactlyOneCloud || exactlyOneMesh);
	m_UI->actionEditGlobalScale->setEnabled(exactlyOneCloud || exactlyOneMesh);
	m_UI->actionComputeKdTree->setEnabled(exactlyOneCloud || exactlyOneMesh);
	m_UI->actionSetSFsAsNormal->setEnabled(exactlyOneCloud || exactlyOneMesh);
	m_UI->actionShowWaveDialog->setEnabled(exactlyOneCloud);
	m_UI->actionCompressFWFData->setEnabled(atLeastOneCloud);

	m_UI->actionKMeans->setEnabled(/*TODO: exactlyOneEntity && exactlyOneSF*/false);
	m_UI->actionFrontPropagation->setEnabled(/*TODO: exactlyOneEntity && exactlyOneSF*/false);

	//actionCreatePlane->setEnabled(true);
	m_UI->actionEditPlane->setEnabled(selInfo.planeCount == 1);
	m_UI->actionFlipPlane->setEnabled(selInfo.planeCount != 0);
	m_UI->actionComparePlanes->setEnabled(selInfo.planeCount == 2);

	m_UI->actionPromoteCircleToCylinder->setEnabled((selInfo.selCount == 1) && (selInfo.circleCount == 1));

	m_UI->actionFindBiggestInnerRectangle->setEnabled(exactlyOneCloud);

	m_UI->menuActiveScalarField->setEnabled((exactlyOneCloud || exactlyOneMesh) && selInfo.sfCount > 0);
	m_UI->actionCrossSection->setEnabled(atLeastOneCloud || atLeastOneMesh || (selInfo.groupCount != 0));
	m_UI->actionExtractSections->setEnabled(atLeastOneCloud);
	m_UI->actionRasterize->setEnabled(exactlyOneCloud);
	m_UI->actionCompute2HalfDimVolume->setEnabled(selInfo.cloudCount == selInfo.selCount && selInfo.cloudCount >= 1 && selInfo.cloudCount <= 2); //one or two clouds!

	m_UI->actionPointListPicking->setEnabled(exactlyOneCloud || exactlyOneMesh);

	// == 2
	bool exactlyTwoEntities = (selInfo.selCount == 2);
	bool exactlyTwoClouds = (selInfo.cloudCount == 2);
	//bool exactlyTwoSF = (selInfo.sfCount == 2);

	m_UI->actionRegister->setEnabled(exactlyTwoEntities);
	m_UI->actionInterpolateColors->setEnabled(exactlyTwoEntities && atLeastOneColor);
	m_UI->actionPointPairsAlign->setEnabled(atLeastOneEntity);
	m_UI->actionBBCenterToOrigin->setEnabled(atLeastOneEntity);
	m_UI->actionBBMinCornerToOrigin->setEnabled(atLeastOneEntity);
	m_UI->actionBBMaxCornerToOrigin->setEnabled(atLeastOneEntity);

	m_UI->actionAlign->setEnabled(exactlyTwoEntities); //Aurelien BEY le 13/11/2008
	m_UI->actionCloudCloudDist->setEnabled(exactlyTwoClouds);
	m_UI->actionCloudMeshDist->setEnabled(exactlyTwoEntities && atLeastOneMesh);
	m_UI->actionCloudPrimitiveDist->setEnabled(atLeastOneCloud && (atLeastOnePrimitive || atLeastOnePolyline));
	m_UI->actionCPS->setEnabled(exactlyTwoClouds);
	m_UI->actionScalarFieldArithmetic->setEnabled(exactlyOneEntity && atLeastOneSF);

	//>1
	bool atLeastTwoEntities = (selInfo.selCount > 1);

	m_UI->actionMerge->setEnabled(atLeastTwoEntities);
	m_UI->actionMatchBBCenters->setEnabled(atLeastTwoEntities);
	m_UI->actionMatchScales->setEnabled(atLeastTwoEntities);

	//standard plugins
	m_pluginUIManager->handleSelectionChanged();
}

void MainWindow::echoMouseWheelRotate(float wheelDelta_deg)
{
	if (!m_UI->actionEnableCameraLink->isChecked())
		return;

	ccGLWindowInterface* sendingWindow = ccGLWindowInterface::FromEmitter(sender());
	if (!sendingWindow)
		return;

	for ( QMdiSubWindow* window : m_mdiArea->subWindowList() )
	{
		ccGLWindowInterface* child = ccGLWindowInterface::FromWidget(window->widget());
		if (child != sendingWindow)
		{
			child->signalEmitter()->blockSignals(true);
			child->onWheelEvent(wheelDelta_deg);
			child->signalEmitter()->blockSignals(false);
			child->redraw();
		}
	}
}

void MainWindow::echoBaseViewMatRotation(const ccGLMatrixd& rotMat)
{
	if (!m_UI->actionEnableCameraLink->isChecked())
		return;

	ccGLWindowInterface* sendingWindow = ccGLWindowInterface::FromEmitter(sender());
	if (!sendingWindow)
		return;

	for ( QMdiSubWindow* window : m_mdiArea->subWindowList() )
	{
		ccGLWindowInterface* child = ccGLWindowInterface::FromWidget(window->widget());
		if (child != sendingWindow)
		{
			child->signalEmitter()->blockSignals(true);
			child->rotateBaseViewMat(rotMat);
			child->signalEmitter()->blockSignals(false);
			child->redraw();
		}
	}
}

 void MainWindow::echoCameraPosChanged(const CCVector3d& P)
 {
	 if (!m_UI->actionEnableCameraLink->isChecked())
		 return;

	 ccGLWindowInterface* sendingWindow = ccGLWindowInterface::FromEmitter(sender());
	 if (!sendingWindow)
		 return;


	 for ( QMdiSubWindow* window : m_mdiArea->subWindowList() )
	 {
		 ccGLWindowInterface* child = ccGLWindowInterface::FromWidget(window->widget());
		 if (child != sendingWindow)
		 {
			 child->signalEmitter()->blockSignals(true);
			 child->setCameraPos(P);
			 child->signalEmitter()->blockSignals(false);
			 child->redraw();
		 }
	 }
 }

 void MainWindow::echoPivotPointChanged(const CCVector3d& P)
 {
	 if (!m_UI->actionEnableCameraLink->isChecked())
		 return;

	 ccGLWindowInterface* sendingWindow = ccGLWindowInterface::FromEmitter(sender());
	 if (!sendingWindow)
		 return;

	 for ( QMdiSubWindow* window : m_mdiArea->subWindowList() )
	 {
		 ccGLWindowInterface* child = ccGLWindowInterface::FromWidget(window->widget());
		 if (child != sendingWindow)
		 {
			 child->signalEmitter()->blockSignals(true);
			 child->setPivotPoint(P);
			 child->signalEmitter()->blockSignals(false);
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
	ccConsole::Error(tr("Not yet implemented! Sorry ..."));
}

void MainWindow::doActionKMeans()//TODO
{
	ccConsole::Error(tr("Not yet implemented! Sorry ..."));
}

void MainWindow::doActionFrontPropagation() //TODO
{
	ccConsole::Error(tr("Not yet implemented! Sorry ..."));
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
	delete s_instance;
	s_instance=nullptr;
}

void MainWindow::GetGLWindows(std::vector<ccGLWindowInterface*>& glWindows)
{
	const QList<QMdiSubWindow*> windows = TheInstance()->m_mdiArea->subWindowList();

	if ( windows.empty() )
		return;

	glWindows.clear();
	glWindows.reserve( windows.size() );

	for ( QMdiSubWindow* window : windows )
	{
		glWindows.push_back(ccGLWindowInterface::FromWidget(window->widget()));
	}
}

ccGLWindowInterface* MainWindow::GetActiveGLWindow()
{
	return TheInstance()->getActiveGLWindow();
}

ccGLWindowInterface* MainWindow::GetGLWindow(const QString& title)
{
	const QList<QMdiSubWindow *> windows = TheInstance()->m_mdiArea->subWindowList();

	if ( windows.empty() )
		return nullptr;

	for ( QMdiSubWindow* window : windows )
	{
		ccGLWindowInterface* win = ccGLWindowInterface::FromWidget(window->widget());
		if (win->getWindowTitle() == title)
			return win;
	}

	return nullptr;
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

void MainWindow::addEditPlaneAction( QMenu &menu ) const
{
	menu.addAction( m_UI->actionEditPlane );
}

ccHObject* MainWindow::dbRootObject()
{
	return (m_ccRoot ? m_ccRoot->getRootEntity() : nullptr);
}

ccUniqueIDGenerator::Shared MainWindow::getUniqueIDGenerator()
{
	return ccObject::GetUniqueIDGenerator();
}

void MainWindow::createGLWindow(ccGLWindowInterface*& window, QWidget*& widget) const
{
	bool stereoMode = ccGLWindowInterface::TestStereoSupport();

	ccGLWindowInterface::Create(window, widget, stereoMode);
	assert(window && widget);
}

void MainWindow::destroyGLWindow(ccGLWindowInterface* view3D) const
{
	if (view3D)
	{
		view3D->asQObject()->setParent(nullptr);
		delete view3D;
	}
}

ccMainAppInterface::ccHObjectContext MainWindow::removeObjectTemporarilyFromDBTree(ccHObject* obj)
{
	ccHObjectContext context;

	assert(obj);
	if (!m_ccRoot || !obj)
		return context;

	//mandatory (to call putObjectBackIntoDBTree)
	context.parent = obj->getParent();

	//remove the object's dependency to its father (in case it undergoes 'severe' modifications)
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

	ccLog::Print(tr("[Global Shift] Max abs. coord = %1 / max abs. diag = %2")
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

void MainWindow::doActionCompressFWFData()
{
	for ( ccHObject *entity : getSelectedEntities() )
	{
		if (!entity || !entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			continue;
		}

		ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);
		cloud->compressFWFData();
	}
}

void MainWindow::doActionShowWaveDialog()
{
	if (!haveSelection())
		return;

	ccHObject* entity = haveOneSelection() ? m_selectedEntities.front() : nullptr;
	if (!entity || !entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error(tr("Select one point cloud!"));
		return;
	}

	ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);
	if (!cloud->hasFWF())
	{
		ccConsole::Error(tr("Cloud has no associated waveform information"));
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
	if (!haveSelection())
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

void MainWindow::doActionFlipPlane()
{
	if (!haveSelection())
	{
		assert(false);
		return;
	}

	for (ccHObject* entity : m_selectedEntities)
	{
		ccPlane* plane = ccHObjectCaster::ToPlane(entity);
		if (plane)
		{
			plane->flip();
			plane->prepareDisplayForRefresh();
		}
	}

	refreshAll();
	updatePropertiesView();
}

void MainWindow::doActionComparePlanes()
{
	if (m_selectedEntities.size() != 2)
	{
		ccConsole::Error(tr("Select 2 planes!"));
		return;
	}

	if (!m_selectedEntities.front()->isKindOf(CC_TYPES::PLANE) ||
		!m_selectedEntities.back()->isKindOf(CC_TYPES::PLANE))
	{
		ccConsole::Error(tr("Select 2 planes!"));
		return;
	}

	ccPlane* p1 = ccHObjectCaster::ToPlane(m_selectedEntities.front());
	ccPlane* p2 = ccHObjectCaster::ToPlane(m_selectedEntities.back());

	QStringList info;
	info << tr("Plane 1: %1").arg(p1->getName());
	ccLog::Print(tr("[Compare] ") + info.last());

	info << tr("Plane 2: %1").arg(p2->getName());
	ccLog::Print(tr("[Compare] ") + info.last());

	CCVector3 N1;
	CCVector3 N2;
	PointCoordinateType d1;
	PointCoordinateType d2;
	p1->getEquation(N1, d1);
	p2->getEquation(N2, d2);

	double angle_rad = N1.angle_rad(N2);
	info << tr("Angle P1/P2: %1 deg.").arg( CCCoreLib::RadiansToDegrees( angle_rad ) );
	ccLog::Print(tr("[Compare] ") + info.last());

	PointCoordinateType planeEq1[4] { N1.x, N1.y, N1.z, d1 };
	PointCoordinateType planeEq2[4] { N2.x, N2.y, N2.z, d2 };
	CCVector3 C1 = p1->getCenter();
	ScalarType distCenter1ToPlane2 = CCCoreLib::DistanceComputationTools::computePoint2PlaneDistance(&C1, planeEq2);
	info << tr("Distance Center(P1)/P2: %1").arg(distCenter1ToPlane2);
	ccLog::Print(tr("[Compare] ") + info.last());

	CCVector3 C2 = p2->getCenter();
	ScalarType distCenter2ToPlane1 = CCCoreLib::DistanceComputationTools::computePoint2PlaneDistance(&C2, planeEq1);
	info << tr("Distance Center(P2)/P1: %1").arg(distCenter2ToPlane1);
	ccLog::Print(tr("[Compare] ") + info.last());

	//pop-up summary
	QMessageBox::information(this, tr("Plane comparison"), info.join("\n"));
	forceConsoleDisplay();
}

void MainWindow::doActionPromoteCircleToCylinder()
{
	if (!haveOneSelection())
	{
		assert(false);
		return;
	}

	ccCircle* circle = ccHObjectCaster::ToCircle(m_selectedEntities.front());
	if (!circle)
	{
		assert(false);
		return;
	}

	static double CylinderHeight = 0.0;
	if (CylinderHeight == 0.0)
	{
		CylinderHeight = 2 * circle->getRadius();
	}
	bool ok = false;
	double value = QInputDialog::getDouble(this, tr("Cylinder height"), tr("Height"), CylinderHeight, 0.0, std::numeric_limits<double>::max(), 6, &ok);
	if (!ok)
	{
		return;
	}

	CylinderHeight = value;

	ccCylinder* cylinder = new ccCylinder(	static_cast<PointCoordinateType>(circle->getRadius()),
											static_cast<PointCoordinateType>(CylinderHeight),
											&circle->getGLTransformationHistory(),
											tr("Cylinder from ") + circle->getName());

	circle->setEnabled(false);
	if (circle->getParent())
	{
		circle->getParent()->addChild(cylinder);
	}

	addToDB(cylinder, true, true);
	setSelectedInDB(circle, false);
	setSelectedInDB(cylinder, true);
}
