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
#include <ccCameraSensor.h>
#include <ccColorScalesManager.h>
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
#include <DepthMapFileFilter.h>

//QCC_glWindow
#include <ccGLWidget.h>
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
#include <ccDisplayOptionsDlg.h>
#include <ccPickOneElementDlg.h>
#include <ccStereoModeDlg.h>

//dialogs
#include "ccAboutDialog.h"
#include "ccAdjustZoomDlg.h"
#include "ccAlignDlg.h" //Aurelien BEY
#include "ccApplication.h"
#include "ccApplyTransformationDlg.h"
#include "ccAskTwoDoubleValuesDlg.h"
#include "ccAskThreeDoubleValuesDlg.h"
#include "ccBoundingBoxEditorDlg.h"
#include "ccCamSensorProjectionDlg.h"
#include "ccClippingBoxTool.h"
#include "ccColorScaleEditorDlg.h"
#include "ccComparisonDlg.h"
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
#include "ccPointPropertiesDlg.h" //Aurelien BEY
#include "ccPrimitiveFactoryDlg.h"
#include "ccPtsSamplingDlg.h"
#include "ccRasterizeTool.h"
#include "ccRegistrationDlg.h" //Aurelien BEY
#include "ccRenderToFileDlg.h"
#include "ccScaleDlg.h"
#include "ccSectionExtractionTool.h"
#include "ccSensorComputeDistancesDlg.h"
#include "ccSensorComputeScatteringAnglesDlg.h"
#include "ccSORFilterDlg.h"
#include "ccSubsamplingDlg.h" //Aurelien BEY
#include "ccTracePolylineTool.h"
#include "ccTranslationManager.h"
#include "ccUnrollDlg.h"
#include "ccVolumeCalcTool.h"
#include "ccWaveformDialog.h"
#include "bdrSettingBDSegDlg.h"
#include "bdrSettingGrdFilterDlg.h"
#include "bdrProjectDlg.h"

//other
#include "ccCropTool.h"
#include "ccGLPluginInterface.h"
#include "ccPersistentSettings.h"
#include "ccRecentFiles.h"
#include "ccRegistrationTools.h"
#include "ccUtils.h"
#include "db_tree/ccDBRoot.h"
#include "pluginManager/ccPluginUIManager.h"

//3D mouse handler
#ifdef CC_3DXWARE_SUPPORT
#include "devices/3dConnexion/cc3DMouseManager.h"
#endif

//Gamepads
#ifdef CC_GAMEPADS_SUPPORT
#include "devices/gamepad/ccGamepadManager.h"
#endif

#ifdef USE_TBB
#include <tbb/tbb_stddef.h>
#endif

//Qt UI files
#include <ui_distanceMapDlg.h>
#include <ui_globalShiftSettingsDlg.h>
#include <ui_mainWindow.h>

//System
#include <iostream>
#include <random>

#include "core/IO/qAdditionalIO/src/BundlerFilter.h"
#ifdef USE_STOCKER
#include "bdrPlaneSegDlg.h"
#include "bdrLine3DppDlg.h"
#include "bdrDeductionDlg.h"
#include "bdrPolyFitDlg.h"
#include "bdrSettingLoD2Dlg.h"
#include "bdrFacetFilterDlg.h"
#include "bdr2.5DimEditor.h"
#include "bdrImageEditorPanel.h"
#include "bdrPlaneEditorDlg.h"

#include "stocker_parser.h"
#include "polyfit/basic/logger.h"
#endif // USE_STOCKER

#include "BlockDBaseIO.h"

#include <QDate>
#include <QFuture>
#include <QtConcurrent>
#include <QFile>

//global static pointer (as there should only be one instance of MainWindow!)
static MainWindow* s_instance  = nullptr;

//default file filter separator
static const QString s_fileFilterSeparator(";;");

enum PickingOperation {	NO_PICKING_OPERATION,
						PICKING_ROTATION_CENTER,
						PICKING_LEVEL_POINTS,
					  };
static ccGLWindow* s_pickingWindow = nullptr;
static PickingOperation s_currentPickingOperation = NO_PICKING_OPERATION;
static std::vector<cc2DLabel*> s_levelLabels;
static ccPointCloud* s_levelMarkersCloud = nullptr;
static ccHObject* s_levelEntity = nullptr;

static QFileDialog::Options CCFileDialogOptions()
{
	//dialog options
	QFileDialog::Options dialogOptions = QFileDialog::Options();
	if (!ccOptions::Instance().useNativeDialogs)
	{
		dialogOptions |= QFileDialog::DontUseNativeDialog;
	}
	return dialogOptions;
}

MainWindow::MainWindow()
	: m_UI(new Ui::MainWindow)
	, m_ccRoot(nullptr)
	, m_buildingRoot(nullptr)
	, m_imageRoot(nullptr)
	, m_uiFrozen(false)
	, m_recentFiles(new ccRecentFiles(this))
	, m_3DMouseManager(nullptr)
	, m_gamepadManager(nullptr)
	, m_viewModePopupButton(nullptr)
	, m_pivotVisibilityPopupButton(nullptr)
	, m_FirstShow(true)
	, m_pickingHub(nullptr)
	// status bar
	, m_progressLabel(nullptr)
	, m_progressBar(nullptr)
	, m_progressButton(nullptr)
	, m_status_pointSnapBufferSpinBox(nullptr)
	, m_status_coord2D(nullptr)
	, m_status_show_coord3D(nullptr)
	, m_status_show_global(nullptr)
	, m_status_depth(nullptr)
	, m_status_coord3D(nullptr)
	// dialog
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
	, m_pbdrPSDlg(nullptr)
	, m_pbdrl3dDlg(nullptr)
	, m_pbdrddtDlg(nullptr)
	, m_pbdrpfDlg(nullptr)
	, m_pbdrSettingLoD2Dlg(nullptr)
	, m_pbdrffDlg(nullptr)
	, m_pbdrImshow(nullptr)	
	, m_pbdrImagePanel(nullptr)
	, m_pbdrPlaneEditDlg(nullptr)
	, polyfit_obj(nullptr)
	, m_GCSvr_prj_id(0)
	, m_pbdrSettingBDSegDlg(nullptr)
	, m_pbdrSettingGrdFilterDlg(nullptr)
	, m_pbdrPrjDlg(nullptr)
{
	m_UI->setupUi( this );

	setWindowTitle(QStringLiteral("BlockBuilder v") + ccApp->versionLongStr(false));
	
	m_pluginUIManager = new ccPluginUIManager( this, this );
	
	ccTranslationManager::get().populateMenu( m_UI->menuLanguage, ccApp->translationPath() );
	
#ifdef Q_OS_MAC
	m_UI->actionAbout->setMenuRole( QAction::AboutRole );
	m_UI->actionAboutPlugins->setMenuRole( QAction::ApplicationSpecificRole );

	m_UI->actionFullScreen->setText( tr( "Enter Full Screen" ) );
	m_UI->actionFullScreen->setShortcut( QKeySequence( Qt::CTRL + Qt::META + Qt::Key_F ) );
#endif

	// Set up dynamic menus
	m_UI->menuFile->insertMenu(m_UI->actionSave, m_recentFiles->menu());

	m_UI->toolBarSFTools->setVisible(false);
	
	//Console
	ccConsole::Init(m_UI->consoleWidget, this, this);
	m_UI->actionEnableQtWarnings->setChecked(ccConsole::QtMessagesEnabled());

	//advanced widgets not handled by QDesigner
	if (0)	// XYLIU, no need
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
			m_viewModePopupButton->setToolTip("Set current view mode");
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
			m_pivotVisibilityPopupButton->setToolTip("Set pivot visibility");
			m_pivotVisibilityPopupButton->setStatusTip(m_pivotVisibilityPopupButton->toolTip());
			m_UI->toolBarView->insertWidget(m_UI->actionZoomAndCenter,m_pivotVisibilityPopupButton);
			m_pivotVisibilityPopupButton->setEnabled(false);
		}
	}

	QWidget *titleWidget = m_UI->DockableDBTree->titleBarWidget();
	QWidget *tempWidget1 = new QWidget();
	m_UI->DockableDBTree->setTitleBarWidget(tempWidget1);
	delete titleWidget; titleWidget = nullptr;

	// 	titleWidget = m_UI->DockableProperties->titleBarWidget();
	// 	QWidget *tempWidget2 = new QWidget();
	// 	m_UI->DockableProperties->setTitleBarWidget(tempWidget2);
	// 	delete titleWidget; titleWidget = nullptr;

	// 	titleWidget = m_UI->DockableConsole->titleBarWidget();
	// 	QWidget *tempWidget3 = new QWidget();
	// 	m_UI->DockableConsole->setTitleBarWidget(tempWidget3);
	// 	delete titleWidget; titleWidget = nullptr;
		
	tabifyDockWidget(m_UI->DockableProperties, m_UI->DockablePanel);
	tabifyDockWidget(m_UI->DockableProperties, m_UI->DockableImage);
	tabifyDockWidget(m_UI->DockableProperties, m_UI->DockableConsole);
	m_UI->DockableProperties->raise();

	//db-main-tree
	{
		m_ccRoot = new StDBMainRoot(m_UI->dbMainTreeView, m_UI->propertiesTreeView_Main, this);
		connect(m_ccRoot, &ccDBRoot::selectionChanged,    this, &MainWindow::updateUIWithSelection);
		connect(m_ccRoot, &ccDBRoot::dbIsEmpty,           [&]() { updateUIWithSelection(); updateMenus(); }); //we don't call updateUI because there's no need to update the properties dialog
		connect(m_ccRoot, &ccDBRoot::dbIsNotEmptyAnymore, [&]() { updateUIWithSelection(); updateMenus(); }); //we don't call updateUI because there's no need to update the properties dialog
		connect(m_ccRoot, &ccDBRoot::itemClicked,		  [&]() { updateDBSelection(CC_TYPES::DB_MAINDB); });
	}

	//db-build-tree
	{
		m_buildingRoot = new StDBBuildingRoot(m_UI->dbBuildTreeView, m_UI->propertiesTreeView_Build, this);
		connect(m_buildingRoot, &ccDBRoot::selectionChanged,	this, &MainWindow::updateUIWithSelection);
		connect(m_buildingRoot, &ccDBRoot::dbIsEmpty,			[&]() { updateUIWithSelection(); updateMenus(); }); //we don't call updateUI because there's no need to update the properties dialog
		connect(m_buildingRoot, &ccDBRoot::dbIsNotEmptyAnymore, [&]() { updateUIWithSelection(); updateMenus(); }); //we don't call updateUI because there's no need to update the properties dialog
		connect(m_buildingRoot, &ccDBRoot::itemClicked,			[&]() { updateDBSelection(CC_TYPES::DB_BUILDING); });
	}

	//db-image-tree // XYLIU
	{
		m_imageRoot = new StDBImageRoot(m_UI->dbImageTreeView, m_UI->propertiesTreeView_Image, this);
 		connect(m_imageRoot, &ccDBRoot::selectionChanged,		this, &MainWindow::updateUIWithSelection);
		connect(m_imageRoot, &ccDBRoot::dbIsEmpty,				[&]() { clearImagePanel();  updateUIWithSelection(); updateMenus(); }); //we don't call updateUI because there's no need to update the properties dialog
  		connect(m_imageRoot, &ccDBRoot::dbIsNotEmptyAnymore,	[&]() { updateUIWithSelection(); updateMenus(); }); //we don't call updateUI because there's no need to update the properties dialog
		connect(m_imageRoot, &ccDBRoot::itemClicked,			[&]() { updateDBSelection(CC_TYPES::DB_IMAGE); });
	}
	switchDatabase(CC_TYPES::DB_MAINDB);

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

	m_UI->vboxLayout->setContentsMargins(0, 0, 0, 0);
	m_UI->vboxLayout1->setContentsMargins(0, 0, 0, 0);
	m_UI->vboxLayout1->setSpacing(0);
	m_UI->vboxLayout2->setContentsMargins(0, 0, 0, 0);
	m_UI->vboxLayout3->setContentsMargins(0, 0, 0, 0);
	m_UI->vboxLayout4->setContentsMargins(0, 0, 0, 0);
	m_UI->verticalLayout->setContentsMargins(0, 0, 0, 0); 
	m_UI->verticalLayout_2->setContentsMargins(0, 0, 0, 0);	
	m_UI->verticalLayout_3->setContentsMargins(0, 0, 0, 0);

	//////////////////////////////////////////////////////////////////////////
	// TODO: status bar
	m_progressLabel = new QLabel; 
	m_progressLabel->setText("items"); 
	m_progressLabel->setMaximumSize(300, 50);

	m_progressBar = new QProgressBar; 
	m_progressBar->setFixedWidth(500);
//	m_progressBar->setWindowModality(Qt::NonModal);

	m_progressButton = new QPushButton;
	m_progressButton->setText("Cancel");

	QStatusBar* status_bar = QMainWindow::statusBar();
	status_bar->addPermanentWidget(m_progressLabel); m_progressLabel->hide();
	status_bar->addPermanentWidget(m_progressBar); m_progressBar->hide();
	status_bar->addPermanentWidget(m_progressButton); m_progressButton->hide();

	// TODO: COORDINATE
	m_status_depth = new QLabel();
	m_status_depth->setToolTip("GL depth");
	m_status_depth->setStatusTip("GL depth at current cursor");
	m_status_depth->setMinimumWidth(100);
	status_bar->addPermanentWidget(m_status_depth);

	m_status_pointSnapBufferSpinBox = new QSpinBox();
	m_status_pointSnapBufferSpinBox->setMinimum(0);
	m_status_pointSnapBufferSpinBox->setToolTip("buffer distance to deduce GL depth for point snapping");
	m_status_pointSnapBufferSpinBox->setStatusTip("buffer distance to deduce GL depth for point snapping");
	connect(m_status_pointSnapBufferSpinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
		this, &MainWindow::pointSnapBufferChanged);
	status_bar->addPermanentWidget(m_status_pointSnapBufferSpinBox);

	m_status_show_coord3D = new QToolButton();
	m_status_show_coord3D->setDefaultAction(m_UI->actionShowCursor3DCoordinates);
	status_bar->addPermanentWidget(m_status_show_coord3D);

	m_status_show_global = new QToolButton();
	m_status_show_global->setDefaultAction(m_UI->actionDisplayGlobalCoord);
	status_bar->addPermanentWidget(m_status_show_global);

	m_status_coord3D = new QLabel();
	m_status_coord3D->setToolTip("3D coord snapped");
	m_status_coord3D->setStatusTip("3D coord the cursor snapped to");
	m_status_coord3D->setMinimumWidth(200);
	status_bar->addPermanentWidget(m_status_coord3D);

	m_status_coord2D = new QLabel();
	m_status_coord2D->setToolTip("Image Coord");
	m_status_coord2D->setStatusTip("Image Coord");
	m_status_coord2D->setMinimumWidth(150);
	status_bar->addPermanentWidget(m_status_coord2D);
	//////////////////////////////////////////////////////////////////////////

	m_pbdrSettingBDSegDlg = new bdrSettingBDSegDlg(this);
	//////////////////////////////////////////////////////////////////////////

	connectActions();

	new3DView(true);

	CreateImageEditor();

	CreateEditorPanel();

	setupInputDevices();

	freezeUI(false);

	updateUI();

	QMainWindow::statusBar()->showMessage(QString("Ready"));
	
#ifdef USE_TBB
	ccConsole::Print( QStringLiteral( "[TBB] Using Intel's Threading Building Blocks %1.%2" )
					  .arg( QString::number( TBB_VERSION_MAJOR ), QString::number( TBB_VERSION_MINOR ) ) );
#endif
	
	ccConsole::Print("BlockBuilder started!");

#ifdef USE_STOCKER
	Logger::initialize();
	Logger::instance()->set_value(Logger::LOG_REGISTER_FEATURES, "*");
#endif // USE_STOCKER

}

MainWindow::~MainWindow()
{
	destroyInputDevices();

	cancelPreviousPickingOperation(false); //just in case

	assert(m_ccRoot && m_mdiArea && m_buildingRoot && m_imageRoot);
	m_ccRoot->disconnect();
	m_buildingRoot->disconnect();
	m_imageRoot->disconnect();
	m_mdiArea->disconnect();

	//we don't want any other dialog/function to use the following structures
	ccDBRoot* ccRoot = m_ccRoot;
	m_ccRoot = nullptr;

	ccDBRoot* bdRoot = m_buildingRoot;
	m_buildingRoot = nullptr;

	ccDBRoot* imgRoot = m_imageRoot;
	m_imageRoot = nullptr;

	//remove all entities from 3D views before quitting to avoid any side-effect
	//(this won't be done automatically since we've just reset m_ccRoot)
	ccRoot->getRootEntity()->setDisplay_recursive(nullptr);
	bdRoot->getRootEntity()->setDisplay_recursive(nullptr);
	imgRoot->getRootEntity()->setDisplay_recursive(nullptr);
	for (int i = 0; i < getGLWindowCount(); ++i)
	{	
		getGLWindow(i)->getSceneDB().clear();
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

	m_pbdrPSDlg = nullptr;
	m_pbdrl3dDlg = nullptr;
	m_pbdrddtDlg = nullptr;
	m_pbdrpfDlg = nullptr;
	m_pbdrffDlg = nullptr;
	m_pbdrSettingLoD2Dlg = nullptr;
	
	m_pbdrImshow = nullptr;
	m_pbdrImagePanel = nullptr;
	m_pbdrPlaneEditDlg = nullptr;

	Logger::terminate();

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
	{
		delete ccRoot;
		ccRoot = nullptr;
	}
	if (bdRoot)
	{
		delete bdRoot;
		bdRoot = nullptr;
	}
	if (imgRoot)
	{
		delete imgRoot;
		imgRoot = nullptr;
	}

	delete m_UI;
	m_UI = nullptr;
	
	ccConsole::ReleaseInstance(false); //if we flush the console, it will try to display the console window while we are destroying everything!
}

void MainWindow::initPlugins( )
{
	m_pluginUIManager->init();
	
	// Set up dynamic tool bars
// 	addToolBar( Qt::RightToolBarArea, m_pluginUIManager->glFiltersToolbar() );
// 	addToolBar( Qt::RightToolBarArea, m_pluginUIManager->mainPluginToolbar() );
// 	
// 	for ( QToolBar *toolbar : m_pluginUIManager->additionalPluginToolbars() )
// 	{
// 		addToolBar( Qt::TopToolBarArea, toolbar );
// 	}
	
	// Set up dynamic menus
	m_UI->menubar->insertMenu( m_UI->menu3DViews->menuAction(), m_pluginUIManager->pluginMenu() );
	m_UI->menuDisplay->insertMenu( m_UI->menuActiveScalarField->menuAction(), m_pluginUIManager->shaderAndFilterMenu() );

// 	m_UI->menuToolbars->addAction( m_pluginUIManager->actionShowMainPluginToolbar() );
// 	m_UI->menuToolbars->addAction( m_pluginUIManager->actionShowGLFilterToolbar() );
}

void MainWindow::doEnableQtWarnings(bool state)
{
	ccConsole::EnableQtMessages(state);
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
	m_UI->menuFile->insertMenu(m_UI->actionCloseAll, m_3DMouseManager->menu());
#endif

#ifdef CC_GAMEPADS_SUPPORT
	m_gamepadManager = new ccGamepadManager( this, this );
	m_UI->menuFile->insertMenu(m_UI->actionCloseAll, m_gamepadManager->menu());
#endif

#if defined(CC_3DXWARE_SUPPORT) || defined(CC_GAMEPADS_SUPPORT)
	m_UI->menuFile->insertSeparator(m_UI->actionCloseAll);
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
	assert(m_buildingRoot);
	assert(m_imageRoot);
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
	connect(m_UI->actionOpen,					&QAction::triggered, this, &MainWindow::doActionLoadFile);
	connect(m_UI->actionSave,					&QAction::triggered, this, &MainWindow::doActionSaveFile);
	connect(m_UI->actionGlobalShiftSettings,	&QAction::triggered, this, &MainWindow::doActionGlobalShiftSeetings);
	connect(m_UI->actionPrimitiveFactory,		&QAction::triggered, this, &MainWindow::doShowPrimitiveFactory);
	connect(m_UI->actionCloseAll,				&QAction::triggered, this, &MainWindow::closeAll);
	connect(m_UI->actionQuit,					&QAction::triggered, this, &QWidget::close);

	//"Edit > Colors" menu
	connect(m_UI->actionSetUniqueColor,				&QAction::triggered, this, &MainWindow::doActionSetUniqueColor);
	connect(m_UI->actionSetColorGradient,			&QAction::triggered, this, &MainWindow::doActionSetColorGradient);
	connect(m_UI->actionChangeColorLevels,			&QAction::triggered, this, &MainWindow::doActionChangeColorLevels);
	connect(m_UI->actionColorize,					&QAction::triggered, this, &MainWindow::doActionColorize);
	connect(m_UI->actionRGBToGreyScale,				&QAction::triggered, this, &MainWindow::doActionRGBToGreyScale);
	connect(m_UI->actionInterpolateColors,			&QAction::triggered, this, &MainWindow::doActionInterpolateColors);
	connect(m_UI->actionEnhanceRGBWithIntensities,	&QAction::triggered, this, &MainWindow::doActionEnhanceRGBWithIntensities);
	connect(m_UI->actionClearColor, &QAction::triggered, this, [=]() {
		clearSelectedEntitiesProperty( ccEntityAction::CLEAR_PROPERTY::COLORS );
	});

	//"Edit > Normals" menu
	connect(m_UI->actionComputeNormals,				&QAction::triggered, this, &MainWindow::doActionComputeNormals);
	connect(m_UI->actionInvertNormals,				&QAction::triggered, this, &MainWindow::doActionInvertNormals);
	connect(m_UI->actionConvertNormalToHSV,			&QAction::triggered, this, &MainWindow::doActionConvertNormalsToHSV);
	connect(m_UI->actionConvertNormalToDipDir,		&QAction::triggered, this, &MainWindow::doActionConvertNormalsToDipDir);
	connect(m_UI->actionOrientNormalsMST,			&QAction::triggered, this, &MainWindow::doActionOrientNormalsMST);
	connect(m_UI->actionOrientNormalsFM,			&QAction::triggered, this, &MainWindow::doActionOrientNormalsFM);
	connect(m_UI->actionClearNormals, &QAction::triggered, this, [=]() {
		clearSelectedEntitiesProperty( ccEntityAction::CLEAR_PROPERTY::NORMALS );
	});

	//"Edit > Octree" menu
	connect(m_UI->actionComputeOctree,		&QAction::triggered, this, &MainWindow::doActionComputeOctree);
	connect(m_UI->actionResampleWithOctree,	&QAction::triggered, this, &MainWindow::doActionResampleWithOctree);

	//"Edit > Grid" menu
	connect(m_UI->actionDeleteScanGrid,		&QAction::triggered, this, &MainWindow::doActionDeleteScanGrids);

	//"Edit > Mesh" menu
	connect(m_UI->actionComputeMeshAA,				&QAction::triggered, this, &MainWindow::doActionComputeMeshAA);
	connect(m_UI->actionComputeMeshLS,				&QAction::triggered, this, &MainWindow::doActionComputeMeshLS);
	connect(m_UI->actionMeshTwoPolylines,			&QAction::triggered, this, &MainWindow::doMeshTwoPolylines);
	connect(m_UI->actionMeshScanGrids,				&QAction::triggered, this, &MainWindow::doActionMeshScanGrids);
	connect(m_UI->actionConvertTextureToColor,		&QAction::triggered, this, &MainWindow::doActionConvertTextureToColor);
	connect(m_UI->actionSamplePointsOnMesh,			&QAction::triggered, this, &MainWindow::doActionSamplePointsOnMesh);
	connect(m_UI->actionSmoothMeshLaplacian,		&QAction::triggered, this, &MainWindow::doActionSmoothMeshLaplacian);
	connect(m_UI->actionSubdivideMesh,				&QAction::triggered, this, &MainWindow::doActionSubdivideMesh);
	connect(m_UI->actionMeasureMeshSurface,			&QAction::triggered, this, &MainWindow::doActionMeasureMeshSurface);
	connect(m_UI->actionMeasureMeshVolume,			&QAction::triggered, this, &MainWindow::doActionMeasureMeshVolume);
	connect(m_UI->actionFlagMeshVertices,			&QAction::triggered, this, &MainWindow::doActionFlagMeshVertices);
	//"Edit > Mesh > Scalar Field" menu
	connect(m_UI->actionSmoothMeshSF,				&QAction::triggered, this, &MainWindow::doActionSmoothMeshSF);
	connect(m_UI->actionEnhanceMeshSF,				&QAction::triggered, this, &MainWindow::doActionEnhanceMeshSF);
	//"Edit > Polyline" menu
	connect(m_UI->actionSamplePointsOnPolyline,		&QAction::triggered, this, &MainWindow::doActionSamplePointsOnPolyline);
	//"Edit > Plane" menu
	connect(m_UI->actionCreatePlane,				&QAction::triggered, this, &MainWindow::doActionCreatePlane);
	connect(m_UI->actionEditPlane,					&QAction::triggered, this, &MainWindow::doActionEditPlane);
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
	connect(m_UI->actionScalarFieldArithmetic,		&QAction::triggered, this, &MainWindow::doActionScalarFieldArithmetic);
	connect(m_UI->actionScalarFieldFromColor,		&QAction::triggered, this, &MainWindow::doActionScalarFieldFromColor);
	connect(m_UI->actionConvertToRGB,				&QAction::triggered, this, &MainWindow::doActionSFConvertToRGB);
	connect(m_UI->actionConvertToRandomRGB,			&QAction::triggered, this, &MainWindow::doActionSFConvertToRandomRGB);
	connect(m_UI->actionRenameSF,					&QAction::triggered, this, &MainWindow::doActionRenameSF);
	connect(m_UI->actionOpenColorScalesManager,		&QAction::triggered, this, &MainWindow::doActionOpenColorScalesManager);
	connect(m_UI->actionAddIdField,					&QAction::triggered, this, &MainWindow::doActionAddIdField);
	connect(m_UI->actionSetSFAsCoord,				&QAction::triggered, this, &MainWindow::doActionSetSFAsCoord);
	connect(m_UI->actionInterpolateSFs,				&QAction::triggered, this, &MainWindow::doActionInterpolateScalarFields);
	connect(m_UI->actionDeleteScalarField, &QAction::triggered, this, [=]() {
		clearSelectedEntitiesProperty( ccEntityAction::CLEAR_PROPERTY::CURRENT_SCALAR_FIELD );
	});
	connect(m_UI->actionDeleteAllSF, &QAction::triggered, this, [=]() {
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
	connect(m_UI->actionDelete,						&QAction::triggered, m_buildingRoot,	&ccDBRoot::deleteSelectedEntities);	// TODO
	connect(m_UI->actionGotoNextZoom,				&QAction::triggered, m_buildingRoot,	&ccDBRoot::gotoNextZoom);

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
	connect(m_UI->actionCPS,						&QAction::triggered, this, &MainWindow::doActionComputeCPS);
	connect(m_UI->actionCloudModelDist,				&QAction::triggered, this, &MainWindow::doActionCloudModelDist);
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
	connect(m_UI->actionDisplayOptions,				&QAction::triggered, this, &MainWindow::showDisplayOptions);
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
	connect(m_UI->actionDisplayShowBBox,			&QAction::triggered, this, &MainWindow::doActionToggleDrawBBox);
	connect(m_UI->actionDisplayGlobalCoord,			&QAction::triggered, this, &MainWindow::doActionDisplayGlobalCoord);

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

	connect(m_UI->actionPointViewEditMode,			&QAction::toggled, this, &MainWindow::toggleActiveWindowPointViewEditMode);
	

	//"About" menu entry
	connect(m_UI->actionHelp,						&QAction::triggered, this, &MainWindow::doActionShowHelpDialog);
	connect(m_UI->actionAboutPlugins,				&QAction::triggered, m_pluginUIManager, &ccPluginUIManager::showAboutDialog);
	connect(m_UI->actionEnableQtWarnings,			&QAction::toggled, this, &MainWindow::doEnableQtWarnings);

	connect(m_UI->actionAbout,	&QAction::triggered, this, [this] () {
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

	//////////////////////////////////////////////////////////////////////////
	//Building Reconstruction
	connect(m_UI->actionBDProjectLoad,				&QAction::triggered, this, &MainWindow::doActionBDProjectLoad);
	connect(m_UI->actionBDProjectSave,				&QAction::triggered, this, &MainWindow::doActionBDProjectSave);
	connect(m_UI->actionBDImagesLoad,				&QAction::triggered, this, &MainWindow::doActionBDImagesLoad);
	
	connect(m_UI->actionBDPlaneSegmentation,		&QAction::triggered, this, &MainWindow::doActionBDPlaneSegmentation);
	connect(m_UI->actionBDRetrieve,					&QAction::triggered, this, &MainWindow::doActionBDRetrieve);
	connect(m_UI->actionBDRetrievePlanePoints,		&QAction::triggered, this, &MainWindow::doActionBDRetrievePlanePoints);
	connect(m_UI->actionBDImage_Lines,				&QAction::triggered, this, &MainWindow::doActionBDImageLines);
	connect(m_UI->actionBDPrimIntersections,		&QAction::triggered, this, &MainWindow::doActionBDPrimIntersections);
	connect(m_UI->actionBDPrimAssignSharpLines,		&QAction::triggered, this, &MainWindow::doActionBDPrimAssignSharpLines);
	connect(m_UI->actionBDPrimPlaneFromSharp,		&QAction::triggered, this, &MainWindow::doActionBDPrimPlaneFromSharp);
	connect(m_UI->actionBDPrimBoundary,				&QAction::triggered, this, &MainWindow::doActionBDPrimBoundary);
	connect(m_UI->actionBDPrimOutline,				&QAction::triggered, this, &MainWindow::doActionBDPrimOutline);
	connect(m_UI->actionBDPrimPlaneFrame,			&QAction::triggered, this, &MainWindow::doActionBDPrimPlaneFrame);
	connect(m_UI->actionBDPrimMergePlane,			&QAction::triggered, this, &MainWindow::doActionBDPrimMergePlane);
	connect(m_UI->actionBDPrimSplitPlane,			&QAction::triggered, this, &MainWindow::doActionBDPrimSplitPlane);	
	connect(m_UI->actionBDPrimCreateGround,			&QAction::triggered, this, &MainWindow::doActionBDPrimCreateGround);
	connect(m_UI->actionBDPrimPointProjection,		&QAction::triggered, this, &MainWindow::doActionBDPrimPointProjection);
	connect(m_UI->actionBDPrimShrinkPlane,			&QAction::triggered, this, &MainWindow::doActionBDPrimShrinkPlane);
	connect(m_UI->actionBDPlaneFromPoints,			&QAction::triggered, this, &MainWindow::doActionBDPlaneFromPoints);
	connect(m_UI->actionBDPlaneFromPolygon,			&QAction::triggered, this, &MainWindow::doActionBDPlaneFromPolygon);
	connect(m_UI->actionBDPlane_Deduction,			&QAction::triggered, this, &MainWindow::doActionBDPlaneDeduction);	
		
	connect(m_UI->actionBDPrimMakePlane,			&QAction::triggered, this, &MainWindow::doActionBDPlaneCreate);
	connect(m_UI->actionBDPolyFit,					&QAction::triggered, this, &MainWindow::doActionBDPolyFit);
	connect(m_UI->actionBDPolyFitHypothesis,		&QAction::triggered, this, &MainWindow::doActionBDPolyFitHypothesis);
	connect(m_UI->actionBDPolyFitConfidence,		&QAction::triggered, this, &MainWindow::doActionBDPolyFitConfidence);
	connect(m_UI->actionBDPolyFitSelection,			&QAction::triggered, this, &MainWindow::doActionBDPolyFitSelection);
	connect(m_UI->actionBDPolyFitFacetFilter,		&QAction::triggered, this, &MainWindow::doActionBDPolyFitFacetFilter);
	connect(m_UI->actionBDPolyFitSettings,			&QAction::triggered, this, &MainWindow::doActionBDPolyFitSettings);
	connect(m_UI->actionBDLoD1Generation,			&QAction::triggered, this, &MainWindow::doActionBDLoD1Generation);
	connect(m_UI->actionBD3D4EM,					&QAction::triggered, this, &MainWindow::doActionBDLoD2Generation);
	connect(m_UI->actionSettingLoD2,				&QAction::triggered, this, &MainWindow::doActionSettingsLoD2);

	connect(m_UI->actionBDTextureMapping,			&QAction::triggered, this, &MainWindow::doActionBDTextureMapping);
	connect(m_UI->actionBDConstrainedMesh,			&QAction::triggered, this, &MainWindow::doActionBDConstrainedMesh);
	connect(m_UI->actionBDDisplayPlaneOn,			&QAction::triggered, this, &MainWindow::doActionBDDisplayPlaneOn);
	connect(m_UI->actionBDDisplayPlaneOff,			&QAction::triggered, this, &MainWindow::doActionBDDisplayPlaneOff);
	connect(m_UI->actionBDDisplayPointOn,			&QAction::triggered, this, &MainWindow::doActionBDDisplayPointOn);
	connect(m_UI->actionBDDisplayPointOff,			&QAction::triggered, this, &MainWindow::doActionBDDisplayPointOff);
	connect(m_UI->actionDisplayWireframe,			&QAction::triggered, this, &MainWindow::doActionDisplayWireframe);
	connect(m_UI->actionDisplayFace,				&QAction::triggered, this, &MainWindow::doActionDisplayFace);
	connect(m_UI->actionDisplayNormalPerFace,		&QAction::triggered, this, &MainWindow::doActionDisplayNormalPerFace);
	connect(m_UI->actionDisplayNormalPerVertex,		&QAction::triggered, this, &MainWindow::doActionDisplayNormalPerVertex);
	connect(m_UI->actionBDFootPrintAuto,			&QAction::triggered, this, &MainWindow::doActionBDFootPrintAuto);
	connect(m_UI->actionBDFootPrintManual,			&QAction::triggered, this, &MainWindow::doActionBDFootPrintManual); 
	connect(m_UI->actionBDFootPrintPack,			&QAction::triggered, this, &MainWindow::doActionBDFootPrintPack);
	connect(m_UI->actionBDFootPrintGetPlane,		&QAction::triggered, this, &MainWindow::doActionBDFootPrintGetPlane);
	connect(m_UI->actionBDMeshToBlock,				&QAction::triggered, this, &MainWindow::doActionBDMeshToBlock);
	connect(m_UI->actionShowBestImage,				&QAction::triggered, this, &MainWindow::doActionShowBestImage); 
	connect(m_UI->actionShowSelectedImage,			&QAction::triggered, this, &MainWindow::doActionShowSelectedImage);
	connect(m_UI->ProjectTabWidget,					SIGNAL(currentChanged(int)), this, SLOT(doActionChangeTabTree(int)));
	connect(m_UI->actionImageOverlay,				&QAction::triggered, this, &MainWindow::toggleImageOverlay);
	connect(m_UI->actionProjectToImage,				&QAction::triggered, this, &MainWindow::doActionProjectToImage); 
	connect(m_UI->actionSelectWorkingPlane,			&QAction::triggered, this, &MainWindow::doActionSelectWorkingPlane); 
	connect(m_UI->actionTogglePlaneEditState,		&QAction::triggered, this, &MainWindow::doActionTogglePlaneEditState);
	connect(m_UI->actionEditSelectedItem,			&QAction::triggered, this, &MainWindow::doActionEditSelectedItem);

	connect(m_UI->NewDatabaseToolButton,			&QAbstractButton::clicked, this, &MainWindow::doActionCreateDatabase);
	connect(m_UI->OpenDatabaseToolButton,			&QAbstractButton::clicked, this, &MainWindow::doActionOpenDatabase);
	connect(m_UI->SaveDatabaseToolButton,			&QAbstractButton::clicked, this, &MainWindow::doActionSaveDatabase);
	QMenu* menuImport = new QMenu(m_UI->ImportDataToolButton);
	//menuImport->addAction(m_UI->actionImportFile);
	menuImport->addAction(m_UI->actionImportFolder);
	m_UI->ImportDataToolButton->setMenu(menuImport);
	m_UI->ImportDataToolButton->setDefaultAction(m_UI->actionImportFile);
	connect(m_UI->actionImportFile,					&QAction::triggered, this, &MainWindow::doActionImportData);
	connect(m_UI->actionImportFolder,				&QAction::triggered, this, &MainWindow::doActionImportFolder);
	connect(m_UI->EditDatabaseToolButton,			&QAbstractButton::clicked, this, &MainWindow::doActionEditDatabase);
	connect(m_UI->createBuildingProjectToolButton,	&QAbstractButton::clicked, this, &MainWindow::doActionCreateBuildingProject);
	connect(m_UI->loadSubstanceToolButton,			&QAbstractButton::clicked, this, &MainWindow::doActionLoadSubstance);
	
	connect(m_UI->actionImageLiDARRegistration,		&QAction::triggered, this, &MainWindow::doActionImageLiDARRegistration);

	//! Segmentation
	connect(m_UI->actionGroundFilteringBatch,		&QAction::triggered, this, &MainWindow::doActionGroundFilteringBatch);
	connect(m_UI->actionClassificationBatch,		&QAction::triggered, this, &MainWindow::doActionClassificationBatch);
	connect(m_UI->actionBuildingSegmentationBatch,	&QAction::triggered, this, &MainWindow::doActionBuildingSegmentationBatch);

	connect(m_UI->actionBuildingSegmentEditor,		&QAction::triggered, this, &MainWindow::doActionBuildingSegmentEditor);
	connect(m_UI->actionPointClassEditor,			&QAction::triggered, this, &MainWindow::doActionPointClassEditor);

	connect(m_UI->actionSettingsGroundFiltering,	&QAction::triggered, this, &MainWindow::doAactionSettingsGroundFiltering);
	connect(m_UI->actionSettingsClassification,		&QAction::triggered, this, &MainWindow::doActionSettingsClassification);
	connect(m_UI->actionSettingsBuildingSeg,		&QAction::triggered, this, &MainWindow::doActionSettingsBuildingSeg);

	//! schedule
	connect(m_UI->actionScheduleProjectID,			&QAction::triggered, this, &MainWindow::doActionScheduleProjectID);
	connect(m_UI->actionScheduleGCServer,			&QAction::triggered, this, &MainWindow::doActionScheduleGCServer);
	connect(m_UI->actionScheduleGCNode,				&QAction::triggered, this, &MainWindow::doActionScheduleGCNode);
	
	connect(m_UI->actionClearEmptyItems,			&QAction::triggered, this, &MainWindow::doActionClearEmptyItems);
	
}

void MainWindow::doActionChangeTabTree(int index)
{
	m_UI->propertiesTreeView_Main->setVisible(index == 0);
 	m_UI->propertiesTreeView_Build->setVisible(index == 1);
 	m_UI->propertiesTreeView_Image->setVisible(index == 2);
}

void MainWindow::updateDBSelection(CC_TYPES::DB_SOURCE type)
{
	if (!(QApplication::keyboardModifiers() & Qt::ControlModifier)) {
		switch (type)
		{
		case CC_TYPES::DB_MAINDB:
			m_buildingRoot->unselectAllEntities();
			m_imageRoot->unselectAllEntities();
			break;
		case CC_TYPES::DB_BUILDING:
			m_ccRoot->unselectAllEntities();
			m_imageRoot->unselectAllEntities();
			break;
		case CC_TYPES::DB_IMAGE:
			m_ccRoot->unselectAllEntities();
			m_buildingRoot->unselectAllEntities();
			break;
		default:
			break;
		}
	}	
}

//bool s_display_overlay_image = false;
void MainWindow::toggleImageOverlay()
{
//	s_display_overlay_image = !s_display_overlay_image;
//	m_UI->actionImageOverlay->setChecked(s_display_overlay_image);
//	m_pbdrImagePanel->OverlayToolButton->setChecked(s_display_overlay_image);

	//! add a camera sensor to GL database
	//ccCameraSensor* camera_sensor = new ccCameraSensor();
}

void MainWindow::clearImagePanel()
{
	m_pbdrImagePanel->clearAll();
}

void MainWindow::CreateImageEditor()
{
	m_pbdrImshow = new bdr2Point5DimEditor();
	m_pbdrImshow->create2DView(m_UI->mapFrame);
	m_pbdrImshow->getGLWindow()->addSceneDB(m_imageRoot->getRootEntity());
	connect(m_pbdrImshow->getGLWindow(), &ccGLWindow::mouseMoved3D, this, &MainWindow::echoImageCursorPos);

	m_pbdrImagePanel = new bdrImageEditorPanel(m_pbdrImshow, m_imageRoot, this);	
	m_UI->verticalLayoutImageEditor->addWidget(m_pbdrImagePanel);	
	
//	connect(m_pbdrImagePanel->OverlayToolButton, &QAbstractButton::clicked, this, &MainWindow::toggleImageOverlay);
//	connect(m_pbdrImagePanel->displayBestToolButton, &QAbstractButton::clicked, this, &MainWindow::doActionShowBestImage);

	connect(m_pbdrImagePanel, &bdrImageEditorPanel::imageDisplayed, this, &MainWindow::doActionShowSelectedImage);
}

void MainWindow::CreateEditorPanel()
{
	m_pbdrPlaneEditDlg = new bdrPlaneEditorDlg(m_pickingHub, this);
	m_pbdrPlaneEditDlg->hide();

	m_UI->verticalLayoutPanel->addWidget(m_pbdrPlaneEditDlg);
}

void MainWindow::Link3DAnd2DWindow()
{
	if (!m_pbdrImshow) { return; }
	m_pbdrImshow->setAssociate3DView(getActiveGLWindow());
	ccGLWindow* win = m_pbdrImshow->getAssociate3DView();
	if (win) {
		connect(win, &ccGLWindow::mouseMoved3D, m_pbdrImagePanel, &bdrImageEditorPanel::updateCursorPos);
//	connect(win, &ccGLWindow::destroyed, m_pbdrImshow, &bdr2Point5DimEditor::destroyAss3DView);
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

int MainWindow::getGLWindowCount() const
{
	return m_mdiArea ? m_mdiArea->subWindowList().size() : 0;
}

void MainWindow::prepareWindowDeletion(QObject* glWindow)
{
	if (!m_ccRoot || !m_imageRoot || !m_buildingRoot)
		return;

	//we assume only ccGLWindow can be connected to this slot!
	ccGLWindow* win = qobject_cast<ccGLWindow*>(glWindow);

	m_ccRoot->hidePropertiesView();
	m_ccRoot->getRootEntity()->removeFromDisplay_recursive(win);
	m_ccRoot->updatePropertiesView();

	m_buildingRoot->hidePropertiesView();
	m_buildingRoot->getRootEntity()->removeFromDisplay_recursive(win);
	m_buildingRoot->updatePropertiesView();

	m_imageRoot->hidePropertiesView();
	m_imageRoot->getRootEntity()->removeFromDisplay_recursive(win);
	m_imageRoot->updatePropertiesView();
}

void MainWindow::showEvent(QShowEvent* event)
{
	QMainWindow::showEvent(event);

	if (!m_FirstShow)
	{
		return;
	}

	QSettings settings;
	QVariant  geometry = settings.value(ccPS::MainWinGeom());

	if (geometry.isValid())
	{
		restoreGeometry(geometry.toByteArray());
		restoreState(settings.value(ccPS::MainWinState()).toByteArray());
	}

	m_FirstShow = false;

	if (!geometry.isValid())
	{
		showMaximized();
	}

	if (isFullScreen())
	{
		m_UI->actionFullScreen->setChecked(true);
	}

#ifdef Q_OS_MAC
	if (isFullScreen())
	{
		m_UI->actionFullScreen->setText(tr("Exit Full Screen"));
	}
	else
	{
		m_UI->actionFullScreen->setText(tr("Enter Full Screen"));
	}
#endif
}

static bool s_autoSaveGuiElementPos = true;
void MainWindow::closeEvent(QCloseEvent *event)
{
	// If we don't have anything displayed, then just close...
	if (m_ccRoot && (m_ccRoot->getRootEntity()->getChildrenNumber() == 0) &&
		m_buildingRoot && (m_buildingRoot->getRootEntity()->getChildrenNumber() == 0) &&
		m_imageRoot && (m_imageRoot->getRootEntity()->getChildrenNumber() == 0))
	{
		event->accept();
	}
	else	// ...otherwise confirm
	{
		QMessageBox message_box(QMessageBox::Question,
			tr("Quit"),
			tr("Are you sure you want to quit?"),
			QMessageBox::Ok | QMessageBox::Cancel,
			this);

		if (message_box.exec() == QMessageBox::Ok)
		{
			event->accept();
		}
		else
		{
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
		if (s_pickingWindow != nullptr)
		{
			cancelPreviousPickingOperation(true);
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

void MainWindow::freezeUI(bool state)
{
	//freeze standard plugins
	m_UI->toolBarMainTools->setDisabled(state);
	m_UI->toolBarSFTools->setDisabled(state);

	m_pluginUIManager->mainPluginToolbar()->setDisabled(state);

	//freeze plugin toolbars
	for (QToolBar *toolbar : m_pluginUIManager->additionalPluginToolbars())
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

	switch (s_currentPickingOperation)
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
		s_pickingWindow->displayNewMessage(QString(), ccGLWindow::LOWER_LEFT_MESSAGE); //clear previous messages
		s_pickingWindow->displayNewMessage("Picking operation aborted", ccGLWindow::LOWER_LEFT_MESSAGE);
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
		ccLog::Warning("The point picked was picked in the wrong window");
		return;
	}

	CCVector3 pickedPoint = pi.P3D;
	switch (s_currentPickingOperation)
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
			mat[0] = X.x; mat[4] = X.y; mat[8] = X.z; mat[12] = 0;
			mat[1] = Y.x; mat[5] = Y.y; mat[9] = Y.z; mat[13] = 0;
			mat[2] = Z.x; mat[6] = Z.y; mat[10] = Z.z; mat[14] = 0;
			mat[3] = 0; mat[7] = 0; mat[11] = 0; mat[15] = 1;

			CCVector3d T = -CCVector3d::fromArray(A->u);
			trans.apply(T);
			T += CCVector3d::fromArray(A->u);
			trans.setTranslation(T);

			assert(haveOneSelection() && m_selectedEntities.front() == s_levelEntity);
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
			s_pickingWindow->displayNewMessage(QString(), ccGLWindow::LOWER_LEFT_MESSAGE, false); //clear previous message
			s_pickingWindow->displayNewMessage(QString("Point (%1 ; %2 ; %3) set as rotation center for interactive transformation")
				.arg(pickedPoint.x, 0, 'f', precision)
				.arg(pickedPoint.y, 0, 'f', precision)
				.arg(pickedPoint.z, 0, 'f', precision),
				ccGLWindow::LOWER_LEFT_MESSAGE, true);
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

void MainWindow::unselectAllInDB()
{
	if (m_ccRoot)
	{
		m_ccRoot->unselectAllEntities(); //we don't want any entity selected (especially existing labels!)
	}
	if (m_buildingRoot)
	{
		m_buildingRoot->unselectAllEntities(); //we don't want any entity selected (especially existing labels!)
	}
	if (m_imageRoot)
	{
		m_imageRoot->unselectAllEntities(); //we don't want any entity selected (especially existing labels!)
	}
}

void MainWindow::switchDatabase(CC_TYPES::DB_SOURCE src)
{
	int i = -1;
	switch (src)
	{
	case CC_TYPES::DB_MAINDB:
		i = 0;
		break;
	case CC_TYPES::DB_BUILDING:
		i = 1;
		break;
	case CC_TYPES::DB_IMAGE:
		i = 2;
		break;
	default:
		break;
	}
	if (i < 0)return;
	
	m_UI->ProjectTabWidget->setCurrentIndex(i);
	if (m_UI->propertiesTreeView_Main->isVisible() != (i == 0)) {
		m_UI->propertiesTreeView_Main->setVisible(i == 0);
	}
	if (m_UI->propertiesTreeView_Build->isVisible() != (i == 1)) {
		m_UI->propertiesTreeView_Build->setVisible(i == 1);
	}
	if (m_UI->propertiesTreeView_Image->isVisible() != (i == 2)) {
		m_UI->propertiesTreeView_Image->setVisible(i == 2);
	}
}

ccDBRoot * MainWindow::db(CC_TYPES::DB_SOURCE tp)
{
	switch (tp)
	{
	case CC_TYPES::DB_MAINDB:
		return db_main();
		break;
	case CC_TYPES::DB_BUILDING:
		return db_building();
		break;
	case CC_TYPES::DB_IMAGE:
		return db_image();
		break;
	default:
		break;
	}
	return nullptr;
}

ccPointCloud* MainWindow::askUserToSelectACloud(ccHObject* defaultCloudEntity/*=0*/, QString inviteMessage/*=QString()*/)
{
	ccHObject::Container clouds;
	ccDBRoot* root = db(getCurrentDB());
	root->getRootEntity()->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, true);
	if (clouds.empty())
	{
		ccConsole::Error("No cloud in database!");
		return 0;
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
			return 0;
	}

	assert(selectedIndex >= 0 && static_cast<size_t>(selectedIndex) < clouds.size());
	return ccHObjectCaster::ToPointCloud(clouds[selectedIndex]);
}

void MainWindow::spawnHistogramDialog(const std::vector<unsigned>& histoValues, double minVal, double maxVal, QString title, QString xAxisLabel)
{
	ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);
	hDlg->setAttribute(Qt::WA_DeleteOnClose, true);
	hDlg->setWindowTitle("Histogram");

	ccHistogramWindow* histogram = hDlg->window();
	{
		histogram->setTitle(title);
		histogram->fromBinArray(histoValues, minVal, maxVal);
		histogram->setAxisLabels(xAxisLabel, "Count");
		histogram->refresh();
	}

	hDlg->show();
}

ccColorScalesManager* MainWindow::getColorScalesManager()
{
	return ccColorScalesManager::GetUniqueInstance();
}

//////////////////////////////////////////////////////////////////////////

// DB

CC_TYPES::DB_SOURCE MainWindow::getCurrentDB()
{
	CC_TYPES::DB_SOURCE tp;
	if (m_UI->ProjectTabWidget->currentIndex() == 0) {
		tp = CC_TYPES::DB_MAINDB;
	}
	else if (m_UI->ProjectTabWidget->currentIndex() == 1) {
		tp = CC_TYPES::DB_BUILDING;
	}
	else if (m_UI->ProjectTabWidget->currentIndex() == 2) {
		tp = CC_TYPES::DB_IMAGE;
	}
	return tp;
}

void MainWindow::removeFromDB(ccHObject* obj, bool autoDelete/*=true*/)
{
	if (!obj)
		return;

	//remove dependency to avoid deleting the object when removing it from DB tree
	if (!autoDelete && obj->getParent())
		obj->getParent()->removeDependencyWith(obj);

	ccDBRoot* root = db(obj->getDBSourceType());
	if (root)
		root->removeElement(obj);
}

void MainWindow::setSelectedInDB(ccHObject* obj, bool selected)
{

	ccDBRoot* root = obj ? db(obj->getDBSourceType()) : nullptr;
	if (root)
	{
		if (selected)
			root->selectEntity(obj);
		else
			root->unselectEntity(obj);
	}
}

void MainWindow::addToDB(ccHObject* obj,
	CC_TYPES::DB_SOURCE dest,
	bool updateZoom/*=true*/,
	bool autoExpandDBTree/*=true*/,
	bool checkDimensions/*=true*/,
	bool autoRedraw/*=true*/)
{
	obj->setDBSourceType_recursive(dest);

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
		bool preserveCoordinateShift = true;
		//here we must test that coordinates are not too big whatever the case because OpenGL
		//really doesn't like big ones (even if we work with GLdoubles :( ).
		if (ccGlobalShiftManager::Handle(P, diag, ccGlobalShiftManager::DIALOG_IF_NECESSARY, false, Pshift, &preserveCoordinateShift, &scale))
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
				ccConsole::Warning(QString("Entity '%1' has been translated: (%2,%3,%4) and rescaled of a factor %5 [original position will be restored when saving]").arg(obj->getName()).arg(Pshift.x, 0, 'f', 2).arg(Pshift.y, 0, 'f', 2).arg(Pshift.z, 0, 'f', 2).arg(scale, 0, 'f', 6));
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
	ccDBRoot* root = db(dest);

	if (/*m_ccRoot*/root)
	{
		//force a 'global zoom' if the DB was emtpy!
		if (!root->getRootEntity() || root->getRootEntity()->getChildrenNumber() == 0)
		{
			updateZoom = true;
		}
		root->addElement(obj, autoExpandDBTree);
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

void MainWindow::addToDB(ccHObject* obj,
	bool updateZoom/* = false*/,
	bool autoExpandDBTree/* = true*/,
	bool checkDimensions /*= false*/,
	bool autoRedraw /*= true*/) 
{
	addToDB(obj, getCurrentDB(), updateZoom, autoExpandDBTree, checkDimensions, autoRedraw);
}

void MainWindow::addToDBAuto(const QStringList& filenames)
{
	ccGLWindow* win = qobject_cast<ccGLWindow*>(QObject::sender());
	ccHObject::Container loaded = addToDB(filenames, getCurrentDB(), QString(), win);
	
	for (ccHObject* obj : loaded) {
		if (!obj) continue;
			
		ccHObject::Container pcs;
		if (obj->isGroup()) {
			pcs = GetEnabledObjFromGroup(obj, CC_TYPES::POINT_CLOUD, false);
		}
		else {
			if (obj->isA(CC_TYPES::POINT_CLOUD)) {
				pcs.push_back(obj);
			}
		}
		int point_size = 0;

		ProgStartNorm("prepare point clouds", pcs.size())
			for (ccHObject* _p : pcs) {
				ccPointCloud* pcObj = ccHObjectCaster::ToPointCloud(_p);
				if (!pcObj) { continue; }
				bool exportDims[3] = { false,false,true };
				pcObj->exportCoordToSF(exportDims);
				point_size += pcObj->size();

				pcObj->prepareDisplayForRefresh();
				ProgStep()
			}
		ProgEnd

			if (point_size > 10000000) {
				ccEntityAction::computeOctree(pcs, this, true);
				std::cout << point_size << " points lod computed" << std::endl;
			}
	}
	refreshAll();
	UpdateUI();
}

std::vector<ccHObject*> MainWindow::addToDB(const QStringList& filenames,
	CC_TYPES::DB_SOURCE dest,
	QString fileFilter/*=QString()*/,
	ccGLWindow* destWin/*=0*/)
{
	ccHObject::Container loads;
	//to use the same 'global shift' for multiple files
	CCVector3d loadCoordinatesShift(0, 0, 0);
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

	const ccOptions& options = ccOptions::Instance();
	FileIOFilter::ResetSesionCounter();

	for (const QString &filename : filenames)
	{
		CC_FILE_ERROR result = CC_FERR_NO_ERROR;
		ccHObject* newGroup = FileIOFilter::LoadFromFile(filename, parameters, result, fileFilter);

		//! add even if the new group is empty
		loads.push_back(newGroup);

		if (newGroup)
		{
			if (!options.normalsDisplayedByDefault)
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
			addToDB(newGroup, dest, true, true, false);
			newGroup->setPath(filename);

			m_recentFiles->addFilePath(filename);
		}

		if (result == CC_FERR_CANCELED_BY_USER)
		{
			//stop importing the file if the user has cancelled the current process!
			break;
		}
	}

//	QMainWindow::statusBar()->showMessage(QString("%1 file(s) loaded").arg(filenames.size()), 2000);
	return loads;
}

ccHObject* MainWindow::dbRootObject(CC_TYPES::DB_SOURCE rt)
{
	ccDBRoot* root = db(rt);
	return (root ? root->getRootEntity() : nullptr);
}

ccHObject* MainWindow::dbRootObject()
{
	return dbRootObject(getCurrentDB());
}

MainWindow::ccHObjectContext MainWindow::removeObjectTemporarilyFromDBTree(ccHObject* obj)
{
	ccHObjectContext context;

	assert(obj);
	ccDBRoot* root = db(obj->getDBSourceType());
	if (!root || !obj)
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

	root->removeElement(obj);

	return context;
}

void MainWindow::putObjectBackIntoDBTree(ccHObject* obj, const ccHObjectContext& context)
{
	assert(obj);
	if (!obj || (!m_ccRoot && !m_buildingRoot && !m_imageRoot))
		return;

	if (context.parent)
	{
		context.parent->addChild(obj, context.parentFlags);
		obj->addDependency(context.parent, context.childFlags);
	}

	//DGM: we must call 'notifyGeometryUpdate' as any call to this method
	//while the object was temporarily 'cut' from the DB tree were
	//ineffective!
	obj->notifyGeometryUpdate();

	ccDBRoot* root = db(obj->getDBSourceType());
	if (root) {
		root->addElement(obj, false);
	}
}

void MainWindow::handleNewLabel(ccHObject* entity)
{
	if (entity)
	{
		addToDB(entity, getCurrentDB()); // TODO
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

ccHObject* MainWindow::askUserToSelect(CC_CLASS_ENUM type, ccHObject* defaultCloudEntity/*=0*/, QString inviteMessage/*=QString()*/, ccHObject* root /*= nullptr*/)
{
	ccHObject::Container entites;
	if (!root) {
		ccDBRoot* db_root_ = db(getCurrentDB());
		if (db_root_) root = db_root_->getRootEntity();
	}
	if (root) {
		root->filterChildren(entites, true, type, true);
	}
	
	if (entites.empty()) {
		ccConsole::Error("No data in database!");
		return 0;
	}
	//default selected index
	int selectedIndex = 0;
	if (defaultCloudEntity) {
		for (size_t i = 1; i < entites.size(); ++i) {
			if (entites[i] == defaultCloudEntity) {
				selectedIndex = static_cast<int>(i);
				break;
			}
		}
	}
	//ask the user to choose a cloud
	{
		selectedIndex = ccItemSelectionDlg::SelectEntity(entites, selectedIndex, this, inviteMessage);
		if (selectedIndex < 0)
			return 0;
	}

	assert(selectedIndex >= 0 && static_cast<size_t>(selectedIndex) < entites.size());
	return entites[selectedIndex];
}

//////////////////////////////////////////////////////////////////////////

void MainWindow::on3DViewActivated(QMdiSubWindow* mdiWin)
{
	ccGLWindow* win = mdiWin ? GLWindowFromWidget(mdiWin->widget()) : nullptr;
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

		m_UI->actionPointViewEditMode->blockSignals(true);
		m_UI->actionPointViewEditMode->setChecked(win->pointViewEditMode());
		m_UI->actionPointViewEditMode->blockSignals(false);

		m_UI->actionAutoPickRotationCenter->blockSignals(true);
		m_UI->actionAutoPickRotationCenter->setChecked(win->autoPickPivotAtCenter());
		m_UI->actionAutoPickRotationCenter->blockSignals(false);
	}
	Link3DAnd2DWindow();

	m_UI->actionLockRotationAxis->setEnabled(win != nullptr);
	m_UI->actionEnableStereo->setEnabled(win != nullptr);
	m_UI->actionExclusiveFullScreen->setEnabled(win != nullptr);
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

void MainWindow::updatePivotVisibilityPopUpMenu(ccGLWindow* win)
{
	if (!m_pivotVisibilityPopupButton)
		return;

	//update the pivot visibility pop-up 'top' icon
	if (win)
	{
		QAction* visibilityAction = nullptr;
		switch (win->getPivotVisibility())
		{
		case ccGLWindow::PIVOT_HIDE:
			visibilityAction = m_UI->actionSetPivotOff;
			break;
		case ccGLWindow::PIVOT_SHOW_ON_MOVE:
			visibilityAction = m_UI->actionSetPivotRotationOnly;
			break;
		case ccGLWindow::PIVOT_ALWAYS_SHOW:
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
	ccGLWindow* active3DView = getActiveGLWindow();
	bool hasMdiChild = (active3DView != nullptr);
	int mdiChildCount = getGLWindowCount();
	bool hasLoadedEntities = (m_ccRoot && m_ccRoot->getRootEntity() && m_ccRoot->getRootEntity()->getChildrenNumber() != 0) ||
		(m_buildingRoot && m_buildingRoot->getRootEntity() && m_buildingRoot->getRootEntity()->getChildrenNumber() != 0) ||
		(m_imageRoot && m_imageRoot->getRootEntity() && m_imageRoot->getRootEntity()->getChildrenNumber() != 0);
	bool hasSelectedEntities = (m_ccRoot && m_ccRoot->countSelectedEntities() > 0) ||
		(m_buildingRoot && m_buildingRoot->countSelectedEntities() > 0) ||
		(m_imageRoot && m_imageRoot->countSelectedEntities() > 0);

	//General Menu
	m_UI->menuEdit->setEnabled(true/*hasSelectedEntities*/);
	m_UI->menuTools->setEnabled(true/*hasSelectedEntities*/);

	//3D Views Menu
	m_UI->actionClose3DView->setEnabled(hasMdiChild);
	m_UI->actionCloseAll3DViews->setEnabled(mdiChildCount != 0);
	m_UI->actionTile3DViews->setEnabled(mdiChildCount > 1);
	m_UI->actionCascade3DViews->setEnabled(mdiChildCount > 1);
	m_UI->actionNext3DView->setEnabled(mdiChildCount > 1);
	m_UI->actionPrevious3DView->setEnabled(mdiChildCount > 1);

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

		for (QMdiSubWindow *window : windows)
		{
			ccGLWindow *child = GLWindowFromWidget(window->widget());

			QString text = QString("&%1 %2").arg(++i).arg(child->windowTitle());
			QAction *action = m_UI->menu3DViews->addAction(text);

			action->setCheckable(true);
			action->setChecked(child == getActiveGLWindow());

			connect(action, &QAction::triggered, this, [=]() {
				setActiveSubWindow(window);
			});
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
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		GLWindowFromWidget(window->widget())->redraw(only2D);
	}
}

void MainWindow::refreshAll(bool only2D/*=false*/)
{
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		GLWindowFromWidget(window->widget())->refresh(only2D);
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
	if (m_buildingRoot)
	{
		m_buildingRoot->updatePropertiesView();
	}
	if (m_imageRoot)
	{
		m_imageRoot->updatePropertiesView();
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

	ccHObject::Container sel_bd;
	if (m_buildingRoot)
	{
		dbTreeSelectionInfo selInfo_img;
		m_buildingRoot->getSelectedEntities(sel_bd, CC_TYPES::OBJECT, &selInfo_img);
		m_selectedEntities.insert(m_selectedEntities.end(), sel_bd.begin(), sel_bd.end());

		selInfo.selCount += selInfo_img.selCount;
		selInfo.sfCount += selInfo_img.sfCount;
		selInfo.colorCount += selInfo_img.colorCount;
		selInfo.normalsCount += selInfo_img.normalsCount;
		selInfo.groupCount += selInfo_img.groupCount;
		selInfo.cloudCount += selInfo_img.cloudCount;
		selInfo.octreeCount += selInfo_img.octreeCount;
		selInfo.gridCound += selInfo_img.gridCound;
		selInfo.meshCount += selInfo_img.meshCount;
		selInfo.planeCount += selInfo_img.planeCount;
		selInfo.polylineCount += selInfo_img.polylineCount;
		selInfo.sensorCount += selInfo_img.sensorCount;
		selInfo.gblSensorCount += selInfo_img.gblSensorCount;
		selInfo.cameraSensorCount += selInfo_img.cameraSensorCount;
		selInfo.kdTreeCount += selInfo_img.kdTreeCount;
	}

	ccHObject::Container sel_img;
	if (m_imageRoot)
	{
		dbTreeSelectionInfo selInfo_img;
		m_imageRoot->getSelectedEntities(sel_img, CC_TYPES::OBJECT, &selInfo_img);
		m_selectedEntities.insert(m_selectedEntities.end(), sel_img.begin(), sel_img.end());

		selInfo.selCount += selInfo_img.selCount;
		selInfo.sfCount += selInfo_img.sfCount;
		selInfo.colorCount += selInfo_img.colorCount;
		selInfo.normalsCount += selInfo_img.normalsCount;
		selInfo.groupCount += selInfo_img.groupCount;
		selInfo.cloudCount += selInfo_img.cloudCount;
		selInfo.octreeCount += selInfo_img.octreeCount;
		selInfo.gridCound += selInfo_img.gridCound;
		selInfo.meshCount += selInfo_img.meshCount;
		selInfo.planeCount += selInfo_img.planeCount;
		selInfo.polylineCount += selInfo_img.polylineCount;
		selInfo.sensorCount += selInfo_img.sensorCount;
		selInfo.gblSensorCount += selInfo_img.gblSensorCount;
		selInfo.cameraSensorCount += selInfo_img.cameraSensorCount;
		selInfo.kdTreeCount += selInfo_img.kdTreeCount;
	}

	switchDatabase(getCurrentDB());

	enableUIItems(selInfo);
	updateViewStateWithSelection();
}

void MainWindow::updateViewStateWithSelection()
{
	if (m_selectedEntities.size() != 1) {
		return;
	}
	ccHObject* obj = m_selectedEntities[0];
	if (obj->isKindOf(CC_TYPES::MESH)) {
		ccMesh* mesh = ccHObjectCaster::ToMesh(obj);
		if (mesh) {
			m_UI->actionDisplayWireframe->setChecked(mesh->isShownAsWire());
			m_UI->actionDisplayFace->setChecked(mesh->isShownAsFace());
			m_UI->actionDisplayNormalPerFace->setChecked(mesh->isShownAsFace() && mesh->triNormsShown());
			m_UI->actionDisplayNormalPerVertex->setChecked(mesh->isShownAsFace() && mesh->normalsShown() && !mesh->triNormsShown());
		}
	}
	else if (obj->isKindOf(CC_TYPES::POINT_CLOUD)) {
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(obj);
		if (cloud) {
			m_UI->actionDisplayWireframe->setChecked(false);
			m_UI->actionDisplayFace->setChecked(false);
			m_UI->actionDisplayNormalPerFace->setChecked(false);
			m_UI->actionDisplayNormalPerVertex->setChecked(cloud->normalsShown());
		}
	}
}

void MainWindow::enableAll()
{
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		window->setEnabled(true);
	}
}

void MainWindow::disableAll()
{
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		window->setEnabled(false);
	}
}

void MainWindow::disableAllBut(ccGLWindow* win)
{
	//we disable all other windows
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		if (GLWindowFromWidget(window->widget()) != win)
		{
			window->setEnabled(false);
		}
	}
}

void MainWindow::enableUIItems(dbTreeSelectionInfo& selInfo)
{
	bool dbIsEmpty = (!m_ccRoot || !m_ccRoot->getRootEntity() || m_ccRoot->getRootEntity()->getChildrenNumber() == 0) &&
		(!m_buildingRoot || !m_buildingRoot->getRootEntity() || m_buildingRoot->getRootEntity()->getChildrenNumber() == 0) &&
		(!m_imageRoot || !m_imageRoot->getRootEntity() || m_imageRoot->getRootEntity()->getChildrenNumber() == 0);
	bool atLeastOneEntity = (selInfo.selCount > 0);
	bool atLeastOneCloud = (selInfo.cloudCount > 0);
	bool atLeastOneMesh = (selInfo.meshCount > 0);
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
	m_UI->actionClone->setEnabled(atLeastOneEntity);
	m_UI->actionDelete->setEnabled(atLeastOneEntity);
	m_UI->actionExportCoordToSF->setEnabled(atLeastOneEntity);
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
	m_UI->actionFitSphere->setEnabled(atLeastOneCloud);
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
	m_UI->actionDistanceToBestFitQuadric3D->setEnabled(atLeastOneCloud);
	m_UI->actionDistanceMap->setEnabled(atLeastOneMesh || atLeastOneCloud);

	m_UI->menuMeshScalarField->setEnabled(atLeastOneSF && atLeastOneMesh);
	//actionSmoothMeshSF->setEnabled(atLeastOneSF && atLeastOneMesh);
	//actionEnhanceMeshSF->setEnabled(atLeastOneSF && atLeastOneMesh);

	m_UI->actionOrientNormalsMST->setEnabled(atLeastOneCloud && atLeastOneNormal);
	m_UI->actionOrientNormalsFM->setEnabled(atLeastOneCloud && atLeastOneNormal);
	m_UI->actionClearNormals->setEnabled(atLeastOneNormal);
	m_UI->actionInvertNormals->setEnabled(atLeastOneNormal);
	m_UI->actionConvertNormalToHSV->setEnabled(atLeastOneNormal);
	m_UI->actionConvertNormalToDipDir->setEnabled(atLeastOneNormal);
	m_UI->actionClearColor->setEnabled(atLeastOneColor);
	m_UI->actionRGBToGreyScale->setEnabled(atLeastOneColor);
	m_UI->actionEnhanceRGBWithIntensities->setEnabled(atLeastOneColor);

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
	m_UI->actionEditGlobalScale->setEnabled(exactlyOneCloud || exactlyOneMesh);
	m_UI->actionComputeKdTree->setEnabled(exactlyOneCloud || exactlyOneMesh);
	m_UI->actionShowWaveDialog->setEnabled(exactlyOneCloud);
	m_UI->actionCompressFWFData->setEnabled(atLeastOneCloud);

	m_UI->actionKMeans->setEnabled(/*TODO: exactlyOneEntity && exactlyOneSF*/false);
	m_UI->actionFrontPropagation->setEnabled(/*TODO: exactlyOneEntity && exactlyOneSF*/false);

	//actionCreatePlane->setEnabled(true);
	m_UI->actionEditPlane->setEnabled(selInfo.planeCount == 1);

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
	m_UI->actionPointPairsAlign->setEnabled(exactlyOneEntity || exactlyTwoEntities);
	m_UI->actionAlign->setEnabled(exactlyTwoEntities); //Aurelien BEY le 13/11/2008
	m_UI->actionCloudCloudDist->setEnabled(exactlyTwoClouds);
	m_UI->actionCloudMeshDist->setEnabled(exactlyTwoEntities && atLeastOneMesh);
	m_UI->actionCPS->setEnabled(exactlyTwoClouds);
	m_UI->actionScalarFieldArithmetic->setEnabled(exactlyOneEntity && atLeastOneSF);

	//>1
	bool atLeastTwoEntities = (selInfo.selCount > 1);

	m_UI->actionMerge->setEnabled(atLeastTwoEntities);
	m_UI->actionMatchBBCenters->setEnabled(atLeastTwoEntities);
	m_UI->actionMatchScales->setEnabled(atLeastTwoEntities);

	//standard plugins
	m_pluginUIManager->handleSelectionChanged();

	//////////////////////////////////////////////////////////////////////////
	// Building Reconstruction
//	m_UI->actionBDImage_Lines->setEnabled(atLeastOneEntity);
	m_UI->actionBDPlane_Deduction->setEnabled(atLeastOneEntity);
	m_UI->actionBDPolyFit->setEnabled(atLeastOneEntity);
}

void MainWindow::echoMouseWheelRotate(float wheelDelta_deg)
{
	if (!m_UI->actionEnableCameraLink->isChecked())
		return;

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		ccGLWindow *child = GLWindowFromWidget(window->widget());
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
	if (!m_UI->actionEnableCameraLink->isChecked())
		return;

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		ccGLWindow *child = GLWindowFromWidget(window->widget());
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
	if (!m_UI->actionEnableCameraLink->isChecked())
		return;

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		ccGLWindow *child = GLWindowFromWidget(window->widget());
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
	if (!m_UI->actionEnableCameraLink->isChecked())
		return;

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;


	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		ccGLWindow *child = GLWindowFromWidget(window->widget());
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
	if (!m_UI->actionEnableCameraLink->isChecked())
		return;

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		ccGLWindow *child = GLWindowFromWidget(window->widget());
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
	if (!m_UI->actionEnableCameraLink->isChecked())
		return;

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		ccGLWindow *child = GLWindowFromWidget(window->widget());
		if (child != sendingWindow)
		{
			child->blockSignals(true);
			child->setPixelSize(pixelSize);
			child->blockSignals(false);
			child->redraw();
		}
	}
}

void MainWindow::echoMouseMoved3D(const CCVector3d & P, bool b3d)
{
	QString string = "3D ";
	if (b3d) {
		if (m_UI->actionDisplayGlobalCoord) {
			// TODO: 
		}
		string += QString("(%1, %2, %3)").arg(P.x).arg(P.y).arg(P.z);
	}
	m_status_coord3D->setText(string);
}

void MainWindow::echoMouseMoved2D(int x, int y, double depth)
{
	QString string = QString("GLdepth %1").arg(depth, 0, 'f', 6);
	m_status_depth->setText(string);
}

void MainWindow::echopointSnapBufferChanged(int buffer)
{
	m_status_pointSnapBufferSpinBox->setValue(buffer);
}

void MainWindow::echoImageCursorPos(const CCVector3d & P, bool b3d)
{
	setStatusImageCoord(P, b3d);
}

void MainWindow::pointSnapBufferChanged(int buffer)
{
	if (getActiveGLWindow())
		getActiveGLWindow()->setPointPickBuffer(buffer);
}

void MainWindow::setStatusImageCoord(const CCVector3d & P, bool b3d)
{
	QString string = "Image ";
	if (b3d) {
		string += QString("(%1, %2)").arg(P.x).arg(P.y);
	}
	m_status_coord2D->setText(string);
}

void MainWindow::dispToConsole(QString message, ConsoleMessageLevel level/*=STD_CONSOLE_MESSAGE*/)
{
	switch (level)
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

//////////////////////////////////////////////////////////////////////////

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
	s_instance = nullptr;
}

void MainWindow::GetGLWindows(std::vector<ccGLWindow*>& glWindows)
{
	const QList<QMdiSubWindow*> windows = TheInstance()->m_mdiArea->subWindowList();

	if (windows.empty())
		return;

	glWindows.clear();
	glWindows.reserve(windows.size());

	for (QMdiSubWindow *window : windows)
	{
		glWindows.push_back(GLWindowFromWidget(window->widget()));
	}
	ccGLWindow* imgGL = TheInstance()->m_pbdrImshow->getGLWindow();
	if (imgGL) {
		glWindows.push_back(imgGL);
	}
}

ccGLWindow* MainWindow::GetActiveGLWindow()
{
	return TheInstance()->getActiveGLWindow();
}

ccGLWindow* MainWindow::GetGLWindow(const QString& title)
{
	const QList<QMdiSubWindow *> windows = TheInstance()->m_mdiArea->subWindowList();

	if (windows.empty())
		return nullptr;

	for (QMdiSubWindow *window : windows)
	{
		ccGLWindow* win = GLWindowFromWidget(window->widget());
		if (win->windowTitle() == title)
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

void MainWindow::addEditPlaneAction(QMenu &menu) const
{
	menu.addAction(m_UI->actionEditPlane);
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

///////////////////////////////////////////////////////////////////////// !Menu-General command
//! general

void MainWindow::toggleSelectedEntitiesProperty(ccEntityAction::TOGGLE_PROPERTY property)
{
	if (!ccEntityAction::toggleProperty(m_selectedEntities, property))
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::clearSelectedEntitiesProperty(ccEntityAction::CLEAR_PROPERTY property)
{
	if (!ccEntityAction::clearProperty(m_selectedEntities, property, this))
	{
		return;
	}

	refreshAll();
	updateUI();
}

//////////////////////////////////////////////////////////////////////////

//"File" menu

void MainWindow::doActionLoadFile()
{
	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::LoadFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
	QString currentOpenDlgFilter = settings.value(ccPS::SelectedInputFilter(), BinFilter::GetFileFilter()).toString();

	// Add all available file I/O filters (with import capabilities)
	const QStringList filterStrings = FileIOFilter::ImportFilterList();
	const QString &allFilter = filterStrings.at(0);

	if (!filterStrings.contains(currentOpenDlgFilter))
	{
		currentOpenDlgFilter = allFilter;
	}

	//file choosing dialog
	QStringList selectedFiles = QFileDialog::getOpenFileNames(this,
		tr("Open file(s)"),
		currentPath,
		filterStrings.join(s_fileFilterSeparator),
		&currentOpenDlgFilter,
		CCFileDialogOptions());
	if (selectedFiles.isEmpty())
		return;

	//save last loading parameters
	currentPath = QFileInfo(selectedFiles[0]).absolutePath();
	settings.setValue(ccPS::CurrentPath(), currentPath);
	settings.setValue(ccPS::SelectedInputFilter(), currentOpenDlgFilter);
	settings.endGroup();

	if (currentOpenDlgFilter == allFilter)
	{
		currentOpenDlgFilter.clear(); //this way FileIOFilter will try to guess the file type automatically!
	}

	//load files
	addToDB(selectedFiles, getCurrentDB(), currentOpenDlgFilter);
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

			//we don't want double insertions if the user has clicked both the father and child
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

	int stdSaveTypes = static_cast<int>(hasCloud)
		+ static_cast<int>(hasMesh)
		+ static_cast<int>(hasImages)
		+ static_cast<int>(hasPolylines)
		+ static_cast<int>(hasSerializable);
	if (stdSaveTypes == 0)
	{
		ccConsole::Error("Can't save selected entity(ies) this way!");
		return;
	}

	//we set up the right file filters, depending on the selected
	//entities type (cloud, mesh, etc.).
	QStringList fileFilters;
	{
		for (const FileIOFilter::Shared &filter : FileIOFilter::GetFilters())
		{
			bool atLeastOneExclusive = false;

			//can this filter export one or several clouds?
			bool canExportClouds = true;
			if (hasCloud)
			{
				bool isExclusive = true;
				bool multiple = false;
				canExportClouds = (filter->canSave(CC_TYPES::POINT_CLOUD, multiple, isExclusive)
					&& (multiple || clouds.getChildrenNumber() == 1));
				atLeastOneExclusive |= isExclusive;
			}

			//can this filter export one or several meshes?
			bool canExportMeshes = true;
			if (hasMesh)
			{
				bool isExclusive = true;
				bool multiple = false;
				canExportMeshes = (filter->canSave(CC_TYPES::MESH, multiple, isExclusive)
					&& (multiple || meshes.getChildrenNumber() == 1));
				atLeastOneExclusive |= isExclusive;
			}

			//can this filter export one or several polylines?
			bool canExportPolylines = true;
			if (hasPolylines)
			{
				bool isExclusive = true;
				bool multiple = false;
				canExportPolylines = (filter->canSave(CC_TYPES::POLY_LINE, multiple, isExclusive)
					&& (multiple || polylines.getChildrenNumber() == 1));
				atLeastOneExclusive |= isExclusive;
			}

			//can this filter export one or several images?
			bool canExportImages = true;
			if (hasImages)
			{
				bool isExclusive = true;
				bool multiple = false;
				canExportImages = (filter->canSave(CC_TYPES::IMAGE, multiple, isExclusive)
					&& (multiple || images.getChildrenNumber() == 1));
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
					canExportSerializables &= (filter->canSave(child->getClassID(), multiple, isExclusive)
						&& (multiple || otherSerializable.getChildrenNumber() == 1));
					atLeastOneExclusive |= isExclusive;
				}
			}

			bool useThisFilter = canExportClouds
				&& canExportMeshes
				&&	canExportImages
				&&	canExportPolylines
				&&	canExportSerializables
				&& (!atLeastOneExclusive || stdSaveTypes == 1);

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
		//with a path inside perturbs the QFileDialog a lot ;))
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
		ccConsole::Warning("[I/O] The following selected entities won't be saved:");
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
		if (haveOneSelection())
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

		result = FileIOFilter::SaveToFile(entitiesToSave.getChildrenNumber() > 1 ? &entitiesToSave : entitiesToSave.getChild(0),
			selectedFilename,
			parameters,
			selectedFilter);

		if (result == CC_FERR_NO_ERROR)
		{
			unselectAllInDB();
		}
	}

	//update default filters
	if (hasCloud)
		settings.setValue(ccPS::SelectedOutputFilterCloud(), selectedFilter);
	if (hasMesh)
		settings.setValue(ccPS::SelectedOutputFilterMesh(), selectedFilter);
	if (hasImages)
		settings.setValue(ccPS::SelectedOutputFilterImage(), selectedFilter);
	if (hasPolylines)
		settings.setValue(ccPS::SelectedOutputFilterPoly(), selectedFilter);

	//we update current file path
	currentPath = QFileInfo(selectedFilename).absolutePath();
	settings.setValue(ccPS::CurrentPath(), currentPath);
	settings.endGroup();
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

void MainWindow::doShowPrimitiveFactory()
{
	if (!m_pfDlg)
		m_pfDlg = new ccPrimitiveFactoryDlg(this);

	m_pfDlg->setModal(false);
	m_pfDlg->setWindowModality(Qt::NonModal);
	m_pfDlg->show();
}

void MainWindow::closeAll()
{
	if (!m_ccRoot || !m_imageRoot || !m_buildingRoot)
	{
		return;
	}

	QMessageBox message_box(QMessageBox::Question,
		tr("Close all"),
		tr("Are you sure you want to remove all loaded entities?"),
		QMessageBox::Yes | QMessageBox::No,
		this);

	if (message_box.exec() == QMessageBox::No)
	{
		return;
	}

	m_ccRoot->unloadAll();
	m_buildingRoot->unloadAll();
	m_imageRoot->unloadAll();

	redrawAll(false);
}

//////////////////////////////////////////////////////////////////////////

//"Edit > Colors" menu

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

//////////////////////////////////////////////////////////////////////////

//"Edit > Normals" menu

void MainWindow::doActionComputeNormals()
{
	if (!ccEntityAction::computeNormals(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionInvertNormals()
{
	if ( !ccEntityAction::invertNormals(m_selectedEntities) )
		return;

	refreshAll();
}

void MainWindow::doActionConvertNormalsToHSV()
{
	if (!ccEntityAction::convertNormalsTo(m_selectedEntities,
		ccEntityAction::NORMAL_CONVERSION_DEST::HSV_COLORS))
	{
		return;
	}

	refreshAll();
	updateUI();
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

void MainWindow::doActionOrientNormalsMST()
{
	if (!ccEntityAction::orientNormalsMST(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionOrientNormalsFM()
{
	if (!ccEntityAction::orientNormalsFM(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

//////////////////////////////////////////////////////////////////////////

//"Edit > Octree" menu

static double s_kdTreeMaxErrorPerCell = 0.1;
void MainWindow::doActionComputeKdTree()
{
	ccGenericPointCloud* cloud = nullptr;

	if ( haveOneSelection() )
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

	if (kdtree->build(s_kdTreeMaxErrorPerCell, CCLib::DistanceComputationTools::MAX_DIST_95_PERCENT, 4, 1000, &pDlg))
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

		addToDB(kdtree, cloud->getDBSourceType());

		refreshAll();
		updateUI();
	}
	else
	{
		ccLog::Error("An error occurred!");
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
	int pointCount = QInputDialog::getInt(this,"Resample with octree", "Points (approx.)", 1000000, 1, INT_MAX, 100000, &ok);
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
				ccConsole::Print("[ResampleWithOctree] Timing: %3.2f s.", eTimer.elapsed() / 1.0e3);
				ccPointCloud* newCloud = ccPointCloud::From(result, cloud);

				delete result;
				result = nullptr;

				if (newCloud)
				{
					addToDB(newCloud, cloud->getDBSourceType());
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

//////////////////////////////////////////////////////////////////////////

//"Edit > Grid" menu

void MainWindow::doActionDeleteScanGrids()
{
	//look for clouds with scan grids
	for (ccHObject *entity : getSelectedEntities())
	{
		if (!entity || !entity->isA(CC_TYPES::POINT_CLOUD))
		{
			continue;
		}

		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		assert(cloud);

		if (cloud->gridCount() > 0)
		{
			cloud->removeGrids();
		}
	}
	refreshAll();
	updateUI();
}

//////////////////////////////////////////////////////////////////////////

//"Edit > Mesh" menu

void MainWindow::doActionComputeMesh(CC_TRIANGULATION_TYPES type)
{
	//ask the user for the max edge length
	static double s_meshMaxEdgeLength = 0.0;
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
		for (ccHObject *entity : getSelectedEntities())
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
		updateNormals = (QMessageBox::question(this,
			"Keep old normals?",
			"Cloud(s) already have normals. Do you want to update them (yes) or keep the old ones (no)?",
			QMessageBox::Yes,
			QMessageBox::No) == QMessageBox::Yes);
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
		ccMesh* mesh = ccMesh::Triangulate(cloud,
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
			addToDB(mesh, cloud->getDBSourceType());
			if (i == 0)
			{
				setSelectedInDB(mesh, true); //auto-select first element
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

void MainWindow::doActionComputeMeshAA()
{
	doActionComputeMesh(DELAUNAY_2D_AXIS_ALIGNED);
}

void MainWindow::doActionComputeMeshLS()
{
	doActionComputeMesh(DELAUNAY_2D_BEST_LS_PLANE);
}

void MainWindow::doMeshTwoPolylines()
{
	if (m_selectedEntities.size() != 2)
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
		addToDB(mesh, p1->getDBSourceType());
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
	for (ccHObject *entity : getSelectedEntities())
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
				addToDB(gridMesh, cloud->getDBSourceType(), false, true, false, false);
				gridMesh->prepareDisplayForRefresh();
			}
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionConvertTextureToColor()
{
	if (!ccEntityAction::convertTextureToColor(m_selectedEntities, this))
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
	assert(dlg.getPointsNumber() >= 0);
	s_ptsSamplingCount = static_cast<unsigned>(dlg.getPointsNumber());
	s_ptsSamplingDensity = dlg.getDensityValue();
	s_ptsSampleNormals = withNormals;

	bool errors = false;

	for (ccHObject *entity : getSelectedEntities())
	{
		if (!entity->isKindOf(CC_TYPES::MESH))
			continue;

		ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(entity);
		assert(mesh);

		ccPointCloud* cloud = mesh->samplePoints(s_useDensity,
			s_useDensity ? s_ptsSamplingDensity : s_ptsSamplingCount,
			withNormals,
			withRGB,
			withTexture,
			&pDlg);

		if (cloud)
		{
			addToDB(cloud, entity->getDBSourceType());
		}
		else
		{
			errors = true;
		}
	}

	if (errors)
		ccLog::Error("[doActionSamplePointsOnMesh] Errors occurred during the process! Result may be incomplete!");

	refreshAll();
}

void MainWindow::doActionSmoothMeshLaplacian()
{
	static unsigned	s_laplacianSmooth_nbIter = 20;
	static double	s_laplacianSmooth_factor = 0.2;

	bool ok;
	s_laplacianSmooth_nbIter = QInputDialog::getInt(this, "Smooth mesh", "Iterations:", s_laplacianSmooth_nbIter, 1, 1000, 1, &ok);
	if (!ok)
		return;
	s_laplacianSmooth_factor = QInputDialog::getDouble(this, "Smooth mesh", "Smoothing factor:", s_laplacianSmooth_factor, 0, 100, 3, &ok);
	if (!ok)
		return;

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	for (ccHObject *entity : getSelectedEntities())
	{
		if (entity->isA(CC_TYPES::MESH) || entity->isA(CC_TYPES::PRIMITIVE)) //FIXME: can we really do this with primitives?
		{
			ccMesh* mesh = ccHObjectCaster::ToMesh(entity);

			if (mesh->laplacianSmooth(s_laplacianSmooth_nbIter,
				static_cast<PointCoordinateType>(s_laplacianSmooth_factor),
				&pDlg))
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

static double s_subdivideMaxArea = 1.0;
void MainWindow::doActionSubdivideMesh()
{
	bool ok;
	s_subdivideMaxArea = QInputDialog::getDouble(this, "Subdivide mesh", "Max area per triangle:", s_subdivideMaxArea, 1e-6, 1e6, 8, &ok);
	if (!ok)
		return;

	//ccProgressDialog pDlg(true, this);
	//pDlg.setAutoClose(false);

	for (ccHObject *entity : getSelectedEntities())
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
				catch (...)
				{
					ccLog::Error(QString("[Subdivide] An error occurred while trying to subdivide mesh '%1' (not enough memory?)").arg(mesh->getName()));
				}

				if (subdividedMesh)
				{
					subdividedMesh->setName(QString("%1.subdivided(S<%2)").arg(mesh->getName()).arg(s_subdivideMaxArea));
					subdividedMesh->setDisplay(mesh->getDisplay());
					mesh->redrawDisplay();
					mesh->setEnabled(false);
					addToDB(subdividedMesh, entity->getDBSourceType());
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

void MainWindow::doActionMeasureMeshSurface()
{
	for (ccHObject *entity : getSelectedEntities())
	{
		if (entity->isKindOf(CC_TYPES::MESH))
		{
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(entity);
			if (mesh)
			{
				double S = CCLib::MeshSamplingTools::computeMeshArea(mesh);
				//we force the console to display itself
				forceConsoleDisplay();
				ccConsole::Print(QString("[Mesh Surface] Mesh '%1': S=%2 (square units)").arg(entity->getName()).arg(S));
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

void MainWindow::doActionMeasureMeshVolume()
{
	for (ccHObject *entity : getSelectedEntities())
	{
		if (entity->isKindOf(CC_TYPES::MESH))
		{
			ccMesh* mesh = ccHObjectCaster::ToMesh(entity);
			if (mesh)
			{
				//we compute the mesh volume
				double V = CCLib::MeshSamplingTools::computeMeshVolume(mesh);
				//we force the console to display itself
				forceConsoleDisplay();
				ccConsole::Print(QString("[Mesh Volume] Mesh '%1': V=%2 (cube units)").arg(entity->getName()).arg(V));

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

void MainWindow::doActionFlagMeshVertices()
{
	bool errors = false;
	bool success = false;

	for (ccHObject *entity : getSelectedEntities())
	{
		if (entity->isKindOf(CC_TYPES::MESH))
		{
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(entity);
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
				if (CCLib::MeshSamplingTools::flagMeshVerticesByType(mesh, flags, &stats))
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
					ccConsole::Print(QString("[Mesh Quality] Mesh '%1' edges: %2 total (normal: %3 / on hole borders: %4 / non-manifold: %5)").arg(entity->getName()).arg(stats.edgesCount).arg(stats.edgesSharedByTwo).arg(stats.edgesNotShared).arg(stats.edgesSharedByMore));
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

//"Edit > Mesh > Scalar Field" menu

void MainWindow::doActionSmoothMeshSF()
{
	if (!ccEntityAction::processMeshSF(m_selectedEntities, ccMesh::SMOOTH_MESH_SF, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionEnhanceMeshSF()
{
	if (!ccEntityAction::processMeshSF(m_selectedEntities, ccMesh::ENHANCE_MESH_SF, this))
		return;

	refreshAll();
	updateUI();
}

//////////////////////////////////////////////////////////////////////////

//"Edit > Polyline" menu
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

	assert(dlg.getPointsNumber() >= 0);
	s_ptsSamplingCount = static_cast<unsigned>(dlg.getPointsNumber());
	s_ptsSamplingDensity = dlg.getDensityValue();
	s_useDensity = dlg.useDensity();

	bool errors = false;

	for (ccHObject *entity : getSelectedEntities())
	{
		if (!entity->isKindOf(CC_TYPES::POLY_LINE))
			continue;

		ccPolyline* poly = ccHObjectCaster::ToPolyline(entity);
		assert(poly);

		ccPointCloud* cloud = poly->samplePoints(s_useDensity,
			s_useDensity ? s_ptsSamplingDensity : s_ptsSamplingCount,
			true);

		if (cloud)
		{
			addToDB(cloud, entity->getDBSourceType());
		}
		else
		{
			errors = true;
		}
	}

	if (errors)
	{
		ccLog::Error("[doActionSamplePointsOnPolyline] Errors occurred during the process! Result may be incomplete!");
	}

	refreshAll();
}

//////////////////////////////////////////////////////////////////////////

//"Edit > Plane" menu

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

//////////////////////////////////////////////////////////////////////////

//"Edit > Sensor > Ground-Based lidar" menu

void MainWindow::doActionShowDepthBuffer()
{
	if (!haveSelection())
		return;

	for (ccHObject *entity : getSelectedEntities())
	{
		if (entity->isKindOf(CC_TYPES::GBL_SENSOR))
		{
			ccGBLSensor* sensor = static_cast<ccGBLSensor*>(m_selectedEntities[0]);
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
					ccConsole::Error(QString("Internal error: sensor ('%1') parent is not a point cloud!").arg(sensor->getName()));
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
		"Select output file",
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
	settings.setValue(ccPS::CurrentPath(), QFileInfo(filename).absolutePath());
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

		for (ccHObject *entity : getSelectedEntities())
		{
			toSave->addChild(entity, ccHObject::DP_NONE);
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
		FileIOFilter::DisplayErrorMessage(result, "saving", filename);
	}
	else
	{
		ccLog::Print(QString("[I/O] File '%1' saved successfully").arg(filename));
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
	if (!haveOneSelection() || !m_selectedEntities[0]->isKindOf(CC_TYPES::GBL_SENSOR))
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
			if (QMessageBox::warning(this,
				"Depth buffer.",
				"Sensor has no depth buffer: do you want to compute it now?",
				QMessageBox::Yes | QMessageBox::No,
				QMessageBox::Yes) == QMessageBox::No)
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
		CCLib::NormalizedProgress nprogress(&pdlg, pointCloud->size());
		pdlg.setMethodTitle(tr("Compute visibility"));
		pdlg.setInfo(tr("Points: %L1").arg(pointCloud->size()));
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

//////////////////////////////////////////////////////////////////////////

//"Edit > Sensor" menu

void MainWindow::doActionCreateGBLSensor()
{
	ccGBLSensorProjectionDlg spDlg(this);
	if (!spDlg.exec())
		return;

	//We create the corresponding sensor for each input cloud (in a perfect world, there should be only one ;)
	for (ccHObject *entity : getSelectedEntities())
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
				if (sensor->computeDepthBuffer(cloud, errorCode))
				{
					ccRenderingTools::ShowDepthBuffer(sensor, this);
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

				addToDB(sensor, entity->getDBSourceType());
			}
			else
			{
				ccLog::Error("Failed to create sensor");
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

	ccGLWindow* win = nullptr;
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

	addToDB(sensor, ent ? ent->getDBSourceType() : getCurrentDB());

	updateUI();
}

void MainWindow::doActionModifySensor()
{
	//there should be only one point cloud with sensor in current selection!
	if (!haveOneSelection() || !m_selectedEntities[0]->isKindOf(CC_TYPES::SENSOR))
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
			if (gbl->computeDepthBuffer(cloud, errorCode))
			{
				//we display depth buffer
				ccRenderingTools::ShowDepthBuffer(gbl, this);
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

void MainWindow::doActionProjectUncertainty()
{
	//there should only be one sensor in the current selection!
	if (!haveOneSelection() || !m_selectedEntities[0]->isKindOf(CC_TYPES::CAMERA_SENSOR))
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
	points.addPointIndex(0, pointCloud->size());

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
	const char dimChar[3] = { 'x','y','z' };
	for (unsigned d = 0; d < 3; ++d)
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
	if (!haveOneSelection() || !m_selectedEntities[0]->isKindOf(CC_TYPES::CAMERA_SENSOR))
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

				for (unsigned index : inCameraFrustum)
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

void MainWindow::doActionComputeDistancesFromSensor()
{
	//we support more than just one sensor in selection
	if (!haveSelection())
	{
		ccConsole::Error("Select at least a sensor.");
		return;
	}

	//start dialog
	ccSensorComputeDistancesDlg cdDlg(this);
	if (!cdDlg.exec())
		return;

	for (ccHObject *entity : getSelectedEntities())
	{
		ccSensor* sensor = ccHObjectCaster::ToSensor(entity);
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

		for (unsigned i = 0; i < cloud->size(); ++i)
		{
			const CCVector3* P = cloud->getPoint(i);
			ScalarType s = static_cast<ScalarType>(squared ? (*P - sensorCenter).norm2() : (*P - sensorCenter).norm());
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
	if (!haveOneSelection() || !m_selectedEntities[0]->isKindOf(CC_TYPES::GBL_SENSOR))
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
			theta *= static_cast<ScalarType>(CC_RAD_TO_DEG);

		angles->setValue(i, theta);
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
	if (!haveOneSelection() || !m_selectedEntities[0]->isKindOf(CC_TYPES::SENSOR))
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

//////////////////////////////////////////////////////////////////////////

//"Edit > Scalar fields" menu

void MainWindow::showSelectedEntitiesHistogram()
{
	for (ccHObject *entity : getSelectedEntities())
	{
		//for "real" point clouds only
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (cloud)
		{
			//we display the histogram of the current scalar field
			ccScalarField* sf = static_cast<ccScalarField*>(cloud->getCurrentDisplayedScalarField());
			if (sf)
			{
				ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);
				hDlg->setAttribute(Qt::WA_DeleteOnClose, true);
				hDlg->setWindowTitle(QString("Histogram [%1]").arg(cloud->getName()));

				ccHistogramWindow* histogram = hDlg->window();
				{
					unsigned numberOfPoints = cloud->size();
					unsigned numberOfClasses = static_cast<unsigned>(sqrt(static_cast<double>(numberOfPoints)));
					//we take the 'nearest' multiple of 4
					numberOfClasses &= (~3);
					numberOfClasses = std::max<unsigned>(4, numberOfClasses);
					numberOfClasses = std::min<unsigned>(256, numberOfClasses);

					histogram->setTitle(QString("%1 (%2 values) ").arg(sf->getName()).arg(numberOfPoints));
					bool showNaNValuesInGrey = sf->areNaNValuesShownInGrey();
					histogram->fromSF(sf, numberOfClasses, true, showNaNValuesInGrey);
					histogram->setAxisLabels(sf->getName(), "Count");
					histogram->refresh();
				}
				hDlg->show();
			}
		}
	}
}

void MainWindow::doActionSFGradient()
{
	if (!ccLibAlgorithms::ApplyCCLibAlgorithm(ccLibAlgorithms::CCLIB_ALGO_SF_GRADIENT, m_selectedEntities, this))
		return;
	refreshAll();
	updateUI();
}

void MainWindow::doActionSFGaussianFilter()
{
	if (!ccEntityAction::sfGaussianFilter(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionSFBilateralFilter()
{
	if (!ccEntityAction::sfBilateralFilter(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionFilterByValue()
{
	typedef std::pair<ccHObject*, ccPointCloud*> EntityAndVerticesType;
	std::vector<EntityAndVerticesType> toFilter;

	for (ccHObject *entity : getSelectedEntities())
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);
		if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD))
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
			//la methode est activee sur le champ scalaire affiche
			CCLib::ScalarField* sf = pc->getCurrentDisplayedScalarField();
			if (sf)
			{
				toFilter.emplace_back(entity, pc);
			}
			else
			{
				ccConsole::Warning(QString("Entity [%1] has no active scalar field !").arg(entity->getName()));
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
		for (auto &item : toFilter)
		{
			ccHObject* ent = item.first;
			ccPointCloud* pc = item.second;
			//CCLib::ScalarField* sf = pc->getCurrentDisplayedScalarField();
			//assert(sf);

			//we set as output (OUT) the currently displayed scalar field
			int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
			assert(outSfIdx >= 0);
			pc->setCurrentOutScalarField(outSfIdx);
			//pc->setCurrentScalarField(outSfIdx);

			ccHObject* resultInside = nullptr;
			ccHObject* resultOutside = nullptr;
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
				addToDB(resultInside, ent->getDBSourceType());

				results.push_back(resultInside);
			}
			if (resultOutside)
			{
				ent->setEnabled(false);
				resultOutside->setDisplay(ent->getDisplay());
				resultOutside->prepareDisplayForRefresh();
				resultOutside->setName(resultOutside->getName() + ".outside");
				addToDB(resultOutside, ent->getDBSourceType());

				results.push_back(resultOutside);
			}
			//*/
		}
	}

	if (!results.empty())
	{
		ccConsole::Warning("Previously selected entities (sources) have been hidden!");
		ccDBRoot* root = db(results.front()->getDBSourceType());
		if (root)
		{
			root->selectEntities(results);
		}
	}

	refreshAll();
}

static double s_constantSFValue = 0.0;
void MainWindow::doActionAddConstantSF()
{
	if (!haveOneSelection())
	{
		if (haveSelection())
			ccConsole::Error("Select only one cloud or one mesh!");
		return;
	}

	ccHObject* ent = m_selectedEntities[0];

	bool lockedVertices;
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent, &lockedVertices);

	//for "real" point clouds only
	if (!cloud)
		return;
	if (lockedVertices && !ent->isAncestorOf(cloud))
	{
		ccUtils::DisplayLockedVerticesWarning(ent->getName(), true);
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
	QString sfName = QInputDialog::getText(this, "New SF name", "SF name (must be unique)", QLineEdit::Normal, defaultName, &ok);
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

void MainWindow::doActionScalarFieldArithmetic()
{
	if (!ccEntityAction::sfArithmetic(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionScalarFieldFromColor()
{
	if (!ccEntityAction::sfFromColor(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionSFConvertToRGB()
{
	if (!ccEntityAction::sfConvertToRGB(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionSFConvertToRandomRGB()
{
	if (!ccEntityAction::sfConvertToRandomRGB(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionRenameSF()
{
	if (!ccEntityAction::sfRename(m_selectedEntities, this))
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
	if (!ccEntityAction::sfAddIdField(m_selectedEntities))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionSetSFAsCoord()
{
	if (!ccEntityAction::sfSetAsCoord(m_selectedEntities, this))
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

//////////////////////////////////////////////////////////////////////////

//"Edit > Waveform" menu

void MainWindow::doActionShowWaveDialog()
{
	if (!haveSelection())
		return;

	ccHObject* entity = haveOneSelection() ? m_selectedEntities[0] : nullptr;
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

void MainWindow::doActionCompressFWFData()
{
	for (ccHObject *entity : getSelectedEntities())
	{
		if (!entity || !entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			continue;
		}

		ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);
		cloud->compressFWFData();
	}
}

//////////////////////////////////////////////////////////////////////////

//"Edit" menu

void MainWindow::doActionClone()
{
	ccHObject* lastClone = nullptr;

	for (ccHObject *entity : getSelectedEntities())
	{
		ccHObject* clone = nullptr;

		if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			clone = ccHObjectCaster::ToGenericPointCloud(entity)->clone();
			if (!clone)
			{
				ccConsole::Error(QString("An error occurred while cloning cloud %1").arg(entity->getName()));
			}
		}
		else if (entity->isKindOf(CC_TYPES::PRIMITIVE))
		{
			clone = static_cast<ccGenericPrimitive*>(entity)->clone();
			if (!clone)
			{
				ccConsole::Error(QString("An error occurred while cloning primitive %1").arg(entity->getName()));
			}
		}
		else if (entity->isA(CC_TYPES::MESH))
		{
			clone = ccHObjectCaster::ToMesh(entity)->cloneMesh();
			if (!clone)
			{
				ccConsole::Error(QString("An error occurred while cloning mesh %1").arg(entity->getName()));
			}
		}
		else if (entity->isA(CC_TYPES::POLY_LINE))
		{
			ccPolyline* poly = ccHObjectCaster::ToPolyline(entity);
			clone = (poly ? new ccPolyline(*poly) : 0);
			if (!clone)
			{
				ccConsole::Error(QString("An error occurred while cloning polyline %1").arg(entity->getName()));
			}
		}
		else if (entity->isA(CC_TYPES::FACET))
		{
			ccFacet* facet = ccHObjectCaster::ToFacet(entity);
			clone = (facet ? facet->clone() : 0);
			if (!clone)
			{
				ccConsole::Error(QString("An error occurred while cloning facet %1").arg(entity->getName()));
			}
		}
		else
		{
			ccLog::Warning(QString("Entity '%1' can't be cloned (type not supported yet!)").arg(entity->getName()));
		}

		if (clone)
		{
			//copy GL transformation history
			clone->setGLTransformationHistory(entity->getGLTransformationHistory());
			//copy display
			clone->setDisplay(entity->getDisplay());

			addToDB(clone, entity->getDBSourceType());
			lastClone = clone;
		}
	}

	if (lastClone)
	{
		setSelectedInDB(lastClone, true);
	}

	updateUI();
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
		for (ccHObject *entity : getSelectedEntities())
		{
			if (!entity)
				continue;

			if (entity->isA(CC_TYPES::POINT_CLOUD))
			{
				ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
				clouds.push_back(cloud);
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
					ccConsole::Warning(QString("Only meshes with standard vertices are handled for now! Can't merge entity '%1'...").arg(entity->getName()));
				}
			}
			else
			{
				ccConsole::Warning(QString("Entity '%1' is neither a cloud nor a mesh, can't merge it!").arg(entity->getName()));
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
		unselectAllInDB();
		assert(!haveSelection());

		//we will remove the useless clouds/meshes later
		ccHObject::Container toBeRemoved;

		ccPointCloud* firstCloud = nullptr;
		ccHObjectContext firstCloudContext;

		//whether to generate the 'original cloud index' scalar field or not
		CCLib::ScalarField* ocIndexSF = nullptr;
		size_t cloudIndex = 0;

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

				if (QMessageBox::question(this, "Original cloud index", "Do you want to generate a scalar field with the original cloud index?") == QMessageBox::Yes)
				{
					int sfIdx = pc->getScalarFieldIndexByName(CC_ORIGINAL_CLOUD_INDEX_SF_NAME);
					if (sfIdx < 0)
					{
						sfIdx = pc->addScalarField(CC_ORIGINAL_CLOUD_INDEX_SF_NAME);
					}
					if (sfIdx < 0)
					{
						ccConsole::Error("Couldn't allocate a new scalar field for storing the original cloud index! Try to free some memory ...");
						return;
					}
					else
					{
						ocIndexSF = pc->getScalarField(sfIdx);
						ocIndexSF->fill(0);
						firstCloud->setCurrentDisplayedScalarField(sfIdx);
					}
				}
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

					ccHObject* toRemove = nullptr;
					//if the entity to remove is a group with a unique child, we can remove it as well
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
					ccConsole::Error("Fusion failed! (not enough memory?)");
					break;
				}
				pc = nullptr;
			}
		}

		if (ocIndexSF)
		{
			ocIndexSF->computeMinAndMax();
			firstCloud->showSF(true);
		}

		//something to remove?
		while (!toBeRemoved.empty())
		{
			if (toBeRemoved.back())
			{
				ccDBRoot* root = db(toBeRemoved.back()->getDBSourceType());
				if (root) root->removeElement(toBeRemoved.back());
			}
			toBeRemoved.pop_back();
		}

		//put back first cloud in DB
		if (firstCloud)
		{
			putObjectBackIntoDBTree(firstCloud, firstCloudContext);
			setSelectedInDB(firstCloud, true);
		}
	}
	//merge meshes?
	else if (!meshes.empty())
	{
		bool createSubMeshes = true;
		//createSubMeshes = (QMessageBox::question(this, "Create sub-meshes", "Do you want to create sub-mesh entities corresponding to each source mesh? (requires more memory)", QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes);

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
				ccConsole::Error("Fusion failed! (not enough memory?)");
				break;
			}
		}

		baseMesh->setDisplay_recursive(meshes.front()->getDisplay());
		baseMesh->setVisible(true);
		addToDB(baseMesh, meshes.front()->getDBSourceType());
		setSelectedInDB(baseMesh, true);
	}

	refreshAll();
	updateUI();
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
	if (!haveSelection()) {
		return;
	}
	//if the transformation is partly converted to global shift/scale
	bool updateGlobalShiftAndScale = false;
	double scaleChange = 1.0;
	CCVector3d shiftChange(0, 0, 0);
	ccGLMatrixd transMat = mat;

	//we must backup 'm_selectedEntities' as removeObjectTemporarilyFromDBTree can modify it!
	ccHObject::Container selectedEntities = getSelectedEntities();

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

	bool firstCloud = true;

	for (ccHObject *entity : selectedEntities) //warning, getSelectedEntites may change during this loop!
	{
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

				if (firstCloud)
				{
					//test if the translated cloud was already "too big"
					//(in which case we won't bother the user about the fact
					//that the transformed cloud will be too big...)
					ccBBox localBBox = entity->getOwnBB();
					CCVector3d Pl = CCVector3d::fromArray(localBBox.minCorner().u);
					double Dl = localBBox.getDiagNormd();

					//the cloud was alright
					if (!ccGlobalShiftManager::NeedShift(Pl)
						&& !ccGlobalShiftManager::NeedRescale(Dl))
					{
						//test if the translated cloud is not "too big" (in local coordinate space)
						ccBBox rotatedBox = entity->getOwnBB() * transMat;
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
							globalTransMat.scale(1.0 / globalScale);
							globalTransMat.setTranslation(globalTransMat.getTranslationAsVec3D() - globalShift);
							//and we apply it to the cloud bounding-box
							ccBBox rotatedBox = cloud->getOwnBB() * globalTransMat;
							double Dg = rotatedBox.getDiagNorm();
							CCVector3d Pg = CCVector3d::fromArray(rotatedBox.getCenter().u);

							//ask the user the right values!
							ccShiftAndScaleCloudDlg sasDlg(Pl2, Dl2, Pg, Dg, this);
							sasDlg.showApplyAllButton(false);
							sasDlg.showTitle(true);
							sasDlg.setKeepGlobalPos(true);
							sasDlg.showKeepGlobalPosCheckbox(false); //we don't want the user to mess with this!
							sasDlg.showPreserveShiftOnSave(true);

							//add "original" entry
							int index = sasDlg.addShiftInfo(ccGlobalShiftManager::ShiftInfo("Original", globalShift, globalScale));
							//sasDlg.setCurrentProfile(index);
							//add "suggested" entry
							CCVector3d suggestedShift = ccGlobalShiftManager::BestShift(Pg);
							double suggestedScale = ccGlobalShiftManager::BestScale(Dg);
							index = sasDlg.addShiftInfo(ccGlobalShiftManager::ShiftInfo("Suggested", suggestedShift, suggestedScale));
							sasDlg.setCurrentProfile(index);
							//add "last" entry (if available)
							std::vector<ccGlobalShiftManager::ShiftInfo> lastInfos;
							if (ccGlobalShiftManager::GetLast(lastInfos))
							{
								sasDlg.addShiftInfo(lastInfos);
							}
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
									transMat.setTranslation(transMat.getTranslationAsVec3D() + shiftChange * scaleChange);
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
		ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(entity);
		entity->setGLTransformation(ccGLMatrix(transMat.data()));
		//DGM FIXME: we only test the entity own bounding box (and we update its shift & scale info) but we apply the transformation to all its children?!
		entity->applyGLTransformation_recursive();
		entity->prepareDisplayForRefresh_recursive();
		putObjectBackIntoDBTree(entity, objContext);
	}

	//reselect previously selected entities!
	ccDBRoot* root = db(selectedEntities.front());
	if (root) root->selectEntities(selectedEntities);

	ccLog::Print("[ApplyTransformation] Applied transformation matrix:");
	ccLog::Print(transMat.toString(12, ' ')); //full precision
	ccLog::Print("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool");

	refreshAll();
}

typedef std::pair<ccHObject*, ccGenericPointCloud*> EntityCloudAssociation;
void MainWindow::doActionApplyScale()
{
	if (!haveSelection()) {
		return;
	}
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
	std::vector< EntityCloudAssociation > candidates;
	{
		bool testBigCoordinates = true;
		//size_t processNum = 0;

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
					lockedVertices = true;
			}
			if (!cloud || !cloud->isKindOf(CC_TYPES::POINT_CLOUD))
			{
				ccLog::Warning(QString("[Apply scale] Entity '%1' can't be scaled this way").arg(entity->getName()));
				continue;
			}
			if (lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(entity->getName(), haveOneSelection());
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

				double maxx = std::max(std::abs(bbMin.x), std::abs(bbMax.x));
				double maxy = std::max(std::abs(bbMin.y), std::abs(bbMax.y));
				double maxz = std::max(std::abs(bbMin.z), std::abs(bbMax.z));

				const double maxCoord = ccGlobalShiftManager::MaxCoordinateAbsValue();
				bool oldCoordsWereTooBig = (maxx > maxCoord
					|| maxy > maxCoord
					|| maxz > maxCoord);

				if (!oldCoordsWereTooBig)
				{
					maxx = std::max(std::abs((bbMin.x - C.x) * scales.x + C.x), std::abs((bbMax.x - C.x) * scales.x + C.x));
					maxy = std::max(std::abs((bbMin.y - C.y) * scales.y + C.y), std::abs((bbMax.y - C.y) * scales.y + C.y));
					maxz = std::max(std::abs((bbMin.z - C.z) * scales.z + C.z), std::abs((bbMax.z - C.z) * scales.z + C.z));

					bool newCoordsAreTooBig = (maxx > maxCoord
						|| maxy > maxCoord
						|| maxz > maxCoord);

					if (newCoordsAreTooBig)
					{
						if (QMessageBox::question(
							this,
							"Big coordinates",
							"Resutling coordinates will be too big (original precision may be lost!). Proceed anyway?",
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
		ccConsole::Warning("[Apply scale] No eligible entities (point clouds or meshes) were selected!");
		return;
	}

	//now do the real scaling work
	{
		for (auto &candidate : candidates)
		{
			ccHObject* ent = candidate.first;
			ccGenericPointCloud* cloud = candidate.second;

			CCVector3 C(0, 0, 0);
			if (keepInPlace)
			{
				C = cloud->getOwnBB().getCenter();
			}

			//we temporarily detach entity, as it may undergo
			//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::scale
			ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(cloud);

			cloud->scale(static_cast<PointCoordinateType>(scales.x),
				static_cast<PointCoordinateType>(scales.y),
				static_cast<PointCoordinateType>(scales.z),
				C);

			putObjectBackIntoDBTree(cloud, objContext);
			cloud->prepareDisplayForRefresh_recursive();

			//don't forget the 'global shift'!
			//DGM: but not the global scale!
			if (rescaleGlobalShift)
			{
				const CCVector3d& shift = cloud->getGlobalShift();
				cloud->setGlobalShift(CCVector3d(shift.x*scales.x,
					shift.y*scales.y,
					shift.z*scales.z));
			}

			ent->prepareDisplayForRefresh_recursive();
		}
	}

	//reselect previously selected entities!
	ccDBRoot* root = db(selectedEntities.front());
	if (root) root->selectEntities(selectedEntities);

	if (!keepInPlace)
		zoomOnSelectedEntities();

	refreshAll();
	updateUI();
}


void MainWindow::doActionCrop()
{
	//find candidates
	std::vector<ccHObject*> candidates;
	ccBBox baseBB;
	{
		const ccHObject::Container& selectedEntities = getSelectedEntities();
		for (ccHObject *entity : selectedEntities)
		{
			if (entity->isA(CC_TYPES::POINT_CLOUD)
				|| entity->isKindOf(CC_TYPES::MESH))
			{
				candidates.push_back(entity);
				baseBB += entity->getOwnBB();
			}
		}
	}

	if (candidates.empty())
	{
		ccConsole::Warning("[Crop] No eligible candidate found!");
		return;
	}

	ccBoundingBoxEditorDlg bbeDlg(this);
	bbeDlg.setBaseBBox(baseBB, false);
	bbeDlg.showInclusionWarning(false);
	bbeDlg.setWindowTitle("Crop");

	if (!bbeDlg.exec())
	{
		//process cancelled by user
		return;
	}

	//deselect all entities
	unselectAllInDB();

	//cropping box
	ccBBox box = bbeDlg.getBox();

	//process cloud/meshes
	bool errors = false;
	bool successes = false;
	{
		for (ccHObject *entity : candidates)
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
				addToDB(croppedEnt, entity->getDBSourceType());
				//select output entity
				db(croppedEnt)->selectEntity(croppedEnt, true);
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
		CCVector3d globalBBmin(0, 0, 0), globalBBmax(0, 0, 0);

		for (ccHObject *entity : getSelectedEntities())
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
				assert(entity->isKindOf(CC_TYPES::MESH));
				ccGenericPointCloud* vertices = static_cast<ccGenericMesh*>(entity)->getAssociatedCloud();
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
				globalBBmin = CCVector3d(std::min(globalBBmin.x, Ag.x),
					std::min(globalBBmin.y, Ag.y),
					std::min(globalBBmin.z, Ag.z));
				globalBBmax = CCVector3d(std::max(globalBBmax.x, Bg.x),
					std::max(globalBBmax.y, Bg.y),
					std::max(globalBBmax.z, Bg.z));

				if (uniqueShift)
					uniqueShift = ((shifted->getGlobalShift() - shift).norm() < ZERO_TOLERANCE);
				if (uniqueScale)
					uniqueScale = (std::abs(shifted->getGlobalScale() - scale) < ZERO_TOLERANCE);
			}

			shiftedEntities.emplace_back(shifted, entity);
		}

		Pg = globalBBmin;
		Dg = (globalBBmax - globalBBmin).norm();

		Pl = CCVector3d::fromArray(localBB.minCorner().u);
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
	int index = sasDlg.addShiftInfo(ccGlobalShiftManager::ShiftInfo("Original", shift, scale));
	sasDlg.setCurrentProfile(index);
	//add "last" entry (if available)
	std::vector<ccGlobalShiftManager::ShiftInfo> lastInfos;
	if (ccGlobalShiftManager::GetLast(lastInfos))
	{
		sasDlg.addShiftInfo(lastInfos);
	}
	//add entries from file (if any)
	sasDlg.addFileInfo();

	if (!sasDlg.exec())
		return;

	shift = sasDlg.getShift();
	scale = sasDlg.getScale();
	bool preserveGlobalPos = sasDlg.keepGlobalPos();

	ccLog::Print("[Global Shift/Scale] New shift: (%f, %f, %f)", shift.x, shift.y, shift.z);
	ccLog::Print("[Global Shift/Scale] New scale: %f", scale);

	//apply new shift
	{
		for (auto &entity : shiftedEntities)
		{
			ccShiftedObject* shifted = entity.first;
			ccHObject* ent = entity.second;
			if (preserveGlobalPos)
			{
				//to preserve the global position of the cloud, we may have to translate and/or rescale the cloud
				CCVector3d Ql = CCVector3d::fromArray(ent->getOwnBB().minCorner().u);
				CCVector3d Qg = shifted->toGlobal3d(Ql);
				CCVector3d Ql2 = Qg * scale + shift;
				CCVector3d T = Ql2 - Ql;

				assert(shifted->getGlobalScale() > 0);
				double scaleCoef = scale / shifted->getGlobalScale();

				if (T.norm() > ZERO_TOLERANCE || std::abs(scaleCoef - 1.0) > ZERO_TOLERANCE)
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

void MainWindow::doActionSubsample()
{
	//find candidates
	std::vector<ccPointCloud*> clouds;
	unsigned maxPointCount = 0;
	double maxCloudRadius = 0;
	ScalarType sfMin = NAN_VALUE;
	ScalarType sfMax = NAN_VALUE;
	{
		for (ccHObject *entity : getSelectedEntities())
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
		ccConsole::Error("Select at least one point cloud!");
		return;
	}

	//Display dialog
	ccSubsamplingDlg sDlg(maxPointCount, maxCloudRadius, this);
	bool hasValidSF = ccScalarField::ValidValue(sfMin) && ccScalarField::ValidValue(sfMax);
	if (hasValidSF)
		sDlg.enableSFModulation(sfMin, sfMax);
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

		for (size_t i = 0; i < clouds.size(); ++i)
		{
			ccPointCloud* cloud = clouds[i];
			CCLib::ReferenceCloud *sampledCloud = sDlg.getSampledCloud(cloud, &pDlg);
			if (!sampledCloud)
			{
				ccConsole::Warning(QString("[Subsampling] Failed to subsample cloud '%1'!").arg(cloud->getName()));
				errors = true;
				continue;
			}

			int warnings = 0;
			ccPointCloud *newPointCloud = cloud->partialClone(sampledCloud, &warnings);

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
				addToDB(newPointCloud, cloud->getDBSourceType());

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

		ccLog::Print("[Subsampling] Timing: %3.3f s.", eTimer.elapsed() / 1000.0);

		if (errors)
		{
			ccLog::Error("Errors occurred (see console)");
		}
	}

	if (!resultingClouds.empty()) {
		ccDBRoot* root = db(resultingClouds.front());
		if (root) root->selectEntities(resultingClouds);
	}

	refreshAll();
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
		ccConsole::Print(QString("[Synchronize] Transformation matrix (%1):").arg(entity->getName()));
		ccConsole::Print(glTrans.toString(12, ' ')); //full precision
		ccConsole::Print("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool");

		//we temporarily detach entity, as it may undergo
		//"severe" modifications (octree deletion, etc.) --> see ccHObject::applyGLTransformation
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

		CCVector3 T = refCenter - center;

		//transformation (used only for translation)
		ccGLMatrix glTrans;
		glTrans += T;

		forceConsoleDisplay();
		ccConsole::Print(QString("[Synchronize] Transformation matrix (%1 --> %2):").arg(entity->getName(), selectedEntities[0]->getName()));
		ccConsole::Print(glTrans.toString(12, ' ')); //full precision
		ccConsole::Print("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool");

		//we temporarily detach entity, as it may undergo
		//"severe" modifications (octree deletion, etc.) --> see ccHObject::applyGLTransformation
		ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(entity);
		entity->applyGLTransformation_recursive(&glTrans);
		putObjectBackIntoDBTree(entity, objContext);

		entity->prepareDisplayForRefresh_recursive();
	}

	//reselect previously selected entities!
	if (!selectedEntities.empty()) {
		ccDBRoot* root = db(selectedEntities.front());
		if (root) root->selectEntities(selectedEntities);
	}

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
		for (ccHObject *entity : getSelectedEntities())
		{
			if (entity->isKindOf(CC_TYPES::POINT_CLOUD)
				|| entity->isKindOf(CC_TYPES::MESH))
			{
				selectedEntities.push_back(entity);
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

	ccLibAlgorithms::ApplyScaleMatchingAlgorithm(s_msAlgorithm,
		selectedEntities,
		s_msRmsDiff,
		s_msFinalOverlap,
		msDlg.getSelectedIndex(),
		this);

	//reselect previously selected entities!
	if (!selectedEntities.empty()) {
		ccDBRoot* root = db(selectedEntities.front());
		if (root) root->selectEntities(selectedEntities);
	}

	refreshAll();
	updateUI();
}

//////////////////////////////////////////////////////////////////////////

//"Tools > Clean" menu

void MainWindow::doActionSORFilter()
{
	ccSORFilterDlg sorDlg(this);

	//set semi-persistent/dynamic parameters
	static int s_sorFilterKnn = 6;
	static double s_sorFilterNSigma = 1.0;
	sorDlg.knnSpinBox->setValue(s_sorFilterKnn);
	sorDlg.nSigmaDoubleSpinBox->setValue(s_sorFilterNSigma);
	if (!sorDlg.exec())
		return;

	//update semi-persistent/dynamic parameters
	s_sorFilterKnn = sorDlg.knnSpinBox->value();
	s_sorFilterNSigma = sorDlg.nSigmaDoubleSpinBox->value();

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	bool firstCloud = true;

	ccHObject::Container selectedEntities = getSelectedEntities(); //we have to use a local copy: 'selectEntity' will change the set of currently selected entities!

	for (ccHObject *entity : selectedEntities)
	{
		//specific test for locked vertices
		bool lockedVertices;
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity, &lockedVertices);
		if (cloud && lockedVertices)
		{
			ccUtils::DisplayLockedVerticesWarning(entity->getName(), haveOneSelection());
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
				ccLog::Warning(QString("[doActionSORFilter] No points were removed from cloud '%1'").arg(cloud->getName()));
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
					addToDB(cleanCloud, cloud->getDBSourceType());

					cloud->setEnabled(false);
					if (firstCloud)
					{
						ccConsole::Warning("Previously selected entities (sources) have been hidden!");
						firstCloud = false;
						db(cleanCloud)->selectEntity(cleanCloud, true);
					}
				}
				else
				{
					ccConsole::Warning(QString("[doActionSORFilter] Not enough memory to create a clean version of cloud '%1'!").arg(cloud->getName()));
				}
			}

			delete selection;
			selection = nullptr;
		}
		else
		{
			//no points fall inside selection!
			if (cloud != nullptr)
			{
				ccConsole::Warning(QString("[doActionSORFilter] Failed to apply the noise filter to cloud '%1'! (not enough memory?)").arg(cloud->getName()));
			}
			else
			{
				ccConsole::Warning("[doActionSORFilter] Trying to apply the noise filter to null cloud");
			}
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionFilterNoise()
{
	PointCoordinateType kernelRadius = ccLibAlgorithms::GetDefaultCloudKernelSize(m_selectedEntities);

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
	kernelRadius = static_cast<PointCoordinateType>(noiseDlg.radiusDoubleSpinBox->value());
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

	for (ccHObject *entity : selectedEntities)
	{
		//specific test for locked vertices
		bool lockedVertices;
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity, &lockedVertices);
		if (cloud && lockedVertices)
		{
			ccUtils::DisplayLockedVerticesWarning(entity->getName(), haveOneSelection());
			continue;
		}

		//computation
		CCLib::ReferenceCloud* selection = CCLib::CloudSamplingTools::noiseFilter(cloud,
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
					cleanCloud->setName(cloud->getName() + QString(".clean"));
					cleanCloud->setDisplay(cloud->getDisplay());
					if (cloud->getParent())
						cloud->getParent()->addChild(cleanCloud);
					addToDB(cleanCloud, cloud->getDBSourceType());

					cloud->setEnabled(false);
					if (firstCloud)
					{
						ccConsole::Warning("Previously selected entities (sources) have been hidden!");
						firstCloud = false;
						db(cleanCloud)->selectEntity(cleanCloud, true);
					}
				}
				else
				{
					ccConsole::Warning(QString("[doActionFilterNoise] Not enough memory to create a clean version of cloud '%1'!").arg(cloud->getName()));
				}
			}

			delete selection;
			selection = nullptr;
		}
		else
		{
			//no points fall inside selection!
			if (cloud != nullptr)
			{
				ccConsole::Warning(QString("[doActionFilterNoise] Failed to apply the noise filter to cloud '%1'! (not enough memory?)").arg(cloud->getName()));
			}
			else
			{
				ccConsole::Warning("[doActionFilterNoise] Trying to apply the noise filter to null cloud");
			}
		}
	}

	refreshAll();
	updateUI();
}

//////////////////////////////////////////////////////////////////////////

//"Tools > Projection" menu

void MainWindow::doActionUnroll()
{
	//there should be only one point cloud with sensor in current selection!
	if (!haveOneSelection())
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

	ccPointCloud::UnrollMode mode = unrollDlg.getType();
	PointCoordinateType radius = static_cast<PointCoordinateType>(unrollDlg.getRadius());
	unsigned char dim = static_cast<unsigned char>(unrollDlg.getAxisDimension());
	bool exportDeviationSF = unrollDlg.exportDeviationSF();
	CCVector3 center = unrollDlg.getAxisPosition();

	//let's rock unroll ;)
	ccProgressDialog pDlg(true, this);

	double startAngle_deg = 0.0, stopAngle_deg = 360.0;
	unrollDlg.getAngleRange(startAngle_deg, stopAngle_deg);
	if (startAngle_deg >= stopAngle_deg)
	{
		QMessageBox::critical(this, "Error", "Invalid angular range");
		return;
	}

	ccPointCloud* output = nullptr;
	switch (mode)
	{
	case ccPointCloud::CYLINDER:
	{
		ccPointCloud::UnrollCylinderParams params;
		params.radius = radius;
		params.axisDim = dim;
		if (unrollDlg.isAxisPositionAuto())
		{
			center = pc->getOwnBB().getCenter();
		}
		params.center = center;
		output = pc->unroll(mode, &params, exportDeviationSF, startAngle_deg, stopAngle_deg, &pDlg);
	}
	break;

	case ccPointCloud::CONE:
	case ccPointCloud::STRAIGHTENED_CONE:
	case ccPointCloud::STRAIGHTENED_CONE2:
	{
		ccPointCloud::UnrollConeParams params;
		params.radius = (mode == ccPointCloud::CONE ? 0 : radius);
		params.apex = center;
		params.coneAngle_deg = unrollDlg.getConeHalfAngle();
		params.axisDim = dim;
		output = pc->unroll(mode, &params, exportDeviationSF, startAngle_deg, stopAngle_deg, &pDlg);
	}
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
		addToDB(output, pc->getDBSourceType(), true, true, false, true);

		updateUI();
	}
}

void MainWindow::doActionRasterize()
{
	if (!haveOneSelection())
	{
		ccConsole::Error("Select only one point cloud!");
		return;
	}

	ccHObject* ent = m_selectedEntities[0];
	if (!ent->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error("Select a point cloud!");
		return;
	}

	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);
	ccRasterizeTool rasterizeTool(cloud, this);
	rasterizeTool.exec();
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
			for (ccHObject *entity : getSelectedEntities())
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
	const unsigned char X = (Z == 2 ? 0 : Z + 1);
	const unsigned char Y = (X == 2 ? 0 : X + 1);

	//number of segments
	unsigned segmentCount = 0;
	unsigned vertexCount = 0;
	{
		for (ccPolyline *poly : polylines)
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
		for (ccPolyline *poly : polylines)
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
		for (ccPolyline *poly : polylines)
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
		addToDB(mesh, polylines.front()->getDBSourceType());
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
		delete vertices;
		vertices = nullptr;
	}
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

//////////////////////////////////////////////////////////////////////////

//"Tools > Registration" menu

void MainWindow::doActionRegister()
{
	if (m_selectedEntities.size() != 2
		|| (!m_selectedEntities[0]->isKindOf(CC_TYPES::POINT_CLOUD) && !m_selectedEntities[0]->isKindOf(CC_TYPES::MESH))
		|| (!m_selectedEntities[1]->isKindOf(CC_TYPES::POINT_CLOUD) && !m_selectedEntities[1]->isKindOf(CC_TYPES::MESH)))
	{
		ccConsole::Error("Select 2 point clouds or meshes!");
		return;
	}

	ccHObject *data = static_cast<ccHObject*>(m_selectedEntities[1]);
	ccHObject *model = static_cast<ccHObject*>(m_selectedEntities[0]);

	ccRegistrationDlg rDlg(data, model, this);
	if (!rDlg.exec())
		return;

	//DGM (23/01/09): model and data order may have changed!
	model = rDlg.getModelEntity();
	data = rDlg.getDataEntity();

	double minRMSDecrease = rDlg.getMinRMSDecrease();
	unsigned maxIterationCount = rDlg.getMaxIterationCount();
	unsigned randomSamplingLimit = rDlg.randomSamplingLimit();
	bool removeFarthestPoints = rDlg.removeFarthestPoints();
	bool useDataSFAsWeights = rDlg.useDataSFAsWeights();
	bool useModelSFAsWeights = rDlg.useModelSFAsWeights();
	bool adjustScale = rDlg.adjustScale();
	int transformationFilters = rDlg.getTransformationFilters();
	unsigned finalOverlap = rDlg.getFinalOverlap();
	CCLib::ICPRegistrationTools::CONVERGENCE_TYPE method = rDlg.getConvergenceMethod();
	int maxThreadCount = rDlg.getMaxThreadCount();

	//semi-persistent storage (for next call)
	rDlg.saveParameters();

	ccGLMatrix transMat;
	double finalError = 0.0;
	double finalScale = 1.0;
	unsigned finalPointCount = 0;

	if (ccRegistrationTools::ICP(data,
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
			summary << "Transformation matrix";
			summary << transMat.toString(3, '\t'); //low precision, just for display
			summary << "----------------";

			ccLog::Print("[Register] Applied transformation matrix:");
			ccLog::Print(transMat.toString(12, ' ')); //full precision
			ccLog::Print("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool");
		}

		if (adjustScale)
		{
			QString scaleString = QString("Scale: %1 (already integrated in above matrix!)").arg(finalScale);
			ccLog::Warning(QString("[Register] ") + scaleString);
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
		ccLog::Print(QString("[Register] ") + overlapString);
		summary << overlapString;

		summary << "----------------";
		summary << "This report has been output to Console (F8)";

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
				QMessageBox::StandardButton result = QMessageBox::question(this,
					"Registration",
					"Data mesh vertices are locked (they may be shared with other meshes): Do you wish to clone this mesh to apply transformation?",
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
						ccLog::Error("Doesn't work on sub-meshes yet!");
					}

					if (newMesh)
					{
						newMesh->setDisplay(data->getDisplay());
						addToDB(newMesh, data->getDBSourceType());
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
			putObjectBackIntoDBTree(pc, objContext);

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
						pc->setGlobalShift(0, 0, 0);
						pc->setGlobalScale(1.0);
						ccLog::Warning(QString("[ICP] Aligned entity global shift has been reset to match the reference!"));
					}
				}
			}

			data->prepareDisplayForRefresh_recursive();
			data->setName(data->getName() + QString(".registered"));
			zoomOn(data);
		}

		//pop-up summary
		QMessageBox::information(this, "Register info", summary.join("\n"));
		forceConsoleDisplay();
	}

	refreshAll();
	updateUI();
}

//////////////////////////////////////////////////////////////////////////

//"Tools > Distances" menu

void MainWindow::doActionCloudCloudDist()
{
	if (getSelectedEntities().size() != 2)
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

	ccOrderChoiceDlg dlg(m_selectedEntities[0], "Compared",
		m_selectedEntities[1], "Reference",
		this);
	if (!dlg.exec())
		return;

	ccGenericPointCloud* compCloud = ccHObjectCaster::ToGenericPointCloud(dlg.getFirstEntity());
	ccGenericPointCloud* refCloud = ccHObjectCaster::ToGenericPointCloud(dlg.getSecondEntity());

	//assert(!m_compDlg);
	if (m_compDlg)
		delete m_compDlg;
	m_compDlg = new ccComparisonDlg(compCloud, refCloud, ccComparisonDlg::CLOUDCLOUD_DIST, this);
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
		ccConsole::Error("Select 2 entities!");
		return;
	}

	bool isMesh[2] = { false,false };
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
		ccConsole::Error("Select at least one mesh!");
		return;
	}
	else if (meshNum + cloudNum < 2)
	{
		ccConsole::Error("Select one mesh and one cloud or two meshes!");
		return;
	}

	ccHObject* compEnt = nullptr;
	ccGenericMesh* refMesh = nullptr;

	if (meshNum == 1)
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
	connect(m_compDlg, &QDialog::finished, this, &MainWindow::deactivateComparisonMode);
	m_compDlg->show();

	freezeUI(true);
}

void MainWindow::doActionComputeCPS()
{
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

	ccOrderChoiceDlg dlg(m_selectedEntities[0], "Compared",
		m_selectedEntities[1], "Reference",
		this);
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
	int result = CCLib::DistanceComputationTools::computeCloud2CloudDistance(compCloud, srcCloud, params, &pDlg);
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
		addToDB(newCloud, compCloud->getDBSourceType());

		//we hide the source cloud (for a clearer display)
		srcCloud->setEnabled(false);
		srcCloud->prepareDisplayForRefresh();
	}

	refreshAll();
}

void MainWindow::doActionCloudModelDist()
{
	if (getSelectedEntities().size() != 2)
	{
		ccConsole::Error("Select 2 entities!");
		return;
	}

	bool isMesh[2] = { false,false };
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
		ccConsole::Error("Select at least one mesh!");
		return;
	}
	else if (meshNum + cloudNum < 2)
	{
		ccConsole::Error("Select one mesh and one cloud or two meshes!");
		return;
	}

	ccHObject* compEnt = nullptr;
	ccGenericMesh* refMesh = nullptr;

	if (meshNum == 1)
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
	m_compDlg = new ccComparisonDlg(compEnt, refMesh, ccComparisonDlg::CLOUDMODEL_DIST, this);
	connect(m_compDlg, &QDialog::finished, this, &MainWindow::deactivateComparisonMode);
	m_compDlg->show();

	freezeUI(true);
}

//////////////////////////////////////////////////////////////////////////

//"Tools > Volume" menu

void MainWindow::doCompute2HalfDimVolume()
{
	if (m_selectedEntities.empty() || m_selectedEntities.size() > 2)
	{
		ccConsole::Error("Select one or two point clouds!");
		return;
	}

	ccGenericPointCloud* cloud1 = nullptr;
	{
		ccHObject* ent = m_selectedEntities[0];
		if (!ent->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			ccConsole::Error("Select point clouds only!");
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
		if (!ent->isKindOf(CC_TYPES::POINT_CLOUD))
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

//////////////////////////////////////////////////////////////////////////

//"Tools > Statistics" menu

void MainWindow::doActionComputeStatParams()
{
	ccEntityAction::computeStatParams(m_selectedEntities, this);
}

void MainWindow::doActionStatisticalTest()
{
	if (!ccEntityAction::statisticalTest(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

//////////////////////////////////////////////////////////////////////////

//"Tools > Segmentation" menu

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
	std::vector<ComponentIndexAndSize>* _sortedIndexes = nullptr;
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
				sortedIndexes.emplace_back(i, components[i]->size());
			}

			ParallelSort(sortedIndexes.begin(), sortedIndexes.end(), ComponentIndexAndSize::DescendingCompOperator);

			_sortedIndexes = &sortedIndexes;
		}
	}

	//we create "real" point clouds for all input components
	{
		ccPointCloud* pc = cloud->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(cloud) : 0;

		//we create a new group to store all CCs
		ccHObject* ccGroup = nullptr;
		DataBaseHObject* db_prj = GetRootDataBase(cloud);
		if (db_prj) {
			ccHObject* product_pool = db_prj->getProductSegmented(); 
			if (product_pool) {
				ccGroup = getChildGroupByName(product_pool, cloud->getName(), true);
			}
		}
		if (!ccGroup) {
			ccGroup = new ccHObject(cloud->getName() + QString(" [segments]"));
		}

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
					
					QString name;
					int bd_num = GetMaxNumberExcludeChildPrefix(ccGroup, BDDB_BUILDING_PREFIX) + 1;
					compCloud->setName(BuildingNameByNumber(bd_num)/*QString("CC#%1").arg(ccGroup->getChildrenNumber())*/);

					//we add new CC to group
					ccGroup->addChild(compCloud);

					if (selectComponents)
						db(compCloud)->selectEntity(compCloud, true);
				}
				else
				{
					ccConsole::Warning("[createComponentsClouds] Failed to create component #%i! (not enough memory)", ccGroup->getChildrenNumber() + 1);
				}
			}

			delete compIndexes;
			compIndexes = nullptr;
		}

		components.clear();

		if (ccGroup->getChildrenNumber() == 0)
		{
			ccConsole::Error("No component was created! Check the minimum size...");
			delete ccGroup;
			ccGroup = nullptr;
		}
		else
		{
			ccGroup->setDisplay(cloud->getDisplay());
			addToDB(ccGroup, cloud->getDBSourceType());

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
	//keep only the point clouds!
	std::vector<ccGenericPointCloud*> clouds;
	{
		for (ccHObject *entity : getSelectedEntities())
		{
			if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
				clouds.push_back(ccHObjectCaster::ToGenericPointCloud(entity));
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
	unselectAllInDB();

	for (ccGenericPointCloud *cloud : clouds)
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

void MainWindow::doActionKMeans()//TODO
{
	ccConsole::Error("Not yet implemented! Sorry ...");
}

void MainWindow::doActionFrontPropagation() //TODO
{
	ccConsole::Error("Not yet implemented! Sorry ...");
}

//////////////////////////////////////////////////////////////////////////

//"Tools > Fit" menu

void MainWindow::doActionFitSphere()
{
	double outliersRatio = 0.5;
	double confidence = 0.99;

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	for (ccHObject *entity : getSelectedEntities())
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (!cloud)
			continue;

		CCVector3 center;
		PointCoordinateType radius;
		double rms;
		if (CCLib::GeometricalAnalysisTools::DetectSphereRobust(cloud,
			outliersRatio,
			center,
			radius,
			rms,
			&pDlg,
			confidence) != CCLib::GeometricalAnalysisTools::NoError)
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
		ccSphere* sphere = new ccSphere(radius, &trans, QString("Sphere r=%1 [rms %2]").arg(radius).arg(rms));
		cloud->addChild(sphere);
		//sphere->setDisplay(cloud->getDisplay());
		sphere->prepareDisplayForRefresh();
		addToDB(sphere, entity->getDBSourceType(), false, false, false);
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
		maxEdgeLength = QInputDialog::getDouble(this, "Fit facet", "Max edge length (0 = no limit)", s_polygonMaxEdgeLength, 0, 1.0e9, 8, &ok);
		if (!ok)
			return;
		s_polygonMaxEdgeLength = maxEdgeLength;
	}

	ccHObject::Container selectedEntities = getSelectedEntities(); //warning, getSelectedEntites may change during this loop!
	bool firstEntity = true;

	for (ccHObject *entity : selectedEntities)
	{
		ccShiftedObject* shifted = nullptr;
		CCLib::GenericIndexedCloudPersist* cloud = nullptr;

		if (entity->isKindOf(CC_TYPES::POLY_LINE))
		{
			ccPolyline* poly = ccHObjectCaster::ToPolyline(entity);
			cloud = static_cast<CCLib::GenericIndexedCloudPersist*>(poly);
			shifted = poly;
		}
		else
		{
			ccGenericPointCloud* gencloud = ccHObjectCaster::ToGenericPointCloud(entity);
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
							contour->setGlobalScale(shifted->getGlobalScale());
							contour->setGlobalShift(shifted->getGlobalShift());
						}
					}
				}
			}
			else
			{
				std::vector<CCVector3> c_hull;
				ccPlane* pPlane = ccPlane::Fit(cloud, &rms, &c_hull);
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
				ccConsole::Print(QString("[Orientation] Entity '%1'").arg(entity->getName()));
				ccConsole::Print("\t- plane fitting RMS: %f", rms);

				//We always consider the normal with a positive 'Z' by default!
				if (N.z < 0.0)
					N *= -1.0;
				ccConsole::Print("\t- normal: (%f,%f,%f)", N.x, N.y, N.z);

				//we compute strike & dip by the way
				PointCoordinateType dip = 0.0f;
				PointCoordinateType dipDir = 0.0f;
				ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);
				QString dipAndDipDirStr = ccNormalVectors::ConvertDipAndDipDirToString(dip, dipDir);
				ccConsole::Print(QString("\t- %1").arg(dipAndDipDirStr));

				//hack: output the transformation matrix that would make this normal points towards +Z
				ccGLMatrix makeZPosMatrix = ccGLMatrix::FromToRotation(N, CCVector3(0, 0, PC_ONE));
				CCVector3 Gt = C;
				makeZPosMatrix.applyRotation(Gt);
				makeZPosMatrix.setTranslation(C - Gt);
				ccConsole::Print("[Orientation] A matrix that would make this plane horizontal (normal towards Z+) is:");
				ccConsole::Print(makeZPosMatrix.toString(12, ' ')); //full precision
				ccConsole::Print("[Orientation] You can copy this matrix values (CTRL+C) and paste them in the 'Apply transformation tool' dialog");

				plane->setName(dipAndDipDirStr);
				plane->applyGLTransformation_recursive(); //not yet in DB
				plane->setVisible(true);
				plane->setSelectionBehavior(ccHObject::SELECTION_FIT_BBOX);

				entity->addChild(plane);
				plane->setDisplay(entity->getDisplay());
				plane->prepareDisplayForRefresh_recursive();
				addToDB(plane, entity->getDBSourceType());

				if (firstEntity)
				{
					unselectAllInDB();
					setSelectedInDB(plane, true);
				}
			}
			else
			{
				ccConsole::Warning(QString("Failed to fit a plane/facet on entity '%1'").arg(entity->getName()));
			}
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionFitQuadric()
{
	bool errors = false;

	//for all selected entities
	for (ccHObject *entity : getSelectedEntities())
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
				addToDB(quadric, cloud->getDBSourceType());

				ccConsole::Print(QString("[doActionFitQuadric] Quadric local coordinate system:"));
				ccConsole::Print(quadric->getTransformation().toString(12, ' ')); //full precision
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
						for (unsigned i = 0; i < newCloud->size(); ++i)
						{
							CCVector3* P = const_cast<CCVector3*>(newCloud->getPoint(i));
							CCVector3 Q = invTrans * (*P);
							Q.u[dZ] = eq[0] + eq[1] * Q.u[dX] + eq[2] * Q.u[dY] + eq[3] * Q.u[dX] * Q.u[dX] + eq[4] * Q.u[dX] * Q.u[dY] + eq[5] * Q.u[dY] * Q.u[dY];
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

//////////////////////////////////////////////////////////////////////////

//"Tools > Batch export" menu

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
		ccConsole::Error("Select at least one point cloud!");
		return;
	}

	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::SaveFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	QString outputFilename = QFileDialog::getSaveFileName(this,
		"Select output file",
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
		ccConsole::Error("Failed to open file for writing! (check file permissions)");
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

			CCVector3 G = *CCLib::Neighbourhood(cloud).getGravityCenter();
			csvStream << cloud->getName() << ";" /*"Name;"*/;
			csvStream << cloud->size() << ";" /*"Points;"*/;
			csvStream << G.x << ";" /*"meanX;"*/;
			csvStream << G.y << ";" /*"meanY;"*/;
			csvStream << G.z << ";" /*"meanZ;"*/;
			for (unsigned j = 0; j < cloud->getNumberOfScalarFields(); ++j)
			{
				CCLib::ScalarField* sf = cloud->getScalarField(j);
				csvStream << sf->getName() << ";" /*"SF name;"*/;

				unsigned validCount = 0;
				double sfSum = 0.0;
				double sfSum2 = 0.0;
				for (unsigned k = 0; k < sf->currentSize(); ++k)
				{
					const ScalarType& val = sf->getValue(k);
					if (CCLib::ScalarField::ValidValue(val))
					{
						++validCount;
						sfSum += val;
						sfSum2 += val * val;
					}
				}
				csvStream << validCount << ";" /*"SF valid values;"*/;
				double mean = sfSum / validCount;
				csvStream << mean << ";" /*"SF mean;"*/;
				csvStream << sqrt(std::abs(sfSum2 / validCount - mean * mean)) << ";" /*"SF std.dev.;"*/;
				csvStream << sfSum << ";" /*"SF sum;"*/;
			}
			csvStream << endl;
		}
	}

	ccConsole::Print(QString("[I/O] File '%1' successfully saved (%2 cloud(s))").arg(outputFilename).arg(clouds.size()));
	csvFile.close();
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
		ccLog::Error("No plane in selection");
		return;
	}

	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::SaveFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	QString outputFilename = QFileDialog::getSaveFileName(this,
		"Select output file",
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
		ccConsole::Error("Failed to open file for writing! (check file permissions)");
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
		CCVector3 N = plane->getNormal();
		PointCoordinateType dip_deg = 0, dipDir_deg = 0;
		ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip_deg, dipDir_deg);

		csvStream << plane->getName() << separator;		//Name
		csvStream << plane->getXWidth() << separator;	//Width
		csvStream << plane->getYWidth() << separator;	//Height
		csvStream << C.x << separator;					//Cx
		csvStream << C.y << separator;					//Cy
		csvStream << C.z << separator;					//Cz
		csvStream << N.x << separator;					//Nx
		csvStream << N.y << separator;					//Ny
		csvStream << N.z << separator;					//Nz
		csvStream << dip_deg << separator;				//Dip
		csvStream << dipDir_deg << separator;			//Dip direction
		csvStream << endl;
	}

	ccConsole::Print(QString("[I/O] File '%1' successfully saved (%2 plane(s))").arg(outputFilename).arg(planes.size()));
	csvFile.close();
}

//////////////////////////////////////////////////////////////////////////

//"Tools > Other" menu

void MainWindow::doComputeGeometricFeature()
{
	static ccLibAlgorithms::GeomCharacteristicSet s_selectedCharacteristics;

	ccGeomFeaturesDlg gfDlg(this);
	double radius = ccLibAlgorithms::GetDefaultCloudKernelSize(m_selectedEntities);
	gfDlg.setRadius(radius);
	gfDlg.setSelectedFeatures(s_selectedCharacteristics);

	if (!gfDlg.exec())
		return;

	radius = gfDlg.getRadius();
	if (!gfDlg.getSelectedFeatures(s_selectedCharacteristics))
	{
		ccLog::Error("Not enough memory");
		return;
	}

	ccLibAlgorithms::ComputeGeomCharacteristics(s_selectedCharacteristics, static_cast<PointCoordinateType>(radius), m_selectedEntities, this);

	refreshAll();
	updateUI();
}

void MainWindow::doRemoveDuplicatePoints()
{
	if (!haveSelection())
		return;

	bool first = true;

	//persistent setting(s)
	QSettings settings;
	settings.beginGroup(ccPS::DuplicatePointsGroup());
	double minDistanceBetweenPoints = settings.value(ccPS::DuplicatePointsMinDist(), 1.0e-12).toDouble();

	bool ok;
	minDistanceBetweenPoints = QInputDialog::getDouble(this, "Remove duplicate points", "Min distance between points:", minDistanceBetweenPoints, 0, 1.0e8, 12, &ok);
	if (!ok)
		return;

	//save parameter
	settings.setValue(ccPS::DuplicatePointsMinDist(), minDistanceBetweenPoints);

	static const char DEFAULT_DUPLICATE_TEMP_SF_NAME[] = "DuplicateFlags";

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	ccHObject::Container selectedEntities = getSelectedEntities(); //we have to use a local copy: 'unselectAllEntities' and 'selectEntity' will change the set of currently selected entities!

	for (ccHObject *entity : selectedEntities)
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
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

			CCLib::GeometricalAnalysisTools::ErrorCode result = CCLib::GeometricalAnalysisTools::FlagDuplicatePoints(cloud,
				minDistanceBetweenPoints,
				&pDlg,
				octree.data());

			if (result == CCLib::GeometricalAnalysisTools::NoError)
			{
				//count the number of duplicate points!
				CCLib::ScalarField* flagSF = cloud->getScalarField(sfIdx);
				unsigned duplicateCount = 0;
				assert(flagSF);
				if (flagSF)
				{
					for (unsigned j = 0; j < flagSF->currentSize(); ++j)
					{
						if (flagSF->getValue(j) != 0)
						{
							++duplicateCount;
						}
					}
				}

				if (duplicateCount == 0)
				{
					ccConsole::Print(QString("Cloud '%1' has no duplicate points").arg(cloud->getName()));
				}
				else
				{
					ccConsole::Warning(QString("Cloud '%1' has %2 duplicate point(s)").arg(cloud->getName()).arg(duplicateCount));

					ccPointCloud* filteredCloud = cloud->filterPointsByScalarValue(0, 0);
					if (filteredCloud)
					{
						int sfIdx2 = filteredCloud->getScalarFieldIndexByName(DEFAULT_DUPLICATE_TEMP_SF_NAME);
						assert(sfIdx2 >= 0);
						filteredCloud->deleteScalarField(sfIdx2);
						filteredCloud->setName(QString("%1.clean").arg(cloud->getName()));
						filteredCloud->setDisplay(cloud->getDisplay());
						filteredCloud->prepareDisplayForRefresh();
						addToDB(filteredCloud, cloud->getDBSourceType());
						if (first)
						{
							db(filteredCloud)->unselectAllEntities();
							first = false;
						}
						cloud->setEnabled(false);
						db(filteredCloud)->selectEntity(filteredCloud, true);
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

//////////////////////////////////////////////////////////////////////////

//"Tools"

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

	if (!haveOneSelection())
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

	enablePickingOperation(win, "Pick three points on the floor plane (click the Level button or press Escape to cancel)");
}

//"Tools > Sand box (research)" menu

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

	for (ccHObject *entity : getSelectedEntities())
	{
		if (!entity->isKindOf(CC_TYPES::MESH) && !entity->isKindOf(CC_TYPES::POINT_CLOUD))
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

		ccBBox box = entity->getOwnBB();
		PointCoordinateType largestDim = box.getMaxBoxDim() + static_cast<PointCoordinateType>(margin);
		PointCoordinateType cellDim = largestDim / steps;
		CCVector3 minCorner = box.getCenter() - CCVector3(1, 1, 1) * (largestDim / 2);

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
			ccLog::Error("Not enough memory!");
			return;
		}

		//cdt.propagateDistance(CHAMFER_345, &pDlg);
		cdt.propagateDistance(&pDlg);

		//convert the grid to a cloud
		ccPointCloud* gridCloud = new ccPointCloud(entity->getName() + QString(".distance_grid(%1)").arg(steps));
		{
			unsigned pointCount = steps * steps*steps;
			if (!gridCloud->reserve(pointCount))
			{
				ccLog::Error("Not enough memory!");
				delete gridCloud;
				return;
			}

			ccScalarField* sf = new ccScalarField("DT values");
			if (!sf->reserveSafe(pointCount))
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
				ccLog::Warning(QString("[DistanceMap] Cloud '%1': no point falls inside the specified range").arg(entity->getName()));
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
				addToDB(gridCloud, entity->getDBSourceType());
			}
		}
	}

	refreshAll();
}

void MainWindow::doActionComputeDistToBestFitQuadric3D()
{
	bool ok = true;
	int steps = QInputDialog::getInt(this, "Distance to best fit quadric (3D)", "Steps (per dim.)", 50, 10, 10000, 10, &ok);
	if (!ok)
		return;

	for (ccHObject *entity : getSelectedEntities())
	{
		if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);
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
				for (int x = 0; x < steps; ++x)
				{
					CCVector3 P;
					P.x = C.x + maxDim * (static_cast<PointCoordinateType>(x) / static_cast<PointCoordinateType>(steps - 1) - PC_ONE / 2);
					for (int y = 0; y < steps; ++y)
					{
						P.y = C.y + maxDim * (static_cast<PointCoordinateType>(y) / static_cast<PointCoordinateType>(steps - 1) - PC_ONE / 2);
						for (int z = 0; z < steps; ++z)
						{
							P.z = C.z + maxDim * (static_cast<PointCoordinateType>(z) / static_cast<PointCoordinateType>(steps - 1) - PC_ONE / 2);
							newCloud->addPoint(P);

							//compute distance to quadric
							CCVector3 Pc = P - *G;
							ScalarType dist = static_cast<ScalarType>(a*Pc.x*Pc.x + b * Pc.y*Pc.y + c * Pc.z*Pc.z
								+ e * Pc.x*Pc.y + f * Pc.y*Pc.z + g * Pc.x*Pc.z
								+ l * Pc.x + m * Pc.y + n * Pc.z + d);

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

				addToDB(newCloud, cloud->getDBSourceType());
			}
			else
			{
				ccConsole::Warning(QString("Failed to compute 3D quadric on cloud '%1'").arg(cloud->getName()));
			}
		}
	}

	refreshAll();
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

	//backup selected entities as removeObjectTemporarilyFromDBTree can modify them
	ccHObject::Container selectedEntities = getSelectedEntities();

	for (ccHObject *entity : selectedEntities) //warning, getSelectedEntites may change during this loop!
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);

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
					for (unsigned j = 0; j < 3; ++j)
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

					entity->prepareDisplayForRefresh_recursive();
				}
			}
		}
	}

	refreshAll();
}

//Aurelien BEY le 13/11/2008 : ajout de la fonction permettant de traiter la fonctionnalite de recalage grossier
void MainWindow::doAction4pcsRegister()
{
	if (QMessageBox::warning(this,
		"Work in progress",
		"This method is still under development: are you sure you want to use it? (a crash may likely happen)",
		QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
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
	if (CCLib::FPCSRegistrationTools::RegisterClouds(subModel,
		subData,
		transform,
		static_cast<ScalarType>(aDlg.getDelta()),
		static_cast<ScalarType>(aDlg.getDelta() / 2),
		static_cast<PointCoordinateType>(aDlg.getOverlap()),
		aDlg.getNbTries(),
		5000,
		&pDlg,
		nbMaxCandidates))
	{
		//output resulting transformation matrix
		{
			ccGLMatrix transMat = FromCCLibMatrix<PointCoordinateType, float>(transform.R, transform.T);
			forceConsoleDisplay();
			ccConsole::Print("[Align] Resulting matrix:");
			ccConsole::Print(transMat.toString(12, ' ')); //full precision
			ccConsole::Print("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool");
		}

		ccPointCloud *newDataCloud = data->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(data)->cloneThis() : ccPointCloud::From(data, data);

		if (data->getParent())
			data->getParent()->addChild(newDataCloud);
		newDataCloud->setName(data->getName() + QString(".registered"));
		transform.apply(*newDataCloud);
		newDataCloud->invalidateBoundingBox(); //invalidate bb
		newDataCloud->setDisplay(data->getDisplay());
		newDataCloud->prepareDisplayForRefresh();
		zoomOn(newDataCloud);
		addToDB(newDataCloud, data->getDBSourceType());

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

void MainWindow::doSphericalNeighbourhoodExtractionTest()
{
	size_t selNum = m_selectedEntities.size();
	if (selNum < 1)
		return;

	//spherical neighborhood extraction radius
	PointCoordinateType sphereRadius = ccLibAlgorithms::GetDefaultCloudKernelSize(m_selectedEntities);
	if (sphereRadius < 0)
	{
		ccConsole::Error("Invalid kernel size!");
		return;
	}

	bool ok;
	double val = QInputDialog::getDouble(this, "SNE test", "Radius:", static_cast<double>(sphereRadius), DBL_MIN, 1.0e9, 8, &ok);
	if (!ok)
		return;
	sphereRadius = static_cast<PointCoordinateType>(val);

	QString sfName = QString("Spherical extraction test") + QString(" (%1)").arg(sphereRadius);

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

		int sfIdx = cloud->getScalarFieldIndexByName(qPrintable(sfName));
		if (sfIdx < 0)
			sfIdx = cloud->addScalarField(qPrintable(sfName));
		if (sfIdx < 0)
		{
			ccConsole::Error(QString("Failed to create scalar field on cloud '%1' (not enough memory?)").arg(cloud->getName()));
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
				ccConsole::Error(QString("Couldn't compute octree for cloud '%1'!").arg(cloud->getName()));
				return;
			}
		}

		CCLib::ScalarField* sf = cloud->getScalarField(sfIdx);
		sf->fill(NAN_VALUE);
		cloud->setCurrentScalarField(sfIdx);

		QElapsedTimer eTimer;
		eTimer.start();

		size_t extractedPoints = 0;
		unsigned char level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(sphereRadius);
		std::random_device rd;   // non-deterministic generator
		std::mt19937 gen(rd());  // to seed mersenne twister.
		std::uniform_int_distribution<unsigned> dist(0, cloud->size() - 1);

		const unsigned samples = 1000;
		for (unsigned j = 0; j < samples; ++j)
		{
			unsigned randIndex = dist(gen);
			CCLib::DgmOctree::NeighboursSet neighbours;
			octree->getPointsInSphericalNeighbourhood(*cloud->getPoint(randIndex), sphereRadius, neighbours, level);
			size_t neihgboursCount = neighbours.size();
			extractedPoints += neihgboursCount;
			for (size_t k = 0; k < neihgboursCount; ++k)
				cloud->setPointScalarValue(neighbours[k].pointIndex, static_cast<ScalarType>(sqrt(neighbours[k].squareDistd)));
		}
		ccConsole::Print("[SNE_TEST] Mean extraction time = %i ms (radius = %f, mean(neighbours) = %3.1f)", eTimer.elapsed(), sphereRadius, extractedPoints / static_cast<double>(samples));

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
	double radius = QInputDialog::getDouble(this, "CNE Test", "radius", 0.02, 1.0e-6, 1.0e6, 6, &ok);
	if (!ok)
		return;

	double height = QInputDialog::getDouble(this, "CNE Test", "height", 0.05, 1.0e-6, 1.0e6, 6, &ok);
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

			CCLib::DgmOctree::CylindricalNeighbourhood cn;
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

	addToDB(cloud, getCurrentDB());

	refreshAll();
	updateUI();
}

static int s_innerRectDim = 2;
void MainWindow::doActionFindBiggestInnerRectangle()
{
	if (!haveSelection())
		return;

	ccHObject* entity = haveOneSelection() ? m_selectedEntities[0] : nullptr;
	if (!entity || !entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error("Select one point cloud!");
		return;
	}

	bool ok;
	int dim = QInputDialog::getInt(this, "Dimension", "Orthogonal dim (X=0 / Y=1 / Z=2)", s_innerRectDim, 0, 2, 1, &ok);
	if (!ok)
		return;
	s_innerRectDim = dim;

	ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(entity);
	ccBox* box = ccInnerRect2DFinder().process(cloud, static_cast<unsigned char>(dim));

	if (box)
	{
		cloud->addChild(box);
		box->setVisible(true);
		box->setDisplay(cloud->getDisplay());
		box->setDisplay(cloud->getDisplay());
		addToDB(box, cloud->getDBSourceType());
	}

	updateUI();
}

void MainWindow::doActionCreateCloudFromEntCenters()
{
	size_t selNum = getSelectedEntities().size();

	ccPointCloud* centers = new ccPointCloud("centers");
	if (!centers->reserve(static_cast<unsigned>(selNum)))
	{
		ccLog::Error("Not enough memory!");
		delete centers;
		centers = nullptr;
		return;
	}

	//look for clouds
	{
		for (ccHObject *entity : getSelectedEntities())
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
		ccLog::Error("No cloud in selection?!");
		delete centers;
		centers = nullptr;
	}
	else
	{
		centers->resize(centers->size());
		centers->setPointSize(10);
		centers->setVisible(true);
		addToDB(centers, getSelectedEntities().front()->getDBSourceType());
	}
}

void MainWindow::doActionComputeBestICPRmsMatrix()
{
	//look for clouds
	std::vector<ccPointCloud*> clouds;
	try
	{
		for (ccHObject *entity : getSelectedEntities())
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
		rmsMatrix.resize(cloudCount*cloudCount, 0);

		//init all possible transformations
		static const double angularStep_deg = 45.0;
		unsigned phiSteps = static_cast<unsigned>(360.0 / angularStep_deg);
		assert(std::abs(360.0 - phiSteps * angularStep_deg) < ZERO_TOLERANCE);
		unsigned thetaSteps = static_cast<unsigned>(180.0 / angularStep_deg);
		assert(std::abs(180.0 - thetaSteps * angularStep_deg) < ZERO_TOLERANCE);
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
				trans.initFromParameters(static_cast<float>(phi_deg * CC_DEG_TO_RAD),
					static_cast<float>(theta_deg * CC_DEG_TO_RAD),
					0,
					CCVector3(0, 0, 0));
				matrices.push_back(trans);
				matrixAngles.push_back(std::pair<double, double>(phi_deg, theta_deg));

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
						ccLog::Error("Not enough memory!");
						return;
					}

					ccGLMatrix BtoA = transFromZeroToA * matrices[k] * transBToZero;
					B->applyRigidTransformation(BtoA);

#ifndef TEST_GENERATION
					double finalRMS = 0.0;
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
					CCVector3 Y(0, 1, 0);
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
					ccHObject* group = new ccHObject(QString("Best case #%1 / #%2 - RMS = %3").arg(i + 1).arg(j + 1).arg(minRMS));
					group->addChild(bestB);
					group->setDisplay_recursive(A->getDisplay());
					addToDB(group, A->getDBSourceType());
					ccLog::Print(QString("[doActionComputeBestICPRmsMatrix] Comparison #%1 / #%2: min RMS = %3 (phi = %4 / theta = %5 deg.)").arg(i + 1).arg(j + 1).arg(minRMS).arg(matrixAngles[bestMatrixIndex].first).arg(matrixAngles[bestMatrixIndex].second));
				}
				else
				{
					assert(!bestB);
					ccLog::Warning(QString("[doActionComputeBestICPRmsMatrix] Comparison #%1 / #%2: INVALID").arg(i + 1).arg(j + 1));
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

		QString outputFilename = QFileDialog::getSaveFileName(this,
			"Select output file",
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
				for (ccPointCloud *cloud : clouds)
				{
					stream << ";";
					stream << cloud->getName();
				}
				stream << endl;
			}

			//rows
			for (size_t j = 0; j < cloudCount; ++j)
			{
				stream << clouds[j]->getName();
				stream << ";";
				for (size_t i = 0; i < cloudCount; ++i)
				{
					stream << rmsMatrix[j*cloudCount + i];
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

//////////////////////////////////////////////////////////////////////////

//"Display" menu

void MainWindow::toggleLockRotationAxis()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		bool wasLocked = win->isRotationAxisLocked();
		bool isLocked = !wasLocked;

		static CCVector3d s_lastAxis(0.0, 0.0, 1.0);
		if (isLocked)
		{
			ccAskThreeDoubleValuesDlg axisDlg("x", "y", "z", -1.0e12, 1.0e12, s_lastAxis.x, s_lastAxis.y, s_lastAxis.z, 4, "Lock rotation axis", this);
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
		ccHObject::Container selectedEntities = getSelectedEntities();

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
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->setBubbleViewMode(true);
		win->redraw(false);
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
			m_cpeDlg, static_cast<void (ccCameraParamEditDlg::*)(QMdiSubWindow *)>(&ccCameraParamEditDlg::linkWith));

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

	ccAdjustZoomDlg azDlg(win, this);

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

	addToDB_Main(viewportObject);
}

//////////////////////////////////////////////////////////////////////////

//"Display > Lights & Materials" menu

void MainWindow::showDisplayOptions()
{
	ccDisplayOptionsDlg displayOptionsDlg(this);
	connect(&displayOptionsDlg, &ccDisplayOptionsDlg::aspectHasChanged, this, [=]() { redrawAll();	});

	displayOptionsDlg.exec();

	disconnect(&displayOptionsDlg);
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

//////////////////////////////////////////////////////////////////////////

//"Display > Shaders & filters" menu

void MainWindow::doActionLoadShader() //TODO
{
	ccConsole::Error("Not yet implemented! Sorry ...");
}

void MainWindow::doActionDeleteShader()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->setShader(nullptr);
	}
}

//////////////////////////////////////////////////////////////////////////

//"Display > Active SF" menu

void MainWindow::doApplyActiveSFAction(int action)
{
	if (!haveOneSelection())
	{
		if (haveSelection())
		{
			ccConsole::Error("Select only one cloud or one mesh!");
		}
		return;
	}
	ccHObject* ent = m_selectedEntities[0];

	bool lockedVertices;
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent, &lockedVertices);

	//for "real" point clouds only
	if (!cloud)
		return;
	if (lockedVertices && !ent->isAncestorOf(cloud))
	{
		//see ccPropertiesTreeDelegate::fillWithMesh
		ccUtils::DisplayLockedVerticesWarning(ent->getName(), true);
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
			cloud->setCurrentDisplayedScalarField(sfIdx - 1);
			cloud->prepareDisplayForRefresh();
		}
		break;
	case 2: //Activate next SF
		if (sfIdx + 1 < static_cast<int>(cloud->getNumberOfScalarFields()))
		{
			cloud->setCurrentDisplayedScalarField(sfIdx + 1);
			cloud->prepareDisplayForRefresh();
		}
		break;
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

//////////////////////////////////////////////////////////////////////////

//"Display" menu

void MainWindow::doActionResetGUIElementsPos()
{
	// show the user it will be maximized
	showMaximized();

	QSettings settings;
	settings.remove(ccPS::MainWinGeom());
	settings.remove(ccPS::MainWinState());

	QMessageBox::information(this,
		tr("Restart"),
		tr("To finish the process, you'll have to close and restart BlockBuilder"));

	//to avoid saving them right away!
	s_autoSaveGuiElementPos = false;
}

void MainWindow::doActionToggleDrawBBox() {
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->toggleDrawBBox();
		win->redraw(false);
	}
}

void MainWindow::doActionDisplayGlobalCoord()
{
	if (m_UI->actionDisplayGlobalCoord->isChecked()) {
		// TODO
	}
	else {

	}
}

//////////////////////////////////////////////////////////////////////////

//"3D Views" menu

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

ccGLWindow* MainWindow::new3DView(bool allowEntitySelection)
{
	assert(m_ccRoot && m_imageRoot && m_mdiArea);

	QWidget* viewWidget = nullptr;
	ccGLWindow* view3D = nullptr;

	createGLWindow(view3D, viewWidget);
	if (!viewWidget || !view3D)
	{
		ccLog::Error("Failed to create the 3D view");
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

	if (allowEntitySelection)
	{
		connect(view3D, &ccGLWindow::entitySelectionChanged, this, [=](ccHObject *entity) {
			setSelectedInDB(entity, true);
// 			m_ccRoot->selectEntity(entity);
// 			m_buildingRoot->selectEntity(entity);
// 			m_imageRoot->selectEntity(entity);
		});

		connect(view3D, &ccGLWindow::entitiesSelectionChanged, this, [=](std::unordered_set<int> entities) {
			m_ccRoot->selectEntities(entities);
			m_buildingRoot->selectEntities(entities);
			m_imageRoot->selectEntities(entities);
		});
	}

	//'echo' mode
	connect(view3D, &ccGLWindow::mouseWheelRotated, this, &MainWindow::echoMouseWheelRotate);
	connect(view3D, &ccGLWindow::cameraDisplaced, this, &MainWindow::echoCameraDisplaced);
	connect(view3D, &ccGLWindow::viewMatRotated, this, &MainWindow::echoBaseViewMatRotation);
	connect(view3D, &ccGLWindow::cameraPosChanged, this, &MainWindow::echoCameraPosChanged);
	connect(view3D, &ccGLWindow::pivotPointChanged, this, &MainWindow::echoPivotPointChanged);
	connect(view3D, &ccGLWindow::pixelSizeChanged, this, &MainWindow::echoPixelSizeChanged);
	connect(view3D, &ccGLWindow::mouseMoved2D, this, &MainWindow::echoMouseMoved2D);
	connect(view3D, &ccGLWindow::mouseMoved3D, this, &MainWindow::echoMouseMoved3D);
	connect(view3D, &ccGLWindow::pointSnapBufferChanged, this, &MainWindow::echopointSnapBufferChanged);

	connect(view3D, &QObject::destroyed, this, &MainWindow::prepareWindowDeletion);
	connect(view3D, &ccGLWindow::filesDropped, this, &MainWindow::addToDBAuto, Qt::QueuedConnection); //DGM: we don't want to block the 'dropEvent' method of ccGLWindow instances!
	connect(view3D, &ccGLWindow::newLabel, this, &MainWindow::handleNewLabel);
	connect(view3D, &ccGLWindow::exclusiveFullScreenToggled, this, &MainWindow::onExclusiveFullScreenToggled);

	if (m_pickingHub)
	{
		//we must notify the picking hub as well if the window is destroyed
		connect(view3D, &QObject::destroyed, m_pickingHub, &ccPickingHub::onActiveWindowDeleted);
	}
	view3D->showCursorCoordinates(true);
	view3D->addSceneDB(m_ccRoot->getRootEntity());
	view3D->addSceneDB(m_buildingRoot->getRootEntity());
	view3D->addSceneDB(m_imageRoot->getRootEntity());
	viewWidget->setAttribute(Qt::WA_DeleteOnClose);
	viewWidget->setWindowFlags(viewWidget->windowFlags()&~Qt::WindowCloseButtonHint);
	viewWidget->setWindowFlags(viewWidget->windowFlags()&~Qt::WindowMinimizeButtonHint);
	viewWidget->setWindowFlags(viewWidget->windowFlags()&~Qt::WindowMaximizeButtonHint);
	updatePropertiesView();

	QMainWindow::statusBar()->showMessage(QString("New 3D View"), 2000);

	viewWidget->showMaximized();
	viewWidget->update();
	setCenteredPerspectiveView(view3D);

	return view3D;
}

void MainWindow::onExclusiveFullScreenToggled(bool state)
{
	//we simply update the fullscreen action method icon (whatever the window)
	ccGLWindow* win = getActiveGLWindow();

	if (win == nullptr)
		return;

	m_UI->actionExclusiveFullScreen->blockSignals(true);
	m_UI->actionExclusiveFullScreen->setChecked(win ? win->exclusiveFullScreen() : false);
	m_UI->actionExclusiveFullScreen->blockSignals(false);

	if (	!state
		&&	win->stereoModeIsEnabled()
		&&	(	win->getStereoParams().glassType == ccGLWindow::StereoParams::NVIDIA_VISION
			||	win->getStereoParams().glassType == ccGLWindow::StereoParams::GENERIC_STEREO_DISPLAY ))
	{
		//auto disable stereo mode as NVidia Vision only works in full screen mode!
		m_UI->actionEnableStereo->setChecked(false);
	}
}

//////////////////////////////////////////////////////////////////////////

//"About" menu entry

void MainWindow::doActionShowHelpDialog()
{
	QMessageBox::information(this,
		tr("Documentation"),
		tr("Please visit http://www.cloudcompare.org/doc"));
}

//////////////////////////////////////////////////////////////////////////

/*** Toolbars ***/

//View toolbar

void MainWindow::zoomOn(ccHObject* object)
{
	ccGLWindow* win = static_cast<ccGLWindow*>(object->getDisplay());
	if (win)
	{
		ccBBox box = object->getDisplayBB_recursive(false,win);
		win->updateConstellationCenterAndZoom(&box);
	}
}

void MainWindow::zoomOnSelectedEntities()
{
	ccGLWindow* win = nullptr;

	ccHObject tempGroup("TempGroup");
	size_t selNum = m_selectedEntities.size();
	for (size_t i = 0; i < selNum; ++i)
	{
		ccHObject *entity = m_selectedEntities[i];

		if (i == 0 || !win)
		{
			//take the first valid window as reference
			win = static_cast<ccGLWindow*>(entity->getDisplay());
		}

		if (win)
		{
			if (entity->getDisplay() == win)
			{
				tempGroup.addChild(entity, ccHObject::DP_NONE);
			}
			else if (entity->getDisplay() != nullptr)
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
			if (win != nullptr)
			{
				win->updateConstellationCenterAndZoom(&box);
			}
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
	enablePickingOperation(win, "Pick a point to be used as rotation center (click on icon again to cancel)");
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
			m_pivotVisibilityPopupButton->setIcon(m_UI->actionSetPivotAlwaysOn->icon());
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
			m_pivotVisibilityPopupButton->setIcon(m_UI->actionSetPivotRotationOnly->icon());
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
			m_pivotVisibilityPopupButton->setIcon(m_UI->actionSetPivotOff->icon());
	}
}

void MainWindow::setView(CC_VIEW_ORIENTATION view)
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->setView(view);
	}
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
			m_viewModePopupButton->setIcon(m_UI->actionSetOrthoView->icon());
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setEnabled(true);
	}
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
			m_viewModePopupButton->setIcon(m_UI->actionSetCenteredPerspectiveView->icon());
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setEnabled(true);
	}
}

void MainWindow::setViewerPerspectiveView(ccGLWindow* win)
{
	if (win)
	{
		win->setPerspectiveState(true, false);
		win->redraw();

		//update pop-up menu 'top' icon
		if (m_viewModePopupButton)
			m_viewModePopupButton->setIcon(m_UI->actionSetViewerPerspectiveView->icon());
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setEnabled(false);
	}
}

//////////////////////////////////////////////////////////////////////////

//hidden

void MainWindow::toggleVisualDebugTraces()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->toggleDebugTrace();
		win->redraw(false, false);
	}
}

//////////////////////////////////////////////////////////////////////////

// viewer edit mode

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
	if (m_compDlg && result == QDialog::Accepted)
	{
		ccHObject* compEntity = m_compDlg->getComparedEntity();
		if (compEntity)
		{
			setSelectedInDB(compEntity, true);
		}
	}

	freezeUI(false);

	updateUI();
}

void MainWindow::activateRegisterPointPairTool()
{
	if (!haveSelection() || m_selectedEntities.size() > 2)
	{
		ccConsole::Error("Select one or two entities (point cloud or mesh)!");
		return;
	}

	ccHObject* aligned = m_selectedEntities[0];
	ccHObject* reference = m_selectedEntities.size() > 1 ? m_selectedEntities[1] : nullptr;

	ccGenericPointCloud* cloud1 = ccHObjectCaster::ToGenericPointCloud(aligned);
	ccGenericPointCloud* cloud2 = (reference ? ccHObjectCaster::ToGenericPointCloud(reference) : nullptr);
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
		m_pprDlg = new ccPointPairRegistrationDlg(m_pickingHub, this, this);
		connect(m_pprDlg, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateRegisterPointPairTool);
		registerOverlayDialog(m_pprDlg, Qt::TopRightCorner);
	}

	ccGLWindow* win = new3DView(true);
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
	if (!haveSelection())
		return;

	if (!m_seTool)
	{
		m_seTool = new ccSectionExtractionTool(this);
		connect(m_seTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateSectionExtractionMode);

		registerOverlayDialog(m_seTool, Qt::TopRightCorner);
	}
	m_seTool->setExtractMode(true);

	//add clouds
	ccGLWindow* firstDisplay = nullptr;
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
						firstDisplay = static_cast<ccGLWindow*>(entity->getDisplay());
					}
					
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
	unselectAllInDB();

	ccGLWindow* win = new3DView(false);
	if (!win)
	{
		ccLog::Error("[SectionExtraction] Failed to create dedicated 3D view!");
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

	ccGLWindow* win = getActiveGLWindow();
	if (win) {
		win->redraw();
	}
}

void MainWindow::activateSegmentationMode()
{
	ccGLWindow* win = getActiveGLWindow();
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
		if (entity->isKindOf(CC_TYPES::POINT_CLOUD) || entity->isKindOf(CC_TYPES::MESH)) {
			m_gsTool->addEntity(entity);
		}		
	}
	m_gsTool->setSegmentMode(ccGraphicalSegmentationTool::SEGMENT_GENERAL);

	if (m_gsTool->getNumberOfValidEntities() == 0)
	{
		ccConsole::Error("No segmentable entity in active window!");
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
	bool deleteHiddenParts = false;

	//shall we apply segmentation?
	if (state)
	{
		ccHObject* firstResult = nullptr;

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
					if (m_buildingRoot)
					{
						m_buildingRoot->getRootEntity()->filterChildren(labels, true, CC_TYPES::LABEL_2D);
					}
					if (m_imageRoot)
					{
						m_imageRoot->getRootEntity()->filterChildren(labels, true, CC_TYPES::LABEL_2D);
					}
					for (ccHObject::Container::iterator it = labels.begin(); it != labels.end(); ++it)
					{
						if ((*it)->isA(CC_TYPES::LABEL_2D)) //Warning: cc2DViewportLabel is also a kind of 'CC_TYPES::LABEL_2D'!
						{
							//we must search for all dependent labels and remove them!!!
							//TODO: couldn't we be more clever and update the label instead?
							cc2DLabel* label = static_cast<cc2DLabel*>(*it);
							bool removeLabel = false;
							for (unsigned i = 0; i < label->size(); ++i)
							{
								if (label->getPickedPoint(i).entity() == entity)
								{
									removeLabel = true;
									break;
								}
							}

							if (removeLabel && label->getParent())
							{
								ccLog::Warning(QString("[Segmentation] Label %1 depends on cloud %2 and will be removed").arg(label->getName(), cloud->getName()));
								ccHObject* labelParent = label->getParent();
								ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(labelParent);
								labelParent->removeChild(label);
								label = nullptr;
								putObjectBackIntoDBTree(labelParent,objContext);
							}
						}
					} //for each label
				} // if (cloud)

				//we temporarily detach the entity, as it may undergo
				//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::createNewCloudFromVisibilitySelection
				ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(entity);

				//apply segmentation
				ccHObject* segmentationResult = nullptr;
				bool deleteOriginalEntity = deleteHiddenParts;
				if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
				{
					ccGenericPointCloud* genCloud = ccHObjectCaster::ToGenericPointCloud(entity);
					ccGenericPointCloud* segmentedCloud = genCloud->createNewCloudFromVisibilitySelection(!deleteHiddenParts);
					if (segmentedCloud && segmentedCloud->size() == 0)
					{
						delete segmentationResult;
						segmentationResult = nullptr;
					}
					else
					{
						segmentationResult = segmentedCloud;
					}

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
						for (unsigned i = 0; i < entity->getChildrenNumber(); ++i)
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
					StPrimGroup* prim_group = nullptr;
					//we must take care of the remaining part
					if (!deleteHiddenParts)
					{
						//no need to put back the entity in DB if we delete it afterwards!
						if (!deleteOriginalEntity)
						{
							//! XYLIU
							switch (m_gsTool->getSegmentMode())
							{
							case ccGraphicalSegmentationTool::SEGMENT_GENERAL:
								entity->setName(entity->getName() + QString(".remaining"));
								break;
							case ccGraphicalSegmentationTool::SEGMENT_PLANE_CREATE:
							{
								int biggest = GetMaxNumberExcludeChildPrefix(objContext.parent, BDDB_PLANESEG_PREFIX);
								segmentationResult->setName(BDDB_PLANESEG_PREFIX + QString::number(biggest + 1));
								ccPointCloud* segment_cloud = ccHObjectCaster::ToPointCloud(segmentationResult);
								if (segment_cloud) {
									segment_cloud->setRGBColor(ccColor::Generator::Random());
									ccHObject* new_plane = FitPlaneAndAddChild(segment_cloud);
									if (new_plane) { 
										new_plane->setDisplay_recursive(getActiveGLWindow());
										SetGlobalShiftAndScale(new_plane);
										addToDB(new_plane, entity->getDBSourceType());
									}
								}
								break;
							}
							case ccGraphicalSegmentationTool::SEGMENT_PLANE_SPLIT:
							{
								//! get primitive group
								StBuilding* cur_building = GetParentBuilding(objContext.parent);
								if (!cur_building) { break; }
								BDBaseHObject* baseObj = GetRootBDBase(cur_building);
								if (!baseObj) { break; }
								prim_group = baseObj->GetPrimitiveGroup(cur_building->getName());
								if (!prim_group) { break; }

								int biggest = GetMaxNumberExcludeChildPrefix(prim_group, BDDB_PLANESEG_PREFIX);
								segmentationResult->setName(BDDB_PLANESEG_PREFIX + QString::number(biggest + 1));
								ccPointCloud* segment_cloud = ccHObjectCaster::ToPointCloud(segmentationResult);
								if (segment_cloud) {
									segment_cloud->setRGBColor(ccColor::Generator::Random());
									ccHObject* new_plane = FitPlaneAndAddChild(segment_cloud);
									if (new_plane) { 
										new_plane->setDisplay_recursive(getActiveGLWindow());
										SetGlobalShiftAndScale(new_plane);
										addToDB(new_plane, entity->getDBSourceType());
									}
								}
								break;
							}							
							default:
								break;
							}
							putObjectBackIntoDBTree(entity, objContext);
						}
					}
					else
					{
						//keep original name(s)
						segmentationResult->setName(entity->getName());
						//! XYLIU
						if (m_gsTool->getSegmentMode() == ccGraphicalSegmentationTool::SEGMENT_PLANE_CREATE) {
							ccPointCloud* segment_cloud = ccHObjectCaster::ToPointCloud(segmentationResult);
							if (segment_cloud) {
								segment_cloud->setRGBColor(segment_cloud->hasColors() ? segment_cloud->getPointColor(0) : ccColor::Generator::Random());
								ccHObject* new_plane = FitPlaneAndAddChild(segment_cloud);
								if (new_plane) addToDB(new_plane, segment_cloud->getDBSourceType(), false, false);
							}
						}						
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

					if (prim_group) // XYLIU
					{
						objContext.parent = prim_group;
					}
					else if (segmentationResult->isA(CC_TYPES::SUB_MESH))
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

					segmentationResult->setDisplay_recursive(getActiveGLWindow());
					segmentationResult->prepareDisplayForRefresh_recursive();
					SetGlobalShiftAndScale(segmentationResult);

					addToDB(segmentationResult, entity->getDBSourceType(), false, false);

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
					entity = nullptr;
				}
				else
				{
					++p;
				}
			}
		}

		//specific actions
		{
			for ( ccGenericPointCloud *cloud : verticesToReset )
			{
				cloud->resetVisibilityArray();
			}
		}

		if (firstResult) {
			setSelectedInDB(firstResult, true);
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
		connect(m_tplTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateTracePolylineMode);
		registerOverlayDialog(m_tplTool, Qt::TopRightCorner);
	}

	m_tplTool->linkWith(win);
	m_tplTool->setTraceMode(0);

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
	if (!haveOneSelection())
	{
		ccConsole::Error("Select one and only one entity!");
		return;
	}

	ccHObject* entity = m_selectedEntities[0];
	if (!entity->isKindOf(CC_TYPES::POINT_CLOUD) && !entity->isKindOf(CC_TYPES::MESH))
	{
		ccConsole::Error("Select a cloud or a mesh");
		return;
	}

	if (!entity->isVisible() || !entity->isEnabled())
	{
		ccConsole::Error("Entity must be visible!");
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
	ccGLWindow* win = getActiveGLWindow();
	if (!win)
	{
		return;
	}

	if (m_ccRoot)
	{
		m_ccRoot->unselectAllEntities(); //we don't want any entity selected (especially existing labels!)
	}
	if (m_buildingRoot)
	{
		m_buildingRoot->unselectAllEntities(); //we don't want any entity selected (especially existing labels!)
	}
	if (m_imageRoot)
	{
		m_imageRoot->unselectAllEntities(); //we don't want any entity selected (especially existing labels!)
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
	//	m_ppDlg->linkWith(0);

	//we enable all GL windows
	enableAll();

	freezeUI(false);

	updateUI();
}

void MainWindow::activateClippingBoxMode()
{
	if ( !haveSelection() )
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
		connect(m_clipTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateClippingBoxMode);
	}
	m_clipTool->linkWith(win);

	ccHObject::Container selectedEntities = getSelectedEntities(); //we have to use a local copy: 'unselectEntity' will change the set of currently selected entities!
	for (ccHObject *entity : selectedEntities)
	{
		if (m_clipTool->addAssociatedEntity(entity))
		{
			//automatically deselect the entity (to avoid seeing its bounding box ;)
			unselectAllInDB();
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
	if (!haveSelection())
		return;

	ccGLWindow* win = getActiveGLWindow();
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
		connect(m_transTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateTranslateRotateMode);
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
		if (state && m_ccRoot && m_buildingRoot && m_imageRoot)
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
				m_buildingRoot->selectEntities(transformedEntities);
				m_imageRoot->selectEntities(transformedEntities);
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

//////////////////////////////////////////////////////////////////////////

// viewer toggle state

void MainWindow::toggleFullScreen(bool state)
{
	if (state)
		showFullScreen();
	else
		showNormal();

#ifdef Q_OS_MAC
	if (state)
	{
		m_UI->actionFullScreen->setText(tr("Exit Full Screen"));
	}
	else
	{
		m_UI->actionFullScreen->setText(tr("Enter Full Screen"));
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

void MainWindow::toggleActiveWindowAutoPickRotCenter(bool state)
{
	ccGLWindow* win = getActiveGLWindow();
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
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->showCursorCoordinates(state);
	}
}

void MainWindow::toggleActiveWindowPointViewEditMode(bool state)
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->setPointViewEditMode(state);
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

			if (	win->getStereoParams().glassType == ccGLWindow::StereoParams::NVIDIA_VISION
				||	win->getStereoParams().glassType == ccGLWindow::StereoParams::GENERIC_STEREO_DISPLAY)
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

			ccGLWindow::StereoParams params = smDlg.getParameters();
			ccLog::Warning(QString("%1").arg(params.stereoStrength));
#ifndef CC_GL_WINDOW_USE_QWINDOW
			if (!params.isAnaglyph())
			{
				ccLog::Error("This version doesn't handle stereo glasses and headsets.\nUse the 'Stereo' version instead.");
				//activation of the stereo mode failed: cancel selection
				m_UI->actionEnableStereo->blockSignals(true);
				m_UI->actionEnableStereo->setChecked(false);
				m_UI->actionEnableStereo->blockSignals(false);
				return;
			}
#endif

			//force perspective state!
			if (!win->getViewportParameters().perspectiveView)
			{
				setCenteredPerspectiveView(win, false);
			}

			if (	params.glassType == ccGLWindow::StereoParams::NVIDIA_VISION
				||	params.glassType == ccGLWindow::StereoParams::GENERIC_STEREO_DISPLAY)
			{
				//force (exclusive) full screen
				m_UI->actionExclusiveFullScreen->setChecked(true);
			}

			if (smDlg.updateFOV())
			{
				//set the right FOV
				double fov_deg = 2 * std::atan(params.screenWidth_mm / (2.0 * params.screenDistance_mm)) * CC_RAD_TO_DEG;
				ccLog::Print(QString("[Stereo] F.O.V. forced to %1 deg.").arg(fov_deg));
				win->setFov(fov_deg);
			}

			if (!win->enableStereoMode(params))
			{
				if (	params.glassType == ccGLWindow::StereoParams::NVIDIA_VISION
					||	params.glassType == ccGLWindow::StereoParams::GENERIC_STEREO_DISPLAY)
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

		if (QMessageBox::question(this,
			"Stereo mode",
			"Stereo-mode only works in perspective mode. Do you want to disable it?",
			QMessageBox::Yes,
			QMessageBox::No) == QMessageBox::No)
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



//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
/// Building Reconstruction

void MainWindow::doActionBDDisplayPlaneOn()
{
	ccHObject* Root_Entity = nullptr;
	if (haveSelection())
		Root_Entity = m_selectedEntities.front();
	else
		Root_Entity = db(getCurrentDB())->getRootEntity();

	ccHObject::Container Objs;
	Root_Entity->filterChildren(Objs, true, CC_TYPES::PLANE, true);

	ProgStart("Show Plane");
	for (auto & Obj : Objs) {
		Obj->setVisible(true);
		Obj->redrawDisplay();
	}
	ProgEnd

	refreshAll();
	UpdateUI();
}

void MainWindow::doActionBDDisplayPlaneOff()
{
	ccHObject* Root_Entity = nullptr;
	if (haveSelection())
		Root_Entity = m_selectedEntities.front();
	else
		Root_Entity = db(getCurrentDB())->getRootEntity();

	ccHObject::Container Objs;
	Root_Entity->filterChildren(Objs, true, CC_TYPES::PLANE, true);
	ProgStartNorm("Hide Plane", Objs.size());
	for (auto & Obj : Objs) {
		Obj->setVisible(false);
		Obj->redrawDisplay();
		ProgStep()
	}
	ProgEnd

	refreshAll();
	UpdateUI();
}

void MainWindow::doActionBDDisplayPointOn()
{
	ccHObject* Root_Entity = nullptr;
	if (haveSelection())
		Root_Entity = m_selectedEntities.front();
	else
		Root_Entity = db(getCurrentDB())->getRootEntity();

	ccHObject::Container Objs;
	Root_Entity->filterChildren(Objs, true, CC_TYPES::POINT_CLOUD, true);
	ProgStart("Show Points");
	for (auto & Obj : Objs) {
		Obj->setVisible(true);
		Obj->redrawDisplay();
	}
	ProgEnd
	refreshAll();
	UpdateUI();
}

void MainWindow::doActionBDDisplayPointOff()
{
	ccHObject* Root_Entity = nullptr;
	if (haveSelection())
		Root_Entity = m_selectedEntities.front();
	else
		Root_Entity = db(getCurrentDB())->getRootEntity();

	ccHObject::Container Objs;
	Root_Entity->filterChildren(Objs, true, CC_TYPES::POINT_CLOUD, false);
	ProgStart("Hide Points");
	for (auto & Obj : Objs) {
		Obj->setVisible(false);
		Obj->redrawDisplay();
	}
	ProgEnd
	refreshAll();
	UpdateUI();
}

void MainWindow::doActionDisplayWireframe()
{
	for (size_t i = 0; i < m_selectedEntities.size(); i++) {
		ccMesh* mesh = ccHObjectCaster::ToMesh(m_selectedEntities[i]);
		if (mesh) {
			mesh->showWired(m_UI->actionDisplayWireframe->isChecked());
			mesh->prepareDisplayForRefresh_recursive();
		}
	}

	UpdateUI();
	refreshAll();
}

void MainWindow::doActionDisplayFace()
{
	for (size_t i = 0; i < m_selectedEntities.size(); i++) {
		ccMesh* mesh = ccHObjectCaster::ToMesh(m_selectedEntities[i]);
		if (mesh) {
			mesh->showFaces(m_UI->actionDisplayFace->isChecked());
			mesh->prepareDisplayForRefresh_recursive();
		}
	}

	UpdateUI();
	refreshAll();
}

void MainWindow::doActionDisplayNormalPerFace()
{
	ProgStartNorm("display normal per face", m_selectedEntities.size())
	for (size_t i = 0; i < m_selectedEntities.size(); i++) {
		ccMesh* mesh = ccHObjectCaster::ToMesh(m_selectedEntities[i]);
		if (!mesh) continue;
		if (m_UI->actionDisplayNormalPerFace->isChecked()) {
			if (!mesh->hasTriNormals()) {
				if (!mesh->computeNormals(false)) {
					continue;
				}
			}
			mesh->showFaces(true);
			mesh->showTriNorms(true);
			mesh->notifyNormalUpdate();
			mesh->getAssociatedCloud()->notifyGeometryUpdate();
			mesh->prepareDisplayForRefresh_recursive();
		}
		else {
			if (!m_UI->actionDisplayNormalPerVertex->isChecked()) {// both disable
				mesh->showNormals(false);
				mesh->notifyNormalUpdate();
				mesh->getAssociatedCloud()->notifyGeometryUpdate();
				mesh->prepareDisplayForRefresh_recursive();
			}
		}
		ProgStep()
	}
	ProgEnd

	UpdateUI();
	refreshAll();
}

void MainWindow::doActionDisplayNormalPerVertex()
{
	ProgStartNorm("display normal per vertex", m_selectedEntities.size())
	for (size_t i = 0; i < m_selectedEntities.size(); i++) {
		if (m_selectedEntities[i]->isA(CC_TYPES::POINT_CLOUD)) {
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_selectedEntities[i]);
			if (!cloud) continue;
			cloud->showNormals(m_UI->actionDisplayNormalPerVertex->isChecked() && cloud->hasNormals());
		}
		else if (m_selectedEntities[i]->isKindOf(CC_TYPES::MESH)) {
			ccMesh* mesh = ccHObjectCaster::ToMesh(m_selectedEntities[i]);
			if (!mesh) continue;
			if (m_UI->actionDisplayNormalPerVertex->isChecked()) {
				if (!mesh->getAssociatedCloud()->hasNormals()) {
					if (!mesh->computeNormals(true)) {
						continue;
					}
				}
				mesh->showFaces(true);
				mesh->showNormals(true);
				mesh->showTriNorms(false);
				mesh->notifyNormalUpdate();
				mesh->getAssociatedCloud()->notifyGeometryUpdate();
				mesh->prepareDisplayForRefresh_recursive();
			}
			else {
				if (!m_UI->actionDisplayNormalPerFace->isChecked()) {// both disable
					mesh->showNormals(false);
					mesh->getAssociatedCloud()->notifyGeometryUpdate();
					mesh->prepareDisplayForRefresh_recursive();
				}
			}
		}
		ProgStep()
	}
	ProgEnd

	UpdateUI();
	refreshAll();
}

//////////////////////////////////////////////////////////////////////////
/// Building Reconstruction

//////////////////////////////////////////////////////////////////////////
QString s_no_project_error = "please open the main project!";
BDBaseHObject::Container GetBDBaseProjx() {
	MainWindow* main = MainWindow::TheInstance();
	ccHObject* root_entity = main->db_building()->getRootEntity();
	assert(root_entity);
	vector<BDBaseHObject*> prjx;
	for (size_t i = 0; i < root_entity->getChildrenNumber(); i++) {
		ccHObject* child = root_entity->getChild(i);
		if (isBuildingProject(child))	{
			prjx.push_back(static_cast<BDBaseHObject*>(child));
		}
	}
	return prjx;
}

ccHObject* MainWindow::LoadBDReconProject(QString Filename)
{
	BDBaseHObject* bd_grp = nullptr;

	stocker::BlockProj block_prj; std::string error_info;
	if (!stocker::LoadProject(Filename.toStdString(), block_prj, error_info)) {
		std::cout << error_info << std::endl;
		//dispToConsole(error_info.c_str(), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return nullptr;
	}

	QFileInfo prj_file(Filename);
	QString prj_name = prj_file.completeBaseName();

	QString bin_file = prj_file.absolutePath() + "\\" + prj_name + ".bin";

	if (QFileInfo(bin_file).exists()) {
		std::cout << "start loading " << bin_file.toStdString() << std::endl;
		QStringList files; files.append(bin_file);
		ccHObject::Container loaded = addToDB_Build(files);
		ccHObject* newGroup = loaded.empty() ? nullptr : loaded.front();

		if (newGroup) {
			bd_grp = new BDBaseHObject(prj_name);
			bd_grp->setName(prj_name);
			newGroup->transferChildren(*bd_grp);
			removeFromDB(newGroup);
			std::cout << bin_file.toStdString() << " loaded" << std::endl;
		}
	}
	if (!bd_grp) {

		QStringList names; QStringList files;
		for (auto & bd : block_prj.m_builder.sbuild) {
			QString building_name = bd->GetName().Str().c_str(); names.append(building_name);
			QFileInfo point_path(bd->data.file_path.ori_points.c_str()); files.append(point_path.absoluteFilePath());
			std::cout << "file: " << point_path.absoluteFilePath().toStdString() << std::endl;
		}
		std::cout << "loading " << files.size() << " files" << std::endl;
		ccHObject::Container loaded = addToDB_Build(files);
		std::cout << files.size() << " files loaded" << std::endl;
		if (loaded.size() == names.size()) {
			bd_grp = new BDBaseHObject(prj_name);
			for (size_t i = 0; i < names.size(); i++) {
				ccHObject* newGroup = loaded[i]; if (!newGroup) { continue; }
				QString building_name = names[i];
				StBuilding* building = new StBuilding(building_name);
				building->setName(building_name);
				newGroup->transferChildren(*building);
				ccHObject::Container clouds;
				building->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD); if (clouds.empty()) continue;
				ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(clouds.front()); if (!cloud) { continue; }
				cloud->setName(building_name + BDDB_ORIGIN_CLOUD_SUFFIX);
				if (cloud->hasColors()) {
					cloud->showSF(false);
					cloud->showColors(true);
				}
				removeFromDB(newGroup);

				bd_grp->addChild(building);
			}
		}
	}
	//! prepare buildings now
	if (bd_grp) {
		bd_grp->block_prj = block_prj;

		bool first_cloud = false;
		for (size_t i = 0; i < bd_grp->getChildrenNumber(); i++) {
			StBuilding* bdObj = ccHObjectCaster::ToStBuilding(bd_grp->getChild(i));
			if (!bdObj) continue;

			auto sp_build = bd_grp->GetBuildingSp(bdObj->getName().toStdString());
			if (!sp_build) {
				continue;
			}

			if (!LoadBuildingInfo(sp_build->data, sp_build->data.file_path.info)) {
				ccPointCloud* cloud = bd_grp->GetOriginPointCloud(bdObj->getName(), false);
				stocker::Contour3d points_global; stocker::Contour3f points_local;
				if (!GetPointsFromCloud(cloud, points_global, points_local)) { continue; }

				CCVector3d minbb, maxbb;
				if (cloud->getGlobalBB(minbb, maxbb)) {
					sp_build->data.bbox.Add({ minbb.x,minbb.y, minbb.z });
					sp_build->data.bbox.Add({ maxbb.x,maxbb.y, maxbb.z });
				}

// 				if (!stocker::PrepareBuildingByPoints(bd_grp->block_prj, sp_build->data, points_global, points_local)) {
// 					continue;
// 				}

				// TODO: save these paras to the StBuilding
				sp_build->data.average_spacing = stocker::ComputeAverageSpacing3f(points_local, true);
				stocker::Contour3d ground_contour = stocker::CalcBuildingGround(points_global, 0.1);
				if (ground_contour.size() < 3) {
					return false;
				}
				sp_build->data.ground_height = ground_contour.front().Z();
				sp_build->data.convex_hull_xy = stocker::ToContour2d(ground_contour);
				if (bd_grp->block_prj.m_options.with_image) {
					for (auto & img : bd_grp->block_prj.m_builder.simage) {
						bool img_check = false;
						for (size_t i = 0; i < 8; i++) {
							if (img->data.CheckInImg3d(sp_build->data.bbox.P(i))) {
								img_check = true;
								break;
							}
						}
						if (img_check) {
							sp_build->data.image_list.push_back(img->data.GetName().Str());
							bd_grp->block_prj.m_builder.InsertImageBuild(img->data.GetName(), sp_build->data.GetName());
						}
					}
				}

				// TODO: still wrong sometimes, the file cannot be saved??
				if (!SaveBuildingInfo(sp_build->data, sp_build->data.file_path.info)) {
					cout << "failed to save building info: " << sp_build->data.GetName().Str() << endl;
					return false;
				}

			}
			else {
				for (auto & img : sp_build->data.image_list) {
					bd_grp->block_prj.m_builder.InsertImageBuild(img.c_str(), sp_build->data.GetName());
				}
			}

			if (!first_cloud) {
				ccPointCloud* cloud = bd_grp->GetOriginPointCloud(bdObj->getName(), false);
				if (cloud) {
					bd_grp->global_shift = stocker::parse_xyz(cloud->getGlobalShift());
					bd_grp->global_scale = cloud->getGlobalScale();
					first_cloud = true;
				}
			}
		}
		std::cout << "building project prepared!" << std::endl;
	}
	
	return bd_grp;
}

void MainWindow::doActionBDProjectLoad()
{
	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::LoadFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	QString Filename =
		QFileDialog::getOpenFileName(this,
			"Open project file",
			currentPath,
			"project (*.bbprj)");

	if (Filename.isEmpty()) return;

	//save last loading parameters
	currentPath = QFileInfo(Filename).absolutePath();
	settings.setValue(ccPS::CurrentPath(), currentPath);
	settings.endGroup();

	//////////////////////////////////////////////////////////////////////////
	 
	try	{
		ccHObject* bd_grp = LoadBDReconProject(Filename);
		if (bd_grp) {
			std::cout << "add to database: " << bd_grp->getName().toStdString() << std::endl;
			switchDatabase(CC_TYPES::DB_BUILDING);
			addToDB_Build(bd_grp, false, false, false, true);
			std::cout << "database added" << std::endl;
		}
		else {
			dispToConsole("error load project", ERR_CONSOLE_MESSAGE);
			return;
		}
	}
	catch (const std::exception& e)
	{
		STOCKER_ERROR_ASSERT(e.what());
		return;
	}
}

bool SaveProject(BDBaseHObject* proj)
{
	try {
		QFileInfo prj_file(proj->block_prj.m_options.prj_file.project_ini.c_str());
		QString prj_name = prj_file.completeBaseName();

		QString selectedFilename = prj_file.absolutePath() + "\\" + prj_name + ".bin";

		CC_FILE_ERROR result = CC_FERR_NO_ERROR;
		FileIOFilter::SaveParameters parameters;
		{
			parameters.alwaysDisplaySaveDialog = true;
			parameters.parentWidget = MainWindow::TheInstance();
		}

		//specific case: BIN format			
		result = FileIOFilter::SaveToFile(proj, selectedFilename, parameters, BinFilter::GetFileFilter());
	}
	catch (const std::exception& e) {
		throw(std::runtime_error(e.what()));
		STOCKER_ERROR_ASSERT(e.what());
		return false;
	}
	return true;
}

void MainWindow::doActionBDProjectSave()
{
	BDBaseHObject::Container prjx;
	if (haveSelection()) {
		prjx.push_back(GetRootBDBase(getSelectedEntities().front()));		
	}
	else {	// save all projects		
		prjx = GetBDBaseProjx();
	}
	
	try {
		for (BDBaseHObject* proj : prjx) {
			if (!SaveProject(proj))	{
				dispToConsole("cannot save project", ERR_CONSOLE_MESSAGE);
				continue;
			}			
		}
	}
	catch (const std::runtime_error& e) {
		dispToConsole("cannot save project", ERR_CONSOLE_MESSAGE);
		dispToConsole(e.what(), ERR_CONSOLE_MESSAGE);
		return;
	}	
}

void MainWindow::doActionBDImagesLoad()
{
	if (!haveSelection()) {
		dispToConsole("please load project or select a point cloud", ERR_CONSOLE_MESSAGE);
		return;
	}
	BDBaseHObject* baseObj = GetRootBDBase(getSelectedEntities().front());
	ccHObject::Container camera_groups = GetEnabledObjFromGroup(m_imageRoot->getRootEntity(), CC_TYPES::ST_PROJECT);
	if (baseObj) {
		//ccHObject* camera_group = getCameraGroup(baseObj->getName());
		
		/// temporarily put this function here, need add a button
		if (getSelectedEntities().front()->isA(CC_TYPES::ST_BUILDING)) {
			for (ccHObject* camera_group : camera_groups) {
				QStringList building_images;
				for (ccHObject* bd : getSelectedEntities()) {
					stocker::BuildUnit bd_unit = baseObj->GetBuildingUnit(bd->getName().toStdString());
					for (auto img : bd_unit.image_list) {
						building_images.append(img.c_str());
					}
				}
				filterCameraByName(camera_group, building_images);
			}
			refreshAll();
			return;
		}
	}
	if (!baseObj && !getSelectedEntities().front()->isA(CC_TYPES::POINT_CLOUD))	{
		dispToConsole("please load project or select a point cloud", ERR_CONSOLE_MESSAGE);
		return;
	}
// 	if (camera_group) {
// 		if (QMessageBox::question(this,	
// 			"Import camera", "cameras exist, import again?",
// 			QMessageBox::Yes, QMessageBox::No)
// 			== QMessageBox::No)
// 			return;
// 	}
	CCVector3d loadCoordinatesShift = baseObj ? CCVector3d(vcgXYZ(baseObj->global_shift)) : CCVector3d(0, 0, 0);
	bool loadCoordinatesTransEnabled = false;
	FileIOFilter::LoadParameters parameters;
	{
		parameters.alwaysDisplayLoadDialog = true;
		parameters.shiftHandlingMode = ccGlobalShiftManager::DIALOG_IF_NECESSARY;
		parameters.coordinatesShift = &loadCoordinatesShift;
		parameters.coordinatesShiftEnabled = &loadCoordinatesTransEnabled;
		parameters.parentWidget = this;
	}
	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	QString out_file, image_list;
	if (baseObj && baseObj->block_prj.m_options.with_image) {
		out_file = baseObj->block_prj.m_options.prj_file.sfm_out.c_str();
		image_list = baseObj->block_prj.m_options.prj_file.image_list.c_str();		
	}

	if (!QFileInfo(out_file).exists() || !QFileInfo(image_list).exists()) {
		if (baseObj) {
			dispToConsole("no .out or image list exists", ERR_CONSOLE_MESSAGE);
		}

		//persistent settings
		QSettings settings;
		settings.beginGroup(ccPS::LoadFile());
		QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
		
		out_file =
			QFileDialog::getOpenFileName(this,
				"Import images",
				currentPath,
				"Bundler output (*.out)");
		if (out_file.isEmpty()) return;

		//save last loading parameters
		currentPath = QFileInfo(out_file).absolutePath();
		settings.setValue(ccPS::CurrentPath(), currentPath);
		settings.endGroup();
		if (baseObj) {
			image_list =
				QFileDialog::getOpenFileName(this,
					"Import images",
					currentPath,
					"Image list (All (*.*);;*.txt");
			if (image_list.isEmpty()) return;

			//save last loading parameters
			currentPath = QFileInfo(image_list).absolutePath();
			settings.setValue(ccPS::CurrentPath(), currentPath);
			settings.endGroup();
		}
	}
	ccPointCloud* hackObj = nullptr;
	if (baseObj) {
		hackObj = new ccPointCloud(image_list);
		hackObj->setGlobalShift(CCVector3d(vcgXYZ(baseObj->global_shift)));
		ccBBox base_box = baseObj->getBB_recursive(false, false);
		hackObj->reserve(2);
		hackObj->addPoint(base_box.getCenter() + base_box.getDiagVec() / 2);
		hackObj->addPoint(base_box.getCenter() - base_box.getDiagVec() / 2);
	}
	else {
		hackObj = ccHObjectCaster::ToPointCloud(getSelectedEntities().front());
	}
	if (hackObj) {
		parameters.additionInfo = (void*)hackObj;
	}
	QString group_name = QFileInfo(out_file).baseName();
	BDImageBaseHObject* bd_grp = nullptr;
	for (ccHObject* cam_group : camera_groups) {
		if (cam_group->getName() == group_name) {
			bd_grp = static_cast<BDImageBaseHObject*>(cam_group);
			break;
		}
	}
	ccHObject* newGroup = FileIOFilter::LoadFromFile(out_file, parameters, result, QString());
	if (!newGroup) {
		return;
	}
	if (!bd_grp) {
		bd_grp = new BDImageBaseHObject(*newGroup);
		bd_grp->setName(group_name);
	}
	for (int i = 0; i < newGroup->getChildrenNumber(); i++) {
		ccHObject* child = newGroup->getChild(i);
		ccHObject::Container check_exist;
		bd_grp->filterChildrenByName(check_exist, false, child->getName(), true);
		if (check_exist.empty()) {
			newGroup->transferChild(child, *bd_grp);
			i--;
		}
	}
	//newGroup->transferChildren(*bd_grp);
		
	addToDB_Image(bd_grp, false, false, false, true);

	if (baseObj && hackObj) {
		delete hackObj;
		hackObj = nullptr;
	}
	if (newGroup) {
		delete newGroup;
		newGroup = nullptr;
	}
		
	refreshAll();
}

// for point cloud (.original by default)
void MainWindow::doActionBDPlaneSegmentation()
{
	if (!haveSelection()) return;	

	ccHObject *entity = getSelectedEntities().front();
	ccHObject::Container _container;
	BDBaseHObject* baseObj = GetRootBDBase(entity);

	if (entity->isA(CC_TYPES::POINT_CLOUD))
		_container.push_back(entity);
	else {
		ccHObject::Container building_groups;		
		if (!baseObj) {
			dispToConsole(s_no_project_error, ERR_CONSOLE_MESSAGE);
			return;
		}
		if (isBuildingProject(entity)) {
			building_groups = GetEnabledObjFromGroup(baseObj, CC_TYPES::ST_BUILDING, true, false);
		}
		else if (entity->isA(CC_TYPES::ST_BUILDING)){			
			building_groups.push_back(entity);
		}
		for (ccHObject* bd : building_groups) {
			ccHObject* pc = baseObj->GetOriginPointCloud(bd->getName(), true);
			if (pc) _container.push_back(pc);
		}
	}

	if (_container.empty()) {
		dispToConsole("[BDRecon] Please select (group of) point cloud / building", ERR_CONSOLE_MESSAGE);
		return;
	}


	if (!m_pbdrPSDlg) m_pbdrPSDlg = new bdrPlaneSegDlg(this);
	m_pbdrPSDlg->setPointClouds(_container);
	if (!m_pbdrPSDlg->exec()) {
		return;
	}

	if (!m_pbdrPSDlg->PlaneSegATPSRadioButton->isChecked()) {
		// check have not normals
		ccHObject::Container normal_container;
		for (auto & cloudObj : _container) {
			ccPointCloud* entity_cloud = ccHObjectCaster::ToPointCloud(cloudObj);
			if (!entity_cloud->hasNormals()) {
				normal_container.push_back(cloudObj);
			}
		}
		if (!normal_container.empty()) {
			if (!ccEntityAction::computeNormals(normal_container, this))
				return;
		}
	}

	double merge_threshold(-1), split_threshold(-1);
	if (m_pbdrPSDlg->MergeCheckBox->isChecked()) {
		merge_threshold = m_pbdrPSDlg->MergeThresholdDoubleSpinBox->value();		
	}
	if (m_pbdrPSDlg->SplitCheckBox->isChecked()) {
		split_threshold = m_pbdrPSDlg->SplitThresholdDoubleSpinBox->value();
	}
	int support_pts = m_pbdrPSDlg->supportPointsSpinBox->value();
	double distance_eps = m_pbdrPSDlg->DistanceEpsilonDoubleSpinBox->value();
	double cluster_eps = m_pbdrPSDlg->ClusterEpsilonDoubleSpinBox->value();

	ProgStartNorm("Plane Segmentation", _container.size())
	for (int i = 0; i < _container.size(); i++) {
		ccHObject* cloudObj = _container[i];
		ccPointCloud* todo_point = nullptr;
		if (baseObj) {
			todo_point = baseObj->GetTodoPoint(GetBaseName(cloudObj->getName()));
		}
		else {
			ccPointCloud* pc = ccHObjectCaster::ToPointCloud(cloudObj);
			if (pc) {
				todo_point = new ccPointCloud("unassigned");
				todo_point->setGlobalScale(pc->getGlobalScale());
				todo_point->setGlobalShift(pc->getGlobalShift());
				todo_point->setDisplay(pc->getDisplay());
				todo_point->showColors(true);
				if (pc->getParent())
					pc->getParent()->addChild(todo_point);
				addToDB(todo_point, pc->getDBSourceType(), false, false);
			}
		}

		ccHObject* seged = nullptr;
		if (m_pbdrPSDlg->PlaneSegRansacRadioButton->isChecked()) {
			double normal_dev = cos(m_pbdrPSDlg->maxNormDevAngleSpinBox->value() * CC_DEG_TO_RAD);
			double prob = m_pbdrPSDlg->probaDoubleSpinBox->value();
			seged = PlaneSegmentationRansac(cloudObj,
				support_pts,
				distance_eps,
				cluster_eps,
				normal_dev,
				prob,
				merge_threshold, split_threshold,
				todo_point);
		}
		else if (m_pbdrPSDlg->PlaneSegRegionGrowRadioButton->isChecked()) {
			double growing_radius = m_pbdrPSDlg->GrowingRadiusDoubleSpinBox->value();
			seged = PlaneSegmentationRgGrow(cloudObj,
				support_pts,
				distance_eps,
				cluster_eps,
				growing_radius,
				merge_threshold, split_threshold);
		}
		else if (m_pbdrPSDlg->PlaneSegATPSRadioButton->isChecked()) {
			double curvature_delta = m_pbdrPSDlg->APTSCurvatureSpinBox->value();
			double nfa_epsilon = m_pbdrPSDlg->APTSNFASpinBox->value();
			double normal_theta = m_pbdrPSDlg->APTSNormalSpinBox->value();

			if (m_pbdrPSDlg->autoParaCheckBox->isChecked()) {
				seged = PlaneSegmentationATPS(cloudObj, todo_point);
			}
			else {
				seged = PlaneSegmentationATPS(cloudObj, todo_point,
					&support_pts,
					&curvature_delta,
					&distance_eps,
					&cluster_eps,
					&nfa_epsilon,
					&normal_theta);
			}
		}
		if (todo_point->size() == 0) {
			removeFromDB(todo_point);
		}

		if (seged) {
			addToDB(seged, entity->getDBSourceType(), false, false);
			cloudObj->setEnabled(false);
		}
		ProgStep()
	}
	ProgEnd

	refreshAll();
	updateUI();
}

// for plane/plane_cloud (.primitive by default)
// this function is useless if the plane segmentation is done by ourselves.
void MainWindow::doActionBDRetrieve()
{
	if (!haveSelection()) return;
	//! retrieve unassigned points from original
	ccHObject* select = m_selectedEntities.front();
	ccHObject::Container buildings;
	if (isBuildingProject(select)) {
		buildings = GetEnabledObjFromGroup(select, CC_TYPES::ST_BUILDING, true, false);
	}
	else if (select->isA(CC_TYPES::ST_BUILDING)) {
		buildings.push_back(select);
	}
	BDBaseHObject* baseObj = GetRootBDBase(select);
	if (!baseObj) {
		return;
	}
	for (ccHObject* building_entity : buildings) {
		QString building_name = building_entity->getName();
		StPrimGroup* prim_group = baseObj->GetPrimitiveGroup(building_name);
		ccPointCloud* orig_cloud = baseObj->GetOriginPointCloud(building_name, false);
		ccPointCloud* todo_cloud = baseObj->GetTodoPoint(building_name);
		
		if (!prim_group || !orig_cloud || !todo_cloud) {
			continue;
		}
		todo_cloud->clear();
		RetrieveUnassignedPoints(orig_cloud, prim_group, todo_cloud);
		todo_cloud->prepareDisplayForRefresh_recursive();
	}
	refreshAll();
}

void MainWindow::doActionBDRetrievePlanePoints()
{
	return; // TODO: didn't finished yet
	if (getSelectedEntities().size() != 1) { dispToConsole("select only one entity", ERR_CONSOLE_MESSAGE); return; }

	ccHObject* select = getSelectedEntities().front();
	ccHObject::Container plane_container = GetPlaneEntitiesBySelected(select);
	BDBaseHObject* baseObj = GetRootBDBase(select); 
	if (!baseObj) { dispToConsole(s_no_project_error, ERR_CONSOLE_MESSAGE); return; }
	ProgStartNorm("retrieve plane points", plane_container.size())
	for (auto pl : plane_container) {
		GetParentBuilding(pl);

		ProgStep()
	}
	ProgEnd

	refreshAll();
	UpdateUI();
}

void MainWindow::doActionBDImageLines()
{
	// Dialog
	bdrLine3DppDlg line3dDlg(this);
	if (!line3dDlg.exec()) {
		return;
	}
	std::string bundle_out;
	std::string image_list;
	std::string building_list;
	std::string output_dir;
	std::string output_name;


	// TODO : thread
	stocker::GenerateLine3DPP(
		bundle_out.c_str(),
		image_list.c_str(),
		building_list.c_str(),
		output_dir.c_str(),
		output_name.c_str());

	QStringList files;
	QString file_path = QString(output_dir.c_str()) + QString(output_name.c_str());
	files.append(file_path);
	addToDB_Build(files);
}

void MainWindow::doActionBDPrimIntersections()
{
	if (!haveSelection()) return;

	// Available selection, root / primitive group / planes under the same primitive group

	std::vector<ccHObject::Container> building_prims;
	
	if (m_selectedEntities.size() == 1) {
		ccHObject* entity = m_selectedEntities.front();
		if (isBuildingProject(entity)) {
			BDBaseHObject* baseObj = GetRootBDBase(entity);
			ccHObject::Container buildings = GetEnabledObjFromGroup(baseObj, CC_TYPES::ST_BUILDING, true, false);
			for (ccHObject* bd : buildings)	{
				StPrimGroup* primGroup = baseObj->GetPrimitiveGroup(bd->getName());
				if (!primGroup->isEnabled()) continue;
				ccHObject::Container cur_valid_planes = primGroup->getValidPlanes();
				if (cur_valid_planes.size() > 1) {
					building_prims.push_back(cur_valid_planes);
				}
			}
		}
		else if (entity->isA(CC_TYPES::ST_PRIMGROUP)) {
			ccHObject::Container entity_planes = GetEnabledObjFromGroup(entity, CC_TYPES::PLANE);
			if (entity_planes.size() >= 2) {
				building_prims.push_back(entity_planes);
			}
		}
	}
	else {
		// TODO select planes / plane clouds under the same primitive group
		ccHObject::Container entity_planes;
		ccHObject::Container entity_polylines;
		for (auto & entity : m_selectedEntities) {
			if (entity->isA(CC_TYPES::PLANE)) {
				entity_planes.push_back(entity);
			}
			else if (entity->isA(CC_TYPES::POLY_LINE)) {
				entity_polylines.push_back(entity);
			}
		}
		if (entity_planes.size() >= 2) {
			building_prims.push_back(entity_planes);
		}
		if (entity_polylines.size() == 2) {
			CreateIntersectionPoint(entity_polylines[0], entity_polylines[1]);	// temporily put here for test
			refreshAll();
			updateUI();
			return;
		}
	}
	if (building_prims.empty()) return;

	bool ok = true;
	double distance = QInputDialog::getDouble(this, "Input Dialog", "Please input distance threshold", 2.0, 0.0, 999999.0, 1, &ok);
	if (!ok) return;
	ProgStartNorm("Plane Intersection", building_prims.size())
	for (size_t i = 0; i < building_prims.size(); i++) {
		ccHObject::Container segs = CalcPlaneIntersections(building_prims[i], distance);
		for (auto & seg : segs) {
			SetGlobalShiftAndScale(seg);
			addToDB(seg, building_prims[i].front()->getDBSourceType(), false, false);
		}
		ProgStep()
	}
	ProgEnd
	refreshAll();
	updateUI();
}

double s_assign_image_line_distance = 0.2;
void MainWindow::doActionBDPrimAssignSharpLines()
{
	if (!haveSelection()) return;
	bool ok = true;
	double input = QInputDialog::getDouble(this, "Input Dialog", "Please input distance threshold", s_assign_image_line_distance, 0.0, 999999.0, 1, &ok);
	if (!ok) return;
	s_assign_image_line_distance = input;

	stocker::Polyline3d all_sharp_lines;
	ccHObject::Container primitive_groups;	
	
	ccHObject* select = m_selectedEntities.front();
	BDBaseHObject* baseObj = GetRootBDBase(select);

	ccHObject::Container buildings;
	if (isBuildingProject(select)) {
		buildings = GetEnabledObjFromGroup(select, CC_TYPES::ST_BUILDING, true, false);
	}
	else if (select->isA(CC_TYPES::ST_BUILDING)) {			
		buildings.push_back(select);
	}
	if (!buildings.empty() && baseObj) {
		std::string error_info;
		if (!stocker::LoadLine3D(baseObj->block_prj.m_options.prj_file.image_border, all_sharp_lines, error_info)) {
			std::cout << error_info << std::endl;
		}
		//! to local
		for (auto & seg : all_sharp_lines) {			
			seg.P0() = baseObj->ToLocal(seg.P0());
			seg.P1() = baseObj->ToLocal(seg.P1());
		}			
		for (auto & bd : buildings)	{
			ccHObject* planegroup_get = baseObj->GetPrimitiveGroup(GetBaseName(bd->getName()));
			if (planegroup_get)	primitive_groups.push_back(planegroup_get);
		}
	}
	
	if (!all_sharp_lines.empty() && primitive_groups.empty()) {
		return;
	}
	if (all_sharp_lines.empty()) {
		//! select the sharp line group
		ccHObject* sharp_group = askUserToSelect(CC_TYPES::HIERARCHY_OBJECT, 0, "please select a segments group");
		ccHObject::Container sharp_container = GetEnabledObjFromGroup(sharp_group, CC_TYPES::POLY_LINE);
		all_sharp_lines = GetPolylineFromEntities(sharp_container);
		if (all_sharp_lines.empty()) return;
	}
	if (primitive_groups.empty()) {
		//! select the plane group
		ccHObject* plane_group_ = askUserToSelect(CC_TYPES::ST_PRIMGROUP, 0, "please select a plane group");
		if (plane_group_) { primitive_groups.push_back(plane_group_); }
		if (primitive_groups.empty()) return;
	}

	ProgStartNorm("Assign Image Lines", primitive_groups.size());
	for (auto & plane_group : primitive_groups)	{
		//! get sharps in bbox
		stocker::Polyline3d sharps_in_bbox; 
		{
			ccBBox box = plane_group->getBB_recursive();
			for (stocker::Seg3d & seg : all_sharp_lines) {
				CCVector3 p0 = CCVector3(vcgXYZ(seg.P0()));
				CCVector3 p1 = CCVector3(vcgXYZ(seg.P1()));
				if (box.contains(p0) && box.contains(p1)) {
					sharps_in_bbox.push_back(stocker::Seg3d(stocker::parse_xyz(p0), stocker::parse_xyz(p1)));
				}
			}
		}

		stocker::Polyline3d unassigned_sharps; 

		//! assign to each plane
		{
			ccHObject::Container plane_container = GetEnabledObjFromGroup(plane_group, CC_TYPES::PLANE);
			set<size_t>assigned_index;
			for (auto & planeObj : plane_container) {
				ccPlane* ccPlane = ccHObjectCaster::ToPlane(planeObj);
				if (!ccPlane) continue;
				stocker::Contour3d cur_plane_points = GetPointsFromCloud3d(planeObj->getParent());
				if (cur_plane_points.size() < 3) { continue; }
				stocker::PlaneUnit plane = stocker::FormPlaneUnit(cur_plane_points, "temp", true);
				stocker::Polyline3d cur_plane_sharps;
				vector<int> index_find;
				stocker::FindSegmentsInPlane(plane, sharps_in_bbox, index_find, s_assign_image_line_distance);
				assigned_index.insert(index_find.begin(), index_find.end());

				for (auto & index : index_find) {
					cur_plane_sharps.push_back(sharps_in_bbox[index]);
				}
				ccHObject* cur_sharps_obj = AddSegmentsAsChildVertices(planeObj->getParent(), cur_plane_sharps, BDDB_IMAGELINE_PREFIX, ccColor::orange);
				if (cur_sharps_obj) { SetGlobalShiftAndScale(cur_sharps_obj); addToDB(cur_sharps_obj, planeObj->getDBSourceType()); }
			}
			for (size_t i = 0; i < sharps_in_bbox.size(); i++) {
				if (assigned_index.find(i) == assigned_index.end()) {
					unassigned_sharps.push_back(sharps_in_bbox[i]);
				}
			}
		}		

		//! add unassigned sharps in bbox to .todo-TodoLine
		{
			ccPointCloud* existing_todo_line = nullptr;
			if (baseObj) existing_todo_line = baseObj->GetTodoLine(GetBaseName(plane_group->getName()));
			
			if (existing_todo_line)	{
				AddSegmentsToVertices(existing_todo_line, unassigned_sharps, BDDB_TODOLINE_PREFIX, ccColor::darkGrey);
			}
			else {
				AddSegmentsAsChildVertices(plane_group->getParent(), unassigned_sharps, BDDB_TODOLINE_PREFIX, ccColor::darkGrey);
			}
		}

		ProgStep()
	}
	ProgEnd
	refreshAll();
	UpdateUI();
}

void MainWindow::doActionBDPrimPlaneFromSharp()
{
	if (!haveSelection()) {
		return;
	}
	ccHObject* unass_sharp_obj = m_selectedEntities.front();
	ccHObject::Container entity_sharps = GetEnabledObjFromGroup(unass_sharp_obj, CC_TYPES::POLY_LINE);
	stocker::Polyline3d unassigned_sharps = GetPolylineFromEntities(entity_sharps);

	vector<vcg::Plane3d> planes; stocker::IntGroup indices;
	stocker::PlaneSegmentationfromLines(unassigned_sharps, planes, indices, 1);
	set<int> assigned_index;

	BDBaseHObject* baseObj = GetRootBDBase(unass_sharp_obj);
	StBuilding* buildingObj = GetParentBuilding(unass_sharp_obj);
	if (!buildingObj || !baseObj) {
		ccLog::Error("no parent building or project found!");
		return;
	}
	StPrimGroup* prim_group = baseObj->GetPrimitiveGroup(buildingObj->getName()); assert(prim_group);
	int biggest = GetMaxNumberExcludeChildPrefix(prim_group, BDDB_PLANESEG_PREFIX); biggest++;
	for (size_t i = 0; i < indices.size(); i++) {
		if (indices[i].size() < 20) {
			continue;
		}
		stocker::Polyline3d lines;
		for (auto & ln_index : indices[i]) {
			stocker::Seg3d seg = unassigned_sharps[ln_index];
			lines.push_back(seg);
			assigned_index.insert(ln_index);
		}
		ccPointCloud* plane_cloud = AddSegmentsAsPlane(lines, "Deduced", ccColor::Generator::Random());
		plane_cloud->setName(BDDB_PLANESEG_PREFIX + biggest++);
		prim_group->addChild(plane_cloud);
	}
	if (prim_group) {
		addToDB(prim_group, baseObj->getDBSourceType());
	}

// 	stocker::Polyline3d remained_unassigned;
// 	for (int i = 0; i < unassigned_sharps.size(); i++) {
// 		if (assigned_index.find(i) == assigned_index.end())	{
// 			remained_unassigned.push_back(unassigned_sharps[i]);
// 		}
// 	}
// 	ccHObject* group_2 = new ccHObject(unass_sharp_obj->getName() + "-unassigned");
// 	AddSegmentsAsChildVertices(group_2, remained_unassigned, "ImageSharp", ccColor::black);
// 	addToDB(group_2);
// 	unass_sharp_obj->setEnabled(false);

	refreshAll();
	UpdateUI();
}

void MainWindow::doActionBDPrimBoundary()
{
	if (!haveSelection()) return;

// 	QStringList methods;
// 	methods.append("image based");
// 	methods.append("boundary points");
// 	bool ok;
//	QString used_method = QInputDialog::getItem(this, "Boundary extraction", "method", methods, 0, false, &ok);
//	if (!ok) return;

 	double distance(0.5), minpts(10), radius(3);
// 	if (used_method == "boundary points") 
// 	{
// 		ccAskThreeDoubleValuesDlg paraDlg("distance", "minpts", "radius", 0, 1.0e12, distance, minpts, radius, 6, "ransac", this);
// 		if (!paraDlg.exec()) {
// 			return;
// 		}
// 	}
	ccHObject *entity = getSelectedEntities().front();
	// plane or point cloud
	ccHObject::Container plane_container = GetPlaneEntitiesBySelected(entity);

	if (plane_container.empty()) {
		dispToConsole("[BDRecon] Please select point cloud / (group of) planes / buildings", ERR_CONSOLE_MESSAGE);
		return;
	}
	ProgStartNorm("Boundary Extraction", plane_container.size());
	for (auto & planeObj : plane_container) {
		ccHObject* boundary = nullptr;		
		try {
			boundary = CalcPlaneBoundary(planeObj, distance, minpts, radius);
		}
		catch (std::runtime_error& e) {
			dispToConsole(e.what(), ERR_CONSOLE_MESSAGE);
			return;
		}
		if (boundary) { addToDB(boundary, planeObj->getDBSourceType(), false, false); }
		ProgStep()
	}
	ProgEnd
	refreshAll();
	UpdateUI();
}

void MainWindow::doActionBDPrimOutline()
{
	bool ok = true;
	double alpha = QInputDialog::getDouble(this, "Input Dialog", "Please input alpha value", 2.0, 0.0, 999999.0, 6, &ok);
	if (!ok) return;

	if (!haveSelection()) return;
	ccHObject *entity = getSelectedEntities().front();
	ccHObject::Container plane_container = GetPlaneEntitiesBySelected(entity);

	if (plane_container.empty()) {
		dispToConsole("[BDRecon] Please select point cloud / (group of) planes / buildings", ERR_CONSOLE_MESSAGE);
		return;
	}

	ProgStartNorm("Outline Calculation", plane_container.size())
	
	for (auto & planeObj : plane_container) {
		try	{
			ccHObject* outline = CalcPlaneOutlines(planeObj, alpha);
			if (outline) { SetGlobalShiftAndScale(outline); addToDB(outline, planeObj->getDBSourceType(), false, false); }
		}
		catch (const std::runtime_error& e)	{
			dispToConsole(e.what(), ERR_CONSOLE_MESSAGE);
			return;
		}
		ProgStep()
	}
	ProgEnd

	refreshAll();
	UpdateUI();
}

void MainWindow::doActionBDPrimPlaneFrame()
{
	if (!haveSelection()) return;

	QStringList methods;
	methods.append("optimization");
	methods.append("linegrow");
	methods.append("houghline");
	bool ok;
	QString used_method = QInputDialog::getItem(this, "Boundary extraction", "method", methods, 0, false, &ok);
	if (!ok) return;

	ccHObject *entity = getSelectedEntities().front();
	ccHObject::Container plane_container = GetPlaneEntitiesBySelected(entity);

	if (plane_container.empty()) {
		dispToConsole("[BDRecon] Please select  (group of) planes / buildings", ERR_CONSOLE_MESSAGE);
		return;
	}

	ProgStartNorm("Frame Optimization", plane_container.size())
	
	for (auto & planeObj : plane_container) {
		try	{
			if (used_method == "linegrow") {
				double alpha(0.5), intersection(1), minpts(10);
				ccAskThreeDoubleValuesDlg paraDlg("alpha", "intersection", "minpts", 0, 1.0e12, alpha, intersection, minpts, 6, "line grow", this);
				if (!paraDlg.exec()) {
					return;
				}
				ccHObject* frame = PlaneFrameLineGrow(planeObj, alpha, intersection, minpts);
				if (frame) { SetGlobalShiftAndScale(frame); addToDB(frame, planeObj->getDBSourceType(), false, false); }
			}
			else if (used_method == "optimization") {
				//! dialog
				stocker::FrameOption option;
				option.buffer_size = 1;
				option.snap_epsilon = 1;
				option.candidate_buffer_h = 1;
				option.candidate_buffer_v = 2;
				option.lamda_coverage = 0.5;
				option.lamda_sharpness = 0.5;
				option.lamda_smooth_term = 10;
				option.bdransac_epsilon = 0.5;
				option.line_ptmin = 10;
				option.bdransac_radius = 3;

				ccHObject* frame = PlaneFrameOptimization(planeObj, option);
				if (frame) { SetGlobalShiftAndScale(frame); addToDB(frame, planeObj->getDBSourceType(), false, false); }
			}
		}
		catch (const std::runtime_error& e)	{
			dispToConsole(e.what(), ERR_CONSOLE_MESSAGE);
			return;
		}
		ProgStep()
	}
	ProgEnd

	refreshAll();
	UpdateUI();
}

void MainWindow::doActionBDPrimMergePlane()
{
	if (!haveSelection()) return;
	
	if (m_selectedEntities.size() < 2) {
		dispToConsole("[BDRecon] Merge Plane - please select at least two planes", ERR_CONSOLE_MESSAGE);
		return;
	}
	ccPointCloud* point_cloud = new ccPointCloud("Plane");
	ccColor::Rgb col = ccColor::Generator::Random();
	
	ccHObject* first_cloud = GetPlaneCloud(m_selectedEntities.front());
	if (!first_cloud) return;
	ccHObject* prim_group = first_cloud->getParent();
	if (!prim_group) return;
	for (auto & ent_pc : m_selectedEntities) {
		ccHObject* plane_cloud = GetPlaneCloud(ent_pc);
		if (!plane_cloud) return;		
		
		if (plane_cloud->getParent() != prim_group) return;
		
		ccPointCloud* pc = ccHObjectCaster::ToPointCloud(plane_cloud);
		if (!pc) return;

		*point_cloud += pc;
		if (pc->hasColors()) {
			col = pc->getPointColor(0);
		}
		pc->setName(pc->getName() + "-del");
		pc->setEnabled(false);		
	}
	point_cloud->setRGBColor(col);
	point_cloud->showColors(true);
	point_cloud->setDisplay(m_selectedEntities[0]->getDisplay());
	FitPlaneAndAddChild(point_cloud);
	
	prim_group->addChild(point_cloud);
	int biggest = GetMaxNumberExcludeChildPrefix(prim_group, BDDB_PLANESEG_PREFIX);
	point_cloud->setName(BDDB_PLANESEG_PREFIX + QString::number(biggest + 1));
	
	addToDB(point_cloud, m_selectedEntities[0]->getDBSourceType(), false, false);
	refreshAll();
	UpdateUI();
}

void MainWindow::doActionBDPrimSplitPlane()
{
	ccGLWindow* win = getActiveGLWindow();
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

	ccHObject* first_entity = getSelectedEntities().front();
	bool planeseg_mode = getSelectedEntities().size() == 1 && GetPlaneFromCloud(first_entity);
	if (!planeseg_mode) {
		dispToConsole("Please select the point cloud of a plane", ERR_CONSOLE_MESSAGE);
		return;
	}
	m_gsTool->setSegmentMode(ccGraphicalSegmentationTool::SEGMENT_PLANE_CREATE);

	for (ccHObject *entity : getSelectedEntities())
	{
		if (entity->isKindOf(CC_TYPES::POINT_CLOUD) || entity->isKindOf(CC_TYPES::MESH)) {
			m_gsTool->addEntity(entity);
		}
	}

	if (m_gsTool->getNumberOfValidEntities() == 0)
	{
		ccConsole::Error("No segmentable entity in active window!");
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

void MainWindow::doActionBDPrimCreateGround()
{
	if (!haveSelection()) return;
	bool ok = true;
	double sample = QInputDialog::getDouble(this, "Input Dialog", "Please input sampling distance", 1.5, 0.0, 999999.0, 1, &ok);
	if (!ok) return;
	ccHObject* entity = getSelectedEntities().front(); assert(entity);
	
	BDBaseHObject* baseObj = GetRootBDBase(entity);

	std::vector<StPrimGroup*> prim_groups;
	if (entity->isA(CC_TYPES::ST_PRIMGROUP)) {
		StPrimGroup* plane_group = ccHObjectCaster::ToStPrimGroup(entity);
		if (plane_group) {
			prim_groups.push_back(plane_group);
		}
	}
	else {		
		if (!baseObj) return;
		ccHObject::Container building_group;
		if (entity->isA(CC_TYPES::ST_BUILDING)) {
			building_group.push_back(entity);
		}
		else if (isBuildingProject(entity)) {
			building_group = GetEnabledObjFromGroup(entity, CC_TYPES::ST_BUILDING, true, false);
		}
		for (ccHObject* bd : building_group) {
			StPrimGroup* primGroup = baseObj->GetPrimitiveGroup(bd->getName());
			if (!primGroup->isEnabled()) { continue; }
			prim_groups.push_back(primGroup);
		}
	}
	ProgStartNorm("Create Ground Plane", prim_groups.size());
	for (StPrimGroup* primGroup : prim_groups) {
		double ground_height;
		stocker::Polyline2d ground_convex;
		if (baseObj) {
			stocker::BuildUnit bd_unit = baseObj->GetBuildingUnit(GetBaseName(primGroup->getName()).toStdString());
			ground_height = bd_unit.ground_height;
			stocker::Contour2d c_h_local;
			for (auto & pt : bd_unit.convex_hull_xy) {
				c_h_local.push_back(ToVec2d(baseObj->ToLocal(ToVec3d(pt))));
			}
			ground_convex = stocker::MakeLoopPolylinefromContour(c_h_local);
			
		}
		else {
			stocker::Contour3d planes_points = GetPointsFromCloud3d(primGroup);
			stocker::Contour3d ground_contour_3d = stocker::CalcBuildingGround(planes_points, 0.5, 3, 3);			
			ground_height = ground_contour_3d.front().Z();
			ground_convex = stocker::MakeLoopPolylinefromContour(stocker::ToContour2d(ground_contour_3d));
		}
		ccBBox box = entity->getBB_recursive();

		stocker::Vec3d box_min = stocker::parse_xyz(box.minCorner());
		stocker::Vec3d box_max = stocker::parse_xyz(box.maxCorner());
		stocker::BBox2d box_2d = stocker::BBox2d(stocker::ToVec2d(box_min), stocker::ToVec2d(box_max));
		stocker::Contour2d pts_2d = stocker::PointSampleInBox2d(box_2d, sample);

		ccPointCloud* plane_cloud = new ccPointCloud("Ground");
		ccColor::Rgb col = ccColor::Generator::Random();
		plane_cloud->setRGBColor(col);
		plane_cloud->showColors(true);

		entity->addChild(plane_cloud);
		for (auto & pt : pts_2d) {
			if (vcg::PointInsidePolygon(pt, ground_convex))	{
				plane_cloud->addPoint(CCVector3(pt.X(), pt.Y(), ground_height));
			}
		}
		ccHObject* plane = nullptr;
		double rms = 0; std::vector<CCVector3> c_hull;
		ccPlane* pPlane = ccPlane::Fit(plane_cloud, &rms, &c_hull);
		if (pPlane) {
			plane = static_cast<ccHObject*>(pPlane);
			pPlane->setColor(col);
			pPlane->enableStippling(true);
		}
		if (plane) {
			plane->setName("Plane");
			plane->applyGLTransformation_recursive();
			plane->showColors(true);
			plane->setVisible(true);

			plane_cloud->addChild(plane);
			plane->setDisplay(plane_cloud->getDisplay());
			plane->prepareDisplayForRefresh_recursive();
		}
		addToDB(plane_cloud, entity->getDBSourceType(), false, false);
		ProgStep()
	}	
	ProgEnd

	refreshAll();
	UpdateUI();
}

static double s_last_shrink_alpha = 2.0;
static double s_last_shrink_distance = 0.5;
void MainWindow::doActionBDPrimShrinkPlane()
{
	if (!haveSelection()) {
		return;
	}
	ccAskTwoDoubleValuesDlg paraDlg("alpha value", "distance", 0, 1.0e12, s_last_shrink_alpha, s_last_shrink_distance, 6, "refit plane", this);
	if (!paraDlg.exec()) {
		return;
	}
	s_last_shrink_alpha = paraDlg.doubleSpinBox1->value();
	s_last_shrink_distance = paraDlg.doubleSpinBox2->value();

	ccHObject *entity = getSelectedEntities().front();
	ccHObject::Container plane_container = GetPlaneEntitiesBySelected(entity);

	if (plane_container.empty()) {
		dispToConsole("[BDRecon] Please select  (group of) planes / buildings", ERR_CONSOLE_MESSAGE);
		return;
	}

	ProgStartNorm("Shrink Planes", plane_container.size())
	
	for (auto & planeObj : plane_container) {
		try {
			ShrinkPlaneToOutline(planeObj, s_last_shrink_alpha, s_last_shrink_distance);
		}
		catch (std::runtime_error& e) {
			dispToConsole(e.what(), ERR_CONSOLE_MESSAGE);
			return;			
		}
		ProgStep()
	}
	ProgEnd

	refreshAll();
	UpdateUI();
}

void MainWindow::doActionBDPrimPointProjection()
{
	if (!haveSelection()) return;
	ccHObject *entity = getSelectedEntities().front();
	ccHObject::Container plane_container = GetPlaneEntitiesBySelected(entity);

	for (ccHObject* plEnt : plane_container) {
		ccPointCloud* pcObj = GetPlaneCloud(plEnt);
		ccPlane* planeObj = ccHObjectCaster::ToPlane(plEnt);

		vcg::Plane3d plane = GetVcgPlane(plEnt);
		for (size_t i = 0; i < pcObj->size(); i++) {
			CCVector3* P = const_cast<CCVector3*>(pcObj->getPoint(i));
			(*P) = CCVector3(vcgXYZ(plane.Projection(vcg::Point3d(P->x, P->y, P->z))));
		}
		pcObj->refreshBB();
	}
	refreshAll();
	UpdateUI();
}

void MainWindow::doActionBDPlaneFromPoints()
{
	ccGLWindow* win = getActiveGLWindow();
	if (!win)
		return;

	if (!haveSelection() || !getSelectedEntities().front()->isA(CC_TYPES::POINT_CLOUD))	//! only parse todo points
		return;

	if (!m_gsTool)
	{
		m_gsTool = new ccGraphicalSegmentationTool(this);
		connect(m_gsTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateSegmentationMode);

		registerOverlayDialog(m_gsTool, Qt::TopRightCorner);
	}

	m_gsTool->linkWith(win);

	m_gsTool->setSegmentMode(ccGraphicalSegmentationTool::SEGMENT_PLANE_SPLIT);

	for (ccHObject *entity : getSelectedEntities())
	{
		if (entity->isKindOf(CC_TYPES::POINT_CLOUD) || entity->isKindOf(CC_TYPES::MESH)) {
			m_gsTool->addEntity(entity);
		}
	}

	if (m_gsTool->getNumberOfValidEntities() == 0)
	{
		ccConsole::Error("No segmentable entity in active window!");
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

void MainWindow::doActionBDPlaneFromPolygon()
{
	if (!haveSelection()) {
		return;
	}
	ccHObject* prim_group = getSelectedEntities().front();
	if (!prim_group->isA(CC_TYPES::ST_PRIMGROUP)) { // TODO: set it automatically, but now, select manually
		dispToConsole("please select a primitive group", ERR_CONSOLE_MESSAGE);
		return;
	}
	// TODO: get what is available now, return their display status, sometimes octree picking changes the color

	ccGLWindow* win = getActiveGLWindow();
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
	m_tplTool->setTraceMode(1, prim_group);

	freezeUI(true);
	m_UI->toolBarView->setDisabled(false);

	//we disable all other windows
	disableAllBut(win);

	if (!m_tplTool->start())
		deactivateTracePolylineMode(false);
	else
		updateOverlayDialogsPlacement();
}

void MainWindow::doActionBDPlaneDeduction()
{
	ccHObject* point_cloud = askUserToSelect(CC_TYPES::POINT_CLOUD, 0, "please select the origin point cloud"); if (!point_cloud)return;
	//! select two groups, one for plane, one for line
	//! first, select the sharp line group
	ccHObject* sharp_group = askUserToSelect(CC_TYPES::HIERARCHY_OBJECT, 0, "please select the unassigned segments group"); if (!sharp_group)return;
	ccHObject::Container sharp_container = GetEnabledObjFromGroup(sharp_group, CC_TYPES::POLY_LINE);

	//! second, select the plane group
	ccHObject* plane_group = askUserToSelect(CC_TYPES::HIERARCHY_OBJECT, 0, "please select the main planes group"); if (!plane_group)return;
	ccHObject::Container plane_container = GetEnabledObjFromGroup(plane_group, CC_TYPES::PLANE);
	
	bool balreadyseged = false;
	ccHObject* plane_group_add = askUserToSelect(CC_TYPES::HIERARCHY_OBJECT, 0, "please select the segmented planes(if already segmented)"); 
	if (plane_group_add) {
		balreadyseged = true;
	}
	
	//////////////////////////////////////////////////////////////////////////
	vector<vcg::Plane3d>planes;
	vector<stocker::Contour3d> planes_points;
	vector<stocker::Polyline3d> planes_sharps;
	stocker::Polyline3d unassigned_sharp_lines;
	vector<stocker::Point_Normal> unassigned_points;
	{
		stocker::Contour3d used_points;
		vector<stocker::Point_Normal> all_points;
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(point_cloud);
		if (cloud) {
			for (unsigned i = 0; i < cloud->size(); i++) {
				CCVector3 pt = *cloud->getPoint(i);
				CCVector3 n = cloud->getPointNormal(i);
				all_points.push_back({ {pt.x, pt.y, pt.z}, {n.x, n.y, n.z} });
			}
		}

		// unassigned sharps
		unassigned_sharp_lines = GetPolylineFromEntities(sharp_container);

		// sharp lines	
		for (auto & plane_entity : plane_container) {
			ccHObject* Plane_Cloud = plane_entity->getParent();
			if (!Plane_Cloud->isEnabled()) { continue; }
//			string plane_name = Plane_Cloud->getName().toStdString();			
// 			// get plane
// 			stocker::PlaneUnit plane_unit = stocker::FormPlaneUnit(GetPointsFromCloud3d(Plane_Cloud), plane_name, true);
// 			pool_plane.push_back(plane_unit);
// 			// get boundary lines
// 			ccHObject::Container boundary_container;
// 			Plane_Cloud->filterChildrenByName(boundary_container, false, "Boundary", true);	
// 			for (auto & bdry_ent : boundary_container) {
// 				ccHObject::Container polyline_container = GetEnabledObjFromGroup(bdry_ent, CC_TYPES::POLY_LINE);
// 				stocker::Polyline3d bdry_lines = GetPolylineFromEntities(polyline_container);
// 				for (auto & ln : bdry_lines) {
// 					pool_line.push_back({ ln, plane_name });
// 				}
// 			}

			ccPlane* ccPlane = ccHObjectCaster::ToPlane(plane_entity);
			if (!ccPlane) continue;

			CCVector3 N; float constVal;
			ccPlane->getEquation(N, constVal);

			vcg::Plane3d vcgPlane;
			vcgPlane.SetDirection({ N.x, N.y, N.z });
			vcgPlane.SetOffset(constVal);

			planes.push_back(vcgPlane);

			stocker::Contour3d cur_plane_points = GetPointsFromCloud3d(Plane_Cloud);
			planes_points.push_back(cur_plane_points);
			used_points.insert(used_points.end(), cur_plane_points.begin(), cur_plane_points.end());

			ccHObject::Container sharp_container;
			Plane_Cloud->filterChildrenByName(sharp_container, false, "ImageSharp", true);
			if (sharp_container.empty()) { planes_sharps.push_back(stocker::Polyline3d()); continue; }
			ccHObject::Container sharp_container_ = GetEnabledObjFromGroup(sharp_container.front(), CC_TYPES::POLY_LINE);
			planes_sharps.push_back(GetPolylineFromEntities(sharp_container_));
		}
		unassigned_points = stocker::GetUnassignedPoints(used_points, all_points);
	}

	stocker::PrimitiveProj prim_proj;
	prim_proj.option.outline_minarea = 3;
	prim_proj.option.alpha_value = 5;
	prim_proj.option.boundary_minpoints = 10;
	prim_proj.PreparePlanes(planes, planes_points, planes_sharps);
	prim_proj.PrepareSharps(unassigned_sharp_lines);

	if (balreadyseged) {

		vector<vcg::Plane3d>planes_seged;
		vector<stocker::Polyline3d> planes_seged_sharps;

		ccHObject::Container plane_container_add = GetEnabledObjFromGroup(plane_group_add, CC_TYPES::PLANE);
		for (auto & plane_entity : plane_container_add) {
			ccHObject* Plane_Cloud = plane_entity->getParent();
			if (!Plane_Cloud->isEnabled()) { continue; }

			ccPlane* ccPlane = ccHObjectCaster::ToPlane(plane_entity);
			if (!ccPlane) continue;

			CCVector3 N; float constVal;
			ccPlane->getEquation(N, constVal);

			vcg::Plane3d vcgPlane;
			vcgPlane.SetDirection({ N.x, N.y, N.z });
			vcgPlane.SetOffset(constVal);
			
			ccHObject::Container sharp_container;
			Plane_Cloud->filterChildrenByName(sharp_container, false, "ImageSharp", true);
			if (sharp_container.empty()) {
				continue;
			}
			ccHObject::Container sharp_container_ = GetEnabledObjFromGroup(sharp_container.front(), CC_TYPES::POLY_LINE);
			planes_seged_sharps.push_back(GetPolylineFromEntities(sharp_container_));
			planes_seged.push_back(vcgPlane);
		}
		prim_proj.MergeNewPlanes(planes_seged, planes_seged_sharps, unassigned_points);
		// TODO: get unassigned_sharps_remained the unassigned sharps is already in m_prim
// 		stocker::Polyline3d unassigned_sharps_remained;
// 		for (auto & sp : prim_proj.m_prim.ssharp) {
// 			if (sp->data.line_shape == stocker::LineBorder_sharp &&
// 				sp->data.head == "unassigned") {
// 				unassigned_sharps_remained.push_back(sp->data.seg);
// 			}			
// 		}
		// TODO: add unassigned points and sharps to DB
		// TODO: prim_proj.DeriveJunctionPlane();
	}
	else {
		prim_proj.PreparePoints(unassigned_points);
		prim_proj.ReconstructPlaneDeduction();
	}	
	
	// primitive to DB
	ccHObject* group = new ccHObject(plane_group->getName() + "-deduction");
	for (auto & prim : prim_proj.m_prim.splane) {
		ccPointCloud* plane_cloud = new ccPointCloud(prim->GetName().Str().c_str());
		ccColor::Rgb col = ccColor::Generator::Random();
		plane_cloud->setRGBColor(col);
		plane_cloud->showColors(true);

		group->addChild(plane_cloud);

		//! get plane points
		for (auto & pt : prim->point) {
			plane_cloud->addPoint(CCVector3(vcgXYZ(pt->data.vert)));
		}

		//! get plane sharps
		stocker::Polyline3d sharp_lines;
		for (auto & ln : prim->sharp) {
			if (ln->data.line_shape == stocker::LineBorder_sharp) {
				sharp_lines.push_back(ln->data.seg);
				plane_cloud->addPoint(CCVector3(vcgXYZ(ln->data.seg.P0())));
				plane_cloud->addPoint(CCVector3(vcgXYZ(ln->data.seg.P1())));
			}			
		}
		
		//! add plane
		FitPlaneAndAddChild(plane_cloud);

		AddSegmentsAsChildVertices(plane_cloud, sharp_lines, "ImageSharp", col);		
	}
	addToDB(group, point_cloud->getDBSourceType());
	refreshAll();
}

double s_snap_todo_distance_threshold = 0.01;
void MainWindow::doActionBDPlaneCreate()
{
	if (m_selectedEntities.size() < 2)
		return;
	ccHObject* first_selected = getSelectedEntities().front();
	BDBaseHObject* baseObj = GetRootBDBase(first_selected);
	StBuilding* buildingObj = GetParentBuilding(first_selected);
	if (!buildingObj || !baseObj) {
		ccLog::Error("no parent building or project found!");
		return;
	}
	StPrimGroup* prim_group = baseObj->GetPrimitiveGroup(buildingObj->getName()); assert(prim_group);
	
	ccHObject::Container polylines;
	for (auto & entity : m_selectedEntities) {
		if (entity->isA(CC_TYPES::POLY_LINE) && GetParentBuilding(entity) == buildingObj) {
			polylines.push_back(entity);
		}
	}
	
	if (polylines.size() < 2) {
		return;
	}

	stocker::Polyline3d lines_pool = GetPolylineFromEntities(polylines);

	ccPointCloud* plane_cloud = AddSegmentsAsPlane(lines_pool, BDDB_DEDUCED_PREFIX, ccColor::Generator::Random());
	if (!plane_cloud) { dispToConsole("cannot add segments as plane!", ERR_CONSOLE_MESSAGE); return; }
	
	int biggest = GetMaxNumberExcludeChildPrefix(prim_group, BDDB_PLANESEG_PREFIX);
	plane_cloud->setName(BDDB_PLANESEG_PREFIX + QString::number(biggest + 1));

	//! retrieve plane cloud from todo points
	ccPointCloud* todo_cloud = baseObj->GetTodoPoint(buildingObj->getName());
	assert(todo_cloud);
	if (!todo_cloud) { delete plane_cloud; plane_cloud = nullptr; return; }

	try	{
		bool ok = true;
		double input = QInputDialog::getDouble(this, "Input Dialog", "Please input distance threshold", s_snap_todo_distance_threshold, 0.0, 999999.0, 3, &ok);
		if (!ok) return;
		s_snap_todo_distance_threshold = input;

		RetrieveAssignedPoints(todo_cloud, plane_cloud, s_snap_todo_distance_threshold);
	}
	catch (const std::runtime_error& e)	{
		dispToConsole(e.what(), ERR_CONSOLE_MESSAGE);
		delete plane_cloud;
		plane_cloud = nullptr;
		return;
	}

	SetGlobalShiftAndScale(plane_cloud);
	plane_cloud->setDisplay_recursive(prim_group->getDisplay());
	prim_group->addChild(plane_cloud);	
	addToDB(plane_cloud, prim_group->getDBSourceType(), false, false);
	refreshAll();
}

static double s_last_polyfit_datafit = 0.4;
static double s_last_polyfit_coverage = 0.4;
static double s_last_polyfit_complexity = 0.2;
void MainWindow::doActionBDPolyFit()
{	
	bool ok = true;
	double line_sample = QInputDialog::getDouble(this, "Input Dialog", "Please input line sampling distance", 1.5, -1, 999999.0, 1, &ok);
	bool b_add_line_sample = false;
	if (ok && line_sample > 0) b_add_line_sample = true;

	ccAskThreeDoubleValuesDlg paraDlg("data fitting", "coverage", "complexity", 0, 1.0e12, s_last_polyfit_datafit, s_last_polyfit_coverage, s_last_polyfit_complexity, 6, "PolyFit parameters", this);
	if (!paraDlg.exec()) {
		return;
	}
	s_last_polyfit_datafit = paraDlg.doubleSpinBox1->value();
	s_last_polyfit_coverage = paraDlg.doubleSpinBox2->value();
	s_last_polyfit_complexity = paraDlg.doubleSpinBox3->value();

	ccHObject* entity = getSelectedEntities().front(); if (!entity) return;
	StPrimGroup* primgroupObj = ccHObjectCaster::ToStPrimGroup(entity);
	if (!primgroupObj) { return; }
	stocker::PolyFitRecon polyfit_recon;
	ccHObject::Container plane_container = primgroupObj->getValidPlanes();

	stocker::PrimitiveProj prim_proj;
	{
		vector<vcg::Plane3d> planes;
		vector<stocker::Contour3d> planes_points;
		vector<stocker::Polyline3d> planes_sharps; 
		for (auto & plane_entity : plane_container) {
			ccHObject* Plane_Cloud = plane_entity->getParent();

			ccPlane* ccPlane = ccHObjectCaster::ToPlane(plane_entity);
			if (!ccPlane) continue;

			CCVector3 N; float constVal;
			ccPlane->getEquation(N, constVal);

			vcg::Plane3d vcgPlane;
			vcgPlane.SetDirection({ N.x, N.y, N.z });
			vcgPlane.SetOffset(constVal);

			planes.push_back(vcgPlane);

			stocker::Contour3d cur_plane_points = GetPointsFromCloud3d(Plane_Cloud);
			planes_points.push_back(cur_plane_points);

			ccHObject::Container sharp_container;
			Plane_Cloud->filterChildrenByName(sharp_container, false, "ImageSharp", true);
			if (sharp_container.empty()) { planes_sharps.push_back(stocker::Polyline3d()); }
			else {
				ccHObject::Container sharp_container_ = GetEnabledObjFromGroup(sharp_container.front(), CC_TYPES::POLY_LINE);
				stocker::Polyline3d cur_plane_sharps = GetPolylineFromEntities(sharp_container_);
				planes_sharps.push_back(cur_plane_sharps);

				if (b_add_line_sample) {
					for (auto & ln : cur_plane_sharps) {
						double _interval = line_sample / ln.Length();
						int i(0);
						do {
							if (i * _interval > 1) break;
							planes_points.back().push_back(ln.Lerp(i * _interval));
							i++;
						} while (1);
					}
				}
			}			
		}
		prim_proj.PreparePlanes(planes, planes_points, planes_sharps);
	}
	polyfit_recon.SetLambda(s_last_polyfit_datafit, s_last_polyfit_coverage, s_last_polyfit_complexity);
	polyfit_recon.GenerateHypothesis(prim_proj.m_prim, "hypothesis.ply", "result.ply");

	QStringList files;
	files.append("result.ply.obj");
	addToDB(files, entity->getDBSourceType());
	refreshAll();
	UpdateUI();
}

#include "polyfit/method/hypothesis_generator.h"
void ParsePolyFitPara(PolyFitObj* poly, bdrPolyFitDlg* pdlg)
{
	poly->strict_intersection = pdlg->PolyfitStrictCheckBox->isChecked();
	poly->auto_filter = pdlg->PolyfitAutoFilterCheckBox->isChecked();
	poly->snap_intersection = pdlg->PolyfitSnapSpinBox->value();
	poly->use_confidence = pdlg->PolyfitConfiCheckBox->isChecked();
	poly->data_fitting = pdlg->PolyfitdoubleSpinBox1->value();
	poly->model_coverage = pdlg->PolyfitdoubleSpinBox2->value();
	poly->model_complexity = pdlg->PolyfitdoubleSpinBox3->value();
}
//! generate hypothesis
void MainWindow::doActionBDPolyFitHypothesis()
{
	if (!haveSelection()) return; 	
	ccHObject* entity = getSelectedEntities().front();		
	if (!entity->isGroup()) return;

	BDBaseHObject* baseObj = GetRootBDBase(entity);
	if (!baseObj) {
		dispToConsole("[BDRecon] PolyFit - Please open the main project file", ERR_CONSOLE_MESSAGE);
		return;
	}
	ccHObject* primitiveObj = nullptr;
	if (entity->getName().endsWith(BDDB_PRIMITIVE_SUFFIX))
		primitiveObj = entity;
	else {
		primitiveObj = baseObj->GetPrimitiveGroup(GetBaseName(entity->getName()));
	}

	if (!primitiveObj) {
		dispToConsole("[BDRecon] Please select a group contains primitives", ERR_CONSOLE_MESSAGE);
		return; 
	}
	
	if (polyfit_obj) {
		polyfit_obj->clear();
	}
	else {
		polyfit_obj = new PolyFitObj();
	}

	if (!m_pbdrpfDlg) {
		m_pbdrpfDlg = new bdrPolyFitDlg(this);
		if (!m_pbdrpfDlg->exec()) {
			return;
		}
	}
	ParsePolyFitPara(polyfit_obj, m_pbdrpfDlg);
	
	ccHObject* hypoObj = nullptr;
	try	{
		ProgStart("Hypothesis Generation")

		hypoObj = PolyfitGenerateHypothesis(primitiveObj, polyfit_obj);

		ProgEnd
	}
	catch (std::runtime_error& e) {
		dispToConsole(QString("[BDRecon] - hypothesis generated error: ") + e.what(), ERR_CONSOLE_MESSAGE);
		return;
	}
	
	if (hypoObj) {
		polyfit_obj->status = PolyFitObj::STT_hypomesh;
		polyfit_obj->building_name = GetBaseName(primitiveObj->getName()).toStdString();
		SetGlobalShiftAndScale(hypoObj);
		addToDB(hypoObj, entity->getDBSourceType());
	}
	else {
		polyfit_obj->status = PolyFitObj::STT_prepared;
		return;
	}

	refreshAll();
	UpdateUI();

	QMessageBox::StandardButton rb = 
	QMessageBox::question(this, "Compute Confidence?", "Compute Confidence Right Now", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
	if (rb == QMessageBox::No)
		return;

	try	{
		ProgStart("Confidence Calculation")

		PolyfitComputeConfidence(hypoObj, polyfit_obj);
		polyfit_obj->status = PolyFitObj::STT_confidence;

		ProgEnd
	}
	catch (std::runtime_error& e) {
		dispToConsole(QString("[BDRecon] - confidence calculated error: ") + e.what(), ERR_CONSOLE_MESSAGE);
		return;
	}
	
	refreshAll();
	UpdateUI();
}

void MainWindow::doActionBDPolyFitConfidence()
{
	if (!haveSelection()) return;
	ccHObject* entity = getSelectedEntities().front();

	if (!entity->isGroup()) return;

	ccHObject::Container prmitive_groups;
	if (entity->getName().endsWith(BDDB_POLYFITHYPO_SUFFIX))
		prmitive_groups.push_back(entity);
	else
		entity->filterChildrenByName(prmitive_groups, true, BDDB_POLYFITHYPO_SUFFIX, false);

	if (prmitive_groups.empty()) return;

	BDBaseHObject* baseObj = GetRootBDBase(entity);
	if (!baseObj) {
		dispToConsole("[BDRecon] PolyFit - Please open the main project file", ERR_CONSOLE_MESSAGE);
		return;
	}
	ccHObject* HypoObj = prmitive_groups.front();
	if (prmitive_groups.size() > 1) {
		ccLog::Warning("[BDRecon] - only the first primitive is processed");
	}

	if (GetBaseName(HypoObj->getName()).toStdString() != polyfit_obj->building_name) {
		dispToConsole("[BDRecon] PolyFit - Please generate hypothesis firstly", ERR_CONSOLE_MESSAGE);
		return;
	}

	try {
		ParsePolyFitPara(polyfit_obj, m_pbdrpfDlg);

		if (polyfit_obj->status < PolyFitObj::STT_hypomesh) {
			dispToConsole("[BDRecon] Please generate hypothesis firstly", ERR_CONSOLE_MESSAGE);
			return;
		}
		else {
			ProgStart("Confidence Calculation")

			PolyfitComputeConfidence(HypoObj, polyfit_obj);
			polyfit_obj->status = PolyFitObj::STT_confidence;

			ProgEnd
		}	
	}
	catch (std::runtime_error& e) {
		dispToConsole(QString("[BDRecon] - confidence calculated error: ") + e.what(), ERR_CONSOLE_MESSAGE);
		return;
	}
	
	refreshAll();
	UpdateUI();
}
//! face selection
void MainWindow::doActionBDPolyFitSelection()
{
	if (!haveSelection()) return;
	ccHObject* entity = getSelectedEntities().front();

	if (!entity->isGroup()) return;

	ccHObject::Container prmitive_groups;
	if (entity->getName().endsWith(BDDB_POLYFITHYPO_SUFFIX))
		prmitive_groups.push_back(entity);
	else
		entity->filterChildrenByName(prmitive_groups, true, BDDB_POLYFITHYPO_SUFFIX, false);

	if (prmitive_groups.empty()) return;

	BDBaseHObject* baseObj = GetRootBDBase(entity);
	if (!baseObj) {
		dispToConsole("[BDRecon] PolyFit - Please open the main project file", ERR_CONSOLE_MESSAGE);
		return;
	}
	ccHObject* HypoObj = prmitive_groups.front();
	if (prmitive_groups.size() > 1) {
		ccLog::Warning("[BDRecon] - only the first primitive is processed");
	}

	if (GetBaseName(HypoObj->getName()).toStdString() != polyfit_obj->building_name) {
		dispToConsole("[BDRecon] PolyFit - Please generate hypothesis firstly", ERR_CONSOLE_MESSAGE);
		return;
	}

	try {
		ParsePolyFitPara(polyfit_obj, m_pbdrpfDlg);

		if (polyfit_obj->status < PolyFitObj::STT_confidence) {
			dispToConsole("[BDRecon] PolyFit - no confidence calculated", ERR_CONSOLE_MESSAGE);
			return;
		}
		ProgStart("Building Modelling")

		ccHObject* optmizeObj = PolyfitFaceSelection(HypoObj, polyfit_obj);
		if (optmizeObj)	{
			polyfit_obj->status = PolyFitObj::STT_optimized;
			addToDB(optmizeObj, baseObj->getDBSourceType());
			std::string file_path;
			if (polyfit_obj->OutputResultToObjFile(baseObj, file_path)) {
				QString model_file(file_path.c_str());

				CCVector3d loadCoordinatesShift(vcgXYZ(baseObj->global_shift));
				bool loadCoordinatesTransEnabled = true;
				FileIOFilter::LoadParameters parameters;
				{
					parameters.alwaysDisplayLoadDialog = false;
					parameters.coordinatesShift = &loadCoordinatesShift;
					parameters.coordinatesShiftEnabled = &loadCoordinatesTransEnabled;
					parameters.shiftHandlingMode = ccGlobalShiftManager::DIALOG_IF_NECESSARY;
					parameters.parentWidget = this;
				}
				CC_FILE_ERROR result = CC_FERR_NO_ERROR;

				if (QFileInfo(model_file).exists()) {
					ccHObject* model = FileIOFilter::LoadFromFile(model_file, parameters, result, QString());
					if (!model)	{
						return;
					}
					model->setDisplay_recursive(HypoObj->getDisplay());
					HypoObj->getParent()->addChild(model);
					HypoObj->setEnabled(false);
					//SetGlobalShiftAndScale(model);
					addToDB(model, baseObj->getDBSourceType());
				}
			}
		}

		ProgEnd
	}
	catch (const std::string& e) {
		dispToConsole("[BDRecon] PolyFit - Please open the main project file", ERR_CONSOLE_MESSAGE);
		return;
	}

	refreshAll();
	UpdateUI();
}

void MainWindow::doActionBDPolyFitFacetFilter()
{
 	if (!haveSelection()) return;
 	ccHObject* entity = getSelectedEntities().front();
 	if (!entity->isGroup()) return;
 
 	BDBaseHObject* baseObj = GetRootBDBase(entity);
 	if (!baseObj) {
 		dispToConsole("[BDRecon] LoD3 - Please open the main project file", ERR_CONSOLE_MESSAGE);
 		return;
 	}
 
 	ccHObject* hypothesisObj = nullptr;
 	if (entity->getName().endsWith(BDDB_POLYFITHYPO_SUFFIX))
 		hypothesisObj = entity;
 	else {
 		hypothesisObj = baseObj->GetHypothesisGroup(GetBaseName(entity->getName()));
 	}
 
 	if (!hypothesisObj) {
 		dispToConsole("[BDRecon] LoD3 - Please select a group contains hypothesis", ERR_CONSOLE_MESSAGE);
 		return;
 	}

	ccHObject::Container facetObjs;
	hypothesisObj->filterChildren(facetObjs, true, CC_TYPES::FACET, true, hypothesisObj->getDisplay());

	if (facetObjs.empty()) {
		dispToConsole("[BDRecon] LoD3 - invalid hypothesis", ERR_CONSOLE_MESSAGE);
		return;
 	}	

	ccGLWindow* win = getActiveGLWindow();
	if (!win)
		return;

	if (!m_pbdrffDlg) m_pbdrffDlg = new bdrFacetFilterDlg(/*win, hypothesisObj, */this);
	m_pbdrffDlg->initWith(win, facetObjs);
	m_pbdrffDlg->setModal(false);
	m_pbdrffDlg->setWindowModality(Qt::NonModal);
	
	
	m_pbdrffDlg->show();
}

void MainWindow::doActionBDPolyFitSettings()
{
	if (!m_pbdrpfDlg) m_pbdrpfDlg = new bdrPolyFitDlg(this);
	m_pbdrpfDlg->setModal(false);
	m_pbdrpfDlg->setWindowModality(Qt::NonModal);
	m_pbdrpfDlg->show();
}

#include "builderlod2/lod2parser.h"
void MainWindow::doActionBDFootPrintAuto()
{
	if (!haveSelection()){
		return;
	}

	ccHObject::Container building_entites;
	for (ccHObject* ent : getSelectedEntities()) {
		ccHObject::Container bds = GetBuildingEntitiesBySelected(ent);
		building_entites.insert(building_entites.end(), bds.begin(), bds.end());
	}
	if (building_entites.empty()) {
		ccConsole::Error("No building in selection!");
		return;
	}
	ProgStartNorm("Generate footprints", building_entites.size())
	for (ccHObject* entity : building_entites) {
		StBuilding* building = GetParentBuilding(entity);
		if (!building) {
			ccConsole::Error("No building in selection!");
			return;
		}
		QString building_name(GetBaseName(building->getName()));

		BDBaseHObject* baseObj = GetRootBDBase(entity);
		if (!baseObj) {
			dispToConsole(s_no_project_error, ERR_CONSOLE_MESSAGE);
			return;
		}
		StPrimGroup* prim_group = baseObj->GetPrimitiveGroup(building_name);
		if (!prim_group) {
			dispToConsole("generate primitives first!", ERR_CONSOLE_MESSAGE);
			return;
		}

		try {
			stocker::BuildUnit build_unit = baseObj->GetBuildingUnit(building_name.toStdString());
			ccHObject::Container footprints = GenerateFootPrints(prim_group, build_unit.ground_height);
			for (ccHObject* ft : footprints) {
				if (ft && ft->isA(CC_TYPES::ST_FOOTPRINT)) {
					SetGlobalShiftAndScale(ft);
					ft->setDisplay_recursive(entity->getDisplay());
					addToDB(ft, baseObj->getDBSourceType(), false, false);
				}
			}
		}
		catch (const std::runtime_error& e) {
			dispToConsole(e.what(), ERR_CONSOLE_MESSAGE);
			//return;
		}
		ProgStep()
	}
	ProgEnd
	//doActionBDProjectSave();
	refreshAll();
	UpdateUI();
}

void MainWindow::doActionBDFootPrintManual()
{
	if (!haveSelection()) return;

	ccHObject *entity = getSelectedEntities().front();
	StBuilding* building = GetParentBuilding(entity);
	if (!building) {
		ccConsole::Error("No building in selection!");
		return;
	}
	QString building_name(GetBaseName(building->getName()));
		
	BDBaseHObject* baseObj = GetRootBDBase(entity);
	if (!baseObj) {
		dispToConsole(s_no_project_error, ERR_CONSOLE_MESSAGE);
		return;
	}
	ccHObject::Container viewer_clouds;
	ccHObject* viewer_cloud = baseObj->GetOriginPointCloud(building_name, true);
	if (viewer_cloud) {
		viewer_clouds.push_back(viewer_cloud);
	}
	else {
		StPrimGroup* prim_group = baseObj->GetPrimitiveGroup(building_name);
		if (!prim_group || !prim_group->isEnabled()) { return; }
		ccHObject::Container primObjs = GetEnabledObjFromGroup(prim_group, CC_TYPES::PLANE, true, true);
		for (size_t i = 0; i < primObjs.size(); i++) {
			if (primObjs[i]->getParent() && primObjs[i]->getParent()->isA(CC_TYPES::POINT_CLOUD)) {
				viewer_clouds.push_back(primObjs[i]->getParent());
			}			
		}
	}
	StBlockGroup* block_group = baseObj->GetBlockGroup(building_name);
	
	if (viewer_clouds.empty() || !block_group) {
		dispToConsole("No building cloud in selection!", ERR_CONSOLE_MESSAGE);
		return;
	}
	stocker::BuildUnit _build = baseObj->GetBuildingUnit(building_name.toStdString());
	double ground = _build.ground_height;

	if (!m_seTool)
	{
		m_seTool = new ccSectionExtractionTool(this);
		connect(m_seTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateSectionExtractionMode);
		registerOverlayDialog(m_seTool, Qt::TopRightCorner);
	}

	m_seTool->setExtractMode(false);
	m_seTool->SetDestAndGround(block_group, ground);
	

	//add clouds
	ccGLWindow* firstDisplay = nullptr;
	{		
		for (ccHObject* pc : viewer_clouds)	{
			if (m_seTool->addCloud(static_cast<ccGenericPointCloud*>(pc))) {
				if (!firstDisplay && pc->getDisplay()) {
					firstDisplay = static_cast<ccGLWindow*>(pc->getDisplay());
				}
			}
		}		
	}

	//deselect all entities
	unselectAllInDB();

	ccGLWindow* win = new3DView(false);
	if (!win)
	{
		ccLog::Error("[SectionExtraction] Failed to create dedicated 3D view!");
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

void MainWindow::doActionBDFootPrintPack()
{
	if (!haveSelection()) return;
	ccHObject *entity = getSelectedEntities().front();
	BDBaseHObject* baseObj = GetRootBDBase(entity);
	ccHObject* bd_entity = GetParentBuilding(entity);
	if (!bd_entity) return;

	ProgStart("polygon partition")
	try	{
		if (!PackFootprints(bd_entity)) {
			return;
		}
		addToDB(bd_entity, bd_entity->getDBSourceType());
	}
	catch (const std::exception& e)	{
		dispToConsole(e.what(), ERR_CONSOLE_MESSAGE);
		return;
	}
	ProgEnd

	refreshAll();
	UpdateUI();
}

void MainWindow::doActionBDFootPrintGetPlane()
{
	if (!haveSelection()) {
		return;
	}
	bool bClearExist = QMessageBox::question(this, "Get planes", "Clear existing planes?",
		QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes;

	CCVector3 s_settings;
	ccAskThreeDoubleValuesDlg setDlg("xy-bias", "z-bias", "minpts", -1.0e12, 1.0e12, 1, 1, 30, 4, "Get planes inside the footprint", this);
	setDlg.showCheckbox("Vertical planes", true, "Get vertical planes?");
	if (setDlg.buttonBox->button(QDialogButtonBox::Ok))
		setDlg.buttonBox->button(QDialogButtonBox::Ok)->setFocus();
	if (!setDlg.exec())
		return;
	s_settings.x = setDlg.doubleSpinBox1->value();
	s_settings.y = setDlg.doubleSpinBox2->value();
	s_settings.z = setDlg.doubleSpinBox3->value();
	bool bVertical = setDlg.getCheckboxState();

	ccHObject* select_ent = getSelectedEntities().front();
	ccHObject::Container building_entites = GetBuildingEntitiesBySelected(select_ent);
	if (building_entites.empty()) return;

	BDBaseHObject* baseObj = GetRootBDBase(select_ent);
	ProgStartNorm("Get planes inside footprints", building_entites.size());
	try	{
		for (ccHObject* buildingEntity : building_entites) {
			StBlockGroup* block_group = baseObj->GetBlockGroup(buildingEntity->getName());
			StPrimGroup* prim_group = baseObj->GetPrimitiveGroup(buildingEntity->getName());
			if (block_group && prim_group) {
				ccHObject::Container footprints;
				if (select_ent->isA(CC_TYPES::ST_FOOTPRINT)) {
					footprints.push_back(select_ent);
				}
				else {
					footprints = block_group->getValidFootPrints();
				}
				for (ccHObject* ft : footprints) {
					GetPlanesInsideFootPrint(ft, prim_group, s_settings, bVertical, bClearExist);
				}
			}
			ProgStep()
		}
	}
	catch (const std::exception& e) {
		dispToConsole(e.what(), ERR_CONSOLE_MESSAGE);
	}
	ProgEnd

	refreshAll();
	UpdateUI();
}

void MainWindow::doActionBDMeshToBlock()
{
	LoadMeshAsBlock("D:/1.obj");
	

	if (!haveSelection()) {
		return;
	}
	ccMesh* mesh = ccHObjectCaster::ToMesh(getSelectedEntities().front());
	if (!mesh) { return; }
	StBlock* block = new StBlock();
}

void MainWindow::doActionBDLoD1Generation()
{
	if (!haveSelection()) {
		return;
	}

	ccHObject::Container building_entites;
	for (ccHObject* ent : getSelectedEntities()) {
		ccHObject::Container bds = GetBuildingEntitiesBySelected(ent);
		building_entites.insert(building_entites.end(), bds.begin(), bds.end());
	}
	if (building_entites.empty()) {
		return;
	}
	ProgStartNorm("lod1 generation", building_entites.size())
	for (ccHObject* bd_entity : building_entites) {
		StBuilding* building = ccHObjectCaster::ToStBuilding(bd_entity);
		if (!building) { ProgStep() continue; }

		try {
			ccHObject* bd_model_obj = LoD1FromFootPrint(building);
			if (bd_model_obj) {
				SetGlobalShiftAndScale(bd_model_obj);
				bd_model_obj->setDisplay_recursive(building->getDisplay());
				addToDB(bd_model_obj, building->getDBSourceType(), false, false);
			}
		}
		catch (std::runtime_error& e) {
			dispToConsole("[BDRecon] cannot build lod1 model", ERR_CONSOLE_MESSAGE);
			dispToConsole(e.what(), ERR_CONSOLE_MESSAGE);
			return;
		}
		ProgStep()
	}
	ProgEnd
// 	if (entity->isA(CC_TYPES::ST_FOOTPRINT)) {
// 
// 	}
// 	else {
// 		//! select the building
// 		StBuilding* building = GetParentBuilding(entity);
// 		if (!building) { return; }
// 
// 		try {
// 			ccHObject* bd_model_obj = LoD1FromFootPrint(building);
// 			if (bd_model_obj) {
// 				SetGlobalShiftAndScale(bd_model_obj);
// 				bd_model_obj->setDisplay_recursive(entity->getDisplay());
// 				addToDB(bd_model_obj, entity->getDBSourceType(), false, false);
// 			}
// 		}
// 		catch (std::runtime_error& e) {
// 			dispToConsole("[BDRecon] cannot build lod1 model", ERR_CONSOLE_MESSAGE);
// 			dispToConsole(e.what(), ERR_CONSOLE_MESSAGE);
// 			return;
// 		}
// 	}
	refreshAll();
	UpdateUI();
}

void MainWindow::doActionBDLoD2Generation()
{
	if (!haveSelection()) return;
	
	if (!m_pbdrSettingLoD2Dlg) {
		m_pbdrSettingLoD2Dlg = new bdrSettingLoD2Dlg(this);
		if (!m_pbdrSettingLoD2Dlg->exec()) {
			return;
		}
	}

// 	m_pbdrSettingLoD2Dlg->setModal(false);
// 	m_pbdrSettingLoD2Dlg->setWindowModality(Qt::NonModal);
	//if (!m_pbdrSettingLoD2Dlg->exec()) return;	// TODO: TEMP!!! 20190731

	ccHObject::Container sels = m_selectedEntities;
	//doActionBDFootPrintAuto();	// TODO: TEMP!!!!
	
	ccHObject::Container building_entites;	// could be a footprint or building entities
	if (sels.size() == 1 && sels.front()->isA(CC_TYPES::ST_FOOTPRINT)) {
		building_entites.push_back(sels.front());
	}
	else {
		for (ccHObject* ent : sels) {
			ccHObject::Container bds = GetBuildingEntitiesBySelected(ent);
			building_entites.insert(building_entites.end(), bds.begin(), bds.end());
		}
	}
	bool ask_for_overwrite = true;
	bool overwrite_all = false;
	if (building_entites.empty()) {
		ccConsole::Error("No building in selection!");
		return;
	}

	ProgStartNorm("LoD2 generation", building_entites.size())
	for (ccHObject* bd_entity : building_entites) {
		BDBaseHObject* baseObj = GetRootBDBase(bd_entity);

// 		double height = DBL_MAX;
// 		if (m_pbdrSettingLoD2Dlg->GroundHeightMode() == 2) {
// 			height = m_pbdrSettingLoD2Dlg->UserDefinedGroundHeight();
// 		}
// 		else /*if (m_pbdrSettingLoD2Dlg->GroundHeightMode() == 0)*/ {
// 			height = baseObj->GetBuildingUnit(GetParentBuilding(bd_entity)->getName().toStdString()).ground_height;
// 		}
		if (bd_entity->isA(CC_TYPES::ST_BUILDING)) {
			QString building_name = bd_entity->getName();
			//! check for footprints
			StBlockGroup* blockGroup = baseObj->GetBlockGroup(building_name);
			if (!blockGroup) {
				continue;
			}
			ccHObject::Container fts = GetEnabledObjFromGroup(blockGroup, CC_TYPES::ST_FOOTPRINT, true, false);
			bool found_footprints = !fts.empty();
			bool conduct_get_ft = false;
			if (found_footprints && ask_for_overwrite) {
				//! ask for skip
				QMessageBox::StandardButton button = QMessageBox::question(this,
					"Footprints exist",
					"overwrite the existing footprints? click no to use the existing footprints for lod2 generation",
					QMessageBox::Yes | QMessageBox::YesToAll | QMessageBox::No | QMessageBox::NoToAll,
					QMessageBox::YesToAll);
				if (button == QMessageBox::Yes) {
					ask_for_overwrite = true;
					conduct_get_ft = true;
				}
				else if (button == QMessageBox::No) {
					ask_for_overwrite = true;
					conduct_get_ft = false;
				}
				else if (button == QMessageBox::YesToAll) {
					ask_for_overwrite = false;
					overwrite_all = true;
					conduct_get_ft = true;
				}
				else if (button == QMessageBox::NoToAll) {
					ask_for_overwrite = false;
					overwrite_all = false;
					conduct_get_ft = false;
				}
			}
			else {
				conduct_get_ft = found_footprints ? overwrite_all : true;
			}
			if (conduct_get_ft) {
				blockGroup->removeAllChildren();
				try
				{
					StPrimGroup* prim_group = baseObj->GetPrimitiveGroup(building_name);
					if (!prim_group) {
						dispToConsole("generate primitives first!", ERR_CONSOLE_MESSAGE);
						return;
					}

					try {
						stocker::BuildUnit build_unit = baseObj->GetBuildingUnit(building_name.toStdString());
						ccHObject::Container footprints = GenerateFootPrints(prim_group, build_unit.ground_height);
						for (ccHObject* ft : footprints) {
							if (ft && ft->isA(CC_TYPES::ST_FOOTPRINT)) {
								SetGlobalShiftAndScale(ft);
								ft->setDisplay_recursive(bd_entity->getDisplay());
								addToDB(ft, baseObj->getDBSourceType(), false, false);
							}
						}
					}
					catch (const std::runtime_error& e) {
						dispToConsole(e.what(), ERR_CONSOLE_MESSAGE);
						throw(std::runtime_error(e.what()));
						STOCKER_ERROR_ASSERT(e.what());
						return;
					}
				}
				catch (const std::exception&)
				{
					continue;
				}
				if (m_pbdrSettingLoD2Dlg->PolygonPartitionGroupBox->isChecked()) {
					PackFootprints(bd_entity);
				}
			}
		}
		
		try {
			ccHObject* bd_model_obj = LoD2FromFootPrint(bd_entity);
			if (bd_model_obj) {
				SetGlobalShiftAndScale(bd_model_obj);
				bd_model_obj->setDisplay_recursive(bd_entity->getDisplay());
				addToDB(bd_model_obj, baseObj->getDBSourceType());
			}
		}
		catch (const std::exception& e) {
			std::cout << "[BDRecon] cannot build lod2 model - ";
			std::cout << e.what() << std::endl;
			throw(std::runtime_error(e.what()));
			STOCKER_ERROR_ASSERT(e.what());
			//continue;
			//dispToConsole("[BDRecon] cannot build lod2 model", ERR_CONSOLE_MESSAGE);
			//dispToConsole(e.what(), ERR_CONSOLE_MESSAGE);
			//return;
		}
		//doActionBDProjectSave();
		ProgStep()
	}
	ProgEnd
	refreshAll();
	UpdateUI();

#if 0 //deprecated
	stocker::BuilderLOD2 builder_3d4em(true);

	/// select polyline for footprint
	{
		ccHObject::Container _container;
		m_ccRoot->getRootEntity()->filterChildren(_container, true, CC_TYPES::POLY_LINE, true);
		if (!_container.empty()) {
			ccPolyline* contour_polygon = nullptr;
			int selectedIndex = 0;
			//ask the user to choose a polyline
			selectedIndex = ccItemSelectionDlg::SelectEntity(_container, selectedIndex, this, "please select the contour polygon");
			if (selectedIndex >= 0 && selectedIndex < _container.size()) {
				contour_polygon = ccHObjectCaster::ToPolyline(_container[selectedIndex]);
			}
			if (contour_polygon) {
				std::vector<CCVector3> contour_points = contour_polygon->getPoints(false);
				std::vector<stocker::Contour3d> bd_contour_points_;
				stocker::Contour3d bd_contour_points;
				if (m_pbdrSettingLoD2Dlg->GroundHeightMode() == 2) {
					double user_defined_ground_height = m_pbdrSettingLoD2Dlg->UserDefinedGroundHeight();
					for (auto & pt : contour_points) {
						bd_contour_points.push_back(stocker::Vec3d(pt.x, pt.y, user_defined_ground_height));
					}
				}
				else {
					for (auto & pt : contour_points) {
						bd_contour_points.push_back(stocker::Vec3d(pt.x, pt.y, pt.z));
					}
				}
				bd_contour_points_.push_back(bd_contour_points);
				builder_3d4em.SetFootPrint(bd_contour_points_);
			}
		}
	}

	if (m_pbdrSettingLoD2Dlg->GroundHeightMode() == 2) {
		builder_3d4em.SetGroundHeight(m_pbdrSettingLoD2Dlg->UserDefinedGroundHeight());
	}

	std::string output_path = m_pbdrSettingLoD2Dlg->OutputFilePathLineEdit->text().toStdString();
	std::string ini_path = m_pbdrSettingLoD2Dlg->ConfigureFilePathLineEdit->text().toStdString();
	builder_3d4em.SetOutputPath(output_path.c_str());
	builder_3d4em.SetConfigurationPath(ini_path.c_str());

	QDir workingDir_old = QCoreApplication::applicationDirPath();
	QDir workingDir_current = GetFileDirectory(output_path.c_str());
	if (!workingDir_current.exists())
		if (!workingDir_current.mkpath(workingDir_current.absolutePath()))
			return;

	QDir::setCurrent(workingDir_current.absolutePath());

	if (!m_pbdrSettingLoD2Dlg->PointcloudFilePathLineEdit->text().isEmpty()) {
		// load from file
		std::string point_path = m_pbdrSettingLoD2Dlg->PointcloudFilePathLineEdit->text().toStdString();

		builder_3d4em.SetBuildingPoints(point_path.c_str());
	}

	if (!builder_3d4em.PlaneSegmentation(false)) return;
	if (!builder_3d4em.BuildingReconstruction()) return;

	if (!QFile::exists(QString(output_path.c_str()))) return;

	QStringList files_add_to_db{ QString(output_path.c_str()) };
	addToDB(files_add_to_db);

	dispToConsole("[BDRecon] Building Reconstruction LOD2 finished!", ccMainAppInterface::STD_CONSOLE_MESSAGE);

	//! restore the working directory
	QDir::setCurrent(workingDir_old.absolutePath());
#endif	
}

void MainWindow::doActionSettingsLoD2()
{
	if (!m_pbdrSettingLoD2Dlg) {
		m_pbdrSettingLoD2Dlg = new bdrSettingLoD2Dlg(this);
		m_pbdrSettingLoD2Dlg->setModal(false);
	}
	if (m_pbdrSettingLoD2Dlg->isHidden()) {
		m_pbdrSettingLoD2Dlg->show();
	}
	else {
		m_pbdrSettingLoD2Dlg->hide();
	}
}

void MainWindow::doActionBDTextureMapping()
{
	if (!haveSelection()) return;
	ccHObject* entity = getSelectedEntities().front();

	ccHObject::Container plane_container = GetPlaneEntitiesBySelected(entity);
	if (plane_container.empty()) {
		dispToConsole("[BDRecon] Please select  (group of) planes / buildings", ERR_CONSOLE_MESSAGE);
		return;
	}

	//! fast mapping for each plane entity
	if (!plane_container.empty())	{
		for (ccHObject* planeObj : plane_container) {
			try	{
				FastPlanarTextureMapping(planeObj);
			}
			catch (std::runtime_error& e) {
				dispToConsole(e.what(), ERR_CONSOLE_MESSAGE);
				dispToConsole("[BDRecon] Fast Planar Texture Mapping failed!", ERR_CONSOLE_MESSAGE);
				return;
			}			
		}
		refreshAll();
		UpdateUI();
		return;
	}
	else {
		//! real tex-recon, generate obj mesh and then load the textured mesh
		if (entity->getName().endsWith(BDDB_POLYFITOPTM_SUFFIX)) {

		}
		else if (entity->getName().endsWith(BDDB_FINALMODEL_SUFFIX)) {

		}
		else if (entity->isA(CC_TYPES::MESH)) {

		}
		entity->setLocked(true);
		return;
	}	
}

void MainWindow::doActionBDConstrainedMesh()
{
	if (!haveSelection()) return;
	ccHObject* entity = getSelectedEntities().front();
	ccHObject::Container plane_container = GetPlaneEntitiesBySelected(entity);
	if (plane_container.empty()) {
		dispToConsole("[BDRecon] Please select  (group of) planes / buildings", ERR_CONSOLE_MESSAGE);
		return;
	}

	for (auto & planeObj : plane_container) {
		try	{
			ccHObject* mesh = ConstrainedMesh(planeObj);
			if (mesh) {
				addToDB(mesh, planeObj->getDBSourceType());
				refreshAll();
				UpdateUI();
			}
		}
		catch (std::runtime_error& e) {
			dispToConsole(e.what(), ERR_CONSOLE_MESSAGE);
			dispToConsole("[BDRecon] Constrained mesh failed", ERR_CONSOLE_MESSAGE);
			return;
		}
	}
}

ccHObject * MainWindow::getCameraGroup(QString name)
{
	ccDBRoot* image_db = db_image();
	ccHObject* root = image_db->getRootEntity();
	for (size_t i = 0; i < root->getChildrenNumber(); i++) {
		if (root->getChild(i)->getName() == name) {
			return root->getChild(i);
		}
	}
	return nullptr;
}

void MainWindow::doActionShowBestImage()
{
	ccGLWindow* glwin = getActiveGLWindow(); assert(glwin); if (!glwin) return;
	ccViewportParameters params = glwin->getViewportParameters();
	ccGLCameraParameters camParas; glwin->getGLCameraParameters(camParas);

	CCVector3d viewPoint = params.getViewPoint();
	ccBBox objBox;
	//! wrong - bug remains
	if (params.objectCenteredView)
	{
		float scale_width = glwin->getCenterRadius(m_pbdrImagePanel->getBoxScale());
		CCVector3 half_box(scale_width, scale_width, scale_width);
		objBox.add(CCVector3::fromArray(params.pivotPoint.u) + half_box);
		objBox.add(CCVector3::fromArray(params.pivotPoint.u) - half_box);
	}
	else {
		//! TODO: mesh.. refer to graphical segmentation
 		ccHObject::Container point_clouds = GetEnabledObjFromGroup(dbRootObject(getCurrentDB()), CC_TYPES::POINT_CLOUD, true, true);
		
 		for (ccHObject* pc : point_clouds) {
 			ccPointCloud* pcObj = ccHObjectCaster::ToPointCloud(pc);
 			if (!pcObj) { continue; }
			ccBBox box_ = pcObj->getTheVisiblePointsBBox(camParas);
			objBox += box_;
 		}
		double scale_box = m_pbdrImagePanel->getBoxScale();
		objBox = ccBBox(objBox.getCenter() - objBox.getDiagVec()*scale_box / 2, objBox.getCenter() + objBox.getDiagVec()*scale_box / 2);
		std::cout << "calc bbox from clouds" << std::endl;
	}
	std::cout << "obj_bbox_min: " << objBox.minCorner().x << " " << objBox.minCorner().y << " " << objBox.minCorner().z << std::endl;
	std::cout << "obj_bbox_max: " << objBox.maxCorner().x << " " << objBox.maxCorner().y << " " << objBox.maxCorner().z << std::endl;
	std::cout << "up dir: " << glwin->getCurrentUpDir().x << " " << glwin->getCurrentUpDir().y << " " << glwin->getCurrentUpDir().z << std::endl;
	
	if (!objBox.isValid()) {
		return;
	}
	m_pbdrImagePanel->setObjViewBox(objBox);
	m_pbdrImagePanel->setObjViewUpDir(glwin->getCurrentUpDir());

	CCVector3 objCenter = //params.objectCenteredView ? CCVector3::fromArray(params.pivotPoint.u) : 
		objBox.getCenter();	// should be seen from the view
	CCVector3 view_to_obj = objCenter - CCVector3::fromArray(viewPoint.u);
	view_to_obj.normalize();
	
	CCVector3 obj_to_view = -view_to_obj;
	vcg::Point3f n(view_to_obj.u), u, v;
	vcg::GetUV(n, u, v);

	CCVector3 obj_o = objCenter - CCVector3::fromArray((u + v).V()) * objBox.getDiagNorm() / 2;
	CCVector3 obj_u = obj_o + CCVector3::fromArray(u.V()) * objBox.getDiagNorm();
	CCVector3 obj_v = obj_o + CCVector3::fromArray(v.V()) * objBox.getDiagNorm();

	//! get all cameras available 
	ccHObject::Container cameras = GetEnabledObjFromGroup(m_imageRoot->getRootEntity(), CC_TYPES::CAMERA_SENSOR, true, true);
	
	// sort by dir and distance
	typedef std::pair<ccCameraSensor*, double> HArea;
	std::vector<HArea> visible_area;
	for (ccHObject* cam : cameras) {
		ccCameraSensor* csObj = ccHObjectCaster::ToCameraSensor(cam); if (!csObj) { continue; }
		csObj->setDisplayOrder(-1);
		if (!csObj->isEnabled()) { continue; }

		CCVector2 objCenter_img;
		if (!csObj->fromGlobalCoordToImageCoord(objCenter, objCenter_img))
			continue;
		visible_area.push_back({ csObj, 0 });

		ccIndexedTransformation trans; csObj->getActiveAbsoluteTransformation(trans);
		const float* M = trans.data();

		CCVector3 cam_dir(-M[2], -M[6], -M[10]); cam_dir.normalize();
		CCVector3 cam_center; csObj->getActiveAbsoluteCenter(cam_center);
		CCVector3 cam_to_obj = (objCenter - cam_center); cam_to_obj.normalize();
		CCVector3 obj_to_cam = -cam_to_obj;
		double obj_angle = obj_to_cam.dot(obj_to_view);
		double cam_angle = cam_dir.dot(cam_to_obj);
		if (obj_angle < 0.0f || cam_angle < 0.0f) {
			continue;
		}

		//! sort by projection area
		CCVector2 objO_img, objU_img, objV_img;
		if (!csObj->fromGlobalCoordToImageCoord(obj_o, objO_img) ||
			!csObj->fromGlobalCoordToImageCoord(obj_u, objU_img) ||
			!csObj->fromGlobalCoordToImageCoord(obj_v, objV_img)) {
			continue;	//! skip temporarily
		}
		
		float double_area = (objU_img - objO_img).cross(objV_img - objO_img);
		visible_area.back().second = double_area;
	}
	if (visible_area.empty()) {
		return;
	}
	std::sort(visible_area.begin(), visible_area.end(), [](HArea _l, HArea _r) {
		return _l.second > _r.second;
	});
	for (size_t i = 0; i < visible_area.size(); i++) {
		ccCameraSensor* camObj = visible_area[i].first;
		assert(camObj); if (!camObj) return;
		camObj->setDisplayOrder(i);
	}
	m_pbdrImagePanel->display(false);

	doActionShowSelectedImage();
}

void MainWindow::doActionShowSelectedImage()
{
	ccHObject::Container sels;
	m_imageRoot->getSelectedEntities(sels, CC_TYPES::CAMERA_SENSOR);
	if (sels.empty()) {
		return;
	}
	ccHObject* sel = sels.front();
	ccCameraSensor* cam = ccHObjectCaster::ToCameraSensor(sel);
	if (!cam) { return; }

	m_pbdrImshow->setImageAndCamera(cam);
	if (m_pbdrImagePanel->isObjChecked()) {
		ccBBox box_2d;
		for (size_t i = 0; i < 8; i++) {
			CCVector3 b_2d;
			m_pbdrImshow->FromGlobalToImage(m_pbdrImagePanel->getObjViewBox().P(i), b_2d);
			box_2d.add(b_2d);
		}
		if (!box_2d.isValid()) return;

		CCVector3d up_3d = m_pbdrImagePanel->getObjViewUpDir();

		CCVector3 center; m_pbdrImshow->FromGlobalToImage(m_pbdrImagePanel->getObjViewBox().getCenter(), center);
		CCVector3 to_end;  m_pbdrImshow->FromGlobalToImage(CCVector3::fromArray(up_3d.u) + m_pbdrImagePanel->getObjViewBox().getCenter(), to_end);
		CCVector3 up_2d = to_end - center; up_2d.normalize();
		
		m_pbdrImshow->update2DDisplayZoom(box_2d, CCVector3d::fromArray(up_2d.u));

		std::cout << "img_bbox_min: " << box_2d.minCorner().x << " " << box_2d.minCorner().y << " " << box_2d.minCorner().z << std::endl;
		std::cout << "img_bbox_max: " << box_2d.maxCorner().x << " " << box_2d.maxCorner().y << " " << box_2d.maxCorner().z << std::endl;
		std::cout << "up dir: " << up_2d.x << " " << up_2d.y << " " << up_2d.z << std::endl;
	}
	else {
		m_pbdrImshow->ZoomFit();
	}
}

void MainWindow::doActionProjectToImage()
{
	if (!haveSelection()) { return; }
	if (!m_pbdrImshow->getImage()) {
		doActionShowBestImage();
	}
	if (m_pbdrImshow->getImage()) {
		m_pbdrImagePanel->setProjection(m_selectedEntities);
	}
}

void MainWindow::doActionSelectWorkingPlane()
{
	
}

void MainWindow::doActionTogglePlaneEditState()
{
	if (!m_pbdrPlaneEditDlg) return;

	//! if no selection and dialog shown, 
	if (!m_pbdrPlaneEditDlg->isHidden()) {
		m_pbdrPlaneEditDlg->cancle();
		m_pbdrPlaneEditDlg->hide();
		refreshAll();
		return;
	}

	if (haveSelection())
	{
		ccPlanarEntityInterface* plane = ccHObjectCaster::ToPlanarEntity(m_selectedEntities.front());
		if (plane) {
			m_pbdrPlaneEditDlg->initWithPlane(plane);
			plane->getPlane()->refreshDisplay();
		}
	}

	m_pbdrPlaneEditDlg->show();
}

void MainWindow::doActionEditSelectedItem()
{
	if (m_UI->DockablePanel->isHidden()) {
		m_UI->DockablePanel->show();
	}
	if (!haveSelection()) { return; }

	ccHObject* selected = m_selectedEntities.front();

	if (selected->isA(CC_TYPES::PLANE) || selected->isA(CC_TYPES::FACET) || selected->isA(CC_TYPES::ST_BLOCK)) {
		ccPlanarEntityInterface* plane = ccHObjectCaster::ToPlanarEntity(m_selectedEntities.front());
		if (plane) {
			if (m_pbdrPlaneEditDlg->isHidden()) {
				m_pbdrPlaneEditDlg->initWithPlane(plane);
				plane->getPlane()->refreshDisplay();
				m_UI->actionTogglePlaneEditState->setChecked(true);
				m_pbdrPlaneEditDlg->show();
			}
			else {
				m_pbdrPlaneEditDlg->initWithPlane(plane);
				plane->getPlane()->refreshDisplay();
			}
		}
	}
	else {

	}
}

void MainWindow::doActionCreateDatabase()
{
	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::LoadFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	QString database_name = QFileDialog::getExistingDirectory(this,
		tr("Open Directory"),
		QFileInfo(currentPath).absoluteFilePath(),
		QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
	
	if (database_name.isEmpty()) {
		return;
	}

	//save last loading parameters
	currentPath = QFileInfo(database_name).absoluteFilePath();
	settings.setValue(ccPS::CurrentPath(), currentPath);
	settings.endGroup();

	//QString database_name = QInputDialog::getText(this, "new database", "Database name", QLineEdit::Normal, QDate::currentDate().toString("yyyy-MM-dd"));
	//! create a database
	DataBaseHObject* new_database = DataBaseHObject::Create(database_name);
	if (new_database) {
		//! project settings dialog
		if (!m_pbdrPrjDlg) { m_pbdrPrjDlg = new bdrProjectDlg(this); m_pbdrPrjDlg->setModal(true); }
		m_pbdrPrjDlg->linkWithProject(new_database);
		m_pbdrPrjDlg->setProjectPath(new_database->getPath(), false);
		if (m_pbdrPrjDlg->exec() && new_database->load()) {
			addToDB_Main(new_database);
		}
		else {
			delete new_database;
			new_database = nullptr;
		}
	}
}

void MainWindow::doActionOpenDatabase()
{
	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::LoadFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	QString database_name = QFileDialog::getExistingDirectory(this,
		tr("Open Directory"),
		QFileInfo(currentPath).absolutePath(),
		QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

	if (!QFileInfo(database_name).exists())	{
		return;
	}

	//save last loading parameters
	currentPath = QFileInfo(database_name).absolutePath();
	settings.setValue(ccPS::CurrentPath(), currentPath);
	settings.endGroup();

	DataBaseHObject* load_database = DataBaseHObject::Create(database_name);

	if (load_database->load()) {
		addToDB_Main(load_database);
		return;
	}	
	else if (load_database) {
		delete load_database;
		load_database = nullptr;
	}

	CCVector3d loadCoordinatesShift(0, 0, 0);
	bool loadCoordinatesTransEnabled = false;
	FileIOFilter::LoadParameters parameters;
	{
		parameters.alwaysDisplayLoadDialog = true;
		parameters.shiftHandlingMode = ccGlobalShiftManager::DIALOG_IF_NECESSARY;
		parameters.coordinatesShift = &loadCoordinatesShift;
		parameters.coordinatesShiftEnabled = &loadCoordinatesTransEnabled;
		parameters.parentWidget = this;
	}
	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	//! find the bin
	QString bin_file = database_name + "/" + QFileInfo(database_name).completeBaseName() + ".bin";
	ccHObject* newGroup = FileIOFilter::LoadFromFile(bin_file, parameters, result, QString());
	if (!newGroup) return;

	load_database = new DataBaseHObject(*newGroup);
	load_database->setName(QFileInfo(database_name).completeBaseName());
	load_database->setPath(QFileInfo(database_name).absoluteFilePath());
	newGroup->transferChildren(*load_database);
	
	addToDB_Main(load_database);
	refreshAll();
	UpdateUI();
}

void MainWindow::doActionSaveDatabase()
{
	if (!haveSelection()) { return; }
	ccHObject*sel = m_selectedEntities.front();
	if (!isDatabaseProject(sel)) { return; }
	QFileInfo prj_path(sel->getPath());
	if (!prj_path.exists()) { return; }

	//! save as bin
	QString bin_file = prj_path.filePath() + "/" + prj_path.completeBaseName() + ".bin";

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	FileIOFilter::SaveParameters parameters;
	{
		parameters.alwaysDisplaySaveDialog = true;
		parameters.parentWidget = MainWindow::TheInstance();
	}

	//specific case: BIN format			
	result = FileIOFilter::SaveToFile(sel, bin_file, parameters, BinFilter::GetFileFilter());
}

void MainWindow::doActionEditDatabase()
{
	if (!m_pbdrPrjDlg) { m_pbdrPrjDlg = new bdrProjectDlg(this); m_pbdrPrjDlg->setModal(true); }
	
	DataBaseHObject* projObj = getCurrentMainDatabase();
	if (!projObj) {
		//! new
		doActionCreateDatabase();
		return;
	}
	
	m_pbdrPrjDlg->linkWithProject(projObj);
	m_pbdrPrjDlg->setProjectPath(projObj->getPath());
	if (m_pbdrPrjDlg->exec()) {
		if (projObj->load()) {
			addToDB_Main(projObj);
			projObj->setDisplay_recursive(projObj->getDisplay() ? projObj->getDisplay() : getActiveGLWindow());
		}
	}
}

void MainWindow::addToDatabase(QStringList files, ccHObject * import_pool, bool remove_exist, bool auto_sort)
{
	ccHObject::Container loaded_files = addToDB(files, CC_TYPES::DB_MAINDB);

	if (import_pool->getName() == "pointClouds" || 
		import_pool->getName() == "filtered" ||
		import_pool->getName() == "classified" || 
		import_pool->getName() == "segmented") {

		for (ccHObject* lf : loaded_files) {
			if (!lf) { continue; }
			if (lf->isA(CC_TYPES::HIERARCHY_OBJECT)) {
				ccHObject::Container loaded_objs;
				lf->filterChildren(loaded_objs, false, CC_TYPES::POINT_CLOUD, true);
				for (ccHObject* pc : loaded_objs) {
					//! check for existed
					if (remove_exist) {
						for (size_t i = 0; i < import_pool->getChildrenNumber(); i++) {
							ccHObject* child = import_pool->getChild(i);
							if (child->getName() == pc->getName()) {
								removeFromDB(child);
							}
						}
					}
					pc->getParent()->transferChild(pc, *import_pool);
					pc->setPath(lf->getPath());
				}
				removeFromDB(lf, false);
			}
			else if (lf->isA(CC_TYPES::POINT_CLOUD)) {
				assert(false);//! not happened
			}
		}
	}
	else if (import_pool->getName() == "images") {

	}

	if (auto_sort) {
		db(CC_TYPES::DB_MAINDB)->sortItemChildren(import_pool, ccDBRoot::SORT_A2Z);
	}
}

ccHObject::Container MainWindow::addPointsToDatabase(QStringList files, ccHObject * import_pool, bool remove_exist, bool auto_sort, bool fastLoad)
{
	ccHObject::Container loaded_files = addToDB(files, CC_TYPES::DB_MAINDB);
	ccHObject::Container trans_files;
	for (ccHObject* lf : loaded_files) {
		if (!lf) { continue; }
		if (fastLoad) {
			if (lf->isA(CC_TYPES::HIERARCHY_OBJECT)) {
				lf->setName(QFileInfo(lf->getName()).completeBaseName());
				// TODO: check existing
				if (remove_exist) {
					for (size_t i = 0; i < import_pool->getChildrenNumber(); i++) {
						ccHObject* child = import_pool->getChild(i);
						if (child->getName() == lf->getName()) {
							if (child->getPath() == lf->getPath()) {
								delete child;
								child = nullptr;
							}
							//removeFromDB(child);
						}
					}
				}
				if (lf->getParent()) {
					lf->getParent()->transferChild(lf, *import_pool);
				}
				else {
					import_pool->addChild(lf);
				}
			}
			continue;
		}
		if (lf->isA(CC_TYPES::HIERARCHY_OBJECT)) {
			ccHObject::Container loaded_objs;
			lf->filterChildren(loaded_objs, false, CC_TYPES::POINT_CLOUD, true);
			for (ccHObject* pc : loaded_objs) {
				//! check for existed
				if (remove_exist) {
					for (size_t i = 0; i < import_pool->getChildrenNumber(); i++) {
						ccHObject* child = import_pool->getChild(i);
						if (child->getName() == pc->getName()) {
							removeFromDB(child);
						}
					}
				}
				pc->getParent()->transferChild(pc, *import_pool);
				pc->setPath(lf->getPath());
				trans_files.push_back(pc);
			}
			removeFromDB(lf, false);
		}
		else if (lf->isA(CC_TYPES::POINT_CLOUD)) {
			assert(false);//! not happened
		}
	}

	if (auto_sort) {
		db(CC_TYPES::DB_MAINDB)->sortItemChildren(import_pool, ccDBRoot::SORT_A2Z);
	}
	return trans_files;
}

//QStringList s_import_filters;
ccHObject::Container MainWindow::getMainDatabases(bool check_enable)
{
	ccHObject* root = db(CC_TYPES::DB_MAINDB)->getRootEntity();
	return GetEnabledObjFromGroup(root, CC_TYPES::ST_PROJECT, check_enable, true);
}

DataBaseHObject* MainWindow::getCurrentMainDatabase(bool check_enable)
{
	ccHObject* baseObj = nullptr;
	ccHObject::Container dbs = getMainDatabases(check_enable);
	if (dbs.size() == 1 && dbs.front()) {
		baseObj = dbs.front();
	}
	else if (dbs.size() > 1) {
		if (haveSelection() && isDatabaseProject(getSelectedEntities().front())) {
			baseObj = getSelectedEntities().front();
		}
		else {
			baseObj = askUserToSelect(CC_TYPES::ST_PROJECT, dbs.front(), "please select the current active project", db(CC_TYPES::DB_MAINDB)->getRootEntity());
		}
	}
	return ToDatabaseProject(baseObj);
}

DataBaseHObject * MainWindow::getCurrentMainDatabase()
{
	DataBaseHObject* baseObj = getCurrentMainDatabase(true);
	if (!baseObj) {
		baseObj = getCurrentMainDatabase(false);
	}
	return baseObj;
}

void MainWindow::doActionImportData()
{
	if (!haveSelection()) {
		return;
	}
	ccHObject* import_pool = m_selectedEntities.front();
	 
	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::LoadFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
	QString currentOpenDlgFilter = settings.value(ccPS::SelectedInputFilter(), BinFilter::GetFileFilter()).toString();

	// Add all available file I/O filters (with import capabilities)
	const QStringList filterStrings = FileIOFilter::ImportFilterList();
	const QString &allFilter = filterStrings.at(0);

	if (!filterStrings.contains(currentOpenDlgFilter)) {
		currentOpenDlgFilter = allFilter;
	}
	//file choosing dialog
	QStringList selectedFiles = QFileDialog::getOpenFileNames(this,
		tr("Open file(s)"),
		currentPath,
		filterStrings.join(s_fileFilterSeparator),
		&currentOpenDlgFilter,
		CCFileDialogOptions());
	if (selectedFiles.isEmpty())
		return;

	//save last loading parameters
	currentPath = QFileInfo(selectedFiles[0]).absolutePath();
	settings.setValue(ccPS::CurrentPath(), currentPath);
	settings.setValue(ccPS::SelectedInputFilter(), currentOpenDlgFilter);
	settings.endGroup();

	addPointsToDatabase(selectedFiles, import_pool, true, true, true);
}

void MainWindow::doActionImportFolder()
{
	//! any database?
	ccHObject* root = db(CC_TYPES::DB_MAINDB)->getRootEntity();
	ccHObject* current_database = nullptr; ccHObject::Container dbs;
	for (size_t i = 0; i < root->getChildrenNumber(); i++) {
		if (isDatabaseProject(root->getChild(i))) {
			dbs.push_back(root->getChild(i));
		}
	}
	if (dbs.empty()) {
		return;
	}
	if (dbs.size() > 1) {
		current_database = askUserToSelect(CC_TYPES::ST_PROJECT);
	}
	else {
		current_database = dbs.front();
	}
	if (!current_database) { return; }

	if (!haveSelection()) {
		return;
	}
	ccHObject* import_pool = m_selectedEntities.front();
	QString dirname = QFileDialog::getExistingDirectory(this,
		tr("Open Directory"),
		"",
		QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

	if (!QFileInfo(dirname).exists()) {
		return;
	}

	QStringList files;
	QStringList nameFilters;
	// TODO: Dialog settings
	//if (import_pool->getName() == "pointClouds")
	nameFilters << "*.las" << "*.laz" << "*.ply" << "*.obj";
// 	else if (import_pool->getName() == "images") {
// 		nameFilters << "*.tif" << "*.tiff";
// 	}
	//else return;

	QDirIterator dir_iter(dirname, nameFilters, QDir::Files | QDir::NoSymLinks | QDir::Readable, QDirIterator::Subdirectories);
	while (dir_iter.hasNext()) {
		files.append(dir_iter.next());
	}
	
	addPointsToDatabase(files, import_pool, true, true, true);
}

void MainWindow::doActionCreateBuildingProject()
{
	if (m_selectedEntities.size() != 1) {
		return;
	}

	ccHObject* select = m_selectedEntities.front();
	//! get point clouds from selection
	ccHObject::Container point_clouds = GetEnabledObjFromGroup(select, CC_TYPES::POINT_CLOUD, true, false);
	if (point_clouds.empty()) {
		// TODO
		return;
	}

	DataBaseHObject* dataObj = GetRootDataBase(select);
	QString project_dir;
	{
		if (dataObj) {
			QString databasePath = dataObj->getPath();
			if (!databasePath.isEmpty()) {
				QString dir = databasePath + "/" + "IF_BUILDINGRECON";
				if (StCreatDir(dir)) {
					project_dir = dir + "/" + select->getName();
					if (!StCreatDir(project_dir)) project_dir.clear();
				}
			}
		}
		if (!QFileInfo(project_dir).exists()) {
			//! get directory
			//persistent settings
			QSettings settings;
			settings.beginGroup(ccPS::LoadFile());
			QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
			project_dir = QFileDialog::getExistingDirectory(this,
				tr("Open Directory"),
				QFileInfo(currentPath).absolutePath(),
				QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
		}
		if (!QFileInfo(project_dir).exists()) { return; }
	}

	//! project name: 
//	BDBaseHObject* baseObj = new BDBaseHObject(select->getName());
	QString bbprj_path = project_dir + "/" + select->getName() + ".bbprj";

	if (QFileInfo(bbprj_path).exists()) {
		QMessageBox message_box(QMessageBox::Question,
			tr("Project exists"),
			tr("Are you sure you want to overwrite?"),
			QMessageBox::Ok | QMessageBox::Cancel,
			this);
		if (message_box.exec() == QMessageBox::Ok) {
			QDir(project_dir).removeRecursively();
		}
		else {
			try {
				ccHObject* bd_grp = LoadBDReconProject(bbprj_path);
				if (bd_grp) {
					switchDatabase(CC_TYPES::DB_BUILDING);
					addToDB_Build(bd_grp);
					if (dataObj) {
						dataObj->setEnabled(false);
					}
				}
			}
			catch (const std::exception& e) {
				STOCKER_ERROR_ASSERT(e.what());
			}
			return;
		}
	}

	stocker::BlockProj block_prj;
	stocker::BuilderOption options;

	try
	{
		QString building_list = project_dir + "/buildings/building.txt";
		{
			QString building_dir = project_dir + "/buildings";
			if (!StCreatDir(building_dir)) { return; }
			FileIOFilter::SaveParameters parameters; {
				parameters.alwaysDisplaySaveDialog = false;
				parameters.parentWidget = this;
			}

			QStringList bd_paths;
			for (ccHObject* pc : point_clouds) {
				QString dir = building_dir + "/" + pc->getName(); if (!StCreatDir(dir)) return;
				QString relative_path = pc->getName() + "/" + pc->getName() + ".ply"; // bd00000001/bd00000001.ply
				QString absolute_path = building_dir + "/" + relative_path;
				if (FileIOFilter::SaveToFile(pc, absolute_path, parameters, "PLY mesh (*.ply)") != CC_FERR_NO_ERROR) {
					continue;
				}
				bd_paths.append(relative_path);
			}
			FILE* fp = fopen(building_list.toStdString().c_str(), "w");
			if (!fp) { return; }
			fprintf(fp, "%d\n", bd_paths.size());
			for (QString p : bd_paths) {
				fprintf(fp, "%s\n", p.toStdString().c_str());
			}
			fclose(fp);
		}

		QString image_dir = project_dir + "/images";
		if (!StCreatDir(image_dir)) { return; }
		QString image_list;
		{
			if (image_list.isEmpty()) {
				QString Filename =
					QFileDialog::getOpenFileName(this,
						"Open image list",
						bbprj_path,
						"image list (*.txt)");
				if (QFileInfo(Filename).exists()) {
					QStringList list; list.append(Filename);
					QStringList file = moveFilesToDir(list, image_dir, false);
					if (!file.isEmpty() && QFileInfo(file.front()).exists()) {
						image_list = file.front();
					}
				}
			}
		}
		
		QString sfm_out;
		{
			QString Filename =
				QFileDialog::getOpenFileName(this,
					"Open sfm out",
					bbprj_path,
					"sfm out (*.out)");
			if (QFileInfo(Filename).exists()) {
				QStringList list; list.append(Filename);
				QStringList file = moveFilesToDir(list, image_dir, false);
				if (!file.isEmpty() && QFileInfo(file.front()).exists()) {
					sfm_out = file.front();
				}
			}
		}
		options.prj_file.building_list = building_list.toStdString();
		options.prj_file.image_list = image_list.toStdString();
		options.prj_file.project_ini = bbprj_path.toStdString();
		options.prj_file.root_dir = project_dir.toStdString();
		options.prj_file.sfm_out = sfm_out.toStdString();

		block_prj.m_options = options;

		//! save .bbprj	
		stocker::SaveProjectIni(bbprj_path.toStdString(), options);
	}	
	catch (const std::exception& e) {
		STOCKER_ERROR_ASSERT(e.what());
		return;
	}

	try {
		ccHObject* bd_grp = LoadBDReconProject(bbprj_path);
		if (bd_grp) {
			switchDatabase(CC_TYPES::DB_BUILDING);
			addToDB_Build(bd_grp, false, false, false, true);
			if (dataObj) {
				dataObj->setEnabled(false);
			}
		}
	}
	catch (const std::exception& e) {
		STOCKER_ERROR_ASSERT(e.what());
	}
}

void MainWindow::doActionLoadSubstance()
{
	if (!haveSelection()) { return; }
	ccHObject* sel = m_selectedEntities.front();
	if (!sel) { return; }
	QStringList files;
	files.append(sel->getPath());
	addPointsToDatabase(files, sel, true, false, false);
}

inline QString getCmdLine(QString prj_path, QString task_name, int prj_id)
{
	QString cmd;
	QString runPath = QCoreApplication::applicationDirPath();
	cmd.append(runPath).append("/bin/RunTask.exe -i ").append(prj_path).append(" -l ").append(task_name).append(" -svrid ").append(QString::number(prj_id));
	return cmd;
}
inline QString getCmdLine(ccHObject* db_prj, QString task_name, int prj_id)
{
	QString prj_path = db_prj->getPath();
	if (!QFileInfo(prj_path).exists()) 
		return QString();
	else return getCmdLine(prj_path + "/", task_name, prj_id);
}

inline QString getCmdLine(ccHObject* db_prj, BlockDB::BLOCK_TASK_ID task_id, int prj_id)
{
	QString prj_path = db_prj->getPath();
	if (!QFileInfo(prj_path).exists())
		return QString();
	else return getCmdLine(prj_path + "/", QString::fromLatin1(BlockDB::g_strTaskTagName[task_id]), prj_id);
}

inline QStringList createTasksFiles(DataBaseHObject* db_prj,
	ccHObject::Container tasks,
	QString if_dir_name,
	QString exe_relative_path,
	QString gcTsk_relative_path,
	QString output_suffix, 
	QStringList para_settings)
{
	QStringList result_files;

	//! generate tsk
	QString if_dir = db_prj->getPath() + "/" + if_dir_name; StCreatDir(if_dir);
	QString tsk_dir = if_dir + "/TSK"; StCreatDir(tsk_dir);
	QString output_dir = if_dir + "/OUTPUT"; StCreatDir(output_dir);
	QStringList tsk_files;
	for (ccHObject* tsk : tasks) {
		QString tsk_file = tsk_dir + "/" + tsk->getName() + ".tsk";
		QString result_file = output_dir + "/" + tsk->getName() + output_suffix;
		if (output_suffix.isEmpty() || output_suffix == "/" || output_suffix == "\\") {
			StCreatDir(result_file);
		}
		FILE* fp = fopen(tsk_file.toStdString().c_str(), "w");
		fprintf(fp, "%s", tsk->getPath().toStdString().c_str());
		if (if_dir_name == "IF_FILTERING") {
			fprintf(fp, " ");
		}
		else {
			fprintf(fp, "\n");
		}
		
		fprintf(fp, "%s\n", result_file.toStdString().c_str());
		for (QString para : para_settings) {
			fprintf(fp, "%s\n", para.toStdString().c_str());
		}
		fclose(fp);
		tsk_files.append(tsk_file);
		result_files.append(result_file);
	}
	//! generate gctsk
	QString exe_path = QCoreApplication::applicationDirPath() + exe_relative_path;
	QString gctsk_path = if_dir + gcTsk_relative_path;
	FILE* fp = fopen(gctsk_path.toStdString().c_str(), "w");
	fprintf(fp, "%d\n", tasks.size());
	for (auto & tsk : tsk_files) {
		fprintf(fp, "%s %s\n", exe_path.toStdString().c_str(), tsk.toStdString().c_str());
	}
	fclose(fp);
	return result_files;
}

inline QStringList createTasksFiles(DataBaseHObject* db_prj,
	const std::vector<BlockDB::blkPtCldInfo>& tasks,
	BlockDB::BLOCK_TASK_ID task_id,
	QString exe_relative_path,
	QString gcTsk_relative_path,
	QString output_suffix,
	QStringList para_settings)
{
	QStringList result_files;

	//! generate tsk
	char strIfDir[256]; sprintf(strIfDir, "%s%s%s%s%s", db_prj->getPath().toStdString().c_str(), "/", IF_PROCESS_DIR, "/", BlockDB::g_strTaskDirName[task_id]);
	QString if_dir = QString::fromLocal8Bit(strIfDir); StCreatDir(if_dir);
	QString tsk_dir = if_dir + "/TSK"; StCreatDir(tsk_dir);
	QString output_dir = if_dir + "/OUTPUT"; StCreatDir(output_dir);
	QStringList tsk_files;
	for (auto task : tasks) {
		//! .task
		QString tsk_file = tsk_dir + "/" + QString::fromLocal8Bit(task.sName) + ".tsk";

		//! input and output file
		QString result_file = output_dir + "/" + QString::fromLocal8Bit(task.sName) + output_suffix;
		if (output_suffix.isEmpty() || output_suffix == "/" || output_suffix == "\\") {
			StCreatDir(result_file);
		}
		FILE* fp = fopen(tsk_file.toStdString().c_str(), "w");
		fprintf(fp, "%s", task.sPath);
		if (task_id == BlockDB::TASK_ID_FILTER) {
			fprintf(fp, " ");
		}
		else {
			fprintf(fp, "\n");
		}

		fprintf(fp, "%s\n", result_file.toStdString().c_str());
		for (QString para : para_settings) {
			fprintf(fp, "%s\n", para.toStdString().c_str());
		}
		fclose(fp);
		tsk_files.append(tsk_file);
		result_files.append(result_file);
	}
	//! generate gctsk
	QString exe_path = QCoreApplication::applicationDirPath() + exe_relative_path;
	QString gctsk_path = if_dir + gcTsk_relative_path;
	FILE* fp = fopen(gctsk_path.toStdString().c_str(), "w");
	fprintf(fp, "%d\n", tasks.size());
	for (auto & tsk : tsk_files) {
		fprintf(fp, "%s %s\n", exe_path.toStdString().c_str(), tsk.toStdString().c_str());
	}
	fclose(fp);
	return result_files;
}

void MainWindow::doActionImageLiDARRegistration()
{
	DataBaseHObject* baseObj = getCurrentMainDatabase();
	if (!baseObj) { ccLog::Error(QString::fromLocal8Bit("请先载入工程!")); return; }

	if (baseObj->m_blkData) {
		if (!baseObj->m_blkData->saveProject(true)) {
			QMessageBox::critical(this, "Error!", QString::fromLocal8Bit("无法生成配准工程"));
			return;
		}
	}
	QString exe_path = QCoreApplication::applicationDirPath() + "/bin/Registration/RegisLiDAR_Image.exe";
	std::string regis_path;
	baseObj->m_blkData->getMetaValue(REGISPRJ_PATH_KEY, regis_path);
	QString xml = QString::fromStdString(regis_path);
	if (QFileInfo(xml).exists()) {
		QProcess::execute(exe_path + " " + xml);
	}
	else {
		QMessageBox::critical(this, "Error!", QString::fromLocal8Bit("无法打开配准工程"));
	}
}

void LoadSettingsFiltering()
{

}

void MainWindow::doActionGroundFilteringBatch()
{
	DataBaseHObject* baseObj = getCurrentMainDatabase();
	if (!baseObj) { ccLog::Error(QString::fromLocal8Bit("请先载入工程!")); return; }

	//! get valid data by level
	std::vector<BlockDB::blkPtCldInfo> poinclouds;
	BlockDB::BLOCK_PtCldLevel _level = BlockDB::PCLEVEL_STRIP;
	for (auto & pt : baseObj->m_blkData->getPtClds()) {
		if (pt.level == _level && pt.nGroupID == baseObj->m_blkData->projHdr().groupID) {
			poinclouds.push_back(pt);
		}
	}
	_level = BlockDB::PCLEVEL_TILE;
	for (auto & pt : baseObj->m_blkData->getPtClds()) {
		if (pt.level == _level && pt.nGroupID == baseObj->m_blkData->projHdr().groupID) {
			poinclouds.push_back(pt);
		}
	}

	QStringList result_files;
	{
		//! parameters
		if (!m_pbdrSettingGrdFilterDlg) { m_pbdrSettingGrdFilterDlg = new bdrSettingGrdFilterDlg(this); }
		QStringList para_settings = m_pbdrSettingGrdFilterDlg->getParameters();
		//! filters
		QString output_suffix = ".las";
		result_files = createTasksFiles(baseObj, poinclouds,
			BlockDB::TASK_ID_FILTER,
			"/bin/FILTERING/filtering_knl.exe",
			"/filtering.gtsk",
			output_suffix,
			para_settings);
	}

	QString cmd = getCmdLine(baseObj, BlockDB::TASK_ID_FILTER, baseObj->m_blkData->projHdr().projectID);
	std::cout << "cmd: " << cmd.toStdString() << std::endl;
	if (cmd.isEmpty()) return;

	//! whether wait for results
	QMessageBox *messageBox = new QMessageBox(this);
	messageBox->setIcon(QMessageBox::Question);
	messageBox->setWindowTitle(QString::fromLocal8Bit("结果自动返回"));
	messageBox->setText(QString::fromLocal8Bit("是否等待界面返回滤波结果?"));
	messageBox->addButton(QString::fromLocal8Bit("否"), QMessageBox::RejectRole);
	messageBox->addButton(QString::fromLocal8Bit("是"), QMessageBox::AcceptRole);
	bool waitForResult = (messageBox->exec() == QDialog::Accepted);

	if (waitForResult) {
		QScopedPointer<ccProgressDialog> pDlg(nullptr);
		pDlg.reset(new ccProgressDialog(false, this));
		pDlg->setMethodTitle(QObject::tr("Ground Filtering"));
		pDlg->setInfo(QObject::tr("Please wait... filtering in progress"));
		pDlg->setRange(0, 0);
		pDlg->setModal(false);
		pDlg->start();

		QFutureWatcher<void> executer;
		connect(&executer, &QFutureWatcher<void>::finished, this, [=]() {
			//! the results should be moved to dirs in the check process
			baseObj->parseResults(BlockDB::TASK_ID_FILTER, result_files, 1);
			//! otherwise open dialog --> import folders automatically
			baseObj->retrieveResults(BlockDB::TASK_ID_FILTER);
			
			QMessageBox::information(this, "Finished!", "The filtering task is completed");
		});
		QObject::connect(&executer, SIGNAL(finished()), pDlg.data(), SLOT(reset()));
		executer.setFuture(QtConcurrent::run([&cmd]() { QProcess::execute(cmd); }));
		pDlg->exec();
		executer.waitForFinished();
	}
	else {
		QProcess::startDetached(cmd);
	}
}

void MainWindow::doActionClassificationBatch()
{
	DataBaseHObject* baseObj = getCurrentMainDatabase();
	if (!baseObj) { ccLog::Error(QString::fromLocal8Bit("请先载入工程!")); return; }

	//! get valid data by level
	std::vector<BlockDB::blkPtCldInfo> poinclouds;
	BlockDB::BLOCK_PtCldLevel _level = BlockDB::PCLEVEL_FILTER;
	for (auto & pt : baseObj->m_blkData->getPtClds()) {
		if (pt.level == _level && pt.nGroupID == baseObj->m_blkData->projHdr().groupID) {
			poinclouds.push_back(pt);
		}
	}

	QStringList result_files;
	{
		//! parameters
		QStringList para_settings;
		//! filters
		QString output_suffix = ".las";		
		result_files = createTasksFiles(baseObj, poinclouds,
			BlockDB::TASK_ID_CLASS,
			"/bin/CLASSIFY/Classify_KNL.exe",
			"/classification.gtsk",
			output_suffix,
			para_settings);
	}

	QString cmd = getCmdLine(baseObj, BlockDB::TASK_ID_FILTER, baseObj->m_blkData->projHdr().projectID);
	std::cout << "cmd: " << cmd.toStdString() << std::endl;
	if (cmd.isEmpty()) return;

	//! whether wait for results
	QMessageBox *messageBox = new QMessageBox(this);
	messageBox->setIcon(QMessageBox::Question);
	messageBox->setWindowTitle(QString::fromLocal8Bit("结果自动返回"));
	messageBox->setText(QString::fromLocal8Bit("是否等待界面返回分类结果?"));
	messageBox->addButton(QString::fromLocal8Bit("否"), QMessageBox::RejectRole);
	messageBox->addButton(QString::fromLocal8Bit("是"), QMessageBox::AcceptRole);
	bool waitForResult = (messageBox->exec() == QDialog::Accepted);
		
	if (waitForResult) {
		QScopedPointer<ccProgressDialog> pDlg(nullptr);
		pDlg.reset(new ccProgressDialog(false, this));
		pDlg->setMethodTitle(QObject::tr("Classification"));
		pDlg->setInfo(QObject::tr("Please wait... classifying in progress"));
		pDlg->setRange(0, 0);
		pDlg->setModal(false);
		pDlg->start();

		QFutureWatcher<void> executer;
		connect(&executer, &QFutureWatcher<void>::finished, this, [=]() {
			//! the results should be moved to dirs in the check process
			baseObj->parseResults(BlockDB::TASK_ID_CLASS, result_files, 1);
			//! otherwise open dialog --> import folders automatically
			baseObj->retrieveResults(BlockDB::TASK_ID_CLASS);

			QMessageBox::information(this, "Finished!", "The classification task is completed");
		});
		QObject::connect(&executer, SIGNAL(finished()), pDlg.data(), SLOT(reset()));
		executer.setFuture(QtConcurrent::run([&cmd]() { QProcess::execute(cmd); }));
		pDlg->exec();
		executer.waitForFinished();
	}
	else {
		QProcess::startDetached(cmd);
	}
}

void MainWindow::doActionBuildingSegmentationBatch()
{
	DataBaseHObject* baseObj = getCurrentMainDatabase();
	if (!baseObj) { ccLog::Error(QString::fromLocal8Bit("请先载入工程!")); return; }

	//! get valid data by level
	std::vector<BlockDB::blkPtCldInfo> poinclouds;
	BlockDB::BLOCK_PtCldLevel _level = BlockDB::PCLEVEL_CLASS;
	for (auto & pt : baseObj->m_blkData->getPtClds()) {
		if (pt.level == _level && pt.nGroupID == baseObj->m_blkData->projHdr().groupID) {
			poinclouds.push_back(pt);
		}
	}

	QStringList result_files;
	{
		//! parameters
		QStringList para_settings;
		//! filters
		QString output_suffix = ".las";
		result_files = createTasksFiles(baseObj, poinclouds,
			BlockDB::TASK_ID_BDSEG,
			"/bin/BUILDINGSEG/BUIDINGSEG_KNL.exe",
			"/segmentation.gtsk",
			output_suffix,
			para_settings);
	}

	QString cmd = getCmdLine(baseObj, BlockDB::TASK_ID_BDSEG, baseObj->m_blkData->projHdr().projectID);
	std::cout << "cmd: " << cmd.toStdString() << std::endl;
	if (cmd.isEmpty()) return;

	//! whether wait for results
	QMessageBox *messageBox = new QMessageBox(this);
	messageBox->setIcon(QMessageBox::Question);
	messageBox->setWindowTitle(QString::fromLocal8Bit("结果自动返回"));
	messageBox->setText(QString::fromLocal8Bit("是否等待界面返回分割结果?"));
	messageBox->addButton(QString::fromLocal8Bit("否"), QMessageBox::RejectRole);
	messageBox->addButton(QString::fromLocal8Bit("是"), QMessageBox::AcceptRole);
	bool waitForResult = (messageBox->exec() == QDialog::Accepted);

	if (waitForResult) {
		QScopedPointer<ccProgressDialog> pDlg(nullptr);
		pDlg.reset(new ccProgressDialog(false, this));
		pDlg->setMethodTitle(QObject::tr("Building segmentation"));
		pDlg->setInfo(QObject::tr("Please wait... segmentation in progress"));
		pDlg->setRange(0, 0);
		pDlg->setModal(false);
		pDlg->start();

		QFutureWatcher<void> executer;
		connect(&executer, &QFutureWatcher<void>::finished, this, [=]() {
			//! the results should be moved to dirs in the check process
			baseObj->parseResults(BlockDB::TASK_ID_BDSEG, result_files, 1);
			//! otherwise open dialog --> import folders automatically
			baseObj->retrieveResults(BlockDB::TASK_ID_BDSEG);

			QMessageBox::information(this, "Finished!", "The segmentation task is completed");
		});
		QObject::connect(&executer, SIGNAL(finished()), pDlg.data(), SLOT(reset()));
		executer.setFuture(QtConcurrent::run([&cmd]() { QProcess::execute(cmd); }));
		pDlg->exec();
		executer.waitForFinished();
	}
	else {
		QProcess::startDetached(cmd);
	}
}

void MainWindow::doActionPointClassEditor()
{
	ccGLWindow* win = getActiveGLWindow();
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
	for (ccHObject *entity : getSelectedEntities())
	{
		if (entity->isKindOf(CC_TYPES::POINT_CLOUD) || entity->isKindOf(CC_TYPES::MESH)) {
			m_gsTool->addEntity(entity);
		}
	}
	m_gsTool->setSegmentMode(ccGraphicalSegmentationTool::SEGMENT_CLASS_EDIT);

	if (m_gsTool->getNumberOfValidEntities() == 0)
	{
		ccConsole::Error("No segmentable entity in active window!");
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

void MainWindow::deactivatePointClassEditor(bool)
{
}

void MainWindow::doActionBuildingSegmentEditor()
{
	ccGLWindow* win = getActiveGLWindow();
	if (!win)
		return;

	if (!haveSelection())
		return;

	if (!m_gsTool)
	{
		m_gsTool = new ccGraphicalSegmentationTool(this);
		connect(m_gsTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateBuildingSegmentEditor);

		registerOverlayDialog(m_gsTool, Qt::TopRightCorner);
	}

	m_gsTool->linkWith(win);

	m_gsTool->setSegmentMode(ccGraphicalSegmentationTool::SEGMENT_BUILD_EIDT);

	ccHObject* select = m_selectedEntities.front();	
	if (!select->isA(CC_TYPES::POINT_CLOUD)) {
		return;
	}
	DataBaseHObject* baseObj = GetRootDataBase(select);

	ccHObject* building_group = nullptr;
	if (baseObj) {
		ccHObject* product_group = baseObj->getProductSegmented();
		building_group = getChildGroupByName(product_group, select->getName(), true, true);
		//building_group = findChildByName(product_group, false, select->getName(), true, CC_TYPES::HIERARCHY_OBJECT, true);
	}
	else {
		building_group = new ccHObject(select->getName());
		building_group->setDisplay(select->getDisplay());
		addToDB_Main(building_group, false, false);
	}
	m_gsTool->setDestinationGroup(building_group);

	for (ccHObject *entity : getSelectedEntities())
	{
		if (entity->isKindOf(CC_TYPES::POINT_CLOUD) || entity->isKindOf(CC_TYPES::MESH)) {
			m_gsTool->addEntity(entity);
		}
	}

	if (m_gsTool->getNumberOfValidEntities() == 0)
	{
		ccConsole::Error("No editable entity in active window!");
		return;
	}

	freezeUI(true);
	m_UI->toolBarView->setDisabled(false);

	//we disable all other windows
	//disableAllBut(win);

	if (!m_gsTool->start())
		deactivateBuildingSegmentEditor(false);
	else
		updateOverlayDialogsPlacement();
}

void MainWindow::deactivateBuildingSegmentEditor(bool state)
{
	m_gsTool->setDestinationGroup(nullptr);

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

void MainWindow::doAactionSettingsGroundFiltering()
{
	if (!m_pbdrSettingBDSegDlg) { m_pbdrSettingBDSegDlg = new bdrSettingBDSegDlg(this); }
	m_pbdrSettingGrdFilterDlg->setModal(false);
	if (m_pbdrSettingGrdFilterDlg->isHidden()) {
		m_pbdrSettingGrdFilterDlg->show();
	}
	else {
		m_pbdrSettingGrdFilterDlg->hide();
	}
}

void MainWindow::doActionSettingsClassification()
{
}

void MainWindow::doActionSettingsBuildingSeg()
{
	if (!m_pbdrSettingBDSegDlg) m_pbdrSettingBDSegDlg = new bdrSettingBDSegDlg(this);
	m_pbdrSettingBDSegDlg->setModal(false);
	if (m_pbdrSettingBDSegDlg->isHidden()) {
		m_pbdrSettingBDSegDlg->show();
	}
	else {
		m_pbdrSettingBDSegDlg->hide();
	}
}

void MainWindow::doActionScheduleProjectID()
{
	DataBaseHObject* base = getCurrentMainDatabase();
	if (base) {
		bool ok;
		int getint = QInputDialog::getInt(this, "Project ID", "Project ID:", base->m_blkData->projHdr().projectID, 0, 99, 1, &ok);
		if (ok) {
			base->m_blkData->projHdr().projectID = getint;
		}
	}
}

void MainWindow::doActionScheduleGCServer()
{
	QString runPath = QCoreApplication::applicationDirPath() + "/bin/WGCS/GCSvr.exe";
	QProcess::startDetached(runPath, QStringList(runPath));
}

void MainWindow::doActionScheduleGCNode()
{
	QString runPath = QCoreApplication::applicationDirPath() + "/bin/WGCS/GCNode.exe";
	QProcess::startDetached(runPath, QStringList(runPath));
}

void MainWindow::doActionClearEmptyItems()
{
// 	if (!haveSelection()) {
// 		return;
// 	}
// 	ccHObject* sel = m_selectedEntities.front();
// 
// 	ccHObject::Container children;
// 	sel->filterChildren(children, true);
// 	do {
// 		sel->filterChildren(children, true);
// 	} while (1);
// 	for (ccHObject* child : children) {
// 		if (child->isKindOf(CC_TYPES::MESH)) {
// 			if (child)
// 			{
// 			}
// 		}
// 	}
}
