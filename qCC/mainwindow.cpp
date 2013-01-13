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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2275                                                              $
//$LastChangedDate:: 2012-10-17 23:30:43 +0200 (mer., 17 oct. 2012)        $
//**************************************************************************
//

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
#include <RegistrationTools.h>  //Aurelien BEY
#include <CloudSamplingTools.h> //Aurelien BEY

//qCC_db
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccMeshGroup.h>
#include <ccOctree.h>
#include <ccGBLSensor.h>
#include <ccNormalVectors.h>
#include <ccProgressDialog.h>
#include <ccPlane.h>
#include <ccImage.h>
#include <cc2DLabel.h>
#include <cc2DViewportObject.h>

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
#include "ccRenderToFileDlg.h" //DGM
#include "ccPointPropertiesDlg.h" //Aurelien BEY
#include "ccPointListPickingDlg.h"
#include "ccNormalComputationDlg.h"
#include "ccCameraParamEditDlg.h"
#include "ccScalarFieldArithmeticDlg.h"
#include "ccSensorComputeDistancesDlg.h"
#include "ccSensorComputeScatteringAnglesDlg.h"
#include "ccCurvatureDlg.h"
#include "ccApplyTransformationDlg.h"
#include "ccCoordinatesShiftManager.h"
#include "ccPointPairRegistrationDlg.h"
#include "ccExportCoordToSFDlg.h"
#include <ui_aboutDlg.h>

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
#include <math.h>
#include <assert.h>

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
    , m_cpeDlg(0)
    , m_gsTool(0)
    , m_transTool(0)
    , m_compDlg(0)
    , m_ppDlg(0)
    , m_plpDlg(0)
	, m_pprDlg(0)
    , m_pluginsGroup(0)
{
    //Dialog "auto-construction"
    setupUi(this);

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

    //Window Mapper
    m_windowMapper = new QSignalMapper(this);
    connect(m_windowMapper, SIGNAL(mapped(QWidget*)), this, SLOT(setActiveSubWindow(QWidget*)));

	//Keyboard shortcuts
	connect(actionToggleVisibility,	SIGNAL(triggered()), this, SLOT(toggleSelectedEntitiesVisibility()));	//'V': toggles selected items visibility
	connect(actionToggleNormals,	SIGNAL(triggered()), this, SLOT(toggleSelectedEntitiesNormals()));		//'N': toggles selected items normals visibility
	connect(actionToggleColors,		SIGNAL(triggered()), this, SLOT(toggleSelectedEntitiesColors()));		//'C': toggles selected items colors visibility
	connect(actionToggleSF,			SIGNAL(triggered()), this, SLOT(toggleSelectedEntitiesSF()));			//'S': toggles selected items SF visibility

    connectActions();

    loadPlugins();

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
    m_compDlg=0;
    m_ppDlg = 0;
    m_plpDlg = 0;
	m_pprDlg = 0;
    m_pluginsGroup = 0;

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

    /*//QT takes care of all siblings!
    if (m_pluginsGroup)
        delete m_pluginsGroup;
    m_pluginsGroup=0;
    //*/

	if (ccRoot)
		delete ccRoot;
}

ccPluginInterface* MainWindow::getValidPlugin(QObject* plugin)
{
    //standard plugin?
    ccStdPluginInterface* ccStdPlugin = qobject_cast<ccStdPluginInterface*>(plugin);
    if (ccStdPlugin)
        return static_cast<ccPluginInterface*>(ccStdPlugin);

    ccGLFilterPluginInterface* ccGLPlugin = qobject_cast<ccGLFilterPluginInterface*>(plugin);
    if (ccGLPlugin)
        return static_cast<ccPluginInterface*>(ccGLPlugin);

    return 0;
}

void MainWindow::loadPlugins()
{
    foreach (QObject *plugin, QPluginLoader::staticInstances())
		addPluginToPluginGroup(plugin);

    ccConsole::Print(QString("Application path: ")+QCoreApplication::applicationDirPath());
    m_pluginsPath = QCoreApplication::applicationDirPath();

    // TODO DGM: I wanted to get automatically the debug versions of plugins, but QT throws an error
    // while loading these debug plugins ('can't mix release and debug versions') ! I didn't manage
    // to sort this out yet ... However, it would be a more convenient/proper way to work ...
    //#ifdef _DEBUG
    //    m_pluginsPath += QString("../plugins/debug");
    //#else
    //Now release plugins are in bin/plugins
    m_pluginsPath += QString("/plugins");
    //#endif

    ccConsole::Print(QString("Plugins lookup dir.: %1").arg(m_pluginsPath));

    QStringList filters;
#if defined(Q_OS_WIN)
    filters << "*.dll";
#endif
#if defined(Q_OS_LINUX)
    filters << "*.so";
#endif
    QDir pluginsDir(m_pluginsPath);
    pluginsDir.setNameFilters(filters);
    foreach (QString filename, pluginsDir.entryList(filters))
    {
        QPluginLoader loader(pluginsDir.absoluteFilePath(filename));
        QObject *plugin = loader.instance();
        if (plugin)
        {
            ccConsole::Print(QString("Found new plugin! ('%1')").arg(filename));
            if (addPluginToPluginGroup(plugin))
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

    if (!m_pluginsGroup || m_pluginsGroup->actions().isEmpty())
    {
        //menuPlugins->setEnabled(false);
        //actionDisplayPluginTools->setEnabled(false);
        toolBarPluginTools->setVisible(false);
        return;
    }

    //first we add all "GL Filters plugins" to plugins toolbar
    int count=0;
    foreach (QAction *action, m_pluginsGroup->actions())
    {
        ccPluginInterface *ccPlugin = getValidPlugin(action->parent());
        if (ccPlugin && ccPlugin->getType() == CC_GL_FILTER_PLUGIN)
        {
            toolBarPluginTools->addAction(action);
            ++count;
        }
    }

    if (count)
    {
        toolBarPluginTools->addAction(actionNoFilter);
        QAction* separator = new QAction(this);
        separator->setSeparator(true);
        toolBarPluginTools->addAction(separator);
    }

    //then, all the others!
    foreach (QAction *action, m_pluginsGroup->actions())
    {
        bool hasPlugin = false;
        ccPluginInterface *ccPlugin = getValidPlugin(action->parent());
        if (ccPlugin && ccPlugin->getType() != CC_GL_FILTER_PLUGIN)
        {
            toolBarPluginTools->addAction(action);
            hasPlugin=true;
        }

        if (hasPlugin)
        {
            toolBarPluginTools->setEnabled(true);
            actionDisplayPluginTools->setEnabled(true);
            actionDisplayPluginTools->setChecked(true);
        }
    }

    //menuPlugins->setEnabled(true);
    //actionDisplayPluginTools->setEnabled(true);
}

bool MainWindow::addPluginToPluginGroup(QObject *plugin)
{
    ccPluginInterface* ccPlugin = getValidPlugin(plugin);
    if (!ccPlugin)
        return false;

    if (!m_pluginsGroup)
        m_pluginsGroup = new QActionGroup(this);

    ccPluginDescription desc;
    ccPlugin->getDescription(desc);

    ccConsole::Print("Plugin name: [%s]",desc.name);
	plugin->setParent(this);
    QAction *action = new QAction(desc.menuName, plugin);
    connect(action, SIGNAL(triggered()), this, SLOT(doPluginAction()));

    //action settings
    if (desc.hasAnIcon)
        action->setIcon(ccPlugin->getIcon());
    action->setToolTip(desc.name);

    switch (ccPlugin->getType())
    {
    case CC_STD_PLUGIN:
        menuPlugins->addAction(action);
        menuPlugins->setEnabled(true);
		static_cast<ccStdPluginInterface*>(ccPlugin)->setMainAppInterface(this);
        break;
    case CC_GL_FILTER_PLUGIN:
        menuShadersAndFilters->addAction(action);
        break;
    }

    //toolBarPluginTools->addAction(action);
    m_pluginsGroup->addAction(action);

    return true;
}

void MainWindow::aboutPlugins()
{
    ccPluginDlg ccpDlg(m_pluginsPath, m_pluginFileNames, this);
    ccpDlg.exec();
}

void MainWindow::doPluginAction()
{
    QAction *action = qobject_cast<QAction*>(sender());
    ccPluginInterface *ccPlugin = getValidPlugin(action->parent());
    if (!ccPlugin)
        return;

    CC_PLUGIN_TYPE type = ccPlugin->getType();
    if (type==CC_STD_PLUGIN)
    {
        ccStdPluginInterface *ccStdPlugin = static_cast<ccStdPluginInterface*>(ccPlugin);

        unsigned uiModificationFlags=0;
        ccProgressDialog pDlg(true,this);
        //pDlg.stop();

		ccHObject::Container selection = m_selectedEntities;
		int errorCode = ccStdPlugin->doAction(selection,uiModificationFlags,&pDlg,this);
        if (errorCode==1)
        {
            //if the plugin has produced new entities
			for (ccHObject::Container::const_iterator it=selection.begin();it!=selection.end();++it)
				//for each output entities, we check if it exists in the original selection
				if (std::find(m_selectedEntities.begin(),m_selectedEntities.end(),*it) == m_selectedEntities.end())
					addToDB(*it);

            if ((uiModificationFlags & CC_PLUGIN_REFRESH_GL_WINDOWS) == CC_PLUGIN_REFRESH_GL_WINDOWS)
                refreshAll();
            if ((uiModificationFlags & CC_PLUGIN_REFRESH_ENTITY_BROWSER) == CC_PLUGIN_REFRESH_ENTITY_BROWSER)
                updateUI();
            if ((uiModificationFlags & CC_PLUGIN_EXPAND_DB_TREE) == CC_PLUGIN_EXPAND_DB_TREE)
                expandDBTreeWithSelection(selection);
        }
        else if (errorCode!=0)
        {
			QString errMessage = ccStdPlugin->getErrorMessage(errorCode);
			ccConsole::Error(errMessage.isEmpty() ? QString("Unknown error") : errMessage);
        }
    }
    else if (type==CC_GL_FILTER_PLUGIN)
    {
        ccGLWindow* win = getActiveGLWindow();
        if (win)
        {
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
                ccConsole::Error("Can't load GL filter (an error occured)!");
        }
    }
}

void MainWindow::connectActions()
{
    assert(m_ccRoot);
    assert(m_mdiArea);

	//Actions/menus disabled by default (for test purpose only!)
#ifndef _DEBUG
	actionSNETest->setVisible(false);
	menuBoundingBox->setVisible(false);
#endif

    /*** MAIN MENU ***/

    //"File" menu
    connect(actionOpen,                         SIGNAL(triggered()),    this,       SLOT(loadFile()));
    connect(actionSave,                         SIGNAL(triggered()),    this,       SLOT(saveFile()));
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
	connect(actionMeshBestFittingQuadric,		SIGNAL(triggered()),    this,       SLOT(doActionComputeQuadric3D()));
    connect(actionSamplePoints,                 SIGNAL(triggered()),    this,       SLOT(doActionSamplePoints()));
    connect(actionSmoothMesh_Laplacian,         SIGNAL(triggered()),    this,       SLOT(doActionSmoothMeshLaplacian()));
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
    connect(actionMultiplySF,                   SIGNAL(triggered()),    this,       SLOT(doActionMultiplySF()));
    connect(actionConvertToRGB,                 SIGNAL(triggered()),    this,       SLOT(doActionSFConvertToRGB()));
	connect(actionRenameSF,						SIGNAL(triggered()),    this,       SLOT(doActionRenameSF()));
    connect(actionDeleteScalarField,            SIGNAL(triggered()),    this,       SLOT(doActionDeleteScalarField()));
    connect(actionDeleteAllSF,                  SIGNAL(triggered()),    this,       SLOT(doActionDeleteAllSF()));
    //"Edit > Bounding-box" menu
    connect(actionComputeBestFitBB,             SIGNAL(triggered()),    this,       SLOT(doComputeBestFitBB()));
    //"Edit" menu
    connect(actionPointListPicking,             SIGNAL(triggered()),    this,       SLOT(activatePointListPickingMode()));
    connect(actionPointPicking,                 SIGNAL(triggered()),    this,       SLOT(activatePointsPropertiesMode()));
    connect(actionClone,                        SIGNAL(triggered()),    this,       SLOT(doActionClone()));
    connect(actionFuse,                         SIGNAL(triggered()),    this,       SLOT(doActionFuse()));
    connect(actionApplyTransformation,			SIGNAL(triggered()),    this,       SLOT(doActionApplyTransformation()));
    connect(actionMultiply,                     SIGNAL(triggered()),    this,       SLOT(doActionMultiply()));
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
    connect(actionAlign,                        SIGNAL(triggered()),    this,       SLOT(doAction4pcsRegister())); //Aurelien BEY le 13/11/2008
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
    //"Tools > Other" menu
    connect(actionDensity,                      SIGNAL(triggered()),    this,       SLOT(doComputeDensity()));
    connect(actionCurvature,                    SIGNAL(triggered()),    this,       SLOT(doComputeCurvature()));
    connect(actionRoughness,                    SIGNAL(triggered()),    this,       SLOT(doComputeRoughness()));
    connect(actionSNETest,						SIGNAL(triggered()),    this,       SLOT(doSphericalNeighbourhoodExtractionTest()));
    connect(actionPlaneOrientation,				SIGNAL(triggered()),    this,       SLOT(doComputePlaneOrientation()));

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
    connect(actionSetViewTop,                   SIGNAL(triggered()),    this,       SLOT(setTopView()));
    connect(actionSetViewBottom,				SIGNAL(triggered()),    this,       SLOT(setBottomView()));
    connect(actionSetViewFront,                 SIGNAL(triggered()),    this,       SLOT(setFrontView()));
    connect(actionSetViewBack,                  SIGNAL(triggered()),    this,       SLOT(setBackView()));
    connect(actionSetViewLeft,					SIGNAL(triggered()),    this,       SLOT(setLeftView()));
    connect(actionSetViewRight,		            SIGNAL(triggered()),    this,       SLOT(setRightView()));

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

    unsigned i,selNum = m_selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = m_selectedEntities[i];
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

    refreshAll();
	updateUI();
}

void MainWindow::doActionSetColorGradient()
{
    ccTwoColorsDlg dlg(this);
    if (!dlg.exec())
        return;

    unsigned char dim = (unsigned char)dlg.directionComboBox->currentIndex();
    bool defaultRamp = (dlg.defaultRampCheckBox->checkState()==Qt::Checked);
    colorType col1[3] = {dlg.s_firstColor.red(),
                         dlg.s_firstColor.green(),
                         dlg.s_firstColor.blue()
                        };
    colorType col2[3] = {dlg.s_secondColor.red(),
                         dlg.s_secondColor.green(),
                         dlg.s_secondColor.blue()
                        };

    unsigned i,selNum = m_selectedEntities.size();
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
            if (defaultRamp)
                static_cast<ccPointCloud*>(cloud)->colorizeWithDefaultRamp(dim);
            else
                static_cast<ccPointCloud*>(cloud)->colorizeByHeight(dim, col1, col2);
            ent->showColors(true);
            ent->prepareDisplayForRefresh();
        }
    }

    refreshAll();
	updateUI();
}

void MainWindow::doActionInvertNormals()
{
    unsigned i,selNum = m_selectedEntities.size();
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
    unsigned i,selNum = m_selectedEntities.size();
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
				ccCloud->prepareDisplayForRefresh_recursive();
			}
        }
    }

    refreshAll();
	updateUI();
}

void MainWindow::doActionComputeOctree()
{
    ccProgressDialog pDlg(true,this);

	//we must backup 'm_selectedEntities' as removeObjectTemporarilyFromDBTree can modify it!
	ccHObject::Container selectedEntities = m_selectedEntities;
    unsigned i,selNum = selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccGenericPointCloud* cloud = 0;
        ccHObject* ent = selectedEntities[i];
        /*if (ent->isKindOf(CC_MESH)) //TODO
            cloud = static_cast<ccGenericMesh*>(ent)->getAssociatedCloud();
        else */
        if (ent->isKindOf(CC_POINT_CLOUD))
            cloud = static_cast<ccGenericPointCloud*>(ent);

        if (cloud)
        {
			//we temporarily detach entity, as it may undergo
			//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::computeOctree
			ccHObject* parent=0;
			removeObjectTemporarilyFromDBTree(cloud,parent);
			QElapsedTimer eTimer;
			eTimer.start();
            ccOctree* octree = cloud->computeOctree(&pDlg);
			int elapsedTime_ms = eTimer.elapsed();
			putObjectBackIntoDBTree(cloud,parent);
            if (octree)
			{
				ccConsole::Print("[doActionComputeOctree] Timing: %2.3f s",elapsedTime_ms/1.0e3);
                octree->setVisible(true);
				octree->prepareDisplayForRefresh();
			}
			else
			{
				ccConsole::Warning(QString("Octree computation on entity '%1' failed!").arg(ent->getName()));
			}
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
    unsigned i,selNum = selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccPointCloud* cloud = 0;
        ccHObject* ent = selectedEntities[i];
        /*if (ent->isKindOf(CC_MESH)) //TODO
            cloud = static_cast<ccGenericMesh*>(ent)->getAssociatedCloud();
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
				ccPointCloud* newCloud = new ccPointCloud(result);
				addToDB(newCloud, true, 0, false, false);
				newCloud->setDisplay(cloud->getDisplay());
				newCloud->prepareDisplayForRefresh();
				delete result;
			}
        }
    }

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
    unsigned i,selNum = selectedEntities.size();
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
    ccAskThreeDoubleValuesDlg dlg("fx","fy","fz",-1.0e6,1.0e6,s_lastMultFactorX,s_lastMultFactorY,s_lastMultFactorZ,6,"Scaling",this);
    if (!dlg.exec())
        return;

	//save values for next time
	s_lastMultFactorX = dlg.doubleSpinBox1->value();
	s_lastMultFactorY = dlg.doubleSpinBox2->value();
	s_lastMultFactorZ = dlg.doubleSpinBox3->value();

	//we must backup 'm_selectedEntities' as removeObjectTemporarilyFromDBTree can modify it!
	ccHObject::Container selectedEntities = m_selectedEntities;
    unsigned i,selNum = selectedEntities.size();
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
}

void MainWindow::doComputeBestFitBB()
{
    QMessageBox msgBox(QMessageBox::Warning, "This method is for test purpose only","Cloud(s) are going to be rotated while still displayed in their previous position! Proceed?");
    msgBox.addButton(new QPushButton("yes"), QMessageBox::YesRole);
    msgBox.addButton(new QPushButton("no"), QMessageBox::NoRole);

    if (msgBox.exec() != 0)
        return;

	//we must backup 'm_selectedEntities' as removeObjectTemporarilyFromDBTree can modify it!
	ccHObject::Container selectedEntities = m_selectedEntities;
	unsigned i,j,selNum = selectedEntities.size();
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
                        eig.getEigenValueAndVector(j,u);
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

    unsigned i,selNum = selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = selectedEntities[i];

		//specific case: clear normals on a mesh
		if (prop == 1 && ent->isKindOf(CC_MESH))
		{
			ccGenericMesh* mesh = static_cast<ccGenericMesh*>(ent);
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
					&& static_cast<ccGenericMesh*>(mesh->getParent())->getAssociatedCloud() == mesh->getAssociatedCloud())
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
    unsigned i,selNum = m_selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = m_selectedEntities[i];
        if (ent->isKindOf(CC_MESH))
        {
            double S = CCLib::MeshSamplingTools::computeMeshArea(static_cast<ccGenericMesh*>(ent));
            //we force the console to display itself
            forceConsoleDisplay();
            ccConsole::Print(QString("[Mesh Surface Measurer] Mesh %1: S=%2 (square units)").arg(ent->getName()).arg(S));
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
	QString defaultRangesSFname(squared ? "Ranges (squared)" : "Ranges");
	
	int sfIdx = cloud->getScalarFieldIndexByName(qPrintable(defaultRangesSFname));
	if (sfIdx<0)
	{
		sfIdx = cloud->addScalarField(qPrintable(defaultRangesSFname),true);
		if (sfIdx<0)
		{
			ccConsole::Error("Not enough memory!");
			return;
		}
	}
	CCLib::ScalarField* distances = cloud->getScalarField(sfIdx);

    //sensor center
    CCVector3 center = sensor->getCenter();

    if (squared)
    {
        for (unsigned i = 0; i < cloud->size(); ++i)
        {
            const CCVector3* P = cloud->getPoint(i);
			distances->setValue(i,(*P-center).norm2());
        }
    }
    else
    {
        for (unsigned i = 0; i < cloud->size(); ++i)
        {
            const CCVector3* P = cloud->getPoint(i);
			distances->setValue(i,(*P-center).norm());
        }
    }

	distances->computeMinAndMax();
	cloud->setCurrentDisplayedScalarField(sfIdx);
	cloud->showSF(true);

    cloud->prepareDisplayForRefresh_recursive();
    refreshAll();

	if (m_ccRoot)
        m_ccRoot->updatePropertiesView();
}

void MainWindow::doActionComputeScatteringAngles()
{
    //there should be only one sensor in current selection!
    if (m_selectedEntities.empty() || m_selectedEntities.size()>1 || !m_selectedEntities[0]->isKindOf(CC_SENSOR))
    {
        ccConsole::Error("Select one and only one sensor!");
        return;
    }

	ccSensor* sensor = ccHObjectCaster::ToSensor(m_selectedEntities[0]);
	assert(sensor);

    //sensor must have a parent cloud with normal
    if (!sensor->getParent() || !sensor->getParent()->isKindOf(CC_POINT_CLOUD) || !sensor->getParent()->hasNormals())
    {
        ccConsole::Error("Sensor must be associated with a point cloud with normals!\nCompute normals before the scattering angles");
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
	#define DEFAULT_SCAT_ANGLES_SF_NAME "Scattering angles"
	int sfIdx = cloud->getScalarFieldIndexByName(DEFAULT_SCAT_ANGLES_SF_NAME);
	if (sfIdx<0)
	{
		sfIdx = cloud->addScalarField(DEFAULT_SCAT_ANGLES_SF_NAME,true);
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
        float cosTheta = ray.dot(normal);
// 	float theta = acos(abs(cosTheta)); //USING abs in this way make cosTheta to be casted to int!
					   //one may include cmath and use std::abs for performing abs on floats
					   //here we use a simpler solution -> check sign
	
	if (cosTheta < 0)
	  cosTheta = -cosTheta;
	
	float theta = acos(cosTheta);
	
		if (toDegreeFlag)
			theta *= (float)(CC_RAD_TO_DEG);
		angles->setValue(i,theta);
    }

    angles->computeMinAndMax();
	cloud->setCurrentDisplayedScalarField(sfIdx);
	cloud->showSF(true);

	cloud->prepareDisplayForRefresh_recursive();
    refreshAll();

	if (m_ccRoot)
        m_ccRoot->updatePropertiesView();
}

void MainWindow::doActionProjectSensor()
{
    ccSensorProjectionDlg spDlg(this);
    if (!spDlg.exec())
        return;

    //We create the corresponding sensor for each input cloud (in a perfect world, there should be only one ;)
	ccHObject::Container selectedEntities = m_selectedEntities;
    unsigned i,selNum = selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = selectedEntities[i];
        if (ent->isKindOf(CC_POINT_CLOUD))
        {
            ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(ent);

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
                    ccBBox box = cloud->getBB();
                    sensor->setDisplay(win);
                    sensor->setVisible(true);
                    win->updateConstellationCenterAndZoom(&box);
                }
                delete projectedPoints;
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
                ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(gbl->getParent());
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

    unsigned i,selNum = m_selectedEntities.size();
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

        if (multEntities)
            delete toSave;
    }
}

void MainWindow::doActionSamplePoints()
{
    ccPtsSamplingDlg dlg(this);
    if (!dlg.exec())
        return;

    ccProgressDialog pDlg(false,this);

    bool withNormals = dlg.generateNormals();
    bool withRGB = dlg.interpolateRGB();
    bool withTexture = dlg.interpolateTexture();
	bool useDensity = dlg.useDensity();

	bool withFeatures = (withNormals || withRGB || withTexture);

	ccHObject::Container selectedEntities = m_selectedEntities;

	for (unsigned i=0;i<selectedEntities.size();++i)
    {
        ccHObject* ent = selectedEntities[i];
        if (ent->isKindOf(CC_MESH))
        {
            ccGenericMesh* mesh = static_cast<ccGenericMesh*>(ent);
			assert(mesh);

            CCLib::GenericIndexedCloud* sampledCloud = 0;
			GenericChunkedArray<1,unsigned>* triIndices = (withFeatures ? new GenericChunkedArray<1,unsigned> : 0);

            if (useDensity)
            {
				sampledCloud = CCLib::MeshSamplingTools::samplePointsOnMesh(mesh,dlg.getDensityValue(),&pDlg,triIndices);
            }
            else
            {
				sampledCloud = CCLib::MeshSamplingTools::samplePointsOnMesh(mesh,dlg.getPointsNumber(),&pDlg,triIndices);
            }

            if (sampledCloud)
            {
				//convert to real point cloud
                ccPointCloud* cloud = new ccPointCloud(sampledCloud);
                delete sampledCloud;
				sampledCloud=0;

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
								mesh->getColorFromTexture(triIndex,*P,C,withRGB);
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
                addToDB(cloud,true,0,false,false);
            }

			if (triIndices)
				triIndices->release();
        }
    }

    refreshAll();
}

void MainWindow::doActionFilterByValue()
{
	ccHObject::Container selectedEntities = m_selectedEntities;
    unsigned i,selNum = selectedEntities.size();

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
            //la méthode est activée sur le champ scalaire affiché
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

    double minVald=0.0;
    double maxVald=1.0;

    if (toFilter.empty())
        return;

    //compute min and max "displayed" scalar values of currently selected
    //entities (the ones with an active scalar field only!)
    bool negativeSF = false;
    for (i=0;i<toFilter.size();++i)
    {
        ccScalarField* sf = toFilter[i].second->getCurrentDisplayedScalarField();
        assert(sf);

        if (!sf->isPositive())
            negativeSF = true;

        if (i==0)
        {
            minVald=(double)sf->getMinDisplayed();
            maxVald=(double)sf->getMaxDisplayed();
        }
        else
        {
            if (minVald>(double)sf->getMinDisplayed())
                minVald=(double)sf->getMinDisplayed();
            if (maxVald<(double)sf->getMaxDisplayed())
                maxVald=(double)sf->getMaxDisplayed();
        }
    }

    ccAskTwoDoubleValuesDlg dlg("Min","Max",(negativeSF ? -DOUBLE_MAX : 0.0),DOUBLE_MAX,minVald,maxVald,6,"Filter by scalar value",this);
    if (!dlg.exec())
        return;

    DistanceType minVal = (DistanceType)dlg.doubleSpinBox1->value();
    DistanceType maxVal = (DistanceType)dlg.doubleSpinBox2->value();

    ccHObject* firstResult = 0;
    for (i=0;i<toFilter.size();++i)
    {
        ccHObject* ent = toFilter[i].first;
        ccPointCloud* pc = toFilter[i].second;
        //CCLib::ScalarField* sf = pc->getCurrentDisplayedScalarField();
        //assert(sf);

        //on met en lecture (OUT) le champ scalaire actuellement affiché
        int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
        assert(outSfIdx>=0);
        pc->setCurrentOutScalarField(outSfIdx);
        //pc->setCurrentScalarField(outSfIdx);

        ccHObject* result = 0;
        if (ent->isKindOf(CC_MESH))
        {
            pc->hidePointsByScalarValue(minVal,maxVal);
            result = static_cast<ccGenericMesh*>(ent)->createNewMeshFromSelection(false);
            pc->unallocateVisibilityArray();
        }
        else if (ent->isKindOf(CC_POINT_CLOUD))
        {
            //pc->hidePointsByScalarValue(minVal,maxVal);
            //result = static_cast<ccGenericPointCloud*>(ent)->hidePointsByScalarValue(false);
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

    QMessageBox msgBox(QMessageBox::Warning, "Scalar Field to RGB", "Mix with existing colors (if relevant)?");
    msgBox.addButton(new QPushButton("yes"), QMessageBox::YesRole);
    msgBox.addButton(new QPushButton("no"), QMessageBox::NoRole);
    msgBox.addButton(QMessageBox::Cancel);

    int answer = msgBox.exec();
    if (answer == 0)
        mixWithExistingColors=true;
    else if (answer == 2) //cancel
        return;

    unsigned i,selNum = m_selectedEntities.size();
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
                pc->setColorWithDistances(mixWithExistingColors);
                ent->showColors(true);
                ent->showSF(false);
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
    unsigned selNum = m_selectedEntities.size();
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
	if (m_ccRoot)
		m_ccRoot->updatePropertiesView();
}

void MainWindow::doActionRenameSF()
{
    unsigned i,selNum = m_selectedEntities.size();
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

PointCoordinateType MainWindow::GetDefaultCloudKernelSize(const ccHObject::Container& entities)
{
	PointCoordinateType sigma = -1.0;

	unsigned i,selNum = entities.size();
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
    unsigned i,selNum = m_selectedEntities.size();
    if (selNum==0)
        return;

	double sigma = GetDefaultCloudKernelSize(m_selectedEntities);
	if (sigma<0.0)
	{
		ccConsole::Error("No elligible point cloud in selection!");
		return;
	}

	ccAskOneDoubleValueDlg dlg("Sigma", 1e-6, DOUBLE_MAX, sigma, 6, 0, this);
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

        //la méthode est activée sur le champ scalaire affiché
        CCLib::ScalarField* sf = pc->getCurrentDisplayedScalarField();
        if (sf)
        {
            //on met en lecture (OUT) le champ scalaire actuellement affiché
            int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
            assert(outSfIdx>=0);

            pc->setCurrentOutScalarField(outSfIdx);
            CCLib::ScalarField* outSF = pc->getCurrentOutScalarField();
            assert(sf);

            QString sfName = QString("%1.smooth(%2)").arg(outSF->getName()).arg(sigma);
            int sfIdx = pc->getScalarFieldIndexByName(qPrintable(sfName));
            if (sfIdx<0)
                sfIdx = pc->addScalarField(qPrintable(sfName),outSF->isPositive()); //output SF has same type as input SF
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
                CCLib::ScalarFieldTools::applyScalarFieldGaussianFilter(sigma,pc,!sf->isPositive(), -1 ,&pDlg, octree);
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
	if (m_ccRoot)
		m_ccRoot->updatePropertiesView();
}

void MainWindow::doActionSFBilateralFilter()
{
    unsigned i,selNum = m_selectedEntities.size();
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
    DistanceType range = sf_test->getMax() - sf_test->getMin();
    DistanceType scalarFieldSigma = range / 4.0f; // using 1/4 of total range


    ccAskTwoDoubleValuesDlg dlg("Spatial sigma", "Scalar sigma", 1e-6, DOUBLE_MAX, sigma, scalarFieldSigma , 6, 0, this);
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
                sfIdx = pc->addScalarField(qPrintable(sfName),outSF->isPositive()); //output SF has same type as input SF
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

                CCLib::ScalarFieldTools::applyScalarFieldGaussianFilter(sigma,pc,!sf->isPositive(), scalarFieldSigma,&pDlg,octree);
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
    if (m_ccRoot)
        m_ccRoot->updatePropertiesView();
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

    unsigned i,selNum = m_selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = m_selectedEntities[i];
        if (ent->isKindOf(CC_MESH))
        {
            ccGenericMesh* mesh = static_cast<ccGenericMesh*>(ent);
            ccGenericPointCloud* cloud = mesh->getAssociatedCloud();

            if (cloud && cloud->isA(CC_POINT_CLOUD)) //TODO
            {
                ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

                //on active le champ scalaire actuellement affiché
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

    unsigned i,selNum = m_selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = m_selectedEntities[i];
        if (ent->isKindOf(CC_MESH))
        {
            ccGenericMesh* mesh = static_cast<ccGenericMesh*>(ent);

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
    unsigned count=ccObjects.size();
    keptObjects.reserve(count);

    for (unsigned i=0;i<count;++i)
    {
        ccHObject* obj = ccObjects[i];
        assert(obj);
        for (unsigned j=0;j<count;++j)
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

    unsigned i,selNum = _selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = _selectedEntities[i];
		if (!ent)
			continue;

        //point clouds are simply added to the first selected ones
        //and then removed
        if (ent->isKindOf(CC_POINT_CLOUD))
        {
            ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(ent);

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
        modelCloud = CCLib::MeshSamplingTools::samplePointsOnMesh(static_cast<ccGenericMesh*>(model),(unsigned)100000,&pDlg);
        if (!modelCloud)
        {
            ccConsole::Error("Failed to sample points on 'model' mesh!");
            return;
        }
    }
    else
    {
        modelCloud = static_cast<ccGenericPointCloud*>(model);
    }

    //if the 'data' entity is a mesh, we need to sample points on it
    CCLib::GenericIndexedCloudPersist* dataCloud = 0;
    if (data->isKindOf(CC_MESH))
    {
        dataCloud = CCLib::MeshSamplingTools::samplePointsOnMesh(static_cast<ccGenericMesh*>(data),(unsigned)50000,&pDlg);
        if (!dataCloud)
        {
            ccConsole::Error("Failed to sample points on 'data' mesh!");
            return;
        }
    }
    else
    {
        dataCloud = static_cast<ccGenericPointCloud*>(data);
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
            dataSfIdx=pc->addScalarField("RegistrationDistances",true);
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

        ccGLMatrix transMat(transform.R,transform.T);

//#ifdef _DEBUG
		forceConsoleDisplay();
		ccConsole::Print("[Register] Resulting matrix:");
		const float* mat = transMat.data();
		ccConsole::Print("%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f",mat[0],mat[4],mat[8],mat[12],mat[1],mat[5],mat[9],mat[13],mat[2],mat[6],mat[10],mat[14],mat[3],mat[7],mat[11],mat[15]);
		ccConsole::Print("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool");
		//for (int i=0;i<4;++i)
		//{
		//	ccConsole::Print("(%6.12f\t%6.12f\t%6.12f\t%6.12f)",mat[0],mat[4],mat[8],mat[12]);
		//	++mat;
		//}
//#endif

        //cloud to move
        ccGenericPointCloud* pc = 0;

        if (data->isKindOf(CC_POINT_CLOUD))
        {
            pc = static_cast<ccGenericPointCloud*>(data);
        }
        else if (data->isKindOf(CC_MESH))
        {
            ccGenericMesh* mesh = static_cast<ccGenericMesh*>(data);
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
                static_cast<ccGenericMesh*>(data)->refreshBB();

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

//Aurelien BEY le 13/11/2008 : ajout de la fonction permettant de traiter la fonctionnalité de recalage grossier
void MainWindow::doAction4pcsRegister()
{
    if (QMessageBox::warning(this,"Work in progress","This method is still under development: are you sure you want to use it? (a crash may likely happen)",QMessageBox::Yes,QMessageBox::No) == QMessageBox::No)
        return;

    float delta, overlap;
    unsigned nbTries, nbMaxCandidates;
    ccProgressDialog pDlg(true,this);
    CCVector3 min, max;
    ccGenericPointCloud *model, *data;
    CCLib::PointProjectionTools::Transformation transform;
    CCLib::ReferenceCloud *subModel, *subData;

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

    model = static_cast<ccGenericPointCloud*>(m_selectedEntities[0]);
    data = static_cast<ccGenericPointCloud*>(m_selectedEntities[1]);

    ccAlignDlg aDlg(model, data);

    if (!aDlg.exec())
        return;

    model = aDlg.getModelObject();
    data = aDlg.getDataObject();

    //Take the correct number of points among the clouds
    subModel = aDlg.getSampledModel();
    subData = aDlg.getSampledData();

    delta = aDlg.getDelta();
    overlap = aDlg.getOverlap();
    nbTries = aDlg.getNbTries();

    nbMaxCandidates = 0;
    if (aDlg.isNumberOfCandidatesLimited())
        nbMaxCandidates = aDlg.getMaxNumberOfCandidates();

    ccPointCloud *newDataCloud=0;

    if (CCLib::FPCSRegistrationTools::RegisterClouds(subModel, subData, transform, delta, delta/2, overlap, nbTries, 5000, &pDlg, nbMaxCandidates))
    {
		//output resulting transformation matrix
		ccGLMatrix transMat(transform.R,transform.T);

		forceConsoleDisplay();
		ccConsole::Print("[Align] Resulting matrix:");
		const float* mat = transMat.data();
		ccConsole::Print("%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f\n%6.12f\t%6.12f\t%6.12f\t%6.12f",mat[0],mat[4],mat[8],mat[12],mat[1],mat[5],mat[9],mat[13],mat[2],mat[6],mat[10],mat[14],mat[3],mat[7],mat[11],mat[15]);
		ccConsole::Print("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool");

		if (data->isA(CC_POINT_CLOUD))
            newDataCloud = static_cast<ccPointCloud*>(data)->cloneThis();
        else
            newDataCloud = new ccPointCloud(data);
        if (data->getParent())
            data->getParent()->addChild(newDataCloud);
        newDataCloud->setName(data->getName()+QString(".registered"));
        newDataCloud->applyTransformation(transform);
        newDataCloud->getBoundingBox(min.u, max.u);
        newDataCloud->setDisplay(data->getDisplay());
        newDataCloud->prepareDisplayForRefresh();
        zoomOn(newDataCloud);
        addToDB(newDataCloud, true, 0, true, false);

        data->setEnabled(false);
        data->prepareDisplayForRefresh_recursive();
    }

    delete subModel;
    delete subData;

    refreshAll();
}

//Aurelien BEY le 4/12/2008 : ajout de la fonction de sous échantillonage de nuages de points
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

    ccPointCloud *newPointCloud = new ccPointCloud(sampledCloud,pointCloud);
    newPointCloud->setName(pointCloud->getName()+QString(".subsampled"));
    newPointCloud->setDisplay(pointCloud->getDisplay());
    newPointCloud->prepareDisplayForRefresh();
    if (pointCloud->getParent())
        pointCloud->getParent()->addChild(newPointCloud);
    addToDB(newPointCloud, true, 0, false, false);

    ccGLWindow* win = static_cast<ccGLWindow*>(pointCloud->getDisplay());
    if (win)
    {
        pointCloud->setEnabled(false);
        newPointCloud->setDisplay(win);
        win->refresh();
    }

    delete sampledCloud;

    refreshAll();
}

void MainWindow::doActionStatisticalTest()
{
    ccPickOneElementDlg pDlg("Distribution",0,this);
    pDlg.addElement("Gauss");
    pDlg.addElement("Weibull");
    pDlg.setDefaultIndex(0);
    if (!pDlg.exec())
        return;

    int dist = pDlg.getSelectedIndex();
    assert(dist>=0 && dist<3);

    ccStatisticalTestDlg* sDlg=0;
    switch (dist)
    {
    case 0:
        sDlg = new ccStatisticalTestDlg("mu","sigma",0,"Chi2-Test (Gauss)",this);
        break;
    case 1:
        sDlg = new ccStatisticalTestDlg("a","b","shift","Chi2-Test (Weibull)",this);
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

    double a=sDlg->getParam1();
    double b=sDlg->getParam2();
    double c=sDlg->getParam3();
    double pChi2=sDlg->getProba();
    int nn=sDlg->getNeighborsNumber();

    CCLib::GenericDistribution* distrib=0;
    switch (dist)
    {
    case 0:
    {
        CCLib::NormalDistribution* N = new CCLib::NormalDistribution();
        N->setParameters(a,b*b); //on demande à l'utilisateur sigma et il faut initialiser la distribution avec sigma2 !!!
        distrib = (CCLib::GenericDistribution*)N;
        break;
    }
    case 1:
        CCLib::WeibullDistribution* W = new CCLib::WeibullDistribution();
        W->setParameters(a,b,c);
        distrib = (CCLib::GenericDistribution*)W;
        break;
    }

    unsigned i,selNum = m_selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = m_selectedEntities[i];
        if (ent->isKindOf(CC_POINT_CLOUD))
        {
            ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(ent);

            if (cloud && cloud->isA(CC_POINT_CLOUD)) //TODO
            {
                ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

                //on met en lecture (OUT) le champ scalaire actuellement affiché
                int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
                pc->setCurrentOutScalarField(outSfIdx);

                //on met en écriture (IN) le champ des distances du Chi2 (s'il existe - on le créé sinon)
                int sfIdx = pc->getScalarFieldIndexByName(CC_CHI2_DISTANCES_DEFAULT_SF_NAME);
                if (sfIdx<0)
                    sfIdx=pc->addScalarField(CC_CHI2_DISTANCES_DEFAULT_SF_NAME,true);
                if (sfIdx<0)
                {
                    ccConsole::Error("Couldn't allocate a new scalar field for computing chi2 distances! Try to free some memory ...");
                    break;
                }
                pc->setCurrentInScalarField(sfIdx);

                //ne pas oublier de calculer l'octree si ce n'est pas déjà fait ;)
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
                bool signedDists = !pc->getCurrentInScalarField()->isPositive();
				QElapsedTimer eTimer;
				eTimer.start();
                double chi2dist = CCLib::StatisticalTestingTools::testCloudWithStatisticalModel(distrib,pc,nn,pChi2,signedDists,&pDlg,theOctree);
				ccConsole::Print("[Chi2 Test] Timing: %3.2f ms.",eTimer.elapsed()/1.0e3);
                ccConsole::Print("[Chi2 Test] %s test result = %f",CC_STATISTICAL_DISTRIBUTION_TITLES[dist],chi2dist);

                //on fait en sorte que la limite théorique de la distance du Chi2 apparaisse clairement
				ccScalarField* sf = static_cast<ccScalarField*>(pc->getCurrentInScalarField());
                sf->computeMinAndMax();
                sf->setMinDisplayed(chi2dist);
                sf->setMinSaturation(chi2dist);
                sf->setMaxSaturation(chi2dist);

                //et on affiche le champ des distances du Chi2
                pc->setCurrentDisplayedScalarField(sfIdx);
				pc->showSF(sfIdx>=0);
                pc->prepareDisplayForRefresh_recursive();
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
    ccPickOneElementDlg pDlg("Distribution",0,this);
    pDlg.addElement("Gauss");
    pDlg.addElement("Weibull");
    pDlg.setDefaultIndex(0);
    if (!pDlg.exec())
        return;

    int dist = pDlg.getSelectedIndex();
    assert(dist>=0 && dist<3);

    CCLib::GenericDistribution* distrib=0;
    switch (dist)
    {
    case 0:
        distrib = new CCLib::NormalDistribution();
        break;
    case 1:
        distrib = new CCLib::WeibullDistribution();
        break;
    }
    assert(distrib);

    unsigned i,selNum = m_selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = m_selectedEntities[i];
        ccPointCloud* pc = ccHObjectCaster::ToPointCloud(ent); //TODO

        if (pc)
        {
            //la méthode est activée sur le champ scalaire affiché
            ccScalarField* sf = pc->getCurrentDisplayedScalarField();
            if (sf)
            {
                assert(sf->isAllocated());

                //on met en lecture (OUT) le champ scalaire actuellement affiché
                int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
                pc->setCurrentOutScalarField(outSfIdx);

                double chi2dist = -1.0;
                unsigned numberOfClasses=0,finalNumberOfClasses=0;
                double* npis = 0;
                unsigned* histo = 0;
				char buffer[256];

                distrib->computeParameters(pc,!sf->isPositive());
                if (distrib->isValid())
                {
                    distrib->getTextualDescription(buffer);
                    ccConsole::Print("[Distribution fitting] %s",buffer);

                    //Auto Chi2
                    numberOfClasses = (int)ceil(sqrt((double)pc->size()));
                    histo = new unsigned[numberOfClasses];
                    npis = new double[numberOfClasses];

                    bool signedDists = !sf->isPositive();
                    chi2dist = CCLib::StatisticalTestingTools::computeAdaptativeChi2Dist(distrib,pc,0,finalNumberOfClasses,signedDists,false,histo,npis);
                }

                if (chi2dist>=0.0)
                {
                    assert(finalNumberOfClasses<=numberOfClasses);

                    ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);
                    hDlg->setWindowTitle("[Distribution fitting]");

                    ccHistogramWindow* win = hDlg->window();
                    win->setHistoValues(histo,(unsigned)numberOfClasses);
					win->setValues(sf);
                    win->setCurveValues(npis,(unsigned)numberOfClasses);
                    win->histoValuesShouldBeDestroyed(true);
                    histo=0;
                    win->curveValuesShouldBeDestroyed(true);
                    npis=0;
                    win->setInfoStr(buffer);
                    win->setMinVal(sf->getMin());
                    win->setMaxVal(sf->getMax());
                    hDlg->show();

                    ccConsole::Print("[Distribution fitting] %s Chi2 Test = %f",CC_STATISTICAL_DISTRIBUTION_TITLES[dist],chi2dist);
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

void MainWindow::doActionLabelConnectedComponents()
{
    ccLabelingDlg dlg(this);
    if (!dlg.exec())
        return;

    int octreeLevel = dlg.getOctreeLevel();
    int minNbPts = dlg.getMinPointsNb();
    bool randColors = dlg.randomColors();

    ccProgressDialog pDlg(false,this);

	ccHObject::Container selectedEntities = m_selectedEntities;
    unsigned i,selNum = selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = selectedEntities[i];
        if (ent->isKindOf(CC_POINT_CLOUD))
        {
            ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(ent);

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

                CCLib::ReferenceCloudContainer theSegmentedLists;

                //we create/activate CCs label scalar field
                int sfIdx = pc->getScalarFieldIndexByName(CC_CONNECTED_COMPONENTS_DEFAULT_LABEL_NAME);
                if (sfIdx<0)
                    sfIdx=pc->addScalarField(CC_CONNECTED_COMPONENTS_DEFAULT_LABEL_NAME,true);
                if (sfIdx<0)
                {
                    ccConsole::Error("Couldn't allocate a new scalar field for computing CC labels! Try to free some memory ...");
                    break;
                }
                pc->setCurrentScalarField(sfIdx);

                //we try to label all CCs
                if (CCLib::AutoSegmentationTools::labelConnectedComponents(cloud,octreeLevel,false,&pDlg,theOctree)>=0)
                {
                    //if successful, we extract each CC (stored in "theSegmentedLists")
                    pc->getCurrentInScalarField()->computeMinAndMax();
                    CCLib::AutoSegmentationTools::extractConnectedComponents(cloud,theSegmentedLists);
                }
                else
                {
                    ccConsole::Warning(QString("[doActionLabelConnectedComponents] Something went wrong while extracting CCs from cloud %1...").arg(cloud->getName()));
                }

                //we delete the CCs label scalar field (we don't need it anymore)
                pc->deleteScalarField(sfIdx);
                sfIdx=-1;

                //we create "real" point clouds for all CCs
                if (!theSegmentedLists.empty())
                {
                    //cloud->setVisible(true);
                    //cloud->showSF(false);

                    //we activate the currently displayed scalar field (so the new clouds will automatically be created
                    //with the corresponding values)
                    //DGM : all SF values are copied! Whatever the selected SF is...
                    //pc->setCurrentScalarField(pc->getCurrentDisplayedScalarFieldIndex());

                    //we create a new group to store all CCs
                    ccHObject* ccGroup = new ccHObject(cloud->getName()+QString(" [CCs]"));

                    int nCC = 0;
                    //for every CCs
                    while (!theSegmentedLists.empty())
                    {
                        CCLib::ReferenceCloud* ts = theSegmentedLists.back();
                        //if it has enough points
                        if ((int)ts->size()>=minNbPts)
                        {
                            //we create a new entity
                            ccPointCloud* newList = new ccPointCloud(ts,pc);
							if (newList)
							{
								//shall we colorize it with random color?
								if (randColors)
								{
									colorType col[3];
									col[0] = colorType(float(MAX_COLOR_COMP)*float(rand())/float(RAND_MAX));
									col[1] = colorType(float(MAX_COLOR_COMP)*float(rand())/float(RAND_MAX));
									col[2] = colorType(float(MAX_COLOR_COMP)*float(rand())/float(RAND_MAX));
									newList->setRGBColor(col);
									newList->showColors(true);
									newList->showSF(false);
								}

								newList->setVisible(true);
								newList->setName(QString("CC#%1").arg(nCC));

								//we add new CC to group
								ccGroup->addChild(newList);
							}
							else
							{
								ccConsole::Warning("[doActionLabelConnectedComponents] Failed to create components #%i! (not enough memory)",nCC+1);
							}
                            ++nCC;
                        }
                        delete ts;
                        theSegmentedLists.pop_back();
                    }

                    if (nCC==0)
                    {
                        delete ccGroup;
                    }
                    else
                    {
                        addToDB(ccGroup,true,0,true,false);
                    }

                    ccConsole::Print(QString("Cloud [%1]: %2 component(s)").arg(cloud->getName()).arg(nCC));

                    cloud->prepareDisplayForRefresh();
                    cloud->setEnabled(false);
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
	unsigned selNum = m_selectedEntities.size();
	for (unsigned i=0;i<selNum;++i)
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
						sfIndex = pc->addScalarField(qPrintable(defaultSFName[d]),false);
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
						sf->setPositive(false); //in case of...
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
    unsigned selNum = m_selectedEntities.size();
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

    ccHeightGridGenerationDlg dlg(this);
    if (!dlg.exec())
        return;

    bool generateCloud = dlg.generateCloud();
    bool generateImage = dlg.generateImage();
    bool generateASCII = dlg.generateASCII();

    if (!generateCloud && !generateImage && !generateASCII)
    {
        ccConsole::Error("Nothing to do! Check 'generate' checkboxes...");
        return;
    }

    //Grid step must be > 0
    double gridStep = dlg.getGridStep();
    assert(gridStep>0);

    ccProgressDialog pDlg(true,this);
    ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(ent);

    //let's rock
    ccPointCloud* outputGrid = 0;
    if (generateCloud)
        outputGrid = new ccPointCloud();

    ccHeightGridGeneration::Compute(cloud,
                                    gridStep,
									dlg.getProjectionDimension(),
                                    dlg.getTypeOfProjection(),
                                    dlg.getFillEmptyCellsStrategy(),
									dlg.getTypeOfSFInterpolation(),
                                    dlg.getCustomHeightForEmptyCells(),
                                    generateImage,
                                    generateASCII,
                                    outputGrid,
                                    &pDlg);

    //a cloud was demanded as output?
    if (outputGrid)
    {
        if (outputGrid->size()>0)
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

            cloud->prepareDisplayForRefresh_recursive();
            cloud->setEnabled(false);
            ccConsole::Warning("Previously selected entity (source) has been hidden!");

            if (win)
                win->refresh();
            updateUI();
        }
        else
        {
            delete outputGrid;
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
    unsigned i,selNum = selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = selectedEntities[i];
        if (ent->isKindOf(CC_POINT_CLOUD))
        {
            ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(ent);

            CCLib::GenericIndexedMesh* dummyMesh = CCLib::PointProjectionTools::computeTriangulation(cloud,type);

            if (dummyMesh)
            {
                ccMesh* mesh = new ccMesh(dummyMesh, cloud);
                if (mesh)
                {
                    mesh->setName(cloud->getName()+QString(".mesh"));
                    mesh->setDisplay(cloud->getDisplay());
					if (cloud->hasColors() && !cloud->hasNormals())
						mesh->showNormals(false);
                    cloud->setVisible(false);
                    cloud->addChild(mesh);
                    cloud->prepareDisplayForRefresh();
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
    unsigned i,selNum = selectedEntities.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = selectedEntities[i];
        if (ent->isKindOf(CC_POINT_CLOUD))
        {
            ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(ent);
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
                if (sfIdx<0) sfIdx=newCloud->addScalarField("Dist2Quadric",true);
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
                            DistanceType dist = (DistanceType)(a*Pc.x*Pc.x + b*Pc.y*Pc.y + c*Pc.z*Pc.z + e*Pc.x*Pc.y + f*Pc.y*Pc.z + g*Pc.x*Pc.z + l*Pc.x + m*Pc.y + n*Pc.z + d);
#else
                            DistanceType dist = Pc.u[hfZ] - (a+b*Pc.u[hfX]+c*Pc.u[hfY]+d*Pc.u[hfX]*Pc.u[hfX]+e*Pc.u[hfX]*Pc.u[hfY]+f*Pc.u[hfY]*Pc.u[hfY]);
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
    unsigned selNum=m_selectedEntities.size();
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

    ccGenericPointCloud* compCloud = static_cast<ccGenericPointCloud*>(dlg.getFirstEntity());
    ccGenericPointCloud* srcCloud = static_cast<ccGenericPointCloud*>(dlg.getSecondEntity());

    if (!compCloud->isA(CC_POINT_CLOUD)) //TODO
    {
        ccConsole::Error("Reference cloud must be a real point cloud!");
        return;
    }
    ccPointCloud* cmpPC = static_cast<ccPointCloud*>(compCloud);

    int sfIdx = cmpPC->getScalarFieldIndexByName("tempScalarField");
    if (sfIdx<0) sfIdx=cmpPC->addScalarField("tempScalarField",true);
    if (sfIdx<0)
    {
        ccConsole::Error("Couldn't allocate a new scalar field for computing distances! Try to free some memory ...");
        return;
    }
    cmpPC->setCurrentScalarField(sfIdx);
    cmpPC->enableScalarField();
    cmpPC->forEach(CCLib::ScalarFieldTools::razDistsToHiddenValue);

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
            newCloud = new ccPointCloud(&CPSet,static_cast<ccPointCloud*>(srcCloud));
        else
            newCloud = new ccPointCloud(&CPSet);

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

//#include "NewKdTree.h"
void MainWindow::doActionComputeNormals()
{
    if (m_selectedEntities.empty())
    {
        ccConsole::Error("Select at least one point cloud");
        return;
    }

    unsigned i,count=m_selectedEntities.size();
	float defaultRadius = 0.0;
	bool onlyMeshes = true;
	for (i=0;i<count;++i)
		if (!m_selectedEntities[i]->isKindOf(CC_MESH))
		{
			if (defaultRadius == 0.0 && m_selectedEntities[i]->isA(CC_POINT_CLOUD))
			{
				ccPointCloud* cloud = static_cast<ccPointCloud*>(m_selectedEntities[i]);
				defaultRadius = cloud->getBB().getDiagNorm()*0.005; //diameter=1% of the bounding box diagonal
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

			//DGM TEST: TO REMOVE
			//CCLib::NewKdTree test;
			//test.build(cloud);

			//continue;

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
				//we hide normals during process
				cloud->showNormals(false);

			for (unsigned j=0; j<normsIndexes->currentSize(); j++)
				cloud->setPointNormalIndex(j, normsIndexes->getValue(j));

			normsIndexes->release();
			normsIndexes=0;

			cloud->showNormals(true);
			cloud->prepareDisplayForRefresh();
		}
		else if (m_selectedEntities[i]->isKindOf(CC_MESH))
		{
			ccGenericMesh* mesh = static_cast<ccGenericMesh*>(m_selectedEntities[i]);
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

    unsigned level;
    ccAskOneIntValueDlg vDlg("Octree level", 1, CCLib::DgmOctree::MAX_OCTREE_LEVEL, CCLib::DgmOctree::MAX_OCTREE_LEVEL/2, "Resolve normal directions");
    if (!vDlg.exec())
        return;
    level = vDlg.getValue();

    for (unsigned i=0; i<m_selectedEntities.size(); i++)
    {
        if (!m_selectedEntities[i]->isA(CC_POINT_CLOUD))
            continue;

        ccPointCloud* cloud = static_cast<ccPointCloud*>(m_selectedEntities[i]);
        ccProgressDialog pDlg(false,this);

        if (!cloud->getOctree())
            if (!cloud->computeOctree((CCLib::GenericProgressCallback*)&pDlg))
            {
                ccConsole::Error(QString("Could not compute octree for cloud '%1'").arg(cloud->getName()));
                continue;
            }

        NormsIndexesTableType* normsIndexes = new NormsIndexesTableType;
        normsIndexes->reserve(cloud->size());
        for (unsigned j=0; j<cloud->size(); j++)
        {
            normsType index = cloud->getPointNormalIndex(j);
            normsIndexes->setValue(j, index);
        }
        ccFastMarchingForNormsDirection::ResolveNormsDirectionByFrontPropagation(cloud, normsIndexes, level, (CCLib::GenericProgressCallback*)&pDlg, cloud->getOctree());
        for (unsigned j=0; j<normsIndexes->currentSize(); j++)
            cloud->setPointNormalIndex(j, normsIndexes->getValue(j));

		normsIndexes->release();
		normsIndexes=0;

        cloud->prepareDisplayForRefresh();
    }

    refreshAll();
}

void MainWindow::doActionSynchronize()
{
    unsigned i,selNum = m_selectedEntities.size();

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
    CCVector3* pCenter=0;
    CCVector3 center;
    if (!unrollDlg.isAxisPositionAuto())
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

    QMdiSubWindow* subWindow = m_mdiArea->addSubWindow(view3D);

    connect(view3D,	SIGNAL(entitySelectionChanged(int)),		m_ccRoot,	SLOT(selectEntity(int)));
    connect(view3D,	SIGNAL(zoomChanged(float)),					this,       SLOT(updateWindowZoomChange(float)));
    connect(view3D,	SIGNAL(panChanged(float,float)),			this,       SLOT(updateWindowPanChange(float,float)));
    connect(view3D,	SIGNAL(viewMatRotated(const ccGLMatrix&)),	this,       SLOT(updateWindowOnViewMatRotation(const ccGLMatrix&)));
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
    ccGLWindow* win = (ccGLWindow*)glWindow;

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
			|| QMessageBox::question(this,"Quit","Are you sure you want to quit?",QMessageBox::Ok,QMessageBox::Cancel)!=QMessageBox::Cancel)
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
    ui.textEdit->append(QString("Version: %1").arg(ccCommon::GetCCVersion()));

	aboutDialog->exec();
}

void MainWindow::help()
{
    //QMessageBox::about(this, QString("Help"),QString("Under construction ;)"));
    QFile doc ("user_guide_CloudCompare.pdf");
    if (!doc.open(QIODevice::ReadOnly))
        QMessageBox::warning(this,  QString("User guide not found"),
                             QString("Goto http://www.danielgm.net/cc/doc/qCC") );
    else
    {
        QString program = "AcroRd32.exe";
        //QString program = "C:\\Program Files\\Adobe\\Acrobat 7.0\\Reader\\AcroRd32.exe";
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
    unsigned selNum=m_selectedEntities.size();
    if (selNum==0 || selNum>2)
    {
        ccConsole::Error("Select one or two entities (point cloud or mesh)!");
        return;
    }

	bool lockedVertices1=false;
	ccGenericPointCloud* cloud1 = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities[0],&lockedVertices1);
	bool lockedVertices2=false;
	ccGenericPointCloud* cloud2 = (m_selectedEntities[1] ? ccHObjectCaster::ToGenericPointCloud(m_selectedEntities[1],&lockedVertices2) : 0);
    if (!cloud1 && !cloud2)
    {
        ccConsole::Error("Select at least one point cloud!");
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

		aligned = static_cast<ccGenericPointCloud*>(dlg.getFirstEntity());
		reference = static_cast<ccGenericPointCloud*>(dlg.getSecondEntity());
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

	//ccGLWindow* win = getActiveGLWindow();
	//if (win)
	//	win->redraw();
}

void MainWindow::activateSegmentationMode()
{
    ccGLWindow* win = getActiveGLWindow();
    if (!win)
        return;

    unsigned i,selNum=m_selectedEntities.size();
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
        ccHObject* firstResult=0;

        unsigned i,n = m_gsTool->getNumberOfValidEntities();
		deleteHiddenPoints = m_gsTool->deleteHiddenPoints();

        for (i=0;i<n;++i)
        {
            ccHObject* anObject = m_gsTool->getEntity(i);
			
            if (anObject->isKindOf(CC_POINT_CLOUD) || anObject->isKindOf(CC_MESH))
			{
				//Special case: labels (do this before temporarily removing 'anObject' from DB!)
				bool lockedVertices;
				ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(anObject,&lockedVertices);
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
							if (label->getPoint(i).cloud == anObject)
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
				//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::createNewCloudFromVisibilitySelection(true)
				ccHObject* parent=0;
				removeObjectTemporarilyFromDBTree(anObject,parent);

				ccHObject* segmentationResult = 0;
				if (anObject->isKindOf(CC_POINT_CLOUD))
				{
					segmentationResult = static_cast<ccGenericPointCloud*>(anObject)->createNewCloudFromVisibilitySelection(true);
				}
				else if (anObject->isKindOf(CC_MESH))
				{
					segmentationResult = static_cast<ccGenericMesh*>(anObject)->createNewMeshFromSelection(true);
				}

				if (!deleteHiddenPoints) //no need to put it back if we delete it afterwards!
					putObjectBackIntoDBTree(anObject,parent);
				else
				{
					//keep original name(s)
					segmentationResult->setName(anObject->getName());
					if (anObject->isKindOf(CC_MESH) && segmentationResult->isKindOf(CC_MESH))
						static_cast<ccGenericMesh*>(segmentationResult)->getAssociatedCloud()->setName(static_cast<ccGenericMesh*>(anObject)->getAssociatedCloud()->getName());

					delete anObject;
					anObject=0;
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

void MainWindow::activateTranslateRotateMode()
{
    unsigned i,selNum=m_selectedEntities.size();
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
    /*ccGLWindow *win = new ccGLWindow(0);
    win->show();*/
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

void MainWindow::setGlobalZoom()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
        win->zoomGlobal();
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
                //m_cpeDlg->makeFrameless(); //this does not work on linux
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

void MainWindow::zoomOnSelectedEntities()
{
    ccGLWindow* win = getActiveGLWindow();
    if (!win)
        return;

    ccHObject* tempGroup = new ccHObject("TempGroup");
    unsigned i,selNum=m_selectedEntities.size();
    for (i=0; i<selNum; ++i)
    {
        if (m_selectedEntities[i]->getDisplay()==win)
            tempGroup->addChild(m_selectedEntities[i],false);
    }

    if (tempGroup->getChildrenNumber()>0)
    {
        ccBBox box = tempGroup->getBB(false, false, win);
        win->updateConstellationCenterAndZoom(&box);
    }

    delete tempGroup;

    refreshAll();
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
			ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(obj);
			const CCVector3* P = cloud->getPoint(pointIndex);
			if (P)
			{
				unsigned precision = ccGui::Parameters().displayedNumPrecision;
				s_pickingWindow->displayNewMessage(QString("Point (%1,%2,%3) set as rotation center").arg(P->x,0,'f',precision).arg(P->y,0,'f',precision).arg(P->z,0,'f',precision),ccGLWindow::LOWER_LEFT_MESSAGE,true);
				s_pickingWindow->setPivotPoint(P->x,P->y,P->z);
				s_pickingWindow->setScreenPan(0,0);
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

void MainWindow::toggleSelectedEntitiesNormals()
{
	ccHObject::Container baseEntities;
	RemoveSiblings(m_selectedEntities,baseEntities);
    for (unsigned i=0; i<baseEntities.size(); ++i)
    {
		baseEntities[i]->toggleNormals_recursive();
		baseEntities[i]->prepareDisplayForRefresh_recursive();
    }

    refreshAll();
}

void MainWindow::toggleSelectedEntitiesColors()
{
	ccHObject::Container baseEntities;
	RemoveSiblings(m_selectedEntities,baseEntities);
    for (unsigned i=0; i<baseEntities.size(); ++i)
    {
		baseEntities[i]->toggleColors_recursive();
		baseEntities[i]->prepareDisplayForRefresh_recursive();
    }

    refreshAll();
}

void MainWindow::toggleSelectedEntitiesSF()
{
	ccHObject::Container baseEntities;
	RemoveSiblings(m_selectedEntities,baseEntities);
    for (unsigned i=0; i<baseEntities.size(); ++i)
    {
		baseEntities[i]->toggleSF_recursive();
		baseEntities[i]->prepareDisplayForRefresh_recursive();
    }

    refreshAll();
}

void MainWindow::toggleSelectedEntitiesVisibility()
{
	ccHObject::Container baseEntities;
	RemoveSiblings(m_selectedEntities,baseEntities);
    for (unsigned i=0; i<baseEntities.size(); ++i)
    {
        baseEntities[i]->toggleVisibility_recursive();
        baseEntities[i]->prepareDisplayForRefresh_recursive();
    }

    refreshAll();
}

void MainWindow::showSelectedEntitiesHistogram()
{
    unsigned selNum=m_selectedEntities.size();
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
                hDlg->setWindowTitle(QString("Histogram[%0.%1]").arg(cloudName).arg(sf->getName()));

                unsigned numberOfPoints = cloud->size();

				ccHistogramWindow* win = hDlg->window();
                win->setInfoStr(qPrintable(QString("[%1] (%2 pts) - %3").arg(cloudName).arg(numberOfPoints).arg(sf->getName())));
                win->setValues(sf);
                win->setNumberOfClasses(ccMin(int(sqrt(double(numberOfPoints))),128));
                win->histoValuesShouldBeDestroyed(false);
                hDlg->show();
            }
        }
    }
}

void MainWindow::doActionClone()
{
	ccHObject::Container selectedEntities = m_selectedEntities;
    unsigned i,selNum=selectedEntities.size();

	ccHObject* lastClone=0;
    for (i=0;i<selNum;++i)
    {
        if (selectedEntities[i]->isKindOf(CC_POINT_CLOUD))
        {
            ccGenericPointCloud* clone = static_cast<ccGenericPointCloud*>(selectedEntities[i])->clone();
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
            ccGenericMesh* clone = static_cast<ccGenericMesh*>(selectedEntities[i])->clone();
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
    unsigned selNum = m_selectedEntities.size();
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
	unsigned trys = 0;
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

	double sfValue = QInputDialog::getDouble(this,"SF value", "value", s_constantSFValue, -2147483647, 2147483647, 6, &ok);
	if (!ok)
		return;

	int pos = cloud->addScalarField(qPrintable(sfName),false);
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

	ccLog::Print(QString("New scalar field added to %1 (constant value: %2 - not strictly positive by default)").arg(cloud->getName()).arg(sfValue));
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

    bool sf1Positive = sf1->isPositive();
    bool sf2Positive = sf2->isPositive();
    bool sfDestPositive = (sf2Positive && sf1Positive) && (op!=ccScalarFieldArithmeticDlg::MINUS);

	int sfIdx = cloud->getScalarFieldIndexByName(qPrintable(sfName));
	if (sfIdx>=0)
	{
		if (sfIdx==sf1Idx || sfIdx==sf2Idx)
		{
			ccConsole::Error(QString("Resulting scalar field will have the same name\nas one of the operand (%1)! Rename it first...").arg(sfName));
			return;
		}
		QMessageBox msgBox(QMessageBox::Warning, "Same scalar field name", "Resulting scalar field already exists! Overwrite it?", QMessageBox::Ok | QMessageBox::Cancel);
		if (msgBox.exec() != 0)
			return;
		cloud->deleteScalarField(sfIdx);
	}

    sfIdx = cloud->addScalarField(qPrintable(sfName),sfDestPositive);
    if (sfIdx<0)
    {
        ccConsole::Error("Failed to create destination scalar field! (not enough memory?)");
        return;
    }
    CCLib::ScalarField* sfDest = cloud->getScalarField(sfIdx);

	unsigned i,valCount = sf1->currentSize();
    sfDest->resize(valCount);
    assert(valCount == sf2->currentSize() && valCount == sfDest->currentSize());
    DistanceType val1,val2,val;
    DistanceType hiddenVal = (sfDestPositive ? HIDDEN_VALUE : BIG_VALUE);

    for (i=0;i<valCount;++i)
    {
        //we must handle visibility for positive scalar fields!
        val1 = sf1->getValue(i);
        if (sf1Positive && val1<0)
            val = hiddenVal;
        else
        {
            //we must handle visibility for positive scalar fields!
            val2 = sf2->getValue(i);
            if (sf2Positive && val2<0)
                val = hiddenVal;
            else
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

    entity->prepareDisplayForRefresh_recursive();

    refreshAll();
	updateUI();
}

void MainWindow::doComputePlaneOrientation()
{
	ccHObject::Container selectedEntities = m_selectedEntities;
    unsigned i,selNum=selectedEntities.size();
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
				CCVector3 N(planteTrans.getColumn(2)); //plane normal

				//the plane normal is equivalent to the 3 first coefficients
				if(N.z < 0.0)
					N *= -1.0; //always with a positive 'Z' by default!
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
				cloud->prepareDisplayForRefresh_recursive();
				addToDB(pPlane);
			}
		}
	}

	refreshAll();
	updateUI();
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
    unsigned i,selNum=entities.size();
    if (selNum<1)
        return false;

    //generic parameters
    bool sfPositive = false;
    QString sfName;

    //curvature parameters
	double curvKernelSize = -1.0;
	CCLib::Neighbourhood::CC_CURVATURE_TYPE curvType;

    //computeScalarFieldGradient parameters
    bool euclidian=false;
    bool inputSFisPositive=false;

    //computeRoughness parameters
    float roughnessKernelSize = 1.0;

    switch (algo)
    {
        case CCLIB_ALGO_DENSITY:
			{
				sfName = CC_LOCAL_DENSITY_FIELD_NAME;
				sfPositive = true;
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
				sfPositive = true;
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
					euclidian = QMessageBox::question(0,"Gradient","Is the scalar field composed of (euclidian) distances ?",QMessageBox::Yes,QMessageBox::No) == QMessageBox::Yes;
				}
				sfPositive = true;
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
					ccAskOneDoubleValueDlg dlg("Kernel size", 1e-6, DOUBLE_MAX, roughnessKernelSize, 6, 0, 0);
					if (!dlg.exec())
						return false;
					roughnessKernelSize = (float)dlg.dValueSpinBox->value();
				}
				
				sfPositive = true;
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
                        //on met en lecture (OUT) le champ scalaire actuellement affiché
                        int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
                        if (outSfIdx<0)
                        {
                            cloud=0;
                        }
                        else
                        {
                            inputSFisPositive = pc->getCurrentDisplayedScalarField()->isPositive();
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
					cloud = static_cast<ccGenericPointCloud*>(entities[i]);
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
                    sfIdx = pc->addScalarField(qPrintable(sfName),sfPositive);
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
                    result = CCLib::ScalarFieldTools::computeScalarFieldGradient(cloud,!inputSFisPositive,euclidian,false,&pDlg,octree);

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
						unsigned neihgboursCount=neighbours.size();
						extractedPoints += (double)neihgboursCount;
						for (unsigned k=0;k<neihgboursCount;++k)
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
    unsigned selNum=m_selectedEntities.size();
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

    ccGenericPointCloud* compCloud = static_cast<ccGenericPointCloud*>(dlg.getFirstEntity());
    ccGenericPointCloud* refCloud = static_cast<ccGenericPointCloud*>(dlg.getSecondEntity());

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
    unsigned selNum=m_selectedEntities.size();
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
        refMesh = static_cast<ccGenericMesh*>(m_selectedEntities[isMesh[0] ? 0 : 1]);
    }
    else
    {
        ccOrderChoiceDlg dlg(m_selectedEntities[0], "Compared",
                             m_selectedEntities[1], "Reference",
                             this);
        if (!dlg.exec())
            return;

        compEnt = dlg.getFirstEntity();
        refMesh = static_cast<ccGenericMesh*>(dlg.getSecondEntity());
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
    }
}

void MainWindow::toggleActiveWindowViewerBasedPerspective()
{
    ccGLWindow* win = getActiveGLWindow();
    if (win)
    {
        win->togglePerspective(false);
        win->redraw();
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
				ccConsole::Warning(QString("Entity '%1' will be recentered: translation=(%2,%3,%4)").arg(obj->getName()).arg(Pshift[0],0,'f',2).arg(Pshift[1],0,'f',2).arg(Pshift[2],0,'f',2));
				if (scale != 1.0)
					ccConsole::Warning(QString("Entity '%1' will be rescaled: scale=%2").arg(obj->getName().arg(scale)));

				//update 'original shift' for clouds
				if (obj->isKindOf(CC_POINT_CLOUD))
				{
					ccGenericPointCloud* pc = static_cast<ccGenericPointCloud*>(obj);
					const double* oShift = pc->getOriginalShift();
					assert(oShift);
					pc->setOriginalShift(Pshift[0]+oShift[0],Pshift[1]+oShift[1],Pshift[2]+oShift[2]);
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
	ccGLWindow* win = (ccGLWindow*)QObject::sender();

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
#ifdef CC_ULT_SUPPORT
    filters.append(CC_FILE_TYPE_FILTERS[ULT]);
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
    settings.setValue("selectedFilter",(int)fType);
    settings.endGroup();
}

void MainWindow::saveFile()
{
    unsigned selNum=m_selectedEntities.size();
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
	#ifdef CC_ULT_SUPPORT
				filters.append(CC_FILE_TYPE_FILTERS[ULT]); //Trick: here we will save an ULD file instead ;)
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

void MainWindow::updateMenus()
{
    ccGLWindow* win = getActiveGLWindow();
    bool hasMdiChild = (win != 0);
    bool hasSelectedEntities = (m_ccRoot && m_ccRoot->countSelectedEntities()>0);

    //General Menu
    menuEdit->setEnabled(hasSelectedEntities);
    menuTools->setEnabled(hasSelectedEntities);

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
    /*actionSetViewTop->setEnabled(hasMdiChild);
    actionViewBottom->setEnabled(hasMdiChild);
    actionSetViewFront->setEnabled(hasMdiChild);
    actionSetViewBack->setEnabled(hasMdiChild);
    actionSetViewLeftSide->setEnabled(hasMdiChild);
    actionSetViewRightSide->setEnabled(hasMdiChild);
    actionGlobalZoom->setEnabled(hasMdiChild);
    actionZoomAndCenter->setEnabled(hasMdiChild);
    //*/

    //oher actions
    actionSegment->setEnabled(hasMdiChild && hasSelectedEntities);
    actionTranslateRotate->setEnabled(hasMdiChild && hasSelectedEntities);
	actionPointPicking->setEnabled(hasMdiChild);
    actionPointListPicking->setEnabled(hasMdiChild);
    actionTestFrameRate->setEnabled(hasMdiChild);
    actionRenderToFile->setEnabled(hasMdiChild);
    actionToggleSunLight->setEnabled(hasMdiChild);
    actionToggleCustomLight->setEnabled(hasMdiChild);
    actionToggleCenteredPerspective->setEnabled(hasMdiChild);
    actionToggleViewerBasedPerspective->setEnabled(hasMdiChild);

    //plugins
    if (m_pluginsGroup)
    {
        foreach (QAction* act, m_pluginsGroup->actions())
        {
            ccPluginInterface *ccPlugin = getValidPlugin(act->parent());
            if (ccPlugin && ccPlugin->getType() == CC_GL_FILTER_PLUGIN)
                act->setEnabled(hasMdiChild);
        }
    }
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
            action ->setChecked(child == (QWidget*)getActiveGLWindow());
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

    //updateUI();
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

    unsigned i,selNum = selection.size();
    for (i=0;i<selNum;++i)
    {
        ccHObject* ent = selection[i];
		//m_ccRoot->expandElement(ent,false);
		if (ent->isA(CC_MESH_GROUP))
			m_ccRoot->expandElement(ent,true);
	}
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

    menuEdit->setEnabled(atLeastOneEntity);
    menuTools->setEnabled(atLeastOneEntity);
    menuGroundBasedLidar->setEnabled(atLeastOneGDBSensor);

    actionZoomAndCenter->setEnabled(atLeastOneEntity && activeWindow);
    actionSave->setEnabled(atLeastOneEntity);
    actionClone->setEnabled(atLeastOneCloud || atLeastOneMesh);
    actionDelete->setEnabled(atLeastOneEntity);
    actionSegment->setEnabled(atLeastOneEntity && activeWindow);
    actionTranslateRotate->setEnabled(atLeastOneEntity && activeWindow);
    actionShowDepthBuffer->setEnabled(atLeastOneGDBSensor);
    actionExportDepthBuffer->setEnabled(atLeastOneGDBSensor);
    actionResampleWithOctree->setEnabled(atLeastOneCloud);
    actionMultiply->setEnabled(atLeastOneEntity);
    actionApplyTransformation->setEnabled(atLeastOneEntity);
    actionComputeOctree->setEnabled(atLeastOneCloud);
    actionComputeNormals->setEnabled(atLeastOneCloud || atLeastOneMesh);
    actionSetColorGradient->setEnabled(atLeastOneCloud || atLeastOneMesh);
    actionSetUniqueColor->setEnabled(atLeastOneCloud || atLeastOneMesh);
    actionComputeMeshAA->setEnabled(atLeastOneCloud);
    actionComputeMeshLS->setEnabled(atLeastOneCloud);
    //actionComputeQuadric3D->setEnabled(atLeastOneCloud);
    actionComputeBestFitBB->setEnabled(atLeastOneEntity);
    actionDensity->setEnabled(atLeastOneCloud);
    actionCurvature->setEnabled(atLeastOneCloud);
    actionRoughness->setEnabled(atLeastOneCloud);
	actionPlaneOrientation->setEnabled(atLeastOneCloud);

    actionFilterByValue->setEnabled(atLeastOneSF);             //&& scalarField
    actionConvertToRGB->setEnabled(atLeastOneSF);              //&& scalarField
	actionRenameSF->setEnabled(atLeastOneSF);                  //&& scalarField
    actionComputeStatParams->setEnabled(atLeastOneSF);         //&& scalarField
    actionShowHistogram->setEnabled(atLeastOneSF);             //&& scalarField
    actionGaussianFilter->setEnabled(atLeastOneSF);            //&& scalarField
    actionBilateralFilter->setEnabled(atLeastOneSF);           //&& scalarField
    actionDeleteScalarField->setEnabled(atLeastOneSF);         //&& scalarField
    actionDeleteAllSF->setEnabled(atLeastOneSF);               //&& scalarField
    actionMultiplySF->setEnabled(/*TODO: atLeastOneSF*/false); //&& scalarField
    actionSFGradient->setEnabled(atLeastOneSF);                //&& scalarField

    actionSamplePoints->setEnabled(atLeastOneMesh);				//&& hasMesh
    actionMeasureMeshSurface->setEnabled(atLeastOneMesh);		//&& hasMesh
	actionSmoothMesh_Laplacian->setEnabled(atLeastOneMesh);		//&& hasMesh

    //menuMeshScalarField->setEnabled(atLeastOneSF && atLeastOneMesh);         //&& scalarField
    actionSmoothMeshSF->setEnabled(atLeastOneSF && atLeastOneMesh);            //&& scalarField
    actionEnhanceMeshSF->setEnabled(atLeastOneSF && atLeastOneMesh);           //&& scalarField

    actionResolveNormalsDirection->setEnabled(atLeastOneCloud && atLeastOneNormal);    //&& hasNormals
    actionClearNormals->setEnabled(atLeastOneNormal);               //&& hasNormals
    actionInvertNormals->setEnabled(atLeastOneNormal);              //&& hasNormals
    actionConvertNormalToHSV->setEnabled(atLeastOneNormal);         //&& hasNormals
    actionColorize->setEnabled(atLeastOneCloud || atLeastOneMesh);  //&& colored
    actionClearColor->setEnabled(atLeastOneColor);                  //&& colored


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
    actionStatisticalTest->setEnabled(exactlyOneEntity && exactlyOneSF);        //&& scalarField
	actionAddConstantSF->setEnabled(exactlyOneCloud || exactlyOneMesh);

	actionKMeans->setEnabled(/*TODO: exactlyOneEntity && exactlyOneSF*/false);                 //&& scalarField
    actionFrontPropagation->setEnabled(/*TODO: exactlyOneEntity && exactlyOneSF*/false);       //&& scalarField
	
	menuActiveScalarField->setEnabled((exactlyOneCloud || exactlyOneMesh) && selInfo.sfCount>0);

	actionPointListPicking->setEnabled(exactlyOneEntity);

	//==2
    bool exactlyTwoEntities = (selInfo.selCount==2);
    bool exactlyTwoClouds = (selInfo.cloudCount==2);
    //bool exactlyTwoSF = (selInfo.sfCount==2);

    actionRegister->setEnabled(exactlyTwoEntities);
	actionPointPairsAlign->setEnabled(exactlyTwoEntities);
    actionAlign->setEnabled(exactlyTwoEntities); //Aurelien BEY le 13/11/2008
    actionSubsample->setEnabled(exactlyOneCloud); //Aurelien BEY le 4/12/2008
    actionCloudCloudDist->setEnabled(exactlyTwoClouds);
    actionCloudMeshDist->setEnabled(exactlyTwoEntities && atLeastOneMesh);      //at least one Mesh!
    actionCPS->setEnabled(exactlyTwoClouds);
    actionScalarFieldArithmetic->setEnabled(exactlyOneEntity && atLeastOneSF);

    //>1
    bool atLeastTwoEntities = (selInfo.selCount>1);

    actionFuse->setEnabled(atLeastTwoEntities);
    actionSynchronize->setEnabled(atLeastTwoEntities);

    //plugins
    if (m_pluginsGroup)
    {
        foreach (QAction* act, m_pluginsGroup->actions())
        {
            ccPluginInterface *ccPlugin = getValidPlugin(act->parent());
            if (ccPlugin && ccPlugin->getType() == CC_STD_PLUGIN)
                act->setEnabled(static_cast<ccStdPluginInterface*>(ccPlugin)->onNewSelection(m_selectedEntities));
        }
    }
}

void MainWindow::updateWindowZoomChange(float zoomFactor)
{
    if (checkBoxCameraLink->checkState() != Qt::Checked)
        return;

    ccGLWindow* sendingWindow = static_cast<ccGLWindow*>(sender());

    QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
    for (int i = 0; i < windows.size(); ++i)
    {
        ccGLWindow *child = static_cast<ccGLWindow*>(windows.at(i)->widget());
        if (child != sendingWindow)
        {
            child->updateZoom(zoomFactor);
            child->redraw();
        }
    }
}

void MainWindow::updateWindowPanChange(float ddx, float ddy)
{
    if (checkBoxCameraLink->checkState() != Qt::Checked)
        return;

    ccGLWindow* sendingWindow = static_cast<ccGLWindow*>(sender());

    QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
    for (int i = 0; i < windows.size(); ++i)
    {
        ccGLWindow *child = static_cast<ccGLWindow*>(windows.at(i)->widget());
        if (child != sendingWindow)
        {
            child->updateScreenPan(ddx,ddy);
            child->redraw();
        }
    }
}

void MainWindow::updateWindowOnViewMatRotation(const ccGLMatrix& rotMat)
{
    if (checkBoxCameraLink->checkState() != Qt::Checked)
        return;

    ccGLWindow* sendingWindow = static_cast<ccGLWindow*>(sender());

    QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
    for (int i = 0; i < windows.size(); ++i)
    {
        ccGLWindow *child = static_cast<ccGLWindow*>(windows.at(i)->widget());
        if (child != sendingWindow)
        {
            child->rotateViewMat(rotMat);
            child->redraw();
        }
    }
}

void MainWindow::dispToConsole(const char* message, ConsoleMessageLevel level/*=STD_CONSOLE_MESSAGE*/)
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

void MainWindow::doActionMultiplySF()  //TODO
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

ccHObject* MainWindow::dbRoot()
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
	//PRIMITIVES TEST - TODO FIXME
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