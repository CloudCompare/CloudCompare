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

#ifndef CC_MAIN_WINDOW_HEADER
#define CC_MAIN_WINDOW_HEADER

//Virtual interface (for plugins)
#include "plugins/ccMainAppInterface.h"

//Qt
#include <QMainWindow>
#include <QString>
#include <QDialog>
#include <QDir>

//CCLib
#include <PointProjectionTools.h>

//GUI (generated with Qt Designer)
#include <ui_mainWindow.h>

//qCC_db
#include <ccGenericMesh.h>

//db
#include "db_tree/ccDBRoot.h"
#include "fileIO/FileIOFilter.h"

class QMdiArea;
class QSignalMapper;
class QAction;
class QActionGroup;
class ccGLWindow;
class ccHObject;
class ccComparisonDlg;
class ccGraphicalSegmentationTool;
class ccGraphicalTransformationTool;
class ccPluginInterface;
class ccPointPropertiesDlg;
class ccCameraParamEditDlg;
class ccPointListPickingDlg;
class ccPointPairRegistrationDlg;
class ccDrawableObject;
class ccOverlayDialog;
class QMdiSubWindow;

//class MainWindow 

//! Main window
class MainWindow : public QMainWindow, public ccMainAppInterface, public Ui::MainWindow
{
    Q_OBJECT

protected:

    //! Default constructor
    MainWindow();

    //! Default desctructor
    virtual ~MainWindow();

public:

    //! Returns the unique instance of this object
    static MainWindow* TheInstance();

    //! Static shortcut to MainWindow::getActiveGLWindow
    static ccGLWindow* GetActiveGLWindow();

    //! Returns a given GL sub-window (determined by its title)
    /** \param title window title
    **/
    static ccGLWindow* GetGLWindow(const QString& title);

    //! Returns all GL sub-windows
    /** \param input/output vector to store all sub-windows
    **/
    static void GetGLWindows(std::vector<ccGLWindow*>& glWindows);

    //! Static shortcut to MainWindow::refreshAll
    static void RefreshAllGLWindow();

    //! Static shortcut to MainWindow::updateUI
    static void UpdateUI();

    //! Deletes current main window instance
    static void DestroyInstance();

    //! Static shortcut to MainwWindow::addToDB
    static void AddToDB(const QString& filename, CC_FILE_TYPES fType = UNKNOWN_FILE);

    //! Returns active GL sub-window (if any)
    virtual ccGLWindow* getActiveGLWindow();

    //! Tries to load (and then adds to main db) several files
    /** \param filenames list of all filenames
        \param fType file type
		\param destWin destination window (0 = active one)
    **/
	virtual void addToDB(const QStringList& filenames, CC_FILE_TYPES fType = UNKNOWN_FILE, ccGLWindow* destWin = 0);

	//inherited from ccMainAppInterface
    virtual void addToDB(ccHObject* obj, bool autoExpandDBTree=true, const char* statusMessage=NULL, bool addToDisplay=true, bool updateZoom=true, ccGLWindow* winDest=0, bool* coordinatesTransEnabled = 0, double* coordinatesShift = 0, double* coordinatesScale = 0);
	virtual void removeFromDB(ccHObject* obj, bool autoDelete=true);
    virtual void dispToConsole(const char* message, ConsoleMessageLevel level=STD_CONSOLE_MESSAGE);
	virtual void forceConsoleDisplay();
	virtual ccHObject* dbRoot();
	virtual QMainWindow* getMainWindow() {return this;}

	//! Returns real 'dbRoot' object
	virtual ccDBRoot* db();

    /*** CCLib "standalone" algorithms ***/
	
	//CCLib algorithms handled by the 'ApplyCCLibAlgortihm' method
	enum CC_LIB_ALGORITHM { CCLIB_ALGO_DENSITY      = 0,
							CCLIB_ALGO_CURVATURE    = 1,
							CCLIB_ALGO_SF_GRADIENT  = 2,
							CCLIB_ALGO_ROUGHNESS    = 3,
							CCLIB_SPHERICAL_NEIGHBOURHOOD_EXTRACTION_TEST = 255,
	};

    static bool ApplyCCLibAlgortihm(CC_LIB_ALGORITHM algo, ccHObject::Container& entities, QWidget* parent = 0, void** additionalParameters = 0);
    
	//! Returns MDI area subwindow corresponding to a given 3D view
	QMdiSubWindow* getMDISubWindow(ccGLWindow* win);

public slots:

    //! Creates a new 3D GL sub-window
    ccGLWindow* new3DView();

protected slots:

    //! Displays 'about' dialog
    void about();
    //! Displays 'help' dialog
    void help();
    //! Displays 'about plugins' dialog
    void aboutPlugins();
    //! Displays file open dialog
    void loadFile();
    //! Displays file save dialog
    void saveFile();

    //! Clones currently selected entities
    void doActionClone();

    //! Updates entities display target when a gl sub-window is deleted
    /** \param glWindow the window that is going to be delete
    **/
    void prepareWindowDeletion(QObject* glWindow);

	//inherited from ccMainAppInterface
    virtual void freezeUI(bool state);
    virtual void redrawAll();
    virtual void enableAll();
    virtual void disableAll();
    virtual void disableAllBut(ccGLWindow* win);
    virtual void refreshAll();
    virtual void updateUI();
    virtual void setFrontView();
    virtual void setBottomView();
    virtual void setTopView();
    virtual void setBackView();
    virtual void setLeftView();
    virtual void setRightView();
    virtual void toggleActiveWindowCenteredPerspective();
    virtual void toggleActiveWindowCustomLight();
    virtual void toggleActiveWindowSunLight();
    virtual void toggleActiveWindowViewerBasedPerspective();
    virtual void zoomOnSelectedEntities();
	
	// For rotation center picking
	virtual void doPickRotationCenter();
	virtual void cancelPickRotationCenter();
	void processPickedRotationCenter(int, unsigned, int, int);

	//! Tries to load (and then adds to main db) several files
    /** \param filenames list of all filenames
    **/
    void addToDBAuto(const QStringList& filenames);

	//! Handles new label
	void handleNewEntity(ccHObject*);

    void setActiveSubWindow(QWidget* window);
    void setGlobalZoom();
    void setLightsAndMaterials();
    void showSelectedEntitiesHistogram();
    void testFrameRate();
    void toggleFullScreen(bool state);
    void update3DViewsMenu();
    void updateMenus();
    void updateUIWithSelection();
    void updateWindowZoomChange(float zoomFactor);
    void updateWindowPanChange(float ddx, float ddy);
    void updateWindowOnViewMatRotation(const ccGLMatrix& rotMat);
    void toggleSelectedEntitiesVisibility();
    void toggleSelectedEntitiesNormals();
    void toggleSelectedEntitiesColors();
    void toggleSelectedEntitiesSF();

    void doActionRenderToFile();

    //menu action
    void doActionSetUniqueColor();
    void doActionColorize();
    void doActionSetColor(bool colorize);
    void doActionSetColorGradient();

    void doActionSFGaussianFilter();
    void doActionSFBilateralFilter();
    void doActionSFConvertToRGB();
	void doActionRenameSF();

	void doComputeDensity();
    void doComputeCurvature();
    void doActionSFGradient();
    void doComputeRoughness();
	void doSphericalNeighbourhoodExtractionTest(); //DGM TODO: remove after test
	void doComputePlaneOrientation();

    void doActionComputeNormals();
    void doActionInvertNormals();
	void doActionConvertNormalsToHSV();
    void doActionComputeOctree();
    void doActionApplyTransformation();
    void doActionFuse();
    void doActionRegister();
    void doAction4pcsRegister(); //Aurelien BEY le 13/11/2008
    void doActionSubsample(); //Aurelien BEY le 4/12/2008
    void doActionStatisticalTest();
    void doActionSamplePoints();
    void doActionLabelConnectedComponents();
    void doActionComputeStatParams();
    void doActionFilterByValue();

    void doActionDeleteScalarField();
    void doActionSmoothMeshSF();
    void doActionEnhanceMeshSF();
	void doActionAddConstantSF();
    void doActionScalarFieldArithmetic();
    void doActionClearColor();
    void doActionResolveNormalsDirection();
    void doActionClearNormals();
    void doActionResampleWithOctree();
    void doActionComputeMeshAA();
    void doActionComputeMeshLS();
    void doActionComputeQuadric3D();
    void doActionMeasureMeshSurface();
    void doActionSmoothMeshLaplacian();
    void doActionComputeCPS();
    void doActionDeleteAllSF();
    void doActionMultiplySF();
    void doActionKMeans();
    void doActionFrontPropagation();
    void doActionMultiply();
    void doActionSynchronize();
    void doActionUnroll();
    void doActionProjectSensor();
    void doActionModifySensor();
    void doActionComputeDistancesFromSensor(); 
    void doActionComputeScatteringAngles(); 
    void doActionShowDepthBuffer();
    void doActionExportDepthBuffer();
    void doActionHeightGridGeneration();
	void doActionExportCoordToSF();
    void doComputeBestFitBB();


    void doActionEditCamera();
	void doActionSaveViewportAsCamera();

    void doPluginAction();

    //Graphical transformation
    void activateTranslateRotateMode();
    void deactivateTranslateRotateMode(bool);

    //Graphical segmentation
    void activateSegmentationMode();
    void deactivateSegmentationMode(bool);

    //Entities comparison
    void doActionCloudCloudDist();
    void doActionCloudMeshDist();
    void deactivateComparisonMode(int);

    //Display points properties
    void activatePointsPropertiesMode();
    void deactivatePointsPropertiesMode(bool);

    //Point list picking mechanism
    void activatePointListPickingMode();
    void deactivatePointListPickingMode(bool);

	//Point-pair registration mechanism
	void activateRegisterPointPairTool();
    void deactivateRegisterPointPairTool(bool);

    //Shaders & plugins
    void doActionLoadShader();
    void doActionDeleteShader();
    void doActionDeactivateGlFilter();

	//Current active scalar field
	void doActionToggleActiveSFColorScale();
	void doActionShowActiveSFPrevious();
	void doActionShowActiveSFNext();

protected:

    //! Removes from a list all elements that are sibling of others
    /** List is updated in place.
    **/
    static void RemoveSiblingsFromCCObjectList(ccHObject::Container& ccObjects);

	//! Removes object temporarily from DB tree
	/** This method must be called before any modification to the db tree
		WARNING: may change 'selectedEntities' container!
	**/
	void removeObjectTemporarilyFromDBTree(ccHObject* obj, ccHObject* &parent);

	//! Adds back object to DB tree
	/** This method should be called once modifications to the db tree are finished
		(see removeObjectTemporarilyFromDBTree).
	**/
	void putObjectBackIntoDBTree(ccHObject* obj, ccHObject* parent);

	//! Returns a default first guess for algorithms kernel size
	static PointCoordinateType GetDefaultCloudKernelSize(const ccHObject::Container& entities);

    void closeEvent(QCloseEvent* event);
    void moveEvent(QMoveEvent* event);
    void resizeEvent(QResizeEvent* event);
    //void keyPressEvent(QKeyEvent* event);

    void loadPlugins();
    bool addPluginToPluginGroup(QObject* plugin);
    ccPluginInterface* getValidPlugin(QObject* plugin);

    //! Makes the window including an entity zoom on it (helper)
    void zoomOn(ccDrawableObject* object);

    //! Clear property process fork
    /** - prop=0 : COLOR
        - prop=1 : NORMALS
        - prop=2 : SCALAR FIELD
        - prop=3 : ALL SCALAR FIELDS
        \param prop property id
    **/
    void doActionClearProperty(int prop);

    //! Active SF action fork
    /** - action=0 : toggle SF color scale
        - action=1 : activate previous SF
        - action=2 : activate next SF
        \param action action id
    **/
	void doApplyActiveSFAction(int action);

    //! Mesh computation fork
    /** \param type triangulation type
    **/
    void doActionComputeMesh(CC_TRIANGULATION_TYPES type);

    //! Apply a specific process to a mesh scalar field
	/** \param process process
    **/
	void doMeshSFAction(ccGenericMesh::MESH_SCALAR_FIELD_PROCESS process);

    //! Connects all QT actions to slots
    void connectActions();

    //Menu
    void enableUIItems(dbTreeSelectionInfo& selInfo);

	//! Expands DB tree for selected items
	void expandDBTreeWithSelection(ccHObject::Container& selection);

    //DB & DB Tree
    ccDBRoot* m_ccRoot;

	//! Currently selected entities;
    ccHObject::Container m_selectedEntities;

    //! UI frozen state (see freezeUI)
    bool m_uiFrozen;

    /******************************/
    /***        MDI AREA        ***/
    /******************************/

    QMdiArea* m_mdiArea;
    QSignalMapper* m_windowMapper;

    //! CloudCompare MDI area overlay dialogs
    struct ccMDIDialogs
    {
        ccOverlayDialog* dialog;
        Qt::Corner position;

        //! Constructor with dialog and position
        ccMDIDialogs(ccOverlayDialog* dlg, Qt::Corner pos)
        {
            dialog = dlg;
            position = pos;
        }
    };

    //! Replaces an MDI dialog at its right position
    void placeMDIDialog(ccMDIDialogs& mdiDlg);

    //! Registers a MDI area overlay dialog
    void registerMDIDialog(ccOverlayDialog* dlg, Qt::Corner pos);

    //! Automatically updates all registered MDI dialogs placement
    void updateMDIDialogsPlacement();

    //! Registered MDI area overlay dialogs
    std::vector<ccMDIDialogs> m_mdiDialogs;

    /*** dialogs ***/

    //! Camera params dialog
    ccCameraParamEditDlg* m_cpeDlg;
    //! Graphical segmentation dialog
    ccGraphicalSegmentationTool* m_gsTool;
    //! Graphical transformation dialog
    ccGraphicalTransformationTool* m_transTool;
    //! Cloud comparison dialog
    ccComparisonDlg* m_compDlg;
    //! Point properties mode dialog
    ccPointPropertiesDlg* m_ppDlg;
    //! Point list picking
    ccPointListPickingDlg* m_plpDlg;
    //! Point-pair registration
    ccPointPairRegistrationDlg* m_pprDlg;

    /*** plugins ***/
    QString m_pluginsPath;
    QStringList m_pluginFileNames;
    QActionGroup* m_pluginsGroup;

};

#endif
