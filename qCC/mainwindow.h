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

#ifndef CC_MAIN_WINDOW_HEADER
#define CC_MAIN_WINDOW_HEADER

//Qt
#include <QMainWindow>

//Local
#include "ccEntityAction.h"
#include "ccMainAppInterface.h"
#include "ccPickingListener.h"

//CCLib
#include <AutoSegmentationTools.h>

class QAction;
class QMdiArea;
class QMdiSubWindow;
class QToolBar;
class QToolButton;

class cc3DMouseManager;
class ccCameraParamEditDlg;
class ccClippingBoxTool;
class ccComparisonDlg;
class ccDBRoot;
class ccDrawableObject;
class ccGamepadManager;
class ccGLWindow;
class ccGraphicalSegmentationTool;
class ccGraphicalTransformationTool;
class ccHObject;
class ccOverlayDialog;
class ccPluginUIManager;
class ccPointListPickingDlg;
class ccPointPairRegistrationDlg;
class ccPointPropertiesDlg;
class ccPrimitiveFactoryDlg;
class ccRecentFiles;
class ccSectionExtractionTool;
class ccStdPluginInterface;
class ccTracePolylineTool;

struct dbTreeSelectionInfo;

namespace Ui {
	class MainWindow;
} 

//! Main window
class MainWindow : public QMainWindow, public ccMainAppInterface, public ccPickingListener 
{
	Q_OBJECT

protected:
	//! Default constructor
	MainWindow();

	//! Default desctructor
	~MainWindow() override;
	
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
	/** \param[in,out] glWindows vector to store all sub-windows
	**/
	static void GetGLWindows(std::vector<ccGLWindow*>& glWindows);

	//! Static shortcut to MainWindow::refreshAll
	static void RefreshAllGLWindow(bool only2D = false);

	//! Static shortcut to MainWindow::updateUI
	static void UpdateUI();

	//! Deletes current main window instance
	static void DestroyInstance();

	//! Returns active GL sub-window (if any)
	ccGLWindow* getActiveGLWindow() override;
	
	//! Returns MDI area subwindow corresponding to a given 3D view
	QMdiSubWindow* getMDISubWindow(ccGLWindow* win);

	//! Returns a given views
	ccGLWindow* getGLWindow(int index) const;

	//! Returns the number of 3D views
	int getGLWindowCount() const;

	//! Tries to load several files (and then pushes them into main DB)
	/** \param filenames list of all filenames
		\param fileFilter selected file filter (i.e. type)
		\param destWin destination window (0 = active one)
	**/
	virtual void addToDB( const QStringList& filenames,
						  QString fileFilter = QString(),
						  ccGLWindow* destWin = nullptr );
	
	//inherited from ccMainAppInterface
	void addToDB( ccHObject* obj,
				  bool updateZoom = false,
				  bool autoExpandDBTree = true,
				  bool checkDimensions = false,
				  bool autoRedraw = true ) override;
	
	void registerOverlayDialog(ccOverlayDialog* dlg, Qt::Corner pos) override;
	void unregisterOverlayDialog(ccOverlayDialog* dlg) override;
	void updateOverlayDialogsPlacement() override;
	void removeFromDB(ccHObject* obj, bool autoDelete = true) override;
	void setSelectedInDB(ccHObject* obj, bool selected) override;
	void dispToConsole(QString message, ConsoleMessageLevel level = STD_CONSOLE_MESSAGE) override;
	void forceConsoleDisplay() override;
	ccHObject* dbRootObject() override;
	inline  QMainWindow* getMainWindow() override { return this; }
	inline  const ccHObject::Container& getSelectedEntities() const override { return m_selectedEntities; }
	void createGLWindow(ccGLWindow*& window, QWidget*& widget) const override;
	void destroyGLWindow(ccGLWindow*) const override;
	ccUniqueIDGenerator::Shared getUniqueIDGenerator() override;
	ccColorScalesManager* getColorScalesManager() override;
	void spawnHistogramDialog(	const std::vector<unsigned>& histoValues,
								double minVal, double maxVal,
								QString title, QString xAxisLabel) override;
	ccPickingHub* pickingHub() override { return m_pickingHub; }
	ccHObjectContext removeObjectTemporarilyFromDBTree(ccHObject* obj) override;
	void putObjectBackIntoDBTree(ccHObject* obj, const ccHObjectContext& context) override;
	
	//! Inherited from ccPickingListener
	void onItemPicked(const PickedItem& pi) override;
	
	//! Returns real 'dbRoot' object
	virtual ccDBRoot* db();

	//! Adds the "Edit Plane" action to the given menu.
	/**
	 * This is the only MainWindow UI action used externally (by ccDBRoot).
	**/
	void  addEditPlaneAction( QMenu &menu ) const;
	
	//! Sets up the UI (menus and toolbars) based on loaded plugins
	void initPlugins();

	//! Updates the 'Properties' view
	void updatePropertiesView();
	
private slots:
	//! Creates a new 3D GL sub-window
	ccGLWindow* new3DView( bool allowEntitySelection );

	//! Zooms in (current 3D view)
	void zoomIn();
	//! Zooms out (current 3D view)
	void zoomOut();

	//! Displays 'help' dialog
	void doActionShowHelpDialog();
	//! Displays file open dialog
	void doActionLoadFile();
	//! Displays file save dialog
	void doActionSaveFile();
	//! Displays the Global Shift settings dialog
	void doActionGlobalShiftSeetings();
	//! Toggles the 'show Qt warnings in Console' option
	void doEnableQtWarnings(bool);

	//! Clones currently selected entities
	void doActionClone();

	//! Updates entities display target when a gl sub-window is deleted
	/** \param glWindow the window that is going to be delete
	**/
	void prepareWindowDeletion(QObject* glWindow);

	//! Slot called when the exclusive fullscreen mode is toggled on a window
	void onExclusiveFullScreenToggled(bool);

	//inherited from ccMainAppInterface
	void freezeUI(bool state) override;
	void redrawAll(bool only2D = false) override;
	void refreshAll(bool only2D = false) override;
	void enableAll() override;
	void disableAll() override;
	void disableAllBut(ccGLWindow* win) override;
	void updateUI() override;
	
	virtual void toggleActiveWindowStereoVision(bool);
	void toggleActiveWindowCenteredPerspective() override;
	void toggleActiveWindowCustomLight() override;
	void toggleActiveWindowSunLight() override;
	void toggleActiveWindowViewerBasedPerspective() override;
	void zoomOnSelectedEntities() override;
	void setGlobalZoom() override;
	
	void increasePointSize() override;
	void decreasePointSize() override;
	
	void toggleLockRotationAxis();
	void doActionEnableBubbleViewMode();
	void setPivotAlwaysOn();
	void setPivotRotationOnly();
	void setPivotOff();
	void toggleActiveWindowAutoPickRotCenter(bool);
	void toggleActiveWindowShowCursorCoords(bool);

	//! Handles new label
	void handleNewLabel(ccHObject*);

	void setActiveSubWindow(QWidget* window);
	void showDisplayOptions();
	void showSelectedEntitiesHistogram();
	void testFrameRate();
	void toggleFullScreen(bool state);
	void toggleVisualDebugTraces();
	void toggleExclusiveFullScreen(bool state);
	void update3DViewsMenu();
	void updateMenus();
	void on3DViewActivated(QMdiSubWindow*);
	void updateUIWithSelection();
	void addToDBAuto(const QStringList& filenames);

	void echoMouseWheelRotate(float);
	void echoCameraDisplaced(float ddx, float ddy);
	void echoBaseViewMatRotation(const ccGLMatrixd& rotMat);
	void echoCameraPosChanged(const CCVector3d&);
	void echoPivotPointChanged(const CCVector3d&);
	void echoPixelSizeChanged(float);

	void doActionRenderToFile();

	//menu action
	void doActionSetUniqueColor();
	void doActionColorize();
	void doActionRGBToGreyScale();
	void doActionSetColor(bool colorize);
	void doActionSetColorGradient();
	void doActionInterpolateColors();
	void doActionChangeColorLevels();
	void doActionEnhanceRGBWithIntensities();

	void doActionSFGaussianFilter();
	void doActionSFBilateralFilter();
	void doActionSFConvertToRGB();
	void doActionSFConvertToRandomRGB();
	void doActionRenameSF();
	void doActionOpenColorScalesManager();
	void doActionAddIdField();
	void doActionSetSFAsCoord();
	void doActionInterpolateScalarFields();

	void doComputeGeometricFeature();
	void doActionSFGradient();
	void doRemoveDuplicatePoints();
	void doSphericalNeighbourhoodExtractionTest(); //DGM TODO: remove after test
	void doCylindricalNeighbourhoodExtractionTest(); //DGM TODO: remove after test
	void doActionFitPlane();
	void doActionFitSphere();
	void doActionFitFacet();
	void doActionFitQuadric();
	void doShowPrimitiveFactory();

	void doActionComputeNormals();
	void doActionInvertNormals();
	void doActionConvertNormalsToHSV();
	void doActionConvertNormalsToDipDir();
	void doActionComputeOctree();
	void doActionComputeKdTree();
	void doActionApplyTransformation();
	void doActionMerge();
	void doActionRegister();
	void doAction4pcsRegister(); //Aurelien BEY le 13/11/2008
	void doActionSubsample(); //Aurelien BEY le 4/12/2008
	void doActionStatisticalTest();
	void doActionSamplePointsOnMesh();
	void doActionSamplePointsOnPolyline();
	void doActionConvertTextureToColor();
	void doActionLabelConnectedComponents();
	void doActionComputeStatParams();
	void doActionFilterByValue();
	
	// Picking opeations
	void enablePickingOperation(ccGLWindow* win, QString message);
	void cancelPreviousPickingOperation(bool aborted);

	// For rotation center picking
	void doPickRotationCenter();
	// For leveling
	void doLevel();
	
	void doActionCreatePlane();
	void doActionEditPlane();

	void doActionDeleteScanGrids();
	void doActionSmoothMeshSF();
	void doActionEnhanceMeshSF();
	void doActionAddConstantSF();
	void doActionScalarFieldArithmetic();
	void doActionScalarFieldFromColor();
	void doActionOrientNormalsFM();
	void doActionOrientNormalsMST();
	void doActionResampleWithOctree();
	void doActionComputeMeshAA();
	void doActionComputeMeshLS();
	void doActionMeshScanGrids();
	void doActionComputeDistanceMap();
	void doActionComputeDistToBestFitQuadric3D();
	void doActionMeasureMeshSurface();
	void doActionMeasureMeshVolume();
	void doActionFlagMeshVertices();
	void doActionSmoothMeshLaplacian();
	void doActionSubdivideMesh();
	void doActionComputeCPS();
	void doActionShowWaveDialog();
	void doActionCompressFWFData();
	void doActionKMeans();
	void doActionFrontPropagation();
	void doActionApplyScale();
	void doActionEditGlobalShiftAndScale();
	void doActionMatchBBCenters();
	void doActionMatchScales();
	void doActionSORFilter();
	void doActionFilterNoise();
	void doActionUnroll();
	void doActionCreateGBLSensor();
	void doActionCreateCameraSensor();
	void doActionModifySensor();
	void doActionProjectUncertainty();
	void doActionCheckPointsInsideFrustum();
	void doActionComputeDistancesFromSensor();
	void doActionComputeScatteringAngles();
	void doActionSetViewFromSensor();
	void doActionShowDepthBuffer();
	void doActionExportDepthBuffer();
	void doActionComputePointsVisibility();
	void doActionRasterize();
	void doCompute2HalfDimVolume();
	void doConvertPolylinesToMesh();
	void doMeshTwoPolylines();
	void doActionExportCoordToSF();
	void doComputeBestFitBB();
	void doActionCrop();

	void doActionEditCamera();
	void doActionAdjustZoom();
	void doActionSaveViewportAsCamera();
	void doActionResetGUIElementsPos();

	//Shaders & plugins
	void doActionLoadShader();
	void doActionDeleteShader();

	void doActionFindBiggestInnerRectangle();

	//Clipping box
	void activateClippingBoxMode();
	void deactivateClippingBoxMode(bool);

	//Graphical transformation
	void activateTranslateRotateMode();
	void deactivateTranslateRotateMode(bool);

	//Graphical segmentation
	void activateSegmentationMode();
	void deactivateSegmentationMode(bool);

	//Polyline tracing
	void activateTracePolylineMode();
	void deactivateTracePolylineMode(bool);

	//Section extraction
	void activateSectionExtractionMode();
	void deactivateSectionExtractionMode(bool);

	//Entities comparison
	void doActionCloudCloudDist();
	void doActionCloudMeshDist();
	void deactivateComparisonMode(int);

	//Point picking mechanism
	void activatePointPickingMode();
	void deactivatePointPickingMode(bool);

	//Point list picking mechanism
	void activatePointListPickingMode();
	void deactivatePointListPickingMode(bool);

	//Point-pair registration mechanism
	void activateRegisterPointPairTool();
	void deactivateRegisterPointPairTool(bool);

	//Current active scalar field
	void doActionToggleActiveSFColorScale();
	void doActionShowActiveSFPrevious();
	void doActionShowActiveSFNext();

	//! Removes all entities currently loaded in the DB tree
	void closeAll();

	//! Batch export some info from a set of selected clouds
	void doActionExportCloudInfo();
	//! Batch export some info from a set of selected planes
	void doActionExportPlaneInfo();

	//! Generates a matrix with the best (registration) RMS for all possible couple among the selected entities
	void doActionComputeBestICPRmsMatrix();

	//! Creates a cloud with the (bounding-box) centers of all selected entities
	void doActionCreateCloudFromEntCenters();

	inline void doActionMoveBBCenterToOrigin()    { doActionFastRegistration(MoveBBCenterToOrigin); }
	inline void doActionMoveBBMinCornerToOrigin() { doActionFastRegistration(MoveBBMinCornerToOrigin); }
	inline void doActionMoveBBMaxCornerToOrigin() { doActionFastRegistration(MoveBBMaxCornerToOrigin); }

private:
	//! Shortcut: asks the user to select one cloud
	/** \param defaultCloudEntity a cloud to select by default (optional)
		\param inviteMessage invite message (default is something like 'Please select an entity:') (optional)
		\return the selected cloud (or null if the user cancelled the operation)
	**/
	ccPointCloud* askUserToSelectACloud(ccHObject* defaultCloudEntity = nullptr, QString inviteMessage = QString());

	enum FastRegistrationMode
	{
		MoveBBCenterToOrigin,
		MoveBBMinCornerToOrigin,
		MoveBBMaxCornerToOrigin
	};

	void doActionFastRegistration(FastRegistrationMode mode);

	void toggleSelectedEntitiesProperty( ccEntityAction::TOGGLE_PROPERTY property );
	void clearSelectedEntitiesProperty( ccEntityAction::CLEAR_PROPERTY property );
	
	void setView( CC_VIEW_ORIENTATION view ) override;
	
	//! Apply transformation to the selected entities
	void applyTransformation(const ccGLMatrixd& transMat);

	//! Creates point clouds from multiple 'components'
	void createComponentsClouds(ccGenericPointCloud* cloud,
								CCLib::ReferenceCloudContainer& components,
								unsigned minPointPerComponent,
								bool randomColors,
								bool selectComponents,
								bool sortBysize = true);

	//! Saves position and state of all GUI elements
	void saveGUIElementsPos();

	void setOrthoView(ccGLWindow* win);
	void setCenteredPerspectiveView(ccGLWindow* win, bool autoRedraw = true);
	void setViewerPerspectiveView(ccGLWindow* win);
	
	void showEvent(QShowEvent* event) override;
	void closeEvent(QCloseEvent* event) override;
	void moveEvent(QMoveEvent* event) override;
	void resizeEvent(QResizeEvent* event) override;
	bool eventFilter(QObject *obj, QEvent *event) override;
	void keyPressEvent(QKeyEvent *event) override;
	
	//! Makes the window including an entity zoom on it (helper)
	void zoomOn(ccHObject* object);

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

	//! Computes the orientation of an entity
	/** Either fit a plane or a 'facet' (2D polygon)
	**/
	void doComputePlaneOrientation(bool fitFacet);

	//! Sets up any input devices (3D mouse, gamepad) and adds their menus
	void setupInputDevices();
	//! Stops input and destroys any input device handling
	void destroyInputDevices();

	//! Connects all QT actions to slots
	void connectActions();

	//! Enables menu entires based on the current selection
	void enableUIItems(dbTreeSelectionInfo& selInfo);

	//! Updates the view mode pop-menu based for a given window (or an absence of!)
	virtual void updateViewModePopUpMenu(ccGLWindow* win);

	//! Updates the pivot visibility pop-menu based for a given window (or an absence of!)
	virtual void updatePivotVisibilityPopUpMenu(ccGLWindow* win);

	//! Checks whether stereo mode can be stopped (if necessary) or not
	bool checkStereoMode(ccGLWindow* win);

	Ui::MainWindow	*m_UI;
	
	//DB & DB Tree
	ccDBRoot* m_ccRoot;

	//! Currently selected entities;
	ccHObject::Container m_selectedEntities;

	//! UI frozen state (see freezeUI)
	bool m_uiFrozen;

	//! Recent files menu
	ccRecentFiles* m_recentFiles;
	
	//! 3D mouse
	cc3DMouseManager* m_3DMouseManager;

	//! Gamepad handler
	ccGamepadManager* m_gamepadManager;

	//! View mode pop-up menu button
	QToolButton* m_viewModePopupButton;

	//! Pivot visibility pop-up menu button
	QToolButton* m_pivotVisibilityPopupButton;

	//! Flag: first time the window is made visible
	bool m_FirstShow;

	//! Point picking hub
	ccPickingHub* m_pickingHub;

	/******************************/
	/***        MDI AREA        ***/
	/******************************/

	QMdiArea* m_mdiArea;

	//! CloudCompare MDI area overlay dialogs
	struct ccMDIDialogs
	{
		ccOverlayDialog* dialog;
		Qt::Corner position;

		//! Constructor with dialog and position
		ccMDIDialogs(ccOverlayDialog* dlg, Qt::Corner pos)
			: dialog(dlg)
			, position(pos)
		{}
	};

	//! Repositions an MDI dialog at its right position
	void repositionOverlayDialog(ccMDIDialogs& mdiDlg);

	//! Registered MDI area 'overlay' dialogs
	std::vector<ccMDIDialogs> m_mdiDialogs;

	/*** dialogs ***/
	//! Camera params dialog
	ccCameraParamEditDlg* m_cpeDlg;
	//! Graphical segmentation dialog
	ccGraphicalSegmentationTool* m_gsTool;
	//! Polyline tracing tool
	ccTracePolylineTool * m_tplTool;
	//! Section extraction dialog
	ccSectionExtractionTool* m_seTool;
	//! Graphical transformation dialog
	ccGraphicalTransformationTool* m_transTool;
	//! Clipping box dialog
	ccClippingBoxTool* m_clipTool;
	//! Cloud comparison dialog
	ccComparisonDlg* m_compDlg;
	//! Point properties mode dialog
	ccPointPropertiesDlg* m_ppDlg;
	//! Point list picking
	ccPointListPickingDlg* m_plpDlg;
	//! Point-pair registration
	ccPointPairRegistrationDlg* m_pprDlg;
	//! Primitive factory dialog
	ccPrimitiveFactoryDlg* m_pfDlg;

	/*** plugins ***/
	//! Manages plugins - menus, toolbars, and the about dialog
	ccPluginUIManager	*m_pluginUIManager;
};

#endif
