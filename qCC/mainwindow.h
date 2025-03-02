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

// Qt
#include <QMainWindow>

// Local
#include "ccEntityAction.h"
#include "ccMainAppInterface.h"
#include "ccPickingListener.h"

// CCCoreLib
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
class ccGLWindowInterface;
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

namespace Ui
{
	class MainWindow;
}

//! Main window
/**
 * \class MainWindow
 * \brief The main window class for the application.
 *
 * This class represents the main window of the application. It inherits from QMainWindow and implements various
 * interfaces such as ccMainAppInterface and ccPickingListener. The main window contains the user interface elements and
 * handles user interactions.
 */
class MainWindow : public QMainWindow
    , public ccMainAppInterface
    , public ccPickingListener
{
	/**
     * \brief The Q_OBJECT macro is used in the declaration of a class that implements signals and slots.
     *
     * The Q_OBJECT macro must appear in the private section of a class definition that declares its own signals and
     * slots or that uses other services provided by Qt's meta-object system. It is necessary for the signals and slots
     * mechanism to work properly.
     */
	Q_OBJECT

  protected:
	/**
     * \brief Constructor for the MainWindow class.
     *
     * Initializes the MainWindow object and sets up the user interface.
     * Creates various member variables and connects signals and slots.
     * Sets the window title, populates menus, and initializes console.
     * Creates advanced widgets and sets up dynamic menus.
     * Creates ccDBRoot object for the database tree view.
     * Creates QMdiArea object for the MDI area.
     * Creates ccPickingHub object for picking functionality.
     * Restores the state of the 'auto-restore' menu entry.
     * Connects actions, creates a new 3D view, sets up input devices.
     * Unfreezes the user interface, updates the UI, and displays a status message.
     * Prints a message to the console indicating that CloudCompare has started.
     */
	MainWindow();

	/**
     * \brief Destructor for the MainWindow class.
     *
     * This destructor is responsible for cleaning up any resources used by the MainWindow object.
     * It overrides the base class destructor.
     */
	~MainWindow() override;

  public:
	/**
     * \brief Returns the instance of the MainWindow class.
     *
     * \return A pointer to the MainWindow instance.
     */
	static MainWindow* TheInstance();

	/**
     * \brief Retrieves the active GL window.
     *
     * This function returns a pointer to the active ccGLWindowInterface object.
     *
     * \return A pointer to the active GL window.
     */
	static ccGLWindowInterface* GetActiveGLWindow();

	/**
     * \brief Retrieves the ccGLWindowInterface object with the specified title.
     *
     * This function returns a pointer to the ccGLWindowInterface object that has the given title.
     *
     * \param title The title of the ccGLWindowInterface object to retrieve.
     * \return A pointer to the ccGLWindowInterface object with the specified title, or nullptr if not found.
     */
	static ccGLWindowInterface* GetGLWindow(const QString& title);

	/**
     * \brief Retrieves the GL windows.
     *
     * This function retrieves a list of GL windows and stores them in the provided vector.
     *
     * \param glWindows A reference to a vector of ccGLWindowInterface pointers where the GL windows will be stored.
     */
	static void GetGLWindows(std::vector<ccGLWindowInterface*>& glWindows);

	/**
     * \brief Refreshes all GL windows.
     *
     * This function refreshes all GL windows in the application. By default, it refreshes both 2D and 3D windows,
     * but you can specify `only2D` parameter as `true` to refresh only the 2D windows.
     *
     * \param only2D Flag indicating whether to refresh only the 2D windows. Default is `false`.
     */
	static void RefreshAllGLWindow(bool only2D = false);

	/**
     * \brief Updates the user interface.
     *
     * This function is responsible for updating the user interface of the main window.
     * It should be called whenever there are changes that require updating the UI elements.
     */
	static void UpdateUI();

	/**
     * \brief Destroys the instance of the MainWindow class.
     *
     * This static function is used to destroy the instance of the MainWindow class.
     * It should be called when the application is shutting down or when the instance is no longer needed.
     */
	static void DestroyInstance();

	/**
     * \brief Returns the active GL window interface.
     *
     * This function returns the ccGLWindowInterface object representing the active GL window.
     *
     * \return The active GL window interface.
     */
	ccGLWindowInterface* getActiveGLWindow() override;

	/**
     * \brief Retrieves the QMdiSubWindow associated with the given ccGLWindowInterface.
     *
     * This function returns the QMdiSubWindow that contains the specified ccGLWindowInterface.
     *
     * \param win The ccGLWindowInterface for which to retrieve the QMdiSubWindow.
     * \return The QMdiSubWindow associated with the ccGLWindowInterface, or nullptr if not found.
     */
	QMdiSubWindow* getMDISubWindow(ccGLWindowInterface* win);

	/**
     * \brief Retrieves the ccGLWindowInterface object at the specified index.
     *
     * This function returns a pointer to the ccGLWindowInterface object located at the given index.
     *
     * \param index The index of the ccGLWindowInterface object to retrieve.
     * \return A constant pointer to the ccGLWindowInterface object at the specified index.
     */
	ccGLWindowInterface* getGLWindow(int index) const;

	/**
     * \brief Returns the number of GL windows.
     *
     * This function returns the number of GL windows in the application.
     *
     * \return The number of GL windows.
     */
	int getGLWindowCount() const;

	/**
     * \brief Adds the specified filenames to the database.
     *
     * This function adds the filenames specified in the `filenames` parameter to the database.
     * The `fileFilter` parameter can be used to filter the files based on a specific pattern.
     * The `destWin` parameter is an optional parameter that specifies the destination window
     * where the files should be added. If no destination window is specified, the files will be
     * added to the default window.
     *
     * \param filenames The list of filenames to be added to the database.
     * \param fileFilter The filter to be applied to the filenames (optional).
     * \param destWin The destination window where the files should be added (optional).
     */
	virtual void addToDB(const QStringList& filenames, QString fileFilter = QString(), ccGLWindowInterface* destWin = nullptr);

	/**
     * \brief Adds an object to the database.
     *
     * This function adds the specified object to the database. It provides options to control the behavior of the
     * operation.
     *
     * \param obj The object to be added to the database.
     * \param updateZoom Flag indicating whether to update the zoom level.
     * \param autoExpandDBTree Flag indicating whether to automatically expand the database tree.
     * \param checkDimensions Flag indicating whether to check the dimensions of the object.
     * \param autoRedraw Flag indicating whether to automatically redraw the interface.
     */
	void addToDB(ccHObject* obj, bool updateZoom = false, bool autoExpandDBTree = true, bool checkDimensions = false, bool autoRedraw = true) override;

	/**
     * \brief Registers an overlay dialog with the specified position.
     *
     * This function registers an overlay dialog with the main window at the specified position.
     *
     * \param dlg The overlay dialog to register.
     * \param pos The position where the overlay dialog should be displayed.
     */
	void registerOverlayDialog(ccOverlayDialog* dlg, Qt::Corner pos) override;

	/**
     * \brief Unregisters an overlay dialog.
     *
     * This function is used to unregister an overlay dialog from the main window.
     *
     * \param dlg The overlay dialog to unregister.
     */
	void unregisterOverlayDialog(ccOverlayDialog* dlg) override;

	/**
     * \brief Updates the placement of overlay dialogs.
     *
     * This function is called to update the placement of overlay dialogs in the main window.
     * It overrides the base class function.
     */
	void updateOverlayDialogsPlacement() override;

	/**
     * \brief Removes the specified object from the database.
     *
     * This function removes the specified object from the database. By default, it also deletes the object from memory.
     *
     * \param obj The object to be removed.
     * \param autoDelete If set to true, the object will be deleted from memory after removal from the database.
     *                   If set to false, the object will only be removed from the database without deleting it from
     * memory.
     */
	void removeFromDB(ccHObject* obj, bool autoDelete = true) override;

	/**
     * \brief Sets the selected state of the given object in the database.
     *
     * This function is used to update the selected state of an object in the database.
     * The selected state determines whether the object is currently selected or not.
     *
     * \param obj A pointer to the ccHObject representing the object.
     * \param selected A boolean value indicating the selected state to set.
     */
	void setSelectedInDB(ccHObject* obj, bool selected) override;

	/**
     * \brief Displays a message to the console.
     *
     * This function is used to display a message to the console. The message can be of type QString.
     * The level parameter is optional and is used to specify the message level. By default, the level is set to
     * STD_CONSOLE_MESSAGE.
     *
     * \param message The message to be displayed.
     * \param level The message level (optional).
     */
	void dispToConsole(QString message, ConsoleMessageLevel level = STD_CONSOLE_MESSAGE) override;

	/**
     * \brief Forces the console display.
     *
     * This function overrides the base class function and forces the console display.
     * It is used in the MainWindow class.
     */
	void forceConsoleDisplay() override;

	/**
     * \brief Returns the root object of the database.
     *
     * This function overrides the base class function and returns the root object of the database.
     *
     * \return A pointer to the root object of the database.
     */
	ccHObject* dbRootObject() override;

	/**
     * \brief Returns the main window object.
     *
     * This function returns a pointer to the main window object.
     *
     * \return A pointer to the main window object.
     */
	inline QMainWindow* getMainWindow() override
	{
		return this;
	}

	/**
     * \brief Loads a file.
     *
     * This function loads a file specified by the given filename.
     *
     * \param filename The path of the file to be loaded.
     * \param silent   A flag indicating whether to display error messages or not.
     *
     * \return A pointer to the loaded ccHObject.
     */
	ccHObject* loadFile(QString filename, bool silent) override;

	/**
     * \brief Returns the selected entities.
     *
     * This function returns a constant reference to the container of selected entities.
     *
     * \return A constant reference to the container of selected entities.
     */
	inline const ccHObject::Container& getSelectedEntities() const override
	{
		return m_selectedEntities;
	}

	/**
     * \brief Creates a GL window and a corresponding QWidget.
     *
     * This function creates a GL window and a corresponding QWidget. The GL window is represented by the
     * `ccGLWindowInterface` pointer, and the QWidget is represented by the `QWidget` pointer. The function is marked as
     * `const` and is overridden from a base class.
     *
     * \param window A reference to a pointer to the `ccGLWindowInterface` object that will be created.
     * \param widget A reference to a pointer to the `QWidget` object that will be created.
     */
	void createGLWindow(ccGLWindowInterface*& window, QWidget*& widget) const override;

	/**
     * \brief Destroys a GL window.
     *
     * This function is called to destroy a GL window.
     *
     * \param window A pointer to the GL window interface.
     */
	void destroyGLWindow(ccGLWindowInterface*) const override;

	/**
     * \brief Returns the shared unique ID generator.
     *
     * This function returns the shared instance of the unique ID generator.
     * The unique ID generator is responsible for generating unique IDs for objects.
     *
     * \return The shared unique ID generator.
     */
	ccUniqueIDGenerator::Shared getUniqueIDGenerator() override;

	/**
     * \brief Returns the color scales manager.
     *
     * This function returns the color scales manager, which is responsible for managing the color scales used in the
     * application.
     *
     * \return A pointer to the ccColorScalesManager object.
     */
	ccColorScalesManager* getColorScalesManager() override;

	/**
     * \brief Spawns a histogram dialog.
     *
     * This function spawns a histogram dialog with the given histogram values, minimum value, maximum value,
     * title, and x-axis label.
     *
     * \param histoValues The histogram values.
     * \param minVal The minimum value.
     * \param maxVal The maximum value.
     * \param title The title of the histogram dialog.
     * \param xAxisLabel The label for the x-axis of the histogram.
     */
	void spawnHistogramDialog(const std::vector<unsigned>& histoValues, double minVal, double maxVal, QString title, QString xAxisLabel) override;

	/**
     * \brief Returns the picking hub.
     *
     * This function returns the picking hub associated with the main window.
     *
     * \return A pointer to the picking hub.
     */
	ccPickingHub* pickingHub() override
	{
		return m_pickingHub;
	}

	/**
     * \brief Removes the specified object temporarily from the database tree.
     *
     * This function removes the given object from the database tree temporarily.
     * The object will not be permanently deleted, but it will be hidden from the tree view.
     *
     * \param obj A pointer to the object to be removed.
     */
	ccHObjectContext removeObjectTemporarilyFromDBTree(ccHObject* obj) override;

	/**
     * \brief Puts the specified object back into the database tree.
     *
     * This function is called to put the given object back into the database tree
     * using the specified context.
     *
     * \param obj The object to be put back into the database tree.
     * \param context The context used to put the object back into the tree.
     */
	void putObjectBackIntoDBTree(ccHObject* obj, const ccHObjectContext& context) override;

	/**
     * \brief Handles the event when an item is picked.
     *
     * This function is called when an item is picked and provides information about the picked item.
     *
     * \param pi The picked item information.
     */
	void onItemPicked(const PickedItem& pi) override;

	/**
     * \brief Returns a container of top-level selected entities.
     *
     * This function returns a container of top-level selected entities in the current context.
     * Top-level entities are the entities that are directly selected by the user, excluding any
     * entities that are children of other selected entities.
     *
     * \return A container of top-level selected entities.
     */
	ccHObject::Container getTopLevelSelectedEntities() const;

	/**
     * \brief Returns the ccDBRoot object.
     *
     * This function returns the ccDBRoot object, which represents the database root.
     *
     * \return A pointer to the ccDBRoot object.
     */
	virtual ccDBRoot* db();

	/**
     * \brief Adds the "Edit Plane" action to the given menu.
     *
     * This function adds an action called "Edit Plane" to the specified menu.
     * The action allows the user to edit a plane.
     *
     * \param menu The menu to which the action will be added.
     */
	void addEditPlaneAction(QMenu& menu) const;

	/**
     * \brief Initializes the plugins.
     *
     * This function is responsible for initializing the plugins used in the application.
     * It should be called before using any functionality provided by the plugins.
     */
	void initPlugins();

	/**
     * \brief Updates the properties view.
     *
     * This function is responsible for updating the properties view in the main window.
     * It should be called whenever there is a change in the properties that need to be displayed.
     */
	void updatePropertiesView();

  private:
	/**
     * \brief Creates a new 3D view.
     *
     * This function creates a new 3D view by calling the internal function new3DViewInternal(true).
     *
     * \return A pointer to the newly created ccGLWindowInterface object.
     */
	ccGLWindowInterface* new3DView()
	{
		return new3DViewInternal(true);
	}

	/**
     * \brief Creates a new 3D view window.
     *
     * This function creates a new 3D view window with the specified settings.
     *
     * \param allowEntitySelection Flag indicating whether entity selection is allowed in the new view.
     * \return A pointer to the newly created ccGLWindowInterface object.
     */
	ccGLWindowInterface* new3DViewInternal(bool allowEntitySelection);

	/**
     * \brief Zooms in.
     *
     * This function is used to zoom in on the current view.
     */
	void zoomIn();

	/**
     * \brief Zooms out.
     *
     * This function is used to zoom out from the current view.
     */
	void zoomOut();

	/**
     * \brief Shows the help dialog.
     *
     * This function is responsible for displaying the help dialog to the user.
     * It provides information and instructions on how to use the application.
     */
	void doActionShowHelpDialog();

	/**
     * \brief Performs the action of loading a file.
     *
     * This function is responsible for loading a file and performing any necessary actions associated with it.
     *
     * \note This function assumes that the necessary file path has been set prior to calling it.
     */
	void doActionLoadFile();

	/**
     * \brief Performs the action of saving a file.
     *
     * This function is responsible for saving a file.
     * It performs the necessary operations to save the current state of the file.
     *
     * \note This function assumes that the file has already been opened and modified.
     *
     * \see openFile()
     * \see modifyFile()
     */
	void doActionSaveFile();

	/**
     * \brief Performs the action of saving the project.
     *
     * This function is responsible for saving the current project.
     * It performs the necessary operations to save the project data
     * to a file or a database.
     */
	void doActionSaveProject();

	/**
     * \brief Performs the global shift settings action.
     *
     * This function is responsible for executing the global shift settings action. It performs the necessary operations
     * to apply the global shift settings.
     */
	void doActionGlobalShiftSettings();

	/**
     * \brief Enables or disables Qt warnings.
     *
     * This function allows you to enable or disable Qt warnings in the application.
     *
     * \param enable A boolean value indicating whether to enable or disable the warnings.
     */
	void doEnableQtWarnings(bool enable);

	/**
     * \brief Performs the action of cloning.
     *
     * This function is responsible for performing the action of cloning.
     * It does XYZ.
     *
     * \return void
     */
	void doActionClone();

	/**
     * \brief Prepares for the deletion of a window.
     *
     * This function is responsible for preparing the deletion of a window in the application.
     * It takes a pointer to a `ccGLWindowInterface` object representing the window to be deleted.
     *
     * \param glWindow A pointer to the window to be deleted.
     */
	void prepareWindowDeletion(ccGLWindowInterface* glWindow);

	/**
     * \brief Handles the toggling of exclusive full screen mode.
     *
     * This function is called when the exclusive full screen mode is toggled.
     * It takes a boolean parameter @p state which represents the new state of the toggle.
     *
     * \param state The new state of the exclusive full screen toggle.
     */
	void onExclusiveFullScreenToggled(bool state);

	/**
     * \brief Freezes or unfreezes the user interface.
     *
     * This function is used to freeze or unfreeze the user interface based on the given state.
     * When the state is true, the user interface will be frozen and no user interaction will be allowed.
     * When the state is false, the user interface will be unfrozen and normal user interaction will be allowed.
     *
     * \param state The state indicating whether to freeze or unfreeze the user interface.
     */
	void freezeUI(bool state) override;

	/**
     * \brief Redraws all elements in the main window.
     *
     * This function is responsible for redrawing all elements in the main window.
     * By default, it redraws both 2D and 3D elements. However, if the `only2D` parameter
     * is set to `true`, it will only redraw the 2D elements.
     *
     * \param only2D Flag indicating whether to redraw only 2D elements.
     */
	void redrawAll(bool only2D = false) override;

	/**
     * \brief Refreshes all elements in the main window.
     *
     * This function refreshes all elements in the main window, including both 2D and 3D elements.
     *
     * \param only2D If set to true, only 2D elements will be refreshed. Default is false.
     */
	void refreshAll(bool only2D = false) override;
	/**
     * \brief Enables all UI elements.
     */
	void enableAll() override;

	/**
     * \brief Disables all UI elements.
     */
	void disableAll() override;

	/**
     * \brief Disables all windows except for the specified window.
     *
     * This function disables all windows in the application except for the specified window.
     *
     * \param win The window to keep enabled.
     */
	void disableAllBut(ccGLWindowInterface* win) override;

	/**
     * \brief Updates the user interface.
     */
	void updateUI() override;

	/**
     * \brief Toggles stereo vision in the active window.
     *
     * \param state If true, stereo vision will be enabled; if false, it will be disabled.
     */
	virtual void toggleActiveWindowStereoVision(bool state);

	/**
     * \brief Toggles centered perspective in the active window.
     */
	void toggleActiveWindowCenteredPerspective() override;

	/**
     * \brief Toggles custom light in the active window.
     */
	void toggleActiveWindowCustomLight() override;

	/**
     * \brief Toggles sunlight in the active window.
     */
	void toggleActiveWindowSunLight() override;

	/**
     * \brief Toggles viewer-based perspective in the active window.
     */
	void toggleActiveWindowViewerBasedPerspective() override;

	/**
     * \brief Zooms in on the selected entities.
     */
	void zoomOnSelectedEntities() override;

	/**
     * \brief Sets the global zoom level.
     */
	void setGlobalZoom() override;

	/**
     * \brief Increases the point size in the active window.
     */
	void increasePointSize() override;

	/**
     * \brief Decreases the point size in the active window.
     */
	void decreasePointSize() override;

	/**
     * \brief Toggles the lock on the rotation axis in the active window.
     */
	void toggleLockRotationAxis();

	/**
     * \brief Enables bubble view mode in the active window.
     */
	void doActionEnableBubbleViewMode();

	/**
     * \brief Sets the pivot to always be on in the active window.
     */
	void setPivotAlwaysOn();

	/**
     * \brief Sets the pivot to rotation-only mode in the active window.
     */
	void setPivotRotationOnly();

	/**
     * \brief Turns off the pivot in the active window.
     */
	void setPivotOff();

	/**
     * \brief Toggles the state of the auto pick rotation center feature in the active window.
     *
     * This function is responsible for toggling the state of the auto pick rotation center feature
     * in the active window. When the state is enabled, the rotation center will be automatically
     * picked based on the selected objects in the window.
     *
     * \param state The new state of the auto pick rotation center feature. Set to true to enable
     *              the feature, or false to disable it.
     */
	void toggleActiveWindowAutoPickRotCenter(bool state);

	/**
     * \brief Toggles the display of cursor coordinates in the active window.
     *
     * \param state If true, cursor coordinates will be shown; if false, they will be hidden.
     */
	void toggleActiveWindowShowCursorCoords(bool state);
	/**
     * \brief Handles new label.
     *
     * This function is called when a new label is created and added to the database.
     *
     * \param entity The new label object.
     */
	void handleNewLabel(ccHObject*);

	/**
     * \brief Sets the active sub-window.
     *
     * This function sets the active sub-window in the MDI area.
     *
     * \param window The window to set as the active sub-window.
     */
	void setActiveSubWindow(QWidget* window);
	void showDisplaySettings();
	void showSelectedEntitiesHistogram();

	/**
     * \brief Tests the frame rate.
     *
     * This function initiates a frame rate test on the active 3D view.
     */
	void testFrameRate();

	/**
     * \brief Toggles full-screen mode.
     *
     * This function toggles the full-screen mode of the main window.
     *
     * \param state The new state of the full-screen mode (true for full-screen, false for normal).
     */
	void toggleFullScreen(bool state);

	/**
     * \brief Toggles visual debug traces.
     *
     * This function toggles the display of visual debug traces in the active 3D view.
     */
	void toggleVisualDebugTraces();

	/**
     * \brief Toggles exclusive full-screen mode.
     *
     * This function toggles the exclusive full-screen mode of the active 3D view.
     *
     * \param state The new state of the exclusive full-screen mode (true for full-screen, false for normal).
     */
	void toggleExclusiveFullScreen(bool state);

	/**
     * \brief Updates the 3D views menu.
     *
     * This function updates the 3D views menu, adding entries for the currently open 3D views.
     */
	void update3DViewsMenu();

	/**
     * \brief Updates the menus.
     *
     * This function updates the state of the various menus based on the current selection and state of the application.
     */
	void updateMenus();

	/**
     * \brief Handles the activation of a 3D view.
     *
     * This function is called when a 3D view is activated in the MDI area.
     *
     * \param window The activated sub-window.
     */
	void on3DViewActivated(QMdiSubWindow*);

	/**
     * \brief Updates the UI with the current selection.
     *
     * This function updates the user interface to reflect the currently selected entities.
     */
	void updateUIWithSelection();

	/**
     * \brief Adds files to the database automatically.
     *
     * This function is called when files are dropped onto an active 3D view. It automatically adds the files to the
     * database.
     *
     * \param filenames The list of filenames to be added to the database.
     */
	void addToDBAuto(const QStringList& filenames);

	/**
     * \brief Echoes mouse wheel rotation.
     *
     * This function is called when the mouse wheel is rotated in an active 3D view. It echoes the rotation to the other
     * 3D views.
     *
     * \param wheelDelta_deg The rotation of the mouse wheel in degrees.
     */
	void echoMouseWheelRotate(float);

	/**
     * \brief Echoes base view matrix rotation.
     *
     * This function is called when the base view matrix is rotated in an active 3D view. It echoes the rotation to the
     * other 3D views.
     *
     * \param rotMat The new base view matrix rotation.
     */
	void echoBaseViewMatRotation(const ccGLMatrixd& rotMat);

	/**
     * \brief Echoes camera position change.
     *
     * This function is called when the camera position is changed in an active 3D view. It echoes the change to the
     * other 3D views.
     *
     * \param P The new camera position.
     */
	void echoCameraPosChanged(const CCVector3d&);

	/**
     * \brief Echoes pivot point change.
     *
     * This function is called when the pivot point is changed in an active 3D view. It echoes the change to the other
     * 3D views.
     *
     * \param P The new pivot point.
     */
	void echoPivotPointChanged(const CCVector3d&);

	/**
     * \brief Renders the scene to a file.
     *
     * This function is called to render the current 3D view to a file.
     */
	void doActionRenderToFile();

	/**
     * \brief Sets the unique color of selected entities.
     *
     * This function sets a unique color for the selected entities.
     */
	void doActionSetUniqueColor();

	/**
     * \brief Colorizes the selected entities.
     *
     * This function colorizes the selected entities.
     */
	void doActionColorize();

	/**
     * \brief Converts the RGB colors of the selected entities to grayscale.
     *
     * This function converts the RGB colors of the selected entities to grayscale.
     */
	void doActionRGBToGreyScale();

	/**
     * \brief Sets the color of the selected entities.
     *
     * This function sets the color of the selected entities. If the 'colorize' parameter is true, the color will be set
     * randomly.
     *
     * \param colorize If true, the color will be set randomly. If false, the user will be prompted to select a color.
     */
	void doActionSetColor(bool colorize);

	/**
     * \brief Sets a color gradient on the selected entities.
     *
     * This function sets a color gradient on the selected entities.
     */
	void doActionSetColorGradient();

	/**
     * \brief Interpolates the colors of the selected entities.
     *
     * This function interpolates the colors of the selected entities.
     */
	void doActionInterpolateColors();

	/**
     * \brief Changes the color levels of the selected entities.
     *
     * This function changes the color levels of the selected entities.
     */
	void doActionChangeColorLevels();

	/**
     * \brief Enhances the RGB colors of the selected entities with their intensities.
     *
     * This function enhances the RGB colors of the selected entities with their intensities.
     */
	void doActionEnhanceRGBWithIntensities();

	/**
     * \brief Colors the selected entities based on their scalar fields.
     *
     * This function colors the selected entities based on their scalar fields.
     */
	void doActionColorFromScalars();

	/**
     * \brief Applies a Gaussian filter to the RGB colors of the selected entities.
     *
     * This function applies a Gaussian filter to the RGB colors of the selected entities.
     */
	void doActionRGBGaussianFilter();

	/**
     * \brief Applies a bilateral filter to the RGB colors of the selected entities.
     *
     * This function applies a bilateral filter to the RGB colors of the selected entities.
     */
	void doActionRGBBilateralFilter();

	/**
     * \brief Applies a mean filter to the RGB colors of the selected entities.
     *
     * This function applies a mean filter to the RGB colors of the selected entities.
     */
	void doActionRGBMeanFilter();

	/**
     * \brief Applies a median filter to the RGB colors of the selected entities.
     *
     * This function applies a median filter to the RGB colors of the selected entities.
     */
	void doActionRGBMedianFilter();

	/**
     * \brief Applies a Gaussian filter to the scalar fields of the selected entities.
     *
     * This function applies a Gaussian filter to the scalar fields of the selected entities.
     */
	void doActionSFGaussianFilter();

	/**
     * \brief Applies a bilateral filter to the scalar fields of the selected entities.
     *
     * This function applies a bilateral filter to the scalar fields of the selected entities.
     */
	void doActionSFBilateralFilter();

	/**
     * \brief Converts the scalar fields of the selected entities to RGB colors.
     *
     * This function converts the scalar fields of the selected entities to RGB colors.
     */
	void doActionSFConvertToRGB();

	/**
     * \brief Converts the scalar fields of the selected entities to random RGB colors.
     *
     * This function converts the scalar fields of the selected entities to random RGB colors.
     */
	void doActionSFConvertToRandomRGB();

	/**
     * \brief Renames the scalar fields of the selected entities.
     *
     * This function allows the user to rename the scalar fields of the selected entities.
     */
	void doActionRenameSF();

	/**
     * \brief Opens the color scales manager.
     *
     * This function opens the color scales manager dialog, allowing the user to manage the available color scales.
     */
	void doActionOpenColorScalesManager();

	/**
     * \brief Adds an ID field to the selected entities.
     *
     * This function adds an ID field to the selected entities.
     */
	void doActionAddIdField();

	/**
     * \brief Splits the cloud using the current scalar field.
     *
     * This function splits the selected cloud(s) using the current scalar field.
     */
	void doActionSplitCloudUsingSF();

	/**
     * \brief Sets the scalar field as the coordinates of the selected entities.
     *
     * This function sets the scalar field as the coordinates of the selected entities.
     */
	void doActionSetSFAsCoord();

	/**
     * \brief Interpolates the scalar fields of the selected entities.
     *
     * This function interpolates the scalar fields of the selected entities.
     */
	void doActionInterpolateScalarFields();

	/**
     * \brief Computes the geometric features of the selected entities.
     *
     * This function computes various geometric features of the selected entities, such as curvature, anisotropy, etc.
     * The user can select which features to compute and the radius of the neighborhood used for the computations.
     */
	void doComputeGeometricFeature();

	/**
     * \brief Computes the scalar field gradient of the selected entities.
     *
     * This function computes the gradient of the current scalar field on the selected entities.
     */
	void doActionSFGradient();

	/**
     * \brief Removes duplicate points from the selected entities.
     *
     * This function removes duplicate points from the selected entities, based on a user-defined minimum distance
     * between points.
     */
	void doRemoveDuplicatePoints();
	void doSphericalNeighbourhoodExtractionTest();   // DGM TODO: remove after test
	void doCylindricalNeighbourhoodExtractionTest(); // DGM TODO: remove after test
	/**
     * \brief Fits a plane to the selected entities.
     *
     * This function fits a plane to the selected entities. The user can choose whether to fit an infinite plane or a
     * bounded plane (facet).
     */
	void doActionFitPlane();

	/**
     * \brief Fits a sphere to the selected entities.
     *
     * This function fits a sphere to the selected entities.
     */
	void doActionFitSphere();

	/**
     * \brief Fits a circle to the selected entities.
     *
     * This function fits a circle to the selected entities.
     */
	void doActionFitCircle();

	/**
     * \brief Fits a facet to the selected entities.
     *
     * This function fits a facet (bounded plane) to the selected entities.
     */
	void doActionFitFacet();

	/**
     * \brief Fits a quadric to the selected entities.
     *
     * This function fits a quadric surface to the selected entities.
     */
	void doActionFitQuadric();

	/**
     * \brief Shows the primitive factory dialog.
     *
     * This function displays the primitive factory dialog, allowing the user to create various geometric primitives.
     */
	void doShowPrimitiveFactory();

	/**
     * \brief Computes the normals of the selected entities.
     *
     * This function computes the normals of the selected entities.
     */
	void doActionComputeNormals();

	/**
     * \brief Inverts the normals of the selected entities.
     *
     * This function inverts the normals of the selected entities.
     */
	void doActionInvertNormals();

	/**
     * \brief Converts the normals of the selected entities to HSV colors.
     *
     * This function converts the normals of the selected entities to HSV colors.
     */
	void doActionConvertNormalsToHSV();

	/**
     * \brief Converts the normals of the selected entities to dip and dip direction.
     *
     * This function converts the normals of the selected entities to dip and dip direction.
     */
	void doActionConvertNormalsToDipDir();
	/**
     * \brief Computes the octree of the selected entities.
     *
     * This function computes the octree data structure for the selected entities.
     */
	void doActionComputeOctree();

	/**
     * \brief Computes the KD-tree of the selected entities.
     *
     * This function computes the KD-tree data structure for the selected entities.
     */
	void doActionComputeKdTree();

	/**
     * \brief Applies a transformation to the selected entities.
     *
     * This function allows the user to apply a transformation (translation, rotation, scaling) to the selected
     * entities.
     */
	void doActionApplyTransformation();

	/**
     * \brief Merges the selected entities.
     *
     * This function merges the selected entities into a single entity.
     */
	void doActionMerge();

	/**
     * \brief Registers the selected entities.
     *
     * This function performs a registration operation on the selected entities.
     */
	void doActionRegister();

	/**
     * \brief Performs a 4-points congruent sets (4PCS) registration.
     *
     * This function performs a 4-points congruent sets (4PCS) registration on the selected entities.
     */
	void doAction4pcsRegister();

	/**
     * \brief Subsamples the selected entities.
     *
     * This function subsamples the selected entities, reducing the number of points.
     */
	void doActionSubsample();

	/**
     * \brief Performs a statistical test on the selected entities.
     *
     * This function performs a statistical test on the selected entities.
     */
	void doActionStatisticalTest();

	/**
     * \brief Samples points on the selected mesh(es).
     *
     * This function samples points on the selected mesh(es).
     */
	void doActionSamplePointsOnMesh();

	/**
     * \brief Samples points on the selected polyline(s).
     *
     * This function samples points on the selected polyline(s).
     */
	void doActionSamplePointsOnPolyline();

	/**
     * \brief Smooths the selected polyline(s).
     *
     * This function smooths the selected polyline(s).
     */
	void doActionSmoohPolyline();

	/**
     * \brief Converts the texture of the selected mesh(es) to color.
     *
     * This function converts the texture of the selected mesh(es) to color.
     */
	void doActionConvertTextureToColor();

	/**
     * \brief Labels the connected components of the selected entities.
     *
     * This function labels the connected components of the selected entities.
     */
	void doActionLabelConnectedComponents();

	/**
     * \brief Computes the statistical parameters of the selected entities.
     *
     * This function computes the statistical parameters (mean, standard deviation, etc.) of the selected entities.
     */
	void doActionComputeStatParams();

	/**
     * \brief Filters the selected entities by value.
     *
     * This function allows the user to filter the selected entities by value (e.g., scalar field value).
     */
	void doActionFilterByValue();

	/**
     * \brief Enables a picking operation.
     *
     * This function enables a picking operation in the specified 3D view and displays the given message.
     *
     * \param win The 3D view window in which the picking operation will take place.
     * \param message The message to display during the picking operation.
     */
	void enablePickingOperation(ccGLWindowInterface* win, QString message);

	/**
     * \brief Cancels the previous picking operation.
     *
     * This function cancels the previous picking operation, either because the operation was aborted by the user or
     * completed successfully.
     *
     * \param aborted Indicates whether the picking operation was aborted (true) or completed successfully (false).
     */
	void cancelPreviousPickingOperation(bool aborted);

	/**
     * \brief Picks the rotation center.
     *
     * This function allows the user to pick a point in the 3D view to be used as the rotation center.
     */
	void doPickRotationCenter();

	/**
     * \brief Levels the selected entities.
     *
     * This function levels the selected entities by aligning them with the horizontal plane.
     */
	void doLevel();

	/**
     * \brief Creates a new plane.
     *
     * This function creates a new plane and adds it to the scene.
     */
	void doActionCreatePlane();

	/**
     * \brief Edits an existing plane.
     *
     * This function allows the user to edit the properties of an existing plane.
     */
	void doActionEditPlane();

	/**
     * \brief Flips the normal of the selected plane(s).
     *
     * This function flips the normal of the selected plane(s).
     */
	void doActionFlipPlane();

	/**
     * \brief Compares two planes.
     *
     * This function compares the properties of two selected planes, such as the angle between them, the distance
     * between their centers, etc.
     */
	void doActionComparePlanes();

	void doActionPromoteCircleToCylinder();

	void doActionDeleteScanGrids();

	/**
     * \brief Smooths the scalar field of the selected mesh(es).
     *
     * This function applies a smoothing operation to the scalar field of the selected mesh(es).
     */
	void doActionSmoothMeshSF();

	/**
     * \brief Enhances the scalar field of the selected mesh(es).
     *
     * This function applies an enhancement operation to the scalar field of the selected mesh(es).
     */
	void doActionEnhanceMeshSF();
	/**
     * \brief Adds a constant scalar field to the selected entities.
     *
     * This function adds a constant scalar field to the selected entities.
     */
	void doActionAddConstantSF();

	/**
     * \brief Adds a classification scalar field to the selected entities.
     *
     * This function adds a classification scalar field to the selected entities.
     */
	void doActionAddClassificationSF();

	/**
     * \brief Performs scalar field arithmetic on the selected entities.
     *
     * This function allows the user to perform arithmetic operations on the scalar fields of the selected entities.
     */
	void doActionScalarFieldArithmetic();

	/**
     * \brief Converts the color of the selected entities to a scalar field.
     *
     * This function converts the color information of the selected entities to a scalar field.
     */
	void doActionScalarFieldFromColor();

	/**
     * \brief Orients the normals of the selected entities using the Fast Marching method.
     *
     * This function orients the normals of the selected entities using the Fast Marching method.
     */
	void doActionOrientNormalsFM();

	/**
     * \brief Orients the normals of the selected entities using the Minimum Spanning Tree method.
     *
     * This function orients the normals of the selected entities using the Minimum Spanning Tree method.
     */
	void doActionOrientNormalsMST();

	/**
     * \brief Shifts the points of the selected entities along their normals.
     *
     * This function shifts the points of the selected entities along their normals by a user-specified amount.
     */
	void doActionShiftPointsAlongNormals();

	/**
     * \brief Resamples the selected entities using their octree.
     *
     * This function resamples the selected entities using their octree data structure.
     */
	void doActionResampleWithOctree();

	/**
     * \brief Computes the mesh of the selected entities using an Axis-Aligned Bounding Box.
     *
     * This function computes the mesh of the selected entities using an Axis-Aligned Bounding Box.
     */
	void doActionComputeMeshAA();

	/**
     * \brief Computes the mesh of the selected entities using a Least-Squares plane.
     *
     * This function computes the mesh of the selected entities using a Least-Squares plane.
     */
	void doActionComputeMeshLS();

	/**
     * \brief Computes the mesh of the selected scan grids.
     *
     * This function computes the mesh of the selected scan grids.
     */
	void doActionMeshScanGrids();

	/**
     * \brief Computes the distance map of the selected entities.
     *
     * This function computes the distance map of the selected entities.
     */
	void doActionComputeDistanceMap();

	/**
     * \brief Computes the distance to the best-fit 3D quadric of the selected entities.
     *
     * This function computes the distance to the best-fit 3D quadric of the selected entities.
     */
	void doActionComputeDistToBestFitQuadric3D();

	/**
     * \brief Measures the surface area of the selected mesh(es).
     *
     * This function measures the surface area of the selected mesh(es).
     */
	void doActionMeasureMeshSurface();

	/**
     * \brief Measures the volume of the selected mesh(es).
     *
     * This function measures the volume of the selected mesh(es).
     */
	void doActionMeasureMeshVolume();

	/**
     * \brief Flags the vertices of the selected mesh(es).
     *
     * This function flags the vertices of the selected mesh(es) based on their connectivity (normal, border,
     * non-manifold).
     */
	void doActionFlagMeshVertices();

	/**
     * \brief Applies Laplacian smoothing to the selected mesh(es).
     *
     * This function applies Laplacian smoothing to the selected mesh(es).
     */
	void doActionSmoothMeshLaplacian();
	/**
     * \brief Subdivides the selected mesh(es).
     *
     * This function subdivides the selected mesh(es) by adding new vertices and faces.
     */
	void doActionSubdivideMesh();

	/**
     * \brief Flips the triangles of the selected mesh(es).
     *
     * This function flips the triangles of the selected mesh(es).
     */
	void doActionFlipMeshTriangles();

	/**
     * \brief Computes the Closest Point Set (CPS) of the selected entities.
     *
     * This function computes the Closest Point Set (CPS) of the selected entities.
     */
	void doActionComputeCPS();

	/**
     * \brief Shows the waveform dialog.
     *
     * This function displays the waveform dialog for the selected point cloud(s).
     */
	void doActionShowWaveDialog();

	/**
     * \brief Compresses the full waveform data of the selected point cloud(s).
     *
     * This function compresses the full waveform data of the selected point cloud(s).
     */
	void doActionCompressFWFData();

	/**
     * \brief Performs a K-Means clustering on the selected entities.
     *
     * This function performs a K-Means clustering on the selected entities.
     */
	void doActionKMeans();

	/**
     * \brief Performs a front propagation on the selected entities.
     *
     * This function performs a front propagation on the selected entities.
     */
	void doActionFrontPropagation();

	/**
     * \brief Applies a scale to the selected entities.
     *
     * This function allows the user to apply a scale to the selected entities.
     */
	void doActionApplyScale();

	/**
     * \brief Edits the global shift and scale of the selected entities.
     *
     * This function allows the user to edit the global shift and scale of the selected entities.
     */
	void doActionEditGlobalShiftAndScale();

	/**
     * \brief Matches the bounding box centers of the selected entities.
     *
     * This function matches the bounding box centers of the selected entities.
     */
	void doActionMatchBBCenters();

	/**
     * \brief Matches the scales of the selected entities.
     *
     * This function matches the scales of the selected entities.
     */
	void doActionMatchScales();

	/**
     * \brief Applies a Statistical Outlier Removal (SOR) filter to the selected entities.
     *
     * This function applies a Statistical Outlier Removal (SOR) filter to the selected entities.
     */
	void doActionSORFilter();

	/**
     * \brief Applies a noise filter to the selected entities.
     *
     * This function applies a noise filter to the selected entities.
     */
	void doActionFilterNoise();

	/**
     * \brief Unrolls the selected entities.
     *
     * This function unrolls the selected entities, such as point clouds or meshes, onto a 2D plane.
     */
	void doActionUnroll();

	/**
     * \brief Creates a ground-based laser (GBL) sensor.
     *
     * This function creates a ground-based laser (GBL) sensor and associates it with the selected entities.
     */
	void doActionCreateGBLSensor();
	/**
     * \brief Creates a camera sensor.
     *
     * This function creates a new camera sensor and associates it with the selected entities.
     */
	void doActionCreateCameraSensor();

	/**
     * \brief Modifies an existing sensor.
     *
     * This function allows the user to modify the parameters of an existing sensor.
     */
	void doActionModifySensor();

	/**
     * \brief Projects the uncertainty of the selected camera sensor.
     *
     * This function projects the uncertainty of the selected camera sensor onto the associated point cloud.
     */
	void doActionProjectUncertainty();

	/**
     * \brief Checks the points inside the frustum of the selected camera sensor.
     *
     * This function checks which points of the associated point cloud are inside the frustum of the selected camera
     * sensor.
     */
	void doActionCheckPointsInsideFrustum();

	/**
     * \brief Computes the distances from the selected sensor.
     *
     * This function computes the distances from the selected sensor (camera or ground-based laser) to the associated
     * point cloud.
     */
	void doActionComputeDistancesFromSensor();

	/**
     * \brief Computes the scattering angles of the selected sensor.
     *
     * This function computes the scattering angles of the selected ground-based laser sensor and the associated point
     * cloud.
     */
	void doActionComputeScatteringAngles();

	/**
     * \brief Sets the view from the selected sensor.
     *
     * This function sets the view of the active 3D window based on the selected sensor.
     */
	void doActionSetViewFromSensor();

	/**
     * \brief Shows the depth buffer of the selected ground-based laser sensor.
     *
     * This function displays the depth buffer of the selected ground-based laser sensor.
     */
	void doActionShowDepthBuffer();

	/**
     * \brief Exports the depth buffer of the selected ground-based laser sensor.
     *
     * This function exports the depth buffer of the selected ground-based laser sensor to a file.
     */
	void doActionExportDepthBuffer();

	/**
     * \brief Computes the visibility of the points with respect to the selected ground-based laser sensor.
     *
     * This function computes the visibility of the points in the associated point cloud with respect to the selected
     * ground-based laser sensor.
     */
	void doActionComputePointsVisibility();

	/**
     * \brief Rasterizes the selected entities.
     *
     * This function rasterizes the selected entities (point clouds or meshes) into a grid.
     */
	void doActionRasterize();

	/**
     * \brief Computes the 2.5D volume of the selected entities.
     *
     * This function computes the 2.5D volume of the selected entities (one or two point clouds).
     */
	void doCompute2HalfDimVolume();

	/**
     * \brief Converts the selected polylines to a mesh.
     *
     * This function converts the selected polylines to a single mesh.
     */
	void doConvertPolylinesToMesh();

	/**
     * \brief Meshes two polylines.
     *
     * This function creates a mesh between two selected polylines.
     */
	void doMeshTwoPolylines();

	/**
     * \brief Exports the coordinates of the selected entities to a scalar field.
     *
     * This function exports the coordinates of the selected entities to a scalar field.
     */
	void doActionExportCoordToSF();

	/**
     * \brief Exports the normals of the selected entities to a scalar field.
     *
     * This function exports the normals of the selected entities to a scalar field.
     */
	void doActionExportNormalToSF();

	/**
     * \brief Sets the scalar fields of the selected entities as their normals.
     *
     * This function sets the scalar fields of the selected entities as their normals.
     */
	void doActionSetSFsAsNormal();
	/**
     * \brief Computes the best-fit bounding box of the selected entities.
     *
     * This function computes the best-fit bounding box of the selected entities, which may be rotated compared to the
     * original bounding box.
     */
	void doComputeBestFitBB();

	/**
     * \brief Crops the selected entities.
     *
     * This function allows the user to crop the selected entities using a bounding box.
     */
	void doActionCrop();

	/**
     * \brief Edits the camera.
     *
     * This function opens the camera parameters editing dialog for the active 3D view.
     */
	void doActionEditCamera();

	/**
     * \brief Adjusts the zoom of the active 3D view.
     *
     * This function opens a dialog that allows the user to adjust the zoom of the active 3D view.
     */
	void doActionAdjustZoom();

	/**
     * \brief Saves the current viewport as a camera.
     *
     * This function creates a new camera object that captures the current viewport settings.
     */
	void doActionSaveViewportAsCamera();

	/**
     * \brief Resets the position of the GUI elements.
     *
     * This function resets the position of the GUI elements to their default positions.
     */
	void doActionResetGUIElementsPos();

	/**
     * \brief Toggles the automatic restoration of the window on startup.
     *
     * This function toggles the automatic restoration of the main window's position and size on startup.
     *
     * \param state The new state of the automatic restoration (true to enable, false to disable).
     */
	void doActionToggleRestoreWindowOnStartup(bool);

	/**
     * \brief Resets all VBOs (Vertex Buffer Objects) associated with the selected entities.
     *
     * This function resets all VBOs associated with the selected entities, freeing up GPU memory.
     */
	void doActionResetAllVBOs();

	/**
     * \brief Loads a new shader.
     *
     * This function allows the user to load a new shader program to be used in the 3D views.
     */
	void doActionLoadShader();

	/**
     * \brief Deletes the current shader.
     *
     * This function removes the current shader program from the active 3D view.
     */
	void doActionDeleteShader();

	/**
     * \brief Performs the action to find the biggest inner rectangle.
     *
     * This function is responsible for finding the biggest inner rectangle in the current context.
     * It performs the necessary calculations and updates the necessary variables to determine the result.
     *
     * \note This function assumes that the necessary data and context have been properly set up before calling it.
     *
     * \see setContext()
     * \see updateVariables()
     *
     * \return void
     */
	void doActionFindBiggestInnerRectangle();

	// Clipping box
	/**
     * \brief Activates the clipping box mode.
     *
     * This function activates the clipping box mode in the main window.
     * In this mode, the user can define a box-shaped region to clip the point cloud data.
     * The clipped data will be displayed within the defined region.
     */
	void activateClippingBoxMode();
	/**
     * \brief Deactivates the clipping box mode.
     *
     * This function disables the clipping box mode in the application. When the clipping box mode is active, certain
     * operations or rendering effects may be limited to the area defined by the clipping box.
     *
     * \param mode The mode to set. Pass `true` to activate the clipping box mode, or `false` to deactivate it.
     */
	void deactivateClippingBoxMode(bool);

	// Graphical transformation
	/**
     * \brief Activates the translate and rotate mode.
     *
     * This function activates the translate and rotate mode in the main window.
     * In this mode, the user can perform translation and rotation operations on the selected objects.
     */
	void activateTranslateRotateMode();
	/**
     * \brief Deactivates the translate and rotate mode.
     *
     * This function is used to deactivate the translate and rotate mode in the application.
     *
     * \param[in] mode A boolean value indicating whether to deactivate the translate and rotate mode.
     */
	void deactivateTranslateRotateMode(bool);

	// Graphical segmentation
	/**
     * \brief Activates the segmentation mode.
     *
     * This function is responsible for activating the segmentation mode in the application.
     * Segmentation mode allows the user to perform segmentation operations on the data.
     */
	void activateSegmentationMode();
	/**
     * \brief Deactivates the segmentation mode.
     *
     * This function is used to deactivate the segmentation mode in the main window.
     *
     * \param[in] mode The mode to be deactivated.
     */
	void deactivateSegmentationMode(bool);

	/**
     * \brief Activates the polyline tracing mode.
     *
     * This function activates the polyline tracing mode, allowing the user to trace a polyline in the active 3D view.
     */
	void activateTracePolylineMode();

	/**
     * \brief Deactivates the polyline tracing mode.
     *
     * This function deactivates the polyline tracing mode.
     *
     * \param state Indicates whether the polyline tracing mode was successfully deactivated (true) or not (false).
     */
	void deactivateTracePolylineMode(bool);

	/**
     * \brief Activates the section extraction mode.
     *
     * This function activates the section extraction mode, allowing the user to extract sections from the selected
     * entities.
     */
	void activateSectionExtractionMode();

	/**
     * \brief Deactivates the section extraction mode.
     *
     * This function deactivates the section extraction mode.
     *
     * \param state Indicates whether the section extraction mode was successfully deactivated (true) or not (false).
     */
	void deactivateSectionExtractionMode(bool);

	/**
     * \brief Computes the cloud-to-cloud distance between the selected entities.
     *
     * This function computes the cloud-to-cloud distance between the selected entities.
     */
	void doActionCloudCloudDist();

	/**
     * \brief Computes the cloud-to-mesh distance between the selected entities.
     *
     * This function computes the cloud-to-mesh distance between the selected entities.
     */
	void doActionCloudMeshDist();

	/**
     * \brief Computes the cloud-to-primitive distance between the selected entities.
     *
     * This function computes the cloud-to-primitive distance between the selected entities.
     */
	void doActionCloudPrimitiveDist();

	/**
     * \brief Deactivates the comparison mode.
     *
     * This function deactivates the comparison mode, which was previously activated by one of the cloud/mesh/primitive
     * distance functions.
     *
     * \param result The result of the comparison (0 for success, other values for failure).
     */
	void deactivateComparisonMode(int);

	/**
     * \brief Activates the point picking mode.
     *
     * This function activates the point picking mode, allowing the user to pick individual points in the active 3D
     * view.
     */
	void activatePointPickingMode();

	/**
     * \brief Deactivates the point picking mode.
     *
     * This function deactivates the point picking mode.
     *
     * \param state Indicates whether the point picking mode was successfully deactivated (true) or not (false).
     */
	void deactivatePointPickingMode(bool);

	/**
     * \brief Activates the point list picking mode.
     *
     * This function activates the point list picking mode, allowing the user to pick a list of points in the active 3D
     * view.
     */
	void activatePointListPickingMode();

	/**
     * \brief Deactivates the point list picking mode.
     *
     * This function deactivates the point list picking mode.
     *
     * \param state Indicates whether the point list picking mode was successfully deactivated (true) or not (false).
     */
	void deactivatePointListPickingMode(bool);

	/**
     * \brief Activates the point-pair registration tool.
     *
     * This function activates the point-pair registration tool, allowing the user to perform a manual registration
     * between two entities.
     */
	void activateRegisterPointPairTool();

	/**
     * \brief Deactivates the point-pair registration tool.
     *
     * This function deactivates the point-pair registration tool.
     *
     * \param state Indicates whether the point-pair registration tool was successfully deactivated (true) or not
     * (false).
     */
	void deactivateRegisterPointPairTool(bool);

	/**
     * \brief Toggles the active scalar field color scale.
     *
     * This function toggles the display of the color scale for the active scalar field.
     */
	void doActionToggleActiveSFColorScale();

	/**
     * \brief Shows the previous active scalar field.
     *
     * This function switches to the previous active scalar field.
     */
	void doActionShowActiveSFPrevious();

	/**
     * \brief Shows the next active scalar field.
     *
     * This function switches to the next active scalar field.
     */
	void doActionShowActiveSFNext();

	/**
     * \brief Closes all entities currently loaded in the database tree.
     *
     * This function removes all entities currently loaded in the database tree.
     */
	void closeAll();

	/**
     * \brief Batch exports information from the selected clouds.
     *
     * This function exports various information (number of points, mean coordinates, scalar field statistics, etc.)
     * from the selected clouds to a CSV file.
     */
	void doActionExportCloudInfo();

	/**
     * \brief Batch exports information from the selected planes.
     *
     * This function exports various information (center, normal, dip, dip direction, etc.) from the selected planes to
     * a CSV file.
     */
	void doActionExportPlaneInfo();

	/**
     * \brief Computes the best ICP RMS matrix for all possible pairs among the selected entities.
     *
     * This function computes the best ICP (Iterative Closest Point) RMS (Root Mean Square) for all possible pairs of
     * selected entities and saves the results to a CSV file.
     */
	void doActionComputeBestICPRmsMatrix();

	/**
     * \brief Creates a cloud with the bounding box centers of the selected entities.
     *
     * This function creates a new point cloud with the bounding box centers of all selected entities.
     */
	void doActionCreateCloudFromEntCenters();

	/**
     * \brief Creates a point cloud with a single point.
     *
     * This function allows the user to create a new point cloud with a single point, whose coordinates can be
     * specified.
     */
	void createSinglePointCloud();

	/**
     * \brief Creates a point cloud from the clipboard data.
     *
     * This function creates a new point cloud from ASCII data copied to the clipboard.
     */
	void createPointCloudFromClipboard();

	/**
     * \brief Moves the bounding box center of the selected entities to the origin.
     *
     * This function applies a transformation that moves the bounding box center of the selected entities to the origin.
     */
	void doActionMoveBBCenterToOrigin();

	/**
     * \brief Moves the bounding box minimum corner of the selected entities to the origin.
     *
     * This function applies a transformation that moves the bounding box minimum corner of the selected entities to the
     * origin.
     */
	void doActionMoveBBMinCornerToOrigin();

	/**
     * \brief Moves the bounding box maximum corner of the selected entities to the origin.
     *
     * This function applies a transformation that moves the bounding box maximum corner of the selected entities to the
     * origin.
     */
	void doActionMoveBBMaxCornerToOrigin();

	/**
     * \brief Restores the position and state of all GUI elements.
     *
     * This function restores the position and state of all GUI elements, such as toolbars, dockable windows, and 3D
     * views.
     */
	void restoreGUIElementsPos();

  private:
	/**
     * \brief Asks the user to select a cloud.
     *
     * This function displays a dialog that prompts the user to select a cloud from the currently loaded entities.
     *
     * \param defaultCloudEntity A cloud entity to be selected by default (optional).
     * \param inviteMessage The message to be displayed in the dialog (optional, default is 'Please select an entity:').
     * \return The selected cloud, or nullptr if the user canceled the operation.
     */
	ccPointCloud* askUserToSelectACloud(ccHObject* defaultCloudEntity = nullptr, QString inviteMessage = QString());

	/**
     * \brief Enumeration for the fast registration modes.
     */
	enum FastRegistrationMode
	{
		MoveBBCenterToOrigin,
		MoveBBMinCornerToOrigin,
		MoveBBMaxCornerToOrigin
	};

	/**
     * \brief Applies a fast registration transformation to the selected entities.
     *
     * This function applies a fast registration transformation to the selected entities, based on the specified mode.
     *
     * \param mode The fast registration mode to be applied.
     */
	void doActionFastRegistration(FastRegistrationMode mode);

	/**
     * \brief Toggles a property of the selected entities.
     *
     * This function toggles a specific property (visibility, normals, scalar field, etc.) of the selected entities.
     *
     * \param property The property to be toggled.
     */
	void toggleSelectedEntitiesProperty(ccEntityAction::TOGGLE_PROPERTY property);

	/**
     * \brief Clears a property of the selected entities.
     *
     * This function clears a specific property (scalar field, normals, etc.) of the selected entities.
     *
     * \param property The property to be cleared.
     */
	void clearSelectedEntitiesProperty(ccEntityAction::CLEAR_PROPERTY property);

	/**
     * \brief Sets the view orientation.
     *
     * This function sets the view orientation of the active 3D window.
     *
     * \param view The new view orientation.
     */
	void setView(CC_VIEW_ORIENTATION view);

	/**
     * \brief Applies a transformation to the selected entities.
     *
     * This function applies the specified transformation matrix to the selected entities.
     *
     * \param transMat The transformation matrix to be applied.
     * \param applyToGlobal Indicates whether the transformation should be applied to the global coordinate system.
     */
	void applyTransformation(const ccGLMatrixd& transMat, bool applyToGlobal);

	/**
     * \brief Creates point clouds from multiple components.
     *
     * This function creates new point cloud entities from the components of the specified cloud, based on various
     * criteria.
     *
     * \param cloud The input cloud to be processed.
     * \param components The reference cloud container for the components.
     * \param minPointPerComponent The minimum number of points per component to create a new cloud.
     * \param randomColors Indicates whether to assign random colors to the new clouds.
     * \param selectComponents Indicates whether to select the new clouds after creation.
     * \param sortBysize Indicates whether to sort the new clouds by size.
     */
	void createComponentsClouds(ccGenericPointCloud* cloud, CCCoreLib::ReferenceCloudContainer& components, unsigned minPointPerComponent, bool randomColors, bool selectComponents, bool sortBysize = true);

	/**
     * \brief Saves the position and state of all GUI elements.
     *
     * This function saves the position and state of all GUI elements, such as toolbars, dockable windows, and 3D views.
     */
	void saveGUIElementsPos();

	/**
     * \brief Sets the active window to an orthographic view.
     *
     * This function sets the active 3D window to an orthographic view.
     *
     * \param win The 3D window to be set to an orthographic view.
     */
	void setOrthoView(ccGLWindowInterface* win);

	/**
     * \brief Sets the active window to a centered perspective view.
     *
     * This function sets the active 3D window to a centered perspective view.
     *
     * \param win The 3D window to be set to a centered perspective view.
     * \param autoRedraw Indicates whether to automatically redraw the window after the view change.
     */
	void setCenteredPerspectiveView(ccGLWindowInterface* win, bool autoRedraw = true);

	/**
     * \brief Sets the active window to a viewer-based perspective view.
     *
     * This function sets the active 3D window to a viewer-based perspective view.
     *
     * \param win The 3D window to be set to a viewer-based perspective view.
     */
	void setViewerPerspectiveView(ccGLWindowInterface* win);

	/**
     * \brief Handles the show event.
     *
     * This function is called when the main window is shown.
     *
     * \param event The show event.
     */
	void showEvent(QShowEvent* event);

	/**
     * \brief Handles the close event.
     *
     * This function is called when the main window is closed.
     *
     * \param event The close event.
     */
	void closeEvent(QCloseEvent* event);

	/**
     * \brief Handles the move event.
     *
     * This function is called when the main window is moved.
     *
     * \param event The move event.
     */
	void moveEvent(QMoveEvent* event);

	/**
     * \brief Handles the resize event.
     *
     * This function is called when the main window is resized.
     *
     * \param event The resize event.
     */
	void resizeEvent(QResizeEvent* event);

	/**
     * \brief Handles the event filter.
     *
     * This function is called to filter events for the main window.
     *
     * \param obj The object that generated the event.
     * \param event The event.
     * \return True if the event was handled, false otherwise.
     */
	bool eventFilter(QObject* obj, QEvent* event);

	/**
     * \brief Handles the key press event.
     *
     * This function is called when a key is pressed in the main window.
     *
     * \param event The key press event.
     */
	void keyPressEvent(QKeyEvent* event);

	/**
     * \brief Zooms in on the specified entity.
     *
     * This function makes the active 3D window zoom in on the specified entity.
     *
     * \param object The entity to zoom in on.
     */
	void zoomOn(ccHObject* object);

	/**
     * \brief Applies an action to the active scalar field.
     *
     * This function applies a specific action (toggle color scale, activate previous/next scalar field) to the active
     * scalar field.
     *
     * \param action The action to be applied.
     */
	void doApplyActiveSFAction(int action);

	/**
     * \brief Computes a mesh using the specified triangulation type.
     *
     * This function computes a mesh for the selected entities using the specified triangulation type.
     *
     * \param type The triangulation type to be used.
     */
	void doActionComputeMesh(CCCoreLib::TRIANGULATION_TYPES type);

	/**
     * \brief Computes the orientation of an entity.
     *
     * This function computes the orientation of the selected entity, either by fitting a plane or a facet (2D polygon).
     *
     * \param fitFacet Indicates whether to fit a facet (true) or a plane (false).
     */
	void doComputePlaneOrientation(bool fitFacet);

	/**
     * \brief Sets up any input devices (3D mouse, gamepad) and adds their menus.
     */
	void setupInputDevices();

	/**
     * \brief Stops input and destroys any input device handling.
     */
	void destroyInputDevices();

	/**
     * \brief Connects all Qt actions to slots.
     */
	void connectActions();

	/**
     * \brief Enables menu items based on the current selection.
     *
     * \param selInfo Information about the current selection.
     */
	void enableUIItems(dbTreeSelectionInfo& selInfo);

	/**
     * \brief Updates the view mode pop-up menu based on the active 3D window.
     *
     * \param win The active 3D window (or nullptr if none).
     */
	void updateViewModePopUpMenu(ccGLWindowInterface* win);

	/**
     * \brief Updates the pivot visibility pop-up menu based on the active 3D window.
     *
     * \param win The active 3D window (or nullptr if none).
     */
	void updatePivotVisibilityPopUpMenu(ccGLWindowInterface* win);

	/**
     * \brief Checks whether the stereo mode can be stopped (if necessary) or not.
     *
     * \param win The 3D window to check.
     * \return True if the stereo mode can be stopped, false otherwise.
     */
	bool checkStereoMode(ccGLWindowInterface* win);

	/**
     * \brief Adds a constant scalar field to the active point cloud.
     *
     * \param cloud The active point cloud to add the scalar field to.
     * \param sfName The name of the new scalar field.
     * \param integerValue Indicates whether the scalar field values should be integers.
     */
	void addConstantSF(ccPointCloud* cloud, QString sfName, bool integerValue);

  private: // members
	/**
     * \brief The main user interface object.
     */
	Ui::MainWindow* m_UI;

	/**
     * \brief The database tree root.
     */
	ccDBRoot* m_ccRoot;

	/**
     * \brief The currently selected entities.
     */
	ccHObject::Container m_selectedEntities;

	/**
     * \brief Indicates whether the user interface is frozen.
     *
     * This flag is set when the user interface is frozen using the `freezeUI` function.
     */
	bool m_uiFrozen;

	/**
     * \brief The manager for recent files.
     */
	ccRecentFiles* m_recentFiles;

	/**
     * \brief The 3D mouse manager.
     */
	cc3DMouseManager* m_3DMouseManager;

	/**
     * \brief The gamepad manager.
     */
	ccGamepadManager* m_gamepadManager;

	/**
     * \brief The view mode pop-up menu button.
     */
	QToolButton* m_viewModePopupButton;

	/**
     * \brief The pivot visibility pop-up menu button.
     */
	QToolButton* m_pivotVisibilityPopupButton;

	/**
     * \brief Indicates whether this is the first time the window is made visible.
     */
	bool m_firstShow;

	/**
     * \brief The point picking hub.
     */
	ccPickingHub* m_pickingHub;

	/******************************/
	/***        MDI AREA        ***/
	/******************************/

	/**
     * \brief The MDI (Multiple Document Interface) area.
     */
	QMdiArea* m_mdiArea;

	/**
     * \brief Represents an MDI area overlay dialog.
     *
     * This structure contains the overlay dialog and its position in the MDI area.
     */
	struct ccMDIDialogs
	{
		/**
         * \brief The overlay dialog.
         */
		ccOverlayDialog* dialog;

		/**
         * \brief The position of the overlay dialog in the MDI area.
         */
		Qt::Corner position;

		/**
         * \brief Constructor with dialog and position.
         *
         * \param dlg The overlay dialog.
         * \param pos The position of the overlay dialog.
         */
		ccMDIDialogs(ccOverlayDialog* dlg, Qt::Corner pos)
		    : dialog(dlg)
		    , position(pos)
		{
		}
	};

	/**
     * \brief Repositions an MDI dialog to its correct position.
     *
     * This function repositions the specified MDI dialog to its correct position in the MDI area.
     *
     * \param mdiDlg The MDI dialog to be repositioned.
     */
	void repositionOverlayDialog(ccMDIDialogs& mdiDlg);

	/**
     * \brief The registered MDI area overlay dialogs.
     */
	std::vector<ccMDIDialogs> m_mdiDialogs;

	/**
     * \brief The camera parameters edit dialog.
     */
	ccCameraParamEditDlg* m_cpeDlg;

	/**
     * \brief The graphical segmentation tool.
     */
	ccGraphicalSegmentationTool* m_gsTool;

	/**
     * \brief The polyline tracing tool.
     */
	ccTracePolylineTool* m_tplTool;

	/**
     * \brief The section extraction tool.
     */
	ccSectionExtractionTool* m_seTool;

	/**
     * \brief The graphical transformation tool.
     */
	ccGraphicalTransformationTool* m_transTool;

	/**
     * \brief The clipping box tool.
     */
	ccClippingBoxTool* m_clipTool;

	/**
     * \brief The cloud comparison dialog.
     */
	ccComparisonDlg* m_compDlg;

	/**
     * \brief The point properties mode dialog.
     */
	ccPointPropertiesDlg* m_ppDlg;

	/**
     * \brief The point list picking dialog.
     */
	ccPointListPickingDlg* m_plpDlg;

	/**
     * \brief The point-pair registration dialog.
     */
	ccPointPairRegistrationDlg* m_pprDlg;

	/**
     * \brief The primitive factory dialog.
     */
	ccPrimitiveFactoryDlg* m_pfDlg;

	/**
     * \brief The plugin UI manager.
     *
     * This manages the plugins' menus, toolbars, and the about dialog.
     */
	ccPluginUIManager* m_pluginUIManager;
};

#endif
