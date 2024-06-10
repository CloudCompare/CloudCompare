// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

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
 * @class MainWindow
 * @brief The main window class for the application.
 *
 * This class represents the main window of the application. It inherits from QMainWindow and implements various interfaces such as ccMainAppInterface and ccPickingListener. The main window contains the user interface elements and handles user interactions.
 */
class MainWindow : public QMainWindow, public ccMainAppInterface, public ccPickingListener
{
	/**
	 * @brief The Q_OBJECT macro is used in the declaration of a class that implements signals and slots.
	 *
	 * The Q_OBJECT macro must appear in the private section of a class definition that declares its own signals and slots or that uses other services provided by Qt's meta-object system.
	 * It is necessary for the signals and slots mechanism to work properly.
	 */
	Q_OBJECT

protected:
	/**
	 * @brief Constructor for the MainWindow class.
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
	 * @brief Destructor for the MainWindow class.
	 *
	 * This destructor is responsible for cleaning up any resources used by the MainWindow object.
	 * It overrides the base class destructor.
	 */
	~MainWindow() override;

public:
	/**
	 * @brief Returns the instance of the MainWindow class.
	 *
	 * @return A pointer to the MainWindow instance.
	 */
	static MainWindow *TheInstance();

	/**
	 * @brief Retrieves the active GL window.
	 *
	 * This function returns a pointer to the active ccGLWindowInterface object.
	 *
	 * @return A pointer to the active GL window.
	 */
	static ccGLWindowInterface *GetActiveGLWindow();

	/**
	 * @brief Retrieves the ccGLWindowInterface object with the specified title.
	 *
	 * This function returns a pointer to the ccGLWindowInterface object that has the given title.
	 *
	 * @param title The title of the ccGLWindowInterface object to retrieve.
	 * @return A pointer to the ccGLWindowInterface object with the specified title, or nullptr if not found.
	 */
	static ccGLWindowInterface *GetGLWindow(const QString &title);

	/**
	 * @brief Retrieves the GL windows.
	 *
	 * This function retrieves a list of GL windows and stores them in the provided vector.
	 *
	 * @param glWindows A reference to a vector of ccGLWindowInterface pointers where the GL windows will be stored.
	 */
	static void GetGLWindows(std::vector<ccGLWindowInterface *> &glWindows);

	/**
	 * @brief Refreshes all GL windows.
	 *
	 * This function refreshes all GL windows in the application. By default, it refreshes both 2D and 3D windows,
	 * but you can specify `only2D` parameter as `true` to refresh only the 2D windows.
	 *
	 * @param only2D Flag indicating whether to refresh only the 2D windows. Default is `false`.
	 */
	static void RefreshAllGLWindow(bool only2D = false);

	/**
	 * @brief Updates the user interface.
	 *
	 * This function is responsible for updating the user interface of the main window.
	 * It should be called whenever there are changes that require updating the UI elements.
	 */
	static void UpdateUI();

	/**
	 * @brief Destroys the instance of the MainWindow class.
	 *
	 * This static function is used to destroy the instance of the MainWindow class.
	 * It should be called when the application is shutting down or when the instance is no longer needed.
	 */
	static void DestroyInstance();

	/**
	 * @brief Returns the active GL window interface.
	 *
	 * This function returns the ccGLWindowInterface object representing the active GL window.
	 *
	 * @return The active GL window interface.
	 */
	ccGLWindowInterface *getActiveGLWindow() override;

	/**
	 * @brief Retrieves the QMdiSubWindow associated with the given ccGLWindowInterface.
	 *
	 * This function returns the QMdiSubWindow that contains the specified ccGLWindowInterface.
	 *
	 * @param win The ccGLWindowInterface for which to retrieve the QMdiSubWindow.
	 * @return The QMdiSubWindow associated with the ccGLWindowInterface, or nullptr if not found.
	 */
	QMdiSubWindow *getMDISubWindow(ccGLWindowInterface *win);

	/**
	 * @brief Retrieves the ccGLWindowInterface object at the specified index.
	 *
	 * This function returns a pointer to the ccGLWindowInterface object located at the given index.
	 *
	 * @param index The index of the ccGLWindowInterface object to retrieve.
	 * @return A constant pointer to the ccGLWindowInterface object at the specified index.
	 */
	ccGLWindowInterface *getGLWindow(int index) const;

	/**
	 * @brief Returns the number of GL windows.
	 *
	 * This function returns the number of GL windows in the application.
	 *
	 * @return The number of GL windows.
	 */
	int getGLWindowCount() const;

	/**
	 * @brief Adds the specified filenames to the database.
	 *
	 * This function adds the filenames specified in the `filenames` parameter to the database.
	 * The `fileFilter` parameter can be used to filter the files based on a specific pattern.
	 * The `destWin` parameter is an optional parameter that specifies the destination window
	 * where the files should be added. If no destination window is specified, the files will be
	 * added to the default window.
	 *
	 * @param filenames The list of filenames to be added to the database.
	 * @param fileFilter The filter to be applied to the filenames (optional).
	 * @param destWin The destination window where the files should be added (optional).
	 */
	virtual void addToDB(const QStringList &filenames,
						 QString fileFilter = QString(),
						 ccGLWindowInterface *destWin = nullptr);

	/**
	 * @brief Adds an object to the database.
	 *
	 * This function adds the specified object to the database. It provides options to control the behavior of the operation.
	 *
	 * @param obj The object to be added to the database.
	 * @param updateZoom Flag indicating whether to update the zoom level.
	 * @param autoExpandDBTree Flag indicating whether to automatically expand the database tree.
	 * @param checkDimensions Flag indicating whether to check the dimensions of the object.
	 * @param autoRedraw Flag indicating whether to automatically redraw the interface.
	 */
	void addToDB(ccHObject *obj,
				 bool updateZoom = false,
				 bool autoExpandDBTree = true,
				 bool checkDimensions = false,
				 bool autoRedraw = true) override;

	/**
	 * @brief Registers an overlay dialog with the specified position.
	 *
	 * This function registers an overlay dialog with the main window at the specified position.
	 *
	 * @param dlg The overlay dialog to register.
	 * @param pos The position where the overlay dialog should be displayed.
	 */
	void registerOverlayDialog(ccOverlayDialog *dlg, Qt::Corner pos) override;

	/**
	 * @brief Unregisters an overlay dialog.
	 *
	 * This function is used to unregister an overlay dialog from the main window.
	 *
	 * @param dlg The overlay dialog to unregister.
	 */
	void unregisterOverlayDialog(ccOverlayDialog *dlg) override;

	/**
	 * @brief Updates the placement of overlay dialogs.
	 *
	 * This function is called to update the placement of overlay dialogs in the main window.
	 * It overrides the base class function.
	 */
	void updateOverlayDialogsPlacement() override;

	/**
	 * @brief Removes the specified object from the database.
	 *
	 * This function removes the specified object from the database. By default, it also deletes the object from memory.
	 *
	 * @param obj The object to be removed.
	 * @param autoDelete If set to true, the object will be deleted from memory after removal from the database.
	 *                   If set to false, the object will only be removed from the database without deleting it from memory.
	 */
	void removeFromDB(ccHObject *obj, bool autoDelete = true) override;

	/**
	 * @brief Sets the selected state of the given object in the database.
	 *
	 * This function is used to update the selected state of an object in the database.
	 * The selected state determines whether the object is currently selected or not.
	 *
	 * @param obj A pointer to the ccHObject representing the object.
	 * @param selected A boolean value indicating the selected state to set.
	 */
	void setSelectedInDB(ccHObject *obj, bool selected) override;

	/**
	 * @brief Displays a message to the console.
	 *
	 * This function is used to display a message to the console. The message can be of type QString.
	 * The level parameter is optional and is used to specify the message level. By default, the level is set to STD_CONSOLE_MESSAGE.
	 *
	 * @param message The message to be displayed.
	 * @param level The message level (optional).
	 */
	void dispToConsole(QString message, ConsoleMessageLevel level = STD_CONSOLE_MESSAGE) override;

	/**
	 * @brief Forces the console display.
	 *
	 * This function overrides the base class function and forces the console display.
	 * It is used in the MainWindow class.
	 */
	void forceConsoleDisplay() override;

	/**
	 * @brief Returns the root object of the database.
	 *
	 * This function overrides the base class function and returns the root object of the database.
	 *
	 * @return A pointer to the root object of the database.
	 */
	ccHObject *dbRootObject() override;

	/**
	 * @brief Returns the main window object.
	 *
	 * This function returns a pointer to the main window object.
	 *
	 * @return A pointer to the main window object.
	 */
	inline QMainWindow *getMainWindow() override { return this; }

	/**
	 * @brief Loads a file.
	 *
	 * This function loads a file specified by the given filename.
	 *
	 * @param filename The path of the file to be loaded.
	 * @param silent   A flag indicating whether to display error messages or not.
	 *
	 * @return A pointer to the loaded ccHObject.
	 */
	ccHObject *loadFile(QString filename, bool silent) override;

	/**
	 * @brief Returns the selected entities.
	 *
	 * This function returns a constant reference to the container of selected entities.
	 *
	 * @return A constant reference to the container of selected entities.
	 */
	inline const ccHObject::Container &getSelectedEntities() const override { return m_selectedEntities; }

	/**
	 * @brief Creates a GL window and a corresponding QWidget.
	 *
	 * This function creates a GL window and a corresponding QWidget. The GL window is represented by the `ccGLWindowInterface` pointer, and the QWidget is represented by the `QWidget` pointer. The function is marked as `const` and is overridden from a base class.
	 *
	 * @param window A reference to a pointer to the `ccGLWindowInterface` object that will be created.
	 * @param widget A reference to a pointer to the `QWidget` object that will be created.
	 */
	void createGLWindow(ccGLWindowInterface *&window, QWidget *&widget) const override;

	/**
	 * @brief Destroys a GL window.
	 *
	 * This function is called to destroy a GL window.
	 *
	 * @param window A pointer to the GL window interface.
	 */
	void destroyGLWindow(ccGLWindowInterface *) const override;

	/**
	 * @brief Returns the shared unique ID generator.
	 *
	 * This function returns the shared instance of the unique ID generator.
	 * The unique ID generator is responsible for generating unique IDs for objects.
	 *
	 * @return The shared unique ID generator.
	 */
	ccUniqueIDGenerator::Shared getUniqueIDGenerator() override;

	/**
	 * @brief Returns the color scales manager.
	 *
	 * This function returns the color scales manager, which is responsible for managing the color scales used in the application.
	 *
	 * @return A pointer to the ccColorScalesManager object.
	 */
	ccColorScalesManager *getColorScalesManager() override;

	/**
	 * @brief Spawns a histogram dialog.
	 *
	 * This function spawns a histogram dialog with the given histogram values, minimum value, maximum value,
	 * title, and x-axis label.
	 *
	 * @param histoValues The histogram values.
	 * @param minVal The minimum value.
	 * @param maxVal The maximum value.
	 * @param title The title of the histogram dialog.
	 * @param xAxisLabel The label for the x-axis of the histogram.
	 */
	void spawnHistogramDialog(const std::vector<unsigned> &histoValues,
							  double minVal, double maxVal,
							  QString title, QString xAxisLabel) override;

	/**
	 * @brief Returns the picking hub.
	 *
	 * This function returns the picking hub associated with the main window.
	 *
	 * @return A pointer to the picking hub.
	 */
	ccPickingHub *pickingHub() override { return m_pickingHub; }

	/**
	 * @brief Removes the specified object temporarily from the database tree.
	 *
	 * This function removes the given object from the database tree temporarily.
	 * The object will not be permanently deleted, but it will be hidden from the tree view.
	 *
	 * @param obj A pointer to the object to be removed.
	 */
	ccHObjectContext removeObjectTemporarilyFromDBTree(ccHObject *obj) override;

	/**
	 * @brief Puts the specified object back into the database tree.
	 *
	 * This function is called to put the given object back into the database tree
	 * using the specified context.
	 *
	 * @param obj The object to be put back into the database tree.
	 * @param context The context used to put the object back into the tree.
	 */
	void putObjectBackIntoDBTree(ccHObject *obj, const ccHObjectContext &context) override;

	/**
	 * @brief Handles the event when an item is picked.
	 *
	 * This function is called when an item is picked and provides information about the picked item.
	 *
	 * @param pi The picked item information.
	 */
	void onItemPicked(const PickedItem &pi) override;

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
	 * @brief Returns the ccDBRoot object.
	 *
	 * This function returns the ccDBRoot object, which represents the database root.
	 *
	 * @return A pointer to the ccDBRoot object.
	 */
	virtual ccDBRoot *db();

	/**
	 * @brief Adds the "Edit Plane" action to the given menu.
	 *
	 * This function adds an action called "Edit Plane" to the specified menu.
	 * The action allows the user to edit a plane.
	 *
	 * @param menu The menu to which the action will be added.
	 */
	void addEditPlaneAction(QMenu &menu) const;

	/**
	 * @brief Initializes the plugins.
	 *
	 * This function is responsible for initializing the plugins used in the application.
	 * It should be called before using any functionality provided by the plugins.
	 */
	void initPlugins();

	/**
	 * @brief Updates the properties view.
	 *
	 * This function is responsible for updating the properties view in the main window.
	 * It should be called whenever there is a change in the properties that need to be displayed.
	 */
	void updatePropertiesView();

private:
	/**
	 * @brief Creates a new 3D view.
	 *
	 * This function creates a new 3D view by calling the internal function new3DViewInternal(true).
	 *
	 * @return A pointer to the newly created ccGLWindowInterface object.
	 */
	ccGLWindowInterface *new3DView() { return new3DViewInternal(true); }

	/**
	 * @brief Creates a new 3D view window.
	 *
	 * This function creates a new 3D view window with the specified settings.
	 *
	 * @param allowEntitySelection Flag indicating whether entity selection is allowed in the new view.
	 * @return A pointer to the newly created ccGLWindowInterface object.
	 */
	ccGLWindowInterface *new3DViewInternal(bool allowEntitySelection);

	/**
	 * @brief Zooms in.
	 *
	 * This function is used to zoom in on the current view.
	 */
	void zoomIn();

	/**
	 * @brief Zooms out.
	 *
	 * This function is used to zoom out from the current view.
	 */
	void zoomOut();

	/**
	 * @brief Shows the help dialog.
	 *
	 * This function is responsible for displaying the help dialog to the user.
	 * It provides information and instructions on how to use the application.
	 */
	void doActionShowHelpDialog();

	/**
	 * @brief Performs the action of loading a file.
	 *
	 * This function is responsible for loading a file and performing any necessary actions associated with it.
	 *
	 * @note This function assumes that the necessary file path has been set prior to calling it.
	 */
	void doActionLoadFile();

	/**
	 * @brief Performs the action of saving a file.
	 *
	 * This function is responsible for saving a file.
	 * It performs the necessary operations to save the current state of the file.
	 *
	 * @note This function assumes that the file has already been opened and modified.
	 *
	 * @see openFile()
	 * @see modifyFile()
	 */
	void doActionSaveFile();

	/**
	 * @brief Performs the action of saving the project.
	 *
	 * This function is responsible for saving the current project.
	 * It performs the necessary operations to save the project data
	 * to a file or a database.
	 */
	void doActionSaveProject();

	/**
	 * @brief Performs the global shift settings action.
	 *
	 * This function is responsible for executing the global shift settings action. It performs the necessary operations to apply the global shift settings.
	 */
	void doActionGlobalShiftSettings();

	/**
	 * @brief Enables or disables Qt warnings.
	 *
	 * This function allows you to enable or disable Qt warnings in the application.
	 *
	 * @param enable A boolean value indicating whether to enable or disable the warnings.
	 */
	void doEnableQtWarnings(bool enable);

	/**
	 * @brief Performs the action of cloning.
	 *
	 * This function is responsible for performing the action of cloning.
	 * It does XYZ.
	 *
	 * @return void
	 */
	void doActionClone();

	/**
	 * @brief Prepares for the deletion of a window.
	 *
	 * This function is responsible for preparing the deletion of a window in the application.
	 * It takes a pointer to a `ccGLWindowInterface` object representing the window to be deleted.
	 *
	 * @param glWindow A pointer to the window to be deleted.
	 */
	void prepareWindowDeletion(ccGLWindowInterface *glWindow);

	/**
	 * @brief Handles the toggling of exclusive full screen mode.
	 *
	 * This function is called when the exclusive full screen mode is toggled.
	 * It takes a boolean parameter @p state which represents the new state of the toggle.
	 *
	 * @param state The new state of the exclusive full screen toggle.
	 */
	void onExclusiveFullScreenToggled(bool state);

	/**
	 * @brief Freezes or unfreezes the user interface.
	 *
	 * This function is used to freeze or unfreeze the user interface based on the given state.
	 * When the state is true, the user interface will be frozen and no user interaction will be allowed.
	 * When the state is false, the user interface will be unfrozen and normal user interaction will be allowed.
	 *
	 * @param state The state indicating whether to freeze or unfreeze the user interface.
	 */
	void freezeUI(bool state) override;

	/**
	 * @brief Redraws all elements in the main window.
	 *
	 * This function is responsible for redrawing all elements in the main window.
	 * By default, it redraws both 2D and 3D elements. However, if the `only2D` parameter
	 * is set to `true`, it will only redraw the 2D elements.
	 *
	 * @param only2D Flag indicating whether to redraw only 2D elements.
	 */
	void redrawAll(bool only2D = false) override;

	/**
	 * @brief Refreshes all elements in the main window.
	 *
	 * This function refreshes all elements in the main window, including both 2D and 3D elements.
	 *
	 * @param only2D If set to true, only 2D elements will be refreshed. Default is false.
	 */
	void refreshAll(bool only2D = false) override;
	/**
	 * @brief Enables all UI elements.
	 */
	void enableAll() override;

	/**
	 * @brief Disables all UI elements.
	 */
	void disableAll() override;

	/**
	 * @brief Disables all windows except for the specified window.
	 *
	 * This function disables all windows in the application except for the specified window.
	 *
	 * @param win The window to keep enabled.
	 */
	void disableAllBut(ccGLWindowInterface *win) override;

	/**
	 * @brief Updates the user interface.
	 */
	void updateUI() override;

	/**
	 * @brief Toggles stereo vision in the active window.
	 *
	 * @param state If true, stereo vision will be enabled; if false, it will be disabled.
	 */
	virtual void toggleActiveWindowStereoVision(bool state);

	/**
	 * @brief Toggles centered perspective in the active window.
	 */
	void toggleActiveWindowCenteredPerspective() override;

	/**
	 * @brief Toggles custom light in the active window.
	 */
	void toggleActiveWindowCustomLight() override;

	/**
	 * @brief Toggles sunlight in the active window.
	 */
	void toggleActiveWindowSunLight() override;

	/**
	 * @brief Toggles viewer-based perspective in the active window.
	 */
	void toggleActiveWindowViewerBasedPerspective() override;

	/**
	 * @brief Zooms in on the selected entities.
	 */
	void zoomOnSelectedEntities() override;

	/**
	 * @brief Sets the global zoom level.
	 */
	void setGlobalZoom() override;

	/**
	 * @brief Increases the point size in the active window.
	 */
	void increasePointSize() override;

	/**
	 * @brief Decreases the point size in the active window.
	 */
	void decreasePointSize() override;

	/**
	 * @brief Toggles the lock on the rotation axis in the active window.
	 */
	void toggleLockRotationAxis();

	/**
	 * @brief Enables bubble view mode in the active window.
	 */
	void doActionEnableBubbleViewMode();

	/**
	 * @brief Sets the pivot to always be on in the active window.
	 */
	void setPivotAlwaysOn();

	/**
	 * @brief Sets the pivot to rotation-only mode in the active window.
	 */
	void setPivotRotationOnly();

	/**
	 * @brief Turns off the pivot in the active window.
	 */
	void setPivotOff();

	/**
	 * @brief Toggles the state of the auto pick rotation center feature in the active window.
	 *
	 * This function is responsible for toggling the state of the auto pick rotation center feature
	 * in the active window. When the state is enabled, the rotation center will be automatically
	 * picked based on the selected objects in the window.
	 *
	 * @param state The new state of the auto pick rotation center feature. Set to true to enable
	 *              the feature, or false to disable it.
	 */
	void toggleActiveWindowAutoPickRotCenter(bool state);

	/**
	 * @brief Toggles the display of cursor coordinates in the active window.
	 *
	 * @param state If true, cursor coordinates will be shown; if false, they will be hidden.
	 */
	void toggleActiveWindowShowCursorCoords(bool state);
	/**
	 * @brief Handles new label.
	 *
	 * This function is called when a new label is created and added to the database.
	 *
	 * @param entity The new label object.
	 */
	void handleNewLabel(ccHObject *);

	/**
	 * @brief Sets the active sub-window.
	 *
	 * This function sets the active sub-window in the MDI area.
	 *
	 * @param window The window to set as the active sub-window.
	 */
	void setActiveSubWindow(QWidget *window);

	/**
	 * @brief Shows the display options dialog.
	 *
	 * This function displays the display options dialog, allowing the user to configure various
	 * display-related settings.
	 */
	void showDisplayOptions();

	/**
	 * @brief Shows the histogram of the selected entities.
	 *
	 * This function displays the histogram of the scalar fields associated with the selected entities.
	 */
	void showSelectedEntitiesHistogram();

	/**
	 * @brief Tests the frame rate.
	 *
	 * This function initiates a frame rate test on the active 3D view.
	 */
	void testFrameRate();

	/**
	 * @brief Toggles full-screen mode.
	 *
	 * This function toggles the full-screen mode of the main window.
	 *
	 * @param state The new state of the full-screen mode (true for full-screen, false for normal).
	 */
	void toggleFullScreen(bool state);

	/**
	 * @brief Toggles visual debug traces.
	 *
	 * This function toggles the display of visual debug traces in the active 3D view.
	 */
	void toggleVisualDebugTraces();

	/**
	 * @brief Toggles exclusive full-screen mode.
	 *
	 * This function toggles the exclusive full-screen mode of the active 3D view.
	 *
	 * @param state The new state of the exclusive full-screen mode (true for full-screen, false for normal).
	 */
	void toggleExclusiveFullScreen(bool state);

	/**
	 * @brief Updates the 3D views menu.
	 *
	 * This function updates the 3D views menu, adding entries for the currently open 3D views.
	 */
	void update3DViewsMenu();

	/**
	 * @brief Updates the menus.
	 *
	 * This function updates the state of the various menus based on the current selection and state of the application.
	 */
	void updateMenus();

	/**
	 * @brief Handles the activation of a 3D view.
	 *
	 * This function is called when a 3D view is activated in the MDI area.
	 *
	 * @param window The activated sub-window.
	 */
	void on3DViewActivated(QMdiSubWindow *);

	/**
	 * @brief Updates the UI with the current selection.
	 *
	 * This function updates the user interface to reflect the currently selected entities.
	 */
	void updateUIWithSelection();

	/**
	 * @brief Adds files to the database automatically.
	 *
	 * This function is called when files are dropped onto an active 3D view. It automatically adds the files to the database.
	 *
	 * @param filenames The list of filenames to be added to the database.
	 */
	void addToDBAuto(const QStringList &filenames);

	/**
	 * @brief Echoes mouse wheel rotation.
	 *
	 * This function is called when the mouse wheel is rotated in an active 3D view. It echoes the rotation to the other 3D views.
	 *
	 * @param wheelDelta_deg The rotation of the mouse wheel in degrees.
	 */
	void echoMouseWheelRotate(float);

	/**
	 * @brief Echoes base view matrix rotation.
	 *
	 * This function is called when the base view matrix is rotated in an active 3D view. It echoes the rotation to the other 3D views.
	 *
	 * @param rotMat The new base view matrix rotation.
	 */
	void echoBaseViewMatRotation(const ccGLMatrixd &rotMat);

	/**
	 * @brief Echoes camera position change.
	 *
	 * This function is called when the camera position is changed in an active 3D view. It echoes the change to the other 3D views.
	 *
	 * @param P The new camera position.
	 */
	void echoCameraPosChanged(const CCVector3d &);

	/**
	 * @brief Echoes pivot point change.
	 *
	 * This function is called when the pivot point is changed in an active 3D view. It echoes the change to the other 3D views.
	 *
	 * @param P The new pivot point.
	 */
	void echoPivotPointChanged(const CCVector3d &);

	void doActionRenderToFile();

	// menu action
	void doActionSetUniqueColor();
	void doActionColorize();
	void doActionRGBToGreyScale();
	void doActionSetColor(bool colorize);
	void doActionSetColorGradient();
	void doActionInterpolateColors();
	void doActionChangeColorLevels();
	void doActionEnhanceRGBWithIntensities();
	void doActionColorFromScalars();
	void doActionRGBGaussianFilter();
	void doActionRGBBilateralFilter();
	void doActionRGBMeanFilter();
	void doActionRGBMedianFilter();

	void doActionSFGaussianFilter();
	void doActionSFBilateralFilter();
	void doActionSFConvertToRGB();
	void doActionSFConvertToRandomRGB();
	void doActionRenameSF();
	void doActionOpenColorScalesManager();
	void doActionAddIdField();
	void doActionSplitCloudUsingSF();
	void doActionSetSFAsCoord();
	void doActionInterpolateScalarFields();

	void doComputeGeometricFeature();
	void doActionSFGradient();
	void doRemoveDuplicatePoints();
	void doSphericalNeighbourhoodExtractionTest();	 // DGM TODO: remove after test
	void doCylindricalNeighbourhoodExtractionTest(); // DGM TODO: remove after test
	void doActionFitPlane();
	void doActionFitSphere();
	void doActionFitCircle();
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
	void doAction4pcsRegister(); // Aurelien BEY le 13/11/2008
	void doActionSubsample();	 // Aurelien BEY le 4/12/2008
	void doActionStatisticalTest();
	void doActionSamplePointsOnMesh();
	void doActionSamplePointsOnPolyline();
	void doActionSmoohPolyline();
	void doActionConvertTextureToColor();
	void doActionLabelConnectedComponents();
	void doActionComputeStatParams();
	void doActionFilterByValue();

	// Picking operations
	void enablePickingOperation(ccGLWindowInterface *win, QString message);
	void cancelPreviousPickingOperation(bool aborted);

	// For rotation center picking
	void doPickRotationCenter();
	// For leveling
	void doLevel();

	void doActionCreatePlane();
	void doActionEditPlane();
	void doActionFlipPlane();
	void doActionComparePlanes();

	void doActionDeleteScanGrids();
	void doActionSmoothMeshSF();
	void doActionEnhanceMeshSF();
	void doActionAddConstantSF();
	void doActionAddClassificationSF();
	void doActionScalarFieldArithmetic();
	void doActionScalarFieldFromColor();
	void doActionOrientNormalsFM();
	void doActionOrientNormalsMST();
	void doActionShiftPointsAlongNormals();
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
	void doActionFlipMeshTriangles();
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
	void doActionExportNormalToSF();
	void doActionSetSFsAsNormal();
	void doComputeBestFitBB();
	void doActionCrop();

	void doActionEditCamera();
	void doActionAdjustZoom();
	void doActionSaveViewportAsCamera();
	void doActionResetGUIElementsPos();
	void doActionToggleRestoreWindowOnStartup(bool);
	void doActionResetAllVBOs();

	// Shaders & plugins
	void doActionLoadShader();
	void doActionDeleteShader();

	void doActionFindBiggestInnerRectangle();

	// Clipping box
	void activateClippingBoxMode();
	void deactivateClippingBoxMode(bool);

	// Graphical transformation
	void activateTranslateRotateMode();
	void deactivateTranslateRotateMode(bool);

	// Graphical segmentation
	void activateSegmentationMode();
	void deactivateSegmentationMode(bool);

	// Polyline tracing
	void activateTracePolylineMode();
	void deactivateTracePolylineMode(bool);

	// Section extraction
	void activateSectionExtractionMode();
	void deactivateSectionExtractionMode(bool);

	// Entities comparison
	void doActionCloudCloudDist();
	void doActionCloudMeshDist();
	void doActionCloudPrimitiveDist();
	void deactivateComparisonMode(int);

	// Point picking mechanism
	void activatePointPickingMode();
	void deactivatePointPickingMode(bool);

	// Point list picking mechanism
	void activatePointListPickingMode();
	void deactivatePointListPickingMode(bool);

	// Point-pair registration mechanism
	void activateRegisterPointPairTool();
	void deactivateRegisterPointPairTool(bool);

	// Current active scalar field
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

	//! Creates a cloud with a single point
	void createSinglePointCloud();
	//! Creates a cloud from the clipboard (ASCII) data
	void createPointCloudFromClipboard();

	inline void doActionMoveBBCenterToOrigin() { doActionFastRegistration(MoveBBCenterToOrigin); }
	inline void doActionMoveBBMinCornerToOrigin() { doActionFastRegistration(MoveBBMinCornerToOrigin); }
	inline void doActionMoveBBMaxCornerToOrigin() { doActionFastRegistration(MoveBBMaxCornerToOrigin); }

	//! Restores position and state of all GUI elements
	void restoreGUIElementsPos();

private:
	//! Shortcut: asks the user to select one cloud
	/** \param defaultCloudEntity a cloud to select by default (optional)
		\param inviteMessage invite message (default is something like 'Please select an entity:') (optional)
		\return the selected cloud (or null if the user cancelled the operation)
	**/
	ccPointCloud *askUserToSelectACloud(ccHObject *defaultCloudEntity = nullptr, QString inviteMessage = QString());

	enum FastRegistrationMode
	{
		MoveBBCenterToOrigin,
		MoveBBMinCornerToOrigin,
		MoveBBMaxCornerToOrigin
	};

	void doActionFastRegistration(FastRegistrationMode mode);

	void toggleSelectedEntitiesProperty(ccEntityAction::TOGGLE_PROPERTY property);
	void clearSelectedEntitiesProperty(ccEntityAction::CLEAR_PROPERTY property);

	void setView(CC_VIEW_ORIENTATION view) override;

	//! Apply transformation to the selected entities
	void applyTransformation(const ccGLMatrixd &transMat, bool applyToGlobal);

	//! Creates point clouds from multiple 'components'
	void createComponentsClouds(ccGenericPointCloud *cloud,
								CCCoreLib::ReferenceCloudContainer &components,
								unsigned minPointPerComponent,
								bool randomColors,
								bool selectComponents,
								bool sortBysize = true);

	//! Saves position and state of all GUI elements
	void saveGUIElementsPos();

	void setOrthoView(ccGLWindowInterface *win);
	void setCenteredPerspectiveView(ccGLWindowInterface *win, bool autoRedraw = true);
	void setViewerPerspectiveView(ccGLWindowInterface *win);

	void showEvent(QShowEvent *event) override;
	void closeEvent(QCloseEvent *event) override;
	void moveEvent(QMoveEvent *event) override;
	void resizeEvent(QResizeEvent *event) override;
	bool eventFilter(QObject *obj, QEvent *event) override;
	void keyPressEvent(QKeyEvent *event) override;

	//! Makes the window including an entity zoom on it (helper)
	void zoomOn(ccHObject *object);

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
	void doActionComputeMesh(CCCoreLib::TRIANGULATION_TYPES type);

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
	void enableUIItems(dbTreeSelectionInfo &selInfo);

	//! Updates the view mode pop-menu based for a given window (or an absence of!)
	void updateViewModePopUpMenu(ccGLWindowInterface *win);

	//! Updates the pivot visibility pop-menu based for a given window (or an absence of!)
	void updatePivotVisibilityPopUpMenu(ccGLWindowInterface *win);

	//! Checks whether stereo mode can be stopped (if necessary) or not
	bool checkStereoMode(ccGLWindowInterface *win);

	//! Adds a single value SF to the active point cloud
	void addConstantSF(ccPointCloud *cloud, QString sfName, bool integerValue);

private: // members
	//! Main UI
	Ui::MainWindow *m_UI;

	//! DB tree
	ccDBRoot *m_ccRoot;

	//! Currently selected entities;
	ccHObject::Container m_selectedEntities;

	//! UI frozen state (see freezeUI)
	bool m_uiFrozen;

	//! Recent files menu
	ccRecentFiles *m_recentFiles;

	//! 3D mouse
	cc3DMouseManager *m_3DMouseManager;

	//! Gamepad handler
	ccGamepadManager *m_gamepadManager;

	//! View mode pop-up menu button
	QToolButton *m_viewModePopupButton;

	//! Pivot visibility pop-up menu button
	QToolButton *m_pivotVisibilityPopupButton;

	//! Flag: first time the window is made visible
	bool m_firstShow;

	//! Point picking hub
	ccPickingHub *m_pickingHub;

	/******************************/
	/***        MDI AREA        ***/
	/******************************/

	QMdiArea *m_mdiArea;

	//! CloudCompare MDI area overlay dialogs
	struct ccMDIDialogs
	{
		ccOverlayDialog *dialog;
		Qt::Corner position;

		//! Constructor with dialog and position
		ccMDIDialogs(ccOverlayDialog *dlg, Qt::Corner pos)
			: dialog(dlg), position(pos)
		{
		}
	};

	//! Repositions an MDI dialog at its right position
	void repositionOverlayDialog(ccMDIDialogs &mdiDlg);

	//! Registered MDI area 'overlay' dialogs
	std::vector<ccMDIDialogs> m_mdiDialogs;

	/*** dialogs ***/
	//! Camera params dialog
	ccCameraParamEditDlg *m_cpeDlg;
	//! Graphical segmentation dialog
	ccGraphicalSegmentationTool *m_gsTool;
	//! Polyline tracing tool
	ccTracePolylineTool *m_tplTool;
	//! Section extraction dialog
	ccSectionExtractionTool *m_seTool;
	//! Graphical transformation dialog
	ccGraphicalTransformationTool *m_transTool;
	//! Clipping box dialog
	ccClippingBoxTool *m_clipTool;
	//! Cloud comparison dialog
	ccComparisonDlg *m_compDlg;
	//! Point properties mode dialog
	ccPointPropertiesDlg *m_ppDlg;
	//! Point list picking
	ccPointListPickingDlg *m_plpDlg;
	//! Point-pair registration
	ccPointPairRegistrationDlg *m_pprDlg;
	//! Primitive factory dialog
	ccPrimitiveFactoryDlg *m_pfDlg;

	/*** plugins ***/
	//! Manages plugins - menus, toolbars, and the about dialog
	ccPluginUIManager *m_pluginUIManager;
};

#endif
