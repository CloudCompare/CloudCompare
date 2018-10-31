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

#ifndef CC_MAIN_APP_INTERFACE_HEADER
#define CC_MAIN_APP_INTERFACE_HEADER

//Qt
#include <QString>

//qCC_db
#include <ccHObject.h>
#include <ccHObjectCaster.h>
//qCC_gl
#include <ccGLUtils.h>

class QMainWindow;
class QWidget;
class ccGLWindow;
class ccColorScalesManager;
class ccOverlayDialog;
class ccPickingHub;

//! Main application interface (for plugins)
class ccMainAppInterface
{
public:
	virtual ~ccMainAppInterface() = default;

	//! Returns main window
	virtual QMainWindow* getMainWindow() = 0;

	//! Returns active GL sub-window (if any)
	virtual ccGLWindow* getActiveGLWindow() = 0;

	//! Creates a new instance of GL window (with its encapsulating widget)
	/** \warning This instance must be destroyed by the application as well (see destroyGLWindow)
		Note that the encapsulating widget is the window instance itself if 'stereo' mode is disabled
	**/
	virtual void createGLWindow(ccGLWindow*& window, QWidget*& widget) const = 0;

	//! Destroys an instance of GL window created by createGLWindow
	virtual void destroyGLWindow(ccGLWindow*) const = 0;

	//! Registers a MDI area 'overlay' dialog
	/** Overlay dialogs are displayed in the central MDI area, above the 3D views.
	The position (pos) is defined relatively to the MDI area (as one of its 4 corners).
	And it is automatically updated when the main window is moved or resized.
	Registered dialogs are automatically released when CloudCompare stops.

	Notes:
	- it may be necessary to call 'updateOverlayDialogsPlacement' after calling this method
	- it's a good idea to freeez the UI when the tool starts to avoid other overlay dialogs
	to appear (don't forget to unfreeze the UI afterwards)
	**/
	virtual void registerOverlayDialog(ccOverlayDialog* dlg, Qt::Corner pos) = 0;

	//! Unregisters a MDI area 'overlay' dialog
	/** \warning Original overlay dialog object will be deleted (see QObject::deleteLater)
	**/
	virtual void unregisterOverlayDialog(ccOverlayDialog* dlg) = 0;

	//! Forces the update of all registered MDI 'overlay' dialogs
	virtual void updateOverlayDialogsPlacement() = 0;

	//! Returns the unique ID generator
	virtual ccUniqueIDGenerator::Shared getUniqueIDGenerator() = 0;

	//! Adds an entity to the main db
	/** \param obj entity
		\param updateZoom updates active GL display zoom to fit the whole scene, including this new entity (addToDisplay must be true)
		\param autoExpandDBTree whether DB tree should automatically be expanded
		\param checkDimensions whether to check entity's dimensions (and potentially asking the user to shift/rescale it) or not
		\param autoRedraw whether to redraw the 3D view automatically or not (warning: if 'updateZoom' is true, the 3D view will always be redrawn)
	**/
	virtual void addToDB(	ccHObject* obj,
							bool updateZoom = false,
							bool autoExpandDBTree = true,
							bool checkDimensions = false,
							bool autoRedraw = true) = 0;

	//! Removes an entity from main db tree
	/** Object is automatically detached from its parent.
		\param obj entity
		\param autoDelete automatically deletes object
	**/
	virtual void removeFromDB(ccHObject* obj, bool autoDelete = true) = 0;

	//! Backup "context" for an object
	/** Used with removeObjectTemporarilyFromDBTree/putObjectBackIntoDBTree.
	**/
	struct ccHObjectContext
	{
		ccHObject* parent = nullptr;
		int childFlags = 0;
		int parentFlags = 0;
	};

	//! Removes object temporarily from DB tree
	/** This method must be called before any modification to the db tree
		\warning May change the set of currently selected entities
	**/
	virtual ccHObjectContext removeObjectTemporarilyFromDBTree(ccHObject* obj) = 0;

	//! Adds back object to DB tree
	/** This method should be called once modifications to the db tree are finished
		(see removeObjectTemporarilyFromDBTree).
	**/
	virtual void putObjectBackIntoDBTree(ccHObject* obj, const ccHObjectContext& context) = 0;

	//! Selects or unselects an entity (in db tree)
	/** \param obj entity
		\param selected whether entity should be selected or not
	**/
	virtual void setSelectedInDB(ccHObject* obj, bool selected) = 0;

	//! Returns currently selected entities ("read only")
	virtual const ccHObject::Container& getSelectedEntities() const = 0;
	
	//! Checks if we have any selections
	bool	haveSelection() const { return !getSelectedEntities().empty(); }
	
	//! Checks if we have exactly one selection
	bool	haveOneSelection() const { return getSelectedEntities().size() == 1; }

	//! Console message level (see dispToConsole)
	enum ConsoleMessageLevel
	{
		STD_CONSOLE_MESSAGE = 0,
		WRN_CONSOLE_MESSAGE = 1,
		ERR_CONSOLE_MESSAGE = 2,
	};

	//! Prints a message to console
	/** \param message message
		\param level message level (standard, warning, error)
	**/
	virtual void dispToConsole(QString message, ConsoleMessageLevel level = STD_CONSOLE_MESSAGE) = 0;

	//! Forces display of console widget
	virtual void forceConsoleDisplay() = 0;

	//! Returns DB root (as a ccHObject)
	virtual ccHObject* dbRootObject() = 0;

	//! Forces redraw of all GL windows
	/** \param only2D whether to redraw everything (false) or only the 2D layer (true)
	**/
	virtual void redrawAll(bool only2D = false) = 0;

	//! Redraws all GL windows that have the 'refresh' flag on
	/** See ccGLWindow::toBeRefreshed and ccDrawableObject::prepareDisplayForRefresh.
		\param only2D whether to redraw everything (false) or only the 2D layer (true)
	**/
	virtual void refreshAll(bool only2D = false) = 0;

	//! Enables all GL windows
	virtual void enableAll() = 0;

	//! Disables all GL windows
	virtual void disableAll() = 0;

	//! Disables all GL windows but the specified one
	virtual void disableAllBut(ccGLWindow* win) = 0;

	//! Updates UI (menu and properties browser) to reflect current selection state
	/** This method should be called whenever a change is made to any selected entity
	**/
	virtual void updateUI() = 0;

	//! Freezes/unfreezes UI
	/** \param state freeze state
	**/
	virtual void freezeUI(bool state) = 0;

	//! Returns color scale manager (unique instance)
	virtual ccColorScalesManager* getColorScalesManager() = 0;

	//! Spawns an histogram dialog
	virtual void spawnHistogramDialog(	const std::vector<unsigned>& histoValues,
										double minVal,
										double maxVal,
										QString title,
										QString xAxisLabel) = 0;

	//! Returns the picking hub (if any)
	virtual ccPickingHub* pickingHub() { return nullptr; }

	//other useful methods
	virtual void setView( CC_VIEW_ORIENTATION view ) = 0;
	
	virtual void toggleActiveWindowCenteredPerspective() = 0;
	virtual void toggleActiveWindowCustomLight() = 0;
	virtual void toggleActiveWindowSunLight() = 0;
	virtual void toggleActiveWindowViewerBasedPerspective() = 0;
	virtual void zoomOnSelectedEntities() = 0;
	virtual void setGlobalZoom() = 0;

	virtual void increasePointSize() = 0;
	virtual void decreasePointSize() = 0;
};

#endif //CC_MAIN_APP_INTERFACE_HEADER
