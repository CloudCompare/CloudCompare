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

class QMainWindow;
class ccGLWindow;
class ccColorScalesManager;

//! Main application interface (for plugins)
class ccMainAppInterface
{
public:

	//! Returns main window
	virtual QMainWindow* getMainWindow() = 0;

	//! Returns active GL sub-window (if any)
	virtual ccGLWindow* getActiveGLWindow() = 0;

	//! Returns the unique ID generator
	virtual ccUniqueIDGenerator::Shared getUniqueIDGenerator() = 0;

	//! Adds an entity to main db
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
							bool autoRedraw = true ) = 0;

	//! Removes an entity from main db tree
	/** Object is automatically detached from its parent.
		\param obj entity
		\param autoDelete automatically deletes object
	**/
	virtual void removeFromDB(ccHObject* obj, bool autoDelete = true) = 0;

	//! Selects or unselects an entity (in db tree)
	/** \param obj entity
		\param selected whether entity should be selected or not
	**/
	virtual void setSelectedInDB(ccHObject* obj, bool selected) = 0;

	//! Returns currently selected entities ("read only")
	virtual const ccHObject::Container& getSelectedEntities() const = 0;

	//! Console message level (see dispToConsole)
	enum ConsoleMessageLevel {	STD_CONSOLE_MESSAGE = 0,
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
										QString xAxisLabel ) = 0;

	//other usefull methods
	virtual void setFrontView() = 0;
	virtual void setBottomView() = 0;
	virtual void setTopView() = 0;
	virtual void setBackView() = 0;
	virtual void setLeftView() = 0;
	virtual void setRightView() = 0;
	virtual void toggleActiveWindowCenteredPerspective() = 0;
	virtual void toggleActiveWindowCustomLight() = 0;
	virtual void toggleActiveWindowSunLight() = 0;
	virtual void toggleActiveWindowViewerBasedPerspective() = 0;
	virtual void zoomOnSelectedEntities() = 0;

};

#endif //CC_MAIN_APP_INTERFACE_HEADER
