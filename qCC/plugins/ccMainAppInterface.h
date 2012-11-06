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
//$Rev:: 2059                                                              $
//$LastChangedDate:: 2012-03-29 19:11:56 +0200 (jeu., 29 mars 2012)        $
//**************************************************************************
//

#ifndef CC_MAIN_APP_INTERFACE
#define CC_MAIN_APP_INTERFACE

class QMainWindow;
class ccGLWindow;
class ccHObject;

//! Main application interface (for plugins)
class ccMainAppInterface
{
public:

    //! Returns main window
	virtual QMainWindow* getMainWindow()=0;

    //! Returns active GL sub-window (if any)
    virtual ccGLWindow* getActiveGLWindow()=0;

    //! Adds an entity to main db
    /** \param obj entity
		\param autoExpandDBTree whether DB tree should automatically be expanded
        \param statusMessage message to be displayed on the status bar
        \param addToDisplay automatically adds this entity to the active GL display
        \param updateZoom updates active GL display zoom to fit the whole scene, including this new entity (addToDisplay must be true)
		\param winDest destination GL window (if 0, the current active one is used)
		\param coordinatesTransEnabled whether transformation (shift + scale) on load has been applied for precedent entity
		\param coordinatesShift if applicable, shift applied to precedent entity
		\param coordinatesScale if applicable, scale applied to precedent entity (-1 = ignored)
    **/
    virtual void addToDB(ccHObject* obj,
						bool autoExpandDBTree=true,
						const char* statusMessage=0,
						bool addToDisplay=true,
						bool updateZoom=true,
						ccGLWindow* winDest=0,
						bool* coordinatesTransEnabled = 0,
						double* coordinatesShift = 0,
						double* coordinatesScale = 0)=0;

    //! Removes an entity from main db
	/** Object is automatically detached from its parent.
		\param obj entity
		\param autoDelete automatically deletes object
	**/
	virtual void removeFromDB(ccHObject* obj, bool autoDelete=true)=0;

	//! Console message level (see dispToConsole)
	enum ConsoleMessageLevel {	STD_CONSOLE_MESSAGE = 0,
								WRN_CONSOLE_MESSAGE = 1,
								ERR_CONSOLE_MESSAGE = 2,
	};

    //! Prints a message to console
    /** \param message message
        \param warning whether the message is a warning or not
    **/
    virtual void dispToConsole(const char* message, ConsoleMessageLevel level=STD_CONSOLE_MESSAGE)=0;

	//! Forces display of console widget
	virtual void forceConsoleDisplay()=0;

	//! Returns DB root (as a ccHObject)
	virtual ccHObject* dbRoot()=0;

    //! Forces redraw of all GL windows
    virtual void redrawAll()=0;

    //! Redraws all GL windows that have the 'refresh' flag on
    /** See ccGLWindow::toBeRefreshed and ccDrawableObject::prepareDisplayForRefresh.
        Warning: automatically calls MainWindow::updateUI.
    **/
    virtual void refreshAll()=0;

    //! Enables all GL windows
    virtual void enableAll()=0;

    //! Disables all GL windows
    virtual void disableAll()=0;

    //! Disables all GL windows but the specified one
    virtual void disableAllBut(ccGLWindow* win)=0;

    //! Updates UI (menu and properties browser) to reflect current selection state
    /** This method should be called whenever a change is made to any selected entity
    **/
    virtual void updateUI()=0;

    //! Freezes/unfreezes UI
    /** \param state freeze state
    **/
    virtual void freezeUI(bool state)=0;

	//other usefull methods
    virtual void setFrontView()=0;
    virtual void setBottomView()=0;
    virtual void setTopView()=0;
    virtual void setBackView()=0;
    virtual void setLeftView()=0;
    virtual void setRightView()=0;
    virtual void toggleActiveWindowCenteredPerspective()=0;
    virtual void toggleActiveWindowCustomLight()=0;
    virtual void toggleActiveWindowSunLight()=0;
    virtual void toggleActiveWindowViewerBasedPerspective()=0;
    virtual void zoomOnSelectedEntities()=0;

};

#endif //CC_MAIN_APP_INTERFACE
