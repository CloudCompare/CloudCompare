#pragma once

// ##########################################################################
// #                                                                        #
// #                   CLOUDCOMPARE LIGHT VIEWER                            #
// #                                                                        #
// #  This project has been initiated under funding from ANR/CIFRE          #
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
// #      +++ COPYRIGHT: EDF R&D + TELECOM ParisTech (ENST-TSI) +++         #
// #                                                                        #
// ##########################################################################

// Qt
#include <QMainWindow>
#include <QStringList>

// CCPluginAPI
#include <ccMainAppInterface.h>

// GUIs
#include <ui_ccviewer.h>

// System
#include <set>

class ccGLWindowInterface;
class ccHObject;
class Mouse3DInput;
class ccGamepadManager;

//! Application main window
class ccViewer : public QMainWindow
    , public ccMainAppInterface
{
	Q_OBJECT

  public:
	//! Default constructor
	ccViewer(QWidget* parent = 0, Qt::WindowFlags flags = 0);

	//! Default destructor
	~ccViewer() override;

	//! Adds entity to display db
	void addToDB(ccHObject* entity,
	             bool       updateZoom       = false,
	             bool       autoExpandDBTree = true,
	             bool       checkDimensions  = false,
	             bool       autoRedraw       = true) override;

	//! Removes an entity from display db
	void removeFromDB(ccHObject* obj, bool autoDelete = true) override;

	//! Checks for loaded entities
	/** If none, a message is displayed to invite the user
	    to drag & drop files.
	**/
	bool checkForLoadedEntities();

  public:
	//! Tries to load (and then adds to main db) a list of entity (files)
	/** \param filenames filenames to load
	    \return the first loaded entity/group
	**/
	ccHObject* addToDB(QStringList filenames);

  public: // ccMainInterface compliance
	QMainWindow* getMainWindow() override
	{
		return this;
	}
	ccGLWindowInterface* getActiveGLWindow() override
	{
		return m_glWindow;
	}
	ccHObject* loadFile(QString filename, bool silent) override
	{
		return addToDB(QStringList{filename});
	}
	void setSelectedInDB(ccHObject* obj, bool selected) override
	{
	}
	const ccHObject::Container& getSelectedEntities() const override;
	void                        dispToConsole(QString message, ConsoleMessageLevel level = STD_CONSOLE_MESSAGE) override;
	ccHObject*                  dbRootObject() override;
	void                        redrawAll(bool only2D = false) override;
	void                        refreshAll(bool only2D = false) override;
	void                        enableAll() override;
	void                        disableAll() override;
	void                        disableAllBut(ccGLWindowInterface* win) override;
	void                        updateUI() override
	{
	}
	void freezeUI(bool state) override
	{
	}
	void setView(CC_VIEW_ORIENTATION view) override;
	void toggleActiveWindowCenteredPerspective() override;
	void toggleActiveWindowCustomLight() override;
	void toggleActiveWindowSunLight() override;
	void toggleActiveWindowViewerBasedPerspective() override;
	void zoomOnSelectedEntities() override
	{
		zoomOnSelectedEntity();
	}
	void                        increasePointSize() override;
	void                        decreasePointSize() override;
	ccUniqueIDGenerator::Shared getUniqueIDGenerator() override;

  protected:
	//! Shows display parameters dialog
	void showDisplayParameters();

	//! Updates display to match display parameters
	void updateDisplay();

	//! Selects entity
	void selectEntity(ccHObject* entity);

	//! Delete selected entity
	void doActionDeleteSelectedEntity();

	//! Slot called when the exclusive full screen mode is called
	void onExclusiveFullScreenToggled(bool);

	void doActionEditCamera();
	void toggleSunLight(bool);
	void toggleCustomLight(bool);
	void toggleStereoMode(bool);
	void toggleFullScreen(bool);
	void toggleRotationAboutVertAxis();
	void doActionAbout();
	void doActionDisplayShortcuts();
	void setPivotAlwaysOn();
	void setPivotRotationOnly();
	void setPivotOff();
	void setOrthoView();
	void setCenteredPerspectiveView();
	void setViewerPerspectiveView();
	void setGlobalZoom() override;
	void zoomOnSelectedEntity();

	// default views
	void setFrontView();
	void setBottomView();
	void setTopView();
	void setBackView();
	void setLeftView();
	void setRightView();
	void setIsoView1();
	void setIsoView2();

	// selected entity properties
	void toggleColorsShown(bool);
	void toggleNormalsShown(bool);
	void toggleMaterialsShown(bool);
	void toggleScalarShown(bool);
	void toggleColorbarShown(bool);
	void changeCurrentScalarField(bool);

	// 3D mouse
	void on3DMouseMove(std::vector<float>&);
	void on3DMouseKeyUp(int);
	void on3DMouseKeyDown(int);
	void on3DMouseCMDKeyDown(int);
	void on3DMouseCMDKeyUp(int);
	void on3DMouseReleased();
	void enable3DMouse(bool state);

	// GL filters
	void doEnableGLFilter();
	void doDisableGLFilter();

  protected: // methods
	//! Loads plugins (from files)
	void loadPlugins();

	//! Makes the GL frame background gradient match the OpenGL window one
	void updateGLFrameGradient();

	//! Updates perspective UI elements
	void reflectPerspectiveState();

	//! Updates pivot UI elements
	void reflectPivotVisibilityState();

	//! Updates lights UI elements
	void reflectLightsState();

	//! Checks whether stereo mode can be stopped (if necessary) or not
	bool checkStereoMode();

  protected: // members
	//! Releases any connected 3D mouse (if any)
	void release3DMouse();

	//! Associated GL context
	ccGLWindowInterface* m_glWindow;

	//! Currently selected object
	ccHObject* m_selectedObject;

	//! 3D mouse handler
	Mouse3DInput* m_3dMouseInput;

	//! Gamepad handler
	ccGamepadManager* m_gamepadManager;

  private:
	//! Associated GUI
	Ui::ccViewerClass ui;
};
