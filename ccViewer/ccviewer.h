#pragma once

//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE LIGHT VIEWER                            #
//#                                                                        #
//#  This project has been initiated under funding from ANR/CIFRE          #
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
//#      +++ COPYRIGHT: EDF R&D + TELECOM ParisTech (ENST-TSI) +++         #
//#                                                                        #
//##########################################################################

//Qt
#include <QMainWindow>
#include <QStringList>

//CCPluginAPI
#include <ccMainAppInterface.h>

//GUIs
#include <ui_ccviewer.h>

//System
#include <set>

class ccGLWindowInterface;
class ccHObject;
class Mouse3DInput;
class ccGamepadManager;

/**
 * \class ccViewer
 * \brief Lightweight CloudCompare 3D point cloud and mesh viewer
 *
 * A standalone application for visualizing and interacting with 3D data.
 * Provides basic functionality for loading, displaying, and manipulating
 * point clouds, meshes, and other 3D entities.
 *
 * \note Inherits from QMainWindow and implements ccMainAppInterface
 * \warning Designed for lightweight viewing, not full editing
 */
class ccViewer : public QMainWindow, public ccMainAppInterface
{
    Q_OBJECT

public:
    /**
     * \brief Constructs the ccViewer main window
     *
     * \param parent Parent widget
     * \param flags Window flags for customizing window appearance
     *
     * \note Initializes the viewer with default settings
     */
    ccViewer(QWidget *parent = 0, Qt::WindowFlags flags = 0);

    /**
     * \brief Destructor
     *
     * Cleans up resources and closes the viewer
     */
    ~ccViewer() override;

    /**
     * \brief Adds an entity to the display database
     *
     * \param entity Pointer to the entity to be added
     * \param updateZoom Whether to adjust zoom to fit the new entity
     * \param autoExpandDBTree Whether to automatically expand the database tree
     * \param checkDimensions Whether to perform dimension checks
     * \param autoRedraw Whether to automatically redraw the scene
     *
     * \note Overrides ccMainAppInterface method
     */
    void addToDB(   ccHObject* entity,
                    bool updateZoom = false,
                    bool autoExpandDBTree = true,
                    bool checkDimensions = false,
                    bool autoRedraw = true) override;

    /**
     * \brief Removes an entity from the display database
     *
     * \param obj Pointer to the entity to be removed
     * \param autoDelete Whether to automatically delete the entity
     *
     * \note Overrides ccMainAppInterface method
     */
    void removeFromDB(ccHObject* obj, bool autoDelete = true) override;

    /**
     * \brief Checks if any entities are loaded
     *
     * \return bool True if entities are loaded, false otherwise
     *
     * \note Displays a message to invite drag & drop if no entities are loaded
     */
    bool checkForLoadedEntities();

    /**
     * \brief Loads and adds multiple files to the database
     *
     * \param filenames List of file paths to load
     * \return ccHObject* Pointer to the first loaded entity/group
     */
    ccHObject* addToDB(QStringList filenames);

public: // ccMainInterface compliance methods
    /**
     * \brief Gets the main window
     *
     * \return QMainWindow* Pointer to the main window
     */
    QMainWindow* getMainWindow() override { return this; }

    /**
     * \brief Gets the active OpenGL window
     *
     * \return ccGLWindowInterface* Pointer to the active GL window
     */
    ccGLWindowInterface* getActiveGLWindow() override { return m_glWindow; }

    /**
     * \brief Loads a single file
     *
     * \param filename Path to the file to load
     * \param silent Whether to suppress user-facing messages
     * \return ccHObject* Pointer to the loaded entity
     */
    ccHObject* loadFile(QString filename, bool silent) override { return addToDB(QStringList{ filename }); }

    /**
     * \brief Sets the selection state of an entity
     *
     * \param obj Pointer to the entity
     * \param selected Whether the entity should be selected
     */
    void setSelectedInDB(ccHObject* obj, bool selected) override {}

    /**
     * \brief Gets the currently selected entities
     *
     * \return const ccHObject::Container& Reference to selected entities
     */
    const ccHObject::Container& getSelectedEntities() const override;

    /**
     * \brief Displays a message in the console
     *
     * \param message Message to display
     * \param level Console message level
     */
    void dispToConsole(QString message, ConsoleMessageLevel level = STD_CONSOLE_MESSAGE) override;

    /**
     * \brief Gets the root object of the database
     *
     * \return ccHObject* Pointer to the root database object
     */
    ccHObject* dbRootObject() override;

    // Various UI and rendering methods
    void redrawAll(bool only2D = false) override;
    void refreshAll(bool only2D = false) override;
    void enableAll() override;
    void disableAll() override;
    void disableAllBut(ccGLWindowInterface* win) override;
    void updateUI() override {}
    void freezeUI(bool state) override {}
    void setView(CC_VIEW_ORIENTATION view) override;
    void toggleActiveWindowCenteredPerspective() override;
    void toggleActiveWindowCustomLight() override;
    void toggleActiveWindowSunLight() override;
    void toggleActiveWindowViewerBasedPerspective() override;
    void zoomOnSelectedEntities() override { zoomOnSelectedEntity(); }
    void increasePointSize() override;
    void decreasePointSize() override;
    ccUniqueIDGenerator::Shared getUniqueIDGenerator() override;

protected:
    // Various UI and interaction methods
    void showDisplayParameters();
    void updateDisplay();
    void selectEntity(ccHObject* entity);
    void doActionDeleteSelectedEntity();
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

    // Predefined view setters
    void setFrontView();
    void setBottomView();
    void setTopView();
    void setBackView();
    void setLeftView();
    void setRightView();
    void setIsoView1();
    void setIsoView2();

    // Entity property toggles
    void toggleColorsShown(bool);
    void toggleNormalsShown(bool);
    void toggleMaterialsShown(bool);
    void toggleScalarShown(bool);
    void toggleColorbarShown(bool);
    void changeCurrentScalarField(bool);

    // 3D mouse interaction methods
    void on3DMouseMove(std::vector<float>&);
    void on3DMouseKeyUp(int);
    void on3DMouseKeyDown(int);
    void on3DMouseCMDKeyDown(int);
    void on3DMouseCMDKeyUp(int);
    void on3DMouseReleased();
    void enable3DMouse(bool state);

    // GL filter methods
    void doEnableGLFilter();
    void doDisableGLFilter();

protected: // methods
    /**
     * \brief Loads plugins from files
     *
     * \note Dynamically loads available plugins
     */
    void loadPlugins();

    /**
     * \brief Updates the GL frame background gradient
     *
     * Synchronizes the background gradient with the OpenGL window
     */
    void updateGLFrameGradient();

    /**
     * \brief Reflects the current perspective state in the UI
     */
    void reflectPerspectiveState();

    /**
     * \brief Reflects the pivot visibility state in the UI
     */
    void reflectPivotVisibilityState();

    /**
     * \brief Reflects the current lights state in the UI
     */
    void reflectLightsState();

    /**
     * \brief Checks if stereo mode can be stopped
     *
     * \return bool True if stereo mode can be stopped, false otherwise
     */
    bool checkStereoMode();

protected: //members

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
