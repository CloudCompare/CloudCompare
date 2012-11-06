//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE LIGHT VIEWER                            #
//#                                                                        #
//#  This project has been initated under funding from ANR/CIFRE           #
//#  This program is free software; you can redistribute it and/or modify  #
//#                                                                        #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#      +++ COPYRIGHT: EDF R&D + TELECOM ParisTech (ENST-TSI) +++         #
//#                                                                        #
//##########################################################################

#ifndef CCVIEWER_H
#define CCVIEWER_H

//Qt
#include <QtGui/QMainWindow>

//GUIs
#include <ui_ccviewer.h>
//#include <ui_ccviewerAbout.h>

class ccGLWindow;
class ccHObject;

//! Application main window
class ccViewer : public QMainWindow
{
	Q_OBJECT

public:
	//! Default constructor
	ccViewer(QWidget *parent = 0, Qt::WindowFlags flags = 0);

	//! Default destructor
	~ccViewer();

	//! Adds entity to display db
	void addToDB(ccHObject* entity);

	//! Checks for loaded entities
	/** If none, a message is displayed to invite the user
        to drag & drop files.
    **/
	bool checkForLoadedEntities();

public slots:

	//! Tries to load (and then adds to main db) a list of entity (files)
    /** \param filenames list of all filenames
    **/
    void addToDB(const QStringList& filenames);

protected slots:

	//! Shows display parameters dialog
	void showDisplayParameters();

	//! Updates display to match display parameters
	void updateDisplay();

	//! Selects entity
	void selectEntity(int uniqueID);

	//! Delete selected entity
	void doActionDeleteSelectedEntity();

    void doActionEditCamera();
    void toggleSunLight(bool);
    void toggleCustomLight(bool);
    void togglePerspectiveOff(bool);
    void toggleCenteredPerspective(bool);
    void toggleViewerBasedPerspective(bool);
	void toggleFullScreen(bool);
    void doActionAbout();
    void doActionDisplayShortcuts();

	//default views
	void setFrontView();
    void setBottomView();
    void setTopView();
    void setBackView();
    void setLeftView();
    void setRightView();

	//selected entity properties
	void toggleColorsShown(bool);
	void toggleNormalsShown(bool);
	void toggleScalarShown(bool);
	void toggleColorbarShown(bool);
	void changeCurrentScalarField(bool);

protected:

	//! Makes the GL frame background gradient match the OpenGL window one
	void updateGLFrameGradient();

	//! Sets perspective state (synch. UI by the way)
	void setPerspective(bool enabled, bool centered);

	//! Associated GL context
	ccGLWindow* m_glWindow;

	//! Currently selected object
	ccHObject* m_selectedObject;

private:
	//! Associated GUI
	Ui::ccViewerClass ui;
};

#endif // CCVIEWER_H
