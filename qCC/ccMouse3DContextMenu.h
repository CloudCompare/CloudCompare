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

#ifndef CC_MOUSE_3D_CONTEXT_MENU_HEADER
#define CC_MOUSE_3D_CONTEXT_MENU_HEADER

//Qt
#include <QMenu>

class QAction;
class ccGLWindow;
class Mouse3DParameters;

//! 3D mouse context menu
class ccMouse3DContextMenu : public QMenu
{
	Q_OBJECT

public:

	//! Default constructor
	ccMouse3DContextMenu(Mouse3DParameters* params, ccGLWindow* win, QWidget* parent=0);

protected slots:

	void rotationModeToggled(bool);
	void panZoomModeToggled(bool);
	void lockHorizonToggled(bool);
	void dominantModeToggled(bool);
	void speedModeChanged();
	void objectModeTriggered();
	void cameraModeTriggered();
	void rotationCenterVisibilityChanged();

protected:

	//! Number of speed control actions
	static const int SPEED_ACTION_COUNT = 5;

	//! Rotation mode checkable action
	QAction* m_rotationMode;
	QAction* m_panZoomMode;

	//! Speed control actions (from slowest to fastest)
	QAction* m_speedActions[SPEED_ACTION_COUNT];

	// Rotation center actions
	QAction* m_autoRotationCenter;
	QAction* m_selectedItemAsRotationCenter;
	QAction* m_alwaysShowRotationCenter;
	QAction* m_showRotationCenterOnMotion;
	QAction* m_alwaysHideRotationCenter;

	// Nabigation modes
	QAction* m_objectMode;
	QAction* m_cameraMode;

	//! Lock horizon
	QAction* m_lockHorizon;
	//! Dominant mode
	QAction* m_dominantMode;

	//! Associated parameters
	Mouse3DParameters* m_params;

	//! Active GL window
	ccGLWindow* m_glWindow;

};

#endif //CC_MOUSE_3D_CONTEXT_MENU_HEADER
