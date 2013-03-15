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
class Mouse3DParameters;

//! 3D mouse context menu
class ccMouse3DContextMenu : public QMenu
{
	Q_OBJECT

public:

	//! Default constructor
	ccMouse3DContextMenu(Mouse3DParameters* params, QWidget* parent=0);

protected slots:

	void rotateCheckBoxToggled(bool);
	void panZoomCheckBoxToggled(bool);
	void speedModeChanged(bool);

protected:

	//! Number of speed control actions
	static const int SPEED_ACTION_COUNT = 5;

	//! Speed control actions (from slowest to fastest)
	QAction* m_speedActions[SPEED_ACTION_COUNT];

	// checkboxes
	QAction* m_rotateCheckbox;
	QAction* m_panZoomCheckbox;

	//! Associated parameters
	Mouse3DParameters* m_params;

};

#endif //CC_MOUSE_3D_CONTEXT_MENU_HEADER
