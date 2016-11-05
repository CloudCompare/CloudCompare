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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#ifndef GAMEPAD_INPUT_HEADER
#define GAMEPAD_INPUT_HEADER

//CCLib
#include <CCConst.h>

//qCC_db
#include <ccGLMatrix.h>

//Qt
#include <QGamepad>
#include <QTimer>

class ccGLWindow;

//! Gaempad handler
class GamepadInput : public QGamepad
{
	Q_OBJECT

public:

	//! Default constructor
	explicit GamepadInput(QObject* parent = 0);
	//! Destructor
	virtual ~GamepadInput();

	void start();
	void stop();

	//! Updates a window with the current gamepad state
	void update(ccGLWindow* win);

signals:

	void updated();

protected slots:

	void updateInternalState();

protected:

	//! Timer to poll the gamepad state
	QTimer m_timer;

	//! Last state
	CCVector3 m_panning;
	bool m_hasPanning;
	CCVector3 m_translation;
	bool m_hasTranslation;
	ccGLMatrixd m_rotation;
	bool m_hasRotation;
	float m_zoom;
};

#endif //MOUSE_3D_INPUT_HEADER
