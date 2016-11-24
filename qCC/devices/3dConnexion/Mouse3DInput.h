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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef MOUSE_3D_INPUT_HEADER
#define MOUSE_3D_INPUT_HEADER

//CCLib
#include <CCConst.h>

//qCC_db
#include <ccGLMatrix.h>

//Qt
#include <QObject>

//system
#include <vector>

class ccGLWindow;

//! 3DxWare driver wrapper for 3D mouse handling
class Mouse3DInput : public QObject
{
	Q_OBJECT

public:

	//! Default constructor
	explicit Mouse3DInput(QObject* parent);
	//! Destructor
	virtual ~Mouse3DInput();

	//! Attempts to connect with the 3DxWare driver
	bool connect(QWidget* mainWidget, QString appName);
	//! Disconnects from the 3DxWare driver
	void disconnectDriver();

	//! Default key codes
	enum VirtualKey
	{
		V3DK_INVALID				= 0,
		V3DK_MENU					= 1,
		V3DK_FIT,
		V3DK_TOP, V3DK_LEFT, V3DK_RIGHT, V3DK_FRONT, V3DK_BOTTOM, V3DK_BACK,
		V3DK_CW, V3DK_CCW,
		V3DK_ISO1, V3DK_ISO2,
		V3DK_1, V3DK_2, V3DK_3, V3DK_4, V3DK_5, V3DK_6, V3DK_7, V3DK_8, V3DK_9, V3DK_10,
		V3DK_ESC, V3DK_ALT, V3DK_SHIFT, V3DK_CTRL,
		V3DK_ROTATE, V3DK_PANZOOM, V3DK_DOMINANT,
		V3DK_PLUS, V3DK_MINUS
	};

	//! Converts 'rotation' part of motion data to a rotation matrix
	static void GetMatrix(const std::vector<float>& motionData, ccGLMatrixd& mat);

	//! Applies motion data to a given 3D window
	static void Apply(const std::vector<float>& motionData, ccGLWindow* win);

	//! Called when a new system message is available
	/** For 'internal' use only
	**/
	bool onSiEvent(void* siGetEventData);

signals:

	void sigMove3d(std::vector<float>& motionData);
	void sigReleased();
	void sigOn3dmouseKeyDown(int virtualKeyCode);
	void sigOn3dmouseKeyUp(int virtualKeyCode);

protected:

	//! Called with the processed motion data when a 3D mouse event is received
	/** The default implementation emits a sigMove3d signal with the motion data
	*/
	virtual	void move3d(std::vector<float>& motionData);

	//! Called when a 3D mouse key is pressed
	/** The default implementation emits a sigOn3dmouseKeyDown signal with the key code.
	*/
	virtual void on3dmouseKeyDown(int virtualKeyCode);

	//! Called when a 3D mouse key is released
	/** The default implementation emits a sigOn3dmouseKeyUp signal with the key code.
	**/
	virtual void on3dmouseKeyUp(int virtualKeyCode);

	//! 3DxWare handle
	void* m_siHandle;
};

#endif //MOUSE_3D_INPUT_HEADER
