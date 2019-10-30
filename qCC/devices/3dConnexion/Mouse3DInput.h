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

	enum V3DCMD
	{
		V3DCMD_NOOP = 0,
		V3DCMD_MENU_OPTIONS = 1,
		V3DCMD_VIEW_FIT = 2,
		V3DCMD_VIEW_TOP = 3,
		V3DCMD_VIEW_LEFT = 4,
		V3DCMD_VIEW_RIGHT = 5,
		V3DCMD_VIEW_FRONT = 6,
		V3DCMD_VIEW_BOTTOM = 7,
		V3DCMD_VIEW_BACK = 8,
		V3DCMD_VIEW_ROLLCW = 9,
		V3DCMD_VIEW_ROLLCCW = 10,
		V3DCMD_VIEW_ISO1 = 11,
		V3DCMD_VIEW_ISO2 = 12,
		V3DCMD_KEY_F1 = 13,
		V3DCMD_KEY_F2 = 14,
		V3DCMD_KEY_F3 = 15,
		V3DCMD_KEY_F4 = 16,
		V3DCMD_KEY_F5 = 17,
		V3DCMD_KEY_F6 = 18,
		V3DCMD_KEY_F7 = 19,
		V3DCMD_KEY_F8 = 20,
		V3DCMD_KEY_F9 = 21,
		V3DCMD_KEY_F10 = 22,
		V3DCMD_KEY_F11 = 23,
		V3DCMD_KEY_F12 = 24,
		V3DCMD_KEY_ESC = 25,
		V3DCMD_KEY_ALT = 26,
		V3DCMD_KEY_SHIFT = 27,
		V3DCMD_KEY_CTRL = 28,
		V3DCMD_FILTER_ROTATE = 29,
		V3DCMD_FILTER_PANZOOM = 30,
		V3DCMD_FILTER_DOMINANT = 31,
		V3DCMD_SCALE_PLUS = 32,
		V3DCMD_SCALE_MINUS = 33,
		V3DCMD_VIEW_SPINCW = 34,
		V3DCMD_VIEW_SPINCCW = 35,
		V3DCMD_VIEW_TILTCW = 36,
		V3DCMD_VIEW_TILTCCW = 37,
		V3DCMD_MENU_POPUP = 38,
		V3DCMD_MENU_BUTTONMAPPINGEDITOR = 39,
		V3DCMD_MENU_ADVANCEDSETTINGSEDITOR = 40,
		V3DCMD_MOTIONMACRO_ZOOM = 41,
		V3DCMD_MOTIONMACRO_ZOOMOUT_CURSORTOCENTER = 42,
		V3DCMD_MOTIONMACRO_ZOOMIN_CURSORTOCENTER = 43,
		V3DCMD_MOTIONMACRO_ZOOMOUT_CENTERTOCENTER = 44,
		V3DCMD_MOTIONMACRO_ZOOMIN_CENTERTOCENTER = 45,
		V3DCMD_MOTIONMACRO_ZOOMOUT_CURSORTOCURSOR = 46,
		V3DCMD_MOTIONMACRO_ZOOMIN_CURSORTOCURSOR = 47,
		V3DCMD_VIEW_QZ_IN = 48,
		V3DCMD_VIEW_QZ_OUT = 49,
		V3DCMD_KEY_ENTER = 50,
		V3DCMD_KEY_DELETE = 51,
		V3DCMD_KEY_F13 = 52,
		V3DCMD_KEY_F14 = 53,
		V3DCMD_KEY_F15 = 54,
		V3DCMD_KEY_F16 = 55,
		V3DCMD_KEY_F17 = 56,
		V3DCMD_KEY_F18 = 57,
		V3DCMD_KEY_F19 = 58,
		V3DCMD_KEY_F20 = 59,
		V3DCMD_KEY_F21 = 60,
		V3DCMD_KEY_F22 = 61,
		V3DCMD_KEY_F23 = 62,
		V3DCMD_KEY_F24 = 63,
		V3DCMD_KEY_F25 = 64,
		V3DCMD_KEY_F26 = 65,
		V3DCMD_KEY_F27 = 66,
		V3DCMD_KEY_F28 = 67,
		V3DCMD_KEY_F29 = 68,
		V3DCMD_KEY_F30 = 69,
		V3DCMD_KEY_F31 = 70,
		V3DCMD_KEY_F32 = 71,
		V3DCMD_KEY_F33 = 72,
		V3DCMD_KEY_F34 = 73,
		V3DCMD_KEY_F35 = 74,
		V3DCMD_KEY_F36 = 75,
		V3DCMD_VIEW_1 = 76,
		V3DCMD_VIEW_2 = 77,
		V3DCMD_VIEW_3 = 78,
		V3DCMD_VIEW_4 = 79,
		V3DCMD_VIEW_5 = 80,
		V3DCMD_VIEW_6 = 81,
		V3DCMD_VIEW_7 = 82,
		V3DCMD_VIEW_8 = 83,
		V3DCMD_VIEW_9 = 84,
		V3DCMD_VIEW_10 = 85,
		V3DCMD_VIEW_11 = 86,
		V3DCMD_VIEW_12 = 87,
		V3DCMD_VIEW_13 = 88,
		V3DCMD_VIEW_14 = 89,
		V3DCMD_VIEW_15 = 90,
		V3DCMD_VIEW_16 = 91,
		V3DCMD_VIEW_17 = 92,
		V3DCMD_VIEW_18 = 93,
		V3DCMD_VIEW_19 = 94,
		V3DCMD_VIEW_20 = 95,
		V3DCMD_VIEW_21 = 96,
		V3DCMD_VIEW_22 = 97,
		V3DCMD_VIEW_23 = 98,
		V3DCMD_VIEW_24 = 99,
		V3DCMD_VIEW_25 = 100,
		V3DCMD_VIEW_26 = 101,
		V3DCMD_VIEW_27 = 102,
		V3DCMD_VIEW_28 = 103,
		V3DCMD_VIEW_29 = 104,
		V3DCMD_VIEW_30 = 105,
		V3DCMD_VIEW_31 = 106,
		V3DCMD_VIEW_32 = 107,
		V3DCMD_VIEW_33 = 108,
		V3DCMD_VIEW_34 = 109,
		V3DCMD_VIEW_35 = 110,
		V3DCMD_VIEW_36 = 111,
		V3DCMD_SAVE_VIEW_1 = 112,
		V3DCMD_SAVE_VIEW_2 = 113,
		V3DCMD_SAVE_VIEW_3 = 114,
		V3DCMD_SAVE_VIEW_4 = 115,
		V3DCMD_SAVE_VIEW_5 = 116,
		V3DCMD_SAVE_VIEW_6 = 117,
		V3DCMD_SAVE_VIEW_7 = 118,
		V3DCMD_SAVE_VIEW_8 = 119,
		V3DCMD_SAVE_VIEW_9 = 120,
		V3DCMD_SAVE_VIEW_10 = 121,
		V3DCMD_SAVE_VIEW_11 = 122,
		V3DCMD_SAVE_VIEW_12 = 123,
		V3DCMD_SAVE_VIEW_13 = 124,
		V3DCMD_SAVE_VIEW_14 = 125,
		V3DCMD_SAVE_VIEW_15 = 126,
		V3DCMD_SAVE_VIEW_16 = 127,
		V3DCMD_SAVE_VIEW_17 = 128,
		V3DCMD_SAVE_VIEW_18 = 129,
		V3DCMD_SAVE_VIEW_19 = 130,
		V3DCMD_SAVE_VIEW_20 = 131,
		V3DCMD_SAVE_VIEW_21 = 132,
		V3DCMD_SAVE_VIEW_22 = 133,
		V3DCMD_SAVE_VIEW_23 = 134,
		V3DCMD_SAVE_VIEW_24 = 135,
		V3DCMD_SAVE_VIEW_25 = 136,
		V3DCMD_SAVE_VIEW_26 = 137,
		V3DCMD_SAVE_VIEW_27 = 138,
		V3DCMD_SAVE_VIEW_28 = 139,
		V3DCMD_SAVE_VIEW_29 = 140,
		V3DCMD_SAVE_VIEW_30 = 141,
		V3DCMD_SAVE_VIEW_31 = 142,
		V3DCMD_SAVE_VIEW_32 = 143,
		V3DCMD_SAVE_VIEW_33 = 144,
		V3DCMD_SAVE_VIEW_34 = 145,
		V3DCMD_SAVE_VIEW_35 = 146,
		V3DCMD_SAVE_VIEW_36 = 147,
		V3DCMD_KEY_TAB = 148,
		V3DCMD_KEY_SPACE = 149,
		V3DCMD_MENU_1 = 150,
		V3DCMD_MENU_2 = 151,
		V3DCMD_MENU_3 = 152,
		V3DCMD_MENU_4 = 153,
		V3DCMD_MENU_5 = 154,
		V3DCMD_MENU_6 = 155,
		V3DCMD_MENU_7 = 156,
		V3DCMD_MENU_8 = 157,
		V3DCMD_MENU_9 = 158,
		V3DCMD_MENU_10 = 159,
		V3DCMD_MENU_11 = 160,
		V3DCMD_MENU_12 = 161,
		V3DCMD_MENU_13 = 162,
		V3DCMD_MENU_14 = 163,
		V3DCMD_MENU_15 = 164,
		V3DCMD_MENU_16 = 165,
		/* Add here as needed. Don't change any values that may be in use */
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
	void sigOn3dmouseCMDKeyDown(int virtualCMDCode);
	void sigOn3dmouseKeyUp(int virtualKeyCode);
	void sigOn3dmouseCMDKeyUp(int virtualCMDCode);

protected:

	//! Called with the processed motion data when a 3D mouse event is received
	/** The default implementation emits a sigMove3d signal with the motion data
	*/
	virtual	void move3d(std::vector<float>& motionData);

	//! Called when a 3D mouse key is pressed
	/** The default implementation emits a sigOn3dmouseKeyDown signal with the key code.
	*/
	virtual void on3dmouseKeyDown(int virtualKeyCode);

	//! Called when a 3D mouse key is pressed after translation to CMD
	/** The default implementation emits a sigOn3dmouseKeyDown signal with the key code.
	*/
	virtual void on3dmouseCMDKeyDown(int virtualCMDCode);

	//! Called when a 3D mouse key is released
	/** The default implementation emits a sigOn3dmouseKeyUp signal with the key code.
	**/
	virtual void on3dmouseKeyUp(int virtualKeyCode);

	//! Called when a 3D mouse key is released after translation to CMD
	/** The default implementation emits a sigOn3dmouseKeyUp signal with the key code.
	**/
	virtual void on3dmouseCMDKeyUp(int virtualCMDCode);

	//! 3DxWare handle
	void* m_siHandle;
};

#endif //MOUSE_3D_INPUT_HEADER
