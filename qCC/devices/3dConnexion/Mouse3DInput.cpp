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

/** This file is inspired from the Qt wrapper for 3dConnexion devices graciously shared by Dabid Dibben:
	http://www.codegardening.com/2011/02/using-3dconnexion-mouse-with-qt.html
**/

#include "Mouse3DInput.h"

//qCC_db
#include <ccLog.h>
//qCC_gl
#include <ccGLWindow.h>
//CCLib
#include <CCPlatform.h>

//Qt
#include <QApplication>

//system
#include <assert.h>
#include <math.h>
#ifdef CC_WINDOWS
#include <windows.h>
#endif

//3DxWare
#include <si.h>
#include <siapp.h>
#include <spwmath.h>

//! Object angular velocity per mouse tick (in radians per ms per count)
static const double c_3dmouseAngularVelocity = 1.0e-6;

//unique instance
static Mouse3DInput* s_mouseInputInstance = 0;

#include <QAbstractNativeEventFilter>
class RawInputEventFilter : public QAbstractNativeEventFilter
{
public:
	virtual bool nativeEventFilter(const QByteArray& eventType, void* msg, long* result) Q_DECL_OVERRIDE
	{
		if (!s_mouseInputInstance || !msg)
		{
			return false;
		}

		SiGetEventData eData;	//Platform-specific event data
#ifdef CC_WINDOWS
		MSG* messageStruct = static_cast<MSG*>(msg);
		SiGetEventWinInit(&eData, messageStruct->message, messageStruct->wParam, messageStruct->lParam);
#endif
		return s_mouseInputInstance->onSiEvent(&eData);
	}
};

Mouse3DInput::Mouse3DInput(QObject* parent)
    : QObject(parent)
    , m_siHandle(SI_NO_HANDLE)
{
	//register current instance
	assert(s_mouseInputInstance == 0);
	s_mouseInputInstance = this;

	//setup event filter
	static RawInputEventFilter s_rawInputEventFilter;
	qApp->installNativeEventFilter(&s_rawInputEventFilter);
}

Mouse3DInput::~Mouse3DInput()
{
	//unregister current instance
	if (s_mouseInputInstance == this)
	{
		s_mouseInputInstance = 0;
	}
}

bool Mouse3DInput::connect(QWidget* mainWidget, QString appName)
{
	if (!mainWidget)
	{
		assert(false);
		return false;
	}

	/*** Attempt to connect with the 3DxWare driver ***/
	assert(m_siHandle == SI_NO_HANDLE);

	SiInitialize();

	//Platform-specific device data
	SiOpenData oData;
	SiOpenWinInit(&oData, (HWND)mainWidget->winId() );
	//3DxWare device handle
	m_siHandle = SiOpen(qPrintable(appName), SI_ANY_DEVICE, SI_NO_MASK, SI_EVENT, &oData);

	if (m_siHandle == SI_NO_HANDLE)
	{
		/* Get and display initialization error */
		ccLog::Warning("[3D Mouse] Could not open a 3DxWare device");
		return false;
	}

	//to avoid drift
	SiRezero(m_siHandle);

	SiDevInfo info;
	if (SiGetDeviceInfo(m_siHandle, &info) == SPW_NO_ERROR)
	{
		if (info.majorVersion == 0 && info.minorVersion == 0)
		{
			/* Not a real device */
			ccLog::Warning("[3D Mouse] Couldn't find a connected device");
			return false;
		}

		SiDeviceName name;
		SiGetDeviceName(m_siHandle, &name);
		ccLog::Print(QString("[3D Mouse] Device: %1 (%2 buttons) - firmware v%3.%4").arg(name.name).arg(info.numButtons).arg(info.majorVersion).arg(info.minorVersion));
	}
	else
	{
		ccLog::Warning("[3D Mouse] Failed to retrieve device info?!");
	}

	return true;
}

void Mouse3DInput::disconnectDriver()
{
	if (m_siHandle != SI_NO_HANDLE)
	{
		SiClose(m_siHandle);
		SiTerminate();
		m_siHandle = SI_NO_HANDLE;
	}
}

bool Mouse3DInput::onSiEvent(void* siGetEventData)
{
	if (m_siHandle == SI_NO_HANDLE || !siGetEventData)
	{
		return false;
	}

	SiSpwEvent siEvent; ///3DxWare data event
	if (SiGetEvent (m_siHandle, 0, static_cast<SiGetEventData*>(siGetEventData), &siEvent) != SI_IS_EVENT)
	{
		return false;
	}

	switch (siEvent.type)
	{
	case SI_MOTION_EVENT:
		{
			const SiSpwData& eventData = siEvent.u.spwData;

			if (	eventData.mData[SI_TX] != 0
			    ||	eventData.mData[SI_TY] != 0
			    ||	eventData.mData[SI_TZ] != 0
			    ||	eventData.mData[SI_RX] != 0
			    ||	eventData.mData[SI_RY] != 0
			    ||	eventData.mData[SI_RZ] != 0 )
			{
				std::vector<float> axes(6);
				double ds = eventData.period * c_3dmouseAngularVelocity; //period is in ms
				//translation data
				axes[0] = - static_cast<float>(eventData.mData[SI_TX] * ds);
				axes[1] =   static_cast<float>(eventData.mData[SI_TY] * ds);
				axes[2] =   static_cast<float>(eventData.mData[SI_TZ] * ds);
				//rotation data
				axes[3] = - static_cast<float>(eventData.mData[SI_RX] * ds);
				axes[4] =   static_cast<float>(eventData.mData[SI_RY] * ds);
				axes[5] =   static_cast<float>(eventData.mData[SI_RZ] * ds);

				move3d(axes);
			}
		}
		break;

	case SI_ZERO_EVENT:
		//FIXME: too flickery!
		emit sigReleased();
		break;

	case SI_BUTTON_EVENT:
		{
			SPWuint32 buttonNumber = siEvent.u.hwButtonEvent.buttonNumber;
			if (buttonNumber != 0)
			{
				if (SiButtonPressed(&siEvent) > 0)
					on3dmouseKeyDown(buttonNumber);
				else if (SiButtonReleased(&siEvent) > 0)
					on3dmouseKeyUp(buttonNumber);
			}
		}
		break;

	case SI_BUTTON_PRESS_EVENT:
		on3dmouseKeyDown(siEvent.u.hwButtonEvent.buttonNumber);
		break;

	case SI_BUTTON_RELEASE_EVENT:
		on3dmouseKeyUp(siEvent.u.hwButtonEvent.buttonNumber);
		break;

	default:
		break;
	}

	return true;
}

void Mouse3DInput::move3d(std::vector<float>& motionData)
{
	emit sigMove3d(motionData);
}

void Mouse3DInput::on3dmouseKeyDown(int virtualKeyCode)
{
	emit sigOn3dmouseKeyDown(virtualKeyCode);
}

void Mouse3DInput::on3dmouseKeyUp(int virtualKeyCode)
{
	emit sigOn3dmouseKeyUp(virtualKeyCode);
}

void Mouse3DInput::GetMatrix(const std::vector<float>& vec, ccGLMatrixd& mat)
{
	assert(vec.size() == 6);

	float axis[3] = { -vec[3], vec[4], -vec[5] };

	Matrix Rd;
	SPW_ArbitraryAxisToMatrix(Rd, axis, 1.0f);

	for (unsigned i=0; i<3; ++i)
	{
		mat.getColumn(i)[0] = Rd[0][i];
		mat.getColumn(i)[1] = Rd[1][i];
		mat.getColumn(i)[2] = Rd[2][i];
	}
}

void Mouse3DInput::Apply(const std::vector<float>& motionData, ccGLWindow* win)
{
	assert(motionData.size() >= 6);

	//no active window?
	if (!win)
	{
		return;
	}

	//copy input parameters
	std::vector<float> vec = motionData;

	//view parameters
	bool objectMode = true;
	bool perspectiveView = win->getPerspectiveState(objectMode); //note: viewer based perspective IS 'camera mode'
	bool bubbleViewMode = win->bubbleViewModeEnabled();

	//panning or zooming
	if (!bubbleViewMode)
	{
		float& X = vec[0];
		float& Y = vec[1];
		float& Z = vec[2];

		//ccLog::Print(QString("Mouse translation: (%1,%2,%3)").arg(X).arg(Y).arg(Z));

		//Zoom: object moves closer/away (only for ortho. mode)
		if (!perspectiveView && fabs(Z) > ZERO_TOLERANCE)
		{
			win->updateZoom(1.0f - Z / 1.5f);
			Z = 0;
		}

		//Zoom & Panning: camera moves right/left + up/down + backward/forward (only for perspective mode)
		if (	fabs(X) > ZERO_TOLERANCE
		    ||	fabs(Y) > ZERO_TOLERANCE
		    ||	fabs(Z) > ZERO_TOLERANCE )
		{
			const ccViewportParameters& viewParams = win->getViewportParameters();

			float scale = static_cast<float>(std::min(win->glWidth(), win->glHeight()) * viewParams.pixelSize);
			if (perspectiveView)
			{
				float tanFOV = tan(static_cast<float>(viewParams.fov * CC_DEG_TO_RAD)/*/2*/);
				X *= tanFOV;
				Y *= tanFOV;
				scale /= win->computePerspectiveZoom();
			}
			else
			{
				scale /= win->getViewportParameters().zoom;
			}

			if (objectMode)
			{
				scale = -scale;
			}
			win->moveCamera(-X*scale, Y*scale, -Z*scale);
		}
	}

	//rotation
	if (	fabs(vec[3]) > ZERO_TOLERANCE
	    ||	fabs(vec[4]) > ZERO_TOLERANCE
	    ||	fabs(vec[5]) > ZERO_TOLERANCE)
	{
		//ccLog::Print(QString("Mouse rotation: (%1,%2,%3)").arg(vec[3]).arg(vec[4]).arg(vec[5]));

		//get corresponding rotation matrix
		ccGLMatrixd rotMat;
		if (!bubbleViewMode)
		{
			Mouse3DInput::GetMatrix(vec, rotMat);
			win->rotateBaseViewMat(objectMode ? rotMat : rotMat.inverse());
		}
		else
		{
			//Ry = horizontal
			//Rx = vertical

			//rotation about the sensor Z axis
			const ccGLMatrixd& viewMat = win->getViewportParameters().viewMat;
			CCVector3d axis = viewMat.getColumnAsVec3D(2);
			rotMat.initFromParameters(-vec[4], axis, CCVector3d(0, 0, 0));

			//rotation about the local X axis
			ccGLMatrixd rotX;
			rotX.initFromParameters(vec[3], CCVector3d(1, 0, 0), CCVector3d(0, 0, 0));
			rotMat = rotX * rotMat;
			win->rotateBaseViewMat(rotMat);
		}

		win->showPivotSymbol(true);
	}
	else
	{
		win->showPivotSymbol(false);
	}

	win->redraw();
}
