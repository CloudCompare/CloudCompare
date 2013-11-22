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

/** This file is inspired from the Qt wrapper for 3dConnexion devices graciously shared by Dabid Dibben:
	http://www.codegardening.com/2011/02/using-3dconnexion-mouse-with-qt.html
**/

#include "Mouse3DInput.h"

//qCC_db
#include <ccPlatform.h>

//Qt
#include <QApplication>

//system
#include <assert.h>
#include <math.h>

#define LOGITECH_VENDOR_ID 0x46d
#define _CONSTANT_INPUT_PERIOD 0

#ifndef RIDEV_DEVNOTIFY
#define RIDEV_DEVNOTIFY 0x00002000
#endif

#ifdef CC_ENV_64
typedef unsigned __int64 QWORD;
#endif

//! Object angular velocity per mouse tick: 0.008 milliradians per second per count
static const double c_3dmouseAngularVelocity = 8.0e-6;
static const int c_defaultTimeToLive = 5;

enum e3dconnexion_pid
{
	eSpacePilot					= 0xc625,
	eSpaceNavigator				= 0xc626,
	eSpaceExplorer				= 0xc627,
	eSpaceNavigatorForNotebooks	= 0xc628,
	eSpacePilotPRO				= 0xc629
};

static const Mouse3DInput::VirtualKey SpaceExplorerKeys[] =
{
	Mouse3DInput::V3DK_INVALID, // there is no button 0
	Mouse3DInput::V3DK_1,
	Mouse3DInput::V3DK_2,
	Mouse3DInput::V3DK_TOP,
	Mouse3DInput::V3DK_LEFT,
	Mouse3DInput::V3DK_RIGHT,
	Mouse3DInput::V3DK_FRONT,
	Mouse3DInput::V3DK_ESC,
	Mouse3DInput::V3DK_ALT,
	Mouse3DInput::V3DK_SHIFT,
	Mouse3DInput::V3DK_CTRL,
	Mouse3DInput::V3DK_FIT,
	Mouse3DInput::V3DK_MENU,
	Mouse3DInput::V3DK_PLUS,
	Mouse3DInput::V3DK_MINUS,
	Mouse3DInput::V3DK_ROTATE
};

static const Mouse3DInput::VirtualKey SpacePilotKeys[] =
{
	Mouse3DInput::V3DK_INVALID,
	Mouse3DInput::V3DK_1,
	Mouse3DInput::V3DK_2,
	Mouse3DInput::V3DK_3,
	Mouse3DInput::V3DK_4,
	Mouse3DInput::V3DK_5,
	Mouse3DInput::V3DK_6,
	Mouse3DInput::V3DK_TOP,
	Mouse3DInput::V3DK_LEFT,
	Mouse3DInput::V3DK_RIGHT,
	Mouse3DInput::V3DK_FRONT,
	Mouse3DInput::V3DK_ESC,
	Mouse3DInput::V3DK_ALT,
	Mouse3DInput::V3DK_SHIFT,
	Mouse3DInput::V3DK_CTRL,
	Mouse3DInput::V3DK_FIT,
	Mouse3DInput::V3DK_MENU,
	Mouse3DInput::V3DK_PLUS,
	Mouse3DInput::V3DK_MINUS,
	Mouse3DInput::V3DK_DOMINANT,
	Mouse3DInput::V3DK_ROTATE
};

struct tag_VirtualKeys
{
	e3dconnexion_pid pid;
	size_t nKeys;
	Mouse3DInput::VirtualKey *vkeys;
};

static const tag_VirtualKeys Mouse3DVirtualKeys[] =
{
	eSpacePilot ,
	sizeof(SpacePilotKeys)/sizeof(SpacePilotKeys[0]), 
	const_cast<Mouse3DInput::VirtualKey*>(SpacePilotKeys),
	eSpaceExplorer,
	sizeof(SpaceExplorerKeys)/sizeof(SpaceExplorerKeys[0]),
	const_cast<Mouse3DInput::VirtualKey*>(SpaceExplorerKeys)
};

//! Converts a hid device keycode (button identifier) of a pre-2009 3Dconnexion USB device to the standard 3d mouse virtual key definition
/** \param pid USB Product ID (PID) of 3D mouse device
	\param hidKeyCode hid keycode as retrieved from a Raw Input packet
	\return the standard 3d mouse virtual key (button identifier) or zero if an error occurs.
**/
static unsigned short HidToVirtualKey(unsigned long pid, unsigned short hidKeyCode)
{
	unsigned short virtualkey = hidKeyCode;
	for (size_t i=0; i < sizeof(Mouse3DVirtualKeys)/sizeof(Mouse3DVirtualKeys[0]); ++i)
	{
		if (pid == Mouse3DVirtualKeys[i].pid)
		{
			if (hidKeyCode < Mouse3DVirtualKeys[i].nKeys)
				virtualkey = static_cast<unsigned short>(Mouse3DVirtualKeys[i].vkeys[hidKeyCode]);
			else
				virtualkey = Mouse3DInput::V3DK_INVALID;
			break;
		}
	}
	// Remaining devices are unchanged
	return virtualkey;
}

//unique instance
static Mouse3DInput* s_mouseInputInstance = 0;

bool Mouse3DInput::RawInputEventFilter(void* msg, long* result)
{
	if (!s_mouseInputInstance)
		return false;

	MSG* messageStruct = static_cast<MSG*>(msg);

	if (messageStruct->message == WM_INPUT)
	{
		HRAWINPUT hRawInput = reinterpret_cast<HRAWINPUT>(messageStruct->lParam);
		s_mouseInputInstance->onRawInput(RIM_INPUT,hRawInput);
		
		//DGM FIXME: what's the meaning of this?
		//if (result)
		//	result = 0;

		return true;
	}

	return false;
}

Mouse3DInput::Mouse3DInput(QWidget* widget)
	: QObject(widget)
	, m_lastInputTime(0)
{
	initializeRawInput(widget->winId());

	//load parameters from persistent settings
	m_mouseParams.fromPersistentSettings();

	//register current instance
	assert(s_mouseInputInstance == 0);
	s_mouseInputInstance = this;

	qApp->setEventFilter(Mouse3DInput::RawInputEventFilter);
}

Mouse3DInput::~Mouse3DInput()
{
	//unregister current instance
	if (s_mouseInputInstance == this)
	{
		//save current parameters to persistent settings
		m_mouseParams.toPersistentSettings();
		s_mouseInputInstance = 0;
	}
}

void Mouse3DInput::move3d(HANDLE device, std::vector<float>& motionData)
{
	Q_UNUSED(device);
	emit sigMove3d(motionData);
}

void Mouse3DInput::on3dmouseKeyDown(HANDLE device, int virtualKeyCode)
{
	Q_UNUSED(device);
	emit sigOn3dmouseKeyDown(virtualKeyCode);
}

void Mouse3DInput::on3dmouseKeyUp(HANDLE device, int virtualKeyCode)
{
	Q_UNUSED(device);
	emit sigOn3dmouseKeyUp(virtualKeyCode);
}

//! Get an initialized array of PRAWINPUTDEVICE for the 3D devices
/** \param[out] numDevices the number of devices to register (currently this is always 1)
**/
static PRAWINPUTDEVICE GetDevicesToRegister(unsigned int& numDevices)
{
	// Array of raw input devices to register
	static RAWINPUTDEVICE s_rawInputDevices[] = {
		{0x01, 0x08, 0x00, 0x00} // Usage Page = 0x01 Generic Desktop Page, Usage Id= 0x08 Multi-axis Controller
	};

	numDevices = sizeof(s_rawInputDevices) / sizeof(s_rawInputDevices[0]);

	return s_rawInputDevices;
}

bool Mouse3DInput::DeviceAvailable()
{
	unsigned int numDevicesOfInterest = 0;
	PRAWINPUTDEVICE devicesToRegister = GetDevicesToRegister(numDevicesOfInterest);

	unsigned int nDevices = 0;
	if (::GetRawInputDeviceList(NULL, &nDevices, sizeof(RAWINPUTDEVICELIST)) != 0 || nDevices == 0)
	{
		//no device detected
		return false;
	}

	std::vector<RAWINPUTDEVICELIST> rawInputDeviceList(nDevices);
	if (::GetRawInputDeviceList(&rawInputDeviceList[0], &nDevices, sizeof(RAWINPUTDEVICELIST)) == static_cast<unsigned int>(-1))
	{
		return false;
	}

	for (unsigned int i = 0; i < nDevices; ++i)
	{
		unsigned int cbSize = sizeof(RID_DEVICE_INFO);
		RID_DEVICE_INFO rdi = { cbSize };

		if (GetRawInputDeviceInfo(rawInputDeviceList[i].hDevice, RIDI_DEVICEINFO, &rdi, &cbSize) > 0)
		{
			//skip non HID and non logitec (3DConnexion) devices
			if (rdi.dwType != RIM_TYPEHID || rdi.hid.dwVendorId != LOGITECH_VENDOR_ID)
				continue;

			//check if devices matches Multi-axis Controller
			for (unsigned int j = 0; j < numDevicesOfInterest; ++j)
			{
				if (devicesToRegister[j].usUsage == rdi.hid.usUsage
					&& devicesToRegister[j].usUsagePage == rdi.hid.usUsagePage)
						return true;
			}
		}
	}

	return false;
}

bool Mouse3DInput::initializeRawInput(HWND hwndTarget)
{
	m_window = hwndTarget;

	// Simply fail if there is no window
	if (!hwndTarget)
		return false;

	unsigned int numDevices = 0;
	PRAWINPUTDEVICE devicesToRegister = GetDevicesToRegister(numDevices);

	if (numDevices == 0)
		return false;

	// Get OS version
	OSVERSIONINFO osvi = {sizeof(OSVERSIONINFO),0};
	::GetVersionEx(&osvi);

	unsigned int cbSize = sizeof(devicesToRegister[0]);
	for (size_t i = 0; i < numDevices; i++)
	{
		// Set the target window to use
		//devicesToRegister[i].hwndTarget = hwndTarget;

		// If Vista or newer, enable receiving the WM_INPUT_DEVICE_CHANGE message.
		if (osvi.dwMajorVersion >= 6)
			devicesToRegister[i].dwFlags |= RIDEV_DEVNOTIFY;
	}
	return (::RegisterRawInputDevices(devicesToRegister, numDevices, cbSize) != FALSE);
}

UINT Mouse3DInput::getRawInputBuffer(PRAWINPUT pData, PUINT pcbSize, UINT cbSizeHeader)
{
#ifdef CC_ENV_64
	return ::GetRawInputBuffer(pData, pcbSize, cbSizeHeader);
#else
	BOOL bIsWow64 = FALSE;
	::IsWow64Process(GetCurrentProcess(), &bIsWow64);
	if (!bIsWow64 || pData == NULL)
		return ::GetRawInputBuffer(pData, pcbSize, cbSizeHeader);

	HWND hwndTarget = m_window; //fParent->winId();

	size_t cbDataSize=0;
	UINT nCount=0;
	PRAWINPUT pri = pData;

	MSG msg;
	while (PeekMessage(&msg, hwndTarget, WM_INPUT, WM_INPUT, PM_NOREMOVE))
	{
		HRAWINPUT hRawInput = reinterpret_cast<HRAWINPUT>(msg.lParam);
		size_t cbSize = *pcbSize - cbDataSize;
		if (::GetRawInputData(hRawInput, RID_INPUT, pri, &cbSize, cbSizeHeader) == static_cast<UINT>(-1))
		{
			if (nCount==0)
				return static_cast<UINT>(-1);
			break;
		}
		++nCount;

		// Remove the message for the data just read
		PeekMessage(&msg, hwndTarget, WM_INPUT, WM_INPUT, PM_REMOVE);

		pri = NEXTRAWINPUTBLOCK(pri);
		cbDataSize = reinterpret_cast<ULONG_PTR>(pri) - reinterpret_cast<ULONG_PTR>(pData);
		if (cbDataSize >= *pcbSize)
		{
			cbDataSize = *pcbSize;
			break;
		}
	}

	return nCount;

#endif
}

void Mouse3DInput::on3dmouseInput()
{
	// don't do any data processing in background
	bool bIsForeground = (::GetActiveWindow() != NULL);
	if (!bIsForeground)
	{
		// set all cached data to zero so that a zero event is seen and the cached data deleted
		for (std::map<HANDLE, InputData>::iterator it = m_device2Data.begin(); it != m_device2Data.end(); it++)
		{
			it->second.axes.assign(6, 0.0f);
			it->second.isDirty = true;
		}
	}

	DWORD dwNow = ::GetTickCount();           // Current time;
	DWORD dwElapsedTime;                      // Elapsed time since we were last here

	if (0 == m_lastInputTime)
	{
		dwElapsedTime = 10;                    // System timer resolution
	}
	else 
	{
		dwElapsedTime = dwNow - m_lastInputTime;
		if (m_lastInputTime > dwNow)
		{
			dwElapsedTime = ~dwElapsedTime+1;
		}
		
		if (dwElapsedTime<1)
		{
			dwElapsedTime=1;
		}
		else if (dwElapsedTime > 500)
		{
			// Check for wild numbers because the device was removed while sending data
			dwElapsedTime = 10;
		}
	}

	// Take a look at the users preferred speed setting and adjust the sensitivity accordingly
	Mouse3DParameters::SpeedMode speedSetting = m_mouseParams.speedMode();
	// See "Programming for the 3D Mouse", Section 6.1.3
	double speed = 0.25;
	switch(speedSetting)
	{
	case Mouse3DParameters::LowestSpeed:
		speed = 0.25;
		break;
	case Mouse3DParameters::LowSpeed:
		speed = 0.5;
		break;
	case Mouse3DParameters::MidSpeed:
		speed = 1.0;
		break;
	case Mouse3DParameters::HighSpeed:
		speed = 2.0;
		break;
	case Mouse3DParameters::HighestSpeed:
		speed = 4.0;
		break;
	default:
		assert(false);
	}

	//we integrate time asap so as to loose 'less' accuracy
	float mouseData2PanZoom = static_cast<float>(c_3dmouseAngularVelocity * (speed * static_cast<double>(dwElapsedTime)));
	float mouseData2Rotation = static_cast<float>(c_3dmouseAngularVelocity * (speed * static_cast<double>(dwElapsedTime))); // v = w * r,  we don't know r yet so lets assume r=1

	std::map<HANDLE, InputData>::iterator iterator = m_device2Data.begin();
	while (iterator != m_device2Data.end())
	{
		//if we have not received data for a while send a zero event
		if ((--(iterator->second.timeToLive)) == 0)
		{
			iterator->second.axes.assign(6, 0.0f);
		}
		else if (!iterator->second.isDirty)
		{
			//if we are not polling then only handle the data that was actually received
			++iterator;
			continue;
		}
		iterator->second.isDirty = false;

		// get a copy of the device
		HANDLE hdevice = iterator->first;

		// get a copy of the motion vectors and apply the user filters
		std::vector<float> motionData = iterator->second.axes;
		assert(motionData.size() == 6);

		// apply the user filters

		// Pan Zoom filter
		{
			// See "Programming for the 3D Mouse", Section 6.1.2
			if (!m_mouseParams.panZoomEnabled())
			{
				// Pan zoom is switched off so set the translation vector values to zero
				motionData[0] =  motionData[1] =  motionData[2] = 0.0f;
			}
			else for (int axis = 0; axis < 3; axis++)
			{
				// convert the translation vector into physical data
				motionData[axis] *= mouseData2PanZoom;
			}
		}

		// Rotate filter
		{
			// See "Programming for the 3D Mouse", Section 6.1.1
			if (!m_mouseParams.rotationEnabled())
			{
				// Rotate is switched off so set the rotation vector values to zero
				motionData[3] =  motionData[4] =  motionData[5] = 0.0f;
			}
			else for (int axis = 3; axis < 6; axis++)
			{
				// convert the directed Rotate vector into physical data
				// See "Programming for the 3D Mouse", Section 7.2.2
				motionData[axis] *= mouseData2Rotation;
			}
		}

		// Now that the data has had the filters and sensitivty settings applied
		// calculate the displacements since the last view update
		// DGM: already done (see above)
		//{
		//	for (int axis = 0; axis < 6; axis++)
		//	{
		//		motionData[axis] *= dwElapsedTime;
		//	}
		//}

		// Now a bit of book keeping before passing on the data
		if (iterator->second.isZero())
		{
			iterator = m_device2Data.erase(iterator);
		}
		else
		{
			++iterator;
		}

		// Work out which will be the next device
		HANDLE hNextDevice = 0;
		if (iterator != m_device2Data.end())
		{
			hNextDevice = iterator->first;
		}

		// Pass the 3dmouse input to the view controller
		move3d(hdevice, motionData);

		// Because we don't know what happened in the previous call, the cache might have
		// changed so reload the iterator
		iterator = m_device2Data.find(hNextDevice);
	}

	m_lastInputTime = !m_device2Data.empty() ? dwNow : 0;
}

void Mouse3DInput::onRawInput(UINT nInputCode, HRAWINPUT hRawInput)
{
	const size_t cbSizeOfBuffer = 1024;
	BYTE pBuffer[cbSizeOfBuffer];

	PRAWINPUT pRawInput = reinterpret_cast<PRAWINPUT>(pBuffer);
	UINT cbSize = cbSizeOfBuffer;

	if (::GetRawInputData(hRawInput, RID_INPUT, pRawInput, &cbSize, sizeof(RAWINPUTHEADER)) == static_cast<UINT>(-1))
	{
		return;
	}

	bool b3dmouseInput = translateRawInputData(nInputCode, pRawInput);
	::DefRawInputProc(&pRawInput, 1, sizeof(RAWINPUTHEADER));

	// Check for any buffered messages
	cbSize = cbSizeOfBuffer;
	UINT nCount = getRawInputBuffer(pRawInput, &cbSize, sizeof(RAWINPUTHEADER));
	if (nCount == (UINT)-1)
	{
		qDebug ("GetRawInputBuffer returned error %d\n", GetLastError());
	}

	while (nCount>0 && nCount !=  static_cast<UINT>(-1))
	{
		PRAWINPUT pri = pRawInput;
		UINT nInput;
		for (nInput=0; nInput<nCount; ++nInput)
		{
			b3dmouseInput |= translateRawInputData(nInputCode, pri);
			// clean the buffer
			::DefRawInputProc(&pri, 1, sizeof(RAWINPUTHEADER));

			pri = NEXTRAWINPUTBLOCK(pri);
		}
		cbSize = cbSizeOfBuffer;
		nCount = getRawInputBuffer(pRawInput, &cbSize, sizeof(RAWINPUTHEADER));
	}

	// If we have mouse input data for the app then tell tha app about it
	if (b3dmouseInput)
		on3dmouseInput();
}

bool Mouse3DInput::translateRawInputData(UINT nInputCode, PRAWINPUT pRawInput)
{
	bool bIsForeground = (nInputCode == RIM_INPUT);

	// We are not interested in keyboard or mouse data received via raw input
	if (pRawInput->header.dwType != RIM_TYPEHID)
		return false;

	RID_DEVICE_INFO sRidDeviceInfo;
	sRidDeviceInfo.cbSize = sizeof(RID_DEVICE_INFO);
	UINT cbSize = sizeof(RID_DEVICE_INFO);

	if (::GetRawInputDeviceInfo(pRawInput->header.hDevice, RIDI_DEVICEINFO, &sRidDeviceInfo, &cbSize) == cbSize)
	{
		if (sRidDeviceInfo.hid.dwVendorId == LOGITECH_VENDOR_ID)
		{
			if (pRawInput->data.hid.bRawData[0] == 0x01)
			{
				// Translation vector
				InputData& deviceData = m_device2Data[pRawInput->header.hDevice];
				deviceData.timeToLive = c_defaultTimeToLive;
				if (bIsForeground)
				{
					short* pnRawData = reinterpret_cast<short*>(&pRawInput->data.hid.bRawData[1]);
					// Cache the pan zoom data
					deviceData.axes[0] = static_cast<float>(pnRawData[0]);
					deviceData.axes[1] = static_cast<float>(pnRawData[1]);
					deviceData.axes[2] = static_cast<float>(pnRawData[2]);

					if (pRawInput->data.hid.dwSizeHid >= 13) // Highspeed package
					{
						// Cache the rotation data
						deviceData.axes[3] = static_cast<float>(pnRawData[3]);
						deviceData.axes[4] = static_cast<float>(pnRawData[4]);
						deviceData.axes[5] = static_cast<float>(pnRawData[5]);
						deviceData.isDirty = true;
						return true;
					}
				}
				else // Zero out the data if the app is not in forground
				{
					deviceData.axes.assign(6, 0.0f);
				}
			}
			else if (pRawInput->data.hid.bRawData[0] == 0x02)
			{
				// Rotation vector
				// If we are not in foreground do nothing
				// The rotation vector was zeroed out with the translation vector in the previous message
				if (bIsForeground)
				{
					InputData& deviceData = m_device2Data[pRawInput->header.hDevice];
					deviceData.timeToLive = c_defaultTimeToLive;

					short* pnRawData = reinterpret_cast<short*>(&pRawInput->data.hid.bRawData[1]);
					// Cache the rotation data
					deviceData.axes[3] = static_cast<float>(pnRawData[0]);
					deviceData.axes[4] = static_cast<float>(pnRawData[1]);
					deviceData.axes[5] = static_cast<float>(pnRawData[2]);
					deviceData.isDirty = true;

					return true;
				}
			}
			else if (pRawInput->data.hid.bRawData[0] == 0x03) // Keystate change
			{
				// this is a package that contains 3d mouse keystate information
				// bit0=key1, bit=key2 etc.
				unsigned long dwKeystate = *reinterpret_cast<unsigned long*>(&pRawInput->data.hid.bRawData[1]);

				// Log the keystate changes
				unsigned long dwOldKeystate = m_device2Keystate[pRawInput->header.hDevice];
				if (dwKeystate != 0)
					m_device2Keystate[pRawInput->header.hDevice] = dwKeystate;
				else
					m_device2Keystate.erase(pRawInput->header.hDevice);

				//  Only call the keystate change handlers if the app is in foreground
				if (bIsForeground)
				{
					unsigned long dwChange = dwKeystate ^ dwOldKeystate;

					for (int nKeycode=1; nKeycode<33; nKeycode++)
					{
						if (dwChange & 0x01)
						{
							int nVirtualKeyCode = HidToVirtualKey(sRidDeviceInfo.hid.dwProductId, static_cast<unsigned short>(nKeycode));
							if (nVirtualKeyCode)
							{
								if (dwKeystate&0x01)
									on3dmouseKeyDown(pRawInput->header.hDevice, nVirtualKeyCode);
								else
									on3dmouseKeyUp(pRawInput->header.hDevice, nVirtualKeyCode);
							}
						}
						dwChange >>= 1;
						dwKeystate >>= 1;
					}
				}
			}
		}
	}

	return false;
}

void Mouse3DInput::GetQuaternion(const std::vector<float>& vec, float* q)
{
	assert(vec.size() == 6);
	assert(q);

	//build quaternion
	const float& pitch	= vec[3]/2.0f;
	const float& yaw	= -vec[5]/2.0f;
	const float& roll	= vec[4]/2.0f;

	float s1 = sin(roll);
	float s2 = sin(yaw);
	float s3 = sin(pitch);
	float s2_x_s1 = s2 * s1;
	float c1 = cos(roll);
	float c2 = cos(yaw);
	float c3 = cos(pitch);
	float c2_x_c1 = c2 * c1;
	
	q[0] = c3 * c2_x_c1  +  s3 * s2_x_s1;
	q[1] = s3 * c2_x_c1  -  c3 * s2_x_s1;
	q[2] = c3 * s2 * c1  +  s3 * c2 * s1;
	q[3] = c3 * c2 * s1  -  s3 * s2 * c1;
}
