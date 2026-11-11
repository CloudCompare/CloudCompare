// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
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
// #          COPYRIGHT: CloudCompare project                               #
// #                                                                        #
// ##########################################################################

#include "Mouse3DInput_macDriver.h"

#if defined(CC_3DMOUSE_HID) && defined(CC_MAC_OS)

// Local
#include "Mouse3DInput.h"
#include "Mouse3DInput_hid.h"

// qCC_db
#include <ccLog.h>

// System
#include <algorithm>
#include <dlfcn.h>
#include <mutex>

// The few declarations we need from the driver's client framework
// (3DconnexionClient/ConnexionClientAPI.h), loaded at run time.
static constexpr const char* c_clientFrameworkPath = "/Library/Frameworks/3DconnexionClient.framework/3DconnexionClient";

static constexpr uint32_t c_clientWildcard     = 0x2A2A2A2A; // '****'
static constexpr uint16_t c_clientModeTakeOver = 1;
static constexpr uint32_t c_maskAll            = 0x3fff;
static constexpr uint16_t c_cmdHandleButtons   = 2;
static constexpr uint16_t c_cmdHandleAxis      = 3;
static constexpr uint32_t c_msgDeviceState     = 0x33645352; // '3dSR'

#pragma pack(push, 2)
struct ConnexionDeviceState
{
	uint16_t version;
	uint16_t client;
	uint16_t command;
	int16_t  param;
	int32_t  value;
	uint64_t time;
	uint8_t  report[8];
	uint16_t buttons8;
	int16_t  axis[6]; // tx, ty, tz, rx, ry, rz
	uint16_t address;
	uint32_t buttons;
};
#pragma pack(pop)

using MessageHandler                       = void (*)(uint32_t, uint32_t, void*);
using AddedHandler                         = void (*)(uint32_t);
using RemovedHandler                       = void (*)(uint32_t);
using SetConnexionHandlersFunction         = int16_t (*)(MessageHandler, AddedHandler, RemovedHandler, bool);
using CleanupConnexionHandlersFunction     = void (*)();
using RegisterConnexionClientFunction      = uint16_t (*)(uint32_t, const uint8_t*, uint16_t, uint32_t);
using SetConnexionClientButtonMaskFunction = void (*)(uint16_t, uint32_t);
using UnregisterConnexionClientFunction    = void (*)(uint16_t);

static void*                                s_framework                    = nullptr;
static SetConnexionHandlersFunction         s_setConnexionHandlers         = nullptr;
static CleanupConnexionHandlersFunction     s_cleanupConnexionHandlers     = nullptr;
static RegisterConnexionClientFunction      s_registerConnexionClient      = nullptr;
static SetConnexionClientButtonMaskFunction s_setConnexionClientButtonMask = nullptr;
static UnregisterConnexionClientFunction    s_unregisterConnexionClient    = nullptr;

//! Mask of the driver's buttons that CloudCompare acts on. Keep in sync with
//! cc3DMouseManager::on3DMouseKeyDown and ccViewer::on3DMouseKeyDown, which warn
//! about the others. The driver numbers buttons like the 3DxWare virtual keys,
//! unlike the raw HID reports: bit 0 is Menu (key 1), bit 1 is Fit (key 2), etc.
static uint32_t HandledButtonsMask()
{
	static const Mouse3DInput::VirtualKey c_handledKeys[]{Mouse3DInput::V3DK_FIT,
	                                                      Mouse3DInput::V3DK_TOP,
	                                                      Mouse3DInput::V3DK_LEFT,
	                                                      Mouse3DInput::V3DK_RIGHT,
	                                                      Mouse3DInput::V3DK_FRONT,
	                                                      Mouse3DInput::V3DK_BOTTOM,
	                                                      Mouse3DInput::V3DK_BACK,
	                                                      Mouse3DInput::V3DK_CW,
	                                                      Mouse3DInput::V3DK_CCW,
	                                                      Mouse3DInput::V3DK_ISO1,
	                                                      Mouse3DInput::V3DK_ISO2};
	uint32_t                              mask = 0;
	for (Mouse3DInput::VirtualKey key : c_handledKeys)
	{
		mask |= 1u << (key - 1);
	}
	return mask;
}

//! The driver calls back a plain function on its own thread, which reaches the
//! client through s_client. The mutex keeps the client alive while a callback
//! uses it.
static MacDriverClient* s_client = nullptr;
static std::mutex       s_clientMutex;

static bool LoadClientFramework()
{
	if (s_framework)
	{
		return true;
	}
	s_framework = dlopen(c_clientFrameworkPath, RTLD_LAZY | RTLD_LOCAL);
	if (!s_framework)
	{
		return false;
	}
	s_setConnexionHandlers         = reinterpret_cast<SetConnexionHandlersFunction>(dlsym(s_framework, "SetConnexionHandlers"));
	s_cleanupConnexionHandlers     = reinterpret_cast<CleanupConnexionHandlersFunction>(dlsym(s_framework, "CleanupConnexionHandlers"));
	s_registerConnexionClient      = reinterpret_cast<RegisterConnexionClientFunction>(dlsym(s_framework, "RegisterConnexionClient"));
	s_setConnexionClientButtonMask = reinterpret_cast<SetConnexionClientButtonMaskFunction>(dlsym(s_framework, "SetConnexionClientButtonMask"));
	s_unregisterConnexionClient    = reinterpret_cast<UnregisterConnexionClientFunction>(dlsym(s_framework, "UnregisterConnexionClient"));
	if (!s_setConnexionHandlers || !s_cleanupConnexionHandlers || !s_registerConnexionClient || !s_unregisterConnexionClient)
	{
		dlclose(s_framework);
		s_framework = nullptr;
		return false;
	}
	return true;
}

MacDriverClient::MacDriverClient(QObject* parent)
    : QObject(parent)
{
}

MacDriverClient::~MacDriverClient()
{
	{
		std::lock_guard<std::mutex> lock(s_clientMutex);
		if (s_client == this)
		{
			s_client = nullptr;
		}
	}
	if (m_clientID != 0)
	{
		s_unregisterConnexionClient(m_clientID);
		s_cleanupConnexionHandlers();
		m_clientID = 0;
	}
}

bool MacDriverClient::registerWithDriver(const QString& appName)
{
	if (!LoadClientFramework())
	{
		return false;
	}

	if (s_setConnexionHandlers(OnDriverMessage, nullptr, nullptr, true) != 0)
	{
		return false;
	}

	// Registered like FreeCAD does: the wildcard signature and the application
	// name as a Pascal string. The driver then sends input only while the
	// application with that name is in front.
	QByteArray pascalName = appName.toUtf8().left(255);
	pascalName.prepend(static_cast<char>(pascalName.size()));
	m_clientID = s_registerConnexionClient(c_clientWildcard, reinterpret_cast<const uint8_t*>(pascalName.constData()), c_clientModeTakeOver, c_maskAll);
	if (m_clientID == 0)
	{
		s_cleanupConnexionHandlers();
		return false;
	}
	if (s_setConnexionClientButtonMask)
	{
		s_setConnexionClientButtonMask(m_clientID, HandledButtonsMask());
	}

	// Only now, so that callbacks never see a half-registered client
	{
		std::lock_guard<std::mutex> lock(s_clientMutex);
		s_client = this;
	}

	ccLog::Print("[3D Mouse] Using the 3Dconnexion driver");
	return true;
}

void MacDriverClient::OnDriverMessage(uint32_t, uint32_t messageType, void* messageArgument)
{
	if (messageType != c_msgDeviceState || !messageArgument)
	{
		return;
	}
	std::lock_guard<std::mutex> lock(s_clientMutex);
	if (!s_client)
	{
		return;
	}
	const auto* state = static_cast<const ConnexionDeviceState*>(messageArgument);
	// Device states are broadcast to every client
	if (state->client != s_client->m_clientID)
	{
		return;
	}
	switch (state->command)
	{
	case c_cmdHandleAxis:
		s_client->handleAxes(state->axis);
		break;
	case c_cmdHandleButtons:
		s_client->handleButtons(state->buttons);
		break;
	default:
		break;
	}
}

void MacDriverClient::handleAxes(const int16_t axes[6])
{
	const int rawAxes[6]{axes[0], axes[1], axes[2], axes[3], axes[4], axes[5]};
	if (std::all_of(rawAxes, rawAxes + 6, [](int axis)
	                { return axis == 0; }))
	{
		if (m_moving)
		{
			m_moving = false;
			Q_EMIT sigReleased();
		}
		return;
	}
	m_moving = true;
	Q_EMIT sigMove3d(RawAxesToMotion(rawAxes));
}

void MacDriverClient::handleButtons(uint32_t buttonMask)
{
	// The driver reports every button, whatever the client's button mask
	buttonMask &= HandledButtonsMask();

	const uint32_t pressed  = buttonMask & ~m_prevButtonMask;
	const uint32_t released = ~buttonMask & m_prevButtonMask;
	m_prevButtonMask        = buttonMask;

	for (unsigned bit = 0; bit < 32; ++bit)
	{
		const uint32_t mask       = 1u << bit;
		const int      virtualKey = static_cast<int>(bit) + 1;
		if (pressed & mask)
		{
			Q_EMIT sigOn3DMouseKeyDown(virtualKey);
		}
		else if (released & mask)
		{
			Q_EMIT sigOn3DMouseKeyUp(virtualKey);
		}
	}
}

#endif // CC_3DMOUSE_HID && CC_MAC_OS
