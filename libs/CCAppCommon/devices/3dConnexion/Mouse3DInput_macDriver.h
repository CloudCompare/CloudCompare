#pragma once

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

// Internal header - client of the 3Dconnexion driver on macOS.
// Guarded by CC_3DMOUSE_HID and CC_MAC_OS.

#ifdef CC_3DMOUSE_HID

// CCCoreLib
#include <CCPlatform.h>

#ifdef CC_MAC_OS

// Qt
#include <QObject>

// System
#include <cstdint>
#include <vector>

//! Receives 3D mouse input from the 3Dconnexion driver (3DxWare) on macOS
/** While the driver runs it holds the device, so hidapi gets no reports. The
    driver's client framework is loaded at run time, so nothing from the
    3Dconnexion SDK is needed to build CloudCompare. The driver sends input
    only to the registered application that is in front.
**/
class MacDriverClient : public QObject
{
	Q_OBJECT

  public:
	//! Constructor
	explicit MacDriverClient(QObject* parent);

	//! Destructor (unregisters from the driver)
	~MacDriverClient() override;

	//! Registers with the driver under the application's name. Returns false
	//! if the driver isn't installed or refuses the registration.
	bool registerWithDriver(const QString& appName);

  Q_SIGNALS:
	void sigMove3d(std::vector<float> motionData);
	void sigReleased();
	void sigOn3DMouseKeyDown(int virtualKeyCode);
	void sigOn3DMouseKeyUp(int virtualKeyCode);

  private:
	//! Driver message callback (called on a driver thread, with s_clientMutex locked
	//! while it uses the client)
	static void OnDriverMessage(uint32_t productID, uint32_t messageType, void* messageArgument);

	void handleAxes(const int16_t axes[6]);
	void handleButtons(uint32_t buttonMask);

	uint16_t m_clientID       = 0;
	bool     m_moving         = false;
	uint32_t m_prevButtonMask = 0;
};

#endif // CC_MAC_OS
#endif // CC_3DMOUSE_HID
