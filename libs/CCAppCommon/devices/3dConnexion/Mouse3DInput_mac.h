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

// Internal header - defines the HIDWorker thread used on macOS.
// Guarded by CC_MAC_HID (defined by CMake on APPLE so that AUTOMOC can see it).

#ifdef CC_MAC_HID

#include "Mouse3DInput.h"

// Qt
#include <QMetaType>
#include <QThread>

// system
#include <atomic>
#include <vector>

// hidapi
#include <hidapi/hidapi.h>

class HIDWorker : public QThread
{
	Q_OBJECT

  public:
	explicit HIDWorker(Mouse3DInput* parent)
	    : QThread(parent)
	    , m_parent(parent)
	{
	}

	~HIDWorker() override
	{
		stop();
		if (isRunning())
		{
			wait();
		}
		closeDevice();
	}

	//! Opens the first available 3DConnexion HID device.
	bool openDevice();

	//! Closes the HID device (idempotent).
	void closeDevice();

	void stop() { m_running.store(false); }

  Q_SIGNALS:
	void sigMove3d(std::vector<float> motionData);
	void sigReleased();
	void sigOn3dmouseKeyDown(int virtualKeyCode);
	void sigOn3dmouseKeyUp(int virtualKeyCode);

  protected:
	void run() override;

  private:
	void processMotion(const unsigned char* buf, int n);
	void processButtons(const unsigned char* buf, int n, unsigned int& prevButtonMask);

	hid_device*      m_handle = nullptr;
	std::atomic_bool m_running{false};
	Mouse3DInput*    m_parent;
	QString          m_devicePath;
};

#endif // CC_MAC_HID