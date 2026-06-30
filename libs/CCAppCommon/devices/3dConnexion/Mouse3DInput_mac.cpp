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

// macOS-only HID-based implementation of the 3DConnexion 3D mouse support.
// Drives the device directly via hidapi, without the proprietary 3DxWare SDK.

#ifdef CC_MAC_HID

#include "Mouse3DInput_mac.h"

#include "Mouse3DInput.h"

// qCC_db
#include <ccLog.h>

// system
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <wchar.h>

// 3DConnexion vendor id
static constexpr unsigned short c_3dconnexionVID = 0x256f;

//! Object angular velocity per mouse tick (in radians per ms per count)
//! Mirrors the definition in Mouse3DInput.cpp (Windows path uses eventData.period,
//! which is roughly the report period in ms - we use the polling period instead).
static const double c_3dmouseAngularVelocity_mac = 1.0e-6;
//! Polling period (ms) - approximately 60 Hz
static constexpr int c_hidPollPeriodMs = 16;

//! Overall speed multiplier (calibrated for Blender-like feel).
static constexpr float c_3dmouseGain = 1.5f;
//! Reference deflection for the progressive curve (typical HID full-scale ~±350).
static constexpr float c_3dmouseProgressiveRef = 250.0f;

//! Progressive (non-linear) axis scaling: small deflections stay fine, large
//! deflections get amplified. Curve: out = raw * (1 + |raw|/ref) * gain * ds.
static float scaleAxis(int raw, double ds)
{
	float a            = static_cast<float>(raw);
	float progressive  = 1.0f + std::min(std::abs(a) / c_3dmouseProgressiveRef, 1.0f);
	return a * progressive * c_3dmouseGain * static_cast<float>(ds);
}

//! Maps a button bitmask bit to a Mouse3DInput::VirtualKey value.
//! Layout documented by the spacenavd / 3DConnexion HID community.
static const int c_buttonMap[] = {
	Mouse3DInput::V3DK_FIT,    // bit 0
	Mouse3DInput::V3DK_MENU,   // bit 1
	Mouse3DInput::V3DK_TOP,    // bit 2
	Mouse3DInput::V3DK_LEFT,   // bit 3
	Mouse3DInput::V3DK_RIGHT,  // bit 4
	Mouse3DInput::V3DK_FRONT,  // bit 5
	Mouse3DInput::V3DK_BOTTOM, // bit 6
	Mouse3DInput::V3DK_BACK,   // bit 7
};
static constexpr size_t c_buttonMapSize = sizeof(c_buttonMap) / sizeof(c_buttonMap[0]);

bool HIDWorker::openDevice()
{
	hid_device_info* devs = hid_enumerate(c_3dconnexionVID, 0x0);
	if (!devs)
	{
		ccLog::Warning("[3D Mouse] No 3DConnexion HID device found");
		return false;
	}

	// Preferred interface: Generic Desktop page (0x01), Multi-axis Controller usage (0x08).
	// This is the interface that carries the 6-DOF motion reports. The SpaceMouse Wireless
	// exposes several HID interfaces; the others (e.g. Pointer, Consumer Control) do not.
	hid_device_info* cur = devs;
	for (; cur; cur = cur->next)
	{
		if (cur->usage_page == 0x01 && cur->usage == 0x08)
		{
			m_handle = hid_open_path(cur->path);
			if (m_handle)
			{
				m_devicePath = cur->path;
				QString name = (cur->product_string ? QString::fromWCharArray(cur->product_string) : QStringLiteral("3DConnexion device"));
				ccLog::Print(QString("[3D Mouse] Device: %1 (HID)").arg(name));
				break;
			}
		}
	}

	// Fallback: first openable interface (for older devices that don't expose a
	// distinct multi-axis usage).
	if (!m_handle)
	{
		cur = devs;
		for (; cur; cur = cur->next)
		{
			m_handle = hid_open_path(cur->path);
			if (m_handle)
			{
				m_devicePath = cur->path;
				QString name = (cur->product_string ? QString::fromWCharArray(cur->product_string) : QStringLiteral("3DConnexion device"));
				ccLog::Print(QString("[3D Mouse] Device: %1 (HID, fallback)").arg(name));
				break;
			}
		}
	}

	hid_free_enumeration(devs);

	if (!m_handle)
	{
		ccLog::Warning("[3D Mouse] Could not open any 3DConnexion HID device "
		               "(is the 3Dconnexion driver holding it exclusively?)");
		return false;
	}

	// Blocking reads with a timeout - avoids the spurious -1 returns that
	// non-blocking hid_read produces on macOS when no data is available.
	// stop() still terminates the loop because we check m_running each
	// iteration and the timeout bounds how long we wait.
	hid_set_nonblocking(m_handle, 0);
	return true;
}

void HIDWorker::closeDevice()
{
	if (m_handle)
	{
		hid_close(m_handle);
		m_handle = nullptr;
	}
}

void HIDWorker::run()
{
	m_running.store(true);

	// State for button edge detection
	unsigned int prevButtonMask = 0;
	// State for "released" emission (analogous to SI_ZERO_EVENT on Windows)
	auto         lastMotionTime = std::chrono::steady_clock::now();
	bool         motionActive    = false;

	unsigned char buf[80] = {0};

	// Give up only after this many consecutive read errors (e.g. device unplugged).
	constexpr int kMaxConsecutiveErrors = 100;
	int           consecutiveErrors      = 0;

	// Read with a timeout so the loop stays responsive to m_running changes
	// and doesn't busy-poll. hid_read_timeout returns 0 on timeout (not -1).
	constexpr int kReadTimeoutMs = 100;

	while (m_running.load())
	{
		int n = hid_read_timeout(m_handle, buf, sizeof(buf), kReadTimeoutMs);
		if (n < 0)
		{
			// Genuine read error (device unplugged, I/O error).
			++consecutiveErrors;
			if (consecutiveErrors >= kMaxConsecutiveErrors)
			{
				ccLog::Warning("[3D Mouse] Too many HID read errors, giving up");
				break;
			}
			QThread::msleep(c_hidPollPeriodMs);
			continue;
		}
		consecutiveErrors = 0;

		if (n == 0)
		{
			// Timeout - no report available. Emit "released" after a quiet
			// period, analogous to SI_ZERO_EVENT on Windows.
			if (motionActive)
			{
				auto now     = std::chrono::steady_clock::now();
				auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastMotionTime).count();
				if (elapsed > 100)
				{
					Q_EMIT sigReleased();
					motionActive = false;
				}
			}
			continue; // hid_read_timeout already waited kReadTimeoutMs
		}

		// Identify the report type. The SpaceMouse Wireless sends 13-byte motion
		// reports (report ID 0x01 + 6 int16 axes). Older devices send 7-byte
		// reports (report ID 0x01 + 6 int8 axes, or just 6 int8 axes without ID).
		// Button reports have report ID 0x03.
		if (n >= 13 && buf[0] == 0x01)
		{
			processMotion(buf, n);
			lastMotionTime = std::chrono::steady_clock::now();
			motionActive   = true;
		}
		else if (n >= 2 && buf[0] == 0x03)
		{
			processButtons(buf, n, prevButtonMask);
		}
		else if (n == 7)
		{
			// Older devices / setups that omit the report ID prefix (6 int8 axes).
			processMotion(buf, n);
			lastMotionTime = std::chrono::steady_clock::now();
			motionActive   = true;
		}
		else
		{
			// Unknown report - silently ignore known harmless status reports
			// (e.g. 0x17 = battery/status). Log other unknowns a few times
			// to aid debugging.
			if (buf[0] != 0x17)
			{
				static int s_unknownCount = 0;
				if (++s_unknownCount <= 5)
				{
					QString hex;
					for (int i = 0; i < n; ++i)
						hex += QString("%1 ").arg(buf[i], 2, 16, QLatin1Char('0'));
					ccLog::Print(QString("[3D Mouse] Unknown report (%1 bytes): %2").arg(n).arg(hex));
				}
			}
		}
	}
}

void HIDWorker::processMotion(const unsigned char* buf, int /*n*/)
{
	// Motion report layout (SpaceMouse Wireless, 13 bytes):
	//   buf[0]    = 0x01 (report ID)
	//   buf[1..2] = tx (int16 little-endian)
	//   buf[3..4] = ty
	//   buf[5..6] = tz
	//   buf[7..8] = rx
	//   buf[9..10]= ry
	//   buf[11..12]=rz
	// Older devices may send a 7-byte report (report ID + 6 int8 axes);
	// we handle both by branching on n.
	const unsigned char* p = buf;
	int                  axisBytes = 2; // int16 by default
	if (buf[0] == 0x01)
	{
		p = buf + 1;
	}
	else
	{
		// No report ID prefix (older devices) - 6 int8 axes.
		axisBytes = 1;
	}

	auto readAxis = [&](int offset) -> int
	{
		if (axisBytes == 2)
		{
			signed short v = static_cast<signed short>(p[offset] | (p[offset + 1] << 8));
			return static_cast<int>(v);
		}
		else
		{
			return static_cast<int>(static_cast<signed char>(p[offset]));
		}
	};

	int tx = readAxis(0);
	int ty = readAxis(2);
	int tz = readAxis(4);
	int rx = readAxis(6);
	int ry = readAxis(8);
	int rz = readAxis(10);

	if (tx == 0 && ty == 0 && tz == 0 && rx == 0 && ry == 0 && rz == 0)
	{
		// No motion - don't fire a move event.
		return;
	}

	// Scaling: progressive (non-linear) curve so small deflections stay fine
	// while large deflections are amplified for fast navigation.
	double ds = c_hidPollPeriodMs * c_3dmouseAngularVelocity_mac;

	std::vector<float> axes(6);
	// Blender-default NDOF axis mapping (see Blender manual, Input → NDOF).
	// Blender swaps the device's physical Y (forward/back) and Z (lift/compress)
	// relative to the raw HID report, so that pushing the cap forward zooms and
	// lifting the cap pans vertically. Sign conventions calibrated for
	// object-centric navigation in CloudCompare:
	//   tx (cap left/right)    -> pan X      (cap left  -> object left)
	//   tz (cap lift/compress) -> pan Y      (cap up    -> object up)
	//   ty (cap forward/back)  -> zoom       (cap fwd   -> zoom in)
	//   rx (pitch)             -> orbit X    (tilt back -> object up)
	//   ry (roll)              -> orbit Y
	//   rz (yaw / twist)       -> orbit Z
	axes[0] = -scaleAxis(tx, ds); // pan X
	axes[1] = -scaleAxis(tz, ds); // pan Y (Blender swap)
	axes[2] = -scaleAxis(ty, ds); // zoom   (Blender swap)
	axes[3] =  scaleAxis(rx, ds); // orbit X (inverted for natural pitch)
	axes[4] =  scaleAxis(ry, ds); // orbit Y
	axes[5] =  scaleAxis(rz, ds); // orbit Z

	Q_EMIT sigMove3d(axes);
}

void HIDWorker::processButtons(const unsigned char* buf, int n, unsigned int& prevButtonMask)
{
	// Button report layout: [0x03, buttonBitmaskLow, (buttonBitmaskHigh ...)]
	unsigned int curButtonMask = 0;
	if (n >= 2)
	{
		curButtonMask = static_cast<unsigned int>(buf[1]);
		if (n >= 3)
		{
			curButtonMask |= static_cast<unsigned int>(buf[2]) << 8;
		}
	}

	// Detect edges.
	unsigned int pressed  = curButtonMask & ~prevButtonMask;
	unsigned int released = ~curButtonMask & prevButtonMask;

	for (size_t bit = 0; bit < c_buttonMapSize; ++bit)
	{
		unsigned int mask = 1u << bit;
		if (pressed & mask)
		{
			Q_EMIT sigOn3dmouseKeyDown(c_buttonMap[bit]);
		}
		if (released & mask)
		{
			Q_EMIT sigOn3dmouseKeyUp(c_buttonMap[bit]);
		}
	}

	// For any bit beyond the static mapping table, emit a generic warning once per press.
	for (size_t bit = c_buttonMapSize; bit < 8 * sizeof(unsigned int); ++bit)
	{
		unsigned int mask = 1u << bit;
		if (pressed & mask)
		{
			ccLog::Warning(QString("[3D mouse] Unmapped button pressed (bit %1)").arg(static_cast<int>(bit)));
		}
	}

	prevButtonMask = curButtonMask;
}

#endif // CC_MAC_HID