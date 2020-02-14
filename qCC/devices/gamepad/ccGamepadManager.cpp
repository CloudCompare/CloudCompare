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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

//Local
#include "ccGamepadManager.h"
#include "ccGLWindow.h"
#include "ccMainAppInterface.h"
#include "devices/gamepad/GamepadInput.h"
#include "ccPickOneElementDlg.h"

//Qt
#include <QAction>
#include <QMainWindow>
#include <QGuiApplication>
#include <QMenu>

ccGamepadManager::ccGamepadManager( ccMainAppInterface *appInterface, QObject *parent )
	: QObject(parent)
	, m_appInterface(appInterface)
	, m_gamepadInput(nullptr)
	, m_menu(nullptr)
	, m_actionEnable(nullptr)
{
	setupMenu();
	setupGamepadInput(); //DGM:at this point we can only init the gamepadInput structure (Qt will send a signal when the gamepad is connected - at least from its point of view)
}

ccGamepadManager::~ccGamepadManager()
{
	releaseDevice();

	if (m_menu)
	{
		delete m_menu;
	}
}

void ccGamepadManager::showMessage(QString message, bool asWarning)
{
	if (!asWarning)
	{
		ccLog::Error(message);
	}
	else
	{
		ccLog::Warning(message);
	}
}

void ccGamepadManager::enableDevice(bool state, bool silent, int deviceID/*=-1*/)
{
	if (!m_gamepadInput)
	{
		assert(false);
		setupGamepadInput();
	}
	
	if (state)
	{
		for (int step = 0; step < 1; ++step) //fake loop for easy break
		{
			QGamepadManager* manager = QGamepadManager::instance();
			if (!manager)
			{
				showMessage("[Gamepad] Manager is not accessible?!", silent);
				state = false;
				break;
			}
			
			if (deviceID < 0)
			{
				ccLog::Print("[Gamepad] Looking for connected gamepads...");
				QList<int> gamepads;
				for (int i = 0; i < 3; ++i)
				{
					QGuiApplication::processEvents();
					gamepads = manager->connectedGamepads();
					if (!gamepads.empty())
					{
						break;
					}
				}

				if (gamepads.empty())
				{
					showMessage("[Gamepad] No device registered", silent);
					state = false;
					break;
				}
				else
				{
					ccLog::Print(QString("[Gamepad] Found %1 gamepad(s)").arg(gamepads.size()));
				}
				deviceID = gamepads.front();
				if (!silent && gamepads.size() > 1)
				{
					//ask the user for the right gamepad
					ccPickOneElementDlg poeDlg("Gamepad", "Connected gamepads", m_appInterface ? m_appInterface->getMainWindow() : nullptr);
					for (int id : gamepads)
					{
						QString name = QGamepad(id).name();
						if (name.isEmpty())
						{
							name = QString("Gamepad #%1").arg(id);
						}
						poeDlg.addElement(name);
					}
					if (!poeDlg.exec())
					{
						return;
					}
					deviceID = gamepads[poeDlg.getSelectedIndex()];
				}
			}
			
			m_gamepadInput->stop(); //just in case
			m_gamepadInput->setDeviceId(deviceID);
				
			if (m_gamepadInput->isConnected())
			{
				ccLog::Print(QString("[Gamepad] Device %1 is now enabled").arg(deviceID));
				m_gamepadInput->start();
				break;
			}
			else
			{
				showMessage(QString("[Gamepad] Device %1 is not connected").arg(deviceID), silent);
				state = false;
				break;
			}
		}
	}
	else if (m_gamepadInput)
	{
		if (deviceID >= 0 && m_gamepadInput->deviceId() != deviceID)
		{
			//this is not the current device?!
			return;
		}
		m_gamepadInput->stop();
		ccLog::Warning(QString("[Gamepad] Device %1 is now disabled").arg(m_gamepadInput->deviceId()));
	}
	
	m_actionEnable->blockSignals(true);
	m_actionEnable->setChecked(state);
	m_actionEnable->blockSignals(false);
}

void ccGamepadManager::releaseDevice()
{
	if (m_gamepadInput == nullptr)
		return;
	
	m_gamepadInput->stop(); //disconnect from the driver
	m_gamepadInput->disconnect(this); //disconnect from Qt ;)
	
	delete m_gamepadInput;
	m_gamepadInput = nullptr;
}

void ccGamepadManager::setupGamepadInput()
{
	if (m_gamepadInput)
	{
		assert(false);
		return;
	}

	m_gamepadInput = new GamepadInput(this);

	connect(m_gamepadInput, &GamepadInput::updated, this, &ccGamepadManager::onGamepadInput, Qt::DirectConnection);

	connect(m_gamepadInput, &GamepadInput::buttonL1Changed, this, [=]() {
		m_appInterface->decreasePointSize();
	});
	connect(m_gamepadInput, &GamepadInput::buttonR1Changed, this, [=]() {
		m_appInterface->increasePointSize();
	});

	connect(m_gamepadInput, &GamepadInput::buttonStartChanged, this, [=](bool state) {
		if (state)
		{
			m_appInterface->setGlobalZoom();
		}
	});
	connect(m_gamepadInput, &GamepadInput::buttonAChanged, this, [=](bool state)
	{
		if (state)
		{
			m_appInterface->toggleActiveWindowViewerBasedPerspective();
		}
	});
	connect(m_gamepadInput, &GamepadInput::buttonBChanged, this, [=](bool state)
	{
		if (state)
		{
			m_appInterface->toggleActiveWindowCenteredPerspective();
		}
	});

	QGamepadManager* manager = QGamepadManager::instance();
	if (manager)
	{
		connect(manager, &QGamepadManager::gamepadConnected, this, [&](int deviceId)
		{
			ccLog::Print(QString("gamepad device %1 has been connected").arg(deviceId));
			//auto-enable the device (if none is enabled yet)
			if (m_actionEnable && !m_actionEnable->isChecked())
			{
				enableDevice(true, true, deviceId);
			}
		});

		connect(manager, &QGamepadManager::gamepadDisconnected, this, [&](int deviceId)
		{
			ccLog::Print(QString("gamepad device %1 has been disconnected").arg(deviceId));
			//auto-disable the device (if this device is the one currently enabled)
			if (m_actionEnable && m_actionEnable->isChecked() && m_gamepadInput && m_gamepadInput->deviceId() == deviceId)
			{
				enableDevice(false, true, deviceId);
			}
		});
	}
	else
	{
		assert(false);
		showMessage("[Gamepad] Manager is not accessible?!", true);
	}
}

void ccGamepadManager::setupMenu()
{
	if (!m_menu)
	{
		m_menu = new QMenu("Gamepad");
		m_menu->setIcon(QIcon(":/CC/images/gamepad.png"));
	}
	
	if (!m_actionEnable)
	{
		m_actionEnable = new QAction(tr("Enable"), this);
		m_actionEnable->setCheckable(true);

		connect(m_actionEnable, &QAction::toggled, [this](bool state) {
			enableDevice(state, false);
		});

		m_menu->addAction(m_actionEnable);
	}
}

void ccGamepadManager::onGamepadInput()
{
	assert(m_gamepadInput);

	ccGLWindow* win = m_appInterface->getActiveGLWindow();
	if (win)
	{
		m_gamepadInput->update(win);
	}
}