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

#include <QAction>
#include <QMainWindow>
#include <QMenu>

#include "ccGamepadManager.h"
#include "ccGLWindow.h"

#include "ccMainAppInterface.h"

#include "devices/gamepad/GamepadInput.h"


ccGamepadManager::ccGamepadManager( ccMainAppInterface *appInterface, QObject *parent ) :
	QObject( parent ),
	mAppInterface( appInterface ),
	mGamepadInput( nullptr )
{
	setupMenu();
	
	//DGM: the first call never works at startup time...
	enableDevice(true, true);	
//	QMetaObject::invokeMethod(this, "enableDevice", Qt::QueuedConnection, Q_ARG(bool, true), Q_ARG(bool, false));
}

ccGamepadManager::~ccGamepadManager()
{
	releaseDevice();	
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

void ccGamepadManager::enableDevice(bool state, bool silent)
{
	if (mGamepadInput != nullptr)
	{
		releaseDevice();
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
				
			int gamepadID = 0;
			//QList<int> gamepads = manager->connectedGamepads();
			//if (gamepads.empty() == 0)
			//{
			//	ShowError("[Gamepad] No device registered", silent);
			//	state = false;
			//	break;
			//}
			//gamepadID = gamepads[0];
			//if (!silent && gamepads.size() > 1)
			//{
			//	//ask the user for the right gamepad
			//	ccPickOneElementDlg poeDlg("Gamepad", "Detected Gamepads", this);
			//	for (int id : gamepads)
			//	{
			//		poeDlg.addElement(QString("%1 (%2)").arg(QGamepad(id).name()).arg(manager->isGamepadConnected(id) ? "ON" : "OFF"));
			//	}
			//	if (!poeDlg.exec())
			//	{
			//		return;
			//	}
			//	gamepadID = gamepads[poeDlg.getSelectedIndex()];
			//}

			if (!mGamepadInput)
			{
				mGamepadInput = new GamepadInput(this);
				
				connect(mGamepadInput, SIGNAL(updated()), this, SLOT(onGamepadInput()), Qt::DirectConnection);
				
				connect(mGamepadInput, &GamepadInput::buttonL1Changed, this, [=]() {
					mAppInterface->decreasePointSize();
				});
				connect(mGamepadInput, &GamepadInput::buttonR1Changed, this, [=]() {
					mAppInterface->increasePointSize();
				});

				connect(mGamepadInput, &GamepadInput::buttonStartChanged, this, [=](bool state) {
					if (state)
					{
						mAppInterface->setGlobalZoom();
					}
				});
				connect(mGamepadInput, &GamepadInput::buttonAChanged, this, [=](bool state)
				{
					if (state)
					{
						mAppInterface->toggleActiveWindowViewerBasedPerspective();
					}
				});
				connect(mGamepadInput, &GamepadInput::buttonBChanged, this, [=](bool state)
				{
					if (state)
					{
						mAppInterface->toggleActiveWindowCenteredPerspective();
					}
				});
				
				connect(manager, &QGamepadManager::connectedGamepadsChanged, this, []() {
					ccLog::Print("Gamepad connected");
				});
			}
			else
			{
				mGamepadInput->stop(); //just in case
			}

			for (gamepadID = 0; gamepadID < 4; ++gamepadID)
			{
				mGamepadInput->setDeviceId(gamepadID);
				
				if (mGamepadInput->isConnected())
				{
					mGamepadInput->start();
					break;
				}
			}
			
			if (gamepadID == 4)
			{
				showMessage("[Gamepad] No device connected", silent);
				state = false;
				break;
			}
		}
	}
	else
	{
		ccLog::Warning("[Gamepad] Device has been disabled");
	}
	
	mActionEnable->blockSignals(true);
	mActionEnable->setChecked(state);
	mActionEnable->blockSignals(false);
}

void ccGamepadManager::releaseDevice()
{
	if (mGamepadInput == nullptr)
		return;
	
	mGamepadInput->stop(); //disconnect from the driver
	mGamepadInput->disconnect(this); //disconnect from Qt ;)
	
	delete mGamepadInput;
	mGamepadInput = nullptr;
}

void ccGamepadManager::setupMenu()
{
	mMenu = new QMenu( "Gamepad" );
	mMenu->setIcon( QIcon(":/CC/images/gamepad.png") );
	
	mActionEnable = new QAction( tr( "Enable" ), this );
	mActionEnable->setCheckable( true );
	
	connect( mActionEnable, &QAction::toggled, [this](bool state) {
		enableDevice(state, false);
	});
	
	mMenu->addAction( mActionEnable );
}

void ccGamepadManager::onGamepadInput()
{
	ccGLWindow* win = mAppInterface->getActiveGLWindow();
	if (win == nullptr)
		return;
	
	mGamepadInput->update(win);
}