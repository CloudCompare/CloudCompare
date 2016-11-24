#ifndef CCGAMEPADMANAGER_H
#define CCGAMEPADMANAGER_H

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

//Qt
#include <QObject>

class QAction;
class QMenu;
class QString;

class ccMainAppInterface;
class GamepadInput;

//! Gamepad manager
class ccGamepadManager : public QObject
{
	Q_OBJECT
	
public:
	ccGamepadManager( ccMainAppInterface *appInterface, QObject *parent );
	~ccGamepadManager();
	
	//! Returns the menu associated with gamepads
	QMenu* menu() { return m_menu; }
	
protected: //methods
	void enableDevice(bool state, bool silent, int deviceID = -1);
	void releaseDevice();

	void showMessage(QString message, bool asWarning);
	void setupMenu();
	void setupGamepadInput();
	
	void onGamepadInput();
	
protected: //members

	ccMainAppInterface* m_appInterface;
	GamepadInput* m_gamepadInput;
	QMenu* m_menu;
	QAction* m_actionEnable;
};

#endif //CCGAMEPADMANAGER_H
