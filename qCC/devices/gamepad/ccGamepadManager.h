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

#include <QObject>

class QAction;
class QMenu;
class QString;

class ccMainAppInterface;
class GamepadInput;


class ccGamepadManager : public QObject
{
	Q_OBJECT
	
public:
	ccGamepadManager( ccMainAppInterface *appInterface, QObject *parent );
	~ccGamepadManager();
	
	//! Gets the menu associated with gamepads
	QMenu	*menu() { return mMenu; }
	
private:
	void enableDevice(bool state, bool silent);
	void releaseDevice();

	void showMessage(QString message, bool asWarning);
	void setupMenu();
	
	void onGamepadInput();
	
	
	ccMainAppInterface	*mAppInterface;
	
	GamepadInput	*mGamepadInput;
	
	QMenu	*mMenu;
	QAction *mActionEnable;
};

#endif
