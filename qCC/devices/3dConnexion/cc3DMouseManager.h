#ifndef CC3DMOUSEMANAGER_H
#define CC3DMOUSEMANAGER_H

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

class ccMainAppInterface;
class Mouse3DInput;


class cc3DMouseManager : public QObject
{
	Q_OBJECT
	
public:
	cc3DMouseManager( ccMainAppInterface *appInterface, QObject *parent );
	~cc3DMouseManager();
	
	//! Gets the menu associated with the 3D mouse
	QMenu	*menu() { return mMenu3DMouse; }
	
	//! Tries to enable (or disable) a 3D mouse device
	/** \param state whether to enable or disable the device
	 *	\param silent whether to issue an error message in case of failure
	**/
	void enable3DMouse(bool state, bool silent);
	
	//! Releases any connected 3D mouse
	void release3DMouse();
	
private:
	void setupMenu();
	
	void on3DMouseKeyUp(int key);
	void on3DMouseKeyDown(int key);
	void on3DMouseMove(std::vector<float> &vec);
	void on3DMouseReleased();
	
	
	ccMainAppInterface	*mAppInterface;
	
	Mouse3DInput	*m3dMouseInput;
	
	QMenu	*mMenu3DMouse;
	QAction *mActionEnable3DMouse;
};

#endif
