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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#ifndef CC_PICKING_HUB_HEADER
#define CC_PICKING_HUB_HEADER

//Local
#include "ccPickingListener.h"

//Plugins
#include <ccMainAppInterface.h>

//qCC_gl
#include <ccGLWidget.h>

//Qt
#include <QObject>

//system
#include <set>

class QMdiSubWindow;
class ccHObject;

//! Point/triangle picking hub
class ccPickingHub : public QObject
{
	Q_OBJECT

public:
	
	//! Default constructor
	ccPickingHub(ccMainAppInterface* app, QObject* parent = 0);

	//! Returns the number of currently registered listeners
	inline size_t listenerCount() const { return m_listeners.size(); }

	//! Adds a listener
	/** \param listener listener to be registered
		\param exclusive prevents new listeners from registering
		\param autoStartPicking automatically enables the picking mode on the active window (if any)
		\return success
	***/
	bool addListener(ccPickingListener* listener, bool exclusive = false, bool autoStartPicking = true);

	//! Removes a listener
	/** \param listener listener to be removed
		\param autoStopPickingIfLast automatically disables the picking mode on the active window (if any) if no other listener is registered
	***/
	void removeListener(ccPickingListener* listener, bool autoStopPickingIfLast = true);

	//! Sets the default picking mode
	/** \param mode picking mode
		\param autoEnableOnActivatedWindow whether picking mode should be enabled automatically on newly activated windows (if listeners are present only)
	**/
	//DGM: too dangerous, we can't change this behavior on the fly
	//void setPickingMode(ccGLWindow::PICKING_MODE mode, bool autoEnableOnActivatedWindow = true);
	
	//! Manual start / stop of the picking mode on the active window
	void togglePickingMode(bool state);

public slots:

	void onActiveWindowChanged(QMdiSubWindow*);
	void onActiveWindowDeleted(QObject*);
	void processPickedItem(ccHObject*, unsigned, int, int, const CCVector3&);

protected:

	//! Listeners
	std::set< ccPickingListener* > m_listeners;

	//! Associated application
	ccMainAppInterface* m_app;

	//! Active GL window
	ccGLWindow* m_activeGLWindow;

	//! Default picking mode
	ccGLWindow::PICKING_MODE m_pickingMode;

	//! Automatically enables the picking mechanism on activated GL windows
	bool m_autoEnableOnActivatedWindow;

	//! Exclusive mode
	bool m_exclusive;

};

#endif //CC_PICKING_HUB_HEADER
