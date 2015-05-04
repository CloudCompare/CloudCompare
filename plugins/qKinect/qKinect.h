//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qKinect                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#               COPYRIGHT: Daniel Girardeau-Montaut                      #
//#                                                                        #
//##########################################################################

#ifndef Q_KINECT_PLUGIN_HEADER
#define Q_KINECT_PLUGIN_HEADER

//Qt
#include <QObject>

#include "../ccStdPluginInterface.h"

class ccKinectDlg;
class QAction;

//! Kinect 3D stream capture plugin
/** This plugin is based on libfreenect (https://github.com/OpenKinect/libfreenect)
**/
class qKinect : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
#ifdef CC_QT5
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qKinect")
#endif

public:

	//! Default constructor
	explicit qKinect(QObject* parent = 0);

	//! Destructor
	virtual ~qKinect();

	//inherited from ccPluginInterface
	virtual QString getName() const { return "Kinect capture"; }
	virtual QString getDescription() const { return "Kinect 3D stream capture (with libfreenect)"; }
	virtual QIcon getIcon() const;

	//inherited from ccStdPluginInterface
	//virtual void onNewSelection(const ccHObject::Container& selectedEntities);
	virtual void getActions(QActionGroup& group);

protected slots:

	//! Starts actions
	void doStartGrabbing();

	//! Grab cloud
	void grabCloud();

	//! On dialog end
	void dialogClosed(int);

	//! Updates real time view
	void updateRTView();

protected:

	//! Associated dialog
	ccKinectDlg* m_kDlg;

	//! Timer to update RT view
	QTimer* m_timer;

	//! Associated action
	QAction* m_action;
};

#endif //Q_KINECT_PLUGIN_HEADER
