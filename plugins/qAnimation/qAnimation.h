//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qAnimation                      #
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
//#         COPYRIGHT: Ryan Wicks, 2G Robotics Inc., 2015				   #
//#                                                                        #
//##########################################################################

#ifndef Q_ANIMATION_PLUGIN_HEADER
#define Q_ANIMATION_PLUGIN_HEADER

//qCC
#include "../ccStdPluginInterface.h"

//Qt
#include <QObject>

#include "VideoStepItem.h"

class ccGLWindow;

// Animation plugin
class qAnimation : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
		Q_INTERFACES(ccStdPluginInterface)
#ifdef CC_QT5
		//replace qDummy by the plugin name (IID should be unique - let's hope your plugin name is unique ;)
		Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qAnimation")
#endif

public:

	//! Default constructor
	qAnimation(QObject* parent = 0);

	//inherited from ccPluginInterface
	virtual QString getName() const { return "qAnimationPlugin"; }
	virtual QString getDescription() const { return "Animation plugin, used to build a movie from a series of views."; }
	virtual QIcon getIcon() const;

	//inherited from ccStdPluginInterface
	void onNewSelection(const ccHObject::Container& selectedEntities);
	virtual void getActions(QActionGroup& group);

	protected slots:

		void doAction();

protected:

	QAction* m_action;

private:


};


#endif
