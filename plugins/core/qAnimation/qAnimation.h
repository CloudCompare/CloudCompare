//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qAnimation                      #
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
//#             COPYRIGHT: Ryan Wicks, 2G Robotics Inc., 2015              #
//#                                                                        #
//##########################################################################

#ifndef Q_ANIMATION_PLUGIN_HEADER
#define Q_ANIMATION_PLUGIN_HEADER

//qCC
#include "ccStdPluginInterface.h"

//Qt
#include <QObject>

class ccGLWindow;

// Animation plugin
class qAnimation : public QObject, public ccStdPluginInterface
{
	Q_OBJECT

	Q_INTERFACES(ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qAnimation" FILE "info.json")

public:

	//! Default constructor
	qAnimation(QObject* parent = nullptr);
	
	virtual ~qAnimation() = default;
	
	//inherited from ccStdPluginInterface
	void onNewSelection(const ccHObject::Container& selectedEntities) override;
	virtual QList<QAction *> getActions() override;

private:

	void doAction();

	QAction* m_action;
};

#endif
