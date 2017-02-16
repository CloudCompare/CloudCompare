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
#include "../ccStdPluginInterface.h"

//Qt
#include <QObject>

class ccGLWindow;

// Animation plugin
class qAnimation : public QObject, public ccStdPluginInterface
{
	Q_OBJECT

	Q_INTERFACES(ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qAnimation")

public:

	//! Default constructor
	qAnimation(QObject* parent = 0);

	//inherited from ccPluginInterface
	virtual QString getName() const override { return "Animation"; }
	virtual QString getDescription() const override { return "Animation plugin, used to build a movie from a series of views."; }
	virtual QIcon getIcon() const override;

	//inherited from ccStdPluginInterface
	void onNewSelection(const ccHObject::Container& selectedEntities) override;
	virtual void getActions(QActionGroup& group) override;

protected slots:

	void doAction();

protected:

	QAction* m_action;

};

#endif
