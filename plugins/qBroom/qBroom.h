//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qBroom                      #
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
//#      COPYRIGHT: Wesley Grimes (Collision Engineering Associates)       #
//#                                                                        #
//##########################################################################

#ifndef Q_BROOM_PLUGIN_HEADER
#define Q_BROOM_PLUGIN_HEADER

#include "../ccStdPluginInterface.h"

//! CEA virtual broom plugin
class qBroom : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qBroom")

public:

	//! Default constructor
	explicit qBroom(QObject* parent = 0);

	//inherited from ccPluginInterface
	virtual QString getName() const { return "CEA virtual broom"; }
	virtual QString getDescription() const { return "Clean a point cloud with a virtual broom"; }
	virtual QIcon getIcon() const;

	//inherited from ccStdPluginInterface
	virtual void onNewSelection(const ccHObject::Container& selectedEntities);
	virtual void getActions(QActionGroup& group);

protected slots:

	//! Slot called when associated ation is triggered
	void doAction();

protected:

	//! Associated action
	QAction* m_action;
};

#endif //Q_BROOM_PLUGIN_HEADER
