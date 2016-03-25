//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qGMMReg                     #
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
//#                     COPYRIGHT: CloudCompare project                    #
//#                                                                        #
//##########################################################################

#ifndef Q_GMMREG_PLUGIN_HEADER
#define Q_GMMREG_PLUGIN_HEADER

//qCC
#include "../ccStdPluginInterface.h"

//Qt
#include <QObject>

//! GMMReg qCC plugin
class qGMMRegPlugin : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qGMMReg")

public:

	//! Default constructor
	explicit qGMMRegPlugin(QObject* parent = 0);

	//inherited from ccPluginInterface
	virtual QString getName() const { return "qGMMRegPlugin"; }
	virtual QString getDescription() const { return "Non-rigid registration (based on the work by Jian and Vemuri)"; }
	virtual QIcon getIcon() const;

	//inherited from ccStdPluginInterface
	void onNewSelection(const ccHObject::Container& selectedEntities);
	virtual void getActions(QActionGroup& group);

protected slots:

	/*** ADD YOUR CUSTOM ACTIONS' SLOTS HERE ***/
	void doAction();

protected:

	//! Default action
	/** You can add as many actions as you want in a plugin.
		All actions will correspond to an icon in the dedicated
		toolbar and an entry in the plugin menu.
	**/
	QAction* m_action;
};

#endif //Q_GMMREG_PLUGIN_HEADER
