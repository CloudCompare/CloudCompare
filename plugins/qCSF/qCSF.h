//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qCSF                        #
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
//#                      COPYRIGHT: Qi jianbo ; Wan peng                   #
//#                                                                        #
//##########################################################################

#ifndef Q_CSF_PLUGIN_HEADER
#define Q_CSF_PLUGIN_HEADER

#include "../ccStdPluginInterface.h"

//! A point-clouds filtering algorithm utilize cloth simulation process. 
class qCSF : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qCSF")

public:

	//! Default constructor
	explicit qCSF(QObject* parent = 0);

	//inherited from ccPluginInterface
	virtual QString getName() const { return "CSF Filter"; }
	virtual QString getDescription() const { return "A point-clouds filtering algorithm utilize cloth simulation process(Qi jianbo,Wan peng,2015)."; }
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

#endif
