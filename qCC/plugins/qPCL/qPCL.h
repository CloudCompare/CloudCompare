//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#               COPYRIGHT: Luca Penasa                                   #
//#                                                                        #
//##########################################################################
//
#ifndef Q_PCL_PLUGIN_HEADER
#define Q_PCL_PLUGIN_HEADER

#include "../ccStdPluginInterface.h"

//Qt
#include <QObject>
#include <QtGui>

class BaseFilter;
class QToolBar;
class QMenu;

//! PCL bridge plugin
class qPCL : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)

public:

	//! Default constructor
	qPCL();

	//! Destructor
	virtual ~qPCL();

	//inherited from ccPluginInterface
	void getDescription(ccPluginDescription& desc);
	virtual QIcon getIcon() const;

	//inherited from ccStdPluginInterface
	bool onNewSelection(const ccHObject::Container& selectedEntities);
	int doAction(ccHObject::Container& selectedEntities,
		unsigned& uiModificationFlags,
		ccProgressDialog* progressCb=NULL,
		QWidget* parent=NULL);
	QString getErrorMessage(int errorCode/*, LANGUAGE lang*/);

	//! Adds a filter
	int addFilter(BaseFilter* filter);

public slots:
	//! Handles new entity
	void handleNewEntity(ccHObject*);

	//! Handles entity (visual) modification
	void handleEntityChange(ccHObject*);

	//! Handles new error message
	void handleErrorMessage(QString);

protected:

	//! Toolbar
	QToolBar* m_toolbar;

	//! Menu
	QMenu* m_menu;

	//! Loaded filters
	std::vector<BaseFilter*> m_filters;
};


#endif//END Q_PCL_PLUGIN_HEADER
