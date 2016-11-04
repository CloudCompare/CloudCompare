//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qM3C2                       #
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
//#            COPYRIGHT: UNIVERSITE EUROPEENNE DE BRETAGNE                #
//#                                                                        #
//##########################################################################

#ifndef Q_M3C2_PLUGIN_HEADER
#define Q_M3C2_PLUGIN_HEADER

//qCC
#include "../ccStdPluginInterface.h"

//qCC_db
#include <ccHObject.h>

//Qt
#include <QObject>

//! M3C2 plugin
/** See "Accurate 3D comparison of complex topography with terrestrial laser scanner:
	application to the Rangitikei canyon (N-Z)", Lague, D., Brodu, N. and Leroux, J.,
	2013, ISPRS journal of Photogrammmetry and Remote Sensing
**/
class qM3C2Plugin : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qM3C2")

public:

	//! Default constructor
	qM3C2Plugin(QObject* parent=0);

	//inherited from ccPluginInterface
	virtual QString getName() const { return "M3C2 distance"; }
	virtual QString getDescription() const { return "Multiscale Model to Model Cloud Comparison (M3C2)"; }
	virtual QIcon getIcon() const;

	//inherited from ccStdPluginInterface
	void onNewSelection(const ccHObject::Container& selectedEntities);
	virtual void getActions(QActionGroup& group);

protected slots:

	void doAction();

protected:

	//! Default action
	QAction* m_action;

	//! Currently selected entities
	ccHObject::Container m_selectedEntities;
};

#endif //Q_M3C2_PLUGIN_HEADER
