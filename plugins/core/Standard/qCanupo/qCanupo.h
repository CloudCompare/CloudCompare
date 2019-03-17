//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qCANUPO                       #
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
//#      COPYRIGHT: UEB (UNIVERSITE EUROPEENNE DE BRETAGNE) / CNRS         #
//#                                                                        #
//##########################################################################

#ifndef Q_CANUPO_PLUGIN_HEADER
#define Q_CANUPO_PLUGIN_HEADER

//qCC
#include <ccStdPluginInterface.h>

//qCC_db
#include <ccHObject.h>

//! CANUPO plugin
/** See "3D Terrestrial lidar data classification of complex natural scenes using a multi-scale dimensionality criterion:
	applications in geomorphology", N. Brodu, D. Lague, 2012, Computer Vision and Pattern Recognition
**/
class qCanupoPlugin : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qCanupo" FILE "info.json")

public:

	//! Default constructor
	qCanupoPlugin(QObject* parent = nullptr);

	//inherited from ccStdPluginInterface
	void onNewSelection(const ccHObject::Container& selectedEntities) override;
	virtual QList<QAction*> getActions() override;
	virtual void registerCommands(ccCommandLineInterface* cmd) override;

protected slots:

	void doClassifyAction();
	void doTrainAction();

protected:

	//! Calssift action
	QAction* m_classifyAction;
	//! Train action
	QAction* m_trainAction;

	//! Currently selected entities
	ccHObject::Container m_selectedEntities;
};

#endif //Q_CANUPO_HEADER
