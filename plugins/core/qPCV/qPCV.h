//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCV                        #
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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#ifndef Q_PCV_PLUGIN_HEADER
#define Q_PCV_PLUGIN_HEADER

#include "ccStdPluginInterface.h"
#include "PCVCommand.h"

//! Wrapper to the ShadeVis algorithm for computing Ambient Occlusion on meshes and point clouds
/** "Visibility based methods and assessment for detail-recovery", M. Tarini, P. Cignoni, R. Scopigno
	Proc. of Visualization 2003, October 19-24, Seattle, USA.
	http://vcg.sourceforge.net/index.php/ShadeVis
**/
class qPCV : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qPCV" FILE "info.json")

public:
	//! Default constructor
	explicit qPCV(QObject* parent = nullptr);
	
	~qPCV()override  = default;

	//inherited from ccStdPluginInterface
	void onNewSelection(const ccHObject::Container& selectedEntities) override;
	QList<QAction *> getActions() override;
	void registerCommands(ccCommandLineInterface *cmd) override;

private:
	//! Slot called when associated ation is triggered
	void doAction();

	//! Associated action
	QAction* m_action;
};

#endif
