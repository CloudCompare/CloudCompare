#ifndef Q_PCL_IO_PLUGIN_HEADER
#define Q_PCL_IO_PLUGIN_HEADER

//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qPclIO                       #
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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#include <ccIOPluginInterface.h>

//! PCL IO plugin (PCD format)
class qPclIO : public QObject, public ccIOPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccPluginInterface ccIOPluginInterface )
	
	Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.qPclIO" FILE "../info.json" )

public:

	explicit qPclIO(QObject* parent = nullptr);

	//inherited from ccIOPluginInterface
	FilterList getFilters() override;

	//inherited from ccDefaultPluginInterface
	void registerCommands(ccCommandLineInterface* cmd) override;
};

#endif //Q_PCL_IO_PLUGIN_HEADER
