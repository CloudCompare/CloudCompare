#ifndef E57_IO_HEADER
#define E57_IO_HEADER

//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include "ccIOPluginInterface.h"

class qE57IO : public QObject, public ccIOPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccIOPluginInterface )
	
	Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.qE57IO" FILE "info.json" )
	
public:
	explicit qE57IO( QObject *parent = nullptr );
	
	void registerCommands( ccCommandLineInterface *cmd ) override;
	
	FilterList getFilters() override;
};

#endif
