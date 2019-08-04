#ifndef FBX_IO_HEADER
#define FBX_IO_HEADER

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

class qFBXIO : public QObject, public ccIOPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccIOPluginInterface )

	Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.qFBXIO" FILE "info.json" )

public:
	explicit qFBXIO( QObject *parent = nullptr );

	void registerCommands( ccCommandLineInterface *cmd ) override;

	FilterList getFilters() override;
};

#endif
