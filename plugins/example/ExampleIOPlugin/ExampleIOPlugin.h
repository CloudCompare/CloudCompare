#ifndef EXAMPLE_IO_PLUGIN_HEADER
#define EXAMPLE_IO_PLUGIN_HEADER

//##########################################################################
//#                                                                        #
//#          CLOUDCOMPARE PLUGIN: ExampleIOPlugin                          #
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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include "ccIOPluginInterface.h"

/** Replace 'ExampleIOPlugin' by your own plugin class name throughout and then
	check 'ExampleIOPlugin.cpp' for more directions.
	
	Each plugin requires an info.json file to provide information about itself -
	the name, authors, maintainers, icon, etc..
	
	The one method you are required to implement is getFilters(). This method
	registers your file I/O methods with CloudCompare.
**/

//! Example I/O Plugin
class ExampleIOPlugin : public QObject, public ccIOPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccIOPluginInterface )
	
	Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.ExampleIO" FILE "info.json" )
	
public:
	explicit ExampleIOPlugin( QObject *parent = nullptr );
	~ExampleIOPlugin() override = default;
	
	void registerCommands( ccCommandLineInterface *cmd ) override;
	
	// inherited from ccIOPluginInterface
	ccIOPluginInterface::FilterList getFilters() override;
};

#endif
