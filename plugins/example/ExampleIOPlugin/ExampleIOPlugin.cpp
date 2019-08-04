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

// First:
//	Replace all occurrences of 'ExampleIOPlugin' by your own plugin class name in this file.
//	This includes the resource path to info.json in the constructor.

// Second:
//	Open ExampleIOPlugin.qrc, change the "prefix" and the icon filename for your plugin.
//	Change the name of the file to <yourPluginName>.qrc

// Third:
//	Open the info.json file and fill in the information about the plugin.
//	 "type" should be: "I/O"
//	 "name" is the name of the plugin (required)
//	 "icon" is the Qt resource path to the plugin's icon (from the .qrc file)
//	 "description" is used as a tootip if the plugin has actions and is displayed in the plugin dialog
//	 "authors", "maintainers", and "references" show up in the plugin dialog as well

#include "ExampleIOPlugin.h"

#include "FooFilter.h"


ExampleIOPlugin::ExampleIOPlugin( QObject* parent )
    : QObject( parent )
    , ccIOPluginInterface( ":/CC/plugin/ExampleIOPlugin/info.json" )
{
}

void ExampleIOPlugin::registerCommands( ccCommandLineInterface *cmd )
{
	// If you want to register this plugin for the command line, create a
	// ccCommandLineInterface::Command and add it here. e.g.:
	//
	// cmd->registerCommand( ccCommandLineInterface::Command::Shared( new FooCommand ) );
}

ccIOPluginInterface::FilterList ExampleIOPlugin::getFilters()
{
	return {
		FileIOFilter::Shared( new FooFilter ),
	};
}
