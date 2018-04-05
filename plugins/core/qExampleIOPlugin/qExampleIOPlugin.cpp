//##########################################################################
//#                                                                        #
//#         CLOUDCOMPARE PLUGIN: qExampleIOPlugin                          #
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

#include "qExampleIOPlugin.h"

#include "FooFilter.h"


qExampleIOPlugin::qExampleIOPlugin( QObject* parent ) :
	QObject( parent )
  , ccIOFilterPluginInterface( ":/CC/plugin/qExampleIOPlugin/info.json" )
{
}

void qExampleIOPlugin::registerCommands( ccCommandLineInterface *cmd )
{
	// If you want to register this plugin for the command line, create a
	// ccCommandLineInterface::Command and add it here. e.g.:
	//
	// cmd->registerCommand( ccCommandLineInterface::Command::Shared( new FooCommand ) );	
}

QVector<FileIOFilter::Shared> qExampleIOPlugin::getFilters()
{
	return QVector<FileIOFilter::Shared>{
		FileIOFilter::Shared( new FooFilter ),
	};
}
