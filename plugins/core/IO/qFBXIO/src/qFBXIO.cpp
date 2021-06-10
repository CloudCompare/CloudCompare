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

#include "qFBXIO.h"

#include "FBXCommand.h"
#include "FBXFilter.h"


qFBXIO::qFBXIO( QObject *parent ) :
    QObject( parent ),
    ccIOPluginInterface( ":/CC/plugin/qFBXIO/info.json" )
{
}

void qFBXIO::registerCommands( ccCommandLineInterface *cmd )
{
	cmd->registerCommand( ccCommandLineInterface::Command::Shared( new FBXCommand ) );
}

ccIOPluginInterface::FilterList qFBXIO::getFilters()
{
	return { FileIOFilter::Shared( new FBXFilter ) };
}
