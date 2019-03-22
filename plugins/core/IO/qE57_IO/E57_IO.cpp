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

#include "E57_IO.h"

#include "E57Filter.h"


E57_IO::E57_IO( QObject *parent ) :
	QObject( parent ),
	ccIOFilterPluginInterface( ":/CC/plugin/E57_IO/info.json" )
{
}

void E57_IO::registerCommands( ccCommandLineInterface *cmd )
{
	Q_UNUSED( cmd );
}

QVector<FileIOFilter::Shared> E57_IO::getFilters()
{
	return { FileIOFilter::Shared( new E57Filter ) };
}
