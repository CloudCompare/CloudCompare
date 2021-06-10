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

#include "qPDALIO.h"

#include "LASFilter.h"


qPDALIO::qPDALIO( QObject *parent ) :
    QObject( parent ),
    ccIOPluginInterface( ":/CC/plugin/qPDALIO/info.json" )
{
}

void qPDALIO::registerCommands( ccCommandLineInterface *cmd )
{
	Q_UNUSED( cmd );
}

ccIOPluginInterface::FilterList qPDALIO::getFilters()
{
	return { FileIOFilter::Shared( new LASFilter ) };
}
