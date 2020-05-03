#ifndef Q_RDB_IO_PLUGIN_HEADER
#define Q_RDB_IO_PLUGIN_HEADER

//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qRDBIO                      #
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
//#          COPYRIGHT: RIEGL Laser Measurement Systems GmbH               #
//#                                                                        #
//##########################################################################

//Qt
#include <QObject>

#include <ccIOPluginInterface.h>

//! RDB file (3D cloud)
class qRDBIO : public QObject, public ccIOPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccIOPluginInterface )
	Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.qRDBIO" FILE "info.json" )

public:

	//! Default constructor
	explicit qRDBIO( QObject *parent = nullptr );
	
	// inherited from ccIOPluginInterface
	ccIOPluginInterface::FilterList getFilters() override;
};

#endif //Q_RDB_IO_PLUGIN_HEADER
