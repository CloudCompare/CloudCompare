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

#ifndef Q_PCL_IO_PLUGIN_HEADER
#define Q_PCL_IO_PLUGIN_HEADER

#include "ccIOPluginInterface.h"

//Qt
#include <QObject>

//! PCL IO plugin (PCD format)
class qPclIO : public QObject, public ccIOPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccIOPluginInterface )
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qPclIO" FILE "info.json")

public:

	explicit qPclIO(QObject* parent = nullptr);

	//inherited from ccIOPluginInterface
	ccIOPluginInterface::FilterList getFilters() override;
};

#endif //Q_PCL_IO_PLUGIN_HEADER
