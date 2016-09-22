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

#include "../../ccIOFilterPluginInterface.h"

//Qt
#include <QObject>

//! PCL IO plugin (PCD format)
class qPclIO : public QObject, public ccIOFilterPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccIOFilterPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qPclIO")

public:

	qPclIO() {}
	virtual ~qPclIO() {}


	//inherited from ccPluginInterface
	virtual QString getName() const { return "PCL I/O filter"; }
	virtual QString getDescription() const { return "Interface to open PCL clouds (*.pcd)"; }

	//inherited from ccIOFilterPluginInterface
	FileIOFilter::Shared getFilter();
};

#endif //Q_PCL_IO_PLUGIN_HEADER
