//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qPhotoScanIO                    #
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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#ifndef Q_PHOTOSCAN_IO_PLUGIN_HEADER
#define Q_PHOTOSCAN_IO_PLUGIN_HEADER

#include "ccIOPluginInterface.h"

//! PhotoScan
class qPhotoscanIO : public QObject, public ccIOPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccIOPluginInterface )
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qPhotoscanIO" FILE "info.json")

public:
	explicit qPhotoscanIO( QObject *parent = nullptr );
	
	~qPhotoscanIO() override = default;

	//inherited from ccIOPluginInterface
	FilterList getFilters() override;
};

#endif //Q_PHOTOSCAN_IO_PLUGIN_HEADER
