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

#include "../ccIOFilterPluginInterface.h"

//! PhotoScan
class qPhotoscanIO : public QObject, public ccIOFilterPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccIOFilterPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qPhotoscanIO")

public:

	//inherited from ccPluginInterface
	virtual QString getName() const { return "Photoscan I/O filter"; }
	virtual QString getDescription() const { return "Photoscan (PSZ) I/O filter"; }

	//inherited from ccIOFilterPluginInterface
	FileIOFilter::Shared getFilter();
};

#endif //Q_PHOTOSCAN_IO_PLUGIN_HEADER
