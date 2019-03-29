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

#include "qPhotoscanIO.h"

//local
#include "PhotoScanFilter.h"


qPhotoscanIO::qPhotoscanIO( QObject* parent )
	: QObject( parent )
	, ccIOPluginInterface( ":/CC/plugin/qPhotoscanIO/info.json" )
{
}

ccIOPluginInterface::FilterList qPhotoscanIO::getFilters()
{
	return { FileIOFilter::Shared( new PhotoScanFilter ) };
}
