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

#ifndef CC_PHOTOSCAN_FILTER_HEADER
#define CC_PHOTOSCAN_FILTER_HEADER

//qCC_io
#include <FileIOFilter.h>

//! Photoscan (PSZ) file I/O filter
class PhotoScanFilter : public FileIOFilter
{
public:
	PhotoScanFilter();

	//inherited from FileIOFilter
	virtual CC_FILE_ERROR loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters);
};

#endif //CC_PHOTOSCAN_FILTER_HEADER
