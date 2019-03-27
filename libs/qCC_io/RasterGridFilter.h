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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_RASTER_FILTER_HEADER
#define CC_RASTER_FILTER_HEADER

#include "FileIOFilter.h"

#ifdef CC_GDAL_SUPPORT

//! Raster grid format file I/O filter
/** Multiple formats are handled: see GDAL (http://www.gdal.org/)
**/
class QCC_IO_LIB_API RasterGridFilter : public FileIOFilter
{
public:
	RasterGridFilter();

	//inherited from FileIOFilter
	CC_FILE_ERROR loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters) override;
};

#endif //CC_GDAL_SUPPORT

#endif //CC_RASTER_FILTER_HEADER
