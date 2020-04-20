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

#ifndef CC_DEPTH_MAP_FILE_FILTER_HEADER
#define CC_DEPTH_MAP_FILE_FILTER_HEADER

#include "FileIOFilter.h"

class ccGBLSensor;

//! Depth map I/O filter
class QCC_IO_LIB_API DepthMapFileFilter : public FileIOFilter
{
public:
	DepthMapFileFilter();
	
	//static accessors
	static inline QString GetFileFilter() { return "Depth Map [ascii] (*.txt *.asc)"; }

	//inherited from FileIOFilter
	bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;
	CC_FILE_ERROR saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters) override;

	//direct method to save a sensor (depth map)
	CC_FILE_ERROR saveToFile(const QString& filename, ccGBLSensor* sensor);
};

#endif //CC_DEPTH_MAP_FILE_FILTER_HEADER
