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

#ifndef CC_ICM_FILTER_HEADER
#define CC_ICM_FILTER_HEADER

#include "FileIOFilter.h"

//! Calibrated images and cloud meta-file I/O filter
class IcmFilter : public FileIOFilter
{
public:
	IcmFilter();

	//inherited from FileIOFilter
	CC_FILE_ERROR loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters) override;

private:
	static int LoadCalibratedImages(ccHObject* entities, const QString& path, const QString& imageDescFilename, const ccBBox& globalBBox);
};

#endif //CC_ICM_FILTER_HEADER
