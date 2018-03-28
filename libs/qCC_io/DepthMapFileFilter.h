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

	//static accessors
	static inline QString GetFileFilter() { return "Depth Map [ascii] (*.txt *.asc)"; }
	static inline QString GetDefaultExtension() { return "txt"; }

	//inherited from FileIOFilter
	virtual bool exportSupported() const override { return true; }
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters) override;
	virtual QStringList getFileFilters(bool onImport) const override { return QStringList(GetFileFilter()); }
	virtual QString getDefaultExtension() const override { return GetDefaultExtension(); }
	virtual bool canLoadExtension(const QString& upperCaseExt) const override;
	virtual bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;

	//direct method to save a sensor (depth map)
	CC_FILE_ERROR saveToFile(const QString& filename, ccGBLSensor* sensor);

};

#endif //CC_DEPTH_MAP_FILE_FILTER_HEADER
