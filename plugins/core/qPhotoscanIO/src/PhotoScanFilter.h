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
class /*QCC_IO_LIB_API*/ PhotoScanFilter : public FileIOFilter
{
public:

	//static accessors
	static inline QString GetFileFilter() { return "Photoscan project (*.psz)"; }
	static inline QString GetDefaultExtension() { return "psz"; }

	//inherited from FileIOFilter
	virtual bool importSupported() const { return true; }
	virtual CC_FILE_ERROR loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters);
	virtual QStringList getFileFilters(bool onImport) const { return QStringList(GetFileFilter()); }
	virtual QString getDefaultExtension() const { return GetDefaultExtension(); }
	virtual bool canLoadExtension(const QString& upperCaseExt) const;
	virtual bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const { return false; }

};

#endif //CC_PHOTOSCAN_FILTER_HEADER
