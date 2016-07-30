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
class QCC_IO_LIB_API IcmFilter : public FileIOFilter
{
public:

	//static accessors
	static inline QString GetFileFilter() { return "Clouds + calibrated images [meta][ascii] (*.icm)"; }
	static inline QString GetDefaultExtension() { return "icm"; }

	//inherited from FileIOFilter
	virtual bool importSupported() const override { return true; }
	virtual CC_FILE_ERROR loadFile(QString filename, ccHObject& container, LoadParameters& parameters) override;
	virtual QStringList getFileFilters(bool onImport) const override { return QStringList(GetFileFilter()); }
	virtual QString getDefaultExtension() const override { return GetDefaultExtension(); }
	virtual bool canLoadExtension(QString upperCaseExt) const override;
	virtual bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;

protected:

	static int LoadCalibratedImages(ccHObject* entities, const QString& path, const QString& imageDescFilename, const ccBBox& globalBBox);
};

#endif //CC_ICM_FILTER_HEADER
