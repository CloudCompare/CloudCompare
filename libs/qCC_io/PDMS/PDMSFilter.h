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

#ifndef CC_PDMS_FILTER_HEADER
#define CC_PDMS_FILTER_HEADER

#ifdef CC_PDMS_SUPPORT

#include "../FileIOFilter.h"

//! PDMS .mac file I/O filter
class QCC_IO_LIB_API PDMSFilter : public FileIOFilter
{
public:

	//static accessors
	static inline QString GetFileFilter() { return "PDMS primitives (*.pdms *.pdmsmac *.mac)"; }
	static inline QString GetDefaultExtension() { return "pdms"; }

	//inherited from FileIOFilter
	virtual bool importSupported() const override { return true; }
	virtual CC_FILE_ERROR loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters) override;
	virtual QStringList getFileFilters(bool onImport) const override { return QStringList(GetFileFilter()); }
	virtual QString getDefaultExtension() const override { return GetDefaultExtension(); }
	virtual bool canLoadExtension(const QString& upperCaseExt) const override;
	virtual bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;

};

#endif //CC_PDMS_SUPPORT

#endif //CC_PDMS_FILTER_HEADER
