//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_MASCARET_FILTER_HEADER
#define CC_MASCARET_FILTER_HEADER

#include "FileIOFilter.h"

//! Mascaret profile I/O filter
/** See http://www.opentelemac.org/
**/
class QCC_IO_LIB_API MascaretFilter : public FileIOFilter
{
public:

	//static accessors
	static inline QString GetFileFilter() { return "(Geo-)Mascaret profile (*.georef)"; }
	static inline QString GetDefaultExtension() { return "georef"; }

	//inherited from FileIOFilter
	virtual bool importSupported() const { return false; }
	virtual bool exportSupported() const { return true; }
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, QString filename, SaveParameters& parameters);
	virtual QStringList getFileFilters(bool onImport) const { return QStringList(GetFileFilter()); }
	virtual QString getDefaultExtension() const { return GetDefaultExtension(); }
	virtual bool canLoadExtension(QString upperCaseExt) const { return false; }
	virtual bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const;
};

#endif //CC_MASCARET_FILTER_HEADER
