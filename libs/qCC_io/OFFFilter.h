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

#ifndef CC_OFF_FILTER_HEADER
#define CC_OFF_FILTER_HEADER

#include "FileIOFilter.h"

//! OFF file I/O filter
/** See http://people.sc.fsu.edu/~jburkardt/data/off/off.html
**/
class QCC_IO_LIB_API OFFFilter : public FileIOFilter
{
public:

	//static accessors
	static inline QString GetFileFilter() { return "OFF mesh (*.off)"; }
	static inline QString GetDefaultExtension() { return "off"; }

	//inherited from FileIOFilter
	virtual bool importSupported() const { return true; }
	virtual bool exportSupported() const { return true; }
	virtual CC_FILE_ERROR loadFile(QString filename, ccHObject& container, LoadParameters& parameters);
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, QString filename);
	virtual QStringList getFileFilters(bool onImport) const { return QStringList(GetFileFilter()); }
	virtual QString getDefaultExtension() const { return GetDefaultExtension(); }
	virtual bool canLoadExtension(QString upperCaseExt) const;
	virtual bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const;

};

#endif //CC_OFF_FILTER_HEADER
