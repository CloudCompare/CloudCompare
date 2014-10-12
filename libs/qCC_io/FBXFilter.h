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

#ifndef CC_FBX_FILTER_HEADER
#define CC_FBX_FILTER_HEADER

#include "FileIOFilter.h"

#ifdef CC_FBX_SUPPORT

//! Autodesk FBX format I/O filter
/** http://www.autodesk.com/products/fbx/overview
**/
class QCC_IO_LIB_API FBXFilter : public FileIOFilter
{
public:

	//static accessors
	static inline QString GetFileFilter() { return "FBX mesh (*.fbx)"; }
	static inline QString GetDefaultExtension() { return "fbx"; }

	//inherited from FileIOFilter
	virtual bool importSupported() const { return true; }
	virtual bool exportSupported() const { return true; }
	virtual CC_FILE_ERROR loadFile(QString filename, ccHObject& container, LoadParameters& parameters);
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, QString filename);
	virtual QStringList getFileFilters(bool onImport) const { return QStringList(GetFileFilter()); }
	virtual QString getDefaultExtension() const { return GetDefaultExtension(); }
	virtual bool canLoadExtension(QString upperCaseExt) const;
	virtual bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const;

	//! Sets default output format (will prevent the dialog to appear when saving FBX files)
	static void SetDefaultOutputFormat(QString format);
};

#endif //CC_FBX_FILTER_HEADER

#endif //CC_FBX_FILTER_HEADER
