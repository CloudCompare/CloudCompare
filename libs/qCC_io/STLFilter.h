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

#ifndef CC_STL_FILTER_HEADER
#define CC_STL_FILTER_HEADER

#include "FileIOFilter.h"

//QT
#include <QFile>

class ccGenericMesh;
class ccMesh;
class ccPointCloud;

//! StereoLithography file I/O filter
/** See http://www.ennex.com/~fabbers/StL.asp
**/
class QCC_IO_LIB_API STLFilter : public FileIOFilter
{
public:

	//static accessors
	static inline QString GetFileFilter() { return "STL mesh (*.stl)"; }
	static inline QString GetDefaultExtension() { return "stl"; }

	//inherited from FileIOFilter
	virtual bool importSupported() const { return true; }
	virtual bool exportSupported() const { return true; }
	virtual CC_FILE_ERROR loadFile(QString filename, ccHObject& container, LoadParameters& parameters);
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, QString filename);
	virtual QStringList getFileFilters(bool onImport) const { return QStringList(GetFileFilter()); }
	virtual QString getDefaultExtension() const { return GetDefaultExtension(); }
	virtual bool canLoadExtension(QString upperCaseExt) const;
	virtual bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const;

protected:

	//! Custom save method
	CC_FILE_ERROR saveToASCIIFile(ccGenericMesh* mesh, FILE *theFile);
	CC_FILE_ERROR saveToBINFile(ccGenericMesh* mesh, FILE *theFile);

	//! Custom load method for ASCII files
	CC_FILE_ERROR loadASCIIFile(QFile& fp,
								ccMesh* mesh,
								ccPointCloud* vertices,
								LoadParameters& parameters);

	//! Custom load method for binary files
	CC_FILE_ERROR loadBinaryFile(QFile& fp,
								ccMesh* mesh,
								ccPointCloud* vertices,
								LoadParameters& parameters);
};

#endif //CC_STL_FILTER_HEADER
