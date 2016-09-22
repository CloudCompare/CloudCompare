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

#ifndef CC_STL_FILTER_HEADER
#define CC_STL_FILTER_HEADER

#include "FileIOFilter.h"

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
	virtual bool importSupported() const override { return true; }
	virtual bool exportSupported() const override { return true; }
	virtual CC_FILE_ERROR loadFile(QString filename, ccHObject& container, LoadParameters& parameters) override;
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, QString filename, SaveParameters& parameters) override;
	virtual QStringList getFileFilters(bool onImport) const override { return QStringList(GetFileFilter()); }
	virtual QString getDefaultExtension() const override { return GetDefaultExtension(); }
	virtual bool canLoadExtension(QString upperCaseExt) const override;
	virtual bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;

protected:

	//! Custom save method
	CC_FILE_ERROR saveToASCIIFile(ccGenericMesh* mesh, FILE *theFile, QWidget* parentWidget = 0);
	CC_FILE_ERROR saveToBINFile(ccGenericMesh* mesh, FILE *theFile, QWidget* parentWidget = 0);

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
