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
class STLFilter : public FileIOFilter
{
public:
	STLFilter();
	
	//inherited from FileIOFilter
	CC_FILE_ERROR loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters) override;

	bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;
	CC_FILE_ERROR saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters) override;

private:
	//! Custom save method
	CC_FILE_ERROR saveToASCIIFile(ccGenericMesh* mesh, FILE *theFile, QWidget* parentWidget = nullptr);
	CC_FILE_ERROR saveToBINFile(ccGenericMesh* mesh, FILE *theFile, QWidget* parentWidget = nullptr);

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
