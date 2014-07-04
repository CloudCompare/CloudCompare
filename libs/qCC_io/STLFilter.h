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
class STLFilter : public FileIOFilter
{
public:

	//inherited from FileIOFilter
	virtual CC_FILE_ERROR loadFile(QString filename, ccHObject& container, bool alwaysDisplayLoadDialog = true, bool* coordinatesShiftEnabled = 0, CCVector3d* coordinatesShift = 0);
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, QString filename);

protected:

	//! Custom save method
	CC_FILE_ERROR saveToASCIIFile(ccGenericMesh* mesh, FILE *theFile);
	CC_FILE_ERROR saveToBINFile(ccGenericMesh* mesh, FILE *theFile);

	//! Custom load method for ASCII files
	CC_FILE_ERROR loadASCIIFile(QFile& fp,
								ccMesh* mesh,
								ccPointCloud* vertices,
								bool alwaysDisplayLoadDialog,
								bool* coordinatesShiftEnabled=0,
								CCVector3d* coordinatesShift=0);

	//! Custom load method for binary files
	CC_FILE_ERROR loadBinaryFile(QFile& fp,
								ccMesh* mesh,
								ccPointCloud* vertices,
								bool alwaysDisplayLoadDialog,
								bool* coordinatesShiftEnabled=0,
								CCVector3d* coordinatesShift=0);
};

#endif //CC_STL_FILTER_HEADER
