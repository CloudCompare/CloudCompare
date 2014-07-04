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

#ifndef CC_BUNDLER_FILTER_HEADER
#define CC_BUNDLER_FILTER_HEADER

#include "FileIOFilter.h"

//! Noah Snavely's Bundler output file filter
/** See http://phototour.cs.washington.edu/
**/
class BundlerFilter : public FileIOFilter
{
public:

	//inherited from FileIOFilter
	virtual CC_FILE_ERROR loadFile(QString filename, ccHObject& container, bool alwaysDisplayLoadDialog = true, bool* coordinatesShiftEnabled = 0, CCVector3d* coordinatesShift = 0);
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, QString filename) {/*NOT IMPLEMENTED*/return CC_FERR_NO_SAVE;}

	//! Specific load method
	CC_FILE_ERROR loadFileExtended(	const QString& filename,
									ccHObject& container,
									bool displayLoadDialog,
									bool* coordinatesShiftEnabled = 0,
									CCVector3d* coordinatesShift = 0,
									const QString& altKeypointsFilename = QString(),
									bool undistortImages = false,
									bool generateColoredDTM = false,
									unsigned coloredDTMVerticesCount = 1000000,
									float scaleFactor = 1.0f);

};

#endif
