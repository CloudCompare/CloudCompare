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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2224                                                              $
//$LastChangedDate:: 2012-07-25 19:13:23 +0200 (mer., 25 juil. 2012)       $
//**************************************************************************
//
#ifndef CC_BIN_FILTER_HEADER
#define CC_BIN_FILTER_HEADER

#include "FileIOFilter.h"

//qCC_db
#include <ccFlags.h>

//Qt
#include <QFile>

class ccGenericPointCloud;

//! CloudCompare dedicated binary point cloud I/O filter
class BinFilter : public FileIOFilter
{
public:

    //inherited from FileIOFilter
    virtual CC_FILE_ERROR loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog = true, bool* coordinatesShiftEnabled = 0, double* coordinatesShift = 0);
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, const char* filename);

protected:

	//! old style BIN loading
    virtual CC_FILE_ERROR loadFileV1(QFile& in, ccHObject& container, unsigned nbScansTotal);

	//! new style BIN loading
    virtual CC_FILE_ERROR loadFileV2(QFile& in, ccHObject& container);
	//virtual CC_FILE_ERROR saveToFileV2(ccHObject* entity, const char* filename);

	//! Per-cloud header flags (old style)
	union HeaderFlags
	{
		struct
		{
			bool bit1;			//bit 1
			bool colors;		//bit 2
			bool normals;		//bit 3
			bool scalarField;	//bit 4
			bool name;			//bit 5
			bool sfName;		//bit 6
			bool bit7;			//bit 7
			bool bit8;			//bit 8
		};
		ccFlags flags;

		//! Default constructor
		HeaderFlags()
		{
			flags.reset();
			bit1=true; //bit '1' is always ON!
		}
	};

	//specific methods (old style)
	static int ReadEntityHeader(QFile& in, unsigned& numberOfPoints, HeaderFlags& header);
};

#endif
