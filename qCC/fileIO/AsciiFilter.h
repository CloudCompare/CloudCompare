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
#ifndef CC_ASCII_FILTER_HEADER
#define CC_ASCII_FILTER_HEADER

#include "FileIOFilter.h"
#include "AsciiOpenDlg.h"

#if !defined(_WIN32) && !defined(WIN32)
#include <stdint.h>
#define __int64 int64_t
#endif

//! ASCII point cloud I/O filter
class AsciiFilter : public FileIOFilter
{
public:

    //inherited from FileIOFilter
    virtual CC_FILE_ERROR loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog = true, bool* coordinatesShiftEnabled = 0, double* coordinatesShift = 0);
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, const char* filename);

protected:

	CC_FILE_ERROR saveFile(ccHObject* entity, FILE *theFile);

	CC_FILE_ERROR loadCloudFromFormatedAsciiFile(const char* filename,
                                                    ccHObject& container,
													const AsciiOpenDlg::Sequence& openSequence,
                                                    char separator,
                                                    unsigned approximateNumberOfLines,
                                                    __int64 fileSize,
                                                    unsigned skipLines=0,
													bool alwaysDisplayLoadDialog=true,
													bool* coordinatesShiftEnabled=0,
													double* coordinatesShift=0);
};

#endif
