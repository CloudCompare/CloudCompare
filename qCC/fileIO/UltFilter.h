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
//$Rev:: 1879                                                              $
//$LastChangedDate:: 2011-08-29 12:35:30 +0200 (lun., 29 août 2011)       $
//**************************************************************************
//
#ifndef CC_ULT_FILTER_HEADER
#define CC_ULT_FILTER_HEADER

#include "FileIOFilter.h"

#ifdef CC_ULT_SUPPORT

//! CloudCompare dedicated ULT point cloud I/O filter
class UltFilter : public FileIOFilter
{
public:

    //inherited from FileIOFilter
    virtual CC_FILE_ERROR loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog = true, bool* coordinatesShiftEnabled = 0, double* coordinatesShift = 0);
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, const char* filename);

protected:

};

#endif //CC_ULT_SUPPORT

#endif
