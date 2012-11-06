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
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//
#ifndef CC_E57_FILTER_HEADER
#define CC_E57_FILTER_HEADER

#include "FileIOFilter.h"

#ifdef CC_E57_SUPPORT

//! E57 filter (based on libE57)
class E57Filter : public FileIOFilter
{

public:

    //inherited from FileIOFilter
    virtual CC_FILE_ERROR loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog = true, bool* coordinatesShiftEnabled = 0, double* coordinatesShift = 0);
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, const char* filename);

};

#endif //CC_E57_SUPPORT

#endif
