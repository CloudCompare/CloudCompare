//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
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

#ifndef GENERIC_OCTREE_HEADER
#define GENERIC_OCTREE_HEADER

#include "CCGeom.h"

namespace CCLib
{

//! A generic octree interface for data communication between library and client applications

#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"

class CC_DLL_API GenericOctree
#else
class GenericOctree
#endif
{
public:

	//! Default destructor
    virtual ~GenericOctree() {};


};

}

#endif
