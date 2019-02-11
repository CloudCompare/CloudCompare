//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef GENERIC_OCTREE_HEADER
#define GENERIC_OCTREE_HEADER

//Local
#include "CCGeom.h"

namespace CCLib
{

//! A generic octree interface for data communication between library and client applications
class CC_CORE_LIB_API GenericOctree
{
public:

	//! Default destructor
    virtual ~GenericOctree() = default;
};

}

#endif //GENERIC_OCTREE_HEADER
