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

#ifndef GENERIC_INDEXED_PERSIST_CLOUD_HEADER
#define GENERIC_INDEXED_PERSIST_CLOUD_HEADER

#include "GenericIndexedCloud.h"

namespace CCLib
{

//! A generic 3D point cloud with index-based and presistent access to points
/** Implements the GenericIndexedCloud interface.
**/

#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"

class CC_DLL_API GenericIndexedCloudPersist : virtual public GenericIndexedCloud
#else
class GenericIndexedCloudPersist : virtual public GenericIndexedCloud
#endif
{

public:

	//! Default destructor
	virtual ~GenericIndexedCloudPersist() {};

	//! Returns the ith point as a persistent pointer
	/**	Virtual method to request a point with a specific index.
		WARNING: the returned object MUST be persistent in order
		to be compatible with parallel strategies!
		\param index of the requested point (between 0 and the cloud size minus 1)
		\return the requested point (or 0 if index is invalid)
	**/
	virtual const CCVector3* getPointPersistentPtr(unsigned index) = 0;
};

}

#endif
