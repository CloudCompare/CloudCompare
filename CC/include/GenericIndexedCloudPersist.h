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

#ifndef GENERIC_INDEXED_PERSIST_CLOUD_HEADER
#define GENERIC_INDEXED_PERSIST_CLOUD_HEADER

//Local
#include "GenericIndexedCloud.h"

namespace CCLib
{

//! A generic 3D point cloud with index-based and presistent access to points
/** Implements the GenericIndexedCloud interface.
**/
class CC_CORE_LIB_API GenericIndexedCloudPersist : virtual public GenericIndexedCloud
{
public:
	//! Default constructor
	GenericIndexedCloudPersist() : GenericIndexedCloud() {}

	//! Mock constructor for compatibility with the PointCloudTpl interface
	/** \warning Parameters are simply ignored
		\param name ignored
		\param ID ignored
	**/
	GenericIndexedCloudPersist(const char* name, unsigned ID) : GenericIndexedCloud() { /* input parameters are ignored */ }


	//! Default destructor
	 ~GenericIndexedCloudPersist() override = default;

	//! Returns the ith point as a persistent pointer
	/**	Virtual method to request a point with a specific index.
		WARNING: the returned object MUST be persistent in order
		to be compatible with parallel strategies!
		\param index of the requested point (between 0 and the cloud size minus 1)
		\return the requested point (or 0 if index is invalid)
	**/
	virtual const CCVector3* getPointPersistentPtr(unsigned index) const = 0;
};

}

#endif //GENERIC_INDEXED_PERSIST_CLOUD_HEADER
