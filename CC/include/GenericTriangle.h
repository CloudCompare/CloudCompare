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

#ifndef GENERIC_TRIANGLE_HEADER
#define GENERIC_TRIANGLE_HEADER

//Local
#include "CCCoreLib.h"
#include "CCGeom.h"

namespace CCLib
{

//! A generic triangle interface for data communication between library and client applications
class CC_CORE_LIB_API GenericTriangle
{
public:

	//! Default destructor
	virtual ~GenericTriangle() {}

	//! Returns the first summit (A)
	/**	Virtual method to access first summit data
		\return the first summit (as a reference to a Generic3dPoint object)
	**/
	virtual const CCVector3* _getA() const = 0;

	//! Returns the second summit (B)
	/**	Virtual method to access second summit data
		\return the second summit (as a reference to a Generic3dPoint object)
	**/
	virtual const CCVector3* _getB() const = 0;

	//! Returns the third summit (C)
	/**	Virtual method to access third summit data
		\return the third summit (as a reference to a Generic3dPoint object)
	**/
	virtual const CCVector3* _getC() const = 0;
};

}

#endif //GENERIC_TRIANGLE_HEADER
