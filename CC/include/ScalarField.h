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

#ifndef CC_SCALAR_FIELD_HEADER
#define CC_SCALAR_FIELD_HEADER

//Local
#include "CCConst.h"
#include "GenericChunkedArray.h"

namespace CCLib
{

//! A simple scalar field (to be associated to a point cloud)
/** Extends the GenericChunkedArray object. It is equivalent to a
	mono-dimensionnal array of scalar values. It has also specific
	parameters for display purposes.

	Invalid values can be represented by NAN_VALUE.
**/
class CC_CORE_LIB_API ScalarField : public GenericChunkedArray<1, ScalarType>
{
public:

	//! Default constructor
	/** [SHAREABLE] Call 'link' when associating this structure to an object.
		\param name scalar field name
	**/
	explicit ScalarField(const char* name = 0);

	//! Copy constructor
	/** \param sf scalar field to copy
		\warning May throw a std::bad_alloc exception
	**/
	ScalarField(const ScalarField& sf);

	//! Sets scalar field name
	void setName(const char* name);

	//! Returns scalar field name
	inline const char* getName() const { return m_name; }

	//! Returns the specific NaN value
	static inline ScalarType NaN() { return NAN_VALUE; }

	//! Computes the mean value (and optionally the variance value) of the scalar field
	/** \param mean a field to store the mean value
		\param variance if not void, the variance will be computed and stored here
	**/
	void computeMeanAndVariance(ScalarType &mean, ScalarType* variance = 0) const;

	//inherited from GenericChunkedArray
	virtual void computeMinAndMax();

	//! Returns whether a scalar value is valid or not
	static inline bool ValidValue(ScalarType value) { return value == value; } //'value == value' fails for NaN values

	//! Sets the value as 'invalid' (i.e. NAN_VALUE)
	inline virtual void flagValueAsInvalid(unsigned index) { setValue(index,NaN()); }

protected:

	//! Default destructor
	/** [SHAREABLE] Call 'release' to destroy this object properly.
	**/
	virtual ~ScalarField() {}

	//! Scalar field name
	char m_name[256];
};

}

#endif //CC_SCALAR_FIELD_HEADER
