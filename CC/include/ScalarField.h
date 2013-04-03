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

#ifndef CC_SCALAR_FIELD_HEADER
#define CC_SCALAR_FIELD_HEADER

//local
#include "CCConst.h"
#include "CCTypes.h"
#include "GenericChunkedArray.h"

namespace CCLib
{

//! A simple scalar field (to be associated to a point cloud)
/** Extends the GenericChunkedArray object. It is equivalent to a
	mono-dimensionnal array of scalar values. It has also specific
	parameters for display purposes.

    There are two kinds of scalar fields:
    - 'positive' scalar fields that ignore all negative values. Those values
        are considered as invalid values (however HIDDEN_VALUE should be used
		to represent them by default). They are not taken into account when
		computing min and max values, etc.
		This kind of scalar field is typically used to store positive distances.
    - 'standard' scalar fields can store any value. Invalid values (hidden points)
		can be represented by NAN_VALUE.
**/

#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"

class CC_DLL_API ScalarField : public GenericChunkedArray<1,ScalarType>
#else
class ScalarField : public GenericChunkedArray<1,ScalarType>
#endif
{
public:

	//! Default constructor
	/** [SHAREABLE] Call 'link' when associating this structure to an object.
		\param name scalar field name
        \param positive specifies whether negative values should be ignored
    **/
	ScalarField(const char* name = 0, bool positive = false);

	//! Sets scalar field name
    void setName(const char* name);

	//! Returns scalar field name
	inline const char* getName() const { return m_name; }

	//! Sets whether scalar field is positive or not
	/** Warning: 'computeMinAndMax' should be called afterwards.
		\param state whether values are only positive or not
	**/
	inline void setPositive(bool state) { m_onlyPositiveValues = state; }

	//! Auto detects if scalar field is positive or not
	/** Warning: 'computeMinAndMax' should be called afterwards.
		\return whether scalar field is only positive or not
	**/
	bool setPositiveAuto();

	//! Returns true if negative values are ignored (strictly positive scalar field)
	inline bool isPositive() const { return m_onlyPositiveValues; }

	//! Returns the specific NaN value for this scalar field
	inline ScalarType NaN() const { return m_onlyPositiveValues ? HIDDEN_VALUE : NAN_VALUE; };

	//! Computes the mean value (and optionnaly the variance value) of the scalar field
	/** \param mean a field to store the mean value
		\param variance if not void, the variance will be computed and stored here
	**/
	void computeMeanAndVariance(ScalarType &mean, ScalarType* variance=0) const;

	//inherited from GenericChunkedArray
	virtual void computeMinAndMax();

	//! Returns whether a scalar value is valid or not
	static inline bool ValidValue(ScalarType value, bool positiveSF) { return positiveSF ? value >= 0 : value == value; } //both tests fail with NaN values!

	//! Returns whether a scalar value is valid or not (non static shortcut to ValidValue)
	inline bool validValue(ScalarType value) const { return ValidValue(value,m_onlyPositiveValues); }

	//! Sets the value as 'invalid' (i.e. HIDDEN_VALUE or NAN_VALUE depending on whether the SF is positive or not)
	virtual void flagValueAsInvalid(unsigned index) { setValue(index,m_onlyPositiveValues ? HIDDEN_VALUE : NAN_VALUE); }

	//! Helper: returns whether a value is acceptable for a strictly positive scalar field
	/** Value must be either positive or HIDDEN_VALUE and not NaN.
	**/
	static inline bool PositiveSfValue(ScalarType value) { return (value >= 0 || value == HIDDEN_VALUE) && (value != NAN_VALUE); }

protected:

	//! Default destructor
	/** [SHAREABLE] Call 'release' to destroy this object properly.
	**/
	virtual ~ScalarField() {};

	//! Scalar field name
	char m_name[256];

	//! If true, only positive values are considered
	bool m_onlyPositiveValues;
};

}

#endif //CC_SCALAR_FIELD_HEADER
