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

#ifndef CC_SCALAR_FIELD_HEADER
#define CC_SCALAR_FIELD_HEADER

#include "CCTypes.h"
#include "GenericChunkedArray.h"

namespace CCLib
{

//! A simple scalar field (to be associated to a point cloud)
/** Extends the GenericChunkedArray object. It is equivalent to a
	mono-dimensionnal array of scalar values. It has also specific
	parameters for display purposes.

    There is two kinds of scalar fields :
    - 'positive' scalar fields ignore all negative values. Those values
        are considered as non-scalar values (typically corresponding to
        'hidden' points in a point cloud). They are taken into account
        while computing min and max values, mean, std. dev., etc.).
        Some particular negative values are recognized as specific markers
        (see HIDDEN_VALUE, OUT_VALUE and SEGMENTED_VALUE). This kind of
        scalar field is used to store positive distances for instance.
    - standard scalar fields can store any value strictly below BIG_VALUE
        (which is in this case the unique specific value used to tag non
        scalar values).

**/

#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"

class CC_DLL_API ScalarField : public GenericChunkedArray<1,DistanceType>
#else
class ScalarField : public GenericChunkedArray<1,DistanceType>
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
		\bool state positive state
	**/
	inline void setPositive(bool state) { m_onlyPositiveValues = state; }

	//! Returns true if negative values are ignored (strictly positive scalar field)
	inline bool isPositive() const { return m_onlyPositiveValues; }

	//! Computes the mean value (and optionnaly the variance value) of the scalar field
	/** \param mean a field to store the mean value
		\param variance if not void, the variance will be computed and stored here
	**/
	void computeMeanAndVariance(DistanceType &mean, DistanceType* variance=0) const;

	//inherited from GenericChunkedArray
	virtual void computeMinAndMax();

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

#endif
