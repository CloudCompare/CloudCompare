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
//$Rev:: 1777                                                              $
//$LastChangedDate:: 2011-02-01 17:40:43 +0100 (mar., 01 févr. 2011)      $
//**************************************************************************
//

#ifndef CC_DB_SCALAR_FIELD_HEADER
#define CC_DB_SCALAR_FIELD_HEADER

//CCLib
#include <ScalarField.h>

//qCC_db
#include "ccSerializableObject.h"
#include "ccColorTablesManager.h"

//! A scalar field associated to display-related parameters
/** Extends the CCLib::ScalarField object.
**/

#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"

class QCC_DB_DLL_API ccScalarField : public CCLib::ScalarField, public ccSerializableObject
#else
class ccScalarField : public CCLib::ScalarField, public ccSerializableObject
#endif
{
public:

	//! Default constructor
	/** \param name scalar field name
        \param positive specifies whether negative values should be ignored
    **/
	ccScalarField(const char* name = 0, bool positive = false);

	/*** Scalar values display handling ***/

	//! Returns the minimum displayed value
	inline DistanceType getMinDisplayed() const {return m_minDisplayed;}
	//! Returns the maximum displayed value
	inline DistanceType getMaxDisplayed() const {return m_maxDisplayed;}
	//! Returns the minimum value to start color gradient
	inline DistanceType getMinSaturation() const {return m_minSaturation;}
	//! Returns the maximum value to end color gradient
	inline DistanceType getMaxSaturation() const {return m_maxSaturation;}

	//! Sets the minimum displayed value
	void setMinDisplayed(DistanceType dist);
	//! Sets the maximum displayed value
	void setMaxDisplayed(DistanceType dist);
	//! Sets the minimum value at which to start color gradient
	void setMinSaturation(DistanceType dist);
	//! Sets the maximum value at which to end color gradient
	void setMaxSaturation(DistanceType dist);

	//! Normalizes a scalar value between 0 and 1
	/** This method relies on the values of MinDisplayed, MinSaturation, MaxSaturation
		and MaxDisplayed.
		\param d a scalar value
		\return a number between 0 and 1 if inside [MinDisplayed:MaxDisplayed] or -1 otherwise
	**/
	DistanceType normalize(DistanceType d) const;

	//! Returns a normalized value (see ScalarField::normalize)
	/** This method relies on the values of MinDisplayed, MinSaturation, MaxSaturation
		and MaxDisplayed.
		\param index scalar value index
		\return a number between 0 and 1 if inside [MinDisplayed:MaxDisplayed] or -1 otherwise
	**/
	inline DistanceType getNormalizedValue(unsigned index) const {return normalize(getValue(index));}

	//! Sets whether min and max saturation values are absolute or not
	/** For signed SF only.
	**/
	void setAbsoluteSaturation(bool state);

	//! Returns whether min and max saturation values are absolute or not
	/** For signed SF only.
	**/
	inline bool absoluteSaturation() const {return m_absSaturation;};

	//! Sets whether scale is logarithmic or not
	void setLogScale(bool state);

	//! Returns whether scalar field is logarithmic or not
	inline bool logScale() const {return m_logScale;};

	//! Sets whether to automatically update boundaries (when scalar field may have changed) or keep user defined ones
	/** Automatically calls 'computeMinAndMax' if toggled to false.
	**/
	void autoUpdateBoundaries(bool state);

	//! Whether boundaries are automatically updated or not
	inline bool areBoundariesAutoUpdated() const { return m_autoBoundaries; }

	//! Sets the boundaries
	/** Automatically switches scalar field to 'autoUpdateBoundaries' mode.
	**/
	void setBoundaries(DistanceType minValue, DistanceType maxValue);

	//inherited
	virtual void computeMinAndMax();

	//! Returns associated color ramp
	inline CC_COLOR_RAMPS getColorRamp() const {return m_activeColorRamp;};

	//! Sets associated color ramp
	virtual void setColorRamp(CC_COLOR_RAMPS cr);

	//! Returns number of color ramp steps
	inline unsigned getColorRampSteps() const {return m_colorRampSteps;}

	//! Sets number of color ramp steps
    void setColorRampSteps(unsigned steps);

	//inherited from ccSerializableObject
	virtual bool isSerializable() const { return true; }
	virtual bool toFile(QFile& out) const;
	virtual bool fromFile(QFile& in, short dataVersion);

protected:

	//! Default destructor
	/** [SHAREABLE] Call 'release' to destroy this object properly.
	**/
	virtual ~ccScalarField() {};

	//! Updates normalization coefficient
	void updateNormalizeCoef();

	//! Minimum displayed value
	DistanceType m_minDisplayed;
	//! Maximum displayed value
	DistanceType m_maxDisplayed;
	//! Minimum saturation value (for color mapping)
	DistanceType m_minSaturation;
	//! Maximum saturation value (for color mapping)
	DistanceType m_maxSaturation;
	//! Minimum saturation value (for log scale color mapping)
	DistanceType m_minSaturationLog;
	//! Maximum saturation value (for log scale color mapping)
	DistanceType m_maxSaturationLog;

	//! Normalisation coef.
	DistanceType m_normalizeCoef;

	//! Flag whether min and max saturation values are absolute or not
	/** For signed SF only.
	**/
	bool m_absSaturation;

	//! logarithmic scale
	bool m_logScale;

	//! Active color ramp
	CC_COLOR_RAMPS m_activeColorRamp;
	//! Active color ramp steps
	unsigned m_colorRampSteps;

	//! Whether to automatically update boundaries (when scalar field may have changed) or keep user defined ones
	bool m_autoBoundaries;
};

#endif
