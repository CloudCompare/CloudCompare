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

#ifndef CC_DB_SCALAR_FIELD_HEADER
#define CC_DB_SCALAR_FIELD_HEADER

//CCLib
#include <ScalarField.h>

//qCC_db
#include "ccSerializableObject.h"
#include "ccColorScale.h"

//System
#include <assert.h>

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
    **/
	ccScalarField(const char* name = 0);

	/*** Scalar values display handling ***/

	//! Range structure
	struct Range
	{
	public:

		//! Default constructor
		Range() : m_min(0), m_start(0), m_stop(0), m_max(0) {}

		//getters
		inline ScalarType min()			const { return m_min;		}
		inline ScalarType start()		const { return m_start;		}
		inline ScalarType stop()		const { return m_stop;		}
		inline ScalarType max()			const { return m_max;		}
		inline ScalarType range()		const { return m_range;		}
		inline ScalarType maxRange()	const { return m_max-m_min; }

		//setters
		void setBounds(ScalarType minVal, ScalarType maxVal, bool resetStartStop = true)
		{
			assert(minVal <=maxVal);
			m_min = minVal; m_max = maxVal;
			if (resetStartStop)
			{
				m_start = m_min;
				m_stop = m_max;
			}
			else
			{
				m_start = inbound(m_start);
				m_stop = inbound(m_stop);
			}
			updateRange();
		}

		inline void setStart(ScalarType value) { m_start = inbound(value); if (m_stop < m_start) m_stop = m_start; updateRange(); }
		inline void setStop(ScalarType value) { m_stop = inbound(value); if (m_stop < m_start) m_start = m_stop; updateRange(); }
		
		//! Returns the nearest inbound value
		inline ScalarType inbound(ScalarType val) const { return (val < m_min ?  m_min : (val > m_max ? m_max : val)); }
		//! Returns whether a value is inbound or not
		inline bool isInbound(ScalarType val) const { return (val >= m_min && val <= m_max); }
		//! Returns whether a value is inside range or not
		inline bool isInRange(ScalarType val) const { return (val >= m_start && val <= m_stop); }

	protected:

		//! Updates actual range
		inline void updateRange() { m_range = std::max(m_stop-m_start,(ScalarType)ZERO_TOLERANCE); }

		ScalarType m_min;		/**< Minimum value **/
		ScalarType m_start;		/**< Current start value (in [min,max]) **/
		ScalarType m_stop;		/**< Current stop value (in [min,max]) **/
		ScalarType m_max;		/**< Minimum value **/
		ScalarType m_range;		/**< Actual range: start-stop (but can't be ZERO!) **/
	};

	//! Access to the range of displayed values
	/** Values outside of the [start;stop] intervale will either be grey
		or invisible (see showNaNValuesInGrey).
	**/
	inline const Range& displayRange() const { return m_displayRange; }

	//! Access to the range of saturation values
	/** Relative color scales will only be applied to values inside the
		[start;stop] intervale.
	**/
	inline const Range& saturationRange() const { return m_logScale ? m_logSaturationRange : m_saturationRange; }

	//! Access to the range of log scale saturation values
	/** Relative color scales will only be applied to values inside the
		[start;stop] intervale.
	**/
	inline const Range& logSaturationRange() const { return m_logSaturationRange; }

	//! Sets the minimum displayed value
	inline void setMinDisplayed(ScalarType val) { m_displayRange.setStart(val); }
	//! Sets the maximum displayed value
	inline void setMaxDisplayed(ScalarType val) { m_displayRange.setStop(val); }
	//! Sets the value at which to start color gradient
	void setSaturationStart(ScalarType val);
	//! Sets the value at which to stop color gradient
	void setSaturationStop(ScalarType val);

	//! Returns the color corresponding to a given value (wrt to the current display parameters)
	/** Warning: must no be called if the SF is not associated to a color scale!
	**/
	inline const colorType* getColor(ScalarType value) const
	{
		assert(m_colorScale);
		return m_colorScale->getColorByRelativePos(normalize(value), m_colorRampSteps, m_showNaNValuesInGrey ? ccColor::lightGrey : 0);
	}

	//! Shortcut to getColor
	inline const colorType* getValueColor(unsigned index) const { return getColor(getValue(index)); }

	//! Sets whether NaN/out of displayed range values should be displayed in grey or hidden
	inline void showNaNValuesInGrey(bool state) { m_showNaNValuesInGrey = state; }

	//! Returns whether NaN values are displayed in grey or hidden
	inline bool areNaNValuesShownInGrey() const { return m_showNaNValuesInGrey; }

	//! Sets whether 0 should always appear in associated color ramp or not
	inline void alwaysShowZero(bool state) { m_alwaysShowZero = state; }

	//! Returns whether 0 should always appear in associated color ramp or not
	inline bool isZeroAlwaysShown() const { return m_alwaysShowZero; }

	//! Sets whether the color scale should be symmetrical or not
	/** For relative color scales only.
	**/
	void setSymmetricalScale(bool state);

	//! Returns whether the color scale s symmetrical or not
	/** For relative color scales only.
	**/
	inline bool symmetricalScale() const { return m_symmetricalScale; }

	//! Sets whether scale is logarithmic or not
	void setLogScale(bool state);

	//! Returns whether scalar field is logarithmic or not
	inline bool logScale() const { return m_logScale; }

	//inherited
	virtual void computeMinAndMax();

	//! Returns associated color scale
	inline const ccColorScale::Shared& getColorScale() const { return m_colorScale; }

	//! Sets associated color scale
	void setColorScale(ccColorScale::Shared scale);

	//! Returns number of color ramp steps
	inline unsigned getColorRampSteps() const { return m_colorRampSteps; }

	//! Sets number of color ramp steps used for display
    void setColorRampSteps(unsigned steps);

	//! Simple histogram structure
	struct Histogram : std::vector<unsigned>
	{
		//! Max histogram value
		unsigned maxValue;

		//! Default constructor
		Histogram() { maxValue = 0; }
	};

	//! Returns associated histogram values (for display)
	const Histogram& getHistogram() const { return m_histogram; }

	//inherited from ccSerializableObject
	virtual bool isSerializable() const { return true; }
	virtual bool toFile(QFile& out) const;
	virtual bool fromFile(QFile& in, short dataVersion, int flags);

protected:

	//! Default destructor
	/** [SHAREABLE] Call 'release' to destroy this object properly.
	**/
	virtual ~ccScalarField() {};

	//! Updates saturation values
	void updateSaturationBounds();

	//! Normalizes a scalar value between 0 and 1 (wrt to current parameters)
	/**	\param val scalar value
		\return a number between 0 and 1 if inside displayed range or -1 otherwise
	**/
	ScalarType normalize(ScalarType val) const;

	//! Displayed values range
	Range m_displayRange;

	//! Saturation values range
	/** For color mapping with relative scales.
	**/
	Range m_saturationRange;

	//! saturation values range (log scale mode)
	/** For log scale color mapping with relative scales.
	**/
	Range m_logSaturationRange;

	// Whether NaN values are shown in grey or are hidden
	bool m_showNaNValuesInGrey;

	//! Whether color scale is symmetrical or not
	/** For relative color scales only.
	**/
	bool m_symmetricalScale;

	//! Whether scale is logarithmic or not
	bool m_logScale;

	//! Whether 0 should always appear in associated color ramp
	bool m_alwaysShowZero;

	//! Active color ramp (for display)
	ccColorScale::Shared m_colorScale;
	
	//! Number of color ramps steps (for display)
	unsigned m_colorRampSteps;

	//! Associated histogram values (for display)
	Histogram m_histogram;
};

#endif //CC_DB_SCALAR_FIELD_HEADER
