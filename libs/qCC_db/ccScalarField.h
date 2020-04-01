//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
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
#include "ccColorScale.h"

//! A scalar field associated to display-related parameters
/** Extends the CCLib::ScalarField object.
**/
class QCC_DB_LIB_API ccScalarField : public CCLib::ScalarField, public ccSerializableObject
{
public:

	//! Default constructor
	/** \param name scalar field name
	**/
	explicit ccScalarField(const char* name = nullptr);

	//! Copy constructor
	/** \param sf scalar field to copy
		\warning May throw a std::bad_alloc exception
	**/
	ccScalarField(const ccScalarField& sf);

	/*** Scalar values display handling ***/

	//! Scalar field range structure
	class QCC_DB_LIB_API Range
	{
	public:

		//! Default constructor
		Range() : m_min(0), m_start(0), m_stop(0), m_max(0), m_range(1) {}

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
		inline void updateRange() { m_range = std::max(m_stop - m_start, static_cast<ScalarType>(ZERO_TOLERANCE)); }

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
	void setMinDisplayed(ScalarType val);
	//! Sets the maximum displayed value
	void setMaxDisplayed(ScalarType val);
	//! Sets the value at which to start color gradient
	void setSaturationStart(ScalarType val);
	//! Sets the value at which to stop color gradient
	void setSaturationStop(ScalarType val);

	//! Returns the color corresponding to a given value (wrt to the current display parameters)
	/** Warning: must no be called if the SF is not associated to a color scale!
	**/
	inline const ccColor::Rgb* getColor(ScalarType value) const
	{
		assert(m_colorScale);
		return m_colorScale->getColorByRelativePos(normalize(value), m_colorRampSteps, m_showNaNValuesInGrey ? &ccColor::lightGreyRGB : nullptr);
	}

	//! Shortcut to getColor
	inline const ccColor::Rgb* getValueColor(unsigned index) const { return getColor(getValue(index)); }

	//! Sets whether NaN/out of displayed range values should be displayed in grey or hidden
	void showNaNValuesInGrey(bool state);

	//! Returns whether NaN values are displayed in grey or hidden
	inline bool areNaNValuesShownInGrey() const { return m_showNaNValuesInGrey; }

	//! Sets whether 0 should always appear in associated color ramp or not
	void alwaysShowZero(bool state);

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
	void computeMinAndMax() override;

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
		unsigned maxValue = 0;
	};

	//! Returns associated histogram values (for display)
	inline const Histogram& getHistogram() const { return m_histogram; }

	//! Returns whether the scalar field in its current configuration MAY have 'hidden' values or not
	/** 'Hidden' values are typically NaN values or values outside of the 'displayed' intervale
		while those values are not displayed in grey (see ccScalarField::showNaNValuesInGrey).
	**/
	bool mayHaveHiddenValues() const;

	//! Sets modification flag state
	inline void setModificationFlag(bool state) { m_modified = state; }
	//! Returns modification flag state
	inline bool getModificationFlag() const { return m_modified; }

	//! Imports the parameters from another scalar field
	void importParametersFrom(const ccScalarField* sf);

	//inherited from ccSerializableObject
	inline bool isSerializable() const override { return true; }
	bool toFile(QFile& out) const override;
	bool fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;

	//! Returns the global shift (if any)
	inline double getGlobalShift() const { return m_globalShift; }
	//! Sets the global shift
	inline void setGlobalShift(double shift) { m_globalShift = shift; }

protected: //methods

	//! Default destructor
	/** Call release instead
	**/
	~ccScalarField() override = default;

	//! Updates saturation values
	void updateSaturationBounds();

	//! Normalizes a scalar value between 0 and 1 (wrt to current parameters)
	/**	\param val scalar value
		\return a number between 0 and 1 if inside displayed range or -1 otherwise
	**/
	ScalarType normalize(ScalarType val) const;

protected: //members

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

	//! Whether NaN values are shown in grey or are hidden
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

	//! Modification flag
	/** Any modification to the scalar field values or parameters
		will turn this flag on.
	**/
	bool m_modified;

	//! Global shift
	double m_globalShift;
};

#endif //CC_DB_SCALAR_FIELD_HEADER
