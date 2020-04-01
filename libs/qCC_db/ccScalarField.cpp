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

#include "ccScalarField.h"

//Local
#include "ccColorScalesManager.h"

//CCLib
#include <CCConst.h>

//system
#include <algorithm>

using namespace CCLib;

//! Default number of classes for associated histogram
const unsigned MAX_HISTOGRAM_SIZE = 512;

ccScalarField::ccScalarField(const char* name/*=0*/)
	: ScalarField(name)
	, m_showNaNValuesInGrey(true)
	, m_symmetricalScale(false)
	, m_logScale(false)
	, m_alwaysShowZero(false)
	, m_colorScale(nullptr)
	, m_colorRampSteps(0)
	, m_modified(true)
	, m_globalShift(0)
{
	setColorRampSteps(ccColorScale::DEFAULT_STEPS);
	setColorScale(ccColorScalesManager::GetUniqueInstance()->getDefaultScale(ccColorScalesManager::BGYR));
}

ccScalarField::ccScalarField(const ccScalarField& sf)
	: ScalarField(sf)
	, m_displayRange(sf.m_displayRange)
	, m_saturationRange(sf.m_saturationRange)
	, m_logSaturationRange(sf.m_logSaturationRange)
	, m_showNaNValuesInGrey(sf.m_showNaNValuesInGrey)
	, m_symmetricalScale(sf.m_symmetricalScale)
	, m_logScale(sf.m_logScale)
	, m_alwaysShowZero(sf.m_alwaysShowZero)
	, m_colorScale(sf.m_colorScale)
	, m_colorRampSteps(sf.m_colorRampSteps)
	, m_histogram(sf.m_histogram)
	, m_modified(sf.m_modified)
	, m_globalShift(sf.m_globalShift)
{
	computeMinAndMax();
}

ScalarType ccScalarField::normalize(ScalarType d) const
{
	if (/*!ValidValue(d) || */!m_displayRange.isInRange(d)) //NaN values are also rejected by 'isInRange'!
	{
		return static_cast<ScalarType>(-1);
	}

	//most probable path first!
	if (!m_logScale)
	{
		if (!m_symmetricalScale)
		{
			if (d <= m_saturationRange.start())
				return 0;
			else if (d >= m_saturationRange.stop())
				return static_cast<ScalarType>(1);
			return (d - m_saturationRange.start()) / m_saturationRange.range();
		}
		else //symmetric scale
		{
			if (fabs(d) <= m_saturationRange.start())
				return static_cast<ScalarType>(0.5);
			
			if (d >= 0)
			{
				if (d >= m_saturationRange.stop())
					return static_cast<ScalarType>(1);
				return (static_cast<ScalarType>(1) + (d - m_saturationRange.start()) / m_saturationRange.range()) / 2;
			}
			else
			{
				if (d <= -m_saturationRange.stop())
					return 0;
				return (static_cast<ScalarType>(1) + (d + m_saturationRange.start()) / m_saturationRange.range()) / 2;
			}
		}
	}
	else //log scale
	{
		ScalarType dLog = log10(std::max(static_cast<ScalarType>(fabs(d)), static_cast<ScalarType>(ZERO_TOLERANCE)));
		if (dLog <= m_logSaturationRange.start())
			return 0;
		else if (dLog >= m_logSaturationRange.stop())
			return static_cast<ScalarType>(1);
		return (dLog - m_logSaturationRange.start()) / m_logSaturationRange.range();
	}

	//can't get here normally!
	assert(false);
	return static_cast<ScalarType>(-1);
}

void ccScalarField::setColorScale(ccColorScale::Shared scale)
{
	if (m_colorScale != scale)
	{
		bool wasAbsolute = (m_colorScale && !m_colorScale->isRelative());
		bool isAbsolute = (scale && !scale->isRelative());

		m_colorScale = scale;

		if (isAbsolute)
			m_symmetricalScale = false;

		if (isAbsolute || wasAbsolute != isAbsolute)
			updateSaturationBounds();

		m_modified = true;
	}
}

void ccScalarField::setSymmetricalScale(bool state)
{
	if (m_symmetricalScale != state)
	{
		m_symmetricalScale = state;
		updateSaturationBounds();

		m_modified = true;
	}
}

void ccScalarField::setLogScale(bool state)
{
	if (m_logScale != state)
	{
		m_logScale = state;
		if (m_logScale && m_minVal < 0)
		{
			ccLog::Warning("[ccScalarField] Scalar field contains negative values! Log scale will only consider absolute values...");
		}

		m_modified = true;
	}
}

void ccScalarField::computeMinAndMax()
{
	ScalarField::computeMinAndMax();

	m_displayRange.setBounds(m_minVal, m_maxVal);

	//update histogram
	{
		if (m_displayRange.maxRange() == 0 || currentSize() == 0)
		{
			//can't build histogram of a flat field
			m_histogram.clear();
		}
		else
		{
			unsigned count = currentSize();
			unsigned numberOfClasses = static_cast<unsigned>(ceil(sqrt(static_cast<double>(count))));
			numberOfClasses = std::max<unsigned>(std::min<unsigned>(numberOfClasses, MAX_HISTOGRAM_SIZE), 4);

			m_histogram.maxValue = 0;

			//reserve memory
			try
			{
				m_histogram.resize(numberOfClasses);
			}
			catch (const std::bad_alloc&)
			{
				ccLog::Warning("[ccScalarField::computeMinAndMax] Failed to update associated histogram!");
				m_histogram.clear();
			}

			if (!m_histogram.empty())
			{
				std::fill(m_histogram.begin(), m_histogram.end(), 0);

				//compute histogram
				{
					ScalarType step = static_cast<ScalarType>(numberOfClasses) / m_displayRange.maxRange();
					for (unsigned i = 0; i < count; ++i)
					{
						const ScalarType& val = getValue(i);

						unsigned bin = static_cast<unsigned>(floor((val - m_displayRange.min())*step));
						++m_histogram[std::min(bin, numberOfClasses - 1)];
					}
				}

				//update 'maxValue'
				m_histogram.maxValue = *std::max_element(m_histogram.begin(), m_histogram.end());
			}
		}
	}

	m_modified = true;

	updateSaturationBounds();
}

void ccScalarField::updateSaturationBounds()
{
	if (!m_colorScale || m_colorScale->isRelative()) //Relative scale (default)
	{
		ScalarType minAbsVal = (m_maxVal < 0 ? std::min(-m_maxVal, -m_minVal) : std::max<ScalarType>(m_minVal, 0));
		ScalarType maxAbsVal = std::max(fabs(m_minVal), fabs(m_maxVal));

		if (m_symmetricalScale)
		{
			m_saturationRange.setBounds(minAbsVal,maxAbsVal);
		}
		else
		{
			m_saturationRange.setBounds(m_minVal,m_maxVal);
		}

		//log scale (we always update it even if m_logScale is not enabled!)
		//if (m_logScale)
		{
			ScalarType minSatLog = log10(std::max(minAbsVal, static_cast<ScalarType>(ZERO_TOLERANCE)));
			ScalarType maxSatLog = log10(std::max(maxAbsVal, static_cast<ScalarType>(ZERO_TOLERANCE)));
			m_logSaturationRange.setBounds(minSatLog, maxSatLog);
		}
	}
	else //absolute scale
	{
		//DGM: same formulas as for the 'relative scale' case but we use the boundaries
		//defined by the scale itself instead of the current SF boundaries...
		double minVal = 0;
		double maxVal = 0;
		m_colorScale->getAbsoluteBoundaries(minVal, maxVal);

		m_saturationRange.setBounds(static_cast<ScalarType>(minVal), static_cast<ScalarType>(maxVal));

		//log scale (we always update it even if m_logScale is not enabled!)
		//if (m_logScale)
		{
			ScalarType minAbsVal = static_cast<ScalarType>(maxVal < 0 ? std::min(-maxVal, -minVal) : std::max(minVal, 0.0));
			ScalarType maxAbsVal = static_cast<ScalarType>(std::max(fabs(minVal), fabs(maxVal)));
			ScalarType minSatLog = log10(std::max(minAbsVal, static_cast<ScalarType>(ZERO_TOLERANCE)));
			ScalarType maxSatLog = log10(std::max(maxAbsVal, static_cast<ScalarType>(ZERO_TOLERANCE)));
			m_logSaturationRange.setBounds(minSatLog, maxSatLog);
		}
	}

	m_modified = true;
}

void ccScalarField::setMinDisplayed(ScalarType val)
{
	m_displayRange.setStart(val);
	m_modified = true;
}
	
void ccScalarField::setMaxDisplayed(ScalarType val)
{
	m_displayRange.setStop(val);
	m_modified = true;
}

void ccScalarField::setSaturationStart(ScalarType val)
{
	if (m_logScale)
	{
		m_logSaturationRange.setStart(val/*log10(std::max(val,(ScalarType)ZERO_TOLERANCE))*/);
	}
	else
	{
		m_saturationRange.setStart(val);
	}
	m_modified = true;
}

void ccScalarField::setSaturationStop(ScalarType val)
{
	if (m_logScale)
	{
		m_logSaturationRange.setStop(val/*log10(std::max(val,(ScalarType)ZERO_TOLERANCE))*/);
	}
	else
	{
		m_saturationRange.setStop(val);
	}
	m_modified = true;
}

void ccScalarField::setColorRampSteps(unsigned steps)
{
	if (steps > ccColorScale::MAX_STEPS)
		m_colorRampSteps = ccColorScale::MAX_STEPS;
	else if (steps < ccColorScale::MIN_STEPS)
		m_colorRampSteps = ccColorScale::MIN_STEPS;
	else
		m_colorRampSteps = steps;

	m_modified = true;
}

bool ccScalarField::toFile(QFile& out) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));

	//name (dataVersion>=20)
	if (out.write(m_name,256) < 0)
		return WriteError();

	//data (dataVersion>=20)
	if (!ccSerializationHelper::GenericArrayToFile<ScalarType, 1, ScalarType>(*this, out))
		return WriteError();

	//displayed values & saturation boundaries (dataVersion>=20)
	double dValue = (double)m_displayRange.start();
	if (out.write((const char*)&dValue, sizeof(double)) < 0)
		return WriteError();
	dValue = (double)m_displayRange.stop();
	if (out.write((const char*)&dValue, sizeof(double)) < 0)
		return WriteError();
	dValue = (double)m_saturationRange.start();
	if (out.write((const char*)&dValue, sizeof(double)) < 0)
		return WriteError();
	dValue = (double)m_saturationRange.stop();
	if (out.write((const char*)&dValue, sizeof(double)) < 0)
		return WriteError();
	dValue = (double)m_logSaturationRange.start();
	if (out.write((const char*)&dValue, sizeof(double)) < 0)
		return WriteError();
	dValue = (double)m_logSaturationRange.stop();
	if (out.write((const char*)&dValue, sizeof(double)) < 0)
		return WriteError();

	//'logarithmic scale' state (dataVersion>=20)
	if (out.write((const char*)&m_logScale, sizeof(bool)) < 0)
		return WriteError();

	//'symmetrical scale' state (dataVersion>=27)
	if (out.write((const char*)&m_symmetricalScale, sizeof(bool)) < 0)
		return WriteError();

	//'NaN values in grey' state (dataVersion>=27)
	if (out.write((const char*)&m_showNaNValuesInGrey, sizeof(bool)) < 0)
		return WriteError();

	//'always show 0' state (dataVersion>=27)
	if (out.write((const char*)&m_alwaysShowZero, sizeof(bool)) < 0)
		return WriteError();

	//color scale (dataVersion>=27)
	{
		bool hasColorScale = (m_colorScale != nullptr);
		if (out.write((const char*)&hasColorScale, sizeof(bool)) < 0)
			return WriteError();

		if (m_colorScale)
			if (!m_colorScale->toFile(out))
				return WriteError();
	}

	//color ramp steps (dataVersion>=20)
	uint32_t colorRampSteps = (uint32_t)m_colorRampSteps;
	if (out.write((const char*)&colorRampSteps, 4) < 0)
		return WriteError();

	//global shift (dataVersion>=42)
	if (out.write((const char*)&m_globalShift, sizeof(double)) < 0)
		return WriteError();

	return true;
}

bool ccScalarField::fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	assert(in.isOpen() && (in.openMode() & QIODevice::ReadOnly));

	if (dataVersion < 20)
		return CorruptError();

	//name (dataVersion >= 20)
	if (in.read(m_name, 256) < 0)
		return ReadError();

	//'strictly positive' state (20 <= dataVersion < 26)
	bool onlyPositiveValues = false;
	if (dataVersion < 26)
	{
		if (in.read((char*)&onlyPositiveValues, sizeof(bool)) < 0)
			return ReadError();
	}

	//data (dataVersion >= 20)
	bool result = false;
	{
		bool fileScalarIsFloat = (flags & ccSerializableObject::DF_SCALAR_VAL_32_BITS);
		if (fileScalarIsFloat && sizeof(ScalarType) == 8) //file is 'float' and current type is 'double'
		{
			result = ccSerializationHelper::GenericArrayFromTypedFile<ScalarType, 1, ScalarType, float>(*this, in, dataVersion);
		}
		else if (!fileScalarIsFloat && sizeof(ScalarType) == 4) //file is 'double' and current type is 'float'
		{
			result = ccSerializationHelper::GenericArrayFromTypedFile<ScalarType, 1, ScalarType, double>(*this, in, dataVersion);
		}
		else
		{
			result = ccSerializationHelper::GenericArrayFromFile<ScalarType, 1, ScalarType>(*this, in, dataVersion);
		}
	}
	if (!result)
	{
		return false;
	}

	//convert former 'hidden/NaN' values for non strictly positive SFs (dataVersion < 26)
	if (dataVersion < 26)
	{
		const ScalarType FORMER_BIG_VALUE = static_cast<ScalarType>(sqrt(3.4e38f) - 1.0f);

		for (unsigned i = 0; i < currentSize(); ++i)
		{
			ScalarType &val = getValue(i);
			//convert former 'HIDDEN_VALUE' and 'BIG_VALUE' to 'NAN_VALUE'
			if ((onlyPositiveValues && val < 0) || (!onlyPositiveValues && val >= FORMER_BIG_VALUE))
			{
				val = NAN_VALUE;
			}
		}
	}

	computeMinAndMax();

	//displayed values & saturation boundaries (dataVersion>=20)
	double minDisplayed = 0;
	if (in.read((char*)&minDisplayed, sizeof(double)) < 0)
		return ReadError();
	double maxDisplayed = 0;
	if (in.read((char*)&maxDisplayed, sizeof(double)) < 0)
		return ReadError();
	double minSaturation = 0;
	if (in.read((char*)&minSaturation, sizeof(double)) < 0)
		return ReadError();
	double maxSaturation = 0;
	if (in.read((char*)&maxSaturation, sizeof(double)) < 0)
		return ReadError();
	double minLogSaturation = 0;
	if (in.read((char*)&minLogSaturation, sizeof(double)) < 0)
		return ReadError();
	double maxLogSaturation = 0;
	if (in.read((char*)&maxLogSaturation, sizeof(double)) < 0)
		return ReadError();

	if (dataVersion < 27)
	{
		//'absolute saturation' state (27>dataVersion>=20)
		bool absSaturation = false;
		if (in.read((char*)&absSaturation, sizeof(bool)) < 0)
			return ReadError();
		//quite equivalent to 'symmetrical mode' now...
		m_symmetricalScale = absSaturation;
	}

	//'logarithmic scale' state (dataVersion>=20)
	if (in.read((char*)&m_logScale, sizeof(bool)) < 0)
	{
		return ReadError();
	}

	if (dataVersion < 27)
	{
		bool autoBoundaries = false;
		//'automatic boundaries update' state (dataVersion>=20)
		if (in.read((char*)&autoBoundaries, sizeof(bool)) < 0)
		{
			return ReadError();
		}
		//warn the user that this option is deprecated
		if (!autoBoundaries)
		{
			ccLog::Warning("[ccScalarField] Former 'released' boundaries are deprecated!");
			ccLog::Warning("[ccScalarField] You'll have to create the corresponding 'absolute' color scale (see the Color Scale Manager) and replace the file.");
		}
	}

	//new attributes
	if (dataVersion >= 27)
	{
		//'symmetrical scale' state (27<=dataVersion)
		if (in.read((char*)&m_symmetricalScale, sizeof(bool)) < 0)
			return ReadError();

		//'NaN values in grey' state (dataVersion>=27)
		if (in.read((char*)&m_showNaNValuesInGrey, sizeof(bool)) < 0)
			return ReadError();

		//'always show 0' state (27<=dataVersion)
		if (in.read((char*)&m_alwaysShowZero, sizeof(bool)) < 0)
			return ReadError();
	}

	//color scale
	{
		ccColorScalesManager* colorScalesManager = ccColorScalesManager::GetUniqueInstance();
		if (!colorScalesManager)
		{
			ccLog::Warning("[ccScalarField::fromFile] Failed to access color scales manager?!");
			assert(false);
		}

		//old versions
		if (dataVersion < 27)
		{
			uint32_t activeColorScale = 0;
			if (in.read((char*)&activeColorScale, 4) < 0)
				return ReadError();

			//Retrieve equivalent default scale
			ccColorScalesManager::DEFAULT_SCALES activeColorScaleType = ccColorScalesManager::BGYR;
			switch (activeColorScale)
			{
			case ccColorScalesManager::BGYR:
				activeColorScaleType = ccColorScalesManager::BGYR;
				break;
			case ccColorScalesManager::GREY:
				activeColorScaleType = ccColorScalesManager::GREY;
				break;
			case ccColorScalesManager::BWR:
				activeColorScaleType = ccColorScalesManager::BWR;
				break;
			case ccColorScalesManager::RY:
				activeColorScaleType = ccColorScalesManager::RY;
				break;
			case ccColorScalesManager::RW:
				activeColorScaleType = ccColorScalesManager::RW;
				break;
			default:
				ccLog::Warning("[ccScalarField::fromFile] Color scale is no more supported!");
				break;
			}
			m_colorScale = ccColorScalesManager::GetDefaultScale(activeColorScaleType);
		}
		else //(dataVersion>=27)
		{
			bool hasColorScale = false;
			if (in.read((char*)&hasColorScale, sizeof(bool)) < 0)
				return ReadError();

			if (hasColorScale)
			{
				ccColorScale::Shared colorScale = ccColorScale::Create("temp");
				if (!colorScale->fromFile(in, dataVersion, flags, oldToNewIDMap))
					return ReadError();
				m_colorScale = colorScale;

				if (colorScalesManager)
				{
					ccColorScale::Shared existingColorScale = colorScalesManager->getScale(colorScale->getUuid());
					if (!existingColorScale)
					{
						colorScalesManager->addScale(colorScale);
					}
					else //same UUID?
					{
						//FIXME: we should look if the color scale is exactly the same!
						m_colorScale = existingColorScale;
					}
				}
			}
		}

		//A scalar field must have a color scale!
		if (!m_colorScale)
			m_colorScale = ccColorScalesManager::GetDefaultScale();

		//color ramp steps (dataVersion>=20)
		uint32_t colorRampSteps = 0;
		if (in.read((char*)&colorRampSteps, 4) < 0)
			return ReadError();
		setColorRampSteps(static_cast<unsigned>(colorRampSteps));
	}

	if (dataVersion >= 42)
	{
		//global shift (dataVersion>=42)
		if (in.read((char*)&m_globalShift, sizeof(double)) < 0)
			return ReadError();
	}

	//update values
	computeMinAndMax();
	m_displayRange.setStart((ScalarType)minDisplayed);
	m_displayRange.setStop((ScalarType)maxDisplayed);
	m_saturationRange.setStart((ScalarType)minSaturation);
	m_saturationRange.setStop((ScalarType)maxSaturation);
	m_logSaturationRange.setStart((ScalarType)minLogSaturation);
	m_logSaturationRange.setStop((ScalarType)maxLogSaturation);

	m_modified = true;

	return true;
}

bool ccScalarField::mayHaveHiddenValues() const
{
	bool hiddenPoints = (		!areNaNValuesShownInGrey()
						&&	(	m_displayRange.stop()	<= m_displayRange.max()
							||	m_displayRange.start()	>= m_displayRange.min() )
						);

	return hiddenPoints;
}

void ccScalarField::showNaNValuesInGrey(bool state)
{
	m_showNaNValuesInGrey = state;
	m_modified = true;
}

void ccScalarField::alwaysShowZero(bool state)
{
	m_alwaysShowZero = state;
	m_modified = true;
}

void ccScalarField::importParametersFrom(const ccScalarField* sf)
{
	if (!sf)
	{
		assert(false);
		return;
	}

	setColorRampSteps(sf->getColorRampSteps());
	setColorScale(sf->getColorScale());
	showNaNValuesInGrey(sf->areNaNValuesShownInGrey());
	setLogScale(sf->logScale());
	setSymmetricalScale(sf->symmetricalScale());
	alwaysShowZero(sf->isZeroAlwaysShown());
	setMinDisplayed(sf->displayRange().start());
	setMaxDisplayed(sf->displayRange().stop());
	setSaturationStart(sf->saturationRange().start());
	setSaturationStop(sf->saturationRange().stop());
}
