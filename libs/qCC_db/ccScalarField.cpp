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

#include "ccScalarField.h"

//Local
#include "ccColorScalesManager.h"

//CCLib
#include <CCConst.h>

using namespace CCLib;

ccScalarField::ccScalarField(const char* name/*=0*/)
	: ScalarField(name)
    , m_minDisplayed(0)
    , m_maxDisplayed(0)
    , m_minSaturation(0)
    , m_maxSaturation(0)
    , m_minSaturationLog(0)
    , m_maxSaturationLog(0)
    , m_normalizeCoef(0)
	, m_absSaturation(false)
	, m_logScale(false)
	, m_colorScale(0)
	, m_colorRampSteps(256)
	, m_autoBoundaries(true)
{
	setColorRampSteps(ccColorScale::DEFAULT_STEPS);
	setColorScale(ccColorScalesManager::GetUniqueInstance()->getDefaultScale(ccColorScalesManager::BGYR));
}

ScalarType ccScalarField::normalize(ScalarType d) const
{
	if (!ValidValue(d) || d<m_minDisplayed || d>m_maxDisplayed) //handle NaN values!
		return -1.0;

	if (m_minVal >=0 || !m_absSaturation)
	{
		if (d<=m_minSaturation)
            return 0.0f;
		if (d>=m_maxSaturation)
            return 1.0f;
		if (!m_logScale)
			return (d-m_minSaturation)*m_normalizeCoef;
		return (log10(std::max(d,(ScalarType)ZERO_TOLERANCE))-m_minSaturationLog)*m_normalizeCoef;
	}
	else
	{
		if (d<0)
		{
			if (-d<=m_minSaturation)
				return 0.5f;
			if (-d>=m_maxSaturation)
				return 0.0f;
			if (!m_logScale)
				return 0.5f+(d+m_minSaturation)*m_normalizeCoef*0.5f;
			return 0.5f+(-log10(std::max(-d,(ScalarType)ZERO_TOLERANCE))+m_minSaturationLog)*m_normalizeCoef*0.5f;
		}
		else
		{
			if (d<=m_minSaturation)
				return 0.5f;
			if (d>=m_maxSaturation)
				return 1.0f;
			if (!m_logScale)
				return 0.5f+(d-m_minSaturation)*m_normalizeCoef*0.5f;
			return 0.5f+(log10(std::max(d,(ScalarType)ZERO_TOLERANCE))-m_minSaturationLog)*m_normalizeCoef*0.5f;
		}
	}
}

void ccScalarField::setAbsoluteSaturation(bool state)
{
	if (m_absSaturation != state)
	{
		m_absSaturation = state;
		computeMinAndMax();
	}
}

void ccScalarField::computeMinAndMax()
{
	if (m_autoBoundaries)
		ScalarField::computeMinAndMax();

	m_minDisplayed = m_minVal;
	m_maxDisplayed = m_maxVal;

	//if log scale, we force absolute saturation for not strictly positive SFs!
	if (m_logScale && m_minVal < 0)
		m_absSaturation = true;

	if (m_absSaturation)
	{
		m_minSaturation = (m_minVal < 0 ? std::min(fabs(m_minDisplayed),fabs(m_maxDisplayed)) : 0);
		m_maxSaturation = std::max(fabs(m_minDisplayed),fabs(m_maxDisplayed));
	}
	else
	{
		m_minSaturation = m_minDisplayed;
		m_maxSaturation = m_maxDisplayed;

		assert(m_minVal >= 0 || !m_logScale);
	}

	if (m_logScale || m_minVal >= 0)
	{
		m_minSaturationLog = log10(std::max(m_minSaturation,(ScalarType)ZERO_TOLERANCE));
		m_maxSaturationLog = log10(std::max(m_maxSaturation,(ScalarType)ZERO_TOLERANCE));
	}

	updateNormalizeCoef();
}

void ccScalarField::setLogScale(bool state)
{
	if (m_logScale == state)
		return;

	m_logScale = state;

	if (m_logScale)
	{
		//we force absolute saturation for not strictly positive SFs
		if (m_minVal < 0 && !m_absSaturation)
		{
			m_minSaturation = (m_minVal >= 0 ? std::min(fabs(m_minDisplayed),fabs(m_maxDisplayed)) : 0);
			m_maxSaturation = std::max(fabs(m_minDisplayed),fabs(m_maxDisplayed));
			m_absSaturation = true;
		}

		m_minSaturationLog = log10(std::max(m_minSaturation,(ScalarType)ZERO_TOLERANCE));
		m_maxSaturationLog = log10(std::max(m_maxSaturation,(ScalarType)ZERO_TOLERANCE));
	}

	updateNormalizeCoef();
}

void ccScalarField::updateNormalizeCoef()
{
	m_normalizeCoef = (ScalarType)1.0;

	if (m_minSaturation < m_maxSaturation)
	{
		if (m_logScale)
		{
			assert(m_minSaturationLog < m_maxSaturationLog);
			m_normalizeCoef /= (m_maxSaturationLog-m_minSaturationLog);
		}
		else
		{
			m_normalizeCoef /= (m_maxSaturation-m_minSaturation);
		}
	}
}

void ccScalarField::setMinDisplayed(ScalarType dist)
{
	m_minDisplayed=dist;
	updateNormalizeCoef();
}

void ccScalarField::setMaxDisplayed(ScalarType dist)
{
	m_maxDisplayed=dist;
	updateNormalizeCoef();
}

void ccScalarField::setMinSaturation(ScalarType dist)
{
	m_minSaturation=dist;

	if (m_logScale || m_minVal >= 0)
		m_minSaturationLog = log10(std::max(m_minSaturation,(ScalarType)ZERO_TOLERANCE));

	updateNormalizeCoef();
}

void ccScalarField::setMaxSaturation(ScalarType dist)
{
	m_maxSaturation=dist;

	if (m_logScale || m_minVal >= 0)
		m_maxSaturationLog = log10(std::max(m_maxSaturation,(ScalarType)ZERO_TOLERANCE));

	updateNormalizeCoef();
}

void ccScalarField::setColorRampSteps(unsigned steps)
{
	if (steps > ccColorScale::MAX_STEPS)
		m_colorRampSteps = ccColorScale::MAX_STEPS;
	else if (steps < ccColorScale::MIN_STEPS)
        m_colorRampSteps = ccColorScale::MIN_STEPS;
    else
        m_colorRampSteps = steps;
}

bool ccScalarField::toFile(QFile& out) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));

	//name (dataVersion>=20)
	if (out.write(m_name,256)<0)
		return WriteError();

	//data (dataVersion>=20)
	if (!ccSerializationHelper::GenericArrayToFile(*this,out))
		return WriteError();

	//displayed values & saturation boundaries (dataVersion>=20)
	double dValue = (double)m_minDisplayed;
	if (out.write((const char*)&dValue,sizeof(double))<0)
		return WriteError();
	dValue = (double)m_maxDisplayed;
	if (out.write((const char*)&dValue,sizeof(double))<0)
		return WriteError();
	dValue = (double)m_minSaturation;
	if (out.write((const char*)&dValue,sizeof(double))<0)
		return WriteError();
	dValue = (double)m_maxSaturation;
	if (out.write((const char*)&dValue,sizeof(double))<0)
		return WriteError();
	dValue = (double)m_minSaturationLog;
	if (out.write((const char*)&dValue,sizeof(double))<0)
		return WriteError();
	dValue = (double)m_maxSaturationLog;
	if (out.write((const char*)&dValue,sizeof(double))<0)
		return WriteError();

	//'absolute saturation' state (dataVersion>=20)
	if (out.write((const char*)&m_absSaturation,sizeof(bool))<0)
		return WriteError();

	//'logarithmic scale' state (dataVersion>=20)
	if (out.write((const char*)&m_logScale,sizeof(bool))<0)
		return WriteError();

	//'automatic boundaries update' state (dataVersion>=20)
	if (out.write((const char*)&m_autoBoundaries,sizeof(bool))<0)
		return WriteError();

	//color scale (dataVersion>=27)
	{
		bool hasColorScale = (m_colorScale != 0);
		if (out.write((const char*)&hasColorScale,sizeof(bool))<0)
			return WriteError();

		if (m_colorScale)
			if (!m_colorScale->toFile(out))
				return WriteError();
	}

	//color ramp steps (dataVersion>=20)
	uint32_t colorRampSteps = (uint32_t)m_colorRampSteps;
	if (out.write((const char*)&colorRampSteps,4)<0)
		return WriteError();

	return true;
}

bool ccScalarField::fromFile(QFile& in, short dataVersion)
{
	assert(in.isOpen() && (in.openMode() & QIODevice::ReadOnly));

	if (dataVersion<20)
		return CorruptError();

	//name (dataVersion>=20)
	if (in.read(m_name,256)<0)
		return ReadError();

	//'strictly positive' state (20 <= dataVersion < 26)
	bool onlyPositiveValues = false;
	if (dataVersion < 26)
	{
		if (in.read((char*)&onlyPositiveValues,sizeof(bool))<0)
			return ReadError();
	}

	//data (dataVersion>=20)
	if (!ccSerializationHelper::GenericArrayFromFile(*this,in,dataVersion))
		return false;

	//convert former 'hidden/NaN' values for non strictly positive SFs (dataVersion < 26)
	if (dataVersion < 26)
	{
		const ScalarType FORMER_BIG_VALUE = (ScalarType)(sqrt(3.4e38f)-1.0f);
		const ScalarType FORMER_HIDDEN_VALUE = (ScalarType)-1.0;

		for (unsigned i=0;i<m_maxCount;++i)
		{
			ScalarType val = getValue(i);
			//convert former 'HIDDEN_VALUE' and 'BIG_VALUE' to 'NAN_VALUE'
			if (onlyPositiveValues && val < 0.0 || !onlyPositiveValues && val >= FORMER_BIG_VALUE)
				val = NAN_VALUE;
		}
	}

	//displayed values & saturation boundaries (dataVersion>=20)
	double dValue = 0;
	if (in.read((char*)&dValue,sizeof(double))<0)
		return ReadError();
	m_minDisplayed = (ScalarType)dValue;
	if (in.read((char*)&dValue,sizeof(double))<0)
		return ReadError();
	m_maxDisplayed = (ScalarType)dValue;
	if (in.read((char*)&dValue,sizeof(double))<0)
		return ReadError();
	m_minSaturation = (ScalarType)dValue;
	if (in.read((char*)&dValue,sizeof(double))<0)
		return ReadError();
	m_maxSaturation = (ScalarType)dValue;
	if (in.read((char*)&dValue,sizeof(double))<0)
		return ReadError();
	m_minSaturationLog = (ScalarType)dValue;
	if (in.read((char*)&dValue,sizeof(double))<0)
		return ReadError();
	m_maxSaturationLog = (ScalarType)dValue;

	//'absolute saturation' state (dataVersion>=20)
	if (in.read((char*)&m_absSaturation,sizeof(bool))<0)
		return ReadError();

	//'logarithmic scale' state (dataVersion>=20)
	if (in.read((char*)&m_logScale,sizeof(bool))<0)
		return ReadError();

	//'automatic boundaries update' state (dataVersion>=20)
	if (in.read((char*)&m_autoBoundaries,sizeof(bool))<0)
		return ReadError();

	//color scale
	{
		ccColorScalesManager* colorScalesManager = ccColorScalesManager::GetUniqueInstance();
		if (!colorScalesManager)
		{
			ccLog::Warning("[ccScalarField::fromFile] Failed to access color scales manager?!");
			assert(false);
		}

		//old versions
		if (dataVersion<27)
		{
			uint32_t activeColorScale = 0;
			if (in.read((char*)&activeColorScale,4)<0)
				return ReadError();

			//Retrieve equivalent default scale
			ccColorScalesManager::DEFAULT_SCALE activeColorScaleType = ccColorScalesManager::BGYR;
			switch(activeColorScale)
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
			if (in.read((char*)&hasColorScale,sizeof(bool))<0)
				return ReadError();

			if (hasColorScale)
			{
				ccColorScale::Shared colorScale = ccColorScale::Shared(new ccColorScale());
				if (!colorScale->fromFile(in,dataVersion))
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

		//A scalar fiels must have a color scale!
		if (!m_colorScale)
			m_colorScale = ccColorScalesManager::GetDefaultScale();

		//color ramp steps (dataVersion>=20)
		uint32_t colorRampSteps = 0;
		if (in.read((char*)&colorRampSteps,4)<0)
			return ReadError();
		setColorRampSteps((unsigned)colorRampSteps);
	}

	//Normalisation coef.
	updateNormalizeCoef();

	return true;
}

void ccScalarField::autoUpdateBoundaries(bool state)
{
	bool updateBoundaries = (!m_autoBoundaries && state);
	m_autoBoundaries = state;

	if (updateBoundaries)
		computeMinAndMax();
}

void ccScalarField::setBoundaries(ScalarType minValue, ScalarType maxValue)
{
	autoUpdateBoundaries(false);
	setMin(minValue);
	setMax(maxValue);

	computeMinAndMax();
}
