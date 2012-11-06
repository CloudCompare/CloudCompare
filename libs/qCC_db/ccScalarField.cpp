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
//$Rev:: 1786                                                              $
//$LastChangedDate:: 2011-02-10 17:37:59 +0100 (jeu., 10 févr. 2011)      $
//**************************************************************************
//

#include "ccScalarField.h"

#include <CCConst.h>

using namespace CCLib;

ccScalarField::ccScalarField(const char* name/*=0*/, bool positive /*=false*/)
	: ScalarField(name,positive)
    , m_minDisplayed(0)
    , m_maxDisplayed(0)
    , m_minSaturation(0)
    , m_maxSaturation(0)
    , m_minSaturationLog(0)
    , m_maxSaturationLog(0)
    , m_normalizeCoef(0)
	, m_absSaturation(false)
	, m_logScale(false)
	, m_activeColorRamp(DEFAULT_COLOR_RAMP)
	, m_colorRampSteps(256)
	, m_autoBoundaries(true)
{
	setColorRampSteps(DEFAULT_COLOR_RAMP_SIZE < 256 ? DEFAULT_COLOR_RAMP_SIZE : 256);
}

DistanceType ccScalarField::normalize(DistanceType d) const
{
	if (d<m_minDisplayed || d>m_maxDisplayed)
		return HIDDEN_VALUE;

	if (m_onlyPositiveValues || !m_absSaturation)
	{
		if (d<=m_minSaturation)
            return 0.0f;
		if (d>=m_maxSaturation)
            return 1.0f;
		if(!m_logScale)
			return (d-m_minSaturation)*m_normalizeCoef;
		return (log10(std::max(d,(DistanceType)ZERO_TOLERANCE))-m_minSaturationLog)*m_normalizeCoef;
	}
	else
	{
		if (d<0)
		{
			if (-d<=m_minSaturation)
				return 0.5f;
			if (-d>=m_maxSaturation)
				return 0.0f;
			if(!m_logScale)
				return 0.5f+(d+m_minSaturation)*m_normalizeCoef*0.5f;
			return 0.5f+(-log10(std::max(-d,(DistanceType)ZERO_TOLERANCE))+m_minSaturationLog)*m_normalizeCoef*0.5f;
		}
		else
		{
			if (d<=m_minSaturation)
				return 0.5f;
			if (d>=m_maxSaturation)
				return 1.0f;
			if(!m_logScale)
				return 0.5f+(d-m_minSaturation)*m_normalizeCoef*0.5f;
			return 0.5f+(log10(std::max(d,(DistanceType)ZERO_TOLERANCE))-m_minSaturationLog)*m_normalizeCoef*0.5f;
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
	if (m_logScale && !m_onlyPositiveValues)
		m_absSaturation = true;

	if (m_absSaturation)
	{
		m_minSaturation = (m_onlyPositiveValues ? std::min(fabs(m_minDisplayed),fabs(m_maxDisplayed)) : 0.0f);
		m_maxSaturation = std::max(fabs(m_minDisplayed),fabs(m_maxDisplayed));
	}
	else
	{
		m_minSaturation = m_minDisplayed;
		m_maxSaturation = m_maxDisplayed;

		assert(m_onlyPositiveValues || !m_logScale);
	}

	if (m_logScale || m_onlyPositiveValues)
	{
		m_minSaturationLog = log10(std::max(m_minSaturation,(DistanceType)ZERO_TOLERANCE));
		m_maxSaturationLog = log10(std::max(m_maxSaturation,(DistanceType)ZERO_TOLERANCE));
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
		if (!m_onlyPositiveValues && !m_absSaturation)
		{
			m_minSaturation = (m_onlyPositiveValues ? std::min(fabs(m_minDisplayed),fabs(m_maxDisplayed)) : 0.0f);
			m_maxSaturation = std::max(fabs(m_minDisplayed),fabs(m_maxDisplayed));
			m_absSaturation = true;
		}

		m_minSaturationLog = log10(std::max(m_minSaturation,(DistanceType)ZERO_TOLERANCE));
		m_maxSaturationLog = log10(std::max(m_maxSaturation,(DistanceType)ZERO_TOLERANCE));
	}

	updateNormalizeCoef();
}

void ccScalarField::updateNormalizeCoef()
{
	if (m_minSaturation>=m_maxSaturation)
	{
		m_normalizeCoef = 1.0f;
		return;
	}

	if (m_logScale)
		m_normalizeCoef = 1.0f/(m_maxSaturationLog-m_minSaturationLog);
	else
		m_normalizeCoef = 1.0f/(m_maxSaturation-m_minSaturation);
}

void ccScalarField::setMinDisplayed(DistanceType dist)
{
	m_minDisplayed=dist;
	updateNormalizeCoef();
}

void ccScalarField::setMaxDisplayed(DistanceType dist)
{
	m_maxDisplayed=dist;
	updateNormalizeCoef();
}

void ccScalarField::setMinSaturation(DistanceType dist)
{
	m_minSaturation=dist;

	if (m_logScale || m_onlyPositiveValues)
		m_minSaturationLog = log10(std::max(m_minSaturation,(DistanceType)ZERO_TOLERANCE));

	updateNormalizeCoef();
}

void ccScalarField::setMaxSaturation(DistanceType dist)
{
	m_maxSaturation=dist;

	if (m_logScale || m_onlyPositiveValues)
		m_maxSaturationLog = log10(std::max(m_maxSaturation,(DistanceType)ZERO_TOLERANCE));

	updateNormalizeCoef();
}

void ccScalarField::setColorRamp(CC_COLOR_RAMPS cr)
{
    m_activeColorRamp = cr;
}

void ccScalarField::setColorRampSteps(unsigned steps)
{
    if (steps > (unsigned)DEFAULT_COLOR_RAMP_SIZE)
        m_colorRampSteps = (unsigned)DEFAULT_COLOR_RAMP_SIZE;
    else if (steps < 2)
        m_colorRampSteps = 2;
    else
        m_colorRampSteps = steps;
}

bool ccScalarField::toFile(QFile& out) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));

	//name (dataVersion>=20)
	if (out.write(m_name,256)<0)
		return WriteError();

	//'strictly positive' state (dataVersion>=20)
	if (out.write((const char*)&m_onlyPositiveValues,sizeof(bool))<0)
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

	//active color ramp (dataVersion>=20)
	uint32_t activeColorRamp = (uint32_t)m_activeColorRamp;
	if (out.write((const char*)&activeColorRamp,4)<0)
		return WriteError();

	//active color ramp steps (dataVersion>=20)
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

	//'strictly positive' state (dataVersion>=20)
	if (in.read((char*)&m_onlyPositiveValues,sizeof(bool))<0)
		return ReadError();

	//data (dataVersion>=20)
	if (!ccSerializationHelper::GenericArrayFromFile(*this,in,dataVersion))
		return false;

	//displayed values & saturation boundaries (dataVersion>=20)
	double dValue = 0;
	if (in.read((char*)&dValue,sizeof(double))<0)
		return ReadError();
	m_minDisplayed = (DistanceType)dValue;
	if (in.read((char*)&dValue,sizeof(double))<0)
		return ReadError();
	m_maxDisplayed = (DistanceType)dValue;
	if (in.read((char*)&dValue,sizeof(double))<0)
		return ReadError();
	m_minSaturation = (DistanceType)dValue;
	if (in.read((char*)&dValue,sizeof(double))<0)
		return ReadError();
	m_maxSaturation = (DistanceType)dValue;
	if (in.read((char*)&dValue,sizeof(double))<0)
		return ReadError();
	m_minSaturationLog = (DistanceType)dValue;
	if (in.read((char*)&dValue,sizeof(double))<0)
		return ReadError();
	m_maxSaturationLog = (DistanceType)dValue;

	//'absolute saturation' state (dataVersion>=20)
	if (in.read((char*)&m_absSaturation,sizeof(bool))<0)
		return ReadError();

	//'logarithmic scale' state (dataVersion>=20)
	if (in.read((char*)&m_logScale,sizeof(bool))<0)
		return ReadError();

	//'automatic boundaries update' state (dataVersion>=20)
	if (in.read((char*)&m_autoBoundaries,sizeof(bool))<0)
		return ReadError();

	//active color ramp (dataVersion>=20)
	uint32_t activeColorRamp = 0;
	if (in.read((char*)&activeColorRamp,4)<0)
		return ReadError();
	m_activeColorRamp = (CC_COLOR_RAMPS)activeColorRamp;

	//active color ramp steps (dataVersion>=20)
	uint32_t colorRampSteps = 0;
	if (in.read((char*)&colorRampSteps,4)<0)
		return ReadError();
	m_colorRampSteps = (unsigned)colorRampSteps;

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

void ccScalarField::setBoundaries(DistanceType minValue, DistanceType maxValue)
{
	autoUpdateBoundaries(false);
	setMin(minValue);
	setMax(maxValue);

	computeMinAndMax();
}
