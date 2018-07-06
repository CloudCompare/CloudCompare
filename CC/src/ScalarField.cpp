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

#include <ScalarField.h>

//System
#include <assert.h>
#include <string.h>

using namespace CCLib;

ScalarField::ScalarField(const char* name/*=0*/)
{
	setName(name);
}

ScalarField::ScalarField(const ScalarField& sf)
{
	setName(sf.m_name);
}

void ScalarField::setName(const char* name)
{
	if (name)
		strncpy(m_name, name, 255);
	else
		strcpy(m_name, "Undefined");
}

void ScalarField::computeMeanAndVariance(ScalarType &mean, ScalarType* variance) const
{
	double _mean = 0.0, _std2 = 0.0;
	std::size_t count = 0;

	for (std::size_t i = 0; i < size(); ++i)
	{
		const ScalarType& val = at(i);
		if (ValidValue(val))
		{
			_mean += val;
			_std2 += static_cast<double>(val) * val;
			++count;
		}
	}

	if (count)
	{
		_mean /= count;
		mean = static_cast<ScalarType>(_mean);

		if (variance)
		{
			_std2 = fabs(_std2 / count - _mean*_mean);
			*variance = static_cast<ScalarType>(_std2);
		}
	}
	else
	{
		mean = 0;
		if (variance)
		{
			*variance = 0;
		}
	}
}

void ScalarField::computeMinAndMax()
{
	if (!empty())
	{
		bool minMaxInitialized = false;
		for (std::size_t i = 0; i < size(); ++i)
		{
			const ScalarType& val = at(i);
			if (ValidValue(val))
			{
				if (minMaxInitialized)
				{
					if (val < m_minVal)
						m_minVal = val;
					else if (val > m_maxVal)
						m_maxVal = val;
				}
				else
				{
					//first valid value is used to init min and max
					m_minVal = m_maxVal = val;
					minMaxInitialized = true;
				}
			}
		}
	}
	else //particular case: no value
	{
		m_minVal = m_maxVal = 0;
	}
}

bool ScalarField::reserveSafe(std::size_t count)
{
	try
	{
		reserve(count);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
	return true;
}

bool ScalarField::resizeSafe(std::size_t count, bool initNewElements/*=false*/, ScalarType valueForNewElements/*=0*/)
{
	try
	{
		if (initNewElements)
			resize(count, valueForNewElements);
		else
			resize(count);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
	return true;
}
