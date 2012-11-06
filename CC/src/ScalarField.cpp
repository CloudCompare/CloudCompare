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

#include "ScalarField.h"

#include "CCConst.h"
#include "CCMiscTools.h"

#include <assert.h>

using namespace CCLib;

ScalarField::ScalarField(const char* name/*=0*/, bool positive /*=false*/)
	: GenericChunkedArray<1,DistanceType>()
	, m_onlyPositiveValues(positive)
{
	setName(name);
}

void ScalarField::setName(const char* name)
{
	if (name)
		strncpy(m_name,name,255);
	else
		strcpy(m_name,"Undefined");
}

void ScalarField::computeMeanAndVariance(DistanceType &mean, DistanceType* variance) const
{
	double _mean=0.0, _std2=0.0;
	unsigned count=0;

	for (unsigned i=0;i<m_maxCount;++i)
	{
		const DistanceType& d = getValue(i);
		if ((m_onlyPositiveValues && d>=0.0) || (!m_onlyPositiveValues && d<BIG_VALUE))
		{
			_mean += (double)d;
			_std2 += (double)d * (double)d;
			++count;
		}
	}

	if (count)
	{
		_mean /= (double)count;
		mean = (DistanceType)_mean;

		if (variance)
		{
			_std2 = fabs(_std2/(double)count - _mean*_mean);
			*variance = (DistanceType)_std2;
		}
	}
	else
	{
		mean=0.0;
		if (variance)
			*variance=0.0;
	}
}

void ScalarField::computeMinAndMax()
{
	if (m_maxCount!=0)
	{
		bool minMaxInitialized=false;
		for (unsigned i=0;i<m_maxCount;++i)
		{
			const DistanceType& val = getValue(i);
			if ((m_onlyPositiveValues && (val>=0.0))||(!m_onlyPositiveValues && (val<BIG_VALUE)))
			{
				if (minMaxInitialized)
				{
					if (val<m_minVal)
						m_minVal=val;
					else if (val>m_maxVal)
						m_maxVal=val;
				}
				else
				{
					//first valid value is used to init min and max
					m_minVal = m_maxVal = val;
					minMaxInitialized=true;
				}
			}
		}
	}
	else //particular case: no value
	{
		m_minVal = m_maxVal = 0.0;
	}
}
