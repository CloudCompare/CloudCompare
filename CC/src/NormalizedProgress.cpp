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

#include "GenericProgressCallback.h"

//system
#include <cassert>

#ifdef USE_QT

//we use Qt for the atomic counter
#include <QAtomicInt>

class AtomicCounter : public QAtomicInt
{
};

#else

//we use a fake QAtomicInt

//! Fake QAtomicInt
/** \warning Not thread safe!
**/
class AtomicCounter
{
public:
	AtomicCounter() : m_value(0) {}
	inline int load() { return m_value; }
	inline void store(int value) { m_value = value; }
	inline int fetchAndAddRelaxed(int add) { int original = m_value; m_value += add; return original; }
	int m_value;
};

#endif

using namespace CCLib;

NormalizedProgress::NormalizedProgress(	GenericProgressCallback* callback,
										unsigned totalSteps,
										unsigned totalPercentage/*=100*/)
	: m_percent(0)
	, m_step(1)
	, m_percentAdd(1.0f)
	, m_counter(new AtomicCounter)
	, progressCallback(callback)
{
	scale(totalSteps, totalPercentage);
}

NormalizedProgress::~NormalizedProgress()
{
	if (m_counter)
	{
		delete m_counter;
	}
}

void NormalizedProgress::scale(	unsigned totalSteps,
								unsigned totalPercentage/*=100*/,
								bool updateCurrentProgress/*=false*/)
{
	if (progressCallback)
	{
		if (totalSteps == 0 || totalPercentage == 0)
		{
			m_step = 1;
			m_percentAdd = 0;
			return;
		}

		if (totalSteps >= 2 * totalPercentage)
		{
			m_step = static_cast<unsigned>(ceil(static_cast<float>(totalSteps) / totalPercentage));
			assert(m_step != 0 && m_step < totalSteps);
			m_percentAdd = static_cast<float>(totalPercentage) / (totalSteps / m_step);
		}
		else
		{
			m_step = 1;
			m_percentAdd = static_cast<float>(totalPercentage) / totalSteps;
		}

		if (updateCurrentProgress)
		{
			m_percent = static_cast<float>(totalPercentage) / totalSteps * static_cast<float>(m_counter->load());
		}
		else
		{
			m_counter->store(0);
		}
	}
}

void NormalizedProgress::reset()
{
	m_percent = 0;
	m_counter->store(0);
	if (progressCallback)
	{
		progressCallback->update(0);
	}
}

bool NormalizedProgress::oneStep()
{
	if (!progressCallback)
	{
		return true;
	}

	unsigned currentCount = static_cast<unsigned>(m_counter->fetchAndAddRelaxed(1)) + 1;
	if ((currentCount % m_step) == 0)
	{
		m_percent += m_percentAdd;
		progressCallback->update(m_percent);
	}

	return !progressCallback->isCancelRequested();
}

bool NormalizedProgress::steps(unsigned n)
{
	if (!progressCallback)
	{
		return true;
	}

	unsigned currentCount = static_cast<unsigned>(m_counter->fetchAndAddRelaxed(n)) + n;
	unsigned d1 = currentCount / m_step;
	unsigned d2 = (currentCount + n) / m_step;

	if (d2 != d1) //thread safe? Well '++int' is a kind of atomic operation ;)
	{
		m_percent += static_cast<float>(d2 - d1) * m_percentAdd;
		progressCallback->update(m_percent);
	}

	return !progressCallback->isCancelRequested();
}
