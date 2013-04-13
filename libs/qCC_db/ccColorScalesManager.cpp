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

#include "ccColorScalesManager.h"

//Local
#include "ccLog.h"

//System
#include <assert.h>

//unique instance
static ccColorScalesManager* s_uniqueInstance = 0;

ccColorScalesManager* ccColorScalesManager::GetUniqueInstance()
{
    if (!s_uniqueInstance)
        s_uniqueInstance = new ccColorScalesManager();

    return s_uniqueInstance;
}

void ccColorScalesManager::ReleaseUniqueInstance()
{
    if (s_uniqueInstance)
        delete s_uniqueInstance;
    s_uniqueInstance=0;
}

ccColorScalesManager::ccColorScalesManager()
{
	//Create default scales
	{
		addScale(Create(BGYR));
		addScale(Create(GREY));
		addScale(Create(BWR));
		addScale(Create(RY));
		addScale(Create(RW));
	}
}

ccColorScalesManager::~ccColorScalesManager()
{
}

ccColorScale::Shared ccColorScalesManager::getScale(QString UUID) const
{
	return m_scales.value(UUID,ccColorScale::Shared(0));
}

void ccColorScalesManager::addScale(ccColorScale::Shared scale)
{
	if (!scale || scale->getUuid().isEmpty())
	{
		ccLog::Error("[ccColorScalesManager::addScale] Invalid scale/UUID!");
		assert(false);
		return;
	}

	m_scales.insert(scale->getUuid(),scale);
}

void ccColorScalesManager::removeScale(QString UUID)
{
	ScalesMap::const_iterator it = m_scales.find(UUID);
	if (it != m_scales.end())
	{
		if ((*it)->isLocked())
		{
			ccLog::Warning(QString("[ccColorScalesManager::addScale] Can't remove a locked scale (%1)!").arg(UUID));
		}
		else
		{
			m_scales.remove(UUID);
		}
	}
}

ccColorScale::Shared ccColorScalesManager::Create(DEFAULT_SCALE scaleType)
{
	QString name;
	switch (scaleType)
	{
	case BGYR:
		name = "Blue>Green>Yellow>Red";
		break;
	case GREY:
		name = "Grey";
		break;
	case BWR:
		name = "Blue>White>Red";
		break;
	case RY:
		name = "Red>Yellow";
		break;
	case RW:
		name = "Red>White";
		break;
	default:
		ccLog::Error(QString("Unhandled pre-defined scale (%1)").arg(scaleType));
		return ccColorScale::Shared(0);
	}

	ccColorScale::Shared scale(new ccColorScale(name, QString::number(scaleType), true)); //all pre-defined scales are relative

	switch (scaleType)
	{
	case BGYR:
		scale->insert(ccColorScaleElement(0.0,Qt::blue),false);
		scale->insert(ccColorScaleElement(1.0/3.0,Qt::green),false);
		scale->insert(ccColorScaleElement(2.0/3.0,Qt::yellow),false);
		scale->insert(ccColorScaleElement(1.0,Qt::red),false);
		break;
	case GREY:
		scale->insert(ccColorScaleElement(0.0,Qt::black),false);
		scale->insert(ccColorScaleElement(1.0,Qt::white),false);
		break;
	case BWR:
		scale->insert(ccColorScaleElement(0.0,Qt::blue),false);
		scale->insert(ccColorScaleElement(0.5,Qt::white),false);
		scale->insert(ccColorScaleElement(1.0,Qt::red),false);
		break;
	case RY:
		scale->insert(ccColorScaleElement(0.0,Qt::red),false);
		scale->insert(ccColorScaleElement(1.0,Qt::yellow),false);
		break;
	case RW:
		scale->insert(ccColorScaleElement(0.0,Qt::red),false);
		scale->insert(ccColorScaleElement(1.0,Qt::white),false);
		break;
	default:
		assert(false);
	}

	//don't forget to update internal representation!
	scale->update();
	scale->setLocked(true);

	return scale;
}
