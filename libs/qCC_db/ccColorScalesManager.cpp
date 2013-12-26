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

//Qt
#include <QSettings>


//System
#include <assert.h>

//unique instance
static ccColorScalesManager* s_uniqueInstance = 0;

/*** Persistent settings ***/

static const char c_csm_groupName[]				= "ccColorScalesManager";
static const char c_csm_relative[]				= "relative";
static const char c_csm_minVal[]				= "minVal";
static const char c_csm_maxVal[]				= "maxVal";
static const char c_csm_scaleName[]				= "scaleName";
static const char c_csm_stepsList[]				= "steps";
static const char c_csm_stepRelativePos[]		= "value";
static const char c_csm_stepColor[]				= "color";

ccColorScalesManager* ccColorScalesManager::GetUniqueInstance()
{
    if (!s_uniqueInstance)
	{
        s_uniqueInstance = new ccColorScalesManager();
		//load custom scales from persistent settings
		s_uniqueInstance->fromPersistentSettings();
	}

    return s_uniqueInstance;
}

void ccColorScalesManager::ReleaseUniqueInstance()
{
    if (s_uniqueInstance)
	{
		s_uniqueInstance->toPersistentSettings();
        delete s_uniqueInstance;
	}
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
		addScale(Create(ABS_NORM_GREY));
	}
}

ccColorScalesManager::~ccColorScalesManager()
{
}

void ccColorScalesManager::fromPersistentSettings()
{
	QSettings settings;
	settings.beginGroup(c_csm_groupName);

	QStringList scales = settings.childGroups();
	ccLog::Print(QString("[ccColorScalesManager] Found %1 custom scale(s) in persistent settings").arg(scales.size()));

	//read each scale
	for (int j=0; j<scales.size(); ++j)
	{
		settings.beginGroup(scales[j]);

		QString name = settings.value(c_csm_scaleName,"unknown").toString();
		bool relative = settings.value(c_csm_relative,true).toBool();

		ccColorScale::Shared scale(new ccColorScale(name,scales[j]));
		if (!relative)
		{
			double minVal = settings.value(c_csm_minVal,0.0).toDouble();
			double maxVal = settings.value(c_csm_maxVal,1.0).toDouble();
			scale->setAbsolute(minVal,maxVal);
		}
		
		int size = settings.beginReadArray(c_csm_stepsList);
		for (int i=0; i<size;++i)
		{
			settings.setArrayIndex(i);
			double relativePos = settings.value(c_csm_stepRelativePos, 0.0).toDouble();
			QRgb rgb = static_cast<QRgb>(settings.value(c_csm_stepColor, 0).toInt());
			QColor color = QColor::fromRgb(rgb);
			scale->insert(ccColorScaleElement(relativePos,color),false);
		}
		settings.endArray();

		settings.endGroup();

		scale->update();
		addScale(scale);
	}

	settings.endGroup();
}

void ccColorScalesManager::toPersistentSettings() const
{
	QSettings settings;
	//remove all existing info
	settings.remove(c_csm_groupName);
	//create new set
	settings.beginGroup(c_csm_groupName);

	//add each scale
	for (ScalesMap::const_iterator it = m_scales.begin(); it != m_scales.end(); ++it)
	{
		if (!(*it)->isLocked()) //locked scales are pre-defined ones!
		{
			settings.beginGroup((*it)->getUuid());

			settings.setValue(c_csm_scaleName,(*it)->getName());
			settings.setValue(c_csm_relative,(*it)->isRelative());
			if (!(*it)->isRelative())
			{
				double minVal,maxVal;
				(*it)->getAbsoluteBoundaries(minVal,maxVal);
				settings.setValue(c_csm_minVal,minVal);
				settings.setValue(c_csm_maxVal,maxVal);
			}

			settings.beginWriteArray(c_csm_stepsList);
			for (int i=0; i<(*it)->stepCount();++i)
			{
				 settings.setArrayIndex(i);
				 settings.setValue(c_csm_stepRelativePos, (*it)->step(i).getRelativePos());
				 int rgb = static_cast<int>((*it)->step(i).getColor().rgb());
				 settings.setValue(c_csm_stepColor, rgb);
			}
			settings.endArray();

			settings.endGroup();
		}
	}

	settings.endGroup();
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

ccColorScale::Shared ccColorScalesManager::Create(DEFAULT_SCALES scaleType)
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
	case ABS_NORM_GREY:
		name = "Intensity [0-1]";
		break;
	default:
		ccLog::Error(QString("Unhandled pre-defined scale (%1)").arg(scaleType));
		return ccColorScale::Shared(0);
	}

	ccColorScale::Shared scale(new ccColorScale(name, QString::number(scaleType)));

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
	case ABS_NORM_GREY:
		scale->insert(ccColorScaleElement(0.0,Qt::black),false);
		scale->insert(ccColorScaleElement(1.0,Qt::white),false);
		scale->setAbsolute(0.0,1.0);
		break;
	default:
		assert(false);
	}

	//don't forget to update internal representation!
	scale->update();
	scale->setLocked(true);

	return scale;
}
