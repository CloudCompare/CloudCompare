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

#include "ccColorScalesManager.h"

//Local
#include "ccLog.h"
#include "ccSingleton.h"

//Qt
#include <QSettings>

//CCLib
#include <MeshSamplingTools.h>

//System
#include <assert.h>

//unique instance
static ccSingleton<ccColorScalesManager> s_uniqueInstance;

/*** Persistent settings ***/

static const char c_csm_groupName[]				= "ccColorScalesManager";
static const char c_csm_relative[]				= "relative";
static const char c_csm_minVal[]				= "minVal";
static const char c_csm_maxVal[]				= "maxVal";
static const char c_csm_scaleName[]				= "scaleName";
static const char c_csm_stepsList[]				= "steps";
static const char c_csm_stepRelativePos[]		= "value";
static const char c_csm_stepColor[]				= "color";
static const char c_csm_customLabels[]			= "labels";
static const char c_csm_customLabelValue[]		= "value";

ccColorScalesManager* ccColorScalesManager::GetUniqueInstance()
{
	if (!s_uniqueInstance.instance)
	{
		s_uniqueInstance.instance = new ccColorScalesManager();
		//load custom scales from persistent settings
		s_uniqueInstance.instance->fromPersistentSettings();
	}

	return s_uniqueInstance.instance;
}

void ccColorScalesManager::ReleaseUniqueInstance()
{
	s_uniqueInstance.release();
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
		addScale(Create(HSV_360_DEG));
		addScale(Create(VERTEX_QUALITY));
		addScale(Create(DIP_BRYW));
		addScale(Create(DIP_DIR_REPEAT));
	}
}

ccColorScalesManager::~ccColorScalesManager()
{
	m_scales.clear();
}

void ccColorScalesManager::fromPersistentSettings()
{
	QSettings settings;
	settings.beginGroup(c_csm_groupName);

	QStringList scales = settings.childGroups();
	ccLog::Print(QString("[ccColorScalesManager] Found %1 custom scale(s) in persistent settings").arg(scales.size()));

	//read each scale
	for (int j = 0; j < scales.size(); ++j)
	{
		settings.beginGroup(scales[j]);

		QString name = settings.value(c_csm_scaleName, "unknown").toString();
		bool relative = settings.value(c_csm_relative, true).toBool();

		ccColorScale::Shared scale(new ccColorScale(name, scales[j]));
		if (!relative)
		{
			double minVal = settings.value(c_csm_minVal, 0.0).toDouble();
			double maxVal = settings.value(c_csm_maxVal, 1.0).toDouble();
			scale->setAbsolute(minVal, maxVal);
		}

		try
		{
			//steps
			{
				int size = settings.beginReadArray(c_csm_stepsList);
				for (int i = 0; i < size; ++i)
				{
					settings.setArrayIndex(i);
					double relativePos = settings.value(c_csm_stepRelativePos, 0.0).toDouble();
					QRgb rgb = static_cast<QRgb>(settings.value(c_csm_stepColor, 0).toInt());
					QColor color = QColor::fromRgb(rgb);
					scale->insert(ccColorScaleElement(relativePos, color), false);
				}
				settings.endArray();
			}

			//custom labels
			{
				int size = settings.beginReadArray(c_csm_customLabels);
				for (int i = 0; i < size; ++i)
				{
					settings.setArrayIndex(i);
					double label = settings.value(c_csm_customLabelValue, 0.0).toDouble();
					scale->customLabels().insert(label);
				}
				settings.endArray();
			}
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Warning(QString("[ccColorScalesManager] Failed to load scale '%1' (not enough memory)").arg(scale->getName()));
			scale.clear();
		}

		settings.endGroup();

		if (scale)
		{
			scale->update();
			addScale(scale);
		}
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

			settings.setValue(c_csm_scaleName, (*it)->getName());
			settings.setValue(c_csm_relative, (*it)->isRelative());
			if (!(*it)->isRelative())
			{
				double minVal, maxVal;
				(*it)->getAbsoluteBoundaries(minVal, maxVal);
				settings.setValue(c_csm_minVal, minVal);
				settings.setValue(c_csm_maxVal, maxVal);
			}

			settings.beginWriteArray(c_csm_stepsList);
			{
				for (int i = 0; i < (*it)->stepCount(); ++i)
				{
					settings.setArrayIndex(i);
					settings.setValue(c_csm_stepRelativePos, (*it)->step(i).getRelativePos());
					int rgb = static_cast<int>((*it)->step(i).getColor().rgb());
					settings.setValue(c_csm_stepColor, rgb);
				}
			}
			settings.endArray();

			settings.beginWriteArray(c_csm_customLabels);
			{
				int i = 0;
				for (ccColorScale::LabelSet::const_iterator itL = (*it)->customLabels().begin(); itL != (*it)->customLabels().end(); ++itL, ++i)
				{
					settings.setArrayIndex(i);
					settings.setValue(c_csm_customLabelValue, *itL);
				}
			}
			settings.endArray();


			settings.endGroup();
		}
	}

	settings.endGroup();
}

ccColorScale::Shared ccColorScalesManager::getScale(QString UUID) const
{
	return m_scales.value(UUID, ccColorScale::Shared(0));
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
	case HSV_360_DEG:
		name = "HSV angle [0-360]";
		break;
	case VERTEX_QUALITY:
		name = "Vertex types default colors";
		break;
	case DIP_BRYW:
		name = "Dip [0-90]";
		break;
	case DIP_DIR_REPEAT:
		name = "Dip direction (repeat) [0-360]";
		break;
	default:
		ccLog::Error(QString("Unhandled pre-defined scale (%1)").arg(scaleType));
		return ccColorScale::Shared(0);
	}

	ccColorScale::Shared scale(new ccColorScale(name, QString::number(scaleType)));

	switch (scaleType)
	{
	case BGYR:
		scale->insert(ccColorScaleElement(      0.0, Qt::blue  ), false);
		scale->insert(ccColorScaleElement(1.0 / 3.0, Qt::green ), false);
		scale->insert(ccColorScaleElement(2.0 / 3.0, Qt::yellow), false);
		scale->insert(ccColorScaleElement(      1.0, Qt::red   ), false);
		break;
	case GREY:
		scale->insert(ccColorScaleElement(0.0, Qt::black), false);
		scale->insert(ccColorScaleElement(1.0, Qt::white), false);
		break;
	case BWR:
		scale->insert(ccColorScaleElement(0.0, Qt::blue ), false);
		scale->insert(ccColorScaleElement(0.5, Qt::white), false);
		scale->insert(ccColorScaleElement(1.0, Qt::red  ), false);
		break;
	case RY:
		scale->insert(ccColorScaleElement(0.0, Qt::red   ), false);
		scale->insert(ccColorScaleElement(1.0, Qt::yellow), false);
		break;
	case RW:
		scale->insert(ccColorScaleElement(0.0, Qt::red  ), false);
		scale->insert(ccColorScaleElement(1.0, Qt::white), false);
		break;
	case ABS_NORM_GREY:
		scale->insert(ccColorScaleElement(0.0, Qt::black), false);
		scale->insert(ccColorScaleElement(1.0, Qt::white), false);
		scale->setAbsolute(0.0, 1.0);
		scale->customLabels().insert(0);
		scale->customLabels().insert(0.5);
		scale->customLabels().insert(1.0);
		break;
	case HSV_360_DEG:
		scale->insert(ccColorScaleElement(  0.0/360.0, Qt::red    ), false);
		scale->insert(ccColorScaleElement( 60.0/360.0, Qt::yellow ), false);
		scale->insert(ccColorScaleElement(120.0/360.0, Qt::green  ), false);
		scale->insert(ccColorScaleElement(180.0/360.0, Qt::cyan   ), false);
		scale->insert(ccColorScaleElement(240.0/360.0, Qt::blue   ), false);
		scale->insert(ccColorScaleElement(300.0/360.0, Qt::magenta), false);
		scale->insert(ccColorScaleElement(360.0/360.0, Qt::red    ), false);
		scale->setAbsolute(0.0, 360.0);
		scale->customLabels().insert(0);
		scale->customLabels().insert(60);
		scale->customLabels().insert(120);
		scale->customLabels().insert(180);
		scale->customLabels().insert(240);
		scale->customLabels().insert(300);
		break;
	case VERTEX_QUALITY:
		scale->insert(ccColorScaleElement(0.0, Qt::blue), false);
		scale->insert(ccColorScaleElement(0.5, Qt::green), false);
		scale->insert(ccColorScaleElement(1.0, Qt::red), false);
		assert(		CCLib::MeshSamplingTools::VERTEX_NORMAL < CCLib::MeshSamplingTools::VERTEX_BORDER
				&&	CCLib::MeshSamplingTools::VERTEX_BORDER < CCLib::MeshSamplingTools::VERTEX_NON_MANIFOLD );
		scale->setAbsolute(CCLib::MeshSamplingTools::VERTEX_NORMAL, CCLib::MeshSamplingTools::VERTEX_NON_MANIFOLD);
		scale->customLabels().insert(0);
		scale->customLabels().insert(0.5);
		scale->customLabels().insert(1.0);
		break;
	case DIP_BRYW:
		scale->insert(ccColorScaleElement(0.00, qRgb(129,   0,   0)), false);
		scale->insert(ccColorScaleElement(0.33, qRgb(255,  68,   0)), false);
		scale->insert(ccColorScaleElement(0.66, qRgb(255, 255,   0)), false);
		scale->insert(ccColorScaleElement(1.00, qRgb(255, 255, 255)), false);
		scale->setAbsolute(0, 90.0);
		scale->customLabels().insert(0);
		scale->customLabels().insert(30);
		scale->customLabels().insert(60);
		scale->customLabels().insert(90);
		break;
	case DIP_DIR_REPEAT:
		scale->insert(ccColorScaleElement(  0.0/360.0, qRgb(255,   0,   0)), false);
		scale->insert(ccColorScaleElement( 30.0/360.0, qRgb(255, 255,   0)), false);
		scale->insert(ccColorScaleElement( 60.0/360.0, qRgb(  0, 255,   0)), false);
		scale->insert(ccColorScaleElement( 90.0/360.0, qRgb(  0, 255, 255)), false);
		scale->insert(ccColorScaleElement(120.0/360.0, qRgb(  0,   0, 255)), false);
		scale->insert(ccColorScaleElement(150.0/360.0, qRgb(255,   0, 255)), false);
		scale->insert(ccColorScaleElement(180.0/360.0, qRgb(255,   0,   0)), false);
		scale->insert(ccColorScaleElement(210.0/360.0, qRgb(255, 255,   0)), false);
		scale->insert(ccColorScaleElement(240.0/360.0, qRgb(  0, 255,   0)), false);
		scale->insert(ccColorScaleElement(270.0/360.0, qRgb(  0, 255, 255)), false);
		scale->insert(ccColorScaleElement(300.0/360.0, qRgb(  0,   0, 255)), false);
		scale->insert(ccColorScaleElement(330.0/360.0, qRgb(255,   0, 255)), false);
		scale->insert(ccColorScaleElement(360.0/360.0, qRgb(255,   0,   0)), false);
		scale->setAbsolute(0, 360.0);
		scale->customLabels().insert(0);
		scale->customLabels().insert(90);
		scale->customLabels().insert(180);
		scale->customLabels().insert(270);
		break;
	default:
		assert(false);
	}

	//don't forget to update internal representation!
	scale->update();
	scale->setLocked(true);

	return scale;
}
