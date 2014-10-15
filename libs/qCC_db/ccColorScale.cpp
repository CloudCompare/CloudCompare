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

#include "ccColorScale.h"

//Qt
#include <QUuid>

//Local
#include "ccLog.h"

ccColorScale::Shared ccColorScale::Create(QString name)
{
	return ccColorScale::Shared(new ccColorScale(name));
}

ccColorScale::ccColorScale(QString name, QString uuid/*=QString()*/)
	: m_name(name)
	, m_uuid(uuid)
	, m_updated(false)
	, m_relative(true)
	, m_locked(false)
	, m_absoluteMinValue(0.0)
	, m_absoluteRange(1.0)
{
	if (m_uuid.isNull())
		generateNewUuid();
}

void ccColorScale::generateNewUuid()
{
	m_uuid = QUuid::createUuid().toString();
}

ccColorScale::~ccColorScale()
{
}

void ccColorScale::insert(const ccColorScaleElement& step, bool autoUpdate/*=true*/)
{
	if (m_locked)
	{
		ccLog::Warning(QString("[ccColorScale::insert] Scale '%1' is locked!").arg(m_name));
		return;
	}

	m_steps.push_back(step);

	m_updated = false;

	if (autoUpdate && m_steps.size() >= (int)MIN_STEPS)
		update();
}

void ccColorScale::clear()
{
	if (m_locked)
	{
		ccLog::Warning(QString("[ccColorScale::clear] Scale '%1' is locked!").arg(m_name));
		return;
	}

	m_steps.clear();

	m_updated = false;
}

void ccColorScale::remove(int index, bool autoUpdate/*=true*/)
{
	if (m_locked)
	{
		ccLog::Warning(QString("[ccColorScale::remove] Scale '%1' is locked!").arg(m_name));
		return;
	}

	m_steps.removeAt(index);
	m_updated = false;

	if (autoUpdate)
		update();
}

void ccColorScale::sort()
{
	qSort(m_steps.begin(), m_steps.end(), ccColorScaleElement::IsSmaller);
}

void ccColorScale::update()
{
	if (m_steps.size() >= (int)MIN_STEPS)
	{
		sort();

		unsigned stepCount = static_cast<unsigned>(m_steps.size());
		assert(stepCount>=2);
		assert(m_steps.front().getRelativePos() == 0.0);
		assert(m_steps.back().getRelativePos() == 1.0);
		if (m_steps.front().getRelativePos() != 0.0 || m_steps.back().getRelativePos() != 1.0)
		{
			//invalid scale: paint it black ;)
			memset(m_rgbaScale,0,sizeof(colorType)*4*MAX_STEPS);
			ccLog::Warning(QString("[ccColorScale] Scale '%1' is invalid! (boundaries are not [0.0-1.0]").arg(getName()));
			return;
		}
		
		colorType* _scale = m_rgbaScale;

		unsigned j = 0; //current intervale
		for (unsigned i=0; i<MAX_STEPS; ++i)
		{
			double relativePos = (double)i/(double)(MAX_STEPS-1);

			//forward to the right intervale
			while (j+2 < stepCount && m_steps[j+1].getRelativePos() < relativePos)
				++j;

			// linear interpolation
			CCVector3d colBefore (	m_steps[j].getColor().redF(),
									m_steps[j].getColor().greenF(),
									m_steps[j].getColor().blueF() );
			CCVector3d colNext (	m_steps[j+1].getColor().redF(),
									m_steps[j+1].getColor().greenF(),
									m_steps[j+1].getColor().blueF() );

			//interpolation coef
			double alpha = (relativePos - m_steps[j].getRelativePos())/(m_steps[j+1].getRelativePos() - m_steps[j].getRelativePos());

			CCVector3d interpCol = colBefore + (colNext-colBefore) * alpha;

			*_scale++ = static_cast<colorType>(interpCol.x * (double)MAX_COLOR_COMP);
			*_scale++ = static_cast<colorType>(interpCol.y * (double)MAX_COLOR_COMP);
			*_scale++ = static_cast<colorType>(interpCol.z * (double)MAX_COLOR_COMP);
			*_scale++ = MAX_COLOR_COMP; //do not dream: no transparency ;)
		}

		m_updated = true;
	}
	else //invalid scale: paint it black ;)
	{
		ccLog::Warning(QString("[ccColorScale] Scale '%1' is invalid! (not enough elements)").arg(getName()));
		memset(m_rgbaScale,0,sizeof(colorType)*4*MAX_STEPS);
	}
}

bool ccColorScale::toFile(QFile& out) const
{
	QDataStream outStream(&out);

	//name (dataVersion>=27)
	outStream << m_name;

	//UUID (dataVersion>=27)
	outStream << m_uuid;

	//relative state (dataVersion>=27)
	if (out.write((const char*)&m_relative,sizeof(bool)) < 0)
		return WriteError();

	//Absolute min value (dataVersion>=27)
	if (out.write((const char*)&m_absoluteMinValue,sizeof(double)) < 0)
		return WriteError();
	//Absolute range (dataVersion>=27)
	if (out.write((const char*)&m_absoluteRange,sizeof(double)) < 0)
		return WriteError();

	//locked state (dataVersion>=27)
	if (out.write((const char*)&m_locked,sizeof(bool)) < 0)
		return WriteError();

	//steps list (dataVersion>=27)
	{
		//steps count
		uint32_t stepCount = (uint32_t)m_steps.size();
		if (out.write((const char*)&stepCount,4) < 0)
			return WriteError();

		//write each step
		for (uint32_t i=0; i<stepCount; ++i)
		{
			outStream << m_steps[i].getRelativePos();
			outStream << m_steps[i].getColor();
		}
	}

	return true;
}

bool ccColorScale::fromFile(QFile& in, short dataVersion, int flags)
{
	if (dataVersion < 27) //structure appeared at version 27!
		return false;
	
	QDataStream inStream(&in);

	//name (dataVersion>=27)
	inStream >> m_name;

	//UUID (dataVersion>=27)
	inStream >> m_uuid;

	//relative state (dataVersion>=27)
	if (in.read((char*)&m_relative,sizeof(bool)) < 0)
		return ReadError();

	//Absolute min value (dataVersion>=27)
	if (in.read((char*)&m_absoluteMinValue,sizeof(double)) < 0)
		return ReadError();
	//Absolute range (dataVersion>=27)
	if (in.read((char*)&m_absoluteRange,sizeof(double)) < 0)
		return ReadError();

	//locked state (dataVersion>=27)
	if (in.read((char*)&m_locked,sizeof(bool)) < 0)
		return ReadError();

	//steps list (dataVersion>=27)
	{
		//steps count
		uint32_t stepCount = 0;
		if (in.read((char*)&stepCount,4) < 0)
			return ReadError();

		//read each step
		m_steps.clear();
		for (uint32_t i=0; i<stepCount; ++i)
		{
			double relativePos = 0.0;
			QColor color(Qt::white);
			inStream >> relativePos;
			inStream >> color;

			m_steps.push_back(ccColorScaleElement(relativePos,color));
		}

		update();
	}

	return true;
}

void ccColorScale::setAbsolute(double minVal, double maxVal)
{
	assert(maxVal >= minVal);

	m_relative = false;

	m_absoluteMinValue = minVal;
	m_absoluteRange = maxVal - minVal;

	//as 'm_absoluteRange' is used for division, we make sure it is not left to 0!
	m_absoluteRange = std::max(m_absoluteRange, 1e-12);
}

void ccColorScale::getAbsoluteBoundaries(double& minVal, double& maxVal) const
{
	minVal = m_absoluteMinValue;
	maxVal = m_absoluteMinValue + m_absoluteRange;
}
