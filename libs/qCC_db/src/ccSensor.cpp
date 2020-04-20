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

#include "ccSensor.h"

ccSensor::ccSensor(const QString& name)
	: ccHObject(name)
	, m_posBuffer(nullptr)
	, m_activeIndex(0)
	, m_color(ccColor::green)
	, m_scale(PC_ONE)
{
	m_rigidTransformation.toIdentity();
}

ccSensor::ccSensor(const ccSensor &sensor)
	: ccHObject(sensor)
	, m_posBuffer(nullptr)
	, m_rigidTransformation(sensor.m_rigidTransformation)
	, m_activeIndex(sensor.m_activeIndex)
	, m_color(sensor.m_color)
	, m_scale(sensor.m_scale)
{
	if (sensor.m_posBuffer)
		m_posBuffer = new ccIndexedTransformationBuffer(*sensor.m_posBuffer);
}

bool ccSensor::addPosition(ccGLMatrix& trans, double index)
{
	if (!m_posBuffer)
	{
		m_posBuffer = new ccIndexedTransformationBuffer();
		addChild(m_posBuffer);
		m_posBuffer->setDisplay(getDisplay());
		m_posBuffer->setVisible(true);
		m_posBuffer->setEnabled(false);
	}

	bool sort = (!m_posBuffer->empty() && m_posBuffer->back().getIndex() > index);
	try
	{
		m_posBuffer->emplace_back(trans,index);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory!
		return false;
	}

	if (sort)
		m_posBuffer->sort();

	return true;
}

void ccSensor::applyGLTransformation(const ccGLMatrix& trans)
{
	//transparent call
	ccHObject::applyGLTransformation(trans);

	//we update the rigid transformation
	m_rigidTransformation = trans * m_rigidTransformation;
}

void ccSensor::getIndexBounds(double& minIndex, double& maxIndex) const
{
	if (m_posBuffer && !m_posBuffer->empty())
	{
		minIndex = m_posBuffer->front().getIndex();
		maxIndex = m_posBuffer->back().getIndex();
	}
	else
	{
		minIndex = maxIndex = 0;
	}
}

bool ccSensor::getAbsoluteTransformation(ccIndexedTransformation& trans, double index) const
{
	trans.toIdentity();
	if (m_posBuffer)
		if (!m_posBuffer->getInterpolatedTransformation(index,trans))
			return false;

	trans *= m_rigidTransformation;

	return true;
}

bool ccSensor::getActiveAbsoluteTransformation(ccIndexedTransformation& trans) const
{
	if (!getAbsoluteTransformation(trans, m_activeIndex))
	{
		ccLog::Warning("[ccSensor::getActiveAbsoluteTransformation] Failed to get a valid transformation for active index!");
		return false;
	}

	return true;
}

bool ccSensor::getActiveAbsoluteCenter(CCVector3& vec) const
{
	ccIndexedTransformation trans;
	
	if (!getActiveAbsoluteTransformation(trans))
		return false;

	vec = trans.getTranslationAsVec3D();
	return true;
}

bool ccSensor::getActiveAbsoluteRotation(ccGLMatrix& rotation) const
{
	ccIndexedTransformation trans;
	
	if (!getActiveAbsoluteTransformation(trans))
		return false;

	ccGLMatrix mat = trans;
	mat.setTranslation(CCVector3(0.0,0.0,0.0));
	rotation = mat;

	return true;
}

bool ccSensor::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//rigid transformation (dataVersion>=34)
	if (!m_rigidTransformation.toFile(out))
		return WriteError();

	//various parameters (dataVersion>=35)
	QDataStream outStream(&out);
	outStream << m_activeIndex;		//active index
	outStream << m_scale;			//scale

	//color (dataVersion>=35)
	if (out.write((const char*)&m_color.rgb,sizeof(ColorCompType)*3) < 0)
		return WriteError();

	//we can't save the associated position buffer (as it may be shared by multiple sensors)
	//so instead we save it's unique ID (dataVersion>=34)
	//WARNING: the buffer must be saved in the same BIN file! (responsibility of the caller)
	uint32_t bufferUniqueID = (m_posBuffer ? static_cast<uint32_t>(m_posBuffer->getUniqueID()) : 0);
	if (out.write((const char*)&bufferUniqueID,4) < 0)
		return WriteError();

	return true;
}

bool ccSensor::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//serialization wasn't possible before v3.4!
	if (dataVersion < 34)
		return false;

	//rigid transformation (dataVersion>=34)
	if (!m_rigidTransformation.fromFile(in, dataVersion, flags, oldToNewIDMap))
		return ReadError();

	//various parameters (dataVersion>=35)
	QDataStream inStream(&in);
	inStream >> m_activeIndex;
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_scale);

	//color (dataVersion>=35)
	if (in.read((char*)&m_color.rgb, sizeof(ColorCompType) * 3) < 0)
		return ReadError();

	//as the associated position buffer can't be saved directly (as it may be shared by multiple sensors)
	//we only store its unique ID (dataVersion>=34) --> we hope we will find it at loading time (i.e. this
	//is the responsibility of the caller to make sure that all dependencies are saved together)
	uint32_t bufferUniqueID = 0;
	if (in.read((char*)&bufferUniqueID, 4) < 0)
		return ReadError();
	//[DIRTY] WARNING: temporarily, we set the vertices unique ID in the 'm_posBuffer' pointer!!!
	*(uint32_t*)(&m_posBuffer) = bufferUniqueID;

	return true;
}

bool ccSensor::applyViewport(ccGenericGLDisplay* win/*=0*/)
{
	//not supported by default, must be reimplemented by the child class
	ccLog::Warning("[ccSensor::applyViewport] Unhandled sensor type!");
	return false;
}
