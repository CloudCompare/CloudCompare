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

#include "ccSensor.h"

ccSensor::ccSensor(QString name)
	: ccHObject(name)
	, m_posBuffer(0)
	, m_activeIndex(0)
{
	m_rigidTransformation.toIdentity();
}

bool ccSensor::addPosition(ccGLMatrix& trans, double index)
{
	if (!m_posBuffer)
		m_posBuffer = QSharedPointer<ccIndexedTransformationBuffer>(new ccIndexedTransformationBuffer());

	bool sort = (!m_posBuffer->empty() && m_posBuffer->back().getIndex() > index);
	try
	{
		m_posBuffer->push_back(ccIndexedTransformation(trans,index));
	}
	catch(std::bad_alloc)
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
	//we update the rigid transformation
	m_rigidTransformation = trans * m_rigidTransformation;
	ccHObject::applyGLTransformation(trans);
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

bool ccSensor::getCenterPosition(ccIndexedTransformation& trans, double index)
{
	trans.toIdentity();
	if (m_posBuffer)
		if (!m_posBuffer->getInterpolatedTransformation(index,trans))
			return false;

	trans *= m_rigidTransformation;

	return true;
}
