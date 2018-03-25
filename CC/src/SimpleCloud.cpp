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

#include "ScalarField.h"
#include "SimpleCloud.h"


using namespace CCLib;

SimpleCloud::SimpleCloud()
	: m_points(0)
	, m_scalarField(0)
	, globalIterator(0)
	, m_validBB(false)
{
	m_scalarField = new ScalarField("Default");
	m_scalarField->link();
	m_points = new PointsContainer();
	m_points->link();
}

SimpleCloud::~SimpleCloud()
{
	m_points->release();
	m_scalarField->release();
}

void SimpleCloud::clear()
{
	m_scalarField->clear();
	m_points->clear();
	placeIteratorAtBeginning();
	m_validBB=false;
}

unsigned SimpleCloud::size() const
{
	return m_points->currentSize();
}

void SimpleCloud::addPoint(const CCVector3 &P)
{
	m_points->addElement(P.u);
	m_validBB=false;
}

void SimpleCloud::addPoint(const PointCoordinateType P[])
{
	m_points->addElement(P);
	m_validBB=false;
}

void SimpleCloud::forEach(genericPointAction action)
{
	unsigned n = m_points->currentSize();

	if (m_scalarField->currentSize() >= n) //existing scalar field?
	{
		for (unsigned i=0; i<n; ++i)
		{
			action(*reinterpret_cast<CCVector3*>(m_points->getValue(i)),(*m_scalarField)[i]);
		}
	}
	else //otherwise (we provide a fake zero distance)
	{
		ScalarType d = 0;
		for (unsigned i=0; i<n; ++i)
		{
			action(*reinterpret_cast<CCVector3*>(m_points->getValue(i)),d);
		}
	}
}

void SimpleCloud::getBoundingBox(CCVector3& bbMin, CCVector3& bbMax)
{
	if (!m_validBB)
	{
		m_points->computeMinAndMax();
		m_validBB = true;
	}

	bbMin = CCVector3(m_points->getMin());
	bbMax = CCVector3(m_points->getMax());
}

bool SimpleCloud::reserve(unsigned n)
{
	if (!m_points->reserve(n))
	{
		return false;
	}

	if (m_scalarField->capacity() != 0 && !m_scalarField->reserve(n))
	{
		return false;
	}

	return true;
}

bool SimpleCloud::resize(unsigned n)
{
	unsigned oldCount = m_points->capacity();
	if (!m_points->resize(n))
	{
		return false;
	}
	if (m_scalarField->capacity() > 0 && !m_scalarField->resize(n))
	{
		//revert to previous state
		m_points->resize(oldCount);
		return false;
	}
	return true;
}

void SimpleCloud::placeIteratorAtBeginning()
{
	globalIterator = 0;
}

const CCVector3* SimpleCloud::getNextPoint()
{
	return reinterpret_cast<CCVector3*>(globalIterator < m_points->currentSize() ? m_points->getValue(globalIterator++) : 0);
}

const CCVector3* SimpleCloud::getPointPersistentPtr(unsigned index)
{
	assert(index < m_points->currentSize());
	return reinterpret_cast<CCVector3*>(m_points->getValue(index));
}

void SimpleCloud::getPoint(unsigned index, CCVector3& P) const
{
	assert(index < m_points->currentSize());
	P = *reinterpret_cast<CCVector3*>(m_points->getValue(index));
}

void SimpleCloud::setPointScalarValue(unsigned pointIndex, ScalarType value)
{
	assert(pointIndex<m_scalarField->currentSize());
	m_scalarField->setValue(pointIndex,value);
}

ScalarType SimpleCloud::getPointScalarValue(unsigned pointIndex)  const
{
	assert(pointIndex<m_scalarField->currentSize());
	return m_scalarField->getValue(pointIndex);
}

bool SimpleCloud::enableScalarField()
{
	return m_scalarField->resize(m_points->capacity());
}

bool SimpleCloud::isScalarFieldEnabled() const
{
	return m_scalarField->isAllocated();
}

void SimpleCloud::applyTransformation(PointProjectionTools::Transformation& trans)
{
	unsigned count = m_points->currentSize();

	if (fabs(trans.s - 1.0) > ZERO_TOLERANCE)
	{
		for (unsigned i=0; i<count; ++i)
		{
			CCVector3* P = reinterpret_cast<CCVector3*>(m_points->getValue(i));
			(*P) *= trans.s;
		}
		m_validBB = false;
	}

	if (trans.R.isValid())
	{
		for (unsigned i=0; i<count; ++i)
		{
			CCVector3* P = reinterpret_cast<CCVector3*>(m_points->getValue(i));
			(*P) = trans.R * (*P);
			m_validBB = false;
		}
	}

	if (trans.T.norm() > ZERO_TOLERANCE)
	{
		for (unsigned i=0; i<count; ++i)
		{
			CCVector3* P = reinterpret_cast<CCVector3*>(m_points->getValue(i));
			(*P) += trans.T;
		}
		m_validBB = false;
	}
}
