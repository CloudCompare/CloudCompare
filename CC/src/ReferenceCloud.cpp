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

#include "ReferenceCloud.h"

//system
#include <assert.h>
#include <algorithm>

using namespace CCLib;

ReferenceCloud::ReferenceCloud(GenericIndexedCloudPersist* associatedCloud)
	: m_theIndexes(0)
	, m_globalIterator(0)
	, m_validBB(false)
	, m_theAssociatedCloud(associatedCloud)
{
	m_theIndexes = new ReferencesContainer();
	m_theIndexes->link();
}

ReferenceCloud::ReferenceCloud(const ReferenceCloud& refCloud)
	: m_theIndexes(0)
	, m_globalIterator(0)
	, m_bbMin(0,0,0)
	, m_bbMax(0,0,0)
	, m_validBB(false)
	, m_theAssociatedCloud(refCloud.m_theAssociatedCloud)
{
	m_theIndexes = new ReferencesContainer();
	m_theIndexes->link();

	//copy data
	if (refCloud.m_theIndexes && refCloud.m_theIndexes->currentSize() != 0)
	{
		//we don't catch any exception so that the caller of the constructor can do it!
		refCloud.m_theIndexes->copy(*m_theIndexes);
	}
}

ReferenceCloud::~ReferenceCloud()
{
	m_theIndexes->release();
}

void ReferenceCloud::clear(bool releaseMemory)
{
	m_theIndexes->clear(releaseMemory);
	invalidateBoundingBox();
}

void ReferenceCloud::updateBBWithPoint(const CCVector3& P)
{
	//X boundaries
	if (m_bbMin.x > P.x)
		m_bbMin.x = P.x;
	else if (m_bbMax.x < P.x)
		m_bbMax.x = P.x;

	//Y boundaries
	if (m_bbMin.y > P.y)
		m_bbMin.y = P.y;
	else if (m_bbMax.y < P.y)
		m_bbMax.y = P.y;

	//Z boundaries
	if (m_bbMin.z > P.z)
		m_bbMin.z = P.z;
	else if (m_bbMax.z < P.z)
		m_bbMax.z = P.z;
}

void ReferenceCloud::computeBB()
{
	//empty cloud?!
	unsigned count = size();
	if (count == 0)
	{
		m_bbMin = m_bbMax = CCVector3(0,0,0);
		return;
	}

	//initialize BBox with first point
	const CCVector3* P = getPointPersistentPtr(0);
	m_bbMin = m_bbMax = *P;

	for (unsigned i=1; i<count; ++i)
	{
		P = getPointPersistentPtr(i);
		updateBBWithPoint(*P);
	}

	m_validBB = true;
}

void ReferenceCloud::getBoundingBox(CCVector3& bbMin, CCVector3& bbMax)
{
	if (!m_validBB)
		computeBB();

	bbMin = m_bbMin;
	bbMax = m_bbMax;
}

bool ReferenceCloud::reserve(unsigned n)
{
	return m_theIndexes->reserve(n);
}

bool ReferenceCloud::resize(unsigned n)
{
	return m_theIndexes->resize(n);
}

const CCVector3* ReferenceCloud::getCurrentPointCoordinates() const
{
	assert(m_theAssociatedCloud && m_globalIterator<size());
	assert(m_theIndexes->getValue(m_globalIterator)<m_theAssociatedCloud->size());
	return m_theAssociatedCloud->getPointPersistentPtr(m_theIndexes->getValue(m_globalIterator));
}

bool ReferenceCloud::addPointIndex(unsigned globalIndex)
{
	if (m_theIndexes->capacity() == m_theIndexes->currentSize())
		if (!m_theIndexes->reserve(m_theIndexes->capacity() + std::min<unsigned>(std::max<unsigned>(1,m_theIndexes->capacity()/2),4096))) //not enough space --> +50% (or 4096)
			return false;

	m_theIndexes->addElement(globalIndex);
	invalidateBoundingBox();

	return true;
}

bool ReferenceCloud::addPointIndex(unsigned firstIndex, unsigned lastIndex)
{
	if (firstIndex >= lastIndex)
	{
		assert(false);
		return false;
	}

	unsigned range = lastIndex-firstIndex; //lastIndex is excluded
    unsigned pos = size();

	if (size()<pos+range && !m_theIndexes->resize(pos+range))
		return false;
	
	for (unsigned i=0; i<range; ++i,++firstIndex)
		m_theIndexes->setValue(pos++,firstIndex);

	invalidateBoundingBox();

	return true;
}

void ReferenceCloud::setPointIndex(unsigned localIndex, unsigned globalIndex)
{
	assert(localIndex < size());
	m_theIndexes->setValue(localIndex,globalIndex);
	invalidateBoundingBox();
}

void ReferenceCloud::forEach(genericPointAction& action)
{
	assert(m_theAssociatedCloud);

	unsigned count = size();
	for (unsigned i=0; i<count; ++i)
	{
		const unsigned& index = m_theIndexes->getValue(i);
		ScalarType d = m_theAssociatedCloud->getPointScalarValue(index);
		ScalarType d2 = d;
		action(*m_theAssociatedCloud->getPointPersistentPtr(index),d2);
		if (d!=d2)
			m_theAssociatedCloud->setPointScalarValue(index,d2);
	}
}

void ReferenceCloud::removePointGlobalIndex(unsigned localIndex)
{
	assert(localIndex < size());

	unsigned lastIndex = size()-1;
	//swap the value to be removed with the last one
	m_theIndexes->setValue(localIndex,m_theIndexes->getValue(lastIndex));
	m_theIndexes->setCurrentSize(lastIndex);
}

void ReferenceCloud::setAssociatedCloud(GenericIndexedCloudPersist* cloud)
{
	m_theAssociatedCloud = cloud;
	invalidateBoundingBox();
}

bool ReferenceCloud::add(const ReferenceCloud& cloud)
{
	if (!m_theIndexes || !cloud.m_theAssociatedCloud || m_theAssociatedCloud != cloud.m_theAssociatedCloud)
		return false;

	unsigned newCount = (cloud.m_theIndexes ? cloud.m_theIndexes->currentSize() : 0);
	if (newCount == 0)
		return true;

	//reserve memory
	unsigned count = m_theIndexes->currentSize();
	if (!m_theIndexes->resize(count + newCount))
		return false;

	//copy new indexes (warning: no duplicate check!)
	for (unsigned i=0; i<newCount; ++i)
		(*m_theIndexes)[count+i] = (*cloud.m_theIndexes)[i];

	invalidateBoundingBox();
	return true;
}
