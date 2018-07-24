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

#include "ReferenceCloud.h"

//system
#include <algorithm>

using namespace CCLib;

ReferenceCloud::ReferenceCloud(GenericIndexedCloudPersist* associatedCloud)
	: m_globalIterator(0)
	, m_theAssociatedCloud(associatedCloud)
{
}

ReferenceCloud::ReferenceCloud(const ReferenceCloud& refCloud)
	: m_globalIterator(0)
	, m_theAssociatedCloud(refCloud.m_theAssociatedCloud)
	, m_theIndexes(refCloud.m_theIndexes) //we don't catch any exception so that the caller of the constructor can do it!
{
}

ReferenceCloud::~ReferenceCloud()
{
}

void ReferenceCloud::clear(bool releaseMemory/*=false*/)
{
	if (releaseMemory)
		m_theIndexes.resize(0);
	else
		m_theIndexes.clear();

	invalidateBoundingBox();
}

void ReferenceCloud::getBoundingBox(CCVector3& bbMin, CCVector3& bbMax)
{
	if (!m_bbox.isValid())
	{
		m_bbox.clear();
		for (unsigned index : m_theIndexes)
		{
			m_bbox.add(*m_theAssociatedCloud->getPoint(index));
		}
	}

	bbMin = m_bbox.minCorner();
	bbMax = m_bbox.maxCorner();
}

bool ReferenceCloud::reserve(unsigned n)
{
	try
	{
		m_theIndexes.reserve(n);
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}

	return true;
}

bool ReferenceCloud::resize(unsigned n)
{
	try
	{
		m_theIndexes.resize(n);
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}

	return true;
}

const CCVector3* ReferenceCloud::getCurrentPointCoordinates() const
{
	assert(m_theAssociatedCloud && m_globalIterator < size());
	assert(m_theIndexes[m_globalIterator] < m_theAssociatedCloud->size());
	return m_theAssociatedCloud->getPointPersistentPtr(m_theIndexes[m_globalIterator]);
}

bool ReferenceCloud::addPointIndex(unsigned globalIndex)
{
	try
	{
		m_theIndexes.push_back(globalIndex);
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}
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

	unsigned range = lastIndex - firstIndex; //lastIndex is excluded
    unsigned pos = size();

	if (size() < pos + range)
	{
		try
		{
			m_theIndexes.resize(pos + range);
		}
		catch (const std::bad_alloc&)
		{
			return false;
		}
	}
	
	for (unsigned i = 0; i < range; ++i, ++firstIndex)
	{
		m_theIndexes[pos++] = firstIndex;
	}

	invalidateBoundingBox();

	return true;
}

void ReferenceCloud::setPointIndex(unsigned localIndex, unsigned globalIndex)
{
	assert(localIndex < size());
	m_theIndexes[localIndex] = globalIndex;
	invalidateBoundingBox();
}

void ReferenceCloud::forEach(genericPointAction action)
{
	assert(m_theAssociatedCloud);

	unsigned count = size();
	for (unsigned i = 0; i < count; ++i)
	{
		const unsigned& index = m_theIndexes[i];
		ScalarType d = m_theAssociatedCloud->getPointScalarValue(index);
		ScalarType d2 = d;
		action(*m_theAssociatedCloud->getPointPersistentPtr(index), d2);
		if (d != d2)
			m_theAssociatedCloud->setPointScalarValue(index, d2);
	}
}

void ReferenceCloud::removePointGlobalIndex(unsigned localIndex)
{
	assert(localIndex < size());

	unsigned lastIndex = size() - 1;
	//swap the value to be removed with the last one
	m_theIndexes[localIndex] = m_theIndexes[lastIndex];
	m_theIndexes.resize(lastIndex);
}

void ReferenceCloud::setAssociatedCloud(GenericIndexedCloudPersist* cloud)
{
	m_theAssociatedCloud = cloud;
	invalidateBoundingBox();
}

bool ReferenceCloud::add(const ReferenceCloud& cloud)
{
	if (!cloud.m_theAssociatedCloud || m_theAssociatedCloud != cloud.m_theAssociatedCloud)
	{
		return false;
	}

	std::size_t newCount = cloud.m_theIndexes.size();
	if (newCount == 0)
		return true;

	//reserve memory
	std::size_t count = m_theIndexes.size();
	try
	{
		m_theIndexes.resize(count + newCount);
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}

	//copy new indexes (warning: no duplicate check!)
	for (unsigned i = 0; i < newCount; ++i)
	{
		m_theIndexes[count + i] = cloud.m_theIndexes[i];
	}

	invalidateBoundingBox();
	return true;
}
