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
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#include "ReferenceCloud.h"

#include <string.h>
#include <assert.h>

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

ReferenceCloud::~ReferenceCloud()
{
	m_theIndexes->release();
}

void ReferenceCloud::clear(bool releaseMemory)
{
	m_theIndexes->clear(releaseMemory);
	m_validBB=false;
}

void ReferenceCloud::updateBBWithPoint(const CCVector3* P)
{
	//X boundaries
	if (m_bbMins[0]>P->x)
		m_bbMins[0]=P->x;
	else if (m_bbMaxs[0]<P->x)
		m_bbMaxs[0]=P->x;

	//Y boundaries
	if (m_bbMins[1]>P->y)
		m_bbMins[1]=P->y;
	else if (m_bbMaxs[1]<P->y)
		m_bbMaxs[1]=P->y;

	//Z boundaries
	if (m_bbMins[2]>P->z)
		m_bbMins[2]=P->z;
	else if (m_bbMaxs[2]<P->z)
		m_bbMaxs[2]=P->z;
}

void ReferenceCloud::computeBB()
{
	assert(m_theAssociatedCloud);

	//empty cloud?!
	if (size()==0)
	{
		m_bbMins[0]=m_bbMaxs[0]=0.0;
		m_bbMins[1]=m_bbMaxs[1]=0.0;
		m_bbMins[2]=m_bbMaxs[2]=0.0;
		return;
	}

	//initialize BBox with first point
	const CCVector3* P = m_theAssociatedCloud->getPointPersistentPtr(m_theIndexes->getValue(0));
	m_bbMins[0]=m_bbMaxs[0]=P->x;
	m_bbMins[1]=m_bbMaxs[1]=P->y;
	m_bbMins[2]=m_bbMaxs[2]=P->z;

	unsigned i,count=size();
	for (i=1;i<count;++i)
	{
		P = m_theAssociatedCloud->getPointPersistentPtr(m_theIndexes->getValue(i));
		updateBBWithPoint(P);
	}

	m_validBB=true;
}

void ReferenceCloud::getBoundingBox(PointCoordinateType Mins[], PointCoordinateType Maxs[])
{
	if (!m_validBB)
		computeBB();

	memcpy(Mins,m_bbMins,sizeof(PointCoordinateType)*3);
	memcpy(Maxs,m_bbMaxs,sizeof(PointCoordinateType)*3);
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

void ReferenceCloud::addPointIndex(unsigned globalIndex)
{
	if (m_theIndexes->capacity()==m_theIndexes->currentSize())
		if (!m_theIndexes->reserve(m_theIndexes->capacity()+std::max((unsigned)1,m_theIndexes->capacity()/2))) //not enough memory --> +50%
			//DGM TODO: we should warn the caller!
			return;

	m_theIndexes->addElement(globalIndex);
	m_validBB=false;
}

bool ReferenceCloud::addPointIndex(unsigned firstIndex, unsigned lastIndex)
{
	if (firstIndex>=lastIndex)
	{
		assert(false);
		return false;
	}

	unsigned range = lastIndex-firstIndex; //lastIndex is excluded
    unsigned pos = size();

	if (size()<pos+range)
		if (!m_theIndexes->resize(pos+range))
			return false;
	for (unsigned i=0;i<range;++i,++firstIndex)
		m_theIndexes->setValue(pos++,firstIndex);

	return true;
}

void ReferenceCloud::setPointIndex(unsigned localIndex, unsigned globalIndex)
{
	assert(localIndex<size());
	m_theIndexes->setValue(localIndex,globalIndex);
	m_validBB=false;
}

void ReferenceCloud::forEach(genericPointAction& anAction)
{
	assert(m_theAssociatedCloud);

	DistanceType d,d2;
	unsigned count=size();
	for (unsigned i=0;i<count;++i)
	{
		const unsigned& index = m_theIndexes->getValue(i);
		d2 = d = m_theAssociatedCloud->getPointScalarValue(index);
		anAction(*m_theAssociatedCloud->getPointPersistentPtr(index),d2);
		if (d!=d2)
			m_theAssociatedCloud->setPointScalarValue(index,d2);
	}
}

void ReferenceCloud::removePointGlobalIndex(unsigned localIndex)
{
	assert(localIndex<size());

	unsigned lastIndex = size()-1;
	//swap the value to be removed with the last one
	m_theIndexes->setValue(localIndex,m_theIndexes->getValue(lastIndex));
	m_theIndexes->setCurrentSize(lastIndex);
}

void ReferenceCloud::setAssociatedCloud(GenericIndexedCloudPersist* cloud)
{
	m_theAssociatedCloud=cloud;
	m_validBB=false;
}
