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

#include "DgmOctreeReferenceCloud.h"

#include <string.h>
#include <assert.h>

using namespace CCLib;

DgmOctreeReferenceCloud::DgmOctreeReferenceCloud(DgmOctree::NeighboursSet* associatedSet, unsigned size/*=0*/)
	: m_globalIterator(0)
	, m_validBB(false)
	, m_set(associatedSet)
	, m_size(size)
{
	assert(associatedSet);
	if (!m_size)
		m_size = associatedSet->size();
}

DgmOctreeReferenceCloud::~DgmOctreeReferenceCloud()
{
}

unsigned DgmOctreeReferenceCloud::size() const
{
	return m_size;
}

const CCVector3* DgmOctreeReferenceCloud::getPoint(unsigned index)
{
	assert(index < size());
	return m_set->at(index).point;
}

const CCVector3* DgmOctreeReferenceCloud::getPointPersistentPtr(unsigned index)
{
	assert(index < size());
	return m_set->at(index).point;
}

void DgmOctreeReferenceCloud::getPoint(unsigned index, CCVector3& P) const
{
	assert(index < size());
	P = *m_set->at(index).point;
}

const CCVector3* DgmOctreeReferenceCloud::getNextPoint()
{
	return (m_globalIterator < size() ? m_set->at(m_globalIterator++).point : 0);
}

void DgmOctreeReferenceCloud::setPointScalarValue(unsigned pointIndex, DistanceType value)
{
	assert(pointIndex < size());
	m_set->at(pointIndex).squareDist = value;
}

DistanceType DgmOctreeReferenceCloud::getPointScalarValue(unsigned pointIndex) const
{
	assert(pointIndex < size());
	return m_set->at(pointIndex).squareDist;
}

void DgmOctreeReferenceCloud::updateBBWithPoint(const CCVector3* P)
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

void DgmOctreeReferenceCloud::computeBB()
{
	//empty cloud?!
	if (m_set->empty())
	{
		m_bbMins[0]=m_bbMaxs[0]=0.0;
		m_bbMins[1]=m_bbMaxs[1]=0.0;
		m_bbMins[2]=m_bbMaxs[2]=0.0;
		return;
	}

	//initialize BBox with first point
	const CCVector3* P = m_set->at(0).point;
	m_bbMins[0]=m_bbMaxs[0]=P->x;
	m_bbMins[1]=m_bbMaxs[1]=P->y;
	m_bbMins[2]=m_bbMaxs[2]=P->z;

	unsigned i,count=size();
	for (i=1;i<count;++i)
		updateBBWithPoint(m_set->at(i).point);

	m_validBB=true;
}

void DgmOctreeReferenceCloud::getBoundingBox(PointCoordinateType Mins[], PointCoordinateType Maxs[])
{
	if (!m_validBB)
		computeBB();

	memcpy(Mins,m_bbMins,sizeof(PointCoordinateType)*3);
	memcpy(Maxs,m_bbMaxs,sizeof(PointCoordinateType)*3);
}

void DgmOctreeReferenceCloud::placeIteratorAtBegining()
{
	m_globalIterator=0;
}

void DgmOctreeReferenceCloud::forwardIterator()
{
	++m_globalIterator;
}

void DgmOctreeReferenceCloud::forEach(genericPointAction& anAction)
{
	unsigned count=size();
	for (unsigned i=0;i<count;++i)
		anAction(*m_set->at(i).point,m_set->at(i).squareDist);
}
