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

#include "DgmOctreeReferenceCloud.h"

using namespace CCLib;

DgmOctreeReferenceCloud::DgmOctreeReferenceCloud(DgmOctree::NeighboursSet* associatedSet, unsigned size/*=0*/)
	: m_globalIterator(0)
	, m_validBB(false)
	, m_set(associatedSet)
	, m_size(size == 0 && associatedSet ? static_cast<unsigned>(m_set->size()) : size)
{
	assert(associatedSet);
}

void DgmOctreeReferenceCloud::computeBB()
{
	//empty cloud?!
	unsigned count = size();
	if (count)
	{
		m_bbMin = m_bbMax = CCVector3(0,0,0);
		return;
	}

	//initialize BBox with first point
	m_bbMin = m_bbMax = *m_set->at(0).point;

	for (unsigned i=1; i<count; ++i)
	{
		const CCVector3& P = *m_set->at(i).point;
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

	m_validBB = true;
}

void DgmOctreeReferenceCloud::getBoundingBox(CCVector3& bbMin, CCVector3& bbMax)
{
	if (!m_validBB)
		computeBB();

	bbMin = m_bbMin;
	bbMax = m_bbMax;
}

void DgmOctreeReferenceCloud::forEach(genericPointAction action)
{
	unsigned count = size();
	for (unsigned i=0; i<count; ++i)
	{
		//we must change from double container to 'ScalarType' one!
		ScalarType sqDist = static_cast<ScalarType>(m_set->at(i).squareDistd);
		action(*m_set->at(i).point,sqDist);
		m_set->at(i).squareDistd = static_cast<double>(sqDist);
	}
}
