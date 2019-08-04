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

#include "SimpleMesh.h"

//local
#include "GenericIndexedCloud.h"

//System
#include <cassert>

using namespace CCLib;

SimpleMesh::SimpleMesh(GenericIndexedCloud* _theVertices, bool linkVerticesWithMesh)
	: GenericIndexedMesh()
	, globalIterator(0)
	, theVertices(_theVertices)
	, verticesLinked(linkVerticesWithMesh)
{
}

SimpleMesh::~SimpleMesh()
{
	if (theVertices && verticesLinked)
	{
		delete theVertices;
		theVertices = nullptr;
	}
}

void SimpleMesh::forEach(genericTriangleAction action)
{
	SimpleTriangle tri;

	for (VerticesIndexes& ti : m_triIndexes)
	{
		theVertices->getPoint(ti.i1, tri.A);
		theVertices->getPoint(ti.i2, tri.B);
		theVertices->getPoint(ti.i3, tri.C);
		action(tri);
	}
}

void SimpleMesh::placeIteratorAtBeginning()
{
	globalIterator = 0;
}

GenericTriangle* SimpleMesh::_getNextTriangle()
{
	return _getTriangle(globalIterator++);
}

GenericTriangle* SimpleMesh::_getTriangle(unsigned triangleIndex)
{
	assert(triangleIndex < m_triIndexes.size());

	const VerticesIndexes& ti = m_triIndexes[triangleIndex];
	theVertices->getPoint(ti.i1, dummyTriangle.A);
	theVertices->getPoint(ti.i2, dummyTriangle.B);
	theVertices->getPoint(ti.i3, dummyTriangle.C);

	return &dummyTriangle; //temporary!
}

void SimpleMesh::getTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C) const
{
	assert(triangleIndex<m_triIndexes.size());

	const VerticesIndexes& ti = m_triIndexes[triangleIndex];
	theVertices->getPoint(ti.i1, A);
	theVertices->getPoint(ti.i2, B);
	theVertices->getPoint(ti.i3, C);
}

void SimpleMesh::getBoundingBox(CCVector3& bbMin, CCVector3& bbMax)
{
	////TODO: how can we know if the vertices cloud changes?!
	//if (!m_bbox.isValid())
	//{
	//	m_bbox.clear();
	//	for (const VerticesIndexes& ti : m_triIndexes)
	//	{
	//		m_bbox.add(*theVertices->getPoint(ti.i1));
	//		m_bbox.add(*theVertices->getPoint(ti.i2));
	//		m_bbox.add(*theVertices->getPoint(ti.i3));
	//	}
	//}

	//bbMin = m_bbox.minCorner();
	//bbMax = m_bbox.maxCorner();

	return theVertices->getBoundingBox(bbMin, bbMax);
}

void SimpleMesh::addTriangle(unsigned i1, unsigned i2, unsigned i3)
{
	m_triIndexes.push_back(VerticesIndexes(i1, i2, i3));

	m_bbox.setValidity(false);
}

bool SimpleMesh::reserve(unsigned n)
{
	try
	{
		m_triIndexes.reserve(n);
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}
	return true;
}

bool SimpleMesh::resize(unsigned n)
{
	try
	{
		m_triIndexes.resize(n);
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}
	return true;
}

VerticesIndexes* SimpleMesh::getTriangleVertIndexes(unsigned triangleIndex)
{
	return &(m_triIndexes[triangleIndex]);
}

VerticesIndexes* SimpleMesh::getNextTriangleVertIndexes()
{
	return getTriangleVertIndexes(globalIterator++);
}
