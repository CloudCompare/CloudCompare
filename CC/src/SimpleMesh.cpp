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

using namespace CCLib;

SimpleMesh::SimpleMesh(GenericIndexedCloud* _theVertices, bool linkVerticesWithMesh)
	: GenericIndexedMesh()
	, m_triIndexes(0)
	, globalIterator(0)
	, theVertices(_theVertices)
	, verticesLinked(linkVerticesWithMesh)
	, bbMin(0,0,0)
	, bbMax(0,0,0)
	, bbIsValid(false)
{
	m_triIndexes = new TriangleIndexesContainer();
	m_triIndexes->link();
}

SimpleMesh::~SimpleMesh()
{
	if (theVertices && verticesLinked)
        delete theVertices;

	m_triIndexes->release();
}

unsigned SimpleMesh::size() const
{
    return m_triIndexes->currentSize();
};

void SimpleMesh::forEach(genericTriangleAction action)
{
	SimpleTriangle tri;
	unsigned count = m_triIndexes->currentSize();
	for (unsigned i=0; i<count; ++i)
	{
		const unsigned *ti = m_triIndexes->getValue(i);
		theVertices->getPoint(ti[0],tri.A);
		theVertices->getPoint(ti[1],tri.B);
		theVertices->getPoint(ti[2],tri.C);
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
	assert(triangleIndex<m_triIndexes->currentSize());

	const unsigned *ti = m_triIndexes->getValue(triangleIndex);
	theVertices->getPoint(ti[0],dummyTriangle.A);
	theVertices->getPoint(ti[1],dummyTriangle.B);
	theVertices->getPoint(ti[2],dummyTriangle.C);

	return &dummyTriangle; //temporary!
}

void SimpleMesh::getTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C)
{
	assert(triangleIndex<m_triIndexes->currentSize());

	const unsigned *ti = m_triIndexes->getValue(triangleIndex);
	theVertices->getPoint(ti[0],A);
	theVertices->getPoint(ti[1],B);
	theVertices->getPoint(ti[2],C);
}

void SimpleMesh::updateBBWithPoint(const CCVector3* P)
{
    if (bbIsValid)
    {
        if (bbMin.x>P->x)
            bbMin.x = P->x;
        else if (bbMax.x<P->x)
            bbMax.x = P->x;

        if (bbMin.y>P->y)
            bbMin.y = P->y;
        else if (bbMax.y<P->y)
            bbMax.y = P->y;

        if (bbMin.z>P->z)
            bbMin.z = P->z;
        else if (bbMax.z<P->z)
            bbMax.z = P->z;
    }
	else
    {
        bbMin = bbMax = *P;
        bbIsValid = true;
    }
}

void SimpleMesh::getBoundingBox(CCVector3& bbMin, CCVector3& bbMax)
{
    /* TODO: how can we know if the vertices cloud changes?!
    if (!bbIsValid)
    {
        unsigned i,n=m_triIndexes->size();
        if (n>0)
        {
            m_triIndexes->placeIteratorAtBeginning();
            for (i=0;i<n;++i)
            {
                const unsigned *ti = m_triIndexes->getCurrentValue();
                updateBBWithPoint(theVertices->getPoint(ti[0]));
                updateBBWithPoint(theVertices->getPoint(ti[1]));
                updateBBWithPoint(theVertices->getPoint(ti[2]));
                m_triIndexes->forwardIterator();
            }
        }
        else
        {
            bbMin.x=bbMin.y=bbMin.z=0.0;
            bbMax.x=bbMax.y=bbMax.z=0.0;
            bbIsValid = true;
        }
    }

    memcpy(Mins,bbMin.u,3*sizeof(PointCoordinateType));
    memcpy(Maxs,bbMax.u,3*sizeof(PointCoordinateType));
    */

    return theVertices->getBoundingBox(bbMin,bbMax);
}

//specific methods
void SimpleMesh::addTriangle(unsigned i1, unsigned i2, unsigned i3)
{
	unsigned t[3] = {i1,i2,i3};
	m_triIndexes->addElement(t);

	bbIsValid = false;
}

bool SimpleMesh::reserve(unsigned n)
{
	return m_triIndexes->reserve(n);
}

bool SimpleMesh::resize(unsigned n)
{
    if (n<m_triIndexes->currentSize())
        bbIsValid = false;

	return m_triIndexes->resize(n);
}

VerticesIndexes* SimpleMesh::getTriangleVertIndexes(unsigned triangleIndex)
{
	return reinterpret_cast<VerticesIndexes*>(m_triIndexes->getValue(triangleIndex));
}

VerticesIndexes* SimpleMesh::getNextTriangleVertIndexes()
{
	return getTriangleVertIndexes(globalIterator++);
}
