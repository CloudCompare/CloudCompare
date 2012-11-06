/*
-----------------------------------------------------------------------------
This source file is part of OGRE
(Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org/

Copyright (c) 2000-2006 Torus Knot Software Ltd
Also see acknowledgements in Readme.html

You may use this sample code for anything you like, it is not covered by the
LGPL like the rest of the engine.
-----------------------------------------------------------------------------
*/


#include "GeomUtils.h"

#include "OgreMeshManager.h"
#include "OgreSubMesh.h"
#include "OgreHardwareBufferManager.h"

using namespace Ogre;

void GeomUtils::createSphere(  const String& strName
							 , const float radius
							 , const int nRings, const int nSegments
							 , bool bNormals
							 , bool bTexCoords)
{
	MeshPtr pSphere = MeshManager::getSingleton().createManual(strName, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	SubMesh *pSphereVertex = pSphere->createSubMesh();
	pSphere->sharedVertexData = new VertexData();

	createSphere(pSphere->sharedVertexData, pSphereVertex->indexData
		, radius
		, nRings, nSegments
		, bNormals // need normals
		, bTexCoords // need texture co-ordinates
		);

	// Generate face list
	pSphereVertex->useSharedVertices = true;

	// the original code was missing this line:
	pSphere->_setBounds( AxisAlignedBox( Vector3(-radius, -radius, -radius), Vector3(radius, radius, radius) ), false );
	pSphere->_setBoundingSphereRadius(radius);
	// this line makes clear the mesh is loaded (avoids memory leaks)
	pSphere->load();
}

void GeomUtils::createSphere(VertexData*& vertexData, IndexData*& indexData
						 , float radius
						 , int nRings, int nSegments
						 , bool bNormals
						 , bool bTexCoords)
{
	assert(vertexData && indexData);

	// define the vertex format
	VertexDeclaration* vertexDecl = vertexData->vertexDeclaration;
	size_t currOffset = 0;
	// positions
	vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_POSITION);
	currOffset += VertexElement::getTypeSize(VET_FLOAT3);

	if (bNormals)
	{
		// normals
		vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_NORMAL);
		currOffset += VertexElement::getTypeSize(VET_FLOAT3);

	}
	// two dimensional texture coordinates
	if (bTexCoords)
	{
		vertexDecl->addElement(0, currOffset, VET_FLOAT2, VES_TEXTURE_COORDINATES, 0);
		currOffset += VertexElement::getTypeSize(VET_FLOAT2);
	}

	// allocate the vertex buffer
	vertexData->vertexCount = (nRings + 1) * (nSegments+1);
	HardwareVertexBufferSharedPtr vBuf = HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(0), vertexData->vertexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
	VertexBufferBinding* binding = vertexData->vertexBufferBinding;
	binding->setBinding(0, vBuf);
	float* pVertex = static_cast<float*>(vBuf->lock(HardwareBuffer::HBL_DISCARD));

	// allocate index buffer
	indexData->indexCount = 6 * nRings * (nSegments + 1);
	indexData->indexBuffer = HardwareBufferManager::getSingleton().createIndexBuffer(HardwareIndexBuffer::IT_16BIT, indexData->indexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
	HardwareIndexBufferSharedPtr iBuf = indexData->indexBuffer;
	unsigned short* pIndices = static_cast<unsigned short*>(iBuf->lock(HardwareBuffer::HBL_DISCARD));

	float fDeltaRingAngle = (Math::PI / nRings);
	float fDeltaSegAngle = (2 * Math::PI / nSegments);
	unsigned short wVerticeIndex = 0 ;

	// Generate the group of rings for the sphere
	for( int ring = 0; ring <= nRings; ring++ ) {
		float r0 = radius * sinf (ring * fDeltaRingAngle);
		float y0 = radius * cosf (ring * fDeltaRingAngle);

		// Generate the group of segments for the current ring
		for(int seg = 0; seg <= nSegments; seg++) {
			float x0 = r0 * sinf(seg * fDeltaSegAngle);
			float z0 = r0 * cosf(seg * fDeltaSegAngle);

			// Add one vertex to the strip which makes up the sphere
			*pVertex++ = x0;
			*pVertex++ = y0;
			*pVertex++ = z0;

			if (bNormals)
			{
				Vector3 vNormal = Vector3(x0, y0, z0).normalisedCopy();
				*pVertex++ = vNormal.x;
				*pVertex++ = vNormal.y;
				*pVertex++ = vNormal.z;
			}
			if (bTexCoords)
			{
				*pVertex++ = (float) seg / (float) nSegments;
				*pVertex++ = (float) ring / (float) nRings;			
			}

			if (ring != nRings) 
			{
				// each vertex (except the last) has six indices pointing to it
				*pIndices++ = wVerticeIndex + nSegments + 1;
				*pIndices++ = wVerticeIndex;               
				*pIndices++ = wVerticeIndex + nSegments;
				*pIndices++ = wVerticeIndex + nSegments + 1;
				*pIndices++ = wVerticeIndex + 1;
				*pIndices++ = wVerticeIndex;
				wVerticeIndex ++;
			}
		}; // end for seg
	} // end for ring

	// Unlock
	vBuf->unlock();
	iBuf->unlock();
}

void GeomUtils::createQuad(VertexData*& vertexData)
{
	assert(vertexData);

	vertexData->vertexCount = 4;
	vertexData->vertexStart = 0;

	VertexDeclaration* vertexDecl = vertexData->vertexDeclaration;
	VertexBufferBinding* bind = vertexData->vertexBufferBinding;

	vertexDecl->addElement(0, 0, VET_FLOAT3, VES_POSITION);

	HardwareVertexBufferSharedPtr vbuf = 
		HardwareBufferManager::getSingleton().createVertexBuffer(
		vertexDecl->getVertexSize(0),
		vertexData->vertexCount,
		HardwareBuffer::HBU_STATIC_WRITE_ONLY);

	// Bind buffer
	bind->setBinding(0, vbuf);
	// Upload data
	float data[]={
		-1,1,-1,  // corner 1
		-1,-1,-1, // corner 2
		1,1,-1,   // corner 3
		1,-1,-1}; // corner 4
		vbuf->writeData(0, sizeof(data), data, true);
}
