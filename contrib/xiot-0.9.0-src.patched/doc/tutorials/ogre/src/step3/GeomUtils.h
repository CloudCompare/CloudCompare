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

#ifndef _GEOMUTILS_H
#define _GEOMUTILS_H

#include "OgreString.h"
#include "OgreVertexIndexData.h"

class GeomUtils
{
public:
	// Create a Mesh with a given name, radius, number of rings and number of segments
	static void createSphere(const Ogre::String& strName
		, const float radius
		, const int nRings, const int nSegments
		, bool bNormals
		, bool bTexCoords
		);


	// Fill up a fresh copy of VertexData and IndexData with a sphere's coords given the number of rings and the number of segments
	static void createSphere(Ogre::VertexData*& vertexData, Ogre::IndexData*& indexData
		, float radius
		, int nRings, int nSegments
		, bool bNormals
		, bool bTexCoords);


	// Fill up a fresh copy of VertexData with a normalized quad
	static void createQuad(Ogre::VertexData*& vertexData);


};


#endif
