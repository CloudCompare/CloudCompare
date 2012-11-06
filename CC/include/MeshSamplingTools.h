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

#ifndef MESH_SAMPLING_TOOLS
#define MESH_SAMPLING_TOOLS

#ifdef _MSC_VER
//To get rid of the really annoying warnings about template class exportation
#pragma warning( disable: 4530 )
#endif

#include "CCToolbox.h"
#include "GenericChunkedArray.h"

//system
#include <vector>

namespace CCLib
{

class GenericProgressCallback;
class GenericMesh;
class SimpleCloud;

//! Mesh sampling algorithms

#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"

class CC_DLL_API MeshSamplingTools : public CCToolbox
#else
class MeshSamplingTools : public CCToolbox
#endif
{
public:

	//! Computes the mesh area
	/** \param theMesh a mesh
		\return the the mesh area
	**/
	static double computeMeshArea(GenericMesh* theMesh);

	//! Samples points on a mesh
	/** The points are sampled on each triangle randomly, by generating
		two numbers between 0 and 1 (a and b). If a+b>1, then a=1-a and
		b=1-b. Let ABC be the triangle, then the new point P will be as
		AP = a.AB+b.AC (AP,AB and AC are vectors here). The number of
		points sampled on each triangle depends on the triangle's area.
		Let s be this area, and µ the sampling density, then N = s*µ is
		the theoric (floating) number of points to sample. The floating
		part of N (let's call it Nf, and let Ni be the integer part) is
		handled by generating another random number between 0 and 1.
		If this number is less than Nf, then Ni=Ni+1. The number of points
		sampled on the triangle will simply be Ni.
		\param theMesh the mesh to be sampled
		\param samplingDensity the sampling surface density
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param[out] triIndices triangle index for each samples point (output only - optional)
		\return the sampled points
	**/
	static SimpleCloud* samplePointsOnMesh(GenericMesh* theMesh,
											double samplingDensity,
                                            GenericProgressCallback* progressCb=0,
											GenericChunkedArray<1,unsigned>* triIndices=0);

	//! Samples points on a mesh
	/** See the other version of this method. Instead of specifying a
		density, it is possible here to specify the total number of
		points to sample (approximative).
		\param theMesh the mesh to be sampled
		\param numberOfPoints the desired number of points on the whole mesh
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param[out] triIndices triangle index for each samples point (output only - optional)
		\return the sampled points
	**/
	static SimpleCloud* samplePointsOnMesh(GenericMesh* theMesh,
											unsigned numberOfPoints,
                                            GenericProgressCallback* progressCb=0,
											GenericChunkedArray<1,unsigned>* triIndices=0);

protected:

	//! Samples points on a mesh - internal method
	/** See public methods descriptions
		\param theMesh the mesh to be sampled
		\param samplingDensity the sampling surfacical density
		\param theoricNumberOfPoints the approximated number of points that will be sampled
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param[out] triIndices triangle index for each samples point (output only - optional)
		\return the sampled points
	**/
	static SimpleCloud* samplePointsOnMesh(GenericMesh* theMesh,
                                            double samplingDensity,
                                            unsigned theoricNumberOfPoints,
                                            GenericProgressCallback* progressCb=0,
											GenericChunkedArray<1,unsigned>* triIndices=0);
};

}

#endif
