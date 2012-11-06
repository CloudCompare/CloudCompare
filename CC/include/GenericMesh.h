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

#ifndef GENERIC_MESH_HEADER
#define GENERIC_MESH_HEADER

#include "GenericTriangle.h"

namespace CCLib
{

//! A generic mesh interface for data communication between library and client applications

#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"

class CC_DLL_API GenericMesh
#else
class GenericMesh
#endif
{
public:

	//! Default destructor
	virtual ~GenericMesh() {};

	//! Generic function to apply to a triangle (used by foreach)
	typedef void genericTriangleAction(GenericTriangle&);

	//! Returns the number of triangles
	/**	Virtual method to request the mesh size
		\return the mesh size
	**/
	virtual unsigned size() const =0;

	//! Fast iteration mechanism
	/**	Virtual method to apply a function to the whole mesh
		\param anAction the function to apply (see GenericMesh::genericTriangleAction)
	**/
	virtual void forEach(genericTriangleAction& anAction) = 0;

	//! Returns the mesh bounding-box
	/**	Virtual method to request the mesh bounding-box limits. It is equivalent to
		the bounding-box of the cloud composed of the mesh vertexes.
		\param Mins a 3 elements array to store the lower BB limits (Xmin,Ymin,Zmin)
		\param Maxs a 3 elements array to store the higher BB limits (Xmax,Ymax,Zmax)
	**/
	virtual void getBoundingBox(PointCoordinateType Mins[], PointCoordinateType Maxs[]) = 0;

	//! Places the mesh iterator at the begining
	/**	Virtual method to handle the mesh global iterator
	**/
	virtual void placeIteratorAtBegining()=0;

	//! Returns the next triangle (relatively to the global iterator position)
	/**	Virtual method to handle the mesh global iterator.
		Global iterator position should be increased each time
		this method is called. The returned object can be temporary.
		\return a triangle
	**/
	virtual GenericTriangle* _getNextTriangle()=0; //temporary

};

}

#endif
