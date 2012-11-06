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

#ifndef DELAUNAY2D_MESH_HEADER
#define DELAUNAY2D_MESH_HEADER

#include "GenericIndexedMesh.h"
#include "Neighbourhood.h"
#include "SimpleTriangle.h"

namespace CCLib
{

class GenericIndexedCloud;

//! A class to compute and handle a Delaunay 2D mesh on a subset of points
class Delaunay2dMesh : public GenericIndexedMesh
{
public:

	//! Delaunay2dMesh constructor
	Delaunay2dMesh();

	//! Delaunay2dMesh destructor
	virtual ~Delaunay2dMesh();

	//! Associate this mesh to a point cloud
	/** This particular mesh structure deals with point indexes instead of points.
		Therefore, it is possible to change the associated point cloud (it the
		new cloud has the same size). For example, it can be useful to compute
		the mesh on 2D points corresponding to 3D points that have been projected
		on a plane and then to link this structure with the 3D original	points.
		\param aCloud a point cloud
		\param passOwnership if true the Delaunay2dMesh destructor will delete the cloud as well
	**/
	virtual void linkMeshWith(GenericIndexedCloud* aCloud, bool passOwnership=false);

	//! Build the Delaunay mesh on top a set of 2D points
	/** \param the2dPoints a set of 2D points
		\return success
	**/
	virtual bool build(CC2DPointsConainer &the2dPoints);

	//inherited methods (see GenericMesh)
	virtual unsigned size() const {return numberOfTriangles;};
	virtual void forEach(genericTriangleAction& anAction);
	virtual void getBoundingBox(PointCoordinateType Mins[], PointCoordinateType Maxs[]);
	virtual void placeIteratorAtBegining();
	virtual GenericTriangle* _getNextTriangle();
	virtual GenericTriangle* _getTriangle(unsigned triangleIndex);
	virtual TriangleSummitsIndexes* getNextTriangleIndexes();
	virtual TriangleSummitsIndexes* getTriangleIndexes(unsigned triangleIndex);
	virtual void getTriangleSummits(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C);

protected:

	//! The associated point cloud
	GenericIndexedCloud* theAssociatedCloud;

	//! The triangle summits indexes
	int *theTrianglesIndexes;

	//! Iterator on the list of triangle summits indexes
	int* globalIterator;

	//! End position of global iterator
	int* globalIteratorEnd;

	//! The number of triangles
	unsigned numberOfTriangles;

	//! Specifies if the associated cloud should be deleted when the mesh is deleted
	bool cloudIsOwnedByMesh;

	//! Dump triangle structure to transmit temporary data
	SimpleTriangle dumpTriangle;

	//! Dump triangle index structure to transmit temporary data
    TriangleSummitsIndexes dumpTriangleIndexes;

};

}

#endif
