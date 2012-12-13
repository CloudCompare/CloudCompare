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

#ifndef POINT_PROJECTION_TOOLS_HEADER
#define POINT_PROJECTION_TOOLS_HEADER

#include "CCToolbox.h"
#include "Matrix.h"

//! Triangulation types
enum CC_TRIANGULATION_TYPES {GENERIC							=		1,		/**< Default triangulation (Delaunay 2D in XY plane) **/
								GENERIC_BEST_LS_PLANE			=		2,		/**< Delaunay 2D in best least square fitting plane **/
								GENERIC_EMPTY					=		3		/**<Empty triangulation (to be filled by user) **/
};

namespace CCLib
{

class GenericIndexedMesh;
class GenericProgressCallback;
class GenericIndexedCloud;
class GenericIndexedCloudPersist;
class GenericCloud;
class SimpleCloud;

//! Several point cloud re-projection algorithms ("developpee", translation, rotation, etc.)
#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"

class CC_DLL_API PointProjectionTools : public CCToolbox
#else
class PointProjectionTools : public CCToolbox
#endif
{
public:

	//! A geometrical transformation (rotation + translation)
	/** P' = R.P + T
	**/
	struct Transformation
	{
	    //! Rotation
		CCLib::SquareMatrix R;
		//! Translation
		CCVector3 T;
	};

	//! Develops a cylinder-shaped point cloud around its main axis
	/** Generates a "developpée" of a cylinder-shaped point cloud.
		WARNING: this method uses the cloud global iterator
		\param theCloud the point cloud to be developed
		\param radius the cylinder radius
		\param dim the dimension along which the cylinder axis is aligned (X=0, Y=1, Z=2)
		\param center a 3D point (as a 3 values array) belonging to the cylinder axis
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return the "developed" cloud
	**/
	static SimpleCloud* developCloudOnCylinder(GenericCloud* theCloud, PointCoordinateType radius, unsigned char dim=2, CCVector3* center=0, GenericProgressCallback* progressCb=0);

	//! Develops a cone-shaped point cloud around its main axis
	/** Generates a "developpée" of a cone-shaped point cloud.
		WARNING: this method uses the cloud global iterator
		\param theCloud the point cloud to be developed
		\param dim the dimension along which the cone axis is aligned (X=0, Y=1, Z=2)
		\param baseRadius the radius of the base of the cone
		\param alpha the angle of the cone "opening"
		\param center the 3D point corresponding to the intersection between the cone axis and its base
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return the "developed" cloud
	**/
	static SimpleCloud* developCloudOnCone(GenericCloud* theCloud, uchar dim, PointCoordinateType baseRadius, float alpha, const CCVector3& center, GenericProgressCallback* progressCb=0);

	//! Applys a geometrical transformation to a point cloud
	/** \param theCloud the point cloud to be "transformed"
		\param trans the geometrical transformation
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return the "transformed" cloud
	**/
	static SimpleCloud* applyTransformation(GenericCloud* theCloud, Transformation& trans, GenericProgressCallback* progressCb=0);

	//! Computes a 2.5D Delaunay triangulation
	/** The triangulation can be either computed on the points projected
		in the XY plane (by default), or projected on the best least-square
		fitting plane. The triangulation is in 2D (in the plane) but the
		3D points are connected, so it's a kind of 2.5D triangulation (that
		may present however several topological aberrations ;).
		\param theCloud a point cloud
		\param type the triangulation strategy
		\return a mesh
	**/
	static GenericIndexedMesh* computeTriangulation(GenericIndexedCloudPersist* theCloud, CC_TRIANGULATION_TYPES type=GENERIC);

};

}

#endif
