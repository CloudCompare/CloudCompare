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

#ifndef LOCAL_MODEL_HEADER
#define LOCAL_MODEL_HEADER

#include "CCConst.h"
#include "CCGeom.h"
#include "Neighbourhood.h"

namespace CCLib
{

class GenericMesh;

//! Local modelization - local surface approximation of a point cloud
class LocalModel
{
public:

	//! LocalModel constructor
	/** \param Yk a small set of points from which to compute apprixamted model
		\param mt the model type
		\param center model "center"
		\param  _squareModelSize model max width (squared)
	**/
	LocalModel(Neighbourhood& Yk, CC_LOCAL_MODEL_TYPES mt, const CCVector3 &center, PointCoordinateType _squareModelSize);

	//! LocalModel destructor
	virtual ~LocalModel();

	//! Returns the model center
	/** \return the model center
	**/
	const CCVector3& getCenter() const  {return modelCenter;};

	//! Returns the model max width (squared)
	/** \return the model max width (squared)
	**/
	PointCoordinateType getSquareSize() const {return squareModelSize;};

	//! Compute the distance between a 3D point and this model
	/** \param aPoint the query point
		\return the (unsigned) distance (or HIDDEN_VALUE if computation failed)
	**/
	DistanceType computeDistanceFromModelToPoint(const CCVector3* aPoint);

protected:

	//! Delaunay 2D1/2 triangulation
	GenericMesh* delaunayTri;

	//! Least square best fitting plane equation
	PointCoordinateType* lsqPlane;

	//! Height function equation
	PointCoordinateType* hf;

	//! Height function first dimension (0=X, 1=Y, 2=Z)
	uchar hfX;
	//! Height function second dimension (0=X, 1=Y, 2=Z)
	uchar hfY;
	//! Height function third dimension (0=X, 1=Y, 2=Z)
	uchar hfZ;

	//! Model type
	CC_LOCAL_MODEL_TYPES mtype;

	//! Model "center"
	CCVector3 modelCenter;

	//! Model gravity center
	CCVector3 gravityCenter;

	//! Model max width (squared)
	PointCoordinateType squareModelSize;
};

}

#endif
