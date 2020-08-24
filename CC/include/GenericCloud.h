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

#ifndef GENERIC_CLOUD_HEADER
#define GENERIC_CLOUD_HEADER

#include <functional>

//Local
#include "CCConst.h"
#include "CCGeom.h"

namespace CCLib
{

//! A generic 3D point cloud interface for data communication between library and client applications
class CC_CORE_LIB_API GenericCloud
{

public:

	//! Default constructor
	GenericCloud() = default;

	//! Default destructor
	virtual ~GenericCloud() = default;

	//! Generic function applied to a point (used by foreach)
	using genericPointAction = std::function<void (const CCVector3 &, ScalarType &)>;

	//! Returns the number of points
	/**	Virtual method to request the cloud size
		\return the cloud size
	**/
	virtual unsigned size() const = 0;

	//! Fast iteration mechanism
	/**	Virtual method to apply a function to the whole cloud
		\param action the function to apply (see GenericCloud::genericPointAction)
	**/
	virtual void forEach(genericPointAction action) = 0;

	//! Returns the cloud bounding box
	/**	Virtual method to request the cloud bounding box limits
		\param bbMin lower bounding-box limits (Xmin,Ymin,Zmin)
		\param bbMax higher bounding-box limits (Xmax,Ymax,Zmax)
	**/
	virtual void getBoundingBox(CCVector3& bbMin, CCVector3& bbMax) = 0;

	//! Returns a given point visibility state (relatively to a sensor for instance)
	/**	Generic method to request a point visibility (should be overloaded if this functionality is required).
		The point visibility is such as defined in Daniel Girardeau-Montaut's PhD manuscript (see Chapter 2, 
		section 2-3-3). In this case, a ground based laser sensor model should be used to determine it.
		This method is called before performing any point-to-cloud comparison. If the result is not
		POINT_VISIBLE, then the comparison won't be performed and the scalar field value associated
		to this point will be this visibility value.
		\param P the 3D point to test
		\return visibility (default: POINT_VISIBLE)
	**/
	virtual inline unsigned char testVisibility(const CCVector3& P) const { return POINT_VISIBLE; }

	//! Sets the cloud iterator at the beginning
	/**	Virtual method to handle the cloud global iterator
	**/
	virtual void placeIteratorAtBeginning() = 0;

	//! Returns the next point (relatively to the global iterator position)
	/**	Virtual method to handle the cloud global iterator.
		Global iterator position should be increased by one each time
		this method is called.
		Warning:
		- the returned object may not be persistent!
		- THIS METHOD MAY NOT BE COMPATIBLE WITH PARALLEL STRATEGIES
		(see the DgmOctree::executeFunctionForAllCellsAtLevel_MT and
		DgmOctree::executeFunctionForAllCellsAtStartingLevel_MT methods).
		\return pointer on next point (or 0 if no more)
	**/
	virtual const CCVector3* getNextPoint() = 0;

	//!	Enables the scalar field associated to the cloud
	/** If the scalar field structure is not yet initialized/allocated,
		this method gives the signal for its creation. Otherwise, if possible
		the structure size should be pre-reserved with the same number of
		elements as the point cloud.
		
		\warning If the cloud is empty, the scalar field will be empty as well.
		         The scalar field will be reserved with the same capacity as the cloud.
	**/
	virtual bool enableScalarField() = 0;

	//! Returns true if the scalar field is enabled, false otherwise
	virtual bool isScalarFieldEnabled() const = 0;

	//! Sets the ith point associated scalar value
	virtual void setPointScalarValue(unsigned pointIndex, ScalarType value) = 0;

	//! Returns the ith point associated scalar value
	virtual ScalarType getPointScalarValue(unsigned pointIndex) const = 0;
};

}

#endif //GENERIC_CLOUD_HEADER
