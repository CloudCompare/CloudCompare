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

#ifndef LOCAL_MODEL_HEADER
#define LOCAL_MODEL_HEADER

#include "Neighbourhood.h"

namespace CCLib
{

//! Local modelization (generic interface)
/** Local surface approximation of a point cloud
**/
class LocalModel
{
public:

	//! Factory
	/** \param type the model type
		\param subset (small) set of points from which to compute the local model
		\param center model "center"
		\param squaredRadius model max radius (squared)
	**/
	static LocalModel* New(	CC_LOCAL_MODEL_TYPES type,
							Neighbourhood& subset,
							const CCVector3 &center,
							PointCoordinateType squaredRadius);

	//! Destructor
	virtual ~LocalModel() = default;

	//! Returns the model type
	virtual CC_LOCAL_MODEL_TYPES getType() const = 0;

	//! Returns the model center
	inline const CCVector3& getCenter() const { return m_modelCenter; }

	//! Returns the model max radius (squared)
	inline PointCoordinateType getSquareSize() const { return m_squaredRadius; }

	//! Compute the (unsigned) distance between a 3D point and this model
	/** \param[in] P the query point
		\param[out] nearestPoint returns the nearest point (optional)
		\return the (unsigned) distance (or NAN_VALUE if the computation failed)
	**/
	virtual ScalarType computeDistanceFromModelToPoint(const CCVector3* P, CCVector3* nearestPoint = nullptr) const = 0;

protected:

	//! Constructor
	/** \param center model "center"
		\param squaredRadius model max "radius" (squared)
	**/
	LocalModel(const CCVector3 &center, PointCoordinateType squaredRadius);

	//! Center
	CCVector3 m_modelCenter;

	//! Max radius (squared)
	PointCoordinateType m_squaredRadius;
};

}

#endif //LOCAL_MODEL_HEADER
