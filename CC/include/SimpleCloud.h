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

#ifndef SIMPLE_CLOUD_HEADER
#define SIMPLE_CLOUD_HEADER

//Local
#include "GenericChunkedArray.h"
#include "GenericIndexedCloudPersist.h"
#include "PointProjectionTools.h"

namespace CCLib
{

class PointsContainer;
class ScalarField;

//! A very simple point cloud (with point duplication mechanism)
/** Implements the GenericIndexedCloud interface. A simple point cloud
	that stores its own point instances and distances in a vector.
**/
class CC_CORE_LIB_API SimpleCloud : public GenericIndexedCloudPersist
{
public:

	//! The SimpleCloud constructor.
	SimpleCloud();

	//! The SimpleCloud destructor.
	virtual ~SimpleCloud();

	//**** inherited form GenericCloud ****//
	virtual unsigned size() const;
	virtual void forEach(genericPointAction action);
	virtual void getBoundingBox(CCVector3& bbMin, CCVector3& bbMax);
	virtual void placeIteratorAtBeginning();
	virtual const CCVector3* getNextPoint();
	virtual bool enableScalarField();
	virtual bool isScalarFieldEnabled() const;
	virtual void setPointScalarValue(unsigned pointIndex, ScalarType value);
	virtual ScalarType getPointScalarValue(unsigned pointIndex) const;

	//**** inherited form GenericIndexedCloud ****//
	inline virtual const CCVector3* getPoint(unsigned index) {return getPointPersistentPtr(index);}
	virtual void getPoint(unsigned index, CCVector3& P) const;

	//**** inherited form GenericIndexedCloudPersist ****//
	virtual const CCVector3* getPointPersistentPtr(unsigned index);

	//! Clears cloud
	void clear();

	//! Point insertion mechanism
	/** The point data will be duplicated in memory.
		\param P the point to insert
	**/
	virtual void addPoint(const CCVector3 &P);

	//! Point insertion mechanism
	/** The point data will be duplicated in memory.
		\param P the point to insert (as a 3-size array)
	**/
	virtual void addPoint(const PointCoordinateType P[]);

	//! Reserves some memory for hosting the points
	/** \param n the number of points
	**/
	virtual bool reserve(unsigned n);

	//! Presets the size of the vector used to store the points
	/** \param n the number of points
	**/
	virtual bool resize(unsigned n);

	//! Applies a rigid transformation to the cloud
	/** WARNING: THIS METHOD IS NOT COMPATIBLE WITH PARALLEL STRATEGIES
		\param trans transformation (scale * rotation matrix + translation vector)
	**/
	virtual void applyTransformation(PointProjectionTools::Transformation& trans);


	//! Returns associated scalar field (if any)
	ScalarField* getScalarField() { return m_scalarField; } 

	//! Returns associated scalar field (if any) (const version)
	const ScalarField* getScalarField() const { return m_scalarField; } 

protected:

	//! Point container
	typedef GenericChunkedArray<3,PointCoordinateType> PointsContainer;

	//! 3D Points container
	PointsContainer* m_points;

	//! The points distances
	ScalarField* m_scalarField;

	//! Iterator on the points container
	unsigned globalIterator;

	//! Bounding-box validity
	bool m_validBB;
};

}

#endif //SIMPLE_CLOUD_HEADER
