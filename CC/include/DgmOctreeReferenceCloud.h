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

#ifndef DGM_OCTREE_REFERENCE_CLOUD_HEADER
#define DGM_OCTREE_REFERENCE_CLOUD_HEADER

#ifdef _MSC_VER
//To get rid of the really annoying warnings about template class exportation
#pragma warning( disable: 4251 )
#pragma warning( disable: 4530 )
#endif

#include "GenericIndexedCloudPersist.h"
#include "GenericChunkedArray.h"
#include "DgmOctree.h"

namespace CCLib
{

//! A kind of ReferenceCloud based on the DgmOctree::NeighboursSet structure
#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"

class CC_DLL_API DgmOctreeReferenceCloud : public GenericIndexedCloudPersist
#else
class DgmOctreeReferenceCloud : public GenericIndexedCloudPersist
#endif
{
public:

	//! Default constructor.
	/** \param associatedSet associated NeighboursSet
		\param count number of values to use (0 = all)
	**/
	DgmOctreeReferenceCloud(DgmOctree::NeighboursSet* associatedSet, unsigned size=0);

	//! Destructor.
	virtual ~DgmOctreeReferenceCloud();

	//**** inherited form GenericCloud ****//
	virtual unsigned size() const;
	virtual void forEach(genericPointAction& anAction);
	virtual void getBoundingBox(PointCoordinateType Mins[], PointCoordinateType Maxs[]);
	//virtual CC_VISIBILITY_TYPE testVisibility(const CCVector3& P) const; //not supported
	virtual void placeIteratorAtBegining();
	virtual const CCVector3* getNextPoint();
	virtual bool enableScalarField() {return true;} //use DgmOctree::PointDescriptor::squareDist by default
	virtual bool isScalarFieldEnabled() const {return true;} //use DgmOctree::PointDescriptor::squareDist by default
	virtual void setPointScalarValue(unsigned pointIndex, DistanceType value); //use DgmOctree::PointDescriptor::squareDist by default
	virtual DistanceType getPointScalarValue(unsigned pointIndex) const; //use DgmOctree::PointDescriptor::squareDist by default

	//**** inherited form GenericIndexedCloud ****//
	virtual const CCVector3* getPoint(unsigned index);
	virtual void getPoint(unsigned index, CCVector3& P) const;

	//**** inherited form GenericIndexedCloudPersist ****//
	virtual const CCVector3* getPointPersistentPtr(unsigned index);

	//! Forwards global iterator
	void forwardIterator();

protected:

	//! Computes the cloud bounding-box (internal)
	virtual void computeBB();

	//! Updates the bounding-box informations with a new point data (internal)
	/** P the new point
	**/
	virtual void updateBBWithPoint(const CCVector3* P);

	//! Iterator on the point references container
	unsigned m_globalIterator;

	//! Cloud bounding-box (min along the 3 dimensions)
	PointCoordinateType m_bbMins[3];

	//! Cloud bounding-box (max along the 3 dimensions)
	PointCoordinateType m_bbMaxs[3];

	//! Bounding-box validity
	bool m_validBB;

	//! Associated PointDescriptor set
	DgmOctree::NeighboursSet* m_set;

	//! Number of points
	unsigned m_size;
};

}

#endif
