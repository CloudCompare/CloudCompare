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

#ifndef DGM_OCTREE_REFERENCE_CLOUD_HEADER
#define DGM_OCTREE_REFERENCE_CLOUD_HEADER

//Local
#include "DgmOctree.h"
#include "GenericIndexedCloudPersist.h"


namespace CCLib
{

//! A kind of ReferenceCloud based on the DgmOctree::NeighboursSet structure
class CC_CORE_LIB_API DgmOctreeReferenceCloud : public GenericIndexedCloudPersist
{
public:

	//! Default constructor.
	/** \param associatedSet associated NeighboursSet
		\param count number of values to use (0 = all)
	**/
	DgmOctreeReferenceCloud(DgmOctree::NeighboursSet* associatedSet, unsigned count = 0);

	//**** inherited form GenericCloud ****//
	inline unsigned size() const override { return m_size; }
	void forEach(genericPointAction action) override;
	void getBoundingBox(CCVector3& bbMin, CCVector3& bbMax) override;
	//virtual unsigned char testVisibility(const CCVector3& P) const; //not supported
	inline void placeIteratorAtBeginning() override { m_globalIterator = 0; }
	inline const CCVector3* getNextPoint() override { return (m_globalIterator < size() ? m_set->at(m_globalIterator++).point : nullptr); }
	inline bool enableScalarField() override { return true; } //use DgmOctree::PointDescriptor::squareDistd by default
	inline bool isScalarFieldEnabled() const override { return true; } //use DgmOctree::PointDescriptor::squareDistd by default
	inline void setPointScalarValue(unsigned pointIndex, ScalarType value) override { assert(pointIndex < size()); m_set->at(pointIndex).squareDistd = static_cast<double>(value); }
	inline ScalarType getPointScalarValue(unsigned pointIndex) const override { assert(pointIndex < size()); return static_cast<ScalarType>(m_set->at(pointIndex).squareDistd); }
	//**** inherited form GenericIndexedCloud ****//
	inline const CCVector3* getPoint(unsigned index) const override { assert(index < size()); return m_set->at(index).point; }
	inline void getPoint(unsigned index, CCVector3& P) const override { assert(index < size()); P = *m_set->at(index).point; }
	//**** inherited form GenericIndexedCloudPersist ****//
	inline const CCVector3* getPointPersistentPtr(unsigned index) const override { assert(index < size()); return m_set->at(index).point; }

	//! Forwards global iterator
	inline void forwardIterator() { ++m_globalIterator; }

protected:

	//! Computes the cloud bounding-box (internal)
	virtual void computeBB();

	//! Iterator on the point references container
	unsigned m_globalIterator;

	//! Bounding-box min corner
	CCVector3 m_bbMin;
	//! Bounding-box max corner
	CCVector3 m_bbMax;
	//! Bounding-box validity
	bool m_validBB;

	//! Associated PointDescriptor set
	DgmOctree::NeighboursSet* m_set;

	//! Number of points
	unsigned m_size;
};

}

#endif //DGM_OCTREE_REFERENCE_CLOUD_HEADER
