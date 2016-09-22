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

#ifndef REFERENCE_CLOUD_HEADER
#define REFERENCE_CLOUD_HEADER

//Local
#include "GenericIndexedCloudPersist.h"
#include "GenericChunkedArray.h"

namespace CCLib
{

//! A very simple point cloud (no point duplication)
/** Implements the GenericIndexedCloudPersist interface. A simple point cloud
	that stores references to Generic3dPoint instances in a vector.
**/
class CC_CORE_LIB_API ReferenceCloud : public GenericIndexedCloudPersist
{
public:

	//! Default constructor
	explicit ReferenceCloud(GenericIndexedCloudPersist* associatedCloud);

	//! Copy constructor
	ReferenceCloud(const ReferenceCloud& refCloud);

	//! Destructor
	virtual ~ReferenceCloud();

	//**** inherited form GenericCloud ****//
	inline virtual unsigned size() const { return m_theIndexes->currentSize(); }
	virtual void forEach(genericPointAction& action);
	virtual void getBoundingBox(CCVector3& bbMin, CCVector3& bbMax);
	inline virtual unsigned char testVisibility(const CCVector3& P) const { assert(m_theAssociatedCloud); return m_theAssociatedCloud->testVisibility(P); }
	inline virtual void placeIteratorAtBegining() { m_globalIterator = 0; }
	inline virtual const CCVector3* getNextPoint() { assert(m_theAssociatedCloud); return (m_globalIterator < size() ? m_theAssociatedCloud->getPoint(m_theIndexes->getValue(m_globalIterator++)) : 0); }
	inline virtual bool enableScalarField() { assert(m_theAssociatedCloud); return m_theAssociatedCloud->enableScalarField(); }
	inline virtual bool isScalarFieldEnabled() const { assert(m_theAssociatedCloud); return m_theAssociatedCloud->isScalarFieldEnabled(); }
	inline virtual void setPointScalarValue(unsigned pointIndex, ScalarType value) { assert(m_theAssociatedCloud && pointIndex<size()); m_theAssociatedCloud->setPointScalarValue(m_theIndexes->getValue(pointIndex),value); }
	inline virtual ScalarType getPointScalarValue(unsigned pointIndex) const { assert(m_theAssociatedCloud && pointIndex<size()); return m_theAssociatedCloud->getPointScalarValue(m_theIndexes->getValue(pointIndex)); }

	//**** inherited form GenericIndexedCloud ****//
	inline virtual const CCVector3* getPoint(unsigned index) { assert(m_theAssociatedCloud && index < size()); return m_theAssociatedCloud->getPoint(m_theIndexes->getValue(index)); }
	inline virtual void getPoint(unsigned index, CCVector3& P) const { assert(m_theAssociatedCloud && index < size()); m_theAssociatedCloud->getPoint(m_theIndexes->getValue(index),P); }

	//**** inherited form GenericIndexedCloudPersist ****//
	inline virtual const CCVector3* getPointPersistentPtr(unsigned index) { assert(m_theAssociatedCloud && index < size()); return m_theAssociatedCloud->getPointPersistentPtr(m_theIndexes->getValue(index)); }

	//! Returns global index (i.e. relative to the associated cloud) of a given element
	/** \param localIndex local index (i.e. relative to the internal index container)
	**/
	inline virtual unsigned getPointGlobalIndex(unsigned localIndex) const { return m_theIndexes->getValue(localIndex); }

	//! Returns the coordinates of the point pointed by the current element
	/** Returns a persistent pointer.
	**/
	virtual const CCVector3* getCurrentPointCoordinates() const;

	//! Returns the global index of the point pointed by the current element
	inline virtual unsigned getCurrentPointGlobalIndex() const { assert(m_globalIterator < size()); return m_theIndexes->getValue(m_globalIterator); }

    //! Returns the current point associated scalar value
	inline virtual ScalarType getCurrentPointScalarValue() const { assert(m_theAssociatedCloud && m_globalIterator<size()); return m_theAssociatedCloud->getPointScalarValue(m_theIndexes->getValue(m_globalIterator)); }

	//! Sets the current point associated scalar value
	inline virtual void setCurrentPointScalarValue(ScalarType value) { assert(m_theAssociatedCloud && m_globalIterator<size()); m_theAssociatedCloud->setPointScalarValue(m_theIndexes->getValue(m_globalIterator),value); }

	//! Forwards the local element iterator
	inline virtual void forwardIterator() { ++m_globalIterator; }

	//! Clears the cloud
	virtual void clear(bool releaseMemory);

	//! Point global index insertion mechanism
	/** \param globalIndex a point global index
		\return false if not enough memory
	**/
	virtual bool addPointIndex(unsigned globalIndex);

	//! Point global index insertion mechanism (range)
	/** \param firstIndex first point global index of range
		\param lastIndex last point global index of range (excluded)
		\return false if not enough memory
	**/
	virtual bool addPointIndex(unsigned firstIndex, unsigned lastIndex);

	//! Sets global index for a given element
	/** \param localIndex local index
        \param globalIndex global index
	**/
	virtual void setPointIndex(unsigned localIndex, unsigned globalIndex);

	//! Reserves some memory for hosting the point references
	/** \param n the number of points (references)
	**/
	virtual bool reserve(unsigned n);

	//! Presets the size of the vector used to store point references
	/** \param n the number of points (references)
	**/
	virtual bool resize(unsigned n);

	//! Returns max capacity
	inline virtual unsigned capacity() const { return m_theIndexes->capacity(); }

	//! Swaps two point references
	/** the point references indexes should be smaller than the total
		number of "reserved" points (see ReferenceCloud::reserve).
		\param i the first point index
		\param j the second point index
	**/
	inline virtual void swap(unsigned i, unsigned j) {m_theIndexes->swap(i,j);}

	//! Removes current element
	/** WARNING: this method change the structure size!
	**/
	inline virtual void removeCurrentPointGlobalIndex() { removePointGlobalIndex(m_globalIterator); }

	//! Removes a given element
	/** WARNING: this method change the structure size!
	**/
	virtual void removePointGlobalIndex(unsigned localIndex);

    //! Returns the associated (source) cloud
	inline virtual GenericIndexedCloudPersist* getAssociatedCloud() { return m_theAssociatedCloud; }

    //! Returns the associated (source) cloud (const version)
	inline virtual const GenericIndexedCloudPersist* getAssociatedCloud() const { return m_theAssociatedCloud; }

    //! Sets the associated (source) cloud
	virtual void setAssociatedCloud(GenericIndexedCloudPersist* cloud);

	//! Add another reference cloud
	/** Warnings:
		- the clouds must have the same reference cloud!
		- no verification for duplicates!
	**/
	bool add(const ReferenceCloud& cloud);
	
	//! Invalidates the bounding-box
	inline void invalidateBoundingBox() { m_validBB = false; }

protected:

	//! Computes the cloud bounding-box (internal)
	virtual void computeBB();

	//! Updates the bounding-box with a new point (internal)
	/** P the new point
	**/
	virtual void updateBBWithPoint(const CCVector3& P);

	//! Container of 3D point indexes
	typedef GenericChunkedArray<1,unsigned> ReferencesContainer;

	//! Indexes of (some of) the associated cloud points
	ReferencesContainer* m_theIndexes;

	//! Iterator on the point references container
	unsigned m_globalIterator;

	//! Bounding-box min corner
	CCVector3 m_bbMin;
	//! Bounding-box max corner
	CCVector3 m_bbMax;
	//! Bounding-box validity
	bool m_validBB;

	//! Associated cloud
	/** The cloud from which references are refering to.
		WARNING: do not use the inner iterator as it
		is used to 'implement' the ReferenceCloud one.
	**/
	GenericIndexedCloudPersist* m_theAssociatedCloud;
};

}

#endif //REFERENCE_CLOUD_HEADER
