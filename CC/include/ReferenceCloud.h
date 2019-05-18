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
#include "BoundingBox.h"

//System
#include <cassert>
#include <vector>
#include <mutex>
#include <atomic>

namespace CCLib
{

//! A very simple point cloud (no point duplication)
/** Implements the GenericIndexedCloudPersist interface. A simple point cloud
	that stores references to Generic3dPoint instances in a vector.

	Partial thread safety for all methods that can change the size of the cloud
	or that change or rely on point ordering.
**/
class CC_CORE_LIB_API ReferenceCloud : public GenericIndexedCloudPersist
{
public:

	//! Default constructor
	explicit ReferenceCloud(GenericIndexedCloudPersist* associatedCloud);

	//! Copy constructor
	ReferenceCloud(const ReferenceCloud& refCloud);

	//! Destructor
	~ReferenceCloud() override = default;

	//**** inherited form GenericCloud ****//
	inline unsigned size() const override { return static_cast<unsigned>(m_theIndexes.size()); }
	void forEach(genericPointAction action) override;
	void getBoundingBox(CCVector3& bbMin, CCVector3& bbMax) override;
	inline unsigned char testVisibility(const CCVector3& P) const override { assert(m_theAssociatedCloud); return m_theAssociatedCloud->testVisibility(P); }
	inline void placeIteratorAtBeginning() override { m_globalIterator = 0; }
	inline const CCVector3* getNextPoint() override { assert(m_theAssociatedCloud); return (m_globalIterator < size() ? m_theAssociatedCloud->getPoint(m_theIndexes[m_globalIterator++]) : nullptr); }
	inline bool enableScalarField() override { assert(m_theAssociatedCloud); return m_theAssociatedCloud->enableScalarField(); }
	inline bool isScalarFieldEnabled() const override { assert(m_theAssociatedCloud); return m_theAssociatedCloud->isScalarFieldEnabled(); }
	inline void setPointScalarValue(unsigned pointIndex, ScalarType value) override { assert(m_theAssociatedCloud && pointIndex < size()); m_theAssociatedCloud->setPointScalarValue(m_theIndexes[pointIndex], value); }
	inline ScalarType getPointScalarValue(unsigned pointIndex) const override { assert(m_theAssociatedCloud && pointIndex < size()); return m_theAssociatedCloud->getPointScalarValue(m_theIndexes[pointIndex]); }

	//**** inherited form GenericIndexedCloud ****//
	inline const CCVector3* getPoint(unsigned index) const override { assert(m_theAssociatedCloud && index < size()); return m_theAssociatedCloud->getPoint(m_theIndexes[index]); }
	inline void getPoint(unsigned index, CCVector3& P) const override { assert(m_theAssociatedCloud && index < size()); m_theAssociatedCloud->getPoint(m_theIndexes[index], P); }

	//**** inherited form GenericIndexedCloudPersist ****//
	inline const CCVector3* getPointPersistentPtr(unsigned index) const override { assert(m_theAssociatedCloud && index < size()); return m_theAssociatedCloud->getPointPersistentPtr(m_theIndexes[index]); }

	//! Returns global index (i.e. relative to the associated cloud) of a given element
	/** \param localIndex local index (i.e. relative to the internal index container)
	**/
	inline virtual unsigned getPointGlobalIndex(unsigned localIndex) const { return m_theIndexes[localIndex]; }

	//! Returns the coordinates of the point pointed by the current element
	/** Returns a persistent pointer.
	**/
	virtual const CCVector3* getCurrentPointCoordinates() const;

	//! Returns the global index of the point pointed by the current element
	inline virtual unsigned getCurrentPointGlobalIndex() const { assert(m_globalIterator < size()); return m_theIndexes[m_globalIterator]; }

    //! Returns the current point associated scalar value
	inline virtual ScalarType getCurrentPointScalarValue() const { assert(m_theAssociatedCloud && m_globalIterator<size()); return m_theAssociatedCloud->getPointScalarValue(m_theIndexes[m_globalIterator]); }

	//! Sets the current point associated scalar value
	inline virtual void setCurrentPointScalarValue(ScalarType value) { assert(m_theAssociatedCloud && m_globalIterator < size()); m_theAssociatedCloud->setPointScalarValue(m_theIndexes[m_globalIterator], value); }

	//! Forwards the local element iterator
	inline virtual void forwardIterator() { ++m_globalIterator; }

	//! Clears the cloud
	/** Thread safe.
	**/
	virtual void clear(bool releaseMemory = false);

	//! Point global index insertion mechanism
	/** \param globalIndex a point global index
		\return false if not enough memory
		Thread safe.
	**/
	virtual bool addPointIndex(unsigned globalIndex);

	//! Point global index insertion mechanism (range)
	/** \param firstIndex first point global index of range
		\param lastIndex last point global index of range (excluded)
		\return false if not enough memory
		Thread safe.
	**/
	virtual bool addPointIndex(unsigned firstIndex, unsigned lastIndex);

	//! Sets global index for a given element
	/** \param localIndex local index
        \param globalIndex global index
	**/
	virtual void setPointIndex(unsigned localIndex, unsigned globalIndex);

	//! Reserves some memory for hosting the point references
	/** \param n the number of points (references)
		Thread safe.
	**/
	virtual bool reserve(unsigned n);

	//! Presets the size of the vector used to store point references
	/** \param n the number of points (references)
		Thread safe.
	**/
	virtual bool resize(unsigned n);

	//! Returns max capacity
	inline virtual unsigned capacity() const { return static_cast<unsigned>(m_theIndexes.capacity()); }

	//! Swaps two point references
	/** the point references indexes should be smaller than the total
		number of "reserved" points (see ReferenceCloud::reserve).
		\param i the first point index
		\param j the second point index
		Thread safe.
	**/
	inline virtual void swap(unsigned i, unsigned j) { m_mutex.lock(); std::swap(m_theIndexes[i], m_theIndexes[j]); m_mutex.unlock(); }

	//! Removes current element
	/** WARNING: this method changes the cloud size!
		Thread safe.
	**/
	inline virtual void removeCurrentPointGlobalIndex() { removePointGlobalIndex(m_globalIterator); }

	//! Removes a given element
	/** WARNING: this method changes the cloud size!
		Thread safe.
	**/
	virtual void removePointGlobalIndex(unsigned localIndex);

    //! Returns the associated (source) cloud
	inline virtual GenericIndexedCloudPersist* getAssociatedCloud() { return m_theAssociatedCloud; }

    //! Returns the associated (source) cloud (const version)
	inline virtual const GenericIndexedCloudPersist* getAssociatedCloud() const { return m_theAssociatedCloud; }

    //! Sets the associated (source) cloud
	virtual void setAssociatedCloud(GenericIndexedCloudPersist* cloud);

	//! Add another reference cloud
	/** \warning Both clouds should have the same reference cloud!
		\warning No verification for duplicates!
		Thread safe.
	**/
	bool add(const ReferenceCloud& cloud);
	
	//! Invalidates the bounding-box
	inline void invalidateBoundingBox() { m_bbox.setValidity(false); }

protected:

	//! Container of 3D point indexes
	using ReferencesContainer = std::vector<unsigned int>;

	//! Indexes of (some of) the associated cloud points
	ReferencesContainer m_theIndexes;

	//! Iterator on the point references container
	std::atomic<unsigned> m_globalIterator;

	//! Bounding-box
	BoundingBox m_bbox;

	//! Associated cloud
	/** The cloud from which references are referring to.
		WARNING: do not use the inner iterator as it
		is used to 'implement' the ReferenceCloud one.
	**/
	GenericIndexedCloudPersist* m_theAssociatedCloud;

	//! For concurrent access
	std::mutex m_mutex;
};

}

#endif //REFERENCE_CLOUD_HEADER
