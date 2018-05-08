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

#ifndef CHUNKED_POINT_CLOUD_HEADER
#define CHUNKED_POINT_CLOUD_HEADER

//Local
#include "GenericChunkedArray.h"
#include "GenericIndexedCloudPersist.h"
#include "PointProjectionTools.h"


namespace CCLib
{

class ScalarField;

//! A storage-efficient point cloud structure that can also handle an unlimited number of scalar fields
/** This structure is based on the GenericChunkedArray structure and the
    GenericIndexedCloud interface. It can store more than 67M points thanks to it's
	"chunked" database (if there is enough free memory, of course). There is no
	512 Mo limit. On the counter part, the access to each point is slightly
	slower (but this shouldn't really be noticeable!).
**/
class CC_CORE_LIB_API ChunkedPointCloud : virtual public GenericIndexedCloudPersist
{
public:

		//! Default constructor
		ChunkedPointCloud();

		//! Default destructor
		virtual ~ChunkedPointCloud();

		//**** inherited form GenericCloud ****//
		inline virtual unsigned size() const override { return m_points->currentSize(); }
		virtual void forEach(genericPointAction action) override;
		virtual void getBoundingBox(CCVector3& bbMin, CCVector3& bbMax) override;
		virtual void placeIteratorAtBeginning() override;
		virtual const CCVector3* getNextPoint() override;
		virtual bool enableScalarField() override;
		virtual bool isScalarFieldEnabled() const override;
		virtual void setPointScalarValue(unsigned pointIndex, ScalarType value) override;
		virtual ScalarType getPointScalarValue(unsigned pointIndex) const override;

		//**** inherited form GenericIndexedCloud ****//
		inline virtual const CCVector3* getPoint(unsigned index) override { return point(index); }
		inline virtual void getPoint(unsigned index, CCVector3& P) const override { P = *point(index); }

		//**** inherited form GenericIndexedCloudPersist ****//
		inline virtual const CCVector3* getPointPersistentPtr(unsigned index) override { return point(index); }

		//**** other methods ****//

		//! Const version of getPoint
		inline virtual const CCVector3* getPoint(unsigned index) const { return point(index); }
		//! Const version of getPointPersistentPtr
		inline virtual const CCVector3* getPointPersistentPtr(unsigned index) const { return point(index); }

		//! Applies a rigid transformation to the cloud, for the scaled scale
		/** WARNING: THIS METHOD IS NOT COMPATIBLE WITH PARALLEL STRATEGIES
			\param trans transformation (scale * rotation matrix + translation vector)
		**/
		virtual void applyTransformation(PointProjectionTools::Transformation& trans);

		//! Resizes the point database
		/** The cloud database is resized with the specified size. If the new size
			is smaller, the overflooding points will be deleted. If its greater,
			the database is filled with blank points (warning, the
			ChunkedPointCloud::addPoint method will insert points after those ones).
			\param newNumberOfPoints the new number of points
			\return true if the method succeeds, false otherwise
		**/
		virtual bool resize(unsigned newNumberOfPoints);

		//! Reserves memory for the point database
		/** This method tries to reserve some memory to store points
			that will be inserted later (with ChunkedPointCloud::addPoint).
			If the new number of points is smaller than the actual one,
			nothing happens.
			\param newNumberOfPoints the new number of points
			\return true if the method succeeds, false otherwise
		**/
		virtual bool reserve(unsigned newNumberOfPoints);

		//! Clears the cloud database
		/** Equivalent to resize(0).
		**/
		virtual void clear();

		//! Adds a 3D point to the database
		/** To assure the best efficiency, the database memory must have already
			been reserved (with ChunkedPointCloud::reserve). Otherwise nothing
			happens.
			\param P a 3D point
		**/
		virtual void addPoint(const CCVector3 &P);

		//! Invalidates bounding box
		/** Bounding box will be recomputed next time a request is made to 'getBoundingBox'.
		**/
		virtual void invalidateBoundingBox();

		/*** scalar fields management ***/

		//! Returns the number of associated (and active) scalar fields
		/** \return the number of active scalar fields
		**/
		inline virtual unsigned getNumberOfScalarFields() const { return (unsigned)m_scalarFields.size(); }

		//! Returns a pointer to a specific scalar field
		/** \param index a scalar field index
			\return a pointer to a ScalarField structure, or 0 if the index is invalid.
		**/
		virtual ScalarField* getScalarField(int index) const;

		//! Returns the name of a specific scalar field
		/** \param index a scalar field index
			\return a pointer to a string structure (null-terminated array of characters), or 0 if the index is invalid.
		**/
		virtual const char* getScalarFieldName(int index) const;

		//! Returns the index of a scalar field represented by its name
		/** \param name a scalar field name
			\return an index (-1 if the scalar field couldn't be found)
		**/
		virtual int getScalarFieldIndexByName(const char* name) const;

		//! Returns the scalar field currently associated to the cloud input
		/** See ChunkedPointCloud::setPointScalarValue.
			\return a pointer to the currently defined INPUT scalar field (or 0 if none)
		**/
		inline virtual ScalarField* getCurrentInScalarField() const { return getScalarField(m_currentInScalarFieldIndex); }

		//! Returns the scalar field currently associated to the cloud output
		/** See ChunkedPointCloud::getPointScalarValue.
			\return a pointer to the currently defined OUTPUT scalar field (or 0 if none)
		**/
		inline virtual ScalarField* getCurrentOutScalarField() const { return getScalarField(m_currentOutScalarFieldIndex); }

		//! Sets the INPUT scalar field
		/** This scalar field will be used by the ChunkedPointCloud::setPointScalarValue method.
			\param index a scalar field index (or -1 if none)
		**/
		inline virtual void setCurrentInScalarField(int index) { m_currentInScalarFieldIndex=index; }

		//! Returns current INPUT scalar field index (or -1 if none)
		inline virtual int getCurrentInScalarFieldIndex() { return m_currentInScalarFieldIndex; }

		//! Sets the OUTPUT scalar field
		/** This scalar field will be used by the ChunkedPointCloud::getPointScalarValue method.
			\param index a scalar field index (or -1 if none)
		**/
		inline virtual void setCurrentOutScalarField(int index) { m_currentOutScalarFieldIndex=index; }

		//! Returns current OUTPUT scalar field index (or -1 if none)
		inline virtual int getCurrentOutScalarFieldIndex() { return m_currentOutScalarFieldIndex; }

		//! Sets both the INPUT & OUTPUT scalar field
		/** This scalar field will be used by both ChunkedPointCloud::getPointScalarValue
			and ChunkedPointCloud::setPointScalarValue methods.
			\param index a scalar field index
		**/
		inline virtual void setCurrentScalarField(int index) { setCurrentInScalarField(index);setCurrentOutScalarField(index); }

		//! Creates a new scalar field and registers it
		/** Warnings:
			- the name must be unique (the method will fail if a SF with the same name already exists)
			- this method DOES resize the scalar field to match the current cloud size
			\param uniqueName scalar field name (must be unique)
			\return index of this new scalar field (or -1 if an error occurred)
		**/
		virtual int addScalarField(const char* uniqueName);

		//! Renames a specific scalar field
		/** Warning: name must not be already given to another SF!
			\param index scalar field index
			\param newName new name
			\return success
		**/
		virtual bool renameScalarField(int index, const char* newName);

		//! Deletes a specific scalar field
		/** WARNING: this operation may modify the scalar fields order
			(especially if the deleted SF is not the last one). However
			current IN & OUT scalar fields will stay up-to-date (while
			their index may change).
			\param index index of scalar field to be deleted
		**/
		virtual void deleteScalarField(int index);

		//! Deletes all scalar fields associated to this cloud
		virtual void deleteAllScalarFields();

		//! Returns cloud capacity (i.e. reserved size)
		inline virtual unsigned capacity() const { return m_points->capacity(); }

protected:

		//! Swaps two points (and their associated scalar values!)
		virtual void swapPoints(unsigned firstIndex, unsigned secondIndex);

		//! Returns non const access to a given point
		/** WARNING: index must be valid
			\param index point index
			\return pointer on point stored data
		**/
		inline virtual CCVector3* point(unsigned index) { assert(index < size()); return reinterpret_cast<CCVector3*>(m_points->getValue(index)); }

		//! Returns const access to a given point
		/** WARNING: index must be valid
			\param index point index
			\return pointer on point stored data
		**/
		inline virtual const CCVector3* point(unsigned index) const { assert(index < size()); return reinterpret_cast<CCVector3*>(m_points->getValue(index)); }

		//! 3D Points database
		GenericChunkedArray<3,PointCoordinateType>* m_points;

		//! Bounding-box validity
		bool m_validBB;

		//! 'Iterator' on the points db
		unsigned m_currentPointIndex;

		//! Associated scalar fields
		std::vector<ScalarField*> m_scalarFields;

		//! Index of current scalar field used for input
		int m_currentInScalarFieldIndex;

		//! Index of current scalar field used for output
		int m_currentOutScalarFieldIndex;
};

}

#endif //CHUNKED_POINT_CLOUD_HEADER
