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

#ifndef CC_LIB_POINT_CLOUD_TPL_HEADER
#define CC_LIB_POINT_CLOUD_TPL_HEADER

//Local
#include "BoundingBox.h"
#include "GenericIndexedCloudPersist.h"
#include "ScalarField.h"

//STL
#include <vector>

namespace CCLib
{

	//! A storage-efficient point cloud structure that can also handle an unlimited number of scalar fields
	template<class T, typename StringType = const char*> class PointCloudTpl : public T
	{
		static_assert(
				std::is_base_of<GenericIndexedCloudPersist, T>::value, 
				"T must be a descendant of GenericIndexedCloudPersist"
				);
		
	public:
		//! Default constructor
		PointCloudTpl()
			: T()
			, m_currentPointIndex(0)
			, m_currentInScalarFieldIndex(-1)
			, m_currentOutScalarFieldIndex(-1)
		{}

		//! Alternate constructor with a name and ID
		PointCloudTpl(StringType name, unsigned ID)
			: T(name, ID)
			, m_currentPointIndex(0)
			, m_currentInScalarFieldIndex(-1)
			, m_currentOutScalarFieldIndex(-1)
		{}

		//! Default destructor
		virtual ~PointCloudTpl()
		{
			deleteAllScalarFields();
		}

		inline unsigned size() const override { return static_cast<unsigned>(m_points.size()); }

		void forEach(GenericCloud::genericPointAction action) override
		{
			//there's no point of calling forEach if there's no activated scalar field!
			ScalarField* currentOutScalarFieldArray = getCurrentOutScalarField();
			if (!currentOutScalarFieldArray)
			{
				assert(false);
				return;
			}

			unsigned n = size();
			for (unsigned i = 0; i < n; ++i)
			{
				action(m_points[i], (*currentOutScalarFieldArray)[i]);
			}
		}

		void getBoundingBox(CCVector3& bbMin, CCVector3& bbMax) override
		{
			if (!m_bbox.isValid())
			{
				m_bbox.clear();
				for (const CCVector3& P : m_points)
				{
					m_bbox.add(P);
				}
			}

			bbMin = m_bbox.minCorner();
			bbMax = m_bbox.maxCorner();
		}

		void placeIteratorAtBeginning() override { m_currentPointIndex = 0; }
		
		const CCVector3* getNextPoint() override { return (m_currentPointIndex < m_points.size() ? point(m_currentPointIndex++) : 0); }
		
		bool enableScalarField() override
		{
			if (m_points.empty() && m_points.capacity() == 0)
			{
				//on must call resize or reserve on the cloud first
				return false;
			}

			ScalarField* currentInScalarField = getCurrentInScalarField();

			if (!currentInScalarField)
			{
				//if we get there, it means that either the caller has forgot to create
				//(and assign) a scalar field to the cloud, or that we are in a compatibility
				//mode with old/basic behaviour: a unique SF for everything (input/output)

				//we look for any already existing "default" scalar field 
				m_currentInScalarFieldIndex = getScalarFieldIndexByName("Default");
				if (m_currentInScalarFieldIndex < 0)
				{
					//if not, we create it
					m_currentInScalarFieldIndex = addScalarField("Default");
					if (m_currentInScalarFieldIndex < 0) //Something went wrong
					{
						return false;
					}
				}

				currentInScalarField = getCurrentInScalarField();
				assert(currentInScalarField);
			}

			//if there's no output scalar field either, we set this new scalar field as output also
			if (!getCurrentOutScalarField())
			{
				m_currentOutScalarFieldIndex = m_currentInScalarFieldIndex;
			}

			if (m_points.empty())
			{
				//if the cloud is empty, with a reserved capacity, we do the same on the SF
				return currentInScalarField->reserveSafe(m_points.capacity());
			}
			else
			{
				//otherwise we resize the SF with the current number of points so that they match
				return currentInScalarField->resizeSafe(m_points.size());
			}
		}
		
		bool isScalarFieldEnabled() const override
		{
			ScalarField* currentInScalarFieldArray = getCurrentInScalarField();
			if (!currentInScalarFieldArray)
			{
				return false;
			}

			std::size_t sfValuesCount = currentInScalarFieldArray->size();
			return (sfValuesCount != 0 && sfValuesCount >= m_points.size());
		}

		void setPointScalarValue(unsigned pointIndex, ScalarType value) override
		{
			assert(m_currentInScalarFieldIndex >= 0 && m_currentInScalarFieldIndex < static_cast<int>(m_scalarFields.size()));
			//slow version
			//ScalarField* currentInScalarFieldArray = getCurrentInScalarField();
			//if (currentInScalarFieldArray)
			//	currentInScalarFieldArray->setValue(pointIndex,value);

			//fast version
			m_scalarFields[m_currentInScalarFieldIndex]->setValue(pointIndex, value);
		}
		
		ScalarType getPointScalarValue(unsigned pointIndex) const override
		{
			assert(m_currentOutScalarFieldIndex >= 0 && m_currentOutScalarFieldIndex < static_cast<int>(m_scalarFields.size()));

			return m_scalarFields[m_currentOutScalarFieldIndex]->getValue(pointIndex);
		}

		inline const CCVector3* getPoint(unsigned index) const override { return point(index); }
		inline void getPoint(unsigned index, CCVector3& P) const override { P = *point(index); }

		inline const CCVector3* getPointPersistentPtr(unsigned index) const override { return point(index); }

		//! Resizes the point database
		/** The cloud database is resized with the specified size. If the new size
			is smaller, the overflooding points will be deleted. If its greater,
			the database is filled with blank points (warning, the
			PointCloud::addPoint method will insert points after those ones).
			\param newNumberOfPoints the new number of points
			\return true if the method succeeds, false otherwise
		**/
		virtual bool resize(unsigned newCount)
		{
			std::size_t oldCount = m_points.size();

			//we try to enlarge the 3D points array
			try
			{
				m_points.resize(newCount);
			}
			catch (const std::bad_alloc&)
			{
				return false;
			}

			//then the scalar fields
			for (std::size_t i = 0; i < m_scalarFields.size(); ++i)
			{
				if (!m_scalarFields[i]->resizeSafe(newCount))
				{
					//if something fails, we restore the previous size for already processed SFs!
					for (std::size_t j = 0; j < i; ++j)
					{
						m_scalarFields[j]->resize(oldCount);
						m_scalarFields[j]->computeMinAndMax();
					}
					//we can assume that newCount > oldNumberOfPoints, so it should always be ok
					m_points.resize(oldCount);
					return false;
				}
				m_scalarFields[i]->computeMinAndMax();
			}

			return true;
		}

		//! Reserves memory for the point database
		/** This method tries to reserve some memory to store points
			that will be inserted later (with PointCloud::addPoint).
			If the new number of points is smaller than the actual one,
			nothing happens.
			\param newNumberOfPoints the new number of points
			\return true if the method succeeds, false otherwise
		**/
		virtual bool reserve(unsigned newCapacity)
		{
			//we try to enlarge the 3D points array
			try
			{
				m_points.reserve(newCapacity);
			}
			catch (const std::bad_alloc&)
			{
				return false;
			}

			//then the scalar fields
			for (std::size_t i = 0; i < m_scalarFields.size(); ++i)
			{
				if (!m_scalarFields[i]->reserveSafe(newCapacity))
					return false;
			}

			//double check
			return (m_points.capacity() >= newCapacity);
		}

		//! Clears the cloud database
		/** Equivalent to resize(0).
		**/
		void reset()
		{
			m_points.resize(0);
			deleteAllScalarFields();
			placeIteratorAtBeginning();
			invalidateBoundingBox();
		}

		//! Adds a 3D point to the database
		/** To assure the best efficiency, the database memory must have already
			been reserved (with PointCloud::reserve). Otherwise nothing
			happens.
			\param P a 3D point
		**/
		void addPoint(const CCVector3 &P)
		{
			//NaN coordinates check
			if (	P.x != P.x
				||	P.y != P.y
				||	P.z != P.z)
			{
				//replace NaN point by (0, 0, 0)
				CCVector3 fakeP(0, 0, 0);
				m_points.push_back(fakeP);
			}
			else
			{
				m_points.push_back(P);
			}

			m_bbox.setValidity(false);
		}

		//! Invalidates bounding box
		/** Bounding box will be recomputed next time a request is made to 'getBoundingBox'.
		**/
		virtual void invalidateBoundingBox() { m_bbox.setValidity(false); }

		/*** scalar fields management ***/

		//! Returns the number of associated (and active) scalar fields
		/** \return the number of active scalar fields
		**/
		inline unsigned getNumberOfScalarFields() const { return static_cast<unsigned>(m_scalarFields.size()); }

		//! Returns a pointer to a specific scalar field
		/** \param index a scalar field index
			\return a pointer to a ScalarField structure, or 0 if the index is invalid.
		**/
		ScalarField* getScalarField(int index) const
		{
			return (index >= 0 && index < static_cast<int>(m_scalarFields.size()) ? m_scalarFields[index] : 0);
		}

		//! Returns the name of a specific scalar field
		/** \param index a scalar field index
			\return a pointer to a string structure (null-terminated array of characters), or 0 if the index is invalid.
		**/
		const char* getScalarFieldName(int index) const
		{
			return (index >= 0 && index < static_cast<int>(m_scalarFields.size()) ? m_scalarFields[index]->getName() : 0);
		}

		//! Returns the index of a scalar field represented by its name
		/** \param name a scalar field name
			\return an index (-1 if the scalar field couldn't be found)
		**/
		int getScalarFieldIndexByName(const char* name) const
		{
			std::size_t sfCount = m_scalarFields.size();
			for (std::size_t i = 0; i < sfCount; ++i)
			{
				//we don't accept two SF with the same name!
				if (strcmp(m_scalarFields[i]->getName(), name) == 0)
					return static_cast<int>(i);
			}

			return -1;
		}

		//! Returns the scalar field currently associated to the cloud input
		/** See PointCloud::setPointScalarValue.
			\return a pointer to the currently defined INPUT scalar field (or 0 if none)
		**/
		inline ScalarField* getCurrentInScalarField() const { return getScalarField(m_currentInScalarFieldIndex); }

		//! Returns the scalar field currently associated to the cloud output
		/** See PointCloud::getPointScalarValue.
			\return a pointer to the currently defined OUTPUT scalar field (or 0 if none)
		**/
		inline ScalarField* getCurrentOutScalarField() const { return getScalarField(m_currentOutScalarFieldIndex); }

		//! Sets the INPUT scalar field
		/** This scalar field will be used by the PointCloud::setPointScalarValue method.
			\param index a scalar field index (or -1 if none)
		**/
		inline void setCurrentInScalarField(int index) { m_currentInScalarFieldIndex = index; }

		//! Returns current INPUT scalar field index (or -1 if none)
		inline int getCurrentInScalarFieldIndex() { return m_currentInScalarFieldIndex; }

		//! Sets the OUTPUT scalar field
		/** This scalar field will be used by the PointCloud::getPointScalarValue method.
			\param index a scalar field index (or -1 if none)
		**/
		inline void setCurrentOutScalarField(int index) { m_currentOutScalarFieldIndex = index; }

		//! Returns current OUTPUT scalar field index (or -1 if none)
		inline int getCurrentOutScalarFieldIndex() { return m_currentOutScalarFieldIndex; }

		//! Sets both the INPUT & OUTPUT scalar field
		/** This scalar field will be used by both PointCloud::getPointScalarValue
			and PointCloud::setPointScalarValue methods.
			\param index a scalar field index
		**/
		inline void setCurrentScalarField(int index) { setCurrentInScalarField(index); setCurrentOutScalarField(index); }

		//! Creates a new scalar field and registers it
		/** Warnings:
			- the name must be unique (the method will fail if a SF with the same name already exists)
			- this method DOES resize the scalar field to match the current cloud size
			\param uniqueName scalar field name (must be unique)
			\return index of this new scalar field (or -1 if an error occurred)
		**/
		virtual int addScalarField(const char* uniqueName)
		{
			//we don't accept two SF with the same name!
			if (getScalarFieldIndexByName(uniqueName) >= 0)
			{
				return -1;
			}

			//create requested scalar field
			ScalarField* sf = new ScalarField(uniqueName);
			if (!sf || (size() && !sf->resizeSafe(m_points.size())))
			{
				//Not enough memory!
				if (sf)
					sf->release();
				return -1;
			}

			try
			{
				//we don't want 'm_scalarFields' to grow by 50% each time! (default behavior of std::vector::push_back)
				m_scalarFields.resize(m_scalarFields.size() + 1, sf);
			}
			catch (const std::bad_alloc&) //out of memory
			{
				sf->release();
				return -1;
			}

			return static_cast<int>(m_scalarFields.size()) - 1;
		}

		//! Renames a specific scalar field
		/** Warning: name must not be already given to another SF!
			\param index scalar field index
			\param newName new name
			\return success
		**/
		bool renameScalarField(int index, const char* newName)
		{
			if (getScalarFieldIndexByName(newName) < 0)
			{
				ScalarField* sf = getScalarField(index);
				if (sf)
				{
					sf->setName(newName);
					return true;
				}
			}
			return false;
		}

		//! Deletes a specific scalar field
		/** WARNING: this operation may modify the scalar fields order
			(especially if the deleted SF is not the last one). However
			current IN & OUT scalar fields will stay up-to-date (while
			their index may change).
			\param index index of scalar field to be deleted
		**/
		virtual void deleteScalarField(int index)
		{
			int sfCount = static_cast<int>(m_scalarFields.size());
			if (index < 0 || index >= sfCount)
				return;

			//we update SF roles if they point to the deleted scalar field
			if (index == m_currentInScalarFieldIndex)
				m_currentInScalarFieldIndex = -1;
			if (index == m_currentOutScalarFieldIndex)
				m_currentOutScalarFieldIndex = -1;

			//if the deleted SF is not the last one, we swap it with the last element
			int lastIndex = sfCount - 1; //lastIndex>=0
			if (index < lastIndex) //i.e.lastIndex>0
			{
				std::swap(m_scalarFields[index], m_scalarFields[lastIndex]);
				//don't forget to update SF roles also if they point to the last element
				if (lastIndex == m_currentInScalarFieldIndex)
					m_currentInScalarFieldIndex = index;
				if (lastIndex == m_currentOutScalarFieldIndex)
					m_currentOutScalarFieldIndex = index;
			}

			//we can always delete the last element (and the vector stays consistent)
			m_scalarFields.back()->release();
			m_scalarFields.pop_back();
		}

		//! Deletes all scalar fields associated to this cloud
		virtual void deleteAllScalarFields()
		{
			m_currentInScalarFieldIndex = m_currentOutScalarFieldIndex = -1;

			while (!m_scalarFields.empty())
			{
				m_scalarFields.back()->release();
				m_scalarFields.pop_back();
			}
		}

		//! Returns cloud capacity (i.e. reserved size)
		inline unsigned capacity() const { return static_cast<unsigned>(m_points.capacity()); }

	protected:
		//! Swaps two points (and their associated scalar values!)
		virtual void swapPoints(unsigned firstIndex, unsigned secondIndex)
		{
			if (firstIndex == secondIndex
				|| firstIndex >= m_points.size()
				|| secondIndex >= m_points.size())
			{
				return;
			}

			std::swap(m_points[firstIndex], m_points[secondIndex]);

			for (std::size_t i = 0; i < m_scalarFields.size(); ++i)
			{
				m_scalarFields[i]->swap(firstIndex, secondIndex);
			}
		}

		//! Returns non const access to a given point
		/** WARNING: index must be valid
			\param index point index
			\return pointer on point stored data
		**/
		inline CCVector3* point(unsigned index) { assert(index < size()); return &(m_points[index]); }

		//! Returns const access to a given point
		/** WARNING: index must be valid
			\param index point index
			\return pointer on point stored data
		**/
		inline const CCVector3* point(unsigned index) const { assert(index < size()); return &(m_points[index]); }

		//! 3D Points database
		std::vector<CCVector3> m_points;

		//! Bounding-box
		BoundingBox m_bbox;

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

#endif //CC_LIB_POINT_CLOUD_TPL_HEADER
