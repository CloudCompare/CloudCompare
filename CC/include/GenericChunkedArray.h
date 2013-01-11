#ifndef GENERIC_CHUNKED_ARRAY
#define GENERIC_CHUNKED_ARRAY

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

#ifdef _MSC_VER
//To get rid of the really annoying warnings about template class exportation
#pragma warning( disable: 4251 )
#pragma warning( disable: 4530 )
#endif

//Chunks can only be handled rapidly if their size is a power of 2
#define CHUNK_INDEX_BIT_DEC 16 //2^14 = 16384 - 2^15 = 32768 - 2^16 = 65536
static const unsigned MAX_NUMBER_OF_ELEMENTS_PER_CHUNK = (1<<CHUNK_INDEX_BIT_DEC);
static const unsigned ELEMENT_INDEX_BIT_MASK = MAX_NUMBER_OF_ELEMENTS_PER_CHUNK-1;

#include "CCShareable.h"

#include <stdlib.h>
#include <string.h>
#include <vector>

//! A generic array structure splitted in several small chunks to avoid the 'biggest contigous memory chunk' limit
/** This very useful structure can be used to store n-uplets (n starting from 1) of scalar types (int, float, etc.)
	or even objects, provided they have comparison operators ("<" and ">").
	[SHAREABLE] It can be shared by multiple objects (it is deleted only when the last one 'releases' it).
**/
template <int N, class ScalarType> class GenericChunkedArray : public CCShareable
{
public:

	//! Default constructor
	/** [SHAREABLE] Call 'link' when associating this array to an object.
	**/
	GenericChunkedArray()
		: CCShareable()
		, m_count(0)
		, m_maxCount(0)
		, m_iterator(0)
	{
		memset(m_minVal,0,sizeof(ScalarType)*N);
		memset(m_maxVal,0,sizeof(ScalarType)*N);
	}

	//! Returns the array size
	/** This corresponds to the number of inserted elements
		\return the number of elements actually inserted into this array
	**/
	inline unsigned currentSize() const {return m_count;}

	//! Returns the maximum array size
	/** This is the total (reserved) size, not only the number of inserted elements
		\return the number of elements that can be stored in this array
	**/
	inline unsigned capacity() const {return m_maxCount;}

	//! Specifies if the array has been initialized or not
	/** The array is initialized after a call to reserve or resize (with at least one element).
		\return true if the array has been alreay initialized, and false otherwise
	**/
	inline bool isAllocated() const {return (capacity()!=0);}

	//! Returns number of components
	inline unsigned dim() const {return N;}

	//! Returns memory (in bytes) currently used by this structure
	inline unsigned memory() const
	{
		return sizeof(GenericChunkedArray) 
				+ N*capacity()*sizeof(ScalarType)
				+ m_theChunks.capacity()*sizeof(ScalarType*)
				+ m_perChunkCount.capacity()*sizeof(unsigned);
	}

	//! Clears the array
	/** \param releaseMemory whether memory should be released or not (for quicker "refill")
	**/
	void clear(bool releaseMemory = true)
	{
		if (releaseMemory)
		{
			while (!m_theChunks.empty())
			{
				delete[] m_theChunks.back();
				m_theChunks.pop_back();
			}
			m_perChunkCount.clear();
			m_maxCount=0;
		}

		m_count=0;
		memset(m_minVal,0,sizeof(ScalarType)*N);
		memset(m_maxVal,0,sizeof(ScalarType)*N);
		placeIteratorAtBegining();
	}

	//! Fills the table with a particular value
	/** \param fillValue filling value/vector (if 0, table is filled with 0)
	**/
	void fill(const ScalarType* fillValue=0)
	{
		if (m_theChunks.empty())
			return;

		if (!fillValue)
		{
			//default fill value = 0
			for (unsigned i=0;i<m_theChunks.size();++i)
				memset(m_theChunks[i],0,m_perChunkCount[i]*sizeof(ScalarType)*N);
		}
		else
		{
			//we initialize the first chunk properly
			//with a recursive copy of N*2^k bytes (k=0,1,2,...)
			ScalarType* _cDest = m_theChunks.front();
			const ScalarType* _cSrc = _cDest;
			//we copy only the first element to init recurrence
			memcpy(_cDest,fillValue,N*sizeof(ScalarType));
			_cDest += N;

			unsigned elemToFill = m_perChunkCount[0];
			unsigned elemFilled = 1;
			unsigned copySize = 1;

			//recurrence
			while (elemFilled<elemToFill)
			{
				unsigned cs = elemToFill-elemFilled;
				if (copySize<cs)
					cs=copySize;
				memcpy(_cDest,_cSrc,cs*sizeof(ScalarType)*N);
				_cDest += cs*(unsigned)N;
				elemFilled += cs;
				copySize<<=1;
			}

			//then we simply have to copy the first chunk to the other ones
			for (unsigned i=1;i<m_theChunks.size();++i)
				memcpy(m_theChunks[i],_cSrc,m_perChunkCount[i]*sizeof(ScalarType)*N);
		}

		//done
		m_count=m_maxCount;
	}

	//****** memory allocators ******//

	//! Reserves memory for the database
	/** This method tries to reserve some memory to store elements
		that will be inserted later (see GenericChunkedArray::addElement).
		If the new number of elements is smaller than the actual one,
		nothing happens.
		\param newNumberOfElements the new number of elements
		\return true if the method succeeds, false otherwise
	**/
	bool reserve(unsigned newNumberOfElements)
	{
		while (m_maxCount<newNumberOfElements)
		{
			if (m_theChunks.empty() || m_perChunkCount.back()==MAX_NUMBER_OF_ELEMENTS_PER_CHUNK)
			{
				m_theChunks.push_back(0);
				m_perChunkCount.push_back(0);
			}

			//the number of new elements that we want to reserve
			unsigned newNumberOfElementsForThisChunk = newNumberOfElements-m_maxCount;
			//free room left in the current chunk
			unsigned freeSpaceInThisChunk = MAX_NUMBER_OF_ELEMENTS_PER_CHUNK-m_perChunkCount.back();
			//of course, we can't take more than that...
			if (freeSpaceInThisChunk < newNumberOfElementsForThisChunk)
				newNumberOfElementsForThisChunk = freeSpaceInThisChunk;

			//let's reallocate the chunk
			void* newTable = realloc(m_theChunks.back(),(m_perChunkCount.back()+newNumberOfElementsForThisChunk)*N*sizeof(ScalarType));
			//not enough memory?!
			if (!newTable)
			{
				//we cancel last insertion if it's an empty chunk
				if (m_perChunkCount.back()==0)
				{
					m_perChunkCount.pop_back();
					m_theChunks.pop_back();
				}
				return false;
			}
			//otherwise we update current structure
			m_theChunks.back() = (ScalarType*)newTable;
			m_perChunkCount.back() += newNumberOfElementsForThisChunk;
			m_maxCount += newNumberOfElementsForThisChunk;
		}

		return true;
	}

	//! Resizes the database
	/** The database is resized with the specified size. If the new size
		is smaller, the overflooding elements will be deleted. If its greater,
		the database is filled with blank values (warning, the GenericChunkedArray::addElement
		method will insert values after the new ones, use the GenericChunkedArray::setValue
		method instead).
		\param newNumberOfElements the new number of n-uplets
		\param initNewElements specifies if the new elements should be initialized with a specific value (in this case, the last parameter shouldn't be 0)
		\param valueForNewElements the default value for the new elements (only necessary if the precedent parameter is true)
		\return true if the method succeeds, false otherwise
	**/
	bool resize(unsigned newNumberOfElements, bool initNewElements=false, const ScalarType* valueForNewElements=0)
	{
		//s'il faut mettre la taille à zéro, ça revient à faire un clear !
		if (newNumberOfElements==0)
		{
			clear();
		}
		//sinon, s'il faut agrandir la place de stockage, alors ça revient à faire un "reserve"
		else if (newNumberOfElements>m_maxCount)
		{
			if (!reserve(newNumberOfElements))
				return false;
			//et si besoin on initialise les nouveaux elements
			if (initNewElements)
			{
				//m_maxCount a normalement été mis à jour par reserve() !
				for (unsigned i=m_count;i<m_maxCount;++i)
					setValue(i,valueForNewElements);
			}
		}
		else //dernier cas, on doit réduire la taille de un ou plusieurs chunks
		{
			while (m_maxCount > newNumberOfElements)
			{
				//no (more) chunk?! we stop
				if (m_perChunkCount.empty())
					return true;

				//nombre d'elements à supprimer
				unsigned spaceToFree = m_maxCount-newNumberOfElements;
				//nombre d'elements dans le dernier chunk
				unsigned numberOfElementsForThisChunk = m_perChunkCount.back();

				//si il y a plus d'elements à supprimer que d'elements dans le dernier chunk
				if (spaceToFree>=numberOfElementsForThisChunk)
				{
					//on supprime le chunk
					m_maxCount -= numberOfElementsForThisChunk;
					delete m_theChunks.back();
					m_theChunks.pop_back();
					m_perChunkCount.pop_back();
				}
				//sinon
				else
				{
					//on réalloue la mémoire prise par le chunk
					numberOfElementsForThisChunk -= spaceToFree;
					assert(numberOfElementsForThisChunk>0);
					void* newTable = realloc(m_theChunks.back(),numberOfElementsForThisChunk*N*sizeof(ScalarType));
					//si la réallocation échoue
					if (!newTable)
						return false;
					m_theChunks.back() = (ScalarType*)newTable;
					m_perChunkCount.back() = numberOfElementsForThisChunk;
					m_maxCount -= spaceToFree;
				}
			}
		}

		m_count=m_maxCount;

		return true;
	}

	//! Sets current size
	/** WARNINGS:
		- min and max boundaries may be modified (see 'computeMinAndMax').
		- global iterator may be invalidated
		\param size new size (should be inferior to m_maxCount)
	**/
	void setCurrentSize(unsigned size)
	{
		if (size>=m_maxCount)
		{
			assert(false);
			return;
		}

		m_count=size;
	}

	//! Direct access operator
	/** \param index an element index
		\return pointer to the ith element.
	**/
	inline ScalarType* operator[] (unsigned index) {return getValue(index);}

	//***** data access *****//

	//! Places global iterator at the begining of the array
	inline void placeIteratorAtBegining() {m_iterator=0;}

	//! Forwards global iterator (one position)
	inline void forwardIterator() {++m_iterator;}

	//! Returns the value currently pointed by the global iterator
	/** Warning: the global iterator must have been previously initizialized
		(see GenericChunkedArray::placeIteratorAtBegining) and it shouldn't
		be out of bounds.
		\return a pointer to the current element.
	**/
	inline ScalarType* getCurrentValue() {return getValue(m_iterator);}

	//! Adds a new element to the database
	/** Warning: the memory should have been previously reserved (see
		GenericChunkedArray::reserve).
		\param newElement the element to insert
	**/
	inline void addElement(const ScalarType* newElement)
	{
		assert(m_count<m_maxCount);
		setValue(m_count++,newElement);
	}

	//! Returns the ith value stored in the array
	/** \param index the index of the element to return
		\return a pointer to the ith element
	**/
	inline ScalarType* getValue(unsigned index) const {assert(index<m_maxCount);return m_theChunks[index >> CHUNK_INDEX_BIT_DEC]+((index & ELEMENT_INDEX_BIT_MASK)*N);}

	//! Sets the value of the ith element
	/** \param index the index of the element to update
		\param value the new value for the element
	**/
	inline void setValue(unsigned index, const ScalarType* value) {assert(index<m_maxCount);memcpy(getValue(index),value,N*sizeof(ScalarType));}

	//! Returns the element with the minimum value stored in the array
	/** The computeMinAndMax method must be called prior to this one
		(and each time the array content is modified).
		\return a pointer to the "minimum" element
	**/
	inline ScalarType* getMin() {return m_minVal;}

	//! Const version of GenericChunkedArray::getMin
	inline const ScalarType* getMin() const {return m_minVal;}

	//! Returns the element with the maximum value stored in the array
	/** The computeMinAndMax method must be called prior to this one
		(and each time the array content is modified).
		\return a pointer to the "maximum" element
	**/
	inline ScalarType* getMax() {return m_maxVal;}

	//! Const version of GenericChunkedArray::getMax
	inline const ScalarType* getMax() const {return m_maxVal;}

	//! Sets the value of the minimum (independantly of what is stored in the array)
	/** \param m the "minimum" element
	**/
	inline void setMin(const ScalarType* m) {memcpy(m_minVal,m,N*sizeof(ScalarType));}

	//! Sets the value of the maximum (independantly of what is stored in the array)
	/** \param M the "maximum" element
	**/
	inline void setMax(const ScalarType* M) {memcpy(m_maxVal,M,N*sizeof(ScalarType));}

	//! Determines "minimum" and "maximum" elements
	/** If elements are composed of several components (n-uplets with n>1),
		the algorithm will look for the element which have the smallest
		(or biggest) component of all (without any consideration for the order
		of the values).
	**/
	virtual void computeMinAndMax()
	{
		//no points?
		if (m_count==0)
		{
			//all boundaries to zero
			memset(m_minVal,0,sizeof(ScalarType)*N);
			memset(m_maxVal,0,sizeof(ScalarType)*N);
			return;
		}

		//we set the first element as min and max boundaries
		memcpy(m_minVal,getValue(0),sizeof(ScalarType)*N);
		memcpy(m_maxVal,m_minVal,sizeof(ScalarType)*N);

		//we update boundaries with all other values
		for (unsigned i=1;i<m_count;++i)
		{
			const ScalarType* val = getValue(i);
			for (unsigned j=0;j<N;++j)
			{
				if (val[j]<m_minVal[j])
					m_minVal[j]=val[j];
				else if (val[j]>m_maxVal[j])
					m_maxVal[j]=val[j];
			}
		}
	}

	//! Swaps two elements
	/** \param firstElementIndex first element index
		\param secondElementIndex second element index
	**/
	void swap(unsigned firstElementIndex, unsigned secondElementIndex)
	{
		assert(firstElementIndex < m_count && secondElementIndex < m_count);
		ScalarType* v1 = getValue(firstElementIndex);
		ScalarType* v2 = getValue(secondElementIndex);
		/*if (N==1) --> case N==1 is specialized below
			std::swap(*v1,*v2);
		else
		{
		//*/
			ScalarType tempVal[N];
			memcpy(tempVal,v1,N*sizeof(ScalarType));
			memcpy(v1,v2,N*sizeof(ScalarType));
			memcpy(v2,tempVal,N*sizeof(ScalarType));
		//}
	}

	//! Returns the number of chunks
	inline unsigned chunksCount() const {return m_theChunks.size();}

	//! Returns the number of points in a given chunk
	inline unsigned chunkSize(unsigned index) const {assert(index < m_theChunks.size()); return m_perChunkCount[index];}

	//! Returns the begining of a given chunk (pointer)
	inline ScalarType* chunkStartPtr(unsigned index) const {assert(index < m_theChunks.size()); return m_theChunks[index];}

	//! Copy array data to another one
	/** \param dest destination array (will be resize if necessary)
		\return success
	**/
	bool copy(GenericChunkedArray<N,ScalarType>& dest) const
	{
		if (!dest.resize(capacity()))
			return false;
		//copy content
		assert(dest.m_theChunks.size() == m_theChunks.size());
		for (unsigned i=0;i<m_theChunks.size();++i)
		{
			assert(dest.m_perChunkCount[i] == m_perChunkCount[i]);
			memcpy(dest.m_theChunks[i],m_theChunks[i],m_perChunkCount[i]*sizeof(ScalarType)*N);
		}
		return true;
	}

protected:

	//! GenericChunkedArray default destructor
	/** [SHAREABLE] Call 'release' to destroy this object properly.
	**/
	virtual ~GenericChunkedArray()
	{
		while (!m_theChunks.empty())
		{
			delete[] m_theChunks.back();
			m_theChunks.pop_back();
		}
	}

	//! Minimum values stored in array (along each dimension)
	ScalarType m_minVal[N];

	//! Maximum values stored in array (along each dimension)
	ScalarType m_maxVal[N];

	//! Arrays 'chunks'
	std::vector<ScalarType*> m_theChunks;
	//! Elements per chunk
	std::vector<unsigned> m_perChunkCount;
	//! Total number of elements
	unsigned m_count;
	//! Max total number of elements
	unsigned m_maxCount;

	//! Iterator
	unsigned m_iterator;
};

//! Specialization of GenericChunkedArray for the case where N=1 (speed up)
template <class ScalarType> class GenericChunkedArray<1,ScalarType> : public CCShareable
{
public:

	//! Default constructor
	/** [SHAREABLE] Call 'link' when associating this array to an object.
	**/
	GenericChunkedArray()
		: CCShareable()
		, m_minVal(0)
		, m_maxVal(0)
		, m_count(0)
		, m_maxCount(0)
		, m_iterator(0)
	{
	}

	//! Returns the array size
	/** This corresponds to the number of inserted elements
		\return the number of elements actually inserted into this array
	**/
	inline unsigned currentSize() const {return m_count;}

	//! Returns the maximum array size
	/** This is the total (reserved) size, not only the number of inserted elements
		\return the number of elements that can be stored in this array
	**/
	inline unsigned capacity() const {return m_maxCount;}

	//! Specifies if the array has been initialized or not
	/** The array is initialized after a call to reserve or resize (with at least one element).
		\return true if the array has been alreay initialized, and false otherwise
	**/
	inline bool isAllocated() const {return (capacity()!=0);}

	//! Returns number of components
	inline unsigned dim() const {return 1;}

	//! Returns memory (in bytes) currently used by this structure
	inline unsigned memory() const
	{
		return sizeof(GenericChunkedArray) 
				+ capacity()*sizeof(ScalarType)
				+ m_theChunks.capacity()*sizeof(ScalarType*)
				+ m_perChunkCount.capacity()*sizeof(unsigned);
	}
	//! Clears the array
	/** \param releaseMemory whether memory should be released or not (for quicker "refill")
	**/
	void clear(bool releaseMemory = true)
	{
		if (releaseMemory)
		{
			while (!m_theChunks.empty())
			{
				delete[] m_theChunks.back();
				m_theChunks.pop_back();
			}
			m_perChunkCount.clear();
			m_maxCount=0;
		}

		m_count=0;
		m_minVal=m_maxVal=0;
		placeIteratorAtBegining();
	}

	//! Fills the table with a particular value
	/** \param fillValue filling value/vector (if 0, table is filled with 0)
	**/
	void fill(const ScalarType& fillValue=0)
	{
		if (m_theChunks.empty())
			return;

		if (fillValue==0)
		{
			//default fill value = 0
			for (unsigned i=0;i<m_theChunks.size();++i)
				memset(m_theChunks[i],0,m_perChunkCount[i]*sizeof(ScalarType));
		}
		else
		{
			//we initialize the first chunk properly
			//with a recursive copy of 2^k bytes (k=0,1,2,...)
			ScalarType* _cDest = m_theChunks.front();
			const ScalarType* _cSrc = _cDest;
			//we copy only the first element to init recurrence
			*_cDest++ = fillValue;

			unsigned elemToFill = m_perChunkCount[0];
			unsigned elemFilled = 1;
			unsigned copySize = 1;

			//recurrence
			while (elemFilled<elemToFill)
			{
				unsigned cs = elemToFill-elemFilled;
				if (copySize<cs)
					cs=copySize;
				memcpy(_cDest,_cSrc,cs*sizeof(ScalarType));
				_cDest += cs;
				elemFilled += cs;
				copySize<<=1;
			}

			//then we simply have to copy the first chunk to the other ones
			for (unsigned i=1;i<m_theChunks.size();++i)
				memcpy(m_theChunks[i],_cSrc,m_perChunkCount[i]*sizeof(ScalarType));
		}

		//done
		m_count=m_maxCount;
	}

	//****** memory allocators ******//

	//! Reserves memory for the database
	/** This method tries to reserve some memory to store elements
		that will be inserted later (see GenericChunkedArray::addElement).
		If the new number of elements is smaller than the actual one,
		nothing happens.
		\param newNumberOfElements the new number of elements
		\return true if the method succeeds, false otherwise
	**/
	bool reserve(unsigned newNumberOfElements)
	{
		while (m_maxCount<newNumberOfElements)
		{
			if (m_theChunks.empty() || m_perChunkCount.back()==MAX_NUMBER_OF_ELEMENTS_PER_CHUNK)
			{
				m_theChunks.push_back(0);
				m_perChunkCount.push_back(0);
			}

			//the number of new elements that we want to reserve
			unsigned newNumberOfElementsForThisChunk = newNumberOfElements-m_maxCount;
			//free room left in the current chunk
			unsigned freeSpaceInThisChunk = MAX_NUMBER_OF_ELEMENTS_PER_CHUNK-m_perChunkCount.back();
			//of course, we can't take more than that...
			if (freeSpaceInThisChunk < newNumberOfElementsForThisChunk)
				newNumberOfElementsForThisChunk = freeSpaceInThisChunk;

			//let's reallocate the chunk
			void* newTable = realloc(m_theChunks.back(),(m_perChunkCount.back()+newNumberOfElementsForThisChunk)*sizeof(ScalarType));
			//not enough memory?!
			if (!newTable)
			{
				//we cancel last insertion if it's an empty chunk
				if (m_perChunkCount.back()==0)
				{
					m_perChunkCount.pop_back();
					m_theChunks.pop_back();
				}
				return false;
			}
			//otherwise we update current structure
			m_theChunks.back() = (ScalarType*)newTable;
			m_perChunkCount.back() += newNumberOfElementsForThisChunk;
			m_maxCount += newNumberOfElementsForThisChunk;
		}

		return true;
	}

	//! Resizes the database
	/** The database is resized with the specified size. If the new size
		is smaller, the overflooding elements will be deleted. If its greater,
		the database is filled with blank values (warning, the GenericChunkedArray::addElement
		method will insert values after the new ones, use the GenericChunkedArray::setValue
		method instead).
		\param newNumberOfElements the new number of n-uplets
		\param initNewElements specifies if the new elements should be initialized with a specific value (in this case, the last parameter shouldn't be 0)
		\param valueForNewElements the default value for the new elements (only necessary if the precedent parameter is true)
		\return true if the method succeeds, false otherwise
	**/
	bool resize(unsigned newNumberOfElements, bool initNewElements=false, const ScalarType& valueForNewElements=0)
	{
		//s'il faut mettre la taille à zéro, ça revient à faire un clear !
		if (newNumberOfElements==0)
		{
			clear();
		}
		//sinon, s'il faut agrandir la place de stockage, alors ça revient à faire un "reserve"
		else if (newNumberOfElements>m_maxCount)
		{
			if (!reserve(newNumberOfElements))
				return false;
			//et si besoin on initialise les nouveaux elements
			if (initNewElements)
			{
				//m_maxCount a normalement été mis à jour par reserve() !
				for (unsigned i=m_count;i<m_maxCount;++i)
					setValue(i,valueForNewElements);
			}
		}
		else //dernier cas, on doit réduire la taille de un ou plusieurs chunks
		{
			while (m_maxCount > newNumberOfElements)
			{
				//no (more) chunk?! we stop
				if (m_perChunkCount.empty())
					return true;

				//nombre d'elements à supprimer
				unsigned spaceToFree = m_maxCount-newNumberOfElements;
				//nombre d'elements dans le dernier chunk
				unsigned numberOfElementsForThisChunk = m_perChunkCount.back();

				//si il y a plus d'elements à supprimer que d'elements dans le dernier chunk
				if (spaceToFree>=numberOfElementsForThisChunk)
				{
					//on supprime le chunk
					m_maxCount -= numberOfElementsForThisChunk;
					delete m_theChunks.back();
					m_theChunks.pop_back();
					m_perChunkCount.pop_back();
				}
				//sinon
				else
				{
					//on réalloue la mémoire prise par le chunk
					numberOfElementsForThisChunk -= spaceToFree;
					assert(numberOfElementsForThisChunk>0);
					void* newTable = realloc(m_theChunks.back(),numberOfElementsForThisChunk*sizeof(ScalarType));
					//si la réallocation échoue
					if (!newTable)
						return false;
					m_theChunks.back() = (ScalarType*)newTable;
					m_perChunkCount.back() = numberOfElementsForThisChunk;
					m_maxCount -= spaceToFree;
				}
			}
		}

		m_count=m_maxCount;

		return true;
	}

	//! Sets current size
	/** WARNINGS:
		- min and max boundaries may be modified (see 'computeMinAndMax').
		- global iterator may be invalidated
		\param size new size (should be inferior to m_maxCount)
	**/
	void setCurrentSize(unsigned size)
	{
		if (size>=m_maxCount)
		{
			assert(false);
			return;
		}

		m_count=size;
	}

	//! Direct access operator
	/** \param index an element index
		\return pointer to the ith element.
	**/
	inline ScalarType& operator[] (unsigned index)
	{
		assert(index<m_maxCount);
		return m_theChunks[index >> CHUNK_INDEX_BIT_DEC][index & ELEMENT_INDEX_BIT_MASK];
	}

	//***** data access *****//

	//! Places global iterator at the begining of the array
	inline void placeIteratorAtBegining() {m_iterator=0;}

	//! Forwards global iterator (one position)
	inline void forwardIterator() {++m_iterator;}

	//! Returns the value currently pointed by the global iterator
	/** Warning: the global iterator must have been previously initizialized
		(see GenericChunkedArray::placeIteratorAtBegining) and it shouldn't
		be out of bounds.
		\return current element value as a reference.
	**/
	inline const ScalarType& getCurrentValue() const {return getValue(m_iterator);}

	//! Returns a pointer on the the value currently pointed by the global iterator
	/** Warning: the global iterator must have been previously initizialized
		(see GenericChunkedArray::placeIteratorAtBegining) and it shouldn't
		be out of bounds.
		\return a pointer to the current element.
	**/
	inline ScalarType* getCurrentValuePtr() {return &(*this)[m_iterator];}

	//! Adds a new element to the database
	/** Warning: the memory should have been previously reserved (see
		GenericChunkedArray::reserve).
		\param newElement the element to insert
	**/
	inline void addElement(const ScalarType& newElement)
	{
		assert(m_count<m_maxCount);
		setValue(m_count++,newElement);
	}

	//! Returns the ith value stored in the array
	/** \param index the index of the element to return
		\return a pointer to the ith element
	**/
	inline const ScalarType& getValue(unsigned index) const
	{
		assert(index<m_maxCount);
		return m_theChunks[index >> CHUNK_INDEX_BIT_DEC][index & ELEMENT_INDEX_BIT_MASK];
	}
//		return (*this)[index];}

	//! Sets the value of the ith element
	/** \param index the index of the element to update
		\param value the new value for the element
	**/
	inline void setValue(unsigned index, const ScalarType& value) {(*this)[index]=value;}

	//! Returns the element with the minimum value stored in the array
	/** The computeMinAndMax method must be called prior to this one
		(and each time the array content is modified).
		\return a pointer to the "minimum" element
	**/
	inline ScalarType getMin() {return m_minVal;}

	//! Const version of GenericChunkedArray::getMin
	inline const ScalarType getMin() const {return m_minVal;}

	//! Returns the element with the maximum value stored in the array
	/** The computeMinAndMax method must be called prior to this one
		(and each time the array content is modified).
		\return a pointer to the "maximum" element
	**/
	inline ScalarType getMax() {return m_maxVal;}

	//! Const version of GenericChunkedArray::getMax
	inline const ScalarType getMax() const {return m_maxVal;}

	//! Sets the value of the minimum (independantly of what is stored in the array)
	/** \param m the "minimum" element
	**/
	inline void setMin(const ScalarType& m) {m_minVal = m;}

	//! Sets the value of the maximum (independantly of what is stored in the array)
	/** \param M the "maximum" element
	**/
	inline void setMax(const ScalarType& M) {m_maxVal = M;}

	//! Determines "minimum" and "maximum" elements
	/** If elements are composed of several components (n-uplets with n>1),
		the algorithm will look for the element which have the smallest
		(or biggest) component of all (without any consideration for the order
		of the values).
	**/
	virtual void computeMinAndMax()
	{
		//no points?
		if (m_maxCount==0)
		{
			//all boundaries to zero
			m_minVal = m_maxVal = 0;
			return;
		}

		//we set the first element as min and max boundaries
		m_minVal = m_minVal = getValue(0);

		//we update boundaries with all other values
		for (unsigned i=1;i<m_maxCount;++i)
		{
			const ScalarType& val = getValue(i);
			if (val<m_minVal)
				m_minVal=val;
			else if (val>m_maxVal)
				m_maxVal=val;
		}
	}

	//! Swaps two elements
	/** \param firstElementIndex first element index
		\param secondElementIndex second element index
	**/
	inline void swap(unsigned firstElementIndex, unsigned secondElementIndex)
	{
		assert(firstElementIndex < m_count && secondElementIndex < m_count);
		ScalarType& v1 = (*this)[firstElementIndex];
		ScalarType& v2 = (*this)[secondElementIndex];
		ScalarType temp = v1;
		v1 = v2;
		v2 = temp;
	}

	//! Returns the number of chunks
	inline unsigned chunksCount() const {return m_theChunks.size();}

	//! Returns the number of points in a given chunk
	inline unsigned chunkSize(unsigned index) const {assert(index < m_theChunks.size()); return m_perChunkCount[index];}

	//! Returns the begining of a given chunk (pointer)
	inline ScalarType* chunkStartPtr(unsigned index) const {assert(index < m_theChunks.size()); return m_theChunks[index];}

	//! Copy array data to another one
	/** \param dest destination array (will be resize if necessary)
		\return success
	**/
	bool copy(GenericChunkedArray<1,ScalarType>& dest) const
	{
		if (!dest.resize(capacity()))
			return false;
		//copy content
		assert(dest.m_theChunks.size() == m_theChunks.size());
		for (unsigned i=0;i<m_theChunks.size();++i)
		{
			assert(dest.m_perChunkCount[i] == m_perChunkCount[i]);
			memcpy(dest.m_theChunks[i],m_theChunks[i],m_perChunkCount[i]*sizeof(ScalarType));
		}
		return true;
	}

protected:

	//! GenericChunkedArray default destructor
	/** [SHAREABLE] Call 'release' to destroy this object properly.
	**/
	virtual ~GenericChunkedArray()
	{
		while (!m_theChunks.empty())
		{
			delete[] m_theChunks.back();
			m_theChunks.pop_back();
		}
	}

	//! Minimum values stored in array (along each dimension)
	ScalarType m_minVal;

	//! Maximum values stored in array (along each dimension)
	ScalarType m_maxVal;

	//! Arrays 'chunks'
	std::vector<ScalarType*> m_theChunks;
	//! Elements per chunk
	std::vector<unsigned> m_perChunkCount;
	//! Total number of elements
	unsigned m_count;
	//! Max total number of elements
	unsigned m_maxCount;

	//! Iterator
	unsigned m_iterator;
};

#endif
