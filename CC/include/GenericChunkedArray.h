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

#ifndef GENERIC_CHUNKED_ARRAY_HEADER
#define GENERIC_CHUNKED_ARRAY_HEADER

#ifdef _MSC_VER
//To get rid of the really annoying warnings about template class exportation
#pragma warning( disable: 4251 )
#pragma warning( disable: 4530 )
#endif

#include "CCPlatform.h"

//DGM: we don't really need to 'chunk' the memory on 64 bits architectures
//But we keep this mechanism as it is handy when displaying entities!

//Chunks can only be handled rapidly if their size is a power of 2
#define CHUNK_INDEX_BIT_DEC 16 //2^14 = 16384 - 2^15 = 32768 - 2^16 = 65536
static const unsigned MAX_NUMBER_OF_ELEMENTS_PER_CHUNK = (1<<CHUNK_INDEX_BIT_DEC);

#ifndef CC_ENV_64
static const unsigned ELEMENT_INDEX_BIT_MASK = MAX_NUMBER_OF_ELEMENTS_PER_CHUNK-1;
#endif

#include "CCShareable.h"

//system
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <vector>

//! A generic array structure split in several small chunks to avoid the 'biggest contigous memory chunk' limit
/** This very useful structure can be used to store n-uplets (n starting from 1) of scalar types (int, float, etc.)
	or even objects, provided they have comparison operators ("<" and ">").
	[SHAREABLE] It can be shared by multiple objects (it is deleted only when the last one 'releases' it).
**/
template <int N, class ElementType> class GenericChunkedArray : public CCShareable
{
public:

	//! Default constructor
	/** [SHAREABLE] Call 'link' when associating this array to an object.
	**/
	GenericChunkedArray()
		: CCShareable()
		, m_count(0)
		, m_capacity(0)
		, m_iterator(0)
	{
		memset(m_minVal, 0, sizeof(ElementType)*N);
		memset(m_maxVal, 0, sizeof(ElementType)*N);
	}

	//! Copy constructor
	/**	\warning May throw a std::bad_alloc exception
	**/
	GenericChunkedArray(const GenericChunkedArray& gca)
		: CCShareable()
		, m_count(0)
		, m_capacity(0)
		, m_iterator(0)
	{
		if (!gca.copy(*this))
		{
			memset(m_minVal, 0, sizeof(ElementType)*N);
			memset(m_maxVal, 0, sizeof(ElementType)*N);
			throw std::bad_alloc();
		}
		else
		{
			memcpy(m_minVal, gca.m_minVal, sizeof(ElementType)*N);
			memcpy(m_maxVal, gca.m_maxVal, sizeof(ElementType)*N);
			m_iterator = gca.m_iterator;
		}
	}

	//! Returns the array size
	/** This corresponds to the number of inserted elements
		\return the number of elements actually inserted into this array
	**/
	inline unsigned currentSize() const { return m_count; }

	//! Returns the maximum array size
	/** This is the total (reserved) size, not only the number of inserted elements
		\return the number of elements that can be stored in this array
	**/
	inline unsigned capacity() const { return m_capacity; }

	//! Specifies if the array has been initialized or not
	/** The array is initialized after a call to reserve or resize (with at least one element).
		\return true if the array has been alreay initialized, and false otherwise
	**/
	inline bool isAllocated() const { return capacity() != 0; }

	//! Returns number of components
	inline unsigned dim() const { return N; }

	//! Returns memory (in bytes) currently used by this structure
	inline size_t memory() const
	{
		return sizeof(GenericChunkedArray) 
#ifndef CC_ENV_64
				+ m_theChunks.capacity()     * sizeof(ElementType*)
				+ m_perChunkCount.capacity() * sizeof(unsigned)
#endif
				+ static_cast<size_t>(N) * static_cast<size_t>(capacity()) * sizeof(ElementType);
	}

	//! Clears the array
	/** \param releaseMemory whether memory should be released or not (for quicker "refill")
	**/
	void clear(bool releaseMemory = true)
	{
		if (releaseMemory)
		{
#ifdef CC_ENV_64
			m_data.clear();
#else
			while (!m_theChunks.empty())
			{
				delete[] m_theChunks.back();
				m_theChunks.pop_back();
			}
			m_perChunkCount.clear();
#endif
			m_capacity = 0;
		}

		m_count = 0;
		memset(m_minVal, 0, sizeof(ElementType)*N);
		memset(m_maxVal, 0, sizeof(ElementType)*N);
		placeIteratorAtBegining();
	}

	//! Fills the table with a particular value
	/** \param fillValue filling value/vector (if 0, table is filled with 0)
	**/
	void fill(const ElementType* fillValue = 0)
	{
		if (capacity() == 0)
		{
			//nothing to do
			return;
		}

		if (!fillValue)
		{
			//default fill value = 0
#ifdef CC_ENV_64
			ElementType zero = 0;
			std::fill(m_data.begin(), m_data.end(), zero);
#else
			for (size_t i = 0; i < m_theChunks.size(); ++i)
				memset(m_theChunks[i], 0, m_perChunkCount[i]*sizeof(ElementType)*N);
#endif
		}
		else
		{
			//we initialize the first chunk properly
			//with a recursive copy of N*2^k bytes (k=0,1,2,...)
#ifdef CC_ENV_64
			ElementType* _cDest = &(m_data.front());
#else
			ElementType* _cDest = m_theChunks.front();
#endif
			const ElementType* _cSrc = _cDest;
			//we copy only the first element to init recurrence
			memcpy(_cDest, fillValue, N*sizeof(ElementType));
			_cDest += N;

#ifdef CC_ENV_64
			unsigned elemToFill = m_capacity;
#else
			unsigned elemToFill = m_perChunkCount[0];
#endif
			unsigned elemFilled = 1;
			unsigned copySize = 1;

			//recurrence
			while (elemFilled < elemToFill)
			{
				unsigned cs = elemToFill - elemFilled;
				if (copySize < cs)
					cs = copySize;
				memcpy(_cDest, _cSrc, cs*sizeof(ElementType)*N);
				_cDest += cs*static_cast<unsigned>(N);
				elemFilled += cs;
				copySize <<= 1;
			}

#ifndef CC_ENV_64
			//then we simply have to copy the first chunk to the other ones
			for (size_t i = 1; i < m_theChunks.size(); ++i)
				memcpy(m_theChunks[i],_cSrc,m_perChunkCount[i]*sizeof(ElementType)*N);
#endif
		}

		//done
		m_count = m_capacity;
	}

	//****** memory allocators ******//

	//! Reserves memory for the array elements
	/** This method tries to reserve some memory to store elements
		that will be inserted later (see GenericChunkedArray::addElement).
		If the new number of elements is smaller than the actual one,
		nothing happens.
		\param capacity the new number of elements
		\return true if the method succeeds, false otherwise
	**/
	bool reserve(unsigned capacity)
	{
#ifdef CC_ENV_64
		try
		{
			if (m_capacity < capacity)
			{
				m_data.resize(capacity * N);
				m_capacity = capacity;
			}
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			return false;
		}
#else
		while (m_capacity < capacity)
		{
			if (m_theChunks.empty() || m_perChunkCount.back() == MAX_NUMBER_OF_ELEMENTS_PER_CHUNK)
			{
				m_theChunks.push_back(0);
				m_perChunkCount.push_back(0);
			}

			//the number of new elements that we want to reserve
			unsigned capacityForThisChunk = capacity - m_capacity;
			//free room left in the current chunk
			unsigned freeSpaceInThisChunk = MAX_NUMBER_OF_ELEMENTS_PER_CHUNK-m_perChunkCount.back();
			//of course, we can't take more than that...
			if (freeSpaceInThisChunk < capacityForThisChunk)
				capacityForThisChunk = freeSpaceInThisChunk;

			//let's reallocate the chunk
			void* newTable = realloc(m_theChunks.back(),(m_perChunkCount.back()+capacityForThisChunk)*N*sizeof(ElementType));
			//not enough memory?!
			if (!newTable)
			{
				//we cancel last insertion if it's an empty chunk
				if (m_perChunkCount.back() == 0)
				{
					m_perChunkCount.pop_back();
					m_theChunks.pop_back();
				}
				return false;
			}
			//otherwise we update current structure
			m_theChunks.back() = (ElementType*)newTable;
			m_perChunkCount.back() += capacityForThisChunk;
			m_capacity += capacityForThisChunk;
		}
#endif

		return true;
	}

	//! Resizes the array
	/** The array is resized with the specified size. If the new size
		is smaller, the overflooding elements will be deleted. If its greater,
		the array is filled with blank values (warning, the GenericChunkedArray::addElement
		method will insert values after the new ones, use the GenericChunkedArray::setValue
		method instead).
		\param count the new number of elements
		\param initNewElements specifies if the new elements should be initialized with a specific value (in this case, the last parameter shouldn't be 0)
		\param valueForNewElements the default value for the new elements (only necessary if the previous parameter is true)
		\return true if the method succeeds, false otherwise
	**/
	bool resize(unsigned count, bool initNewElements = false, const ElementType* valueForNewElements = 0)
	{
		//if the new size is 0, we can simply clear the array!
		if (count == 0)
		{
			clear();
		}
		//otherwise if we need to enlarge the array we must 'reserve' some memory
		else if (count > m_capacity)
		{
			if (!reserve(count))
				return false;

			//eventually we can fill it with a custom value if necessary
			if (initNewElements)
			{
				//m_capacity should be up-to-date after a call to 'reserve'
				for (unsigned i = m_count; i < m_capacity; ++i)
					setValue(i, valueForNewElements);
			}
		}
		else //last case: we have to reduce the array size
		{
#ifdef CC_ENV_64
			try
			{
				m_data.resize(count * N); //shouldn't fail, smaller
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory
				return false;
			}
		
			m_capacity = count;
#else
			while (m_capacity > count)
			{
				//no (more) chunk?! we stop
				if (m_perChunkCount.empty())
					return true;

				//number of elements to remove
				unsigned spaceToFree = m_capacity - count;
				//number of elements in this chunk
				unsigned numberOfElementsForThisChunk = m_perChunkCount.back();

				//if there's more elements to remove than elements in this chunk
				if (spaceToFree >= numberOfElementsForThisChunk)
				{
					//simply remove the chunk
					m_capacity -= numberOfElementsForThisChunk;
					delete m_theChunks.back();
					m_theChunks.pop_back();
					m_perChunkCount.pop_back();
				}
				//otherwise
				else
				{
					//we resize the chunk
					numberOfElementsForThisChunk -= spaceToFree;
					assert(numberOfElementsForThisChunk != 0);
					void* newTable = realloc(m_theChunks.back(),numberOfElementsForThisChunk*N*sizeof(ElementType));
					//if 'realloc' failed?!
					if (!newTable)
						return false;
					m_theChunks.back() = static_cast<ElementType*>(newTable);
					m_perChunkCount.back() = numberOfElementsForThisChunk;
					m_capacity -= spaceToFree;
				}
			}
#endif
		}

		m_count = m_capacity;

		return true;
	}

	//! Removes unused capacity
	inline void shrinkToFit()
	{
		if (currentSize() < capacity())
			resize(currentSize());
	}

	//! Sets current size
	/** WARNINGS:
		- min and max boundaries may not be valid anymore (see GenericChunkedArray::computeMinAndMax).
		- global iterator may be invalidated
		\param size new size (must be inferior to m_capacity)
	**/
	void setCurrentSize(unsigned size)
	{
		if (size > m_capacity)
		{
			assert(false);
			return;
		}

		m_count = size;
	}

	//! Direct access operator
	/** \param index an element index
		\return pointer to the ith element.
	**/
	inline ElementType* operator[] (unsigned index) { return getValue(index); }

	//***** data access *****//

	//! Places global iterator at the beginning of the array
	inline void placeIteratorAtBegining() { m_iterator = 0; }

	//! Forwards global iterator (one position)
	inline void forwardIterator() { ++m_iterator; }

	//! Returns the value currently pointed by the global iterator
	/** \warning The global iterator must have been previously initialized
		(see GenericChunkedArray::placeIteratorAtBegining) and it shouldn't
		be out of bounds.
		\return a pointer to the current element.
	**/
	inline ElementType* getCurrentValue() { return getValue(m_iterator); }

	//! Adds a new element to the array
	/** Warning: the memory should have been previously reserved (see
		GenericChunkedArray::reserve).
		\param newElement the element to insert
	**/
	inline void addElement(const ElementType* newElement)
	{
		assert(m_count < m_capacity);
		setValue(m_count++, newElement);
	}

	//! Returns the ith value stored in the array
	/** \param index the index of the element to return
		\return a pointer to the ith element
	**/
	inline ElementType* getValue(unsigned index)
	{
		assert(index < m_capacity);
#ifdef CC_ENV_64
		return &(m_data[index * N]);
#else
		return m_theChunks[index >> CHUNK_INDEX_BIT_DEC]+((index & ELEMENT_INDEX_BIT_MASK)*N);
#endif
	}

	//! Returns the ith value stored in the array (const version)
	/** \param index the index of the element to return
		\return a pointer to the ith element
	**/
	inline const ElementType* getValue(unsigned index) const
	{
		assert(index < m_capacity);
#ifdef CC_ENV_64
		return &(m_data[index * N]);
#else
		return m_theChunks[index >> CHUNK_INDEX_BIT_DEC]+((index & ELEMENT_INDEX_BIT_MASK)*N);
#endif
	}

	//! Sets the value of the ith element
	/** \param index the index of the element to update
		\param value the new value for the element
	**/
	inline void setValue(unsigned index, const ElementType* value)
	{
		assert(index < m_capacity);
		memcpy(getValue(index), value, N*sizeof(ElementType));
	}

	//! Returns the element with the minimum value stored in the array
	/** The computeMinAndMax method must be called prior to this one
		(and each time the array content is modified).
		\return a pointer to the "minimum" element
	**/
	inline ElementType* getMin() { return m_minVal; }

	//! Const version of GenericChunkedArray::getMin
	inline const ElementType* getMin() const { return m_minVal; }

	//! Returns the element with the maximum value stored in the array
	/** The computeMinAndMax method must be called prior to this one
		(and each time the array content is modified).
		\return a pointer to the "maximum" element
	**/
	inline ElementType* getMax() { return m_maxVal; }

	//! Const version of GenericChunkedArray::getMax
	inline const ElementType* getMax() const { return m_maxVal; }

	//! Sets the value of the minimum (independently of what is stored in the array)
	/** \param m the "minimum" element
	**/
	inline void setMin(const ElementType* m) { memcpy(m_minVal,m,N*sizeof(ElementType)); }

	//! Sets the value of the maximum (independently of what is stored in the array)
	/** \param M the "maximum" element
	**/
	inline void setMax(const ElementType* M) { memcpy(m_maxVal,M,N*sizeof(ElementType)); }

	//! Determines "minimum" and "maximum" elements
	/** If elements are composed of several components (n-uplets with n>1),
		the algorithm will look for the element which have the smallest
		(or biggest) component of all (without any consideration for the order
		of the values).
	**/	
	virtual void computeMinAndMax()
	{
		//no points?
		if (m_count == 0)
		{
			//all boundaries to zero
			memset(m_minVal, 0, sizeof(ElementType)*N);
			memset(m_maxVal, 0, sizeof(ElementType)*N);
			return;
		}
		
		//we set the first element as min and max boundaries
		memcpy(m_minVal, getValue(0), sizeof(ElementType)*N);
		memcpy(m_maxVal, m_minVal, sizeof(ElementType)*N);
		
		unsigned int count = m_count - 1;
		
		// do we have an odd number of (remaining) elements to check?
		bool odd = count & 1;
		if ( odd )
		{
			count--;
		}
		
		//we update boundaries with all other values
		for (unsigned i = 1; i < count; i += 2)
		{
			const ElementType* val = getValue(i);
			const ElementType* val2 = getValue(i + 1);

			for (unsigned j = 0; j < N; ++j)
			{
				ElementType maximum;
				ElementType	minimum;

				if (val[j] > val2[j])
				{
					minimum = val2[j];
					maximum = val[j];
				}
				else
				{
					minimum = val[j];
					maximum = val2[j];
				}
				
				if (maximum > m_maxVal[j])
				{
					m_maxVal[j] = maximum;
				}
				
				if (minimum < m_minVal[j])
				{
					m_minVal[j] = minimum;
				}
			}
		}
		
		// if we have an extra element, check it
		if (odd)
		{
			const ElementType* val = getValue(m_count - 1);

			for (unsigned j = 0; j < N; ++j)
			{
				if (val[j] > m_maxVal[j])
				{
					m_maxVal[j] = val[j];
				}

				if (val[j] < m_minVal[j])
				{
					m_minVal[j] = val[j];
				}
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
		ElementType* v1 = getValue(firstElementIndex);
		ElementType* v2 = getValue(secondElementIndex);
		//if (N==1) --> case N==1 is specialized below
		//	std::swap(*v1,*v2);
		//else
		//{
		//
			ElementType tempVal[N];
			memcpy(tempVal, v1, N*sizeof(ElementType));
			memcpy(v1, v2, N*sizeof(ElementType));
			memcpy(v2, tempVal, N*sizeof(ElementType));
		//}
	}

#ifdef CC_ENV_64
	//! Returns a pointer on the (contiguous) data array
	inline ElementType* data() { return &(m_data.front()); }

	//! Returns a pointer on the (contiguous) data array (const version)
	inline const ElementType* data() const { return &(m_data.front()); }
#endif //!CC_ENV_64
	
	//! Returns the number of chunks
	inline unsigned chunksCount() const
	{
#ifdef CC_ENV_64
		//fake chunk count
		return (m_count >> CHUNK_INDEX_BIT_DEC) + ((m_count & (MAX_NUMBER_OF_ELEMENTS_PER_CHUNK-1)) ? 1 : 0);
#else
		return static_cast<unsigned>(m_theChunks.size());
#endif
	}

	//! Returns the number of points in a given chunk
	inline unsigned chunkSize(unsigned index) const
	{
		assert(index < chunksCount());
#ifdef CC_ENV_64
		return  (index + 1 < chunksCount() ? MAX_NUMBER_OF_ELEMENTS_PER_CHUNK : currentSize() - index * MAX_NUMBER_OF_ELEMENTS_PER_CHUNK);
#else
		return m_perChunkCount[index];
#endif
	}

	//! Returns the beginning of a given chunk (pointer)
	inline ElementType* chunkStartPtr(unsigned index)
	{
		assert(index < chunksCount());
#ifdef CC_ENV_64
		return data() + (index * MAX_NUMBER_OF_ELEMENTS_PER_CHUNK * N);
#else
		return m_theChunks[index];
#endif
	}

	//! Returns the beginning of a given chunk (pointer)
	inline const ElementType* chunkStartPtr(unsigned index) const
	{
		assert(index < chunksCount());
#ifdef CC_ENV_64
		return data() + (index * MAX_NUMBER_OF_ELEMENTS_PER_CHUNK * N);
#else
		return m_theChunks[index];
#endif
	}

	//! Copy array data to another one
	/** \warning only the array content is copied!
		\param dest destination array (will be resized if necessary)
		\return success
	**/
	bool copy(GenericChunkedArray<N, ElementType>& dest) const
	{
		unsigned count = currentSize();
		if (!dest.resize(count))
		{
			return false;
		}
		
		//copy content		
#ifdef CC_ENV_64
		std::copy(m_data.begin(), m_data.end(), dest.m_data.begin());
#else
		unsigned copyCount = 0;
		assert(dest.m_theChunks.size() <= m_theChunks.size());
		for (size_t i=0; i<dest.m_theChunks.size(); ++i)
		{
			unsigned toCopyCount = std::min<unsigned>(count-copyCount,m_perChunkCount[i]);
			assert(dest.m_perChunkCount[i] >= toCopyCount);
			memcpy(dest.m_theChunks[i],m_theChunks[i],toCopyCount*sizeof(ElementType)*N);
			copyCount += toCopyCount;
		}
#endif
		return true;
	}

protected:

	//! GenericChunkedArray default destructor
	/** [SHAREABLE] Call 'release' to destroy this object properly.
	**/
	virtual ~GenericChunkedArray()
	{
#ifndef CC_ENV_64
		while (!m_theChunks.empty())
		{
			delete[] m_theChunks.back();
			m_theChunks.pop_back();
		}
#endif
	}

	//! Minimum values stored in array (along each dimension)
	ElementType m_minVal[N];

	//! Maximum values stored in array (along each dimension)
	ElementType m_maxVal[N];

#ifdef CC_ENV_64
	//! Data
	std::vector<ElementType> m_data;
#else
	//! Arrays 'chunks'
	std::vector<ElementType*> m_theChunks;
	//! Elements per chunk
	std::vector<unsigned> m_perChunkCount;
#endif

	//! Total number of elements
	unsigned m_count;
	//! Max total number of elements
	unsigned m_capacity;

	//! Iterator
	unsigned m_iterator;
};

//! Specialization of GenericChunkedArray for the case where N=1 (speed up)
template <class ElementType> class GenericChunkedArray<1, ElementType> : public CCShareable
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
		, m_capacity(0)
		, m_iterator(0)
	{}

	//! Copy constructor
	/**	\warning May throw a std::bad_alloc exception
	**/
	GenericChunkedArray(const GenericChunkedArray& gca)
		: CCShareable()
		, m_minVal(gca.m_minVal)
		, m_maxVal(gca.m_maxVal)
		, m_count(0)
		, m_capacity(0)
		, m_iterator(0)
	{
		if (!gca.copy(*this))
		{
			throw std::bad_alloc();
		}
		else
		{
			m_minVal = gca.m_minVal;
			m_maxVal = gca.m_maxVal;
			m_iterator = gca.m_iterator;
		}
	}

	//! Returns the array size
	/** This corresponds to the number of inserted elements
		\return the number of elements actually inserted into this array
	**/
	inline unsigned currentSize() const { return m_count; }

	//! Returns the maximum array size
	/** This is the total (reserved) size, not only the number of inserted elements
		\return the number of elements that can be stored in this array
	**/
	inline unsigned capacity() const { return m_capacity; }

	//! Specifies if the array has been initialized or not
	/** The array is initialized after a call to reserve or resize (with at least one element).
		\return true if the array has been alreay initialized, and false otherwise
	**/
	inline bool isAllocated() const { return capacity() != 0; }

	//! Returns number of components
	inline unsigned dim() const {return 1;}

	//! Returns memory (in bytes) currently used by this structure
	inline size_t memory() const
	{
		return sizeof(GenericChunkedArray) 
#ifndef CC_ENV_64
				+ m_theChunks.capacity()     * sizeof(ElementType*)
				+ m_perChunkCount.capacity() * sizeof(unsigned)
#endif
				+ static_cast<size_t>(capacity()) * sizeof(ElementType);
	}
	//! Clears the array
	/** \param releaseMemory whether memory should be released or not (for quicker "refill")
	**/
	void clear(bool releaseMemory = true)
	{
		if (releaseMemory)
		{
#ifdef CC_ENV_64
			m_data.clear();
#else
			while (!m_theChunks.empty())
			{
				delete[] m_theChunks.back();
				m_theChunks.pop_back();
			}
			m_perChunkCount.clear();
#endif
			m_capacity = 0;
		}

		m_count = 0;
		m_minVal = m_maxVal = 0;
		placeIteratorAtBegining();
	}

	//! Fills the table with a particular value
	/** \param fillValue filling value/vector (if 0, table is filled with 0)
	**/
	void fill(const ElementType& fillValue = 0)
	{
		if (capacity() == 0)
		{
			//nothing to do
			return;
		}

#ifdef CC_ENV_64
		std::fill(m_data.begin(), m_data.end(), fillValue);
#else
		if (fillValue == 0)
		{
			//default fill value = 0
			for (size_t i=0; i<m_theChunks.size(); ++i)
				memset(m_theChunks[i],0,m_perChunkCount[i]*sizeof(ElementType));
		}
		else
		{
			//we initialize the first chunk properly
			//with a recursive copy of 2^k bytes (k=0,1,2,...)
			ElementType* _cDest = m_theChunks.front();
			const ElementType* _cSrc = _cDest;
			//we copy only the first element to init recurrence
			*_cDest++ = fillValue;

			unsigned elemToFill = m_perChunkCount[0];
			unsigned elemFilled = 1;
			unsigned copySize = 1;

			//recurrence
			while (elemFilled < elemToFill)
			{
				unsigned cs = elemToFill-elemFilled;
				if (copySize < cs)
					cs = copySize;
				memcpy(_cDest,_cSrc,cs*sizeof(ElementType));
				_cDest += cs;
				elemFilled += cs;
				copySize <<= 1;
			}

			//then we simply have to copy the first chunk to the other ones
			for (size_t i=1; i<m_theChunks.size(); ++i)
				memcpy(m_theChunks[i],_cSrc,m_perChunkCount[i]*sizeof(ElementType));
		}
#endif

		//done
		m_count = m_capacity;
	}

	//****** memory allocators ******//

	//! Reserves memory for the array
	/** This method tries to reserve some memory to store elements
		that will be inserted later (see GenericChunkedArray::addElement).
		If the new number of elements is smaller than the actual one,
		nothing happens.
		\param capacity the new number of elements
		\return true if the method succeeds, false otherwise
	**/
	bool reserve(unsigned capacity)
	{
#ifdef CC_ENV_64
		try
		{
			if (m_capacity < capacity)
			{
				m_data.resize(capacity);
				m_capacity = capacity;
			}
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			return false;
		}
#else
		while (m_capacity < capacity)
		{
			if (m_theChunks.empty() || m_perChunkCount.back() == MAX_NUMBER_OF_ELEMENTS_PER_CHUNK)
			{
				m_theChunks.push_back(0);
				m_perChunkCount.push_back(0);
			}

			//the number of new elements that we want to reserve
			unsigned capacityForThisChunk = capacity - m_capacity;
			//free room left in the current chunk
			unsigned freeSpaceInThisChunk = MAX_NUMBER_OF_ELEMENTS_PER_CHUNK - m_perChunkCount.back();
			//of course, we can't take more than that...
			if (freeSpaceInThisChunk < capacityForThisChunk)
				capacityForThisChunk = freeSpaceInThisChunk;

			//let's reallocate the chunk
			void* newTable = realloc(m_theChunks.back(),(m_perChunkCount.back()+capacityForThisChunk)*sizeof(ElementType));
			//not enough memory?!
			if (!newTable)
			{
				//we cancel last insertion if it's an empty chunk
				if (m_perChunkCount.back() == 0)
				{
					m_perChunkCount.pop_back();
					m_theChunks.pop_back();
				}
				return false;
			}
			//otherwise we update current structure
			m_theChunks.back() = static_cast<ElementType*>(newTable);
			m_perChunkCount.back() += capacityForThisChunk;
			m_capacity += capacityForThisChunk;
		}
#endif
		return true;
	}

	//! Resizes the array
	/** The array is resized with the specified size. If the new size
		is smaller, the overflooding elements will be deleted. If its greater,
		the array is filled with blank values (warning, the GenericChunkedArray::addElement
		method will insert values after the new ones, use the GenericChunkedArray::setValue
		method instead).
		\param count the new number of elements
		\param initNewElements specifies if the new elements should be initialized with a specific value (in this case, the last parameter shouldn't be 0)
		\param valueForNewElements the default value for the new elements (only necessary if the previous parameter is true)
		\return true if the method succeeds, false otherwise
	**/
	bool resize(unsigned count, bool initNewElements = false, const ElementType& valueForNewElements = 0)
	{
		//if the new size is 0, we can simply clear the array!
		if (count == 0)
		{
			clear();
		}
		//otherwise if we need to enlarge the array we must 'reserve' some memory
		else if (count > m_capacity)
		{
			if (!reserve(count))
			{
				return false;
			}
			
			//eventually we can fill it with a custom value
			if (initNewElements)
			{
				//m_capacity should be up-to-date after a call to 'reserve'
				for (unsigned i = m_count; i < m_capacity; ++i)
				{
					setValue(i, valueForNewElements);
				}
			}
		}
		else //last case: we have to reduce the array size
		{
#ifdef CC_ENV_64
			try
			{
				m_data.resize(count); //shouldn't fail, smaller
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory
				return false;
			}
		
			m_capacity = count;
#else
			while (m_capacity > count)
			{
				//no (more) chunk?! we stop
				if (m_perChunkCount.empty())
					return true;

				//number of elements to remove
				unsigned spaceToFree = m_capacity-count;
				//number of elements in this chunk
				unsigned numberOfElementsForThisChunk = m_perChunkCount.back();

				//if there's more elements to remove than elements in this chunk
				if (spaceToFree >= numberOfElementsForThisChunk)
				{
					//simply remove the chunk
					m_capacity -= numberOfElementsForThisChunk;
					delete m_theChunks.back();
					m_theChunks.pop_back();
					m_perChunkCount.pop_back();
				}
				//otherwise
				else
				{
					//we resize the chunk
					numberOfElementsForThisChunk -= spaceToFree;
					assert(numberOfElementsForThisChunk > 0);
					void* newTable = realloc(m_theChunks.back(),numberOfElementsForThisChunk*sizeof(ElementType));
					//if 'realloc' failed?!
					if (!newTable)
						return false;
					m_theChunks.back() = static_cast<ElementType*>(newTable);
					m_perChunkCount.back() = numberOfElementsForThisChunk;
					m_capacity -= spaceToFree;
				}
			}
#endif
		}

		m_count = m_capacity;

		return true;
	}

	//! Removes unused capacity
	inline void shrinkToFit()
	{
		if (currentSize() < capacity())
			resize(currentSize());
	}

	//! Sets current size
	/** WARNINGS:
		- min and max boundaries may not be valid anymore (see GenericChunkedArray::computeMinAndMax).
		- global iterator may be invalidated
		\param size new size (must be inferior to m_capacity)
	**/
	void setCurrentSize(unsigned size)
	{
		if (size > m_capacity)
		{
			assert(false);
			return;
		}

		m_count = size;
	}

	//! Direct access operator
	/** \param index an element index
		\return value of the ith element.
	**/
	inline ElementType& operator[] (unsigned index) { return getValue(index); }

	//***** data access *****//

	//! Places global iterator at the beginning of the array
	inline void placeIteratorAtBegining() { m_iterator = 0; }

	//! Forwards global iterator (one position)
	inline void forwardIterator() { ++m_iterator; }

	//! Returns the value currently pointed by the global iterator
	/** Warning: the global iterator must have been previously initizialized
		(see GenericChunkedArray::placeIteratorAtBegining) and it shouldn't
		be out of bounds.
		\return current element value as a reference.
	**/
	inline const ElementType& getCurrentValue() const { return getValue(m_iterator); }

	//! Returns a pointer on the the value currently pointed by the global iterator
	/** Warning: the global iterator must have been previously initizialized
		(see GenericChunkedArray::placeIteratorAtBegining) and it shouldn't
		be out of bounds.
		\return a pointer to the current element.
	**/
	inline ElementType* getCurrentValuePtr() { return &(getValue(m_iterator)); }

	//! Adds a new element to the array
	/** Warning: the memory should have been previously reserved (see
		GenericChunkedArray::reserve).
		\param newElement the element to insert
	**/
	inline void addElement(const ElementType& newElement)
	{
		assert(m_count < m_capacity);
		setValue(m_count++, newElement);
	}

	//! Returns the ith value stored in the array
	/** \param index the index of the element to return
		\return a pointer to the ith element
	**/
	inline ElementType& getValue(unsigned index)
	{
		assert(index < m_capacity);
#ifdef CC_ENV_64
		return m_data[index];
#else
		return m_theChunks[index >> CHUNK_INDEX_BIT_DEC][index & ELEMENT_INDEX_BIT_MASK];
#endif
	}

	//! Returns the ith value stored in the array (const version)
	/** \param index the index of the element to return
		\return a pointer to the ith element
	**/
	inline const ElementType& getValue(unsigned index) const
	{
		assert(index < m_capacity);
#ifdef CC_ENV_64
		return m_data[index];
#else
		return m_theChunks[index >> CHUNK_INDEX_BIT_DEC][index & ELEMENT_INDEX_BIT_MASK];
#endif
	}

	//! Sets the value of the ith element
	/** \param index the index of the element to update
		\param value the new value for the element
	**/
	inline void setValue(unsigned index, const ElementType& value) { getValue(index) = value; }

	//! Returns the element with the minimum value stored in the array
	/** The computeMinAndMax method must be called prior to this one
		(and each time the array content is modified).
		\return a pointer to the "minimum" element
	**/
	inline ElementType getMin() { return m_minVal; }

	//! Const version of GenericChunkedArray::getMin
	inline const ElementType getMin() const { return m_minVal; }

	//! Returns the element with the maximum value stored in the array
	/** The computeMinAndMax method must be called prior to this one
		(and each time the array content is modified).
		\return a pointer to the "maximum" element
	**/
	inline ElementType getMax() { return m_maxVal; }

	//! Const version of GenericChunkedArray::getMax
	inline const ElementType getMax() const { return m_maxVal; }

	//! Sets the value of the minimum (independently of what is stored in the array)
	/** \param m the "minimum" element
	**/
	inline void setMin(const ElementType& m) { m_minVal = m; }

	//! Sets the value of the maximum (independently of what is stored in the array)
	/** \param M the "maximum" element
	**/
	inline void setMax(const ElementType& M) { m_maxVal = M; }

	//! Determines "minimum" and "maximum" elements
	/** If elements are composed of several components (n-uplets with n>1),
		the algorithm will look for the element which have the smallest
		(or biggest) component of all (without any consideration for the order
		of the values).
	**/
	virtual void computeMinAndMax()
	{
		//no points?
		if (m_capacity == 0)
		{
			//all boundaries to zero
			m_minVal = m_maxVal = 0;
			return;
		}

		//we set the first element as min and max boundaries
		m_minVal = m_maxVal = getValue(0);

		//we update boundaries with all other values
		for (unsigned i = 1; i < m_capacity; ++i)
		{
			const ElementType& val = getValue(i);
			if (val < m_minVal)
				m_minVal = val;
			else if (val > m_maxVal)
				m_maxVal = val;
		}
	}

	//! Swaps two elements
	/** \param firstElementIndex first element index
		\param secondElementIndex second element index
	**/
	inline void swap(unsigned firstElementIndex, unsigned secondElementIndex)
	{
		assert(firstElementIndex < m_count && secondElementIndex < m_count);
		ElementType& v1 = (*this)[firstElementIndex];
		ElementType& v2 = (*this)[secondElementIndex];
		ElementType temp = v1;
		v1 = v2;
		v2 = temp;
	}

#ifdef CC_ENV_64
	//! Returns a pointer on the (contiguous) data array
	inline ElementType* data() { return &(m_data.front()); }

	//! Returns a pointer on the (contiguous) data array (const version)
	inline const ElementType* data() const { return &(m_data.front()); }
#endif //!CC_ENV_64

	//! Returns the number of chunks
	inline unsigned chunksCount() const
	{
#ifdef CC_ENV_64
		//fake chunk count
		return (m_count >> CHUNK_INDEX_BIT_DEC) + ((m_count & (MAX_NUMBER_OF_ELEMENTS_PER_CHUNK-1)) ? 1 : 0);
#else
		return static_cast<unsigned>(m_theChunks.size());
#endif
	}

	//! Returns the number of points in a given chunk
	inline unsigned chunkSize(unsigned index) const
	{
		assert(index < chunksCount());
#ifdef CC_ENV_64
		return  (index + 1 < chunksCount() ? MAX_NUMBER_OF_ELEMENTS_PER_CHUNK : currentSize() - index * MAX_NUMBER_OF_ELEMENTS_PER_CHUNK);
#else
		return m_perChunkCount[index];
#endif
	}

	//! Returns the beginning of a given chunk (pointer)
	inline ElementType* chunkStartPtr(unsigned index)
	{
		assert(index < chunksCount());
#ifdef CC_ENV_64
		return data() + (index * MAX_NUMBER_OF_ELEMENTS_PER_CHUNK);
#else
		return m_theChunks[index];
#endif
	}

	//! Returns the beginning of a given chunk (pointer)
	inline const ElementType* chunkStartPtr(unsigned index) const
	{
		assert(index < chunksCount());
#ifdef CC_ENV_64
		return data() + (index * MAX_NUMBER_OF_ELEMENTS_PER_CHUNK);
#else
		return m_theChunks[index];
#endif
	}

	//! Copy array data to another one
	/** \warning only the array content is copied!
		\param dest destination array (will be resized if necessary)
		\return success
	**/
	bool copy(GenericChunkedArray<1, ElementType>& dest) const
	{
		unsigned count = currentSize();
		if (!dest.resize(count))
		{
			return false;
		}
		
		//copy content		
#ifdef CC_ENV_64
		std::copy(m_data.begin(), m_data.end(), dest.m_data.begin());
#else
		unsigned copyCount = 0;
		assert(dest.m_theChunks.size() <= m_theChunks.size());
		for (size_t i=0; i<dest.m_theChunks.size(); ++i)
		{
			unsigned toCopyCount = std::min<unsigned>(count-copyCount,m_perChunkCount[i]);
			assert(dest.m_perChunkCount[i] >= toCopyCount);
			memcpy(dest.m_theChunks[i],m_theChunks[i],toCopyCount*sizeof(ElementType));
			copyCount += toCopyCount;
		}
#endif
		return true;
	}

protected:

	//! GenericChunkedArray default destructor
	/** [SHAREABLE] Call 'release' to destroy this object properly.
	**/
	virtual ~GenericChunkedArray()
	{
#ifndef CC_ENV_64
		while (!m_theChunks.empty())
		{
			delete[] m_theChunks.back();
			m_theChunks.pop_back();
		}
#endif
	}

	//! Minimum values stored in array (along each dimension)
	ElementType m_minVal;

	//! Maximum values stored in array (along each dimension)
	ElementType m_maxVal;

#ifdef CC_ENV_64
	//! Data
	std::vector<ElementType> m_data;
#else
	//! Arrays 'chunks'
	std::vector<ElementType*> m_theChunks;
	//! Elements per chunk
	std::vector<unsigned> m_perChunkCount;
#endif

	//! Total number of elements
	unsigned m_count;
	//! Max total number of elements
	unsigned m_capacity;

	//! Iterator
	unsigned m_iterator;
};

#endif //GENERIC_CHUNKED_ARRAY_HEADER
