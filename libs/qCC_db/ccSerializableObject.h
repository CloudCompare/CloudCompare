//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_SERIALIZABLE_OBJECT_HEADER
#define CC_SERIALIZABLE_OBJECT_HEADER

//Local
#include "ccLog.h"

//CCLib
#include <GenericChunkedArray.h>
#include <CCTypes.h>

//System
#include <stdio.h>
#include <stdint.h>

//Qt
#include <QFile>
#include <QDataStream>

//! Serializable object interface
class ccSerializableObject
{
public:

	//! Desctructor
	virtual ~ccSerializableObject() {}

	//! Returns whether object is serializable of not
	virtual bool isSerializable() const { return false; }

	//! Saves data to binay stream
	/** \param out output file (already opened)
		\return success
	**/
	virtual bool toFile(QFile& out) const { return false; }

	//! Deserialization flags (bit-field)
	enum DeserializationFlags
	{
		DF_POINT_COORDS_64_BITS	= 1, /**< Point coordinates are stored as 64 bits double (otherwise 32 bits floats) **/
		//DGM: inversion is 'historical' ;)
		DF_SCALAR_VAL_32_BITS	= 2, /**< Scalar values are stored as 32 bits floats (otherwise 64 bits double) **/
	};

	//! Loads data from binay stream
	/** \param in input file (already opened)
		\param dataVersion file version
		\param flags deserialization flags (see ccSerializableObject::DeserializationFlags)
		\return success
	**/
	virtual bool fromFile(QFile& in, short dataVersion, int flags) { return false; }

	//! Sends a custom error message (write error) and returns 'false'
	/** Shortcut for returning a standardized error message in the toFile method.
		\return always false
	**/
	static bool WriteError() { ccLog::Error("Write error (disk full or no access right?)"); return false; }

	//! Sends a custom error message (read error) and returns 'false'
	/** Shortcut for returning a standardized error message in the fromFile method.
		\return always false
	**/
	static bool ReadError() { ccLog::Error("Read error (corrupted file or no access right?)"); return false; }

	//! Sends a custom error message (not enough memory) and returns 'false'
	/** Shortcut for returning a standardized error message in the fromFile method.
		\return always false
	**/
	static bool MemoryError() { ccLog::Error("Not enough memory"); return false; }

	//! Sends a custom error message (corrupted file) and returns 'false'
	/** Shortcut for returning a standardized error message in the fromFile method.
		\return always false
	**/
	static bool CorruptError() { ccLog::Error("File seems to be corrupted"); return false; }
};

//! Serialization helpers
class ccSerializationHelper
{
public:

	//! Reads one or several 'PointCoordinateType' values from a QDataStream either in float or double format depending on the 'flag' value
	static void CoordsFromDataStream(QDataStream& stream, int flags, PointCoordinateType* out, unsigned count = 1)
	{
		if (flags & ccSerializableObject::DF_POINT_COORDS_64_BITS)
		{
			for (unsigned i=0; i<count; ++i, ++out)
			{
				double val;
				stream >> val;
				*out = static_cast<PointCoordinateType>(val);
			}
		}
		else
		{
			for (unsigned i=0; i<count; ++i, ++out)
			{
				float val;
				stream >> val;
				*out = static_cast<PointCoordinateType>(val);
			}
		}
	}

	//! Reads one or several 'ScalarType' values from a QDataStream either in float or double format depending on the 'flag' value
	static void ScalarsFromDataStream(QDataStream& stream, int flags, ScalarType* out, unsigned count = 1)
	{
		if (flags & ccSerializableObject::DF_SCALAR_VAL_32_BITS)
		{
			for (unsigned i=0; i<count; ++i, ++out)
			{
				float val;
				stream >> val;
				*out = static_cast<PointCoordinateType>(val);
			}
		}
		else
		{
			for (unsigned i=0; i<count; ++i, ++out)
			{
				double val;
				stream >> val;
				*out = static_cast<PointCoordinateType>(val);
			}
		}
	}

	//! Helper: saves a GenericChunkedArray structure to file
	/** \param chunkArray GenericChunkedArray structure to save (must be allocated)
		\param out output file (must be already opened)
		\return success
	**/
	template <int N, class ElementType> static bool GenericArrayToFile(const GenericChunkedArray<N,ElementType>& chunkArray, QFile& out) 
	{
		assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));

		if (!chunkArray.isAllocated())
			return ccSerializableObject::MemoryError();

		//component count (dataVersion>=20)
		::uint8_t componentCount = static_cast< ::uint8_t >(N);
		if (out.write((const char*)&componentCount,1) < 0)
			return ccSerializableObject::WriteError();

		//element count = array size (dataVersion>=20)
		::uint32_t elementCount = static_cast< ::uint32_t >(chunkArray.currentSize());
		if (out.write((const char*)&elementCount,4) < 0)
			return ccSerializableObject::WriteError();

		//array data (dataVersion>=20)
		{
#ifdef CC_ENV_64
			if (out.write((const char*)chunkArray.data(),sizeof(ElementType)*N*chunkArray.currentSize()) < 0)
				return ccSerializableObject::WriteError();
#else
			//--> we write each chunk as a block (faster)
			while (elementCount != 0)
			{
				unsigned chunksCount = chunkArray.chunksCount();
				for (unsigned i=0; i<chunksCount; ++i)
				{
					//DGM: since dataVersion>=22, we make sure to write as much items as declared in 'currentSize'!
					unsigned toWrite = std::min<unsigned>(elementCount,chunkArray.chunkSize(i));
					if (out.write((const char*)chunkArray.chunkStartPtr(i),sizeof(ElementType)*N*toWrite) < 0)
						return ccSerializableObject::WriteError();
					assert(toWrite <= elementCount);
					elementCount -= toWrite;
				}
			}
#endif //CC_ENV_64
		}
		return true;
	}

	//! Helper: loads a GenericChunkedArray structure from file
	/** \param chunkArray GenericChunkedArray structure to load
		\param in input file (must be already opened)
		\param dataVersion version current data version
		\return success
	**/
	template <int N, class ElementType> static bool GenericArrayFromFile(GenericChunkedArray<N,ElementType>& chunkArray, QFile& in, short dataVersion) 
	{
		::uint8_t componentCount = 0;
		::uint32_t elementCount = 0;
		if (!ReadArrayHeader(in,dataVersion,componentCount,elementCount))
			return false;
		if (componentCount != N)
			return ccSerializableObject::CorruptError();

		if (elementCount)
		{
			//try to allocate memory
			if (!chunkArray.resize(elementCount))
				return ccSerializableObject::MemoryError();

			//array data (dataVersion>=20)
			{
#ifdef CC_ENV_64
				//Apparently Qt and/or Windows don't like to read too many bytes in a row...
				static const qint64 MaxElementPerChunk = (static_cast<qint64>(1) << 24);
				qint64 byteCount = static_cast<qint64>(sizeof(ElementType)*N) * chunkArray.currentSize();
				char* dest = (char*)chunkArray.data();
				while (byteCount > 0)
				{
					qint64 chunkSize = std::min(MaxElementPerChunk, byteCount);
					if (in.read(dest, chunkSize) < 0)
						return ccSerializableObject::ReadError();
					byteCount -= chunkSize;
					dest += chunkSize;
				}
#else
				//--> we read each chunk as a block (faster)
				unsigned chunksCount = chunkArray.chunksCount();
				for (unsigned i=0; i<chunksCount; ++i)
					if (in.read((char*)chunkArray.chunkStartPtr(i),sizeof(ElementType)*N*chunkArray.chunkSize(i)) < 0)
						return ccSerializableObject::ReadError();
#endif //CC_ENV_64
			}

			//update array boundaries
			chunkArray.computeMinAndMax();
		}

		return true;
	}

	//! Helper: loads a GenericChunkedArray structure from a file stored with a different type
	/** \param chunkArray GenericChunkedArray structure to load
		\param in input file (must be already opened)
		\param dataVersion version current data version
		\return success
	**/
	template <int N, class ElementType, class FileElementType> static bool GenericArrayFromTypedFile(GenericChunkedArray<N,ElementType>& chunkArray, QFile& in, short dataVersion)
	{
		::uint8_t componentCount = 0;
		::uint32_t elementCount = 0;
		if (!ReadArrayHeader(in,dataVersion,componentCount,elementCount))
			return false;
		if (componentCount != N)
			return ccSerializableObject::CorruptError();

		if (elementCount)
		{
			//try to allocate memory
			if (!chunkArray.resize(elementCount))
				return ccSerializableObject::MemoryError();

			//array data (dataVersion>=20)
			//--> saldy we can't read it as a block...
			//we must convert each element, value by value!
			FileElementType dummyArray[N] = {0};
#ifdef CC_ENV_64
			ElementType* data = chunkArray.data();
			for (unsigned i=0; i<elementCount; ++i)
			{
				if (in.read((char*)dummyArray,sizeof(FileElementType)*N) >= 0)
				{
					for (unsigned k=0; k<N; ++k)
						*data++ = static_cast<ElementType>(dummyArray[k]);
				}
				else
				{
					return ccSerializableObject::ReadError();
				}
			}
#else
			unsigned chunksCount = chunkArray.chunksCount();
			for (unsigned i=0; i<chunksCount; ++i)
			{
				unsigned chunkSize = chunkArray.chunkSize(i);
				ElementType* chunkStart = chunkArray.chunkStartPtr(i);
				for (unsigned j=0; j<chunkSize; ++j)
				{
					if (in.read((char*)dummyArray,sizeof(FileElementType)*N) >= 0)
					{
						for (unsigned k=0; k<N; ++k)
							*chunkStart++ = static_cast<ElementType>(dummyArray[k]);
					}
					else
					{
						return ccSerializableObject::ReadError();
					}
				}
			}
#endif //CC_ENV_64

			//update array boundaries
			chunkArray.computeMinAndMax();
		}

		return true;
	}

protected:

	static bool ReadArrayHeader(QFile& in,
								short dataVersion,
								::uint8_t &componentCount,
								::uint32_t &elementCount)
	{
		assert(in.isOpen() && (in.openMode() & QIODevice::ReadOnly));

		if (dataVersion < 20)
			return ccSerializableObject::CorruptError();

		//component count (dataVersion>=20)
		if (in.read((char*)&componentCount,1) < 0)
			return ccSerializableObject::ReadError();

		//element count = array size (dataVersion>=20)
		if (in.read((char*)&elementCount,4) < 0)
			return ccSerializableObject::ReadError();

		return true;
	}
};

#endif //CC_SERIALIZABLE_OBJECT_HEADER
