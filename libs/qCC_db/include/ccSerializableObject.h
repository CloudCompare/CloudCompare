//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
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
#include <CCPlatform.h>
#include <CCTypes.h>

//System
#include <cassert>
#include <cstdint>

//Qt
#include <QDataStream>
#include <QMultiMap>
#include <QFile>

//! Serializable object interface
class ccSerializableObject
{
public:

	//! Desctructor
	virtual ~ccSerializableObject() = default;

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

	//! Map of loaded uniqie IDs (old ID --> new ID)
	typedef QMultiMap<unsigned, unsigned> LoadedIDMap;

	//! Loads data from binay stream
	/** \param in input file (already opened)
		\param dataVersion file version
		\param flags deserialization flags (see ccSerializableObject::DeserializationFlags)
		\return success
	**/
	virtual bool fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) { return false; }

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
			for (unsigned i = 0; i < count; ++i, ++out)
			{
				double val;
				stream >> val;
				*out = static_cast<PointCoordinateType>(val);
			}
		}
		else
		{
			for (unsigned i = 0; i < count; ++i, ++out)
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
			for (unsigned i = 0; i < count; ++i, ++out)
			{
				float val;
				stream >> val;
				*out = static_cast<PointCoordinateType>(val);
			}
		}
		else
		{
			for (unsigned i = 0; i < count; ++i, ++out)
			{
				double val;
				stream >> val;
				*out = static_cast<PointCoordinateType>(val);
			}
		}
	}

	//! Helper: saves a vector to file
	/** \param data vector to save (must be allocated)
		\param out output file (must be already opened)
		\return success
	**/
	template <class Type, int N, class ComponentType> static bool GenericArrayToFile(const std::vector<Type>& data, QFile& out)
	{
		assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));
		
		//removed to allow saving empty clouds
		//if (data.empty())
		//{
		//	return ccSerializableObject::MemoryError();
		//}

		//component count (dataVersion>=20)
		::uint8_t componentCount = static_cast<::uint8_t>(N);
		if (out.write((const char*)&componentCount, 1) < 0)
			return ccSerializableObject::WriteError();

		//element count = array size (dataVersion>=20)
		::uint32_t elementCount = static_cast<::uint32_t>(data.size());
		if (out.write((const char*)&elementCount, 4) < 0)
			return ccSerializableObject::WriteError();

		//array data (dataVersion>=20)
		{
			//DGM: do it by chunks, in case it's too big to be processed by the system
			const char* _data = (const char*)data.data();
			qint64 byteCount = static_cast<qint64>(elementCount);
			byteCount *= sizeof(Type);
			while (byteCount != 0)
			{
				static const qint64 s_maxByteSaveCount = (1 << 26); //64 Mb each time
				qint64 saveCount = std::min(byteCount, s_maxByteSaveCount);
				if (out.write(_data, saveCount) < 0)
					return ccSerializableObject::WriteError();
				_data += saveCount;
				byteCount -= saveCount;
			}
		}
		return true;
	}

	//! Helper: loads a vector structure from file
	/** \param data vector to load
		\param in input file (must be already opened)
		\param dataVersion version current data version
		\return success
	**/
	template <class Type, int N, class ComponentType> static bool GenericArrayFromFile(std::vector<Type>& data, QFile& in, short dataVersion)
	{
		::uint8_t componentCount = 0;
		::uint32_t elementCount = 0;
		if (!ReadArrayHeader(in, dataVersion, componentCount, elementCount))
		{
			return false;
		}
		if (componentCount != N)
		{
			return ccSerializableObject::CorruptError();
		}

		if (elementCount)
		{
			//try to allocate memory
			try
			{
				data.resize(elementCount);
			}
			catch (const std::bad_alloc&)
			{
				return ccSerializableObject::MemoryError();
			}

			//array data (dataVersion>=20)
			{
				//Apparently Qt and/or Windows don't like to read too many bytes in a row...
				static const qint64 MaxElementPerChunk = (static_cast<qint64>(1) << 24);
				assert(sizeof(ComponentType) * N == sizeof(Type));
				qint64 byteCount = static_cast<qint64>(data.size()) * (sizeof(ComponentType) * N);
				char* dest = (char*)data.data();
				while (byteCount > 0)
				{
					qint64 chunkSize = std::min(MaxElementPerChunk, byteCount);
					if (in.read(dest, chunkSize) < 0)
					{
						return ccSerializableObject::ReadError();
					}
					byteCount -= chunkSize;
					dest += chunkSize;
				}
			}
		}

		return true;
	}

	//! Helper: loads a vector structure from a file stored with a different type
	/** \param data vector to load
		\param in input file (must be already opened)
		\param dataVersion version current data version
		\return success
	**/
	template <class Type, int N, class ComponentType, class FileComponentType> static bool GenericArrayFromTypedFile(std::vector<Type>& data, QFile& in, short dataVersion)
	{
		::uint8_t componentCount = 0;
		::uint32_t elementCount = 0;
		if (!ReadArrayHeader(in, dataVersion, componentCount, elementCount))
		{
			return false;
		}
		if (componentCount != N)
		{
			return ccSerializableObject::CorruptError();
		}

		if (elementCount)
		{
			//try to allocate memory
			try
			{
				data.resize(elementCount);
			}
			catch (const std::bad_alloc&)
			{
				return ccSerializableObject::MemoryError();
			}

			//array data (dataVersion>=20)
			//--> saldy we can't read it as a block...
			//we must convert each element, value by value!
			FileComponentType dummyArray[N] = { 0 };

			ComponentType* _data = (ComponentType*)data.data();
			for (unsigned i = 0; i < elementCount; ++i)
			{
				if (in.read((char*)dummyArray, sizeof(FileComponentType) * N) >= 0)
				{
					for (unsigned k = 0; k < N; ++k)
					{
						*_data++ = static_cast<ComponentType>(dummyArray[k]);
					}
				}
				else
				{
					return ccSerializableObject::ReadError();
				}
			}
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
		if (in.read((char*)&componentCount, 1) < 0)
			return ccSerializableObject::ReadError();

		//element count = array size (dataVersion>=20)
		if (in.read((char*)&elementCount, 4) < 0)
			return ccSerializableObject::ReadError();

		return true;
	}
};

#endif //CC_SERIALIZABLE_OBJECT_HEADER
