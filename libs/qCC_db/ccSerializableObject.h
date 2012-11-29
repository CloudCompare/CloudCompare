#ifndef CC_SERIALIZABLE_OBJECT_HEADER
#define CC_SERIALIZABLE_OBJECT_HEADER

//Local
#include "ccLog.h"

//CCLib
#include <GenericChunkedArray.h>

//System
#include <stdio.h>
#include <stdint.h>

//Qt
#include <QFile>

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

	//! Loads data from binay stream
	/** \param in input file (already opened)
		\param dataVersion file version
		\return success
	**/
	virtual bool fromFile(QFile& in, short dataVersion) { return false; }

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

	//! Helper: saves a GenericChunkedArray structure to file
	/** \param chunkArray GenericChunkedArray structure to save (must be allocated)
		\param out output file (must be already opened)
		\return success
	**/
	template <int N, class ScalarType> static bool GenericArrayToFile(const GenericChunkedArray<N,ScalarType>& chunkArray, QFile& out) 
	{
		assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));

		if (!chunkArray.isAllocated())
			return ccSerializableObject::MemoryError();

		//N = components count (dataVersion>=20)
		::uint8_t components = (::uint8_t)N;
		if (out.write((const char*)&components,1)<0)
			return ccSerializableObject::WriteError();

		//array size (dataVersion>=20)
		::uint32_t count = (::uint32_t)chunkArray.currentSize();
		if (out.write((const char*)&count,4)<0)
			return ccSerializableObject::WriteError();

		//array data (dataVersion>=20)
		//--> we write each chunk as a block (faster)
		while (count!=0)
		{
			for (unsigned i=0;i<chunkArray.chunksCount();++i)
			{
				//DGM: since dataVersion>=22, we make sure to write as much items as declared in 'currentSize'!
				unsigned toWrite = std::min<unsigned>(count,chunkArray.chunkSize(i));
				if (out.write((const char*)chunkArray.chunkStartPtr(i),sizeof(ScalarType)*N*toWrite)<0)
					return ccSerializableObject::WriteError();
				assert(toWrite<=count);
				count -= toWrite;
			}
		}

		return true;
	}

	//! Helper: loads a GenericChunkedArray structure from file
	/** \param chunkArray GenericChunkedArray structure to load
		\param in input file (must be already opened)
		\param data version current data version
		\return success
	**/
	template <int N, class ScalarType> static bool GenericArrayFromFile(GenericChunkedArray<N,ScalarType>& chunkArray, QFile& in, short dataVersion) 
	{
		assert(in.isOpen() && (in.openMode() & QIODevice::ReadOnly));

		if (dataVersion<20)
			return ccSerializableObject::CorruptError();

		//N = components count (dataVersion>=20)
		::uint8_t components = 0;
		if (in.read((char*)&components,1)<0)
			return ccSerializableObject::ReadError();
		if (components != N)
			return ccSerializableObject::CorruptError();

		//array size (dataVersion>=20)
		::uint32_t count = 0;
		if (in.read((char*)&count,4)<0)
			return ccSerializableObject::ReadError();

		//try to allocate memory
		if (!chunkArray.resize(count))
			return ccSerializableObject::MemoryError();

		//array data (dataVersion>=20)
		//--> we read each chunk as a block (faster)
		for (unsigned i=0;i<chunkArray.chunksCount();++i)
			if (in.read((char*)chunkArray.chunkStartPtr(i),sizeof(ScalarType)*N*chunkArray.chunkSize(i))<0)
				return ccSerializableObject::ReadError();

		//update array boundaries
		chunkArray.computeMinAndMax();

		return true;
	}
};

#endif
