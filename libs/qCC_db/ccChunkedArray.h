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

#ifndef CC_CHUNK_ARRAY_HEADER
#define CC_CHUNK_ARRAY_HEADER

//Local
#include "ccHObject.h"

//CCLib
#include <GenericChunkedArray.h>

//! Shareable 'chunked' array that can be properly inserted in the DB tree
template <int N, class ElementType> class ccChunkedArray : public GenericChunkedArray<N,ElementType>, public ccHObject
{
public:

	//! Default constructor
	ccChunkedArray(QString name = QString())
		: GenericChunkedArray<N,ElementType>()
		, ccHObject(name)
	{
		setFlagState(CC_LOCKED,true);
	}

	//! Duplicates array
	virtual ccChunkedArray<N,ElementType>* clone()
	{
		ccChunkedArray<N,ElementType>* cloneArray = new ccChunkedArray<N,ElementType>(getName());
		if (!this->copy(*cloneArray))
		{
			ccLog::Error("[ccChunkedArray::clone] Failed to clone array (not enough memory?)");
			cloneArray->release();
			cloneArray = 0;
		}
		return cloneArray;
	}

	//inherited from ccHObject
	virtual CC_CLASS_ENUM getClassID() const { return CC_TYPES::CHUNKED_ARRAY; }
	virtual bool isShareable() const { return true; }
	virtual bool isSerializable() const { return true; }

protected:

	//inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const { return ccSerializationHelper::GenericArrayToFile(*this,out); }
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags) { return ccSerializationHelper::GenericArrayFromFile(*this,in,dataVersion); }

};

#endif //CC_CHUNK_ARRAY_HEADER
