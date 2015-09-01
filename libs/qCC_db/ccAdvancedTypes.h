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

#ifndef CC_ADVANCED_TYPES_HEADER
#define CC_ADVANCED_TYPES_HEADER

//Local
#include "qCC_db.h"
#include "ccBasicTypes.h"
#include "ccChunkedArray.h"
#include "ccNormalCompressor.h"

/***************************************************
	  Advanced cloudCompare types (containers)
***************************************************/

//! Array of compressed 3D normals (single index)
class QCC_DB_LIB_API NormsIndexesTableType : public ccChunkedArray<1,CompressedNormType>
{
public:
	//! Default constructor
	NormsIndexesTableType() : ccChunkedArray<1,CompressedNormType>("Compressed normals") {}

	//inherited from ccChunkedArray/ccHObject
	virtual CC_CLASS_ENUM getClassID() const { return CC_TYPES::NORMAL_INDEXES_ARRAY; }

	//! Duplicates array (overloaded from ccChunkedArray::clone
	virtual NormsIndexesTableType* clone()
	{
		NormsIndexesTableType* cloneArray = new NormsIndexesTableType();
		if (!copy(*cloneArray))
		{
			ccLog::Error("[NormsIndexesTableType::clone] Failed to clone array (not enough memory?)");
			cloneArray->release();
			return 0;
		}
		cloneArray->setName(getName());
		return cloneArray;
	}

	//inherited from ccHObject/ccChunkedArray
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags);
};

//! Array of (uncompressed) 3D normals (Nx,Ny,Nz)
class QCC_DB_LIB_API NormsTableType : public ccChunkedArray<3,PointCoordinateType>
{
public:
	//! Default constructor
	NormsTableType() : ccChunkedArray<3,PointCoordinateType>("Normals") {}

	//inherited from ccChunkedArray/ccHObject
	virtual CC_CLASS_ENUM getClassID() const { return CC_TYPES::NORMALS_ARRAY; }

	//! Duplicates array (overloaded from ccChunkedArray::clone
	virtual NormsTableType* clone()
	{
		NormsTableType* cloneArray = new NormsTableType();
		if (!copy(*cloneArray))
		{
			ccLog::Error("[NormsTableType::clone] Failed to clone array (not enough memory?)");
			cloneArray->release();
			return 0;
		}
		cloneArray->setName(getName());
		return cloneArray;
	}
};

//! Array of RGB colors for each point
class QCC_DB_LIB_API ColorsTableType : public ccChunkedArray<3,ColorCompType>
{
public:
	//! Default constructor
	ColorsTableType() : ccChunkedArray<3,ColorCompType>("RGB colors") {}

	//inherited from ccChunkedArray/ccHObject
	virtual CC_CLASS_ENUM getClassID() const { return CC_TYPES::RGB_COLOR_ARRAY; }

	//! Duplicates array (overloaded from ccChunkedArray::clone
	virtual ColorsTableType* clone()
	{
		ColorsTableType* cloneArray = new ColorsTableType();
		if (!copy(*cloneArray))
		{
			ccLog::Error("[ColorsTableType::clone] Failed to clone array (not enough memory?)");
			cloneArray->release();
			return 0;
		}
		cloneArray->setName(getName());
		return cloneArray;
	}
};

//! Array of 2D texture coordinates
class QCC_DB_LIB_API TextureCoordsContainer : public ccChunkedArray<2,float>
{
public:
	//! Default constructor
	TextureCoordsContainer() : ccChunkedArray<2,float>("Texture coordinates") {}

	//inherited from ccChunkedArray/ccHObject
	virtual CC_CLASS_ENUM getClassID() const { return CC_TYPES::TEX_COORDS_ARRAY; }

	//! Duplicates array (overloaded from ccChunkedArray::clone
	virtual TextureCoordsContainer* clone()
	{
		TextureCoordsContainer* cloneArray = new TextureCoordsContainer();
		if (!copy(*cloneArray))
		{
			ccLog::Error("[TextureCoordsContainer::clone] Failed to clone array (not enough memory?)");
			cloneArray->release();
			return 0;
		}
		cloneArray->setName(getName());
		return cloneArray;
	}
};

#endif //CC_ADVANCED_TYPES_HEADER
