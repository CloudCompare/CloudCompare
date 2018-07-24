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

#ifndef CC_ADVANCED_TYPES_HEADER
#define CC_ADVANCED_TYPES_HEADER

//Local
#include "ccArray.h"
#include "ccNormalCompressor.h"
#include "ccColorTypes.h"

/***************************************************
	  Advanced cloudCompare types (containers)
***************************************************/

//! Array of compressed 3D normals (single index)
class NormsIndexesTableType : public ccArray<CompressedNormType, 1, CompressedNormType>
{
public:
	//! Default constructor
	NormsIndexesTableType() : ccArray<CompressedNormType, 1, CompressedNormType>("Compressed normals") {}

	//inherited from ccArray/ccHObject
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::NORMAL_INDEXES_ARRAY; }

	//! Duplicates array (overloaded from ccArray::clone)
	virtual NormsIndexesTableType* clone() override
	{
		NormsIndexesTableType* cloneArray = new NormsIndexesTableType();
		if (!copy(*cloneArray))
		{
			ccLog::Warning("[NormsIndexesTableType::clone] Failed to clone array (not enough memory)");
			cloneArray->release();
			return 0;
		}
		cloneArray->setName(getName());
		return cloneArray;
	}

	//inherited from ccHObject/ccArray
	QCC_DB_LIB_API virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags) override;
};

//! Array of (uncompressed) 3D normals (Nx,Ny,Nz)
class NormsTableType : public ccArray<CCVector3, 3,PointCoordinateType>
{
public:
	//! Default constructor
	NormsTableType() : ccArray<CCVector3, 3, PointCoordinateType>("Normals") {}

	//inherited from ccArray/ccHObject
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::NORMALS_ARRAY; }

	//! Duplicates array (overloaded from ccArray::clone)
	virtual NormsTableType* clone() override
	{
		NormsTableType* cloneArray = new NormsTableType();
		if (!copy(*cloneArray))
		{
			ccLog::Warning("[NormsTableType::clone] Failed to clone array (not enough memory)");
			cloneArray->release();
			return 0;
		}
		cloneArray->setName(getName());
		return cloneArray;
	}
};

//! Array of RGB colors for each point
class ColorsTableType : public ccArray<ccColor::Rgb, 3, ColorCompType>
{
public:
	//! Default constructor
	ColorsTableType() : ccArray<ccColor::Rgb, 3, ColorCompType>("RGB colors") {}

	//inherited from ccArray/ccHObject
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::RGB_COLOR_ARRAY; }

	//! Duplicates array (overloaded from ccArray::clone)
	virtual ColorsTableType* clone() override
	{
		ColorsTableType* cloneArray = new ColorsTableType();
		if (!copy(*cloneArray))
		{
			ccLog::Warning("[ColorsTableType::clone] Failed to clone array (not enough memory)");
			cloneArray->release();
			return 0;
		}
		cloneArray->setName(getName());
		return cloneArray;
	}
};

//! 2D texture coordinates
struct TexCoords2D
{
	TexCoords2D() : tx(-1.0f), ty(-1.0f) {}
	TexCoords2D(float x, float y) : tx(x), ty(y) {}

	union
	{
		struct
		{
			float tx, ty;
		};
		float t[2];
	};
};

//! Array of 2D texture coordinates
class TextureCoordsContainer : public ccArray<TexCoords2D, 2, float>
{
public:
	//! Default constructor
	TextureCoordsContainer() : ccArray<TexCoords2D, 2, float>("Texture coordinates") {}

	//inherited from ccArray/ccHObject
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::TEX_COORDS_ARRAY; }

	//! Duplicates array (overloaded from ccArray::clone)
	virtual TextureCoordsContainer* clone() override
	{
		TextureCoordsContainer* cloneArray = new TextureCoordsContainer();
		if (!copy(*cloneArray))
		{
			ccLog::Warning("[TextureCoordsContainer::clone] Failed to clone array (not enough memory)");
			cloneArray->release();
			return 0;
		}
		cloneArray->setName(getName());
		return cloneArray;
	}
};

#endif //CC_ADVANCED_TYPES_HEADER
