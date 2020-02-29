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

#include "ccAdvancedTypes.h"

NormsIndexesTableType::NormsIndexesTableType()
	: ccArray<CompressedNormType, 1, CompressedNormType>("Compressed normals")
{
}

bool NormsIndexesTableType::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (dataVersion < 41)
	{
		//in previous versions (< 41) the normals were compressed on 15 bytes (2*6+3) as unsigned short
		static const unsigned OLD_QUANTIZE_LEVEL = 6;

		ccArray<unsigned short, 1, unsigned short>* oldNormals = new ccArray<unsigned short, 1, unsigned short>();
		if (!ccSerializationHelper::GenericArrayFromFile<unsigned short, 1, unsigned short>(*oldNormals, in, dataVersion))
		{
			oldNormals->release();
			return false;
		}

		bool success = false;
		try
		{
			resize(oldNormals->size());
		}
		catch (const std::bad_alloc&)
		{
			oldNormals->release();
			return false;
		}

		//convert old normals to new ones
		for (size_t i = 0; i < oldNormals->size(); ++i)
		{
			CCVector3 N;
			//decompress (with the old parameters)
			{
				unsigned short n = oldNormals->at(i);
				ccNormalCompressor::Decompress(n, N.u, OLD_QUANTIZE_LEVEL);
			}
			//and recompress
			CompressedNormType index = static_cast<CompressedNormType>(ccNormalCompressor::Compress(N.u));
			at(i) = index;
		}

		oldNormals->release();
		return true;
	}
	else
	{
		return ccSerializationHelper::GenericArrayFromFile<CompressedNormType, 1, CompressedNormType>(*this, in, dataVersion);
	}
}
