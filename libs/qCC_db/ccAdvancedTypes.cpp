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

#include "ccAdvancedTypes.h"

bool NormsIndexesTableType::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (dataVersion < 41)
	{
		//in previous versions (< 41) the normals were compressed on 15 bytes (2*6+3) as unsigned short
		static const unsigned OLD_QUANTIZE_LEVEL = 6;

		ccChunkedArray<1,unsigned short>* oldNormals = new ccChunkedArray<1,unsigned short>();
		if (!ccSerializationHelper::GenericArrayFromFile(*oldNormals,in,dataVersion))
		{
			oldNormals->release();
			return false;
		}

		bool success = false;
		if (resize(oldNormals->currentSize()))
		{
			//convert old normals to new ones
			for (unsigned i=0; i<oldNormals->currentSize(); ++i)
			{
				CCVector3 N;
				//decompress (with the old parameters)
				{
					unsigned short n = oldNormals->getValue(i);
					ccNormalCompressor::Decompress(n, N.u, OLD_QUANTIZE_LEVEL);
				}
				//and recompress
				CompressedNormType index = static_cast<CompressedNormType>(ccNormalCompressor::Compress(N.u));
				setValue(i, index);
			}

			success = true;
		}

		oldNormals->release();

		return success;
	}
	else
	{
		return ccSerializationHelper::GenericArrayFromFile(*this,in,dataVersion);
	}
}
