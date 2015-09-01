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

#ifndef CC_NORMAL_COMPRESSOR_HEADER
#define CC_NORMAL_COMPRESSOR_HEADER

//CCLib
#include <CCTypes.h>

//Local
#include "qCC_db.h"
#include "ccBasicTypes.h"

//! Normal compressor
class QCC_DB_LIB_API ccNormalCompressor
{
public:

	//! Compressed normals quantization level (number of directions: 2^(2*N+3))
	static const unsigned char QUANTIZE_LEVEL = 9; //2097152 normals * 12 bytes = 24 Mb of memory

	//! Compression algorithm
	static unsigned Compress(const PointCoordinateType N[3], unsigned char level = QUANTIZE_LEVEL);

	//! Decompression algorithm
	static void Decompress(unsigned index, PointCoordinateType N[3], unsigned char level = QUANTIZE_LEVEL);

	//! Inverts a (compressed) normal
	inline static void InvertNormal(CompressedNormType &code) { code ^= (static_cast<CompressedNormType>(7) << 2*QUANTIZE_LEVEL); } //See 'Decompress' for a better understanding

};

 #endif //CC_NORMAL_COMPRESSOR_HEADER
