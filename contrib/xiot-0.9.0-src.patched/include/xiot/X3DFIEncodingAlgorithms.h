/*=========================================================================
     This file is part of the XIOT library.

     Copyright (C) 2008-2009 EDF R&D
     Author: Kristian Sons (xiot@actor3d.com)

     This library is free software; you can redistribute it and/or modify
     it under the terms of the GNU Lesser Public License as published by
     the Free Software Foundation; either version 2.1 of the License, or
     (at your option) any later version.

     The XIOT library is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU Lesser Public License for more details.

     You should have received a copy of the GNU Lesser Public License
     along with XIOT; if not, write to the Free Software
     Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
     MA 02110-1301  USA
=========================================================================*/
#ifndef X3D_X3DFIENCODINGALGORITHMS_H
#define X3D_X3DFIENCODINGALGORITHMS_H

#include <xiot/X3DTypes.h>
#include <xiot/FIEncodingAlgorithms.h>
#include <string>

namespace XIOT {

/**
 * Encoding algorithm to encode/decode arrays of type int.
 *
 * This algorithm implements the Delta zlib integer array encoder
 * as described in 5.5.2 in ISO/IEC 19776-3:2007 - Part 3
 *
 * This encoding applies to MFInt32, SFImage and MFImage fields
 *
 * @link(http://www.web3d.org/x3d/specifications/ISO-IEC-19776-3-X3DEncodings-CompressedBinaryEncoding/Part03/EncodingOfFields.html#DeltazlibIntegerArrayEncoder)
 *
 * The URI for identifying this encoder is: "encoder://web3d.org/DeltazlibIntArrayEncoder"
 *
 * @ingroup x3dloader
 */
class XIOT_EXPORT DeltazlibIntArrayAlgorithm : public FI::IEncodingAlgorithm
{
public:
	/// The position of this algorithm in the FI algorithm table.
	static const int ALGORITHM_ID = 34; 

	virtual std::string decodeToString(const FI::NonEmptyOctetString &octets) const;

	/**
	 * Decodes a given vector of unsigned chars to a vector of int using the 
	 * Delta zlib integer array decoder.
	 */
	static void decodeToIntArray(const FI::NonEmptyOctetString &octets,  std::vector<int> &vec);

	static void encode(const int* values, size_t size, FI::NonEmptyOctetString &octets, bool isImage = false);
	
};

/**
 * Encoding algorithm to encode/decode arrays of type float.
 *
 * This algorithm implements the Quantized zlib float array encoder
 * as described in 5.4.3 in ISO/IEC 19776-3:2007 - Part 3
 *
 * @link(http://www.web3d.org/x3d/specifications/ISO-IEC-19776-3-X3DEncodings-CompressedBinaryEncoding/Part03/EncodingOfFields.html#DeltazlibIntegerArrayEncoder)
 *
 * The URI for identifying this encoder is:  "encoder://web3d.org/QuantizedDoubleArrayEncoder"
 *
 * @ingroup x3dloader
 */
class XIOT_EXPORT QuantizedzlibFloatArrayAlgorithm : public FI::IEncodingAlgorithm
{
public:
	/// The position of this algorithm in the FI algorithm table.
	static const int ALGORITHM_ID = 35; 

	virtual std::string decodeToString(const FI::NonEmptyOctetString &octets) const;

	/**
	 * Decodes a given vector of unsigned chars to a vector of float using the 
	 * Quantized zlib float array encoder.
	 */
	static void decodeToFloatArray(const FI::NonEmptyOctetString &octets, std::vector<float> &vec);

	static void encode(const float* values, size_t size, FI::NonEmptyOctetString &octets);
	
};




}; // End namespace X3D;


#endif

