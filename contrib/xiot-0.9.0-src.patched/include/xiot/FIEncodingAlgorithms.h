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
#ifndef FI_FIENCODINGALGORITHMS_H
#define FI_FIENCODINGALGORITHMS_H

#include <xiot/FITypes.h>
#include <string>

namespace FI {

/**
 * Interface that all algorithms need to implement
 */
class OPENFI_EXPORT IEncodingAlgorithm {
	public:
		/**
		* All algorithms have to implement this method to decode the encoded
		* byte array to it´s XML string representation
		*
		* @octets The byte array containing the encoded data
		* @return The string representation of the decoded input
		*/
		virtual std::string decodeToString(const NonEmptyOctetString &octets) const = 0;
};

/**
 * 10.5 The "int" encoding algorithm
 */
class OPENFI_EXPORT IntEncodingAlgorithm : public IEncodingAlgorithm
{
public:
	/// This encoding algorithm has a vocabulary table index of 4
	static const int ALGORITHM_ID = 4;
	virtual std::string decodeToString(const FI::NonEmptyOctetString &octets) const;
	static void decodeToIntArray(const FI::NonEmptyOctetString &octets, std::vector<int> &vec);
	static void encode(const int* values, size_t size, FI::NonEmptyOctetString &octets);
};

/**
 * 10.7 The "boolean" encoding algorithm
 * @warning Not implemented yet
 */
class OPENFI_EXPORT BooleanEncodingAlgorithm : public IEncodingAlgorithm
{
public:
	/// This encoding algorithm has a vocabulary table index of 6
	static const int ALGORITHM_ID = 6;

	virtual std::string decodeToString(const FI::NonEmptyOctetString &octets) const;
	static void decodeToBoolArray(const FI::NonEmptyOctetString &octets, std::vector<bool> &vec);
};

/**
 * 10.8 The "float" encoding algorithm
 * 
 * @warning Not implemented yet
 */
class OPENFI_EXPORT FloatEncodingAlgorithm : public IEncodingAlgorithm
{
public:
	/// This encoding algorithm has a vocabulary table index of 7
	static const int ALGORITHM_ID = 7;
	virtual std::string decodeToString(const FI::NonEmptyOctetString &octets) const;
	static void decodeToFloatArray(const FI::NonEmptyOctetString &octets, std::vector<float> &vec);
	static void encode(const float* values, size_t size, FI::NonEmptyOctetString &octets);
};


/**
 * Some helpers to convert bytes into other datatypes
 */
class Tools {
public:
	union float_to_unsigned_int_to_bytes
		{
			float f;
			unsigned int ui;
			unsigned char ub[4]; // unsigned bytes
		};

	/// converts the next 4 bytes of the input pointer to one 4 byte 
	/// unsigned int swapping the positions of the bytes
	inline static unsigned int readUInt(const unsigned char* bytes)
	{
		float_to_unsigned_int_to_bytes v;
		v.ub[0] = bytes[3];
		v.ub[1] = bytes[2];
		v.ub[2] = bytes[1];
		v.ub[3] = bytes[0];
		return v.ui;
	};

	/// converts the next 4 bytes of the input pointer to one 4 byte float
	/// swapping the positions of the bytes
	inline static float readFloat(const unsigned char* bytes)
	{
		float_to_unsigned_int_to_bytes v;
		v.ub[0] = bytes[3];
		v.ub[1] = bytes[2];
		v.ub[2] = bytes[1];
		v.ub[3] = bytes[0];
		return v.f;
	};

	inline static int reverseBytes(const int* x) {
		/* break x apart, then put it back together backwards */
		int part1 = (*x)  & 0xFF;
		int part2 = ((*x) >> 8) & 0xFF;
		int part3 = ((*x) >> 16) & 0xFF;
		int part4 = ((*x) >> 24) & 0xFF;
		return (part1 << 24) | ( part2 << 16) | (part3 << 8) | part4;
  }

};

}; // End namespace X3D;


#endif

