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
#ifndef FI_FICONSTANTS_H
#define FI_FICONSTANTS_H

namespace FI {

/// Some usefull constants for binary parsing
struct Constants {
	// Header
	static const unsigned char IDENT1 = 0xE0;
	static const unsigned char IDENT2 = 0x00;
	static const unsigned char VERSION1 = 0x00;
	static const unsigned char VERSION2 = 0x01;

	static const unsigned char NOTATIONS_ID = 0xC0;
	static const unsigned char UNPARSED_ENTITIES_ID = 0xC0;

	// Masks (better name for this?)
	static const unsigned char TWO_BITS = 0xC0;
	static const unsigned char FOUR_BITS = 0xF0;
	static const unsigned char SIX_BITS = 0xFC;
	static const unsigned char SEVEN_BITS = 0xFE;
	static const unsigned char LAST_TWO_BITS = 3;
	static const unsigned char LAST_THREE_BITS = 7;
	static const unsigned char LAST_FOUR_BITS = 0xF;
	static const unsigned char LAST_FIVE_BITS = 0x1F;
	static const unsigned char LAST_SIX_BITS = 0x3F;
	static const unsigned char LAST_SEVEN_BITS = 0x7F;
	static const unsigned char THREE_TO_FIVE = 0x38;

	// Element
	static const unsigned char ELEMENT_NAMESPACE_ATTRIBUTES_MASK = 0x3F;
	static const unsigned char ELEMENT_NAMESPACE_ATTRIBUTES_FLAG = 0x38;
	static const unsigned char ELEMENT_LITERAL_QNAME_FLAG = 0x3C;
	static const unsigned char ELEMENT_CHARACTER_CHUNK = 0x80;

	// Attributes
	static const unsigned char ATTRIBUTE_LITERAL_QNAME_FLAG = 0x78;
	
	// Data
	static const unsigned char NON_EMPTY_OCTET_STRING_2ND_MEDIUM = 0x40;
	static const unsigned char NON_EMPTY_OCTET_STRING_2ND_LARGE = 0x60;

	static const unsigned char NON_EMPTY_OCTET_STRING_5TH_MEDIUM = 0x08;
	static const unsigned char NON_EMPTY_OCTET_STRING_5TH_LARGE = 0x0C;

	static const unsigned char NON_EMPTY_OCTET_STRING_7TH_MEDIUM = 2;
	static const unsigned char NON_EMPTY_OCTET_STRING_7TH_LARGE = 3;

	static const unsigned char ENCODED_CHARACTER_STRING_3RD_MASK = 0x30;
	static const unsigned char ENCODED_CHARACTER_STRING_3RD_UTF8 = 0x00;
	static const unsigned char ENCODED_CHARACTER_STRING_3RD_UTF16 = 0x10;
	static const unsigned char ENCODED_CHARACTER_STRING_3RD_RESTRICTED_ALPHABET = 0x20;
	static const unsigned char ENCODED_CHARACTER_STRING_3RD_ENCODING_ALGORITHM = 0x30;

	static const unsigned char ENCODED_CHARACTER_STRING_5TH_MASK = 0x0C;
	static const unsigned char ENCODED_CHARACTER_STRING_5TH_UTF8 = 0;
	static const unsigned char ENCODED_CHARACTER_STRING_5TH_UTF16 = 4;
	static const unsigned char ENCODED_CHARACTER_STRING_5TH_RESTRICTED_ALPHABET = 8;
	static const unsigned char ENCODED_CHARACTER_STRING_5TH_ENCODING_ALGORITHM = 0x0C;

	static const unsigned char INTEGER_4TH_LENGTH_MASK = 0x1C;
	static const unsigned char INTEGER_4TH_LENGTH_SMALL = 0x10;
	static const unsigned char INTEGER_4TH_LENGTH_MEDIUM = 0x14;
	static const unsigned char INTEGER_4TH_LENGTH_LARGE = 0x18;

	static const unsigned char INTEGER_3RD_LENGTH_MASK = 0x38;
	static const unsigned char INTEGER_3RD_LENGTH_SMALL = 0x20;
	static const unsigned char INTEGER_3RD_LENGTH_MEDIUM = 0x28;
	static const unsigned char INTEGER_3RD_LENGTH_LARGE = 0x30;

	static const unsigned char INTEGER_2ND_LENGTH_MASK = 0x60;
	static const unsigned char INTEGER_2ND_LENGTH_MEDIUM = 0x40;
	static const unsigned char INTEGER_2ND_LENGTH_LARGE = 0x60;

	static const unsigned char TERMINATOR_SINGLE = 0xF0;
	static const unsigned char TERMINATOR_DOUBLE = 0xFF;

	// Build in 1-10
	static const unsigned int ENCODING_ALGORITHM_BUILTIN_END = 9; 
	// 11-31 for future use
	// 32 start application algorithms
	static const unsigned int ENCODING_ALGORITHM_APPLICATION_START = 32;



};

} // end namespace FI

#endif
