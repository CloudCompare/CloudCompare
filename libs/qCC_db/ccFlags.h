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

#ifndef CC_FLAGS_HEADER
#define CC_FLAGS_HEADER

//System
#include <cstring>

//! Flags
class ccFlags
{
public:

	//! Sets all bits to 0
	void reset()
	{
		memset(table,0,sizeof(bool)*8);
	}

	//! Converts a byte to this structure
	void fromByte(unsigned char byte)
	{
		unsigned char i,mask = 1;
		for (i=0;i<8;++i)
		{
			table[i] = ((byte & mask) == mask);
			mask <<= 1;
		}
	}

	//! Converts this structure to a byte
	unsigned char toByte() const
	{
		unsigned char i,byte = 0,mask = 1;
		for (i=0;i<8;++i)
		{
			if (table[i])
				byte |= mask;
			mask <<= 1;
		}

		return byte;
	}

	//! Table of 8 booleans (one per bit)
	bool table[8];
};

#endif //CC_FLAGS_HEADER
