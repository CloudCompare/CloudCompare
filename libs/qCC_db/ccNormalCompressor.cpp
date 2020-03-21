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

#include "ccNormalCompressor.h"

//CCLib
#include <CCConst.h>

//System
#include <assert.h>

void ccNormalCompressor::InvertNormal(CompressedNormType &code)
{
	if (code != NULL_NORM_CODE)
	{
		//see 'Decompress' for a better understanding
		code ^= (static_cast<CompressedNormType>(7) << 2 * QUANTIZE_LEVEL);
	}
}

unsigned ccNormalCompressor::Compress(const PointCoordinateType n[3])
{
	assert(QUANTIZE_LEVEL != 0);

	/// compute in which sector lie the elements
	unsigned res = 0;
	PointCoordinateType x;
	PointCoordinateType y;
	PointCoordinateType z;
	if (n[0] >= 0) { x = n[0]; } else { res |= 4; x = -n[0]; }
	if (n[1] >= 0) { y = n[1]; } else { res |= 2; y = -n[1]; }
	if (n[2] >= 0) { z = n[2]; } else { res |= 1; z = -n[2]; }

	/// scale the sectored vector - early return for null vector
	PointCoordinateType psnorm = x + y + z;
	if (psnorm == 0)
	{
		return NULL_NORM_CODE;
	}
	x /= psnorm; y /= psnorm; z /= psnorm;

	/// compute the box
	PointCoordinateType box[6] = { 0, 0, 0, 1, 1, 1 };
	/// then for each required level, quantize...
	bool flip = false;
	for (unsigned char level = QUANTIZE_LEVEL; level != 0; )
	{
		//next level
		res <<= 2;
		--level;

		PointCoordinateType halfBox[3] = {	(box[0] + box[3]) / 2,
											(box[1] + box[4]) / 2,
											(box[2] + box[5]) / 2 };

		unsigned sector = 3;
		if (flip)
		{
			     if (z < halfBox[2]) sector = 2;
			else if (y < halfBox[1]) sector = 1;
			else if (x < halfBox[0]) sector = 0;
		}
		else
		{
			     if (z > halfBox[2]) sector = 2;
			else if (y > halfBox[1]) sector = 1;
			else if (x > halfBox[0]) sector = 0;
		}
		res |= sector;
		
		if (level != 0) //skip this at the last level
		{
			if (flip)
			{
				if (sector != 3)
					psnorm = box[sector];
				box[0] = halfBox[0];
				box[1] = halfBox[1];
				box[2] = halfBox[2];
				if (sector != 3)
				{
					box[3 + sector] = box[sector];
					box[sector] = psnorm;
				}
				else
				{
					flip = false;
				}
			}
			else
			{
				if (sector != 3)
					psnorm = box[3 + sector];
				box[3] = halfBox[0];
				box[4] = halfBox[1];
				box[5] = halfBox[2];
				if (sector != 3)
				{
					box[sector] = box[3 + sector];
					box[3 + sector] = psnorm;
				}
				else
				{
					flip = true;
				}
			}
		}
	}

	return res;
}

void ccNormalCompressor::Decompress(unsigned index, PointCoordinateType n[3], unsigned char level/*=QUANTIZE_LEVEL*/)
{
	assert(level != 0);

	/// special case for the null code
	if (index == NULL_NORM_CODE)
	{
		n[0] = n[1] = n[2] = 0;
		return;
	}

	/// recompute the box in the sector...
	PointCoordinateType box[6] = { 0, 0, 0, 1, 1, 1 };
	bool flip = false;

	unsigned char l_shift = level * 2;
	for (unsigned char k = 0; k < level; ++k)
	{
		l_shift -= 2;
		const unsigned sector = ((index >> l_shift) & 3);
		if (flip)
		{
			const PointCoordinateType tmp = box[sector];
			box[0] = (box[0] + box[3]) / 2;
			box[1] = (box[1] + box[4]) / 2;
			box[2] = (box[2] + box[5]) / 2;
			if (sector != 3)
			{
				box[3+sector] = box[sector];
				box[sector] = tmp;
			}
			else
			{
				flip = false;
			}
		}
		else
		{
			const PointCoordinateType tmp = (sector != 3 ? box[3 + sector] : 0);
			
			box[3] = (box[0] + box[3]) / 2;
			box[4] = (box[1] + box[4]) / 2;
			box[5] = (box[2] + box[5]) / 2;
			
			if (sector != 3)
			{
				box[sector] = box[3 + sector];
				box[3 + sector] = tmp;
			}
			else
			{
				flip = true;
			}
		}
	}

	//get the sector
	const unsigned sector = index >> (level + level);

	n[0] = ((sector & 4) != 0 ? -(box[3] + box[0]) : box[3] + box[0]);
	n[1] = ((sector & 2) != 0 ? -(box[4] + box[1]) : box[4] + box[1]);
	n[2] = ((sector & 1) != 0 ? -(box[5] + box[2]) : box[5] + box[2]);
}
