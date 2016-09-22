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

/************************************************************************/
/* Quantize a normal => 2D problem.                                     */
/* input :																*/
/*	n : a vector (normalized or not)[xyz]								*/
/*	level : the level of the quantization result is 3+2*level bits		*/
/* output :																*/
/*	res : the result - least significant bits are filled !!!			*/
/************************************************************************/
unsigned ccNormalCompressor::Compress(const PointCoordinateType n[3], unsigned char level/*=QUANTIZE_LEVEL*/)
{
	if (level == 0)
		return 0;

	/// compute in which sector lie the elements
	unsigned res = 0;
	PointCoordinateType x,y,z;
	if (n[0] >= 0) { x = n[0]; } else { res |= 4; x = -n[0]; }
	if (n[1] >= 0) { y = n[1]; } else { res |= 2; y = -n[1]; }
	if (n[2] >= 0) { z = n[2]; } else { res |= 1; z = -n[2]; }

	/// scale the sectored vector - early return for null vector
	PointCoordinateType psnorm = x + y + z;
	if (psnorm == 0)
	{
		res <<= (level*2);
		return res;
	}
	psnorm = 1 / psnorm;
	x *= psnorm; y *= psnorm; z *= psnorm;

	/// compute the box
	PointCoordinateType box[6] = { 0, 0, 0, 1, 1, 1 };
	/// then for each required level, quantize...
	bool flip = false;
	while (level > 0)
	{
		//next level
		res <<= 2;
		--level;
		PointCoordinateType halfBox[3] = {	(box[0] + box[3])/2,
											(box[1] + box[4])/2,
											(box[2] + box[5])/2 };

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
		/// do not do the last operation ...
		if (level == 0)
			return res;
		// fd : a little waste for less branching and smaller
		// code.... which one will be fastest ???
		if (flip)
		{
			if (sector != 3)
				psnorm = box[sector];
			box[0] = halfBox[0];
			box[1] = halfBox[1];
			box[2] = halfBox[2];
			if (sector != 3)
			{
				box[3+sector] = box[sector];
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
				psnorm = box[3+sector];
			box[3] = halfBox[0];
			box[4] = halfBox[1];
			box[5] = halfBox[2];
			if (sector != 3)
			{
				box[sector] = box[3+sector];
				box[3+sector] = psnorm;
			}
			else
			{
				flip = true;
			}
		}
	}

	return 0;
}

/************************************************************************/
/* DeQuantize a normal => 2D problem.                                   */
/* input :                                                              */
/*		index : quantized normal                                        */
/*		level : the level of the quantized normal has 3+2*level bits	*/
/* output :																*/
/*		res : the result : a NON-normalized normal is returned			*/
/************************************************************************/
void ccNormalCompressor::Decompress(unsigned index, PointCoordinateType n[3], unsigned char level/*=QUANTIZE_LEVEL*/)
{
	/// special case for level = 0
	if (level == 0)
	{
		n[0] = ((index & 4) != 0 ? -PC_ONE : PC_ONE);
		n[1] = ((index & 2) != 0 ? -PC_ONE : PC_ONE);
		n[2] = ((index & 1) != 0 ? -PC_ONE : PC_ONE);
		return;
	}

	bool flip = false;

	/// recompute the box in the sector...
	PointCoordinateType box[6] = { 0, 0, 0, 1, 1, 1 };

	unsigned char l_shift = level*2;
	for (unsigned char k=0; k<level; ++k)
	{
		l_shift -= 2;
		unsigned sector = ((index >> l_shift) & 3);
		if (flip)
		{
			PointCoordinateType tmp = box[sector];
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
			PointCoordinateType tmp = (sector != 3 ? box[3+sector] : 0);
			
			box[3] = (box[0] + box[3]) / 2;
			box[4] = (box[1] + box[4]) / 2;
			box[5] = (box[2] + box[5]) / 2;
			
			if (sector != 3)
			{
				box[sector] = box[3+sector];
				box[3+sector] = tmp;
			}
			else
			{
				flip = true;
			}
		}
	}

	//get the sector
	unsigned sector = index >> (level+level);

	n[0] = ((sector & 4) != 0 ? -(box[3] + box[0]) : box[3] + box[0]);
	n[1] = ((sector & 2) != 0 ? -(box[4] + box[1]) : box[4] + box[1]);
	n[2] = ((sector & 1) != 0 ? -(box[5] + box[2]) : box[5] + box[2]);
}
