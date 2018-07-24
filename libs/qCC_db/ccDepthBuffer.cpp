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

#include "ccDepthBuffer.h"

//algorithm
#include <vector>
#include <string.h>

ccDepthBuffer::ccDepthBuffer()
	: deltaPhi(0)
	, deltaTheta(0)
	, width(0)
	, height(0)
{
}

void ccDepthBuffer::clear()
{
	zBuff.resize(0);
	width = height = 0;
	deltaPhi = deltaTheta = 0;
}

ccDepthBuffer::~ccDepthBuffer()
{
	clear();
}

int ccDepthBuffer::fillHoles()
{
	if (zBuff.empty())
	{
		//z-buffer not initialized!
		return -1;
	}

	//new temp buffer
	int dx = width + 2;
	int dy = height + 2;
	unsigned tempZBuffSize = dx*dy;
	std::vector<PointCoordinateType> zBuffTemp;
	try
	{
		zBuffTemp.resize(tempZBuffSize);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return -2;
	}

	//copy old zBuffer in temp one (with 1 pixel border)
	{
		PointCoordinateType *_zBuffTemp = zBuffTemp.data() + (dx + 1); //2nd line, 2nd column
		const PointCoordinateType *_zBuff = zBuff.data(); //first line, first column of the true buffer
		for (unsigned y = 0; y < height; ++y)
		{
			memcpy(_zBuffTemp, _zBuff, width * sizeof(PointCoordinateType));
			_zBuffTemp += dx;
			_zBuff += width;
		}
	}

	//fill holes with their neighbor's mean value
	{
		for (unsigned y = 0; y < height; ++y)
		{
			const PointCoordinateType* zu = zBuffTemp.data() + y*dx;
			const PointCoordinateType* z = zu + dx;
			const PointCoordinateType* zd = z + dx;
			for (unsigned x = 0; x < width; ++x, ++zu, ++z, ++zd)
			{
				if (z[1] == 0) //hole
				{
					unsigned char nsup = 0; //non empty holes
					//upper line
					nsup += (zu[0] > 0);
					nsup += (zu[1] > 0);
					nsup += (zu[2] > 0);
					//current line
					nsup += ( z[0] > 0);
					nsup += ( z[2] > 0);
					//next line
					nsup += (zd[0] > 0);
					nsup += (zd[1] > 0);
					nsup += (zd[2] > 0);

					if (nsup > 3)
					{
						zBuff[x + y*width] = (zu[0] + zu[1] + zu[2] + z[0] + z[2] + zd[0] + zd[1] + zd[2]) / nsup;
					}
				}
			}
		}
	}

	return 0;
}
