//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ChamferDistanceTransform.h"

//system
#include <algorithm>
#include <cassert>
#include <cstring>

using namespace CCLib;

//! Forward mask shifts and weights (Chamfer 3-4-5)
const signed char ForwardNeighbours345[14*4] = {
	-1,-1,-1, 5,
	 0,-1,-1, 4,
	 1,-1,-1, 5,
	-1, 0,-1, 4,
	 0, 0,-1, 3,
	 1, 0,-1, 4,
	-1, 1,-1, 5,
	 0, 1,-1, 4,
	 1, 1,-1, 5,
	-1,-1, 0, 4,
	 0,-1, 0, 3,
	 1,-1, 0, 4,
	-1, 0, 0, 3,
	 0, 0, 0, 0
};

//! Backward mask shifts and weights (Chamfer 3-4-5)
const signed char BackwardNeighbours345[14*4] = {
	 0, 0, 0, 0,
	 1, 0, 0, 3,
	-1, 1, 0, 4,
	 0, 1, 0, 3,
	 1, 1, 0, 4,
	-1,-1, 1, 5,
	 0,-1, 1, 4,
	 1,-1, 1, 5,
	-1, 0, 1, 4,
	 0, 0, 1, 3,
	 1, 0, 1, 4,
	-1, 1, 1, 5,
	 0, 1, 1, 4,
	 1, 1, 1, 5
};

//! Forward mask shifts and weights (Chamfer 1-1-1)
const signed char ForwardNeighbours111[14*4] = {
	-1,-1,-1, 1,
	 0,-1,-1, 1,
	 1,-1,-1, 1,
	-1, 0,-1, 1,
	 0, 0,-1, 1,
	 1, 0,-1, 1,
	-1, 1,-1, 1,
	 0, 1,-1, 1,
	 1, 1,-1, 1,
	-1,-1, 0, 1,
	 0,-1, 0, 1,
	 1,-1, 0, 1,
	-1, 0, 0, 1,
	 0, 0, 0, 0
};

//! Backward masks shifts and weights (Chamfer 1-1-1)
const signed char BackwardNeighbours111[14*4] = {
	 0, 0, 0, 0,
	 1, 0, 0, 1,
	-1, 1, 0, 1,
	 0, 1, 0, 1,
	 1, 1, 0, 1,
	-1,-1, 1, 1,
	 0,-1, 1, 1,
	 1,-1, 1, 1,
	-1, 0, 1, 1,
	 0, 0, 1, 1,
	 1, 0, 1, 1,
	-1, 1, 1, 1,
	 0, 1, 1, 1,
	 1, 1, 1, 1
};

//ChamferDistanceTransform::GridElement ChamferDistanceTransform::propagateDistance(	unsigned iStart,
//																					unsigned jStart,
//																					unsigned kStart,
//																					bool forward,
//																					const signed char neighbours[14][4],
//																					NormalizedProgress* normProgress/*=0*/)
//{
//	assert(!m_grid.empty());
//
//	GridElement* _grid = &(m_grid[pos2index(iStart, jStart, kStart)]);
//
//	//accelerating structure
//	int neighborShift[14];
//	{
//		for (unsigned char v=0; v<14; ++v)
//		{
//			neighborShift[v] =	static_cast<int>(neighbours[v][0]) +
//								static_cast<int>(neighbours[v][1]) * static_cast<int>(m_rowSize) +
//								static_cast<int>(neighbours[v][2]) * static_cast<int>(m_sliceSize);
//		}
//	}
//
//	GridElement maxDist = 0;
//
//	int order = forward ? 1 : -1;
//
//	for (unsigned k=0; k<m_innerSize.z; ++k)
//	{
//		for (unsigned j=0; j<m_innerSize.y; ++j)
//		{
//			for (unsigned i=0; i<m_innerSize.x; ++i)
//			{
//				GridElement minVal = _grid[neighborShift[0]] + static_cast<GridElement>(neighbours[0][3]);
//
//				for (unsigned char v=1; v<14; ++v)
//				{
//					GridElement neighborVal = _grid[neighborShift[v]] + static_cast<GridElement>(neighbours[v][3]);
//					minVal = std::min(minVal, neighborVal);
//				}
//
//				*_grid = minVal;
//
//				//we track the max distance
//				if (minVal > maxDist)
//				{
//					maxDist = minVal;
//				}
//
//				_grid += order;
//			}
//			_grid += order*2; //next line
//
//			if (normProgress && !normProgress->oneStep())
//			{
//				break;
//			}
//		}
//
//		_grid += (order*2) * static_cast<int>(m_rowSize); //next slice
//	}
//
//	return maxDist;
//}

int ChamferDistanceTransform::propagateDistance(CC_CHAMFER_DISTANCE_TYPE type, GenericProgressCallback* progressCb)
{
	if (m_grid.empty())
	{
		assert(false);
        return -1;
	}

	const signed char* fwNeighbours = nullptr;
	const signed char* bwNeighbours = nullptr;
	switch (type)
	{
	case CHAMFER_111:
		{
			fwNeighbours = ForwardNeighbours111;
			bwNeighbours = BackwardNeighbours111;
		}
		break;

	case CHAMFER_345:
		{
			fwNeighbours = ForwardNeighbours345;
			bwNeighbours = BackwardNeighbours345;
		}
		break;

	default:
		//unhandled type?!
		assert(false);
		return -1;
	}

	NormalizedProgress normProgress(progressCb,m_innerSize.y*m_innerSize.z*2);
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setMethodTitle("Chamfer distance");
			char buffer[256];
			sprintf(buffer, "Box: [%u x %u x %u]", m_innerSize.x, m_innerSize.y, m_innerSize.z);
			progressCb->setInfo(buffer);
		}
        progressCb->update(0);
		progressCb->start();
	}

	//1st pass: forward scan
	{
		GridElement* _grid = &(m_grid[pos2index(0, 0, 0)]);

		//accelerating structure
		int neighborShift[14];
		{
			for (unsigned char v=0; v<14; ++v)
			{
				const signed char* fwNeighbour = fwNeighbours + 4*v;
				neighborShift[v] =	static_cast<int>(fwNeighbour[0]) +
									static_cast<int>(fwNeighbour[1]) * static_cast<int>(m_rowSize) +
									static_cast<int>(fwNeighbour[2]) * static_cast<int>(m_sliceSize);
			}
		}

		for (unsigned k=0; k<m_innerSize.z; ++k)
		{
			for (unsigned j=0; j<m_innerSize.y; ++j)
			{
				for (unsigned i=0; i<m_innerSize.x; ++i)
				{
					GridElement minVal = _grid[neighborShift[0]] + static_cast<GridElement>(fwNeighbours[3]);

					for (unsigned char v=1; v<14; ++v)
					{
						const signed char* fwNeighbour = fwNeighbours + 4*v;
						GridElement neighborVal = _grid[neighborShift[v]] + static_cast<GridElement>(fwNeighbour[3]);
						minVal = std::min(minVal, neighborVal);
					}

					*_grid++ = minVal;
				}
				_grid += 2; //next line

				if (progressCb && !normProgress.oneStep())
				{
					break;
				}
			}

			_grid += 2*m_rowSize; //next slice
		}
	}

	//2nd pass: backward scan
	GridElement maxDist = 0;
	{
		//accelerating structure
		int neighborShift[14];
		{
			for (unsigned char v=0; v<14; ++v)
			{
				const signed char* bwNeighbour = bwNeighbours + 4*v;
				neighborShift[v] =	static_cast<int>(bwNeighbour[0]) +
									static_cast<int>(bwNeighbour[1]) * static_cast<int>(m_rowSize) +
									static_cast<int>(bwNeighbour[2]) * static_cast<int>(m_sliceSize);
			}
		}

		GridElement* _grid = &(m_grid[pos2index(static_cast<int>(m_innerSize.x) - 1, static_cast<int>(m_innerSize.y) - 1, static_cast<int>(m_innerSize.z) - 1)]);

		for (unsigned k = 0; k < m_innerSize.z; ++k)
		{
			for (unsigned j = 0; j < m_innerSize.y; ++j)
			{
				for (unsigned i = 0; i < m_innerSize.x; ++i)
				{
					GridElement minVal = _grid[neighborShift[0]] + static_cast<GridElement>(bwNeighbours[3]);

					for (unsigned char v = 1; v < 14; ++v)
					{
						const signed char* bwNeighbour = bwNeighbours + 4 * v;
						GridElement neighborVal = _grid[neighborShift[v]] + static_cast<GridElement>(bwNeighbour[3]);
						minVal = std::min(minVal, neighborVal);
					}

					*_grid-- = minVal;

					//we track the max distance
					if (minVal > maxDist)
					{
						maxDist = minVal;
					}
				}
				_grid -= 2; //next line

				if (progressCb && !normProgress.oneStep())
				{
					break;
				}
			}

			_grid -= 2 * m_rowSize; //next slice
		}
	}

	return static_cast<int>(maxDist);
}
