//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ChamferDistanceTransform.h"

//local
#include "GenericProgressCallback.h"

//system
#include <algorithm>
#include <string.h>
#include <assert.h>
#include <stdio.h> //for sprintf

using namespace CCLib;

//! Forward mask shifts and weights (Chamfer 3-4-5)
const int ForwardNeighbours345[14][4] = {
	{-1,-1,-1, 5},
	{ 0,-1,-1, 4},
	{ 1,-1,-1, 5},
	{-1, 0,-1, 4},
	{ 0, 0,-1, 3},
	{ 1, 0,-1, 4},
	{-1, 1,-1, 5},
	{ 0, 1,-1, 4},
	{ 1, 1,-1, 5},
	{-1,-1, 0, 4},
	{ 0,-1, 0, 3},
	{ 1,-1, 0, 4},
	{-1, 0, 0, 3},
	{ 0, 0, 0, 0}
};

//! Backward mask shifts and weights (Chamfer 3-4-5)
const int BackwardNeighbours345[14][4] = {
	{ 0, 0, 0, 0},
	{ 1, 0, 0, 3},
	{-1, 1, 0, 4},
	{ 0, 1, 0, 3},
	{ 1, 1, 0, 4},
	{-1,-1, 1, 5},
	{ 0,-1, 1, 4},
	{ 1,-1, 1, 5},
	{-1, 0, 1, 4},
	{ 0, 0, 1, 3},
	{ 1, 0, 1, 4},
	{-1, 1, 1, 5},
	{ 0, 1, 1, 4},
	{ 1, 1, 1, 5}
};

//! Forward mask shifts and weights (Chamfer 1-1-1)
const int ForwardNeighbours111[14][4] = {
	{-1,-1,-1, 1},
	{ 0,-1,-1, 1},
	{ 1,-1,-1, 1},
	{-1, 0,-1, 1},
	{ 0, 0,-1, 1},
	{ 1, 0,-1, 1},
	{-1, 1,-1, 1},
	{ 0, 1,-1, 1},
	{ 1, 1,-1, 1},
	{-1,-1, 0, 1},
	{ 0,-1, 0, 1},
	{ 1,-1, 0, 1},
	{-1, 0, 0, 1},
	{ 0, 0, 0, 0}
};

//! Backward masks shifts and weights (Chamfer 1-1-1)
const int BackwardNeighbours111[14][4] = {
	{ 0, 0, 0, 0},
	{ 1, 0, 0, 1},
	{-1, 1, 0, 1},
	{ 0, 1, 0, 1},
	{ 1, 1, 0, 1},
	{-1,-1, 1, 1},
	{ 0,-1, 1, 1},
	{ 1,-1, 1, 1},
	{-1, 0, 1, 1},
	{ 0, 0, 1, 1},
	{ 1, 0, 1, 1},
	{-1, 1, 1, 1},
	{ 0, 1, 1, 1},
	{ 1, 1, 1, 1}
};

ChamferDistanceTransform::GridElement ChamferDistanceTransform::propagateDistance(	unsigned iStart,
																					unsigned jStart,
																					unsigned kStart,
																					bool forward,
																					const int neighbours[14][4],
																					NormalizedProgress* normProgress/*=0*/)
{
	assert(!m_grid.empty());

	GridElement* _grid = &(m_grid[pos2index(iStart,jStart,kStart)]);

	//accelerating structure
	int neighborShift[14];
	{
		for (unsigned char v=0; v<14; ++v)
		{
			neighborShift[v] =	neighbours[v][0]               +
								neighbours[v][1] * m_rowSize   +
								neighbours[v][2] * m_sliceSize ;
		}
	}

	GridElement maxDist = 0;

	int order = forward ? 1 : -1;

	for (unsigned k=0; k<m_gridZ; ++k)
	{
		for (unsigned j=0; j<m_gridY; ++j)
		{
			for (unsigned i=0; i<m_gridX; ++i)
			{
				GridElement minVal = _grid[neighborShift[0]] + static_cast<GridElement>(neighbours[0][3]);

				for (unsigned char v=1; v<14; ++v)
				{
					minVal = std::min<GridElement>(minVal,_grid[neighborShift[v]] + static_cast<GridElement>(neighbours[v][3]));
				}

				*_grid = minVal;

				//we track the max distance
				if (minVal > maxDist)
				{
					maxDist = minVal;
				}

				_grid += order;
			}
			_grid += order*2; //next line

			if (normProgress && !normProgress->oneStep())
			{
				break;
			}
		}

		_grid += (order*2) * m_rowSize; //next slice
	}

	return maxDist;
}

int ChamferDistanceTransform::propagateDistance(CC_CHAMFER_DISTANCE_TYPE type, GenericProgressCallback* progressCb)
{
	if (m_grid.empty())
	{
		assert(false);
        return -1;
	}

	NormalizedProgress normProgress(progressCb,m_gridY*m_gridZ);
	if (progressCb)
	{
		progressCb->setMethodTitle("Chamfer distance");
		char buffer[256];
		sprintf(buffer,"Box: [%u x %u x %u]",m_gridX,m_gridY,m_gridZ);
		progressCb->setInfo(buffer);
        progressCb->reset();
		progressCb->start();
	}

	GridElement maxDist = 0;

	switch (type)
	{
	case CHAMFER_111:
		{
					  propagateDistance(0,        0,        0,        true, ForwardNeighbours111, progressCb ? &normProgress : 0);
			maxDist = propagateDistance(m_gridX-1,m_gridY-1,m_gridZ-1,false,BackwardNeighbours111,progressCb ? &normProgress : 0);
		}
		break;
	case CHAMFER_345:
		{
					  propagateDistance(0,        0,        0,        true, ForwardNeighbours345, progressCb ? &normProgress : 0);
			maxDist = propagateDistance(m_gridX-1,m_gridY-1,m_gridZ-1,false,BackwardNeighbours345,progressCb ? &normProgress : 0);
		}
		break;
	}

	return static_cast<int>(maxDist);
}
