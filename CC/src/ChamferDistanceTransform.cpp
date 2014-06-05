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
#include <stdio.h>

using namespace CCLib;

//! Point coordinates to index conversion macro
#define DG_pos2index(i,j,k) unsigned(i)+unsigned(j)*m_decY+unsigned(k)*m_decZ+m_decIndex

//! Forward mask shifts and weights (Chamfer 3-4-5)
const int forwardNeighbours345[14][4] = {
	{-1,-1,-1,5},
	{ 0,-1,-1,4},
	{ 1,-1,-1,5},
	{-1, 0,-1,4},
	{ 0, 0,-1,3},
	{ 1, 0,-1,4},
	{-1, 1,-1,5},
	{ 0, 1,-1,4},
	{ 1, 1,-1,5},
	{-1,-1, 0,4},
	{ 0,-1, 0,3},
	{ 1,-1, 0,4},
	{-1, 0, 0,3},
	{ 0, 0, 0,0}
};

//! Backward mask shifts and weights (Chamfer 3-4-5)
const int backwardNeighbours345[14][4] = {
	{ 0, 0, 0,0},
	{ 1, 0, 0,3},
	{-1, 1, 0,4},
	{ 0, 1, 0,3},
	{ 1, 1, 0,4},
	{-1,-1,1,5},
	{ 0,-1,1,4},
	{ 1,-1,1,5},
	{-1, 0,1,4},
	{ 0, 0,1,3},
	{ 1, 0,1,4},
	{-1, 1,1,5},
	{ 0, 1,1,4},
	{ 1, 1,1,5}
};

//! Forward mask shifts and weights (Chamfer 1-1-1)
const int forwardNeighbours111[14][4] = {
	{-1,-1,-1,1},
	{ 0,-1,-1,1},
	{ 1,-1,-1,1},
	{-1, 0,-1,1},
	{ 0, 0,-1,1},
	{ 1, 0,-1,1},
	{-1, 1,-1,1},
	{ 0, 1,-1,1},
	{ 1, 1,-1,1},
	{-1,-1, 0,1},
	{ 0,-1, 0,1},
	{ 1,-1, 0,1},
	{-1, 0, 0,1},
	{ 0, 0, 0,0}
};

//! Backward masks shifts and weights (Chamfer 1-1-1)
const int backwardNeighbours111[14][4] = {
	{ 0, 0, 0,0},
	{ 1, 0, 0,1},
	{-1, 1, 0,1},
	{ 0, 1, 0,1},
	{ 1, 1, 0,1},
	{-1,-1,1,1},
	{ 0,-1,1,1},
	{ 1,-1,1,1},
	{-1, 0,1,1},
	{ 0, 0,1,1},
	{ 1, 0,1,1},
	{-1, 1,1,1},
	{ 0, 1,1,1},
	{ 1, 1,1,1}
};

ChamferDistanceTransform::ChamferDistanceTransform(unsigned Di, unsigned Dj, unsigned Dk)
	: m_grid(0)
	, m_gridX(Di)
	, m_gridY(Dj)
	, m_gridZ(Dk)
	, m_decY((int)(m_gridX+2))
	, m_decZ(m_decY*(int)(m_gridY+2))
	, m_decIndex(1+m_decY+m_decZ)

{
}

ChamferDistanceTransform::~ChamferDistanceTransform()
{
	if (m_grid)
        delete[] m_grid;
}

void ChamferDistanceTransform::setZero(int i, int j, int k)
{
	assert(m_grid);
	m_grid[DG_pos2index(i,j,k)] = 0;
}

void ChamferDistanceTransform::setZero(int cellPos[])
{
	assert(m_grid);
	m_grid[DG_pos2index(cellPos[0],cellPos[1],cellPos[2])] = 0;
}

ChamferDistanceTransform::GridElement ChamferDistanceTransform::getValue(int i, int j, int k) const
{
	assert(m_grid);
	return m_grid[DG_pos2index(i,j,k)];
}

ChamferDistanceTransform::GridElement ChamferDistanceTransform::getValue(int cellPos[])
{
	assert(m_grid);
	return m_grid[DG_pos2index(cellPos[0],cellPos[1],cellPos[2])];
}

ChamferDistanceTransform::GridElement ChamferDistanceTransform::propagateDistance(GridElement iStart,
												 GridElement jStart,
												 GridElement kStart,
												 int sign,
												 const int neighbours[14][4],
												 NormalizedProgress* normProgress/*=0*/)
{
	assert(m_grid);

	GridElement* _grid = m_grid+DG_pos2index(iStart,jStart,kStart); //on commence avec une tranche de decalage

	//accelerating structure
	int voisDec[14];
	for (uchar v=0;v<14;++v)
	{
		voisDec[v] = neighbours[v][0]+
						neighbours[v][1]*m_decY+
							neighbours[v][2]*m_decZ;
	}

	GridElement maxDist = 0;

	for (GridElement k=0;k<m_gridZ;++k)
	{
		for (GridElement j=0;j<m_gridY;++j)
		{
			for (GridElement i=0;i<m_gridX;++i)
			{
				GridElement minVal = _grid[voisDec[0]]+(GridElement)neighbours[0][3];

				for (uchar v=1;v<14;++v)
					minVal = std::min<GridElement>(minVal,_grid[voisDec[v]]+(GridElement)neighbours[v][3]);

				*_grid = minVal;

				//we track the max distance
				if (minVal > maxDist)
					maxDist = minVal;

				_grid += sign;
			}
			_grid += sign*2; //next line

			if (normProgress && !normProgress->oneStep())
				break;
		}

		_grid += sign*2*m_decY; //next slice
	}

	return maxDist;
}

bool ChamferDistanceTransform::init()
{
	int gridSize = m_decZ*(int)(m_gridZ+2);

	//grid initialization
	m_grid = new GridElement[gridSize];
	if (!m_grid)
        return false;

	//we fill the grid with non zero values
	//Default value is 250 (0xFA) --> i.e. 0xFAFA for short values (< 0xFFFF to avoid overflow later)
	memset(m_grid,250,sizeof(GridElement)*gridSize);

	return true;
}

int ChamferDistanceTransform::propagateDistance(CC_CHAMFER_DISTANCE_TYPE type, GenericProgressCallback* progressCb)
{
	if (!m_grid)
        return -1;

	NormalizedProgress* normProgress=0;
    if(progressCb)
    {
		normProgress = new NormalizedProgress(progressCb,m_gridY*m_gridZ);
		progressCb->setMethodTitle("Chamfer distance");
		char buffer[256];
		sprintf(buffer,"Box: [%i*%i*%i]",m_gridX,m_gridY,m_gridZ);
		progressCb->setInfo(buffer);
        progressCb->reset();
		progressCb->start();
	}

	GridElement maxDist = 0;

	switch(type)
	{
	case CHAMFER_111:
		propagateDistance(0,0,0,1,forwardNeighbours111,normProgress);
		maxDist = propagateDistance(m_gridX-1,m_gridY-1,m_gridZ-1,-1,backwardNeighbours111,normProgress);
		break;
	case CHAMFER_345:
		propagateDistance(0,0,0,1,forwardNeighbours345,normProgress);
		maxDist = propagateDistance(m_gridX-1,m_gridY-1,m_gridZ-1,-1,backwardNeighbours345,normProgress);
		break;
	}

	if (normProgress)
		delete normProgress;

	return (int)maxDist;
}
