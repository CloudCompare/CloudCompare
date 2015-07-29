#include "../include/ChamferDistanceTransformSigned.h"

//Local
#include "GenericProgressCallback.h"
#include "GenericIndexedMesh.h"
#include "DistanceComputationTools.h"

//system
#include <algorithm>
#include <string.h>
#include <assert.h>
#include <stdio.h> //for sprintf

using namespace CCLib;

//! 3x3x3 neighborhood
const char Neighbours333[27][4] = {
	//back slice
	{-1,-1,-1, 2},
	{ 0,-1,-1, 1},
	{ 1,-1,-1, 2},
	{-1, 0,-1, 1},
	{ 0, 0,-1, 0},
	{ 1, 0,-1, 1},
	{-1, 1,-1, 2},
	{ 0, 1,-1, 1},
	{ 1, 1,-1, 2},

	//current slice (backward half)
	{-1,-1, 0, 1},
	{ 0,-1, 0, 0},
	{ 1,-1, 0, 1},
	{-1, 0, 0, 0},
	
	//center (13th value)
	{ 0, 0, 0,-1},
	
	//current slice (forward half)
	{ 1, 0, 0, 0},
	{-1, 1, 0, 1},
	{ 0, 1, 0, 0},
	{ 1, 1, 0, 1},

	//next slice
	{-1,-1, 1, 2},
	{ 0,-1, 1, 1},
	{ 1,-1, 1, 2},
	{-1, 0, 1, 1},
	{ 0, 0, 1, 0},
	{ 1, 0, 1, 1},
	{-1, 1, 1, 2},
	{ 0, 1, 1, 1},
	{ 1, 1, 1, 2}
};

//inspired from ITK's FastChamferDistanceImageFilter
bool ChamferDistanceTransformSigned::propagateDistance(GenericProgressCallback* progressCb/*=0*/)
{
	const float Weights[3] = { 0.92644f, 1.34065f, 1.65849f };

	NormalizedProgress normProgress(progressCb,m_innerSize.y*m_innerSize.z*2);
	if (progressCb)
	{
		progressCb->setMethodTitle("Signed Chamfer distance");
		char buffer[256];
		sprintf(buffer,"Box: [%u x %u x %u]",m_innerSize.x,m_innerSize.y,m_innerSize.z);
		progressCb->setInfo(buffer);
        progressCb->reset();
		progressCb->start();
	}

	/** 1st Scan , using neighbors from centerNeighborIndex+1 to neighborhoodSize-1 */
	{
		const size_t firstNeighborIndex = 27/2 + 1; //14
		const size_t lastNeighborIndex = 27 - 1; //26

		for (size_t k=0; k<m_innerSize.z; ++k)
		{
			for (size_t j=0; j<m_innerSize.y; ++j)
			{
				for (size_t i=0; i<m_innerSize.x; ++i)
				{
					GridElement center_value = getValue(static_cast<int>(i),
														static_cast<int>(j),
														static_cast<int>(k));

					/** Update Positive Distance */
					if ( center_value > -Weights[0] )
					{
						float val[3] = {	center_value + Weights[0],
											center_value + Weights[1],
											center_value + Weights[2] };

						for (size_t n=firstNeighborIndex; n<=lastNeighborIndex; ++n)
						{
							const char* pos = Neighbours333[n];
							Tuple3i neighborPos(static_cast<int>(i) + static_cast<int>(pos[0]),
												static_cast<int>(j) + static_cast<int>(pos[1]),
												static_cast<int>(k) + static_cast<int>(pos[2]) );
					
							assert(pos[3] >= 0 && pos[3] < 3);
							if (val[pos[3]] < getValue(neighborPos))
							{
								setValue(neighborPos, val[pos[3]]);
							}
						}
					}
					/** Update Negative Distance */
					else if ( center_value < Weights[0] )
					{
						float val[3] = {	center_value - Weights[0],
											center_value - Weights[1],
											center_value - Weights[2] };

						for (size_t n=firstNeighborIndex; n<=lastNeighborIndex; ++n)
						{
							const char* pos = Neighbours333[n];
							Tuple3i neighborPos(static_cast<int>(i) + static_cast<int>(pos[0]),
												static_cast<int>(j) + static_cast<int>(pos[1]),
												static_cast<int>(k) + static_cast<int>(pos[2]) );
						
							assert(pos[3] >= 0 && pos[3] < 3);
							if (val[pos[3]] > getValue(neighborPos))
							{
								setValue(neighborPos, val[pos[3]]);
							}
						}
					}
				}

				if (progressCb && !normProgress.oneStep())
				{
					//process cancelled by the user
					return false;
				}
			}
		}
	}

	/** 2nd Scan , using neighbors from 0 to centerNeighborIndex-1 */
	{
		const size_t firstNeighborIndex = 0;
		const size_t lastNeighborIndex = 27 / 2 - 1; //12

		for (size_t k=0; k<m_innerSize.z; ++k)
		{
			for (size_t j=0; j<m_innerSize.y; ++j)
			{
				for (size_t i=0; i<m_innerSize.x; ++i)
				{
					GridElement center_value = getValue(static_cast<int>(i),
														static_cast<int>(j),
														static_cast<int>(k));

					/** Update Positive Distance */
					if ( center_value > -Weights[0] )
					{
						float val[3] = {	center_value + Weights[0],
											center_value + Weights[1],
											center_value + Weights[2] };

						for (size_t n=firstNeighborIndex; n<=lastNeighborIndex; ++n)
						{
							const char* pos = Neighbours333[n];
							Tuple3i neighborPos(static_cast<int>(i) + static_cast<int>(pos[0]),
												static_cast<int>(j) + static_cast<int>(pos[1]),
												static_cast<int>(k) + static_cast<int>(pos[2]) );
						
							assert(pos[3] >= 0 && pos[3] < 3);
							if (val[pos[3]] < getValue(neighborPos))
							{
								setValue(neighborPos, val[pos[3]]);
							}
						}
					}
					/** Update Negative Distance */
					else if ( center_value < Weights[0] )
					{
						float val[3] = {	center_value - Weights[0],
											center_value - Weights[1],
											center_value - Weights[2] };

						for (size_t n=firstNeighborIndex; n<=lastNeighborIndex; ++n)
						{
							const char* pos = Neighbours333[n];
							Tuple3i neighborPos(static_cast<int>(i) + static_cast<int>(pos[0]),
												static_cast<int>(j) + static_cast<int>(pos[1]),
												static_cast<int>(k) + static_cast<int>(pos[2]) );
						
							assert(pos[3] >= 0 && pos[3] < 3);
							if (val[pos[3]] > getValue(neighborPos))
							{
								setValue(neighborPos, val[pos[3]]);
							}
						}
					}
				}

				if (progressCb && !normProgress.oneStep())
				{
					//process cancelled by the user
					return false;
				}
			}
		}
	}

	return true;
}

//! Intersects this grid with a mesh
bool ChamferDistanceTransformSigned::initDT(GenericIndexedMesh* mesh,
											PointCoordinateType cellLength,
											const CCVector3& gridMinCorner,
											GenericProgressCallback* progressCb/*=0*/)
{
	if (!mesh || !isInitialized())
	{
		assert(false);
		return false;
	}

	//cell dimension
	PointCoordinateType halfCellSize = cellLength / 2;

	std::vector<CellToTest> cellsToTest(1); //initial size must be > 0
	unsigned cellsToTestCount = 0;

	//number of triangles
	unsigned numberOfTriangles = mesh->size();

	//progress notification
	NormalizedProgress nProgress(progressCb, numberOfTriangles);
	if (progressCb)
	{
		char buffer[64];
		sprintf(buffer, "Triangles: %u", numberOfTriangles);
		progressCb->reset();
		progressCb->setInfo(buffer);
		progressCb->setMethodTitle("Init Distance Transform");
		progressCb->start();
	}

	//for each triangle: look for intersecting cells
	mesh->placeIteratorAtBegining();
	for (unsigned n = 0; n<numberOfTriangles; ++n)
	{
		//get the positions (in the grid) of each vertex 
		const GenericTriangle* T = mesh->_getNextTriangle();

		//current triangle vertices
		const CCVector3* triPoints[3] = { T->_getA(),
			T->_getB(),
			T->_getC() };

		CCVector3 AB = (*triPoints[1]) - (*triPoints[0]);
		CCVector3 BC = (*triPoints[2]) - (*triPoints[1]);
		CCVector3 CA = (*triPoints[0]) - (*triPoints[2]);

		//be sure that the triangle is not degenerate!!!
		if (AB.norm2() > ZERO_TOLERANCE &&
			BC.norm2() > ZERO_TOLERANCE &&
			CA.norm2() > ZERO_TOLERANCE)
		{
			Tuple3i cellPos[3];
			{
				for (int k = 0; k < 3; k++)
				{
					CCVector3 P = *(triPoints[k]) - gridMinCorner;
					cellPos[k].x = std::min(static_cast<int>(P.x / cellLength), static_cast<int>(size().x) - 1);
					cellPos[k].y = std::min(static_cast<int>(P.y / cellLength), static_cast<int>(size().y) - 1);
					cellPos[k].z = std::min(static_cast<int>(P.z / cellLength), static_cast<int>(size().z) - 1);
				}
			}

			//compute the triangle bounding-box
			Tuple3i minPos, maxPos;
			{
				for (int k = 0; k < 3; k++)
				{
					minPos.u[k] = std::min(cellPos[0].u[k], std::min(cellPos[1].u[k], cellPos[2].u[k]));
					maxPos.u[k] = std::max(cellPos[0].u[k], std::max(cellPos[1].u[k], cellPos[2].u[k]));
				}
			}

			//first cell
			assert(cellsToTest.capacity() != 0);
			cellsToTestCount = 1;
			CellToTest* _currentCell = &cellsToTest[0/*cellsToTestCount-1*/];

			_currentCell->pos = minPos;

			//max distance (in terms of cell) between the vertices
			int maxSize = 0;
			{
				Tuple3i delta = maxPos - minPos + Tuple3i(1, 1, 1);
				maxSize = std::max(delta.x, delta.y);
				maxSize = std::max(maxSize, delta.z);
			}

			//test each cell
			for (int k = minPos.z; k <= maxPos.z; ++k)
			{
				CCVector3 C(0, 0, gridMinCorner.z + k * cellLength + halfCellSize);
				for (int j = minPos.y; j <= maxPos.y; ++j)
				{
					C.y = gridMinCorner.y + j * cellLength + halfCellSize;
					for (int i = minPos.x; i <= maxPos.x; ++i)
					{
						//compute the (absolute) cell center
						C.x = gridMinCorner.x + i * cellLength + halfCellSize;

						ScalarType dist = DistanceComputationTools::computePoint2TriangleDistance(&C, T, true);
						GridElement& cellValue = getValue(i, j, k);
						if (fabs(cellValue) > fabs(dist))
						{
							cellValue = dist;
						}
					}
				}
			}
		}

		if (progressCb && !nProgress.oneStep())
		{
			//cancel by user
			return false;
		}
	}

	return true;
}
