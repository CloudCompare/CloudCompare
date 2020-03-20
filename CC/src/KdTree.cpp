//##########################################################################
//#																		   #
//#								  CCLIB									   #
//#																		   #
//#	 This program is free software; you can redistribute it and/or modify  #
//#	 it under the terms of the GNU Library General Public License as	   #
//#	 published by the Free Software Foundation; version 2 or later of the  #
//#	 License.															   #
//#																		   #
//#	 This program is distributed in the hope that it will be useful,	   #
//#	 but WITHOUT ANY WARRANTY; without even the implied warranty of		   #
//#	 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the		   #
//#	 GNU General Public License for more details.						   #
//#																		   #
//#			 COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)			   #
//#																		   #
//##########################################################################

#include "KdTree.h"

#include "GenericIndexedCloud.h"
#include "GenericProgressCallback.h"

//system
#include <algorithm>

using namespace CCLib;

KDTree::KDTree()
	: m_root(nullptr)
	, m_associatedCloud(nullptr)
	, m_cellCount(0)
{
}

KDTree::~KDTree()
{
	deleteSubTree(m_root);
}

bool KDTree::buildFromCloud(GenericIndexedCloud *cloud, GenericProgressCallback *progressCb)
{
	unsigned cloudsize = cloud->size();

	m_indexes.resize(0);
	m_cellCount = 0;
	m_associatedCloud = nullptr;
	m_root = nullptr;

	if (cloudsize == 0)
		return false;

	try
	{
		m_indexes.resize(cloudsize);
	}
	catch (.../*const std::bad_alloc&*/) //out of memory
	{
		return false;
	}

	m_associatedCloud = cloud;

	for (unsigned i=0; i<cloudsize; i++)
		m_indexes[i] = i;

	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setInfo("Building KD-tree");
		}
		progressCb->update(0);
		progressCb->start();
	}

	m_root = buildSubTree(0, cloudsize-1, nullptr, m_cellCount, progressCb);

	if (progressCb)
		progressCb->stop();

	//if the tree building has failed (memory issues)
	if (!m_root)
	{
		m_associatedCloud = nullptr;
		m_cellCount = 0;
		return false;
	}

	try
	{
		m_indexes.resize(cloudsize);
	}
	catch (.../*const std::bad_alloc&*/) //out of memory
	{
		m_associatedCloud = nullptr;
		m_cellCount = 0;
		return false;
	}

	return true;
}


void KDTree::deleteSubTree(KdCell *cell)
{
	if (!cell)
		return;

	deleteSubTree(cell->leSon);
	deleteSubTree(cell->gSon);
	delete cell;
	assert(m_cellCount>0);
	m_cellCount--;
}

/*** Comparison functions used by the sort function (Strict ordering must be used) ***/
static CCLib::GenericIndexedCloud* s_comparisonCloud = nullptr;

//! Compares X coordinates of two points designated by their index
static bool ComparisonX(const unsigned &a, const unsigned &b)
{
	return (s_comparisonCloud->getPoint(a)->x < s_comparisonCloud->getPoint(b)->x);
}

//! Compares Y coordinates of two points designated by their index
static bool ComparisonY(const unsigned &a, const unsigned &b)
{
	return (s_comparisonCloud->getPoint(a)->y < s_comparisonCloud->getPoint(b)->y);
}

//! Compares Z coordinates of two points designated by their index
static bool ComparisonZ(const unsigned &a, const unsigned &b)
{
	return (s_comparisonCloud->getPoint(a)->z < s_comparisonCloud->getPoint(b)->z);
}

KDTree::KdCell* KDTree::buildSubTree(unsigned first, unsigned last, KdCell* father, unsigned &nbBuildCell, GenericProgressCallback *progressCb)
{
	KdCell* cell = new KdCell;
	if (!cell)
		return nullptr;
	m_cellCount++;

	unsigned dim = (father == nullptr ? 0 : ((father->cuttingDim+1) % 3));

	//Compute outside bounding box (have to be done before building the current cell sons)
	cell->father = father;
	cell->startingPointIndex = first;
	cell->nbPoints = last-first+1;
	cell->cuttingDim = dim;
	updateOutsideBoundingBox(cell);
	if (progressCb)
		progressCb->update(m_cellCount*100.0f/(m_indexes.size()*2.0f-1.0f));

	//If there is only one point to insert, build a leaf
	if (first == last)
	{
		cell->cuttingDim = 0;
		cell->leSon = nullptr;
		cell->gSon = nullptr;
	}
	else
	{
		//sort the remaining points considering dimension dim
		s_comparisonCloud = m_associatedCloud; //TODO: not compatible with parallelism!!!
		if (dim == 0)
			sort(m_indexes.begin()+first, m_indexes.begin()+(last+1), ComparisonX);
		else if (dim == 1)
			sort(m_indexes.begin()+first, m_indexes.begin()+(last+1), ComparisonY);
		else if (dim == 2)
			sort(m_indexes.begin()+first, m_indexes.begin()+(last+1), ComparisonZ);
		//find the median point in the sorted tab
		unsigned split = (first+last)/2;
		const CCVector3* P = m_associatedCloud->getPoint(m_indexes[split]);
		cell->cuttingCoordinate = P->u[dim];
		//recursively build the other two sub trees
		//trap the memory issues. At this point, none of the cell sons can be set to 0. Otherwise there has been memory allocation failure.
		cell->leSon = cell->gSon = nullptr;
		cell->leSon = buildSubTree(first, split, cell, nbBuildCell, progressCb);
		if (cell->leSon == nullptr)
		{
			deleteSubTree(cell);
			//the tree beyond the current cell will be deleted when noticing that this cell is set to 0
			return nullptr;
		}
		cell->gSon = buildSubTree(split+1, last, cell, nbBuildCell, progressCb);
		if (cell->gSon == nullptr)
		{
			deleteSubTree(cell);
			//the tree beyond the current cell will be deleted when noticing that this cell is set to 0
			return nullptr;
		}

	}
	//Compute inside bounding box (have to be done once sons have been built)
	updateInsideBoundingBox(cell);

	return cell;
}


bool KDTree::findNearestNeighbour(	const PointCoordinateType *queryPoint,
									unsigned &nearestPointIndex,
									ScalarType maxDist)
{
	if (m_root == nullptr)
		return false;

	maxDist *= maxDist;

	//Go down the tree to find which cell contains the query point (at most log2(N) tests where N is the total number of points in the cloud)
	KdCell* cellPtr = m_root;
	while (cellPtr->leSon != nullptr || cellPtr->gSon != nullptr)
	{
		if (queryPoint[cellPtr->cuttingDim] <= cellPtr->cuttingCoordinate)
			cellPtr = cellPtr->leSon;
		else
			cellPtr = cellPtr->gSon;
	}

	//Once we found the cell containing the query point, the nearest neighbour has great chances to lie in this cell
	bool found = false;
	for (unsigned i=0; i<cellPtr->nbPoints; i++)
	{
		const CCVector3 *p = m_associatedCloud->getPoint(m_indexes[cellPtr->startingPointIndex+i]);
		PointCoordinateType sqrdist = CCVector3::vdistance2(p->u, queryPoint);
		if (sqrdist < maxDist)
		{
			maxDist = static_cast<ScalarType>(sqrdist);
			nearestPointIndex = m_indexes[cellPtr->startingPointIndex+i];
			found = true;
		}
	}

	//Go up in the tree to check that neighbours cells do not contain a nearer point than the one we found
	while (cellPtr != nullptr)
	{
		KdCell* prevPtr = cellPtr;
		cellPtr = cellPtr->father;
		if (cellPtr != nullptr)
		{
			ScalarType sqrdist = InsidePointToCellDistance(queryPoint, cellPtr);
			if (sqrdist >= 0 && sqrdist*sqrdist < maxDist)
			{
				KdCell* brotherPtr = (cellPtr->leSon == prevPtr ? cellPtr->gSon : cellPtr->leSon);
				int a = checkNearerPointInSubTree(queryPoint, maxDist, brotherPtr);
				if (a >= 0)
				{
					nearestPointIndex = a;
					found = true;
				}
			}
			else
			{
				cellPtr = nullptr;
			}
		}
	}

	return found;
}

bool KDTree::findPointBelowDistance(const PointCoordinateType *queryPoint,
									ScalarType maxDist)
{
	if (m_root == nullptr)
		return false;

	maxDist *= maxDist;

	KdCell* cellPtr = m_root;
	//Go down the tree to find which cell contains the query point (at most log2(N) tests where N is the total number of points in the cloud)
	while (!(cellPtr->leSon == nullptr && cellPtr->gSon == nullptr))
	{
		if (queryPoint[cellPtr->cuttingDim] <= cellPtr->cuttingCoordinate)
			cellPtr = cellPtr->leSon;
		else
			cellPtr = cellPtr->gSon;
	}

	//Once we found the cell containing the query point, there are great chance to find a point if it exists
	for (unsigned i=0; i<cellPtr->nbPoints; i++)
	{
		const CCVector3 *p = m_associatedCloud->getPoint(m_indexes[cellPtr->startingPointIndex+i]);
		PointCoordinateType sqrdist = CCVector3::vdistance2(p->u, queryPoint);
		if (sqrdist < static_cast<PointCoordinateType>(maxDist))
			return true;
	}

	//Go up in the tree to check that neighbours cells do not contain a point
	while (cellPtr != nullptr)
	{
		KdCell* prevPtr = cellPtr;
		cellPtr = cellPtr->father;
		if (cellPtr != nullptr)
		{
			ScalarType sqrdist = InsidePointToCellDistance(queryPoint, cellPtr);
			if (sqrdist >= 0 && sqrdist*sqrdist < maxDist)
			{
				KdCell* brotherPtr = (cellPtr->leSon == prevPtr ? cellPtr->gSon : cellPtr->leSon);
				if (checkDistantPointInSubTree(queryPoint, maxDist, brotherPtr))
					return true;
			}
			else
			{
				cellPtr = nullptr;
			}
		}
	}

	return false;
}

unsigned KDTree::findPointsLyingToDistance(const PointCoordinateType *queryPoint,
											ScalarType distance,
											ScalarType tolerance,
											std::vector<unsigned> &points)
{
	if (m_root == nullptr)
		return 0;

	distanceScanTree(queryPoint, distance, tolerance, m_root, points);

	return static_cast<unsigned>(points.size());
}


void KDTree::updateInsideBoundingBox(KdCell* cell)
{
	if (cell->leSon != nullptr && cell->gSon != nullptr)
	{
		cell->inbbmax.x = std::max(cell->leSon->inbbmax.x, cell->gSon->inbbmax.x);
		cell->inbbmax.y = std::max(cell->leSon->inbbmax.y, cell->gSon->inbbmax.y);
		cell->inbbmax.z = std::max(cell->leSon->inbbmax.z, cell->gSon->inbbmax.z);
		cell->inbbmin.x = std::min(cell->leSon->inbbmin.x, cell->gSon->inbbmin.x);
		cell->inbbmin.y = std::min(cell->leSon->inbbmin.y, cell->gSon->inbbmin.y);
		cell->inbbmin.z = std::min(cell->leSon->inbbmin.z, cell->gSon->inbbmin.z);
	}
	else
	{
		const CCVector3* P = m_associatedCloud->getPoint(m_indexes[cell->startingPointIndex]);
		cell->inbbmin = cell->inbbmax = *P;
		for (unsigned i=1; i<cell->nbPoints; i++)
		{
			P = m_associatedCloud->getPoint(m_indexes[i+cell->startingPointIndex]);
			cell->inbbmax.x = std::max(cell->inbbmax.x, P->x);
			cell->inbbmax.y = std::max(cell->inbbmax.y, P->y);
			cell->inbbmax.z = std::max(cell->inbbmax.z, P->z);
			cell->inbbmin.x = std::min(cell->inbbmin.x, P->x);
			cell->inbbmin.y = std::min(cell->inbbmin.y, P->y);
			cell->inbbmin.z = std::min(cell->inbbmin.z, P->z);
		}
	}
}


void KDTree::updateOutsideBoundingBox(KdCell *cell)
{
	if (cell->father == nullptr)
	{
		cell->boundsMask = 0;
	}
	else
	{
		unsigned char bound = 1;
		cell->boundsMask = cell->father->boundsMask;
		cell->outbbmax = cell->father->outbbmax;
		cell->outbbmin = cell->father->outbbmin;
		const CCVector3* P = m_associatedCloud->getPoint(m_indexes[cell->startingPointIndex]);
		//Check if this cell is its father leSon (if...) or gSon (else...)
		if (P->u[cell->father->cuttingDim] <= cell->father->cuttingCoordinate)
		{
			//Bounding box max point is linked to the bits [3..5] in the bounds mask
			bound = bound<<(3+cell->father->cuttingDim);
			cell->boundsMask = cell->boundsMask | bound;
			cell->outbbmax.u[cell->father->cuttingDim] = cell->father->cuttingCoordinate;
		}
		else
		{
			//Bounding box min point is linked to the bits[0..2] in the bounds mask
			bound = bound<<(cell->father->cuttingDim);
			cell->boundsMask = cell->boundsMask | bound;
			cell->outbbmin.u[cell->father->cuttingDim] = cell->father->cuttingCoordinate;
		}
	}
}

ScalarType KDTree::pointToCellSquareDistance(const PointCoordinateType *queryPoint, KdCell *cell)
{
	PointCoordinateType dx;
	PointCoordinateType dy;
	PointCoordinateType dz;

	//Each d(x)(y)(z) represents the distance to the nearest bounding box plane (if the point is outside)
	if (cell->inbbmin.x <= queryPoint[0] && queryPoint[0] <= cell->inbbmax.x)
		dx = 0;
	else
		dx = std::min(std::abs(queryPoint[0]-cell->inbbmin.x), std::abs(queryPoint[0]-cell->inbbmax.x));
	
	if (cell->inbbmin.y <= queryPoint[1] && queryPoint[1] <= cell->inbbmax.y)
		dy = 0;
	else
		dy = std::min(std::abs(queryPoint[1]-cell->inbbmin.y), std::abs(queryPoint[1]-cell->inbbmax.y));
	
	if (cell->inbbmin.z <= queryPoint[2] && queryPoint[2] <= cell->inbbmax.z)
		dz = 0;
	else
		dz = std::min(std::abs(queryPoint[2]-cell->inbbmin.z), std::abs(queryPoint[2]-cell->inbbmax.z));

	return static_cast<ScalarType>(dx*dx + dy*dy + dz*dz);
}

void KDTree::pointToCellDistances(	const PointCoordinateType *queryPoint,
									KdCell *cell,
									ScalarType& min,
									ScalarType& max)
{
	PointCoordinateType dx;
	PointCoordinateType dy;
	PointCoordinateType dz;

	min = sqrt(pointToCellSquareDistance(queryPoint, cell));
	dx = std::max(std::abs(queryPoint[0]-cell->inbbmin.x), std::abs(queryPoint[0]-cell->inbbmax.x));
	dy = std::max(std::abs(queryPoint[1]-cell->inbbmin.y), std::abs(queryPoint[1]-cell->inbbmax.y));
	dz = std::max(std::abs(queryPoint[2]-cell->inbbmin.z), std::abs(queryPoint[2]-cell->inbbmax.z));
	max = static_cast<ScalarType>( sqrt(dx*dx + dy*dy + dz*dz) );
}

ScalarType KDTree::InsidePointToCellDistance(const PointCoordinateType *queryPoint, KdCell *cell)
{
	PointCoordinateType dx;
	PointCoordinateType dy;
	PointCoordinateType dz;
	PointCoordinateType max;

	dx = dy = dz = -1;

	if ((cell->boundsMask&1) && (cell->boundsMask&8))
		dx = std::min(std::abs(queryPoint[0]-cell->outbbmin.x), std::abs(queryPoint[0]-cell->outbbmax.x));
	else if (cell->boundsMask&1)
		dx = std::abs(queryPoint[0]-cell->outbbmin.x);
	else if (cell->boundsMask&8)
		dx = std::abs(queryPoint[0]-cell->outbbmax.x);

	if ((cell->boundsMask&2) && (cell->boundsMask&16))
		dy = std::min(std::abs(queryPoint[1]-cell->outbbmin.y), std::abs(queryPoint[1]-cell->outbbmax.y));
	else if (cell->boundsMask&2)
		dy = std::abs(queryPoint[1]-cell->outbbmin.y);
	else if (cell->boundsMask&16)
		dy = std::abs(queryPoint[1]-cell->outbbmax.y);

	if ((cell->boundsMask&4) && (cell->boundsMask&32))
		dz = std::min(std::abs(queryPoint[2]-cell->outbbmin.z), std::abs(queryPoint[2]-cell->outbbmax.z));
	else if (cell->boundsMask&4)
		dz = std::abs(queryPoint[2]-cell->outbbmin.z);
	else if (cell->boundsMask&32)
		dz = std::abs(queryPoint[2]-cell->outbbmax.z);

	if (dx < 0 && dy < 0 && dz < 0)
		return -1;

	max = std::max(dx, std::max(dy, dz));
	if (dx < 0)
		dx = max;
	if (dy < 0)
		dy = max;
	if (dz < 0)
		dz = max;

	return static_cast<ScalarType>( std::min(dx, std::min(dy, dz)) );
}

int KDTree::checkNearerPointInSubTree(	const PointCoordinateType *queryPoint,
										ScalarType& maxSqrDist,
										KdCell *cell)
{
	if (pointToCellSquareDistance(queryPoint, cell) >= maxSqrDist)
		return -1;

	if (cell->leSon == nullptr && cell->gSon == nullptr)
	{
		int a = -1;
		for (unsigned i=0; i<cell->nbPoints; i++)
		{
			const CCVector3 *p = m_associatedCloud->getPoint(m_indexes[cell->startingPointIndex+i]);
			PointCoordinateType dist = CCVector3::vdistance2(p->u, queryPoint);
			if (dist < maxSqrDist)
			{
				a = m_indexes[cell->startingPointIndex+i];
				maxSqrDist = static_cast<ScalarType>(dist);
			}
		}

		return a;
	}

	int b = checkNearerPointInSubTree(queryPoint, maxSqrDist, cell->gSon);
	if (b >= 0)
		return b;

	return checkNearerPointInSubTree(queryPoint, maxSqrDist, cell->leSon);
}

bool KDTree::checkDistantPointInSubTree(const PointCoordinateType *queryPoint, ScalarType &maxSqrDist, KdCell *cell)
{
	if (pointToCellSquareDistance(queryPoint, cell)>=maxSqrDist)
		return false;

	if (cell->leSon == nullptr && cell->gSon == nullptr)
	{
		for (unsigned i=0; i<cell->nbPoints; i++)
		{
			const CCVector3 *p = m_associatedCloud->getPoint(m_indexes[cell->startingPointIndex+i]);
			PointCoordinateType dist = CCVector3::vdistance2(p->u, queryPoint);
			if (dist < maxSqrDist)
				return true;
		}
		return false;
	}

	if (checkDistantPointInSubTree(queryPoint, maxSqrDist, cell->leSon))
		return true;
	if (checkDistantPointInSubTree(queryPoint, maxSqrDist, cell->gSon))
		return true;

	return false;
}

void KDTree::distanceScanTree(
	const PointCoordinateType *queryPoint,
	ScalarType distance,
	ScalarType tolerance,
	KdCell *cell,
	std::vector<unsigned> &localArray)
{
	ScalarType min;
	ScalarType max;

	pointToCellDistances(queryPoint, cell, min, max);

	if ((min<=distance+tolerance) && (max>=distance-tolerance))
	{
		if ((cell->leSon!=nullptr) && (cell->gSon!=nullptr))
		{
			//This case shall always happen (the other case is for leaves that contain more than one point - bucket KDtree)
			if (cell->nbPoints == 1)
			{
				localArray.push_back(m_indexes[cell->startingPointIndex]);
			}
			else
			{
				for (unsigned i=0; i<cell->nbPoints; i++)
				{
					const CCVector3 *p = m_associatedCloud->getPoint(m_indexes[i+cell->startingPointIndex]);
					PointCoordinateType dist = CCVector3::vdistance(queryPoint, p->u);
					if (distance-tolerance <= dist && dist <= distance+tolerance)
						localArray.push_back(m_indexes[cell->startingPointIndex+i]);
				}
			}
		}
		else
		{
			distanceScanTree(queryPoint, distance, tolerance, cell->leSon, localArray);
			distanceScanTree(queryPoint, distance, tolerance, cell->gSon, localArray);
		}
	}
}
