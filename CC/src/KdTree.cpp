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
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#include "KdTree.h"
#include "CCMiscTools.h"
#include <algorithm>

using namespace CCLib;

KDTree::KDTree()
{
    root = 0;
    associatedCloud = 0;
    nbCell = 0;
}

KDTree::~KDTree()
{
    DeleteSubTree(root);
    //associatedCloud = 0;
    //nbCell = 0;
    //root = 0;
}

bool KDTree::BuildFromCloud(GenericIndexedCloud *cloud, GenericProgressCallback *progressCb)
{
    unsigned i, cloudsize=cloud->size();

    indexes.clear();
    nbCell = 0;
	associatedCloud = 0;

    if(cloudsize == 0)
        return false;

	try
	{
		list.resize(cloudsize);
	}
	catch (.../*const std::bad_alloc&*/) //out of memory
	{
		return false;
	}

	associatedCloud = cloud;

	for(i=0; i<cloudsize; i++)
    {
        list[i].index = i;
        cloud->getPoint(i,list[i].point);
    }

    if(progressCb)
    {
        progressCb->reset();
        progressCb->setInfo("Building KD-tree");
        progressCb->start();
    }

    root = BuildSubTree(0, cloudsize-1, (kdcell*)0, nbCell, progressCb);

    if(progressCb)
        progressCb->stop();

    //if the tree building has failed (memory issues)
    if(!root)
    {
        associatedCloud = 0;
        nbCell = 0;
        return false;
    }

	try
	{
		indexes.resize(cloudsize);
	}
	catch (.../*const std::bad_alloc&*/) //out of memory
	{
        associatedCloud = 0;
        nbCell = 0;
        return false;
    }
    for(i=0; i<cloudsize; i++)
        indexes[i]=list[i].index;
    list.clear();

    return true;
}


void KDTree::DeleteSubTree(kdcell *cell)
{
    if(!cell)
        return;

    DeleteSubTree(cell->leSon);
    DeleteSubTree(cell->gSon);
    delete cell;
    nbCell--;
}

KDTree::kdcell* KDTree::BuildSubTree(unsigned first, unsigned last, kdcell* father, unsigned &nbBuildCell, GenericProgressCallback *progressCb)
{
    kdcell* cell = new kdcell;
    if (!cell)
        return 0;
    nbCell++;

    unsigned dim = (father == 0 ? 0 : (father->cuttingDim+1)%3 );

    //Compute outside bounding box (have to be done before building the current cell sons)
    cell->father = father;
    cell->startingPointIndex = first;
    cell->nbPoints = last-first+1;
    cell->cuttingDim = dim;
    UpdateOutsideBoundingBox(cell);
    if(progressCb)
        progressCb->update((float)nbCell*100.0f/(float)(list.size()*2-1));

    //If there is only one point to insert, build a leaf
    if(first == last)
    {
        cell->cuttingDim = 0;
        cell->leSon = 0;
        cell->gSon = 0;
    }
    else
    {
        //sort the remaining points considering dimension dim
        if(dim == 0)
            sort(list.begin()+first, list.begin()+(last+1), ComparisonX);
        else if(dim == 1)
            sort(list.begin()+first, list.begin()+(last+1), ComparisonY);
        else if(dim == 2)
            sort(list.begin()+first, list.begin()+(last+1), ComparisonZ);
        //find the median point in the sorted tab
        unsigned split = (first+last)/2;
        const CCVector3& p = list[split].point;
        cell->cuttingCoordinate = p.u[dim];
        //recursively build the other two sub trees
        //trap the memory issues. At this point, none of the cell sons can be set to 0. Otherwise there has been memory allocation failure.
        cell->leSon = cell->gSon = 0;
        cell->leSon = BuildSubTree(first, split, cell, nbBuildCell, progressCb);
        if(cell->leSon == 0)
        {
            DeleteSubTree(cell);
            //the tree beyond the current cell will be deleted when noticing that this cell is set to 0
            return 0;
        }
        cell->gSon = BuildSubTree(split+1, last, cell, nbBuildCell, progressCb);
        if(cell->gSon == 0)
        {
            DeleteSubTree(cell);
            //the tree beyond the current cell will be deleted when noticing that this cell is set to 0
            return 0;
        }

    }
    //Compute inside bounding box (have to be done once sons have been built)
    UpdateInsideBoundingBox(cell);

    return cell;
}


bool KDTree::FindNearestNeighbour(
    const PointCoordinateType *queryPoint,
    unsigned &nearestPointIndex,
    PointCoordinateType maxDist)
{
    kdcell *cellPtr, *prevPtr, *brotherPtr;
    unsigned i;
    int a;
    PointCoordinateType sqrdist;
    bool found = false;

    if(root == 0)
        return false;

    cellPtr = root;
    maxDist *= maxDist;
    //Go down the tree to find which cell contains the query point (at most log2(N) tests where N is the total number of points in the cloud)
    while(!(cellPtr->leSon == 0 && cellPtr->gSon == 0))
    {
        if(queryPoint[cellPtr->cuttingDim]<=cellPtr->cuttingCoordinate)
            cellPtr = cellPtr->leSon;
        else
            cellPtr = cellPtr->gSon;
    }

    //Once we found the cell containing the query point, the nearest neighbour has great chances to lie in this cell
    for(i=0; i<cellPtr->nbPoints; i++)
    {
        const CCVector3 *p = associatedCloud->getPoint(indexes[cellPtr->startingPointIndex+i]);
        sqrdist = CCVector3::vdistance2(p->u, queryPoint);
        if(sqrdist<maxDist)
        {
            maxDist = sqrdist;
            nearestPointIndex = indexes[cellPtr->startingPointIndex+i];
            found = true;
        }
    }

    //Go up in the tree to check that neighbours cells do not contain a nearer point than the one we found
    while(cellPtr != 0)
    {
        prevPtr = cellPtr;
        cellPtr = cellPtr->father;
        if(cellPtr != 0)
        {
            sqrdist = InsidePointToCellDistance(queryPoint, cellPtr);
            if(sqrdist>=0. && sqrdist*sqrdist<maxDist)
            {
                if(cellPtr->leSon == prevPtr)
                    brotherPtr = cellPtr->gSon;
                else
                    brotherPtr = cellPtr->leSon;
                a = CheckNearerPointInSubTree(queryPoint, maxDist, brotherPtr);
                if(a >= 0)
                {
                    nearestPointIndex = a;
                    found = true;
                }
            }
            else
                cellPtr = 0;
        }
    }

    return found;
}


bool KDTree::FindPointBelowDistance(
    const PointCoordinateType *queryPoint,
    PointCoordinateType maxDist)
{
    kdcell *cellPtr, *prevPtr, *brotherPtr;
    unsigned i;
    PointCoordinateType sqrdist;

    if(root == 0)
        return false;

    cellPtr = root;
    maxDist *= maxDist;
    //Go down the tree to find which cell contains the query point (at most log2(N) tests where N is the total number of points in the cloud)
    while(!(cellPtr->leSon == 0 && cellPtr->gSon == 0))
    {
        if(queryPoint[cellPtr->cuttingDim]<=cellPtr->cuttingCoordinate)
            cellPtr = cellPtr->leSon;
        else
            cellPtr = cellPtr->gSon;
    }

    //Once we found the cell containing the query point, there are great chance to find a point if it exists
    for(i=0; i<cellPtr->nbPoints; i++)
    {
		const CCVector3 *p = associatedCloud->getPoint(indexes[cellPtr->startingPointIndex+i]);
        sqrdist = CCVector3::vdistance2(p->u, queryPoint);
        if(sqrdist<maxDist)
            return true;
    }

    //Go up in the tree to check that neighbours cells do not contain a point
    while(cellPtr != 0)
    {
        prevPtr = cellPtr;
        cellPtr = cellPtr->father;
        if(cellPtr != 0)
        {
            sqrdist = InsidePointToCellDistance(queryPoint, cellPtr);
            if(sqrdist>=0. && sqrdist*sqrdist<maxDist)
            {
                if(cellPtr->leSon == prevPtr)
                    brotherPtr = cellPtr->gSon;
                else
                    brotherPtr = cellPtr->leSon;
                if(CheckDistantPointInSubTree(queryPoint, maxDist, brotherPtr))
                    return true;
            }
            else
                cellPtr = 0;
        }
    }

    return false;
}

unsigned KDTree::FindPointsLyingToDistance(
    const PointCoordinateType *queryPoint,
    PointCoordinateType distance,
    PointCoordinateType tolerance,
    std::vector<unsigned> &points)
{
    if(root == 0)
        return 0;

    DistanceScanTree(queryPoint, distance, tolerance, root, points);

    return points.size();
}


void KDTree::UpdateInsideBoundingBox(kdcell* cell)
{
    if((cell->leSon!=0) && (cell->gSon!=0))
    {
        cell->inbbmax.x = ccMax(cell->leSon->inbbmax.x, cell->gSon->inbbmax.x);
        cell->inbbmax.y = ccMax(cell->leSon->inbbmax.y, cell->gSon->inbbmax.y);
        cell->inbbmax.z = ccMax(cell->leSon->inbbmax.z, cell->gSon->inbbmax.z);
        cell->inbbmin.x = ccMin(cell->leSon->inbbmin.x, cell->gSon->inbbmin.x);
        cell->inbbmin.y = ccMin(cell->leSon->inbbmin.y, cell->gSon->inbbmin.y);
        cell->inbbmin.z = ccMin(cell->leSon->inbbmin.z, cell->gSon->inbbmin.z);
    }
    else
    {
        CCVector3& p = list[cell->startingPointIndex].point;
        cell->inbbmin = cell->inbbmax = p;
        for(unsigned i=1; i<cell->nbPoints; i++)
        {
            p = list[i+cell->startingPointIndex].point;
            cell->inbbmax.x = ccMax(cell->inbbmax.x, p.x);
            cell->inbbmax.y = ccMax(cell->inbbmax.y, p.y);
            cell->inbbmax.z = ccMax(cell->inbbmax.z, p.z);
            cell->inbbmin.x = ccMin(cell->inbbmin.x, p.x);
            cell->inbbmin.y = ccMin(cell->inbbmin.y, p.y);
            cell->inbbmin.z = ccMin(cell->inbbmin.z, p.z);
        }
    }
}


void KDTree::UpdateOutsideBoundingBox(kdcell *cell)
{
    if(cell->father == 0)
    {
        cell->boundsMask = 0;
    }
    else
    {
        unsigned char bound = 1;
        cell->boundsMask = cell->father->boundsMask;
        cell->outbbmax = cell->father->outbbmax;
        cell->outbbmin = cell->father->outbbmin;
        const CCVector3& p = list[cell->startingPointIndex].point;
        //Check if this cell is its father leSon (if...) or gSon (else...)
        if(p.u[cell->father->cuttingDim] <= cell->father->cuttingCoordinate)
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


DistanceType KDTree::PointToCellSquareDistance(const PointCoordinateType *queryPoint, kdcell *cell)
{
    DistanceType dx, dy, dz;

    //Each d(x)(y)(z) represents the distance to the nearest bounding box plane (if the point is outside)
    if(cell->inbbmin.x<=queryPoint[0] && queryPoint[0]<=cell->inbbmax.x)
        dx = 0.;
    else
        dx = ccMin(fabs(queryPoint[0]-cell->inbbmin.x), fabs(queryPoint[0]-cell->inbbmax.x));
    if(cell->inbbmin.y<=queryPoint[1] && queryPoint[1]<=cell->inbbmax.y)
        dy = 0.;
    else
        dy = ccMin(fabs(queryPoint[1]-cell->inbbmin.y), fabs(queryPoint[1]-cell->inbbmax.y));
    if(cell->inbbmin.z<=queryPoint[2] && queryPoint[2]<=cell->inbbmax.z)
        dz = 0.;
    else
        dz = ccMin(fabs(queryPoint[2]-cell->inbbmin.z), fabs(queryPoint[2]-cell->inbbmax.z));

    return (dx*dx)+(dy*dy)+(dz*dz);
}


void KDTree::PointToCellDistances(const PointCoordinateType *queryPoint, kdcell *cell, DistanceType& min, DistanceType &max)
{
    DistanceType dx, dy, dz;

    min = sqrt(PointToCellSquareDistance(queryPoint, cell));
    dx = ccMax(fabs(queryPoint[0]-cell->inbbmin.x), fabs(queryPoint[0]-cell->inbbmax.x));
    dy = ccMax(fabs(queryPoint[1]-cell->inbbmin.y), fabs(queryPoint[1]-cell->inbbmax.y));
    dz = ccMax(fabs(queryPoint[2]-cell->inbbmin.z), fabs(queryPoint[2]-cell->inbbmax.z));
    max = sqrt((dx*dx)+(dy*dy)+(dz*dz));
}


DistanceType KDTree::InsidePointToCellDistance(const PointCoordinateType *queryPoint, kdcell *cell)
{
    DistanceType dx, dy, dz, max;

    dx = dy = dz = -1;

    if((cell->boundsMask&1) && (cell->boundsMask&8))
        dx = ccMin(fabs(queryPoint[0]-cell->outbbmin.x), fabs(queryPoint[0]-cell->outbbmax.x));
    else if(cell->boundsMask&1)
        dx = fabs(queryPoint[0]-cell->outbbmin.x);
    else if(cell->boundsMask&8)
        dx = fabs(queryPoint[0]-cell->outbbmax.x);

    if((cell->boundsMask&2) && (cell->boundsMask&16))
        dy = ccMin(fabs(queryPoint[1]-cell->outbbmin.y), fabs(queryPoint[1]-cell->outbbmax.y));
    else if(cell->boundsMask&2)
        dy = fabs(queryPoint[1]-cell->outbbmin.y);
    else if(cell->boundsMask&16)
        dy = fabs(queryPoint[1]-cell->outbbmax.y);

    if((cell->boundsMask&4) && (cell->boundsMask&32))
        dz = ccMin(fabs(queryPoint[2]-cell->outbbmin.z), fabs(queryPoint[2]-cell->outbbmax.z));
    else if(cell->boundsMask&4)
        dz = fabs(queryPoint[2]-cell->outbbmin.z);
    else if(cell->boundsMask&32)
        dz = fabs(queryPoint[2]-cell->outbbmax.z);

    if(dx < 0. && dy < 0. && dz < 0.)
        return -1.;

    max = ccMax(dx, ccMax(dy, dz));
    if(dx < 0.)
        dx = max;
    if(dy < 0.)
        dy = max;
    if(dz < 0.)
        dz = max;

    return ccMin(dx, ccMin(dy, dz));
}


int KDTree::CheckNearerPointInSubTree(const PointCoordinateType *queryPoint, DistanceType &maxSqrDist, kdcell *cell)
{
    if(PointToCellSquareDistance(queryPoint, cell)>=maxSqrDist)
        return -1;

    if(cell->leSon == 0 && cell->gSon == 0)
    {
        int a = -1;
        for(unsigned i=0; i<cell->nbPoints; i++)
        {
            const CCVector3 *p = associatedCloud->getPoint(indexes[cell->startingPointIndex+i]);
            DistanceType dist = CCVector3::vdistance2(p->u, queryPoint);
            if(dist<maxSqrDist)
            {
                a = indexes[cell->startingPointIndex+i];
                maxSqrDist = dist;
            }
        }

        return a;
    }

	int b = CheckNearerPointInSubTree(queryPoint,  maxSqrDist, cell->gSon);
	if (b >= 0)
		return b;

	return CheckNearerPointInSubTree(queryPoint,  maxSqrDist, cell->leSon);
}


bool KDTree::CheckDistantPointInSubTree(const PointCoordinateType *queryPoint, DistanceType &maxSqrDist, kdcell *cell)
{
    if(PointToCellSquareDistance(queryPoint, cell)>=maxSqrDist)
        return false;

    if(cell->leSon == 0 && cell->gSon == 0)
    {
        for(unsigned i=0; i<cell->nbPoints; i++)
        {
            const CCVector3 *p = associatedCloud->getPoint(indexes[cell->startingPointIndex+i]);
            DistanceType dist = CCVector3::vdistance2(p->u, queryPoint);
            if(dist<maxSqrDist)
                return true;
        }
        return false;
    }

    if(CheckDistantPointInSubTree(queryPoint,  maxSqrDist, cell->leSon))
        return true;
    if(CheckDistantPointInSubTree(queryPoint,  maxSqrDist, cell->gSon))
        return true;

    return false;
}


void KDTree::DistanceScanTree(
    const PointCoordinateType *queryPoint,
    DistanceType distance,
    DistanceType tolerance,
    kdcell *cell,
    std::vector<unsigned> &localArray)
{
    DistanceType min, max;

    PointToCellDistances(queryPoint, cell, min, max);

    if((min<=distance+tolerance) && (max>=distance-tolerance))
    {
        if((cell->leSon!=0) && (cell->gSon!=0))
        {
            //This case shall allways happen (the other case is for leaves that contain more than one point - bucket KDtree)
            if(cell->nbPoints == 1)
            {
                localArray.push_back(indexes[cell->startingPointIndex]);
            }
            else
            {
                for(unsigned i=0; i<cell->nbPoints; i++)
                {
                    const CCVector3 *p = associatedCloud->getPoint(indexes[i+cell->startingPointIndex]);
                    DistanceType dist = CCVector3::vdistance(queryPoint, p->u);
                    if((distance-tolerance<=dist) && (dist<=distance+tolerance))
                        localArray.push_back(indexes[cell->startingPointIndex+i]);
                }
            }
        }
        else
        {
            DistanceScanTree(queryPoint, distance, tolerance, cell->leSon, localArray);
            DistanceScanTree(queryPoint, distance, tolerance, cell->gSon, localArray);
        }
    }
}
