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

#ifndef KD_TREE_HEADER
#define KD_TREE_HEADER

#ifdef _MSC_VER
//To get rid of the really annoying warnings about template class exportation
#pragma warning( disable: 4251 )
#pragma warning( disable: 4530 )
#endif

#include <math.h>
#include <vector>

#include "GenericIndexedCloud.h"
#include "PointProjectionTools.h"
#include "GenericProgressCallback.h"

namespace CCLib
{
//!A Kd Tree Class which implements functions related to point to point distance
#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"

class CC_DLL_API KDTree
#else
class KDTree
#endif
{
public:

    //! Default constructor
    KDTree();

    //! Default constructor
    virtual ~KDTree();

    //! Builds the KD-tree
    /** \param cloud the point cloud from which to buil the KDtree
		\param progressCb the client method can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return success
    **/
    bool BuildFromCloud(GenericIndexedCloud *cloud, GenericProgressCallback *progressCb = 0);

    //! Gets the point cloud from which the tree has been build
    /** \return associated cloud
    **/
    GenericIndexedCloud* GetAssociatedCloud() const { return associatedCloud; }

    //! Nearest point search
    /**!
        \param queryPoint coordinates of the query point from which we want the nearest point in the tree
        \param nearestPointIndex [out] index of the point that lies the nearest from query Point. Corresponding coordinates can be retrieved using GetAssociatedCloud()->getPoint(nearestPointIndex)
        \param maxDist distance above which the function doesn't consider points
        \return true if it finds a point p such that ||p-queryPoint||<=maxDist. False otherwise
    **/
    bool FindNearestNeighbour(const PointCoordinateType *queryPoint,
                                unsigned &nearestPointIndex,
                                PointCoordinateType maxDist);


    //! Optimized version of nearest point research which only check if there is a point p int the tree such that ||p-queryPoint||<=maxDist (see FindNearestNeighbour())
    bool FindPointBelowDistance(
        const PointCoordinateType *queryPoint,
        PointCoordinateType maxDist);


    //! Searches for the points that lie to a given distance (up to a tolerance) from a query point
    /**
        \param queryPoint query point coordinates
        \param distance distance wished between the query point and resulting points
        \param tolerance error allowed by the function : each resulting point p is such that distance-tolerance<=||p-queryPoint||<=distance+tolerance
        \param points [out] array of point indexes. Each point stored in this array lie to distance (up to tolerance) from queryPoint
        \return the number of matching points
    **/
    unsigned FindPointsLyingToDistance(
        const PointCoordinateType *queryPoint,
        PointCoordinateType distance,
        PointCoordinateType tolerance,
        std::vector<unsigned> &points);



protected:

    //! A KDTre cell struct
    typedef struct kd_cell
    {
        //!Inside bounding box max point (the inside bounding box is the smallest cube containing all the points in the cell)
        CCVector3 inbbmax;
        //!Inside bounding box min point (the inside bounding box is the smallest cube containing all the points in the cell)
        CCVector3 inbbmin;
        //!Outside bounding box min point (the outside bounding box is the bigest cube contained inside the cutting planes that lead to the cell)
        CCVector3 outbbmin;
        //!Outside bounding box max point (the outside bounding box is the bigest cube contained inside the cutting planes that lead to the cell)
        CCVector3 outbbmax;
        //!mask to know if the outside box is bounded for a given dimmension
        /**if boundsMask & (2^d) then outbbmin.u[d] is bounded (else the box is opened in outmin.u[d] - i.e. outbbmin.u[d] = -infinite)
        if boundsmask & (2^(3+d)) then outbbmax.u[d] is bounded (else the box is opened in outmax.u[d] - i.e. outbbmax.u[d] = infinite)*/
        unsigned char boundsMask;
        //!Dimension (0, 1 or 2 for x, y or z) which is used to separate the two sons
        unsigned cuttingDim;
        //!Place where the space is cut into two sub-spaces (sons)
        PointCoordinateType cuttingCoordinate;
        //!Each point p which lie in leSon is such as p[cuttingDim] <= cuttingCoordinate
        struct kd_cell* leSon;
        //!Each point p which lie in gSon is such as p[cuttingDim] > cuttingCoordinate
        struct kd_cell* gSon;
        //!To go up in the tree
        struct kd_cell* father;
        //!Index of the first element that belongs to this cell
        unsigned startingPointIndex;
        //!Number of elements in this cell
        unsigned nbPoints;
    } kdcell;


    //! Structure to link a point and its index
    typedef struct
    {
        //! Point index
        unsigned index;
        //! Point coordinates
        CCVector3 point; //DGM (02/10/2011): has to be a copy now, as it is not compatible with parallel strategies or 'light' interaction with client db
    } PointAndIndex;


    /*** Protected attributes ***/

    //! Tree root
    kdcell *root;
    //! Point indexes
    std::vector<unsigned> indexes;
    //! Array
    std::vector<PointAndIndex> list;
    //! Associated cloud
    GenericIndexedCloud *associatedCloud;
    //! Number of cells
    unsigned nbCell;


    /*** Protected methods ***/

    //! Builds a sub tree
    /** \param first first index
        \param last last index
        \param father father cell
        \param nbBuildCell nbBuildCell
		\param progressCb the client method can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return sub tree (cell)
    **/
    kdcell *BuildSubTree(unsigned first, unsigned last, kdcell *father, unsigned &nbBuildCell, GenericProgressCallback *progressCb = 0);

    //! Deletes a sub tree
    void DeleteSubTree(kdcell *cell);


    //! Computes a cell inside bounding box using the sons ones. The sons bounding boxes have to be up to date considering the points they contain.
    void UpdateInsideBoundingBox(kdcell* cell);


    //! Computes a cell outside bounding box using the father one and the cutting plane.
    void UpdateOutsideBoundingBox(kdcell *cell);


    //! Computes the distance between a point and a cell inside bounding box
    /**!
        \param queryPoint queryPoint coordinates
        \param cell the cell from which we want to compute the distance
        \return 0 if the point is inside the cell, the suare of the distance bewteen the two elements if the point is outside
    **/
    DistanceType PointToCellSquareDistance(const PointCoordinateType *queryPoint, kdcell *cell);


    //! Computes the distance between a point and the outside bounding box of the cell in which it lies.
    /**!
        \param queryPoint the query point coordinates
        \param cell the cell containting the query point
        \return the distance between the point and the cell border. If this value is negative, it means that the cell has no border.
    **/
    DistanceType InsidePointToCellDistance(const PointCoordinateType *queryPoint, kdcell *cell);

    //! Computes the distances (min & max) between a point and a cell inside bounding box
    /**!
        \param queryPoint the query point coordinates
        \param cell the cell from which we want to compute the distance
        \param min [out] the minimal distance between the query point and the inside bounding box of cell
        \param max [out] the maximal distance between the query point and the inside bounding box of cell
    **/
    void PointToCellDistances(const PointCoordinateType *queryPoint, kdcell *cell, DistanceType &min, DistanceType &max);


    //! Checks if there is a point in kdcell that is less than minDist-appart from the query point, starting from cell cell
    /**
        \param queryPoint the query Point coordinates
        \param maxSqrDist square of the maximal distance from querypoint
        \param cell kdtree-cell from which to start the research
        \return -1 if there is no nearer point from querypoint. The nearest point index found in cell if there is one that is at most maxdist appart from querypoint
    **/
    int CheckNearerPointInSubTree(const PointCoordinateType *queryPoint, DistanceType& maxSqrDist, kdcell *cell);


    //! Checks if there is a point in kdcell that is less than minDist-appart from the query point, starting from cell cell
    /** Optimiszed version of CheckNearerPointInSubTree since we don't want to find the nearest point, but only check if there is a point that is close enough
        \param queryPoint the query Point coordinates
        \param maxSqrDist square of the maximal distance from querypoint
        \param cell kdtree-cell from which to start the research
        \return true if there is a point in the subtree starting at cell that is close enough from the query point
    **/
    bool CheckDistantPointInSubTree(const PointCoordinateType *queryPoint, DistanceType &maxSqrDist, kdcell *cell);


    //! Recursive function which store every point lying to a given distance from the query point
    /**!
        \param queryPoint the query point coordinates
        \param distance the wished distance from the query point
        \param tolerance tolerance for resulting points : p is in resulting array if ||p-queryPoint||<=tolerance
        \param cell current cell to explore (used for recursion)
        \param array [out] output of the algorithm. Resulting points indexes in associatedCloud are stored in this array
    **/
    void DistanceScanTree(const PointCoordinateType *queryPoint, 
							DistanceType distance, 
							DistanceType tolerance, 
							kdcell *cell, 
							std::vector<unsigned> &localArray);


    /*** Comparison functions used by the sort function (Strict ordering must be used) ***/

    //! Compares X coordinates of two points designated by their index
    static bool ComparisonX(const PointAndIndex &a, const PointAndIndex &b)
    {
        return (a.point.x<b.point.x);
    }

    //! Compares Y coordinates of two points designated by their index
    static bool ComparisonY(const PointAndIndex &a, const PointAndIndex &b)
    {
        return (a.point.y<b.point.y);
    }

    //! Compares Z coordinates of two points designated by their index
    static bool ComparisonZ(const PointAndIndex &a, const PointAndIndex &b)
    {
        return (a.point.z<b.point.z);
    }
};

}
#endif
