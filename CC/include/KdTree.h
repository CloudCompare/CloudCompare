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

#ifndef KD_TREE_HEADER
#define KD_TREE_HEADER

//Local
#include "PointProjectionTools.h"

namespace CCLib
{

class GenericIndexedCloud;
class GenericProgressCallback;

//! A Kd Tree Class which implements functions related to point to point distance
class CC_CORE_LIB_API KDTree
{
public:

	//! Default constructor
	KDTree();

	//! Destructor
	virtual ~KDTree();

	//! Builds the KD-tree
	/** \param cloud the point cloud from which to buil the KDtree
		\param progressCb the client method can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return success
	**/
	bool buildFromCloud(GenericIndexedCloud *cloud, GenericProgressCallback *progressCb = nullptr);

	//! Gets the point cloud from which the tree has been build
	/** \return associated cloud
	**/
	GenericIndexedCloud* getAssociatedCloud() const { return m_associatedCloud; }

	//! Nearest point search
	/** \param queryPoint coordinates of the query point from which we want the nearest point in the tree
		\param nearestPointIndex [out] index of the point that lies the nearest from query Point. Corresponding coordinates can be retrieved using getAssociatedCloud()->getPoint(nearestPointIndex)
		\param maxDist distance above which the function doesn't consider points
		\return true if it finds a point p such that ||p-queryPoint||<=maxDist. False otherwise
	**/
	bool findNearestNeighbour(	const PointCoordinateType *queryPoint,
								unsigned &nearestPointIndex,
								ScalarType maxDist);


	//! Optimized version of nearest point search method
	/** Only checks if there is a point p into the tree such that ||p-queryPoint||<=maxDist (see FindNearestNeighbour())
	**/
	bool findPointBelowDistance(const PointCoordinateType *queryPoint,
								ScalarType maxDist);


	//! Searches for the points that lie to a given distance (up to a tolerance) from a query point
	/** \param queryPoint query point coordinates
		\param distance distance wished between the query point and resulting points
		\param tolerance error allowed by the function : each resulting point p is such that distance-tolerance<=||p-queryPoint||<=distance+tolerance
		\param points [out] array of point m_indexes. Each point stored in this array lie to distance (up to tolerance) from queryPoint
		\return the number of matching points
	**/
	unsigned findPointsLyingToDistance(const PointCoordinateType *queryPoint,
										ScalarType distance,
										ScalarType tolerance,
										std::vector<unsigned> &points);

protected:

	//! A KDTre cell struct
	struct KdCell
	{
		KdCell()
			: cuttingDim(0)
			, cuttingCoordinate(0)
			, leSon(nullptr)
			, gSon(nullptr)
			, father(nullptr)
			, startingPointIndex(0)
			, nbPoints(0)
			, boundsMask(0)
		{}
		
		//Warning: put the non aligned members (< 4 bytes) at the end to avoid too much alignment padding!

		//! Inside bounding box max point
		/** The inside bounding box is the smallest cube containing all the points in the cell
		**/
		CCVector3 inbbmax;																			//12 bytes
		//! Inside bounding box min point
		/** The inside bounding box is the smallest cube containing all the points in the cell
		**/
		CCVector3 inbbmin;																			//12 bytes
		//! Outside bounding box min point
		/** The outside bounding box is the bigest cube contained inside the cutting planes that lead to the cell
		**/
		CCVector3 outbbmin;																			//12 bytes
		//! Outside bounding box max point
		/** The outside bounding box is the bigest cube contained inside the cutting planes that lead to the cell
		**/
		CCVector3 outbbmax;																			//12 bytes

		//! Dimension (0, 1 or 2 for x, y or z) which is used to separate the two children
		unsigned cuttingDim;																		//4 bytes
		//! Place where the space is cut into two sub-spaces (children)
		PointCoordinateType cuttingCoordinate;														//4 bytes
		//! Each point p which lie in leSon is such as p[cuttingDim] <= cuttingCoordinate
		struct KdCell* leSon;																		//8 bytes
		//! Each point p which lie in gSon is such as p[cuttingDim] > cuttingCoordinate
		struct KdCell* gSon;																		//8 bytes
		//! To go up in the tree
		struct KdCell* father;																		//8 bytes
		//! Index of the first element that belongs to this cell
		unsigned startingPointIndex;																//4 bytes
		//! Number of elements in this cell
		unsigned nbPoints;																			//4 bytes

		//! Mask to know if the outside box is bounded for a given dimmension
		/** if boundsMask & (2^d) then outbbmin.u[d] is bounded (else the box is opened in outmin.u[d] - i.e. outbbmin.u[d] = -infinite)
			if boundsmask & (2^(3+d)) then outbbmax.u[d] is bounded (else the box is opened in outmax.u[d] - i.e. outbbmax.u[d] = infinite)
		**/
		unsigned char boundsMask;																	//1 byte (+ 3 for alignment)

		//Total																						//92 bytes
	};

	/*** Protected attributes ***/

	//! Tree root
	KdCell* m_root;
	//! Point indexes
	std::vector<unsigned> m_indexes;
	//! Associated cloud
	GenericIndexedCloud* m_associatedCloud;
	//! Number of cells
	unsigned m_cellCount;


	/*** Protected methods ***/

	//! Builds a sub tree
	/** \param first first index
		\param last last index
		\param father father cell
		\param nbBuildCell nbBuildCell
		\param progressCb the client method can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return sub tree (cell)
	**/
	KdCell* buildSubTree(unsigned first, unsigned last, KdCell *father, unsigned &nbBuildCell, GenericProgressCallback *progressCb = nullptr);

	//! Deletes a sub tree
	void deleteSubTree(KdCell *cell);

	//! Computes a cell inside bounding box using the sons ones. The sons bounding boxes have to be up to date considering the points they contain.
	void updateInsideBoundingBox(KdCell* cell);

	//! Computes a cell outside bounding box using the father one and the cutting plane.
	void updateOutsideBoundingBox(KdCell* cell);

	//! Computes the distance between a point and a cell inside bounding box
	/** \param queryPoint queryPoint coordinates
		\param cell the cell from which we want to compute the distance
		\return 0 if the point is inside the cell, the suare of the distance between the two elements if the point is outside
	**/
	ScalarType pointToCellSquareDistance(const PointCoordinateType *queryPoint, KdCell *cell);

	//! Computes the distance between a point and the outside bounding box of the cell in which it lies.
	/** \param queryPoint the query point coordinates
		\param cell the cell containting the query point
		\return the distance between the point and the cell border. If this value is negative, it means that the cell has no border.
	**/
	ScalarType InsidePointToCellDistance(const PointCoordinateType *queryPoint, KdCell *cell);

	//! Computes the distances (min & max) between a point and a cell inside bounding box
	/** \param queryPoint the query point coordinates
		\param cell the cell from which we want to compute the distance
		\param min [out] the minimal distance between the query point and the inside bounding box of cell
		\param max [out] the maximal distance between the query point and the inside bounding box of cell
	**/
	void pointToCellDistances(const PointCoordinateType *queryPoint, KdCell *cell, ScalarType &min, ScalarType &max);

	//! Checks if there is a point in KdCell that is less than minDist-apart from the query point, starting from cell cell
	/** \param queryPoint the query Point coordinates
		\param maxSqrDist square of the maximal distance from querypoint
		\param cell kdtree-cell from which to start the research
		\return -1 if there is no nearer point from querypoint. The nearest point index found in cell if there is one that is at most maxdist apart from querypoint
	**/
	int checkNearerPointInSubTree(const PointCoordinateType *queryPoint, ScalarType& maxSqrDist, KdCell *cell);

	//! Checks if there is a point in KdCell that is less than minDist-apart from the query point, starting from cell cell
	/** Optimiszed version of CheckNearerPointInSubTree since we don't want to find the nearest point, but only check if there is a point that is close enough
		\param queryPoint the query Point coordinates
		\param maxSqrDist square of the maximal distance from querypoint
		\param cell kdtree-cell from which to start the research
		\return true if there is a point in the subtree starting at cell that is close enough from the query point
	**/
	bool checkDistantPointInSubTree(const PointCoordinateType *queryPoint, ScalarType &maxSqrDist, KdCell *cell);

	//! Recursive function which store every point lying to a given distance from the query point
	/** \param queryPoint the query point coordinates
		\param distance the wished distance from the query point
		\param tolerance tolerance for resulting points : p is in resulting array if ||p-queryPoint||<=tolerance
		\param cell current cell to explore (used for recursion)
		\param[out] localArray output of the algorithm. Resulting points m_indexes in associatedCloud are stored in this array
	**/
	void distanceScanTree(	const PointCoordinateType* queryPoint, 
							ScalarType distance, 
							ScalarType tolerance, 
							KdCell* cell, 
							std::vector<unsigned>& localArray);
};

}

#endif //KD_TREE_HEADER
