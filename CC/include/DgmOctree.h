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

#ifndef DGM_OCTREE_HEADER
#define DGM_OCTREE_HEADER

#ifdef _MSC_VER
//To get rid of the really annoying warnings about template class exportation
#pragma warning( disable: 4251 )
#pragma warning( disable: 4530 )
#endif

//enables methods related to Sankaranarayanan et al. nearest neighbors search algorithm
//-> deprecated, as it doesn't prove to be faster than actual implementation
//#define ENABLE_SANKARANARAYANAN_NN_SEARCH

#ifndef _DEBUG
//enables multi-threading handling
#define ENABLE_MT_OCTREE
#endif

#include "GenericOctree.h"
#include "CCMiscTools.h"
#include "CCTypes.h"
#include "CCConst.h"

#include <vector>
#include <assert.h>
#include <string.h>

#ifndef SQRT_3
#define SQRT_3 1.7320508075688772935274463415059
#endif

//DGM: tests in progress
//#define TEST_CELLS_FOR_SPHERICAL_NN

namespace CCLib
{

class GenericProgressCallback;
class ReferenceCloud;

class GenericIndexedCloudPersist;

/*** MACROS ***/

//! Returns the binary shift for a given level of subdivision
/** This binary shift is used to truncate a full cell code in order
	to deduce the cell code for a given level of subdivision.
	\param level the level of subdivision
	\return the binary shift
**/
#define GET_BIT_SHIFT(level) (3*(CCLib::DgmOctree::MAX_OCTREE_LEVEL-level))

//! Returns the octree length (in term of cells) for a given level of subdivision
/** \param level the level of subdivision
	\return 2^level
**/
#define OCTREE_LENGTH(level) (1<<level)

//! The octree structure used throughout the library
/** Implements the GenericOctree interface.
	Corresponds to the octree structure developped during Daniel
	Girardeau-Montaut's PhD (see PhD manuscript, Chapter 4).
**/

#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"
class CC_DLL_API DgmOctree : public GenericOctree
#else
class DgmOctree : public GenericOctree
#endif
{
public:

	/*******************************/
	/**         STURCUTRES        **/
	/*******************************/

	//Max octree subdivision level
	//Number of bits used to code cells position: 3*MAX_OCTREE_LEVEL
	#ifdef OCTREE_CODES_64_BITS
	static const int MAX_OCTREE_LEVEL = 21;
	#else
	static const int MAX_OCTREE_LEVEL = 10;
	#endif

	//! Max octree length at last level of subdivision (number of cells)
	static const int MAX_OCTREE_LENGTH = OCTREE_LENGTH(MAX_OCTREE_LEVEL)-1;

	//! Invalid cell code
	static const OctreeCellCodeType INVALID_CELL_CODE = (((OctreeCellCodeType)1)<<(3*MAX_OCTREE_LEVEL+1));

	//! Octree cell codes container
	typedef std::vector<OctreeCellCodeType> cellCodesContainer;

	//! Octree cell indexes container
	typedef std::vector<unsigned> cellIndexesContainer;

	//! Structure used during nearest neighbour search
	/** Association between a point, its index and its square distance to the query point.
        It has a comparison operator for fast sorting (stdlib).
	**/
	struct PointDescriptor
	{
	    //! Point
		const CCVector3* point;
		//! Point index
		unsigned pointIndex;
		//! Point associated distance value
		DistanceType squareDist;

		//! Default constructor
		PointDescriptor()
			: point(0)
			, pointIndex(0)
			, squareDist(-1.0)
		{
		}

		//! Constructor with point and its index
		PointDescriptor(const CCVector3* P, unsigned index)
			: point(P)
			, pointIndex(index)
			, squareDist(-1.0)
		{
		}

		//! Constructor with point, its index and square distance
		PointDescriptor(const CCVector3* P, unsigned index, DistanceType d2)
			: point(P)
			, pointIndex(index)
			, squareDist(d2)
		{
		}

        //! Comparison operator
        /** \param a point A
            \param b point B
            \return whether the square distance associated to A is smaller than the square distance associated to B
        **/
		static bool distComp(const PointDescriptor& a, const PointDescriptor& b)
		{
			return a.squareDist < b.squareDist;
		}
	};

    //! A set of neighbours
	typedef std::vector<PointDescriptor> NeighboursSet;

	//! Structure used during nearest neighbour search
	struct CellDescriptor
	{
	    //! Cell center
		CCVector3 center;
		//! First point index in associated NeighboursSet
		unsigned index;

		//! Default empty constructor
		CellDescriptor()
		{
		}

		//! Constructor from a point and an index
		CellDescriptor(const PointCoordinateType c[], unsigned i)
			: center(c)
			, index(i)
		{
		}
	};

	//! A set of neighbour cells descriptors
	typedef std::vector<CellDescriptor> NeighbourCellsSet;

	//! Container of in/out parameters for nearest neighbour(s) search
	/** This structure is generic and can be used in multiple cases.
		It is particularilly useful when searching nearest neighbours around points
		that lie in the same octree cell. In this case, serveral informations about
		this cell should be given to the search aglorithm through this structure, but only
		once,before the first seach. Then the search algorithm can be called multiple times,
		and only few informations need to be updated (the query point, etc.).
	**/
	struct NearestNeighboursSearchStruct
	{
		/*** Information to set before search ***/

		//! Query point
		/** Should be updated each time.
		**/
		CCVector3 queryPoint;
		//! Level of subdivision of the octree at which to start the search
		/** Should be set once and for all.
		**/
		uchar level;
		//! Minimal number of neighbours to find
		/** used only during multiple neighbours search (see findNearestNeighborsStartingFromCell).
			This is only indicative and not guaranteed.
		**/
		unsigned minNumberOfNeighbors;
		//! Position in the octree of the cell including the query point
		/** The position is expressed for the level of subdivision at which the search will
			be processed. Use see DgmOctree::getCellPos to determine this position.
			This information should only be updated if the cell changes.
		**/
		int cellPos[3];
		//! Coordinates of the center of the cell including the query point
		/** Use DgmOctree::computeCellCenter to determine these coordinates.
			This information should only be updated if the cell changes.
		**/
		PointCoordinateType cellCenter[3];
		//! Truncated code of the cell including the query point
		/** Use DgmOctree::generateTruncatedCellCode to determine this. If the cell
			doesn't exist in the octree, the field "alreadyVisitedNeighbourhoodSize"
			can be set directly to 1 instead of 0
		**/
		OctreeCellCodeType truncatedCellCode;

		//! Maximum neihgbours distance
		/** The NN search process will stop if it reaches this radius even if it
			hasn't find any neighbour (acceleration). To disable this behavior,
			set the maxSearchSquareDist to -1).
		**/
		DistanceType maxSearchSquareDist;

		/*** Information to set to 0 before search ***/

		//! List of indexes of the cells that have benn already visited by the algorithm
		/** This field is updated by the search algorithm. It should only be emptied
			if the cell that includes the query points change. Only used by the
			"unique nearest point" search algorithm.
		**/
		cellIndexesContainer minimalCellsSetToVisit;

		//! All the points that belong to the cubical neighbourhood of the current cell
		/** This structure is only used by the "multiple nearest neighbours" search algorithms.
			The nearest points (relatively to the query point) are stored at the begining of
			the vector. They are associated to their square distance to the query point.
		**/
		NeighboursSet pointsInNeighbourhood;

		//! Size of the cell neighbourhood that has been already visited by the algorithm
		/** This field is updated by the search algorithm. It should only be reset to 0
			when the cell that includes the query point changes. A value of 0 means that
			no cell has been visited yet, 1 means that only the cell that includes the
			query point has been visited, 2 means that this cell and its 27 neighbourhing
			cells have been visited, etc.
		**/
		int alreadyVisitedNeighbourhoodSize;

		/*** Result ***/

		//! The nearest point
		/** This field is only used by the "unique nearest neighbour" search algorithm
			(see DgmOctree::findTheNearestNeighborStartingFromCell).
		**/
		unsigned theNearestPointIndex;

		//! Default constructor
		NearestNeighboursSearchStruct()
			: queryPoint(0.0)
			, level(1)
			, minNumberOfNeighbors(1)
			, truncatedCellCode(INVALID_CELL_CODE)
			, maxSearchSquareDist(-1.0)
			, alreadyVisitedNeighbourhoodSize(0)
			, theNearestPointIndex(0)
		{
			memset(cellPos,0,sizeof(int)*3);
			memset(cellCenter,0,sizeof(PointCoordinateType)*3);
		}
	};

	struct NearestNeighboursSphericalSearchStruct : public NearestNeighboursSearchStruct
	{
#ifdef TEST_CELLS_FOR_SPHERICAL_NN
		//! All the points that belong to the spherical neighbourhood of the current cell
		NeighboursSet pointsInSphericalNeighbourhood;

		//! Meta data describing cells neighbourhood (associated to pointsInNeighbourhoodUnsorted)
		NeighbourCellsSet cellsInNeighbourhood;

		//! max SQUARE distance from query point to cell center (to be sure of total inclusion)
		PointCoordinateType maxInD2;

		//! min SQUARE distance from query point to cell center (to be sure of total exclusion)
		PointCoordinateType minOutD2;
#endif
		//! Wheter pointsInSphericalNeighbourhood is ready or not
		bool ready;

		//! Updates maxD2 and minD2 with search radius and cellSize
		void prepare(PointCoordinateType radius, PointCoordinateType cellSize)
		{
#ifdef TEST_CELLS_FOR_SPHERICAL_NN
			PointCoordinateType cellDiag = cellSize * (PointCoordinateType)(SQRT_3/2.0);
			minOutD2 = (radius+cellDiag);
			minOutD2 *= minOutD2;
			maxInD2 = (radius-cellDiag);
			if (maxInD2<(PointCoordinateType)0.0)
				maxInD2 = (PointCoordinateType)0.0;
			else
				maxInD2 *= maxInD2;
#endif
		}

		NearestNeighboursSphericalSearchStruct()
			: NearestNeighboursSearchStruct()
			, ready(false)
#ifdef TEST_CELLS_FOR_SPHERICAL_NN
			, maxInD2(0.0)
			, minOutD2(FLOAT_MAX)
#endif
		{
		}
	};

	//! Association between an index and the code of an octree cell
	/** Index could be the index of a point, in which case the code
		would correspond to the octree cell where the point lies.
	**/
	struct indexAndCode
	{
        //! index
		unsigned theIndex;
	    //! cell code
		OctreeCellCodeType theCode;

		//! Default constructor
		indexAndCode()
			: theIndex(0)
			, theCode(0)
		{
		}

		//! Constructor from an index and a code
		indexAndCode(unsigned index, OctreeCellCodeType code)
			: theIndex(index)
			, theCode(code)
		{
		}

		//! Copy constructor
		indexAndCode(const indexAndCode& ic)
			: theIndex(ic.theIndex)
			, theCode(ic.theCode)
		{
		}

		//! Code-based comparison operator
		/** \param a first indexAndCode structure
			\param b second indexAndCode structure
			\return whether the code of 'a' is smaller than the code of 'b'
		**/
		static bool codeComp(const indexAndCode& a, const indexAndCode& b) throw()
		{
			return a.theCode < b.theCode;
		}

		//! Index-based comparison operator
		/** \param a first indexAndCode structure
			\param b second indexAndCode structure
			\return whether the index of 'a' is smaller than the index of 'b'
		**/
		static bool indexComp(const indexAndCode& a, const indexAndCode& b) throw()
		{
			return a.theIndex < b.theIndex;
		}

	};

	//! Container of 'indexAndCode' structures
	typedef std::vector<indexAndCode> cellsContainer;

    //! Octree cell descriptor
	struct octreeCell
	{
	    //! Octree to which the cell belongs
	    const DgmOctree* parentOctree;
	    //! Cell level of subdivision
	    uchar level;
	    //! Truncated cell code
	    OctreeCellCodeType truncatedCode;
	    //! Cell index in octree structure (see m_thePointsAndTheirCellCodes)
	    unsigned index;
	    //! Set of points lying inside this cell
        ReferenceCloud* points;

        //! Default constructor
        octreeCell(DgmOctree* parentOctree);

        //! Default destructor
        virtual ~octreeCell();
	};

	//! Generic form of a function that can be applied automatically to all cells of the octree
	/** See DgmOctree::executeFunctionForAllCellsAtLevel and DgmOctree::executeFunctionForAllCellsAtStartingLevel.
		The parameters of such a function are:
		- (octreeCell) cell descriptor
		- (void**) table of user parameters for the fonction (maybe void)
		- return success
	**/
	typedef bool (*octreeCellFunc)(const octreeCell& cell, void**);

	/******************************/
	/**          METHODS         **/
	/******************************/

	//! DgmOctree constructor
	/** \param aCloud the cloud to construct the octree on
	**/
	DgmOctree(GenericIndexedCloudPersist* aCloud);

	//! DgmOctree destructor
	virtual ~DgmOctree();

	//! Clears the octree
	virtual void clear();

	//! Builds the structure
	/** Octree 3D limits are determined automatically.
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return the number of points projected in the octree
	**/
	int build(GenericProgressCallback* progressCb=0);

	//! Builds the structure with constraints
	/** Octree spatial limits must be specified. Also, if specified, points falling outside
		the "pointsFilter" limits won't be projected into the octree structure. Otherwise, all
		points will be taken into account. Octree 3D limits in space should be cubical.
		\param octreeMin the lower limits for the octree cells along X, Y and Z
		\param octreeMax the upper limits for the octree cells along X, Y and Z
		\param pointsMinFilter the lower limits for the projected points along X, Y and Z (is specified)
		\param pointsMaxFilter the upper limits for the projected points along X, Y and Z (is specified)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return the number of points projected in the octree
	**/
	int build(const CCVector3& octreeMin, const CCVector3& octreeMax, const CCVector3* pointsMinFilter=0, const CCVector3* pointsMaxFilter=0, GenericProgressCallback* progressCb=0);

	/**** GETTERS ****/

	//! Returns the number of points projected into the octree
	/** \return the number of projected points
	**/
	inline unsigned getNumberOfProjectedPoints() const {return m_numberOfProjectedPoints;};

	//! Returns the lower boundaries of the octree
	/** \return the lower coordinates along X,Y and Z
	**/
	inline const CCVector3& getOctreeMins() const {return m_dimMin;}

	//! Returns the higher boundaries of the octree
	/** \return the higher coordinates along X,Y and Z
	**/
	inline const CCVector3& getOctreeMaxs() const {return m_dimMax;}

	//! Returns the octree bounding box
	/**	Method to request the octree bounding box limits
		\param Mins a 3 elements array to store the lower BB limits (Xmin,Ymin,Zmin)
		\param Maxs a 3 elements array to store the higher BB limits (Xmax,Ymax,Zmax)
	**/
	void getBoundingBox(PointCoordinateType Mins[], PointCoordinateType Maxs[]) const;

	//! Returns the lowest cell positions in the octree along all dimensions and for a given level of subdivision
	/** For example, at a level	n, the octree length is 2^n cells along each
		dimension. The lowest cell position along each dimension will be expressed
		between 0 and 2^n-1.
		\param level the level of subdivision
		\return the lowest cell position along X,Y and Z for a given level of subdivision
	**/
	inline const int* getMinFillIndexes(uchar level) const {return m_fillIndexes+6*level;}

	//! Returns the highest cell positions in the octree along all dimensions and for a given level of subdivision
	/** For example, at a level	n, the octree length is 2^n cells along each
		dimension. The highest cell position along each dimension will be expressed
		between 0 and 2^n-1.
		\param level the level of subdivision
		\return the highest cell position along X,Y and Z for a given level of subdivision
	**/
	inline const int* getMaxFillIndexes(uchar level) const {return m_fillIndexes+6*level+3;}

	//! Returns the octree cells length for a given level of subdivision
	/** As the octree is cubical, cells are cubical.
		\param level the level of subdivision (up to MAX_OCTREE_LEVEL+1 for convenience)
		\return the cell size
	**/
	inline const PointCoordinateType& getCellSize(uchar level) const {return m_cellSize[level];}

    //! Returns distance form a cell to the filled octree borders in all directions.
	/** WARNING: distance values may be negative! (if cell is outside
		\param cellPos cell position
        \param level level at which octree grid is considered
        \param cellDists output
    **/
    void getCellDistanceFromBorders(const int* cellPos,
                                    uchar level,
                                    int* cellDists) const;

    //! Returns distance from cell center to cell neighbourhood INSIDE filled octree
	/** WARNING: if cell neighbourhood is totally outside filled octree,
		the method returns false and cellDists is invalid.
		\param cellPos center cell position
        \param level level at which octree grid is considered
        \param neighbourhoodLength cell neighbourhood "radius"
        \param cellDists output
		\return whether cell is inbounds (at least partially) or outbounds
    **/
    bool getCellDistanceFromBorders(const int* cellPos,
                                    uchar level,
                                    int neighbourhoodLength,
                                    int* cellDists) const;

	//! Returns the points lying in a specific cell
	/** Each cell at a given level of subdivision can be recognized by the index
		in the DgmOctree structure of the first point that lies inside it. By
		construction, we are assured that every point lying in the same cell for
		a given level of subdivision are next to each others in the octree
		structure (which is the vector "m_thePointsAndTheirCellCodes" in practical).
		This is the quickest way to access the points inside a given cell (but its
		kind of hard to know directly what is the index of a given cell ;)
		\param cloud ReferenceCloud to store the points lying inside the cell
		\param cellIndex the cell index
		\param level the level of subdivision
	**/
	void getPointsInCellByCellIndex(ReferenceCloud* cloud, unsigned cellIndex, uchar level) const;

	//! Returns the points lying in a specific cell
	/** In this case, the cell is recognized by its "code" which is unique. However,
		one must be sure that the cell does "exist" in the octree (e.g. there is
		indeed points inside this cell and at least one of them has been projected
		into the octree). Otherwise the program may crash or the method may return
		wrong data.
		\param cellCode the cell code
		\param level the level of subdivision
		\param isCodeTruncated specifies if the code is given in a truncated form or not
		\return the set of points lying in the cell (references, no duplication)
	**/
	inline ReferenceCloud* getPointsInCell(OctreeCellCodeType cellCode, uchar level, bool isCodeTruncated=false) const
	{
		unsigned cellIndex = getCellIndex(cellCode,GET_BIT_SHIFT(level),isCodeTruncated);
		assert(cellIndex<m_numberOfProjectedPoints);
		getPointsInCellByCellIndex(m_dumpCloud,cellIndex,level);

		return m_dumpCloud;
	}

	//! Returns the points lying in multiple cells
	/** Cells are recognized here by their unique "code". They should be sorted
		along by codes, with an ascendant order. See DgmOctree::getPointsInCellByCellIndex
		for more information.
		\param cellCodes the cells codes
		\param level the level of subdivision
		\param areCodesTruncated specifies if the codes are given in a truncated form or not
		\return the set of points lying in the cell (references, no duplication)
	**/
	ReferenceCloud* getPointsInCellsWithSortedCellCodes(cellCodesContainer& cellCodes, uchar level, bool areCodesTruncated=false) const;

	/**** NEIGHBOURHOOD SEARCH ****/

	//! Finds the nearest neighbours around a query point
	/** This is the simplest form of the nearest neighbour search algorithm.
		It should only be used for unique/few requests as it is not optimized
		for repetitive search around points lying in the same octree cell (see
		DgmOctree::findNearestNeighborsStartingFromCell for example). Moreover,
		distances between each neighbour and the query aren't stored in this
		version of the algorithm.
		\param _queryPoint the query point
		\param Yk the nearest neighbours
		\param maxNumberOfNeighbors the maximal number of points to find
		\param level the sudivision level of the octree at which to perform the search
		\param maxSquareDist the square distance between the farthest "nearest neighbour" and the query point
		\param maxSearchDist the maximum search distance (ignored if -1)
		\return the number of neighbours found
	**/
	unsigned findPointNeighbourhood(const CCVector3* _queryPoint,
							  ReferenceCloud* Yk,
							  unsigned maxNumberOfNeighbors,
							  uchar level,
							  DistanceType &maxSquareDist,
							  DistanceType maxSearchDist=-1.0) const;

	//! Advanced form of the nearest neighbour search algorithm (unique neighbour)
	/** This version is optimized for a unique nearest-neighbour search.
		See DgmOctree::NearestNeighboursSearchStruct for more details.
		WARNING: if 'getOnlyPointsWithPositiveDist' is true, be sure to activate an OUTPUT
		scalar field on the associated cloud - it will be used to test for 'postitive distances'
		\param nNSS NN search parameters
		\param getOnlyPointsWithPositiveDist select only nearest neighbours with positive distances
		\return the square distance between the query point and its nearest neighbour (or -1 if none was found)
	**/
	DistanceType findTheNearestNeighborStartingFromCell(NearestNeighboursSearchStruct &nNSS,
												bool getOnlyPointsWithPositiveDist) const;

	//! Advanced form of the nearest neighbours search algorithm (multiple neighbours)
	/** This version is optimized for a multiple nearest neighbours search
		that is applied around several query points included in the same octree
		cell. See DgmOctree::NearestNeighboursSearchStruct for more details.
		WARNING: if 'getOnlyPointsWithPositiveDist' is true, be sure to activate an OUTPUT
		scalar field on the associated cloud - it will be used to test for 'postitive distances'
		\param nNSS NN search parameters
		\param bypassFirstCell indicates if the first cell (the one that includes the query point) should be visited or not
		\param getOnlyPointsWithPositiveDist select only nearest neighbours with positive distances
		\return the number of neighbours found
	**/
	unsigned findNearestNeighborsStartingFromCell(NearestNeighboursSearchStruct &nNSS,
                                                    bool bypassFirstCell=false,
                                                    bool getOnlyPointsWithPositiveDist=false) const;

	//! Advanced form of the nearest neighbours search algorithm (in a sphere)
	/** This version is optimized for a spatially bounded search instead of
		a search bounded by a number of neighbours.
		\param nNSS a pack of parameters
		\param radius the sphere radius
		\param sortValues specifies if the neighbours needs to be sorted by their distance to the query point or not
		\return the number of neighbours found
	**/
	int findNeighborsInASphereStartingFromCell(NearestNeighboursSphericalSearchStruct &nNSS,
                                                PointCoordinateType radius,
                                                bool sortValues=true) const;

	//DGM TODO: doc
	int getPointsInSphericalNeighbourhood(const CCVector3& sphereCenter, PointCoordinateType radius, NeighboursSet& neighbours) const;

	/***** CELLS POSITION HANDLING *****/

	//! Generates the truncated cell code of a cell given its position at a given level of subdivision
	/** For a given level of subdivision (lets call it N), the cell position
		can be expressed as 3 integer coordinates between 0 and 2^N-1 (the
		number of cells along each dimension). This method computes the
		corresponding cell code, truncated at the level N (meaning that it
		is only valid for the Nth level, not for other levels).
		\param pos the cell position
		\param level the level of subdivision
		\return the truncated cell code
	**/
	OctreeCellCodeType generateTruncatedCellCode(const int pos[],uchar level) const;

#ifndef OCTREE_CODES_64_BITS
	//! Short version of generateTruncatedCellCode
	OctreeCellCodeType generateTruncatedCellCode(const short pos[],uchar level) const;
#endif

	//! Returns the position FOR THE DEEPEST LEVEL OF SUBDIVISION of the cell that includes a given point
	/** The cell coordinates can be negative or greater than 2^MAX_OCTREE_LEVEL-1
		as the point can lie outside the octree bounding-box.
		\param thePoint the query point
		\param cellPos the computed position
	**/
	inline void getTheCellPosWhichIncludesThePoint(const CCVector3* thePoint, int cellPos[]) const
	{
		const PointCoordinateType& cs = getCellSize(MAX_OCTREE_LEVEL);
		//DGM: if we admit that dd>=0, then the 'floor' operator is useless (int cast = truncation)
		cellPos[0] = (int)/*floor*/((thePoint->x - m_dimMin[0])/cs);
		cellPos[1] = (int)/*floor*/((thePoint->y - m_dimMin[1])/cs);
		cellPos[2] = (int)/*floor*/((thePoint->z - m_dimMin[2])/cs);
	}

	//! Returns the position for a given level of subdivision of the cell that includes a given point
	/** The cell coordinates can be negative or greater than 2^N-1  (where N
		is the level of subdivision) as the point can lie outside the octree
		bounding-box.
		\param thePoint the query point
		\param cellPos the computed position
		\param level the level of subdivision
	**/
	void getTheCellPosWhichIncludesThePoint(const CCVector3* thePoint,int cellPos[], uchar level) const;

	//! Returns the position for a given level of subdivision of the cell that includes a given point
	/** The cell coordinates can be negative or greater than 2^N-1  (where N
		is the level of subdivision) as the point can lie outside the octree
		bounding-box. In this version, method indicates if the query point
		is inside ("inbounds") or outside the octree bounding-box.
		\param thePoint the query point
		\param cellPos the computed position
		\param level the level of subdivision
		\param inbounds indicates if the query point is inside or outside the octree bounding-box
	**/
	void getTheCellPosWhichIncludesThePoint(const CCVector3* thePoint,int cellPos[], uchar level, bool& inbounds) const;

	//! Returns the cell position for a given level of subdivision of a cell designated by its code
	/** \param code the cell code
		\param level the level of subdivision
		\param pos the computed position
		\param isCodeTruncated indicates if the given code is truncated or not
	**/
	void getCellPos(OctreeCellCodeType code, uchar level, int pos[], bool isCodeTruncated) const;

	//! Returns the cell center for a given level of subdivision of a cell designated by its code
	/** \param code the cell code
		\param level the level of subdivision
		\param center the computed center
		\param isCodeTruncated indicates if the given code is truncated or not
	**/
	void computeCellCenter(OctreeCellCodeType code, uchar level,PointCoordinateType center[], bool isCodeTruncated=false) const;

	//! Returns the cell center for a given level of subdivision of a cell designated by its position
	/** \param cellPos the cell position
		\param level the level of subdivision
		\param center the computed center
	**/
	void computeCellCenter(const int cellPos[], uchar level, PointCoordinateType center[]) const;

#ifndef OCTREE_CODES_64_BITS
	//! Short version of computeCellCenter
	void computeCellCenter(const short cellPos[], uchar level, PointCoordinateType center[]) const;
#endif

	//! Returns the spatial limits of a given cell
	/** \param code the cell code
		\param level the level of subdivision
		\param cellMin the minimum coordinates along each dimension
		\param cellMax the maximum coordinates along each dimension
		\param isCodeTruncated indicates if the given code is truncated or not
	**/
	void computeCellLimits(OctreeCellCodeType code, uchar level, PointCoordinateType cellMin[], PointCoordinateType cellMax[], bool isCodeTruncated=false) const;

	/**** OCTREE DIAGNOSIS ****/

	//! Determines the best level of subdivision of the octree at which to apply the nearest neighbours search algorithm (inside a sphere) depending on the sphere radius
	/** \param radius the sphere radius
		\return the 'best' level
	**/
	uchar findBestLevelForAGivenNeighbourhoodSizeExtraction(float radius) const;

	//! Determines the best level of subdivision of the octree at which to apply a cloud-2-cloud distance computation algorithm
	/** The octree instance on which is "applied" this method should be the compared cloud's one.
		"theOtherOctree" should be the reference cloud's octree.
		\param theOtherOctree the octree of the other cloud
		\return the 'best' level
	**/
	uchar findBestLevelForComparisonWithOctree(const DgmOctree* theOtherOctree) const;

	//! Determines the best subdivision level of the octree to assure a mean number of points per cell
	/** \param indicativeNumberOfPointsPerCell 'desired' number of points per cell
		\return the 'best' level
	**/
	uchar findBestLevelForAGivenPopulationPerCell(unsigned indicativeNumberOfPointsPerCell) const;

	//! Determines the best subdivision level of the octree to match a given number of cells
	/** \param indicativeNumberOfCells 'desired' number of cells
		\return the 'best' level
	**/
	uchar findBestLevelForAGivenCellNumber(unsigned indicativeNumberOfCells) const;

	//! Returns the ith cell code
	inline const OctreeCellCodeType& getCellCode(unsigned index) const {return m_thePointsAndTheirCellCodes[index].theCode;};

	//! Returns the list of codes corresponding to the octree cells for a given level of subdivision
	/** Only the non empty cells are represented in the octree structure.
		\param level the level of subdivision
		\param vec the list of codes
		\param truncatedCodes indicates if the resulting codes should be truncated or not
	**/
	void getCellCodes(uchar level, cellCodesContainer& vec, bool truncatedCodes=false) const;

	//! Returns the list of indexes corresponding to the octree cells for a given level of subdivision
	/** Only the non empty cells are represented in the octree structure.
		Cell indexes are expressed relatively to the DgmOctree structure. They correspond
		to the indexes of the first points of each cell.
		\param level the level of subdivision
		\param vec the list of indexes
	**/
	void getCellIndexes(uchar level, cellIndexesContainer& vec) const;

	//! Returns the list of indexes and codes corresponding to the octree cells for a given level of subdivision
	/** Only the non empty cells are represented in the octree structure.
		Cell indexes are expressed relatively to the DgmOctree structure. They correspond
		to the indexes of the first points of each cell.
		\param level the level of subdivision
		\param vec the list of codes & indexes
		\param truncatedCodes indicates if the resulting codes should be truncated or not
	**/
	void getCellCodesAndIndexes(uchar level, cellsContainer& vec, bool truncatedCodes=false) const;


	//! Returns the cells that differ between two octrees (for a same implicit level of subdivision)
	/** Warning: the two octrees should have been computed with the same bounding-box.
		\param codesA the cell codes of the first octree for the implicit level
		\param codesB the cell codes of the second octree for the implicit level
		\param diffA the cells of the first octree that are not in the second octree
		\param diffB the cells of the second octree that are not in the first octree
	**/
	void diff(const cellCodesContainer& codesA, const cellCodesContainer& codesB, cellCodesContainer& diffA, cellCodesContainer& diffB) const;

	//! Returns the differences (in terms of number of cells) between two octrees for a given level of subdivision
	/** Warning: the two octrees should have been computed with the same bounding-box.
		\param octreeLevel the octree level
		\param codesA the cell codes (and point index) of the first octree
		\param codesB the cell codes (and point index) of the second octree
		\param diffA the number of cells of the first octree that are not in the second octree
		\param diffB the number of cells of the second octree that are not in the first octree
		\param cellsA the number of cells of the first octree for the given number of subdivision
		\param cellsB the number of cells of the second octree for the given number of subdivision
	**/
	void diff(uchar octreeLevel, const cellsContainer &codesA, const cellsContainer &codesB, int &diffA, int &diffB, int &cellsA, int &cellsB) const;

	//! Returns the number of cells for a given level of subdivision
	inline const unsigned& getCellNumber(uchar level) const {assert(level<=MAX_OCTREE_LEVEL);return m_cellCount[level];};

	//! Computes mean octree density (point/cell) at a given level of subdivision
	/** \param level the level of subdivision
        \return mean density (point/cell)
    **/
	double computeMeanOctreeDensity(uchar level) const;

	//! Computes the minimal distance between a point and the borders (faces) of the cell (cube) in which it is included
	/** \param queryPoint the point
		\param cs the cell size (as cells are cubical, it's the same along every dimension)
		\param cellCenter the cell center
		\return the minimal distance
	**/
	static inline PointCoordinateType ComputeMinDistanceToCellBorder(const CCVector3* queryPoint, PointCoordinateType cs, const PointCoordinateType* cellCenter)
	{
		PointCoordinateType d1 = fabs(cellCenter[0]-queryPoint->x);
		PointCoordinateType d2 = fabs(cellCenter[1]-queryPoint->y);
		if (d2>d1)
			d1=d2;
		d2 = fabs(cellCenter[2]-queryPoint->z);
		return cs*0.5f - (d2>d1 ? d2 : d1);
	}

	/**** ADVANCED METHODS ****/

	//! Computes the connected components (considering the octree cells only) for a given level of subidivision (partial)
	/** The octree is seen as a regular 3D grid, and each cell of this grid is either set to 0
		(if no points lies in it) or to 1 (if some points lie in it, e.g. if it is indeed a
		cell of this octree). This version of the algorithm can be applied by considering only
		a specified list of octree cells (ignoring the others).
		\param cellCodes the cell codes to consider for the CC computation
		\param level the level of subidivision at which to perform the algorithm
		\param sixConnexity indicates if the CC's 3D connexity should be 6 (26 otherwise)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return error code:
			- '+0' = OK
			- '-1' = no cells (input)
			- '-2' = not enough memory
			- '-3' = no CC found
	**/
	int extractCCs(const cellCodesContainer& cellCodes, uchar level, bool sixConnexity, GenericProgressCallback* progressCb=0) const;

	//! Computes the connected components (considering the octree cells only) for a given level of subidivision (complete)
	/** The octree is seen as a regular 3D grid, and each cell of this grid is either set to 0
		(if no points lies in it) or to 1 (if some points lie in it, e.g. if it is indeed a
		cell of this octree). This version of the algorithm is directly applied on the whole
		octree.
		\param level the level of subidivision at which to perform the algorithm
		\param sixConnexity indicates if the CC's 3D connexity should be 6 (26 otherwise)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return error code:
			- '+0' = OK
			- '-1' = no cells (input)
			- '-2' = not enough memory
			- '-3' = no CC found
	**/
	int extractCCs(uchar level, bool sixConnexity, GenericProgressCallback* progressCb=0) const;

	/**** OCTREE VISITOR ****/

	//! Method to apply automatically a specific function to each cell of the octree
	/** The function to apply should be of the form DgmOctree::octreeCellFunc. In this case
		the octree cells are scanned one by one at the same level of subdivision, but the
		scan can also be sometimes done (inside the cell) at deeper levels, in order to limit
		the number of points in a cell. Thanks to this, the function is applied on a limited
		number of points, avoiding great loss of performances. The only limitation is when the
		level of subdivision is deepest level. In this case no more splitting is possible.
		\param startingLevel the initial level of subdivision
		\param func the function to apply
		\param additionalParameters the function parameters
		\param minNumberOfPointsPerCell	minimal number of points inside a cell (indicative)
		\param maxNumberOfPointsPerCell maximum number of points inside a cell (indicative)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param functionTitle function title
		\return the number of processed cells (or 0 is something went wrong)
	**/
	unsigned executeFunctionForAllCellsAtStartingLevel(uchar startingLevel,
                                                        octreeCellFunc func,
                                                        void** additionalParameters,
														unsigned minNumberOfPointsPerCell,
                                                        unsigned maxNumberOfPointsPerCell,
                                                        GenericProgressCallback* progressCb=0,
                                                        const char* functionTitle=0);

	//! Method to apply automatically a specific function to each cell of the octree
	/** The function to apply should be of the form DgmOctree::octreeCellFunc. In this case
		the octree cells are scanned one by one at the same level of subdivision.
		\param level the level of subdivision
		\param func the function to apply
		\param additionalParameters the function parameters
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param functionTitle function title
		\return the number of processed cells (or 0 is something went wrong)
	**/
	unsigned executeFunctionForAllCellsAtLevel(uchar level,
                                                octreeCellFunc func,
                                                void** additionalParameters,
                                                GenericProgressCallback* progressCb=0,
                                                const char* functionTitle=0);

#ifdef ENABLE_MT_OCTREE
	//! Multi-threaded version of executeFunctionForAllCellsAtLevel
	/** Based on QtConcurrent::map system. Dispacthes automatically
		computation on as much cores on the system.
		\return the number of processed cells (or 0 is something went wrong)
	**/
	unsigned executeFunctionForAllCellsAtLevel_MT(uchar level,
													octreeCellFunc func,
													void** additionalParameters,
													GenericProgressCallback* progressCb=0,
													const char* functionTitle=0);

	//! Multi-threaded version of executeFunctionForAllCellsAtLevel
	/** Based on QtConcurrent::map system. Dispacthes automatically
		computation on as much cores on the system.
		\return the number of processed cells (or 0 is something went wrong)
	**/
	unsigned executeFunctionForAllCellsAtStartingLevel_MT(uchar level,
                                                        octreeCellFunc func,
                                                        void** additionalParameters,
														unsigned minNumberOfPointsPerCell,
                                                        unsigned maxNumberOfPointsPerCell,
                                                        GenericProgressCallback* progressCb=0,
                                                        const char* functionTitle=0);
#endif

	//! Returns the associated cloud
	inline GenericIndexedCloudPersist* associatedCloud() const
	{
	    return m_theAssociatedCloud;
	}

    //! Returns the octree 'structure'
	const cellsContainer& pointsAndTheirCellCodes() const
	{
	    return m_thePointsAndTheirCellCodes;
	}

#ifdef ENABLE_SANKARANARAYANAN_NN_SEARCH

	//! Structure used to describe and sort cells
	/** See "A Fast k-Neighborhood Algorithm for Large Point-Clouds", Sankaranarayanan et al.
	**/
	struct cellDescription
	{
	    //! Truncated cell code
		OctreeCellCodeType truncatedCode;
		//! First point index
		unsigned firstPointIndex;
		//! min distance (number of cells)
		int minDist;
		//! max distance (number of cells)
		int maxDist;

		//! Min distance based comparison operator
        /** \param cd1 first cell descriptor
            \param cd2 second cell descriptor
            \return whether the min. distance of 'cd1' is smaller than the min. distance of 'cd2'
        **/
		static bool minDistComp()(const cellDescription& cd1, const cellDescription& cd2)
		{
			return cd1.minDist > cd2.minDist;
		}

		//! Max distance based comparison operator
        /** \param cd1 first cell descriptor
            \param cd2 second cell descriptor
            \return whether the max. distance of 'cd1' is greater than the max. distance of 'cd2'
        **/
		static bool maxDistComp()(const cellDescription& cd1, const cellDescription& cd2)
		{
			return cd1.maxDist > cd2.maxDist;
		}
	};


	//! Pre-computation step for the "Fast k-Neighborhood Algorithm for Large Point-Clouds" of Sankaranarayanan et. al (deprecated)
	/** This step determines the potentially nearest neighbours for all the points lying inside a given cell, and places them in
		a container (thePoints) that will be used to determine the exact NNs for each point (see DgmOctree::getNNPointsAmong).
		\param truncatedCellCode the truncated cell code where the points lie
		\param cellPos its position
		\param level the octree level at which the computation is done
		\param numberOfNeighbours the number of exact neighbours that will be requested
		\param thePoints an empty container that will be filled with the candidates
	**/
	void prepareCellForNNSearch(OctreeCellCodeType truncatedCellCode, int cellPos[], uchar level, int numberOfNeighbours, NeighboursSet &thePoints) const;

	//! Finds the nearest neighbours (see "Fast k-Neighborhood Algorithm for Large Point-Clouds" of Sankaranarayanan et. al) - DEPRECATED
	/** Once a cell has been "prepared" (see DgmOctree::prepareCellForNNSearch), this function determines the
		exact NNs of a point that lies in this cell.
		\param thePoints an set of candidates (resulting from the preparation step)
		\param queryPoint the query point
		\param numberOfNeighbours the desired number of exact NNs
		\param Zk a structure to store the NNs
		\param alreadySorted optimization: specifies if the container "thePoints" is already sorted (i.e. if it has already been used once by this function for another query point).
	**/
	void getNNPointsAmong(NeighboursSet &thePoints, CCVector3* queryPoint, int numberOfNeighbours, ReferenceCloud* Zk, bool alreadySorted=false) const;
#endif

protected:

	/*******************************/
	/**         STRUCTURES        **/
	/*******************************/

	//! Internal structure used to perform a top-down scan of the octree
	struct octreeTopDownScanStruct
	{
	    //! Cell position inside subdivision level
		unsigned pos;
		//! Number of points in cell
		unsigned elements;
		//! Subdivision level
		uchar level;
	};

	/********************************/
	/**         ATTRIBUTES         **/
	/********************************/

	//! The coded octree structure
	cellsContainer m_thePointsAndTheirCellCodes;

    //! Associated cloud
	GenericIndexedCloudPersist* m_theAssociatedCloud;

	//! Number of points projected in the octree
	unsigned m_numberOfProjectedPoints;

	//! Min coordinates of the octree bounding-box
	CCVector3 m_dimMin;
	//! Max coordinates of the octree bounding-box
	CCVector3 m_dimMax;

	//! Min coordinates of the bounding-box of the set of points projected in the octree
	CCVector3 m_pointsMin;
	//! Max coordinates of the bounding-box of the set of points projected in the octree
	CCVector3 m_pointsMax;

	//! Cell dimensions for all subdivision levels
	PointCoordinateType m_cellSize[MAX_OCTREE_LEVEL+2];
	//! Min and max occupied cells indexes, for all dimensions and every subdivision level
	int m_fillIndexes[(MAX_OCTREE_LEVEL+1)*6];
	//! Number of cells per level of subdivision
	unsigned m_cellCount[MAX_OCTREE_LEVEL+1];
	//! Max cell population per level of subdivision
	unsigned m_maxCellPopulation[MAX_OCTREE_LEVEL+1];
	//! Average cell population per level of subdivision
	double m_averageCellPopulation[MAX_OCTREE_LEVEL+1];
	//! Std. dev. of cell population per level of subdivision
	double m_stdDevCellPopulation[MAX_OCTREE_LEVEL+1];

	//! Dump cloud
	ReferenceCloud* m_dumpCloud;

	/******************************/
	/**         METHODS          **/
	/******************************/

	//! Generic method to build the octree structure
	/** \param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return the number of points projected in the octree
	**/
	int genericBuild(GenericProgressCallback* progressCb=0);

	//! Updates the tables containing octree limits and boundaries
	void updateMinAndMaxTables();

	//! Updates the tables containing the octree cells length for each level of subdivision
	void updateCellSizeTable();

	//! Updates the tables containing the number of octree cells for each level of subdivision
	void updateCellCountTable();

	//! Computes statistics about cells for a given level of subdivision
	/** This method requires some computation, therefore it shouldn't be
		called too often.
		\param level the level of subdivision
	**/
	void computeCellsStatistics(uchar level);

	//! Returns the indexes of the neighbourhing (existing) cells of a given cell
	/** This function is used by the nearest neighbours search algorithms.
		\param cellPos the query cell
		\param neighborCellsIndexes the found neighbourhing cells
		\param neighbourhoodLength the distance (in terms of cells) at which to look for neighbour cells
		\param level the level of subdivision
	**/
	void getNeighborCellsAround(const int cellPos[],
									cellIndexesContainer &neighborCellsIndexes,
									int neighbourhoodLength,
									uchar level) const;

	//! Gets point in the neighbourhing cells of a specific cell
	/** WARNING: if 'getOnlyPointsWithPositiveDist' is true, be sure to activate an OUTPUT
		scalar field on the associated cloud - it will be used to test for 'postitive distances'
		\param nNSS NN search parameters (from which are used: cellPos, pointsInNeighbourCells and level)
		\param neighbourhoodLength the new distance (in terms of cells) at which to look for neighbour cells
	**/
	void getPointsInNeighbourCellsAround(NearestNeighboursSearchStruct &nNSS,
											int neighbourhoodLength) const;

#ifdef TEST_CELLS_FOR_SPHERICAL_NN
	void getPointsInNeighbourCellsAround(NearestNeighboursSphericalSearchStruct &nNSS,
												int minNeighbourhoodLength,
												int maxNeighbourhoodLength) const;
#endif

	//! Gets point associated to positive scalars in the neighbourhing cells of a specific cell
	/** Same version as getPointsInNeighbourCellsAround, but only for neighbours with positive scalars.
		\param nNSS NN search parameters (from which are used: cellPos, pointsInNeighbourCells and level)
		\param neighbourhoodLength the new distance (in terms of cells) at which to look for neighbour cells
	**/
    void getPointsWithPositiveDistanceInNeighbourCellsAround(NearestNeighboursSearchStruct &nNSS,
															int neighbourhoodLength) const;

	//! Returns the index of a given cell represented by its code
	/** The index is found thanks to a binary search. The index of an existing cell
		is between 0 and the number of points projected in the octree minus 1. If
		the cell code cannot be found in the octree structure, then the method returns
		an index equal to the number of projected points (m_numberOfProjectedPoints).
		\param cellCode the octree cell code
		\param bitDec the binary shift corresponding to the level of subdivision (see GET_BIT_SHIFT)
		\param isCodeTruncated indicates if the cell code is truncated or not
		\return the "index" of the cell (or 'm_numberOfProjectedPoints' if none found)
	**/
	unsigned getCellIndex(OctreeCellCodeType cellCode, uchar bitDec, bool isCodeTruncated=false) const;

	//! Returns the index of a given cell represented by its code
	/** Same algorithm as the other "getCellIndex" method, but in an optimized form.
		The binary search can be performed on a sub-part of the DgmOctree structure.
		\param truncatedCellCode the octree truncated cell code
		\param bitDec the binary shift corresponding to the level of subdivision (see GET_BIT_SHIFT)
		\param begin the first index of the sub-list in which to perform the binary search
		\param end the last index of the sub-list in which to perform the binary search
		\return the "index" of the cell (or 'm_numberOfProjectedPoints' if none found)
	**/
	unsigned getCellIndex(OctreeCellCodeType truncatedCellCode, uchar bitDec, unsigned begin, unsigned end) const;

};

}

#endif
