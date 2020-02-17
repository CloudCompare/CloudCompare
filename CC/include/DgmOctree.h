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

#ifndef DGM_OCTREE_HEADER
#define DGM_OCTREE_HEADER

//Local
#include "GenericOctree.h"
#include "CCPlatform.h"

//system
#include <cassert>
#include <cstring>
#include <vector>

#ifdef CC_ENV_64
//enables 64 bits code octree (can go up to level 21, but take 50% more memory)
#define OCTREE_CODES_64_BITS
#endif

//DGM: tests in progress
//#define TEST_CELLS_FOR_SPHERICAL_NN

namespace CCLib
{

class ReferenceCloud;
class GenericIndexedCloudPersist;
class GenericProgressCallback;
class NormalizedProgress;

//! The octree structure used throughout the library
/** Implements the GenericOctree interface.
	Corresponds to the octree structure developed during Daniel
	Girardeau-Montaut's PhD (see PhD manuscript, Chapter 4).
**/
class CC_CORE_LIB_API DgmOctree : public GenericOctree
{
public:

	//! Returns the binary shift for a given level of subdivision
	/** This binary shift is used to truncate a full cell code in order
		to deduce the cell code for a given level of subdivision.
		\param level the level of subdivision
		\return the binary shift
	**/
	static unsigned char GET_BIT_SHIFT(unsigned char level);

	//! Returns the octree length (in term of cells) for a given level of subdivision
	/** \param level the level of subdivision
		\return 2^level
	**/
	static int OCTREE_LENGTH(int level);

	/*******************************/
	/**         STRUCTURES        **/
	/*******************************/

	//! Max octree subdivision level
	/** Number of bits used to code the cells position: 3*MAX_OCTREE_LEVEL
		\warning Never pass a 'constant initializer' by reference
	**/
#ifdef OCTREE_CODES_64_BITS
	static const int MAX_OCTREE_LEVEL = 21;
#else
	static const int MAX_OCTREE_LEVEL = 10;
#endif

	//! Type of the code of an octree cell
	/** \warning 3 bits per level are required.
		\warning Never pass a 'constant initializer' by reference
	**/
#ifdef OCTREE_CODES_64_BITS
	using CellCode = unsigned long long; //max 21 levels (but twice more memory!)
#else
	using CellCode = unsigned; //max 10 levels
#endif

	//! Max octree length at last level of subdivision (number of cells)
	/** \warning Never pass a 'constant initializer' by reference
	**/
	static const int MAX_OCTREE_LENGTH = (1 << MAX_OCTREE_LEVEL);

	//! Invalid cell code
	/** \warning Never pass a 'constant initializer' by reference
	**/
	static const CellCode INVALID_CELL_CODE = (~static_cast<CellCode>(0));

	//! Octree cell codes container
	using cellCodesContainer = std::vector<CellCode>;

	//! Octree cell indexes container
	using cellIndexesContainer = std::vector<unsigned int>;

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
		double squareDistd;

		//! Default constructor
		PointDescriptor()
			: point(nullptr)
			, pointIndex(0)
			, squareDistd(-1.0)
		{
		}

		//! Constructor with point and its index
		PointDescriptor(const CCVector3* P, unsigned index)
			: point(P)
			, pointIndex(index)
			, squareDistd(-1.0)
		{
		}

		//! Constructor with point, its index and square distance
		PointDescriptor(const CCVector3* P, unsigned index, double d2)
			: point(P)
			, pointIndex(index)
			, squareDistd(d2)
		{
		}

		//! Comparison operator
		/** \param a point A
			\param b point B
			\return whether the square distance associated to A is smaller than the square distance associated to B
		**/
		static bool distComp(const PointDescriptor& a, const PointDescriptor& b)
		{
			return a.squareDistd < b.squareDistd;
		}
	};

	//! A set of neighbours
	using NeighboursSet = std::vector<PointDescriptor>;

	//! Structure used during nearest neighbour search
	struct CellDescriptor
	{
		//! Cell center
		CCVector3 center;
		//! First point index in associated NeighboursSet
		unsigned index;

		//! Default empty constructor
		CellDescriptor() = default;

		//! Constructor from a point and an index
		CellDescriptor(const CCVector3& C, unsigned i)
			: center(C)
			, index(i)
		{}
	};

	//! A set of neighbour cells descriptors
	using NeighbourCellsSet = std::vector<CellDescriptor>;

	//! Container of in/out parameters for nearest neighbour(s) search
	/** This structure is generic and can be used in multiple cases.
		It is particularly useful when searching nearest neighbours around points
		that lie in the same octree cell. In this case, several informations about
		this cell should be given to the search algorithm through this structure, but only
		once,before the first search. Then the search algorithm can be called multiple times,
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
		unsigned char level;
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
		Tuple3i cellPos;
		//! Coordinates of the center of the cell including the query point
		/** Use DgmOctree::computeCellCenter to determine these coordinates.
			This information should only be updated if the cell changes.
		**/
		CCVector3 cellCenter;

		//! Maximum neihgbours distance
		/** The NN search process will stop if it reaches this radius even if it
			hasn't find any neighbour (acceleration). To disable this behavior,
			set the maxSearchSquareDistd to something <= 0).
		**/
		double maxSearchSquareDistd;

		/*** Information to set to 0 before search ***/

		//! List of indexes of the cells that have been already visited by the algorithm
		/** This field is updated by the search algorithm. It should only be emptied
			if the cell that includes the query points change. Only used by the
			"unique nearest point" search algorithm.
		**/
		cellIndexesContainer minimalCellsSetToVisit;

		//! All the points that belong to the cubical neighbourhood of the current cell
		/** This structure is only used by the "multiple nearest neighbours" search algorithms.
			The nearest points (relatively to the query point) are stored at the beginning of
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
			: queryPoint(0,0,0)
			, level(1)
			, minNumberOfNeighbors(1)
			, cellPos(0,0,0)
			, cellCenter(0,0,0)
			, maxSearchSquareDistd(0)
			, alreadyVisitedNeighbourhoodSize(0)
			, theNearestPointIndex(0)
		{}
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
		//! Whether pointsInSphericalNeighbourhood is ready or not
		bool ready;

		//! Updates maxD2 and minD2 with search radius and cellSize
		inline void prepare(PointCoordinateType radius, PointCoordinateType cellSize)
		{
#ifdef TEST_CELLS_FOR_SPHERICAL_NN
			PointCoordinateType cellDiag = cellSize * static_cast<PointCoordinateType>(SQRT_3/2);
			minOutD2 = radius+cellDiag;
			minOutD2 *= minOutD2;
			maxInD2 = radius-cellDiag;
			if (maxInD2 <= 0)
				maxInD2 = 0;
			else
				maxInD2 *= maxInD2;
#endif
		}

		NearestNeighboursSphericalSearchStruct()
			: NearestNeighboursSearchStruct()
			, ready(false)
#ifdef TEST_CELLS_FOR_SPHERICAL_NN
			, maxInD2(0.0)
			, minOutD2(FLT_MAX)
#endif
		{}
	};

	//! Association between an index and the code of an octree cell
	/** Index could be the index of a point, in which case the code
		would correspond to the octree cell where the point lies.
	**/
	struct IndexAndCode
	{
		//! index
		unsigned theIndex;
		//! cell code
		CellCode theCode;

		//! Default constructor
		IndexAndCode()
			: theIndex(0)
			, theCode(0)
		{
		}

		//! Constructor from an index and a code
		IndexAndCode(unsigned index, CellCode code)
			: theIndex(index)
			, theCode(code)
		{
		}

		//! Copy constructor
		IndexAndCode(const IndexAndCode& ic)
			: theIndex(ic.theIndex)
			, theCode(ic.theCode)
		{
		}

		//! Code-based 'less than' comparison operator
		inline bool operator < (const IndexAndCode& iac) const
		{
			return theCode < iac.theCode;
		}

		//! Code-based 'greater than' comparison operator
		inline bool operator > (const IndexAndCode& iac) const
		{
			return theCode > iac.theCode;
		}

		//! Compares two IndexAndCode instances based on their code
		/** \param a first IndexAndCode structure
			\param b second IndexAndCode structure
			\return whether the code of 'a' is smaller than the code of 'b'
		**/
		static bool codeComp(const IndexAndCode& a, const IndexAndCode& b) throw()
		{
			return a.theCode < b.theCode;
		}

		//! Compares two IndexAndCode instances based on their index
		/** \param a first IndexAndCode structure
			\param b second IndexAndCode structure
			\return whether the index of 'a' is smaller than the index of 'b'
		**/
		static bool indexComp(const IndexAndCode& a, const IndexAndCode& b) throw()
		{
			return a.theIndex < b.theIndex;
		}

	};

	//! Container of 'IndexAndCode' structures
	using cellsContainer = std::vector<IndexAndCode>;

	//! Octree cell descriptor
	struct octreeCell
	{
		//Warning: put the non aligned members (< 4 bytes) at the end to avoid too much alignment padding!

		//! Octree to which the cell belongs
		const DgmOctree* parentOctree;												//8 bytes
		//! Truncated cell code
		CellCode truncatedCode;														//8 bytes
		//! Cell index in octree structure (see m_thePointsAndTheirCellCodes)
		unsigned index;																//4 bytes
		//! Set of points lying inside this cell
		ReferenceCloud* points;														//8 bytes
		//! Cell level of subdivision
		unsigned char level;														//1 byte (+ 3 for alignment)

		//Total																		//32 bytes (for 64 bits arch.)

		//! Default constructor
		explicit octreeCell(const DgmOctree* parentOctree);

		//! Default destructor
		virtual ~octreeCell();

	private:
		
		//! Copy constructor
		octreeCell(const octreeCell& cell);
	};

	//! Generic form of a function that can be applied automatically to all cells of the octree
	/** See DgmOctree::executeFunctionForAllCellsAtLevel and 
		DgmOctree::executeFunctionForAllCellsStartingAtLevel.
		The parameters of such a function are:
		- (octreeCell) cell descriptor
		- (void**) table of user parameters for the function (maybe void)
		- (NormalizedProgress*) optional (normalized) progress callback
		- return success
	**/
	using octreeCellFunc = bool (*)(const octreeCell &, void **, NormalizedProgress *);

	/******************************/
	/**          METHODS         **/
	/******************************/

	//! DgmOctree constructor
	/** \param cloud the cloud to construct the octree on
	**/
	explicit DgmOctree(GenericIndexedCloudPersist* cloud);

	//! DgmOctree destructor
	~DgmOctree() override = default;

	//! Clears the octree
	virtual void clear();

	//! Builds the structure
	/** Octree 3D limits are determined automatically.
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return the number of points projected in the octree
	**/
	int build(GenericProgressCallback* progressCb = nullptr);

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
	int build(	const CCVector3& octreeMin,
				const CCVector3& octreeMax,
				const CCVector3* pointsMinFilter = nullptr,
				const CCVector3* pointsMaxFilter = nullptr,
				GenericProgressCallback* progressCb = nullptr);

	/**** GETTERS ****/

	//! Returns the number of points projected into the octree
	/** \return the number of projected points
	**/
	inline unsigned getNumberOfProjectedPoints() const { return m_numberOfProjectedPoints; }

	//! Returns the lower boundaries of the octree
	/** \return the lower coordinates along X,Y and Z
	**/
	inline const CCVector3& getOctreeMins() const { return m_dimMin; }

	//! Returns the higher boundaries of the octree
	/** \return the higher coordinates along X,Y and Z
	**/
	inline const CCVector3& getOctreeMaxs() const { return m_dimMax; }

	//! Returns the octree bounding box
	/**	Method to request the octree bounding box limits
		\param bbMin lower bounding-box limits (Xmin,Ymin,Zmin)
		\param bbMax higher bounding-box limits (Xmax,Ymax,Zmax)
	**/
	void getBoundingBox(CCVector3& bbMin, CCVector3& bbMax) const;

	//! Returns the lowest cell positions in the octree along all dimensions and for a given level of subdivision
	/** For example, at a level	n, the octree length is 2^n cells along each
		dimension. The lowest cell position along each dimension will be expressed
		between 0 and 2^n-1.
		\param level the level of subdivision
		\return the lowest cell position along X,Y and Z for a given level of subdivision
	**/
	inline const int* getMinFillIndexes(unsigned char level) const { return m_fillIndexes + 6*level; }

	//! Returns the highest cell positions in the octree along all dimensions and for a given level of subdivision
	/** For example, at a level	n, the octree length is 2^n cells along each
		dimension. The highest cell position along each dimension will be expressed
		between 0 and 2^n-1.
		\param level the level of subdivision
		\return the highest cell position along X,Y and Z for a given level of subdivision
	**/
	inline const int* getMaxFillIndexes(unsigned char level) const { return m_fillIndexes + 6*level + 3; }

	//! Returns the octree cells length for a given level of subdivision
	/** As the octree is cubical, cells are cubical.
		\param level the level of subdivision (up to MAX_OCTREE_LEVEL+1 for convenience)
		\return the cell size
	**/
	inline const PointCoordinateType& getCellSize(unsigned char level) const { return m_cellSize[level]; }

	//! Returns distance form a cell to the filled octree borders in all directions.
	/** WARNING: distance values may be negative! (if cell is outside
		\param cellPos cell position
		\param level level at which octree grid is considered
		\param cellDists output
	**/
	void getCellDistanceFromBorders(const Tuple3i& cellPos,
									unsigned char level,
									int* cellDists) const;

	//! Returns distance from cell center to cell neighbourhood INSIDE filled octree
	/** WARNING: if cell neighbourhood is totally outside filled octree,
		the method returns false and cellDists is invalid.
		\param cellPos center cell position
		\param level level at which octree grid is considered
		\param neighbourhoodLength cell neighbourhood "radius"
		\param cellDists output
	**/
	void getCellDistanceFromBorders(const Tuple3i& cellPos,
									unsigned char level,
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
		\param clearOutputCloud whether to clear the input cloud prior to inserting the points or not
		\return success
	**/
	bool getPointsInCellByCellIndex(ReferenceCloud* cloud,
									unsigned cellIndex,
									unsigned char level,
									bool clearOutputCloud = true) const;

	//! Returns the points lying in a specific cell
	/** \param cellCode the unique cell code
		\param level the level of subdivision
		\param[out] subset set of points lying in the cell (references, no duplication)
		\param isCodeTruncated specifies if the code is given in a truncated form or not
		\param clearOutputCloud whether to clear or not the output cloud (subest) if no points lie in the specified cell
		\return success
	**/
	bool getPointsInCell(	CellCode cellCode,
							unsigned char level,
							ReferenceCloud* subset,
							bool isCodeTruncated = false,
							bool clearOutputCloud = true) const;

	//! Returns the points lying in multiple cells
	/** Cells are recognized here by their unique "code". They should be sorted
		along by codes, with an ascendant order. See DgmOctree::getPointsInCellByCellIndex
		for more information.
		\param cellCodes the cells codes
		\param level the level of subdivision
		\param[out] subset set of points lying in the cell (references, no duplication)
		\param areCodesTruncated specifies if the codes are given in a truncated form or not
		\return the set of points lying in the cell (references, no duplication)
	**/
	ReferenceCloud* getPointsInCellsWithSortedCellCodes(cellCodesContainer& cellCodes,
														unsigned char level,
														ReferenceCloud* subset,
														bool areCodesTruncated = false) const;

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
		\param level the subdivision level of the octree at which to perform the search
		\param maxSquareDist the square distance between the farthest "nearest neighbour" and the query point
		\param maxSearchDist the maximum search distance (ignored if <= 0)
		\param[out] the final neighborhood (half)size (optional)
		\return the number of neighbours found
	**/
	unsigned findPointNeighbourhood(const CCVector3* _queryPoint,
									ReferenceCloud* Yk,
									unsigned maxNumberOfNeighbors,
									unsigned char level,
									double &maxSquareDist,
									double maxSearchDist = 0,
									int* finalNeighbourhoodSize = nullptr) const;

	//! Advanced form of the nearest neighbour search algorithm (unique neighbour)
	/** This version is optimized for a unique nearest-neighbour search.
		See DgmOctree::NearestNeighboursSearchStruct for more details.
		\param nNSS NN search parameters
		\return the square distance between the query point and its nearest neighbour (or -1 if none was found - i.e. maxSearchDist was reached)
	**/
	double findTheNearestNeighborStartingFromCell(NearestNeighboursSearchStruct &nNSS) const;

	//! Advanced form of the nearest neighbours search algorithm (multiple neighbours)
	/** This version is optimized for a multiple nearest neighbours search
		that is applied around several query points included in the same octree
		cell. See DgmOctree::NearestNeighboursSearchStruct for more details.
		\param nNSS NN search parameters
		\param getOnlyPointsWithValidScalar whether to ignore points having an invalid associated scalar value
		\return the number of neighbours found
	**/
	unsigned findNearestNeighborsStartingFromCell(NearestNeighboursSearchStruct &nNSS,
													bool getOnlyPointsWithValidScalar = false) const;

	//! Advanced form of the nearest neighbours search algorithm (in a sphere)
	/** This version is optimized for a spatially bounded search instead of
		a search bounded by a number of neighbours.
		\warning the number of points in the output buffer (nNSS.pointsInNeighbourhood) may be greater
		than the actual count of closest points inside the sphere! (which is returned by the method).
		Only the 'k' first points are actually inside the sphere (the others are not removed for the sake
		of performance).
		\param nNSS a pack of parameters
		\param radius the sphere radius
		\param sortValues specifies if the neighbours needs to be sorted by their distance to the query point or not
		\return the number of neighbours found
	**/
	int findNeighborsInASphereStartingFromCell(	NearestNeighboursSphericalSearchStruct &nNSS,
												double radius,
												bool sortValues = true) const;

public: //extraction of points inside geometrical volumes (sphere, cylinder, box, etc.)

	//deprecated
	//int getPointsInSphericalNeighbourhood(const CCVector3& sphereCenter, PointCoordinateType radius, NeighboursSet& neighbours) const;

	//! Returns the points falling inside a sphere
	/** Use findBestLevelForAGivenNeighbourhoodSizeExtraction to get the right
		value for 'level' (only once as it only depends on the radius value ;).
		\param sphereCenter center
		\param radius radius
		\param[out] neighbours points falling inside the sphere
		\param level subdivision level at which to apply the extraction process
		\return the number of extracted points
	**/
	int getPointsInSphericalNeighbourhood(	const CCVector3& sphereCenter,
											PointCoordinateType radius,
											NeighboursSet& neighbours,
											unsigned char level) const;

	//! Input/output parameters structure for getPointsInCylindricalNeighbourhood
	struct CylindricalNeighbourhood
	{
		//! Cylinder center
		CCVector3 center;
		//! Cylinder axis (direction)
		CCVector3 dir;
		//! Cylinder radius
		PointCoordinateType radius;
		//! Cylinder (half) length
		PointCoordinateType maxHalfLength;
		//! Neighbour points falling inside the cylinder
		NeighboursSet neighbours;
		//! subdivision level at which to apply the extraction process
		unsigned char level;
		//! Whether to look in both directions or only along the positive direction (i.e. half cylinder)
		bool onlyPositiveDir;

		//! Default constructor
		CylindricalNeighbourhood()
			: center(0,0,0)
			, dir(0,0,1)
			, radius(0)
			, maxHalfLength(0)
			, level(0)
			, onlyPositiveDir(false)
		{}
	};

	//! Returns the points falling inside a cylinder
	/** Use findBestLevelForAGivenNeighbourhoodSizeExtraction to get the right
		value for 'level' (only once as it only depends on the radius value ;).
		\warning the 'squareDistd' field of each neighbour in the NeighboursSet
		structure is in fact the signed distance (not squared) of the point
		relatively to the cylinder's center and projected along its axis.
		\param params input/output parameters structure
		\return the number of extracted points
	**/
	std::size_t getPointsInCylindricalNeighbourhood(CylindricalNeighbourhood& params) const;

	//! Input/output parameters structure for getPointsInCylindricalNeighbourhoodProgressive
	struct ProgressiveCylindricalNeighbourhood : CylindricalNeighbourhood
	{
		//! Current search depth
		PointCoordinateType currentHalfLength;
		//! Vector to store potential candidates for the next pass
		/** Candidates are points close enough to the cylinder's axis but too far
			from its actual center.
		**/
		NeighboursSet potentialCandidates;
		//! Previous search box (min corner)
		Tuple3i prevMinCornerPos;
		//! Previous search box (max corner)
		Tuple3i prevMaxCornerPos;

		ProgressiveCylindricalNeighbourhood()
			: CylindricalNeighbourhood()
			, currentHalfLength(0)
			, prevMinCornerPos(-1,-1,-1)
			, prevMaxCornerPos(0,0,0)
		{}

	};

	//! Same as getPointsInCylindricalNeighbourhood with progressive approach
	/** Can be called multiple times (the 'currentHalfLength' parameter will increase
		each time until 'maxHalfLength' is reached).
	**/
	std::size_t getPointsInCylindricalNeighbourhoodProgressive(ProgressiveCylindricalNeighbourhood& params) const;

	//! Input/output parameters structure for getPointsInBoxNeighbourhood
	struct BoxNeighbourhood
	{
		//! Box center
		CCVector3 center;
		//! Box axes (optional)
		CCVector3* axes;
		//! Box dimensions
		CCVector3 dimensions;
		//! Neighbour points falling inside the box
		NeighboursSet neighbours;
		//! subdivision level at which to apply the extraction process
		unsigned char level;

		//! Default constructor
		BoxNeighbourhood()
			: center(0,0,0)
			, axes(nullptr)
			, dimensions(0,0,0)
			, level(0)
		{}
	};

	//! Returns the points falling inside a box
	/** \warning the 'squareDistd' field of each neighbour in the NeighboursSet
		structure is not used/set
		\return the number of extracted points
	**/
	std::size_t getPointsInBoxNeighbourhood(BoxNeighbourhood& params) const;


public:	/***** CELLS POSITION HANDLING *****/

	//! Generates the truncated cell code of a cell given its position at a given level of subdivision
	/** For a given level of subdivision (lets call it N), the cell position
		can be expressed as 3 integer coordinates between 0 and 2^N-1 (the
		number of cells along each dimension). This method computes the
		corresponding cell code, truncated at the level N (meaning that it
		is only valid for the Nth level, not for other levels).
		\param cellPos the cell position
		\param level the level of subdivision
		\return the truncated cell code
	**/
	static CellCode GenerateTruncatedCellCode(const Tuple3i& cellPos, unsigned char level);

#ifndef OCTREE_CODES_64_BITS
	//! Short version of GenerateTruncatedCellCode
	static CellCode GenerateTruncatedCellCode(const Tuple3s& pos, unsigned char level);
#endif

	//! Returns the position FOR THE DEEPEST LEVEL OF SUBDIVISION of the cell that includes a given point
	/** The cell coordinates can be negative or greater than 2^MAX_OCTREE_LEVEL-1
		as the point can lie outside the octree bounding-box.
		\param thePoint the query point
		\param cellPos the computed position
	**/
	inline void getTheCellPosWhichIncludesThePoint(const CCVector3* thePoint, Tuple3i& cellPos) const
	{
		const PointCoordinateType& cs = getCellSize(MAX_OCTREE_LEVEL);
		//DGM: if we admit that cs >= 0, then the 'floor' operator is useless (int cast = truncation)
		cellPos.x = static_cast<int>(/*floor*/(thePoint->x - m_dimMin.x)/cs);
		cellPos.y = static_cast<int>(/*floor*/(thePoint->y - m_dimMin.y)/cs);
		cellPos.z = static_cast<int>(/*floor*/(thePoint->z - m_dimMin.z)/cs);
	}

	//! Returns the position for a given level of subdivision of the cell that includes a given point
	/** The cell coordinates can be negative or greater than 2^N-1  (where N
		is the level of subdivision) as the point can lie outside the octree
		bounding-box.
		\param thePoint the query point
		\param cellPos the computed position
		\param level the level of subdivision
	**/
	inline void getTheCellPosWhichIncludesThePoint(const CCVector3* thePoint, Tuple3i& cellPos, unsigned char level) const
	{
		assert(level <= MAX_OCTREE_LEVEL);

		getTheCellPosWhichIncludesThePoint(thePoint, cellPos);

		const unsigned char dec = MAX_OCTREE_LEVEL - level;
		cellPos.x >>= dec;
		cellPos.y >>= dec;
		cellPos.z >>= dec;
	}

	//! Returns the position for a given level of subdivision of the cell that includes a given point
	/** The cell coordinates can be negative or greater than 2^N-1  (where N
		is the level of subdivision) as the point can lie outside the octree
		bounding-box. In this version, method indicates if the query point
		is inside ("inbounds") or outside the octree bounding-box.
		\param thePoint the query point
		\param cellPos the computed position
		\param level the level of subdivision
		\param inBounds indicates if the query point is inside or outside the octree bounding-box
	**/
	inline void getTheCellPosWhichIncludesThePoint(const CCVector3* thePoint, Tuple3i& cellPos, unsigned char level, bool& inBounds) const
	{
		assert(level <= MAX_OCTREE_LEVEL);

		getTheCellPosWhichIncludesThePoint(thePoint, cellPos);

		inBounds =	(	cellPos.x >= 0 && cellPos.x < MAX_OCTREE_LENGTH
					 && cellPos.y >= 0 && cellPos.y < MAX_OCTREE_LENGTH
					 && cellPos.z >= 0 && cellPos.z < MAX_OCTREE_LENGTH );

		const unsigned char dec = MAX_OCTREE_LEVEL - level;
		cellPos.x >>= dec;
		cellPos.y >>= dec;
		cellPos.z >>= dec;
	}

	//! Returns the cell position for a given level of subdivision of a cell designated by its code
	/** \param code the cell code
		\param level the level of subdivision
		\param cellPos the computed position
		\param isCodeTruncated indicates if the given code is truncated or not
	**/
	void getCellPos(CellCode code, unsigned char level, Tuple3i& cellPos, bool isCodeTruncated) const;

	//! Returns the cell center for a given level of subdivision of a cell designated by its code
	/** \param code the cell code
		\param level the level of subdivision
		\param center the computed center
		\param isCodeTruncated indicates if the given code is truncated or not
	**/
	inline void computeCellCenter(CellCode code, unsigned char level, CCVector3& center, bool isCodeTruncated = false) const
	{
		Tuple3i cellPos;
		getCellPos(code,level,cellPos,isCodeTruncated);

		return computeCellCenter(cellPos,level,center);
	}

	//! Returns the cell center for a given level of subdivision of a cell designated by its position
	/** \param cellPos the cell position
		\param level the level of subdivision
		\param center the computed center
	**/
	inline void computeCellCenter(const Tuple3i& cellPos, unsigned char level, CCVector3& center) const
	{
		const PointCoordinateType& cs = getCellSize(level);
		center.x = m_dimMin.x + cs * (static_cast<PointCoordinateType>(0.5) + static_cast<PointCoordinateType>(cellPos.x));
		center.y = m_dimMin.y + cs * (static_cast<PointCoordinateType>(0.5) + static_cast<PointCoordinateType>(cellPos.y));
		center.z = m_dimMin.z + cs * (static_cast<PointCoordinateType>(0.5) + static_cast<PointCoordinateType>(cellPos.z));
	}

#ifndef OCTREE_CODES_64_BITS
	//! Short version of computeCellCenter
	inline void computeCellCenter(const Tuple3s& cellPos, unsigned char level, CCVector3& center) const
	{
		const PointCoordinateType& cs = getCellSize(level);
		center.x = m_dimMin.x + cs * (static_cast<PointCoordinateType>(0.5) + static_cast<PointCoordinateType>(cellPos.x));
		center.y = m_dimMin.y + cs * (static_cast<PointCoordinateType>(0.5) + static_cast<PointCoordinateType>(cellPos.y));
		center.z = m_dimMin.z + cs * (static_cast<PointCoordinateType>(0.5) + static_cast<PointCoordinateType>(cellPos.z));
	}
#endif

	//! Returns the spatial limits of a given cell
	/** \param code the cell code
		\param level the level of subdivision
		\param cellMin the minimum coordinates along each dimension
		\param cellMax the maximum coordinates along each dimension
		\param isCodeTruncated indicates if the given code is truncated or not
	**/
	void computeCellLimits(CellCode code, unsigned char level, CCVector3& cellMin, CCVector3& cellMax, bool isCodeTruncated = false) const;

	//! Returns the index of a given cell represented by its code
	/** The index is found thanks to a binary search. The index of an existing cell
		is between 0 and the number of points projected in the octree minus 1. If
		the cell code cannot be found in the octree structure, then the method returns
		an index equal to the number of projected points (m_numberOfProjectedPoints).
		\param truncatedCellCode truncated cell code (i.e. original cell code shifted of 'bitDec' bits)
		\param bitDec binary shift corresponding to the level of subdivision (see GET_BIT_SHIFT)
		\return the index of the cell (or 'm_numberOfProjectedPoints' if none found)
	**/
	unsigned getCellIndex(CellCode truncatedCellCode, unsigned char bitDec) const;

	/**** OCTREE DIAGNOSIS ****/

	//! Determines the best level of subdivision of the octree at which to apply the nearest neighbours search algorithm (inside a sphere) depending on the sphere radius
	/** \param radius the sphere radius
		\return the 'best' level
	**/
	unsigned char findBestLevelForAGivenNeighbourhoodSizeExtraction(PointCoordinateType radius) const;

	//! Determines the best level of subdivision of the octree at which to apply a cloud-2-cloud distance computation algorithm
	/** The octree instance on which is "applied" this method should be the compared cloud's one.
		"theOtherOctree" should be the reference cloud's octree.
		\param theOtherOctree the octree of the other cloud
		\return the 'best' level
	**/
	unsigned char findBestLevelForComparisonWithOctree(const DgmOctree* theOtherOctree) const;

	//! Determines the best subdivision level of the octree that gives the average population per cell closest to the input value
	/** \param indicativeNumberOfPointsPerCell 'desired' average number of points per cell
		\return the 'best' level
	**/
	unsigned char findBestLevelForAGivenPopulationPerCell(unsigned indicativeNumberOfPointsPerCell) const;

	//! Determines the best subdivision level of the octree to match a given number of cells
	/** \param indicativeNumberOfCells 'desired' number of cells
		\return the 'best' level
	**/
	unsigned char findBestLevelForAGivenCellNumber(unsigned indicativeNumberOfCells) const;

	//! Returns the ith cell code
	inline const CellCode& getCellCode(unsigned index) const { return m_thePointsAndTheirCellCodes[index].theCode; }

	//! Returns the list of codes corresponding to the octree cells for a given level of subdivision
	/** Only the non empty cells are represented in the octree structure.
		\param level the level of subdivision
		\param vec the list of codes
		\param truncatedCodes indicates if the resulting codes should be truncated or not
		\return false if an error occurred (e.g. not enough memory)
	**/
	bool getCellCodes(unsigned char level, cellCodesContainer& vec, bool truncatedCodes = false) const;

	//! Returns the list of indexes corresponding to the octree cells for a given level of subdivision
	/** Only the non empty cells are represented in the octree structure.
		Cell indexes are expressed relatively to the DgmOctree structure. They correspond
		to the indexes of the first points of each cell.
		\param level the level of subdivision
		\param vec the list of indexes
		\return false if an error occurred (e.g. not enough memory)
	**/
	bool getCellIndexes(unsigned char level, cellIndexesContainer& vec) const;

	//! Returns the list of indexes and codes corresponding to the octree cells for a given level of subdivision
	/** Only the non empty cells are represented in the octree structure.
		Cell indexes are expressed relatively to the DgmOctree structure. They correspond
		to the indexes of the first points of each cell.
		\param level the level of subdivision
		\param vec the list of codes & indexes
		\param truncatedCodes indicates if the resulting codes should be truncated or not
		\return false if an error occurred (e.g. not enough memory)
	**/
	bool getCellCodesAndIndexes(unsigned char level, cellsContainer& vec, bool truncatedCodes = false) const;


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
		\return false if it could not calculate the differences
	**/
	bool diff(unsigned char octreeLevel, const cellsContainer &codesA, const cellsContainer &codesB, int &diffA, int &diffB, int &cellsA, int &cellsB) const;

	//! Returns the number of cells for a given level of subdivision
	inline const unsigned& getCellNumber(unsigned char level) const
	{
		assert(level <= MAX_OCTREE_LEVEL);
		return m_cellCount[level];
	}

	//! Computes mean octree density (point/cell) at a given level of subdivision
	/** \param level the level of subdivision
		\return mean density (point/cell)
	**/
	double computeMeanOctreeDensity(unsigned char level) const;

	//! Computes the minimal distance between a point and the borders (faces) of the cell (cube) in which it is included
	/** \param queryPoint the point
		\param cs the cell size (as cells are cubical, it's the same along every dimension)
		\param cellCenter the cell center
		\return the minimal distance
	**/
	static inline PointCoordinateType ComputeMinDistanceToCellBorder(const CCVector3& queryPoint, PointCoordinateType cs, const CCVector3& cellCenter)
	{
		PointCoordinateType d1 = std::abs(cellCenter.x - queryPoint.x);
		PointCoordinateType d2 = std::abs(cellCenter.y - queryPoint.y);
		if (d2 > d1)
			d1 = d2;
		
		d2 = std::abs(cellCenter.z - queryPoint.z);
		return cs/2 - (d2 > d1 ? d2 : d1);
	}

	/**** ADVANCED METHODS ****/

	//! Computes the connected components (considering the octree cells only) for a given level of subdivision (partial)
	/** The octree is seen as a regular 3D grid, and each cell of this grid is either set to 0
		(if no points lies in it) or to 1 (if some points lie in it, e.g. if it is indeed a
		cell of this octree). This version of the algorithm can be applied by considering only
		a specified list of octree cells (ignoring the others).
		\param cellCodes the cell codes to consider for the CC computation
		\param level the level of subdivision at which to perform the algorithm
		\param sixConnexity indicates if the CC's 3D connexity should be 6 (26 otherwise)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return error code:
			- '>= 0' = number of components
			- '-1' = no cells (input)
			- '-2' = not enough memory
			- '-3' = no CC found
	**/
	int extractCCs(	const cellCodesContainer& cellCodes,
					unsigned char level,
					bool sixConnexity,
					GenericProgressCallback* progressCb = nullptr) const;

	//! Computes the connected components (considering the octree cells only) for a given level of subdivision (complete)
	/** The octree is seen as a regular 3D grid, and each cell of this grid is either set to 0
		(if no points lies in it) or to 1 (if some points lie in it, e.g. if it is indeed a
		cell of this octree). This version of the algorithm is directly applied on the whole
		octree.
		\param level the level of subdivision at which to perform the algorithm
		\param sixConnexity indicates if the CC's 3D connexity should be 6 (26 otherwise)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return error code:
			- '>= 0' = number of components
			- '-1' = no cells (input)
			- '-2' = not enough memory
			- '-3' = no CC found
	**/
	int extractCCs(	unsigned char level,
					bool sixConnexity,
					GenericProgressCallback* progressCb = nullptr) const;

	/**** OCTREE VISITOR ****/

	//! Method to apply automatically a specific function to each cell of the octree
	/** The function to apply should be of the form DgmOctree::octreeCellFunc. In this case
		the octree cells are scanned one by one at the same level of subdivision, but the
		scan can also be sometimes done (inside the cell) at deeper levels, in order to limit
		the number of points in a cell. Thanks to this, the function is applied on a limited
		number of points, avoiding great loss of performances. The only limitation is when the
		level of subdivision is deepest level. In this case no more splitting is possible.

		Parallel processing is based on QtConcurrent::map system.

		\param startingLevel the initial level of subdivision
		\param func the function to apply
		\param additionalParameters the function parameters
		\param minNumberOfPointsPerCell	minimal number of points inside a cell (indicative)
		\param maxNumberOfPointsPerCell maximum number of points inside a cell (indicative)
		\param multiThread whether to use parallel processing or not
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param functionTitle function title
		\param maxThreadCount the maximum number of threads to use (0 = all). Ignored if 'multiThread' is false.
		\return the number of processed cells (or 0 is something went wrong)
	**/
	unsigned executeFunctionForAllCellsStartingAtLevel(	unsigned char startingLevel,
														octreeCellFunc func,
														void** additionalParameters,
														unsigned minNumberOfPointsPerCell,
														unsigned maxNumberOfPointsPerCell,
														bool multiThread = true,
														GenericProgressCallback* progressCb = nullptr,
														const char* functionTitle = nullptr,
														int maxThreadCount = 0);

	//! Method to apply automatically a specific function to each cell of the octree
	/** The function to apply should be of the form DgmOctree::octreeCellFunc. In this case
		the octree cells are scanned one by one at the same level of subdivision.

		Parallel processing is based on QtConcurrent::map system.

		\param level the level of subdivision
		\param func the function to apply
		\param additionalParameters the function parameters
		\param multiThread whether to use parallel processing or not
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param functionTitle function title
		\param maxThreadCount the maximum number of threads to use (0 = all). Ignored if 'multiThread' is false.
		\return the number of processed cells (or 0 is something went wrong)
	**/
	unsigned executeFunctionForAllCellsAtLevel(	unsigned char level,
												octreeCellFunc func,
												void** additionalParameters,
												bool multiThread = false,
												GenericProgressCallback* progressCb = nullptr,
												const char* functionTitle = nullptr,
												int maxThreadCount = 0);

	//! Ray casting processes
	enum RayCastProcess { RC_NEAREST_POINT, RC_CLOSE_POINTS };

	//! Ray casting algorithm
	bool rayCast(	const CCVector3& rayAxis,
					const CCVector3& rayOrigin,
					double maxRadiusOrFov,
					bool isFOV, //whether the previous parameter is a radius (distance) or a FOV (in radians)
					RayCastProcess process,
					std::vector<PointDescriptor>& output) const;

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

	//! Returns whether multi-threading (parallel) computation is supported or not
	static bool MultiThreadSupport();

protected:

	/*******************************/
	/**         STRUCTURES        **/
	/*******************************/

	//! Internal structure used to perform a top-down scan of the octree
	struct octreeTopDownScanStruct
	{
		//Warning: put the non aligned members (< 4 bytes) at the end to avoid too much alignment padding!

		//! Cell position inside subdivision level
		unsigned pos;									//4 bytes
		//! Number of points in cell
		unsigned elements;								//4 bytes
		//! Subdivision level
		unsigned char level;							//1 byte (+ 3 for alignment)

		//Total											//12 bytes
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
	
	//! Nearest power of 2 less than the number of points (used for binary search)
	unsigned m_nearestPow2;

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

	/******************************/
	/**         METHODS          **/
	/******************************/

	//! Generic method to build the octree structure
	/** \param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\return the number of points projected in the octree
	**/
	int genericBuild(GenericProgressCallback* progressCb = nullptr);

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
	void computeCellsStatistics(unsigned char level);

	//! Returns the indexes of the neighbourhing (existing) cells of a given cell
	/** This function is used by the nearest neighbours search algorithms.
		\param cellPos the query cell
		\param neighborCellsIndexes the found neighbourhing cells
		\param neighbourhoodLength the distance (in terms of cells) at which to look for neighbour cells
		\param level the level of subdivision
	**/
	void getNeighborCellsAround(const Tuple3i& cellPos,
								cellIndexesContainer &neighborCellsIndexes,
								int neighbourhoodLength,
								unsigned char level) const;

	//! Gets point in the neighbourhing cells of a specific cell
	/** \param nNSS NN search parameters (from which are used: cellPos, pointsInNeighbourCells and level)
		\param neighbourhoodLength the new distance (in terms of cells) at which to look for neighbour cells
		\param getOnlyPointsWithValidScalar whether to ignore points having an invalid associated scalar value
	**/
	void getPointsInNeighbourCellsAround(NearestNeighboursSearchStruct &nNSS,
											int neighbourhoodLength,
											bool getOnlyPointsWithValidScalar = false) const;

#ifdef TEST_CELLS_FOR_SPHERICAL_NN
	void getPointsInNeighbourCellsAround(NearestNeighboursSphericalSearchStruct &nNSS,
												int minNeighbourhoodLength,
												int maxNeighbourhoodLength) const;
#endif

	//! Returns the index of a given cell represented by its code
	/** Same algorithm as the other "getCellIndex" method, but in an optimized form.
		The binary search can be performed on a sub-part of the DgmOctree structure.
		\param truncatedCellCode truncated cell code (i.e. original cell code shifted of 'bitDec' bits)
		\param bitDec binary shift corresponding to the level of subdivision (see GET_BIT_SHIFT)
		\param begin first index of the sub-list in which to perform the binary search
		\param end last index of the sub-list in which to perform the binary search
		\return the index of the cell (or 'm_numberOfProjectedPoints' if none found)
	**/
	unsigned getCellIndex(CellCode truncatedCellCode, unsigned char bitDec, unsigned begin, unsigned end) const;
};

}

#endif //DGM_OCTREE_HEADER
