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

#ifndef TRUE_KD_TREE_HEADER
#define TRUE_KD_TREE_HEADER

//Local
#include "DistanceComputationTools.h"
#include "ReferenceCloud.h"

//system
#include <cstdint>

namespace CCLib
{

class GenericIndexedCloudPersist;
class GenericProgressCallback;

//! Proper KD-tree implementation
class CC_CORE_LIB_API TrueKdTree
{
public:

	//Warning: never pass a 'constant initializer' by reference
	static const uint8_t X_DIM = 0;
	static const uint8_t Y_DIM = 1;
	static const uint8_t Z_DIM = 2;
	static const uint8_t NODE_TYPE = 0;
	static const uint8_t LEAF_TYPE = 1;

	//! Tree base node
	class BaseNode
	{
	public:
		explicit BaseNode(uint8_t nodeType) : parent(nullptr), type(nodeType) {}
		virtual ~BaseNode() = default;

		bool isNode() const { return type == NODE_TYPE; }
		bool isLeaf() const { return type == LEAF_TYPE; }

	public:

		//Warning: put the non aligned members (< 4 bytes) at the end to avoid too much alignment padding!
		BaseNode* parent;					//8 bytes

	protected:
		const uint8_t type;					//1 byte (+ 3 for alignment)

		//Total								//12 bytes
	};

	//! Tree node
	class Node : public BaseNode
	{
	public:

		//Warning: put the non aligned members (< 4 bytes) at the end to avoid too much alignment padding!
		PointCoordinateType splitValue;		//4 bytes
		BaseNode* leftChild;				//8 bytes
		BaseNode* rightChild;				//8 bytes
		uint8_t splitDim;					//1 byte (+ 3 bytes for alignment)

		//Total								//24 bytes (+ 12 for father)
		
		Node()
			: BaseNode(NODE_TYPE)
			, splitValue(0)
			, leftChild(nullptr)
			, rightChild(nullptr)
			, splitDim(X_DIM)
		{}

		~Node() override
		{
			delete leftChild;
			delete rightChild;
		}
	};

	//! Tree leaf
	class Leaf : public BaseNode
	{
	public:

		//Warning: put the non aligned members (< 4 bytes) at the end to avoid too much alignment padding!
		ReferenceCloud* points;				// 8 bytes
		PointCoordinateType planeEq[4];		//16 bytes
		ScalarType error;					// 4 bytes
		int userData;						// 4 bytes

		//Total								//32 bytes (+ 12 for father)

		//! Constructor
		/** The Leaf class takes ownership of its associated subset
		**/
		Leaf(ReferenceCloud* set, const PointCoordinateType planeEquation[], ScalarType _error)
			: BaseNode(LEAF_TYPE)
			, points(set)
			, error(_error)
			, userData(0)
		{ 
			memcpy(planeEq, planeEquation, sizeof(PointCoordinateType) * 4);
		}

		~Leaf() override
		{
			delete points;
		}
	};

	//! A vector of leaves
	using LeafVector = std::vector<Leaf *>;

	//! Default constructor
	explicit TrueKdTree(GenericIndexedCloudPersist* cloud);

	//! Destructor
	~TrueKdTree();

	//! Returns the associated cloud
	inline GenericIndexedCloudPersist* associatedCloud() const { return m_associatedCloud; }

	//! Builds KD-tree
	/** \param maxError maximum error per cell (relatively to the best LS plane fit)
		\param errorMeasure error measurement
		\param minPointCountPerCell minimum number of points per cell (can't be smaller than 3)
		\param maxPointCountPerCell maximum number of points per cell (speed-up - ignored if < 6)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
	**/
	bool build(	double maxError,
				DistanceComputationTools::ERROR_MEASURES errorMeasure = DistanceComputationTools::RMS,
				unsigned minPointCountPerCell = 3,
				unsigned maxPointCountPerCell = 0,
				GenericProgressCallback* progressCb = nullptr);

	//! Clears structure
	void clear();

	//! Returns max error threshold used for planarity-based split strategy
	inline double getMaxError() const { return m_maxError; }

	//! Returns max error estimator used for planarity-based split strategy
	inline DistanceComputationTools::ERROR_MEASURES getMaxErrorType() const { return m_errorMeasure; }

	//! Returns all leaf nodes
	bool getLeaves(LeafVector& leaves) const;

protected:

	//! Recursive split process
	BaseNode* split(ReferenceCloud* subset);

	//! Root node
	BaseNode* m_root;

	//! Associated cloud
	GenericIndexedCloudPersist* m_associatedCloud;

	//! Max error for planarity-based split strategy (see m_errorMeasure)
	double m_maxError;

	//! Error measurement
	DistanceComputationTools::ERROR_MEASURES m_errorMeasure;

	//! Min number of points per cell (speed-up)
	/** Can't be < 3
	**/
	unsigned m_minPointCountPerCell;

	//! Max number of points per cell (speed-up)
	/** Ignored if < 6
	**/
	unsigned m_maxPointCountPerCell;
};

} //namespace CCLib

#endif //TRUE_KD_TREE_HEADER
