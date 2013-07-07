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

#ifndef TRUE_KD_TREE_HEADER
#define TRUE_KD_TREE_HEADER

//local
#include "CCTypes.h"
#include "CCConst.h"
#include "ReferenceCloud.h"

//system
#include <assert.h>
#include <stdint.h> //for uint fixed-sized types
#include <string.h>

namespace CCLib
{

class GenericIndexedCloudPersist;
class GenericProgressCallback;

//! Proper KD-tree implementation
#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"
class CC_DLL_API TrueKdTree
#else
class TrueKdTree
#endif
{
public:

	static const uint8_t X_DIM = 0;
	static const uint8_t Y_DIM = 1;
	static const uint8_t Z_DIM = 2;

	static const uint8_t NODE_TYPE = 0;
	static const uint8_t LEAF_TYPE = 1;

	//! Tree base node
	struct BaseNode
	{
	public:
		BaseNode(uint8_t nodeType) : parent(0), type(nodeType) {}
		virtual ~BaseNode() {}

		bool isNode() const { return type == NODE_TYPE; }
		bool isLeaf() const { return type == LEAF_TYPE; }

	public:
		BaseNode* parent;

	protected:
		const uint8_t type;

	};

	//! Tree node
	struct Node : public BaseNode
	{
	public:
		uint8_t splitDim;
		PointCoordinateType splitValue;
		BaseNode* leftChild;
		BaseNode* rightChild;
		
		Node() : BaseNode(NODE_TYPE), splitDim(X_DIM), splitValue(0), leftChild(0), rightChild(0) {}
		virtual ~Node()
		{
			if (leftChild) delete leftChild;
			if (rightChild) delete rightChild;
		};
	};

	//! Tree leaf
	struct Leaf : public BaseNode
	{
	public:
		ReferenceCloud* points;
		PointCoordinateType planeEq[4];
		ScalarType rms;
		int userData;

		Leaf(ReferenceCloud* set) : BaseNode(LEAF_TYPE), points(set), rms(-1.0), userData(0) { memset(planeEq, 0, sizeof(PointCoordinateType)*4); }
		virtual ~Leaf() { if (points) delete points; };
	};

	//! A vector of leaves
	typedef std::vector<Leaf*> LeafVector;

	//! Default constructor
	TrueKdTree(GenericIndexedCloudPersist* cloud);

	//! Returns the associated cloud
	inline GenericIndexedCloudPersist* associatedCloud() const { return m_associatedCloud; }

	//! Builds KD-tree
	/** \param maxRMS maximum RMS per cell (LS plane fitting)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
	**/
	bool build(double maxRMS, GenericProgressCallback* progressCb=0);

	//! Clears structure
	void clear();

	//! Returns max RMS used for planarity-based split strategy
	double getMaxRMS() const { return m_maxRMS; }

	//! Returns all leaf nodes
	bool getLeaves(LeafVector& leaves) const;


protected:

	//! Recursive split process
	BaseNode* split(ReferenceCloud* subset);

	//! Root node
	BaseNode* m_root;

    //! Associated cloud
	GenericIndexedCloudPersist* m_associatedCloud;

	//! Max RMS for planarity-based split strategy
	double m_maxRMS;

};

} //namespace CCLib

#endif //TRUE_KD_TREE_HEADER