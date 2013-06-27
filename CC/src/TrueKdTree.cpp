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

#include "TrueKdTree.h"

//local
#include "GenericProgressCallback.h"
#include "GenericIndexedCloudPersist.h"
#include "Neighbourhood.h"
#include "DistanceComputationTools.h"

//system
#include <algorithm>
#include <assert.h>

using namespace CCLib;

TrueKdTree::TrueKdTree(GenericIndexedCloudPersist* cloud)
	: m_root(0)
	, m_associatedCloud(cloud)
	, m_maxRMS(0.0)
{
	assert(cloud);
}

void TrueKdTree::clear()
{
	if (m_root)
		delete m_root;
	m_root = 0;
}

//shared structure used to sort the points along a single dimension (see TrueKdTree::split)
static std::vector<PointCoordinateType> s_sortedCoordsForSplit;

TrueKdTree::BaseNode* TrueKdTree::split(ReferenceCloud* subset)
{
	assert(subset); //subset will always be taken care of by this method
	
	unsigned count = subset->size();

	const PointCoordinateType* planeEquation = Neighbourhood(subset).getLSQPlane();
	if (!planeEquation)
	{
		//an error occured during LS plane computation?!
		delete subset;
		return 0;
	}
	
	assert(fabs(CCVector3(planeEquation).norm2() - 1.0) < 1.0e-6);
	//ScalarType rms = (count != 3 ? DistanceComputationTools::computeCloud2PlaneDistanceRMS(subset, planeEquation) : 0);
	ScalarType rms = (count != 3 ? DistanceComputationTools::ComputeCloud2PlaneRobustMax(subset, planeEquation, 0.02f) : 0);
	
	//if we have less than 6 points, then the subdivision would produce a subset with less than 3 points
	//(and we can't fit a plane on less than 3 points!)
	bool isLeaf = (count < 6 || rms <= m_maxRMS);
	if (isLeaf)
	{
		Leaf* leaf = new Leaf(subset);
		memcpy(leaf->planeEq,planeEquation,sizeof(PointCoordinateType)*4);
		leaf->rms = rms;
		
		return leaf;
	}

	/*** proceed with a 'standard' binary partition ***/

	//cell limits (dimensions)
	CCVector3 dims;
	{
		CCVector3 bbMin,bbMax;
		subset->getBoundingBox(bbMin.u,bbMax.u);
		dims = bbMax-bbMin;
	}

	//find the largest dimension
	uint8_t splitDim = X_DIM;
	if (dims.y > dims.x)
		splitDim = Y_DIM;
	if (dims.z > dims.u[splitDim])
		splitDim = Z_DIM;

	//find the median by sorting the points coordinates
	assert(s_sortedCoordsForSplit.size() >= (size_t)count);
	for (unsigned i=0; i<count; ++i)
	{
		const CCVector3* P = subset->getPoint(i);
		s_sortedCoordsForSplit[i] = P->u[splitDim];
	}
	std::sort(s_sortedCoordsForSplit.begin(),s_sortedCoordsForSplit.begin()+count);

	unsigned splitCount = count>>1;
	assert(splitCount >= 3); //count >= 6 (see above)
	
	//we must check that the split value is the 'first one'
	if (s_sortedCoordsForSplit[splitCount-1] == s_sortedCoordsForSplit[splitCount])
	{
		if (s_sortedCoordsForSplit[2] != s_sortedCoordsForSplit[splitCount]) //can we go backward?
		{
			while (/*splitCount>0 &&*/ s_sortedCoordsForSplit[splitCount-1] == s_sortedCoordsForSplit[splitCount])
			{
				assert(splitCount > 0);
				--splitCount;
			}
		}
		else if (s_sortedCoordsForSplit[count-3] != s_sortedCoordsForSplit[splitCount]) //can we go forward?
		{
			do
			{
				++splitCount;
				assert(splitCount < count);
			}
			while (/*splitCount+1<count &&*/ s_sortedCoordsForSplit[splitCount] == s_sortedCoordsForSplit[splitCount-1]);
		}
		else //in fact we can't split this cell!
		{
			Leaf* leaf = new Leaf(subset);
			memcpy(leaf->planeEq,planeEquation,sizeof(PointCoordinateType)*4);
			leaf->rms = rms;
			return leaf;
		}
	}

	PointCoordinateType splitCoord = s_sortedCoordsForSplit[splitCount]; //count > 3 --> splitCount >= 2

	ReferenceCloud* leftSubset = new ReferenceCloud(subset->getAssociatedCloud());
	ReferenceCloud* rightSubset = new ReferenceCloud(subset->getAssociatedCloud());
	if (!leftSubset->reserve(splitCount) || !rightSubset->reserve(count-splitCount))
	{
		//not enough memory!
		delete leftSubset;
		delete rightSubset;
		delete subset;
		return 0;
	}

	//fill subsets
	for (unsigned i=0; i<count; ++i)
	{
		const CCVector3* P = subset->getPoint(i);
		if (P->u[splitDim] < splitCoord)
		{
			leftSubset->addPointIndex(subset->getPointGlobalIndex(i));
		}
		else
		{
			rightSubset->addPointIndex(subset->getPointGlobalIndex(i));
		}
	}

	//release some memory before the next incursion!
	delete subset;
	subset = 0;

	//process subsets (if any)
	BaseNode* leftChild = split(leftSubset);
	if (!leftChild)
		return 0;
	BaseNode* rightChild = split(rightSubset);
	if (!rightChild)
	{
		delete leftChild;
		return 0;
	}

	Node* node = new Node;
	node->leftChild = leftChild;
	leftChild->parent = node;
	node->rightChild = rightChild;
	rightChild->parent = node;
	node->splitDim = splitDim;
	node->splitValue = splitCoord;

	return node;
}

bool TrueKdTree::build(double maxRMS, GenericProgressCallback* progressCb/*=0*/)
{
	if (!m_associatedCloud)
		return false;

	//tree already computed! (call clear before)
	if (m_root)
		return false;

	unsigned count = m_associatedCloud->size();
	if (count == 0) //no point, no node!
	{
		return false;
	}

	//structures used to sort the points along the 3 dimensions
	try
	{
		s_sortedCoordsForSplit.resize(count);
	}
	catch(std::bad_alloc)
	{
		//not enough memory!
		return false;
	}

	//initial 'subset' to start recursion
	ReferenceCloud* subset = new ReferenceCloud(m_associatedCloud);
	if (!subset->addPointIndex(0,count))
	{
		//not enough memory
		delete subset;
		return false;
	}

	//launch recursive process
	m_maxRMS = maxRMS;
	m_root = split(subset);

	//clear static structure
	s_sortedCoordsForSplit.clear();

	return (m_root != 0);
}

//! Recursive visitor for TrueKdTree::getLeaves
class GetLeavesVisitor
{
public:

	GetLeavesVisitor(TrueKdTree::LeafVector& leaves) : m_leaves(&leaves) {}

	void visit(TrueKdTree::BaseNode* node)
	{
		if (!node)
			return;

		if (node->isNode())
		{
			visit(static_cast<TrueKdTree::Node*>(node)->leftChild);
			visit(static_cast<TrueKdTree::Node*>(node)->rightChild);
		}
		else //if (node->isLeaf())
		{
			assert(m_leaves);
			m_leaves->push_back(static_cast<TrueKdTree::Leaf*>(node));
		}
	}

protected:
	TrueKdTree::LeafVector* m_leaves;

};

bool TrueKdTree::getLeaves(LeafVector& leaves) const
{
	if (!m_root)
		return false;

	try
	{		
		GetLeavesVisitor(leaves).visit(m_root);
	}
	catch(std::bad_alloc)
	{
		return false;
	}

	return true;
}
