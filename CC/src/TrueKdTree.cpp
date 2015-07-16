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

//system
#include <algorithm>
#include <assert.h>

//Qt
#ifdef USE_QT
#include <QCoreApplication>
#endif

using namespace CCLib;

TrueKdTree::TrueKdTree(GenericIndexedCloudPersist* cloud)
	: m_root(0)
	, m_associatedCloud(cloud)
	, m_maxError(0.0)
	, m_errorMeasure(DistanceComputationTools::RMS)
	, m_maxPointCountPerCell(0)
{
	assert(m_associatedCloud);
}

TrueKdTree::~TrueKdTree()
{
	clear();
}

void TrueKdTree::clear()
{
	if (m_root)
		delete m_root;
	m_root = 0;
}

//shared structure used to sort the points along a single dimension (see TrueKdTree::split)
static std::vector<PointCoordinateType> s_sortedCoordsForSplit;
static GenericProgressCallback* s_progressCb = 0;
static unsigned s_lastProgressCount = 0;
static unsigned s_totalProgressCount = 0;
static unsigned s_lastProgress = 0;

static void InitProgress(GenericProgressCallback* progressCb, unsigned totalCount)
{
	s_progressCb = totalCount ? progressCb : 0;
	s_totalProgressCount = totalCount;
	s_lastProgressCount = 0;
	s_lastProgress = 0;

	if (s_progressCb)
	{
		s_progressCb->setMethodTitle("Kd-tree computation");
		char info[256];
		sprintf(info,"Points: %u",totalCount);
		s_progressCb->setInfo(info);
		s_progressCb->start();
#ifdef USE_QT
		QCoreApplication::processEvents();
#endif
	}
}

static inline void UpdateProgress(unsigned increment)
{
	if (s_progressCb)
	{
		assert(s_totalProgressCount != 0);
		s_lastProgressCount += increment;
		float fPercent = static_cast<float>(s_lastProgressCount) / static_cast<float>(s_totalProgressCount) * 100.0f;
		unsigned uiPercent = static_cast<unsigned>(fPercent);
		if (uiPercent > s_lastProgress)
		{
			s_progressCb->update(fPercent);
			s_lastProgress = uiPercent;
#ifdef USE_QT
			QCoreApplication::processEvents();
#endif
		}
	}
}

TrueKdTree::BaseNode* TrueKdTree::split(ReferenceCloud* subset)
{
	assert(subset); //subset will always be taken care of by this method
	
	unsigned count = subset->size();

	const PointCoordinateType* planeEquation = Neighbourhood(subset).getLSPlane();
	if (!planeEquation)
	{
		//an error occurred during LS plane computation?!
		delete subset;
		return 0;
	}

	//we always split sets larger than a given size (but we can't skip cells with less than 6 points!)
	ScalarType error = -1;
	if (count < m_maxPointCountPerCell || m_maxPointCountPerCell < 6)
	{
		assert(fabs(CCVector3(planeEquation).norm2() - 1.0) < 1.0e-6);
		error = (count != 3 ? DistanceComputationTools::ComputeCloud2PlaneDistance(subset, planeEquation, m_errorMeasure) : 0);
	
		//if we have less than 6 points, then the subdivision would produce a subset with less than 3 points
		//(and we can't fit a plane on less than 3 points!)
		bool isLeaf = (count < 6 || error <= m_maxError);
		if (isLeaf)
		{
			UpdateProgress(count);
			//the Leaf class takes ownership of the subset!
			return new Leaf(subset,planeEquation,error);
		}
	}

	/*** proceed with a 'standard' binary partition ***/

	//cell limits (dimensions)
	CCVector3 dims;
	{
		CCVector3 bbMin,bbMax;
		subset->getBoundingBox(bbMin,bbMax);
		dims = bbMax - bbMin;
	}

	//find the largest dimension
	uint8_t splitDim = X_DIM;
	if (dims.y > dims.x)
		splitDim = Y_DIM;
	if (dims.z > dims.u[splitDim])
		splitDim = Z_DIM;

	//find the median by sorting the points coordinates
	assert(s_sortedCoordsForSplit.size() >= static_cast<size_t>(count));
	for (unsigned i=0; i<count; ++i)
	{
		const CCVector3* P = subset->getPoint(i);
		s_sortedCoordsForSplit[i] = P->u[splitDim];
	}
	std::sort(s_sortedCoordsForSplit.begin(),s_sortedCoordsForSplit.begin()+count);

	unsigned splitCount = count/2;
	assert(splitCount >= 3); //count >= 6 (see above)
	
	//we must check that the split value is the 'first one'
	if (s_sortedCoordsForSplit[splitCount-1] == s_sortedCoordsForSplit[splitCount])
	{
		if (s_sortedCoordsForSplit[2] != s_sortedCoordsForSplit[splitCount]) //can we go backward?
		{
			while (/*splitCount>0 &&*/ s_sortedCoordsForSplit[splitCount-1] == s_sortedCoordsForSplit[splitCount])
			{
				assert(splitCount > 3);
				--splitCount;
			}
		}
		else if (s_sortedCoordsForSplit[count-3] != s_sortedCoordsForSplit[splitCount]) //can we go forward?
		{
			do
			{
				++splitCount;
				assert(splitCount < count-3);
			}
			while (/*splitCount+1<count &&*/ s_sortedCoordsForSplit[splitCount] == s_sortedCoordsForSplit[splitCount-1]);
		}
		else //in fact we can't split this cell!
		{
			UpdateProgress(count);
			if (error < 0)
				error = (count != 3 ? DistanceComputationTools::ComputeCloud2PlaneDistance(subset, planeEquation, m_errorMeasure) : 0);
			//the Leaf class takes ownership of the subset!
			return new Leaf(subset,planeEquation,error);
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
	{
		delete rightSubset;
		return 0;
	}
	BaseNode* rightChild = split(rightSubset);
	if (!rightChild)
	{
		delete leftChild;
		return 0;
	}

	Node* node = new Node;
	{
		node->leftChild = leftChild;
		leftChild->parent = node;
		node->rightChild = rightChild;
		rightChild->parent = node;
		node->splitDim = splitDim;
		node->splitValue = splitCoord;
	}
	return node;
}

bool TrueKdTree::build(	double maxError,
						DistanceComputationTools::ERROR_MEASURES errorMeasure/*=DistanceComputationTools::RMS*/,
						unsigned maxPointCountPerCell/*=0*/,
						GenericProgressCallback* progressCb/*=0*/)
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
	catch (const std::bad_alloc&)
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

	InitProgress(progressCb,count);

	//launch recursive process
	m_maxError = maxError;
	m_maxPointCountPerCell = maxPointCountPerCell;
	m_errorMeasure = errorMeasure;
	m_root = split(subset);

	//clear static structure
	s_sortedCoordsForSplit.clear();

	return (m_root != 0);
}

//! Recursive visitor for TrueKdTree::getLeaves
class GetLeavesVisitor
{
public:

	explicit GetLeavesVisitor(TrueKdTree::LeafVector& leaves) : m_leaves(&leaves) {}

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
	catch (const std::bad_alloc&)
	{
		return false;
	}

	return true;
}
