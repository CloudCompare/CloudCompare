//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

//Always first
#include "ccIncludeGL.h"

#include "ccKdTree.h"

//Local
#include "ccGenericPointCloud.h"
#include "ccPointCloud.h"

//CCLib
#include <DistanceComputationTools.h>

ccKdTree::ccKdTree(ccGenericPointCloud* aCloud)
	: CCLib::TrueKdTree(aCloud)
	, ccHObject("Kd-tree")
	, m_associatedGenericCloud(aCloud)
{
	setVisible(false);
	lockVisibility(false);
}

ccBBox ccKdTree::getMyOwnBB()
{
	return (m_associatedGenericCloud ? m_associatedGenericCloud->getMyOwnBB() : ccBBox());
}

ccBBox ccKdTree::getDisplayBB()
{
	return (m_associatedGenericCloud ? m_associatedGenericCloud->getDisplayBB() : ccBBox());
}

//recursive helper for ccKdTree::multiplyBoundingBox
static PointCoordinateType s_multFactor = 0;
void MultiplyNode(ccKdTree::BaseNode* node)
{
	if (node && node->isNode())
	{
		ccKdTree::Node* trueNode = static_cast<ccKdTree::Node*>(node);
		trueNode->splitValue *= s_multFactor;
		MultiplyNode(trueNode->leftChild);
		MultiplyNode(trueNode->rightChild);
	}
}

void ccKdTree::multiplyBoundingBox(const PointCoordinateType multFactor)
{
	s_multFactor = multFactor;
	
	if (m_root)
		MultiplyNode(m_root);
}

//recursive helper for ccKdTree::translateBoundingBox
static CCVector3 s_translation;
void TranslateNode(ccKdTree::BaseNode* node)
{
	if (node && node->isNode())
	{
		ccKdTree::Node* trueNode = static_cast<ccKdTree::Node*>(node);
		trueNode->splitValue += s_translation.u[trueNode->splitDim];
		TranslateNode(trueNode->leftChild);
		TranslateNode(trueNode->rightChild);
	}
}

void ccKdTree::translateBoundingBox(const CCVector3& T)
{
	s_translation = T;

	if (m_root)
		TranslateNode(m_root);
}

//recursive helper for ccKdTree::drawMeOnly
static ccBBox s_drawCellBBox;
void DrawNode(ccKdTree::BaseNode* node)
{
	if (!node)
		return;

	if (node->isNode())
	{
		ccKdTree::Node* trueNode = static_cast<ccKdTree::Node*>(node);
		//draw left child
		PointCoordinateType oldBBPos = s_drawCellBBox.maxCorner().u[trueNode->splitDim];
		s_drawCellBBox.maxCorner().u[trueNode->splitDim] = trueNode->splitValue;
		DrawNode(trueNode->leftChild);
		s_drawCellBBox.maxCorner().u[trueNode->splitDim] = oldBBPos;  //restore old limit

		//then draw right child
		oldBBPos = s_drawCellBBox.minCorner().u[trueNode->splitDim];
		s_drawCellBBox.minCorner().u[trueNode->splitDim] = trueNode->splitValue;
		DrawNode(trueNode->rightChild);
		s_drawCellBBox.minCorner().u[trueNode->splitDim] = oldBBPos; //restore old limit
	}
	else //if (node->isLeaf())
	{
		ccKdTree::Leaf* leaf = static_cast<ccKdTree::Leaf*>(node);
		s_drawCellBBox.draw(ccColor::green);
	}
}

void ccKdTree::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (!m_associatedGenericCloud || !m_root)
		return;

	if (MACRO_Draw3D(context))
	{
		bool pushName = MACRO_DrawEntityNames(context);

		if (pushName)
		{
			//not fast at all!
			if (MACRO_DrawFastNamesOnly(context))
				return;
			glPushName(getUniqueID());
		}

		s_drawCellBBox = m_associatedGenericCloud->getBB();
		DrawNode(m_root);

		if (pushName)
			glPopName();
	}
}

bool ccKdTree::convertCellIndexToSF()
{
	if (!m_associatedGenericCloud || !m_associatedGenericCloud->isA(CC_POINT_CLOUD))
		return false;

	//get leaves
	std::vector<Leaf*> leaves;
	if (!getLeaves(leaves) || leaves.empty())
		return false;

	ccPointCloud* pc = static_cast<ccPointCloud*>(m_associatedGenericCloud);

	const char c_defaultSFName[] = "Kd-tree indexes";
	int sfIdx = pc->getScalarFieldIndexByName(c_defaultSFName);
	if (sfIdx < 0)
		sfIdx = pc->addScalarField(c_defaultSFName);
	if (sfIdx < 0)
	{
		ccLog::Error("Not enough memory!");
		return false;
	}
	pc->setCurrentScalarField(sfIdx);

	//for each cell
	for (size_t i=0; i<leaves.size(); ++i)
	{
		CCLib::ReferenceCloud* subset = leaves[i]->points;
		if (subset)
		{
			for (unsigned j=0; j<subset->size(); ++j)
				subset->setPointScalarValue(j,(ScalarType)i);
		}
	}

	pc->getScalarField(sfIdx)->computeMinAndMax();
	pc->setCurrentDisplayedScalarField(sfIdx);
	pc->showSF(true);

	return true;
}

//Helper for ccKdTree::getCellBBox
static ccBBox s_UpdatedBox;
void UpdateBBox(ccKdTree::BaseNode* node)
{
	assert(node);
	if (node && node->parent)
	{
		assert(node->parent->isNode()); //a leaf can't have children!
		ccKdTree::Node* parent = static_cast<ccKdTree::Node*>(node->parent);

		//we choose the right 'side' of the box that corresponds to the parent's split plane
		CCVector3& boxCorner = (parent->leftChild == node ? s_UpdatedBox.maxCorner() : s_UpdatedBox.minCorner());

		//if this side has not been setup yet...
		if (boxCorner.u[parent->splitDim] != boxCorner.u[parent->splitDim]) //NaN
		{
			boxCorner.u[parent->splitDim] = parent->splitValue;
		}

		UpdateBBox(node->parent);
	}
}

ccBBox ccKdTree::getCellBBox(BaseNode* node) const
{
	if (!node || !m_associatedCloud)
		return ccBBox();

	//invalidate current (static) bounding box
	s_UpdatedBox.maxCorner() = CCVector3(NAN_VALUE,NAN_VALUE,NAN_VALUE);
	s_UpdatedBox.minCorner() = CCVector3(NAN_VALUE,NAN_VALUE,NAN_VALUE);

	UpdateBBox(node);

	//finish the job
	{
		CCVector3 bbMin,bbMax;
		m_associatedCloud->getBoundingBox(bbMin.u,bbMax.u);
		for (int i=0;i<3;++i)
		{
			if (s_UpdatedBox.minCorner().u[i] != s_UpdatedBox.minCorner().u[i]) //still NaN value?
				s_UpdatedBox.minCorner().u[i] = bbMin.u[i]; //we use the main bb limit
			if (s_UpdatedBox.maxCorner().u[i] != s_UpdatedBox.maxCorner().u[i]) //still NaN value?
				s_UpdatedBox.maxCorner().u[i] = bbMax.u[i]; //we use the main bb limit
		}
	}

	return s_UpdatedBox;
}

//Helper for ccKdTree::fuseCells
static ccKdTree::Leaf* s_seedCell = 0;
static ccBBox s_seedCellBox;
static ccBBox s_adjacentCellBox;
static PointCoordinateType s_minAdjacentAngleCos = 0;
static PointCoordinateType s_maxAdjacentRMS = 0;

void TestAdjacentCells(ccKdTree::BaseNode* node)
{
	assert(node);
	if (node->isNode())
	{
		//test bounding box
		if (	s_adjacentCellBox.contains(s_seedCellBox.minCorner())
			||	s_adjacentCellBox.contains(s_seedCellBox.maxCorner())
			||	s_seedCellBox.contains(s_adjacentCellBox.minCorner())
			||	s_seedCellBox.contains(s_adjacentCellBox.maxCorner()))
		{
			ccKdTree::Node* trueNode = static_cast<ccKdTree::Node*>(node);
			//draw left child
			PointCoordinateType oldBBPos = s_adjacentCellBox.maxCorner().u[trueNode->splitDim];
			s_adjacentCellBox.maxCorner().u[trueNode->splitDim] = trueNode->splitValue;
			TestAdjacentCells(trueNode->leftChild);
			s_adjacentCellBox.maxCorner().u[trueNode->splitDim] = oldBBPos;  //restore old limit

			//then draw right child
			oldBBPos = s_adjacentCellBox.minCorner().u[trueNode->splitDim];
			s_adjacentCellBox.minCorner().u[trueNode->splitDim] = trueNode->splitValue;
			TestAdjacentCells(trueNode->rightChild);
			s_adjacentCellBox.minCorner().u[trueNode->splitDim] = oldBBPos; //restore old limit
		}
	}
	else //if (node->isLeaf())
	{
		ccKdTree::Leaf* leaf = static_cast<ccKdTree::Leaf*>(node);
		if (leaf->userData == -1) //we only test cells not fused yet
		{
			//test bounding box
			if (	s_adjacentCellBox.contains(s_seedCellBox.minCorner())
				||	s_adjacentCellBox.contains(s_seedCellBox.maxCorner())
				||	s_seedCellBox.contains(s_adjacentCellBox.minCorner())
				||	s_seedCellBox.contains(s_adjacentCellBox.maxCorner()))
			{
				//compare normals
				if (CCVector3(s_seedCell->planeEq).dot(CCVector3(leaf->planeEq)) >= s_minAdjacentAngleCos)
				{
					//now we must check that the two clouds can actually be merged
					ScalarType rms = CCLib::DistanceComputationTools::computeCloud2PlaneDistanceRMS(leaf->points, s_seedCell->planeEq);

					if (rms < s_maxAdjacentRMS)
					{
						//we 'merge' them
						leaf->userData = s_seedCell->userData;
					}

				}
			}
		}
		s_adjacentCellBox.draw(ccColor::green);
	}
}

//Helper for ccKdTree::fuseCells
static bool AscendingLeafRMSComparison(const ccKdTree::Leaf* a, const ccKdTree::Leaf* b)
{
	return a->rms < b->rms;
}

bool ccKdTree::fuseCells(double maxRMS)
{
	if (!m_associatedGenericCloud || !m_associatedGenericCloud->isA(CC_POINT_CLOUD) || maxRMS < 0.0)
		return false;

	//get leaves
	std::vector<Leaf*> leaves;
	if (!getLeaves(leaves) || leaves.empty())
		return false;

	ccPointCloud* pc = static_cast<ccPointCloud*>(m_associatedGenericCloud);

	//sort cells based on their initial RMS
	std::sort(leaves.begin(),leaves.end(),AscendingLeafRMSComparison);

	//set all 'userData' to -1 (i.e. unfused cells)
	{
		for (size_t i=0; i<leaves.size(); ++i)
		{
			leaves[i]->userData = -1;
			//check by the way that the plane normal is unit!
			assert(fabs(CCVector3(leaves[i]->planeEq).norm2() - 1.0) < 1.0e-6);
		}
	}

	//fuse all cells, starting from the ones with the best RMS
	int macroIndex = 1; //starts at 1 (0 is reserved for cells already above the max RMS)
	{
		//main parameters
		s_minAdjacentAngleCos = cos(10.0 * CC_DEG_TO_RAD);
		s_maxAdjacentRMS = maxRMS;
		
		for (size_t i=0; i<leaves.size(); ++i)
		{
			if (leaves[i]->userData >= maxRMS)
			{
				leaves[i]->userData = 0; //special group for cells already above the user defined threshold!
			}

			//not already fused?
			if (leaves[i]->userData == -1)
			{
				//we create a new "macro cell" index
				leaves[i]->userData = macroIndex++; 
			}

			//determine the current cell bounding box
			ccBBox cellBox = getCellBBox(leaves[i]);

			//now check all cells adjacent to this one
			{
				s_seedCell = leaves[i];
				s_seedCellBox = cellBox;
				s_adjacentCellBox = pc->getBB();

				TestAdjacentCells(m_root);
			}
		}
	}

	//convert fused indexes to SF
	{
		const char c_defaultSFName[] = "Fused Kd-tree indexes";
		int sfIdx = pc->getScalarFieldIndexByName(c_defaultSFName);
		if (sfIdx < 0)
			sfIdx = pc->addScalarField(c_defaultSFName);
		if (sfIdx < 0)
		{
			ccLog::Error("Not enough memory!");
			return false;
		}
		pc->setCurrentScalarField(sfIdx);

		int unfusedIndexes = macroIndex;
		for (size_t i=0; i<leaves.size(); ++i)
		{
			CCLib::ReferenceCloud* subset = leaves[i]->points;
			if (subset)
			{
				int index = leaves[i]->userData;
				if (index <= 0)
					index = macroIndex++;
				for (unsigned j=0; j<subset->size(); ++j)
					subset->setPointScalarValue(j,(ScalarType)index);
			}
		}

		pc->getScalarField(sfIdx)->computeMinAndMax();
		pc->setCurrentDisplayedScalarField(sfIdx);
		pc->showSF(true);
	}

	return true;
}