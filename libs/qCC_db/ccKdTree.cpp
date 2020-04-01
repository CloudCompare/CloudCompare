//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
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
#include "ccScalarField.h"

ccKdTree::ccKdTree(ccGenericPointCloud* aCloud)
	: CCLib::TrueKdTree(aCloud)
	, ccHObject("Kd-tree")
	, m_associatedGenericCloud(aCloud)
{
	setVisible(false);
	lockVisibility(false);
}

ccBBox ccKdTree::getOwnBB(bool withGLFeatures/*=false*/)
{
	return (m_associatedGenericCloud ? m_associatedGenericCloud->getOwnBB(withGLFeatures) : ccBBox());
}

//! Recursive visitor for ccKdTree::multiplyBoundingBox
class MultiplyBoundingBoxVisitor
{
public:
	
	MultiplyBoundingBoxVisitor(PointCoordinateType multFactor) : m_multFactor(multFactor) {}

	void visit(ccKdTree::BaseNode* node)
	{
		if (node && node->isNode())
		{
			ccKdTree::Node* trueNode = static_cast<ccKdTree::Node*>(node);
			trueNode->splitValue *= m_multFactor;
			visit(trueNode->leftChild);
			visit(trueNode->rightChild);
		}
	}

protected:
	PointCoordinateType m_multFactor;
};

void ccKdTree::multiplyBoundingBox(const PointCoordinateType multFactor)
{
	if (m_root)
		MultiplyBoundingBoxVisitor(multFactor).visit(m_root);
}

//! Recursive visitor for ccKdTree::translateBoundingBox
class TranslateBoundingBoxVisitor
{
public:
	
	TranslateBoundingBoxVisitor(const CCVector3& T) : m_translation(T) {}

	void visit(ccKdTree::BaseNode* node)
	{
		if (node && node->isNode())
		{
			ccKdTree::Node* trueNode = static_cast<ccKdTree::Node*>(node);
			trueNode->splitValue += m_translation.u[trueNode->splitDim];
			visit(trueNode->leftChild);
			visit(trueNode->rightChild);
		}
	}

protected:
	CCVector3 m_translation;

};

void ccKdTree::translateBoundingBox(const CCVector3& T)
{
	if (m_root)
		TranslateBoundingBoxVisitor(T).visit(m_root);
}

//! Recursive visitor for ccKdTree::drawMeOnly
class DrawMeOnlyVisitor
{
public:
	
	DrawMeOnlyVisitor(const ccBBox& box) : m_drawCellBBox(box) {}

	void visit(CC_DRAW_CONTEXT& context, ccKdTree::BaseNode* node)
	{
		if (!node)
			return;

		if (node->isNode())
		{
			ccKdTree::Node* trueNode = static_cast<ccKdTree::Node*>(node);
			//visit left child
			PointCoordinateType oldBBPos = m_drawCellBBox.maxCorner().u[trueNode->splitDim];
			m_drawCellBBox.maxCorner().u[trueNode->splitDim] = trueNode->splitValue;
			visit(context, trueNode->leftChild);
			m_drawCellBBox.maxCorner().u[trueNode->splitDim] = oldBBPos;  //restore old limit

			//then visit right child
			oldBBPos = m_drawCellBBox.minCorner().u[trueNode->splitDim];
			m_drawCellBBox.minCorner().u[trueNode->splitDim] = trueNode->splitValue;
			visit(context, trueNode->rightChild);
			m_drawCellBBox.minCorner().u[trueNode->splitDim] = oldBBPos; //restore old limit
		}
		else //if (node->isLeaf())
		{
			m_drawCellBBox.draw(context, ccColor::green);
		}
	}

protected:
	ccBBox m_drawCellBBox;

};

void ccKdTree::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (!m_associatedGenericCloud || !m_root)
		return;

	if (!MACRO_Draw3D(context))
		return;
	
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert( glFunc != nullptr );
	
	if ( glFunc == nullptr )
		return;
	
	bool pushName = MACRO_DrawEntityNames(context);

	if (pushName)
	{
		//not fast at all!
		if (MACRO_DrawFastNamesOnly(context))
			return;
		glFunc->glPushName(getUniqueIDForDisplay());
	}

	DrawMeOnlyVisitor(m_associatedGenericCloud->getOwnBB()).visit(context, m_root);

	if (pushName)
		glFunc->glPopName();
}

bool ccKdTree::convertCellIndexToSF()
{
	if (!m_associatedGenericCloud || !m_associatedGenericCloud->isA(CC_TYPES::POINT_CLOUD))
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

bool ccKdTree::convertCellIndexToRandomColor()
{
	if (!m_associatedGenericCloud || !m_associatedGenericCloud->isA(CC_TYPES::POINT_CLOUD))
		return false;

	//get leaves
	std::vector<Leaf*> leaves;
	if (!getLeaves(leaves) || leaves.empty())
		return false;

	ccPointCloud* pc = static_cast<ccPointCloud*>(m_associatedGenericCloud);
	if (!pc->resizeTheRGBTable())
		return false;

	//for each cell
	for (size_t i = 0; i < leaves.size(); ++i)
	{
		ccColor::Rgba col(ccColor::Generator::Random(), ccColor::MAX);
		CCLib::ReferenceCloud* subset = leaves[i]->points;
		if (subset)
		{
			for (unsigned j = 0; j < subset->size(); ++j)
				pc->setPointColor(subset->getPointGlobalIndex(j), col);
		}
	}

	pc->showColors(true);

	return true;
}

//! Recursive visitor for ccKdTree::getCellBBox
class GetCellBBoxVisitor
{
public:
	
	ccBBox m_UpdatedBox;

	GetCellBBoxVisitor()
	{
		//invalidate the initial bounding box
		m_UpdatedBox.maxCorner() = CCVector3(PC_NAN,PC_NAN,PC_NAN);
		m_UpdatedBox.minCorner() = CCVector3(PC_NAN,PC_NAN,PC_NAN);
	}
	
	void visit(ccKdTree::BaseNode* node)
	{
		assert(node);
		if (node && node->parent)
		{
			assert(node->parent->isNode()); //a leaf can't have children!
			ccKdTree::Node* parent = static_cast<ccKdTree::Node*>(node->parent);

			//we choose the right 'side' of the box that corresponds to the parent's split plane
			CCVector3& boxCorner = (parent->leftChild == node ? m_UpdatedBox.maxCorner() : m_UpdatedBox.minCorner());

			//if this side has not been setup yet...
			if (boxCorner.u[parent->splitDim] != boxCorner.u[parent->splitDim]) //NaN
				boxCorner.u[parent->splitDim] = parent->splitValue;

			visit(node->parent);
		}
	}
};

ccBBox ccKdTree::getCellBBox(BaseNode* node) const
{
	if (!node || !m_associatedCloud)
		return ccBBox();

	GetCellBBoxVisitor helper;
	helper.visit(node);

	//finish the job
	ccBBox& box = helper.m_UpdatedBox;
	{
		CCVector3 bbMin;
		CCVector3 bbMax;
		m_associatedCloud->getBoundingBox(bbMin,bbMax);
		for (int i=0; i<3; ++i)
		{
			if (box.minCorner().u[i] != box.minCorner().u[i]) //still NaN value?
				box.minCorner().u[i] = bbMin.u[i]; //we use the main bb limit
			if (box.maxCorner().u[i] != box.maxCorner().u[i]) //still NaN value?
				box.maxCorner().u[i] = bbMax.u[i]; //we use the main bb limit
		}
		box.setValidity(true);
	}

	return box;
}

//! Recursive visitor for ccKdTree::getNeighborLeaves
class GetNeighborLeavesVisitor
{
public:

	GetNeighborLeavesVisitor(ccKdTree::BaseNode* cell,
							ccKdTree::LeafSet& neighbors,
							const ccBBox& cellBox,
							const ccBBox& treeBox)
		: m_targetCell(cell)
		, m_targetCellBox(cellBox)
		, m_currentCellBox(treeBox)
		, m_neighbors(&neighbors)
		, m_userDataFilterEnabled(false)
		, m_userDataFilterValue(0)
	{
	}

	void setUserDataFilter(int value)
	{
		m_userDataFilterEnabled = true;
		m_userDataFilterValue = value;
	}

	void visit(ccKdTree::BaseNode* node)
	{
		assert(node);
		if (!node || node == m_targetCell)
			return;

		if (node->isNode())
		{
			//test bounding box
			if (m_currentCellBox.minDistTo(m_targetCellBox) == 0)
			{
				ccKdTree::Node* trueNode = static_cast<ccKdTree::Node*>(node);
				//visit left child
				PointCoordinateType oldBBPos = m_currentCellBox.maxCorner().u[trueNode->splitDim];
				m_currentCellBox.maxCorner().u[trueNode->splitDim] = trueNode->splitValue;
				visit(trueNode->leftChild);
				m_currentCellBox.maxCorner().u[trueNode->splitDim] = oldBBPos;  //restore old limit

				//then visit right child
				oldBBPos = m_currentCellBox.minCorner().u[trueNode->splitDim];
				m_currentCellBox.minCorner().u[trueNode->splitDim] = trueNode->splitValue;
				visit(trueNode->rightChild);
				m_currentCellBox.minCorner().u[trueNode->splitDim] = oldBBPos; //restore old limit
			}
		}
		else //if (node->isLeaf())
		{
			ccKdTree::Leaf* leaf = static_cast<ccKdTree::Leaf*>(node);
			if (m_currentCellBox.minDistTo(m_targetCellBox) == 0)
			{
				//the caller can set a filter on the user data value!
				if (!m_userDataFilterEnabled || m_userDataFilterValue == leaf->userData)
				{
					assert(m_neighbors);
					m_neighbors->insert(leaf);
				}
			}
		}
	}

protected:
	ccKdTree::BaseNode* m_targetCell;
	ccBBox m_targetCellBox;
	ccBBox m_currentCellBox;
	ccKdTree::LeafSet* m_neighbors;
	bool m_userDataFilterEnabled;
	int m_userDataFilterValue;

};

bool ccKdTree::getNeighborLeaves(ccKdTree::BaseNode* cell, ccKdTree::LeafSet& neighbors, const int* userDataFilter/*=0*/)
{
	if (!m_root)
		return false;

	//determine the cell bounding box
	ccBBox cellBox = getCellBBox(cell);
	if (!cellBox.isValid())
		return false;

	try
	{
		GetNeighborLeavesVisitor visitor(cell, neighbors, cellBox, getOwnBB(false));
		if (userDataFilter)
			visitor.setUserDataFilter(*userDataFilter);
		visitor.visit(m_root);
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}

	return true;
}
