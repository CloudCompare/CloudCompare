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
#include <Neighbourhood.h>
#include <GenericProgressCallback.h>

//Qt
#include <QApplication>

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

	void visit(ccKdTree::BaseNode* node)
	{
		if (!node)
			return;

		if (node->isNode())
		{
			ccKdTree::Node* trueNode = static_cast<ccKdTree::Node*>(node);
			//visit left child
			PointCoordinateType oldBBPos = m_drawCellBBox.maxCorner().u[trueNode->splitDim];
			m_drawCellBBox.maxCorner().u[trueNode->splitDim] = trueNode->splitValue;
			visit(trueNode->leftChild);
			m_drawCellBBox.maxCorner().u[trueNode->splitDim] = oldBBPos;  //restore old limit

			//then visit right child
			oldBBPos = m_drawCellBBox.minCorner().u[trueNode->splitDim];
			m_drawCellBBox.minCorner().u[trueNode->splitDim] = trueNode->splitValue;
			visit(trueNode->rightChild);
			m_drawCellBBox.minCorner().u[trueNode->splitDim] = oldBBPos; //restore old limit
		}
		else //if (node->isLeaf())
		{
			m_drawCellBBox.draw(ccColor::green);
		}
	}

protected:
	ccBBox m_drawCellBBox;

};

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

		DrawMeOnlyVisitor(m_associatedGenericCloud->getBB()).visit(m_root);

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

//! Recursive visitor for ccKdTree::getCellBBox
class GetCellBBoxVisitor
{
public:
	
	ccBBox m_UpdatedBox;

	GetCellBBoxVisitor()
	{
		//invalidate the initial bounding box
		m_UpdatedBox.maxCorner() = CCVector3(NAN_VALUE,NAN_VALUE,NAN_VALUE);
		m_UpdatedBox.minCorner() = CCVector3(NAN_VALUE,NAN_VALUE,NAN_VALUE);
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
		CCVector3 bbMin,bbMax;
		m_associatedCloud->getBoundingBox(bbMin.u,bbMax.u);
		for (int i=0;i<3;++i)
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
		GetNeighborLeavesVisitor visitor(cell, neighbors, cellBox, getMyOwnBB());
		if (userDataFilter)
			visitor.setUserDataFilter(*userDataFilter);
		visitor.visit(m_root);
	}
	catch (std::bad_alloc)
	{
		return false;
	}

	return true;
}

static bool AscendingLeafRMSComparison(const ccKdTree::Leaf* a, const ccKdTree::Leaf* b)
{
	return a->rms < b->rms;
}
static bool DescendingLeafSizeComparison(const ccKdTree::Leaf* a, const ccKdTree::Leaf* b)
{
	return a->points->size() > b->points->size();
}

struct Candidate
{
	ccKdTree::Leaf* leaf;
	ScalarType dist;
	ScalarType radius;
	CCVector3 centroid;

	Candidate() : leaf(0), dist(NAN_VALUE), radius(0) {}
	Candidate(ccKdTree::Leaf* l) : leaf(l), dist(NAN_VALUE), radius(0)
	{
		if (leaf && leaf->points)
		{
			CCLib::Neighbourhood N(leaf->points);
			centroid = *N.getGravityCenter();
			radius = N.computeLargestRadius();
		}
	}
};

static bool CandidateDistAscendingComparison(const Candidate& a, const Candidate& b)
{
	return a.dist < b.dist;
}

bool ccKdTree::fuseCells(double maxRMS, double maxAngle_deg, double overlapCoef, bool closestFirst/*=true*/, CCLib::GenericProgressCallback* progressCb/*=0*/)
{
	if (!m_associatedGenericCloud || !m_associatedGenericCloud->isA(CC_POINT_CLOUD) || maxRMS < 0.0)
		return false;

	//get leaves
	std::vector<Leaf*> leaves;
	if (!getLeaves(leaves) || leaves.empty())
		return false;

	//progress notification
	CCLib::NormalizedProgress* nProgress = 0;
	if (progressCb)
	{
		progressCb->reset();
		progressCb->setMethodTitle("Fuse Kd-tree cells");
		progressCb->setInfo(qPrintable(QString("cells: %1\nmax RMS: %2").arg(leaves.size()).arg(maxRMS)));
		nProgress = new CCLib::NormalizedProgress(progressCb,(unsigned)leaves.size());
		progressCb->start();
	}

	ccPointCloud* pc = static_cast<ccPointCloud*>(m_associatedGenericCloud);

	//sort cells based on their population size (we start by the biggest ones)
	std::sort(leaves.begin(),leaves.end(),DescendingLeafSizeComparison);

	//set all 'userData' to -1 (i.e. unfused cells)
	{
		for (size_t i=0; i<leaves.size(); ++i)
		{
			leaves[i]->userData = -1;
			//check by the way that the plane normal is unit!
			assert(fabs(CCVector3(leaves[i]->planeEq).norm2() - 1.0) < 1.0e-6);
		}
	}

	// max angle between fused 'planes'
	const double c_minCosNormAngle = cos(maxAngle_deg * CC_DEG_TO_RAD);

	//fuse all cells, starting from the ones with the best RMS
	const int unvisitedNeighborValue = -1;
	int macroIndex = 1; //starts at 1 (0 is reserved for cells already above the max RMS)
	{
		for (size_t i=0; i<leaves.size(); ++i)
		{
			Leaf* currentCell = leaves[i];
			if (currentCell->rms >= maxRMS)
				currentCell->userData = 0; //0 = special group for cells already above the user defined threshold!

			//already fused?
			if (currentCell->userData != -1)
			{
				if (nProgress && !nProgress->oneStep()) //process canceled by user
					break;
				continue;
			}

			//we create a new "macro cell" index
			currentCell->userData = macroIndex++;

			//we init the current set of 'fused' points with the cell's points
			CCLib::ReferenceCloud* currentPointSet = currentCell->points;
			//current fused set centroid and radius
			CCVector3 currentCentroid;
			{
				CCLib::Neighbourhood N(currentPointSet);
				currentCentroid = *N.getGravityCenter();
			}
			CCVector3 currentNormal(currentCell->planeEq);

			//visited neighbors
			LeafSet visitedNeighbors;
			//set of candidates
			std::list<Candidate> candidates;

			//we are going to iteratively look for neighbor cells that could be fused to this one
			LeafVector cellsToTest;
			cellsToTest.push_back(currentCell);

			if (nProgress && !nProgress->oneStep()) //process canceled by user
				break;

			while (!cellsToTest.empty() || !candidates.empty())
			{
				//get all neighbors around the 'waiting' cell(s)
				if (!cellsToTest.empty())
				{
					LeafSet neighbors;
					while (!cellsToTest.empty())
					{
						if (!getNeighborLeaves(cellsToTest.back(), neighbors, &unvisitedNeighborValue)) //we only consider unvisited cells!
						{
							//an error occured
							return false;
						}
						cellsToTest.pop_back();
					}

					//add those (new) neighbors to the 'visitedNeighbors' set and to the candidates set by the way
					//if they are not yet there
					for (LeafSet::iterator it=neighbors.begin(); it != neighbors.end(); ++it)
					{
						Leaf* neighbor = *it;
						std::pair<LeafSet::iterator,bool> ret = visitedNeighbors.insert(neighbor);
						//neighbour not already in the set?
						if (ret.second)
						{
							//we create the corresponding candidate
							try
							{
								candidates.push_back(Candidate(neighbor));
							}
							catch (std::bad_alloc)
							{
								//not enough memory!
								ccLog::Warning("[ccKdTree::fuseCells] Not enough memory!");
								return false;
							}
						}
					}
				}

				//is there remaining candidates?
				if (!candidates.empty())
				{
					//update the set of candidates
					if (closestFirst && candidates.size() > 1)
					{
						for (std::list<Candidate>::iterator it = candidates.begin(); it !=candidates.end(); ++it)
							it->dist = (it->centroid-currentCentroid).norm2();

						//sort candidates by their distance
						candidates.sort(CandidateDistAscendingComparison);
					}
					
					//we will keep track of the best fused 'couple' at each pass
					std::list<Candidate>::iterator bestIt = candidates.end();
					CCLib::ReferenceCloud* bestFused = 0;
					CCVector3 bestNormal(0,0,0);
					double bestRMS = -1.0;

					unsigned skipCount = 0;
					for (std::list<Candidate>::iterator it = candidates.begin(); it != candidates.end(); /*++it*/)
					{
						assert(it->leaf && it->leaf->points);
						assert(currentPointSet->getAssociatedCloud() == it->leaf->points->getAssociatedCloud());

						//if the leaf orientation is too different
						if (fabs(CCVector3(it->leaf->planeEq).dot(currentNormal)) < c_minCosNormAngle)
						{
							it = candidates.erase(it);
							//++it;
							//++skipCount;
							continue;
						}

						//compute the minimum distance between the candidate centroid and the 'currentPointSet'
						PointCoordinateType minDistToMainSet = 0.0;
						{
							for (unsigned j=0; j<currentPointSet->size(); ++j)
							{
								const CCVector3* P = currentPointSet->getPoint(j);
								PointCoordinateType d2 = (*P-it->centroid).norm2();
								if (d2 < minDistToMainSet || j == 0)
									minDistToMainSet = d2;
							}
							minDistToMainSet = sqrt(minDistToMainSet);
						}
						
						//if the leaf is too far
						if (it->radius < minDistToMainSet / overlapCoef)
						{
							++it;
							++skipCount;
							continue;
						}

						//fuse the best fused set with the current candidate
						CCLib::ReferenceCloud* fused = new CCLib::ReferenceCloud(*currentPointSet);
						if (!fused->add(*(it->leaf->points)))
						{
							//not enough memory!
							ccLog::Warning("[ccKdTree::fuseCells] Not enough memory!");
							delete fused;
							if (currentPointSet != currentCell->points)
								delete currentPointSet;
							return false;
						}

						//fit a plane and estimate resulting RMS
						double rms = -1.0;
						const PointCoordinateType* planeEquation = CCLib::Neighbourhood(fused).getLSQPlane();
						if (planeEquation)
							//rms = CCLib::DistanceComputationTools::computeCloud2PlaneDistanceRMS(fused, planeEquation);
							rms = CCLib::DistanceComputationTools::ComputeCloud2PlaneRobustMax(fused, planeEquation, 0.02f);
						//double rms = 0.0; //FIXME TEST

						if (rms < 0.0 || rms > maxRMS)
						{
							//candidate is rejected
							it = candidates.erase(it);
						}
						else
						{
							//otherwise we keep track of the best one!
							if (bestRMS < 0.0 || rms < bestRMS)
							{
								bestIt = it;
								bestRMS = rms;
								if (bestFused)
									delete bestFused;
								bestFused = fused;
								bestNormal = CCVector3(planeEquation);
								fused = 0;
								
								if (closestFirst)
									break; //if we have found a good candidate, we stop here (closest first ;)
							}
							++it;
						}

						if (fused)
						{
							delete fused;
							fused = 0;
						}
					}

					//we have a (best) candidate for this pass?
					if (bestIt != candidates.end())
					{
						assert(bestFused && bestRMS >= 0.0);
						if (currentPointSet != currentCell->points)
							delete currentPointSet;
						currentPointSet = bestFused;
						{
							//update infos
							CCLib::Neighbourhood N(currentPointSet);
							//currentCentroid = *N.getGravityCenter(); //if we update it, the search will naturally shift along one dimension!
							//currentNormal = bestNormal; //same thing here for normals
						}

						bestIt->leaf->userData = currentCell->userData;
						//bestIt->leaf->userData = macroIndex++; //FIXME TEST

						//we will test this cell's neighbors as well
						cellsToTest.push_back(bestIt->leaf);

						if (nProgress && !nProgress->oneStep()) //process canceled by user
						{
							//permaturate end!
							candidates.clear();
							cellsToTest.clear();
							i = leaves.size();
							break;
						}
						QApplication::processEvents();

						//we also remove it from the candidates list
						candidates.erase(bestIt);
					}

					if (skipCount == candidates.size() && cellsToTest.empty())
					{
						//only far leaves remain...
						candidates.clear();
					}

				}
			
			} //no more candidates or cells to test

			//end of the fusion process for the current leaf
			if (currentPointSet != currentCell->points)
				delete currentPointSet;
			currentPointSet = 0;
		}
	}

	//convert fused indexes to SF
	{
		pc->enableScalarField();

		for (size_t i=0; i<leaves.size(); ++i)
		{
			CCLib::ReferenceCloud* subset = leaves[i]->points;
			if (subset)
			{
				ScalarType scalar = (ScalarType)leaves[i]->userData;
				if (leaves[i]->userData <= 0) //for unfused cells, we create new individual groups
					scalar = (ScalarType)(macroIndex++);
					//scalar = NAN_VALUE; //FIXME TEST
				for (unsigned j=0; j<subset->size(); ++j)
					subset->setPointScalarValue(j,scalar);
			}
		}

		//pc->setCurrentDisplayedScalarField(sfIdx);
	}

	return true;
}