//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qFacets                       #
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
//#                      COPYRIGHT: Thomas Dewez, BRGM                     #
//#                                                                        #
//##########################################################################

#include "kdTreeForFacetExtraction.h"

//CCLib
#include <GenericProgressCallback.h>
#include <ParallelSort.h>

//qCC_db
#include <ccPointCloud.h>

//Qt
#include <QApplication>

//static bool AscendingLeafErrorComparison(const ccKdTree::Leaf* a, const ccKdTree::Leaf* b)
//{
//	return a->error < b->error;
//}

static bool DescendingLeafSizeComparison(const ccKdTree::Leaf* a, const ccKdTree::Leaf* b)
{
	return a->points->size() > b->points->size();
}

struct Candidate
{
	ccKdTree::Leaf* leaf;
	PointCoordinateType dist;
	PointCoordinateType radius;
	CCVector3 centroid;

	Candidate() : leaf(nullptr), dist(PC_NAN), radius(0) {}
	Candidate(ccKdTree::Leaf* l) : leaf(l), dist(PC_NAN), radius(0)
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

bool ccKdTreeForFacetExtraction::FuseCells(	ccKdTree* kdTree,
											double maxError,
											CCLib::DistanceComputationTools::ERROR_MEASURES errorMeasure,
											double maxAngle_deg,
											PointCoordinateType overlapCoef/*=1*/,
											bool closestFirst/*=true*/,
											CCLib::GenericProgressCallback* progressCb/*=0*/)
{
	if (!kdTree)
		return false;

	ccGenericPointCloud* associatedGenericCloud = kdTree->associatedGenericCloud();
	if (!associatedGenericCloud || !associatedGenericCloud->isA(CC_TYPES::POINT_CLOUD) || maxError < 0.0)
		return false;

	//get leaves
	std::vector<ccKdTree::Leaf*> leaves;
	if (!kdTree->getLeaves(leaves) || leaves.empty())
		return false;

	//progress notification
	CCLib::NormalizedProgress nProgress(progressCb, static_cast<unsigned>(leaves.size()));
	if (progressCb)
	{
		progressCb->update(0);
		if (progressCb->textCanBeEdited())
		{
			progressCb->setMethodTitle("Fuse Kd-tree cells");
			progressCb->setInfo(qPrintable(QString("Cells: %1\nMax error: %2").arg(leaves.size()).arg(maxError)));
		}
		progressCb->start();
	}

	ccPointCloud* pc = static_cast<ccPointCloud*>(associatedGenericCloud);

	//sort cells based on their population size (we start by the biggest ones)
	ParallelSort(leaves.begin(), leaves.end(), DescendingLeafSizeComparison);

	//set all 'userData' to -1 (i.e. unfused cells)
	{
		for (size_t i=0; i<leaves.size(); ++i)
		{
			leaves[i]->userData = -1;
			//check by the way that the plane normal is unit!
			assert(static_cast<double>(fabs(CCVector3(leaves[i]->planeEq).norm2()) - 1.0) < 1.0e-6);
		}
	}

	// cosine of the max angle between fused 'planes'
	const double c_minCosNormAngle = cos(maxAngle_deg * CC_DEG_TO_RAD);

	//fuse all cells, starting from the ones with the best error
	const int unvisitedNeighborValue = -1;
	bool cancelled = false;
	int macroIndex = 1; //starts at 1 (0 is reserved for cells already above the max error)
	{
		for (size_t i=0; i<leaves.size(); ++i)
		{
			ccKdTree::Leaf* currentCell = leaves[i];
			if (currentCell->error >= maxError)
				currentCell->userData = 0; //0 = special group for cells already above the user defined threshold!

			//already fused?
			if (currentCell->userData != -1)
			{
				if (progressCb && !nProgress.oneStep()) //process canceled by user
				{
					cancelled = true;
					break;
				}
				continue;
			}

			//we create a new "macro cell" index
			currentCell->userData = macroIndex++;

			//we init the current set of 'fused' points with the cell's points
			CCLib::ReferenceCloud* currentPointSet = currentCell->points;
			//get current fused set centroid and normal
			CCVector3 currentCentroid = *CCLib::Neighbourhood(currentPointSet).getGravityCenter();
			CCVector3 currentNormal(currentCell->planeEq);

			//visited neighbors
			ccKdTree::LeafSet visitedNeighbors;
			//set of candidates
			std::list<Candidate> candidates;

			//we are going to iteratively look for neighbor cells that could be fused to this one
			ccKdTree::LeafVector cellsToTest;
			cellsToTest.push_back(currentCell);

			if (progressCb && !nProgress.oneStep()) //process canceled by user
			{
				cancelled = true;
				break;
			}

			while (!cellsToTest.empty() || !candidates.empty())
			{
				//get all neighbors around the 'waiting' cell(s)
				if (!cellsToTest.empty())
				{
					ccKdTree::LeafSet neighbors;
					while (!cellsToTest.empty())
					{
						if (!kdTree->getNeighborLeaves(cellsToTest.back(), neighbors, &unvisitedNeighborValue)) //we only consider unvisited cells!
						{
							//an error occurred
							return false;
						}
						cellsToTest.pop_back();
					}

					//add those (new) neighbors to the 'visitedNeighbors' set
					//and to the candidates set by the way if they are not yet there
					for (ccKdTree::LeafSet::iterator it=neighbors.begin(); it != neighbors.end(); ++it)
					{
						ccKdTree::Leaf* neighbor = *it;
						std::pair<ccKdTree::LeafSet::iterator,bool> ret = visitedNeighbors.insert(neighbor);
						//neighbour not already in the set?
						if (ret.second)
						{
							//we create the corresponding candidate
							try
							{
								candidates.push_back(Candidate(neighbor));
							}
							catch (const std::bad_alloc&)
							{
								//not enough memory!
								ccLog::Warning("[ccKdTreeForFacetExtraction] Not enough memory!");
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
					CCLib::ReferenceCloud* bestFused = nullptr;
					CCVector3 bestNormal(0,0,0);
					double bestError = -1.0;

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
							for (unsigned j = 0; j < currentPointSet->size(); ++j)
							{
								const CCVector3* P = currentPointSet->getPoint(j);
								PointCoordinateType d2 = (*P - it->centroid).norm2();
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

						//fuse the main set with the current candidate
						CCLib::ReferenceCloud* fused = new CCLib::ReferenceCloud(*currentPointSet);
						if (!fused->add(*(it->leaf->points)))
						{
							//not enough memory!
							ccLog::Warning("[ccKdTreeForFacetExtraction] Not enough memory!");
							delete fused;
							if (currentPointSet != currentCell->points)
								delete currentPointSet;
							return false;
						}

						//fit a plane and estimate the resulting error
						double error = -1.0;
						const PointCoordinateType* planeEquation = CCLib::Neighbourhood(fused).getLSPlane();
						if (planeEquation)
							error = CCLib::DistanceComputationTools::ComputeCloud2PlaneDistance(fused, planeEquation, errorMeasure);

						if (error < 0.0 || error > maxError)
						{
							//candidate is rejected
							it = candidates.erase(it);
						}
						else
						{
							//otherwise we keep track of the best one!
							if (bestError < 0.0 || error < bestError)
							{
								bestIt = it;
								bestError = error;
								if (bestFused)
									delete bestFused;
								bestFused = fused;
								bestNormal = CCVector3(planeEquation);
								fused = nullptr;

								if (closestFirst)
									break; //if we have found a good candidate, we stop here (closest first ;)
							}
							++it;
						}

						if (fused)
						{
							delete fused;
							fused = nullptr;
						}
					}

					//we have a (best) candidate for this pass?
					if (bestIt != candidates.end())
					{
						assert(bestFused && bestError >= 0.0);
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

						if (progressCb && !nProgress.oneStep()) //process canceled by user
						{
							//premature end!
							candidates.clear();
							cellsToTest.clear();
							cancelled = true;
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
			currentPointSet = nullptr;

			if (cancelled)
				break;
		}
	}

	//convert fused indexes to SF
	if (!cancelled)
	{
		if (!pc->enableScalarField())
		{
			ccLog::Error("Not enough memory");
			return false;
		}

		for (size_t i = 0; i < leaves.size(); ++i)
		{
			CCLib::ReferenceCloud* subset = leaves[i]->points;
			if (subset)
			{
				ScalarType scalar = static_cast<ScalarType>(leaves[i]->userData);
				if (leaves[i]->userData <= 0) //for unfused cells, we create new individual groups
				{
					scalar = static_cast<ScalarType>(macroIndex++);
					//scalar = NAN_VALUE; //FIXME TEST
				}
				for (unsigned j = 0; j < subset->size(); ++j)
				{
					subset->setPointScalarValue(j, scalar);
				}
			}
		}

		//pc->setCurrentDisplayedScalarField(sfIdx);
	}

	return !cancelled;
}