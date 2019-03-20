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

#ifndef QFACET_FACETS_CLASSIFIER_HEADER
#define QFACET_FACETS_CLASSIFIER_HEADER

//Qt
#include <QProgressDialog>
#include <QApplication>

//qCC_db
#include "ccFacet.h"
#include "ccPointCloud.h"
#include "ccPolyline.h"

static const QString s_OriFamilyKey = "orientation.family.index";
static const QString s_OriFamilyNameKey = "orientation.family.name";
static const QString s_OriSubFamilyKey = "orientation.subfamily.index";

// default ratio for the 'dark' version of a color
const double c_darkColorRatio = 0.25;

class FacetsClassifier
{
public:

	//! Set of facets (pointers to)
	typedef std::vector<ccFacet*> FacetSet;

	//! Computes minimal 'orthogonal' distance between two facets
	static PointCoordinateType CommputeHDistBetweenFacets(const ccFacet* f1, const ccFacet* f2)
	{
		CCVector3 AB = f1->getCenter() - f2->getCenter();
		return std::min(fabs(AB.dot(f1->getNormal())), fabs(AB.dot(f2->getNormal())));
	}

	//! Generates a given sub-family color
	static void GenerateSubfamilyColor(ccColor::Rgb& col,
		double dip,
		double dipDir,
		unsigned subFamilyIndex,
		unsigned subFamilyCount,
		ccColor::Rgb* darkCol = 0)
	{
		//convert dip & dip dir. to HSV
		double H = dipDir;
		if (H == 360.0) //H is in [0;360[
			H = 0;
		double S = dip / 90.0; //S is in [0;1]
		double L = 0.5;

		if (subFamilyCount > 1)
		{
			assert(subFamilyIndex >= 1);
			//FIXME: how could we do this?!
			//V = 0.5 + 0.5 * static_cast<double>(subFamilyIndex-1)/static_cast<double>(subFamilyCount-1);
		}

		col = ccColor::Convert::hsl2rgb(H, S, L);
		if (darkCol)
		{
			*darkCol = ccColor::Convert::hsl2rgb(H, S, c_darkColorRatio);
		}
	}

	static void GetFamilyIndexes(ccFacet* facet,
		unsigned dSteps,
		unsigned ddSteps,
		double angularStep_deg,
		unsigned& iDip,
		unsigned& iDipDir)
	{
		assert(facet);
		CCVector3 N = facet->getNormal();

		PointCoordinateType dipDir = 0, dip = 0;
		ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);

		iDip = static_cast<unsigned>(floor(dip / angularStep_deg));
		if (iDip == dSteps)
			iDip--;
		iDipDir = static_cast<unsigned>(floor(dipDir / angularStep_deg));
		if (iDipDir == ddSteps)
			iDipDir--;
	}

	static inline QString GetFamilyName(double dip, double dipDir, double angularStep_deg)
	{
		//return QString("(%1+/-%2) / (%3+/-%4)").arg(dip).arg(angularStep_deg/2).arg(dipDir).arg(angularStep_deg/2);
		return QString("%1_%2").arg(dipDir).arg(dip);
	}

	static inline QString GetSubFamilyName(int subFamilyIndex)
	{
		return QString("f%1").arg(subFamilyIndex, 4, 10, QChar('0'));
	}

	//! Subdivides a set of facets with similar orientation
	static bool ProcessFamiliy(ccHObject* parent,
		FacetSet& family,
		unsigned familyIndex,
		unsigned iDip,
		unsigned iDipDir,
		double angularStep_deg,
		double maxDist)
	{
		size_t count = family.size();
		if (count == 0)
			return true;

		double dip = (iDip + 0.5) * angularStep_deg;
		double dipDir = (iDipDir + 0.5) * angularStep_deg;

		QString familyName = GetFamilyName(dip, dipDir, angularStep_deg);

		//create family group
		ccHObject* familyGroup = new ccHObject(QString("F%1_").arg(familyIndex, 2, 10, QChar('0')) + familyName);
		if (parent)
			parent->addChild(familyGroup);

		//tag all facets consequently
		for (FacetSet::iterator it = family.begin(); it != family.end(); ++it)
		{
			(*it)->setMetaData(s_OriFamilyKey, QVariant(static_cast<uint>(familyIndex)));
			(*it)->setMetaData(s_OriFamilyNameKey, QVariant(familyName));
		}

		//now we are going to regroup the facets in sub-families
		unsigned subFamilyIndex = 0;

		if (count == 1 || count == 2)
		{
			//1 or 2 subsets at most!
			family[0]->setMetaData(s_OriSubFamilyKey, QVariant(static_cast<uint>(++subFamilyIndex)));
			ccHObject* subFamilyGroup = new ccHObject(GetSubFamilyName(subFamilyIndex));
			familyGroup->addChild(subFamilyGroup);
			assert(family[0]->getParent() == 0);
			subFamilyGroup->addChild(family[0]);

			if (count == 2)
			{
				PointCoordinateType dist = CommputeHDistBetweenFacets(family[0], family[1]);
				if (dist <= maxDist)
				{
					//same sub-family as #1
					family[1]->setMetaData(s_OriSubFamilyKey, QVariant(static_cast<uint>(subFamilyIndex)));
					assert(family[1]->getParent() == 0);
					subFamilyGroup->addChild(family[1]);
				}
				else
				{
					//new sub-family
					family[1]->setMetaData(s_OriSubFamilyKey, QVariant(static_cast<uint>(++subFamilyIndex)));
					ccHObject* subFamilyGroup2 = new ccHObject(GetSubFamilyName(subFamilyIndex));
					familyGroup->addChild(subFamilyGroup2);
					assert(family[1]->getParent() == 0);
					subFamilyGroup2->addChild(family[1]);
				}
			}
		}
		else
		{
			//create relative distance matrix
			CCLib::SquareMatrixf distMat(static_cast<unsigned>(count));
			if (distMat.isValid())
			{
				for (unsigned it1 = 0; it1 + 1 != count; ++it1)
				{
					for (unsigned it2 = it1 + 1; it2 != count; ++it2)
					{
						PointCoordinateType dist = CommputeHDistBetweenFacets(family.at(it1), family.at(it2));
						distMat.setValue(it1, it2, static_cast<float>(dist));
						distMat.setValue(it2, it1, static_cast<float>(dist));
					}
				}

				//regroup elements
				while (true)
				{
					//we look for the two nearest facets
					std::pair<unsigned, unsigned> bestCouple(0, 0);
					PointCoordinateType bestCoupleDist = -1;
					for (unsigned i = 0; i + 1 < count; ++i)
					{
						if (distMat.getValue(i, i) == 0) //not already assigned
						{
							for (unsigned j = i + 1; j < count; ++j)
							{
								if (distMat.getValue(j, j) == 0) //not already assigned
								{
									if (bestCoupleDist < 0 || bestCoupleDist > distMat.getValue(i, j))
									{
										bestCouple.first = i;
										bestCouple.second = j;
										bestCoupleDist = distMat.getValue(i, j);
									}
								}
							}
						}
					}

					if (bestCoupleDist < 0 || bestCoupleDist > maxDist)
						break; //no more available couples!

					//othrewise we push this couple in the new family set
					std::vector<unsigned> subFamily(2);
					subFamily[0] = bestCouple.first;
					subFamily[1] = bestCouple.second;
					//flag them as 'assigned'
					distMat.setValue(bestCouple.first, bestCouple.first, -1);
					distMat.setValue(bestCouple.second, bestCouple.second, -1);

					//now look for other available facets for this sub-family
					while (true)
					{
						PointCoordinateType bestDist = -1;
						unsigned bestIndex = 0;
						for (unsigned i = 0; i < count; ++i)
						{
							if (distMat.getValue(i, i) == 0) //not already assigned
							{
								PointCoordinateType minDist = -1;
								for (unsigned j = 0; j < subFamily.size(); ++j)
								{
									const PointCoordinateType& dist = distMat.getValue(i, subFamily[j]);
									if (dist < maxDist)
									{
										//still elligible?
										if (minDist < 0 || dist < minDist)
											minDist = dist;
									}
									else
									{
										minDist = -1;
										break;
									}
								} //end of sub-family test

								if (minDist >= 0 && (bestDist < 0 || minDist < bestDist))
								{
									bestDist = minDist;
									bestIndex = i;
								}
							}
						}

						if (bestDist >= 0)
						{
							//add candidate to sub-family
							subFamily.push_back(bestIndex);
							//flag it as 'assigned'
							distMat.setValue(bestIndex, bestIndex, -1);
						}
						else
						{
							break;
						}
					}
					//end of sub-family

					//we can set the class for all the sub-family elements
					{
						++subFamilyIndex;
						ccHObject* subFamilyGroup = new ccHObject(GetSubFamilyName(subFamilyIndex));
						familyGroup->addChild(subFamilyGroup);
						for (unsigned j = 0; j < subFamily.size(); ++j)
						{
							ccFacet* facet = family.at(subFamily[j]);
							facet->setMetaData(s_OriSubFamilyKey, QVariant(static_cast<uint>(subFamilyIndex)));
							assert(facet->getParent() == 0);
							subFamilyGroup->addChild(facet);
						}
					}
				}
				// no more sub-families

				//don't forget to set the class for the remaining isolated elements
				{
					for (unsigned i = 0; i < count; ++i)
					{
						if (distMat.getValue(i, i) == 0) //not already assigned
						{
							ccFacet* facet = family[i];
							assert(facet->getParent() == 0);
							facet->setMetaData(s_OriSubFamilyKey, QVariant(static_cast<uint>(++subFamilyIndex)));
							ccHObject* subFamilyGroup = new ccHObject(GetSubFamilyName(subFamilyIndex));
							familyGroup->addChild(subFamilyGroup);
							subFamilyGroup->addChild(facet);
						}
					}
				}

			}
			else
			{
				//not enough memory
				for (unsigned i = 0; i < count; ++i)
					familyGroup->addChild(family[i]);
				return false;
			}
		}

		//eventually we set the right colors for each facet
		unsigned subFamilyCount = subFamilyIndex;
		if (subFamilyCount)
		{
			assert(subFamilyCount > 0);
			ccColor::Rgb col, darkCol;
			for (unsigned i = 0; i < count; ++i)
			{
				ccFacet* facet = family.at(i);
				subFamilyIndex = facet->getMetaData(s_OriSubFamilyKey).toUInt();
				GenerateSubfamilyColor(col, dip, dipDir, subFamilyIndex, subFamilyCount, &darkCol);
				facet->setColor(col);
				if (facet->getContour())
				{
					facet->getContour()->setColor(darkCol);
					facet->getContour()->setWidth(2);
				}
			}
		}

		return true;
	}

	//! Classifies the facets based on their orientation
	static bool ByOrientation(ccHObject* facetGroup, double angularStep_deg, double maxDist)
	{
		assert(facetGroup && angularStep_deg > 0 && maxDist >= 0);

		ccHObject::Container facets;
		if (facetGroup)
			facetGroup->filterChildren(facets, true, CC_TYPES::FACET);

		size_t facetCount = facets.size();
		if (facetCount == 0)
		{
			//nothing to do
			return true;
		}

		//remove all facets from the group first (without deleting them!)
		{
			for (size_t i = 0; i < facetCount; ++i)
			{
				ccFacet* facet = static_cast<ccFacet*>(facets[i]);
				assert(facet && facet->getParent());
				facet->getParent()->removeDependencyWith(facet);
				facet->getParent()->removeChild(facet);
			}
			//and remove all remaining children from the input group!
			facetGroup->removeAllChildren();
		}

		//dip steps (dip in [0,90])
		unsigned dSteps = static_cast<unsigned>(ceil(90.0 / angularStep_deg));
		//dip direction steps (dip dir. in [0,360])
		unsigned ddSteps = static_cast<unsigned>(ceil(360.0 / angularStep_deg));

		bool error = false;
		if (facetCount == 1)
		{
			ccFacet* facet = static_cast<ccFacet*>(facets.front());

			//unique facet = unique family
			FacetSet family(1, facet);

			unsigned iDip = 0, iDipDir = 0;
			GetFamilyIndexes(facet, dSteps, ddSteps, angularStep_deg, iDip, iDipDir);

			error = !ProcessFamiliy(facetGroup, family, 1, iDip, iDipDir, angularStep_deg, maxDist);
		}
		else
		{
			unsigned gridSize = dSteps * ddSteps;

			//grid to store all orientation 'families'
			FacetSet** grid = new FacetSet*[gridSize];
			if (!grid)
			{
				//not enough memory
				error = true;
			}
			else
			{
				memset(grid, 0, sizeof(FacetSet*)*gridSize);

				QProgressDialog pDlg("Families classification", QString(), 0, static_cast<int>(facetCount));
				pDlg.show();
				QApplication::processEvents();

				//project each facet in grid
				unsigned setCount = 0;
				for (size_t i = 0; i < facetCount; ++i)
				{
					ccFacet* facet = static_cast<ccFacet*>(facets[i]);

					unsigned iDip = 0, iDipDir = 0;
					GetFamilyIndexes(facet, dSteps, ddSteps, angularStep_deg, iDip, iDipDir);

					unsigned facetIndex = iDipDir + iDip * ddSteps;
					assert(facetIndex < gridSize);
					FacetSet* set = grid[facetIndex];
					if (!set)
					{
						set = new FacetSet;
						if (!set)
						{
							//not enough memory
							error = true;
							break;
						}
						grid[iDipDir + iDip * ddSteps] = set;
						++setCount;
					}

					try
					{
						set->push_back(facet);
					}
					catch (const std::bad_alloc&)
					{
						//not enough memory
						error = true;
						break;
					}

					pDlg.setValue(static_cast<int>(i));
				}

				if (!error)
				{
					unsigned familyIndex = 0;

					QProgressDialog pDlg("Sub-families classification", QString(), 0, static_cast<int>(setCount));
					pDlg.show();
					QApplication::processEvents();

					//now scan the grid and tag all 'families'
					FacetSet** set = grid;
					int progress = 0;
					for (unsigned j = 0; j < dSteps; ++j)
					{
						for (unsigned i = 0; i < ddSteps; ++i, ++set)
						{
							//new family?
							if (*set)
							{
								error = !ProcessFamiliy(facetGroup, **set, ++familyIndex, j, i, angularStep_deg, maxDist);
								if (error)
								{
									//stop the process
									//i = ddSteps;
									j = dSteps;
									break;
								}

								pDlg.setValue(++progress);
							}
						}
					}
				}
			}

			//release grid
			{
				for (unsigned i = 0; i < gridSize; ++i)
					if (grid[i])
						delete grid[i];
				delete[] grid;
				grid = 0;
			}
		}

		//check that no parent-less facets remain!
		{
			for (size_t i = 0; i < facetCount; ++i)
			{
				ccFacet* facet = static_cast<ccFacet*>(facets[i]);
				assert(facet->getParent() || error);
				if (!facet->getParent())
					facetGroup->addChild(facet);
			}
		}

		//update associated display to all children as the whole structure may have changed!
		facetGroup->setDisplay_recursive(facetGroup->getDisplay());

		return !error;
	}
};

#endif //QFACET_FACETS_CLASSIFIER_HEADER
