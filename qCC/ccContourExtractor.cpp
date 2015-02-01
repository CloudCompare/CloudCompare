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

#include "ccContourExtractor.h"

//local
#include "ccContourExtractorDlg.h"

//qCC_db
#include <ccLog.h>
#include <ccPointCloud.h>
#include <cc2DLabel.h>

//qCC_gl
#include <ccGLWindow.h>

//CCLib
#include <Neighbourhood.h>
#include <DistanceComputationTools.h>
#include <PointProjectionTools.h>

//System
#include <assert.h>
#include <math.h>

//list of already used point to avoid hull's inner loops
enum HullPointFlags {	POINT_NOT_USED	= 0,
						POINT_USED		= 1,
						POINT_IGNORED	= 2,
						POINT_FROZEN	= 3,
};

bool ccContourExtractor::ExtractConcaveHull2D(	std::vector<CCLib::PointProjectionTools::IndexedCCVector2>& points,
												std::list<CCLib::PointProjectionTools::IndexedCCVector2*>& hullPoints,
												ContourType contourType,
												PointCoordinateType maxSquareEdgeLength/*=0*/,
												bool enableVisualDebugMode/*=false*/)
{
	//first compute the Convex hull
	if (!CCLib::PointProjectionTools::extractConvexHull2D(points,hullPoints))
		return false;

	//do we really need to compute the concave hull?
	if (hullPoints.size() < 2 || maxSquareEdgeLength < 0)
		return true;

	unsigned pointCount = static_cast<unsigned>(points.size());

	std::vector<HullPointFlags> pointFlags;
	try
	{
		pointFlags.resize(pointCount,POINT_NOT_USED);
	}
	catch(...)
	{
		//not enough memory
		return false;
	}

	//hack: compute the theoretical 'minimal' edge length
	PointCoordinateType minSquareEdgeLength = 0;
	{
		CCVector2 minP,maxP;
		for (size_t i=0; i<pointCount; ++i)
		{
			const CCLib::PointProjectionTools::IndexedCCVector2& P = points[i];
			if (i)
			{
				minP.x = std::min(P.x,minP.x);
				minP.y = std::min(P.y,minP.y);
				maxP.x = std::max(P.x,maxP.x);
				maxP.y = std::max(P.y,maxP.y);
			}
			else
			{
				minP = maxP = P;
			}
		}
		minSquareEdgeLength = (maxP-minP).norm2() / static_cast<PointCoordinateType>(1.0e7); //10^-7 of the max bounding rectangle side
		minSquareEdgeLength = std::min(minSquareEdgeLength, maxSquareEdgeLength/10);

		//we remove very small edges
		for (std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::iterator itA = hullPoints.begin(); itA != hullPoints.end(); ++itA)
		{
			std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::iterator itB = itA; ++itB;
			if (itB == hullPoints.end())
				itB = hullPoints.begin();
			if ((**itB-**itA).norm2() < minSquareEdgeLength)
			{
				pointFlags[(*itB)->index] = POINT_FROZEN;
				hullPoints.erase(itB);
			}
		}

		if (contourType != FULL)
		{
			//we will now try to determine which part of the contour is the 'upper' one and which one is the 'lower' one

			//search for the min and max vertices
			std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::iterator itLeft = hullPoints.begin();
			std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::iterator itRight = hullPoints.begin();
			{
				for (std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::iterator it = hullPoints.begin(); it != hullPoints.end(); ++it)
				{
					if ((*it)->x < (*itLeft)->x || ((*it)->x == (*itLeft)->x && (*it)->y < (*itLeft)->y))
					{
						itLeft = it;
					}
					if ((*it)->x > (*itRight)->x || ((*it)->x == (*itRight)->x && (*it)->y < (*itRight)->y))
					{
						itRight = it;
					}
				}
			}
			assert(itLeft != itRight);
			//find the right way to go
			{
				std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::iterator itBefore = itLeft;
				if (itBefore == hullPoints.begin()) itBefore = hullPoints.end(); --itBefore;
				std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::iterator itAfter = itLeft; ++itAfter;
				if (itAfter == hullPoints.end()) itAfter = hullPoints.begin();

				bool forward = (cross(**itLeft, **itBefore, **itAfter) < 0 && contourType == LOWER);
				if (!forward)
					std::swap(itLeft,itRight);
			}

			//copy the right part
			std::list<CCLib::PointProjectionTools::IndexedCCVector2*> halfHullPoints;
			try
			{
				for (std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::iterator it = itLeft; ; ++it)
				{
					if (it == hullPoints.end())
						it = hullPoints.begin();
					halfHullPoints.push_back(*it);
					if (it == itRight)
						break;
				}
			}
			catch (std::bad_alloc)
			{
				//not enough memory
				return false;
			}
			//replace the input hull by the selected part
			hullPoints = halfHullPoints;
		}

		if (hullPoints.size() < 2)
		{
			//no more edges?!
			return false;
		}
	}


	//DEBUG MECHANISM
	ccContourExtractorDlg debugDialog;
	ccPointCloud* debugCloud = 0;
	ccPolyline* debugContour = 0;
	ccPointCloud* debugContourVertices = 0;
	if (enableVisualDebugMode)
	{
		debugDialog.init();
		debugDialog.setGeometry(50,50,800,600);
		debugDialog.show();

		//create point cloud with all (2D) input points
		{
			debugCloud = new ccPointCloud;
			debugCloud->reserve(pointCount);
			for (size_t i=0; i<pointCount; ++i)
			{
				const CCLib::PointProjectionTools::IndexedCCVector2& P = points[i];
				debugCloud->addPoint(CCVector3(P.x,P.y,0));
			}
			debugCloud->setPointSize(3);
			debugDialog.addToDisplay(debugCloud,false); //the window will take care of deleting this entity!
		}

		//create polyline
		{
			debugContourVertices = new ccPointCloud;
			debugContour = new ccPolyline(debugContourVertices);
			debugContour->addChild(debugContourVertices);
			unsigned hullSize = static_cast<unsigned>(hullPoints.size());
			debugContour->reserve(hullSize);
			debugContourVertices->reserve(hullSize);
			unsigned index = 0;
			for (std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::iterator itA = hullPoints.begin(); itA != hullPoints.end(); ++itA, ++index)
			{
				const CCLib::PointProjectionTools::IndexedCCVector2* P = *itA;
				debugContourVertices->addPoint(CCVector3(P->x,P->y,0));
				debugContour->addPointIndex(index/*(*itA)->index*/);
			}
			debugContour->setColor(ccColor::red);
			debugContourVertices->setVisible(false);
			debugContour->setClosed(contourType == FULL);
			debugDialog.addToDisplay(debugContour,false); //the window will take care of deleting this entity!
		}

		//set zoom
		{
			ccBBox box = debugCloud->getOwnBB();
			debugDialog.zoomOn(box);
		}
		debugDialog.refresh();
	}

	//we repeat the process until nothing changes!
	bool somethingHasChanged = true;
	//while (somethingHasChanged) //DGM: doesn't seem necessary
	{
		somethingHasChanged = false;

		//reset point flags
		for (size_t i=0; i<pointCount; ++i)
		{
			if (pointFlags[i] != POINT_FROZEN)
				pointFlags[i] = POINT_NOT_USED;
		}
		
		//build the initial edge list & flag the convex hull points
		std::list<std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::iterator> edges;
		{
			for (std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::iterator itA = hullPoints.begin(); itA != hullPoints.end(); ++itA)
			{
				try
				{
					edges.push_back(itA);
				}
				catch(...)
				{
					//not enough memory
					return false;
				}
				pointFlags[(*itA)->index] = POINT_USED;
			}
		}

		//we look for edges that are longer than the maximum specified length
		while (!edges.empty())
		{
			//current edge (AB)
			std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::iterator itA = edges.front();
			edges.pop_front();

			std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::iterator itB = itA; ++itB;
			if (itB == hullPoints.end())
			{
				if (contourType == FULL)
					itB = hullPoints.begin();
				else
					continue;
			}

			//create label
			cc2DLabel* edgeLabel = 0;
			if (enableVisualDebugMode && !debugDialog.isSkipepd())
			{
				edgeLabel = new cc2DLabel("edge");
				unsigned indexA = 0;
				unsigned indexB = 0;
				for (size_t i=0; i<pointCount; ++i)
				{
					const CCLib::PointProjectionTools::IndexedCCVector2& P = points[i];
					if (&P == *itA)
						indexA = static_cast<unsigned>(i);
					if (&P == *itB)
						indexB = static_cast<unsigned>(i);
				}
				edgeLabel->addPoint(debugCloud,indexA);
				edgeLabel->addPoint(debugCloud,indexB);
				edgeLabel->setVisible(true);
				edgeLabel->setDisplayedIn2D(false);
				debugDialog.addToDisplay(edgeLabel);
				debugDialog.refresh();
			}

			//long edge?
			PointCoordinateType squareLengthAB = (**itB-**itA).norm2();
			if (squareLengthAB > maxSquareEdgeLength)
			{
				//look for the nearest point in the input set
				PointCoordinateType minDist2 = -1;
				unsigned minIndex = 0;
				for (unsigned i=0; i<pointCount; ++i)
				{
					const CCLib::PointProjectionTools::IndexedCCVector2& P = points[i];
					if (pointFlags[P.index] != POINT_NOT_USED)
						continue;

					//skip the edge vertices!
					if (P.index == (*itA)->index || P.index == (*itB)->index)
						continue;

					//we only consider 'inner' points
					if (cross(**itA, **itB, P) < 0)
					{
						continue;
					}

					PointCoordinateType dist2 = CCLib::DistanceComputationTools::ComputeSquareDistToSegment(P,**itA,**itB,true);
					if (dist2 >= 0 && (minDist2 < 0 || dist2 < minDist2))
					{
						minDist2 = dist2;
						minIndex = i;
					}
				}

				//if we have found a candidate
				if (minDist2 >= 0)
				{
					const CCLib::PointProjectionTools::IndexedCCVector2& P = points[minIndex];

					//create label
					cc2DLabel* label = 0;
					if (enableVisualDebugMode && !debugDialog.isSkipepd())
					{
						label = new cc2DLabel("nearest point");
						label->addPoint(debugCloud,minIndex);
						label->setVisible(true);
						label->setSelected(true);
						debugDialog.addToDisplay(label);
						debugDialog.displayMessage(QString("nearest point found index #%1 (dist = %2)").arg(minIndex).arg(sqrt(minDist2)),true);
					}

					assert(pointFlags[P.index] == POINT_NOT_USED); //we don't consider already used points!

					CCVector2 AP = (P-**itA);
					CCVector2 PB = (**itB-P);
					PointCoordinateType squareLengthAP = AP.norm2();
					PointCoordinateType squareLengthPB = PB.norm2();
					//check that we don't create too small edges!
					if (squareLengthAP < minSquareEdgeLength || squareLengthPB < minSquareEdgeLength)
					{
						pointFlags[P.index] = POINT_IGNORED;
						edges.push_front(itA); //retest the edge!
						if (enableVisualDebugMode)
							debugDialog.displayMessage("nearest point is too close!",true);
					}
					//at least one of the new segments must be smaller than the initial one!
					else if ( squareLengthAP < squareLengthAB || squareLengthPB < squareLengthAB )
					{
						//now check that the point is not nearer to the neighbor edges
						//DGM: only if the edge could 'need' it!

						//next edge vertex (BC)
						std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::iterator itC = itB; ++itC;
						if (itC == hullPoints.end())
						{
							if (contourType == FULL)
								itC = hullPoints.begin();
						}

						PointCoordinateType dist2ToRight = -1;

						if (itC != hullPoints.end())
						{
							CCVector2 BC = (**itC-**itB);
							PointCoordinateType squareLengthBC = BC.norm2();
							if (squareLengthBC > maxSquareEdgeLength)
								dist2ToRight = CCLib::DistanceComputationTools::ComputeSquareDistToSegment(P,**itB,**itC,true);
						}

						if (dist2ToRight < 0 || minDist2 <= dist2ToRight)
						{
							//previous edge vertex (OA)
							std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::iterator itO = itA;
							if (itO == hullPoints.begin())
							{
								itO = hullPoints.end();
								if (contourType == FULL)
									--itO;
							}
							else
							{
								--itO;
							}

							PointCoordinateType dist2ToLeft = -1;

							if (itO != hullPoints.end())
							{
								CCVector2 OA = (**itA-**itO);
								PointCoordinateType squareLengthOA = OA.norm2();
								if (squareLengthOA > maxSquareEdgeLength)
									dist2ToLeft = CCLib::DistanceComputationTools::ComputeSquareDistToSegment(P,**itO,**itA,true);
							}

							if (dist2ToLeft < 0 || minDist2 <= dist2ToLeft)
							{
								//last check: the new segments must not intersect with the actual hull!
								bool intersect = false;
								{
									for (std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::iterator itI = hullPoints.begin(); itI != hullPoints.end(); ++itI)
									{
										std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::iterator itJ = itI; ++itJ;
										if (itJ == hullPoints.end())
										{
											if (contourType == FULL)
												itJ = hullPoints.begin();
											else
												continue;
										}

										//we avoid testing with already connected segments!
										if (	(*itI)->index == (*itA)->index
											||	(*itJ)->index == (*itA)->index
											||	(*itI)->index == (*itB)->index
											||	(*itJ)->index == (*itB)->index )
											continue;

										if (	CCLib::PointProjectionTools::segmentIntersect(**itI,**itJ,**itA,P)
											||	CCLib::PointProjectionTools::segmentIntersect(**itI,**itJ,P,**itB) )
										{
											intersect = true;
											break;
										}
									}
								}
								if (!intersect)
								{
									hullPoints.insert(itB == hullPoints.begin() ? hullPoints.end() : itB, &points[minIndex]);
									somethingHasChanged = true;

									if (enableVisualDebugMode && !debugDialog.isSkipepd())
									{
										if (debugContour)
										{
											debugContourVertices->clear();
											unsigned hullSize = static_cast<unsigned>(hullPoints.size());
											debugContourVertices->reserve(hullSize);
											unsigned index = 0;
											for (std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::iterator it = hullPoints.begin(); it != hullPoints.end(); ++it, ++index)
											{
												const CCLib::PointProjectionTools::IndexedCCVector2* P = *it;
												debugContourVertices->addPoint(CCVector3(P->x,P->y,0));
											}
											debugContour->reserve(hullSize);
											debugContour->addPointIndex(hullSize-1);
											debugDialog.refresh();
										}
										debugDialog.displayMessage("point has been added to contour",true);
									}

									//we'll inspect the two new segments later
									try
									{
										if (squareLengthAP > maxSquareEdgeLength)
										{
											edges.push_front(itA);
										}
										if (squareLengthPB > maxSquareEdgeLength)
										{
											std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::iterator itP = itA; ++itP;
											edges.push_front(itP);
										}
									}
									catch(...)
									{
										//not enough memory
										return false;
									}

									//we won't use P anymore!
									pointFlags[P.index] = POINT_USED;
								}
								else
								{
									if (enableVisualDebugMode)
										debugDialog.displayMessage("[rejected] new edge would intersect the current contour!",true);
								}
							}
							else
							{
								if (enableVisualDebugMode)
									debugDialog.displayMessage("[rejected] point is closer to left edge!",true);
							}
						}
						else
						{
							if (enableVisualDebugMode)
								debugDialog.displayMessage("[rejected] point is closer to right edge!",true);
						}
					}
					else
					{
						if (enableVisualDebugMode)
							debugDialog.displayMessage("[rejected] created edges would be longer!",true);
					}

					//remove label
					if (label)
					{
						assert(enableVisualDebugMode);
						debugDialog.removFromDisplay(label);
						delete label;
						label = 0;
						//debugDialog.refresh();
					}

				} //end of candidate examination
				else
				{
					if (enableVisualDebugMode)
						debugDialog.displayMessage("no candidate found",true);
				}
			} //end of current edge examination
			else
			{
				//if (enableVisualDebugMode)
				//	debugDialog.displayMessage("[edge rejected] edge is too small",true);
			}

			if (edgeLabel)
			{
				assert(enableVisualDebugMode);
				debugDialog.removFromDisplay(edgeLabel);
				delete edgeLabel;
				edgeLabel = 0;
				//debugDialog.refresh();
			}

		} //no more edges
	}

	return true;
}

typedef std::list<CCLib::PointProjectionTools::IndexedCCVector2*> Hull2D;

ccPolyline* ccContourExtractor::ExtractFlatContour(	CCLib::GenericIndexedCloudPersist* points,
													PointCoordinateType maxEdgeLength/*=0*/,
													const PointCoordinateType* preferredNormDim/*=0*/,
													const PointCoordinateType* preferredUpDir/*=0*/,
													ContourType contourType/*=FULL*/,
													std::vector<unsigned>* originalPointIndexes/*=0*/,
													bool enableVisualDebugMode/*=false*/)
{
	assert(points);
	if (!points)
		return 0;
	unsigned ptsCount = points->size();
	if (ptsCount < 3)
		return 0;

	CCLib::Neighbourhood Yk(points);
	CCVector3 O,X,Y; //local base
	bool useOXYasBase = false;

	//we project the input points on a plane
	std::vector<CCLib::PointProjectionTools::IndexedCCVector2> points2D;
	PointCoordinateType* planeEq = 0;
	//if the user has specified a default direction, we'll use it as 'projecting plane'
	PointCoordinateType preferredPlaneEq[4] = {0, 0, 0, 0};
	if (preferredNormDim != 0)
	{
		const CCVector3* G = points->getPoint(0); //any point through which the point passes is ok
		preferredPlaneEq[0] = preferredNormDim[0];
		preferredPlaneEq[1] = preferredNormDim[1];
		preferredPlaneEq[2] = preferredNormDim[2];
		CCVector3::vnormalize(preferredPlaneEq);
		preferredPlaneEq[3] = CCVector3::vdot(G->u,preferredPlaneEq);
		planeEq = preferredPlaneEq;

		if (preferredUpDir != 0)
		{
			O = *G;
			Y = CCVector3(preferredUpDir);
			X = Y.cross(CCVector3(preferredNormDim));
			useOXYasBase = true;
		}
	}

	if (!Yk.projectPointsOn2DPlane<CCLib::PointProjectionTools::IndexedCCVector2>(points2D,planeEq,&O,&X,&Y,useOXYasBase))
	{
		ccLog::Warning("[ExtractFlatContour] Failed to project the points on the LS plane (not enough memory?)!");
		return 0;
	}

	//update the points indexes (not done by Neighbourhood::projectPointsOn2DPlane)
	{
		for (unsigned i=0; i<ptsCount; ++i)
			points2D[i].index = i;
	}

	//try to get the points on the convex/concave hull to build the contour and the polygon
	Hull2D hullPoints;
	if (!ExtractConcaveHull2D(	points2D,
								hullPoints,
								contourType,
								maxEdgeLength*maxEdgeLength,
								enableVisualDebugMode) )
	{
		ccLog::Warning("[ExtractFlatContour] Failed to compute the convex hull of the input points!");
		return 0;
	}

	if (originalPointIndexes)
	{
		try
		{
			originalPointIndexes->resize(hullPoints.size(),0);
		}
		catch(std::bad_alloc)
		{
			//not enough memory
			ccLog::Error("[ExtractFlatContour] Not enough memory!");
			return 0;
		}

		unsigned i=0;
		for (Hull2D::const_iterator it = hullPoints.begin(); it != hullPoints.end(); ++it, ++i)
			(*originalPointIndexes)[i] = (*it)->index;
	}

	unsigned hullPtsCount = static_cast<unsigned>(hullPoints.size());

	//create vertices
	ccPointCloud* contourVertices = new ccPointCloud();
	{
		if (!contourVertices->reserve(hullPtsCount))
		{
			delete contourVertices;
			contourVertices = 0;
			ccLog::Error("[ExtractFlatContour] Not enough memory!");
			return 0;
		}

		//projection on the LS plane (in 3D)
		for (Hull2D::const_iterator it = hullPoints.begin(); it != hullPoints.end(); ++it)
			contourVertices->addPoint(O + X*(*it)->x + Y*(*it)->y);
		contourVertices->setName("vertices");
		contourVertices->setEnabled(false);
	}

	//we create the corresponding (3D) polyline
	ccPolyline* contourPolyline = new ccPolyline(contourVertices);
	if (contourPolyline->reserve(hullPtsCount))
	{
		contourPolyline->addPointIndex(0,hullPtsCount);
		contourPolyline->setClosed(contourType == FULL);
		contourPolyline->setVisible(true);
		contourPolyline->setName("contour");
		contourPolyline->addChild(contourVertices);
	}
	else
	{
		delete contourPolyline;
		contourPolyline = 0;
		ccLog::Warning("[ExtractFlatContour] Not enough memory to create the contour polyline!");
	}

	return contourPolyline;
}

bool ccContourExtractor::ExtractFlatContour(CCLib::GenericIndexedCloudPersist* points,
											PointCoordinateType maxEdgeLength,
											std::vector<ccPolyline*>& parts,
											bool allowSplitting/*=true*/,
											const PointCoordinateType* preferredDim/*=0*/,
											bool enableVisualDebugMode/*=false*/)
{
	parts.clear();

	//extract whole contour
	ccPolyline* basePoly = ExtractFlatContour(points,maxEdgeLength,preferredDim,0,FULL,0,enableVisualDebugMode);
	if (!basePoly)
	{
		return false;
	}
	else if (!allowSplitting)
	{
		parts.push_back(basePoly);
		return true;
	}

	//and split it if necessary
	bool success = basePoly->split(maxEdgeLength,parts);

	delete basePoly;
	basePoly = 0;

	return success;

}
