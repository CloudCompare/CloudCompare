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

#include "ccContourExtractor.h"

//local
#include "ccContourExtractorDlg.h"

//qCC_db
#include <cc2DLabel.h>
#include <ccLog.h>
#include <ccPointCloud.h>

//qCC_gl
#include <ccGLWindow.h>

//CCLib
#include <DistanceComputationTools.h>
#include <Neighbourhood.h>
#include <PointProjectionTools.h>

#ifdef USE_TBB
#include <tbb/parallel_for.h>
#endif

//System
#include <cassert>
#include <cmath>
#include <set>

//list of already used point to avoid hull's inner loops
enum HullPointFlags {	POINT_NOT_USED	= 0,
						POINT_USED		= 1,
						POINT_IGNORED	= 2,
						POINT_FROZEN	= 3,
};

using Vertex2D = CCLib::PointProjectionTools::IndexedCCVector2;
using VertexIterator = std::list<Vertex2D *>::iterator;
using ConstVertexIterator = std::list<Vertex2D *>::const_iterator;

namespace 
{
	struct Edge
	{
		Edge() : nearestPointIndex(0), nearestPointSquareDist(-1.0f) {}
		
		Edge(const VertexIterator& A, unsigned _nearestPointIndex, float _nearestPointSquareDist)
			: itA(A)
			, nearestPointIndex(_nearestPointIndex)
			, nearestPointSquareDist(_nearestPointSquareDist)
		{}
		
		//operator
		inline bool operator< (const Edge& e) const { return nearestPointSquareDist < e.nearestPointSquareDist; }
		
		VertexIterator itA;
		unsigned nearestPointIndex;
		float nearestPointSquareDist;
	};
}

//! Finds the nearest (available) point to an edge
/** \return The nearest point distance (or -1 if no point was found!)
**/
static PointCoordinateType FindNearestCandidate(unsigned& minIndex,
												const VertexIterator& itA,
												const VertexIterator& itB,
												const std::vector<Vertex2D>& points,
												const std::vector<HullPointFlags>& pointFlags,
												PointCoordinateType minSquareEdgeLength,
												bool allowLongerChunks = false,
												double minCosAngle = -1.0)
{
	//look for the nearest point in the input set
	PointCoordinateType minDist2 = -1;
	const CCVector2 AB = **itB-**itA;
	const PointCoordinateType squareLengthAB = AB.norm2();
	const unsigned pointCount = static_cast<unsigned>(points.size());

#ifdef USE_TBB
	tbb::parallel_for( static_cast<unsigned int>(0), pointCount, [&](unsigned int i) {
		const Vertex2D& P = points[i];
		if (pointFlags[P.index] != POINT_NOT_USED)
			return;

		//skip the edge vertices!
		if (P.index == (*itA)->index || P.index == (*itB)->index)
		{
			return;
		}

		//we only consider 'inner' points
		const CCVector2 AP = P-**itA;
		if (AB.x * AP.y - AB.y * AP.x < 0)
		{
			return;
		}

		//check the angle
		if (minCosAngle > -1.0)
		{
			const CCVector2 PB = **itB - P;
			const PointCoordinateType dotProd = AP.x * PB.x + AP.y * PB.y;
			const PointCoordinateType minDotProd = static_cast<PointCoordinateType>(minCosAngle * std::sqrt(AP.norm2() * PB.norm2()));
			if (dotProd < minDotProd)
			{
				return;
			}
		}

		const PointCoordinateType dot = AB.dot(AP); // = cos(PAB) * ||AP|| * ||AB||
		if (dot >= 0 && dot <= squareLengthAB)
		{
			const CCVector2 HP = AP - AB * (dot / squareLengthAB);
			const PointCoordinateType dist2 = HP.norm2();
			if (minDist2 < 0 || dist2 < minDist2)
			{
				//the 'nearest' point must also be a valid candidate
				//(i.e. at least one of the created edges is smaller than the original one
				//and we don't create too small edges!)
				const PointCoordinateType squareLengthAP = AP.norm2();
				const PointCoordinateType squareLengthBP = (P-**itB).norm2();
				if (	squareLengthAP >= minSquareEdgeLength
					&&	squareLengthBP >= minSquareEdgeLength
					&&	(allowLongerChunks || (squareLengthAP < squareLengthAB || squareLengthBP < squareLengthAB))
					)
				{
					minDist2 = dist2;
					minIndex = i;
				}
			}
		}
	} );
#else
	for (unsigned i = 0; i < pointCount; ++i)
	{
		const Vertex2D& P = points[i];
		if (pointFlags[P.index] != POINT_NOT_USED)
			continue;

		//skip the edge vertices!
		if (P.index == (*itA)->index || P.index == (*itB)->index)
		{
			continue;
		}

		//we only consider 'inner' points
		CCVector2 AP = P - **itA;
		if (AB.x * AP.y - AB.y * AP.x < 0)
		{
			continue;
		}

		//check the angle
		if (minCosAngle > -1.0)
		{
			CCVector2 PB = **itB - P;
			PointCoordinateType dotProd = AP.x * PB.x + AP.y * PB.y;
			PointCoordinateType minDotProd = static_cast<PointCoordinateType>(minCosAngle * std::sqrt(AP.norm2() * PB.norm2()));
			if (dotProd < minDotProd)
			{
				continue;
			}
		}

		PointCoordinateType dot = AB.dot(AP); // = cos(PAB) * ||AP|| * ||AB||
		if (dot >= 0 && dot <= squareLengthAB)
		{
			CCVector2 HP = AP - AB * (dot / squareLengthAB);
			PointCoordinateType dist2 = HP.norm2();
			if (minDist2 < 0 || dist2 < minDist2)
			{
				//the 'nearest' point must also be a valid candidate
				//(i.e. at least one of the created edges is smaller than the original one
				//and we don't create too small edges!)
				PointCoordinateType squareLengthAP = AP.norm2();
				PointCoordinateType squareLengthBP = (P - **itB).norm2();
				if (	squareLengthAP >= minSquareEdgeLength
					&&	squareLengthBP >= minSquareEdgeLength
					&&	(allowLongerChunks || (squareLengthAP < squareLengthAB || squareLengthBP < squareLengthAB))
					)
				{
					minDist2 = dist2;
					minIndex = i;
				}
			}
		}
	}
#endif
	
	return (minDist2 < 0 ? minDist2 : minDist2/squareLengthAB);
}

bool ccContourExtractor::ExtractConcaveHull2D(	std::vector<Vertex2D>& points,
												std::list<Vertex2D*>& hullPoints,
												ContourType contourType,
												bool allowMultiPass,
												PointCoordinateType maxSquareEdgeLength/*=0*/,
												bool enableVisualDebugMode/*=false*/,
												double maxAngleDeg/*=0.0*/)
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
		pointFlags.resize(pointCount, POINT_NOT_USED);
	}
	catch(...)
	{
		//not enough memory
		return false;
	}

	double minCosAngle = maxAngleDeg <= 0 ? -1.0 : std::cos(maxAngleDeg * M_PI / 180.0);

	//hack: compute the theoretical 'minimal' edge length
	PointCoordinateType minSquareEdgeLength = 0;
	{
		CCVector2 minP;
		CCVector2 maxP;
		for (size_t i=0; i<pointCount; ++i)
		{
			const Vertex2D& P = points[i];
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
		minSquareEdgeLength = (maxP - minP).norm2() / static_cast<PointCoordinateType>(1.0e7); //10^-7 of the max bounding rectangle side
		minSquareEdgeLength = std::min(minSquareEdgeLength, maxSquareEdgeLength / 10);

		//we remove very small edges
		for (VertexIterator itA = hullPoints.begin(); itA != hullPoints.end(); ++itA)
		{
			VertexIterator itB = itA; ++itB;
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
			VertexIterator itLeft = hullPoints.begin();
			VertexIterator itRight = hullPoints.begin();
			{
				for (VertexIterator it = hullPoints.begin(); it != hullPoints.end(); ++it)
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
				VertexIterator itBefore = itLeft;
				if (itBefore == hullPoints.begin())
					itBefore = hullPoints.end(); --itBefore;
				VertexIterator itAfter = itLeft; ++itAfter;
				if (itAfter == hullPoints.end())
					itAfter = hullPoints.begin();

				bool forward = ((**itBefore - **itLeft).cross(**itAfter - **itLeft) < 0 && contourType == LOWER);
				if (!forward)
					std::swap(itLeft,itRight);
			}

			//copy the right part
			std::list<Vertex2D*> halfHullPoints;
			try
			{
				for (VertexIterator it = itLeft; ; ++it)
				{
					if (it == hullPoints.end())
						it = hullPoints.begin();
					halfHullPoints.push_back(*it);
					if (it == itRight)
						break;
				}
			}
			catch (const std::bad_alloc&)
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
	ccPointCloud* debugCloud = nullptr;
	ccPolyline* debugContour = nullptr;
	ccPointCloud* debugContourVertices = nullptr;
	
	if (enableVisualDebugMode)
	{
		debugDialog.init();
		debugDialog.setGeometry(50, 50, 800, 600);
		debugDialog.show();

		//create point cloud with all (2D) input points
		{
			debugCloud = new ccPointCloud;
			debugCloud->reserve(pointCount);
			for (size_t i = 0; i < pointCount; ++i)
			{
				const Vertex2D& P = points[i];
				debugCloud->addPoint(CCVector3(P.x, P.y, 0));
			}
			debugCloud->setPointSize(3);
			debugDialog.addToDisplay(debugCloud, false); //the window will take care of deleting this entity!
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
			for (VertexIterator itA = hullPoints.begin(); itA != hullPoints.end(); ++itA, ++index)
			{
				const Vertex2D* P = *itA;
				debugContourVertices->addPoint(CCVector3(P->x, P->y, 0));
				debugContour->addPointIndex(index/*(*itA)->index*/);
			}
			debugContour->setColor(ccColor::red);
			debugContourVertices->setEnabled(false);
			debugContour->setClosed(contourType == FULL);
			debugDialog.addToDisplay(debugContour, false); //the window will take care of deleting this entity!
		}

		//set zoom
		{
			ccBBox box = debugCloud->getOwnBB();
			debugDialog.zoomOn(box);
		}
		debugDialog.refresh();
	}

	//Warning: high STL containers usage ahead ;)
	unsigned step = 0;
	bool somethingHasChanged = true;
	while (somethingHasChanged)
	{
		try
		{
			somethingHasChanged = false;
			++step;

			//reset point flags
			//for (size_t i=0; i<pointCount; ++i)
			//{
			//	if (pointFlags[i] != POINT_FROZEN)
			//		pointFlags[i] = POINT_NOT_USED;
			//}

			//build the initial edge list & flag the convex hull points
			std::multiset<Edge> edges;
			//initial number of edges
			assert(hullPoints.size() >= 2);
			size_t initEdgeCount = hullPoints.size();
			if (contourType != FULL)
				--initEdgeCount;

			VertexIterator itB = hullPoints.begin();
			for (size_t i = 0; i < initEdgeCount; ++i)
			{
				VertexIterator itA = itB; ++itB;
				if (itB == hullPoints.end())
					itB = hullPoints.begin();

				//we will only process the edges that are longer than the maximum specified length
				if ((**itB - **itA).norm2() > maxSquareEdgeLength)
				{
					unsigned nearestPointIndex = 0;
					PointCoordinateType minSquareDist = FindNearestCandidate(
						nearestPointIndex,
						itA,
						itB,
						points,
						pointFlags,
						minSquareEdgeLength,
						step > 1,
						minCosAngle);

					if (minSquareDist >= 0)
					{
						Edge e(itA, nearestPointIndex, minSquareDist);
						edges.insert(e);
					}
				}

				pointFlags[(*itA)->index] = POINT_USED;
			}

			//flag the last vertex as well for non closed contours!
			if (contourType != FULL)
				pointFlags[(*hullPoints.rbegin())->index] = POINT_USED;

			while (!edges.empty())
			{
				//current edge (AB)
				//this should be the edge with the nearest 'candidate'
				Edge e = *edges.begin();
				edges.erase(edges.begin());

				VertexIterator itA = e.itA;
				VertexIterator itB = itA; ++itB;
				if (itB == hullPoints.end())
				{
					assert(contourType == FULL);
					itB = hullPoints.begin();
				}

				//nearest point
				const Vertex2D& P = points[e.nearestPointIndex];
				assert(pointFlags[P.index] == POINT_NOT_USED); //we don't consider already used points!

				//create labels
				cc2DLabel* edgeLabel = nullptr;
				cc2DLabel* label = nullptr;
				
				if (enableVisualDebugMode && !debugDialog.isSkipped())
				{
					edgeLabel = new cc2DLabel("edge");
					unsigned indexA = 0;
					unsigned indexB = 0;
					for (size_t i = 0; i < pointCount; ++i)
					{
						const Vertex2D& P = points[i];
						if (&P == *itA)
							indexA = static_cast<unsigned>(i);
						if (&P == *itB)
							indexB = static_cast<unsigned>(i);
					}
					edgeLabel->addPickedPoint(debugCloud, indexA);
					edgeLabel->addPickedPoint(debugCloud, indexB);
					edgeLabel->setVisible(true);
					edgeLabel->setDisplayedIn2D(false);
					debugDialog.addToDisplay(edgeLabel);
					debugDialog.refresh();

					label = new cc2DLabel("nearest point");
					label->addPickedPoint(debugCloud, e.nearestPointIndex);
					label->setVisible(true);
					label->setSelected(true);
					debugDialog.addToDisplay(label);
					debugDialog.displayMessage(QString("nearest point found index #%1 (dist = %2)").arg(e.nearestPointIndex).arg(sqrt(e.nearestPointSquareDist)),true);
				}

				//check that we don't create too small edges!
				//CCVector2 AP = (P-**itA);
				//CCVector2 PB = (**itB-P);
				//PointCoordinateType squareLengthAP = (P-**itA).norm2();
				//PointCoordinateType squareLengthPB = (**itB-P).norm2();
				////at least one of the new segments must be smaller than the initial one!
				//assert( squareLengthAP < e.squareLength || squareLengthPB < e.squareLength );
				//if (squareLengthAP < minSquareEdgeLength || squareLengthPB < minSquareEdgeLength)
				//{
				//	pointFlags[P.index] = POINT_IGNORED;
				//	edges.push(e); //retest the edge!
				//	if (enableVisualDebugMode)
				//		debugDialog.displayMessage("nearest point is too close!",true);
				//}

				//last check: the new segments must not intersect with the actual hull!
				bool intersect = false;
				//if (false)
				{
					for (VertexIterator itJ = hullPoints.begin(), itI = itJ++; itI != hullPoints.end(); ++itI, ++itJ)
					{
						if (itJ == hullPoints.end())
						{
							if (contourType == FULL)
								itJ = hullPoints.begin();
							else
								break;
						}

						if (	((*itI)->index != (*itA)->index && (*itJ)->index != (*itA)->index && CCLib::PointProjectionTools::segmentIntersect(**itI,**itJ,**itA,P))
							||	((*itI)->index != (*itB)->index && (*itJ)->index != (*itB)->index && CCLib::PointProjectionTools::segmentIntersect(**itI,**itJ,P,**itB)) )
						{
							intersect = true;
							break;
						}
					}
				}
				if (!intersect)
				{
					//add point to concave hull
					VertexIterator itP = hullPoints.insert(itB == hullPoints.begin() ? hullPoints.end() : itB, &points[e.nearestPointIndex]);

					//we won't use P anymore!
					pointFlags[P.index] = POINT_USED;

					somethingHasChanged = true;

					if (enableVisualDebugMode && !debugDialog.isSkipped())
					{
						if (debugContour)
						{
							debugContourVertices->clear();
							unsigned hullSize = static_cast<unsigned>(hullPoints.size());
							debugContourVertices->reserve(hullSize);
							unsigned index = 0;
							for (VertexIterator it = hullPoints.begin(); it != hullPoints.end(); ++it, ++index)
							{
								const Vertex2D* P = *it;
								debugContourVertices->addPoint(CCVector3(P->x,P->y,0));
							}
							debugContour->reserve(hullSize);
							debugContour->addPointIndex(hullSize-1);
							debugDialog.refresh();
						}
						debugDialog.displayMessage("point has been added to contour",true);
					}

					//update all edges that were having 'P' as their nearest candidate as well
					if (!edges.empty())
					{
						std::vector<VertexIterator> removed;
						std::multiset<Edge>::const_iterator lastValidIt = edges.end();
						for (std::multiset<Edge>::const_iterator it = edges.begin(); it != edges.end(); ++it)
						{
							if ((*it).nearestPointIndex == e.nearestPointIndex)
							{
								//we'll have to put them back afterwards!
								removed.push_back((*it).itA);
							
								edges.erase(it);
								if (edges.empty())
									break;
								if (lastValidIt != edges.end())
									it = lastValidIt;
								else
									it = edges.begin();
							}
							else
							{
								lastValidIt = it;
							}
						}

						//update the removed edges info and put them back in the main list
						for (size_t i = 0; i < removed.size(); ++i)
						{
							VertexIterator itC = removed[i];
							VertexIterator itD = itC; ++itD;
							if (itD == hullPoints.end())
								itD = hullPoints.begin();
						
							unsigned nearestPointIndex = 0;
							PointCoordinateType minSquareDist = FindNearestCandidate(
								nearestPointIndex,
								itC,
								itD,
								points,
								pointFlags,
								minSquareEdgeLength,
								false,
								minCosAngle);

							if (minSquareDist >= 0)
							{
								Edge e(itC, nearestPointIndex, minSquareDist);
								edges.insert(e);
							}
						}
					}

					//we'll inspect the two new segments later (if necessary)
					if ((P-**itA).norm2() > maxSquareEdgeLength)
					{
						unsigned nearestPointIndex = 0;
						PointCoordinateType minSquareDist = FindNearestCandidate(
							nearestPointIndex,
							itA,
							itP,
							points,
							pointFlags,
							minSquareEdgeLength,
							false,
							minCosAngle);

						if (minSquareDist >= 0)
						{
							Edge e(itA,nearestPointIndex,minSquareDist);
							edges.insert(e);
						}
					}
					if ((**itB-P).norm2() > maxSquareEdgeLength)
					{
						unsigned nearestPointIndex = 0;
						PointCoordinateType minSquareDist = FindNearestCandidate(
							nearestPointIndex,
							itP,
							itB,
							points,
							pointFlags,
							minSquareEdgeLength,
							false,
							minCosAngle);

						if (minSquareDist >= 0)
						{
							Edge e(itP,nearestPointIndex,minSquareDist);
							edges.insert(e);
						}
					}
				}
				else
				{
					if (enableVisualDebugMode)
						debugDialog.displayMessage("[rejected] new edge would intersect the current contour!",true);
				}
			
				//remove labels
				if (label)
				{
					assert(enableVisualDebugMode);
					debugDialog.removFromDisplay(label);
					delete label;
					label = nullptr;
					//debugDialog.refresh();
				}

				if (edgeLabel)
				{
					assert(enableVisualDebugMode);
					debugDialog.removFromDisplay(edgeLabel);
					delete edgeLabel;
					edgeLabel = nullptr;
					//debugDialog.refresh();
				}
			}
		}
		catch (...)
		{
			//not enough memory
			return false;
		}

		if (!allowMultiPass)
			break;
	}

	return true;
}

using Hull2D = std::list<Vertex2D *>;

ccPolyline* ccContourExtractor::ExtractFlatContour(	CCLib::GenericIndexedCloudPersist* points,
													bool allowMultiPass,
													PointCoordinateType maxEdgeLength/*=0*/,
													const PointCoordinateType* preferredNormDim/*=0*/,
													const PointCoordinateType* preferredUpDir/*=0*/,
													ContourType contourType/*=FULL*/,
													std::vector<unsigned>* originalPointIndexes/*=0*/,
													bool enableVisualDebugMode/*=false*/,
													double maxAngleDeg/*=0.0*/)
{
	assert(points);
	
	if (!points)
		return nullptr;
	
	unsigned ptsCount = points->size();
	
	if (ptsCount < 3)
		return nullptr;

	CCLib::Neighbourhood Yk(points);
	
	//local base
	CCVector3 O;
	CCVector3 X;
	CCVector3 Y;
	
	CCLib::Neighbourhood::InputVectorsUsage vectorsUsage = CCLib::Neighbourhood::None;

	//we project the input points on a plane
	std::vector<Vertex2D> points2D;
	PointCoordinateType* planeEq = nullptr;
	
	if (preferredUpDir != nullptr)
	{
		Y = CCVector3(preferredUpDir);
		vectorsUsage = CCLib::Neighbourhood::UseYAsUpDir;
	}

	//if the user has specified a default direction, we'll use it as 'projecting plane'
	PointCoordinateType preferredPlaneEq[4] = { 0, 0, 1, 0 };

	if (preferredNormDim != nullptr)
	{
		const CCVector3* G = points->getPoint(0); //any point through which the plane passes is ok
		preferredPlaneEq[0] = preferredNormDim[0];
		preferredPlaneEq[1] = preferredNormDim[1];
		preferredPlaneEq[2] = preferredNormDim[2];
		CCVector3::vnormalize(preferredPlaneEq);
		preferredPlaneEq[3] = CCVector3::vdot(G->u, preferredPlaneEq);
		planeEq = preferredPlaneEq;

		if (preferredUpDir != nullptr)
		{
			O = *G;
			//Y = CCVector3(preferredUpDir); //already done above
			X = Y.cross(CCVector3(preferredNormDim));
			vectorsUsage = CCLib::Neighbourhood::UseOXYasBase;
		}
	}

	if (!Yk.projectPointsOn2DPlane<Vertex2D>(points2D, planeEq, &O, &X, &Y, vectorsUsage))
	{
		ccLog::Warning("[ExtractFlatContour] Failed to project the points on the LS plane (not enough memory?)!");
		return nullptr;
	}

	//update the points indexes (not done by Neighbourhood::projectPointsOn2DPlane)
	{
		for (unsigned i = 0; i < ptsCount; ++i)
		{
			points2D[i].index = i;
		}
	}

	//try to get the points on the convex/concave hull to build the contour and the polygon
	Hull2D hullPoints;
	if (!ExtractConcaveHull2D(	points2D,
								hullPoints,
								contourType,
								allowMultiPass,
								maxEdgeLength*maxEdgeLength,
								enableVisualDebugMode,
								maxAngleDeg))
	{
		ccLog::Warning("[ExtractFlatContour] Failed to compute the convex hull of the input points!");
		return nullptr;
	}

	if (originalPointIndexes)
	{
		try
		{
			originalPointIndexes->resize(hullPoints.size(), 0);
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			ccLog::Error("[ExtractFlatContour] Not enough memory!");
			return nullptr;
		}

		unsigned i = 0;
		for (Hull2D::const_iterator it = hullPoints.begin(); it != hullPoints.end(); ++it, ++i)
		{
			(*originalPointIndexes)[i] = (*it)->index;
		}
	}

	unsigned hullPtsCount = static_cast<unsigned>(hullPoints.size());

	//create vertices
	ccPointCloud* contourVertices = new ccPointCloud();
	{
		if (!contourVertices->reserve(hullPtsCount))
		{
			delete contourVertices;
			contourVertices = nullptr;
			ccLog::Error("[ExtractFlatContour] Not enough memory!");
			return nullptr;
		}

		//projection on the LS plane (in 3D)
		for (Hull2D::const_iterator it = hullPoints.begin(); it != hullPoints.end(); ++it)
		{
			contourVertices->addPoint(O + X*(*it)->x + Y*(*it)->y);
		}
		
		contourVertices->setName("vertices");
		contourVertices->setEnabled(false);
	}

	//we create the corresponding (3D) polyline
	ccPolyline* contourPolyline = new ccPolyline(contourVertices);
	if (contourPolyline->reserve(hullPtsCount))
	{
		contourPolyline->addPointIndex(0, hullPtsCount);
		contourPolyline->setClosed(contourType == FULL);
		contourPolyline->setVisible(true);
		contourPolyline->setName("contour");
		contourPolyline->addChild(contourVertices);
	}
	else
	{
		delete contourPolyline;
		contourPolyline = nullptr;
		ccLog::Warning("[ExtractFlatContour] Not enough memory to create the contour polyline!");
	}

	return contourPolyline;
}

bool ccContourExtractor::ExtractFlatContour(CCLib::GenericIndexedCloudPersist* points,
											bool allowMultiPass,
											PointCoordinateType maxEdgeLength,
											std::vector<ccPolyline*>& parts,
											ContourType contourType/*=FULL*/,
											bool allowSplitting/*=true*/,
											const PointCoordinateType* preferredNormDir/*=nullptr*/,
											const PointCoordinateType* preferredUpDir/*=nullptr*/,
											bool enableVisualDebugMode/*=false*/)
{
	parts.clear();

	//extract whole contour
	ccPolyline* basePoly = ExtractFlatContour(points, allowMultiPass, maxEdgeLength, preferredNormDir, preferredUpDir, contourType, nullptr, enableVisualDebugMode);
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
	bool success = basePoly->split(maxEdgeLength, parts);

	delete basePoly;
	basePoly = nullptr;

	return success;
}
