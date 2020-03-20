//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include <PointProjectionTools.h>

//local
#include <Delaunay2dMesh.h>
#include <DistanceComputationTools.h>
#include <GenericProgressCallback.h>
#include <Neighbourhood.h>
#include <ParallelSort.h>
#include <PointCloud.h>
#include <SimpleMesh.h>

//system
#include <set>

using namespace CCLib;

PointCloud* PointProjectionTools::developCloudOnCylinder(	GenericCloud* cloud,
															PointCoordinateType radius,
															unsigned char dim,
															CCVector3* center,
															GenericProgressCallback* progressCb)
{
	if (!cloud)
		return nullptr;

	unsigned char dim1 = (dim > 0 ? dim-1 : 2);
	unsigned char dim2 = (dim < 2 ? dim+1 : 0);

	unsigned count = cloud->size();

	PointCloud* newCloud = new PointCloud();
	if (!newCloud->reserve(count)) //not enough memory
		return nullptr;

	//we compute cloud bounding box center if no center is specified
	CCVector3 C;
	if (!center)
	{
		CCVector3 bbMin;
		CCVector3 bbMax;
		cloud->getBoundingBox(bbMin,bbMax);
		C = (bbMin+bbMax)/2;
		center = &C;
	}

	NormalizedProgress nprogress(progressCb, count);
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setMethodTitle("Develop");
			char buffer[256];
			sprintf(buffer, "Number of points = %u", count);
			progressCb->setInfo(buffer);
		}
		progressCb->update(0);
		progressCb->start();
	}

	const CCVector3* Q = nullptr;
	cloud->placeIteratorAtBeginning();
	while ((Q = cloud->getNextPoint()))
	{
		CCVector3 P = *Q-*center;
		PointCoordinateType u = sqrt(P.u[dim1] * P.u[dim1] + P.u[dim2] * P.u[dim2]);
		PointCoordinateType lon = atan2(P.u[dim1],P.u[dim2]);

		newCloud->addPoint(CCVector3(lon*radius,P.u[dim],u-radius));

		if (progressCb && !nprogress.oneStep())
		{
			break;
		}

	}

	if (progressCb)
	{
		progressCb->stop();
	}

	return newCloud;
}

//deroule la liste sur un cone dont le centre est "center" et d'angle alpha en degres
PointCloud* PointProjectionTools::developCloudOnCone(GenericCloud* cloud, unsigned char dim, PointCoordinateType baseRadius, float alpha, const CCVector3& center, GenericProgressCallback* progressCb)
{
	if (!cloud)
		return nullptr;

	unsigned count = cloud->size();

	PointCloud* outCloud = new PointCloud();
	if (!outCloud->reserve(count)) //not enough memory
		return nullptr;

	unsigned char dim1 = (dim>0 ? dim-1 : 2);
	unsigned char dim2 = (dim<2 ? dim+1 : 0);

	float tan_alpha = tanf(alpha*static_cast<float>(CC_DEG_TO_RAD));
	//float cos_alpha = cos(alpha*CC_DEG_TO_RAD);
	//float sin_alpha = sin(alpha*CC_DEG_TO_RAD);
	float q = 1.0f/(1.0f+tan_alpha*tan_alpha);

	cloud->placeIteratorAtBeginning();

	NormalizedProgress nprogress(progressCb, count);
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setMethodTitle("DevelopOnCone");
			char buffer[256];
			sprintf(buffer, "Number of points = %u", count);
			progressCb->setInfo(buffer);
		}
		progressCb->update(0);
		progressCb->start();
	}

	for (unsigned i = 0; i < count; i++)
	{
		const CCVector3 *Q = cloud->getNextPoint();
		CCVector3 P = *Q-center;

		PointCoordinateType u = sqrt(P.u[dim1]*P.u[dim1] + P.u[dim2]*P.u[dim2]);
		PointCoordinateType lon = atan2(P.u[dim1],P.u[dim2]);

		//projection sur le cone
		PointCoordinateType z2 = (P.u[dim]+u*tan_alpha)*q;
		PointCoordinateType x2 = z2*tan_alpha;
		//ordonnee
		//#define ORTHO_CONIC_PROJECTION
		#ifdef ORTHO_CONIC_PROJECTION
		PointCoordinateType lat = sqrt(x2*x2+z2*z2)*cos_alpha;
		if (lat*z2 < 0.0)
			lat=-lat;
		#else
		PointCoordinateType lat = P.u[dim];
		#endif
		//altitude
		PointCoordinateType dX = u-x2;
		PointCoordinateType dZ = P.u[dim]-z2;
		PointCoordinateType alt = sqrt(dX*dX+dZ*dZ);
		//on regarde de quel cote de la surface du cone le resultat tombe par p.v.
		if (x2*P.u[dim] - z2*u < 0)
			alt=-alt;

		outCloud->addPoint(CCVector3(lon*baseRadius,lat+center[dim],alt));

		if (progressCb && !nprogress.oneStep())
		{
			break;
		}
	}

	if (progressCb)
	{
		progressCb->stop();
	}

	return outCloud;
}

PointCloud* PointProjectionTools::applyTransformation(GenericCloud* cloud, Transformation& trans, GenericProgressCallback* progressCb)
{
	assert(cloud);

	unsigned count = cloud->size();

	PointCloud* transformedCloud = new PointCloud();
	if (!transformedCloud->reserve(count))
		return nullptr; //not enough memory

	NormalizedProgress nprogress(progressCb, count);
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setMethodTitle("ApplyTransformation");
			char buffer[256];
			sprintf(buffer, "Number of points = %u", count);
			progressCb->setInfo(buffer);
		}
		progressCb->update(0);
		progressCb->start();
	}

	cloud->placeIteratorAtBeginning();
	const CCVector3* P;

	if (trans.R.isValid())
	{
		while ((P = cloud->getNextPoint()))
		{
			//P' = s*R.P+T
			CCVector3 newP = trans.s * (trans.R * (*P)) + trans.T;

			transformedCloud->addPoint(newP);

			if (progressCb && !nprogress.oneStep())
			{
				break;
			}
		}
	}
	else
	{
		while ((P = cloud->getNextPoint()))
		{
			//P' = s*P+T
			CCVector3 newP = trans.s * (*P) + trans.T;

			transformedCloud->addPoint(newP);

			if (progressCb && !nprogress.oneStep())
			{
				break;
			}
		}
	}

	if (progressCb)
	{
		progressCb->stop();
	}

	return transformedCloud;
}

GenericIndexedMesh* PointProjectionTools::computeTriangulation(	GenericIndexedCloudPersist* cloud,
																CC_TRIANGULATION_TYPES type/*=DELAUNAY_2D_AXIS_ALIGNED*/,
																PointCoordinateType maxEdgeLength/*=0*/,
																unsigned char dim/*=0*/,
																char* errorStr/*=nullptr*/)
{
	if (!cloud)
	{
		if (errorStr)
			strcpy(errorStr, "Invalid input cloud");
		return nullptr;
	}


	switch (type)
	{
	case DELAUNAY_2D_AXIS_ALIGNED:
		{
			if (dim > 2)
			{
				if (errorStr)
					strcpy(errorStr, "Invalid projection dimension");
				return nullptr;
			}
			const unsigned char Z = static_cast<unsigned char>(dim);
			const unsigned char X = (Z == 2 ? 0 : Z + 1);
			const unsigned char Y = (X == 2 ? 0 : X + 1);

			unsigned count = cloud->size();
			std::vector<CCVector2> the2DPoints;
			try
			{
				the2DPoints.resize(count);
			}
			catch (.../*const std::bad_alloc&*/) //out of memory
			{
				if (errorStr)
					strcpy(errorStr, "Not enough memory");
				break;
			}

			cloud->placeIteratorAtBeginning();
			for (unsigned i = 0; i < count; ++i)
			{
				const CCVector3* P = cloud->getPoint(i);
				the2DPoints[i].x = P->u[X];
				the2DPoints[i].y = P->u[Y];
			}

			Delaunay2dMesh* dm = new Delaunay2dMesh();
			char triLibErrorStr[1024];
			if (!dm->buildMesh(the2DPoints, 0, triLibErrorStr))
			{
				if (errorStr)
					strcpy(errorStr, triLibErrorStr);
				delete dm;
				return nullptr;
			}
			dm->linkMeshWith(cloud, false);

			//remove triangles with too long edges
			if (maxEdgeLength > 0)
			{
				dm->removeTrianglesWithEdgesLongerThan(maxEdgeLength);
				if (dm->size() == 0)
				{
					//no more triangles?
					if (errorStr)
						strcpy(errorStr, "No triangle left after pruning");
					delete dm;
					return nullptr;
				}
			}

			return static_cast<GenericIndexedMesh*>(dm);
		}

	case DELAUNAY_2D_BEST_LS_PLANE:
		{
			Neighbourhood Yk(cloud);
			GenericIndexedMesh* mesh = Yk.triangulateOnPlane(false, maxEdgeLength, errorStr);
			return mesh;
		}

	default:
		//shouldn't happen
		assert(false);
		break;
	}

	return nullptr;
}

// Lexicographic sorting operator
inline bool LexicographicSort(const CCVector2& a, const CCVector2& b)
{
	return a.x < b.x || (a.x == b.x && a.y < b.y);
}

bool PointProjectionTools::extractConvexHull2D(	std::vector<IndexedCCVector2>& points,
												std::list<IndexedCCVector2*>& hullPoints)
{
	std::size_t n = points.size();

	// Sort points lexicographically
	ParallelSort(points.begin(), points.end(), LexicographicSort);

	// Build lower hull
	{
		for (std::size_t i=0; i<n; i++)
		{
			while (hullPoints.size() >= 2)
			{
				std::list<IndexedCCVector2*>::iterator itB = hullPoints.end(); --itB;
				std::list<IndexedCCVector2*>::iterator itA = itB; --itA;
				if ((**itB - **itA).cross(points[i] - **itA) <= 0)
				{
					hullPoints.pop_back();
				}
				else
				{
					break;
				}
			}

			try
			{
				hullPoints.push_back(&points[i]);
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory
				return false;
			}
		}
	}

	// Build upper hull
	{
		std::size_t t = hullPoints.size()+1;
		for (int i=static_cast<int>(n)-2; i>=0; i--)
		{
			while (hullPoints.size() >= t)
			{
				std::list<IndexedCCVector2*>::iterator itB = hullPoints.end(); --itB;
				std::list<IndexedCCVector2*>::iterator itA = itB; --itA;
				if ((**itB - **itA).cross(points[i] - **itA) <= 0)
				{
					hullPoints.pop_back();
				}
				else
				{
					break;
				}
			}

			try
			{
				hullPoints.push_back(&points[i]);
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory
				return false;
			}
		}
	}

	//remove last point if it's the same as the first one
	if (hullPoints.size() > 1
		&&	hullPoints.front()->x == hullPoints.back()->x
		&&	hullPoints.front()->y == hullPoints.back()->y )
	{
		hullPoints.pop_back();
	}

	return true;
}

bool PointProjectionTools::segmentIntersect(const CCVector2& A, const CCVector2& B, const CCVector2& C, const CCVector2& D)
{
	CCVector2 AB = B-A;
	CCVector2 AC = C-A;
	CCVector2 AD = D-A;
	PointCoordinateType cross_AB_AC = AB.cross(AC);
	PointCoordinateType cross_AB_AD = AB.cross(AD);
	
	//both C and D are on the same side of AB?
	if (cross_AB_AC * cross_AB_AD > 0)
	{
		//no intersection
		return false;
	}

	CCVector2 CD = D-C;
	CCVector2 CB = B-C;
	PointCoordinateType cross_CD_CA = -CD.cross(AC);
	PointCoordinateType cross_CD_CB = CD.cross(CB);

	//both A and B are on the same side of CD?
	if (cross_CD_CA * cross_CD_CB > 0)
	{
		//no intersection
		return false;
	}

	PointCoordinateType cross_AB_CD = AB.cross(CD);
	if (std::abs(cross_AB_CD) != 0) //AB and CD are not parallel
	{
		//where do they intersect?
		//PointCoordinateType v = cross_AB_AC/cross_AB_CD;
		//assert(v >= 0 && v <= 1);

		return true;
	}
	else //AB and CD are parallel (therefore they are colinear - see above tests)
	{
		PointCoordinateType dAB = AB.norm();
		
		PointCoordinateType dot_AB_AC = AB.dot(AC);
		if (dot_AB_AC >= 0 && dot_AB_AC < dAB * AC.norm())
		{
			//C is between A and B
			return true;
		}

		PointCoordinateType dot_AB_AD = AB.dot(AD);
		if (dot_AB_AD >= 0 && dot_AB_AD < dAB * AD.norm())
		{
			//D is between A and B
			return true;
		}

		//otherwise there's an intersection only if B and C are on both sides!
		return (dot_AB_AC * dot_AB_AD < 0);
	}
}

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
static PointCoordinateType FindNearestCandidate(	unsigned& minIndex,
													const VertexIterator& itA,
													const VertexIterator& itB,
													const std::vector<Vertex2D>& points,
													const std::vector<HullPointFlags>& pointFlags,
													PointCoordinateType minSquareEdgeLength,
													PointCoordinateType maxSquareEdgeLength,
													bool allowLongerChunks = false)
{
	//look for the nearest point in the input set
	PointCoordinateType minDist2 = -1;
	CCVector2 AB = **itB - **itA;
	PointCoordinateType squareLengthAB = AB.norm2();
	unsigned pointCount = static_cast<unsigned>(points.size());
	for (unsigned i = 0; i < pointCount; ++i)
	{
		const Vertex2D& P = points[i];
		if (pointFlags[P.index] != POINT_NOT_USED)
			continue;

		//skip the edge vertices!
		if (P.index == (*itA)->index || P.index == (*itB)->index)
			continue;

		//we only consider 'inner' points
		CCVector2 AP = P - **itA;
		if (AB.x * AP.y - AB.y * AP.x < 0)
		{
			continue;
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
	return (minDist2 < 0 ? minDist2 : minDist2/squareLengthAB);
}

bool PointProjectionTools::extractConcaveHull2D(std::vector<IndexedCCVector2>& points,
												std::list<IndexedCCVector2*>& hullPoints,
												PointCoordinateType maxSquareEdgeLength/*=0*/)
{
	//first compute the Convex hull
	if (!extractConvexHull2D(points, hullPoints))
		return false;

	//do we really need to compute the concave hull?
	if (hullPoints.size() < 2 || maxSquareEdgeLength <= 0)
		return true;

	unsigned pointCount = static_cast<unsigned>(points.size());

	std::vector<HullPointFlags> pointFlags;
	try
	{
		pointFlags.resize(pointCount, POINT_NOT_USED);
	}
	catch (...)
	{
		//not enough memory
		return false;
	}

	//hack: compute the theoretical 'minimal' edge length
	PointCoordinateType minSquareEdgeLength = 0;
	{
		CCVector2 minP;
		CCVector2 maxP;
		for (std::size_t i = 0; i < pointCount; ++i)
		{
			const IndexedCCVector2& P = points[i];
			if (i)
			{
				minP.x = std::min(P.x, minP.x);
				minP.y = std::min(P.y, minP.y);
				maxP.x = std::max(P.x, maxP.x);
				maxP.y = std::max(P.y, maxP.y);
			}
			else
			{
				minP = maxP = P;
			}
		}
		minSquareEdgeLength = (maxP-minP).norm2() / static_cast<PointCoordinateType>(1.0e7); //10^-7 of the max bounding rectangle side
		minSquareEdgeLength = std::min(minSquareEdgeLength, maxSquareEdgeLength / 10);

		//we remove very small edges
		for (std::list<IndexedCCVector2*>::iterator itA = hullPoints.begin(); itA != hullPoints.end(); ++itA)
		{
			std::list<IndexedCCVector2*>::iterator itB = itA; ++itB;
			if (itB == hullPoints.end())
				itB = hullPoints.begin();
			if ((**itB-**itA).norm2() < minSquareEdgeLength)
			{
				pointFlags[(*itB)->index] = POINT_FROZEN;
				hullPoints.erase(itB);
			}
		}

		if (hullPoints.size() < 2)
		{
			//no more edges?!
			return false;
		}
	}

	//we repeat the process until nothing changes!
	//Warning: high STL containers usage ahead ;)
	unsigned step = 0;
	bool somethingHasChanged = true;
	while (somethingHasChanged)
	{
		try
		{
			somethingHasChanged = false;
			++step;

			////reset point flags
			//for (std::size_t i=0; i<pointCount; ++i)
			//{
			//	if (pointFlags[i] != POINT_FROZEN)
			//		pointFlags[i] = POINT_NOT_USED;
			//}
		
			//build the initial edge list & flag the convex hull points
			std::multiset<Edge> edges;
			{
				for (std::list<IndexedCCVector2*>::iterator itA = hullPoints.begin(); itA != hullPoints.end(); ++itA)
				{
					std::list<IndexedCCVector2*>::iterator itB = itA; ++itB;
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
																maxSquareEdgeLength,
																step > 1);

						if (minSquareDist >= 0)
						{
							Edge e(itA, nearestPointIndex, minSquareDist);
							edges.insert(e);
						}
					}
				
					pointFlags[(*itA)->index] = POINT_USED;
				}
			}

			while (!edges.empty())
			{
				//current edge (AB)
				//this should be the edge with the nearest 'candidate'
				Edge e = *edges.begin();
				edges.erase(edges.begin());

				VertexIterator itA = e.itA;
				VertexIterator itB = itA; ++itB;
				if (itB == hullPoints.end())
					itB = hullPoints.begin();

				//nearest point
				const Vertex2D& P = points[e.nearestPointIndex];
				if (pointFlags[P.index] != POINT_NOT_USED)
				{
					//assert(false); //DGM: in fact it happens!
					break;
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
				//}

				//last check: the new segments must not intersect with the actual hull!
				bool intersect = false;
				//if (false)
				{
					for (VertexIterator itJ = hullPoints.begin(), itI = itJ++; itI != hullPoints.end(); ++itI, ++itJ)
					{
						if (itJ == hullPoints.end())
							itJ = hullPoints.begin();

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
						for (std::size_t i=0; i<removed.size(); ++i)
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
																	maxSquareEdgeLength);

							if (minSquareDist >= 0)
							{
								Edge e(itC,nearestPointIndex,minSquareDist);
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
																maxSquareEdgeLength);

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
																maxSquareEdgeLength);

						if (minSquareDist >= 0)
						{
							Edge e(itP,nearestPointIndex,minSquareDist);
							edges.insert(e);
						}
					}
				}
			}
		}
		catch (...)
		{
			//not enough memory
			return false;
		}
	}

	return true;
}

void PointProjectionTools::Transformation::apply(GenericIndexedCloudPersist& cloud) const
{
	//always apply the scale before everything (applying before or after rotation does not changes anything)
	if (std::abs(s - 1.0) > ZERO_TOLERANCE)
	{
		for (unsigned i = 0; i< cloud.size(); ++i)
		{
			CCVector3* P = const_cast<CCVector3*>(cloud.getPoint(i));
			*P *= s;
		}
	}

	if (R.isValid())
	{
		for (unsigned i = 0; i< cloud.size(); ++i)
		{
			CCVector3* P = const_cast<CCVector3*>(cloud.getPoint(i));
			(*P) = R * (*P);
		}
	}

	if (T.norm() > ZERO_TOLERANCE) //T applied only if it makes sense
	{
		for (unsigned i = 0; i< cloud.size(); ++i)
		{
			CCVector3* P = const_cast<CCVector3*>(cloud.getPoint(i));
			(*P) += T;
		}
	}
}
