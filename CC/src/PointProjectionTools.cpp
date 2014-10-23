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

#include "PointProjectionTools.h"

//local
#include "SimpleCloud.h"
#include "Delaunay2dMesh.h"
#include "GenericIndexedMesh.h"
#include "GenericProgressCallback.h"
#include "Neighbourhood.h"
#include "SimpleMesh.h"

//system
#include <assert.h>
#include <string.h>

using namespace CCLib;

SimpleCloud* PointProjectionTools::developCloudOnCylinder(GenericCloud* theCloud,
															PointCoordinateType radius,
															unsigned char dim,
															CCVector3* center,
															GenericProgressCallback* progressCb)
{
	if (!theCloud)
		return 0;

	uchar dim1 = (dim>0 ? dim-1 : 2);
	uchar dim2 = (dim<2 ? dim+1 : 0);

	unsigned count = theCloud->size();

	SimpleCloud* newList = new SimpleCloud();
	if (!newList->reserve(count)) //not enough memory
		return 0;

	//we compute cloud bounding box center if no center is specified
	CCVector3 C;
	if (!center)
	{
		PointCoordinateType Mins[3],Maxs[3];
		theCloud->getBoundingBox(Mins,Maxs);
		C = (CCVector3(Mins)+CCVector3(Maxs))*0.5;
		center = &C;
	}

	NormalizedProgress* nprogress = 0;
	if (progressCb)
	{
		progressCb->reset();
		progressCb->setMethodTitle("Develop");
		char buffer[256];
		sprintf(buffer,"Number of points = %u",count);
		nprogress = new NormalizedProgress(progressCb,count);
		progressCb->setInfo(buffer);
		progressCb->start();
	}

	const CCVector3 *Q;
	CCVector3 P;
	PointCoordinateType u,lon;

	theCloud->placeIteratorAtBegining();
	while ((Q = theCloud->getNextPoint()))
	{
		P = *Q-*center;
		u = sqrt(P.u[dim1] * P.u[dim1] + P.u[dim2] * P.u[dim2]);
		lon = atan2(P.u[dim1],P.u[dim2]);

		newList->addPoint(CCVector3(lon*radius,P.u[dim],u-radius));

		if (nprogress)
		{
			if (!nprogress->oneStep())
				break;
		}

	}

	if (progressCb)
	{
		delete nprogress;
		progressCb->stop();
	}

	return newList;
}

//deroule la liste sur un cone dont le centre est "center" et d'angle alpha en degres
SimpleCloud* PointProjectionTools::developCloudOnCone(GenericCloud* theCloud, uchar dim, PointCoordinateType baseRadius, float alpha, const CCVector3& center, GenericProgressCallback* progressCb)
{
	if (!theCloud)
		return 0;

	unsigned count = theCloud->size();

	SimpleCloud* cloud = new SimpleCloud();
	if (!cloud->reserve(count)) //not enough memory
		return 0;

	uchar dim1 = (dim>0 ? dim-1 : 2);
	uchar dim2 = (dim<2 ? dim+1 : 0);

	float tan_alpha = tan(alpha*static_cast<float>(CC_DEG_TO_RAD));
	//float cos_alpha = cos(alpha*CC_DEG_TO_RAD);
	//float sin_alpha = sin(alpha*CC_DEG_TO_RAD);
	float q = 1.0f/(1.0f+tan_alpha*tan_alpha);

	CCVector3 P;
	PointCoordinateType u,lon,z2,x2,dX,dZ,lat,alt;

	theCloud->placeIteratorAtBegining();
	//normsType* _theNorms = theNorms.begin();

	NormalizedProgress* nprogress = 0;
	if (progressCb)
	{
		progressCb->reset();
		progressCb->setMethodTitle("DevelopOnCone");
		char buffer[256];
		sprintf(buffer,"Number of points = %u",count);
		nprogress = new NormalizedProgress(progressCb,count);
		progressCb->setInfo(buffer);
		progressCb->start();
	}

	for (unsigned i=0; i<count; i++)
	{
		const CCVector3 *Q = theCloud->getNextPoint();
		P = *Q-center;

		u = sqrt(P.u[dim1]*P.u[dim1] + P.u[dim2]*P.u[dim2]);
		lon = atan2(P.u[dim1],P.u[dim2]);

		//projection sur le cone
		z2 = (P.u[dim]+u*tan_alpha)*q;
		x2 = z2*tan_alpha;
		//ordonnee
		//#define ORTHO_CONIC_PROJECTION
		#ifdef ORTHO_CONIC_PROJECTION
		lat = sqrt(x2*x2+z2*z2)*cos_alpha;
		if (lat*z2 < 0.0)
			lat=-lat;
		#else
		lat = P.u[dim];
		#endif
		//altitude
		dX = u-x2;
		dZ = P.u[dim]-z2;
		alt = sqrt(dX*dX+dZ*dZ);
		//on regarde de quel cote de la surface du cone le resultat tombe par p.v.
		if (x2*P.u[dim] - z2*u < 0)
			alt=-alt;

		cloud->addPoint(CCVector3(lon*baseRadius,lat+center[dim],alt));

		if (nprogress && !nprogress->oneStep())
			break;
	}

	if (nprogress)
	{
		delete nprogress;
		nprogress = 0;
	}
	if (progressCb)
	{
		progressCb->stop();
	}

	return cloud;
}

SimpleCloud* PointProjectionTools::applyTransformation(GenericCloud* theCloud, Transformation& trans, GenericProgressCallback* progressCb)
{
    assert(theCloud);

    unsigned count = theCloud->size();

    SimpleCloud* transformedCloud = new SimpleCloud();
    if (!transformedCloud->reserve(count))
        return 0; //not enough memory

    NormalizedProgress* nprogress = 0;
    if (progressCb)
    {
        progressCb->reset();
        progressCb->setMethodTitle("ApplyTransformation");
        nprogress = new NormalizedProgress(progressCb,count);
        char buffer[256];
        sprintf(buffer,"Number of points = %u",count);
        progressCb->setInfo(buffer);
        progressCb->start();
    }

    theCloud->placeIteratorAtBegining();
    const CCVector3* P;

	if (trans.R.isValid())
	{
		while ((P = theCloud->getNextPoint()))
		{
			//P' = s*R.P+T
			CCVector3 newP = trans.s * (trans.R * (*P)) + trans.T;

			transformedCloud->addPoint(newP);

			if (nprogress && !nprogress->oneStep())
				break;
		}
	}
	else
	{
		while ((P = theCloud->getNextPoint()))
		{
			//P' = s*P+T
			CCVector3 newP = trans.s * (*P) + trans.T;

			transformedCloud->addPoint(newP);

			if (nprogress && !nprogress->oneStep())
				break;
		}
	}

	if (progressCb)
		progressCb->stop();

	return transformedCloud;
}

GenericIndexedMesh* PointProjectionTools::computeTriangulation(	GenericIndexedCloudPersist* theCloud,
																CC_TRIANGULATION_TYPES type/*=GENERIC*/,
																PointCoordinateType maxEdgeLength/*=0*/,
																char* errorStr/*=0*/)
{
	if (!theCloud)
	{
		if (errorStr)
			strcpy(errorStr, "Invalid input cloud");
		return 0;
	}

	switch(type)
	{
	case GENERIC:
		{
			unsigned count = theCloud->size();
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

			theCloud->placeIteratorAtBegining();
			CCVector3 P;
			for (unsigned i=0; i<count; ++i)
			{
				theCloud->getPoint(i,P);
				the2DPoints[i].x = P.x;
				the2DPoints[i].y = P.y;
			}

			Delaunay2dMesh* dm = new Delaunay2dMesh();
			char triLibErrorStr[1024];
			if (!dm->buildMesh(the2DPoints,0,triLibErrorStr))
			{
				if (errorStr)
					strcpy(errorStr, triLibErrorStr);
				delete dm;
				return 0;
			}
			dm->linkMeshWith(theCloud,false);

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
					return 0;
				}
			}

			return static_cast<GenericIndexedMesh*>(dm);
		}
		break;
	case GENERIC_BEST_LS_PLANE:
		{
			Neighbourhood Yk(theCloud);
			GenericIndexedMesh* mesh = Yk.triangulateOnPlane(false,maxEdgeLength,errorStr);
			return mesh;
		}
		break;
	default:
		//shouldn't happen
		assert(false);
		break;
	}

	return 0;
}

// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the points are collinear.
inline PointCoordinateType cross(const CCVector2& O, const CCVector2& A, const CCVector2& B)
{
	return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

// Lexicographic sorting operator
inline bool LexicographicSort(const CCVector2& a, const CCVector2& b)
{
	return a.x < b.x || (a.x == b.x && a.y < b.y);
}

bool PointProjectionTools::extractConvexHull2D(	std::vector<IndexedCCVector2>& points,
												std::list<IndexedCCVector2*>& hullPoints)
{
	size_t n = points.size();

	// Sort points lexicographically
	std::sort(points.begin(), points.end(), LexicographicSort);

	// Build lower hull
	{
		for (size_t i=0; i<n; i++)
		{
			while (hullPoints.size() >= 2)
			{
				std::list<IndexedCCVector2*>::iterator itB = hullPoints.end(); itB--;
				std::list<IndexedCCVector2*>::iterator itA = itB; itA--;
				if (cross(**itA, **itB, points[i]) <= 0)
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
			catch (std::bad_alloc)
			{
				//not enough memory
				return false;
			}
		}
	}

	// Build upper hull
	{
		size_t t = hullPoints.size()+1;
		for (int i=static_cast<int>(n)-2; i>=0; i--)
		{
			while (hullPoints.size() >= t)
			{
				std::list<IndexedCCVector2*>::iterator itB = hullPoints.end(); itB--;
				std::list<IndexedCCVector2*>::iterator itA = itB; itA--;
				if (cross(**itA, **itB, points[i]) <= 0)
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
			catch (std::bad_alloc)
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

//! Returns the (squared) distance from a point to a segment
/** Only if the point lies 'in front of' the segment! Returns -1.0 otherwise.
**/
PointCoordinateType ComputeSquareDistToEdge(const CCVector2& P, const CCVector2& A, const CCVector2& B)
{
	CCVector2 AP = P-A;
	CCVector2 AB = B-A;
	PointCoordinateType dot = AB.dot(AP); // = cos(PAB) * ||AP|| * ||AB||
	if (dot < 0)
	{
		//return AP.norm2();
		return -1.0;
	}
	else
	{
		PointCoordinateType squareLengthAB = AB.norm2();
		if (dot > squareLengthAB)
		{
			//return (P-B).norm2();
			return -1.0;
		}
		else
		{
			CCVector2 HP = AP - AB * (dot / squareLengthAB);
			return HP.norm2();
		}
	}
}

//! Returns true if the AB and CD segments intersect each other
bool Intersect(const CCVector2& A, const CCVector2& B, const CCVector2& C, const CCVector2& D)
{
	if (cross(A,B,C) * cross(A,B,D) >= 0) //both points are on the same side? No intersection
		return false;

	CCVector2 AB = B-A;
	CCVector2 AC = C-A;
	CCVector2 CD = D-C;

	PointCoordinateType q = CD.y * AB.x - CD.x * AB.y;
	if (fabs(q) > ZERO_TOLERANCE) //AB and CD are not parallel
	{
		//where do they interest?
		PointCoordinateType p = AC.x * AB.y - AC.y * AB.x;
		PointCoordinateType v = p/q;
		if (v <= 0 || v >= 1) //not CD!
			return false;

		//test further with AB
		p = CD.y * AC.x - CD.x * AC.y;
		v= p/q;
		return (v > 0 && v < 1); //not AB?
	}
	else //AB and CD are parallel
	{
		PointCoordinateType dAB = AB.norm();
		PointCoordinateType dAC = AC.norm();
		PointCoordinateType dot = AB.dot(AC) / (dAB * dAC);
		if (fabs(dot) < 0.999)
		{
			//not colinear
			return false;
		}
		else
		{
			//colinear --> do they actually intersect?
			return (dAC < dAB || (D-A).norm() < dAB);
		}
	}
}

//list of already used point to avoid hull's inner loops
enum HullPointFlags {	POINT_NOT_USED	= 0,
						POINT_USED		= 1,
						POINT_IGNORED	= 2,
};

bool PointProjectionTools::extractConcaveHull2D(std::vector<IndexedCCVector2>& points,
												std::list<IndexedCCVector2*>& hullPoints,
												PointCoordinateType maxSquareEdgeLength/*=0*/)
{
	//first compute the Convex hull
	if (!extractConvexHull2D(points,hullPoints))
		return false;

	//shall we compute the concave hull?
	if (hullPoints.size() >= 2 && maxSquareEdgeLength > 0)
	{
		size_t pointCount = points.size();

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

		//hack: compute the 'minimal' edge length
		PointCoordinateType minSquareEdgeLength = 0;
		{
			CCVector2 minP,maxP;
			for (size_t i=0; i<pointCount; ++i)
			{
				const IndexedCCVector2& P = points[i];
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
			for (std::list<IndexedCCVector2*>::iterator itA = hullPoints.begin(); itA != hullPoints.end(); ++itA)
			{
				std::list<IndexedCCVector2*>::iterator itB = itA; ++itB;
				if (itB == hullPoints.end())
					itB = hullPoints.begin();
				if ((**itB-**itA).norm2() < minSquareEdgeLength)
				{
					pointFlags[(*itB)->index] = POINT_IGNORED;
					hullPoints.erase(itB);
				}
			}

			if (hullPoints.size() < 2)
			{
				//no more edges?!
				return false;
			}
		}

		//build the initial edge list & flag the convex hull points
		std::list<std::list<PointProjectionTools::IndexedCCVector2*>::iterator> edges;
		{
			for (std::list<PointProjectionTools::IndexedCCVector2*>::iterator itA = hullPoints.begin(); itA != hullPoints.end(); ++itA)
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
			std::list<PointProjectionTools::IndexedCCVector2*>::iterator itA = edges.front();
			std::list<PointProjectionTools::IndexedCCVector2*>::iterator itB = itA; ++itB;
			if (itB == hullPoints.end())
				itB = hullPoints.begin();

			edges.pop_front();

			//long edge?
			PointCoordinateType squareLengthAB = (**itB-**itA).norm2();
			if (squareLengthAB > maxSquareEdgeLength)
			{
				//look for the nearest point in the input set
				PointCoordinateType minDist2 = -1;
				size_t minIndex = 0;
				for (size_t i=0; i<pointCount; ++i)
				{
					const PointProjectionTools::IndexedCCVector2& P = points[i];
					if (pointFlags[P.index] == POINT_IGNORED)
						continue;

					//skip the edge vertices!
					if (P.index == (*itA)->index || P.index == (*itB)->index)
						continue;

					//we only consider 'inner' points
					if (cross(**itA, **itB, P) < 0)
					{
						continue;
					}

					PointCoordinateType dist2 = ComputeSquareDistToEdge(P,**itA,**itB);
					if (dist2 >= 0 && (minDist2 < 0 || dist2 < minDist2))
					{
						minDist2 = dist2;
						minIndex = i;
					}
				}

				//if we have found a candidate
				if (minDist2 >= 0)
				{
					const PointProjectionTools::IndexedCCVector2& P = points[minIndex];

					if (pointFlags[P.index] == POINT_NOT_USED) //we don't consider already used points!
					{
						CCVector2 AP = (P-**itA);
						CCVector2 PB = (**itB-P);
						PointCoordinateType squareLengthAP = AP.norm2();
						PointCoordinateType squareLengthPB = PB.norm2();
						//check that we don't create too small edges!
						if (squareLengthAP < minSquareEdgeLength || squareLengthPB < minSquareEdgeLength)
						{
							pointFlags[P.index] = POINT_IGNORED;
							edges.push_front(itA); //retest the edge!
						}
						//at least one of the new segments must be smaller than the initial one!
						else if ( squareLengthAP < squareLengthAB || squareLengthPB < squareLengthAB )
						{
							//now check that the point is not nearer to the neighbor edges
							//DGM: only if the edge could 'need' it!

							//next edge vertex (BC)
							std::list<PointProjectionTools::IndexedCCVector2*>::iterator itC = itB; ++itC;
							if (itC == hullPoints.end())
								itC = hullPoints.begin();

							PointCoordinateType dist2ToRight = -1;

							CCVector2 BC = (**itC-**itB);
							PointCoordinateType squareLengthBC = BC.norm2();
							if (squareLengthBC > maxSquareEdgeLength)
								dist2ToRight = ComputeSquareDistToEdge(P,**itB,**itC);

							if (dist2ToRight < 0 || minDist2 <= dist2ToRight)
							{
								//previous edge vertex (OA)
								std::list<PointProjectionTools::IndexedCCVector2*>::iterator itO = itA;
								if (itO == hullPoints.begin())
									itO = hullPoints.end();
								--itO;

								PointCoordinateType dist2ToLeft = -1;

								CCVector2 OA = (**itA-**itO);
								PointCoordinateType squareLengthOA = OA.norm2();
								if (squareLengthOA > maxSquareEdgeLength)
									dist2ToLeft = ComputeSquareDistToEdge(P,**itO,**itA);

								if (dist2ToLeft < 0 || minDist2 <= dist2ToLeft)
								{
									//last check: the new segments must not intersect with the actual hull!
									bool intersect = false;
									{
										for (std::list<IndexedCCVector2*>::iterator itI = hullPoints.begin(); itI != hullPoints.end(); ++itI)
										{
											std::list<IndexedCCVector2*>::iterator itJ = itI; ++itJ;
											if (itJ == hullPoints.end())
												itJ = hullPoints.begin();

											//we avoid testing with already connected segments!
											if (	(*itI)->index == (*itA)->index
												||	(*itJ)->index == (*itA)->index
												||	(*itI)->index == (*itB)->index
												||	(*itJ)->index == (*itB)->index )
												continue;

											if (Intersect(**itI,**itJ,**itA,P) || Intersect(**itI,**itJ,P,**itB))
											{
												intersect = true;
												break;
											}
										}
									}
									if (!intersect)
									{
										hullPoints.insert(itB == hullPoints.begin() ? hullPoints.end() : itB, &points[minIndex]);

										//we'll inspect the two new segments later
										try
										{
											if (squareLengthAP > maxSquareEdgeLength)
											{
												edges.push_back(itA);
											}
											if (squareLengthPB > maxSquareEdgeLength)
											{
												std::list<PointProjectionTools::IndexedCCVector2*>::iterator itP = itA; ++itP;
												edges.push_back(itP);
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
								}
							}
						}
					}
				} //end of candidate examination
			} //end of current edge examination
		} //no more edges
	}

	return true;
}