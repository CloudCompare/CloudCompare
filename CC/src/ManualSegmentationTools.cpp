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

#include "ManualSegmentationTools.h"

//local
#include "Matrix.h"
#include "CCTypes.h"
#include "GenericProgressCallback.h"
#include "GenericIndexedCloudPersist.h"
#include "ReferenceCloud.h"
#include "GenericIndexedMesh.h"
#include "SimpleMesh.h"
#include "Polyline.h"

//system
#include <string.h>
#include <assert.h>

using namespace CCLib;

ReferenceCloud* ManualSegmentationTools::segment(GenericIndexedCloudPersist* aCloud, const Polyline* poly, bool keepInside, const float* viewMat)
{
    assert(poly && aCloud);

	CCLib::SquareMatrix* trans = (viewMat ? new CCLib::SquareMatrix(viewMat) : 0);

	ReferenceCloud* Y = new ReferenceCloud(aCloud);

	//we check for each point if it falls inside the polyline
	unsigned count = aCloud->size();
	for (unsigned i=0; i<count; ++i)
	{
		CCVector3 P;
		aCloud->getPoint(i,P);

		//we project the point in screen space first if necessary
		if (trans)
			trans->apply(P.u);

		bool pointInside = isPointInsidePoly(CCVector2(P.x,P.y),poly);
		if ((keepInside && pointInside) || (!keepInside && !pointInside))
		{
			if (!Y->addPointIndex(i))
			{
				//not engouh memory
				delete Y;
				Y=0;
				break;
			}
		}
	}

	if (trans)
        delete trans;

	return Y;
}

bool ManualSegmentationTools::isPointInsidePoly(const CCVector2& P, const GenericIndexedCloud* polyVertices)
{
	//number of vertices
	unsigned vertCount = (polyVertices ? polyVertices->size() : 0);
	if (vertCount<2)
		return false;

	bool inside = false;

	CCVector3 A;
	polyVertices->getPoint(0,A);
	for (unsigned i=1; i<=vertCount; ++i)
	{
		CCVector3 B;
		polyVertices->getPoint(i%vertCount,B);

		//Point Inclusion in Polygon Test (inspired from W. Randolph Franklin - WRF)
		//The polyline is considered as a 2D polyline here!
		if ( (B.y<=P.y && P.y<A.y) || (A.y<=P.y && P.y<B.y) )
		{
			PointCoordinateType t = (P.x-B.x)*(A.y-B.y) - (A.x-B.x)*(P.y-B.y);
			if (A.y < B.y)
				t=-t;
			if (t < 0)
				inside = !inside;
		}

		A=B;
	}

	return inside;
}

bool ManualSegmentationTools::isPointInsidePoly(const CCVector2& P,
												const std::vector<CCVector2>& polyVertices)
{
	//number of vertices
	size_t vertCount = polyVertices.size();
	if (vertCount<2)
		return false;

	bool inside = false;

	for (unsigned i=1; i<=vertCount; ++i)
	{
		const CCVector2& A = polyVertices[i-1];
		const CCVector2& B = polyVertices[i%vertCount];

		//Point Inclusion in Polygon Test (inspired from W. Randolph Franklin - WRF)
		//The polyline is considered as a 2D polyline here!
		if ( (B.y<=P.y && P.y<A.y) || (A.y<=P.y && P.y<B.y) )
		{
			PointCoordinateType t = (P.x-B.x)*(A.y-B.y) - (A.x-B.x)*(P.y-B.y);
			if (A.y < B.y)
				t=-t;
			if (t < 0)
				inside = !inside;
		}
	}

	return inside;
}

ReferenceCloud* ManualSegmentationTools::segment(GenericIndexedCloudPersist* aCloud, ScalarType minDist, ScalarType maxDist)
{
	if (!aCloud)
	{
		assert(false);
		return 0;
	}

	ReferenceCloud* Y = new ReferenceCloud(aCloud);

	//for each point
	for (unsigned i=0; i<aCloud->size(); ++i)
	{
		const ScalarType dist = aCloud->getPointScalarValue(i);
		//we test if its assocaited scalar value falls inside the specified intervale
		if (dist >= minDist && dist <= maxDist)
		{
			if (!Y->addPointIndex(i))
			{
				//not engouh memory
				delete Y;
				Y=0;
				break;
			}
		}
	}

	return Y;
}

GenericIndexedMesh* ManualSegmentationTools::segmentMesh(GenericIndexedMesh* theMesh, ReferenceCloud* pointIndexes, bool pointsWillBeInside, GenericProgressCallback* progressCb, GenericIndexedCloud* destCloud, unsigned indexShift)
{
	if (!theMesh || !pointIndexes || !pointIndexes->getAssociatedCloud())
		return 0;

	//by default we try a fast process (but with a higher memory consumption)
	unsigned numberOfPoints = pointIndexes->getAssociatedCloud()->size();
	unsigned numberOfIndexes = pointIndexes->size();

	//we determine for each point if it is used in the output mesh or not
	//(and we compute its new index by the way: 0 means that the point is not used, otherwise its index will be newPointIndexes-1)
	std::vector<unsigned> newPointIndexes;
	{
		try
		{
			newPointIndexes.resize(numberOfPoints,0);
		}
		catch (std::bad_alloc)
		{
			return 0; //not enough memory
		}

		for (unsigned i=0; i<numberOfIndexes; ++i)
		{
			assert(pointIndexes->getPointGlobalIndex(i) < numberOfPoints);
			newPointIndexes[pointIndexes->getPointGlobalIndex(i)] = i+1;
		}
	}

	//negative array for the case where input points are "outside"
	if (!pointsWillBeInside)
	{
		unsigned newIndex = 0;
		for (unsigned i=0;i<numberOfPoints;++i)
			newPointIndexes[i] = (newPointIndexes[i] == 0 ? ++newIndex : 0);
	}

	//create resulting mesh
	SimpleMesh* newMesh = 0;
	{
		unsigned numberOfTriangles = theMesh->size();

		//progress notification
		NormalizedProgress* nprogress = 0;
		if (progressCb)
		{
			progressCb->reset();
			progressCb->setMethodTitle("Extract mesh");
			char buffer[256];
			sprintf(buffer,"New vertex number: %u",numberOfIndexes);
			nprogress = new NormalizedProgress(progressCb,numberOfTriangles);
			progressCb->setInfo(buffer);
			progressCb->start();
		}

		newMesh = new SimpleMesh(destCloud ? destCloud : pointIndexes->getAssociatedCloud());
		unsigned count = 0;

		theMesh->placeIteratorAtBegining();
		for (unsigned i=0; i<numberOfTriangles; ++i)
		{
			bool triangleIsOnTheRightSide = true;

			const TriangleSummitsIndexes* tsi = theMesh->getNextTriangleIndexes(); //DGM: getNextTriangleIndexes is faster for mesh groups!
			int newVertexIndexes[3];

			//VERSION: WE KEEP THE TRIANGLE ONLY IF ITS 3 VERTICES ARE INSIDE
			for (uchar j=0;j <3; ++j)
			{
				const unsigned& currentVertexFlag = newPointIndexes[tsi->i[j]];

				//if the vertex is rejected, we discard this triangle
				if (currentVertexFlag == 0)
				{
					triangleIsOnTheRightSide = false;
					break;
				}
				newVertexIndexes[j] = currentVertexFlag-1;
			}

			//if we keep the triangle
			if (triangleIsOnTheRightSide)
			{
				if (count == newMesh->size() && !newMesh->reserve(newMesh->size() + 1000)) //auto expand mesh size
				{
					//stop process
					delete newMesh;
					newMesh = 0;
					break;
				}
				++count;

				newMesh->addTriangle(	indexShift + newVertexIndexes[0],
										indexShift + newVertexIndexes[1],
										indexShift + newVertexIndexes[2] );
			}

			if (nprogress && !nprogress->oneStep())
			{
				//cancel process
				break;
			}
		}

		if (nprogress)
		{
			delete nprogress;
			nprogress = 0;
		}

		if (newMesh)
		{
			if (newMesh->size() == 0)
			{
				delete newMesh;
				newMesh = 0;
			}
			else if (count < newMesh->size())
			{
				newMesh->resize(count); //should always be ok as count<maxNumberOfTriangles
			}
		}
	}

	return newMesh;
}
