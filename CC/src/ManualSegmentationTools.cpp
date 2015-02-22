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
#include "ChunkedPointCloud.h"

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
		{
			P = (*trans) * P;
		}

		bool pointInside = isPointInsidePoly(CCVector2(P.x,P.y),poly);
		if ((keepInside && pointInside) || (!keepInside && !pointInside))
		{
			if (!Y->addPointIndex(i))
			{
				//not engouh memory
				delete Y;
				Y = 0;
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

const unsigned c_origIndexFlag = 0x80000000; //original index flag (bit 31)
const unsigned c_origIndexMask = 0x7FFFFFFF; //original index maskl
const unsigned c_defaultArrayGrowth = 100;

#include <map>
#include <stdint.h> //for uint fixed-sized types

struct PlusMinusIndexes
{
	PlusMinusIndexes() : minusIndex(0), plusIndex(0) {}
	PlusMinusIndexes(unsigned plus, unsigned minus) : minusIndex(minus), plusIndex(plus) {}
	PlusMinusIndexes(const PlusMinusIndexes& pmi) : minusIndex(pmi.minusIndex), plusIndex(pmi.plusIndex){}
	unsigned minusIndex;
	unsigned plusIndex;
};
static std::map< uint64_t, PlusMinusIndexes > s_edgePoint;

bool ComputeEdgePoint(const CCVector3d& A, unsigned iA,
	const CCVector3d& B, unsigned iB,
	unsigned& iCplus, unsigned& iCminus,
	double planeCoord, unsigned char planeDim,
	ChunkedPointCloud* plusVertices, ChunkedPointCloud* minusVertices)
{
	assert(plusVertices && minusVertices);

	//first look if we already know this edge
	uint64_t key = 0;
	{
		unsigned minIndex = iA;
		unsigned maxIndex = iB;
		if (minIndex > maxIndex)
			std::swap(minIndex, maxIndex);
		key = (static_cast<uint64_t>(minIndex) << 32) | static_cast<uint64_t>(maxIndex);
	}
	
	//if the key (edge) alread exists
	if (s_edgePoint.find(key) != s_edgePoint.end())
	{
		const PlusMinusIndexes& pmi = s_edgePoint[key];
		iCplus = pmi.plusIndex;
		iCminus = pmi.minusIndex;
	}
	//otherwise we'll create it
	else
	{
		CCVector3d I = A + (B - A) * (planeCoord - A.u[planeDim]) / (B.u[planeDim] - A.u[planeDim]);

		//add vertex to the plus 'vertices' set
		{
			unsigned vertCount = plusVertices->size();
			if (vertCount == plusVertices->capacity()
				&& !plusVertices->reserve(vertCount + c_defaultArrayGrowth))
			{
				//not enough memory!
				return false;
			}
			plusVertices->addPoint(CCVector3::fromArray(I.u));
			iCplus = vertCount;
		}
		//add vertex to the minus 'vertices' set
		{
			unsigned vertCount = minusVertices->size();
			if (vertCount == minusVertices->capacity()
				&& !minusVertices->reserve(vertCount + c_defaultArrayGrowth))
			{
				//not enough memory!
				return false;
			}
			minusVertices->addPoint(CCVector3::fromArray(I.u));
			iCminus = vertCount;
		}

		s_edgePoint[key] = PlusMinusIndexes(iCplus,iCminus);
	}

	return true;
}

//bool AddTriangle(const CCVector3d& A, unsigned iA, bool existA,
//	const CCVector3d& B, unsigned iB, bool existB,
//	const CCVector3d& C, unsigned iC, bool existC,
//	SimpleMesh* mesh,
//	ChunkedPointCloud* vertices,
//	bool directOrder)
//{
//	assert(mesh && vertices);
//	
//	const CCVector3d* V[3] = { &A, &B, &C };
//	unsigned indexes[3] = { iA, iB, iC };
//	bool exists[3] = { existA, existB, existC };
//
//	//process each vertex
//	for (unsigned char i = 0; i < 3; ++i)
//	{
//		//if the vertex doesn't exist yet (neither in the
//		//original vertex set nor in the new one
//		if (!exists[i])
//		{
//			//add vertex to the 'vertices' set
//			unsigned vertCount = vertices->size();
//			if (	vertCount == vertices->capacity()
//				&&	!vertices->reserve(vertCount + c_defaultArrayGrowth))
//			{
//				//not enough memory!
//				return false;
//			}
//			vertices->addPoint(CCVector3::fromArray(V[i]->u));
//			indexes[i] = vertCount;
//		}
//		//otherwise the vertex is either:
//		// - an original vertex (with c_origIndexFlag set, so >> 1)
//		// - or a vertex already in the new set
//	}
//
//	//now add the triangle
//	if (	mesh->size() == mesh->capacity()
//		&& !mesh->reserve(mesh->size() + c_defaultArrayGrowth))
//	{
//		//not enough memory
//		return false;
//	}
//
//	if (directOrder)
//		mesh->addTriangle(indexes[0], indexes[1], indexes[2]);
//	else
//		mesh->addTriangle(indexes[0], indexes[2], indexes[1]);
//
//	return true;
//}

bool AddTriangle(unsigned iA, unsigned iB, unsigned iC,
	SimpleMesh* mesh,
	bool directOrder)
{
	assert(mesh);

	//now add the triangle
	if (	mesh->size() == mesh->capacity()
		&&	!mesh->reserve(mesh->size() + c_defaultArrayGrowth))
	{
		//not enough memory
		return false;
	}

	if (directOrder)
		mesh->addTriangle(iA, iB, iC);
	else
		mesh->addTriangle(iA, iC, iB);

	return true;
}

bool MergeOldTriangles(GenericIndexedMesh* origMesh,
	GenericIndexedCloudPersist* origVertices,
	SimpleMesh* newMesh,
	ChunkedPointCloud* newVertices,
	const std::vector<unsigned>& preservedTriangleIndexes)
{
	assert(origMesh && origVertices && newMesh && newVertices);
	
	unsigned importedTriCount = static_cast<unsigned>(preservedTriangleIndexes.size());
 	unsigned origVertCount = origVertices->size();
	unsigned origTriCount = origMesh->size();
	unsigned newVertCount = newVertices->size();
	unsigned newTriCount = newMesh->size();

	try
	{
		//first determine the number of original vertices that should be imported
		std::vector<unsigned> newIndexMap;
		newIndexMap.resize(origVertCount, 0);

		//either for the preserved triangles
		{
			for (unsigned i = 0; i < importedTriCount; ++i)
			{
				unsigned triIndex = preservedTriangleIndexes[i];
				const TriangleSummitsIndexes* tsi = origMesh->getTriangleIndexes(triIndex);
				newIndexMap[tsi->i1] = 1;
				newIndexMap[tsi->i2] = 1;
				newIndexMap[tsi->i3] = 1;
			}
		}

		//or by the new triangles
		{
			for (unsigned i = 0; i < newTriCount; ++i)
			{
				const TriangleSummitsIndexes* tsi = newMesh->getTriangleIndexes(i);
				if (tsi->i1 & c_origIndexFlag)
					newIndexMap[tsi->i1 & c_origIndexMask] = 1;
				if (tsi->i2 & c_origIndexFlag)
					newIndexMap[tsi->i2 & c_origIndexMask] = 1;
				if (tsi->i3 & c_origIndexFlag)
					newIndexMap[tsi->i3 & c_origIndexMask] = 1;
			}
		}

		//count the number of used vertices
		unsigned importedVertCount = 0;
		{
			for (unsigned i = 0; i < origVertCount; ++i)
				if (newIndexMap[i])
					++importedVertCount;
		}

		if (importedVertCount == 0)
		{
			//nothing to do
			//(shouldn't happen but who knows?)
			return true;
		}

		//reserve the memory to import the original vertices
		if (!newVertices->reserve(newVertices->size() + importedVertCount))
		{
			//not enough memory
			return false;
		}
		//then copy them
		{
			//update the destination indexes by the way
			unsigned lastVertIndex = newVertCount;
			for (unsigned i = 0; i < origVertCount; ++i)
			{
				if (newIndexMap[i])
				{
					newVertices->addPoint(*origVertices->getPoint(i));
					newIndexMap[i] = lastVertIndex++;
				}
			}
		}

		//update the existing indexes
		{
			for (unsigned i = 0; i < newTriCount; ++i)
			{
				TriangleSummitsIndexes* tsi = newMesh->getTriangleIndexes(i);
				if (tsi->i1 & c_origIndexFlag)
					tsi->i1 = newIndexMap[tsi->i1 & c_origIndexMask];
				if (tsi->i2 & c_origIndexFlag)
					tsi->i2 = newIndexMap[tsi->i2 & c_origIndexMask];
				if (tsi->i3 & c_origIndexFlag)
					tsi->i3 = newIndexMap[tsi->i3 & c_origIndexMask];
			}
		}

		if (importedTriCount)
		{
			//reserve the memory to import the original triangles
			if (!newMesh->reserve(newMesh->size() + importedTriCount))
			{
				//not enough memory
				return false;
			}
			//then copy them
			{
				for (unsigned i = 0; i < importedTriCount; ++i)
				{
					unsigned triIndex = preservedTriangleIndexes[i];
					const TriangleSummitsIndexes* tsi = origMesh->getTriangleIndexes(triIndex);
					newMesh->addTriangle(newIndexMap[tsi->i1], newIndexMap[tsi->i2], newIndexMap[tsi->i3]);
				}
			}
		}
	}
	catch (std::bad_alloc)
	{
		//not enough memory
		return false;
	}

	newMesh->resize(newMesh->size());
	newVertices->resize(newVertices->size());

	return true;
}

bool ManualSegmentationTools::segmentMeshWitAAPlane(GenericIndexedMesh* mesh,
	GenericIndexedCloudPersist* vertices,
	PlaneCutterParams& ioParams,
	GenericProgressCallback* progressCb/*=0*/)
{
	if (	!mesh
		||	!vertices
		|| mesh->size() == 0
		|| vertices->size() < 3
		||	ioParams.planeOrthoDim > 2)
	{
		//invalid input parameters
		return false;
	}

	//should be empty (just in case)
	s_edgePoint.clear();

	//working dimensions
	unsigned char Z = ioParams.planeOrthoDim;
	unsigned char X = (Z == 2 ? 0 : Z + 1);
	unsigned char Y = (X == 2 ? 0 : X + 1);

	const double& epsilon = ioParams.epsilon;
	const double& planeZ = ioParams.planeCoord;

	//indexes of original triangle that are not modified bt copied "as is"
	std::vector<unsigned> preservedTrianglesMinus;
	std::vector<unsigned> preservedTrianglesPlus;

	ChunkedPointCloud* minusVertices = new ChunkedPointCloud;
	SimpleMesh* minusMesh = new SimpleMesh(minusVertices, true);
	ChunkedPointCloud* plusVertices = new ChunkedPointCloud;
	SimpleMesh* plusMesh = new SimpleMesh(plusVertices, true);

	bool error = false;

	//for each triangle
	try
	{
		unsigned triCount = mesh->size();
		for (unsigned i = 0; i < triCount; ++i)
		{
			//original vertices
			const GenericTriangle* tri = mesh->_getTriangle(i);
			CCVector3d V[3] = { CCVector3d::fromArray(tri->_getA()->u),
								CCVector3d::fromArray(tri->_getB()->u),
								CCVector3d::fromArray(tri->_getC()->u) };

			//original vertices indexes
			const TriangleSummitsIndexes* tsi = mesh->getTriangleIndexes(i);
			unsigned origIndexes[3] = { tsi->i1, tsi->i2, tsi->i3 };

			//test each vertex
			char relativePos[3] = { 1, 1, 1 };
			std::vector<unsigned char> minusVertIndexes, plusVertIndexes;
			for (unsigned char j = 0; j < 3; ++j)
			{
				const CCVector3d& v = V[j];
				if (fabs(v.u[Z] - planeZ) < epsilon)
				{
					relativePos[j] = 0;
				}
				else
				{
					if (v.u[Z] < planeZ)
					{
						minusVertIndexes.push_back(j);
						relativePos[j] = -1;
					}
					else
					{
						//relativePos is already equal to 1
						//relativePos[j] = 1;
						plusVertIndexes.push_back(j);
					}
				}
			}

			//depending on the number of entities on the plane
			//we'll process the triangles differently
			switch (minusVertIndexes.size() + plusVertIndexes.size())
			{
			case 0: //all vertices 'in' the plane
			{
				//the triangle is inside the plane!
				preservedTrianglesMinus.push_back(i);
			}
			break;

			case 1: //2 vertices 'in' the plane
			{
				//the triangle is either on one side or antehr ;)
				//const std::vector<unsigned char>& nonEmptySet = (minusVertices.empty() ? plusVertices : minusVertices);
				//assert(nonEmptySet.size() != 0);
				if (minusVertIndexes.empty())
				{
					//the only vertex far from the plane is on the 'plus' side
					preservedTrianglesPlus.push_back(i);
				}
				else
				{
					//the only vertex far from the plane is on the 'minus' side
					preservedTrianglesMinus.push_back(i);
				}
			}
			break;

			case 2: //1 vertex 'in' the plane
			{
				//3 cases:
				if (minusVertIndexes.empty())
				{
					//the two vertices far from the plane are on the 'plus' side
					preservedTrianglesPlus.push_back(i);
				}
				else if (plusVertIndexes.empty())
				{
					//the two vertices far from the plane are on the 'minus' side
					preservedTrianglesMinus.push_back(i);
				}
				else
				{
					//the two vertices far from the plane are on both sides
					//the plane will cut through the edge connecting those two vertices
					unsigned char iMinus = minusVertIndexes.front();
					unsigned char iPlus = plusVertIndexes.front();

					unsigned iCplus,ICminus;
					if (!ComputeEdgePoint(V[iMinus], origIndexes[iMinus],
						V[iPlus], origIndexes[iPlus],
						iCplus, ICminus,
						planeZ, Z,
						plusVertices, minusVertices))
					{
						//early stop
						i = triCount;
						error = true;
						break;
					}

					//we can now create two triangles
					unsigned char i0 = 3 - iMinus - iPlus;
					if (!AddTriangle(
						origIndexes[i0] | c_origIndexFlag,
						origIndexes[iMinus] | c_origIndexFlag,
						ICminus,
						minusMesh,
						((i0 + 1) % 3) == iMinus)

					||	!AddTriangle(
						origIndexes[i0] | c_origIndexFlag,
						origIndexes[iPlus] | c_origIndexFlag,
						iCplus,
						plusMesh,
						((i0 + 1) % 3) == iPlus))
					{
						//early stop
						i = triCount;
						error = true;
						break;
					}
				}
			}
			break;

			case 3: //no vertex 'in' the plane
			{
				if (minusVertIndexes.empty())
				{
					//all vertices are on the 'plus' side
					preservedTrianglesPlus.push_back(i);
				}
				else if (plusVertIndexes.empty())
				{
					//all vertices are on the 'minus' side
					preservedTrianglesMinus.push_back(i);
				}
				else
				{
					//we have one vertex on one side and two on the other side
					unsigned char iLeft, iRight1, iRight2;
					bool leftIsMinus = true;
					if (minusVertIndexes.size() == 1)
					{
						assert(plusVertIndexes.size() == 2);
						iLeft = minusVertIndexes.front();
						iRight1 = plusVertIndexes[0];
						iRight2 = plusVertIndexes[1];
						leftIsMinus = true;
					}
					else
					{
						assert(minusVertIndexes.size() == 2);
						iLeft = plusVertIndexes.front();
						iRight1 = minusVertIndexes[0];
						iRight2 = minusVertIndexes[1];
						leftIsMinus = false;
					}

					//the plane cuts through the two edges having the 'single' vertex in common
					unsigned i1plus, i1minus;
					unsigned i2plus, i2minus;
					if (!ComputeEdgePoint(V[iRight1], origIndexes[iRight1], V[iLeft], origIndexes[iLeft], i1plus, i1minus, planeZ, Z, plusVertices, minusVertices)
					||	!ComputeEdgePoint(V[iRight2], origIndexes[iRight2], V[iLeft], origIndexes[iLeft], i2plus, i2minus, planeZ, Z, plusVertices, minusVertices))
					{
						//early stop
						i = triCount;
						error = true;
						break;
					}

					//we are going to create 3 triangles
					if (!AddTriangle(
						origIndexes[iLeft] | c_origIndexFlag,
						leftIsMinus ? i1minus : i1plus,
						leftIsMinus ? i2minus : i2plus,
						leftIsMinus ? minusMesh : plusMesh,
						((iLeft + 1) % 3) == iRight1)

					||	!AddTriangle(
						leftIsMinus ? i1plus : i1minus,
						leftIsMinus ? i2plus : i2minus,
						origIndexes[iRight1] | c_origIndexFlag,
						leftIsMinus ? plusMesh : minusMesh,
						((iRight2 + 1) % 3) == iRight1)

					||	!AddTriangle(
						origIndexes[iRight1] | c_origIndexFlag,
						leftIsMinus ? i2plus : i2minus,
						origIndexes[iRight2] | c_origIndexFlag,
						leftIsMinus ? plusMesh : minusMesh,
						((iRight2 + 1) % 3) == iRight1) )
					{
						//early stop
						i = triCount;
						error = true;
						break;
					}
				}
			}
			break;

			}
		}
		//end for each triangle

		//now add the remaining triangles
	}
	catch (std::bad_alloc)
	{
		//not enough memory
		error = true;
	}

	//free some memory
	s_edgePoint.clear();

	if (!error)
	{
		//import the 'preserved' (original) triangles 
		if (	!MergeOldTriangles(mesh, vertices, minusMesh, minusVertices, preservedTrianglesMinus)
			||	!MergeOldTriangles(mesh, vertices, plusMesh, plusVertices, preservedTrianglesPlus))
		{
			error = true;
		}
	}

	if (error)
	{
		delete minusMesh;
		delete plusMesh;
		return false;
	}

	ioParams.minusMesh = minusMesh;
	ioParams.plusMesh = plusMesh;
	return true;
}
