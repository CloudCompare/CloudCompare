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

#include <ManualSegmentationTools.h>

//local
#include <GenericProgressCallback.h>
#include <PointCloud.h>
#include <Polyline.h>
#include <SimpleMesh.h>

//system
#include <cstdint>
#include <map>


using namespace CCLib;

ReferenceCloud* ManualSegmentationTools::segment(GenericIndexedCloudPersist* aCloud, const Polyline* poly, bool keepInside, const float* viewMat)
{
	assert(poly && aCloud);

	CCLib::SquareMatrix* trans = (viewMat ? new CCLib::SquareMatrix(viewMat) : nullptr);

	ReferenceCloud* Y = new ReferenceCloud(aCloud);

	//we check for each point if it falls inside the polyline
	unsigned count = aCloud->size();
	for (unsigned i = 0; i < count; ++i)
	{
		CCVector3 P;
		aCloud->getPoint(i, P);

		//we project the point in screen space first if necessary
		if (trans)
		{
			P = (*trans) * P;
		}

		bool pointInside = isPointInsidePoly(CCVector2(P.x, P.y), poly);
		if ((keepInside && pointInside) || (!keepInside && !pointInside))
		{
			if (!Y->addPointIndex(i))
			{
				//not enough memory
				delete Y;
				Y = nullptr;
				break;
			}
		}
	}

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
	polyVertices->getPoint(0, A);
	for (unsigned i = 1; i <= vertCount; ++i)
	{
		CCVector3 B;
		polyVertices->getPoint(i % vertCount, B);

		//Point Inclusion in Polygon Test (inspired from W. Randolph Franklin - WRF)
		//The polyline is considered as a 2D polyline here!
		if ((B.y <= P.y && P.y < A.y) || (A.y <= P.y && P.y < B.y))
		{
			PointCoordinateType t = (P.x - B.x)*(A.y - B.y) - (A.x - B.x)*(P.y - B.y);
			if (A.y < B.y)
				t = -t;
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
	std::size_t vertCount = polyVertices.size();
	if (vertCount < 2)
		return false;

	bool inside = false;

	for (unsigned i = 1; i <= vertCount; ++i)
	{
		const CCVector2& A = polyVertices[i - 1];
		const CCVector2& B = polyVertices[i%vertCount];

		//Point Inclusion in Polygon Test (inspired from W. Randolph Franklin - WRF)
		//The polyline is considered as a 2D polyline here!
		if ((B.y <= P.y && P.y < A.y) || (A.y <= P.y && P.y < B.y))
		{
			PointCoordinateType t = (P.x - B.x)*(A.y - B.y) - (A.x - B.x)*(P.y - B.y);
			if (A.y < B.y)
				t = -t;
			if (t < 0)
				inside = !inside;
		}
	}

	return inside;
}

ReferenceCloud* ManualSegmentationTools::segmentReferenceCloud(ReferenceCloud* cloud,
	ScalarType minDist,
	ScalarType maxDist,
	bool outside/*=false*/)
{
	if (!cloud)
	{
		assert(false);
		return nullptr;
	}
	ReferenceCloud* Y = new ReferenceCloud(cloud->getAssociatedCloud());

	//for each point
	for (unsigned i = 0; i < cloud->size(); ++i)
	{
		const ScalarType dist = cloud->getPointScalarValue(i);
		//we test if its associated scalar value falls inside the specified interval
		if ((dist >= minDist && dist <= maxDist) ^ outside)
		{
			if (!Y->addPointIndex(cloud->getPointGlobalIndex(i)))
			{
				//not enough memory
				delete Y;
				Y = nullptr;
				break;
			}
		}
	}

	return Y;

}

ReferenceCloud* ManualSegmentationTools::segment(	GenericIndexedCloudPersist* cloud,
													ScalarType minDist,
													ScalarType maxDist,
													bool outside/*=false*/)
{
	if (!cloud)
	{
		assert(false);
		return nullptr;
	}

	ReferenceCloud* cloudREFTest = dynamic_cast<ReferenceCloud*>(cloud);
	if (cloudREFTest)
		return segmentReferenceCloud(cloudREFTest, minDist, maxDist, outside);

	ReferenceCloud* Y = new ReferenceCloud(cloud);

	//for each point
	for (unsigned i=0; i<cloud->size(); ++i)
	{
		const ScalarType dist = cloud->getPointScalarValue(i);
		//we test if its associated scalar value falls inside the specified interval
		if ((dist >= minDist && dist <= maxDist) ^ outside)
		{
			if (!Y->addPointIndex(i))
			{
				//not enough memory
				delete Y;
				Y = nullptr;
				break;
			}
		}
	}

	return Y;
}

GenericIndexedMesh* ManualSegmentationTools::segmentMesh(GenericIndexedMesh* theMesh, ReferenceCloud* pointIndexes, bool pointsWillBeInside, GenericProgressCallback* progressCb, GenericIndexedCloud* destCloud, unsigned indexShift)
{
	if (!theMesh || !pointIndexes || !pointIndexes->getAssociatedCloud())
		return nullptr;

	//by default we try a fast process (but with a higher memory consumption)
	unsigned numberOfPoints = pointIndexes->getAssociatedCloud()->size();
	unsigned numberOfIndexes = pointIndexes->size();

	//we determine for each point if it is used in the output mesh or not
	//(and we compute its new index by the way: 0 means that the point is not used, otherwise its index will be newPointIndexes-1)
	std::vector<unsigned> newPointIndexes;
	{
		try
		{
			newPointIndexes.resize(numberOfPoints, 0);
		}
		catch (const std::bad_alloc&)
		{
			return nullptr; //not enough memory
		}

		for (unsigned i = 0; i < numberOfIndexes; ++i)
		{
			assert(pointIndexes->getPointGlobalIndex(i) < numberOfPoints);
			newPointIndexes[pointIndexes->getPointGlobalIndex(i)] = i + 1;
		}
	}

	//negative array for the case where input points are "outside"
	if (!pointsWillBeInside)
	{
		unsigned newIndex = 0;
		for (unsigned i = 0; i < numberOfPoints; ++i)
			newPointIndexes[i] = (newPointIndexes[i] == 0 ? ++newIndex : 0);
	}

	//create resulting mesh
	SimpleMesh* newMesh = nullptr;
	{
		unsigned numberOfTriangles = theMesh->size();

		//progress notification
		NormalizedProgress nprogress(progressCb, numberOfTriangles);
		if (progressCb)
		{
			if (progressCb->textCanBeEdited())
			{
				progressCb->setMethodTitle("Extract mesh");
				char buffer[256];
				sprintf(buffer, "New vertex number: %u", numberOfIndexes);
				progressCb->setInfo(buffer);
			}
			progressCb->update(0);
			progressCb->start();
		}

		newMesh = new SimpleMesh(destCloud ? destCloud : pointIndexes->getAssociatedCloud());
		unsigned count = 0;

		theMesh->placeIteratorAtBeginning();
		for (unsigned i = 0; i < numberOfTriangles; ++i)
		{
			bool triangleIsOnTheRightSide = true;

			const VerticesIndexes* tsi = theMesh->getNextTriangleVertIndexes(); //DGM: getNextTriangleVertIndexes is faster for mesh groups!
			int newVertexIndexes[3];

			//VERSION: WE KEEP THE TRIANGLE ONLY IF ITS 3 VERTICES ARE INSIDE
			for (unsigned char j = 0; j < 3; ++j)
			{
				const unsigned& currentVertexFlag = newPointIndexes[tsi->i[j]];

				//if the vertex is rejected, we discard this triangle
				if (currentVertexFlag == 0)
				{
					triangleIsOnTheRightSide = false;
					break;
				}
				newVertexIndexes[j] = currentVertexFlag - 1;
			}

			//if we keep the triangle
			if (triangleIsOnTheRightSide)
			{
				if (count == newMesh->capacity() && !newMesh->reserve(newMesh->size() + 4096)) //auto expand mesh size
				{
					//stop process
					delete newMesh;
					newMesh = nullptr;
					break;
				}

				newMesh->addTriangle(	indexShift + newVertexIndexes[0],
										indexShift + newVertexIndexes[1],
										indexShift + newVertexIndexes[2] );
				++count;
			}

			if (progressCb && !nprogress.oneStep())
			{
				//cancel process
				break;
			}
		}

		if (newMesh)
		{
			if (newMesh->size() == 0)
			{
				delete newMesh;
				newMesh = nullptr;
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
const unsigned c_srcIndexFlag  = 0x40000000; //source index flag (bit 30)
const unsigned c_realIndexMask = 0x3FFFFFFF; //original index mask (bit 0 to 29) --> max allowed index = 1073741823 ;)
const unsigned c_defaultArrayGrowth = 1024;

struct InsideOutsideIndexes
{
	InsideOutsideIndexes() : insideIndex(0), outsideIndex(0) {}
	InsideOutsideIndexes(unsigned inside, unsigned outside) : insideIndex(inside), outsideIndex(outside) {}
	unsigned insideIndex;
	unsigned outsideIndex;
};
static std::map< uint64_t, InsideOutsideIndexes > s_edgePoint;

bool AddVertex(CCVector3d& P, PointCloud* vertices, unsigned& index)
{
	assert(vertices);
	//add vertex to the 'vertices' set
	unsigned vertCount = vertices->size();
	if (vertCount == vertices->capacity()
		&& !vertices->reserve(vertCount + c_defaultArrayGrowth))
	{
		//not enough memory!
		return false;
	}
	vertices->addPoint(CCVector3::fromArray(P.u));
	index = vertCount;
	return true;
}


bool ComputeEdgePoint(const CCVector3d& A, unsigned iA,
	const CCVector3d& B, unsigned iB,
	unsigned& iCoutside, unsigned& iCinside,
	double planeCoord, unsigned char planeDim,
	PointCloud* outsideVertices, PointCloud* insideVertices)
{
	assert(outsideVertices || insideVertices);

	//first look if we already know this edge
	uint64_t key = 0;
	{
		unsigned minIndex = iA;
		unsigned maxIndex = iB;
		if (minIndex > maxIndex)
			std::swap(minIndex, maxIndex);
		key = (static_cast<uint64_t>(minIndex) << 32) | static_cast<uint64_t>(maxIndex);
	}
	
	//if the key (edge) already exists
	if (s_edgePoint.find(key) != s_edgePoint.end())
	{
		const InsideOutsideIndexes& pmi = s_edgePoint[key];
		iCoutside = pmi.outsideIndex;
		iCinside = pmi.insideIndex;
	}
	//otherwise we'll create it
	else
	{
		CCVector3d I = A + (B - A) * (planeCoord - A.u[planeDim]) / (B.u[planeDim] - A.u[planeDim]);

		//add vertex to the inside 'vertices' set
		iCinside = 0;
		if (insideVertices && !AddVertex(I, insideVertices, iCinside))
			return false;
		//add vertex to the outside 'vertices' set
		iCoutside = 0;
		if (outsideVertices && !AddVertex(I, outsideVertices, iCoutside))
			return false;

		s_edgePoint[key] = InsideOutsideIndexes(iCinside, iCoutside);
	}

	return true;
}

bool AddTriangle(unsigned iA, unsigned iB, unsigned iC,
	SimpleMesh* mesh,
	bool directOrder)
{
	//special case: the mesh might not exist (if we skip the 'outside' mesh creation)
	//so we accept this eventuallity to simply the code
	if (!mesh)
		return true;

	//now add the triangle
	if (	mesh->size() == mesh->capacity()
		&& (mesh->size() + c_defaultArrayGrowth > c_realIndexMask
		|| !mesh->reserve(mesh->size() + c_defaultArrayGrowth)))
	{
		//not enough memory (or too many triangles!)
		return false;
	}

	if (directOrder)
		mesh->addTriangle(iA, iB, iC);
	else
		mesh->addTriangle(iA, iC, iB);

	return true;
}

bool MergeOldTriangles(	GenericIndexedMesh* origMesh,
						GenericIndexedCloudPersist* origVertices,
						SimpleMesh* newMesh,
						PointCloud* newVertices,
						const std::vector<unsigned>& preservedTriangleIndexes,
						std::vector<unsigned>* origTriIndexesMap = nullptr)
{
	assert(origMesh && origVertices && newMesh && newVertices);
	
	unsigned importedTriCount = static_cast<unsigned>(preservedTriangleIndexes.size());
 	unsigned origVertCount = origVertices->size();
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
				const VerticesIndexes* tsi = origMesh->getTriangleVertIndexes(triIndex);
				newIndexMap[tsi->i1] = 1;
				newIndexMap[tsi->i2] = 1;
				newIndexMap[tsi->i3] = 1;
			}
		}

		//or by the new triangles
		{
			for (unsigned i = 0; i < newTriCount; ++i)
			{
				const VerticesIndexes* tsi = newMesh->getTriangleVertIndexes(i);
				if (tsi->i1 & c_origIndexFlag)
					newIndexMap[tsi->i1 & c_realIndexMask] = 1;
				if (tsi->i2 & c_origIndexFlag)
					newIndexMap[tsi->i2 & c_realIndexMask] = 1;
				if (tsi->i3 & c_origIndexFlag)
					newIndexMap[tsi->i3 & c_realIndexMask] = 1;
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
				VerticesIndexes* tsi = newMesh->getTriangleVertIndexes(i);
				if (tsi->i1 & c_origIndexFlag)
					tsi->i1 = newIndexMap[tsi->i1 & c_realIndexMask];
				if (tsi->i2 & c_origIndexFlag)
					tsi->i2 = newIndexMap[tsi->i2 & c_realIndexMask];
				if (tsi->i3 & c_origIndexFlag)
					tsi->i3 = newIndexMap[tsi->i3 & c_realIndexMask];
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
				assert(!origTriIndexesMap || newMesh->size() == origTriIndexesMap->size());
				for (unsigned i = 0; i < importedTriCount; ++i)
				{
					unsigned triIndex = preservedTriangleIndexes[i];
					const VerticesIndexes* tsi = origMesh->getTriangleVertIndexes(triIndex);
					newMesh->addTriangle(newIndexMap[tsi->i1], newIndexMap[tsi->i2], newIndexMap[tsi->i3]);
					if (origTriIndexesMap)
						origTriIndexesMap->push_back(triIndex);
				}
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	newMesh->resize(newMesh->size());
	newVertices->resize(newVertices->size());

	assert(!origTriIndexesMap || newMesh->size() == origTriIndexesMap->size());

	return true;
}

bool ImportSourceVertices(GenericIndexedCloudPersist* srcVertices,
							SimpleMesh* newMesh,
							PointCloud* newVertices)
{
	assert(srcVertices && newMesh && newVertices);

	unsigned srcVertCount = srcVertices->size();
	unsigned newVertCount = newVertices->size();
	unsigned newTriCount = newMesh->size();

	try
	{
		//first determine the number of source vertices that should be imported
		std::vector<unsigned> newIndexMap;
		newIndexMap.resize(srcVertCount, 0);

		for (unsigned i = 0; i < newTriCount; ++i)
		{
			const VerticesIndexes* tsi = newMesh->getTriangleVertIndexes(i);
			if (tsi->i1 & c_srcIndexFlag)
				newIndexMap[tsi->i1 & c_realIndexMask] = 1;
			if (tsi->i2 & c_srcIndexFlag)
				newIndexMap[tsi->i2 & c_realIndexMask] = 1;
			if (tsi->i3 & c_srcIndexFlag)
				newIndexMap[tsi->i3 & c_realIndexMask] = 1;
		}

		//count the number of used vertices
		unsigned importedVertCount = 0;
		{
			for (unsigned i = 0; i < srcVertCount; ++i)
				if (newIndexMap[i])
					++importedVertCount;
		}

		if (importedVertCount == 0)
		{
			//nothing to do
			//(shouldn't happen but who knows?)
			return true;
		}

		//reserve the memory to import the source vertices
		if (!newVertices->reserve(newVertices->size() + importedVertCount))
		{
			//not enough memory
			return false;
		}
		//then copy them
		{
			//update the destination indexes by the way
			unsigned lastVertIndex = newVertCount;
			for (unsigned i = 0; i < srcVertCount; ++i)
			{
				if (newIndexMap[i])
				{
					newVertices->addPoint(*srcVertices->getPoint(i));
					newIndexMap[i] = lastVertIndex++;
				}
			}
		}

		//update the existing indexes
		{
			for (unsigned i = 0; i < newTriCount; ++i)
			{
				VerticesIndexes* tsi = newMesh->getTriangleVertIndexes(i);
				if (tsi->i1 & c_srcIndexFlag)
					tsi->i1 = newIndexMap[tsi->i1 & c_realIndexMask];
				if (tsi->i2 & c_srcIndexFlag)
					tsi->i2 = newIndexMap[tsi->i2 & c_realIndexMask];
				if (tsi->i3 & c_srcIndexFlag)
					tsi->i3 = newIndexMap[tsi->i3 & c_realIndexMask];
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	newVertices->resize(newVertices->size());

	return true;
}
bool ManualSegmentationTools::segmentMeshWithAAPlane(GenericIndexedMesh* mesh,
	GenericIndexedCloudPersist* vertices,
	MeshCutterParams& ioParams,
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

	if (mesh->size() > c_realIndexMask)
	{
		//too many triangles!
		return false;
	}

	//should be empty (just in case)
	s_edgePoint.clear();

	//working dimensions
	unsigned char Z = ioParams.planeOrthoDim;
	//unsigned char X = (Z == 2 ? 0 : Z + 1);
	//unsigned char Y = (X == 2 ? 0 : X + 1);

	const double& epsilon = ioParams.epsilon;
	const double& planeZ = ioParams.planeCoord;

	//indexes of original triangle that are not modified bt copied "as is"
	std::vector<unsigned> preservedTrianglesMinus;
	std::vector<unsigned> preservedTrianglesPlus;

	PointCloud* insideVertices = new PointCloud;
	SimpleMesh* minusMesh = new SimpleMesh(insideVertices, true);
	PointCloud* outsideVertices = new PointCloud;
	SimpleMesh* plusMesh = new SimpleMesh(outsideVertices, true);

	bool error = false;

	//for each triangle
	try
	{
		unsigned triCount = mesh->size();
		for (unsigned i = 0; i < triCount; ++i)
		{
			//original vertices indexes
			const VerticesIndexes* tsi = mesh->getTriangleVertIndexes(i);
			CCVector3d V[3] = { CCVector3d::fromArray(vertices->getPoint(tsi->i1)->u),
								CCVector3d::fromArray(vertices->getPoint(tsi->i2)->u),
								CCVector3d::fromArray(vertices->getPoint(tsi->i3)->u) };

			const unsigned origVertIndexes[3] = {
				tsi->i1 | c_origIndexFlag,
				tsi->i2 | c_origIndexFlag,
				tsi->i3 | c_origIndexFlag };

			//test each vertex
			//char relativePos[3] = { 1, 1, 1 };
			std::vector<unsigned char> minusVertIndexes;
			std::vector<unsigned char> plusVertIndexes;
			for (unsigned char j = 0; j < 3; ++j)
			{
				const CCVector3d& v = V[j];
				if (std::abs(v.u[Z] - planeZ) < epsilon)
				{
					//relativePos[j] = 0;
				}
				else
				{
					if (v.u[Z] < planeZ)
					{
						minusVertIndexes.push_back(j);
						//relativePos[j] = -1;
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
				//the triangle is either on one side or another ;)
				//const std::vector<unsigned char>& nonEmptySet = (insideVertices.empty() ? outsideVertices : insideVertices);
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

					unsigned iCoutside;
					unsigned iCinside;
					if (!ComputeEdgePoint(V[iMinus], origVertIndexes[iMinus],
						V[iPlus], origVertIndexes[iPlus],
						iCoutside, iCinside,
						planeZ, Z,
						outsideVertices, insideVertices))
					{
						//early stop
						i = triCount;
						error = true;
						break;
					}

					//we can now create two triangles
					unsigned char iCenter = 3 - iMinus - iPlus;
					if (!AddTriangle(
						origVertIndexes[iCenter],
						origVertIndexes[iMinus],
						iCinside,
						minusMesh,
						((iCenter + 1) % 3) == iMinus)

					||	!AddTriangle(
						origVertIndexes[iCenter],
						origVertIndexes[iPlus],
						iCoutside,
						plusMesh,
						((iCenter + 1) % 3) == iPlus))
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
					unsigned char iLeft;
					unsigned char iRight1;
					unsigned char iRight2;
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
					unsigned i1outside;
					unsigned i1inside;
					unsigned i2outside;
					unsigned i2inside;
					if (!ComputeEdgePoint(V[iRight1], origVertIndexes[iRight1], V[iLeft], origVertIndexes[iLeft], i1outside, i1inside, planeZ, Z, outsideVertices, insideVertices)
					||	!ComputeEdgePoint(V[iRight2], origVertIndexes[iRight2], V[iLeft], origVertIndexes[iLeft], i2outside, i2inside, planeZ, Z, outsideVertices, insideVertices))
					{
						//early stop
						i = triCount;
						error = true;
						break;
					}

					//we are going to create 3 triangles
					if (!AddTriangle(
						origVertIndexes[iLeft],
						leftIsMinus ? i1inside : i1outside,
						leftIsMinus ? i2inside : i2outside,
						leftIsMinus ? minusMesh : plusMesh,
						((iLeft + 1) % 3) == iRight1)

					||	!AddTriangle(
						leftIsMinus ? i1outside : i1inside,
						leftIsMinus ? i2outside : i2inside,
						origVertIndexes[iRight1],
						leftIsMinus ? plusMesh : minusMesh,
						((iRight2 + 1) % 3) == iRight1)

					||	!AddTriangle(
						origVertIndexes[iRight1],
						leftIsMinus ? i2outside : i2inside,
						origVertIndexes[iRight2],
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
	catch (const std::bad_alloc&)
	{
		//not enough memory
		error = true;
	}

	//free some memory
	s_edgePoint.clear();

	if (!error)
	{
		//import the 'preserved' (original) triangles 
		if (	!MergeOldTriangles(mesh, vertices, minusMesh, insideVertices, preservedTrianglesMinus)
			||	!MergeOldTriangles(mesh, vertices, plusMesh, outsideVertices, preservedTrianglesPlus))
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

	ioParams.insideMesh  = minusMesh;
	ioParams.outsideMesh = plusMesh;
	return true;
}

bool ManualSegmentationTools::segmentMeshWithAABox(GenericIndexedMesh* origMesh,
	GenericIndexedCloudPersist* origVertices,
	MeshCutterParams& ioParams,
	GenericProgressCallback* progressCb/*=0*/)
{
	if (!origMesh
		|| !origVertices
		|| origMesh->size() == 0
		|| origVertices->size() < 3
		|| ioParams.bbMin.x >= ioParams.bbMax.x
		|| ioParams.bbMin.y >= ioParams.bbMax.y
		|| ioParams.bbMin.z >= ioParams.bbMax.z)
	{
		//invalid input parameters
		return false;
	}

	if (origMesh->size() > c_realIndexMask)
	{
		//too many triangles!
		return false;
	}

	const double& epsilon = ioParams.epsilon;
	const CCVector3d& bbMin = ioParams.bbMin;
	const CCVector3d& bbMax = ioParams.bbMax;

	//indexes of original triangle that are not modified bt copied "as is"
	std::vector<unsigned> preservedTrianglesInside1;	//insde (1)
	std::vector<unsigned> preservedTrianglesInside2;	//insde (2)
	std::vector<unsigned> preservedTrianglesOutside;	//outside

	//inside meshes (swapped for each dimension)
	PointCloud* insideVertices1 = new PointCloud;
	SimpleMesh* insideMesh1 = new SimpleMesh(insideVertices1, true);
	PointCloud* insideVertices2 = new PointCloud;
	SimpleMesh* insideMesh2 = new SimpleMesh(insideVertices2, true);
	
	//outside mesh (output)
	PointCloud* outsideVertices = nullptr;
	SimpleMesh* outsideMesh = nullptr;
	if (ioParams.generateOutsideMesh)
	{
		outsideVertices = new PointCloud;
		outsideMesh = new SimpleMesh(outsideVertices, true);
	}

	//pointers on input and output structures (will change for each dimension)
	std::vector<unsigned>* preservedTrianglesInside = &preservedTrianglesInside1;
	std::vector<unsigned>* formerPreservedTriangles = &preservedTrianglesInside2;
	PointCloud* insideVertices = insideVertices1;
	SimpleMesh* insideMesh = insideMesh1;
	GenericIndexedMesh* sourceMesh = origMesh;
	GenericIndexedCloudPersist* sourceVertices = origVertices;
	
	CCVector3d boxCenter = (ioParams.bbMin + ioParams.bbMax) / 2;
	CCVector3d boxHalfSize = (ioParams.bbMax - ioParams.bbMin) / 2;
	bool error = false;

	//for each triangle
	try
	{
		//for each plane
		for (unsigned d = 0; d < 6; ++d)
		{
			//Extract the 'plane' information corresponding to the input box faces
			//-X,+X,-Y,+Y,-Z,+Z
			unsigned char Z = static_cast<unsigned char>(d / 2); 
			double planeCoord = ((d & 1) ? bbMax : bbMin).u[Z];
			bool keepBelow = ((d & 1) ? true : false);

			assert(preservedTrianglesInside && formerPreservedTriangles);
			assert(insideVertices && insideMesh);
			assert(sourceVertices && sourceMesh);
			s_edgePoint.clear();

			std::vector<unsigned> origTriIndexesMapInsideBackup;
			if (ioParams.trackOrigIndexes)
			{
				origTriIndexesMapInsideBackup = ioParams.origTriIndexesMapInside;
				ioParams.origTriIndexesMapInside.resize(0);
			}

			//look for original triangles
			//(the first time they only come from the original mesh but afterwards
			// they can come from the original mesh through the 'preserved' list
			// or from the previous 'inside' mesh as we have to test those triangles
			// against the new plane)
			unsigned sourceTriCount = sourceMesh ? sourceMesh->size() : 0; //source: previous/original mesh
			unsigned formerPreservedTriCount = static_cast<unsigned>(formerPreservedTriangles->size());
			unsigned triCount = sourceTriCount + formerPreservedTriCount;
			
			for (unsigned i = 0; i < triCount; ++i)
			{
				bool triangleIsOriginal = false;
				unsigned souceTriIndex = 0;
				const VerticesIndexes* tsi = nullptr;
				if (i < sourceTriCount)
				{
					souceTriIndex = i;
					triangleIsOriginal = (sourceMesh == origMesh);
					tsi = sourceMesh->getTriangleVertIndexes(souceTriIndex);
				}
				else
				{
					souceTriIndex = (*formerPreservedTriangles)[i - sourceTriCount];
					triangleIsOriginal = true;
					tsi = origMesh->getTriangleVertIndexes(souceTriIndex);
				}

				//vertices indexes
				unsigned vertIndexes[3] = { tsi->i1, tsi->i2, tsi->i3 };
				if (triangleIsOriginal)
				{
					//we flag the vertices indexes as referring to the 'original' mesh
					vertIndexes[0] |= c_origIndexFlag;
					vertIndexes[1] |= c_origIndexFlag;
					vertIndexes[2] |= c_origIndexFlag;
				}
				else
				{
					//we flag the vertices indexes as referring to the 'source' mesh
					if ((vertIndexes[0] & c_origIndexFlag) == 0)
						vertIndexes[0] |= c_srcIndexFlag;
					if ((vertIndexes[1] & c_origIndexFlag) == 0)
						vertIndexes[1] |= c_srcIndexFlag;
					if ((vertIndexes[2] & c_origIndexFlag) == 0)
						vertIndexes[2] |= c_srcIndexFlag;
				}

				//get the vertices (from the right source!)
				CCVector3d V[3] = { CCVector3d::fromArray(( (vertIndexes[0] & c_origIndexFlag) ? origVertices : sourceVertices)->getPoint(vertIndexes[0] & c_realIndexMask)->u),
									CCVector3d::fromArray(( (vertIndexes[1] & c_origIndexFlag) ? origVertices : sourceVertices)->getPoint(vertIndexes[1] & c_realIndexMask)->u),
									CCVector3d::fromArray(( (vertIndexes[2] & c_origIndexFlag) ? origVertices : sourceVertices)->getPoint(vertIndexes[2] & c_realIndexMask)->u) };

				if (d == 0)
				{
					//perform a triangle-box overlap test the first time!
					if (!CCMiscTools::TriBoxOverlapd(boxCenter, boxHalfSize, V))
					{
						if (ioParams.generateOutsideMesh)
							preservedTrianglesOutside.push_back(i);
						continue;
					}
				}

				//test the position of each vertex relatively to the current plane
				//char relativePos[3] = { 1, 1, 1 };
				//bool insideXY[3] = { false, false, false };
				std::vector<unsigned char> insideLocalVertIndexes;
				std::vector<unsigned char> outsideLocalVertIndexes;
				for (unsigned char j = 0; j < 3; ++j)
				{
					const CCVector3d& v = V[j];
					if (std::abs(v.u[Z] - planeCoord) < epsilon)
					{
						//relativePos[j] = 0;
					}
					else
					{
						if (v.u[Z] < planeCoord)
						{
							insideLocalVertIndexes.push_back(j);
							//relativePos[j] = -1;
						}
						else
						{
							//relativePos is already equal to 1
							//relativePos[j] = 1;
							outsideLocalVertIndexes.push_back(j);
						}
					}
				}

				//depending on the number of entities on the plane
				//we'll process the triangles differently
				bool isFullyInside = false;
				bool isFullyOutside = false;
				switch (insideLocalVertIndexes.size() + outsideLocalVertIndexes.size())
				{
				case 0: //all vertices 'in' the plane
				{
					//we arbitrarily decide that the triangle is inside!
					isFullyInside = true;
				}
				break;

				case 1: //2 vertices 'in' the plane
				{
					//the triangle is either on one side or another ;)
					if (insideLocalVertIndexes.empty())
					{
						//the only vertex far from the plane is on the 'otuside'
						isFullyOutside = true;
					}
					else
					{
						//the only vertex far from the plane is on the 'inside'
						isFullyInside = true;
					}
				}
				break;

				case 2: //1 vertex 'in' the plane
				{
					//3 cases:
					if (insideLocalVertIndexes.empty())
					{
						//the two vertices far from the plane are 'outside'
						isFullyOutside = true;
					}
					else if (outsideLocalVertIndexes.empty())
					{
						//the two vertices far from the plane are 'inside'
						isFullyInside = true;
					}
					else
					{
						//the two vertices far from the plane are on both sides
						//the plane will cut through the edge connecting those two vertices
						unsigned char iInside = insideLocalVertIndexes.front();
						unsigned char iOuside = outsideLocalVertIndexes.front();

						unsigned char iCenter = 3 - iInside - iOuside;
						unsigned iCoutside;
						unsigned iCinside;
						//we can now create one vertex and two new triangles
						if (!ComputeEdgePoint(
							V[iInside], vertIndexes[iInside],
							V[iOuside], vertIndexes[iOuside],
							iCoutside, iCinside,
							planeCoord, Z,
							outsideVertices, insideVertices)

						|| !AddTriangle(
							vertIndexes[iCenter],
							vertIndexes[iInside],
							keepBelow ? iCinside : iCoutside,
							keepBelow ? insideMesh : outsideMesh,
							((iCenter + 1) % 3) == iInside)

						|| !AddTriangle(
							vertIndexes[iCenter],
							vertIndexes[iOuside],
							keepBelow ? iCoutside : iCinside,
							keepBelow ? outsideMesh : insideMesh,
							((iCenter + 1) % 3) == iOuside))
						{
							//early stop
							i = triCount;
							error = true;
							break;
						}

						//remember (origin) source triangle index
						if (ioParams.trackOrigIndexes)
						{
							assert(triangleIsOriginal || souceTriIndex < origTriIndexesMapInsideBackup.size());
							unsigned origTriIndex = triangleIsOriginal ? souceTriIndex : origTriIndexesMapInsideBackup[souceTriIndex];
							//the source triangle is split in two so each side get one new triangle
							ioParams.origTriIndexesMapInside.push_back(origTriIndex);
							if (ioParams.generateOutsideMesh)
								ioParams.origTriIndexesMapOutside.push_back(origTriIndex);
						}
					}
				}
				break;

				case 3: //no vertex 'in' the plane
				{
					if (insideLocalVertIndexes.empty())
					{
						//all vertices are 'outside'
						isFullyOutside = true;
					}
					else if (outsideLocalVertIndexes.empty())
					{
						//all vertices are 'inside'
						isFullyInside = true;
					}
					else
					{
						//we have one vertex on one side and two on the other side
						unsigned char iLeft;
						unsigned char iRight1;
						unsigned char iRight2;
						bool leftIsInside = true;
						if (insideLocalVertIndexes.size() == 1)
						{
							assert(outsideLocalVertIndexes.size() == 2);
							iLeft = insideLocalVertIndexes.front();
							iRight1 = outsideLocalVertIndexes[0];
							iRight2 = outsideLocalVertIndexes[1];
							leftIsInside = keepBelow;
						}
						else
						{
							assert(insideLocalVertIndexes.size() == 2);
							iLeft = outsideLocalVertIndexes.front();
							iRight1 = insideLocalVertIndexes[0];
							iRight2 = insideLocalVertIndexes[1];
							leftIsInside = !keepBelow;
						}

						//the plane cuts through the two edges having the 'single' vertex in common
						//we are going to create 3 triangles
						unsigned i1outside;
						unsigned i1inside;
						unsigned i2outside;
						unsigned i2inside;
						if (  !ComputeEdgePoint(	V[iRight1], vertIndexes[iRight1],
													V[iLeft], vertIndexes[iLeft],
													i1outside, i1inside,
													planeCoord, Z,
													outsideVertices, insideVertices)
							
							|| !ComputeEdgePoint(	V[iRight2], vertIndexes[iRight2],
													V[iLeft], vertIndexes[iLeft],
													i2outside, i2inside,
													planeCoord, Z,
													outsideVertices, insideVertices)

							|| !AddTriangle(	vertIndexes[iLeft],
												leftIsInside ? i1inside : i1outside,
												leftIsInside ? i2inside : i2outside,
												leftIsInside ? insideMesh : outsideMesh,
												((iLeft + 1) % 3) == iRight1)

							|| !AddTriangle(	leftIsInside ? i1outside : i1inside,
												leftIsInside ? i2outside : i2inside,
												vertIndexes[iRight1],
												leftIsInside ? outsideMesh : insideMesh,
												((iRight2 + 1) % 3) == iRight1)

							|| !AddTriangle(	vertIndexes[iRight1],
												leftIsInside ? i2outside : i2inside,
												vertIndexes[iRight2],
												leftIsInside ? outsideMesh : insideMesh,
												((iRight2 + 1) % 3) == iRight1)
							)
						{
							//early stop
							i = triCount;
							error = true;
							break;
						}

						//remember (origin) source triangle index
						if (ioParams.trackOrigIndexes)
						{
							assert(triangleIsOriginal || souceTriIndex < origTriIndexesMapInsideBackup.size());
							unsigned origTriIndex = triangleIsOriginal ? souceTriIndex : origTriIndexesMapInsideBackup[souceTriIndex];
							//each side gets at least one new triangle
							ioParams.origTriIndexesMapInside.push_back(origTriIndex);
							if (ioParams.generateOutsideMesh)
								ioParams.origTriIndexesMapOutside.push_back(origTriIndex);
							//the third triangle has been added either to the 'inside' or to the 'outside' mesh
							if (!leftIsInside)
								ioParams.origTriIndexesMapInside.push_back(origTriIndex);
							else if (ioParams.generateOutsideMesh)
								ioParams.origTriIndexesMapOutside.push_back(origTriIndex);
						}
					}
				}
				break;

				}

				if (isFullyInside || isFullyOutside)
				{
					//inverted selection?
					if (!keepBelow)
						std::swap(isFullyInside, isFullyOutside);
					
					if (triangleIsOriginal)
					{
						if (isFullyInside)
							preservedTrianglesInside->push_back(souceTriIndex);
						else if (ioParams.generateOutsideMesh)
							preservedTrianglesOutside.push_back(souceTriIndex);
					}
					else
					{
						//we import the former triangle
						if (!AddTriangle(vertIndexes[0], vertIndexes[1], vertIndexes[2], isFullyInside ? insideMesh : outsideMesh, true))
						{
							//early stop
							error = true;
							break;
						}
						if (ioParams.trackOrigIndexes)
						{
							assert(souceTriIndex < origTriIndexesMapInsideBackup.size());
							unsigned origTriIndex = origTriIndexesMapInsideBackup[souceTriIndex];
							if (isFullyInside)
								ioParams.origTriIndexesMapInside.push_back(origTriIndex);
							else if (ioParams.generateOutsideMesh)
								ioParams.origTriIndexesMapOutside.push_back(origTriIndex);
						}
					}
				}

			}
			//end for each triangle

			if (   !ImportSourceVertices(sourceVertices, insideMesh, insideVertices)
				|| (ioParams.generateOutsideMesh && !ImportSourceVertices(sourceVertices, outsideMesh, outsideVertices))
				)
			{
				//early stop
				error = true;
				break;
			}

			if (insideMesh->size() == 0 && preservedTrianglesInside->empty())
			{
				//no triangle inside!
				break;
			}

			if (d < 5)
			{
				//clear the source mesh and swap the buffers
				if (insideMesh == insideMesh1)
				{
					assert(sourceMesh == insideMesh2 || sourceMesh == origMesh);
					insideMesh2->clear();
					insideVertices2->reset();
					sourceMesh = insideMesh1;
					sourceVertices = insideVertices1;
					insideMesh = insideMesh2;
					insideVertices = insideVertices2;
					preservedTrianglesInside2.resize(0);
					preservedTrianglesInside = &preservedTrianglesInside2;
					formerPreservedTriangles = &preservedTrianglesInside1;
				}
				else
				{
					assert(sourceMesh == insideMesh1 || sourceMesh == origMesh);
					insideMesh1->clear();
					insideVertices1->reset();
					sourceMesh = insideMesh2;
					sourceVertices = insideVertices2;
					insideMesh = insideMesh1;
					insideVertices = insideVertices1;
					preservedTrianglesInside1.resize(0);
					preservedTrianglesInside = &preservedTrianglesInside1;
					formerPreservedTriangles = &preservedTrianglesInside2;
				}
			}
		}
		//end for each plane

		//now add the remaining triangles
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		error = true;
	}

	//free some memory
	s_edgePoint.clear();
	formerPreservedTriangles->resize(0);

	if (!error)
	{
		//import the 'preserved' (original) triangles 
		if (	!MergeOldTriangles(	origMesh, origVertices,
									insideMesh, insideVertices,
									*preservedTrianglesInside,
									ioParams.trackOrigIndexes ? &ioParams.origTriIndexesMapInside : nullptr)
			||	(	ioParams.generateOutsideMesh
				&&	!MergeOldTriangles(	origMesh, origVertices,
										outsideMesh, outsideVertices,
										preservedTrianglesOutside,
										ioParams.trackOrigIndexes ? &ioParams.origTriIndexesMapOutside : nullptr))
			)
		{
			error = true;
		}
	}

	if (insideMesh == insideMesh1)
	{
		delete insideMesh2;
		insideMesh2 = nullptr;
		insideVertices2 = nullptr;
	}
	else
	{
		delete insideMesh1;
		insideMesh1 = nullptr;
		insideVertices1 = nullptr;
	}

	if (error)
	{
		delete insideMesh;
		delete outsideMesh;
		return false;
	}

	if (insideMesh)
	{
		insideMesh->resize(insideMesh->size());
	}
	if (outsideMesh)
	{
		outsideMesh->resize(outsideMesh->size());
	}

	ioParams.insideMesh = insideMesh;
	ioParams.outsideMesh = outsideMesh;
	return true;
}
