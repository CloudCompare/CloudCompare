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

#include <MeshSamplingTools.h>

//local
#include <GenericIndexedMesh.h>
#include <GenericProgressCallback.h>
#include <GenericTriangle.h>
#include <PointCloud.h>
#include <ScalarField.h>

//system
#include <random>

using namespace CCLib;

double MeshSamplingTools::computeMeshArea(GenericMesh* mesh)
{
	if (!mesh)
	{
		assert(false);
		return -1.0;
	}

	//total area
	double Stotal = 0.0;

	mesh->placeIteratorAtBeginning();
	for (unsigned n=0; n<mesh->size(); ++n)
	{
		GenericTriangle* tri = mesh->_getNextTriangle();

		//vertices
		const CCVector3* O = tri->_getA();
		const CCVector3* A = tri->_getB();
		const CCVector3* B = tri->_getC();

		//compute the area of the triangle (= half of the vector product norm)
		CCVector3 OA = *A - *O;
		CCVector3 OB = *B - *O;
		Stotal += OA.cross(OB).norm();
	}

	return Stotal / 2;
}

double MeshSamplingTools::computeMeshVolume(GenericMesh* mesh)
{
	if (!mesh)
	{
		assert(false);
		return -1.0;
	}

	//total volume
	double Vtotal = 0.0;

	CCVector3 origin;
	CCVector3 upperCorner;
	mesh->getBoundingBox(origin,upperCorner);

	mesh->placeIteratorAtBeginning();
	for (unsigned n=0; n<mesh->size(); ++n)
	{
		GenericTriangle* tri = mesh->_getNextTriangle();

		//vertices (expressed in the local mesh ref. so as to avoid numerical inaccuracies)
		const CCVector3 A = *tri->_getA() - origin;
		const CCVector3 B = *tri->_getB() - origin;
		const CCVector3 C = *tri->_getC() - origin;

		//see "EFFICIENT FEATURE EXTRACTION FOR 2D/3D OBJECTS IN MESH REPRESENTATION" by Cha Zhang and Tsuhan Chen (2001)
		//We compute the (signed) volume of the tetrahedron defined by each triangle and the origin
		double signedVol = (- static_cast<double>(C.x*B.y*A.z)
							+ static_cast<double>(B.x*C.y*A.z)
							+ static_cast<double>(C.x*A.y*B.z)
							- static_cast<double>(A.x*C.y*B.z)
							- static_cast<double>(B.x*A.y*C.z)
							+ static_cast<double>(A.x*B.y*C.z)) / 6;

		Vtotal += signedVol;
	}

	return std::abs(Vtotal); //in case the triangles are in the wrong order!
}

unsigned long long MeshSamplingTools::ComputeEdgeKey(unsigned i1, unsigned i2)
{
	//build unique index
	if (i1 > i2)
		std::swap(i1,i2);
	return ((static_cast<unsigned long long>(i2) << 32) | static_cast<unsigned long long>(i1));
}

void MeshSamplingTools::DecodeEdgeKey(unsigned long long key, unsigned& i1, unsigned& i2)
{
	i1 = static_cast<unsigned>(  key        & 0x00000000FFFFFFFF );
	i2 = static_cast<unsigned>( (key >> 32) & 0x00000000FFFFFFFF );
}

bool MeshSamplingTools::buildMeshEdgeUsageMap(GenericIndexedMesh* mesh, EdgeUsageMap& edgeMap)
{
	edgeMap.clear();

	if (!mesh)
		return false;

	try
	{
		mesh->placeIteratorAtBeginning();
		//for all triangles
		for (unsigned n=0; n<mesh->size(); ++n)
		{
			VerticesIndexes* tri = mesh->getNextTriangleVertIndexes();

			//for all edges
			for (unsigned j=0; j<3; ++j)
			{
				unsigned i1 = tri->i[j];
				unsigned i2 = tri->i[(j+1) % 3];
				//build unique index
				unsigned long long edgeKey = ComputeEdgeKey(i1,i2);
				++edgeMap[edgeKey];
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	return true;
}

bool MeshSamplingTools::computeMeshEdgesConnectivity(GenericIndexedMesh* mesh, EdgeConnectivityStats& stats)
{
	stats = EdgeConnectivityStats();

	if (!mesh)
		return false;

	//count the number of triangles using each edge
	EdgeUsageMap edgeCounters;
	if (!buildMeshEdgeUsageMap(mesh,edgeCounters))
		return false;

	//for all edges
	stats.edgesCount = static_cast<unsigned>(edgeCounters.size());
	for (std::map<unsigned long long, unsigned>::const_iterator edgeIt = edgeCounters.begin(); edgeIt != edgeCounters.end(); ++edgeIt)
	{
		assert(edgeIt->second != 0);
		if (edgeIt->second == 1)
			++stats.edgesNotShared;
		else if (edgeIt->second == 2)
			++stats.edgesSharedByTwo;
		else
			++stats.edgesSharedByMore;
	}

	return true;
}

bool MeshSamplingTools::flagMeshVerticesByType(GenericIndexedMesh* mesh, ScalarField* flags, EdgeConnectivityStats* stats/*=0*/)
{
	if (!mesh || !flags || flags->currentSize() == 0)
		return false;

	//'non-processed' flag
	flags->fill(NAN_VALUE);

	//count the number of triangles using each edge
	EdgeUsageMap edgeCounters;
	if (!buildMeshEdgeUsageMap(mesh, edgeCounters))
		return false;

	//now scan all the edges and flag their vertices
	{
		if (stats)
			stats->edgesCount = static_cast<unsigned>(edgeCounters.size());

		//for all edges
		for (std::map<unsigned long long, unsigned>::const_iterator edgeIt = edgeCounters.begin(); edgeIt != edgeCounters.end(); ++edgeIt)
		{
			unsigned i1;
			unsigned i2;
			DecodeEdgeKey(edgeIt->first, i1, i2);

			ScalarType flag = NAN_VALUE;
			if (edgeIt->second == 1)
			{
				//only one triangle uses this edge
				flag = static_cast<ScalarType>(VERTEX_BORDER);
				if (stats)
					++stats->edgesNotShared;
			}
			else if (edgeIt->second == 2)
			{
				//two triangles use this edge
				flag = static_cast<ScalarType>(VERTEX_NORMAL);
				if (stats)
					++stats->edgesSharedByTwo;
			}
			else if (edgeIt->second > 2)
			{
				//more than two triangles use this edge!
				flag = static_cast<ScalarType>(VERTEX_NON_MANIFOLD);
				if (stats)
					++stats->edgesSharedByMore;
			}
			//else --> isolated vertex?

			flags->setValue(i1, flag);
			flags->setValue(i2, flag);
		}
	}

	flags->computeMinAndMax();

	return true;
}

PointCloud* MeshSamplingTools::samplePointsOnMesh(	GenericMesh* mesh,
													unsigned numberOfPoints,
													GenericProgressCallback* progressCb/*=0*/,
													std::vector<unsigned>* triIndices/*=0*/)
{
	if (!mesh)
        return nullptr;

	//total mesh surface
	double Stotal = computeMeshArea(mesh);

	if (Stotal < ZERO_TOLERANCE)
        return nullptr;

	double samplingDensity = numberOfPoints / Stotal;

    //no normal needs to be computed here
	return samplePointsOnMesh(mesh, samplingDensity, numberOfPoints, progressCb, triIndices);
}

PointCloud* MeshSamplingTools::samplePointsOnMesh(	GenericMesh* mesh,
													double samplingDensity,
													GenericProgressCallback* progressCb/*=0*/,
													std::vector<unsigned>* triIndices/*=0*/)
{
	if (!mesh)
        return nullptr;

	//we must compute the total area to deduce the number of points
	double Stotal = computeMeshArea(mesh);

	unsigned theoreticNumberOfPoints = static_cast<unsigned>(ceil(Stotal * samplingDensity));

	return samplePointsOnMesh(mesh, samplingDensity, theoreticNumberOfPoints, progressCb, triIndices);
}

PointCloud* MeshSamplingTools::samplePointsOnMesh(	GenericMesh* mesh,
													double samplingDensity,
													unsigned theoreticNumberOfPoints,
													GenericProgressCallback* progressCb,
													std::vector<unsigned>* triIndices/*=0*/)
{
	if (theoreticNumberOfPoints < 1)
        return nullptr;

	assert(mesh);
	unsigned triCount = (mesh ? mesh->size() : 0);
	if (triCount == 0)
		return nullptr;

	PointCloud* sampledCloud = new PointCloud();
	if (!sampledCloud->reserve(theoreticNumberOfPoints)) //not enough memory
	{
		delete sampledCloud;
		return nullptr;
	}

	if (triIndices)
	{
	    triIndices->clear(); //just in case
		try
		{
			triIndices->reserve(theoreticNumberOfPoints);
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory? DGM TODO: we should warn the caller
			delete sampledCloud;
			return nullptr;
		}
	}

	NormalizedProgress normProgress(progressCb, triCount);
    if (progressCb)
    {
		if (progressCb->textCanBeEdited())
		{
			progressCb->setMethodTitle("Mesh sampling");
			char buffer[256];
			sprintf(buffer, "Triangles: %u\nPoints: %u", triCount, theoreticNumberOfPoints);
			progressCb->setInfo(buffer);
		}
        progressCb->update(0);
		progressCb->start();
	}

	unsigned addedPoints = 0;
	std::random_device rd;   // non-deterministic generator
	std::mt19937 gen(rd());  // to seed mersenne twister.
	std::uniform_real_distribution<double> dist(0, 1);

	//for each triangle
	mesh->placeIteratorAtBeginning();
	for (unsigned n = 0; n < triCount; ++n)
	{
		const GenericTriangle* tri = mesh->_getNextTriangle();

		//vertices (OAB)
		const CCVector3 *O = tri->_getA();
		const CCVector3 *A = tri->_getB();
		const CCVector3 *B = tri->_getC();

		//edges (OA and OB)
		CCVector3 u = *A - *O;
		CCVector3 v = *B - *O;

		//we compute the (twice) the triangle area
		CCVector3 N = u.cross(v);
		double S = N.normd() / 2;

		//we deduce the number of points to generate on this face
		double fPointsToAdd = S*samplingDensity;
		unsigned pointsToAdd = static_cast<unsigned>(fPointsToAdd);

		//take care of the remaining fractional part
		double fracPart = fPointsToAdd - static_cast<double>(pointsToAdd);
		if (fracPart > 0)
		{
			//we add a point with the same probability as its (relative) area
			if (dist(gen) <= fracPart)
				pointsToAdd += 1;
		}

		if (pointsToAdd)
		{
			if (addedPoints + pointsToAdd >= theoreticNumberOfPoints)
			{
				theoreticNumberOfPoints += pointsToAdd;
				//reserve memory for the cloud
				if (!sampledCloud->reserve(theoreticNumberOfPoints))
				{
					delete sampledCloud;
					sampledCloud = nullptr;
					if (triIndices)
					{
						triIndices->resize(0);
					}
					break;
				}
				//reserve memory for the triangle indexes
				if (triIndices && triIndices->capacity() < theoreticNumberOfPoints) //not enough memory
				{
					try
					{
						triIndices->reserve(theoreticNumberOfPoints);
					}
					catch (const std::bad_alloc&)
					{
						delete sampledCloud;
						sampledCloud = nullptr;
						if (triIndices)
						{
							triIndices->resize(0);
						}
						break;
					}
				}
			}

			for (unsigned i = 0; i < pointsToAdd; ++i)
			{
				//we generate random points as in:
				//'Greg Turk. Generating random points in triangles. In A. S. Glassner, editor, Graphics Gems, pages 24-28. Academic Press, 1990.'
				double x = dist(gen);
				double y = dist(gen);

				//we test if the generated point lies on the right side of (AB)
				if (x + y > 1.0)
				{
					x = 1.0 - x;
					y = 1.0 - y;
                }

				CCVector3 P = (*O) + static_cast<PointCoordinateType>(x) * u + static_cast<PointCoordinateType>(y) * v;

				sampledCloud->addPoint(P);
				if (triIndices)
					triIndices->push_back(n);
				++addedPoints;
			}
		}

		if (progressCb && !normProgress.oneStep())
			break;
	}

	if (sampledCloud) //can be in case of memory overflow!
	{
		if (addedPoints)
		{
			sampledCloud->resize(addedPoints); //should always be ok as addedPoints < theoreticNumberOfPoints
			if (triIndices)
				triIndices->resize(addedPoints);
		}
		else
		{
			if (triIndices)
				triIndices->resize(0);
		}
	}

	return sampledCloud;
}
