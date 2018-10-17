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

#ifndef MESH_SAMPLING_TOOLS_HEADER
#define MESH_SAMPLING_TOOLS_HEADER

//Local
#include "CCToolbox.h"

//system
#include <map>
#include <vector>

namespace CCLib
{

class GenericProgressCallback;
class GenericMesh;
class GenericIndexedMesh;
class PointCloud;
class ScalarField;

//! Mesh sampling algorithms
class CC_CORE_LIB_API MeshSamplingTools : public CCToolbox
{
public:

	//! Computes the mesh area
	/** \param mesh triangular mesh
		\return mesh area
	**/
	static double computeMeshArea(GenericMesh* mesh);

	//! Computes the mesh volume
	/** \warning Make sure the input mesh is closed!
		See MeshSamplingTools::computeMeshEdgesConnectivity.
		\param mesh triangular mesh (closed!)
		\return mesh volume
	**/
	static double computeMeshVolume(GenericMesh* mesh);

	//! Statistics on the edges connectivty of a mesh
	struct EdgeConnectivityStats
	{
		EdgeConnectivityStats()
			: edgesCount(0)
			, edgesNotShared(0)
			, edgesSharedByTwo(0)
			, edgesSharedByMore(0)
		{}

		//! Total number of edges
		unsigned edgesCount;
		//! Edges not shared (i.e. used by only one triangle)
		unsigned edgesNotShared;
		//! Edges shared by exactly two triangles
		unsigned edgesSharedByTwo;
		//! Edges shared by more than two triangles
		unsigned edgesSharedByMore;
	};

	//! Computes some statistics on the edges connectivty of a mesh
	/** This methods counts the number of edges shared by 1, 2 or more faces.
		One ore more edges used only by 1 face each indicates the presence of
		at least one hole. Edges used by more than two faces are non-manifold.
		\param[in] mesh triangular mesh
		\param[out] stats output statistics
		\return false if an error occurred (invalid input or not enough memory)
	**/
	static bool computeMeshEdgesConnectivity(GenericIndexedMesh* mesh, EdgeConnectivityStats& stats);

	//! Flags used by the MeshSamplingTools::flagMeshVerticesByType method.
	enum VertexFlags
	{
		VERTEX_NORMAL		= 0,	/**< Normal vertex **/
		VERTEX_BORDER		= 1,	/**< Vertex on a border/hole **/
		VERTEX_NON_MANIFOLD	= 2		/**< Vertex on a non-manifold edge **/
	};

	//! Flags the vertices of a mesh depending on their type
	/** See MeshSamplingTools::VertexFlags.
		\param[in] mesh triangular mesh
		\param[in] flags already allocated scalar field to store the per-vertex flags
		\param[out] stats output statistics (optional)
		\return false if an error occurred (invalid input or not enough memory)
	**/
	static bool flagMeshVerticesByType(GenericIndexedMesh* mesh, ScalarField* flags, EdgeConnectivityStats* stats = nullptr);

	//! Samples points on a mesh
	/** The points are sampled on each triangle randomly, by generating
		two numbers between 0 and 1 (a and b). If a+b > 1, then a = 1-a and
		b = 1-b. Let ABC be the triangle, then the new point P will be as
		AP = a.AB+b.AC (AP,AB and AC are vectors here). The number of
		points sampled on each triangle depends on the triangle's area.
		Let s be this area, and µ the sampling density, then N = s*µ is
		the theoretic (floating) number of points to sample. The floating
		part of N (let's call it Nf, and let Ni be the integer part) is
		handled by generating another random number between 0 and 1.
		If this number is less than Nf, then Ni = Ni+1. The number of points
		sampled on the triangle will simply be Ni.
		\param mesh the mesh to be sampled
		\param samplingDensity the sampling surface density
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param[out] triIndices triangle index for each samples point (output only - optional)
		\return the sampled points
	**/
	static PointCloud* samplePointsOnMesh(	GenericMesh* mesh,
											double samplingDensity,
											GenericProgressCallback* progressCb = nullptr,
											std::vector<unsigned>* triIndices = nullptr);

	//! Samples points on a mesh
	/** See the other version of this method. Instead of specifying a
		density, it is possible here to specify the total number of
		points to sample (approximative).
		\param mesh the mesh to be sampled
		\param numberOfPoints the desired number of points on the whole mesh
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param[out] triIndices triangle index for each samples point (output only - optional)
		\return the sampled points
	**/
	static PointCloud* samplePointsOnMesh(	GenericMesh* mesh,
											unsigned numberOfPoints,
											GenericProgressCallback* progressCb = nullptr,
											std::vector<unsigned>* triIndices = nullptr);

protected:

	//! Samples points on a mesh - internal method
	/** See public methods descriptions
		\param mesh the mesh to be sampled
		\param samplingDensity the sampling surfacical density
		\param theoreticNumberOfPoints the approximated number of points that will be sampled
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param[out] triIndices triangle index for each samples point (output only - optional)
		\return the sampled points
	**/
	static PointCloud* samplePointsOnMesh(	GenericMesh* mesh,
											double samplingDensity,
											unsigned theoreticNumberOfPoints,
											GenericProgressCallback* progressCb = nullptr,
											std::vector<unsigned>* triIndices = nullptr);

	//! Map used to count the number of triangles using each edge
	/** Edges are represented by two 32 bits indexes merged as a 64 integer
	**/
	using EdgeUsageMap = std::map<unsigned long long, unsigned>;

	//! Computes the unique key corresponding to an edge
	static unsigned long long ComputeEdgeKey(unsigned i1, unsigned i2);
	//! Computes the edge vertex indexes from its unique key
	static void DecodeEdgeKey(unsigned long long key, unsigned& i1, unsigned& i2);

	//! Creates a map to count the number of triangles using each edge
	static bool buildMeshEdgeUsageMap(GenericIndexedMesh* mesh, EdgeUsageMap& edgeMap);
};

}

#endif //MESH_SAMPLING_TOOLS_HEADER
