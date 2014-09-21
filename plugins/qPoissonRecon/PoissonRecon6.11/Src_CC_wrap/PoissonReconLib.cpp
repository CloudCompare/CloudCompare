//##########################################################################
//#                                                                        #
//#               CLOUDCOMPARE WRAPPER: PoissonReconLib                    #
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
//#               COPYRIGHT: Daniel Girardeau-Montaut                      #
//#                                                                        #
//##########################################################################

#include "PoissonReconLib.h"

#ifdef _WIN32
#include <Windows.h>
#include <Psapi.h>
#endif // _WIN32

#ifdef WITH_OPENMP
#include <omp.h>
#endif

//PoissonRecon
#include "../Src/Ply.h"
#include "../Src/Array.h"
#include "../Src/Octree.h"
#include "../Src/SparseMatrix.h"

#define DumpOutput(...) ((void)0)
#include "../Src/MultiGridOctreeData.h" //only after DumpOutput has been defined!

PoissonReconLib::Parameters::Parameters()
	: depth(8) //8
	, cgDepth(0) //0
	, kernelDepth(0) //?
	, adaptiveExp(1) //AdaptiveExponent (1)
	, cgMaxIters(8) //8
	, fullDepth(5) //5
	, minDepth(0) //0
	, maxSolveDepth(0) //?
	, boundary(1) //1
	, threads(1) //ideally omp_get_num_procs()
	, samplesPerNode(1.0f) //1.0f
	, scale(1.1f) //1.1f
	, cgAccuracy(1.0e-3f) //1.0e-3f
	, pointWeight(4.0f) //4.0f
	, complete(false)
	, showResidual(false)
	, confidence(false)
	, normalWeights(false)
	, nonManifold(false)
	, density(false)
{
#ifdef WITH_OPENMP
	threads = omp_get_num_procs();
#endif
}

template< class PointCoordinateType, class Real , int Degree , class Vertex >
bool Execute(PoissonReconLib::Parameters params, PointStream< PointCoordinateType >* pointStream, CoredVectorMeshData< PlyValueVertex< PointCoordinateType > >& mesh)
{
	XForm4x4< Real > xForm = XForm4x4< Real >::Identity();
	XForm4x4< Real > iXForm = xForm.inverse();

	//DGM: reset static parameters!!!
	TreeNodeData::NodeCount = 0;

	Octree< Real , Degree > tree;
	tree.threads = params.threads;

	if (params.maxSolveDepth == 0)
		params.maxSolveDepth = params.depth;

	OctNode< TreeNodeData >::SetAllocator( MEMORY_ALLOCATOR_BLOCK_SIZE );

	if (params.maxSolveDepth < 2)
		return false;
	int kernelDepth = params.kernelDepth != 0 ?  params.kernelDepth : params.maxSolveDepth-2;
	if( kernelDepth > params.depth )
		return false;
	params.fullDepth = std::min(params.fullDepth, params.depth);

	tree.maxMemoryUsage = 0;
	typename Octree< Real , Degree >::PointInfo* pointInfo = new typename Octree< Real , Degree >::PointInfo();
	typename Octree< Real , Degree >::NormalInfo* normalInfo = new typename Octree< Real , Degree >::NormalInfo();
	std::vector< Real >* kernelDensityWeights = new std::vector< Real >();
	std::vector< Real >* centerWeights = new std::vector< Real >();

	int pointCount = tree.template SetTree< PointCoordinateType >( pointStream , params.minDepth , params.depth, params.fullDepth , kernelDepth , static_cast<Real>(params.samplesPerNode) , params.scale, params.confidence , params.normalWeights, params.pointWeight, params.adaptiveExp, *pointInfo , *normalInfo , *kernelDensityWeights , *centerWeights , params.boundary, xForm , params.complete );
	if( !params.density )
	{
		delete kernelDensityWeights;
		kernelDensityWeights = NULL;
	}

	//DumpOutput( "Input Points: %d\n" , pointCount );
	//DumpOutput( "Leaves/Nodes: %d/%d\n" , tree.tree.leaves() , tree.tree.nodes() );

	double maxMemoryUsage = tree.maxMemoryUsage;
	tree.maxMemoryUsage = 0;

	Pointer( Real ) constraints = tree.SetLaplacianConstraints( *normalInfo );
	delete normalInfo;
	normalInfo = 0;

	maxMemoryUsage = std::max< double >( maxMemoryUsage , tree.maxMemoryUsage );

	tree.maxMemoryUsage = 0;

	Pointer( Real ) solution = tree.SolveSystem( *pointInfo , constraints , params.showResidual , params.cgMaxIters, params.maxSolveDepth, params.cgDepth, params.cgAccuracy );

	delete pointInfo;
	pointInfo = 0;
	FreePointer( constraints );

	maxMemoryUsage = std::max< double >( maxMemoryUsage , tree.maxMemoryUsage );

	Real isoValue = tree.GetIsoValue( solution , *centerWeights );
	delete centerWeights;
	centerWeights = 0;

	//DumpOutput( "Iso-Value: %e\n" , isoValue );

	//output
	tree.maxMemoryUsage = 0;

	tree.GetMCIsoSurface( kernelDensityWeights ? GetPointer( *kernelDensityWeights ) : NullPointer< Real >() , solution , isoValue , mesh , true , !params.nonManifold , false );

	maxMemoryUsage = std::max< double >( maxMemoryUsage , tree.maxMemoryUsage );

	//DumpOutput( "Vertices / Polygons: %d / %d\n" , mesh.outOfCorePointCount()+mesh.inCorePoints.size() , mesh.polygonCount() );

	FreePointer( solution );

	return true;
}

bool PoissonReconLib::Reconstruct(Parameters params, PointStream< float >* pointStream, CoredVectorMeshData< PlyValueVertex< float > >& mesh)
{
	return Execute<float, float, 2, PlyValueVertex< float > >(params, pointStream, mesh);
}

bool PoissonReconLib::Reconstruct(Parameters params, PointStream< double >* pointStream, CoredVectorMeshData< PlyValueVertex< double > >& mesh)
{
	return Execute<double, double, 2, PlyValueVertex< double > >(params, pointStream, mesh);
}
