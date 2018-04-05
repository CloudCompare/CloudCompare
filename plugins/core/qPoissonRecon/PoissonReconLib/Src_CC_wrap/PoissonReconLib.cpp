//##########################################################################
//#                                                                        #
//#               CLOUDCOMPARE WRAPPER: PoissonReconLib                    #
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
#include <assert.h>

//PoissonRecon
#include "../Src/MemoryUsage.h"
#include "../Src/MyTime.h"
#include "../Src/Ply.h"
#include "../Src/Array.h"
#include "../Src/Octree.h"
#include "../Src/SparseMatrix.h"

#define DumpOutput(...) ((void)0)
#include "../Src/MultiGridOctreeData.h" //only after DumpOutput has been defined!

#define BSPLINE_DEGREE 2

PoissonReconLib::Parameters::Parameters()
	: depth(8) //8
	, cgDepth(0) //0
	, kernelDepth(0) //?
	, adaptiveExp(1) //AdaptiveExponent (1)
	, iters(8) //8
	, fullDepth(5) //5
	, maxSolveDepth(0) //?
	, boundary(DIRICHLET)
	, threads(1) //ideally omp_get_num_procs()
	, samplesPerNode(1.5f) //1.5f
	, scale(1.1f) //1.1f
	, cgAccuracy(1.0e-3f) //1.0e-3f
	, pointWeight(4.0f) //4.0f
	, showResidual(false)
	, confidence(false)
	, nonManifold(false)
	, density(false)
	, colorInterp(16.0f)
{
#ifdef WITH_OPENMP
	threads = omp_get_num_procs();
#endif
}

template< class Real >
XForm4x4< Real > GetPointXForm(OrientedPointStream< Real >& stream, Real scaleFactor)
{
	Point3D< Real > min, max;
	stream.boundingBox(min, max);

	Real scale = std::max< Real >(max[0] - min[0], std::max< Real >(max[1] - min[1], max[2] - min[2]));
	scale *= scaleFactor;

	XForm4x4< Real > tXForm = XForm4x4< Real >::Identity();
	XForm4x4< Real > sXForm = XForm4x4< Real >::Identity();
	Point3D< Real > center = (max + min) / 2;
	for (int i = 0; i < 3; i++)
	{
		sXForm(i, i) = static_cast<Real>(1.0 / scale);
		tXForm(3, i) = -center[i] + scale / 2;
	}
	return sXForm * tXForm;
}

template< class Real, int Degree, BoundaryType BType, class Vertex >
bool Execute(	PoissonReconLib::Parameters params,
				OrientedPointStream< Real >* pointStream,
				bool withColors,
				CoredVectorMeshData< Vertex >& mesh,
				XForm4x4< Real >& iXForm)
{
	typedef typename Octree< Real >::template DensityEstimator< WEIGHT_DEGREE > DensityEstimator; 
	typedef typename Octree< Real >::template InterpolationInfo< false > InterpolationInfo;
	typedef OrientedPointStreamWithData< Real, Point3D< Real > > PointStreamWithData;
	typedef TransformedOrientedPointStream< Real > XPointStream;
	typedef TransformedOrientedPointStreamWithData< Real, Point3D< Real > > XPointStreamWithData;
	Reset< Real >();

	//DGM: do this begore initializing the octree!!
	OctNode< TreeNodeData >::SetAllocator(MEMORY_ALLOCATOR_BLOCK_SIZE);

	Octree< Real > tree;
	tree.threads = params.threads;

	if (params.maxSolveDepth == 0)
		params.maxSolveDepth = params.depth;

	if (params.maxSolveDepth < 2)
		return false;
	int kernelDepth = params.kernelDepth != 0 ?  params.kernelDepth : params.maxSolveDepth-2;
	if( kernelDepth > params.depth )
		return false;
	params.fullDepth = std::min(params.fullDepth, params.depth);

	try
	{
		XForm4x4< Real > xForm = XForm4x4< Real >::Identity();
		{
			xForm = GetPointXForm(*pointStream, static_cast<Real>(params.scale));
		}
		iXForm = xForm.inverse();

		std::vector< typename Octree< Real >::PointSample > samples;
		std::vector< ProjectiveData< Point3D< Real >, Real > > sampleData;

		if (withColors)
		{
			XPointStreamWithData _pointStream(xForm, *((PointStreamWithData*)pointStream));
			int pointCount = tree.template init< Point3D< Real > >(
				_pointStream,
				params.depth,
				params.confidence,
				samples,
				&sampleData);
		}
		else
		{
			XPointStream _pointStream(xForm, *pointStream);
			int pointCount = tree.template init< Point3D< Real > >(
				_pointStream,
				params.depth,
				params.confidence,
				samples,
				0);
		}

		DenseNodeData< Real, Degree > solution;
		DensityEstimator* density = NULL;
		{
			int solveDepth = params.depth;

			tree.resetNodeIndices();

			// Get the kernel density estimator [If discarding, compute anew. Otherwise, compute once.]
			density = tree.template setDensityEstimator< WEIGHT_DEGREE >(samples, kernelDepth, params.samplesPerNode);

			// Transform the Hermite samples into a vector field
			Real pointWeightSum = 0;
			SparseNodeData< Point3D< Real >, NORMAL_DEGREE > normalInfo = tree.template setNormalField< NORMAL_DEGREE >(samples, *density, pointWeightSum, BType == BOUNDARY_NEUMANN);

			if (!params.density)
			{
				delete density;
				density = 0;
			}

			// Trim the tree and prepare for multigrid
			{
				std::vector< int > indexMap;

				tree.template inalizeForBroodedMultigrid< NORMAL_DEGREE, Degree, BType >(params.fullDepth, typename Octree< Real >::template HasNormalDataFunctor< NORMAL_DEGREE >(normalInfo), &indexMap);

				normalInfo.remapIndices(indexMap);
				if (params.density)
				{
					density->remapIndices(indexMap);
				}
			}

			// Add the FEM constraints
			DenseNodeData< Real, Degree > constraints;
			{
				constraints = tree.template initDenseNodeData< Degree >();
				tree.template addFEMConstraints< Degree, BType, NORMAL_DEGREE, BType >(FEMVFConstraintFunctor< NORMAL_DEGREE, BType, Degree, BType >(1., 0.), normalInfo, constraints, solveDepth);
			}

			// Free up the normal info [If we don't need it for subseequent iterations.]
			normalInfo.clear();

			// Add the interpolation constraints
			InterpolationInfo* iInfo = NULL;
			if (params.pointWeight > 0)
			{
				Real targetValue = static_cast<Real>(0.5);
				iInfo = new InterpolationInfo(tree, samples, targetValue, params.adaptiveExp, static_cast<Real>(params.pointWeight) * pointWeightSum, (Real)0);
				tree.template addInterpolationConstraints< Degree, BType >(*iInfo, constraints, solveDepth);
			}

			//DumpOutput("Leaf Nodes / Active Nodes / Ghost Nodes: %d / %d / %d\n", (int)tree.leaves(), (int)tree.nodes(), (int)tree.ghostNodes());
			//DumpOutput("Memory Usage: %.3f MB\n", float(MemoryInfo::Usage()) / (1 << 20));

			// Solve the linear system
			double lowResIterMultiplier = 1.0;
			{
				typename Octree< Real >::SolverInfo solverInfo;
				solverInfo.cgDepth = params.cgDepth;
				solverInfo.iters = params.iters;
				solverInfo.cgAccuracy = params.cgAccuracy;
				solverInfo.verbose = false;
				solverInfo.showResidual = params.showResidual;
				solverInfo.lowResIterMultiplier = std::max< double >(1.0, lowResIterMultiplier);
				solution = tree.template solveSystem< Degree, BType >(FEMSystemFunctor< Degree, BType >(0, 1., 0), iInfo, constraints, solveDepth, solverInfo);
				if (iInfo)
				{
					delete iInfo;
					iInfo = NULL;
				}
			}
		}

		Real isoValue = 0;
		{
			double valueSum = 0, weightSum = 0;
			typename Octree< Real >::template MultiThreadedEvaluator< Degree, BType > evaluator(&tree, solution, params.threads);

#pragma omp parallel for num_threads( params.threads ) reduction( + : valueSum , weightSum )
			for (int j = 0; j < samples.size(); j++)
			{
				const ProjectiveData< OrientedPoint3D< Real >, Real >& sample = samples[j].sample;
				if (sample.weight > 0)
				{
					weightSum += sample.weight;
					valueSum += evaluator.value(sample.data.p / sample.weight, omp_get_thread_num(), samples[j].node) * sample.weight;
				}
			}
			isoValue = static_cast<Real>(valueSum / weightSum);
			//DumpOutput("Iso-Value: %e\n", isoValue);
		}

		SparseNodeData< ProjectiveData< Point3D< Real >, Real >, DATA_DEGREE >* colorData = NULL;
		if (withColors)
		{
			colorData = new SparseNodeData< ProjectiveData< Point3D< Real >, Real >, DATA_DEGREE >();
			*colorData = tree.template setDataField< DATA_DEGREE, false >(samples, sampleData, (DensityEstimator*)NULL);
			for (const OctNode< TreeNodeData >* n = tree.tree().nextNode(); n; n = tree.tree().nextNode(n))
			{
				ProjectiveData< Point3D< Real >, Real >* clr = (*colorData)(n);
				if (clr) (*clr) *= static_cast<Real>(pow(params.colorInterp, tree.depth(n)));
			}
		}

		bool linearFit = false;
		bool polygonMesh = false;
		tree.template getMCIsoSurface< Degree, BType, WEIGHT_DEGREE, DATA_DEGREE >(
			density,
			colorData,
			solution,
			isoValue,
			mesh,
			!linearFit,
			!params.nonManifold,
			polygonMesh);

		if (density)
		{
			delete density;
			density = NULL;
		}

		//DumpOutput("Vertices / Polygons: %d / %d\n", mesh.outOfCorePointCount() + mesh.inCorePoints.size(), mesh.polygonCount());
		if (colorData)
		{
			delete colorData;
			colorData = NULL;
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
	catch (std::exception e)
	{
		//not enough memory
		return false;
	}
	catch (...)
	{
		//not enough memory
		return false;
	}

	return true;
}

bool PoissonReconLib::Reconstruct(	Parameters params,
									OrientedPointStreamWithData< float , Point3D< float > >* pointStream,
									CoredVectorMeshData< PlyColorAndValueVertex< float > >& mesh,
									XForm4x4< float >& iXForm)
{
	switch (params.boundary)
	{
	case Parameters::FREE:
		return Execute<	float, BSPLINE_DEGREE, BOUNDARY_FREE, PlyColorAndValueVertex< float > >(params, pointStream, true, mesh, iXForm);
	case Parameters::DIRICHLET:
		return Execute<	float, BSPLINE_DEGREE, BOUNDARY_DIRICHLET, PlyColorAndValueVertex< float > >(params, pointStream, true, mesh, iXForm);
	case Parameters::NEUMANN:
		return Execute<	float, BSPLINE_DEGREE, BOUNDARY_NEUMANN, PlyColorAndValueVertex< float > >(params, pointStream, true, mesh, iXForm);
	default:
		assert(false);
		break;
	}

	return false;
}

bool PoissonReconLib::Reconstruct(	Parameters params,
									OrientedPointStream< float >* pointStream,
									CoredVectorMeshData< PlyValueVertex< float > >& mesh,
									XForm4x4< float >& iXForm)
{
	switch (params.boundary)
	{
	case Parameters::FREE:
		return Execute<	float, BSPLINE_DEGREE, BOUNDARY_FREE, PlyValueVertex< float > >(params, pointStream, false, mesh, iXForm);
	case Parameters::DIRICHLET:
		return Execute<	float, BSPLINE_DEGREE, BOUNDARY_DIRICHLET, PlyValueVertex< float > >(params, pointStream, false, mesh, iXForm);
	case Parameters::NEUMANN:
		return Execute<	float, BSPLINE_DEGREE, BOUNDARY_NEUMANN, PlyValueVertex< float > >(params, pointStream, false, mesh, iXForm);
	default:
		assert(false);
		break;
	}

	return false;
}

bool PoissonReconLib::Reconstruct(	Parameters params,
									OrientedPointStreamWithData< double , Point3D< double > >* pointStream,
									CoredVectorMeshData< PlyColorAndValueVertex< double > >& mesh,
									XForm4x4< double >& iXForm)
{
	switch (params.boundary)
	{
	case Parameters::FREE:
		return Execute<	double, BSPLINE_DEGREE, BOUNDARY_FREE, PlyColorAndValueVertex< double > >(params, pointStream, true, mesh, iXForm);
	case Parameters::DIRICHLET:
		return Execute<	double, BSPLINE_DEGREE, BOUNDARY_DIRICHLET, PlyColorAndValueVertex< double > >(params, pointStream, true, mesh, iXForm);
	case Parameters::NEUMANN:
		return Execute<	double, BSPLINE_DEGREE, BOUNDARY_NEUMANN, PlyColorAndValueVertex< double > >(params, pointStream, true, mesh, iXForm);
	default:
		assert(false);
		break;
	}

	return false;
}

bool PoissonReconLib::Reconstruct(	Parameters params,
									OrientedPointStream< double >* pointStream,
									CoredVectorMeshData< PlyValueVertex< double > >& mesh,
									XForm4x4< double >& iXForm)
{
	switch (params.boundary)
	{
	case Parameters::FREE:
		return Execute<	double, BSPLINE_DEGREE, BOUNDARY_FREE, PlyValueVertex< double > >(params, pointStream, false, mesh, iXForm);
	case Parameters::DIRICHLET:
		return Execute<	double, BSPLINE_DEGREE, BOUNDARY_DIRICHLET, PlyValueVertex< double > >(params, pointStream, false, mesh, iXForm);
	case Parameters::NEUMANN:
		return Execute<	double, BSPLINE_DEGREE, BOUNDARY_NEUMANN, PlyValueVertex< double > >(params, pointStream, false, mesh, iXForm);
	default:
		assert(false);
		break;
	}

	return false;
}
