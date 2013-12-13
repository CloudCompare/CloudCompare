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

//system
#include <assert.h>
#ifdef WITH_OPENMP
#include <omp.h>
#endif

//PoissonRecon
#ifdef _WIN32
#include <Windows.h>
#endif
#include "../Src/Time.h"
#include "../Src/MarchingCubes.h"
#include "../Src/Octree.h"
#include "../Src/SparseMatrix.h"
#include "../Src/CmdLineParser.h"
#include "../Src/PPolynomial.h"
#include "../Src/Ply.h"
#include "../Src/MemoryUsage.h"

//#include <stdio.h>
//#define DumpOutput printf
#define DumpOutput(...) ((void)0)
#include "../Src/MultiGridOctreeData.h" //only after DumpOutput has been defined!

//specialization of Octree::setTree to fit library 'input'
template< int Degree , bool OutputDensity >
class LibOctree : public Octree<Degree, OutputDensity>
{
public:
	template<typename T>
	int setTree(	unsigned count,
					const T* P,
					const T* N,
					int maxDepth,
					int minDepth,
					int splatDepth,
					Real samplesPerNode,
					Real scaleFactor,
					bool useConfidence,
					Real constraintWeight,
					int adaptiveExponent)
	{
		if ( splatDepth < 0 )
			splatDepth = 0;
		
		this->samplesPerNode = samplesPerNode;
		this->splatDepth = splatDepth;

		if( _boundaryType == 0 )
		{
			maxDepth++,
			minDepth = std::max< int >( 1 , minDepth )+1;
			if (splatDepth > 0 )
				splatDepth++;
		}
		else
		{
			minDepth = std::max< int >( 0 , minDepth );
		}
		_minDepth = std::min< int >( minDepth , maxDepth );
		_constrainValues = (constraintWeight>0);

		double pointWeightSum = 0;
		Point3D< Real > min , max;

		typename TreeOctNode::NeighborKey3 neighborKey;
		neighborKey.set( maxDepth );

		tree.setFullDepth( _minDepth );
		// Read through once to get the center and scale
		{
			const T* _P = P;
			for (unsigned k=0; k<count; ++k, _P+=3)
			{
				Point3D< Real > p(	static_cast<Real>(_P[0]),
									static_cast<Real>(_P[1]),
									static_cast<Real>(_P[2]) );

				for( int i=0 ; i<DIMENSION ; i++ )
				{
					if (k)
					{
						if( p[i] < min[i] )
							min[i] = p[i];
						else if( p[i] > max[i] )
							max[i] = p[i];
					}
					else
					{
						min[i] = max[i] = p[i];
					}
				}
			}

			_center = ( max+min ) /2;
			_scale = std::max< Real >( max[0]-min[0] , std::max< Real >( max[1]-min[1] , max[2]-min[2] ) ) * 2;
			if( _boundaryType == 0 )
				_scale *= 2;
		}

		//update scale and center with scale factor
		{
			_scale *= scaleFactor;
			for( int i=0 ; i<DIMENSION ; i++ )
				_center[i] -= _scale/2;
		}

		if( splatDepth > 0 )
		{
			const T* _P = P;
			const T* _N = N;
			for (unsigned k=0; k<count; ++k, _P+=3, _N+=3)
			{
				Point3D< Real > p(	static_cast<Real>(_P[0]),
									static_cast<Real>(_P[1]),
									static_cast<Real>(_P[2]) );

				p = ( p - _center ) / _scale;
				
				if( !_inBounds(p) )
					continue;
				
				Point3D< Real > myCenter = Point3D< Real >( Real(0.5) , Real(0.5) , Real(0.5) );
				Real myWidth = Real(1.0);
				Real weight = Real(1.0);

				if( useConfidence )
				{
					Point3D< Real > n(	static_cast<Real>(_N[0]),
										static_cast<Real>(_N[1]),
										static_cast<Real>(_N[2]) );
					weight = Real( Length(n) );
				}

				TreeOctNode* temp = &tree;
				int d = 0;
				while( d < splatDepth )
				{
					UpdateWeightContribution( temp , p , neighborKey , weight );
					if( !temp->children )
						temp->initChildren();
					int cIndex = TreeOctNode::CornerIndex( myCenter , p );
					temp = temp->children + cIndex;
					myWidth /= 2;
					if( cIndex&1 ) myCenter[0] += myWidth/2;
					else           myCenter[0] -= myWidth/2;
					if( cIndex&2 ) myCenter[1] += myWidth/2;
					else           myCenter[1] -= myWidth/2;
					if( cIndex&4 ) myCenter[2] += myWidth/2;
					else           myCenter[2] -= myWidth/2;
					d++;
				}
				UpdateWeightContribution( temp , p , neighborKey , weight );
			}
		}

		//normals
		normals = new std::vector< Point3D<Real> >();
		int cnt = 0;
		{
			const T* _P = P;
			const T* _N = N;
			for (unsigned k=0; k<count; ++k, _P+=3, _N+=3)
			{
				Point3D< Real > p(	static_cast<Real>(_P[0]),
									static_cast<Real>(_P[1]),
									static_cast<Real>(_P[2]) );
				p = ( p - _center ) / _scale;
				if( !_inBounds(p) )
					continue;

				Point3D< Real > n(	static_cast<Real>(_N[0]),
									static_cast<Real>(_N[1]),
									static_cast<Real>(_N[2]) );
				n *= Real(-1.0);
				//normalize n
				Real l = Real( Length( n ) );
				if( l!=l || l<=EPSILON )
					continue;
				if( !useConfidence )
					n /= l;

				Point3D< Real > myCenter = Point3D< Real >( Real(0.5) , Real(0.5) , Real(0.5) );
				Real myWidth = Real(1.0);

				Real pointWeight = Real(1.0f);
				if ( samplesPerNode > 0 && splatDepth )
				{
					pointWeight = SplatOrientedPoint( p , n , neighborKey , splatDepth , samplesPerNode , _minDepth , maxDepth );
				}
				else
				{
					TreeOctNode* temp = &tree;
					int d = 0;
					if( splatDepth )
					{
						while( d < splatDepth )
						{
							int cIndex = TreeOctNode::CornerIndex(myCenter,p);
							temp = &temp->children[cIndex];
							myWidth /= 2;
							if(cIndex&1) myCenter[0] += myWidth/2;
							else		 myCenter[0] -= myWidth/2;
							if(cIndex&2) myCenter[1] += myWidth/2;
							else		 myCenter[1] -= myWidth/2;
							if(cIndex&4) myCenter[2] += myWidth/2;
							else		 myCenter[2] -= myWidth/2;
							d++;
						}
						pointWeight = GetSampleWeight( temp , p , neighborKey );
					}
					{
						for (int i=0 ; i<DIMENSION ; i++ )
							n[i] *= pointWeight;
					}
					while( d < maxDepth )
					{
						if( !temp->children )
							temp->initChildren();
						int cIndex = TreeOctNode::CornerIndex(myCenter,p);
						temp = &temp->children[cIndex];
						myWidth /= 2;
						if(cIndex&1) myCenter[0] += myWidth/2;
						else		 myCenter[0] -= myWidth/2;
						if(cIndex&2) myCenter[1] += myWidth/2;
						else		 myCenter[1] -= myWidth/2;
						if(cIndex&4) myCenter[2] += myWidth/2;
						else		 myCenter[2] -= myWidth/2;
						d++;
					}
					SplatOrientedPoint( temp , p , n , neighborKey );
				}
				pointWeightSum += pointWeight;
				if ( _constrainValues )
				{
					int d = 0;
					TreeOctNode* temp = &tree;
					Point3D< Real > myCenter = Point3D< Real >( Real(0.5) , Real(0.5) , Real(0.5) );
					Real myWidth = Real(1.0);
					while( true )
					{
						int idx = temp->nodeData.pointIndex;
						if( idx == -1 )
						{
							idx = static_cast<int>( _points.size() );
							_points.push_back( PointData( p , Real(1.0) ) );
							temp->nodeData.pointIndex = idx;
						}
						else
						{
							_points[idx].weight += Real(1.0);
							_points[idx].position += p;
						}

						int cIndex = TreeOctNode::CornerIndex( myCenter , p );
						if ( !temp->children )
							break;
						
						temp = &temp->children[cIndex];
						myWidth /= 2;
						if( cIndex&1 ) myCenter[0] += myWidth/2;
						else		   myCenter[0] -= myWidth/2;
						if( cIndex&2 ) myCenter[1] += myWidth/2;
						else		   myCenter[1] -= myWidth/2;
						if( cIndex&4 ) myCenter[2] += myWidth/2;
						else		   myCenter[2] -= myWidth/2;
						d++;
					}
				}

				++cnt;
			}
		}

		if( _boundaryType == 0 )
			pointWeightSum *= Real(4.0);
		
		constraintWeight *= static_cast<Real>(pointWeightSum);
		constraintWeight /= static_cast<Real>(cnt);

		MemoryUsage( );

		if( _constrainValues )
		{
			for( TreeOctNode* node=tree.nextNode() ; node ; node=tree.nextNode(node) )
			{
				if( node->nodeData.pointIndex != -1 )
				{
					int idx = node->nodeData.pointIndex;
					_points[idx].position /= _points[idx].weight;
					int e = ( _boundaryType == 0 ? node->depth()-1 : node->depth() ) * adaptiveExponent - ( _boundaryType == 0 ? maxDepth-1 : maxDepth ) * (adaptiveExponent-1);
					
					if ( e < 0 )
						_points[idx].weight /= Real( 1<<(-e) );
					else
						_points[idx].weight *= Real( 1<<  e  );

					_points[idx].weight *= Real( constraintWeight );
				}
			}
		}

#if FORCE_NEUMANN_FIELD
		if( _boundaryType == 1 )
		{
			for( TreeOctNode* node=tree.nextNode() ; node ; node=tree.nextNode( node ) )
			{
				int d , off[3];
				node->depthAndOffset( d , off );
				int res = 1<<d;
				if( node->nodeData.normalIndex < 0 )
					continue;
				
				Point3D< Real >& normal = (*normals)[node->nodeData.normalIndex];
				for( int d=0 ; d<3 ; d++ )
					if ( off[d]==0 || off[d]==res-1 )
						normal[d] = 0;
			}
		}
#endif // FORCE_NEUMANN_FIELD

		MemoryUsage();
		return cnt;
	}
};


static const bool OutputDensity = false;

bool PoissonReconLib::Reconstruct(	unsigned count,
									const float* P,
									const float* N,
									const Parameters& inParams,
									CoredVectorMeshData< Vertex >& outMesh)
{
	Parameters params = inParams;
	if (params.depth < 2)
		return false;

	//params.depth = std::max(params.minDepth,params.depth);

	//TODO: let the user set these parameters?
	assert(params.depth > 2);
	int maxSolveDepth = params.depth;
	int kernelDepth = params.depth-2;
	
	params.solverDivide = std::max(params.minDepth,params.solverDivide); //solverDivide must be at least as large as minDepth
	params.isoDivide = std::max(params.minDepth,params.isoDivide); //isoDivide must be at least as large as minDepth
	kernelDepth = std::min(kernelDepth,params.depth); //kernelDepth can't be greater than depth

	try
	{
		//Tree construction
		OctNode< TreeNodeData< OutputDensity > , Real >::SetAllocator( MEMORY_ALLOCATOR_BLOCK_SIZE );
		LibOctree< 2 , OutputDensity > tree;
	#ifdef WITH_OPENMP
		tree.threads = omp_get_num_procs();
	#else
		tree.threads = 1;
	#endif
		tree.setBSplineData( params.depth , 1 );
		int pointCount = tree.setTree<float>( count, P, N, params.depth, params.minDepth, kernelDepth, params.samplesPerNode, params.scale, params.useConfidence, params.pointWeight, params.adaptiveExponent );
		tree.ClipTree();
		tree.finalize( params.isoDivide );

		//Set constraints
		tree.SetLaplacianConstraints();

		//Solve linear system
		tree.LaplacianMatrixIteration( params.solverDivide, /*ShowResidual*/0, params.minIters, params.solverAccuracy, maxSolveDepth, params.fixedIters );

		//get triangles
		Real isoValue = tree.GetIsoValue();
		tree.GetMCIsoTriangles( isoValue , params.isoDivide, &outMesh , 0 , 1 , !params.nonManifold, /*PolygonMesh*/0 );
	}
	catch(...)
	{
		//an error occurred!
		return false;
	}

	return true;
}
