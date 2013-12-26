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
#include <string.h>
#include <stdio.h>
#ifdef WITH_OPENMP
#include <omp.h>
#endif

//PoissonRecon
#ifdef _WIN32
#include <Windows.h>
#endif
#include "../Src/poissonTime.h"
#include "../Src/MarchingCubes.h"
#include "../Src/Octree.h"
#include "../Src/SparseMatrix.h"
#include "../Src/CmdLineParser.h"
#include "../Src/PPolynomial.h"
#include "../Src/Ply.h"
#include "../Src/MemoryUsage.h"

#define DumpOutput(...) ((void)0)
#include "../Src/MultiGridOctreeData.h" //only after DumpOutput has been defined!

static const bool OutputDensity = false;

//specialization of Octree::setTree to fit library 'input'
class LibOctree : public Octree<2, OutputDensity>
{
public:

	int setTree(	unsigned count,
					const Point3D< Real > * inPoints,
					const Point3D< Real > * inNormals,
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

		if( this->_boundaryType == 0 )
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
		this->_minDepth = std::min< int >( minDepth , maxDepth );
		this->_constrainValues = (constraintWeight>0);

		double pointWeightSum = 0;
		Point3D< Real > min , max;

		TreeOctNode::NeighborKey3 neighborKey;
		neighborKey.set( maxDepth );

		this->tree.setFullDepth( _minDepth );
		// Read through once to get the center and scale
		{
			for (unsigned k=0; k<count; ++k)
			{
				const Point3D< Real >& p = inPoints[k];

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

			this->_center = ( max+min ) /2;
			this->_scale = std::max< Real >( max[0]-min[0] , std::max< Real >( max[1]-min[1] , max[2]-min[2] ) ) * 2;
			if( this->_boundaryType == 0 )
				this->_scale *= 2;
		}

		//update scale and center with scale factor
		{
			this->_scale *= scaleFactor;
			for( int i=0 ; i<DIMENSION ; i++ )
				this->_center[i] -= _scale/2;
		}

		if( splatDepth > 0 )
		{
			const Point3D< Real >* _p = inPoints;
			const Point3D< Real >* _n = inNormals;
			for (unsigned k=0; k<count; ++k, ++_p, ++_n)
			{
				Point3D< Real > p = ( inPoints[k] - _center ) / _scale;
				
				if( !_inBounds(p) )
					continue;
				
				Point3D< Real > myCenter = Point3D< Real >( Real(0.5) , Real(0.5) , Real(0.5) );
				Real myWidth = Real(1.0);
				Real weight = Real(1.0);

				if( useConfidence )
				{
					weight = Real( Length(inNormals[k]) );
				}

				TreeOctNode* temp = &this->tree;
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
		this->normals = new std::vector< Point3D<Real> >();
		int cnt = 0;
		{
			for (unsigned k=0; k<count; ++k)
			{
				Point3D< Real > p = ( inPoints[k] - _center ) / _scale;
				if( !_inBounds(p) )
					continue;

				Point3D< Real > n = inNormals[k] * Real(-1.0);
				
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
					TreeOctNode* temp = &this->tree;
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
				if ( this->_constrainValues )
				{
					int d = 0;
					TreeOctNode* temp = &this->tree;
					Point3D< Real > myCenter = Point3D< Real >( Real(0.5) , Real(0.5) , Real(0.5) );
					Real myWidth = Real(1.0);
					while( true )
					{
						int idx = temp->nodeData.pointIndex;
						if( idx == -1 )
						{
							idx = static_cast<int>( this->_points.size() );
							this->_points.push_back( PointData( p , Real(1.0) ) );
							temp->nodeData.pointIndex = idx;
						}
						else
						{
							this->_points[idx].weight += Real(1.0);
							this->_points[idx].position += p;
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

		if( this->_boundaryType == 0 )
			pointWeightSum *= Real(4.0);
		
		constraintWeight *= static_cast<Real>(pointWeightSum);
		constraintWeight /= static_cast<Real>(cnt);

		MemoryUsage( );

		if( this->_constrainValues )
		{
			for( TreeOctNode* node=this->tree.nextNode() ; node ; node=this->tree.nextNode(node) )
			{
				if( node->nodeData.pointIndex != -1 )
				{
					int idx = node->nodeData.pointIndex;
					this->_points[idx].position /= this->_points[idx].weight;
					int e = ( this->_boundaryType == 0 ? node->depth()-1 : node->depth() ) * adaptiveExponent - ( this->_boundaryType == 0 ? maxDepth-1 : maxDepth ) * (adaptiveExponent-1);
					
					if ( e < 0 )
						this->_points[idx].weight /= Real( 1<<(-e) );
					else
						this->_points[idx].weight *= Real( 1<<  e  );

					this->_points[idx].weight *= Real( constraintWeight );
				}
			}
		}

#if FORCE_NEUMANN_FIELD
		if( this->_boundaryType == 1 )
		{
			for( TreeOctNode* node=this->tree.nextNode() ; node ; node=this->tree.nextNode( node ) )
			{
				int d , off[3];
				node->depthAndOffset( d , off );
				int res = 1<<d;
				if( node->nodeData.normalIndex < 0 )
					continue;
				
				Point3D< Real >& normal = (*this->normals)[node->nodeData.normalIndex];
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

PoissonReconLib::~PoissonReconLib()
{
	if (m_octree)
		delete m_octree;
	m_octree = 0;
}

bool PoissonReconLib::init(	unsigned count,
							const Point* inPoints,
							const Point* inNormals,
							const Parameters& inParams,
							int* outThreadCount/*=0*/)
{
	m_params = inParams;
	if (m_params.depth < 2)
		return false;

	//m_params.depth = std::max(m_params.minDepth,m_params.depth);

	//TODO: let the user set these parameters?
	assert(m_params.depth > 2);
	int kernelDepth = m_params.depth-2;
	
	m_params.solverDivide = std::max(m_params.minDepth,m_params.solverDivide); //solverDivide must be at least as large as minDepth
	m_params.isoDivide = std::max(m_params.minDepth,m_params.isoDivide); //isoDivide must be at least as large as minDepth
	kernelDepth = std::min(kernelDepth,m_params.depth); //kernelDepth can't be greater than depth

	if (m_octree)
		delete m_octree;
	m_octree = 0;

	try
	{
		//Tree construction
		OctNode< TreeNodeData< OutputDensity > , Real >::SetAllocator( MEMORY_ALLOCATOR_BLOCK_SIZE );
		LibOctree* octree = new LibOctree;
#ifdef WITH_OPENMP
		octree->threads = omp_get_num_procs();
#else
		octree->threads = 1;
#endif
		if (outThreadCount)
			*outThreadCount = octree->threads;
		octree->setBSplineData( m_params.depth , 1 );
		int pointCount = octree->setTree( count, inPoints, inNormals, m_params.depth, m_params.minDepth, kernelDepth, m_params.samplesPerNode, m_params.scale, m_params.useConfidence, m_params.pointWeight, m_params.adaptiveExponent );
		octree->ClipTree();
		octree->finalize( m_params.isoDivide );

		m_octree = octree;
	}
	catch(...)
	{
		//an error occurred!
		return false;
	}

	return true;
}

bool PoissonReconLib::reconstruct( CoredVectorMeshData< Vertex >& outMesh )
{
	if (!m_octree)
		return false;

	try
	{
		//Set constraints
		m_octree->SetLaplacianConstraints();

		//Solve linear system
		int maxSolveDepth = m_params.depth;
		m_octree->LaplacianMatrixIteration( m_params.solverDivide, /*ShowResidual*/0, m_params.minIters, m_params.solverAccuracy, maxSolveDepth, m_params.fixedIters );

		//get triangles
		Real isoValue = m_octree->GetIsoValue();
		m_octree->GetMCIsoTriangles( isoValue , m_params.isoDivide, &outMesh , 0 , 1 , !m_params.nonManifold, /*PolygonMesh*/0 );
	}
	catch(...)
	{
		//an error occurred!
		return false;
	}

	return true;
}
