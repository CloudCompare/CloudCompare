/*
Copyright (c) 2006, Michael Kazhdan and Matthew Bolitho
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer. Redistributions in binary form must reproduce
the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution. 

Neither the name of the Johns Hopkins University nor the names of its contributors
may be used to endorse or promote products derived from this software without specific
prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#ifndef MULTI_GRID_OCTREE_DATA_INCLUDED
#define MULTI_GRID_OCTREE_DATA_INCLUDED

#define NEW_CODE 1

//#define MAX_MEMORY_GB 15
#define MAX_MEMORY_GB 0

#define GRADIENT_DOMAIN_SOLUTION 1	// Given the constraint vector-field V(p), there are two ways to solve for the coefficients, x, of the indicator function
									// with respect to the B-spline basis {B_i(p)}
									// 1] Find x minimizing:
									//			|| V(p) - \sum_i \nabla x_i B_i(p) ||^2
									//		which is solved by the system A_1x = b_1 where:
									//			A_1[i,j] = < \nabla B_i(p) , \nabla B_j(p) >
									//			b_1[i]   = < \nabla B_i(p) , V(p) >
									// 2] Formulate this as a Poisson equation:
									//			\sum_i x_i \Delta B_i(p) = \nabla \cdot V(p)
									//		which is solved by the system A_2x = b_2 where:
									//			A_2[i,j] = - < \Delta B_i(p) , B_j(p) >
									//			b_2[i]   = - < B_i(p) , \nabla \cdot V(p) >
									// Although the two system matrices should be the same (assuming that the B_i satisfy dirichlet/neumann boundary conditions)
									// the constraint vectors can differ when V does not satisfy the Neumann boundary conditions:
									//		A_1[i,j] = \int_R < \nabla B_i(p) , \nabla B_j(p) >
									//               = \int_R [ \nabla \cdot ( B_i(p) \nabla B_j(p) ) - B_i(p) \Delta B_j(p) ]
									//               = \int_dR < N(p) , B_i(p) \nabla B_j(p) > + A_2[i,j]
									// and the first integral is zero if either f_i is zero on the boundary dR or the derivative of B_i across the boundary is zero.
									// However, for the constraints we have:
									//		b_1(i)   = \int_R < \nabla B_i(p) , V(p) >
									//               = \int_R [ \nabla \cdot ( B_i(p) V(p) ) - B_i(p) \nabla \cdot V(p) ]
									//               = \int_dR < N(p) ,  B_i(p) V(p) > + b_2[i]
									// In particular, this implies that if the B_i satisfy the Neumann boundary conditions (rather than Dirichlet),
									// and V is not zero across the boundary, then the two constraints are different.
									// Forcing the < V(p) , N(p) > = 0 on the boundary, by killing off the component of the vector-field in the normal direction
									// (FORCE_NEUMANN_FIELD), makes the two systems equal, and the value of this flag should be immaterial.
									// Note that under interpretation 1, we have:
									//		\sum_i b_1(i) = < \nabla \sum_ i B_i(p) , V(p) > = 0
									// because the B_i's sum to one. However, in general, we could have
									//		\sum_i b_2(i) \neq 0.
									// This could cause trouble because the constant functions are in the kernel of the matrix A, so CG will misbehave if the constraint
									// has a non-zero DC term. (Again, forcing < V(p) , N(p) > = 0 along the boundary resolves this problem.)

#define FORCE_NEUMANN_FIELD 1		// This flag forces the normal component across the boundary of the integration domain to be zero.
									// This should be enabled if GRADIENT_DOMAIN_SOLUTION is not, so that CG doesn't run into trouble.

#define ROBERTO_TOLDO_FIX 1

#if !FORCE_NEUMANN_FIELD
#pragma message( "[WARNING] Not zeroing out normal component on boundary" )
#endif // !FORCE_NEUMANN_FIELD

#include "Hash.h"
#include "BSplineData.h"
#include "PointStream.h"

#ifndef _OPENMP
int omp_get_num_procs( void ){ return 1; }
int omp_get_thread_num( void ){ return 0; }
#endif // _OPENMP

class TreeNodeData
{
public:
	static int NodeCount;
	int nodeIndex;

	TreeNodeData( void );
	~TreeNodeData( void );
};

class VertexData
{
	typedef OctNode< TreeNodeData > TreeOctNode;
public:
	static const int VERTEX_COORDINATE_SHIFT = ( sizeof( long long ) * 8 ) / 3;
	static long long   EdgeIndex( const TreeOctNode* node , int eIndex , int maxDepth , int index[DIMENSION] );
	static long long   EdgeIndex( const TreeOctNode* node , int eIndex , int maxDepth );
	static long long   FaceIndex( const TreeOctNode* node , int fIndex , int maxDepth,int index[DIMENSION] );
	static long long   FaceIndex( const TreeOctNode* node , int fIndex , int maxDepth );
	static long long CornerIndex( const TreeOctNode* node , int cIndex , int maxDepth , int index[DIMENSION] );
	static long long CornerIndex( const TreeOctNode* node , int cIndex , int maxDepth );
	static long long CenterIndex( const TreeOctNode* node , int maxDepth , int index[DIMENSION] );
	static long long CenterIndex( const TreeOctNode* node , int maxDepth );
	static long long CornerIndex( int depth , const int offSet[DIMENSION] , int cIndex , int maxDepth , int index[DIMENSION] );
	static long long CenterIndex( int depth , const int offSet[DIMENSION] , int maxDepth , int index[DIMENSION] );
	static long long CornerIndexKey( const int index[DIMENSION] );
};

class SortedTreeNodes
{
	typedef OctNode< TreeNodeData > TreeOctNode;
protected:
	void _sortByZCoordinate( void );
public:
	Pointer( TreeOctNode* ) treeNodes;
	int *nodeCount;
	int maxDepth;
	SortedTreeNodes( void );
	~SortedTreeNodes( void );
	void set( TreeOctNode& root , std::vector< int >* map );
	Pointer( Pointer( int ) ) sliceOffsets;
	static int Slices( int depth );
	std::pair< int , int > sliceSpan( int depth , int off , int d ) const;

	template< int Indices >
	struct  _Indices
	{
		int idx[Indices];
		_Indices( void ){ memset( idx , -1 , sizeof( int ) * Indices ); }
		int& operator[] ( int i ) { return idx[i]; }
		const int& operator[] ( int i ) const { return idx[i]; }
	};
	typedef _Indices< Square::CORNERS > SquareCornerIndices;
	typedef _Indices< Square::EDGES > SquareEdgeIndices;
	typedef _Indices< Square::FACES > SquareFaceIndices;

	struct SliceTableData
	{
		std::vector< SquareCornerIndices > cTable;
		std::vector< SquareEdgeIndices > eTable;
		std::vector< SquareFaceIndices > fTable;
		int cCount , eCount , fCount , nodeOffset , nodeCount;
		SliceTableData( void ){ fCount = eCount = cCount = 0; }
		~SliceTableData( void ){ clear(); }
		void clear( void ) { cTable.clear() , eTable.clear() , fTable.clear() , fCount = eCount = cCount = 0; }
		SquareCornerIndices& cornerIndices( const TreeOctNode* node );
		SquareCornerIndices& cornerIndices( int idx );
		const SquareCornerIndices& cornerIndices( const TreeOctNode* node ) const;
		const SquareCornerIndices& cornerIndices( int idx ) const;
		SquareEdgeIndices& edgeIndices( const TreeOctNode* node );
		SquareEdgeIndices& edgeIndices( int idx );
		const SquareEdgeIndices& edgeIndices( const TreeOctNode* node ) const;
		const SquareEdgeIndices& edgeIndices( int idx ) const;
		SquareFaceIndices& faceIndices( const TreeOctNode* node );
		SquareFaceIndices& faceIndices( int idx );
		const SquareFaceIndices& faceIndices( const TreeOctNode* node ) const;
		const SquareFaceIndices& faceIndices( int idx ) const;
	protected:
		std::vector< int > _cMap , _eMap , _fMap;
		friend class SortedTreeNodes;
	};
	struct XSliceTableData
	{
		std::vector< SquareCornerIndices > eTable;
		std::vector< SquareEdgeIndices > fTable;
		int fCount , eCount , nodeOffset , nodeCount;
		XSliceTableData( void ){ fCount = eCount = 0; }
		~XSliceTableData( void ){ clear(); }
		void clear( void ) { fTable.clear() , eTable.clear() , fCount = eCount = 0; }
		SquareCornerIndices& edgeIndices( const TreeOctNode* node );
		SquareCornerIndices& edgeIndices( int idx );
		const SquareCornerIndices& edgeIndices( const TreeOctNode* node ) const;
		const SquareCornerIndices& edgeIndices( int idx ) const;
		SquareEdgeIndices& faceIndices( const TreeOctNode* node );
		SquareEdgeIndices& faceIndices( int idx );
		const SquareEdgeIndices& faceIndices( const TreeOctNode* node ) const;
		const SquareEdgeIndices& faceIndices( int idx ) const;
	protected:
		std::vector< int > _eMap , _fMap;
		friend class SortedTreeNodes;
	};
	void setSliceTableData (  SliceTableData& sData , int depth , int offset , int threads ) const;
	void setXSliceTableData( XSliceTableData& sData , int depth , int offset , int threads ) const;
};


template< class Real >
class Octree
{
	typedef OctNode< TreeNodeData > TreeOctNode;
public:
	template< class V >
	struct ProjectiveData
	{
		V v;
		Real w;
		ProjectiveData( V vv=V(0) , Real ww=Real(0) ) : v(vv) , w(ww) { }
		operator V (){ return w!=0 ? v/w : v*w; }
		ProjectiveData& operator += ( const ProjectiveData& p ){ v += p.v , w += p.w ; return *this; }
		ProjectiveData& operator -= ( const ProjectiveData& p ){ v -= p.v , w -= p.w ; return *this; }
		ProjectiveData& operator *= ( Real s ){ v *= s , w *= s ; return *this; }
		ProjectiveData& operator /= ( Real s ){ v /= s , w /= s ; return *this; }
		ProjectiveData operator + ( const ProjectiveData& p ) const { return ProjectiveData( v+p.v , w+p.w ); }
		ProjectiveData operator - ( const ProjectiveData& p ) const { return ProjectiveData( v-p.v , w-p.w ); }
		ProjectiveData operator * ( Real s ) const { return ProjectiveData( v*s , w*s ); }
		ProjectiveData operator / ( Real s ) const { return ProjectiveData( v/s , w/s ); }
	};
	struct PointData
	{
		Point3D< Real > position;
		Real weightedCoarserValue;
		Real weight;
		PointData( Point3D< Real > p=Point3D< Real >() , Real w=0 ) { position = p , weight = w , weightedCoarserValue = Real(0); }
	};
	template< class Data >
	struct SparseNodeData
	{
		std::vector< int > indices;
		std::vector< Data > data;
		int index( const TreeOctNode* node ) const { return node->nodeData.nodeIndex>=indices.size() ? -1 : indices[ node->nodeData.nodeIndex ]; }
	};
protected:
	SortedTreeNodes _sNodes;
	int _splatDepth;
	int _minDepth;
	int _fullDepth;
	bool _constrainValues;
	int _boundaryType;
	Real _scale;
	Point3D< Real > _center;
	std::vector< int > _pointCount;
	BSplineData< 2 > _fData;

	bool _InBounds( Point3D< Real > ) const;

	double GetLaplacian  ( const typename BSplineData< 2 >::Integrator& integrator , int d , const int off1[3] , const int off2[3] , bool childParent ) const;
	double GetDivergence1( const typename BSplineData< 2 >::Integrator& integrator , int d , const int off1[3] , const int off2[3] , bool childParent , const Point3D< Real >& normal1 ) const;
	double GetDivergence2( const typename BSplineData< 2 >::Integrator& integrator , int d , const int off1[3] , const int off2[3] , bool childParent , const Point3D< Real >& normal2 ) const;
	Point3D< double > GetDivergence1( const typename BSplineData< 2 >::Integrator& integrator , int d , const int off1[3] , const int off2[3] , bool childParent ) const;
	Point3D< double > GetDivergence2( const typename BSplineData< 2 >::Integrator& integrator , int d , const int off1[3] , const int off2[3] , bool childParent ) const;

	template< class C , int N > struct Stencil{ C values[N][N][N]; };
	struct CenterValueStencil
	{
		Stencil< double , 3 > stencil;
		Stencil< double , 3 > stencils[8];
	};
	struct CornerValueStencil
	{
		Stencil< double , 3 > stencil[8];
		Stencil< double , 3 > stencils[8][8];
	};
	struct CornerNormalStencil
	{
		Stencil< Point3D< double > , 3 > stencil[8];
		Stencil< Point3D< double > , 3 > stencils[8][8];
	};

	void _setMultiColorIndices( int start , int end , std::vector< std::vector< int > >& indices ) const;
	int _SolveSystemGS( SparseNodeData< PointData >& pointInfo , int depth , const typename BSplineData< 2 >::Integrator& integrator , const SortedTreeNodes& sNodes , Pointer( Real ) solution , Pointer( Real ) constraints , Pointer( Real ) metSolutionConstraints , int iters , bool coarseToFine , bool showResidual=false , double* bNorm2=NULL , double* inRNorm2=NULL , double* outRNorm2=NULL , bool forceSilent=false );
	int _SolveSystemCG( SparseNodeData< PointData >& pointInfo , int depth , const typename BSplineData< 2 >::Integrator& integrator , const SortedTreeNodes& sNodes , Pointer( Real ) solution , Pointer( Real ) constraints , Pointer( Real ) metSolutionConstraints , int iters , bool coarseToFine , bool showResidual=false , double* bNorm2=NULL , double* inRNorm2=NULL , double* outRNorm2=NULL , double accuracy=0 );

	int GetMatrixRowSize( const typename TreeOctNode::Neighbors5& neighbors5 , bool symmetric ) const;
	int SetMatrixRow( const SparseNodeData< PointData >& pointInfo , const typename TreeOctNode::Neighbors5& neighbors5 , Pointer( MatrixEntry< Real > ) row , int offset , const typename BSplineData< 2 >::Integrator& integrator , const Stencil< double , 5 >& stencil , bool symmetric ) const;

	void SetDivergenceStencil ( int depth , const typename BSplineData< 2 >::Integrator& integrator , Stencil< Point3D< double > , 5 >& stencil , bool scatter ) const;
	void SetDivergenceStencils( int depth , const typename BSplineData< 2 >::Integrator& integrator , Stencil< Point3D< double > , 5 > stencil[2][2][2] , bool scatter ) const;
	void SetLaplacianStencil  ( int depth , const typename BSplineData< 2 >::Integrator& integrator , Stencil< double , 5 >& stencil ) const;
	void SetLaplacianStencils ( int depth , const typename BSplineData< 2 >::Integrator& integrator , Stencil< double , 5 > stencil[2][2][2] ) const;
	void SetCenterEvaluationStencil ( const typename BSplineData< 2 >::template CenterEvaluator< 1 >& evaluator , int depth , Stencil< double , 3 >& stencil ) const;
	void SetCenterEvaluationStencils( const typename BSplineData< 2 >::template CenterEvaluator< 1 >& evaluator , int depth , Stencil< double , 3 > stencil[8] ) const;
	void SetCornerEvaluationStencil ( const typename BSplineData< 2 >::template CornerEvaluator< 2 >& evaluator , int depth , Stencil< double , 3 > stencil [8]    ) const;
	void SetCornerEvaluationStencils( const typename BSplineData< 2 >::template CornerEvaluator< 2 >& evaluator , int depth , Stencil< double , 3 > stencils[8][8] ) const;
	void SetCornerNormalEvaluationStencil ( const typename BSplineData< 2 >::template CornerEvaluator< 2 >& evaluator , int depth , Stencil< Point3D< double > , 3 > stencil [8]    ) const;
	void SetCornerNormalEvaluationStencils( const typename BSplineData< 2 >::template CornerEvaluator< 2 >& evaluator , int depth , Stencil< Point3D< double > , 3 > stencils[8][8] ) const;
	void SetCornerNormalEvaluationStencil ( const typename BSplineData< 2 >::template CornerEvaluator< 2 >& evaluator , int depth , Stencil< Point3D< double > , 5 > stencil [8]    ) const;
	void SetCornerNormalEvaluationStencils( const typename BSplineData< 2 >::template CornerEvaluator< 2 >& evaluator , int depth , Stencil< Point3D< double > , 5 > stencils[8][8] ) const;

	static void UpdateCoarserSupportBounds( const TreeOctNode* node , int& startX , int& endX , int& startY , int& endY , int& startZ , int& endZ );

	void UpdateConstraintsFromCoarser( const SparseNodeData< PointData >& pointInfo , const typename TreeOctNode::Neighbors5& neighbors5 , const typename TreeOctNode::Neighbors5& pNeighbors5 , TreeOctNode* node , Pointer( Real ) constraints , ConstPointer( Real ) metSolution , const typename BSplineData< 2 >::Integrator& integrator , const Stencil< double , 5 >& stencil ) const;
	// Updates the constraints @(depth-1) based on the solution coefficients @(depth)
	void UpdateConstraintsFromFiner( const typename BSplineData< 2 >::Integrator& integrator , int depth , const SortedTreeNodes& sNodes , ConstPointer( Real ) fineSolution , Pointer( Real ) coarseConstraints ) const;
	// Evaluate the points @(depth) using coefficients @(depth-1)
	void SetPointValuesFromCoarser( SparseNodeData< PointData >& pointInfo , int depth , const SortedTreeNodes& sNodes , ConstPointer( Real ) coarseCoefficients );
	// Evalutes the solution @(depth) at the points @(depth-1) and updates the met constraints @(depth-1)
	void SetPointConstraintsFromFiner ( const SparseNodeData< PointData >& pointInfo , int depth , const SortedTreeNodes& sNodes , ConstPointer( Real ) finerCoefficients , Pointer( Real ) metConstraints ) const;
	Real _WeightedCoarserFunctionValue( const PointData& pointData , const typename TreeOctNode::NeighborKey3& neighborKey3 , const TreeOctNode* node , ConstPointer( Real ) coarseCoefficients ) const;
	Real _WeightedFinerFunctionValue  ( const PointData& pointData , const typename TreeOctNode::NeighborKey3& neighborKey3 , const TreeOctNode* node , ConstPointer( Real )  finerCoefficients ) const;

	// Down samples constraints @(depth) to constraints @(depth-1)
	template< class C > void DownSample( int depth , const SortedTreeNodes& sNodes , ConstPointer( C ) fineConstraints    , Pointer( C ) coarseConstraints ) const;
	// Up samples solution @(depth-1) to solution @(depth)
	template< class C > void UpSample  ( int depth , const SortedTreeNodes& sNodes , ConstPointer( C ) coarseCoefficients , Pointer( C )  fineCoefficients ) const;
	int GetSliceMatrixAndUpdateConstraints( const SparseNodeData< PointData >& pointInfo , SparseMatrix< Real >& matrix , Pointer( Real ) constraints , const typename BSplineData< 2 >::Integrator& integrator , int depth , const SortedTreeNodes& sNodes , ConstPointer( Real ) metSolution , bool coarseToFine , int nStart , int nEnd );
	int GetMatrixAndUpdateConstraints( const SparseNodeData< PointData >& pointInfo , SparseSymmetricMatrix< Real >& matrix , Pointer( Real ) constraints , const typename BSplineData< 2 >::Integrator& integrator , int depth , const SortedTreeNodes& sNodes , ConstPointer( Real ) metSolution , bool coarseToFine );


	int UpdateWeightContribution( std::vector< Real >& kernelDensityWeights , TreeOctNode* node , const Point3D<Real>& position , typename TreeOctNode::NeighborKey3& neighborKey , Real weight=Real(1.0) );
	Real GetSamplesPerNode( ConstPointer( Real ) kernelDensityWeight , const TreeOctNode* node , const Point3D< Real >& position , typename TreeOctNode::ConstNeighborKey3& neighborKey );
	Real GetSamplesPerNode( ConstPointer( Real ) kernelDensityWeight ,       TreeOctNode* node , const Point3D< Real >& position , typename TreeOctNode::NeighborKey3&      neighborKey );
public:
	void GetSampleDepthAndWeight( ConstPointer( Real ) kernelDensityWeight , const Point3D< Real >& position , typename TreeOctNode::NeighborKey3& neighborKey , Real& depth , Real& weight );
	void GetSampleDepthAndWeight( ConstPointer( Real ) kernelDensityWeight , const Point3D< Real >& position , typename TreeOctNode::ConstNeighborKey3& neighborKey , Real& depth , Real& weight );
protected:
	void GetSampleDepthAndWeight( ConstPointer( Real ) kernelDensityWeight , const TreeOctNode* node , const Point3D< Real >& position , typename TreeOctNode::ConstNeighborKey3& neighborKey , Real& depth , Real& weight );
	void GetSampleDepthAndWeight( ConstPointer( Real ) kernelDensityWeight ,       TreeOctNode* node , const Point3D< Real >& position , typename TreeOctNode::NeighborKey3&      neighborKey , Real& depth , Real& weight );
	template< class V >
	int SplatPointData( TreeOctNode* node , const Point3D<Real>& point , const V& v , SparseNodeData< V >& data , typename TreeOctNode::NeighborKey3& neighborKey );
	template< class V >
	int SplatPointData( TreeOctNode* node , const Point3D<Real>& point , const V& v , SparseNodeData< V >& data , typename TreeOctNode::ConstNeighborKey3& neighborKey );
	template< class V >
	Real SplatPointData( ConstPointer( Real ) kernelDensityWeights , const Point3D< Real >& point , const V& v , SparseNodeData< V >& data , typename TreeOctNode::NeighborKey3& neighborKey , int minDepth , int maxDepth , int dim=DIMENSION );
	template< class V >
	void MultiSplatPointData( ConstPointer( Real ) kernelDensityWeights , const Point3D< Real >& point , const V& v , SparseNodeData< V >& data , typename TreeOctNode::NeighborKey3& neighborKey , int maxDepth , int dim=DIMENSION );
	template< class V >
	void MultiSplatPointData( ConstPointer( Real ) kernelDensityWeights , const Point3D< Real >& point , const V& v , SparseNodeData< V >& data , typename TreeOctNode::ConstNeighborKey3& neighborKey , int dim=DIMENSION );

	int HasNormals( TreeOctNode* node , const SparseNodeData< Point3D< Real > >& normalInfo );

	///////////////////////////
	// Iso-Surfacing Methods //
	///////////////////////////
	struct IsoEdge
	{
		long long edges[2];
		IsoEdge( void ){ edges[0] = edges[1] = 0; }
		IsoEdge( long long v1 , long long v2 ){ edges[0] = v1 , edges[1] = v2; }
		long long& operator[]( int idx ){ return edges[idx]; }
		const long long& operator[]( int idx ) const { return edges[idx]; }
	};
	struct FaceEdges
	{
		IsoEdge edges[2];
		int count;
	};
	template< class Vertex >
	struct SliceValues
	{
		typename SortedTreeNodes::SliceTableData sliceData;
		Pointer( Real ) cornerValues ; Pointer( Point3D< Real > ) cornerNormals ; Pointer( char ) cornerSet;
		Pointer( long long ) edgeKeys ; Pointer( char ) edgeSet;
		Pointer( FaceEdges ) faceEdges ; Pointer( char ) faceSet;
		Pointer( char ) mcIndices;
		hash_map< long long , std::vector< IsoEdge > > faceEdgeMap;
		hash_map< long long , std::pair< int , Vertex > > edgeVertexMap;
		hash_map< long long , long long > vertexPairMap;

		SliceValues( void );
		~SliceValues( void );
		void reset( bool nonLinearFit );
	protected:
		int _oldCCount , _oldECount , _oldFCount , _oldNCount;
	};
	template< class Vertex >
	struct XSliceValues
	{
		typename SortedTreeNodes::XSliceTableData xSliceData;
		Pointer( long long ) edgeKeys ; Pointer( char ) edgeSet;
		Pointer( FaceEdges ) faceEdges ; Pointer( char ) faceSet;
		hash_map< long long , std::vector< IsoEdge > > faceEdgeMap;
		hash_map< long long , std::pair< int , Vertex > > edgeVertexMap;
		hash_map< long long , long long > vertexPairMap;

		XSliceValues( void );
		~XSliceValues( void );
		void reset( void );
	protected:
		int _oldECount , _oldFCount;
	};
	template< class Vertex >
	struct SlabValues
	{
		XSliceValues< Vertex > _xSliceValues[2];
		SliceValues< Vertex > _sliceValues[2];
		SliceValues< Vertex >& sliceValues( int idx ){ return _sliceValues[idx&1]; }
		const SliceValues< Vertex >& sliceValues( int idx ) const { return _sliceValues[idx&1]; }
		XSliceValues< Vertex >& xSliceValues( int idx ){ return _xSliceValues[idx&1]; }
		const XSliceValues< Vertex >& xSliceValues( int idx ) const { return _xSliceValues[idx&1]; }
	};
	template< class Vertex >
	void SetSliceIsoCorners( ConstPointer( Real ) solution , ConstPointer( Real ) coarseSolution , Real isoValue , int depth , int slice ,         std::vector< SlabValues< Vertex > >& sValues , const typename BSplineData< 2 >::template CornerEvaluator< 2 >& evaluator , const Stencil< double , 3 > stencil[8] , const Stencil< double , 3 > stencils[8][8] , const Stencil< Point3D< double > , 3 > nStencil[8] , const Stencil< Point3D< double > , 3 > nStencils[8][8] , int threads );
	template< class Vertex >
	void SetSliceIsoCorners( ConstPointer( Real ) solution , ConstPointer( Real ) coarseSolution , Real isoValue , int depth , int slice , int z , std::vector< SlabValues< Vertex > >& sValues , const typename BSplineData< 2 >::template CornerEvaluator< 2 >& evaluator , const Stencil< double , 3 > stencil[8] , const Stencil< double , 3 > stencils[8][8] , const Stencil< Point3D< double > , 3 > nStencil[8] , const Stencil< Point3D< double > , 3 > nStencils[8][8] , int threads );
	template< class Vertex >
	void SetSliceIsoVertices( ConstPointer( Real ) kernelDensityWeights , const SparseNodeData< ProjectiveData< Point3D< Real > > >* colorData , Real isoValue , int depth , int slice ,         int& vOffset , CoredMeshData< Vertex >& mesh , std::vector< SlabValues< Vertex > >& sValues , int threads );
	template< class Vertex >
	void SetSliceIsoVertices( ConstPointer( Real ) kernelDensityWeights , const SparseNodeData< ProjectiveData< Point3D< Real > > >* colorData , Real isoValue , int depth , int slice , int z , int& vOffset , CoredMeshData< Vertex >& mesh , std::vector< SlabValues< Vertex > >& sValues , int threads );
	template< class Vertex >
	void SetXSliceIsoVertices( ConstPointer( Real ) kernelDensityWeights , const SparseNodeData< ProjectiveData< Point3D< Real > > >* colorData , Real isoValue , int depth , int slab , int& vOffset , CoredMeshData< Vertex >& mesh , std::vector< SlabValues< Vertex > >& sValues , int threads );
	template< class Vertex >
	void CopyFinerSliceIsoEdgeKeys( int depth , int slice ,         std::vector< SlabValues< Vertex > >& sValues , int threads );
	template< class Vertex >
	void CopyFinerSliceIsoEdgeKeys( int depth , int slice , int z , std::vector< SlabValues< Vertex > >& sValues , int threads );
	template< class Vertex >
	void CopyFinerXSliceIsoEdgeKeys( int depth , int slab , std::vector< SlabValues< Vertex > >& sValues , int threads );
	template< class Vertex >
	void SetSliceIsoEdges( int depth , int slice ,         std::vector< SlabValues< Vertex > >& slabValues , int threads );
	template< class Vertex >
	void SetSliceIsoEdges( int depth , int slice , int z , std::vector< SlabValues< Vertex > >& slabValues , int threads );
	template< class Vertex >
	void SetXSliceIsoEdges( int depth , int slice , std::vector< SlabValues< Vertex > >& slabValues , int threads );

	template< class Vertex , class _Vertex >
	void SetIsoSurface( int depth , int offset , const SliceValues< Vertex >& bValues , const SliceValues< Vertex >& fValues , const XSliceValues< Vertex >& xValues , CoredMeshData< Vertex >& mesh , bool polygonMesh , bool addBarycenter , int& vOffset , int threads );

	template< class Vertex , class _Vertex >
	static int AddIsoPolygons( CoredMeshData< Vertex >& mesh , std::vector< std::pair< int , Vertex > >& polygon , bool polygonMesh , bool addBarycenter , int& vOffset );

	template< class Vertex >
	bool GetIsoVertex( ConstPointer( Real ) kernelDensityWeights , const SparseNodeData< ProjectiveData< Point3D< Real > > >* colorData , Real isoValue , typename TreeOctNode::ConstNeighborKey3& neighborKey3 , const TreeOctNode* node , int edgeIndex , int z , const SliceValues< Vertex >& sValues , Vertex& vertex );
	template< class Vertex >
	bool GetIsoVertex( ConstPointer( Real ) kernelDensityWeights , const SparseNodeData< ProjectiveData< Point3D< Real > > >* colorData , Real isoValue , typename TreeOctNode::ConstNeighborKey3& neighborKey3 , const TreeOctNode* node , int cornerIndex , const SliceValues< Vertex >& bValues , const SliceValues< Vertex >& fValues , Vertex& vertex );


	////////////////////////
	// Evaluation Methods //
	////////////////////////
	template< class V >
	V getCornerValue( const typename TreeOctNode::ConstNeighborKey3& neighborKey3 , const TreeOctNode* node , int corner , ConstPointer( V ) solution , ConstPointer( V ) metSolution , const typename BSplineData< 2 >::template CornerEvaluator< 2 >& evaluator , const Stencil< double , 3 >& stencil , const Stencil< double , 3 > stencils[8] , bool isInterior ) const;
	Point3D< Real > getCornerNormal( const typename TreeOctNode::ConstNeighbors5& neighbors5 , const typename TreeOctNode::ConstNeighbors5& pNeighbors5 , const TreeOctNode* node , int corner , ConstPointer( Real ) solution , ConstPointer( Real ) metSolution , const typename BSplineData< 2 >::template CornerEvaluator< 2 >& evaluator , const Stencil< Point3D< double > , 5 >& nStencil , const Stencil< Point3D< double > , 5 > nStencils[8] , bool isInterior ) const;
	std::pair< Real , Point3D< Real > > getCornerValueAndNormal( const typename TreeOctNode::ConstNeighborKey3& neighborKey3 , const TreeOctNode* node , int corner , ConstPointer( Real ) solution , ConstPointer( Real ) metSolution , const typename BSplineData< 2 >::template CornerEvaluator< 2 >& evaluator , const Stencil< double , 3 >& vStencil , const Stencil< double , 3 > vStencils[8] , const Stencil< Point3D< double > , 3 >& nStencil , const Stencil< Point3D< double > , 3 > nStencils[8] , bool isInterior ) const;
	template< class V >
	V getCenterValue( const typename TreeOctNode::ConstNeighborKey3& neighborKey3 , const TreeOctNode* node , ConstPointer( V ) solution , ConstPointer( V ) metSolution , const typename BSplineData< 2 >::template CenterEvaluator< 1 >& evaluator , const Stencil< double , 3 >& stencil , const Stencil< double , 3 >& pStencil , bool isInterior ) const;

	static bool _IsInset( const TreeOctNode* node );
	static bool _IsInsetSupported( const TreeOctNode* node );

	void refineBoundary( std::vector< int >* map );
public:
	static double maxMemoryUsage;
	int threads;
	TreeOctNode tree;

	static double MemoryUsage( void );
	Octree( void );

	void MakeComplete( std::vector< int >* map=NULL );
	void Finalize( std::vector< int >* map=NULL );
	void ClipTree( const SparseNodeData< Point3D< Real > >& normalInfo );

protected:
	template< class V > V _Evaluate(         ConstPointer( V )  coefficients , Point3D< Real > p , typename TreeOctNode::ConstNeighborKey3& neighborKey3 ) const;
	template< class V > V _Evaluate( const SparseNodeData< V >& coefficients , Point3D< Real > p , typename TreeOctNode::ConstNeighborKey3& neighborKey3 ) const;
public:
	template< class V > V Evaluate(         ConstPointer( V )  coefficients , Point3D< Real > p , const BSplineData< 2 >* fData = NULL , int depth=-1 ) const;
	template< class V > V Evaluate( const SparseNodeData< V >& coefficients , Point3D< Real > p , const BSplineData< 2 >* fData = NULL , int depth=-1 ) const;
	template< class V > Pointer( V ) Evaluate( ConstPointer( V ) coefficients , int& res , Real isoValue=0.f , int depth=-1 );
	// After calling set tree, the indices of the octree node will be stored by depth, and within depth they will be sorted by z-coordinate
	template< class PointReal >
	int SetTree( OrientedPointStream< PointReal >* pointStream , int minDepth , int maxDepth , int fullDepth , int splatDepth , Real samplesPerNode ,
		Real scaleFactor , bool useConfidence , bool useNormalWeight , Real constraintWeight , int adaptiveExponent ,
		std::vector< Real >& kernelDensityWeights ,
		SparseNodeData< PointData >& pointInfo , SparseNodeData< Point3D< Real > >& normalInfo , std::vector< Real >& centerWeights ,
		XForm4x4< Real >& xForm , int boundaryType=BSplineElements< 2 >::NONE , bool makeComplete=false );

	template< class PointReal , class Data , class _Data >
	int SetTree( OrientedPointStreamWithData< PointReal , Data >* pointStream , int minDepth , int maxDepth , int fullDepth , int splatDepth , Real samplesPerNode ,
		Real scaleFactor , bool useConfidence , bool useNormalWeight ,
		Real constraintWeight , int adaptiveExponent ,
		std::vector< Real >& kernelDensityWeights ,
		SparseNodeData< PointData >& pointInfo , SparseNodeData< Point3D< Real > >& normalInfo , std::vector< Real >& centerWeights ,
		SparseNodeData< ProjectiveData< _Data > >& dataValues ,
		XForm4x4< Real >& xForm , int boundaryType=BSplineElements< 2 >::NONE , bool makeComplete=false );

	Pointer( Real ) SetLaplacianConstraints( const SparseNodeData< Point3D< Real > >& normalInfo );
	Pointer( Real ) SolveSystem( SparseNodeData< PointData >& pointInfo , Pointer( Real ) constraints , bool showResidual , int iters , int maxSolveDepth , int cgDepth=0 , double cgAccuracy=0 );

	Real GetIsoValue( ConstPointer( Real ) solution , const std::vector< Real >& centerWeights );
	template< class Vertex , class _Vertex >
	void GetMCIsoSurface( ConstPointer( Real ) kernelDensityWeights , const SparseNodeData< ProjectiveData< Point3D< Real > > >* colorData , ConstPointer( Real ) solution , Real isoValue , CoredMeshData< Vertex >& mesh , bool nonLinearFit=true , bool addBarycenter=false , bool polygonMesh=false );
};
template< class Real >
void Reset( void )
{
	TreeNodeData::NodeCount=0;
	Octree< Real >::maxMemoryUsage = 0;
}

#include "MultiGridOctreeData.inl"
#include "MultiGridOctreeData.SortedTreeNodes.inl"
#include "MultiGridOctreeData.IsoSurface.inl"
#endif // MULTI_GRID_OCTREE_DATA_INCLUDED
