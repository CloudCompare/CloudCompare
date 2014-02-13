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
typedef float Real;
typedef float MatrixReal;


template< bool StoreDensity >
class TreeNodeData
{
public:
	int nodeIndex;
	union
	{
		int mcIndex;
		int normalIndex;
	};
	Real centerWeightContribution[StoreDensity?2:1];
	Real constraint , solution;
	int pointIndex;

	TreeNodeData(void);
	~TreeNodeData(void);
};

template< bool OutputDensity >
class RootInfo
{
	typedef OctNode< TreeNodeData< OutputDensity > , Real > TreeOctNode;
public:
	const TreeOctNode* node;
	int edgeIndex;
	long long key;
};

template< bool OutputDensity >
class VertexData
{
	typedef OctNode< TreeNodeData< OutputDensity > , Real > TreeOctNode;
public:
	static const int VERTEX_COORDINATE_SHIFT = ( sizeof( long long ) * 8 ) / 3;
	static long long EdgeIndex( const TreeOctNode* node , int eIndex , int maxDepth , int index[DIMENSION] );
	static long long EdgeIndex( const TreeOctNode* node , int eIndex , int maxDepth );
	static long long FaceIndex( const TreeOctNode* node , int fIndex , int maxDepth,int index[DIMENSION] );
	static long long FaceIndex( const TreeOctNode* node , int fIndex , int maxDepth );
	static long long CornerIndex( const TreeOctNode* node , int cIndex , int maxDepth , int index[DIMENSION] );
	static long long CornerIndex( const TreeOctNode* node , int cIndex , int maxDepth );
	static long long CenterIndex( const TreeOctNode* node , int maxDepth , int index[DIMENSION] );
	static long long CenterIndex( const TreeOctNode* node , int maxDepth );
	static long long CornerIndex( int depth , const int offSet[DIMENSION] , int cIndex , int maxDepth , int index[DIMENSION] );
	static long long CenterIndex( int depth , const int offSet[DIMENSION] , int maxDepth , int index[DIMENSION] );
	static long long CornerIndexKey( const int index[DIMENSION] );
};
template< bool OutputDensity >
class SortedTreeNodes
{
	typedef OctNode< TreeNodeData< OutputDensity > , Real > TreeOctNode;
public:
	Pointer( TreeOctNode* ) treeNodes;
	int *nodeCount;
	int maxDepth;
	SortedTreeNodes( void );
	~SortedTreeNodes( void );
	void set( TreeOctNode& root );
	struct CornerIndices
	{
		int idx[Cube::CORNERS];
		CornerIndices( void ) { memset( idx , -1 , sizeof( int ) * Cube::CORNERS ); }
		int& operator[] ( int i ) { return idx[i]; }
		const int& operator[] ( int i ) const { return idx[i]; }
	};
	struct CornerTableData
	{
		CornerTableData( void ) { cCount=0; }
		~CornerTableData( void ) { clear(); }
		void clear( void ) { cTable.clear() ; cCount = 0; }
		CornerIndices& operator[] ( const TreeOctNode* node );
		const CornerIndices& operator[] ( const TreeOctNode* node ) const;
		CornerIndices& cornerIndices( const TreeOctNode* node );
		const CornerIndices& cornerIndices( const TreeOctNode* node ) const;
		int cCount;
		std::vector< CornerIndices > cTable;
		std::vector< int > offsets;
	};
	void setCornerTable( CornerTableData& cData , const TreeOctNode* rootNode , int depth , int threads ) const;
	void setCornerTable( CornerTableData& cData , const TreeOctNode* rootNode ,             int threads ) const { setCornerTable( cData , rootNode , maxDepth-1 , threads ); }
	void setCornerTable( CornerTableData& cData ,                                           int threads ) const { setCornerTable( cData , NULL     , maxDepth-1 , threads ); }
	int getMaxCornerCount( int depth , int maxDepth , int threads ) const ;
	struct EdgeIndices
	{
		int idx[Cube::EDGES];
		EdgeIndices( void ) { memset( idx , -1 , sizeof( int ) * Cube::EDGES ); }
		int& operator[] ( int i ) { return idx[i]; }
		const int& operator[] ( int i ) const { return idx[i]; }
	};
	struct EdgeTableData
	{
		EdgeTableData( void ) { eCount=0; }
		~EdgeTableData( void ) { clear(); }
		void clear( void ) { eTable.clear() , eCount=0; }
		EdgeIndices& operator[] ( const TreeOctNode* node );
		const EdgeIndices& operator[] ( const TreeOctNode* node ) const;
		EdgeIndices& edgeIndices( const TreeOctNode* node );
		const EdgeIndices& edgeIndices( const TreeOctNode* node ) const;
		int eCount;
		std::vector< EdgeIndices > eTable;
		std::vector< int > offsets;
	};
	void setEdgeTable( EdgeTableData& eData , const TreeOctNode* rootNode , int depth , int threads );
	void setEdgeTable( EdgeTableData& eData , const TreeOctNode* rootNode ,             int threads ) { setEdgeTable( eData , rootNode , maxDepth-1 , threads ); }
	void setEdgeTable( EdgeTableData& eData ,                                           int threads ) { setEdgeTable( eData , NULL , maxDepth-1 , threads ); }
	int getMaxEdgeCount( const TreeOctNode* rootNode , int depth , int threads ) const ;
};


template< int Degree , bool OutputDensity >
class Octree
{
protected:
	typedef OctNode< TreeNodeData< OutputDensity > , Real > TreeOctNode;
	SortedTreeNodes< OutputDensity > _sNodes;
	Real samplesPerNode;
	int splatDepth;
	int _minDepth;
	bool _constrainValues;
	int _boundaryType;
	Real _scale;
	Point3D< Real > _center;
	std::vector< int > _pointCount;
	struct PointData
	{
		Point3D< Real > position;
		Real coarserValue;
		Real weight;
		PointData( Point3D< Real > p=Point3D< Real >() , Real w=0 ) { position = p , weight = w , coarserValue = Real(0); }
	};
	std::vector< PointData > _points;

	bool _inBounds( Point3D< Real > ) const;

	Real radius;
	int width;
	double GetLaplacian  ( const typename BSplineData< Degree , Real >::Integrator& integrator , int d , const int off1[3] , const int off2[3] , bool childParent ) const;
	double GetDivergence1( const typename BSplineData< Degree , Real >::Integrator& integrator , int d , const int off1[3] , const int off2[3] , bool childParent , const Point3D< Real >& normal1 ) const;
	double GetDivergence2( const typename BSplineData< Degree , Real >::Integrator& integrator , int d , const int off1[3] , const int off2[3] , bool childParent , const Point3D< Real >& normal2 ) const;
	Point3D< double > GetDivergence1( const typename BSplineData< Degree , Real >::Integrator& integrator , int d , const int off1[3] , const int off2[3] , bool childParent ) const;
	Point3D< double > GetDivergence2( const typename BSplineData< Degree , Real >::Integrator& integrator , int d , const int off1[3] , const int off2[3] , bool childParent ) const;

	class AdjacencyCountFunction
	{
	public:
		int adjacencyCount;
		void Function(const TreeOctNode* node1,const TreeOctNode* node2);
	};
	class AdjacencySetFunction{
	public:
		int *adjacencies,adjacencyCount;
		void Function(const TreeOctNode* node1,const TreeOctNode* node2);
	};

	class RefineFunction{
	public:
		int depth;
		void Function(TreeOctNode* node1,const TreeOctNode* node2);
	};
	class FaceEdgesFunction
	{
	public:
		int fIndex , maxDepth;
		std::vector< std::pair< RootInfo< OutputDensity > , RootInfo< OutputDensity > > >* edges;
		hash_map< long long , std::pair< RootInfo< OutputDensity > , int > >* vertexCount;
		void Function( const TreeOctNode* node1 , const TreeOctNode* node2 );
	};

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
		Stencil< Point3D< double > , 5 > stencil[8];
		Stencil< Point3D< double > , 5 > stencils[8][8];
	};

	int _SolveFixedDepthMatrix( int depth , const typename BSplineData< Degree , Real >::Integrator& integrator , const SortedTreeNodes< OutputDensity >& sNodes , Real* subConstraints ,                     bool showResidual , int minIters , double accuracy , bool noSolve = false , int fixedIters=-1 );
	int _SolveFixedDepthMatrix( int depth , const typename BSplineData< Degree , Real >::Integrator& integrator , const SortedTreeNodes< OutputDensity >& sNodes , Real* subConstraints , int startingDepth , bool showResidual , int minIters , double accuracy , bool noSolve = false , int fixedIters=-1 );

	void SetMatrixRowBounds( const TreeOctNode* node , int rDepth , const int rOff[3] , int& xStart , int& xEnd , int& yStart , int& yEnd , int& zStart , int& zEnd ) const;
	int GetMatrixRowSize( const typename TreeOctNode::Neighbors5& neighbors5 , bool symmetric ) const;
	int GetMatrixRowSize( const typename TreeOctNode::Neighbors5& neighbors5 , int xStart , int xEnd , int yStart , int yEnd , int zStart , int zEnd , bool symmetric ) const;
	int SetMatrixRow( const typename TreeOctNode::Neighbors5& neighbors5 , Pointer( MatrixEntry< MatrixReal > ) row , int offset , const typename BSplineData< Degree , Real >::Integrator& integrator , const Stencil< double , 5 >& stencil , bool symmetric ) const;
	int SetMatrixRow( const typename TreeOctNode::Neighbors5& neighbors5 , Pointer( MatrixEntry< MatrixReal > ) row , int offset , const typename BSplineData< Degree , Real >::Integrator& integrator , const Stencil< double , 5 >& stencil , int xStart , int xEnd , int yStart , int yEnd , int zStart , int zEnd , bool symmetric ) const;

	void SetLaplacianStencil ( int depth , const typename BSplineData< Degree , Real >::Integrator& integrator , Stencil< double , 5 >& stencil ) const;
	void SetLaplacianStencils( int depth , const typename BSplineData< Degree , Real >::Integrator& integrator , Stencil< double , 5 > stencil[2][2][2] ) const;
	void SetDivergenceStencil ( int depth , const typename BSplineData< Degree , Real >::Integrator& integrator , Stencil< Point3D< double > , 5 >& stencil , bool scatter ) const;
	void SetDivergenceStencils( int depth , const typename BSplineData< Degree , Real >::Integrator& integrator , Stencil< Point3D< double > , 5 > stencil[2][2][2] , bool scatter ) const;
	void SetCenterEvaluationStencil ( const typename BSplineData< Degree , Real >::template CenterEvaluator< 1 >& evaluator , int depth , Stencil< double , 3 >& stencil ) const;
	void SetCenterEvaluationStencils( const typename BSplineData< Degree , Real >::template CenterEvaluator< 1 >& evaluator , int depth , Stencil< double , 3 > stencil[8] ) const;
	void SetCornerEvaluationStencil ( const typename BSplineData< Degree , Real >::template CornerEvaluator< 2 >& evaluator , int depth , Stencil< double , 3 > stencil [8]    ) const;
	void SetCornerEvaluationStencils( const typename BSplineData< Degree , Real >::template CornerEvaluator< 2 >& evaluator , int depth , Stencil< double , 3 > stencils[8][8] ) const;
	void SetCornerNormalEvaluationStencil ( const typename BSplineData< Degree , Real >::template CornerEvaluator< 2 >& evaluator , int depth , Stencil< Point3D< double > , 5 > stencil [8]    ) const;
	void SetCornerNormalEvaluationStencils( const typename BSplineData< Degree , Real >::template CornerEvaluator< 2 >& evaluator , int depth , Stencil< Point3D< double > , 5 > stencils[8][8] ) const;

	static void UpdateCoarserSupportBounds( const TreeOctNode* node , int& startX , int& endX , int& startY , int& endY , int& startZ , int& endZ );
	void UpdateConstraintsFromCoarser( const typename TreeOctNode::Neighbors5& neighbors5 , const typename TreeOctNode::Neighbors5& pNeighbors5 , TreeOctNode* node , const Real* metSolution , const typename BSplineData< Degree , Real >::Integrator& integrator , const Stencil< double , 5 >& stencil ) const;
	int UpdateConstraintsToCoarser( const typename BSplineData< Degree , Real >::Integrator& integrator , int depth , const SortedTreeNodes< OutputDensity >& sNodes , const Real* fineSolution , Real* coarseConstraints ) const;
	void _UpdateConstraintsToCoarser( const typename TreeOctNode::Neighbors5& neighbors5 , TreeOctNode* node , const Real* fineSolution , Real* coarseConstraints , const typename BSplineData< Degree , Real >::Integrator& integrator , const Stencil< double , 5 >& stencil ) const;
	void SetCoarserPointValues( int depth , const SortedTreeNodes< OutputDensity >& sNodes , Real* metSolution );
	Real WeightedCoarserFunctionValue( const typename TreeOctNode::NeighborKey3& neighborKey3 , const TreeOctNode* node , Real* metSolution ) const;
	void UpSampleCoarserSolution( int depth , const SortedTreeNodes< OutputDensity >& sNodes , Vector< Real >& solution ) const;
	void DownSampleFinerConstraints( int depth , SortedTreeNodes< OutputDensity >& sNodes ) const;
	template< class C > void DownSample( int depth , const SortedTreeNodes< OutputDensity >& sNodes , C* constraints ) const;
	template< class C > void   UpSample( int depth , const SortedTreeNodes< OutputDensity >& sNodes , C* coefficients ) const;
	template< class C > void   UpSample( int depth , const SortedTreeNodes< OutputDensity >& sNodes , const C* coarseCoefficients , C* fineCoefficients ) const;
	int GetFixedDepthLaplacian( SparseSymmetricMatrix< Real >& matrix , int depth , const typename BSplineData< Degree , Real >::Integrator& integrator , const SortedTreeNodes< OutputDensity >& sNodes , const Real* metSolution );
	int GetRestrictedFixedDepthLaplacian( SparseSymmetricMatrix< Real >& matrix , int depth , const typename BSplineData< Degree , Real >::Integrator& integrator , const int* entries , int entryCount , const TreeOctNode* rNode, Real radius , const SortedTreeNodes< OutputDensity >& sNodes , const Real* metSolution );

	void SetIsoCorners( Real isoValue , TreeOctNode* leaf , typename SortedTreeNodes< OutputDensity >::CornerTableData& cData , Pointer( char ) valuesSet , Pointer( Real ) values , typename TreeOctNode::ConstNeighborKey3& nKey , const Real* metSolution , const typename BSplineData< Degree , Real >::template CornerEvaluator< 2 >& evaluator , const Stencil< double , 3 > stencil[8] , const Stencil< double , 3 > stencils[8][8] );
	static int IsBoundaryFace( const TreeOctNode* node , int faceIndex , int subdivideDepth );
	static int IsBoundaryEdge( const TreeOctNode* node , int edgeIndex , int subdivideDepth );
	static int IsBoundaryEdge( const TreeOctNode* node , int dir , int x , int y , int subidivideDepth );

	// For computing the iso-surface there is a lot of re-computation of information across shared geometry.
	// For function values we don't care so much.
	// For edges we need to be careful so that the mesh remains water-tight
	struct RootData : public SortedTreeNodes< OutputDensity >::CornerTableData , public SortedTreeNodes< OutputDensity >::EdgeTableData
	{
		// Edge to iso-vertex map
		hash_map< long long , int > boundaryRoots;
		// Vertex to ( value , normal ) map
		hash_map< long long , std::pair< Real , Point3D< Real > > > *boundaryValues;
		Pointer( int ) interiorRoots;
		Pointer( Real ) cornerValues;
		Pointer( Point3D< Real > ) cornerNormals;
		Pointer( char ) cornerValuesSet;
		Pointer( char ) cornerNormalsSet;
		Pointer( char ) edgesSet;
	};

	template< class Vertex >
	int SetBoundaryMCRootPositions( int sDepth , Real isoValue , RootData& rootData , CoredMeshData< Vertex >* mesh , int nonLinearFit );
	template< class Vertex >
	int SetMCRootPositions( TreeOctNode* node , int sDepth , Real isoValue , typename TreeOctNode::ConstNeighborKey3& neighborKey3 , RootData& rootData ,
		std::vector< Vertex >* interiorVertices , CoredMeshData< Vertex >* mesh , const Real* metSolution , const typename BSplineData< Degree , Real >::template CornerEvaluator< 2 >& evaluator , const Stencil< Point3D< double > , 5 > stencil[8] , const Stencil< Point3D< double > , 5 > stencils[8][8] , int nonLinearFit );
	template< class Vertex >
	int GetMCIsoTriangles( TreeOctNode* node , CoredMeshData< Vertex >* mesh , RootData& rootData ,
		std::vector< Vertex >* interiorVertices , int offSet , int sDepth , bool polygonMesh , std::vector< Vertex >* barycenters );
	template< class Vertex >
	static int AddTriangles( CoredMeshData< Vertex >* mesh , std::vector< CoredPointIndex >& edges , std::vector< Vertex >* interiorVertices , int offSet , bool polygonMesh , std::vector< Vertex >* barycenters );

	void GetMCIsoEdges( TreeOctNode* node , int sDepth , std::vector< std::pair< RootInfo< OutputDensity > , RootInfo< OutputDensity > > >& edges );
	static int GetEdgeLoops( std::vector< std::pair< RootInfo< OutputDensity > , RootInfo< OutputDensity > > >& edges , std::vector< std::vector< std::pair< RootInfo< OutputDensity > , RootInfo< OutputDensity > > > >& loops);
	static int InteriorFaceRootCount( const TreeOctNode* node , const int &faceIndex , int maxDepth );
	static int EdgeRootCount( const TreeOctNode* node , int edgeIndex , int maxDepth );
	static void GetRootSpan( const RootInfo< OutputDensity >& ri , Point3D< Real >& start , Point3D< Real >& end );
	template< class Vertex >
	int GetRoot( const RootInfo< OutputDensity >& ri , Real isoValue , typename TreeOctNode::ConstNeighborKey3& neighborKey3 , Vertex& vertex , RootData& rootData , int sDepth , const Real* metSolution , const typename BSplineData< Degree , Real >::template CornerEvaluator< 2 >& evaluator , const Stencil< Point3D< double > , 5 > nStencil[8] , const Stencil< Point3D< double > , 5 > nStencils[8][8] , int nonLinearFit );
	static int GetRootIndex( const TreeOctNode* node , int edgeIndex , int maxDepth , RootInfo< OutputDensity >& ri );
	static int GetRootIndex( const TreeOctNode* node , int edgeIndex , int maxDepth , int sDepth , RootInfo< OutputDensity >& ri );
	static int GetRootIndex( const RootInfo< OutputDensity >& ri , RootData& rootData , CoredPointIndex& index );
	static int GetRootPair( const RootInfo< OutputDensity >& root , int maxDepth , RootInfo< OutputDensity >& pair );

	int UpdateWeightContribution( TreeOctNode* node , const Point3D<Real>& position , typename TreeOctNode::NeighborKey3& neighborKey , Real weight=Real(1.0) );
	Real GetSampleWeight( const TreeOctNode* node , const Point3D<Real>& position , typename TreeOctNode::ConstNeighborKey3& neighborKey );
	void GetSampleDepthAndWeight( const TreeOctNode* node , const Point3D<Real>& position , typename TreeOctNode::ConstNeighborKey3& neighborKey , Real samplesPerNode , Real& depth , Real& weight );
	Real GetSampleWeight( TreeOctNode* node , const Point3D<Real>& position , typename TreeOctNode::NeighborKey3& neighborKey );
	void GetSampleDepthAndWeight( TreeOctNode* node , const Point3D<Real>& position , typename TreeOctNode::NeighborKey3& neighborKey , Real samplesPerNode , Real& depth , Real& weight );
	int SplatOrientedPoint( TreeOctNode* node , const Point3D<Real>& point , const Point3D<Real>& normal , typename TreeOctNode::NeighborKey3& neighborKey );
	Real SplatOrientedPoint( const Point3D<Real>& point , const Point3D<Real>& normal , typename TreeOctNode::NeighborKey3& neighborKey , int kernelDepth , Real samplesPerNode , int minDepth , int maxDepth );

	int HasNormals(TreeOctNode* node,Real epsilon);
	Real getCenterValue( const typename TreeOctNode::ConstNeighborKey3& neighborKey3 , const TreeOctNode* node , const Real* metSolution , const typename BSplineData< Degree , Real >::template CenterEvaluator< 1 >& evaluator , const Stencil< double , 3 >& stencil , const Stencil< double , 3 >& pStencil , bool isInterior ) const;
	Real getCornerValue( const typename TreeOctNode::ConstNeighborKey3& neighborKey3 , const TreeOctNode* node , int corner , const Real* metSolution , const typename BSplineData< Degree , Real >::template CornerEvaluator< 2 >& evaluator , const Stencil< double , 3 >& stencil , const Stencil< double , 3 > stencils[8] , bool isInterior ) const;
	Point3D< Real > getCornerNormal( const typename TreeOctNode::ConstNeighbors5& neighbors5 , const typename TreeOctNode::ConstNeighbors5& pNeighbors5 , const TreeOctNode* node , int corner , const Real* metSolution , const typename BSplineData< Degree , Real >::template CornerEvaluator< 2 >& evaluator , const Stencil< Point3D< double > , 5 >& nStencil , const Stencil< Point3D< double > , 5 > nStencils[8] , bool isInterior ) const;
	static bool _IsInset( const TreeOctNode* node );
	static bool _IsInsetSupported( const TreeOctNode* node );
public:
	int threads;
	static double maxMemoryUsage;
	static double MemoryUsage( void );
	std::vector< Point3D<Real> >* normals;
	Real postDerivativeSmooth;
	TreeOctNode tree;
	BSplineData< Degree , Real > fData;
	Octree( void );

	void setBSplineData( int maxDepth , int boundaryType=BSplineElements< Degree >::NONE );
	void finalize( int subdivisionDepth );
	int refineBoundary( int subdivisionDepth );
	Pointer( Real ) GetSolutionGrid( int& res , Real isoValue=0.f , int depth=-1 );
	int setTree( char* fileName , int maxDepth , int minDepth , int kernelDepth , Real samplesPerNode ,
		Real scaleFactor , int useConfidence , Real constraintWeight , int adaptiveExponent , XForm4x4< Real > xForm=XForm4x4< Real >::Identity );

	void SetLaplacianConstraints(void);
	void ClipTree(void);
	int LaplacianMatrixIteration( int subdivideDepth , bool showResidual , int minIters , double accuracy , int maxSolveDepth , int fixedIters );

	Real GetIsoValue( void );
	template< class Vertex >
	void GetMCIsoTriangles( Real isoValue , int subdivideDepth , CoredMeshData< Vertex >* mesh , int fullDepthIso=0 , int nonLinearFit=1 , bool addBarycenter=false , bool polygonMesh=false );
};

#include "MultiGridOctreeData.inl"
#endif // MULTI_GRID_OCTREE_DATA_INCLUDED
