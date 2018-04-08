Changes made to PoissonRecon 5.71 <http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version5.71/> for compilation on Mac OS X 10.8:

1) The header file "Time.h" causes problems because it gets confused with the standard time.h header file, so changed header and source name Time.[h|cpp] to poissonTime.[h|cpp]

	1a) change in PoissonRecon5.71/CMakeLists.txt
	1b) change include in MultiGridOctreeData.inl

2) Ply.h included definitions of specialized static members in the header.  This caused duplicate symbols on link, so moved them to a newly created file "Ply.cpp".

	2a) Add Ply.cpp to PoissonRecon5.71/CMakeLists.txt

3) Additional scoping and template specification added in the following files:

	- BSplineData.inl
	- Octree.inl
	- SparseMatrix.inl

4) Fixed bug in MultiGridOctreeData.h with missing () and function call

5) Added missing forward declarations in Octree.h

6) Added missing header to Polynomial.inl

--
Andy Maloney
asmaloney@gmail.com
28 April 2014