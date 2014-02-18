Changes made to PoissonRecon 5.5 <http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version5.5/> for compilation on Mac OS X 10.6:

1) The header file "Time.h" causes problems for GCC because it gets confused with the standard time.h header file, so changed header and source name Time.[h|cpp] to poissonTime.[h|cpp]
   
2) In OctTree.inl, removed 3 instances of extra template specialization like this:
   
   -		const OctNode< NodeData , Real >::Neighbors3& _neighbors = setNeighbors( node->parent );
   +		const Neighbors3& _neighbors = setNeighbors( node->parent );

3) Ply.h included definitions of specialized static members in the header.  This caused duplicate symbols on link, so moved them to a newly created file "Ply.cpp".

--
Andy Maloney
asmaloney@gmail.com
13 December 2013