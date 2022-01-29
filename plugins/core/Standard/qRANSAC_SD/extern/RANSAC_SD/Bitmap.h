#ifndef BITMAP_HEADER
#define BITMAP_HEADER
#include <vector>
#include <utility>
#include <GfxTL/VectorXD.h>
#include <vector>

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

DLL_LINKAGE void DilateSquare(const std::vector< char > &bitmap, size_t uextent,
	size_t vextent, bool uwrap, bool vwrap,
	std::vector< char > *dilated);
DLL_LINKAGE void DilateCross(const std::vector< char > &bitmap, size_t uextent,
	size_t vextent, bool uwrap, bool vwrap,
	std::vector< char > *dilated);
DLL_LINKAGE void ErodeSquare(const std::vector< char > &bitmap, size_t uextent,
	size_t vextent, bool uwrap, bool vwrap,
	std::vector< char > *eroded);
DLL_LINKAGE void ErodeCross(const std::vector< char > &bitmap, size_t uextent,
	size_t vextent, bool uwrap, bool vwrap,
	std::vector< char > *eroded);
DLL_LINKAGE void Components(const std::vector< char > &bitmap, size_t uextent,
	size_t vextent, bool uwrap, bool vwrap,
	std::vector< int > *componentsImg,
	std::vector< std::pair< int, size_t > > *labels);
DLL_LINKAGE void PreWrappedComponents(const std::vector< char > &bitmap, size_t uextent,
	size_t vextent, std::vector< int > *componentsImg,
	std::vector< int > *relabelComponentsImg,
	const std::vector< std::pair< int, size_t > > &inLabels,
	std::vector< std::pair< int, size_t > > *labels);
DLL_LINKAGE int Label(int n[], int size, int *curLabel,
	std::vector< std::pair< int, size_t > > *labels);
DLL_LINKAGE void AssociateLabel(int a, int b,
	std::vector< std::pair< int, size_t > > *labels);
DLL_LINKAGE int ReduceLabel(int a, const std::vector<
	std::pair< int, size_t > > &labels);
// finds the loops around a connected component as polygons
DLL_LINKAGE void ComponentLoops(const std::vector< int > &componentImg, size_t uextent,
	size_t vextent, int label, bool uwrap, bool vwrap,
	std::vector< std::vector< GfxTL::VectorXD< 2, size_t > > > *polys);

#endif
