#ifndef BITMAP_HEADER
#define BITMAP_HEADER
#include <MiscLib/Vector.h>
#include <utility>
#include <GfxTL/VectorXD.h>
#include <MiscLib/Vector.h>

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

DLL_LINKAGE void DilateSquare(const MiscLib::Vector< char > &bitmap, size_t uextent,
	size_t vextent, bool uwrap, bool vwrap,
	MiscLib::Vector< char > *dilated);
DLL_LINKAGE void DilateCross(const MiscLib::Vector< char > &bitmap, size_t uextent,
	size_t vextent, bool uwrap, bool vwrap,
	MiscLib::Vector< char > *dilated);
DLL_LINKAGE void ErodeSquare(const MiscLib::Vector< char > &bitmap, size_t uextent,
	size_t vextent, bool uwrap, bool vwrap,
	MiscLib::Vector< char > *eroded);
DLL_LINKAGE void ErodeCross(const MiscLib::Vector< char > &bitmap, size_t uextent,
	size_t vextent, bool uwrap, bool vwrap,
	MiscLib::Vector< char > *eroded);
DLL_LINKAGE void Components(const MiscLib::Vector< char > &bitmap, size_t uextent,
	size_t vextent, bool uwrap, bool vwrap,
	MiscLib::Vector< int > *componentsImg,
	MiscLib::Vector< std::pair< int, size_t > > *labels);
DLL_LINKAGE void PreWrappedComponents(const MiscLib::Vector< char > &bitmap, size_t uextent,
	size_t vextent, MiscLib::Vector< int > *componentsImg,
	MiscLib::Vector< int > *relabelComponentsImg,
	const MiscLib::Vector< std::pair< int, size_t > > &inLabels,
	MiscLib::Vector< std::pair< int, size_t > > *labels);
DLL_LINKAGE int Label(int n[], int size, int *curLabel,
	MiscLib::Vector< std::pair< int, size_t > > *labels);
DLL_LINKAGE void AssociateLabel(int a, int b,
	MiscLib::Vector< std::pair< int, size_t > > *labels);
DLL_LINKAGE int ReduceLabel(int a, const MiscLib::Vector<
	std::pair< int, size_t > > &labels);
// finds the loops around a connected component as polygons
DLL_LINKAGE void ComponentLoops(const MiscLib::Vector< int > &componentImg, size_t uextent,
	size_t vextent, int label, bool uwrap, bool vwrap,
	MiscLib::Vector< MiscLib::Vector< GfxTL::VectorXD< 2, size_t > > > *polys);

#endif
