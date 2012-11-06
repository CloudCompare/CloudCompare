#ifndef GfxTL__SWAP_HEADER__
#define GfxTL__SWAP_HEADER__
#include <algorithm>

namespace GfxTL
{

template< class ContainerT, class HandleT >
void Swap(HandleT a, HandleT b, ContainerT *container)
{
	using namespace std;
	swap(container->at(a), container->at(b));
}

};

#endif
