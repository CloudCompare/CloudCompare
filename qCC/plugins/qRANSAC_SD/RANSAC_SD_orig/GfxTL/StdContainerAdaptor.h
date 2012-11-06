#ifndef __GfxTL_STDCONTAINERADAPTOR_HEADER__
#define __GfxTL_STDCONTAINERADAPTOR_HEADER__
#include <memory>

namespace GfxTL
{
	template< template< class, class > class ContainerT,
		template< class > class AllocatorT = std::allocator >
	struct StdContainerAdaptor
	{
		template< class T >
		class Container
		: public ContainerT< T, AllocatorT< T > >
		{
			public:
				typedef ContainerT< T, std::allocator< T > > ContainerType;
				Container() {}
				Container(typename ContainerType::size_type s)
				: ContainerType(s)
				{}
				Container(typename ContainerType::size_type s,
					const typename ContainerType::value_type &v)
				: ContainerType(s, v)
				{}
		};
	};
};

#endif

