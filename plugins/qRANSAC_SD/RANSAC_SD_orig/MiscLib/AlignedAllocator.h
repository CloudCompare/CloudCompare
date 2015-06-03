#ifndef MiscLib__ALIGNEDALLOCATOR_HEADER__
#define MiscLib__ALIGNEDALLOCATOR_HEADER__
#include <memory>
#ifndef __APPLE__
#include <malloc.h>
#else
#include <stdlib.h>
#endif
#include <xmmintrin.h>
#include <limits>
#ifdef max
#undef max
#endif

namespace MiscLib
{

enum { DefaultAlignment = sizeof(size_t) };

template< class T, unsigned int Align = DefaultAlignment >
class AlignedAllocator
{
public:
	typedef size_t size_type;
	typedef ptrdiff_t difference_type;
	typedef T *pointer;
	typedef const T *const_pointer;
	typedef T &reference;
	typedef const T &const_reference;
	typedef T value_type;
	template< class U >
	struct rebind { typedef AlignedAllocator< U, Align > other; };

	AlignedAllocator() throw() {}
	AlignedAllocator(const AlignedAllocator< T, Align > &) throw() {}
	template< class U >
	AlignedAllocator(const AlignedAllocator< U, Align > &) throw() {}
	pointer address(reference x) const { return &x; }
	const_pointer address(const_reference x) const { return &x; }
	pointer allocate(size_type s, std::allocator< void >::const_pointer hint = 0)
	{ return (T *)_mm_malloc(s * sizeof(T), Align); }
	void deallocate(pointer p, size_type) { _mm_free(p); }
	size_type max_size() const throw() { return std::numeric_limits< size_type >::max(); }
	void construct(pointer p, const T& val) { new(static_cast< void * >(p)) T(val); }
	void destroy(pointer p) { p->~T(); }
	template< class U >
	bool operator==(const AlignedAllocator< U, Align > &) const { return true; }
	template< class U >
	bool operator==(const U &) const { return false; }
	template< class U >
	bool operator!=(const AlignedAllocator< U, Align > &) const { return false; }
	template< class U >
	bool operator!=(const U &) const { return true; }
};

template< unsigned int Align = sizeof(size_t) >
struct MakeFixedAlignedAllocator
{
	template< class T >
	struct AllocType : public AlignedAllocator< T, Align > {};
};

};

#endif
