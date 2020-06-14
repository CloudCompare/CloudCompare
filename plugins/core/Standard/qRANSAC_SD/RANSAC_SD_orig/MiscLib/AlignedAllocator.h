#ifndef MiscLib__ALIGNEDALLOCATOR_HEADER__
#define MiscLib__ALIGNEDALLOCATOR_HEADER__
#include <memory>
#ifndef __APPLE__
#include <malloc.h>
#else
#include <stdlib.h>
#endif

#if defined(__x86_64__) || defined(__i386__) || defined(_M_IX86) || defined(_M_X64)
#include <xmmintrin.h>
#endif

#ifdef _mm_malloc
#ifndef a_malloc
#define a_malloc(sz, align) _mm_malloc((sz), (align))
#endif // !a_malloc
#endif // !_mm_malloc
#ifdef _mm_free
#ifndef a_free
#define a_free(ptr)  _mm_free((ptr))
#endif // !a_free
#endif // !_mm_free

#ifndef a_free  
#define a_free(a)      free(a) 
#endif // !_mm_free
#ifndef a_malloc
#ifndef __APPLE__
#define a_malloc(sz, align) aligned_alloc((align), (sz))
#else
#define a_malloc(sz, align) malloc(sz) // OSX aligns all allocations to 16 byte boundaries (except valloc which aligns to page boundaries) - so specific alignment requests are ignored.
#endif
#endif // !_mm_malloc


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
	{ return (T *)a_malloc(s * sizeof(T), Align); }
	void deallocate(pointer p, size_type) { a_free(p); }
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
