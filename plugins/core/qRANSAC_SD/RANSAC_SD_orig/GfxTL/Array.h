#ifndef GfxTL__ARRAY_HEADER__
#define GfxTL__ARRAY_HEADER__
#include <iterator>
#include <iostream>

namespace GfxTL
{

template< unsigned int DimT, class IteratorT >
class ArrayAccessor
{
public:
	ArrayAccessor(IteratorT arr, const size_t *fac)
	: m_arr(arr)
	, m_fac(fac)
	{}

	ArrayAccessor< DimT - 1, IteratorT > operator[](size_t i)
	{
		return ArrayAccessor< DimT - 1, IteratorT >(m_arr + i * (*m_fac), m_fac + 1);
	}

	const ArrayAccessor< DimT - 1, IteratorT > operator[](size_t i) const
	{
		return ArrayAccessor< DimT - 1, IteratorT >(m_arr + i * (*m_fac), m_fac + 1);
	}

	IteratorT begin() const { return m_arr; }
	IteratorT end() const { return m_arr + *(m_fac - 1); }

private:
	IteratorT m_arr;
	const size_t *m_fac;
};

template< class IteratorT >
class ArrayAccessor< 1, IteratorT >
{
public:
	typedef typename std::iterator_traits< IteratorT >::value_type value_type;
	typedef typename std::iterator_traits< IteratorT >::reference reference;

	ArrayAccessor(IteratorT arr, const size_t *fac)
	: m_arr(arr)
	, m_fac(fac)
	{}

	reference operator[](size_t i) const
	{
		return m_arr[i];
	}

	IteratorT begin() const { return m_arr; }
	IteratorT end() const { return m_arr + *(m_fac - 1); }

private:
	IteratorT m_arr;
	const size_t *m_fac;
};

template< unsigned int DimT, class IteratorT >
class Array
{
public:
	typedef typename std::iterator_traits< IteratorT >::value_type value_type;
	typedef IteratorT iterator;
	typedef Array< DimT, IteratorT > ThisType;
	template< unsigned int D >
	friend std::ostream &operator<<(std::ostream &o, const Array< D, IteratorT > &a);

	Array()
	{
		for(unsigned int i = 0; i < DimT; ++i)
			m_ext[i] = 0;
	}

	Array(IteratorT arr, const size_t *ext)
	: m_arr(arr)
	{
		SetExtent(ext);
	}

	void SetExtent(const size_t *ext)
	{
		for(unsigned int i = 0; i < DimT; ++i)
			m_ext[i] = ext[i];
		m_fac[DimT - 1] = 1;
		for(unsigned int i = DimT - 1; i != 0; --i)
			m_fac[i - 1] = m_fac[i] * m_ext[i];
	}

	void Iterator(iterator i) { m_arr = i; }

	ArrayAccessor< DimT - 1, IteratorT > operator[](size_t i)
	{
		return ArrayAccessor< DimT - 1, IteratorT >(m_arr + i * m_fac[0], m_fac + 1);
	}

	const ArrayAccessor< DimT - 1, IteratorT > operator[](size_t i) const
	{
		return ArrayAccessor< DimT - 1, IteratorT >(m_arr + i * m_fac[0], m_fac + 1);
	}

	const size_t Extent(unsigned int d) const
	{
		return m_ext[d];
	}

	const size_t *Extent() const
	{
		return m_ext;
	}

	iterator begin() { return m_arr; }
	iterator end() { return m_arr + m_fac[0] * m_ext[0]; }
	iterator begin() const { return m_arr; }
	iterator end() const { return m_arr + m_fac[0] * m_ext[0]; }

private:
	Array(const ThisType &a)
	{}

	void operator=(const ThisType &a)
	{}

private:
	IteratorT m_arr;
	size_t m_fac[DimT];
	size_t m_ext[DimT];
};

template< unsigned int DimT, class IteratorT >
std::ostream &operator<<(std::ostream &o, const Array< DimT, IteratorT > &a)
{
	o << "[";
	for(size_t i = 0; i < a.Extent(0); ++i)
	{
		o << Array< DimT - 1, IteratorT >(a.m_arr + a.m_fac[0] * i, a.m_ext + 1);
		if(i < a.Extent(0) - 1)
			o << ", ";
	}
	o << "]";
	return o;
}

template< class IteratorT >
std::ostream &operator<<(std::ostream &o, const Array< 1, IteratorT > &a)
{
	o << "[";
	for(size_t i = 0; i < a.Extent(0); ++i)
	{
		o << a.m_arr[i];
		if(i < a.Extent(0) - 1)
			o << ", ";
	}
	o << "]";
	return o;
}

};

#endif
