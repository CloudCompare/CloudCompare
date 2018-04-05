#ifndef GfxTL__ARRAYARITHMETIC_HEADER__
#define GfxTL__ARRAYARITHMETIC_HEADER__
#include <algorithm>

namespace GfxTL
{

// c-style (row major) array layout
template< unsigned int DimT >
class ArrayArithmetic
{
public:
	ArrayArithmetic() {}

	template< class ExtentT >
	ArrayArithmetic(const ExtentT &ext)
	{
		Extent(ext);
	}

	template< class ExtentT >
	void Extent(const ExtentT &ext)
	{
		m_fac[DimT - 1] = 1;
		for(unsigned int i = DimT - 1; i != 0; --i)
			m_fac[i - 1] = m_fac[i] * ext[i];
		m_transFac[0] = 1;
		for(unsigned int i = 0; i < DimT - 1; ++i)
			m_transFac[i + 1] = m_transFac[i] * ext[i];
	}

	template< class SubscriptT >
	size_t Sub2Idx(const SubscriptT &sub) const
	{
		size_t idx = sub[0] * m_fac[0];
		for(unsigned int i = 1; i < DimT; ++i)
			idx += sub[i] * m_fac[i];
		return idx;
	}

	template< class SubscriptT >
	size_t Sub2TransposedIdx(const SubscriptT &sub) const
	{
		size_t idx = sub[0] * m_transFac[0];
		for(unsigned int i = 1; i < DimT; ++i)
			idx += sub[i] * m_transFac[i];
		return idx;
	}

	template< class SubscriptT >
	intptr_t Sub2Off(const SubscriptT &sub) const
	{
		intptr_t off = sub[0] * m_fac[0];
		for(unsigned int i = 1; i < DimT; ++i)
			off += sub[i] * m_fac[i];
		return off;
	}

	template< class SubscriptT >
	void Idx2Sub(size_t idx, SubscriptT *sub) const
	{
		for(unsigned int i = 0; i < DimT; ++i)
		{
			(*sub)[i] = idx / m_fac[i];
			idx %= m_fac[i];
		}
	}

	template< class SubscriptT >
	void TransposedIdx2Sub(size_t tidx, SubscriptT *sub) const
	{
		for(unsigned int i = DimT - 1; i != -1; --i)
		{
			(*sub)[i] = tidx / m_transFac[i];
			tidx %= m_transFac[i];
		}
	}

	// convert to column major index (Fortran and Matlab array layout)
	size_t Idx2TransposedIdx(size_t idx) const
	{
		size_t tidx = 0;
		for(unsigned int i = 0; i < DimT; ++i)
		{
			size_t j = idx / m_fac[i];
			idx %= m_fac[i];
			tidx += j * m_transFac[i];
		}
		return tidx;
	}

	// convert column major index (Fortran and Matlab array layout) to row major (c/c++ array layout)
	size_t TransposedIdx2Idx(size_t tidx) const
	{
		size_t idx = 0;
		for(unsigned int i = DimT - 1; i != -1; --i)
		{
			size_t j = tidx / m_transFac[i];
			tidx %= m_transFac[i];
			idx += j * m_fac[i];
		}
		return idx;
	}

private:
	size_t m_fac[DimT];
	size_t m_transFac[DimT];
};

};

#endif
