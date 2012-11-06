#ifndef GfxTL__NEARESTNEIGHBOR_HEADER__
#define GfxTL__NEARESTNEIGHBOR_HEADER__

namespace GfxTL
{

template< class ScalarT, class IndexT = size_t >
struct NN
{
	typedef ScalarT ScalarType;
	typedef IndexT IndexType;
	ScalarType sqrDist;
	IndexType index;
	NN() {}
	NN(ScalarType d, IndexType i)
	: sqrDist(d)
	, index(i)
	{}
	bool operator<(const NN &nn) const
	{
		return sqrDist < nn.sqrDist;
	}
	bool operator>(const NN &nn) const
	{
		return sqrDist > nn.sqrDist;
	}
	bool operator<=(const NN &nn) const
	{
		return sqrDist <= nn.sqrDist;
	}
	bool operator>=(const NN &nn) const
	{
		return sqrDist >= nn.sqrDist;
	}
	bool operator==(const NN &nn) const
	{
		return sqrDist == nn.sqrDist && index == nn.index;
	}
	bool operator!=(const NN &nn) const
	{
		return sqrDist != nn.sqrDist || index != nn.index;
	}
	operator IndexType() const
	{
		return index;
	}
	operator IndexType &()
	{
		return index;
	}
	operator ScalarType() const
	{
		return sqrDist;
	}
};

};

#endif
