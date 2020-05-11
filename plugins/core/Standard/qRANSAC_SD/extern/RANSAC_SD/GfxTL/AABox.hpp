
namespace GfxTL
{
	template< class Point >
	AABox< Point >::AABox(const PointType &pmin, const PointType &pmax)
	: _pmin(pmin)
	, _pmax(pmax)
	{}

	template< class Point >
	void AABox< Point >::Infinite()
	{
		for(unsigned int i = 0; i < Dim; ++i)
		{
			_pmin[i] = -std::numeric_limits< ScalarType >::infinity();
			_pmax[i] = std::numeric_limits< ScalarType >::infinity();
		}
	}

	template< class Point >
	void AABox< Point >::Split(unsigned int axis, ScalarType s,
		AABox< Point > *left, AABox< Point > *right) const
	{
		*left = *right = *this;
		if(_pmax[axis] > s)
			left->_pmax[axis] = s;
		if(_pmin[axis] < s)
			right->_pmin[axis] = s;
	}

	template< class Point >
	void AABox< Point >::Center(Point *center) const
	{
		*center = _pmin;
		*center += _pmax;
		*center /= 2;
	}

	template< class Point >
	Point AABox< Point >::operator[](unsigned int index) const
	{
		Point p;
		for(unsigned int i = 0; i < Dim; ++i)
		{
			if(index & (1 << i))
				p[i] = _pmin[i];
			else
				p[i] = _pmax[i];
		}
		return p;
	}

	template< class Point >
	Point &AABox< Point >::Min()
	{
		return _pmin;
	}

	template< class Point >
	const Point &AABox< Point >::Min() const
	{
		return _pmin;
	}

	template< class Point >
	Point &AABox< Point >::Max()
	{
		return _pmax;
	}

	template< class Point >
	const Point &AABox< Point >::Max() const
	{
		return _pmax;
	}

	template< class Point >
	typename AABox< Point >::ScalarType AABox< Point >::DiagLength() const
	{
		return (_pmax - _pmin).Length();
	}

	template< class Point >
	bool AABox< Point >::IsInside(const Point &p) const
	{
		for(unsigned int i = 0; i < Dim; ++i)
			if(_pmin[i] > p[i] || _pmax[i] < p[i])
				return false;
		return true;
	}

	template< class Point >
	AABox< Point > &AABox< Point >::operator+=(const PointType &p)
	{
		for(unsigned int i = 0; i < Dim; ++i)
		{
			if(_pmin[i] > p[i])
				_pmin[i] = p[i];
			if(_pmax[i] < p[i])
				_pmax[i] = p[i];
		}
		return *this;
	}
};

