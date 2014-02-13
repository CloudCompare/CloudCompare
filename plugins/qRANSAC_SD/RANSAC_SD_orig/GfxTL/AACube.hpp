
namespace GfxTL
{

	template< class Point >
	AACube< Point >::AACube()
	: _width(std::numeric_limits< ScalarType >::infinity())
	{
		for(unsigned int i = 0; i < Dim; ++i)
			_backBottomLeft[i] =
				-std::numeric_limits< ScalarType >::infinity();
	}

	template< class Point >
	AACube< Point >::AACube(const Point &backBottomLeft, ScalarType width)
	: _backBottomLeft(backBottomLeft)
	, _width(width)
	{}

	template< class Point >
	AACube< Point >::AACube(const Point *points, size_t size)
	{
		Bound(points, size);
	}

	template< class Point >
	AACube< Point >::AACube(unsigned int box, const AACube< Point > &cube)
	{
		cube.SubCube(box, this);
	}

	template< class Point >
	void AACube< Point >::DividingPlane(unsigned int axis, Point *n,
		ScalarType *d) const
	{
		for(int i = 0; i < Dim; ++i)
			(*n)[i] = (i == axis)? (ScalarType)-1
				: (ScalarType)0;
		Point center;
		Center(&center);
		*d = center * (*n);
	}

	template< class Point >
	void AACube< Point >::DividingPlane(unsigned int axis, ScalarType *s)
	{
		Point center;
		Center(&center);
		*s = center[axis];
	}

	template< class Point >
	void AACube< Point >::Center(Point *c) const
	{
		ScalarType r = _width / 2;
		*c = _backBottomLeft;
		for(int i = 0; i < Dim; ++i)
			(*c)[i] += r;
	}

	template< class Point >
	void AACube< Point >::Center(const Point &c)
	{
		ScalarType r = _width / 2;
		_backBottomLeft = c;
		for(int i = 0; i < Dim; ++i)
			_backBottomLeft[i] -= r;
	}

	template< class Point >
	void AACube< Point >::SubCube(unsigned int box,
		AACube< Point > *cube) const
	{
		cube->_backBottomLeft = _backBottomLeft;
		ScalarType r = _width / 2;
		for(int i = 0; i < Dim; ++i)
		{
			if(!(box & (1 << i)))
				cube->_backBottomLeft[i] += r;
		}
		cube->_width = r;
	}

	template< class Point >
	bool AACube< Point >::IsSubCube(unsigned int *box,
		const AACube< Point > &cube) const
	{
		ScalarType r = _width / 2;
		if(r != cube._width)
			return false;
		for(unsigned int i = 0; i < NCorners; ++i)
		{
			Point p = _backBottomLeft;
			for(int j = 0; j < Dim; ++j)
			{
				if(!(i & (1 << j)))
					p[j] += r;
			}
			if(p == cube._backBottomLeft)
			{
				*box = i;
				return true;
			}
		}
		return false;
	}

	template< class Point >
	bool AACube< Point >::IsInside(const Point &p) const
	{
		for(unsigned int i = 0; i < Dim; ++i)
			if(p[i] < _backBottomLeft[i] || p[i] > _backBottomLeft[i] + _width)
				return false;
		return true;
	}

	template< class Point >
	typename AACube< Point >::ScalarType
		AACube< Point >::Width() const
	{
		return _width;
	}

	template< class Point >
	void AACube< Point >::Width(ScalarType w)
	{
		_width = w;
	}

	template< class Point >
	const Point &AACube< Point >::LeftBottomBack() const
	{
		return _backBottomLeft;
	}

	template< class Point >
	void AACube< Point >::LeftBottomBack(const Point &lbb)
	{
		_backBottomLeft = lbb;
	}

	template< class Point >
	Point AACube< Point >::operator[](int index) const
	{
		Point p = _backBottomLeft;
		for(int i = 0; i < Dim; ++i)
		{
			if(!(index & (1 << i)))
				p[i] += _width;
		}
		return p;
	}

	template< class Point >
	typename AACube< Point >::ScalarType
		AACube< Point >::DiagLength() const
	{
		return std::sqrt(ScalarType(Dim)) * _width;
	}

	template< class Point >
	void AACube< Point >::Inflate(ScalarType v)
	{
		for(unsigned int i = 0; i < Dim; ++i)
			_backBottomLeft[i] -= v;
		_width += 2 * v;
	}

	template< class Point >
	typename AACube< Point >::ScalarType AACube< Point >::
		Distance(const PointType &x) const
	{
		Point p, cMin, cMax;
		cMin = (*this)[AACube< Point >::NCorners - 1];
		cMax = (*this)[0];
		for(unsigned int i = 0; i < Point::Dim; ++i)
		{
			if(x[i] <= cMin[i])
				p[i] = cMin[i];
			else if(x[i] < cMax[i])
				p[i] = x[i];
			else
				p[i] = cMax[i];
		}
		return (p - x).Length();
	}

	template< class Point >
	typename AACube< Point >::ScalarType AACube< Point >::
		SqrDistance(const PointType &x) const
	{
		Point p, cMin, cMax;
		cMin = (*this)[AACube< Point >::NCorners - 1];
		cMax = (*this)[0];
		for(unsigned int i = 0; i < Point::Dim; ++i)
		{
			if(x[i] <= cMin[i])
				p[i] = cMin[i];
			else if(x[i] < cMax[i])
				p[i] = x[i];
			else
				p[i] = cMax[i];
		}
		return (p - x).SqrLength();
	}

	template< class Point >
	void AACube< Point >::Translate(const PointType &t)
	{
		_backBottomLeft += t;
	}

	template< class Point >
	void AACube< Point >::Scale(ScalarType s)
	{
		_backBottomLeft *= s;
		_width *= s;
	}

};

