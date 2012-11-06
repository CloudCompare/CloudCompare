
namespace GfxTL
{
	template< class PointT >
	Plane< PointT >::Plane()
	{}

	template< class PointT >
	Plane< PointT >::Plane(const PointType &normal, const PointType &origin)
	: _normal(normal)
	{
		_d = -(_normal * origin);
	}

	template< class PointT >
	void Plane< PointT >::Set(const PointType &origin, const PointType &normal)
	{
		_normal = normal;
		_d = -(_normal * origin);
	}

	template< class PointT >
	typename Plane< PointT >::ScalarType Plane< PointT >::SignedDistance(
		const PointType &p) const
	{
		return _normal * p + _d;
	}

	template< class PointT >
	void Plane< PointT >::Orient(const PointType &n)
	{
		ScalarType factor = Math< ScalarType >::Sign(_normal * n);
		_normal *= factor;
		_d *= factor;
	}

	template< class PointT >
	void Plane< PointT >::Project(const PointType &p, PointType *s) const
	{
		*s = p - SignedDistance(p) * _normal;
	}

	template< class PointT >
	void Plane< PointT >::Normal(PointType *normal) const
	{
		*normal = _normal;
	}

	template< class PointT >
	void Plane< PointT >::Normal(const PointType &normal)
	{
		_normal = normal;
	}

	template< class PointT >
	typename Plane< PointT >::PointType &Plane< PointT >::Normal()
	{
		return _normal;
	}

	template< class PointT >
	const typename Plane< PointT >::PointType &Plane< PointT >::Normal() const
	{
		return _normal;
	}

	template< class PointT >
	void Plane< PointT >::Origin(const PointType &o)
	{
		_d = -(_normal * o);
	}

	template< class PointT >
	typename Plane< PointT >::ScalarType Plane< PointT >::Intersect(
		const PointType &p, const PointType &r) const
	{
		return -SignedDistance(p) / (_normal * r);
	}
};
