
namespace GfxTL
{
	template< class PointT >
	AAPlane< PointT >::AAPlane()
	{}

	template< class PointT >
	AAPlane< PointT >::AAPlane(unsigned int axis, ScalarType value)
	: _axis(axis)
	, _value(value)
	{
	}

	template< class PointT >
	AAPlane< PointT >::AAPlane(unsigned int axis, const PointType &p)
	: _axis(axis)
	, _value(p[axis])
	{
	}

	template< class PointT >
	void AAPlane< PointT >::Set(unsigned int axis, ScalarType value)
	{
		_axis = axis;
		_value = value;
	}
};
