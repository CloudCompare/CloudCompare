#ifndef __GfxTL__AAPLANE_HEADER__
#define __GfxTL__AAPLANE_HEADER__

namespace GfxTL
{
	template< class PointT >
	class AAPlane
	{
		public:
			typedef PointT PointType;
			typedef typename PointType::ScalarType ScalarType;

			AAPlane();
			AAPlane(unsigned int axis, ScalarType value);
			AAPlane(unsigned int axis, const PointType &p);
			inline ScalarType operator()(const PointType &p) const;
			void Set(unsigned int axis, ScalarType value);
			inline unsigned int Axis() const;
			inline ScalarType Value() const;
			void Value(ScalarType v);

		private:
			unsigned int _axis;
			ScalarType _value;
	};

	template< class PointT >
	typename AAPlane< PointT >::ScalarType AAPlane< PointT >::operator()(
		const PointType &p) const
	{
		return p[_axis] - _value;
	}

	template< class PointT >
	unsigned int AAPlane< PointT >::Axis() const
	{
		return _axis;
	}

	template< class PointT >
	typename AAPlane< PointT >::ScalarType AAPlane< PointT >::Value() const
	{
		return _value;
	}

	template< class PointT >
	void AAPlane< PointT >::Value(ScalarType v)
	{
		_value = v;
	}
};

#include "AAPlane.hpp"

#endif