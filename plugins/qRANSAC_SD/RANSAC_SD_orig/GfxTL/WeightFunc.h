#ifndef __GfxTL_WEIGHTFUNC_HEADER__
#define __GfxTL_WEIGHTFUNC_HEADER__
#include <iterator>
#include <GfxTL/MathHelper.h>

namespace GfxTL
{

	template< class ScalarT >
	class BSplineWeightFunc
	{
	public:
		typedef ScalarT ScalarType;

		BSplineWeightFunc(ScalarType r)
		{
			_r = r;
			_f = ScalarType(1.5) / r;
		}

		ScalarType operator()(ScalarType dist) const
		{
			if(dist >= _r)
				return 0;
			return EvalBSpline(_f * dist);
		}

	private:
		ScalarType EvalBSpline(ScalarType t) const
		{
			//B-spline (degree = 2)
			if(t < ScalarType(0.5))
				return -t * t + ScalarType(0.75);
			else
				return ScalarType(0.5) * (ScalarType(1.5) - t)
					* (ScalarType(1.5) - t);
			/*ScalarType x = t + (ScalarType)1.5;	// shifted value
			if(((ScalarType)-1.5 <= t) && (t < (ScalarType)-0.5))
				return  (ScalarType)0.5 * x * x;
			if(((ScalarType)-0.5 <= t) && (t < (ScalarType)0.5))
				return (ScalarType)-1.0 * x * x + (ScalarType)3.0 * x
					- (ScalarType)(3.0 / 2.0);
			if(((ScalarType)0.5 <= t) && (t < (ScalarType)1.5))
				return (ScalarType)0.5 * x * x - (ScalarType)3.0 * x
					+ (ScalarType)(9.0 / 2.0);
			return (ScalarType)0.0;	// bspline zero*/
		}

	private:
		ScalarType _f;
		ScalarType _r;
	};

	template< class Point >
	class UnitWeightFunc
	{
		public:
			typedef Point PointType;
			typedef typename Point::ScalarType ScalarType;

			double Weight(const Point &)
			{
				return (double)1;
			}

			double operator()(const Point &)
			{
				return (double)1;
			}
	};

	class UnitWeightIterator
	{
		public:
			typedef unsigned int value_type;
			typedef unsigned int *pointer;
			typedef unsigned int &reference;
			typedef std::forward_iterator_tag iterator_category;
			typedef size_t difference_type;

			const unsigned int operator*() const
			{
				return 1;
			}

			UnitWeightIterator operator++() const
			{
				return *this;
			}
	};

	template< class PointT >
	class InverseDistanceSingularWeightFunc
	{
		public:
			typedef PointT PointType;
			typedef typename PointType::ScalarType ScalarType;

			InverseDistanceSingularWeightFunc(const PointType &center,
				ScalarType radius)
			: _center(center)
			, _radius(radius)
			{}

			ScalarType operator()(const PointType &p) const
			{
				ScalarType d = (p - _center).Length();
				return operator()(d);
			}

			ScalarType operator()(ScalarType d) const
			{
				ScalarType a = Math< ScalarType >::Max(_radius - d, 0)
					/ (_radius * d);
				return a * a;
			}

		private:
			PointType _center;
			ScalarType _radius;
	};

	template< class PointT >
	class InterpolatingExponentialWeightFunc
	{
		public:
			typedef PointT PointType;
			typedef typename PointType::ScalarType ScalarType;

			InterpolatingExponentialWeightFunc(const PointType &center,
				ScalarType sqrRadius)
			: _center(center)
			{
				_scale = ScalarType(11.38335808) / sqrRadius;
			}

			InterpolatingExponentialWeightFunc()
			{
				_center.Zero();
				_scale = 1;
			}

			ScalarType operator()(const PointType &p) const
			{
				return operator()(_center.SqrDistance(p));
			}

			ScalarType operator()(ScalarType sqrDist) const
			{
				sqrDist *= _scale;
				return std::exp(-sqrDist) / sqrDist;
			}

			bool WeightAndDerivative(const PointType &x, ScalarType *weight,
				PointType *derivative)
			{
				ScalarType sqrDist = _scale * _center.SqrDistance(x);
				if(sqrDist < 1.0e-6)
					return false;
				*weight = std::exp(-sqrDist) / sqrDist;
				for(unsigned int i = 0; i < PointType::Dim; ++i)
				{
					(*derivative)[i] = *weight * (2 * (x[i] - _center[i]));
					(*derivative)[i] += (*derivative)[i] / sqrDist;
				}
				return true;
			}

		private:
			PointType _center;
			ScalarType _scale;
	};

	template< class PointT >
	class CubicGaussApproximationWeightFunc
	{
		public:
			typedef PointT PointType;
			typedef typename PointType::ScalarType ScalarType;

			CubicGaussApproximationWeightFunc(ScalarType radius)
			: _radius(radius)
			, _sqrRadius(_radius * _radius)
			, _cbcRadius(_sqrRadius * _radius)
			{}

			CubicGaussApproximationWeightFunc(PointType center,
				ScalarType radius)
			: _center(center)
			, _radius(radius)
			, _sqrRadius(_radius * _radius)
			, _cbcRadius(_sqrRadius * _radius)
			{}

			ScalarType operator()(const PointType &p) const
			{
				return operator()((p - _center).Length());
			}

			ScalarType operator()(ScalarType dist) const
			{
				ScalarType sqrDist = dist * dist;
				return 2 * sqrDist * dist / _cbcRadius -
					3 * sqrDist / _sqrRadius + 1;
			}

			bool WeightAndDerivative(const PointType &x, ScalarType *weight,
				PointType *derivative)
			{
				return false;
			}

		private:
			ScalarType _radius, _sqrRadius, _cbcRadius;
			PointType _center;
	};
};

#endif
