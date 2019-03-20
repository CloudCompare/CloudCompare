#ifndef GfxTL__MAXNORM_HEADER__
#define GfxTL__MAXNORM_HEADER__
#include <GfxTL/MathHelper.h>
#include <GfxTL/ScalarTypeConversion.h>
#include <GfxTL/ScalarTypeDeferer.h>

namespace GfxTL
{
	template< class VectorKernelT >
	struct MaxNorm
	: public VectorKernelT
	{
		template< class ScalarAT, class ScalarBT >
		struct DistanceType
		{
			typedef typename ScalarTypeConversion< ScalarAT,
				ScalarBT >::DifferenceType Type;
		};

		template< class PointAT, class PointBT >
		typename DistanceType
		<
			typename ScalarTypeDeferer< PointAT >::ScalarType,
			typename ScalarTypeDeferer< PointBT >::ScalarType
		>::Type
		Distance(const PointAT &p, const PointBT &v) const
		{
			typedef typename DistanceType
			<
				typename ScalarTypeDeferer< PointAT >::ScalarType,
				typename ScalarTypeDeferer< PointBT >::ScalarType
			>::Type DistType;
			DistType di, max = Math< DistType >::Abs(p[0] - v[0]);
			for(size_t i = 1; i < m_dim; ++i)
			{
				di = Math< DistType >::Abs(p[i] - v[i]);
				if(di > max)
					max = di;
			}
			return max;
		}

		template< class PointAT, class PointBT >
		typename DistanceType
		<
			typename ScalarTypeDeferer< PointAT >::ScalarType,
			typename ScalarTypeDeferer< PointBT >::ScalarType
		>::Type
		SqrDistance(const PointAT &p, const PointBT &v) const
		{
			typedef typename DistanceType
			<
				typename ScalarTypeDeferer< PointAT >::ScalarType,
				typename ScalarTypeDeferer< PointBT >::ScalarType
			>::Type DistType;
			DistType di, max = Math< DistType >::Abs(p[0] - v[0]);
			for(size_t i = 1; i < m_dim; ++i)
			{
				di = Math< DistType >::Abs(p[i] - v[i]);
				if(di > max)
					max = di;
			}
			return max;
		}

		template< class ScalarT >
		ScalarT RootOfDistance(ScalarT sqrDistance) const
		{
			return sqrDistance;
		}

		template< class DistScalarT, class DiffScalarT >
		DistScalarT IncrementalBoxSqrDistance(DistScalarT boxDist, DiffScalarT boxDiff,
			DiffScalarT cutDiff) const
		{
			return std::max(boxDist, (DistScalarT)Math< DiffScalarT >::Abs(cutDiff));
		}

		template< class PointAT, class PointBT >
		typename DistanceType
		<
			typename ScalarTypeDeferer< PointAT >::ScalarType,
			typename ScalarTypeDeferer< PointBT >::ScalarType
		>::Type
		BoxSqrDistance(const PointAT &a, const PointBT &min,
			const PointBT &max) const
		{
			typename DistanceType
			<
				typename ScalarTypeDeferer< PointAT >::ScalarType,
				typename ScalarTypeDeferer< PointBT >::ScalarType
			>::Type sqrDist = 0, t;
			for(unsigned int i = 0; i < m_dim; ++i)
			{
				if(a[i] < min[i])
				{
					t = min[i] - a[i];
					if(t > sqrDist)
						sqrDist = t;
				}
				else if(a[i] > max[i])
				{
					t = a[i] - max[i];
					if(t > sqrDist)
						sqrDist = t;
				}
			}
			return sqrDist;
		}
	};
};

#endif
