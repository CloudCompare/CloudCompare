#ifndef GfxTL__L1NORM_HEADER__
#define GfxTL__L1NORM_HEADER__
#include <GfxTL/MathHelper.h>
#include <GfxTL/ScalarTypeConversion.h>
#include <GfxTL/ScalarTypeDeferer.h>

namespace GfxTL
{
	template< class VectorKernelT >
	struct L1Norm
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
			DistType d = p[0] - v[0], di;
			d = Math< DistType >::Abs(d);
			for(unsigned int i = 1; i < m_dim; ++i)
			{
				di = p[i] - v[i];
				d += Math< DistType >::Abs(di);
			}
			return d;
		}

		template< class PointAT, class PointBT >
		typename DistanceType
		<
			typename ScalarTypeDeferer< PointAT >::ScalarType,
			typename ScalarTypeDeferer< PointBT >::ScalarType
		>::Type
		SqrDistance(const PointAT &p, const PointBT &v) const
		{
			return Distance(p, v);
		}

		template< class ScalarT >
		ScalarT RootOfDistance(ScalarT sqrDistance) const
		{
			return sqrDistance;
		}

		template< class DistScalarT, class DiffScalarT >
		DistScalarT IncrementalBoxSqrDistance(DistScalarT boxSqrDist, DiffScalarT boxDiff,
			DiffScalarT cutDiff) const
		{
			return boxSqrDist + (DistScalarT)cutDiff
				- (DistScalarT)boxDiff;
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
					sqrDist += t;
				}
				else if(a[i] > max[i])
				{
					t = a[i] - max[i];
					sqrDist += t;
				}
			}
			return sqrDist;
		}
	};
};

#endif
