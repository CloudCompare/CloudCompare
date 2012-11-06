#ifndef GfxTL__L2NORM_HEADER__
#define GfxTL__L2NORM_HEADER__
#include <GfxTL/MathHelper.h>
#include <GfxTL/ScalarTypeConversion.h>
#include <GfxTL/ScalarTypeDeferer.h>

namespace GfxTL
{
	struct DynamicMaskElementSize
	{
		public:
			DynamicMaskElementSize() : m_maskElemSize(1) {}

			void MaskElementSize(unsigned int s)
			{
				m_maskElemSize = s;
			}

			const unsigned int MaskElementSize() const
			{
				return m_maskElemSize;
			}

		protected:
			unsigned int m_maskElemSize;
	};

	template< unsigned int SizeT >
	struct FixedMaskElementSize
	{
		public:
			const unsigned int MaskElementSize() const
			{
				return m_maskElemSize;
			}

			void MaskElementSize(unsigned int s)
			{
				assert(s == SizeT);
			}

		protected:
			enum { m_maskElemSize = SizeT };
	};

	template< class VectorKernelT, class MaskElemSizeT = FixedMaskElementSize< 1 > >
	struct L2NormWithMask
	: public VectorKernelT
	, public MaskElemSizeT
	{
		typedef VectorKernelT VectorKernelType;
		typedef MaskElemSizeT MaskElementSizeType;

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
			d *= d;
			for(unsigned int i = 1; i < VectorKernelT::m_dim; ++i)
			{
				di = p[i] - v[i];
				d += di * di;
			}
			return std::sqrt(d);
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
			DistType d = p[0] - v[0], di;
			d *= d;
			for(unsigned int i = 1; i < VectorKernelT::m_dim; ++i)
			{
				di = p[i] - v[i];
				d += di * di;
			}
			return d;
		}

		template< class PointAT, class PointBT >
		typename DistanceType
		<
			typename ScalarTypeDeferer< PointAT >::ScalarType,
			typename ScalarTypeDeferer< PointBT >::ScalarType
		>::Type
		SqrDistance(const PointAT &p, const PointBT &v,
			typename DistanceType
			<
				typename ScalarTypeDeferer< PointAT >::ScalarType,
				typename ScalarTypeDeferer< PointBT >::ScalarType
			>::Type abortDist) const
		{
			typedef typename DistanceType
			<
				typename ScalarTypeDeferer< PointAT >::ScalarType,
				typename ScalarTypeDeferer< PointBT >::ScalarType
			>::Type DistType;
			DistType d = p[0] - v[0], di;
			d *= d;
			for(unsigned int i = 1; i < VectorKernelT::m_dim && d < abortDist; ++i)
			{
				di = p[i] - v[i];
				d += di * di;
			}
			return d;
		}

		template< class PointAT, class PointBT, class MaskAT, class MaskBT >
		typename DistanceType
		<
			typename ScalarTypeDeferer< PointAT >::ScalarType,
			typename ScalarTypeDeferer< PointBT >::ScalarType
		>::Type
		SqrDistance(const PointAT &p, const MaskAT &ma, const PointBT &v, 
			const MaskBT &mb) const
		{
			typedef typename DistanceType
			<
				typename ScalarTypeDeferer< PointAT >::ScalarType,
				typename ScalarTypeDeferer< PointBT >::ScalarType
			>::Type DistType;
			DistType d = 0, dj;
			for(unsigned int i = 0; i < VectorKernelT::m_dim / MaskElemSizeT::m_maskElemSize; ++i)
			{
				if(!(ma[i] && mb[i]))
					continue;
				for(unsigned int j = 0; j < MaskElemSizeT::m_maskElemSize; ++j)
				{
					dj = p[i * MaskElemSizeT::m_maskElemSize + j] - v[i * MaskElemSizeT::m_maskElemSize + j];
					d += dj * dj;
				}
			}
			return d;
		}

		template< class PointAT, class PointBT, class MaskAT, class MaskBT >
		typename DistanceType
		<
			typename ScalarTypeDeferer< PointAT >::ScalarType,
			typename ScalarTypeDeferer< PointBT >::ScalarType
		>::Type
		SqrDistance(const PointAT &p, const MaskAT &ma, const PointBT &v, 
			const MaskBT &mb,
			typename DistanceType
			<
				typename ScalarTypeDeferer< PointAT >::ScalarType,
				typename ScalarTypeDeferer< PointBT >::ScalarType
			>::Type abortDist) const
		{
			typedef typename DistanceType
			<
				typename ScalarTypeDeferer< PointAT >::ScalarType,
				typename ScalarTypeDeferer< PointBT >::ScalarType
			>::Type DistType;
			DistType d = 0, dj;
			for(unsigned int i = 0; i < VectorKernelT::m_dim / MaskElemSizeT::m_maskElemSize && d < abortDist; ++i)
			{
				if(!(ma[i] && mb[i]))
					continue;
				for(unsigned int j = 0; j < MaskElemSizeT::m_maskElemSize && d < abortDist; ++j)
				{
					dj = p[i * MaskElemSizeT::m_maskElemSize + j] - v[i * MaskElemSizeT::m_maskElemSize + j];
					d += dj * dj;
				}
			}
			return d;
		}

		template< class PointAT, class PointBT, class MaskBT >
		typename DistanceType
		<
			typename ScalarTypeDeferer< PointAT >::ScalarType,
			typename ScalarTypeDeferer< PointBT >::ScalarType
		>::Type
		SqrDistance(const PointAT &p, const PointBT &v, 
			const MaskBT &mb) const
		{
			typedef typename DistanceType
			<
				typename ScalarTypeDeferer< PointAT >::ScalarType,
				typename ScalarTypeDeferer< PointBT >::ScalarType
			>::Type DistType;
			DistType d = 0, dj;
			for(unsigned int i = 0; i < VectorKernelT::m_dim / MaskElemSizeT::m_maskElemSize; ++i)
			{
				if(!mb[i])
					continue;
				for(unsigned int j = 0; j < MaskElemSizeT::m_maskElemSize; ++j)
				{
					dj = p[i * MaskElemSizeT::m_maskElemSize + j] - v[i * MaskElemSizeT::m_maskElemSize + j];
					d += dj * dj;
				}
			}
			return d;
		}

		template< class PointAT, class PointBT, class MaskBT >
		typename DistanceType
		<
			typename ScalarTypeDeferer< PointAT >::ScalarType,
			typename ScalarTypeDeferer< PointBT >::ScalarType
		>::Type
		SqrDistance(const PointAT &p, const PointBT &v, 
			const MaskBT &mb,
			typename DistanceType
			<
				typename ScalarTypeDeferer< PointAT >::ScalarType,
				typename ScalarTypeDeferer< PointBT >::ScalarType
			>::Type abortDist) const
		{
			typedef typename DistanceType
			<
				typename ScalarTypeDeferer< PointAT >::ScalarType,
				typename ScalarTypeDeferer< PointBT >::ScalarType
			>::Type DistType;
			DistType d = 0, dj;
			for(unsigned int i = 0; i < VectorKernelT::m_dim / MaskElemSizeT::m_maskElemSize && d < abortDist; ++i)
			{
				if(!mb[i])
					continue;
				for(unsigned int j = 0; j < MaskElemSizeT::m_maskElemSize && d < abortDist; ++j)
				{
					dj = p[i * MaskElemSizeT::m_maskElemSize + j] - v[i * MaskElemSizeT::m_maskElemSize + j];
					d += dj * dj;
				}
			}
			return d;
		}

		template< class PointAT, class PointBT, class MaskAT, class MaskBT >
		typename DistanceType
		<
			typename ScalarTypeDeferer< PointAT >::ScalarType,
			typename ScalarTypeDeferer< PointBT >::ScalarType
		>::Type
		Distance(const PointAT &p, const PointBT &v, const MaskAT &ma,
			const MaskBT &mb) const
		{
			return std::sqrt(SqrDistance(p, v, ma, mb));
		}

		template< class ScalarT >
		ScalarT RootOfDistance(ScalarT sqrDistance) const
		{
			return std::sqrt(sqrDistance);
		}

		template< class DistScalarT, class DiffScalarT >
		DistScalarT IncrementalBoxSqrDistance(DistScalarT boxSqrDist, DiffScalarT boxDiff,
			DiffScalarT cutDiff) const
		{
			return boxSqrDist - ((DistScalarT)boxDiff * (DistScalarT)boxDiff)
				+ ((DistScalarT)cutDiff * (DistScalarT)cutDiff);
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
			for(unsigned int i = 0; i < VectorKernelT::m_dim; ++i)
			{
				if(a[i] < min[i])
				{
					t = min[i] - a[i];
					sqrDist += t * t;
				}
				else if(a[i] > max[i])
				{
					t = a[i] - max[i];
					sqrDist += t * t;
				}
			}
			return sqrDist;
		}

		template< class PointAT, class PointBT >
		typename DistanceType
		<
			typename ScalarTypeDeferer< PointAT >::ScalarType,
			typename ScalarTypeDeferer< PointBT >::ScalarType
		>::Type
		MaxBoxSqrDistance(const PointAT &a, const PointBT &min,
			const PointBT &max) const
		{
			typedef typename DistanceType
			<
				typename ScalarTypeDeferer< PointAT >::ScalarType,
				typename ScalarTypeDeferer< PointBT >::ScalarType
			>::Type DistType;
			DistType sqrDist = 0, t;
			for(unsigned int i = 0; i < VectorKernelT::m_dim; ++i)
				sqrDist += (t = std::max(Math< DistType >::Abs(min[i] - a[i]),
					Math< DistType >::Abs(max[i] - a[i]))) * t;
			return sqrDist;
		}

		template< class PointAT, class MaskAT, class PointBT >
		typename DistanceType
		<
			typename ScalarTypeDeferer< PointAT >::ScalarType,
			typename ScalarTypeDeferer< PointBT >::ScalarType
		>::Type
		BoxSqrDistance(const PointAT &a, const MaskAT &mask, const PointBT &min,
			const PointBT &max) const
		{
			typename DistanceType
			<
				typename ScalarTypeDeferer< PointAT >::ScalarType,
				typename ScalarTypeDeferer< PointBT >::ScalarType
			>::Type sqrDist = 0, t;
			for(unsigned int i = 0; i < VectorKernelT::m_dim / MaskElemSizeT::m_maskElemSize; ++i)
			{
				if(!mask[i]) continue;
				for(unsigned int j = 0; j < MaskElemSizeT::m_maskElemSize; ++j)
				{
					if(min[i * MaskElemSizeT::m_maskElemSize + j] > max[i * MaskElemSizeT::m_maskElemSize + j]) continue;
					if(a[i * MaskElemSizeT::m_maskElemSize + j] < min[i * MaskElemSizeT::m_maskElemSize + j])
					{
						t = min[i * MaskElemSizeT::m_maskElemSize + j] - a[i * MaskElemSizeT::m_maskElemSize + j];
						sqrDist += t * t;
					}
					else if(a[i * MaskElemSizeT::m_maskElemSize + j] > max[i * MaskElemSizeT::m_maskElemSize + j])
					{
						t = a[i * MaskElemSizeT::m_maskElemSize + j] - max[i * MaskElemSizeT::m_maskElemSize + j];
						sqrDist += t * t;
					}
				}
			}
			return sqrDist;
		}

		template< class PointAT, class MaskAT, class PointBT >
		typename DistanceType
		<
			typename ScalarTypeDeferer< PointAT >::ScalarType,
			typename ScalarTypeDeferer< PointBT >::ScalarType
		>::Type
		MaxBoxSqrDistance(const PointAT &a, const MaskAT &mask,
			const PointBT &min, const PointBT &max) const
		{
			typedef typename DistanceType
			<
				typename ScalarTypeDeferer< PointAT >::ScalarType,
				typename ScalarTypeDeferer< PointBT >::ScalarType
			>::Type DistType;
			DistType sqrDist = 0, t;
			for(unsigned int i = 0; i < VectorKernelT::m_dim / MaskElemSizeT::m_maskElemSize; ++i)
			{
				if(!mask[i]) continue;
				for(unsigned int j = 0; j < MaskElemSizeT::m_maskElemSize; ++j)
				{
					if(min[i * MaskElemSizeT::m_maskElemSize + j] > max[i * MaskElemSizeT::m_maskElemSize + j]) continue;
					sqrDist += (t = std::max(Math< DistType >::Abs(min[i * MaskElemSizeT::m_maskElemSize + j] - a[i * MaskElemSizeT::m_maskElemSize + j]),
						Math< DistType >::Abs(max[i * MaskElemSizeT::m_maskElemSize + j] - a[i * MaskElemSizeT::m_maskElemSize + j]))) * t;
				}
			}
			return sqrDist;
		}

		template< class PointAT, class PointBT, class WidthT >
		typename DistanceType
		<
			typename ScalarTypeDeferer< PointAT >::ScalarType,
			WidthT
		>::Type
		AACubeSqrDistance(const PointAT &a, const PointBT &min,
			WidthT width) const
		{
			typename DistanceType
			<
				typename ScalarTypeDeferer< PointAT >::ScalarType,
				WidthT
			>::Type sqrDist = 0, t;
			for(unsigned int i = 0; i < VectorKernelT::m_dim; ++i)
			{
				if(min[i] > a[i])
				{
					t = min[i] - a[i];
					sqrDist += t * t;
				}
				else if(a[i] > min[i] + width)
				{
					t = a[i] - (min[i] + width);
					sqrDist += t * t;
				}
				//t = min[i] - a[i];
				//if(t > 0)
				//	sqrDist += t * t;
				//else if(t < -width)
				//{
				//	t += width;
				//	sqrDist += t * t;
				//}
			}
			return sqrDist;
		}

		template< class PointAT, class PointBT >
		void AssignAsAABoxMin(const PointAT &vec, PointBT *bboxMin) const
		{
			for(unsigned int i = 0; i < VectorKernelT::m_dim; ++i)
				(*bboxMin)[i] = vec[i];
		}

		template< class PointAT, class PointBT >
		void AssignAsAABoxMax(const PointAT &vec, PointBT *bboxMax) const
		{
			for(unsigned int i = 0; i < VectorKernelT::m_dim; ++i)
				(*bboxMax)[i] = vec[i];
		}

		template< class PointAT, class MaskAT, class PointBT >
		void AssignAsAABoxMinMaskInfinity(const PointAT &vec, const MaskAT &mask,
			PointBT *bboxMin) const
		{
			typedef typename ScalarTypeDeferer< PointBT >::ScalarType ScalarType;
			for(unsigned int i = 0; i < VectorKernelT::m_dim / MaskElemSizeT::m_maskElemSize; ++i)
				if(mask[i])
					for(unsigned int j = 0; j < MaskElemSizeT::m_maskElemSize; ++j)
						(*bboxMin)[i * MaskElemSizeT::m_maskElemSize + j] = vec[i * MaskElemSizeT::m_maskElemSize + j];
				else
					for(unsigned int j = 0; j < MaskElemSizeT::m_maskElemSize; ++j)
						(*bboxMin)[i * MaskElemSizeT::m_maskElemSize + j] = -std::numeric_limits< ScalarType >::infinity();
		}

		template< class PointAT, class MaskAT, class PointBT >
		void AssignAsAABoxMinMaskIgnore(const PointAT &vec, const MaskAT &mask,
			PointBT *bboxMin) const
		{
			typedef typename ScalarTypeDeferer< PointBT >::ScalarType ScalarType;
			for(unsigned int i = 0; i < VectorKernelT::m_dim / MaskElemSizeT::m_maskElemSize; ++i)
				if(mask[i])
					for(unsigned int j = 0; j < MaskElemSizeT::m_maskElemSize; ++j)
						(*bboxMin)[i * MaskElemSizeT::m_maskElemSize + j] = vec[i * MaskElemSizeT::m_maskElemSize + j];
				else
					for(unsigned int j = 0; j < MaskElemSizeT::m_maskElemSize; ++j)
						(*bboxMin)[i * MaskElemSizeT::m_maskElemSize + j] = std::numeric_limits< ScalarType >::infinity();
		}

		template< class PointAT, class MaskAT, class PointBT >
		void AssignAsAABoxMaxMaskInfinity(const PointAT &vec, const MaskAT &mask,
			PointBT *bboxMax) const
		{
			typedef typename ScalarTypeDeferer< PointBT >::ScalarType ScalarType;
			for(unsigned int i = 0; i < VectorKernelT::m_dim / MaskElemSizeT::m_maskElemSize; ++i)
				if(mask[i])
					for(unsigned int j = 0; j < MaskElemSizeT::m_maskElemSize; ++j)
						(*bboxMax)[i * MaskElemSizeT::m_maskElemSize + j] = vec[i * MaskElemSizeT::m_maskElemSize + j];
				else
					for(unsigned int j = 0; j < MaskElemSizeT::m_maskElemSize; ++j)
						(*bboxMax)[i * MaskElemSizeT::m_maskElemSize + j] = std::numeric_limits< ScalarType >::infinity();
		}

		template< class PointAT, class MaskAT, class PointBT >
		void AssignAsAABoxMaxMaskIgnore(const PointAT &vec, const MaskAT &mask,
			PointBT *bboxMax) const
		{
			typedef typename ScalarTypeDeferer< PointBT >::ScalarType ScalarType;
			for(unsigned int i = 0; i < VectorKernelT::m_dim / MaskElemSizeT::m_maskElemSize; ++i)
				if(mask[i])
					for(unsigned int j = 0; j < MaskElemSizeT::m_maskElemSize; ++j)
						(*bboxMax)[i * MaskElemSizeT::m_maskElemSize + j] = vec[i * MaskElemSizeT::m_maskElemSize + j];
				else
					for(unsigned int j = 0; j < MaskElemSizeT::m_maskElemSize; ++j)
						(*bboxMax)[i * MaskElemSizeT::m_maskElemSize + j] = -std::numeric_limits< ScalarType >::infinity();
		}

		template< class BoxPointT >
		void InitMaximalAABox(BoxPointT box[2]) const
		{
			typedef typename ScalarTypeDeferer< BoxPointT >::ScalarType
				ScalarType;
			for(unsigned int i = 0; i < VectorKernelT::m_dim; ++i)
			{
				box[0][i] = std::numeric_limits< ScalarType >::min();
				box[1][i] = std::numeric_limits< ScalarType >::max();
			}
		}

		template< class PointT, class BoxPointT >
		void IncludeInAABox(const PointT &p, BoxPointT box[2]) const
		{
			for(unsigned int i = 0; i < VectorKernelT::m_dim; ++i)
			{
				if(box[0][i] > p[i])
					box[0][i] = p[i];
				else if(box[1][i] < p[i])
					box[1][i] = p[i];
			}
		}

		template< class PointT, class MaskT, class BoxPointT >
		void IncludeInAABoxMaskInfinity(const PointT &p, const MaskT &mask, BoxPointT box[2]) const
		{
			typedef typename ScalarTypeDeferer< BoxPointT >::ScalarType
				ScalarType;
			for(unsigned int i = 0; i < VectorKernelT::m_dim / MaskElemSizeT::m_maskElemSize; ++i)
			{
				if(mask[i])
					for(unsigned int j = 0; j < MaskElemSizeT::m_maskElemSize; ++j)
					{
						if(box[0][i * MaskElemSizeT::m_maskElemSize + j] > p[i * MaskElemSizeT::m_maskElemSize + j])
							box[0][i * MaskElemSizeT::m_maskElemSize + j] = p[i * MaskElemSizeT::m_maskElemSize + j];
						else if(box[1][i * MaskElemSizeT::m_maskElemSize + j] < p[i * MaskElemSizeT::m_maskElemSize + j])
							box[1][i * MaskElemSizeT::m_maskElemSize + j] = p[i * MaskElemSizeT::m_maskElemSize + j];
					}
				else
					for(unsigned int j = 0; j < MaskElemSizeT::m_maskElemSize; ++j)
					{
						box[0][i * MaskElemSizeT::m_maskElemSize + j] = -std::numeric_limits< ScalarType >::infinity();
						box[1][i * MaskElemSizeT::m_maskElemSize + j] = std::numeric_limits< ScalarType >::infinity();
					}
			}
		}

		template< class PointT, class MaskT, class BoxPointT >
		void IncludeInAABoxMaskIgnore(const PointT &p, const MaskT &mask, BoxPointT box[2]) const
		{
			typedef typename ScalarTypeDeferer< BoxPointT >::ScalarType
				ScalarType;
			for(unsigned int i = 0; i < VectorKernelT::m_dim / MaskElemSizeT::m_maskElemSize; ++i)
				if(mask[i])
					for(unsigned int j = 0; j < MaskElemSizeT::m_maskElemSize; ++j)
					{
						if(box[0][i * MaskElemSizeT::m_maskElemSize + j] > p[i * MaskElemSizeT::m_maskElemSize + j])
							box[0][i * MaskElemSizeT::m_maskElemSize + j] = p[i * MaskElemSizeT::m_maskElemSize + j];
						else if(box[1][i * MaskElemSizeT::m_maskElemSize + j] < p[i * MaskElemSizeT::m_maskElemSize + j])
							box[1][i * MaskElemSizeT::m_maskElemSize + j] = p[i * MaskElemSizeT::m_maskElemSize + j];
					}
		}
	};

	template< class VectorKernelT >
	struct L2Norm
	: public L2NormWithMask< VectorKernelT, FixedMaskElementSize< 1 > >
	{};

	template< class MaskElemSizeT >
	struct MakeL2Norm
	{
		template< class VectorKernelT >
		struct L2NormType
		: public L2NormWithMask< VectorKernelT, MaskElemSizeT >
		{};
	};
};

#endif
