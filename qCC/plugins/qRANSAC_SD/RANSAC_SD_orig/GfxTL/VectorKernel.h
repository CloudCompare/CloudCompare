#ifndef GfxTL__VECTORKERNEL_HEADER__
#define GfxTL__VECTORKERNEL_HEADER__

#include <GfxTL/ScalarTypeConversion.h>
#include <GfxTL/ScalarTypeDeferer.h>
#include <GfxTL/VectorXD.h>
#include <GfxTL/NullClass.h>

namespace GfxTL
{
	struct DynamicDimensionVectorKernelStrategy
	{
		public:
			void Dimension(unsigned int dim)
			{
				m_dim = dim;
			}

			const unsigned int Dimension() const
			{
				return m_dim;
			}

		protected:
			unsigned int m_dim;
	};

	template< unsigned int DimT >
	struct FixedDimensionVectorKernelStrategy
	{
		public:
			const unsigned int Dimension() const
			{
				return m_dim;
			}

		protected:
			enum { m_dim = DimT };
	};

	template< class DimensionStrategyT, class BaseT = NullClass >
	struct VectorKernel
	: public BaseT
	, public DimensionStrategyT
	{
		typedef DimensionStrategyT DimensionStrategyType;

		template< class PointAT, class PointBT >
		void AssignVector(const PointAT &a, const PointBT *b) const
		{
			for(unsigned int i = 0; i < DimensionStrategyT::m_dim; ++i)
				(*b)[i] = a[i];
		}

		template< class PointAT, class PointBT, class PointDT >
		void Add(const PointAT &a, const PointBT &b, PointDT *d) const
		{
			for(unsigned int i = 0; i < DimensionStrategyT::m_dim; ++i)
				(*d)[i] = a[i] + b[i];
		}

		template< class PointAT, class PointBT, class PointDT >
		void Sub(const PointAT &a, const PointBT &b, PointDT *d) const
		{
			for(unsigned int i = 0; i < DimensionStrategyT::m_dim; ++i)
				(*d)[i] = a[i] - b[i];
		}

		template< class PointAT, class PointBT >
		typename ScalarTypeConversion
		<
			typename ScalarTypeConversion
			<
				typename ScalarTypeDeferer< PointAT >::ScalarType,
				typename ScalarTypeDeferer< PointBT >::ScalarType
			>::MultiplicationType,
			typename ScalarTypeConversion
			<
				typename ScalarTypeDeferer< PointAT >::ScalarType,
				typename ScalarTypeDeferer< PointBT >::ScalarType
			>::MultiplicationType
		>::AdditionType Dot(const PointAT &a, const PointBT &b) const
		{
			typename ScalarTypeConversion
			<
				typename ScalarTypeConversion
				<
					typename ScalarTypeDeferer< PointAT >::ScalarType,
					typename ScalarTypeDeferer< PointBT >::ScalarType
				>::MultiplicationType,
				typename ScalarTypeConversion
				<
					typename ScalarTypeDeferer< PointAT >::ScalarType,
					typename ScalarTypeDeferer< PointBT >::ScalarType
				>::MultiplicationType
			>::AdditionType d = a[0] * b[0];
			for(unsigned int i = 1; i < DimensionStrategyT::m_dim; ++i)
				d += a[i] * b[i];
			return d;
		}

		template< class PointAT, class PointBT >
		void Mul(typename ScalarTypeDeferer< PointAT >::ScalarType s, const PointAT &a,
			PointBT *b) const
		{
			for(unsigned int i = 0; i < DimensionStrategyT::m_dim; ++i)
				(*b)[i] = s * a[i];
		}
	};

	template< class BaseT = NullClass >
	struct DynVectorKernel
	: public VectorKernel< DynamicDimensionVectorKernelStrategy, BaseT >
	{};

	template< unsigned int DimT >
	struct VectorKernelD
	{
		template< class BaseT = NullClass >
		struct VectorKernelType
		: public VectorKernel< FixedDimensionVectorKernelStrategy< DimT >, BaseT >
		{};
	};
};

#endif
