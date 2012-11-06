#ifndef __GfxTL_NULLSTRATEGY_HEADER__
#define __GfxTL_NULLSTRATEGY_HEADER__

namespace GfxTL
{

	class NullStrategy
	{
		public:
			template< class Point >
			class CellData
			{};

			template< class Base >
			class StrategyBase
			: public Base
			{
				public:
					typedef typename Base::CellType CellType;
//					typedef typename CellType::PointType PointType;
//					typedef typename CellType::ScalarType ScalarType;

					template< class BoundingVolume >
					inline bool RefreshCell(const BoundingVolume &bv,
						CellType *cell);
					template< class ParamT >
					inline void Parameters(const ParamT &);
					void StartPointInfoCache();
					void StopPointInfoCache();
					
				protected:
					bool ShouldSubdivide(const CellType &cell) const;
					void InitCellData(CellType *);
					void InitLevelDependentCellData(CellType *);
			};
	};

	template< class Base >
	 template< class BoundingVolume >
	inline bool NullStrategy::StrategyBase< Base >::
		RefreshCell(const BoundingVolume &, CellType *)
	{
		return false;
	}

	template< class Base >
	 template< class ParamT >
	inline void NullStrategy::StrategyBase< Base >::
		Parameters(const ParamT &)
	{}

};

#include "NullStrategy.hpp"

#endif