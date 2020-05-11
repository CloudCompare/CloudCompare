#ifndef GfxTL__NULLTREESTRATEGY_HEADER__
#define GfxTL__NULLTREESTRATEGY_HEADER__

namespace GfxTL
{
	struct NullTreeStrategy
	{
		struct CellData
		{};
		
		template< class BaseT >
		struct StrategyBase
		: public BaseT
		{};
	};
};

#endif
