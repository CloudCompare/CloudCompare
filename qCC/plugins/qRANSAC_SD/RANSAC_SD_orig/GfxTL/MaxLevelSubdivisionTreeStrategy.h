#ifndef GfxTL__MAXLEVELSUBDIVISIONTREESTRATEGY_HEADER__
#define GfxTL__MAXLEVELSUBDIVISIONTREESTRATEGY_HEADER__

namespace GfxTL
{
	template< class InheritedStrategyT >
	struct MaxLevelSubdivisionTreeStrategy
	{
		typedef typename InheritedStrategyT::value_type value_type;

		class CellData
		: public InheritedStrategyT::CellData
		{};

		template< class BaseT >
		class StrategyBase
		: public InheritedStrategyT::StrategyBase< BaseT >
		{
		public:
			typedef typename InheritedStrategyT::StrategyBase< BaseT >
				BaseType;
			typedef typename BaseType::CellType CellType;

			StrategyBase()
			: m_maxSubdivisionLevel(15)
			{}

			size_t MaxSubdivisionLevel() const
			{
				return m_maxSubdivisionLevel;
			}

			void MaxSubdivisionLevel(size_t level)
			{
				m_maxSubdivisionLevel = level;
			}

		protected:
			template< class BuildInformationT >
			bool ShouldSubdivide(const CellType &cell, BuildInformationT &bi) const
			{
				return bi.Level() < m_maxSubdivisionLevel;
			}

		private:
			size_t m_maxSubdivisionLevel;
		};
	};
};

#endif
