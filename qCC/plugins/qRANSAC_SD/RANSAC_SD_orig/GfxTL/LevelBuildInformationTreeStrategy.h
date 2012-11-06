#ifndef GfxTL__LEVELBUILDINFORMATIONTREESTRATEGY_HEADER__
#define GfxTL__LEVELBUILDINFORMATIONTREESTRATEGY_HEADER__

namespace GfxTL
{
	template< class InheritedStrategyT >
	struct LevelBuildInformationTreeStrategy
	{
		typedef typename InheritedStrategyT::value_type value_type;
		struct CellData
		: public InheritedStrategyT::CellData
		{};

		template< class BaseT >
		class StrategyBase
		: public InheritedStrategyT::template StrategyBase< BaseT >
		{
		public:
			typedef typename InheritedStrategyT::template StrategyBase< BaseT >
				BaseType;
			typedef typename BaseType::CellType CellType;

		protected:
			class BuildInformation
			: public BaseType::BuildInformation
			{
			public:
				size_t Level() const { return m_level; }
				void Level(size_t l) { m_level = l; }
			private:
				size_t m_level;
			};

			template< class BuildInformationT >
			void InitRootBuildInformation(BuildInformationT *bi)
			{
				BaseType::InitRootBuildInformation(bi);
				bi->Level(0);
			}

			template< class BuildInformationT >
			void InitBuildInformation(const CellType &parent,
				const BuildInformationT &parentInfo, unsigned int childIdx,
				BuildInformationT *bi) const
			{
				BaseType::InitBuildInformation(parent, parentInfo, childIdx,
					bi);
				bi->Level(parentInfo.Level() + 1);
			}
		};
	};
};

#endif
