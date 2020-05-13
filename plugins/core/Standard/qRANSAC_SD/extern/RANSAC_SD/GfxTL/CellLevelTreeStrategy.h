#ifndef GfxTL__CELLLEVELTREESTRATEGY_HEADER__
#define GfxTL__CELLLEVELTREESTRATEGY_HEADER__

namespace GfxTL
{
	template< class InheritedStrategyT >
	struct CellLevelTreeStrategy
	{
		typedef typename InheritedStrategyT::value_type value_type;
		class CellData
		: public InheritedStrategyT::CellData
		{
		public:
			const size_t Level() const { return m_level; }
			void Level(size_t l) { m_level = l; }

		private:
			size_t m_level;
		};

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

			template< class BaseTraversalT >
			class CellLevelTraversalInformation
			: public BaseTraversalT
			{};

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

			template< class BuildInformationT >
			void InitRoot(const BuildInformationT &bi, CellType *root)
			{
				BaseType::InitRoot(bi, root);
				root->Level(bi.Level());
			}

			template< class BuildInformationT >
			void InitCell(const CellType &parent, const BuildInformationT &pbi,
				unsigned int child, const BuildInformationT &bi,
				CellType *cell)
			{
				BaseType::InitCell(parent, pbi, child, bi, cell);
				cell->Level(bi.Level());
			}

			template< class TraversalInformationT >
			const size_t CellLevel(const CellType &cell,
				const TraversalInformationT &) const
			{
				return cell.Level();
			}
		};
	};
};

#endif
