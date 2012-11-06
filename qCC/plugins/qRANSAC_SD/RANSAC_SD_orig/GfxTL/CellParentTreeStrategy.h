#ifndef GfxTL__CELLPARENTTREESTRATEGY_HEADER__
#define GfxTL__CELLPARENTTREESTRATEGY_HEADER__

namespace GfxTL
{

template< class InheritedStrategyT >
struct CellParentTreeStrategy
{
	typedef typename InheritedStrategyT::value_type value_type;

	template< class BaseT >
	class StrategyBase;

	class CellData
	: public InheritedStrategyT::CellData
	{
	public:
		template< class BaseT >
		friend class StrategyBase;
		typedef typename InheritedStrategyT::CellData BaseType;

	private:
		CellData *m_parent;
	};

	template< class BaseT >
	class StrategyBase
	: public InheritedStrategyT::template StrategyBase< BaseT >
	{
	public:
		typedef typename InheritedStrategyT::template StrategyBase< BaseT > BaseType;
		typedef typename BaseType::CellType CellType;

		const CellType * const CellParent(const CellType &c) const
		{
			return (const CellType * const)c.m_parent;
		}

		CellType * const CellParent(CellType &c)
		{
			return (CellType *)c.m_parent;
		}

	protected:
		class BuildInformation
		: public BaseType::BuildInformation
		{
		public:
			CellType *Parent() const { return m_parent; }
			void Parent(CellType *p) { m_parent = p; }
		private:
			CellType *m_parent;
		};

		template< class BuildInformationT >
		void InitRootBuildInformation(BuildInformationT *bi)
		{
			BaseType::InitRootBuildInformation(bi);
			bi->Parent(NULL);
		}

		template< class BuildInformationT >
		void InitBuildInformation(const CellType &parent,
			const BuildInformationT &parentInfo, unsigned int childIdx,
			BuildInformationT *bi)
		{
			BaseType::InitBuildInformation(parent, parentInfo, childIdx, bi);
			bi->Parent(const_cast< CellType * >(&parent));
		}

		template< class BuildInformationT >
		void InitCell(const CellType &parent,
			const BuildInformationT &parentInfo, unsigned int childIdx,
			const BuildInformationT &bi, CellType *cell)
		{
			BaseType::InitCell(parent, parentInfo, childIdx, bi, cell);
			cell->m_parent = bi.Parent();
		}
	};
};

};

#endif
