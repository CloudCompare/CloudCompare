#ifndef GfxTL__CELLCENTERAACUBETREESTRATEGY_HEADER__
#define GfxTL__CELLCENTERAACUBETREESTRATEGY_HEADER__
#include <GfxTL/ScalarTypeDeferer.h>
#include <GfxTL/VectorXD.h>

namespace GfxTL
{
	template< unsigned int DimT, class InheritedStrategyT >
	struct CellCenterAACubeTreeStrategy
	{
		typedef typename InheritedStrategyT::value_type value_type;
		class CellData
		: public InheritedStrategyT::CellData
		{
		public:
			typedef typename GfxTL::ScalarTypeDeferer< value_type >::ScalarType
				ScalarType;
			const GfxTL::VectorXD< DimT, ScalarType > &Center() const
			{ return m_center; }
			GfxTL::VectorXD< DimT, ScalarType > &Center()
			{ return m_center; }

		private:
			GfxTL::VectorXD< DimT, ScalarType > m_center;
		};

		template< class BaseT >
		class StrategyBase
		: public InheritedStrategyT::template StrategyBase< BaseT >
		{
		public:
			typedef typename InheritedStrategyT::template StrategyBase< BaseT >
				BaseType;
			typedef typename BaseType::CellType CellType;
			typedef typename GfxTL::ScalarTypeDeferer< value_type >::ScalarType
				ScalarType;

		protected:
			typedef GfxTL::VectorXD< DimT, ScalarType > CellCenterType;

			template< class BaseTraversalT >
			class CellCenterTraversalInformation
			: public BaseTraversalT
			{};

			template< class BuildInformationT >
			void InitRoot(const BuildInformationT &bi, CellType *root)
			{
				BaseType::InitRoot(bi, root);
				bi.Cube().Center(&root->Center());
			}

			template< class BuildInformationT >
			void InitCell(const CellType &parent, const BuildInformationT &pbi,
				unsigned int child, const BuildInformationT &bi,
				CellType *cell)
			{
				BaseType::InitCell(parent, pbi, child, bi, cell);
				bi.Cube().Center(&cell->Center());
			}

			template< class TraversalInformationT >
			void CellCenter(const CellType &cell,
				const TraversalInformationT &, CellCenterType *center) const
			{
				*center = cell.Center();
			}
		};
	};
};

#endif
