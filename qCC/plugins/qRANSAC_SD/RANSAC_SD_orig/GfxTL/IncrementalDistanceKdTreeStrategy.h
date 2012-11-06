#ifndef GfxTL__INCREMENTALDISTANCEKDTREESTRATEGY_HEADER__
#define GfxTL__INCREMENTALDISTANCEKDTREESTRATEGY_HEADER__

namespace GfxTL
{
	template< class InheritedStrategyT >
	struct IncrementalDistanceKdTreeStrategy
	{
		typedef typename InheritedStrategyT::value_type value_type;

		class CellData
		: public InheritedStrategyT::CellData
		{
			public:
				typedef typename InheritedStrategyT::CellData BaseType;
				typedef typename BaseType::value_type value_type;
				typedef typename ScalarTypeDeferer< value_type >::ScalarType
					ScalarType;

				ScalarType *AxisInterval() { return m_bdim; }
				const ScalarType *AxisInterval() const { return m_bdim; }

			private:
				ScalarType m_bdim[2];
		};

		template< class BaseT >
		class StrategyBase
		: public InheritedStrategyT::template StrategyBase< BaseT >
		{
		public:
			typedef typename InheritedStrategyT::template StrategyBase< BaseT >
				BaseType;
			typedef typename BaseType::CellType CellType;
			typedef typename ScalarTypeDeferer< value_type >::ScalarType
				ScalarType;
			typedef typename ScalarTypeConversion< ScalarType,
				ScalarType >::DifferenceType DiffScalarType;

		protected:
			template< class BuildInformationT >
			void ComputeSplit(const BuildInformationT &bi, CellType *cell)
			{
				BaseType::ComputeSplit(bi, cell);
				cell->AxisInterval()[0] = bi.CellBBox()[0][cell->SplitAxis()];
				cell->AxisInterval()[1] = bi.CellBBox()[1][cell->SplitAxis()];
			}

			template< class BuildInformationT >
			bool AlternateSplit(const BuildInformationT &bi, CellType *cell)
			{
				bool retVal = BaseType::AlternateSplit(bi, cell);
				if(retVal)
				{
					cell->AxisInterval()[0] = bi.CellBBox()[0][cell->SplitAxis()];
					cell->AxisInterval()[1] = bi.CellBBox()[1][cell->SplitAxis()];
				}
				return retVal;
			}

			template< class BaseTraversalT >
			class TraversalInformation
			: public BaseType::template TraversalInformation< BaseTraversalT >
			{
				public:
					typedef typename BaseType::template DistanceType
					<
						typename ScalarTypeDeferer< value_type >::ScalarType,
						typename ScalarTypeDeferer<
							typename BaseType::template TraversalInformation< BaseTraversalT >::GlobalType::PointType
						>::ScalarType
					>::Type DistScalarType;
					DistScalarType &BoxSqrDist()
					{ return m_boxSqrDist; }
					const DistScalarType &BoxSqrDist() const
					{ return m_boxSqrDist; }

				private:
					DistScalarType m_boxSqrDist;
			};

			template< class TraversalInformationT >
			void InitRootTraversalInformation(const CellType &root,
				TraversalInformationT *ti) const
			{
				BaseType::InitRootTraversalInformation(root, ti);
				ti->BoxSqrDist() = BaseType::BoxSqrDistance(ti->Global().Point(),
					BaseType::RootCellBBox()[0], BaseType::RootCellBBox()[1]);
			}

			template< class TraversalInformationT >
			void InitTraversalInformation(const CellType &parent,
				const TraversalInformationT &pTi, unsigned int childIdx,
				TraversalInformationT *ti) const
			{
				BaseType::InitTraversalInformation(parent, pTi, childIdx, ti);
				// do incremental distance update
				DiffScalarType boxDiff = 0;
				if(pTi.Global().Point()[parent.SplitAxis()]
					< parent.AxisInterval()[0])
					boxDiff = parent.AxisInterval()[0]
						- pTi.Global().Point()[parent.SplitAxis()];
				else if(pTi.Global().Point()[parent.SplitAxis()]
					> parent.AxisInterval()[1])
					boxDiff = pTi.Global().Point()[parent.SplitAxis()]
						- parent.AxisInterval()[1];
				DiffScalarType cutDiff = 0;
				if(childIdx == 0)
				{
					if(pTi.Global().Point()[parent.SplitAxis()]
						< parent.AxisInterval()[0])
						cutDiff = parent.AxisInterval()[0]
							- pTi.Global().Point()[parent.SplitAxis()];
					else if(pTi.Global().Point()[parent.SplitAxis()]
						> parent.SplitValue())
						cutDiff = pTi.Global().Point()[parent.SplitAxis()]
							- parent.SplitValue();
				}
				else
				{
					if(pTi.Global().Point()[parent.SplitAxis()]
						< parent.SplitValue())
						cutDiff = parent.SplitValue()
							- pTi.Global().Point()[parent.SplitAxis()];
					else if(pTi.Global().Point()[parent.SplitAxis()]
						> parent.AxisInterval()[1])
						cutDiff = pTi.Global().Point()[parent.SplitAxis()]
							- parent.AxisInterval()[1];
				}
				ti->BoxSqrDist() = BaseType::IncrementalBoxSqrDistance(
					pTi.BoxSqrDist(), boxDiff, cutDiff);
				//// do incremental distance update
				//// compute previous distance
				//DiffScalarType cutDiff = pTi.Global().Point()[parent.SplitAxis()] -
				//	parent.SplitValue();
				//DiffScalarType boxDiff;
				//if(childIdx == 0)
				//{
				//	if(cutDiff <= 0) // inside
				//	{
				//		ti->BoxSqrDist() = pTi.BoxSqrDist();
				//		return;
				//	}
				//	boxDiff = pTi.Global().Point()[parent.SplitAxis()] -
				//		parent.AxisInterval()[1];
				//}
				//else
				//{
				//	if(cutDiff >= 0) // inside
				//	{
				//		ti->BoxSqrDist() = pTi.BoxSqrDist();
				//		return;
				//	}
				//	boxDiff = parent.AxisInterval()[0] -
				//		pTi.Global().Point()[parent.SplitAxis()];
				//}
				//if(boxDiff < 0)
				//	boxDiff = 0;
				//ti->BoxSqrDist() = BaseType::IncrementalBoxSqrDistance(
				//	pTi.BoxSqrDist(), boxDiff, cutDiff);
			}

			template< class TraversalInformationT >
			typename TraversalInformationT::DistScalarType CellSqrDistance(
				const CellType &cell, const TraversalInformationT &ti) const
			{
				return ti.BoxSqrDist();
			}
		};
	};
};

#endif
