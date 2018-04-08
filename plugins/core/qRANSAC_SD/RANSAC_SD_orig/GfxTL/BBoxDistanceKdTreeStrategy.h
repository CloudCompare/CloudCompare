#ifndef GfxTL__BBOXDISTANCEKDTREESTRATEGY_HEADER__
#define GfxTL__BBOXDISTANCEKDTREESTRATEGY_HEADER__

namespace GfxTL
{
	template< class InheritedStrategyT >
	struct BBoxDistanceKdTreeStrategy
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

				CellData()
				{
					m_bbox[0] = m_bbox[1] = NULL;
				}

				~CellData()
				{
					delete[] m_bbox[0];
					delete[] m_bbox[1];
				}

				ScalarType **BBox()
				{
					return m_bbox;
				}

				const ScalarType * const *BBox() const
				{
					return m_bbox;
				}

			private:
				ScalarType *m_bbox[2];
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
				typedef typename BaseType::DereferencedType DereferencedType;
				typedef typename ScalarTypeConversion< ScalarType,
					ScalarType >::DifferenceType DiffScalarType;

				StrategyBase()
				{}

				~StrategyBase()
				{}

			protected:
				template< class BuildInformationT >
				void InitRoot(const BuildInformationT &bi, CellType *cell)
				{
					BaseType::InitRoot(bi, cell);
					cell->BBox()[0] = new ScalarType[BaseType::m_dim];
					cell->BBox()[1] = new ScalarType[BaseType::m_dim];
					BaseType::AssignVector(bi.BBox()[0], &cell->BBox()[0]);
					BaseType::AssignVector(bi.BBox()[1], &cell->BBox()[1]);
				}

				template< class BuildInformationT >
				void InitCell(const CellType &parent,
					const BuildInformationT &parentInfo, unsigned int childIdx,
					const BuildInformationT &bi, CellType *cell)
				{
					BaseType::InitCell(parent, parentInfo, childIdx, bi, cell);
					cell->BBox()[0] = new ScalarType[BaseType::m_dim];
					cell->BBox()[1] = new ScalarType[BaseType::m_dim];
					BaseType::AssignVector(bi.BBox()[0], &cell->BBox()[0]);
					BaseType::AssignVector(bi.BBox()[1], &cell->BBox()[1]);
				}

				template< class TraversalInformationT >
				void UpdateCellWithBack(const TraversalInformationT &ti,
					CellType *cell)
				{
					BaseType::UpdateCellWithBack(ti, cell);
					BaseType::IncludeInAABox(BaseType::back(), cell->BBox());
				}

				template< class TraversalInformationT >
				typename BaseType::template DistanceType
					<
						ScalarType,
						typename ScalarTypeDeferer<
							typename TraversalInformationT::GlobalType::PointType
						>::ScalarType
					>::Type
				CellSqrDistance(const CellType &cell, const TraversalInformationT &ti) const
				{
					return BaseType::BoxSqrDistance(ti.Global().Point(), cell.BBox()[0],
						cell.BBox()[1]);
				}
		};
	};
};

#endif
