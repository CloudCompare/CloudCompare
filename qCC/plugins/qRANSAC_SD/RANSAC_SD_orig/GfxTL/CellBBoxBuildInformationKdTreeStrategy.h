#ifndef GfxTL__CELLBBOXBUILDINFORMATIONKDTREESTRATEGY_HEADER__
#define GfxTL__CELLBBOXBUILDINFORMATIONKDTREESTRATEGY_HEADER__

namespace GfxTL
{
	template< class InheritedStrategyT >
	struct CellBBoxBuildInformationKdTreeStrategy
	{
		typedef typename InheritedStrategyT::value_type value_type;

		class CellData
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
			typedef typename ScalarTypeDeferer< value_type >::ScalarType
				ScalarType;
			typedef typename ScalarTypeConversion< ScalarType,
				ScalarType >::DifferenceType DiffScalarType;

			StrategyBase()
			{
				m_bbox[0] = NULL;
				m_bbox[1] = NULL;
			}

			~StrategyBase()
			{
				delete [] m_bbox[0];
				delete [] m_bbox[1];
			}

			const ScalarType * const *RootCellBBox() const { return m_bbox; }

		protected:
			class BuildInformation
			: public BaseType::BuildInformation
			{
				public:
					ScalarType &OldBound() { return m_oldBound; }
					const ScalarType OldBound() const
					{ return m_oldBound; }
					const ScalarType * const *CellBBox() const
					{
						return m_cellBbox;
					}
					void CellBBox(ScalarType **cellBbox)
					{
						m_cellBbox = cellBbox;
					}

				private:
					ScalarType m_oldBound;
					ScalarType **m_cellBbox;
			};

			template< class BuildInformationT >
			void InitRootBuildInformation(BuildInformationT *bi)
			{
				BaseType::InitRootBuildInformation(bi);
				delete [] m_bbox[0]; // delete because dimension could have changed
				delete [] m_bbox[1];
				// init bbox
				m_bbox[0] = new ScalarType[BaseType::m_dim];
				m_bbox[1] = new ScalarType[BaseType::m_dim];
				// init the values (box of zero volume)
				typename BaseType::HandleType i = bi->Range().first;
				this->AssignAsAABoxMin(this->at(this->Dereference(i)), &m_bbox[0]);
				this->AssignAsAABoxMax(this->at(this->Dereference(i)), &m_bbox[1]);
				for(++i; i != bi->Range().second; ++i)
					this->IncludeInAABox(this->at(this->Dereference(i)), m_bbox);
				bi->CellBBox(m_bbox);
			}

			template< class BuildInformationT >
			void InitBuildInformation(const CellType &parent,
				const BuildInformationT &parentInfo, unsigned int childIdx,
				BuildInformationT *bi)
			{
				BaseType::InitBuildInformation(parent, parentInfo, childIdx, bi);
				bi->CellBBox(m_bbox);
			}

			template< class BuildInformationT >
			void EnterGlobalBuildInformation(const CellType &cell,
				BuildInformationT *bi)
			{
				BaseType::EnterGlobalBuildInformation(cell, bi);
				if(bi->CreateChild() == 0)
				{
					bi->OldBound() = m_bbox[1][cell.SplitAxis()];
					m_bbox[1][cell.SplitAxis()] = cell.SplitValue();
				}
				else
				{
					bi->OldBound() = m_bbox[0][cell.SplitAxis()];
					m_bbox[0][cell.SplitAxis()] = cell.SplitValue();
				}
			}

			template< class BuildInformationT >
			void LeaveGlobalBuildInformation(const CellType &cell,
				const BuildInformationT &bi)
			{
				BaseType::LeaveGlobalBuildInformation(cell, bi);
				if(bi.CreateChild() == 1)
					m_bbox[1][cell.SplitAxis()] = bi.OldBound();
				else
					m_bbox[0][cell.SplitAxis()] = bi.OldBound();
			}

		private:
			ScalarType *m_bbox[2];
		};
	};
};

#endif
