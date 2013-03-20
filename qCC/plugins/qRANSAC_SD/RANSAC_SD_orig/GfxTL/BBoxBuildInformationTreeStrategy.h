#ifndef GfxTL__BBOXBUILDINFORMATIONTREESTRATEGY_HEADER__
#define GfxTL__BBOXBUILDINFORMATIONTREESTRATEGY_HEADER__
#include <algorithm>

namespace GfxTL
{
	template< class InheritedStrategyT >
	struct BBoxBuildInformationTreeStrategy
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

		protected:
			// BuildInformation objects do not get copied or copy constructed
			// once they have been initialized. Therefore vector members of
			// unknown dimension do not pose a problem.
			class BuildInformation
			: public BaseType::BuildInformation
			{
				public:
					BuildInformation()
					{
						m_bbox[0] = m_bbox[1] = NULL;
					}

					~BuildInformation()
					{
						delete[] m_bbox[0];
						delete[] m_bbox[1];
					}

					typename ScalarTypeDeferer< value_type >::ScalarType **
					BBox()
					{
						return m_bbox;
					}

					const typename ScalarTypeDeferer< value_type >::ScalarType * const *
					BBox() const
					{
						return m_bbox;
					}

					typename ScalarTypeDeferer< value_type >::ScalarType *m_bbox[2];
			};

			template< class BuildInformationT >
			void InitRootBuildInformation(BuildInformationT *bi)
			{
				BaseType::InitRootBuildInformation(bi);
				// init bbox
				bi->m_bbox[0] =
					new typename ScalarTypeDeferer< value_type >::ScalarType[BaseType::m_dim];
				bi->m_bbox[1] =
					new typename ScalarTypeDeferer< value_type >::ScalarType[BaseType::m_dim];
				// init the values (box of zero volume)
				typename BaseType::HandleType i = bi->Range().first;
				BaseType::AssignAsAABoxMin(this->at(this->Dereference(i)), &bi->m_bbox[0]);
				BaseType::AssignAsAABoxMax(this->at(this->Dereference(i)), &bi->m_bbox[1]);
				for(++i; i != bi->Range().second; ++i)
					BaseType::IncludeInAABox(this->at(this->Dereference(i)), bi->BBox());
			}

			template< class BuildInformationT >
			void InitBuildInformation(const CellType &parent,
				const BuildInformationT &parentInfo, unsigned int childIdx,
				BuildInformationT *bi)
			{
				BaseType::InitBuildInformation(parent, parentInfo, childIdx,
					bi);
				// init bbox
				bi->m_bbox[0] =
					new typename ScalarTypeDeferer< value_type >::ScalarType[BaseType::m_dim];
				bi->m_bbox[1] =
					new typename ScalarTypeDeferer< value_type >::ScalarType[BaseType::m_dim];
				// init the values (box of zero volume)
				if(!(bi->Range().second - bi->Range().first))
				{
					std::fill(bi->m_bbox[0], bi->m_bbox[0] + BaseType::m_dim,
						typename ScalarTypeDeferer< value_type >::ScalarType(0));
					std::fill(bi->m_bbox[1], bi->m_bbox[1] + BaseType::m_dim,
						typename ScalarTypeDeferer< value_type >::ScalarType(0));
					return;
				}
				typename BaseType::HandleType i = bi->Range().first;
				BaseType::AssignAsAABoxMin(this->at(this->Dereference(i)), &bi->m_bbox[0]);
				BaseType::AssignAsAABoxMax(this->at(this->Dereference(i)), &bi->m_bbox[1]);
				for(++i; i != bi->Range().second; ++i)
					BaseType::IncludeInAABox(this->at(this->Dereference(i)), bi->BBox());
			}
		};		
	};
};

#endif
