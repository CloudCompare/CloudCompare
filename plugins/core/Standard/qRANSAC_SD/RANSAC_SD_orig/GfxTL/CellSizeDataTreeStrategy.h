#ifndef GfxTL__CELLSIZEDATATREESTRATEGY_HEADER__
#define GfxTL__CELLSIZEDATATREESTRATEGY_HEADER__

namespace GfxTL
{
	template< class InheritedStrategyT, class KernelT >
	struct CellSizeDataTreeStrategy
	{
		typedef typename KernelT::value_type value_type;

		class CellData
		: public InheritedStrategyT::CellData
		{
		public:
			typedef typename KernelT::value_type value_type;
			typedef unsigned int size_type;
			size_type Size() const
			{
				return m_size;
			}
			void Size(unsigned int s)
			{
				m_size = s;
			}

			size_type m_size;
		};

		template< class BaseT >
		class StrategyBase
		: public InheritedStrategyT::template StrategyBase< BaseT >
		, public KernelT
		{
		public:
			typedef typename InheritedStrategyT::template StrategyBase< BaseT >
				BaseType;
			typedef typename BaseT::CellType CellType;
			typedef typename KernelT::HandleType HandleType;
			typedef typename KernelT::DereferencedType DereferencedType;
			typedef std::pair< HandleType, HandleType > CellRange;
			typedef typename KernelT::value_type value_type;
			typedef StrategyBase< BaseT > ThisType;

		protected:
			class BuildInformation
			{
				public:
					CellRange &Range() { return m_range; }
					const CellRange &Range() const { return m_range; }

				private:
					CellRange m_range;
			};

			template< class BaseTraversalT >
			class GlobalTraversalInformation
			: public BaseTraversalT
			{};

			template< class BaseTraversalT >
			class TraversalInformation
			: public BaseTraversalT
			{
				public:
					CellRange &Range() { return m_range; }
					const CellRange &Range() const { return m_range; }

				private:
					CellRange m_range;
			};

			template< class BuildInformationT >
			void InitRootBuildInformation(BuildInformationT *bi) const
			{
				RootRange(&bi->Range());
			}

			template< class BuildInformationT >
			void InitBuildInformation(const CellType &parent,
				const BuildInformationT &parentInfo, unsigned int childIdx,
				BuildInformationT *bi) const
			{
				Range(parent, parentInfo.Range(), childIdx, &bi->Range());
			}

			template< class BuildInformationT >
			void InitRoot(const BuildInformationT &bi, CellType *cell)
			{
				cell->m_size = bi.Range().second - bi.Range().first;
			}

			template< class BuildInformationT >
			void InitCell(const CellType &parent,
				const BuildInformationT &parentInfo, unsigned int childIdx,
				const BuildInformationT &bi, CellType *cell)
			{
				cell->m_size = bi.Range().second - bi.Range().first;
			}

			template< class TraversalInformationT >
			void InitRootTraversalInformation(const CellType &root,
				TraversalInformationT *ti) const
			{
				RootRange(&ti->Range());
			}

			template< class TraversalInformationT >
			void InitTraversalInformation(const CellType &parent,
				const TraversalInformationT &pTi, unsigned int childIdx,
				TraversalInformationT *ti) const
			{
				Range(parent, pTi.Range(), childIdx, &ti->Range());
			}

			void RootRange(CellRange *r) const
			{
				r->first = KernelT::BeginHandle();
				r->second = KernelT::EndHandle();
			}

			void Range(const CellType &parent,
				const CellRange &parentRange, unsigned int child,
				CellRange *r) const
			{
				r->first = parentRange.first;
				for(unsigned int i = 0; i < child; ++i)
					if(&(parent[i]) > (CellType *)1)
						r->first += parent[i].m_size;
				r->second = r->first + parent[child].m_size;
			}

			template< class TraversalInformationT >
			void GetCellRange(const CellType &cell, const TraversalInformationT &ti,
				CellRange *range) const
			{
				*range = ti.Range();
			}

			template< class SplitterT, class BuildInformationT >
			void SplitData(const SplitterT &split, const CellType &,
				const BuildInformationT &parentInfo, CellType *left,
				CellType *right)
			{
				unsigned int sizes[2];
				SplitData(split, parentInfo.Range(), &sizes[0], &sizes[1]);
				left->m_size = sizes[0];
				right->m_size = sizes[1];
			}

			template< class SplitterT, class BuildInformationT >
			void SplitData(const SplitterT *splitters,
				const unsigned int numSplitters, const CellType &,
				const BuildInformationT &parentInfo, CellType **cells)
			{
				unsigned int *sizes = new unsigned int[1 << numSplitters];
				SplitData(splitters, numSplitters, parentInfo.Range(),
					sizes);
				unsigned int childCount = 0;
				for(unsigned int i = 0;
					i < (unsigned)(1 << numSplitters); ++i)
					if(sizes[i])
					{
						cells[i] = new CellType;
						cells[i]->m_size = sizes[i];
						++childCount;
					}
					else
						cells[i] = NULL;
				if(!cells[0] && childCount)
					cells[0] = (CellType *)0x1;
				delete[] sizes;
			}

			template< class SplitterT >
			void SplitData(const SplitterT *splitters,
				const unsigned int numSplitters,
				const CellRange &range, unsigned int *sizes)
			{
				const unsigned int numChildren = 1 << numSplitters;
				SplitData(splitters[0], range, &(sizes[0]),
					&(sizes[numChildren >> 1]));
				if(numSplitters == 1)
					return;
				CellRange leftRange(range.first,
					range.first + sizes[0]),
					rightRange(leftRange.second, range.second);
				SplitData(splitters + 1, numSplitters - 1, leftRange,
					sizes);
				SplitData(splitters + 1, numSplitters - 1, rightRange,
					sizes + (numChildren >> 1));
			}

			template< class SplitterT >
			void SplitData(const SplitterT &split,
				const CellRange &range, unsigned int *left, unsigned int *right)
			{
				if(range.second - range.first == 0)
				{
					*left = 0;
					*right = 0;
					return;
				}
				HandleType j = range.first;
				HandleType k = range.second - 1;
				while(1)
				{
					while(j <= k && split(at(Dereference(j))))
						++j;
					while(j < k && !split(at(Dereference(k))))
						--k;
					if(j < k)
					{
						SwapHandles(k, j);
						++j;
						--k;
					}
					else
						break;
				}
				*left = j - range.first;
				*right = (range.second - range.first)
					- *left;
			}

			template< class SplitterT >
			bool SplitAndInsert(const SplitterT &split,
				CellRange parentRange, CellType *left, CellType *right)
			{
				if(split(KernelT::back()))
				{
					++(left->m_size);
					return true;
				}
				else
				{
					++(right->m_size);
					return false;
				}
			}

			void InsertBack(const CellRange &range, CellType *)
			{
				KernelT::InsertBack(range.second - 1);
			}

			bool Remove(CellType &cell, DereferencedType s)
			{
				if(cell(at(s)))
				{
					--cell[0].m_size;
					return true;
				}
				else
				{
					--cell[1].m_size;
					return false;
				}
			}

			void Remove(DereferencedType s, CellType *)
			{
				KernelT::Remove(s);
			}
		};
	};
};

#endif
