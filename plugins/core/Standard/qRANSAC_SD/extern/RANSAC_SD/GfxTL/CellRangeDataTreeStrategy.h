#ifndef GfxTL__CELLRANGEDATATREESTRATEGY_HEADER__
#define GfxTL__CELLRANGEDATATREESTRATEGY_HEADER__
#include <vector>

namespace GfxTL
{
	template< class InheritedStrategyT, class KernelT >
	struct CellRangeDataTreeStrategy
	{
		typedef typename KernelT::value_type value_type;

		class CellData
		: public InheritedStrategyT::CellData
		{
		public:
			typedef typename KernelT::value_type value_type;
			typedef typename KernelT::HandleType HandleType;
			typedef std::pair< HandleType, HandleType > CellRange;
			size_t Size() const { return m_range.second - m_range.first; }
			const CellRange &Range() const { return m_range; }
			CellRange &Range() { return m_range; }
			CellRange m_range;
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
			{};

			template< class BaseTraversalT >
			class CellRangeTraversalInformation
			: public BaseTraversalT
			{};
			
			template< class BuildInformationT >
			void InitRootBuildInformation(BuildInformationT *bi) const
			{
				bi->Range() = CellRange(KernelT::BeginHandle(),
					KernelT::EndHandle());
			}

			template< class BuildInformationT >
			void InitBuildInformation(const CellType &parent,
				const BuildInformationT &parentInfo, unsigned int childIdx,
				BuildInformationT *bi) const
			{
				bi->Range() = parent[childIdx].Range();
			}

			template< class BuildInformationT >
			void InitRoot(const BuildInformationT &bi, CellType *cell)
			{
				cell->m_range.first = KernelT::BeginHandle();
				cell->m_range.second = KernelT::EndHandle();
			}

			template< class BuildInformationT >
			void InitCell(const CellType &parent,
				const BuildInformationT &parentInfo, unsigned int childIdx,
				const BuildInformationT &bi, CellType *cell)
			{
				cell->Range() = bi.Range();
			}

			template< class TraversalInformationT >
			void InitRootTraversalInformation(const CellType &root,
				TraversalInformationT *ti) const
			{}

			template< class TraversalInformationT >
			void GetCellRange(const CellType &cell, const TraversalInformationT &ti,
				CellRange *range) const
			{
				*range = cell.Range();
			}

			template< class TraversalInformationT >
			void InitTraversalInformation(const CellType &parent,
				const TraversalInformationT &pTi, unsigned int childIdx,
				TraversalInformationT *ti) const
			{}

			template< class SplitterT, class BuildInformationT >
			void SplitData(const SplitterT &split, const CellType &parent,
				const BuildInformationT &, CellType *left,
				CellType *right)
			{
				size_t sizes[2];
				SplitData(split, parent.Range(), &sizes[0], &sizes[1]);
				left->m_range.first = parent.m_range.first;
				left->m_range.second = parent.m_range.first + sizes[0];
				right->m_range.first = left->m_range.second;
				right->m_range.second = right->m_range.first + sizes[1];
			}

			template< class SplitterT, class BuildInformationT >
			void SplitData(const SplitterT &split, const CellType &parent,
				const BuildInformationT &, CellType *left, BuildInformationT *leftBi,
				CellType *right, BuildInformationT *rightBi)
			{
				size_t sizes[2];
				SplitData(split, parent.Range(), &sizes[0], &sizes[1]);
				left->m_range.first = parent.m_range.first;
				left->m_range.second = parent.m_range.first + sizes[0];
				right->m_range.first = left->m_range.second;
				right->m_range.second = right->m_range.first + sizes[1];
			}

			template< class SplitterT, class BuildInformationT >
			void SplitData(const SplitterT *splitters,
				const unsigned int numSplitters, const CellType &parent,
				const BuildInformationT &parentInfo, CellType **cells)
			{
				size_t *sizes = new size_t[size_t(1u) << numSplitters];
				SplitData(splitters, numSplitters, parentInfo.Range(),
					sizes);
				unsigned int childCount = 0;
				HandleType begin = parent.m_range.first;
				for(unsigned int i = 0; i < (1u << numSplitters); ++i)
					if(sizes[i])
					{
						cells[i] = new CellType;
						cells[i]->m_range.first = begin;
						cells[i]->m_range.second = begin + sizes[i];
						begin = cells[i]->m_range.second;
						++childCount;
					}
					else
						cells[i] = NULL;
				if(!cells[0] && childCount)
					cells[0] = (CellType *)0x1;
				delete [] sizes;
			}

			template< class SplitterT >
			void SplitData(const SplitterT *splitters,
				const unsigned int numSplitters,
				const CellRange &range, size_t *sizes)
			{
				const unsigned int numChildren = 1 << numSplitters;
				SplitData(splitters[0], range, &(sizes[0]),
					&(sizes[numChildren >> 1]));
				if(numSplitters == 1)
					return;
				CellRange leftRange(range.first, range.first + sizes[0]),
					rightRange(leftRange.second, range.second);
				SplitData(splitters + 1, numSplitters - 1, leftRange, sizes);
				SplitData(splitters + 1, numSplitters - 1, rightRange,
					sizes + (numChildren >> 1));
			}

			template< class SplitterT >
			void SplitData(const SplitterT &split,
				const CellRange &range, size_t *left, size_t *right)
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
					while(j <= k && split(this->at(this->Dereference(j))))
						++j;
					while(j < k && !split(this->at(this->Dereference(k))))
						--k;
					if(j < k)
					{
						this->SwapHandles(k, j);
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

			void PartitionDataRange(const CellRange &range,
				const std::vector< size_t > &clusterid,
				const std::vector< size_t > &clusterCount,
				CellType **bis)
			{
				std::vector< size_t > clusterPositions(clusterCount.size());
				clusterPositions[0] = 0;
				bis[0]->Range() = CellRange(range.first,
					range.first + clusterCount[0]);
				for(size_t i = 1; i < clusterCount.size(); ++i)
				{
					clusterPositions[i] = clusterPositions[i - 1] + clusterCount[i - 1];
					bis[i]->Range() = CellRange(range.first + clusterPositions[i],
						range.first + clusterPositions[i] + clusterCount[i]);
				}
				std::vector< size_t > partitioning(clusterid.size());
				for(size_t i = 0; i < clusterid.size(); ++i)
					partitioning[i] = clusterPositions[clusterid[i]]++;
				for(size_t i = 0; i < partitioning.size(); ++i)
					while(i != partitioning[i])
					{
						KernelT::SwapHandles(range.first + i,
							range.first + partitioning[i]);
						std::swap(partitioning[i], partitioning[partitioning[i]]);
					}
			}
		};
	};
};

#endif
