#ifndef REBUILDAACUBETREESTRATEGY_HEADER
#define REBUILDAACUBETREESTRATEGY_HEADER
#include <GfxTL/NullClass.h>
#include <GfxTL/VectorXD.h>
#include <limits>

template< class InheritedStrategyT >
struct RebuildAACubeTreeStrategy
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
		typedef typename GfxTL::ScalarTypeDeferer< value_type >::ScalarType
			ScalarType;
		typedef GfxTL::VectorXD< CellType::Dim, ScalarType > PointType;

		size_t Rebuild()
		{
			if(!BaseType::Root())
				return 0;
			BaseType::Root()->Range() = typename BaseType::CellRange(
				BaseType::BeginHandle(), BaseType::EndHandle());
			//BaseType::Root()->Size(BaseType::size());
			if(!BaseType::Root()->Size() || BaseType::Root()->Size() < BaseType::MaxBucketSize())
			{
				for(unsigned int i = 0; i < CellType::NChildren; ++i)
				{
					if(this->ExistChild(*BaseType::Root(), i))
						delete &((*BaseType::Root())[i]);
					BaseType::Root()->Child(i, NULL);
				}
			}
			if(this->IsLeaf(*BaseType::Root()))
				return 0;
			typename BaseType::HandleType cur = BaseType::BeginHandle();
			size_t maxDepth = 0;
			PointType min, max;
			for(unsigned int i = 0; i < BaseType::m_dim; ++i)
			{
				min[i] = -std::numeric_limits< ScalarType >::infinity();
				max[i] = std::numeric_limits< ScalarType >::infinity();
			}
			for(unsigned int i = 0; i < CellType::NChildren; ++i)
			{
				if(!this->ExistChild(*BaseType::Root(), i))
					continue;
				PointType cmin, cmax;
				for(unsigned int j = 0; j < BaseType::m_dim; ++j)
				{
					if(i & (1 << (BaseType::m_dim - j - 1)))
					{
						cmin[j] = BaseType::Root()->Center()[j];
						cmax[j] = max[j];
					}
					else
					{
						cmin[j] = min[j];
						cmax[j] = BaseType::Root()->Center()[j];
					}
				}
				size_t d = Rebuild(*BaseType::Root(), i, cmin, cmax, &cur);
				if(d > maxDepth)
					maxDepth = d;
			}
			BaseType::Root()->Range() = typename BaseType::CellRange(
				BaseType::BeginHandle(), cur);
			return maxDepth;
		};

	private:
		size_t Rebuild(CellType &parent, size_t childIdx, const PointType &min,
			const PointType &max, typename BaseType::HandleType *cur)
		{
			CellType &cell = parent[childIdx];
			if(this->IsLeaf(cell))
			{
				typename BaseType::HandleType h = *cur;
				if(h >= BaseType::EndHandle())
				{
					cell.Range() = typename BaseType::CellRange(h, h);
					return cell.Level();
				}
				size_t s = cell.Size();
				for(size_t i = 0; i < s && h < BaseType::EndHandle(); ++i, ++h)
				{
					size_t dref = this->Dereference(h);
					bool inside = true;
					for(unsigned int j = 0; j < BaseType::m_dim; ++j)
					{
						if(BaseType::at(dref)[j] <= min[j] || BaseType::at(dref)[j] > max[j])
						{
							inside = false;
							break;
						}
					}
					if(!inside)
						break;
				}
				cell.Range() = typename BaseType::CellRange(*cur, h);
				*cur = h;
				return cell.Level();
			}
			else
			{
				// rebuild children
				typename BaseType::HandleType start = *cur;
				unsigned int numChilds = 0;
				size_t maxDepth = 0;
				for(unsigned int i = 0; i < CellType::NChildren; ++i)
				{
					if(!this->ExistChild(cell, i))
						continue;
					PointType cmin, cmax;
					for(unsigned int j = 0; j < BaseType::m_dim; ++j)
					{
						if(i & (1 << (BaseType::m_dim - j - 1)))
						{
							cmin[j] = cell.Center()[j];
							cmax[j] = max[j];
						}
						else
						{
							cmin[j] = min[j];
							cmax[j] = cell.Center()[j];
						}
					}
					size_t d = Rebuild(cell, i, cmin, cmax, cur);
					if(d > maxDepth)
						maxDepth = d;
					if(cell[i].Size() == 0)
					{
						delete &(cell[i]);
						cell.Child(i, (CellType *)1);
					}
					else
						++numChilds;
				}
				cell.Range() = typename BaseType::CellRange(start, *cur);
				if(numChilds == 0)
				{
					cell.Child(0, NULL);
					maxDepth = cell.Level();
				}
				else if(cell.Size() < BaseType::MaxBucketSize())
				{
					// make cell a leaf
					for(unsigned int i = 0; i < CellType::NChildren; ++i)
					{
						if(!this->ExistChild(cell, i))
							continue;
						delete &(cell[i]);
						cell.Child(i, NULL);
					}
					cell.Child(0, NULL);
					maxDepth = cell.Level();
				}
				return maxDepth;
			}
		}
	};
};

#endif
