#ifndef SCOREAACUBETREESTRATEGY_HEADER
#define SCOREAACUBETREESTRATEGY_HEADER
#include "basic.h"
#include <MiscLib/Vector.h>
#include <GfxTL/NullClass.h>
#include <GfxTL/ScalarTypeDeferer.h>
#include <MiscLib/Random.h>

template< unsigned int DimT, class InheritedStrategyT >
struct ScoreAACubeTreeStrategy
{
	typedef typename InheritedStrategyT::value_type value_type;

	class CellData
	: public InheritedStrategyT::CellData
	{
	public:
		typedef typename GfxTL::ScalarTypeDeferer< value_type >::ScalarType
			ScalarType;
		const ScalarType Radius() const { return m_radius; }
		ScalarType &Radius() { return m_radius; }
		//size_t GlobalSize() const { return m_globalSize; }
		//void GlobalSize(size_t globalSize) { m_globalSize = globalSize; }

	private:
		ScalarType m_radius;
		//size_t m_globalSize;
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

		template< class ShapeT, class ScoreT >
		void Score(const ShapeT &shape, /*size_t maxCellSize,*/
			ScoreT *score) const
		{
			typedef typename BaseType::template 
				TraversalInformationBase< GfxTL::NullClass > tibT;
			typedef	typename BaseType::template 
				CellCenterTraversalInformation< tibT > TraversalInformation;
			TraversalInformation ti;
			this->InitRootTraversalInformation(*BaseType::Root(), &ti);
			Score(*BaseType::Root(), ti, shape, /*maxCellSize,*/ score);
		}

	protected:
		template< class BuildInformationT >
		void InitRoot(const BuildInformationT &bi, CellType *root)
		{
			BaseType::InitRoot(bi, root);
			root->Radius() = bi.Cube().DiagLength() / 2;
		}
		template< class BuildInformationT >
		void InitCell(const CellType &parent, const BuildInformationT &pbi,
			unsigned int child, const BuildInformationT &bi,
			CellType *cell)
		{
			BaseType::InitCell(parent, pbi, child, bi, cell);
			cell->Radius() = bi.Cube().DiagLength() / 2;
		}

	private:
		template< class TraversalInformationT, class ShapeT, class ScoreT >
		void Score(const CellType &cell, const TraversalInformationT &ti,
			const ShapeT &shape, /*size_t maxCellSize,*/ ScoreT *score) const
		{
			if(/*cell.Size() <= maxCellSize ||*/ this->IsLeaf(cell))
			{
				//score->UpperBound() += cell.GlobalSize();
				//score->SampledPoints() += cell.Size();
				//typename BaseType::CellRange range;
				//BaseType::GetCellRange(cell, ti, &range);
				//if(maxCellSize > 1)
				//{
				//	size_t r = MiscLib::rn_rand() % cell.Size();
				//	(*score)(shape, *this, Dereference(range.first + r));
				//}
				//else
				//{
					for(typename BaseType::HandleType h = cell.Range().first;
						h != cell.Range().second; ++h)
						(*score)(shape, *this, this->Dereference(h));
				//}
				return;
			}
			for(unsigned int i = 0; i < CellType::NChildren; ++i)
			{
				if(!this->ExistChild(cell, i))
					continue;
				TraversalInformationT cti;
				this->InitTraversalInformation(cell, ti, i, &cti);
				//typename BaseType::CellCenterType center;
				//CellCenter(cell[i], cti, &center);
				ScalarType dist = shape.Distance(*((const Vec3f *)&cell[i].Center()/*center*/));
				if(dist < cell[i].Radius() + score->Epsilon())
				{
					Score(cell[i], cti, shape, /*maxCellSize,*/ score);
				}
			}
		}
	};
};

#endif
