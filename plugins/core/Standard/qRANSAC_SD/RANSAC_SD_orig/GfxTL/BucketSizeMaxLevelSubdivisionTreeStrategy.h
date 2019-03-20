#ifndef GfxTL__BUCKETSIZEMAXLEVELSUBDIVISIONTREESTRATEGY_HEADER__
#define GfxTL__BUCKETSIZEMAXLEVELSUBDIVISIONTREESTRATEGY_HEADER__

namespace GfxTL
{
	template< class InheritedStrategyT >
	struct BucketSizeMaxLevelSubdivisionTreeStrategy
	{
		typedef typename InheritedStrategyT::value_type value_type;

		struct CellData
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

				StrategyBase()
				: m_maxBucketSize(10)
				, m_maxLevel(20)
				{}
				unsigned int &MaxBucketSize() { return m_maxBucketSize; }
				const unsigned int MaxBucketSize() const
				{ return m_maxBucketSize; }
				unsigned int &MaxSubdivisionLevel() { return m_maxLevel; }
				const unsigned int MaxSubdivisionLevel() const
				{ return m_maxLevel; }

			protected:
				template< class BuildInformationT >
				bool ShouldSubdivide(const CellType &cell,
					const BuildInformationT &bi) const
				{
					return cell.Size() > m_maxBucketSize
						&& bi.Level() < m_maxLevel;
				}

			private:
				unsigned int m_maxBucketSize;
				unsigned int m_maxLevel;
		};
	};
};

#endif
