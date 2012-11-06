#ifndef GfxTL__BUCKETSIZESUBDIVISIONTREESTRATEGY_HEADER__
#define GfxTL__BUCKETSIZESUBDIVISIONTREESTRATEGY_HEADER__

namespace GfxTL
{
	template< class InheritedStrategyT >
	struct BucketSizeSubdivisionTreeStrategy
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
				{}
				void InitMaxBucketSize(size_t size)
				{
					m_maxBucketSize = size;
				}
				size_t &MaxBucketSize() { return m_maxBucketSize; }
				const size_t MaxBucketSize() const { return m_maxBucketSize; }

			protected:
				template< class BuildInformationT >
				bool ShouldSubdivide(const CellType &cell,
					const BuildInformationT &bi) const
				{
					return cell.Size() > m_maxBucketSize;
				}

			private:
				size_t m_maxBucketSize;
		};
	};
};

#endif
