#ifndef GfxTL__SHUFFLEINDICESTREEDATAKERNELSTRATEGY_HEADER__
#define GfxTL__SHUFFLEINDICESTREEDATAKERNELSTRATEGY_HEADER__
#include <vector>
#include <GfxTL/Swap.h>

namespace GfxTL
{
	template< class DataKernelT,
		class ShuffleIndicesT = std::vector< unsigned int > >
	class ShuffleIndicesTreeDataKernelStrategy
	: public DataKernelT
	{
	public:
		typedef typename DataKernelT::value_type value_type;
		typedef typename DataKernelT::HandleType HandleType;
		typedef typename DataKernelT::DereferencedType DereferencedType;

		ShuffleIndicesTreeDataKernelStrategy() : m_shuffleIndices(NULL) {}
		void ShuffleIndices(ShuffleIndicesT *shuffleIndices)
		{ m_shuffleIndices = shuffleIndices; }
		ShuffleIndicesT *ShuffleIndices() const { return m_shuffleIndices; }

	protected:
		void SwapHandles(HandleType a, HandleType b)
		{
			if(m_shuffleIndices)
				Swap(a, b, m_shuffleIndices);
			DataKernelT::SwapHandles(a, b);
		}

	private:
		ShuffleIndicesT *m_shuffleIndices;
	};
};

#endif
