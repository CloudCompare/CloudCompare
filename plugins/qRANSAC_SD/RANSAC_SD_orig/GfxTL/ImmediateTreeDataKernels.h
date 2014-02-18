#ifndef GfxTL__IMMEDIATETREEDATAKERNELS_HEADER__
#define GfxTL__IMMEDIATETREEDATAKERNELS_HEADER__
#include <GfxTL/Swap.h>
#include <iterator>

namespace GfxTL
{
	// ImmediateTreeDataKernels let the tree operate directly on the data,
	// i.e. the data can be reordered to accomodate the tree cell structure.
	// Therefore a const DataT cannot be accepted.
	// There are two possibilities how the kernels maintain the data. The
	// first option makes the data a member of the strategy itself.
	// The second option operates on a range of random access iterators.
	// The two options are implemented as two different Kernels
	// ImmediateMemberTreeDataKernel is a realization of the first possibility
	// ImmediateIteratorTreeDataKernel implements the second one

	template< class DataT >
	class ImmediateMemberTreeDataKernel
	{
		public:
			typedef typename DataT::value_type value_type;
			typedef typename DataT::size_type HandleType;
			typedef HandleType DereferencedType;
			typedef typename DataT::iterator iterator;
			typedef typename DataT::const_iterator const_iterator;

			DataT &ContainedData()
			{
				return m_data;
			}

			const DataT &ContainedData() const
			{
				return m_data;
			}

			DereferencedType Dereference(HandleType h) const
			{
				return h;
			}

			value_type &at(DereferencedType s)
			{
				return m_data.at(s);
			}

			const value_type &at(DereferencedType s) const
			{
				return m_data.at(s);
			}

			value_type &back()
			{
				return m_data.back();
			}

			const value_type &back() const
			{
				return m_data.back();
			}

			iterator begin()
			{
				return m_data.begin();
			}

			iterator end()
			{
				return m_data.end();
			}

			const_iterator begin() const
			{
				return m_data.begin();
			}

			const_iterator end() const
			{
				return m_data.end();
			}

			size_t size() const
			{
				return m_data.size();
			}

			HandleType BeginHandle() const { return 0; }
			HandleType EndHandle() const { return size(); }

		protected:
			void SwapHandles(HandleType a, HandleType b)
			{
				Swap(a, b, &m_data);
			}

			void InsertBack(HandleType h)
			{
				/*
				value_type v = back();
				std::copy_backward(begin() + range.second - 1, end() - 1, end());
				at(range.second - 1) = v;
				*/
			}

			void Remove(DereferencedType s)
			{
				m_data.erase(m_data.begin() + s);
			}

		private:
			DataT m_data;
	};

	template< class DataT >
	class ImmediateMemberTreeDataKernel< DataT * >
	{
		public:
			typedef typename DataT::value_type value_type;
			typedef typename DataT::size_type HandleType;
			typedef HandleType DereferencedType;
			typedef typename DataT::iterator iterator;
			typedef typename DataT::const_iterator const_iterator;

			void ContainedData(DataT *data)
			{
				m_data = data;
			}

			DataT &ContainedData()
			{
				return *m_data;
			}

			const DataT &ContainedData() const
			{
				return *m_data;
			}

			DereferencedType Dereference(HandleType h) const
			{
				return h;
			}

			value_type &at(DereferencedType s)
			{
				return m_data->at(s);
			}

			const value_type &at(DereferencedType s) const
			{
				return m_data->at(s);
			}

			value_type &back()
			{
				return m_data->back();
			}

			const value_type &back() const
			{
				return m_data->back();
			}

			iterator begin()
			{
				return m_data->begin();
			}

			iterator end()
			{
				return m_data->end();
			}

			const_iterator begin() const
			{
				return m_data->begin();
			}

			const_iterator end() const
			{
				return m_data->end();
			}

			size_t size() const
			{
				return m_data->size();
			}

			HandleType BeginHandle() const { return 0; }
			HandleType EndHandle() const { return size(); }

		protected:
			void SwapHandles(HandleType a, HandleType b)
			{
				Swap(a, b, m_data);
			}

			void InsertBack(HandleType h)
			{
				/*value_type v = back();
				std::copy_backward(begin() + range.second - 1, end() - 1, end());
				at(range.second - 1) = v;*/
			}

			void Remove(DereferencedType s)
			{
				m_data->erase(m_data->begin() + s);
			}

		private:
			DataT *m_data;
	};

	template< class DataT >
	class ImmediateRangeTreeDataKernel
	{
		public:
			typedef typename DataT::value_type value_type;
			typedef typename DataT::size_type HandleType;
			typedef HandleType DereferencedType;
			typedef typename DataT::iterator iterator;
			typedef typename DataT::const_iterator const_iterator;

			void ContainedData(DataT *data)
			{
				m_data = data;
			}

			void DataRange(HandleType begin, HandleType end)
			{
				m_beginRange = begin;
				m_endRange = end;
			}

			DataT &ContainedData()
			{
				return *m_data;
			}

			const DataT &ContainedData() const
			{
				return *m_data;
			}

			DereferencedType Dereference(HandleType h) const
			{
				return h;
			}

			value_type &at(DereferencedType s)
			{
				return m_data->at(s);
			}

			const value_type &at(DereferencedType s) const
			{
				return m_data->at(s);
			}

			value_type &back()
			{
				return m_data->begin() + m_endRange - 1;
			}

			const value_type &back() const
			{
				return m_data->begin() + m_endRange - 1;
			}

			iterator begin()
			{
				return m_data->begin() + m_beginRange;
			}

			iterator end()
			{
				return m_data->begin() + m_endRange;
			}

			const_iterator begin() const
			{
				return m_data->begin() + m_beginRange;
			}

			const_iterator end() const
			{
				return m_data->begin() + m_endRange;
			}

			HandleType size() const
			{
				return m_endRange - m_beginRange;
			}

			HandleType BeginHandle() const { return m_beginRange; }
			HandleType EndHandle() const { return m_endRange; }

		protected:
			void SwapHandles(HandleType a, HandleType b)
			{
				Swap(a, b, m_data);
			}

			void InsertBack(HandleType h)
			{
				/*value_type v = back();
				std::copy_backward(begin() + range.second - 1, end() - 1, end());
				at(range.second - 1) = v;*/
			}

			void Remove(DereferencedType s)
			{
				m_data->erase(m_data->begin() + s);
			}

		private:
			DataT *m_data;
			HandleType m_beginRange;
			HandleType m_endRange;
	};

	// this kernel does not support insertion or removal
	template< class IteratorT >
	class ImmediateIteratorTreeDataKernel
	{
		public:
			typedef typename std::iterator_traits< IteratorT >::value_type value_type;
			typedef typename std::iterator_traits< IteratorT >::reference reference;
			typedef size_t HandleType;
			typedef HandleType DereferencedType;
			typedef IteratorT iterator;
			typedef IteratorT const_iterator;

			void Begin(IteratorT begin)
			{
				m_begin = begin;
			}

			void End(IteratorT end)
			{
				m_end = end;
			}

			DereferencedType Dereference(HandleType h) const
			{
				return h;
			}

			reference at(DereferencedType s)
			{
				return m_begin[s];
			}

			reference at(DereferencedType s) const
			{
				return m_begin[s];
			}

			reference back()
			{
				return *(m_end - 1);
			}

			reference back() const
			{
				return *(m_end - 1);
			}

			iterator begin()
			{
				return m_begin;
			}

			iterator end()
			{
				return m_end;
			}

			const_iterator begin() const
			{
				return m_begin;
			}

			const_iterator end() const
			{
				return m_end;
			}

			size_t size() const
			{
				return m_end - m_begin;
			}

			HandleType BeginHandle() const { return 0; }
			HandleType EndHandle() const { return size(); }

		protected:
			void SwapHandles(HandleType a, HandleType b)
			{
				using namespace std;
				swap(at(a), at(b));
			}

		private:
			IteratorT m_begin;
			IteratorT m_end;
	};
};

#endif
