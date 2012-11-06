#ifndef GfxTL__INDEXEDTREEDATAKERNELS_HEADER__
#define GfxTL__INDEXEDTREEDATAKERNELS_HEADER__
#include <GfxTL/IndexedIterator.h>
#include <GfxTL/Swap.h>

namespace GfxTL
{
	// IndexedTreeDataKernels let the tree operate on a set of indeces into the data

	template< class DataT, class IndicesT >
	class IndexedMemberTreeDataKernel
	{
	public:
		typedef typename DataT::value_type value_type;
		typedef typename IndicesT::size_type HandleType;
		typedef typename IndicesT::value_type DereferencedType;
		typedef IndexedIterator< typename IndicesT::iterator,
			typename DataT::iterator > iterator;
		typedef IndexedIterator< typename IndicesT::const_iterator,
			typename DataT::const_iterator > const_iterator;
		typedef typename DataT::iterator dereferenced_iterator;
		typedef typename DataT::const_iterator const_dereferenced_iterator;
		typedef DereferencedType InvariantIdType;

		DataT &ContainedData()
		{
			return m_data;
		}

		const DataT &ContainedData() const
		{
			return m_data;
		}

		void InitIndices()
		{
			m_indices.resize(m_data.size());
			for(size_t i = 0; i < m_indices.size(); ++i)
				m_indices[i] = i;
		}

		DereferencedType Dereference(HandleType h) const
		{
			return m_indices[h];
		}

		InvariantIdType InvariantId(HandleType h) const
		{
			return Dereference(h);
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
			return IndexIterate(m_indices.begin(), m_data.begin());
		}

		iterator end()
		{
			return IndexIterate(m_indices.end(), m_data.begin());
		}

		const_iterator begin() const
		{
			return IndexIterate(m_indices.begin(), m_data.begin());
		}

		const_iterator end() const
		{
			return IndexIterate(m_indices.end(), m_data.begin());
		}

		dereferenced_iterator dereferenced_begin()
		{
			return m_data.begin();
		}

		dereferenced_iterator dereferenced_end()
		{
			return m_data.end();
		}

		const_dereferenced_iterator dereferenced_begin() const
		{
			return m_data.begin();
		}

		const_dereferenced_iterator dereferenced_end() const
		{
			return m_data.end();
		}

		size_t size() const
		{
			return m_indices.size();
		}

		HandleType BeginHandle() const { return 0; }
		HandleType EndHandle() const { return size(); }

protected:
		void SwapHandles(HandleType a, HandleType b)
		{
			Swap(a, b, &m_indices);
		}

		void InsertBack(HandleType h)
		{
			m_indices.resize(m_indices.size() + 1);
			typename IndicesT::value_type v = m_data.size() - 1;
			std::copy_backward(m_indices.begin() + h, m_indices.end() - 1, m_indices.end());
			m_indices.at(h) = v;
		}

	private:
		DataT m_data;
		IndicesT m_indices;
	};

	template< class DataT, class IndicesT >
	class IndexedMemberTreeDataKernel< DataT *, IndicesT >
	{
	public:
		typedef typename DataT::value_type value_type;
		typedef typename IndicesT::size_type HandleType;
		typedef typename IndicesT::value_type DereferencedType;
		typedef IndexedIterator< typename IndicesT::const_iterator,
			typename DataT::const_iterator > const_iterator;
		typedef const_iterator iterator;
		typedef typename DataT::const_iterator dereferenced_iterator;
		typedef typename DataT::const_iterator const_dereferenced_iterator;
		typedef DereferencedType InvariantIdType;

		void IndexedData(const DataT *data, bool initIndices = true)
		{
			m_data = data;
			if(initIndices)
			{
				m_indices.resize(m_data->size());
				for(size_t i = 0; i < m_indices.size(); ++i)
					m_indices[i] = i;
			}
		}

		const DataT *IndexedData() const
		{
			return m_data;
		}

		DereferencedType Dereference(HandleType h) const
		{
			return m_indices[h];
		}

		InvariantIdType InvariantId(HandleType h) const
		{
			return Dereference(h);
		}

		const value_type &at(DereferencedType s) const
		{
			return m_data->at(s);
		}

		const value_type &back() const
		{
			return m_data->back();
		}

		iterator begin()
		{
			return IndexIterate(m_indices.begin(), m_data->begin());
		}

		iterator end()
		{
			return IndexIterate(m_indices.end(), m_data->begin());
		}

		const_iterator begin() const
		{
			return IndexIterate(m_indices.begin(), m_data->begin());
		}

		const_iterator end() const
		{
			return IndexIterate(m_indices.end(), m_data->begin());
		}

		dereferenced_iterator dereferenced_begin()
		{
			return m_data->begin();
		}

		dereferenced_iterator dereferenced_end()
		{
			return m_data->end();
		}

		const_dereferenced_iterator dereferenced_begin() const
		{
			return m_data->begin();
		}

		const_dereferenced_iterator dereferenced_end() const
		{
			return m_data->end();
		}

		size_t size() const
		{
			return m_indices.size();
		}

		HandleType BeginHandle() const { return 0; }
		HandleType EndHandle() const { return size(); }

protected:
		void SwapHandles(HandleType a, HandleType b)
		{
			Swap(a, b, &m_indices);
		}

		void InsertBack(HandleType h)
		{
			m_indices.resize(m_indices.size() + 1);
			typename IndicesT::value_type v = m_data->size() - 1;
			std::copy_backward(m_indices.begin() + h, m_indices.end() - 1, m_indices.end());
			m_indices.at(h) = v;
		}

	private:
		const DataT *m_data;
		IndicesT m_indices;
	};

	template< class IteratorT, class IndicesT >
	class IndexedIteratorTreeDataKernel
	{
	public:
		typedef typename std::iterator_traits< IteratorT >::value_type value_type;
		typedef typename IndicesT::size_type HandleType;
		typedef typename IndicesT::value_type DereferencedType;
		typedef IndexedIterator< typename IndicesT::iterator,
			IteratorT > iterator;
		typedef IndexedIterator< typename IndicesT::const_iterator,
			IteratorT > const_iterator;
		typedef IteratorT dereferenced_iterator;
		typedef IteratorT const_dereferenced_iterator;
		typedef DereferencedType InvariantIdType;

		void IndexedData(IteratorT begin, IteratorT end, bool initIndices = true)
		{
			m_begin = begin;
			m_end = end;
			if(initIndices)
			{
				m_indices.resize(m_end - m_begin);
				for(size_t i = 0; i < m_indices.size(); ++i)
					m_indices[i] = i;
			}
		}

		DereferencedType Dereference(HandleType h) const
		{
			return m_indices[h];
		}

		InvariantIdType InvariantId(HandleType h) const
		{
			return Dereference(h);
		}

		const value_type &at(DereferencedType s) const
		{
			return *(m_begin + s);
		}

		const value_type &back() const
		{
			return *(m_end - 1);
		}

		size_t size() const
		{
			return m_indices.size();
		}

		iterator begin()
		{
			return IndexIterate(m_indices.begin(), m_begin);
		}

		iterator end()
		{
			return IndexIterate(m_indices.end(), m_begin);
		}

		const_iterator begin() const
		{
			return IndexIterate(m_indices.begin(), m_begin);
		}

		const_iterator end() const
		{
			return IndexIterate(m_indices.end(), m_begin);
		}

		dereferenced_iterator dereferenced_begin()
		{
			return m_begin;
		}

		dereferenced_iterator dereferenced_end()
		{
			return m_end;
		}

		const_dereferenced_iterator dereferenced_begin() const
		{
			return m_begin;
		}

		const_dereferenced_iterator dereferenced_end() const
		{
			return m_end;
		}

		HandleType BeginHandle() const { return 0; }
		HandleType EndHandle() const { return size(); }

	protected:
		void SwapHandles(HandleType a, HandleType b)
		{
			Swap(a, b, &m_indices);
		}

	private:
		IteratorT m_begin;
		IteratorT m_end;
		IndicesT m_indices;
	};

	template< class IndexIteratorT, class DataIteratorT >
	class IteratedIndexedIteratorTreeDataKernel
	{
	public:
		typedef typename std::iterator_traits< DataIteratorT >
			::value_type value_type;
		typedef size_t HandleType;
		typedef typename std::iterator_traits< IndexIteratorT >
			::value_type DereferencedType;
		typedef IndexedIterator< IndexIteratorT, DataIteratorT > iterator;
		typedef IndexedIterator< IndexIteratorT, DataIteratorT > const_iterator;
		typedef DataIteratorT dereferenced_iterator;
		typedef DataIteratorT const_dereferenced_iterator;
		typedef DereferencedType InvariantIdType;

		void IndexedData(IndexIteratorT beginIndices,
			IndexIteratorT endIndices, DataIteratorT beginData)
		{
			m_beginIndices = beginIndices;
			m_endIndices = endIndices;
			m_beginData = beginData;
		}

		void IndexedRange(IndexIteratorT beginIndices,
			IndexIteratorT endIndices)
		{
			m_beginIndices = beginIndices;
			m_endIndices = endIndices;
		}

		DereferencedType Dereference(HandleType h) const
		{
			return m_beginIndices[h];
		}

		InvariantIdType InvariantId(HandleType h) const
		{
			return Dereference(h);
		}

		const value_type &at(DereferencedType s) const
		{
			return *(m_beginData + s);
		}

		const value_type &back() const
		{
			return *(m_beginData[*(m_endIndices - 1)]);
		}

		size_t size() const
		{
			return m_endIndices - m_beginIndices;
		}

		iterator begin()
		{
			return IndexIterate(m_beginIndices, m_beginData);
		}

		iterator end()
		{
			return IndexIterate(m_endIndices, m_beginData);
		}

		const_iterator begin() const
		{
			return IndexIterate(m_beginIndices, m_beginData);
		}

		const_iterator end() const
		{
			return IndexIterate(m_endIndices, m_beginData);
		}

		dereferenced_iterator dereferenced_begin()
		{
			return m_beginData;
		}

		dereferenced_iterator dereferenced_end()
		{
			return m_beginData + size();
		}
		 
		
		const_dereferenced_iterator dereferenced_begin() const
		{
			return m_beginData;
		}

		const_dereferenced_iterator dereferenced_end() const
		{
			return m_beginData + size();
		}

		HandleType BeginHandle() const { return 0; }
		HandleType EndHandle() const { return size(); }

	protected:
		void SwapHandles(HandleType a, HandleType b)
		{
			std::swap(m_beginIndices[a], m_beginIndices[b]);
		}

	private:
		IndexIteratorT m_beginIndices;
		IndexIteratorT m_endIndices;
		DataIteratorT m_beginData;
	};
};

#endif
