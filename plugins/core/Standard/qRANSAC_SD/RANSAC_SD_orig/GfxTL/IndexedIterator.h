#ifndef GfxTL__INDEXEDITERATOR_HEADER__
#define GfxTL__INDEXEDITERATOR_HEADER__
#include <iterator>

namespace GfxTL
{
	template< class IndexIteratorT, class IteratorT >
	class IndexedIterator
	{
		public:
			typedef typename std::iterator_traits< IteratorT >::value_type
				value_type;
			typedef typename std::iterator_traits< IteratorT >::pointer
				pointer;
			typedef typename std::iterator_traits< IteratorT >::reference
				reference;
			typedef typename std::iterator_traits< IndexIteratorT >
				::iterator_category iterator_category;
			typedef typename std::iterator_traits< IndexIteratorT >
				::difference_type difference_type;
			typedef IndexedIterator< IndexIteratorT, IteratorT > ThisType;
			typedef typename std::iterator_traits< IndexIteratorT >::value_type IndexType;

			IndexedIterator()
			{}

			IndexedIterator(const ThisType &it)
			: m_idxIt(it.m_idxIt)
			, m_it(it.m_it)
			{}

			IndexedIterator(IndexIteratorT idxIt, IteratorT it)
			: m_idxIt(idxIt)
			, m_it(it)
			{}

			template< class IdxItT, class ItT >
			IndexedIterator(const IndexedIterator< IdxItT, ItT > &it)
			: m_idxIt(it.IndexIterator())
			, m_it(it.Iterator())
			{}

			pointer operator->()
			{
				return &m_it[*m_idxIt];
			}

			reference operator*()
			{
				return m_it[*m_idxIt];
			}

			ThisType &operator++()
			{
				++m_idxIt;
				return *this;
			}

			ThisType operator++(int)
			{
				ThisType cpy(*this);
				++m_idxIt;
				return cpy;
			}

			ThisType &operator--()
			{
				--m_idxIt;
				return *this;
			}

			ThisType operator--(int)
			{
				ThisType cpy(*this);
				--m_idxIt;
				return cpy;
			}

			bool operator==(const ThisType &i) const
			{
				return m_idxIt == i.m_idxIt && m_it == i.m_it;
			}

			bool operator!=(const ThisType &i) const
			{
				return !operator==(i);
			}

			difference_type operator-(const ThisType &i) const
			{
				return m_idxIt - i.m_idxIt;
			}

			ThisType operator+(difference_type d) const
			{
				return ThisType(m_idxIt + d, m_it);
			}

			ThisType operator-(difference_type d) const
			{
				return ThisType(m_idxIt - d, m_it);
			}

			reference operator[](difference_type d)
			{
				return m_it[m_idxIt[d]];
			}

			IndexType Index() const
			{
				return *m_idxIt;
			}

			const IndexIteratorT &IndexIterator() const { return m_idxIt; }
			const IteratorT &Iterator() const { return m_it; }

			template< class IdxItT, class ItT >
			ThisType &operator=(const IndexedIterator< IdxItT, ItT > &it)
			{
				m_idxIt = it.IndexIterator();
				m_it = it.Iterator();
				return *this;
			}

			bool operator<(const ThisType &i) const
			{
				return m_idxIt < i.m_idxIt;
			}
			bool operator>(const ThisType &i) const
			{
				return m_idxIt > i.m_idxIt;
			}
			bool operator<=(const ThisType &i) const
			{
				return m_idxIt <= i.m_idxIt;
			}
			bool operator>=(const ThisType &i) const
			{
				return m_idxIt >= i.m_idxIt;
			}

		private:
			IndexIteratorT m_idxIt;
			IteratorT m_it;
	};

	template< class IndexIteratorT, class IteratorT >
	IndexedIterator< IndexIteratorT, IteratorT > IndexIterate(IndexIteratorT idxIt,
		IteratorT it)
	{
		return IndexedIterator< IndexIteratorT, IteratorT >(idxIt, it);
	}

};

#endif
