#ifndef INDEXITERATOR_HEADER
#define INDEXITERATOR_HEADER
#include <iterator>

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

class DLL_LINKAGE IndexIterator
{
public:
	typedef const size_t value_type;
	typedef value_type *pointer;
	typedef value_type &reference;
	typedef std::forward_iterator_tag iterator_category;
	typedef size_t size_type;
	typedef size_t difference_type;
	IndexIterator() {}
	IndexIterator(size_t index) : m_index(index) {}
	const size_t operator*() const { return m_index; }
	const size_t *operator->() const { return &m_index; }
	IndexIterator &operator++() { ++m_index; return *this; }
	IndexIterator operator++(int) { size_t save = m_index; ++m_index; return IndexIterator(save); }
	bool operator==(IndexIterator a) const { return m_index == a.m_index; }
	bool operator!=(IndexIterator a) const { return m_index != a.m_index; }
	IndexIterator &operator+=(size_t offset)
	{
		m_index += offset;
		return *this;
	}
	IndexIterator &operator-=(size_t offset)
	{
		m_index -= offset;
		return *this;
	}
	IndexIterator operator+(size_t offset) const
	{
		return IndexIterator(m_index + offset);
	}
	IndexIterator operator-(size_t offset) const
	{
		return IndexIterator(m_index - offset);
	}
	size_t operator-(IndexIterator i) const
	{
		return m_index - i.m_index;
	}

	size_t operator[](size_t i) const
	{
		return m_index + i;
	}

private:
	size_t m_index;
};

#endif
