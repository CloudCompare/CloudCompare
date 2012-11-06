#ifndef MiscLib__NOSHRINKVECTOR_HEADER__
#define MiscLib__NOSHRINKVECTOR_HEADER__
#include <MiscLib/AlignedAllocator.h>

namespace MiscLib
{
	// This is a special implementation of std::vector. You should be able to use it whereever you could you use std::vector.
	// For some reason it is faster than the actual std::vector (at least under windows and with the Intel Compiler)
	// It also has the special property that it never (!!!) frees memory automatically, even if you call clear()
	// The only way to get rid of the allocated mem is to call ClearTotal() (or during destruction of course)
	// Note that element constructors and destructors are invoked correctly though, e.g. when calling clear
	// all elements are destructed.
	// The advantage is however that no copying takes place for resize operations (unless new memory must be allocated)
	// So if you have an array whose size constantly varies between 0 and a max size this class is a good choice
	// SO BE CAREFUL WHEN USING THIS CLASS, IT MAY WASTE A LOT OF MEMORY!
	template< class T, class AllocatorT = MiscLib::AlignedAllocator< T > >
	class NoShrinkVector
	: protected AllocatorT
	{
	public:
		typedef size_t size_type;
		typedef T value_type;
		typedef T *iterator;
		typedef const T *const_iterator;
		typedef T &reference;
		typedef const T &const_reference;
		typedef T *pointer;
		typedef const T *const_pointer;
		typedef size_t ptrdiff_t;
		typedef std::reverse_iterator< T * > reverse_iterator;
		typedef std::reverse_iterator< const T * > const_reverse_iterator;

		NoShrinkVector()
		{
			m_begin = NULL;
			m_end = NULL;
			m_capacity = NULL;
		}

		NoShrinkVector(size_type s)
		{
			m_begin = AllocatorT::allocate(s);
			m_end = m_begin + s;
			m_capacity = m_end;
			value_type v;
			for(size_type i = 0; i < s; ++i)
				AllocatorT::construct(m_begin + i, v);
		}

		NoShrinkVector(size_type s, const T &v)
		{
			m_begin = AllocatorT::allocate(s);
			m_end = m_begin + s;
			m_capacity = m_end;
			for(size_type i = 0; i < s; ++i)
				AllocatorT::construct(m_begin + i, v);
		}

		NoShrinkVector(const NoShrinkVector< T, AllocatorT > &v)
		{		
			size_type s = v.size();
			if(!s)
			{
				m_begin = NULL;
				m_end = NULL;
				m_capacity = NULL;
				return;
			}
			m_begin = AllocatorT::allocate(s);
			m_end = m_begin + s;
			m_capacity = m_end;
			for(size_type i = 0; i < s; ++i)
				AllocatorT::construct(m_begin + i, v.m_begin[i]);
		}

		template< class OtherAllocatorT >
		NoShrinkVector(const NoShrinkVector< T, OtherAllocatorT > &v)
		{		
			size_type s = v.size();
			if(!s)
			{
				m_begin = NULL;
				m_end = NULL;
				m_capacity = NULL;
				return;
			}
			m_begin = AllocatorT::allocate(s);
			m_end = m_begin + s;
			m_capacity = m_end;
			for(size_type i = 0; i < s; ++i)
				AllocatorT::construct(m_begin + i, v.m_begin[i]);
		}

		~NoShrinkVector()
		{
			if(m_begin)
			{
				for(size_type i = 0; i < size(); ++i)
					AllocatorT::destroy(m_begin + i);
				AllocatorT::deallocate(m_begin, capacity());
			}
		}

		NoShrinkVector< T > &operator=(const NoShrinkVector< T, AllocatorT > &v)
		{
			if(&v == this)
				return *this;
			size_type s = v.size();
			if(!s)
			{
				clear();
				return *this;
			}
			if(m_begin)
			{
				for(size_type i = 0; i < size(); ++i)
					AllocatorT::destroy(m_begin + i);
				AllocatorT::deallocate(m_begin, capacity());
			}
			m_begin = AllocatorT::allocate(s);
			m_end = m_begin + s;
			m_capacity = m_end;
			for(size_type i = 0; i < s; ++i)
				AllocatorT::construct(m_begin + i, v.m_begin[i]);
			return *this;
		}

		template< class OtherAllocatorT >
		NoShrinkVector< T > &operator=(const NoShrinkVector< T, OtherAllocatorT > &v)
		{
			size_type s = v.size();
			if(!s)
			{
				clear();
				return *this;
			}
			if(m_begin)
			{
				for(size_type i = 0; i < size(); ++i)
					AllocatorT::destroy(m_begin + i);
				AllocatorT::deallocate(m_begin, capacity());
			}
			m_begin = AllocatorT::allocate(s);
			m_end = m_begin + s;
			m_capacity = m_end;
			for(size_type i = 0; i < s; ++i)
				AllocatorT::construct(m_begin + i, v.m_begin[i]);
			return *this;
		}

		void clear()
		{
			for(size_type i = 0; i < size(); ++i)
				AllocatorT::destroy(m_begin + i);
			m_end = m_begin;
		}

		void ClearTotal()
		{
			if(m_begin)
			{
				for(size_type i = 0; i < size(); ++i)
					AllocatorT::destroy(m_begin + i);
				AllocatorT::deallocate(m_begin, capacity());
			}
			m_end = m_begin = m_capacity = NULL;
		}

		void reserve(size_type s)
		{
			if(!s) return;
			if(capacity() < s)
			{
				size_type olds = size();
				T *newBegin = AllocatorT::allocate(s);
				if(m_begin)
				{
					for(size_type i = 0; i < olds; ++i)
					{
						AllocatorT::construct(newBegin + i, m_begin[i]);
						AllocatorT::destroy(m_begin + i);
					}
					AllocatorT::deallocate(m_begin, capacity());
				}
				m_end = newBegin + olds;
				m_begin = newBegin;
				m_capacity = m_begin + s;
			}
		}

		size_type size() const
		{
			return m_end - m_begin;
		}

		size_type capacity() const
		{
			return m_capacity - m_begin;
		}

		void resize(size_type s, const value_type &v)
		{
			if(!s)
			{
				clear();
				return;
			}
			if(capacity() >= s)
			{
				for(size_type i = s; i < size(); ++i)
					AllocatorT::destroy(m_begin + i);
				for(size_type i = size(); i < s; ++i)
					AllocatorT::construct(m_begin + i, v);
				m_end = m_begin + s;
				return;
			}
			T *newBegin = AllocatorT::allocate(2 * s);
			if(m_begin)
			{
				for(size_type i = 0; i < size(); ++i)
				{
					AllocatorT::construct(newBegin + i, m_begin[i]);
					AllocatorT::destroy(m_begin + i);
				}
				AllocatorT::deallocate(m_begin, capacity());
				for(size_type i = size(); i < s; ++i)
					AllocatorT::construct(newBegin + i, v);
			}
			else
			{
				for(size_type i = 0; i < s; ++i)
					AllocatorT::construct(newBegin + i, v);
			}
			m_end = newBegin + s;
			m_begin = newBegin;
			m_capacity = m_begin + 2 * s;
		}

		void resize(size_type s)
		{
			resize(s, value_type());
		}

		operator T *()
		{
			return m_begin;
		}

		operator const T *() const
		{
			return m_begin;
		}

		T &at(size_type i)
		{
			return m_begin[i];
		}

		const T &at(size_type i) const
		{
			return m_begin[i];
		}

		void push_back(const T &v)
		{
			if(m_end >= m_capacity)
			{
				size_type olds = size();
				size_type s = olds * 2;
				if(!s) s = 1;
				T *newBegin = AllocatorT::allocate(s);
				if(m_begin)
				{
					for(size_type i = 0; i < olds; ++i)
					{
						AllocatorT::construct(newBegin + i, m_begin[i]);
						AllocatorT::destroy(m_begin + i);
					}
					AllocatorT::deallocate(m_begin, capacity());
				}
				m_end = newBegin + olds;
				m_begin = newBegin;
				m_capacity = m_begin + s;
			}
			AllocatorT::construct(m_end, v);
			++m_end;
		}

		void insert(T *where, const T &v)
		{
			size_type whereIdx = where - m_begin;
			if(m_end >= m_capacity)
			{
				size_type olds = size();
				size_type s = olds * 2;
				if(!s) s = 1;
				T *newBegin = AllocatorT::allocate(s);
				if(m_begin)
				{
					for(size_type i = 0; i < olds; ++i)
					{
						AllocatorT::construct(newBegin + i, m_begin[i]);
						AllocatorT::destroy(m_begin + i);
					}
					AllocatorT::deallocate(m_begin, capacity());
				}
				m_end = newBegin + olds;
				m_begin = newBegin;
				m_capacity = m_begin + s;
				where = m_begin + whereIdx;
			}
			if(size() > whereIdx)
			{
				AllocatorT::construct(m_end, m_begin[size() - 1]);
				for(size_type i = size() - 1; i > whereIdx; --i)
					m_begin[i] = m_begin[i - 1];
				*where = v;
			}
			else
				AllocatorT::construct(where, v);
			++m_end;
		}

		void erase(T *where)
		{
			for(size_type i = where - m_begin; i < size() - 1; ++i)
				m_begin[i] = m_begin[i + 1];
			--m_end;
			AllocatorT::destroy(m_end);
		}

		void pop_back()
		{
			--m_end;
			AllocatorT::destroy(m_end);
		}

		T *begin()
		{
			return m_begin;
		}

		const T *begin() const
		{
			return m_begin;
		}

		T *end()
		{
			return m_end;
		}

		const T *end() const
		{
			return m_end;
		}

		reverse_iterator rbegin()
		{
			return std::reverse_iterator< T * >(m_end);
		}

		const_reverse_iterator rbegin() const
		{
			return std::reverse_iterator< const T * >(m_end);
		}

		reverse_iterator rend()
		{
			return std::reverse_iterator< T * >(m_begin);
		}

		const_reverse_iterator rend() const
		{
			return std::reverse_iterator< const T * >(m_begin);
		}

		T &back()
		{
			return *(m_end - 1);
		}

		const T &back() const
		{
			return *(m_end - 1);
		}

		T &front()
		{
			return *m_begin;
		}

		const T &front() const
		{
			return *m_begin;
		}

	private:
		T *m_begin;
		T *m_end;
		T *m_capacity;
	};
};

#endif
