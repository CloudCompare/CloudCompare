#ifndef GfxTL__FLATCOPYVECTOR_HEADER__
#define GfxTL__FLATCOPYVECTOR_HEADER__
#ifndef __APPLE__
#include <malloc.h>
#else
#include <stdlib.h>
#endif
#include <memory.h>
#include <iterator>

#if defined(__x86_64__) || defined(__i386__) || defined(_M_IX86) || defined(_M_X64)
#include <xmmintrin.h>
#endif


#ifdef _mm_malloc
#ifndef a_malloc
#define a_malloc(sz, align) _mm_malloc((sz), (align))
#endif // !a_malloc
#endif // !_mm_malloc
#ifdef _mm_free
#ifndef a_free
#define a_free(ptr) _mm_free((ptr))
#endif // !a_free
#endif // !_mm_free

#ifndef a_free  
#define a_free(a)      free(a) 
#endif // !_mm_free
#ifndef a_malloc
#ifndef __APPLE__
#define a_malloc(sz, align) aligned_alloc((align), (sz))
#else
#define a_malloc(sz, align) malloc(sz) // OSX aligns all allocations to 16 byte boundaries (except valloc which aligns to page boundaries) - so specific alignment requests are ignored.
#endif
#endif // !_mm_malloc

namespace GfxTL
{
	template< class T >
	class FlatCopyVector
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
			typedef size_t difference_type;
			typedef std::reverse_iterator< T * > reverse_iterator;
			typedef std::reverse_iterator< const T * > const_reverse_iterator;

			FlatCopyVector()
			{
				m_begin = NULL;
				m_end = NULL;
				m_capacity = NULL;
			}

			FlatCopyVector(size_t s)
			{
				m_begin = (T *)a_malloc(s * sizeof(T), 16); //new T[s];
				m_end = m_begin + s;
				m_capacity = m_end;
			}

			FlatCopyVector(const FlatCopyVector< T > &v)
			{		
				size_t s = v.size();
				if(!s)
				{
					m_begin = NULL;
					m_end = NULL;
					m_capacity = NULL;
					return;
				}
				m_begin = (T *)a_malloc(s * sizeof(T), 16); //new T[s];
				m_end = m_begin + s;
				m_capacity = m_end;
				memcpy(m_begin, v.m_begin, s * sizeof(T));
			}

			~FlatCopyVector()
			{
				if(m_begin)
					a_free(m_begin); //delete[] m_begin;
			}

			FlatCopyVector< T > &operator=(const FlatCopyVector< T > &v)
			{
				if(&v == this)
					return *this;
				size_t s = v.size();
				if(!s)
				{
					clear();
					return *this;
				}
				if(m_begin)
					a_free(m_begin); //delete[] m_begin;
				m_begin = (T *)a_malloc(s * sizeof(T), 16); //new T[s];
				m_end = m_begin + s;
				m_capacity = m_end;
				memcpy(m_begin, v.m_begin, s * sizeof(T));
				return *this;
			}

			void clear()
			{
				m_end = m_begin;
			}

			void reserve(size_t s)
			{
				if(!s) return;
				if((size_t)(m_capacity - m_begin) < s)
				{
					size_t olds = size();
					T *newBegin = (T *)a_malloc(s * sizeof(T), 16); //new T[s];
					if(m_begin)
					{
						memcpy(newBegin, m_begin, olds * sizeof(T));
						a_free(m_begin); //delete[] m_begin;
					}
					m_end = newBegin + olds;
					m_begin = newBegin;
					m_capacity = m_begin + s;
				}
			}

			size_t size() const
			{
				return m_end - m_begin;
			}

			size_t capacity() const
			{
				return m_capacity - m_begin;
			}

			void resize(size_t s)
			{
				if(!s)
				{
					clear();
					return;
				}
				if((size_t)(m_capacity - m_begin) >= s)
				{
					m_end = m_begin + s;
					return;
				}
				T *newBegin = (T *)a_malloc(s * sizeof(T), 16); //new T[s];
				if(m_begin)
				{
					memcpy(newBegin, m_begin, size() * sizeof(T));
					a_free(m_begin); //delete[] m_begin;
				}
				m_end = newBegin + s;
				m_begin = newBegin;
				m_capacity = m_end;
			}

			void resize(size_t s, const value_type &v)
			{
				size_t oldsize = size();
				resize(s);
				if(s > oldsize)
				{
					for(size_t i = oldsize; i < s; ++i)
						m_begin[i] = v;
				}
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

			void push_back(const T &nn)
			{
				if(m_end >= m_capacity)
				{
					size_t olds = size();
					size_t s = olds * 2;
					if(!s) s = 1;
					T *newBegin = (T *)a_malloc(s * sizeof(T), 16); //new T[s];
					if(m_begin)
					{
						memcpy(newBegin, m_begin, olds * sizeof(T));
						a_free(m_begin); //delete[] m_begin;
					}
					m_end = newBegin + olds;
					m_begin = newBegin;
					m_capacity = m_begin + s;
				}
				*m_end = nn;
				++m_end;
			}

			void insert(T *where, const T &nn)
			{
				if(m_end >= m_capacity)
				{
					size_t whereIdx = where - m_begin;
					size_t olds = size();
					size_t s = olds * 2;
					if(!s) s = 1;
					T *newBegin = (T *)a_malloc(s * sizeof(T), 16); //new T[s];
					if(m_begin)
					{
						memcpy(newBegin, m_begin, olds * sizeof(T));
						a_free(m_begin); //delete[] m_begin;
					}
					m_end = newBegin + olds;
					m_begin = newBegin;
					m_capacity = m_begin + s;
					where = m_begin + whereIdx;
				}
				memmove(where + 1, where, (m_end - where) * sizeof(T));
				*where = nn;
				++m_end;
			}

			void erase(T *where)
			{
				memmove(where, where + 1, (m_end - where - 1) * sizeof(T));
				--m_end;
			}

			void pop_back()
			{
				if(m_end > m_begin)
					--m_end;
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
