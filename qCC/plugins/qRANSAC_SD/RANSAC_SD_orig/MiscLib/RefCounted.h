#ifndef MiscLib__REFCOUNTED_HEADER__
#define MiscLib__REFCOUNTED_HEADER__
#ifdef DOPARALLEL
#include <omp.h>
#endif

namespace MiscLib
{
	template< class T >
	class RefCounted
	: public T
	{
	public:
		RefCounted()
		: m_refCount(1)
		{}

		RefCounted(const RefCounted< T > &r)
		: T(r)
		, m_refCount(1)
		{
			// do not copy the ref count!
		}

		unsigned int AddRef() const
		{
			//#pragma omp atomic
			++m_refCount;
			return m_refCount;
		}

		unsigned int Release() const
		{
			if(m_refCount == 1)
			{
				//#pragma omp critical
				{
					if(m_refCount)
					{
						m_refCount = 0;
						delete this;
					}
				}
				return 0;
			}
			//#pragma omp atomic
			--m_refCount;
			return m_refCount;
		}

		RefCounted &operator=(const RefCounted &r)
		{
			*((T *)this) = r;
			return *this; // do not copy the ref count!
		}

	protected:
		virtual ~RefCounted()
		{}

	private:
		mutable unsigned int m_refCount;
	};
};

#endif
