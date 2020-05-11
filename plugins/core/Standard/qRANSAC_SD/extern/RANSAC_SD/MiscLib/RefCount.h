#ifndef MiscLib__REFCOUNT_HEADER__
#define MiscLib__REFCOUNT_HEADER__
#ifdef DOPARALLEL
#include <omp.h>
#endif

namespace MiscLib
{
	class RefCount
	{
		public:
			inline RefCount();
			inline RefCount(const RefCount &);
			inline unsigned int AddRef() const;
			inline unsigned int Release() const;
			inline RefCount &operator=(const RefCount &);

		protected:
			virtual ~RefCount();

		private:
			mutable unsigned int m_refCount;
	};

	RefCount::RefCount()
	: m_refCount(1)
	{}

	RefCount::RefCount(const RefCount &)
	: m_refCount(1)
	{
		// do not copy the ref count!
	}

	unsigned int RefCount::AddRef() const
	{
#ifdef DOPARALLEL
		#pragma omp atomic
#endif
		++m_refCount;
		return m_refCount;
	}

	unsigned int RefCount::Release() const
	{
		if(m_refCount == 1)
		{
#ifdef DOPARALLEL
			#pragma omp critical
#endif
			{
				if(m_refCount)
				{
					m_refCount = 0;
					delete this;
				}
			}
			return 0;
		}
#ifdef DOPARALLEL
		#pragma omp atomic
#endif
		--m_refCount;
		return m_refCount;
	}

	RefCount &RefCount::operator=(const RefCount &)
	{
		// do not copy the ref count!!!
		return *this;
	}
};

#endif
