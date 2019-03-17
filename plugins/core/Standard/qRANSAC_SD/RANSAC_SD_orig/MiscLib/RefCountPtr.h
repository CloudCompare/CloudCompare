#ifndef MiscLib__REFCOUNTPTR_HEADER__
#define MiscLib__REFCOUNTPTR_HEADER__

namespace MiscLib
{
	template< class T >
	class RefCountPtr
	{
	public:
		RefCountPtr()
		: m_ptr(0)
		{}

		template< class P >
		RefCountPtr(P *ptr)
		: m_ptr(ptr)
		{
			if(m_ptr)
				m_ptr->AddRef();
		}

		RefCountPtr(const RefCountPtr< T > &ptr)
		: m_ptr(ptr.m_ptr)
		{
			if(m_ptr)
				m_ptr->AddRef();
		}

		template< class P >
		RefCountPtr(const RefCountPtr< P > &ptr)
		: m_ptr(ptr.m_ptr)
		{
			if(m_ptr)
				m_ptr->AddRef();
		}

		~RefCountPtr()
		{
			if(m_ptr)
				m_ptr->Release();
		}

		void Release()
		{
			if(m_ptr)
			{
				m_ptr->Release();
				m_ptr = 0;
			}
		}

		template< class N >
		T *New()
		{
			if(m_ptr)
				m_ptr->Release();
			m_ptr = new N();
			return m_ptr;
		}

		RefCountPtr< T > &operator=(const RefCountPtr< T > &ptr)
		{
			if(m_ptr == ptr.m_ptr)
				return *this;
			if(m_ptr)
				m_ptr->Release();
			m_ptr = ptr.m_ptr;
			if(m_ptr)
				m_ptr->AddRef();
			return *this;
		}

		template< class P >
		RefCountPtr< T > &operator=(const RefCountPtr< P > &ptr)
		{
			if(m_ptr == ptr.m_ptr)
				return *this;
			if(m_ptr)
				m_ptr->Release();
			m_ptr = ptr.m_ptr;
			if(m_ptr)
				m_ptr->AddRef();
			return *this;
		}

		template< class P >
		RefCountPtr< T > &operator=(P *ptr)
		{
			if(m_ptr == ptr)
				return *this;
			if(m_ptr)
				m_ptr->Release();
			m_ptr = ptr;
			if(m_ptr)
				m_ptr->AddRef();
			return *this;
		}

		T *operator->()
		{
			return m_ptr;
		}

		const T *operator->() const
		{
			return m_ptr;
		}

		T &operator*()
		{
			return *m_ptr;
		}

		const T &operator*() const
		{
			return *m_ptr;
		}

		operator T *()
		{
			return m_ptr;
		}

		operator const T *() const
		{
			return m_ptr;
		}

		T *Ptr()
		{
			return m_ptr;
		}

		const T *Ptr() const
		{
			return m_ptr;
		}

		template< class P >
		bool operator==(const RefCountPtr< P > &ptr) const
		{
			return m_ptr == ptr.m_ptr;
		}

		template< class P >
		bool operator==(P *ptr) const
		{
			return m_ptr == ptr;
		}

	private:
		T *m_ptr;
	};
};

#endif
