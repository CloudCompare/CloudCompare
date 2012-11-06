#ifndef GfxTL__DYNVECTORKERNEL_HEADER__
#define GfxTL__DYNVECTORKERNEL_HEADER__

namespace GfxTL
{
	template< class T >
	class DynVectorKernel
	{
		public:
			DynVectorKernel(unsigned int dim);

			void Add(const T a[], const T b[], T *r) const;
			T Dot(const T a[], const T b[]) const;
			T SqrDistance(const T a[], const T b[]) const;

		private:
			unsigned int m_dim;
	};

	template< class T >
	DynVectorKernel< T >::DynVectorKernel(unsigned int dim)
	: m_dim(dim)
	{}

	template< class T >
	void DynVectorKernel< T >::Add(const T a[], const T b[], T *r) const
	{
		for(unsigned int i = 0; i < m_dim; ++i)
			r[i] = a[i] + b[i];
	}

	template< class T >
	T DynVectorKernel< T >::Dot(const T a[], const T b[]) const
	{
		T r = a[0] * b[0];
		for(size_t i = 1; i < m_dim; ++i)
			r += a[i] * b[i];
		return r;
	}

	template< class T >
	T DynVectorKernel< T >::SqrDistance(const T a[], const T b[]) const
	{
		T r = 0, d;
		for(size_t i = 0; i < m_dim; ++i)
		{
			d = a[i] - b[i];
			r += d * d;
		}
		return r;
	}
};

#endif
