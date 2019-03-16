#ifndef __GfxTL_JACOBI_HEADER__
#define __GfxTL_JACOBI_HEADER__
#include <GfxTL/MatrixXX.h>
#include <GfxTL/VectorXD.h>
#include <GfxTL/MathHelper.h>
#include <GfxTL/Array.h>
#include <memory>
#if !defined(_WIN32) && !defined(WIN32)
#include <unistd.h>
#endif
namespace GfxTL
{
	//Computes all eigenvalues and eigenvectors of a real symmetric matrix a[1..n][1..n]. On
	//output, elements of a above the diagonal are destroyed. d[1..n] returns the eigenvalues of a.
	//v[1..n][1..n] is a matrix whose columns contain, on output, the normalized eigenvectors of
	//a. nrot returns the number of Jacobi rotations that were required.
	#define __GfxTL_JACOBI_ROTATE(a, i, j, k, l) \
		g = (a)[j][i]; \
		h = (a)[l][k]; \
		(a)[j][i] = g - s * (h + g * tau); \
		(a)[l][k] = h + s *(g - h * tau);

	template< unsigned int N, class T >
	bool Jacobi(const MatrixXX< N, N, T > &m, VectorXD< N, T > *d,
		MatrixXX< N, N, T > *v, int *nrot = NULL)
	{
		using namespace std;
		MatrixXX< N, N, T > a(m);
		int j, iq, ip, i;
		T tresh, theta, tau, t, sm, s, h, g, c;
		T volatile temp1, temp2;
		VectorXD< N, T > b, z;
		for(ip = 0; ip < N; ip++)
		{
			for(iq = 0; iq < N; iq++)
				(*v)[iq][ip] = T(0);
			(*v)[ip][ip]=T(1);
		}
		for(ip = 0; ip < N; ip++)
		{
			b[ip] = (*d)[ip] = a[ip][ip];
			z[ip]=T(0);
		}
		if(nrot)
			*nrot=0;
		for(i = 1; i <= 200; i++)
		{
			sm = T(0);
			for(ip = 0; ip < N - 1; ip++)
			{
				for(iq = ip + 1; iq < N; iq++)
					sm += std::abs(a[iq][ip]);
			}
			if(sm == T(0))
				return true;
			if(i < 4)
				tresh = T(0.2) * sm / (N * N);
			else
				tresh = T(0.0);
			for(ip = 0; ip < N - 1; ip++)
			{
				for(iq = ip + 1; iq < N; iq++)
				{
					g = T(100) * std::abs(a[iq][ip]);
					temp1 = std::abs((*d)[ip]) + g;
					temp2 = std::abs((*d)[iq]) + g;
					if(i > 4 &&
						temp1 == std::abs((*d)[ip]) &&
						temp2 == std::abs((*d)[iq]))
						a[iq][ip] = T(0);
					else if(std::abs(a[iq][ip]) > tresh)
					{
						h = (*d)[iq] - (*d)[ip];
						temp1 = (std::abs(h) + g);
						if(temp1 == std::abs(h))
							t = a[iq][ip] / h;
						else
						{
							theta = T(0.5) * h / a[iq][ip];
							t = T(1) / (std::abs(theta) + 
								sqrt(T(1) + theta * theta));
							if(theta < T(0))
								t = -t;
						}
						c = T(1) / sqrt(T(1) + t * t);
						s = t * c;
						tau = s / (T(1) + c);
						h = t * a[iq][ip];
						z[ip] -= h;
						z[iq] += h;
						(*d)[ip] -= h;	
						(*d)[iq] += h;
						a[iq][ip] = T(0);
						for(j = 0; j <= ip - 1; j++)
						{ 
							__GfxTL_JACOBI_ROTATE(a,j,ip,j,iq)
						}
						for(j=ip+1;j<=iq-1;j++)
						{
							__GfxTL_JACOBI_ROTATE(a,ip,j,j,iq)
						}
						for(j=iq+1;j<N;j++)
						{
							__GfxTL_JACOBI_ROTATE(a,ip,j,iq,j)
						}
						for(j=0;j<N;j++)
						{
							__GfxTL_JACOBI_ROTATE((*v),j,ip,j,iq)
						}
						if(nrot)
							++(*nrot);
					}
				}
			}
			for(ip = 0; ip < N; ip++)
			{
				b[ip] += z[ip];
				(*d)[ip] = b[ip];
				z[ip] = T(0);
			}
		}
		return false;
	}

	template< unsigned int N, class T >
	void EigSortDesc(VectorXD< N, T > *d, MatrixXX< N, N, T > *v)
	{
		int k, j, i;
		T p;
		for(i = 0; i < N; ++i)
		{
			p = (*d)[k = i];
			for(j = i + 1 ; j < N; ++j)
				if((*d)[j] >= p)
					p = (*d)[k = j];
			if(k != i)
			{
				(*d)[k] = (*d)[i];
				(*d)[i] = p;
				for(j = 0; j < N; ++j)
				{
					p = (*v)[i][j];
					(*v)[i][j] = (*v)[k][j];
					(*v)[k][j] = p;
				}
			}
		}
	}

	//Computes all eigenvalues and eigenvectors of a real symmetric matrix a[1..n][1..n]. On
	//output, elements of a above the diagonal are destroyed. d[1..n] returns the eigenvalues of a.
	//v[1..n][1..n] is a matrix whose columns contain, on output, the normalized eigenvectors of
	//a. nrot returns the number of Jacobi rotations that were required.
	template< class IteratorT, class VectorT >
	bool Jacobi(Array< 2, IteratorT > *a, VectorT *d,
		Array< 2, IteratorT > *v, int *nrot = NULL)
	{
		using namespace std;
		typedef typename Array< 2, IteratorT >::value_type T;
		intptr_t N = (*a).Extent(0);
		intptr_t j, iq, ip, i;
		T tresh, theta, tau, t, sm, s, h, g, c;
		T volatile temp1, temp2;
		auto_ptr< T > ab(new T[N]), az(new T[N]);
		T *b = ab.get(), *z = az.get();
		for(ip = 0; ip < N; ip++)
		{
			for(iq = 0; iq < N; iq++)
				(*v)[iq][ip] = T(0);
			(*v)[ip][ip]=T(1);
		}
		for(ip = 0; ip < N; ip++)
		{
			b[ip] = (*d)[ip] = (*a)[ip][ip];
			z[ip]=T(0);
		}
		if(nrot)
			*nrot=0;
		for(i = 1; i <= 200; i++)
		{
			sm = T(0);
			for(ip = 0; ip < N - 1; ip++)
			{
				for(iq = ip + 1; iq < N; iq++)
					sm += std::abs((*a)[iq][ip]);
			}
			if(sm == T(0))
				return true;
			if(i < 4)
				tresh = T(0.2) * sm / (N * N);
			else
				tresh = T(0.0);
			for(ip = 0; ip < N - 1; ip++)
			{
				for(iq = ip + 1; iq < N; iq++)
				{
					g = T(100) * std::abs((*a)[iq][ip]);
					temp1 = std::abs((*d)[ip]) + g;
					temp2 = std::abs((*d)[iq]) + g;
					if(i > 4 &&
						temp1 == std::abs((*d)[ip]) &&
						temp2 == std::abs((*d)[iq]))
						(*a)[iq][ip] = T(0);
					else if(std::abs((*a)[iq][ip]) > tresh)
					{
						h = (*d)[iq] - (*d)[ip];
						temp1 = (std::abs(h) + g);
						if(temp1 == std::abs(h))
							t = (*a)[iq][ip] / h;
						else
						{
							theta = T(0.5) * h / (*a)[iq][ip];
							t = T(1) / (std::abs(theta) + 
								sqrt(T(1) + theta * theta));
							if(theta < T(0))
								t = -t;
						}
						c = T(1) / sqrt(T(1) + t * t);
						s = t * c;
						tau = s / (T(1) + c);
						h = t * (*a)[iq][ip];
						z[ip] -= h;
						z[iq] += h;
						(*d)[ip] -= h;	
						(*d)[iq] += h;
						(*a)[iq][ip] = T(0);
						for(j = 0; j <= ip - 1; j++)
						{ 
							__GfxTL_JACOBI_ROTATE((*a),j,ip,j,iq)
						}
						for(j=ip+1;j<=iq-1;j++)
						{
							__GfxTL_JACOBI_ROTATE((*a),ip,j,j,iq)
						}
						for(j=iq+1;j<N;j++)
						{
							__GfxTL_JACOBI_ROTATE((*a),ip,j,iq,j)
						}
						for(j=0;j<N;j++)
						{
							__GfxTL_JACOBI_ROTATE((*v),j,ip,j,iq)
						}
						if(nrot)
							++(*nrot);
					}
				}
			}
			for(ip = 0; ip < N; ip++)
			{
				b[ip] += z[ip];
				(*d)[ip] = b[ip];
				z[ip] = T(0);
			}
		}
		return false;
	}

	template< class IteratorT, class VectorT >
	void EigSortDesc(VectorT *d, Array< 2, IteratorT > *v)
	{
		typedef typename Array< 2, IteratorT >::value_type T;
		size_t N = v->Extent(0);
		int k, j, i;
		T p;
		for(i = 0; i < N; ++i)
		{
			p = (*d)[k = i];
			for(j = i + 1 ; j < N; ++j)
				if((*d)[j] >= p)
					p = (*d)[k = j];
			if(k != i)
			{
				(*d)[k] = (*d)[i];
				(*d)[i] = p;
				for(j = 0; j < N; ++j)
				{
					p = (*v)[i][j];
					(*v)[i][j] = (*v)[k][j];
					(*v)[k][j] = p;
				}
			}
		}
	}

	#undef __GfxTL_JACOBI_ROTATE
};

#endif
