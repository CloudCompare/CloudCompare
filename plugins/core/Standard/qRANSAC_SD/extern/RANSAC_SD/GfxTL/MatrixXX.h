#ifndef GfxTL__MATRIXX_HEADER__
#define GfxTL__MATRIXX_HEADER__
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <iostream>
#include <algorithm>
#include <GfxTL/StdOverrides.h>
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

namespace GfxTL
{
// forward declaration
template< unsigned int D, class T >
class VectorXD;

// forward declaration
template< unsigned int C, unsigned int R, class T >
class MatrixXX;

// matrix operators
template< unsigned int C, unsigned int R, class T >
MatrixXX< C, R, T > & operator+=(MatrixXX< C, R, T > &m, const MatrixXX< C, R, T > &b);
template< unsigned int C, unsigned int R, class T >
MatrixXX< C, R, T > &operator-=(MatrixXX< C, R, T > &m, const MatrixXX< C, R, T > &b);
template< unsigned int C, unsigned int R, class T >
MatrixXX< C, R, T > &operator*=(MatrixXX< C, R, T > &m, const MatrixXX< C, R, T > &b);
template< unsigned int C, unsigned int R, class T >
MatrixXX< C, R, T > &operator*=(MatrixXX< C, R, T > &m, T s);
template< unsigned int C, unsigned int R, class T >
MatrixXX< C, R, T > &operator/=(MatrixXX< C, R, T > &m, T s);
template< unsigned int C, unsigned int R, class T >
bool operator==(const MatrixXX< C, R, T > &a, const MatrixXX< C, R, T > &b);
template< unsigned int C, unsigned int R, class T >
bool operator!=(const MatrixXX< C, R, T > &a, const MatrixXX< C, R, T > &b);

namespace Internal
{
	/** Matrix in column major order */
	template< unsigned int C, unsigned int R, class T >
	class BaseMatrixXX
	{
	public:
		typedef T ScalarType;
		typedef T value_type;
		typedef BaseMatrixXX< C, R, T > ThisType;
		enum { Rows = R, Cols = C, Entries = C * R };

		VectorXD< R, T > &operator[](unsigned int col)
		{
			return *(reinterpret_cast< VectorXD< R, T > * >(&_m[col * R]));
		}

		const VectorXD< R, T > &operator[](unsigned int col) const
		{
			return *(reinterpret_cast< const VectorXD< R, T > * >(&_m[col * R]));
		}

		void Zero()
		{
			memset(_m, 0, sizeof(_m));
		}

		T *Data()
		{
			return _m;
		}

		const T *Data() const
		{
			return _m;
		}

		T Normalize()
		{
			T s = 0;
			for(unsigned int i = 0; i < C * R; ++i)
				s += _m[i] * _m[i];
			if(s == 0)
				return s;
			s = T(std::sqrt(s));
			for(unsigned int i = 0; i < C * R; ++i)
				_m[i] /= s;
			return s;
		}

        void Transpose(BaseMatrixXX< R, C, T > *t) const
		{
			for(unsigned int c = 0; c < C; ++c)
				for(unsigned int r = 0; r < R; ++r)
					(*t)[r][c] = (*this)[c][r];
		}

	protected:
		T _m[Entries];
	};
};

// matrix class
template< unsigned int C, unsigned int R, class T >
class MatrixXX
: public Internal::BaseMatrixXX< C, R, T >
{
public:
	typedef Internal::BaseMatrixXX< C, R, T > SuperType;
	typedef MatrixXX< C, R, T > ThisType;
	typedef MatrixXX< R, C, T > TransposedType;

	MatrixXX() {}

	MatrixXX(const SuperType &mat)
	: SuperType(mat)
	{}

	MatrixXX(const MatrixXX< C, R, T > &mat)
	{
		memcpy(SuperType::_m, mat._m, sizeof(SuperType::_m));
	}

	explicit MatrixXX(T v)
	{
		for(unsigned int i = 0; i < SuperType::Entries; ++i)
			SuperType::_m[i] = v;
	}

	explicit MatrixXX(const T *m)
	{
		memcpy(SuperType::_m, m, sizeof(SuperType::_m));
	}

	MatrixXX(const MatrixXX< 1, R, T > &a, const MatrixXX< 1, R, T > &b)
	{
		AssertDim< C, 2 >::AssertEqual();
		(*this)[0] = a;
		(*this)[1] = b;
	}

	MatrixXX(const MatrixXX< 1, R, T > &a, const MatrixXX< 1, R, T > &b,
		const MatrixXX< 1, R, T > &c)
	{
		AssertDim< C, 3 >::AssertEqual();
		(*this)[0] = a;
		(*this)[1] = b;
		(*this)[2] = c;
	}

	MatrixXX(const MatrixXX< 1, R, T > &a, const MatrixXX< 1, R, T > &b,
		const MatrixXX< 1, R, T > &c, const MatrixXX< 1, R, T > &d)
	{
		AssertDim< C, 4 >::AssertEqual();
		(*this)[0] = a;
		(*this)[1] = b;
		(*this)[2] = c;
		(*this)[3] = c;
	}

	ThisType &operator=(T v)
	{
		for(unsigned int i = 0; i < SuperType::Entries; ++i)
			SuperType::_m[i] = v;
		return *this;
	}

	MatrixXX< C, R, T > &ComponentMul(const MatrixXX< C, R, T > &a)
	{
		for(unsigned int i = 0; i < SuperType::Entries; ++i)
			SuperType::_m[i] *= a._m[i];
		return *this;
	}

	TransposedType Transposed()
	{
		TransposedType t;
		SuperType::Transpose(&t);
		return t;
	}

protected:
	template< unsigned int A, unsigned int B >
	struct AssertDim
	{
		static inline void AssertEqual()
		{
//#ifdef WIN32
//			Error_InvalidDimension();
//#else
			exit(123);
//#endif
		}
	};

	template< unsigned int A >
	struct AssertDim< A, A >
	{
		static inline void AssertEqual() {}
	};
};

// matrix specialization
template< class T >
class MatrixXX< 1, 1, T >
: public Internal::BaseMatrixXX< 1, 1, T >
{
public:
	typedef Internal::BaseMatrixXX< 1, 1, T > SuperType;
	typedef MatrixXX< 1, 1, T > ThisType;
	typedef MatrixXX< 1, 1, T > TransposedType;

	MatrixXX() {}

	MatrixXX(const SuperType &mat)
	: SuperType(mat)
	{}

	MatrixXX(const MatrixXX< 1, 1, T > &mat)
	{
		memcpy(SuperType::_m, mat._m, sizeof(SuperType::_m));
	}

	MatrixXX(T v)
	{
		SuperType::_m[0] = v;
	}

	explicit MatrixXX(const T *m)
	{
		memcpy(SuperType::_m, m, sizeof(SuperType::_m));
	}

	ThisType &operator=(T v)
	{
		SuperType::_m[0] = v;
		return *this;
	}

	MatrixXX< 1, 1, T > &ComponentMul(const MatrixXX< 1, 1, T > &a)
	{
		for(unsigned int i = 0; i < SuperType::Entries; ++i)
			SuperType::_m[i] *= a._m[i];
		return *this;
	}

	operator const T() const
	{
		return SuperType::_m[0];
	}

	operator T &()
	{
		return SuperType::_m[0];
	}
};

// matrix operator implemenation
template< unsigned int C, unsigned int R, class T >
inline MatrixXX< C, R, T > &operator+=(MatrixXX< C, R, T > &m, const MatrixXX< C, R, T > &b)
{
	for(unsigned int i = 0; i < MatrixXX< C, R, T >::Entries; ++i)
		m.Data()[i] += b.Data()[i];
	return m;
}

template< unsigned int C, unsigned int R, class T >
inline MatrixXX< C, R, T > &operator-=(MatrixXX< C, R, T > &m, const MatrixXX< C, R, T > &b)
{
	for(unsigned int i = 0; i < MatrixXX< C, R, T >::Entries; ++i)
		m.Data()[i] -= b.Data()[i];
	return m;
}

template< unsigned int C, unsigned int R, class T >
inline MatrixXX< C, R, T > &operator*=(MatrixXX< C, R, T > &m, const MatrixXX< C, R, T > &b)
{
	m = m * b;
	return m;
}

template< unsigned int C, unsigned int R, class T >
inline MatrixXX< C, R, T > &operator*=(MatrixXX< C, R, T > &m, const MatrixXX< 1, 1, T > &b)
{
	return m *= (const T)b;
}

template< unsigned int C, unsigned int R, class T >
inline MatrixXX< C, R, T > &operator*=(MatrixXX< C, R, T > &m, T s)
{
	for(unsigned int i = 0; i < MatrixXX< C, R, T >::Entries; ++i)
		m.Data()[i] *= s;
	return m;
}

template< unsigned int C, unsigned int R, class T >
inline MatrixXX< C, R, T > &operator/=(MatrixXX< C, R, T > &m, T s)
{
	for(unsigned int i = 0; i < MatrixXX< C, R, T >::Entries; ++i)
		m.Data()[i] /= s;
	return m;
}

template< unsigned int C, unsigned int R, class T >
inline MatrixXX< C, R, T > operator-(const MatrixXX< C, R, T > &a)
{
	MatrixXX< C, R, T > m;
	for(unsigned int i = 0; i < MatrixXX< C, R, T >::Entries; ++i)
		m.Data()[i] = typename MatrixXX< C, R, T >::ScalarType(-1) * a.Data()[i];
	return m;
}

template< unsigned int C, unsigned int R, class T >
inline bool operator==(const MatrixXX< C, R, T > &a,
	const MatrixXX< C, R, T > &b)
{
	return memcmp(a.Data(), b.Data(), sizeof(a)) == 0;
}

template< unsigned int C, unsigned int R, class T >
inline bool operator!=(const MatrixXX< C, R, T > &a, const MatrixXX< C, R, T > &b)
{
	return memcmp(a.Data(), b.Data(), sizeof(a)) != 0;
}

template< unsigned int C, unsigned int R, class T >
inline const MatrixXX< C, R, T > ComponentMul(const MatrixXX< C, R, T > &a,
	const MatrixXX< C, R, T > &b)
{
	MatrixXX< C, R, T > c(a);
	c.ComponentMul(b);
	return c;
}

template< unsigned int C, unsigned int R, class T >
inline const MatrixXX< C, R, T > SqrComponentMul(const MatrixXX< C, R, T > &a)
{
	return ComponentMul(a, a);
}

template< unsigned int C, unsigned int R, class T >
inline const MatrixXX< C, R, T > operator*(T s, const MatrixXX< C, R, T > &a)
{
	MatrixXX< C, R, T > c(a);
	c *= s;
	return c;
}

// matrix multiplication
template< unsigned int C, unsigned int R, unsigned int C2, class T >
inline const MatrixXX< C2, R, T > operator*(const MatrixXX< C, R, T > &a,
	const MatrixXX< C2, C, T > &b)
{
	MatrixXX< C2, R, T > x;
	for(int c = 0; c < C2; ++c)
	{
		for(int r = 0; r < R; ++r)
		{
			x[c][r] = a[0][r] * b[c][0];
			for(int i = 1; i < C; ++i)
				x[c][r] += a[i][r] * b[c][i];
		}
	}
	return x;
}

template< unsigned int C, unsigned int R, class T >
inline const MatrixXX< C, R, T > operator*(const MatrixXX< 1, 1, T > &a,
	const MatrixXX< C, R, T > &b)
{
	return ((const T)a) * b;
}

template< unsigned int C, unsigned int R, class T >
inline const MatrixXX< C, R, T > operator+(const MatrixXX< C, R, T > &a,
	const MatrixXX< C, R, T > &b)
{
	MatrixXX< C, R, T > c;
	for(unsigned int i = 0; i < MatrixXX< C, R, T >::Entries; ++i)
		c.Data()[i] = a.Data()[i] + b.Data()[i];
	return c;
}

template< unsigned int C, unsigned int R, class T >
inline const MatrixXX< C, R, T > operator-(const MatrixXX< C, R, T > &a,
	const MatrixXX< C, R, T > &b)
{
	MatrixXX< C, R, T > c;
	for(unsigned int i = 0; i < MatrixXX< C, R, T >::Entries; ++i)
		c.Data()[i] = a.Data()[i] - b.Data()[i];
	return c;
}

template< unsigned int N, class T >
inline T Trace(const MatrixXX< N, N, T > &a)
{
	T t = 0;
	for(unsigned int i = 0; i < N; ++i)
		t += a[i][i];
	return t;
}

template< unsigned int N, class T >
inline T TraceAbs(const MatrixXX< N, N, T > &a)
{
	T t = 0;
	for(unsigned int i = 0; i < N; ++i)
	{
		if(a[i][i] > 0)
			t += a[i][i];
		else
			t -= a[i][i];
	}
	return t;
}

// this is a rather slow O(n!) algorithm!
// it is only suitable for small matrices!
// larger matrices should use LU decomposition
template< unsigned int N, class T >
inline T Determinant(const MatrixXX< N, N, T > &a)
{
	int j = 1;
	T det = 0;
	for(unsigned int i = 0; i < N; ++i, j *= -1)
	{
		MatrixXX< N - 1, N - 1, T > m;
		for(unsigned int k = 1, kk = 0; kk < N - 1; ++kk, ++k)
		{
			for(unsigned int l = 0, ll = 0; ll < N - 1; ++ll, ++l)
			{
				if(l == i)
					++l;
				m[ll][kk] = a[l][k];
			}
		}
		det += j * Determinant(m);
	}
	return det;
}

template< class T >
inline T Determinant(const MatrixXX< 3, 3, T > &a)
{
	return a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
		- a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
		+ a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
}

template< class T >
inline T Determinant(const MatrixXX< 2, 2, T > &a)
{
	return a[0][0] * a[1][1] - a[1][0] * a[0][1];
}

template< class T >
inline T Determinant(const MatrixXX< 1, 1, T > &a)
{
	return a[0][0];
}

template< class T >
inline void Rotation(T rad, MatrixXX< 2, 2, T > *a)
{
	(*a)[0][0] = std::cos(rad);
	(*a)[0][1] = std::sin(rad);
	(*a)[1][0] = -(*a)[0][1];
	(*a)[1][1] = (*a)[0][0];
}

template< unsigned int N, class T >
void Identity(MatrixXX< N, N, T > *a)
{
	for(unsigned int c = 0; c < N; ++c)
	{
		unsigned int r = 0;
		for(; r < c; ++r)
			(*a)[c][r] = 0;
		(*a)[c][r] = 1;
		for(++r; r < N; ++r)
			(*a)[c][r] = 0;
	}
}

template< unsigned int N, class T >
class IdentityMatrixX
: public MatrixXX< N, N, T >
{
	public:
		IdentityMatrixX()
		{
			Identity(this);
		}
};

template< unsigned int C, unsigned int R, class T >
std::ostream &operator<<(std::ostream &o, const MatrixXX< C, R, T > &mat)
{
	o << "[";
	for(unsigned int i = 0; i < C; ++i)
	{
		o << mat[i];
		if(i < C - 1)
			o << ", ";
	}
	o << "]";
	return o;
}

template< unsigned int C, unsigned int R, class T >
inline MatrixXX< C, R, T > min(const MatrixXX< C, R, T > &a, const MatrixXX< C, R, T > &b)
{
	using namespace std;
	MatrixXX< C, R, T > m;
	for(unsigned int c = 0; c < C; ++c)
		for(unsigned int r = 0; r < R; ++r)
			m[c][r] = min(a[c][r], b[c][r]);
	return m;
}

template< unsigned int C, unsigned int R, class T >
inline MatrixXX< C, R, T > max(const MatrixXX< C, R, T > &a, const MatrixXX< C, R, T > &b)
{
	using namespace std;
	MatrixXX< C, R, T > m;
	for(unsigned int c = 0; c < C; ++c)
		for(unsigned int r = 0; r < R; ++r)
			m[c][r] = max(a[c][r], b[c][r]);
	return m;
}

typedef MatrixXX< 2, 2, float > mat2;
typedef MatrixXX< 3, 3, float > mat3;
typedef MatrixXX< 4, 4, float > mat4;
typedef MatrixXX< 2, 2, float > Mat2f;
typedef MatrixXX< 3, 3, float > Mat3f;
typedef MatrixXX< 4, 4, float > Mat4f;
typedef MatrixXX< 2, 2, float > Matrix2f;
typedef MatrixXX< 3, 3, float > Matrix3f;
typedef MatrixXX< 4, 4, float > Matrix4f;

};

#include <GfxTL/VectorXD.h>

#endif
