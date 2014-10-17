#ifndef __GfxTL_VECTORXD_HEADER__
#define __GfxTL_VECTORXD_HEADER__
#include <assert.h>
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <memory.h>
#include <GfxTL/MatrixXX.h>
#include <GfxTL/MathHelper.h>
#include <GfxTL/NullClass.h>
#include <GfxTL/StdOverrides.h>
#include <iostream>
#include <algorithm>
//#include <boost/utility.hpp>
//#include <boost/type_traits.hpp>

namespace GfxTL
{
	template< unsigned int D, class T >
	class VectorXD
	: public MatrixXX< 1, D, T >
	{
		public:
			typedef T ScalarType;
			typedef T value_type;
			typedef VectorXD< D, T > ThisType;
			typedef MatrixXX< 1, D, T > SuperType;

			enum
			{
				Dim = D
			};

			VectorXD() {}

			VectorXD(const VectorXD< D, T > &v)
			: SuperType(v)
			{}

			explicit VectorXD(const T *v)
			{
				memcpy(SuperType::_m, v, sizeof(SuperType::_m));
			}

			template< class S >
			explicit VectorXD(const S *v)
			{
				for(unsigned int i = 0; i < Dim; ++i)
					SuperType::_m[i] = (T)v[i];
			}

			template< class S >
			explicit VectorXD(const VectorXD< D, S > &v)
			{
				for(unsigned int i = 0; i < Dim; ++i)
					SuperType::_m[i] = (T)v[i];
			}

			VectorXD(const SuperType &s)
			: SuperType(s)
			{}

			explicit VectorXD(T x)
			{
				for(unsigned int i = 0; i < Dim; ++i)
					SuperType::_m[i] = x;
			}

			/*template< class S >
			explicit VectorXD(const S x,
				typename boost::enable_if_c< boost::is_convertible< S, ScalarType >::value, NullClass >::type &dummy = *((NullClass *)0))
			{
				for(unsigned int i = 0; i < Dim; ++i)
					SuperType::_m[i] = ScalarType(x);
			}
			//*/

			template< unsigned int X >
			explicit VectorXD(const VectorXD< X, T > &vec)
			{
				for(unsigned int i = 0; i < std::min(D, X); ++i)
					SuperType::_m[i] = vec[i];
				for(unsigned int i = X; i < D; ++i)
					SuperType::_m[i] = 0;
			}

			explicit VectorXD(const VectorXD< D - 1, T > &v, T s)
			{
				for(unsigned int i = 0; i < D - 1; ++i)
					SuperType::_m[i] = v[i];
				SuperType::_m[D - 1] = s;
			}

			explicit VectorXD(const VectorXD< D - 2, T > &v, T s, T s2)
			{
				for(unsigned int i = 0; i < D - 2; ++i)
					SuperType::_m[i] = v[i];
				SuperType::_m[D - 2] = s;
				SuperType::_m[D - 1] = s2;
			}

			explicit VectorXD(const VectorXD< D - 3, T > &v, T s, T s2, T s3)
			{
				for(unsigned int i = 0; i < D - 2; ++i)
					SuperType::_m[i] = v[i];
				SuperType::_m[D - 3] = s;
				SuperType::_m[D - 2] = s2;
				SuperType::_m[D - 1] = s3;
			}

			VectorXD(T x, T y)
			{
//#ifdef WIN32
//				SuperType::AssertDim< D, 2 >::AssertEqual();
//#endif
				SuperType::_m[0] = x;
				SuperType::_m[1] = y;
			}

			VectorXD(T x, T y, T z)
			{
//#ifdef WIN32
//				SuperType::AssertDim< D, 3 >::AssertEqual();
//#endif
				SuperType::_m[0] = x;
				SuperType::_m[1] = y;
				SuperType::_m[2] = z;
			}

			VectorXD(T x, T y, T z, T w)
			{
//#ifdef WIN32
//				SuperType::AssertDim< D, 4 >::AssertEqual();
//#endif
				SuperType::_m[0] = x;
				SuperType::_m[1] = y;
				SuperType::_m[2] = z;
				SuperType::_m[3] = w;
			}

			/*ThisType &operator=(const ThisType &v)
			{
				return (ThisType &)
					(SuperType::operator=(v));
			}*/

			ThisType &operator=(const SuperType &s)
			{
				return (ThisType &)
					(SuperType::operator=(s));
			}

			template< class S >
			ThisType &operator=(const MatrixXX< 1, D, S > &v)
			{
				for(unsigned int i = 0; i < D; ++i)
					SuperType::_m[i] = (T)v[0][i];
				return *this;
			}

			inline ScalarType &operator[](unsigned int i)
			{
				return SuperType::_m[i];
			}

			inline ScalarType operator[](unsigned int i) const
			{
				return SuperType::_m[i];
			}

			/*operator ScalarType *()
			{
				return SuperType::_m;
			}

			operator const ScalarType *() const
			{
				return SuperType::_m;
			}*/

			VectorXD< D, T > &operator+=(const MatrixXX< 1, D, T > &a)
			{
				for(unsigned int i = 0; i < D; ++i)
					SuperType::_m[i] += a.Data()[i];
				return *this;
			}

			VectorXD< D, T > &operator-=(const MatrixXX< 1, D, T > &a)
			{
				for(unsigned int i = 0; i < D; ++i)
					SuperType::_m[i] -= a.Data()[i];
				return *this;
			}

			template< class S >
			VectorXD< D, T > &operator*=(S s)
			{
				for(unsigned int i = 0; i < D; ++i)
					SuperType::_m[i] *= s;
				return *this;
			}

			template< class S >
			VectorXD< D, T > &operator/=(S s)
			{
				for(unsigned int i = 0; i < D; ++i)
					SuperType::_m[i] /= s;
				return *this;
			}

			VectorXD< D, T > operator-() const
			{
				return ScalarType(-1) * (*this);
			}

			ScalarType Length() const
			{
				return sqrt(SqrLength());
			}

			ScalarType SqrLength() const
			{
				ScalarType s = 0;
				for(unsigned int i = 0; i < D; ++i)
					s += SuperType::_m[i] * SuperType::_m[i];
				return s;
			}

			ScalarType L1Length() const
			{
				ScalarType s = 0;
				for(unsigned int i = 0; i < D; ++i)
					s += fabs(SuperType::_m[i]);
				return s;
			}

			void Zero()
			{
				memset(SuperType::_m, 0, sizeof(SuperType::_m));
			}

			bool operator==(const VectorXD< D, T > &a) const
			{
				return memcmp(SuperType::_m, a._m, sizeof(SuperType::_m)) == 0;
			}

			ScalarType SqrDistance(const VectorXD< D, T > &v) const
			{
				ScalarType diff, d;
				diff = SuperType::_m[0] - v[0];
				d = diff * diff;
				for(unsigned int i = 1; i < Dim; ++i)
				{
					diff = SuperType::_m[i] - v[i];
					d += diff * diff;
				}
				return d;
			}

			ScalarType Distance(const VectorXD< D, T > &v) const
			{
				return sqrt(SqrDistance(v));
			}

			ScalarType L1Distance(const VectorXD< D, T > &v) const
			{
				ScalarType d = Math< ScalarType >::Abs(SuperType::_m[0] - v[0]);
				for(unsigned int i = 1; i < Dim; ++i)
					d += Math< ScalarType >::Abs(SuperType::_m[i] - v[i]);
				return d;
			}

			ScalarType MaxDistance(const VectorXD< D, T > &v) const
			{
				ScalarType d = Math< ScalarType >::Abs(SuperType::_m[0] - v[0]),
					di;
				for(unsigned int i = 1; i < Dim; ++i)
				{
					di = Math< ScalarType >::Abs(SuperType::_m[i] - v[i]);
					if(di > d)
						d = di;
				}
				return d;
			}

			VectorXD< Dim + 1, T > Homogene() const
			{
				VectorXD< Dim + 1, T > p;
				for(unsigned int i = 0; i < Dim; ++i)
					p[i] = SuperType::_m[i];
				p[Dim] = 1;
				return p;
			}

			typedef ScalarType *iterator;
			typedef const ScalarType *const_iterator;

			ScalarType *begin() { return SuperType::_m; }
			ScalarType *end() { return SuperType::_m + D; }
			const ScalarType *begin() const { return SuperType::_m; }
			const ScalarType *end() const { return SuperType::_m + D; }
	};

	template< unsigned int D, class T >
	std::ostream &operator<<(std::ostream &o, const VectorXD< D, T > &vec)
	{
		o << "[";
		for(unsigned int i = 0; i < D; ++i)
		{
			o << vec[i];
			if(i < D - 1)
				o << ", ";
		}
		o << "]";
		return o;
	}

	template< unsigned int D, class T >
	inline const VectorXD< D, T > operator+(const VectorXD< D, T > &a,
		const VectorXD< D, T > &b)
	{
		VectorXD< D, T > v;
		for(unsigned int i = 0; i < D; ++i)
			v[i] = a[i] + b[i];
		return v;
	}

	template< class T >
	inline const VectorXD< 1, T > operator+(const VectorXD< 1, T > &a,
		const VectorXD< 1, T > &b)
	{
		return VectorXD< 1, T >(a[0] + b[0]);
	}

	template< class T >
	inline const VectorXD< 2, T > operator+(const VectorXD< 2, T > &a,
		const VectorXD< 2, T > &b)
	{
		return VectorXD< 2, T >(a[0] + b[0], a[1] + b[1]);
	}

	template< class T >
	inline const VectorXD< 3, T > operator+(const VectorXD< 3, T > &a,
		const VectorXD< 3, T > &b)
	{
		return VectorXD< 3, T >(a[0] + b[0], a[1] + b[1], a[2]+ b[2]);
	}

	template< class T >
	inline const VectorXD< 4, T > operator+(const VectorXD< 4, T > &a,
		const VectorXD< 4, T > &b)
	{
		return VectorXD< 4, T >(a[0] + b[0], a[1] + b[1], a[2]+ b[2],
			a[3] + b[3]);
	}

	template< unsigned int D, class T >
	inline const VectorXD< D, T > operator-(const VectorXD< D, T > &a,
		const VectorXD< D, T > &b)
	{
		VectorXD< D, T > v;
		for(unsigned int i = 0; i < D; ++i)
			v[i] = a[i] - b[i];
		return v;
	}

	template< class T >
	inline const VectorXD< 1, T > operator-(const VectorXD< 1, T > &a,
		const VectorXD< 1, T > &b)
	{
		return VectorXD< 1, T >(a[0] - b[0]);
	}

	template< class T >
	inline const VectorXD< 2, T > operator-(const VectorXD< 2, T > &a,
		const VectorXD< 2, T > &b)
	{
		return VectorXD< 2, T >(a[0] - b[0], a[1] - b[1]);
	}

	template< class T >
	inline const VectorXD< 3, T > operator-(const VectorXD< 3, T > &a,
		const VectorXD< 3, T > &b)
	{
		return VectorXD< 3, T >(a[0] - b[0], a[1] - b[1], a[2]- b[2]);
	}

	template< class T >
	inline const VectorXD< 4, T > operator-(const VectorXD< 4, T > &a,
		const VectorXD< 4, T > &b)
	{
		return VectorXD< 4, T >(a[0] - b[0], a[1] - b[1], a[2]- b[2],
			a[3] - b[3]);
	}

	// scalar product (inner product)
	template< unsigned int D, class T >
	inline typename VectorXD< D, T >::ScalarType operator*(const VectorXD< D, T > &a,
		const VectorXD< D, T > &b)
	{
		typename VectorXD< D, T >::ScalarType retVal = a[0] * b[0];
		for(unsigned int i = 1; i < D; ++i)
			retVal += a[i] * b[i];
		return retVal;
	}

	// outer product
	template< unsigned int A, unsigned int B, class T >
	inline MatrixXX< B, A, T > OuterProduct(const VectorXD< A, T > &a, const VectorXD< B, T > &b)
	{
		MatrixXX< B, A, T > m;
		for(unsigned int c = 0; c < B; ++c)
			for(unsigned int r = 0; r < A; ++r)
				m[c][r] = a[r] * b[c];
		return m;
	}

	template< unsigned int D, class T >
	inline MatrixXX< D, D, T > SqrOuterProduct(const VectorXD< D, T > &a)
	{
		MatrixXX< D, D, T > m;
		for(unsigned int c = 0; c < D; ++c)
		{
			m[c][c] = a[c] * a[c];
			for(unsigned int r = c + 1; r < D; ++r)
			{
				m[c][r] = a[r] * a[c];
				m[r][c] = m[c][r];
			}
		}
		return m;
	}

	template< class T >
	inline typename VectorXD< 1, T >::ScalarType operator*(const VectorXD< 1, T > &a,
		const VectorXD< 1, T > &b)
	{
		return a[0] * b[0];
	}

	template< class T >
	inline typename VectorXD< 2, T >::ScalarType operator*(const VectorXD< 2, T > &a,
		const VectorXD< 2, T > &b)
	{
		return a[0] * b[0] + a[1] * b[1];
	}

	template< class T >
	inline typename VectorXD< 3, T >::ScalarType operator*(const VectorXD< 3, T > &a,
		const VectorXD< 3, T > &b)
	{
		return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
	}

	template< class T >
	inline typename VectorXD< 4, T >::ScalarType operator*(const VectorXD< 4, T > &a,
		const VectorXD< 4, T > &b)
	{
		return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
	}

	template< unsigned int D, class T >
	inline VectorXD< D, T > operator*(T s, const VectorXD< D, T > &a)
	{
		VectorXD< D, T > v;
		for(unsigned int i = 0; i < D; ++i)
			v[i] = s * a[i];
		return v;
	}

	template< class T >
	inline const VectorXD< 1, T > operator*(T s, const VectorXD< 1, T > &a)
	{
		return VectorXD< 1, T >(s * a[0]);
	}

	template< class T >
	inline const VectorXD< 2, T > operator*(T s, const VectorXD< 2, T > &a)
	{
		return VectorXD< 2, T >(s * a[0], s * a[1]);
	}

	template< class T >
	inline const VectorXD< 3, T > operator*(T s, const VectorXD< 3, T > &a)
	{
		return VectorXD< 3, T >(s * a[0], s * a[1], s * a[2]);
	}

	template< class T >
	inline const VectorXD< 4, T > operator*(T s, const VectorXD< 4, T > &a)
	{
		return VectorXD< 4, T >(s * a[0], s * a[1], s * a[2],
			s * a[3]);
	}

	template< unsigned int D, class T >
	inline VectorXD< D, T > operator*(const VectorXD< D, T > &a, T s)
	{
		VectorXD< D, T > v;
		for(unsigned int i = 0; i < D; ++i)
			v[i] = a[i] * s;
		return v;
	}

	template< class T >
	inline const VectorXD< 1, T > operator*(const VectorXD< 1, T > &a, T s)
	{
		return VectorXD< 1, T >(a[0] * s);
	}

	template< class T >
	inline const VectorXD< 2, T > operator*(const VectorXD< 2, T > &a, T s)
	{
		return VectorXD< 2, T >(a[0] * s, a[1] * s);
	}

	template< class T >
	inline const VectorXD< 3, T > operator*(const VectorXD< 3, T > &a, T s)
	{
		return VectorXD< 3, T >(a[0] * s, a[1] * s, a[2] * s);
	}

	template< class T >
	inline const VectorXD< 4, T > operator*(const VectorXD< 4, T > &a, T s)
	{
		return VectorXD< 4, T >(a[0] * s, a[1] * s, a[2] * s,
			a[3] * s);
	}

	template< unsigned int D, class T >
	inline VectorXD< D, T > operator/(const VectorXD< D, T > &a, T s)
	{
		VectorXD< D, T > v;
		for(unsigned int i = 0; i < D; ++i)
			v[i] = a[i] / s;
		return v;
	}

	template< class T >
	inline const VectorXD< 1, T > operator/(const VectorXD< 1, T > &a, T s)
	{
		return VectorXD< 1, T >(a[0] / s);
	}

	template< class T >
	inline const VectorXD< 2, T > operator/(const VectorXD< 2, T > &a, T s)
	{
		return VectorXD< 2, T >(a[0] / s, a[1] / s);
	}

	template< class T >
	inline const VectorXD< 3, T > operator/(const VectorXD< 3, T > &a, T s)
	{
		return VectorXD< 3, T >(a[0] / s, a[1] / s, a[2] / s);
	}

	template< class T >
	inline const VectorXD< 4, T > operator/(const VectorXD< 4, T > &a, T s)
	{
		return VectorXD< 4, T >(a[0] / s, a[1] / s, a[2] / s,
			a[3] / s);
	}

	template< class T >
	inline const VectorXD< 3, T > operator%(const VectorXD< 3, T > &a,
		const VectorXD< 3, T > &b)
	{
		return VectorXD< 3, T >(a[1] * b[2] - b[1] * a[2],
			a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
	}

	template< unsigned int D, class T >
	inline VectorXD< D, T > min(const VectorXD< D, T > &a, const VectorXD< D, T > &b)
	{
		using namespace std;
		VectorXD< D, T > m;
		for(unsigned int r = 0; r < D; ++r)
			m[r] = min(a[r], b[r]);
		return m;
	}

	template< unsigned int D, class T >
	inline VectorXD< D, T > max(const VectorXD< D, T > &a, const VectorXD< D, T > &b)
	{
		using namespace std;
		VectorXD< D, T > m;
		for(unsigned int r = 0; r < D; ++r)
			m[r] = max(a[r], b[r]);
		return m;
	}

	template< unsigned int D, class T >
	inline VectorXD< D, T > sqrt(const VectorXD< D, T > &a)
	{
		VectorXD< D, T > v;
		for(unsigned int i = 0; i < D; ++i)
			v[i] = sqrt(a[i]);
		return v;
	}

	typedef VectorXD< 1, float > Vector1Df;
	typedef VectorXD< 1, double > Vector1Dd;
	typedef VectorXD< 2, float > Vector2Df;
	typedef VectorXD< 2, double > Vector2Dd;
	typedef VectorXD< 3, float > Vector3Df;
	typedef VectorXD< 3, double > Vector3Dd;
	typedef VectorXD< 4, float > Vector4Df;
	typedef VectorXD< 4, double > Vector4Dd;

	typedef VectorXD< 1, float > vec1;
	typedef VectorXD< 2, float > vec2;
	typedef VectorXD< 3, float > vec3;
	typedef VectorXD< 4, float > vec4;
	typedef VectorXD< 1, float > vec1f;
	typedef VectorXD< 2, float > vec2f;
	typedef VectorXD< 3, float > vec3f;
	typedef VectorXD< 4, float > vec4f;
	typedef VectorXD< 1, float > Vec1f;
	typedef VectorXD< 2, float > Vec2f;
	typedef VectorXD< 3, float > Vec3f;
	typedef VectorXD< 4, float > Vec4f;
	typedef VectorXD< 1, double > Vec1d;
	typedef VectorXD< 2, double > Vec2d;
	typedef VectorXD< 3, double > Vec3d;
	typedef VectorXD< 4, double > Vec4d;
	typedef VectorXD< 1, int > ivec1;
	typedef VectorXD< 2, int > ivec2;
	typedef VectorXD< 3, int > ivec3;
	typedef VectorXD< 4, int > ivec4;
	typedef VectorXD< 1, unsigned int > uvec1;
	typedef VectorXD< 2, unsigned int > uvec2;
	typedef VectorXD< 3, unsigned int > uvec3;
	typedef VectorXD< 4, unsigned int > uvec4;

	template< class T >
	class Quaternion
	: public VectorXD< 4, T >
	{
		public:
			typedef T ScalarType;
			typedef VectorXD< 4, T > SuperType;

			Quaternion()
			{}

			Quaternion(T w, T x, T y, T z)
			: VectorXD< 4, T >(w, x, y, z)
			{}

			Quaternion< T > &operator*=(const Quaternion< T > &b)
			{
				Quaternion q = (*this) * b;
				*this = q;
				return *this;
			}

			void Rotation(T deg, T a0, T a1, T a2)
			{
				T rad = (deg * (T)M_PI / (T)180.0) / (T)2.0;
				(*this)[0] = std::cos(rad);
				T s = std::sin(rad);
				(*this)[1] = a0 * s;
				(*this)[2] = a1 * s;
				(*this)[3] = a2 * s;
				(*this) /= SuperType::Length();
			}

			void RotationRad(T rad, T a0, T a1, T a2)
			{
				(*this)[0] = std::cos(rad / 2);
				T s = std::sin(rad / 2);
				(*this)[1] = a0 * s;
				(*this)[2] = a1 * s;
				(*this)[3] = a2 * s;
				(*this) /= SuperType::Length();
			}

			void Rotation(T deg, const VectorXD< 3, T > &a)
			{
				T l = a.Length();
				Rotation(deg, a[0] / l, a[1] / l, a[2] / l);
			}

			void Rotation(T *deg, T *a0, T *a1, T *a2) const
			{
				*deg = ((T)std::acos((*this)[0]) * (T)2.0) * (T)180.0 / (T)M_PI;
				T sinA = (T)std::sqrt(1.0 - (*this)[0] * (*this)[0]);
				T absSin = sinA < 0? -sinA : sinA;
				if(absSin < (T)0.00005)
					sinA = (T)1.0;
				*a0 = (*this)[1] / sinA;
				*a1 = (*this)[2] / sinA;
				*a2 = (*this)[3] / sinA;
			}

			void RotationRad(T *rad, T *a0, T *a1, T *a2) const
			{
				*rad = ((T)std::acos((*this)[0]) * (T)2.0);
				T sinA = (T)std::sqrt(1.0 - (*this)[0] * (*this)[0]);
				T absSin = sinA < 0? -sinA : sinA;
				if(absSin < (T)0.00005)
					sinA = (T)1.0;
				*a0 = (*this)[1] / sinA;
				*a1 = (*this)[2] / sinA;
				*a2 = (*this)[3] / sinA;
			}

			void Rotation(T *deg, VectorXD< 3, T > *axis) const
			{
				Rotation(deg, &(*axis)[0], &(*axis)[1], &(*axis)[2]);
			}

			template< class V >
			void Rotate(const V &p, V *r) const
			{
				Quaternion< T > pp(0, p[0], p[1], p[2]);
				Quaternion< T > rr = (*this) * pp * Conjugate();
				(*r)[0] = rr[1];
				(*r)[1] = rr[2];
				(*r)[2] = rr[3];
			}

			Quaternion< T > Conjugate() const
			{
				return Quaternion< T >((*this)[0], -(*this)[1], -(*this)[2],
					-(*this)[3]);
			}

			Quaternion< T > Inverse() const
			{
				Quaternion< T > c = Conjugate();
				T s = 1 / (c.Length() * SuperType::Length());
				for(unsigned int i = 0; i < 4; ++i)
					c[i] *= s;
				return c;
			}

			template< class M >
			void RotationMatrix(M *mat)
			{
				Quaternion< T > &q = *this;
				T ww = q[0] * q[0];
				T xx = q[1] * q[1];
				T yy = q[2] * q[2];
				T zz = q[3] * q[3];
				T xy = q[1] * q[2] * 2;
				T zw = q[0] * q[3] * 2;
				T xz = q[1] * q[3] * 2;
				T xw = q[1] * q[0] * 2;
				T yw = q[0] * q[2] * 2;
				T yz = q[2] * q[3] * 2;

				(*mat)[0][0] = ww + xx - yy - zz;
				(*mat)[0][1] = xy + zw;
				(*mat)[0][2] = xz - yw;

				(*mat)[1][0] = xy - zw;
				(*mat)[1][1] = ww - xx + yy - zz;
				(*mat)[1][2] = yz + xw;

				(*mat)[2][0] = xz + yw;
				(*mat)[2][1] = yz - xw;
				(*mat)[2][2] = ww - xx - yy + zz;

				/*(*mat)[0][0] = (T)1.0 - y2 - z2;
				(*mat)[0][1] = xy - wz;
				(*mat)[0][2] = xz + wy;

				(*mat)[1][0] = xy + wz;
				(*mat)[1][1] = (T)1.0 - x2 - z2;
				(*mat)[1][2] = yz - wx;

				(*mat)[2][0] = xz - wy;
				(*mat)[2][1] = yz + wx;
				(*mat)[2][2] = (T)1.0 - x2 - y2;*/
			}
	};

	template< class T >
	const Quaternion< T > operator*(const Quaternion< T > &a,
		const Quaternion< T > &b)
	{
		return Quaternion< T >(
			a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
			a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
			a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
			a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
			);
	}
};
#endif
