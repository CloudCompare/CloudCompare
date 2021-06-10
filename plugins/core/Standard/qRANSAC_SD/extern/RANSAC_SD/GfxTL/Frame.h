#ifndef GfxTL__FRAME_HEADER__
#define GfxTL__FRAME_HEADER__
#include <GfxTL/HyperplaneCoordinateSystem.h>
#include <GfxTL/VectorXD.h>
#include <GfxTL/MatrixXX.h>

namespace GfxTL
{

template< unsigned int DimT, class ScalarT >
class Frame
{};

template< class ScalarT >
class Frame< 2, ScalarT >
{
public:
	typedef ScalarT ScalarType;
	enum { Dim = 2 };
	typedef typename HyperplaneCoordinateSystem< ScalarType, Dim >::PointType
		PointType;

	void Canonical();
	template< class V >
	void FromNormal(const V &v)
	{
		m_hcs.FromNormal(v);
		for(unsigned int i = 0; i < Dim; ++i)
			m_xaxis[i] = v[i];
	}
	void ToLocal(const PointType &p, PointType *l) const;
	void ToGlobal(const PointType &p, PointType *g) const;
	void RotateFrame(ScalarType radians);
	operator PointType *() { return reinterpret_cast< PointType * >(this); }
	operator const PointType *() const { return reinterpret_cast< const PointType * >(this); }

private:
	PointType m_xaxis;
	HyperplaneCoordinateSystem< ScalarType, Dim > m_hcs;
};

template< class ScalarT >
void Frame< 2, ScalarT >::Canonical()
{
	for(unsigned int i = 0; i < Dim; ++i)
	{
		for(unsigned int j = 0; j < Dim; ++j)
			(*this)[i][j] = 0;
		(*this)[i][i] = 1;
	}
}

template< class ScalarT >
void Frame< 2, ScalarT >::ToLocal(const PointType &p, PointType *l) const
{
	for(unsigned int i = 0; i < Dim; ++i)
		(*l)[i] = (*this)[i] * p;
}

template< class ScalarT >
void Frame< 2, ScalarT >::ToGlobal(const PointType &p, PointType *g) const
{
	*g = p[0] * (*this)[0];
	for(unsigned int i = 1; i < Dim; ++i)
		*g += p[i] * (*this)[i];
}

template< class ScalarT >
void Frame< 2, ScalarT >::RotateFrame(ScalarType radians)
{
	GfxTL::MatrixXX< 2, 2, ScalarType > r;
	GfxTL::Rotation(radians, &r);
	for(unsigned int i = 0; i < Dim; ++i)
		(*this)[i] = r * (*this)[i];
}

template< class ScalarT >
class Frame< 3, ScalarT >
{
public:
	typedef ScalarT ScalarType;
	enum { Dim = 3 };
	typedef typename HyperplaneCoordinateSystem< ScalarType, Dim >::PointType
		PointType;

	void Canonical();
	template< class V >
	void FromNormal(const V &v)
	{
		m_hcs.FromNormal(v);
		for(unsigned int i = 0; i < Dim; ++i)
			m_normal[i] = v[i];
	}
	template< class GPointT, class LPointT >
	void ToLocal(const GPointT &p, LPointT *l) const;
	void ToTangent(const PointType &p, GfxTL::VectorXD< Dim - 1, ScalarType > *t) const;
	ScalarType Height(const PointType &p) const;
	template< class LPointT, class GPointT >
	void ToGlobal(const LPointT &p, GPointT *g) const;
	void RotateOnNormal(ScalarType radians);
	operator PointType *() { return reinterpret_cast< PointType * >(this); }
	operator const PointType *() const { return reinterpret_cast< const PointType * >(this); }

private:
	HyperplaneCoordinateSystem< ScalarType, Dim > m_hcs;
	PointType m_normal;
};

template< class ScalarT >
void Frame< 3, ScalarT >::Canonical()
{
	for(unsigned int i = 0; i < Dim; ++i)
	{
		for(unsigned int j = 0; j < Dim; ++j)
			(*this)[i][j] = 0;
		(*this)[i][i] = 1;
	}
}

template< class ScalarT >
 template< class GPointT, class LPointT >
void Frame< 3, ScalarT >::ToLocal(const GPointT &p, LPointT *l) const
{
	if(&p != l)
	{
		for(unsigned int i = 0; i < Dim; ++i)
			(*l)[i] = (*this)[i] * PointType(p);
	}
	else
	{
		LPointT tmp;
		for(unsigned int i = 0; i < Dim; ++i)
			tmp[i] = (*this)[i] * PointType(p);
		*l = tmp;
	}
}

template< class ScalarT >
void Frame< 3, ScalarT >::ToTangent(const PointType &p,
	GfxTL::VectorXD< Dim - 1, ScalarType > *t) const
{
	for(unsigned int i = 0; i < Dim - 1; ++i)
		(*t)[i] = (*this)[i] * p;
}

template< class ScalarT >
typename Frame< 3, ScalarT >::ScalarType
	Frame< 3, ScalarT >::Height(const PointType &p) const
{
	return p * (*this)[Dim - 1];
}

template< class ScalarT >
 template< class LPointT, class GPointT >
void Frame< 3, ScalarT >::ToGlobal(const LPointT &p, GPointT *g) const
{
	if(&p != g)
	{
		*g = ScalarType(p[0]) * (*this)[0];
		for(unsigned int i = 1; i < Dim; ++i)
			*g += GPointT(ScalarType(p[i]) * (*this)[i]);
	}
	else
	{
		GPointT tmp;
		tmp = ScalarType(p[0]) * (*this)[0];
		for(unsigned int i = 1; i < Dim; ++i)
			tmp += GPointT(ScalarType(p[i]) * (*this)[i]);
		*g = tmp;
	}
}

template< class ScalarT >
void Frame< 3, ScalarT >::RotateOnNormal(ScalarType radians)
{
	PointType newDirs[Dim - 1];
	ScalarType c = std::cos(radians), s = std::sin(radians);
	newDirs[0] = c * (*this)[0] + s * (*this)[1];
	newDirs[1] = -s * (*this)[0] + c * (*this)[1];
	(*this)[0] = newDirs[0];
	(*this)[1] = newDirs[1];
}

};

#endif
