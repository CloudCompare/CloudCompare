#ifndef __GfxTL_HYPERPLANECOORDINATESYSTEM_HEADER__
#define __GfxTL_HYPERPLANECOORDINATESYSTEM_HEADER__
#include <GfxTL/VectorXD.h>
#include <GfxTL/MathHelper.h>

namespace GfxTL
{

template< class Scalar, unsigned int D >
class HyperplaneCoordinateSystem
{
public:
	enum { Dim = D, HDim = Dim - 1 };
};

template< class ScalarT >
class HyperplaneCoordinateSystem< ScalarT, 2 >
{
public:
	enum { Dim = 2, HDim = Dim - 1 };
	typedef ScalarT ScalarType;
	typedef VectorXD< Dim, ScalarType > PointType;

	template< class V >
	void FromNormal(const V &v)
	{
		m_axis[0][0] = -v[1];
		m_axis[0][1] = v[0];
	}

	PointType &operator[](unsigned int i)
	{
		return m_axis[i];
	}

	const PointType &operator[](unsigned int i) const
	{
		return m_axis[i];
	}

private:
	PointType m_axis[Dim - 1];
};

template< class Scalar >
class HyperplaneCoordinateSystem< Scalar, 3 >
{
public:
	enum { Dim = 3, HDim = Dim - 1 };
	typedef Scalar ScalarType;
	typedef VectorXD< Dim, ScalarType > PointType;

	HyperplaneCoordinateSystem()
	{
		for(unsigned int i = 0; i < Dim - 1; ++i)
			_axis[i].Zero();
	}

	HyperplaneCoordinateSystem(ScalarType x, ScalarType y, ScalarType z)
	{
		FromNormal(PointType(x, y, z));
	}

	template< class V >
	HyperplaneCoordinateSystem(V n)
	{
		FromNormal(n);
	}

	void FromNormal(ScalarType x, ScalarType y, ScalarType z)
	{
		FromNormal(PointType(x, y, z));
	}

	template< class V >
	void FromNormal(const V &n)
	{
		FromNormal(PointType(n[0], n[1], n[2]));
	}

	void FromNormal(const PointType &n)
	{
		// perform arbitrary axis algorithm as
		// specified by AutoCAD
		if(Math< ScalarType >::Abs(n[0]) < ScalarType(0.015625f)
			&& Math< ScalarType >::Abs(n[1]) < ScalarType(0.015625f))
			_axis[0] = PointType(0, 1, 0) % n;
		else
			_axis[0] = PointType(0, 0, 1) % n;
		_axis[0].Normalize();
		_axis[1] = n % _axis[0];
		_axis[1].Normalize();
	}

	PointType &operator[](unsigned int i)
	{
		return _axis[i];
	}

	const PointType &operator[](unsigned int i) const
	{
		return _axis[i];
	}

private:
	PointType _axis[Dim - 1];
};

};

#endif
