#ifndef __AACUBE_HEADER__
#define __AACUBE_HEADER__
#include <algorithm>
#include <utility>
#include <limits>

namespace GfxTL
{

	template< class Point >
	class AACube
	{
		public:
			typedef Point PointType;
			typedef typename Point::ScalarType ScalarType;
			enum
			{
				Dim = Point::Dim,
				NCorners = 1 << Point::Dim
			};

			AACube();
			AACube(const Point &backBottomLeft, ScalarType width);
			AACube(const Point *points, size_t size);
			AACube(unsigned int box, const AACube< Point > &cube);

			template< class Points >
			void Bound(const Points &points, size_t size);
			template< class IteratorT >
			void Bound(IteratorT begin, IteratorT end);
			template< class IteratorT >
			void BoundNonCentered(IteratorT begin, IteratorT end);
			template< class Points >
			void BoundRotationInvariant(const Points &points, size_t size);
			void DividingPlane(unsigned int axis, Point *n,
				ScalarType *d) const;
			void DividingPlane(unsigned int axis, ScalarType *s);
			void Center(Point *c) const;
			void Center(const Point &c);
			void SubCube(unsigned int box, AACube< Point > *cube) const;
			bool IsSubCube(unsigned int *box,
				const AACube< Point > &cube) const;
			bool IsInside(const Point &p) const;
			ScalarType Width() const;
			void Width(ScalarType w);
			const Point &LeftBottomBack() const;
			void LeftBottomBack(const Point &lbb);
			Point operator[](int index) const;
			ScalarType DiagLength() const;
			void Inflate(ScalarType v);
			ScalarType Distance(const PointType &p) const;
			ScalarType SqrDistance(const PointType &p) const;
			void Translate(const PointType &t);
			void Scale(ScalarType s);
			const Point &Min() const { return _backBottomLeft; }
			Point &Min() { return _backBottomLeft; }
			Point Max() const
			{
				Point m(_backBottomLeft);
				for(unsigned int i = 0; i < Dim; ++i)
					m[i] += _width;
				return m;
			}

		private:
			Point _backBottomLeft;
			ScalarType _width;
	};

	template< class Point >
	 template< class Points >
	void AACube< Point >::Bound(const Points &points, size_t size)
	{
		_width = 0;
		if(size > 0)
		{
			Point pmax, pmin;
			for(unsigned int u = 0; u < Dim; ++u)
				pmax[u] = pmin[u] = points[0][u];
			for(size_t i = 1; i < size; ++i)
			{
				for(unsigned int j = 0; j < Dim; ++j)
				{
					if(pmin[j] > points[i][j])
						pmin[j] = points[i][j];
					else if(pmax[j] < points[i][j])
						pmax[j] = points[i][j];
				}
			}
			Point center = pmin +
				(ScalarType).5 * (pmax - pmin);
			Point r = pmax - center;
			ScalarType rmax = r[0];
			for(unsigned int u = 1; u < Dim; ++u)
				if(r[u] > rmax)
					rmax = r[u];
			_backBottomLeft = center;
			for(unsigned int u = 0; u < Dim; ++u)
				_backBottomLeft[u] -= rmax;
			_width = 2 * rmax;
		}
	}

	template< class Point >
	 template< class IteratorT >
	void AACube< Point >::Bound(IteratorT begin, IteratorT end)
	{
		_width = 0;
		if(end - begin > 0)
		{
			Point pmax, pmin;
			for(unsigned int u = 0; u < Dim; ++u)
				pmax[u] = pmin[u] = (*begin)[u];
			IteratorT i = begin;
			for(++i; i != end; ++i)
			{
				for(unsigned int j = 0; j < Dim; ++j)
				{
					if(pmin[j] > (*i)[j])
						pmin[j] = (*i)[j];
					else if(pmax[j] < (*i)[j])
						pmax[j] = (*i)[j];
				}
			}
			Point center = pmin +
				(ScalarType).5 * (pmax - pmin);
			Point r = pmax - center;
			ScalarType rmax = r[0];
			for(unsigned int u = 1; u < Dim; ++u)
				if(r[u] > rmax)
					rmax = r[u];
			_backBottomLeft = center;
			for(unsigned int u = 0; u < Dim; ++u)
				_backBottomLeft[u] -= rmax;
			_width = 2 * rmax;
		}
	}

	template< class Point >
	 template< class IteratorT >
	void AACube< Point >::BoundNonCentered(IteratorT begin, IteratorT end)
	{
		using namespace std;
		_width = 0;
		if(end - begin > 0)
		{
			Point pmax, pmin;
			for(unsigned int u = 0; u < Dim; ++u)
				pmax[u] = pmin[u] = (*begin)[u];
			IteratorT i = begin;
			for(++i; i != end; ++i)
			{
				for(unsigned int j = 0; j < Dim; ++j)
				{
					if(pmin[j] > (*i)[j])
						pmin[j] = (*i)[j];
					else if(pmax[j] < (*i)[j])
						pmax[j] = (*i)[j];
				}
			}
			_backBottomLeft = pmin;
			_width = pmax[0] - pmin[0];
			for(unsigned int i = 1; i < Dim; ++i)
				_width = max(_width, pmax[i] - pmin[i]);
		}
	}

	template< class Point >
	 template< class Points >
	void AACube< Point >::BoundRotationInvariant(const Points &points,
		size_t size)
	{
		_width = 0;
		if(size > 0)
		{
			Point pmax, pmin;
			pmax = pmin = points[0];
			for(size_t i = 1; i < size; ++i)
			{
				for(unsigned int j = 0; j < Dim; ++j)
				{
					if(pmin[j] > points[i][j])
						pmin[j] = points[i][j];
					else if(pmax[j] < points[i][j])
						pmax[j] = points[i][j];
				}
			}
			_width = (pmax - pmin).Length();
			Point center = pmin + (ScalarType).5 * (pmax - pmin);
			_backBottomLeft = center;
			for(unsigned int u = 0; u < Dim; ++u)
				_backBottomLeft[u] -= _width / 2;
		}
	}

};

#include "AACube.hpp"

#endif
