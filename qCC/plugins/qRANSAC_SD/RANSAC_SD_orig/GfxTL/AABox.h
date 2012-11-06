#ifndef __GfxTL_AABOX_HEADER__
#define __GfxTL_AABOX_HEADER__
#include <limits>
#include <algorithm>
#include <GfxTL/MathHelper.h>
#include <GfxTL/AACube.h>

namespace GfxTL
{

	// AABox - Axis aligned box in N-dimensions
	// N is deferred from Point
	// requirements for Point:
	//	Point must declare public type ScalarType
	//	Point::Dim must be the dimension
	//	Point must support array style indexing with operator[]
	template< class Point >
	class AABox
	{
		public:
			typedef Point PointType;
			typedef typename Point::ScalarType ScalarType;
			enum { Dim = PointType::Dim };

			AABox() {}
			// constructs box with given min and max corners
			AABox(const PointType &pmin, const PointType &pmax);
			AABox(const AACube< PointType > &cube) : _pmin(cube.Min()), _pmax(cube.Max()) {}
			// constructs infinitely large box, that is all elements of
			// pmin == -infinity and of pmax == infinity
			void Infinite();
			// splits the box along the plane perpendicular to axis and
			// passing through s
			// If s is not within the box left and right will be set
			// to box
			void Split(unsigned int axis, ScalarType s,
				AABox< Point > *left, AABox< Point > *right) const;
			inline void Center(Point *center) const;
			inline Point operator[](unsigned int index) const;
			inline Point &Min();
			inline const Point &Min() const;
			inline Point &Max();
			inline const Point &Max() const;
			inline ScalarType DiagLength() const;
			inline bool IsInside(const Point &p) const;
			template< class Points >
			void Bound(const Points &points, size_t size);
			template< class PointForwardIterator >
			void Bound(PointForwardIterator begin, PointForwardIterator end);
			void Bound(const PointType &p, ScalarType r);
			AABox< Point > &operator+=(const PointType &p);
			AABox< Point > &operator=(const AACube< PointType > &cube) { _pmin = cube.Min(); _pmax = cube.Max(); return *this; }
			AABox< Point > &IncrementalBound(const PointType &p);
			AABox< Point > &IncrementalBound(const PointType &p, ScalarType r);
			AABox< Point > &IncrementalBound(const AABox< Point > &box);
			template< class PointT >
			ScalarType MaxSqrDist(const PointT &p) const;
			template< class PointT >
			ScalarType MinSqrDist(const PointT &p) const;
			template< class PointT >
			ScalarType Distance(const PointT &p) const;

		private:
			PointType _pmin, _pmax;
	};

	template< class Point >
	 template< class Points >
	void AABox< Point >::Bound(const Points &points, size_t size)
	{
		for(unsigned int i = 0; i < Dim; ++i)
		{
			_pmin[i] = -std::numeric_limits< ScalarType >::infinity();
			_pmax[i] = std::numeric_limits< ScalarType >::infinity();
		}
		if(size > 0)
		{
			for(unsigned int i = 0; i < Dim; ++i)
				_pmax[i] = _pmin[i] = points[0][i];
			for(size_t i = 1; i < size; ++i)
			{
				for(unsigned int j = 0; j < Dim; ++j)
				{
					if(_pmin[j] > points[i][j])
						_pmin[j] = points[i][j];
					else if(_pmax[j] < points[i][j])
						_pmax[j] = points[i][j];
				}
			}
		}
	}

	template< class Point >
	 template< class PointForwardIterator >
	void AABox< Point >::Bound(PointForwardIterator begin,
		PointForwardIterator end)
	{
		for(unsigned int i = 0; i < Dim; ++i)
		{
			_pmin[i] = -std::numeric_limits< ScalarType >::infinity();
			_pmax[i] = std::numeric_limits< ScalarType >::infinity();
		}
		if(begin == end)
			return;
		for(unsigned int i = 0; i < Dim; ++i)
			_pmax[i] = _pmin[i] = (*begin)[i];
		for(++begin; begin != end; ++begin)
		{
			for(unsigned int j = 0; j < Dim; ++j)
			{
				if(_pmin[j] > (*begin)[j])
					_pmin[j] = (*begin)[j];
				else if(_pmax[j] < (*begin)[j])
					_pmax[j] = (*begin)[j];
			}
		}
	}

	template< class Point >
	void AABox< Point >::Bound(const PointType &p, ScalarType r)
	{
		for(unsigned int j = 0; j < Dim; ++j)
		{
			_pmin[j] = p[j] - r;
			_pmax[j] = p[j] + r;
		}
	}

	template< class Point >
	AABox< Point > &AABox< Point >::IncrementalBound(const PointType &p)
	{
		for(unsigned int i = 0; i < Dim; ++i)
		{
			if(_pmin[i] > p[i])
				_pmin[i] = p[i];
			if(_pmax[i] < p[i])
				_pmax[i] = p[i];
		}
		return *this;
	}

	template< class Point >
	AABox< Point > &AABox< Point >::IncrementalBound(const PointType &p,
		ScalarType r)
	{
		ScalarType v;
		for(unsigned int j = 0; j < Dim; ++j)
		{
			if(_pmin[j] > (v = p[j] - r))
				_pmin[j] = v;
			if(_pmax[j] < (v = p[j] + r))
				_pmax[j] = v;
		}
		return *this;
	}

	template< class Point >
	AABox< Point > &AABox< Point >::IncrementalBound(const AABox< Point > &box)
	{
		IncrementalBound(box.Min());
		IncrementalBound(box.Max());
		return *this;
	}

	template< class Point >
		template< class PointT >
	typename AABox< Point >::ScalarType
		AABox< Point >::MaxSqrDist(const PointT &p) const
	{
		ScalarType maxDist = 0, tmp;
		for(unsigned int i = 0; i < Dim; ++i)
			maxDist += (tmp = std::max(
				Math< ScalarType >::Abs(Min()[i] - p[i]),
				Math< ScalarType >::Abs(Max()[i] - p[i]))) * tmp;
		return maxDist;
	}

	template< class Point >
		template< class PointT >
	typename AABox< Point >::ScalarType
		AABox< Point >::MinSqrDist(const PointT &p) const
	{
		ScalarType sqrDist = 0, t;
		for(unsigned int i = 0; i < Dim; ++i)
		{
			if(p[i] < Min()[i])
			{
				t = Min()[i] - p[i];
				sqrDist += t * t;
			}
			else if(p[i] > Max()[i])
			{
				t = p[i] - Max()[i];
				sqrDist += t * t;
			}
		}
		return sqrDist;
	}

	template< class Point >
		template< class PointT >
	typename AABox< Point >::ScalarType
		AABox< Point >::Distance(const PointT &p) const
	{
		return std::sqrt(MinSqrDist(p));
	}
};

#include "AABox.hpp"

#endif

