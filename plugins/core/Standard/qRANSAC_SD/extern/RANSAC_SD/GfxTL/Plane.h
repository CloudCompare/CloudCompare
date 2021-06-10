#ifndef __GfxTL_PLANE_HEADER__
#define __GfxTL_PLANE_HEADER__
#include <GfxTL/Covariance.h>
#include <GfxTL/Jacobi.h>
#include <GfxTL/Mean.h>
#include <GfxTL/MatrixXX.h>
#include <GfxTL/VectorXD.h>
#include <GfxTL/MathHelper.h>
#include <GfxTL/WeightFunc.h>

namespace GfxTL
{
	template< class PointT >
	class Plane
	{
		public:
			typedef PointT PointType;
			typedef typename PointType::ScalarType ScalarType;

			Plane();
			Plane(const PointType &normal, const PointType &origin);
			Plane(PointType n, ScalarType d)
			: _normal(n)
			, _d(d)
			{}
			void Set(const PointType &origin, const PointType &normal);
			template< class PointsForwardIt, class WeightsForwardIt >
			bool Fit(const PointType &origin, PointsForwardIt begin,
				PointsForwardIt end, WeightsForwardIt weights);
			template< class PointsForwardIt >
			bool Fit(const PointType &origin, PointsForwardIt begin,
				PointsForwardIt end);
			template< class PointsForwardIt, class WeightsForwardIt >
			bool Fit(PointsForwardIt begin, PointsForwardIt end,
				WeightsForwardIt weights);
			template< class PointsForwardIt >
			bool Fit(PointsForwardIt begin, PointsForwardIt end);
			ScalarType SignedDistance(const PointType &p) const;
			ScalarType Distance(const PointType &p) const { return Math< ScalarType >::Abs(SignedDistance(p)); }
			ScalarType SignedDistanceToOrigin() const { return -_d; }
			void Orient(const PointType &n);
			void Project(const PointType &p, PointType *s) const;
			void Normal(PointType *normal) const;
			void Normal(const PointType &normal);
			PointType &Normal();
			const PointType &Normal() const;
			void Origin(const PointType &o);
			ScalarType Intersect(const PointType &p, const PointType &r) const;
			void OffsetInNormalDirection(ScalarType offset);

		private:
			PointType _normal;
			ScalarType _d;
	};

	template< class PointT >
	 template< class PointsForwardIt, class WeightsForwardIt >
	bool Plane< PointT >::Fit(const PointType &origin, PointsForwardIt begin,
		PointsForwardIt end, WeightsForwardIt weights)
	{
		MatrixXX< PointType::Dim, PointType::Dim, ScalarType > c, v;
		CovarianceMatrix(origin, begin, end, weights, &c);
		VectorXD< PointType::Dim, ScalarType > d;
		if(!Jacobi(c, &d, &v))
		{
			//std::cout << "Jacobi failed:" << std::endl;
			//std::cout << "origin = " << origin[0] << "," << origin[1] << "," << origin[2] << std::endl
			//	<< "cov:" << std::endl
			//	<< c[0][0] << c[1][0] << c[2][0] << std::endl
			//	<< c[0][1] << c[1][1] << c[2][1] << std::endl
			//	<< c[0][2] << c[1][2] << c[2][2] << std::endl;
			//std::cout << "recomp origin:" << std::endl;
			//PointT com;
			//Mean(begin, end, weights, &com);
			//std::cout << "origin = " << origin[0] << "," << origin[1] << "," << origin[2] << std::endl;
			//std::cout << "recomp covariance:" << std::endl;
			//CovarianceMatrix(com, begin, end, weights, &c);
			//std::cout << "cov:" << std::endl
			//<< c[0][0] << c[1][0] << c[2][0] << std::endl
			//<< c[0][1] << c[1][1] << c[2][1] << std::endl
			//<< c[0][2] << c[1][2] << c[2][2] << std::endl;
			//std::cout << "weights and points:" << std::endl;
			//WeightsForwardIt w = weights;
			//for(PointsForwardIt i = begin; i != end; ++i, ++w)
			//	std::cout << (*i)[0] << "," << (*i)[1] << "," << (*i)[2]
			//		<< " weight=" << (*w) << std::endl;
			return false;
		}
		for(unsigned int i = 0; i < PointType::Dim; ++i)
			d[i] = Math< ScalarType >::Abs(d[i]);
		EigSortDesc(&d, &v);
		_normal = PointType(v[PointType::Dim - 1]);
		_d = -(_normal * origin);
		return true;
	}

	template< class PointT >
	 template< class PointsForwardIt >
	bool Plane< PointT >::Fit(const PointType &origin, PointsForwardIt begin,
		PointsForwardIt end)
	{
		return Fit(origin, begin, end, UnitWeightIterator());
	}

	template< class PointT >
	 template< class PointsForwardIt, class WeightsForwardIt >
	bool Plane< PointT >::Fit(PointsForwardIt begin,
		PointsForwardIt end, WeightsForwardIt weights)
	{
		// copy weights
		//std::vector< ScalarType > ww(end - begin);
		//for(unsigned int i = 0; i < ww.size(); ++i)
		//	ww[i] = weights[i];
		PointT com;
		Mean(begin, end, weights, &com);
		bool retVal = Fit(com, begin, end, weights);
		// cmp weights
		//WeightsForwardIt w = weights;
		//for(unsigned int i = 0; i < ww.size(); ++i, ++w)
		//	if(ww[i] != *w)
		//		std::cout << "WEIGHT CHANGED!!! " << ww[i] << "!=" << *w << std::endl;
		return retVal;
		//MatrixXX< PointType::Dim, PointType::Dim, ScalarType > c, v;
		//PointType origin;
		//CovarianceMatrix(begin, end, weights, &origin, &c);
		//VectorXD< PointType::Dim, ScalarType > d;
		//if(!Jacobi(c, &d, &v))
		//	return false;
		//for(unsigned int i = 0; i < PointType::Dim; ++i)
		//	d[i] = Math< ScalarType >::Abs(d[i]);
		//EigSortDesc(&d, &v);
		//_normal = PointType(v[PointType::Dim - 1]);
		//_normal.Normalize();
		//_d = -(_normal * origin);
		//return true;
	}

	template< class PointT >
	 template< class PointsForwardIt >
	bool Plane< PointT >::Fit(PointsForwardIt begin, PointsForwardIt end)
	{
		return Fit(begin, end, UnitWeightIterator());
	}

	template< class PointT >
	void Plane< PointT >::OffsetInNormalDirection(ScalarType offset)
	{
		_d += offset;
	}
};

#include "Plane.hpp"

#endif

