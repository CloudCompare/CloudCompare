#ifndef __GfxTL_MEAN_HEADER__
#define __GfxTL_MEAN_HEADER__
#include <GfxTL/WeightFunc.h>
#include <GfxTL/Covariance.h>
#include <GfxTL/MatrixXX.h>
#include <GfxTL/VectorXD.h>
#include <GfxTL/Jacobi.h>
#include <GfxTL/MathHelper.h>
#include <GfxTL/WeightFunc.h>

namespace GfxTL
{
	template< class PointT, class PointsForwardIt, class WeightsForwardIt >
	void Mean(PointsForwardIt begin, PointsForwardIt end,
		WeightsForwardIt weights, PointT *mean)
	{
		mean->Zero();
		typename PointT::ScalarType totalWeight = 0;
		for(; begin != end; ++begin, ++weights)
		{
			*mean += typename PointT::ScalarType(*weights)
				* ((const PointT)(*begin));
			totalWeight += *weights;
		}
		if(totalWeight)
			*mean /= totalWeight;
	}

	template< class PointsForwardIt, class WeightsForwardIt,
		class ScalarT >
	void Mean(PointsForwardIt begin, PointsForwardIt end,
		WeightsForwardIt weights, VectorXD< 3, ScalarT > *mean)
	{
		mean->Zero();
		ScalarT totalWeight = 0, w;
		for(; begin != end; ++begin, ++weights)
		{
			w = (ScalarT)*weights;
			(*mean)[0] += w * (*begin)[0];
			(*mean)[1] += w * (*begin)[1];
			(*mean)[2] += w * (*begin)[2];
			totalWeight += w;
		}
		if(totalWeight)
			*mean /= totalWeight;
	}

	template< class PointT, class PointsForwardIt >
	void Mean(PointsForwardIt begin, PointsForwardIt end, PointT *mean)
	{
		Mean(begin, end, UnitWeightIterator(), mean);
	}

	// This computes an average of unoriented normals
	template< class NormalsItT, class WeightsItT, class MeanT >
	bool MeanOfNormals(NormalsItT begin, NormalsItT end, WeightsItT weights,
		MeanT *mean)
	{
		typedef typename MeanT::ScalarType ScalarType;
		enum { Dim = MeanT::Dim };
		GfxTL::MatrixXX< Dim, Dim, ScalarType > cov, eigenVectors;
		GfxTL::VectorXD< Dim, ScalarType > center, eigenValues;
		center.Zero();
		CovarianceMatrix(center, begin, end, weights, &cov);
		if(!Jacobi(cov, &eigenValues, &eigenVectors))
		{
			mean->Zero();
			return false;
		}
		// find the maximal eigenvalue and corresponding vector
		ScalarType maxEigVal = eigenValues[0];
		unsigned int maxEigIdx = 0;
		for(unsigned int i = 1; i < Dim; ++i)
			if(eigenValues[i] > maxEigVal)
			{
				maxEigVal = eigenValues[i];
				maxEigIdx = i;
			}
		*mean = MeanT(eigenVectors[maxEigIdx]);
		return true;
	}

	template< class NormalsItT, class MeanT >
	bool MeanOfNormals(NormalsItT begin, NormalsItT end, MeanT *mean)
	{
		return MeanOfNormals(begin, end, UnitWeightIterator(), mean);
	}
};

#endif
