#ifndef __GfxTL_COVARIANCE_HEADER__
#define __GfxTL_COVARIANCE_HEADER__
#include <GfxTL/WeightFunc.h>
#include <GfxTL/ScalarTypeDeferer.h>
#include <GfxTL/VectorXD.h>

namespace GfxTL
{
	template< class MatrixT >
	class IncrementalCovarianceMatrix
	{
	public:
		typedef typename MatrixT::ScalarType ScalarType;
		typedef MatrixT MatrixType;
		typedef VectorXD< MatrixT::Rows, ScalarType > PointType;
		IncrementalCovarianceMatrix()
		{
			m_matrix.Zero();
			m_mean.Zero();
			m_totalWeight = 0;
		}

		template< class PointT >
		void Add(const PointT &p)
		{
			Add(1, p);
		}

		template< class WeightT, class PointT >
		void Add(WeightT w, const PointT &p)
		{
			ScalarType oldTotalWeight = m_totalWeight;
			m_totalWeight += ScalarType(w);
			PointType diff = p - m_mean;
			PointType r = (ScalarType(w) / ScalarType(m_totalWeight)) * diff;
			m_mean += r;
			m_matrix += OuterProduct(oldTotalWeight * r, diff);
		}

		void Matrix(MatrixT *m) const
		{
			*m = m_matrix;
			if(m_totalWeight > 0)
				*m /= m_totalWeight;
		}

		void Mean(PointType *mean) const
		{
			*mean = m_mean;
		}

		void MeanAndMatrix(PointType *mean, MatrixT *m) const
		{
			Mean(mean);
			Matrix(m);
		}

		void Reset()
		{
			m_matrix.Zero();
			m_mean.Zero();
			m_totalWeight = 0;
		}

	private:
		MatrixType m_matrix;
		PointType m_mean;
		ScalarType m_totalWeight;
	};

	template< class ScalarT >
	class IncrementalCovarianceMatrix< MatrixXX< 3, 3, ScalarT > >
	{
	public:
		typedef ScalarT ScalarType;
		typedef MatrixXX< 3, 3, ScalarT > MatrixType;
		typedef VectorXD< 3, ScalarType > PointType;

		IncrementalCovarianceMatrix()
		{
			m_matrix.Zero();
			m_mean.Zero();
			m_totalWeight = 0;
		}

		template< class PointT >
		void Add(const PointT &p)
		{
			Add(1, p);
		}

		template< class WeightT, class PointT >
		void Add(WeightT w, const PointT &p)
		{
			ScalarType oldTotalWeight = m_totalWeight;
			m_totalWeight += ScalarType(w);
			PointType diff = p - m_mean;
			ScalarType dummy;
			PointType r = (ScalarType(w) / m_totalWeight) * diff;
			m_mean += r;
			m_matrix[0][0] += (dummy = oldTotalWeight * diff[0]) * r[0];
			m_matrix[0][1] += dummy * r[1];
			m_matrix[0][2] += dummy * r[2];
			m_matrix[1][1] += (dummy = oldTotalWeight * diff[1]) * r[1];
			m_matrix[1][2] += dummy * r[2];
			m_matrix[2][2] += oldTotalWeight * diff[2] * r[2];
		}

		void Matrix(MatrixType *m) const
		{
			if(m_totalWeight > 0)
			{
				(*m)[0][0] = m_matrix[0][0] / m_totalWeight;
				(*m)[0][1] = m_matrix[0][1] / m_totalWeight;
				(*m)[0][2] = m_matrix[0][2] / m_totalWeight;
				(*m)[1][1] = m_matrix[1][1] / m_totalWeight;
				(*m)[1][2] = m_matrix[1][2] / m_totalWeight;
				(*m)[2][2] = m_matrix[2][2] / m_totalWeight;
			}
			else
			{
				(*m)[0][0] = m_matrix[0][0];
				(*m)[0][1] = m_matrix[0][1];
				(*m)[0][2] = m_matrix[0][2];
				(*m)[1][1] = m_matrix[1][1];
				(*m)[1][2] = m_matrix[1][2];
				(*m)[2][2] = m_matrix[2][2];
			}
			(*m)[1][0] = (*m)[0][1];
			(*m)[2][0] = (*m)[0][2];
			(*m)[2][1] = (*m)[1][2];
		}

		void Mean(PointType *mean) const
		{
			*mean = m_mean;
		}

		void MeanAndMatrix(PointType *mean, MatrixType *m) const
		{
			Mean(mean);
			Matrix(m);
		}

		void Reset()
		{
			m_matrix.Zero();
			m_mean.Zero();
			m_totalWeight = 0;
		}

	private:
		MatrixType m_matrix;
		PointType m_mean;
		ScalarType m_totalWeight;
	};

	template< class PointT, class PointsForwardIt, class WeightsForwardIt,
		class MatrixT >
	void CovarianceMatrix(const PointT &center, PointsForwardIt begin,
		PointsForwardIt end, WeightsForwardIt weights, MatrixT *matrix)
	{
		typedef typename MatrixT::ScalarType ScalarType;
		matrix->Zero();
		ScalarType totalWeight = 0;
		for(; begin != end; ++begin, ++weights)
		{
			typename PointT::TransposedType diffT;
			PointT diff = ((PointT)(*begin));
			diff -= center;
			diff.Transpose(&diffT);
			ScalarType w = ScalarType(*weights);
			totalWeight += w;
			diff *= w;
			MatrixT dd = diff * diffT;
			*matrix += dd;
		}
		*matrix /= totalWeight;
	}

	template< class PointT, class PointsForwardIt, class WeightsForwardIt >
	void CovarianceMatrix(const PointT &center, PointsForwardIt begin,
		PointsForwardIt end, WeightsForwardIt weights,
		MatrixXX< 3, 3, typename PointT::ScalarType > *matrix)
	{
		typedef typename PointT::ScalarType ScalarType;
		matrix->Zero();
		ScalarType totalWeight = 0, dummy;
		for(; begin != end; ++begin, ++weights)
		{
			PointT diff = PointT(*begin);
			diff -= center;
			ScalarType w = ScalarType(*weights);
			(*matrix)[0][0] += (dummy = w * diff[0]) * diff[0];
			(*matrix)[0][1] += dummy * diff[1];
			(*matrix)[0][2] += dummy * diff[2];
			(*matrix)[1][1] += (dummy = w * diff[1]) * diff[1];
			(*matrix)[1][2] += dummy * diff[2];
			(*matrix)[2][2] += w * diff[2] * diff[2];
			totalWeight += w;
		}
		(*matrix)[1][0] = (*matrix)[0][1];
		(*matrix)[2][0] = (*matrix)[0][2];
		(*matrix)[2][1] = (*matrix)[1][2];
		(*matrix) /= totalWeight;
	}

	template< class PointT, class PointsForwardIt, class MatrixT >
	void CovarianceMatrix(const PointT &center, PointsForwardIt begin,
		PointsForwardIt end, MatrixT *matrix)
	{
		CovarianceMatrix(center, begin, end, UnitWeightIterator(), matrix);
	}

	// one pass covariance
	template< class PointsIteratorT, class WeightsIteratorT, class PointT,
		class MatrixT >
	void CovarianceMatrix(PointsIteratorT begin, PointsIteratorT end,
		WeightsIteratorT weights, PointT *mean, MatrixT *matrix)
	{
		IncrementalCovarianceMatrix< MatrixT > cov;
		for(; begin != end; ++begin, ++weights)
			cov.Add(*weights, *begin);
		cov.MeanAndMatrix(mean, matrix);
	}

	// one pass covariance (optimized for 3x3 matrix)
	template< class PointsIteratorT, class WeightsIteratorT, class PointT,
		class ScalarT >
	void CovarianceMatrix(PointsIteratorT begin, PointsIteratorT end,
		WeightsIteratorT weights, PointT *mean,
		MatrixXX< 3, 3, ScalarT > *matrix)
	{
		typedef typename std::iterator_traits< WeightsIteratorT >
			::value_type WeightType;
		typedef typename ScalarTypeDeferer<
			typename std::iterator_traits< PointsIteratorT >::value_type >
			::ScalarType ScalarType;
		matrix->Zero();
		if(begin == end)
			return;
		WeightType w, totalWeight, oldTotalWeight;
		w = *weights;
		*mean = *begin;
		totalWeight = w;
		for(++begin, ++weights; begin != end; ++begin, ++weights)
		{
			oldTotalWeight = totalWeight;
			w = *weights;
			totalWeight += w;
			PointT diff;
			for(unsigned int i = 0; i < 3; ++i)
				diff[i] = (*begin)[i] - (*mean)[i];
			ScalarType dummy, ww;
			ww = ScalarType(oldTotalWeight);
			PointT r = (ScalarType(w) / ScalarType(totalWeight)) * diff;
			*mean += r;
			(*matrix)[0][0] += (dummy = ww * diff[0]) * r[0];
			(*matrix)[0][1] += dummy * r[1];
			(*matrix)[0][2] += dummy * r[2];
			(*matrix)[1][1] += (dummy = ww * diff[1]) * r[1];
			(*matrix)[1][2] += dummy * r[2];
			(*matrix)[2][2] += ww * diff[2] * r[2];
		}
		(*matrix)[0][0] /= ScalarType(totalWeight);
		(*matrix)[0][1] /= ScalarType(totalWeight);
		(*matrix)[0][2] /= ScalarType(totalWeight);
		(*matrix)[1][1] /= ScalarType(totalWeight);
		(*matrix)[1][2] /= ScalarType(totalWeight);
		(*matrix)[2][2] /= ScalarType(totalWeight);
		(*matrix)[1][0] = (*matrix)[0][1];
		(*matrix)[2][0] = (*matrix)[0][2];
		(*matrix)[2][1] = (*matrix)[1][2];
	}
};

#endif
