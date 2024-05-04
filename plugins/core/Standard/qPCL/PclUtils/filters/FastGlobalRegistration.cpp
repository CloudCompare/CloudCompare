// ----------------------------------------------------------------------------
// -                       Fast Global Registration                           -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) Intel Corporation 2016
// Qianyi Zhou <Qianyi.Zhou@gmail.com>
// Jaesik Park <syncle@gmail.com>
// Vladlen Koltun <vkoltun@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include "FastGlobalRegistration.h"

#pragma warning( push )
#pragma warning( disable : 4267 )
#pragma warning( disable : 4244 )
#include <flann/flann.hpp>
#pragma warning( pop )

#include <Eigen/Geometry>

#include <ccLog.h>

namespace fgr {

	typedef flann::Index<flann::L2<float>> KDTree;
	typedef std::vector<int> Indexes;
	typedef std::vector<float> Distances;
}

using namespace Eigen;
using namespace std;
using namespace fgr;

static constexpr int ReferenceIndex = 0; //reference index
static constexpr int AlignedIndex = 1; //aligned index

void CApp::LoadFeature(const Points& pts, const Features& feat)
{
	pointClouds_.reserve(pointClouds_.size() + 1);
	pointClouds_.push_back(pts);
	features_.reserve(features_.size() + 1);
	features_.push_back(feat);
}

template <typename T>
static fgr::KDTree BuildKDTree(const vector<T>& data)
{
	const size_t rows = data.size();
	const size_t dim = data.front().size();

	Distances dataset(rows * dim);
	for (size_t i = 0; i < rows; i++)
		for (size_t j = 0; j < dim; j++)
			dataset[i * dim + j] = static_cast<float>(data[i][j]);

	flann::Matrix<float> dataset_mat(dataset.data(), rows, dim);
	KDTree tree(dataset_mat, flann::KDTreeSingleIndexParams(15));
	tree.buildIndex();
	
	return tree;
}

template <typename T>
static void SearchKDTree(	fgr::KDTree& tree,
							const T& input, 
							int& nearestIndex,
							float& smallestSquareDist)
{
	static const size_t Rows = 1;
	static const size_t NN = 1;
	const size_t cols = static_cast<size_t>(input.size());

	Distances query(Rows * cols);
	for (size_t i = 0; i < cols; i++)
		query[i] = input(i);
	flann::Matrix<float> query_mat(query.data(), Rows, cols);

	flann::Matrix<int> indices_mat(&nearestIndex, Rows, NN);
	flann::Matrix<float> dists_mat(&smallestSquareDist, Rows, NN);

	tree.knnSearch(query_mat, indices_mat, dists_mat, NN, flann::SearchParams(128));
}

static void TransformPoints(fgr::Points& points, const Eigen::Matrix4f& Trans)
{
	size_t npc = points.size();

	Matrix3f R = Trans.block<3, 3>(0, 0);
	Vector3f t = Trans.block<3, 1>(0, 3);

	for (size_t cnt = 0; cnt < npc; cnt++)
	{
		Vector3f temp = R * points[cnt] + t;
		points[cnt] = temp;
	}
}

static bool Similar(	const Eigen::Vector3f& ACloud1, const Eigen::Vector3f& BCloud1,
						const Eigen::Vector3f& ACloud2, const Eigen::Vector3f& BCloud2,
						double maxRelativeScaleDiff)
{
	float ABCloud1 = (BCloud1 - ACloud1).norm();
	float ABCloud2 = (BCloud2 - ACloud2).norm();
	return (maxRelativeScaleDiff * ABCloud1 < ABCloud2) && (ABCloud2 * maxRelativeScaleDiff < ABCloud1);
}

void CApp::AdvancedMatching(bool crossCheck/*=true*/, bool tupleConstraint/*=true*/)
{
	correspondances_.clear();

	// choose the cloud order (largest first)
	int cloud1Index = ReferenceIndex;
	int cloud2Index = AlignedIndex;
	bool swapped = false;

	if (pointClouds_[cloud2Index].size() > pointClouds_[cloud1Index].size())
	{
		std::swap(cloud1Index, cloud2Index);
		swapped = true;
	}

	///////////////////////////
	/// BUILD FLANNTREE
	///////////////////////////

	const Features& cloud1Features = features_[cloud1Index];
	const Features& cloud2Features = features_[cloud2Index];

	KDTree feature_tree_cloud1 = BuildKDTree(cloud1Features);
	KDTree feature_tree_cloud2 = BuildKDTree(cloud2Features);

	///////////////////////////
	/// INITIAL MATCHING
	///////////////////////////

	Correspondences corres;
	{
		const size_t cloud1Size = pointClouds_[cloud1Index].size();
		const size_t cloud2Size = pointClouds_[cloud2Index].size();

		// First, look at correspondances from cloud2 to cloud1
		Indexes cloud1_to_cloud2(cloud1Size, -1);
		for (int indexInC2 = 0; indexInC2 < cloud2Size; ++indexInC2)
		{
			int nearestIndexInC1 = -1;
			float squareDistance = 0; //not used
			SearchKDTree(feature_tree_cloud1, cloud2Features[indexInC2], nearestIndexInC1, squareDistance);
			assert(nearestIndexInC1 != -1);

			// Then check the 'backward' correspondance starting from the nearest point in cloud1 (if not already done)
			if (cloud1_to_cloud2[nearestIndexInC1] == -1)
			{
				int nearestIndexInC2 = -1;
				SearchKDTree(feature_tree_cloud2, cloud1Features[nearestIndexInC1], nearestIndexInC2, squareDistance);
				assert(nearestIndexInC2 != -1);
				cloud1_to_cloud2[nearestIndexInC1] = nearestIndexInC2;
				
				if (!crossCheck)
				{
					// add the corresponding 'backward' correspondance from cloud1 to cloud2 to the global correspondances list
					corres.push_back(Pair(nearestIndexInC1, nearestIndexInC2));
				}
			}

			if (!crossCheck || indexInC2 == cloud1_to_cloud2[nearestIndexInC1])
			{
				// add the 'forward' correspondance from cloud2 to cloud1
				corres.push_back(Pair(nearestIndexInC1, indexInC2));
			}
		}

		corres.shrink_to_fit();
	}

	ccLog::Print(QString("Number of pairs %1 cross-check: %2").arg(crossCheck ? "with" : "without").arg(corres.size()));

	///////////////////////////
	/// TUPLE CONSTRAINT
	///////////////////////////
	if (tupleConstraint)
	{
		srand(static_cast<unsigned>(time(NULL)));

		const size_t ncorr = corres.size();
		const size_t number_of_trial = ncorr * 100;

		const Points& cloud1 = pointClouds_[cloud1Index];
		const Points& cloud2 = pointClouds_[cloud2Index];

		size_t cnt = 0;
		Correspondences corres_tuple;
		corres_tuple.reserve(tuple_max_cnt_ * 3);
		for (size_t i = 0; i < number_of_trial; i++)
		{
			size_t rand0 = (static_cast<size_t>(rand()) % ncorr);
			size_t rand1 = (static_cast<size_t>(rand()) % ncorr);
			if (rand1 == rand0)
			{
				continue;
			}
			size_t rand2 = (static_cast<size_t>(rand()) % ncorr);
			if (rand2 == rand0 || rand2 == rand1)
			{
				continue;
			}

			const Pair& pair0 = corres[rand0];
			const Pair& pair1 = corres[rand1];
			const Pair& pair2 = corres[rand2];

			// collect 3 points from first cloud
			const Eigen::Vector3f& ptCloud1_0 = cloud1[pair0.first];
			const Eigen::Vector3f& ptCloud1_1 = cloud1[pair1.first];
			const Eigen::Vector3f& ptCloud1_2 = cloud1[pair2.first];

			// collect the 3 corresponding points from second cloud
			const Eigen::Vector3f& ptCloud2_0 = cloud2[pair0.second];
			const Eigen::Vector3f& ptCloud2_1 = cloud2[pair1.second];
			const Eigen::Vector3f& ptCloud2_2 = cloud2[pair2.second];

			if (   Similar(ptCloud1_0, ptCloud1_1, ptCloud2_0, ptCloud2_1, tuple_scale_)
				&& Similar(ptCloud1_0, ptCloud1_2, ptCloud2_0, ptCloud2_2, tuple_scale_)
				&& Similar(ptCloud1_1, ptCloud1_2, ptCloud2_1, ptCloud2_2, tuple_scale_))
			{
				//confirm the pairs
				corres_tuple.push_back(Pair(pair0.first, pair0.second));
				corres_tuple.push_back(Pair(pair1.first, pair1.second));
				corres_tuple.push_back(Pair(pair2.first, pair2.second));

				if (++cnt >= tuple_max_cnt_)
				{
					break;
				}
			}
		}
		corres_tuple.shrink_to_fit();
		corres.swap(corres_tuple);

		ccLog::Print("Number of pairs after tuple constraint: " + QString::number(corres.size()));
	}

	if (swapped)
	{
		for (Pair& pair : corres)
		{
			std::swap(pair.first, pair.second);
		}
	}

	correspondances_.swap(corres);
}

void CApp::NormalizePoints()
{
	float scale = 0;

	for (int i = 0; i < 2; ++i)
	{
		// compute mean
		Vector3f mean(0, 0, 0);

		size_t npti = pointClouds_[i].size();
		for (size_t ii = 0; ii < npti; ++ii)
		{
			mean += pointClouds_[i][ii];
		}
		mean = mean / npti;
		means_[i] = mean;

		//printf("normalize points :: mean[%d] = [%f %f %f]\n", i, mean(0), mean(1), mean(2));

		for (size_t ii = 0; ii < npti; ++ii)
		{
			pointClouds_[i][ii] -= mean;
		}

		// compute scale
		float maxNorm = 0;
		for (size_t ii = 0; ii < npti; ++ii)
		{
			float norm = pointClouds_[i][ii].norm(); // because we extract mean in the previous stage.
			if (norm > maxNorm)
				maxNorm = norm;
		}

		if (maxNorm > scale)
			scale = maxNorm;
	}

	//// mean of the scale variation
	if (use_absolute_scale_)
	{
		globalScale_ = 1.0f;
		startScale_ = scale;
	}
	else
	{
		globalScale_ = scale; // second choice: we keep the maximum scale.
		startScale_ = 1.0f;
	}
	//printf("normalize points :: global scale : %f\n", globalScale_);

	for (int i = 0; i < 2; ++i)
	{
		size_t npti = pointClouds_[i].size();
		for (size_t ii = 0; ii < npti; ++ii)
		{
			pointClouds_[i][ii] /= globalScale_;
		}
	}
}

bool CApp::OptimizePairwise(bool decrease_mu)
{
	//printf("Pairwise rigid pose optimization\n");
	transOutput_ = Eigen::Matrix4f::Identity();

	if (correspondances_.size() < 10)
		return false;

	// make another copy of pointClouds_[AlignedIndex].
	Points pcj_copy = pointClouds_[AlignedIndex];

	std::vector<double> s(correspondances_.size(), 1.0);

	Eigen::Matrix4f trans;
	trans.setIdentity();
	double par = startScale_;

	for (size_t itr = 0; itr < iteration_number_; itr++)
	{
		// graduated non-convexity.
		if (decrease_mu)
		{
			if ((itr % 4) == 0 && (par > max_corr_dist_))
			{
				par /= div_factor_;
			}
		}

		static const int NVariable = 6;	// 3 for rotation and 3 for translation
		Eigen::MatrixXd JTJ(NVariable, NVariable);
		JTJ.setZero();
		Eigen::MatrixXd JTr(NVariable, 1);
		JTr.setZero();

		//double r2 = 0.0;

		for (int c = 0; c < correspondances_.size(); c++)
		{
			int ii = correspondances_[c].first;
			int jj = correspondances_[c].second;
			const Eigen::Vector3f& p = pointClouds_[ReferenceIndex][ii];
			const Eigen::Vector3f& q = pcj_copy[jj];
			Eigen::Vector3f rpq = p - q;

			double temp = par / (rpq.dot(rpq) + par);
			s[c] = temp * temp;

			Eigen::MatrixXd J(NVariable, 1);
			J.setZero();
			J(1) = -q(2);
			J(2) = q(1);
			J(3) = -1;
			double r = rpq(0);
			JTJ += J * J.transpose() * s[c];
			JTr += J * r * s[c];
			//r2 += r * r * s[c];

			J.setZero();
			J(2) = -q(0);
			J(0) = q(2);
			J(4) = -1;
			r = rpq(1);
			JTJ += J * J.transpose() * s[c];
			JTr += J * r * s[c];
			//r2 += r * r * s[c];

			J.setZero();
			J(0) = -q(1);
			J(1) = q(0);
			J(5) = -1;
			r = rpq(2);
			JTJ += J * J.transpose() * s[c];
			JTr += J * r * s[c];
			//r2 += r * r * s[c];

			//r2 += (par * (1.0 - sqrt(s[c])) * (1.0 - sqrt(s[c])));
		}

		Eigen::MatrixXd result = -JTJ.llt().solve(JTr);

		Eigen::Affine3d aff_mat;
		aff_mat.linear() = static_cast<Eigen::Matrix3d>(  Eigen::AngleAxisd(result(2), Eigen::Vector3d::UnitZ())
														* Eigen::AngleAxisd(result(1), Eigen::Vector3d::UnitY())
														* Eigen::AngleAxisd(result(0), Eigen::Vector3d::UnitX()));
		aff_mat.translation() = Eigen::Vector3d(result(3), result(4), result(5));

		Eigen::Matrix4f delta = aff_mat.matrix().cast<float>();

		trans = delta * trans;
		TransformPoints(pcj_copy, delta);
	}

	transOutput_ = trans * transOutput_;

	return true;
}

Eigen::Matrix4f CApp::GetOutputTrans() const
{
	Eigen::Matrix3f R = transOutput_.block<3, 3>(0, 0);
	Eigen::Vector3f t = transOutput_.block<3, 1>(0, 3);

	Eigen::Matrix4f transTemp;
	transTemp.fill(0.0f);

	transTemp.block<3, 3>(0, 0) = R;
	transTemp.block<3, 1>(0, 3) = -R*means_[AlignedIndex] + t*globalScale_ + means_[ReferenceIndex];
	transTemp(3, 3) = 1;
	
	return transTemp;
}
