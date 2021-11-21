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

using namespace Eigen;
using namespace std;
using namespace fgr;

void CApp::ReadFeature(const char* filepath)
{
	Points pts;
	Feature feat;
	ReadFeature(filepath, pts, feat);
	LoadFeature(pts,feat);
}

void CApp::LoadFeature(const Points& pts, const Feature& feat)
{
	pointcloud_.push_back(pts);
	features_.push_back(feat);
}

void CApp::ReadFeature(const char* filepath, Points& pts, Feature& feat)
{
	printf("ReadFeature ... ");
	FILE* fid = fopen(filepath, "rb");
	int nvertex;
	fread(&nvertex, sizeof(int), 1, fid);
	int ndim;
	fread(&ndim, sizeof(int), 1, fid);

	// read from feature file and fill out pts and feat
	for (int v = 0; v < nvertex; v++)	{

		Vector3f pts_v;
		fread(&pts_v(0), sizeof(float), 3, fid);

		VectorXf feat_v(ndim);
		fread(&feat_v(0), sizeof(float), ndim, fid);

		pts.push_back(pts_v);
		feat.push_back(feat_v);
	}
	fclose(fid);
	printf("%d points with %d feature dimensions.\n", nvertex, ndim);
}

template <typename T>
void CApp::BuildKDTree(const vector<T>& data, KDTree* tree)
{
	int rows, dim;
	rows = (int)data.size();
	dim = (int)data[0].size();
	std::vector<float> dataset(rows * dim);
	flann::Matrix<float> dataset_mat(&dataset[0], rows, dim);
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < dim; j++)
			dataset[i * dim + j] = data[i][j];
	KDTree temp_tree(dataset_mat, flann::KDTreeSingleIndexParams(15));
	temp_tree.buildIndex();
	*tree = temp_tree;
}

template <typename T>
void CApp::SearchKDTree(KDTree* tree, const T& input, 
							std::vector<int>& indices,
							std::vector<float>& dists, int nn)
{
	int rows_t = 1;
	int dim = input.size();

	std::vector<float> query;
	query.resize(rows_t*dim);
	for (int i = 0; i < dim; i++)
		query[i] = input(i);
	flann::Matrix<float> query_mat(&query[0], rows_t, dim);

	indices.resize(rows_t*nn);
	dists.resize(rows_t*nn);
	flann::Matrix<int> indices_mat(&indices[0], rows_t, nn);
	flann::Matrix<float> dists_mat(&dists[0], rows_t, nn);

	tree->knnSearch(query_mat, indices_mat, dists_mat, nn, flann::SearchParams(128));
}

void CApp::AdvancedMatching()
{
	int fi = 0;
	int fj = 1;

	printf("Advanced matching : [%d - %d]\n", fi, fj);
	bool swapped = false;

	if (pointcloud_[fj].size() > pointcloud_[fi].size())
	{
		int temp = fi;
		fi = fj;
		fj = temp;
		swapped = true;
	}

	int nPti = pointcloud_[fi].size();
	int nPtj = pointcloud_[fj].size();

	///////////////////////////
	/// BUILD FLANNTREE
	///////////////////////////

	KDTree feature_tree_i(flann::KDTreeSingleIndexParams(15));
	BuildKDTree(features_[fi], &feature_tree_i);

	KDTree feature_tree_j(flann::KDTreeSingleIndexParams(15));
	BuildKDTree(features_[fj], &feature_tree_j);

	bool crosscheck = true;
	bool tuple = true;

	std::vector<int> corres_K, corres_K2;
	std::vector<float> dis;
	std::vector<int> ind;

	std::vector<std::pair<int, int> > corres;
	std::vector<std::pair<int, int> > corres_cross;
	std::vector<std::pair<int, int> > corres_ij;
	std::vector<std::pair<int, int> > corres_ji;

	///////////////////////////
	/// INITIAL MATCHING
	///////////////////////////

	std::vector<int> i_to_j(nPti, -1);
	for (int j = 0; j < nPtj; j++)
	{
		SearchKDTree(&feature_tree_i, features_[fj][j], corres_K, dis, 1);
		int i = corres_K[0];
		if (i_to_j[i] == -1)
		{
			SearchKDTree(&feature_tree_j, features_[fi][i], corres_K, dis, 1);
			int ij = corres_K[0];
			i_to_j[i] = ij;
		}
		corres_ji.push_back(std::pair<int, int>(i, j));
	}

	for (int i = 0; i < nPti; i++)
	{
		if (i_to_j[i] != -1)
			corres_ij.push_back(std::pair<int, int>(i, i_to_j[i]));
	}

	int ncorres_ij = corres_ij.size();
	int ncorres_ji = corres_ji.size();

	// corres = corres_ij + corres_ji;
	for (int i = 0; i < ncorres_ij; ++i)
		corres.push_back(std::pair<int, int>(corres_ij[i].first, corres_ij[i].second));
	for (int j = 0; j < ncorres_ji; ++j)
		corres.push_back(std::pair<int, int>(corres_ji[j].first, corres_ji[j].second));

	printf("Number of points that remain: %d\n", (int)corres.size());

	///////////////////////////
	/// CROSS CHECK
	/// input : corres_ij, corres_ji
	/// output : corres
	///////////////////////////
	if (crosscheck)
	{
		printf("\t[cross check] ");

		// build data structure for cross check
		corres.clear();
		corres_cross.clear();
		std::vector<std::vector<int> > Mi(nPti);
		std::vector<std::vector<int> > Mj(nPtj);

		int ci, cj;
		for (int i = 0; i < ncorres_ij; ++i)
		{
			ci = corres_ij[i].first;
			cj = corres_ij[i].second;
			Mi[ci].push_back(cj);
		}
		for (int j = 0; j < ncorres_ji; ++j)
		{
			ci = corres_ji[j].first;
			cj = corres_ji[j].second;
			Mj[cj].push_back(ci);
		}

		// cross check
		for (int i = 0; i < nPti; ++i)
		{
			for (int ii = 0; ii < Mi[i].size(); ++ii)
			{
				int j = Mi[i][ii];
				for (int jj = 0; jj < Mj[j].size(); ++jj)
				{
					if (Mj[j][jj] == i)
					{
						corres.push_back(std::pair<int, int>(i, j));
						corres_cross.push_back(std::pair<int, int>(i, j));
					}
				}
			}
		}
		printf("Number of points that remain after cross-check: %d\n", (int)corres.size());
	}

	///////////////////////////
	/// TUPLE CONSTRAINT
	/// input : corres
	/// output : corres
	///////////////////////////
	if (tuple)
	{
		srand(time(NULL));

		printf("\t[tuple constraint] ");
		int rand0, rand1, rand2;
		int idi0, idi1, idi2;
		int idj0, idj1, idj2;
		float scale = tuple_scale_;
		int ncorr = corres.size();
		int number_of_trial = ncorr * 100;
		std::vector<std::pair<int, int> > corres_tuple;

		int cnt = 0;
		int i;
		for (i = 0; i < number_of_trial; i++)
		{
			rand0 = rand() % ncorr;
			rand1 = rand() % ncorr;
			rand2 = rand() % ncorr;

			idi0 = corres[rand0].first;
			idj0 = corres[rand0].second;
			idi1 = corres[rand1].first;
			idj1 = corres[rand1].second;
			idi2 = corres[rand2].first;
			idj2 = corres[rand2].second;

			// collect 3 points from i-th fragment
			Eigen::Vector3f pti0 = pointcloud_[fi][idi0];
			Eigen::Vector3f pti1 = pointcloud_[fi][idi1];
			Eigen::Vector3f pti2 = pointcloud_[fi][idi2];

			float li0 = (pti0 - pti1).norm();
			float li1 = (pti1 - pti2).norm();
			float li2 = (pti2 - pti0).norm();

			// collect 3 points from j-th fragment
			Eigen::Vector3f ptj0 = pointcloud_[fj][idj0];
			Eigen::Vector3f ptj1 = pointcloud_[fj][idj1];
			Eigen::Vector3f ptj2 = pointcloud_[fj][idj2];

			float lj0 = (ptj0 - ptj1).norm();
			float lj1 = (ptj1 - ptj2).norm();
			float lj2 = (ptj2 - ptj0).norm();

			if ((li0 * scale < lj0) && (lj0 < li0 / scale) &&
				(li1 * scale < lj1) && (lj1 < li1 / scale) &&
				(li2 * scale < lj2) && (lj2 < li2 / scale))
			{
				corres_tuple.push_back(std::pair<int, int>(idi0, idj0));
				corres_tuple.push_back(std::pair<int, int>(idi1, idj1));
				corres_tuple.push_back(std::pair<int, int>(idi2, idj2));
				cnt++;
			}

			if (cnt >= tuple_max_cnt_)
				break;
		}

		printf("%d tuples (%d trial, %d actual).\n", cnt, number_of_trial, i);
		corres.clear();

		for (int i = 0; i < corres_tuple.size(); ++i)
			corres.push_back(std::pair<int, int>(corres_tuple[i].first, corres_tuple[i].second));
	}

	if (swapped)
	{
		std::vector<std::pair<int, int> > temp;
		for (int i = 0; i < corres.size(); i++)
			temp.push_back(std::pair<int, int>(corres[i].second, corres[i].first));
		corres.clear();
		corres = temp;
	}

	printf("\t[final] matches %d.\n", (int)corres.size());
	corres_ = corres;
}

// Normalize scale of points.
// X' = (X-\mu)/scale
void CApp::NormalizePoints()
{
	int num = 2;
	float scale = 0;

	Means.clear();

	for (int i = 0; i < num; ++i)
	{
		float max_scale = 0;

		// compute mean
		Vector3f mean;
		mean.setZero();

		int npti = pointcloud_[i].size();
		for (int ii = 0; ii < npti; ++ii)
		{
			Eigen::Vector3f p(pointcloud_[i][ii](0), pointcloud_[i][ii](1), pointcloud_[i][ii](2));
			mean = mean + p;
		}
		mean = mean / npti;
		Means.push_back(mean);

		printf("normalize points :: mean[%d] = [%f %f %f]\n", i, mean(0), mean(1), mean(2));

		for (int ii = 0; ii < npti; ++ii)
		{
			pointcloud_[i][ii](0) -= mean(0);
			pointcloud_[i][ii](1) -= mean(1);
			pointcloud_[i][ii](2) -= mean(2);
		}

		// compute scale
		for (int ii = 0; ii < npti; ++ii)
		{
			Eigen::Vector3f p(pointcloud_[i][ii](0), pointcloud_[i][ii](1), pointcloud_[i][ii](2));
			float temp = p.norm(); // because we extract mean in the previous stage.
			if (temp > max_scale)
				max_scale = temp;
		}

		if (max_scale > scale)
			scale = max_scale;
	}

	//// mean of the scale variation
	if (use_absolute_scale_) {
		GlobalScale = 1.0f;
		StartScale = scale;
	} else {
		GlobalScale = scale; // second choice: we keep the maximum scale.
		StartScale = 1.0f;
	}
	printf("normalize points :: global scale : %f\n", GlobalScale);

	for (int i = 0; i < num; ++i)
	{
		int npti = pointcloud_[i].size();
		for (int ii = 0; ii < npti; ++ii)
		{
			pointcloud_[i][ii](0) /= GlobalScale;
			pointcloud_[i][ii](1) /= GlobalScale;
			pointcloud_[i][ii](2) /= GlobalScale;
		}
	}
}

double CApp::OptimizePairwise(bool decrease_mu_)
{
	printf("Pairwise rigid pose optimization\n");

	double par;
	int numIter = iteration_number_;
	TransOutput_ = Eigen::Matrix4f::Identity();

	par = StartScale;

	int i = 0;
	int j = 1;

	// make another copy of pointcloud_[j].
	Points pcj_copy;
	int npcj = pointcloud_[j].size();
	pcj_copy.resize(npcj);
	for (int cnt = 0; cnt < npcj; cnt++)
		pcj_copy[cnt] = pointcloud_[j][cnt];

	if (corres_.size() < 10)
		return -1;

	std::vector<double> s(corres_.size(), 1.0);

	Eigen::Matrix4f trans;
	trans.setIdentity();

	for (int itr = 0; itr < numIter; itr++) {

		// graduated non-convexity.
		if (decrease_mu_)
		{
			if (itr % 4 == 0 && par > max_corr_dist_) {
				par /= div_factor_;
			}
		}

		const int nvariable = 6;	// 3 for rotation and 3 for translation
		Eigen::MatrixXd JTJ(nvariable, nvariable);
		Eigen::MatrixXd JTr(nvariable, 1);
		Eigen::MatrixXd J(nvariable, 1);
		JTJ.setZero();
		JTr.setZero();

		double r;
		double r2 = 0.0;

		for (int c = 0; c < corres_.size(); c++) {
			int ii = corres_[c].first;
			int jj = corres_[c].second;
			Eigen::Vector3f p, q;
			p = pointcloud_[i][ii];
			q = pcj_copy[jj];
			Eigen::Vector3f rpq = p - q;

			int c2 = c;

			float temp = par / (rpq.dot(rpq) + par);
			s[c2] = temp * temp;

			J.setZero();
			J(1) = -q(2);
			J(2) = q(1);
			J(3) = -1;
			r = rpq(0);
			JTJ += J * J.transpose() * s[c2];
			JTr += J * r * s[c2];
			r2 += r * r * s[c2];

			J.setZero();
			J(2) = -q(0);
			J(0) = q(2);
			J(4) = -1;
			r = rpq(1);
			JTJ += J * J.transpose() * s[c2];
			JTr += J * r * s[c2];
			r2 += r * r * s[c2];

			J.setZero();
			J(0) = -q(1);
			J(1) = q(0);
			J(5) = -1;
			r = rpq(2);
			JTJ += J * J.transpose() * s[c2];
			JTr += J * r * s[c2];
			r2 += r * r * s[c2];

			r2 += (par * (1.0 - sqrt(s[c2])) * (1.0 - sqrt(s[c2])));
		}

		Eigen::MatrixXd result(nvariable, 1);
		result = -JTJ.llt().solve(JTr);

		Eigen::Affine3d aff_mat;
		aff_mat.linear() = (Eigen::Matrix3d) Eigen::AngleAxisd(result(2), Eigen::Vector3d::UnitZ())
			* Eigen::AngleAxisd(result(1), Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(result(0), Eigen::Vector3d::UnitX());
		aff_mat.translation() = Eigen::Vector3d(result(3), result(4), result(5));

		Eigen::Matrix4f delta = aff_mat.matrix().cast<float>();

		trans = delta * trans;
		TransformPoints(pcj_copy, delta);

	}

	TransOutput_ = trans * TransOutput_;
	return par;
}

void CApp::TransformPoints(Points& points, const Eigen::Matrix4f& Trans)
{
	int npc = (int)points.size();
	Matrix3f R = Trans.block<3, 3>(0, 0);
	Vector3f t = Trans.block<3, 1>(0, 3);
	Vector3f temp;
	for (int cnt = 0; cnt < npc; cnt++) {
		temp = R * points[cnt] + t;
		points[cnt] = temp;
	}
}

Eigen::Matrix4f CApp::GetOutputTrans()
{
	Eigen::Matrix3f R;
	Eigen::Vector3f t;
	R = TransOutput_.block<3, 3>(0, 0);
	t = TransOutput_.block<3, 1>(0, 3);

	Eigen::Matrix4f transtemp;
	transtemp.fill(0.0f);

	transtemp.block<3, 3>(0, 0) = R;
	transtemp.block<3, 1>(0, 3) = -R*Means[1] + t*GlobalScale + Means[0];
	transtemp(3, 3) = 1;
	
	return transtemp;
}
	
void CApp::WriteTrans(const char* filepath)
{
	FILE* fid = fopen(filepath, "w");

	// Below line indicates how the transformation matrix aligns two point clouds
	// e.g. T * pointcloud_[1] is aligned with pointcloud_[0].
	// '2' indicates that there are two point cloud fragments.
	fprintf(fid, "0 1 2\n");

	Eigen::Matrix4f transtemp = GetOutputTrans();

	fprintf(fid, "%.10f %.10f %.10f %.10f\n", transtemp(0, 0), transtemp(0, 1), transtemp(0, 2), transtemp(0, 3));
	fprintf(fid, "%.10f %.10f %.10f %.10f\n", transtemp(1, 0), transtemp(1, 1), transtemp(1, 2), transtemp(1, 3));
	fprintf(fid, "%.10f %.10f %.10f %.10f\n", transtemp(2, 0), transtemp(2, 1), transtemp(2, 2), transtemp(2, 3));
	fprintf(fid, "%.10f %.10f %.10f %.10f\n", 0.0f, 0.0f, 0.0f, 1.0f);

	fclose(fid);
}

Eigen::Matrix4f CApp::ReadTrans(const char* filename)
{
	Eigen::Matrix4f temp;
	temp.fill(0);
	int temp0, temp1, temp2, cnt = 0;
	FILE* fid = fopen(filename, "r");
	while (fscanf(fid, "%d %d %d", &temp0, &temp1, &temp2) == 3)
	{
		for (int j = 0; j < 4; j++)
		{
			float a, b, c, d;
			fscanf(fid, "%f %f %f %f", &a, &b, &c, &d);
			temp(j, 0) = a;
			temp(j, 1) = b;
			temp(j, 2) = c;
			temp(j, 3) = d;
		}
	}
	return temp;
}

void CApp::BuildDenseCorrespondence(const Eigen::Matrix4f& trans, 
		Correspondences& corres)
{   
	int fi = 0;
	int fj = 1;
	Points pci = pointcloud_[fi];
	Points pcj = pointcloud_[fj];
	TransformPoints(pcj, trans);

	KDTree feature_tree_i(flann::KDTreeSingleIndexParams(15));
	BuildKDTree(pci, &feature_tree_i);
	std::vector<int> ind;
	std::vector<float> dist;
	corres.clear();
	for (int j = 0; j < pcj.size(); ++j)
	{
		SearchKDTree(&feature_tree_i, pcj[j], ind, dist, 1);
		float dist_j = sqrt(dist[0]);
		if (dist_j / GlobalScale < max_corr_dist_ / 2.0)
			corres.push_back(std::pair<int, int>(ind[0], j));
	}
}

void CApp::Evaluation(const char* gth, const char* estimation, const char *output)
{
	float inlier_ratio = -1.0f;
	float overlapping_ratio = -1.0f;

	int fi = 0;
	int fj = 1;

	std::vector<std::pair<int, int> > corres;
	Eigen::Matrix4f gth_trans = ReadTrans(gth);
	BuildDenseCorrespondence(gth_trans, corres);
	printf("Groundtruth correspondences [%d-%d] : %d\n", fi, fj, 
			(int)corres.size());

	int ncorres = corres.size();
	float err_mean = 0.0f;

	Points pci = pointcloud_[fi];
	Points pcj = pointcloud_[fj];
	Eigen::Matrix4f est_trans = ReadTrans(estimation);
	std::vector<float> error;
	error.reserve(ncorres);
	for (int i = 0; i < ncorres; ++i)
	{
		int idi = corres[i].first;
		int idj = corres[i].second;
		Eigen::Vector4f pi(pci[idi](0), pci[idi](1), pci[idi](2), 1);
		Eigen::Vector4f pj(pcj[idj](0), pcj[idj](1), pcj[idj](2), 1);
		Eigen::Vector4f pjt = est_trans*pj;
		float errtemp = (pi - pjt).norm();
		error.push_back(errtemp);
		// this is based on the RMSE defined in
		// https://en.wikipedia.org/wiki/Root-mean-square_deviation
		errtemp = errtemp * errtemp;
		err_mean += errtemp;
	}
	err_mean /= ncorres; // this is MSE = mean(d^2)
	err_mean = sqrt(err_mean); // this is RMSE = sqrt(MSE)
	printf("mean error : %0.4e\n", err_mean);

	//overlapping_ratio = (float)ncorres / min(
	//		pointcloud_[fj].size(), pointcloud_[fj].size());
	overlapping_ratio = (float)ncorres / pointcloud_[fj].size();
	
	// write errors
	FILE* fid = fopen(output, "w");
	fprintf(fid, "%d %d %e %e %e\n", fi, fj, err_mean, 
			inlier_ratio, overlapping_ratio);
	fclose(fid);
}
