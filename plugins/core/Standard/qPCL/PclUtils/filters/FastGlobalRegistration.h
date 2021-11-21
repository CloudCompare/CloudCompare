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
#include <vector>

#pragma warning( push )
#pragma warning( disable : 4267 )
#include <flann/flann.hpp>
#pragma warning( pop )

#include <Eigen/Core>
#include <Eigen/Geometry>

#define DIV_FACTOR			1.4		// Division factor used for graduated non-convexity
#define USE_ABSOLUTE_SCALE	0		// Measure distance in absolute scale (1) or in scale relative to the diameter of the model (0)
#define MAX_CORR_DIST		0.025	// Maximum correspondence distance (also see comment of USE_ABSOLUTE_SCALE)
#define ITERATION_NUMBER	64		// Maximum number of iteration
#define TUPLE_SCALE			0.95	// Similarity measure used for tuples of feature points.
#define TUPLE_MAX_CNT		1000	// Maximum tuple numbers.

namespace fgr {
  
typedef std::vector<Eigen::Vector3f> Points;
typedef std::vector<Eigen::VectorXf> Feature;
typedef flann::Index<flann::L2<float> > KDTree;
typedef std::vector<std::pair<int, int> > Correspondences;

class CApp{
public:
	CApp(double div_factor         = DIV_FACTOR,
	    bool    use_absolute_scale = USE_ABSOLUTE_SCALE,
	    double  max_corr_dist      = MAX_CORR_DIST,
	    int     iteration_number   = ITERATION_NUMBER,
	    float   tuple_scale        = TUPLE_SCALE,
	    int     tuple_max_cnt      = TUPLE_MAX_CNT):
		div_factor_(div_factor),
		use_absolute_scale_(use_absolute_scale),
		max_corr_dist_(max_corr_dist),
		iteration_number_(iteration_number),
		tuple_scale_(tuple_scale),
		tuple_max_cnt_(tuple_max_cnt){}
	void LoadFeature(const Points& pts, const Feature& feat);
	void ReadFeature(const char* filepath);
	void NormalizePoints();
	void AdvancedMatching();
	Eigen::Matrix4f ReadTrans(const char* filepath);
	void WriteTrans(const char* filepath);
	Eigen::Matrix4f GetOutputTrans();
	double OptimizePairwise(bool decrease_mu_);
	void Evaluation(const char* gth, const char* estimation, const char *output);

private:
	// containers
	std::vector<Points> pointcloud_;
	std::vector<Feature> features_;
	Eigen::Matrix4f TransOutput_;
	std::vector<std::pair<int, int> > corres_;

	// for normalization
	Points Means;
	float GlobalScale = 1.0f;
	float StartScale = 1.0f;

	// some internal functions
	void ReadFeature(const char* filepath, Points& pts, Feature& feat);
	void TransformPoints(Points& points, const Eigen::Matrix4f& Trans);
	void BuildDenseCorrespondence(const Eigen::Matrix4f& gth, 
			Correspondences& corres);
	
	template <typename T>
	void BuildKDTree(const std::vector<T>& data, KDTree* tree);
	template <typename T>
	void SearchKDTree(KDTree* tree,
		const T& input,
		std::vector<int>& indices,
		std::vector<float>& dists,
		int nn);

	double div_factor_;
	bool   use_absolute_scale_;
	double max_corr_dist_;
	int    iteration_number_;
	float  tuple_scale_;
	int    tuple_max_cnt_;
};

}
