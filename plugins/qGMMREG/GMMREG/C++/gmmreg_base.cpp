#include "gmmreg_base.h"

#include <assert.h>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>

#include <vcl_iostream.h>
#include <vcl_string.h>

#include "gmmreg_utils.h"
#include "utils/io_utils.h"
#include "utils/match_utils.h"
#include "utils/misc_utils.h"

using namespace gmmreg;

int Base::Initialize(	const vnl_matrix<double>& model,
						const vnl_matrix<double>& scene,
						const vnl_matrix<double>* control,
						int b_normalize )
{
	//PrepareOwnOptions and MultiScaleOptions should have been already called
	if (   PrepareInput(model, scene, control) < 0
		|| SetInitParams(0) < 0
		|| PrepareCommonOptions(b_normalize) < 0 )
	{
		return -1;
	}

	PrepareBasisKernel();

	initialized_ = true;
	return 0;
}

int Base::Initialize(const char* f_config)
{
	if (   PrepareInput(f_config) < 0
		|| SetInitParams(f_config) < 0
		|| PrepareCommonOptions(f_config) < 0
		|| PrepareOwnOptions(f_config) < 0)
	{
		return -1;
	}

	PrepareBasisKernel();

	initialized_ = true;
	return 0;
}

int Base::Run()
{
	if (!initialized_)
	{
		return -1;
	}

	vnl_vector<double> params;

	//Registration
	if (StartRegistration(params) < 0)
	{
		return -1;
	}

	//Finalizing results
	PerformTransform(params);
	DenormalizeAll();

	return 0;
}

int Base::Run(const char* f_config)
{
	if (!initialized_ && !Initialize(f_config))
	{
		return -1;
	}

	//Registration
	vnl_vector<double> params;
	if (StartRegistration(params) < 0)
	{
		return -1;
	}

	//Finalizing results
	PerformTransform(params);
	DenormalizeAll();

	//Saving results
	SaveResults(f_config);

	return 0;
}

int Base::PrepareInput(const vnl_matrix<double>& model, const vnl_matrix<double>& scene, const vnl_matrix<double>* control)
{
	try
	{
		model_ = model;
		scene_ = scene;
		if (control)
			ctrl_pts_ = *control;
		else
			ctrl_pts_ = model;
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return -1;
	}

	m_ = model_.rows();
	n_ = ctrl_pts_.rows();
	d_ = model_.cols();
	transformed_model_.set_size(m_, d_);
	s_ = scene_.rows();
	assert(scene_.cols() == d_);

	return 0;
}

int Base::PrepareInput(const char* f_config)
{
	char f_model[256] = {0}, f_scene[256] = {0};
	GetPrivateProfileString(common_section_, "model", NULL, f_model, 256, f_config);
	if (LoadMatrixFromTxt(f_model, model_) < 0)
	{
		return -1;
	}
	GetPrivateProfileString(common_section_, "scene", NULL, f_scene, 256, f_config);
	if (LoadMatrixFromTxt(f_scene, scene_) < 0)
	{
		return -1;
	}

	m_ = model_.rows();
	d_ = model_.cols();
	transformed_model_.set_size(m_, d_);
	s_ = scene_.rows();
	assert(scene_.cols() == d_);

	return 0;
}

int Base::SetCtrlPts(const char* filename)
{
	if (!filename || strlen(filename) == 0)
	{
		std::cout << "The control point set is not specified, "
			"the model points are used as control points." << std::endl;
		ctrl_pts_ = model_;
		n_ = ctrl_pts_.rows();
		return n_;
	}
	else
	{
		if (LoadMatrixFromTxt(filename, ctrl_pts_) < 0)
		{
			return -1;
		}
		assert(ctrl_pts_.cols() == d_);
		n_ = ctrl_pts_.rows();
		return n_;
	}
}

void Base::DenormalizeAll() {
	if (b_normalize_) {
		Denormalize(transformed_model_, scene_centroid_, scene_scale_);
		Denormalize(model_, scene_centroid_, scene_scale_);
		Denormalize(scene_, scene_centroid_, scene_scale_);
	}
}

void Base::SaveTransformed(const char* filename, const char* f_config)
{
	std::ofstream outfile(filename, std::ios_base::out);
	transformed_model_.print(outfile);

	char section_correspondence[256] = "CORRESPONDENCE";
	int num = GetPrivateProfileInt(section_correspondence,
		"num_of_thresholds", 0, f_config);
	if (num > 0) {
		char s_min[256], s_max[256], s_pairs[256];
		GetPrivateProfileString(section_correspondence,
			"min_threshold", NULL, s_min, 255, f_config);
		GetPrivateProfileString(section_correspondence,
			"max_threshold", NULL, s_max, 255, f_config);
		GetPrivateProfileString(section_correspondence,
			"matched_pairs", NULL, s_pairs, 255, f_config);
		std::ofstream f_pair(s_pairs, std::ios_base::out);
		double min_threshold, max_threshold, interval;
		min_threshold = atof(s_min);
		max_threshold = atof(s_max);
		if (num == 1) {
			interval = 0.0f;
		} else {
			interval = (max_threshold - min_threshold) / (num - 1);
		}
		//vnl_matrix<double> working_M, working_S;
		vnl_matrix<double> dist;
		vnl_matrix<int> pairs;
		ComputeSquaredDistanceMatrix(transformed_model_, scene_, dist);
		for (int i = 0; i < num; ++i) {
			double threshold  = min_threshold + i * interval;
			//int n_match = find_working_pair(model, scene, transformed_model,
			//                                threshold, working_M, working_S);
			PickIndices<double>(dist, pairs, threshold * threshold);
			//printf("%f : %d\n",threshold, n_match);
			f_pair << "distance threshold : " << threshold << std::endl;
			f_pair << "# of matched point pairs : " << pairs.cols() << std::endl;
			unsigned j;
			for (j = 0; j < pairs.cols(); ++j) {
				f_pair.width(6);
				f_pair << std::left << pairs(0, j);
			}
			f_pair << std::endl;
			for (j = 0; j < pairs.cols(); ++j) {
				f_pair.width(6);
				f_pair << std::left << pairs(1, j);
			}
			f_pair << std::endl;
		}
	}
	std::cout << "Please find the transformed model set in "
		<< filename << std::endl;
}

int Base::PrepareCommonOptions(int b_normalize)
{
	b_normalize_ = b_normalize;
	if (b_normalize_)
	{
		Normalize(model_, model_centroid_, model_scale_);
		Normalize(scene_, scene_centroid_, scene_scale_);
		Normalize(ctrl_pts_, model_centroid_, model_scale_);
	}

	return 0;
}

int Base::PrepareCommonOptions(const char* f_config)
{
	int b_normalize = GetPrivateProfileInt(section_, "normalize", 1, f_config);
	return PrepareCommonOptions(b_normalize);
}

int Base::MultiScaleOptions(unsigned int level,
							const std::vector<float>& v_scale,
							const std::vector<int>& v_func_evals)
{
	if (v_scale.size() < level || v_func_evals.size() < level)
	{
		std::cerr << " too many levels " << std::endl;
		return -1;
	}

	try
	{
		level_ = level;
		v_scale_ = v_scale;
		v_func_evals_ = v_func_evals;
	}
	catch (const std::bad_alloc&)
	{
		std::cerr << " not enough memory " << std::endl;
		return -1;
	}

	return 0;
}

int Base::MultiScaleOptions(const char* f_config)
{
	if (f_config)
	{
		unsigned level = GetPrivateProfileInt(section_, "level", 1, f_config);

		static const char delims[] = " -,;";
		
		char s_scale[256] = { 0 };
		GetPrivateProfileString(section_, "sigma", NULL, s_scale, 255, f_config);
		std::vector<float> v_scale;
		utils::parse_tokens(s_scale, delims, v_scale);
		
		char s_func_evals[256] = { 0 };
		GetPrivateProfileString(section_, "max_function_evals", NULL, s_func_evals, 255, f_config);
		std::vector<int> v_func_evals;
		utils::parse_tokens(s_func_evals, delims, v_func_evals);

		return MultiScaleOptions(level, v_scale, v_func_evals);
	}
	else
	{
		assert(false);
		std::cerr << " no ini file specified " << std::endl;
		return -1;
	}
}
