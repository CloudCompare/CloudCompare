//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################
//
#ifndef MLSSMOOTHINGUPSAMPLING_H
#define MLSSMOOTHINGUPSAMPLING_H

#include "BaseFilter.h"

class MLSDialog;

struct MLSParameters
{
	///NOTE: DISTINCT CLOUD METHOD NOT IMPLEMENTED
	enum UpsamplingMethod { NONE, SAMPLE_LOCAL_PLANE, RANDOM_UNIFORM_DENSITY, VOXEL_GRID_DILATION };

	MLSParameters()
		: order_ (0)
		, polynomial_fit_(false)
		, search_radius_(0)
		, sqr_gauss_param_(0)
		, compute_normals_(false)
		, upsample_method_(NONE)
		, upsampling_radius_(0)
		, upsampling_step_(0)
		, step_point_density_(0)
		, dilation_voxel_size_(0)
		, dilation_iterations_(0)
	{
	}

	int order_;
	bool polynomial_fit_;
	double search_radius_;
	double sqr_gauss_param_;
	bool compute_normals_;
	UpsamplingMethod upsample_method_;
	double upsampling_radius_;
	double upsampling_step_;
	int step_point_density_;
	double dilation_voxel_size_;
	int dilation_iterations_;
};

class MLSSmoothingUpsampling : public BaseFilter
{
public:
	MLSSmoothingUpsampling();
	virtual ~MLSSmoothingUpsampling();

protected:

	//inherited from BaseFilter
	int openInputDialog();
	int compute();
	void getParametersFromDialog();

	MLSDialog* m_dialog;
	bool m_dialogHasParent;
	MLSParameters * m_parameters; //We directly store all the parameters here
};

#endif // MLSSMOOTHINGUPSAMPLING_H
