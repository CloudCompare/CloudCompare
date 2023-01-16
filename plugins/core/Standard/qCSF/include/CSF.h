#pragma once

//#######################################################################################
//#                                                                                     #
//#                              CLOUDCOMPARE PLUGIN: qCSF                              #
//#                                                                                     #
//#        This program is free software; you can redistribute it and/or modify         #
//#        it under the terms of the GNU General Public License as published by         #
//#        the Free Software Foundation; version 2 or later of the License.             #
//#                                                                                     #
//#        This program is distributed in the hope that it will be useful,              #
//#        but WITHOUT ANY WARRANTY; without even the implied warranty of               #
//#        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                 #
//#        GNU General Public License for more details.                                 #
//#                                                                                     #
//#        Please cite the following paper, If you use this plugin in your work.        #
//#                                                                                     #
//#  Zhang W, Qi J, Wan P, Wang H, Xie D, Wang X, Yan G. An Easy-to-Use Airborne LiDAR  #
//#  Data Filtering Method Based on Cloth Simulation. Remote Sensing. 2016; 8(6):501.   #
//#                                                                                     #
//#                                     Copyright �                                     #
//#               RAMM laboratory, School of Geography, Beijing Normal University       #
//#                               (http://ramm.bnu.edu.cn/)                             #
//#                                                                                     #
//#                      Wuming Zhang; Jianbo Qi; Peng Wan; Hongtao Wang                #
//#                                                                                     #
//#                      contact us: 2009zwm@gmail.com; wpqjbzwm@126.com                #
//#                                                                                     #
//#######################################################################################

#include "wlPointCloud.h"
#include "Cloth.h"

//system
#include <vector>
#include <string>

class ccMainAppInterface;
class ccPointCloud;
class QWidget;
class ccMesh;

class CSF
{
public:

	//! Parameters
	struct Parameters
	{
		int k_nearest_points = 1;
		bool smoothSlope = true;
		double time_step = 0.65;
		double class_threshold = 0.5;
		double cloth_resolution = 1.5;
		int rigidness = 3;
		int iterations = 500;

		//  constants
		double clothYHeight = 0.05; // origin cloth height
		int clothBuffer = 2; // cloth buffer (grid margin size)
		double gravity = 0.2;
	};

	CSF(wl::PointCloud& cloud, const Parameters& params);
	virtual ~CSF() = default;

	//! The main program: Do filtering
	bool do_filtering(	std::vector<unsigned>& groundIndexes,
						std::vector<unsigned>& offGroundIndexes,
						bool exportClothMesh,
						ccMesh* &clothMesh,
						ccMainAppInterface* app = nullptr,
						QWidget* parent = nullptr);

	//! Shortcut for CloudCompare clouds
	/** \return the ground and off-ground clouds if successful
	**/
	static bool Apply(	ccPointCloud* cloud,
						const Parameters& params,
						ccPointCloud*& groundCloud,
						ccPointCloud*& offGroundCloud,
						bool exportClothMesh,
						ccMesh*& clothMesh,
						ccMainAppInterface* app = nullptr);

public:

private:
	Parameters m_params;
	wl::PointCloud& m_pointCloud;
};
