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
//#                                     Copyright ©                                     #
//#               RAMM laboratory, School of Geography, Beijing Normal University       #
//#                               (http://ramm.bnu.edu.cn/)                             #
//#                                                                                     #
//#                      Wuming Zhang; Jianbo Qi; Peng Wan; Hongtao Wang                #
//#                                                                                     #
//#                      contact us: 2009zwm@gmail.com; wpqjbzwm@126.com                #
//#                                                                                     #
//#######################################################################################

#ifndef _CSF_H_
#define _CSF_H_

#include "wlPointCloud.h"
#include "Cloth.h"

//system
#include <vector>
#include <string>

class ccMainAppInterface;
class QWidget;
class ccMesh;

class CSF
{
public:
	CSF(wl::PointCloud& cloud);
	virtual ~CSF() = default;

	//input PC from files
	bool readPointsFromFile(std::string filename);

	//save the ground points to file
	void saveGroundPoints(const std::vector<int>& grp, std::string path = "");
	void saveOffGroundPoints(const std::vector<int>& grp, std::string path = "");
	
	//The main program: Do filtering
	bool do_filtering(	std::vector<int>& groundIndexes,
						std::vector<int>& offGroundIndexes,
						bool exportClothMesh,
						ccMesh* &clothMesh,
						ccMainAppInterface* app = 0,
						QWidget* parent = 0);

private:
	wl::PointCloud& point_cloud;

public:
 
	struct Parameters
	{
		//parameters
		int k_nearest_points;

		bool bSloopSmooth;

		double time_step;

		double class_threshold;

		double cloth_resolution;

		int rigidness;

		int iterations;
	};
	
	Parameters params;
};

#endif