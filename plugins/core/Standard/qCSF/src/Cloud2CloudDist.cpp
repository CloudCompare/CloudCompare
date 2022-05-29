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

#include "Cloud2CloudDist.h"
 
//system
#include <cmath>
#include <limits>

// For each lidar point, we find its neighbors in cloth particles by  Rounding operation.
// use for neighbor particles to do bilinear interpolation.
bool Cloud2CloudDist::Compute(	const Cloth& cloth,
								const wl::PointCloud& pc,
								double class_threshold,
								std::vector<unsigned>& groundIndexes,
								std::vector<unsigned>& offGroundIndexes,
								unsigned N/*=3*/)
{
	if (cloth.step_x < std::numeric_limits<double>::epsilon()
		|| cloth.step_y < std::numeric_limits<double>::epsilon())
	{
		// invalid cloth parameters
		return false;
	}

	try
	{
		//ÕÒµ½Ã¿¸ö¼¤¹âÀ×´ïµãµ½²¼ÁÏÖ±½ÓµÄ¾àÀë£¬ÓÃ¸Ã¾àÀëãÐÖµÀ´¶ÔµãÔÆ½øÐÐ·ÖÀà
		//Ë«ÏßÐÔ²åÖµ
		// for each lidar point, find the projection in the cloth grid, and the sub grid which contains it.
		//use the four corner of the subgrid to do bilinear interpolation;
		for (unsigned i = 0; i < static_cast<unsigned>(pc.size()); i++)
		{
			//½«¸Ã×ø±êÓë²¼ÁÏµÄ×óÉÏ½Ç×ø±êÏà¼õ
			double deltaX = pc[i].x - cloth.origin_pos.x;
			double deltaZ = pc[i].z - cloth.origin_pos.z;
			//µÃµ½¼¤¹âµãËùÔÚ²¼ÁÏÐ¡Íø¸ñ×óÉÏ½ÇµÄ×ø±ê ¼ÙÉèËÄ¸ö½Çµã·Ö±ðÎª0 1 2 3 Ë³Ê±Õë±àºÅ
			int col0 = static_cast<int>(deltaX / cloth.step_x);
			int row0 = static_cast<int>(deltaZ / cloth.step_y);
			int col1 = col0 + 1;
			int row1 = row0;
			int col2 = col0 + 1;
			int row2 = row0 + 1;
			int col3 = col0;
			int row3 = row0 + 1;
			//ÒÔ×ÓÍø¸ñ×óÉÏ½Ç½¨Á¢×ø±êÏµ£¬²¢½«Æä¹éÒ»»¯µ½[0,1]
			double subdeltaX = (deltaX - col0*cloth.step_x) / cloth.step_x;
			double subdeltaZ = (deltaZ - row0*cloth.step_y) / cloth.step_y;
			//cout << subdeltaX << " " << subdeltaZ << endl;
			//Ë«ÏßÐÔ²åÖµ bilinear interpolation;
			//f(x,y)=f(0,0)(1-x)(1-y)+f(0,1)(1-x)y+f(1,1)xy+f(1,0)x(1-y)
			double fxy = cloth.getParticle(col0, row0).pos.y * (1 - subdeltaX)*(1 - subdeltaZ)
				+ cloth.getParticle(col3, row3).pos.y * (1 - subdeltaX)*subdeltaZ
				+ cloth.getParticle(col2, row2).pos.y * subdeltaX*subdeltaZ
				+ cloth.getParticle(col1, row1).pos.y * subdeltaX*(1 - subdeltaZ);
			double height_var = fxy - pc[i].y;
			if (std::abs(height_var) < class_threshold)
			{
				groundIndexes.push_back(i);
			}
			else
			{
				offGroundIndexes.push_back(i);
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	groundIndexes.shrink_to_fit();
	offGroundIndexes.shrink_to_fit();

	return true;
}
