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

#ifndef WL_PCL_POINT_CLOUD_H_
#define WL_PCL_POINT_CLOUD_H_

//system
#include <vector>

namespace wl
{
	//! Point type
	struct Point
	{
		union
		{
			struct
			{
				float x;
				float y;
				float z;
			};
			float u[3];
		};

		Point()
			: x(0), y(0), z(0)
		{}
	};

	class PointCloud : public std::vector < Point >
	{
	public:
		
		void computeBoundingBox(Point& bbMin, Point& bbMax)
		{
			if (empty())
			{
				bbMin = bbMax = Point();
				return;
			}

			bbMin = bbMax = at(0);
			for (std::size_t i = 1; i < size(); i++)
			{
				const wl::Point& P = at(i);
				for (int d = 0; d < 3; ++d)
				{
					if (P.u[d] < bbMin.u[d])
					{
						bbMin.u[d] = P.u[d];
					}
					else if (P.u[d] > bbMax.u[d])
					{
						bbMax.u[d] = P.u[d];
					}
				}
			}
		}
	};
}

#endif //WL_POINT_CLOUD_H_
