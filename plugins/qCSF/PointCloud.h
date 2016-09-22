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
