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

	typedef std::vector< Point > PointCloud;
}

#endif //WL_POINT_CLOUD_H_
