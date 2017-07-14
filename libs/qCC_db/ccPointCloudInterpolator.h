#ifndef CC_POINT_CLOUD_INTERPOLATOR
#define CC_POINT_CLOUD_INTERPOLATOR

class ccPointCloud;

//Local
#include "qCC_db.h"

//System
#include <vector>

namespace CCLib
{
	class GenericProgressCallback;
}

class QCC_DB_LIB_API ccPointCloudInterpolator
{
public:

	//! Generic interpolation parameters
	struct Parameters
	{
		enum Method { NEAREST_NEIGHBOR, K_NEAREST_NEIGHBORS, RADIUS };
		enum Algo { AVERAGE, MEDIAN, NORMAL_DIST };

		Method method = NEAREST_NEIGHBOR;
		Algo algo = AVERAGE;
		unsigned knn = 0;
		float radius = 0;
		double sigma = 0;
	};

	//! Interpolate scalar fields from another cloud
	static bool InterpolateScalarFieldsFrom(ccPointCloud* destCloud,
											ccPointCloud* srccloud,
											const std::vector<int>& sfIndexes,
											const Parameters& params,
											CCLib::GenericProgressCallback* progressCb = 0,
											unsigned char octreeLevel = 0);


};

#endif //CC_POINT_CLOUD_INTERPOLATOR
