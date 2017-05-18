#ifndef CC_POINT_CLOUD_INTERPOLATOR
#define CC_POINT_CLOUD_INTERPOLATOR

class ccPointCloud;

//Local
#include "qCC_db.h"

//CCLib
#include <GenericProgressCallback.h>

//System
#include <vector>

class QCC_DB_LIB_API ccPointCloudInterpolator
{
public:

	//! Generic interpolation parameters
	struct Parameters
	{
		enum Method { NEAREST_NEIGHBOR, K_NEAREST_NEIGHBORS, RADIUS };

		Method method = NEAREST_NEIGHBOR;
		unsigned knn = 0;
		float radius = 0;
		double sigma = 0;
	};

	//! Interpolate scalar fields from another cloud
	static bool InterpolateScalarFieldsFrom(ccPointCloud* destCloud,
											ccPointCloud* srccloud,
											const std::vector<int>& sfIndexes,
											const Parameters& params,
											CCLib::GenericProgressCallback* progressCb = NULL,
											unsigned char octreeLevel = 0);


};

#endif //CC_POINT_CLOUD_INTERPOLATOR
