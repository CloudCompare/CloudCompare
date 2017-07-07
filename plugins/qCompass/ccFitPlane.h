
#ifndef CC_FITPLANE_HEADER
#define CC_FITPLANE_HEADER

#include <ccPlane.h>
#include <ccNormalVectors.h>

class ccFitPlane :
	public ccPlane
{
public:
	ccFitPlane(ccPlane* p);
	~ccFitPlane();

	void updateAttributes(float rms, float search_r);

	//create a FitPlane object from a point cloud
	static ccFitPlane* Fit(CCLib::GenericIndexedCloudPersist* cloud, double *rms);

	//returns true if object is a plane created by ccCompass (has the associated data)
	static bool isFitPlane(ccHObject* object);

};

#endif