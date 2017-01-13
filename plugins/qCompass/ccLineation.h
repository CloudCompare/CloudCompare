#ifndef CC_LINEATION_HEADER
#define CC_LINEATION_HEADER

#include <ccPolyline.h>
#include <ccSphere.h>
#include <ccCylinder.h>
#include <ccCone.h>
#include <GenericIndexedCloudPersist.h>
#include <ccPointCloud.h>

class ccLineation : public ccPolyline
{
public:
	ccLineation(ccPointCloud* associatedCloud)
		: ccPolyline(associatedCloud)
	{ }

	virtual ~ccLineation() {}

	//inherited from ccHObject
	inline virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::POLY_LINE; }

	//get the direction of this lineation (not normalized) 
	CCVector3 getDirection()
	{
		if (size() != 2)
		{
			return CCVector3();
		} else
		{
			const CCVector3 start = *getPoint(0);
			const CCVector3 end = *getPoint(1);
			return end - start;
		}
	}

	ccColor::Rgba colour = ccColor::green;

protected:
	float m_relMarkerScale = 5.0f;

	//overidden from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;

};

#endif
