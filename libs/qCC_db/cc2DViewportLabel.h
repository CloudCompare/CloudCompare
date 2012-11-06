#ifndef CC_2D_VIEWPORT_LABEL_HEADER
#define CC_2D_VIEWPORT_LABEL_HEADER

//Local
#include "cc2DViewportObject.h"

//! 2D viewport label
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API cc2DViewportLabel : public cc2DViewportObject
#else
class cc2DViewportLabel : public cc2DViewportObject
#endif
{
public:

	//! Default constructor
	cc2DViewportLabel(const char* name=0);

	//inherited from ccHObject
    virtual CC_CLASS_ENUM getClassID() const {return CC_2D_VIEWPORT_LABEL;};
	virtual bool isSerializable() const { return true; }

	//! Returns ROI (relative to screen)
	const float* roi() const { return m_roi; }

	//! Sets ROI (relative to screen)
	void setRoi(const float* roi);

protected:

	//inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion);

    //! Draws the entity only (not its children)
    virtual void drawMeOnly(CC_DRAW_CONTEXT& context);

	//! label ROI
	/** ROI is relative to screen
	**/
	float m_roi[4];
};

#endif
