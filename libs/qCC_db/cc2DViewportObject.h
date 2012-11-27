#ifndef CC_2D_VIEWPORT_OBJECT_HEADER
#define CC_2D_VIEWPORT_OBJECT_HEADER

//Local
#include "ccHObject.h"
#include "ccGLMatrix.h"
#include "ccGenericGLDisplay.h"

//Qt
#include <QString>

//! 2D viewport object
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API cc2DViewportObject : public ccHObject
#else
class cc2DViewportObject : public ccHObject
#endif
{
public:

	//! Default constructor
	cc2DViewportObject(QString name=QString());

	//inherited from ccHObject
    virtual CC_CLASS_ENUM getClassID() const {return CC_2D_VIEWPORT_OBJECT;};
	virtual bool isSerializable() const { return true; }

	//! Sets perspective view state
	void setParameters(const ccViewportParameters& params) { m_params = params; }

	//! Gets parameters
	const ccViewportParameters& getParameters() const { return m_params; }

protected:

    //inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion);

	//! Viewport parameters
	ccViewportParameters m_params;
};

#endif
