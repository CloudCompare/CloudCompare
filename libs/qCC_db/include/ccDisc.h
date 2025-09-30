#ifndef CCDISC_H
#define CCDISC_H

#include <ccGenericPrimitive.h>

//! Disc (primitive)
/** 3D disc primitive
 **/
class QCC_DB_LIB_API ccDisc : public ccGenericPrimitive
{
public:
	//! Default drawing precision
	/** \warning Never pass a 'constant initializer' by reference
	 **/
	static const unsigned DEFAULT_DRAWING_PRECISION = 24;

	//! Default constructor
	/** Simple disc constructor
		\param radius cylinder radius
		\param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
		\param name name
		\param precision drawing precision (angular step = 360/precision)
		\param uniqueID unique ID (handle with care)
	**/
	ccDisc(PointCoordinateType radius,
		   PointCoordinateType xOff      = 0,
		   PointCoordinateType yOff      = 0,
		   const ccGLMatrix*   transMat  = nullptr,
		   QString             name      = QString("Disc"),
		   unsigned            precision = DEFAULT_DRAWING_PRECISION,
		   unsigned            uniqueID  = ccUniqueIDGenerator::InvalidUniqueID);

	//! Simplified constructor
	/** For ccHObject factory only!
	**/
	ccDisc(QString name = QString("Disc"));

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override
	{
		return CC_TYPES::DISC;
	}

	// inherited from ccGenericPrimitive
	virtual QString getTypeName() const override
	{
		return "Disc";
	}
	virtual ccGenericPrimitive* clone() const override;

	void setRadius(PointCoordinateType radius)
	{
		m_radius = radius;
	}

protected:
	// inherited from ccGenericPrimitive
	bool  toFile_MeOnly(QFile& out, short dataVersion) const override;
	bool  fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	short minimumFileVersion_MeOnly() const override;
	bool  buildUp() override;

	//! Radius
	PointCoordinateType m_radius;

	//! Displacement of axes along X-axis (Snout mode)
	PointCoordinateType m_xOff;

	//! Displacement of axes along Y-axis (Snout mode)
	PointCoordinateType m_yOff;
};

#endif // CCDISC_H
