//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: Chris Brown                                        #
//#                                                                        #
//##########################################################################

#pragma once

//Local
#include "ccGenericPrimitive.h"

class ccPlane;

//! Coordinate System (primitive)
/** Coordinate System primitive
**/
class QCC_DB_LIB_API ccCoordinateSystem : public ccGenericPrimitive
{
public:

	//! Default constructor
	/** Coordinate System is essentially just a way to visualize a transform matrix.
		\param displayScale global scale/size
		\param axisWidth axes width
		\param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
		\param name name
	**/
	ccCoordinateSystem(	PointCoordinateType displayScale,
						PointCoordinateType axisWidth,
						const ccGLMatrix* transMat = nullptr,
						QString name = QString("CoordinateSystem"));

	//! Default constructor
	/** Coordinate System is essentially just a way to visualize a transform matrix.
		\param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
		\param name name
	**/
	ccCoordinateSystem(	const ccGLMatrix* transMat,
						QString name = QString("CoordinateSystem"));

	//! Simplified constructor
	/** For ccHObject factory only!
	**/
	ccCoordinateSystem(QString name = QString("CoordinateSystem"));

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::COORDINATESYSTEM; }

	//inherited from ccGenericPrimitive
	virtual QString getTypeName() const override { return "CoordinateSystem"; }
	virtual ccGenericPrimitive* clone() const override;
	
	inline bool axisPlanesAreShown() const { return m_showAxisPlanes; }
	void showAxisPlanes(bool show) { m_showAxisPlanes = show; }

	inline bool axisLinesAreShown() const { return m_showAxisLines; }
	void showAxisLines(bool show) { m_showAxisLines = show; }
	
	//ccPlane get2AxisPlane(int axisNum);
	inline CCVector3 getOrigin() const { return m_transformation.getTranslationAsVec3D(); };
	inline PointCoordinateType getAxisWidth() const { return m_width; };
	void setAxisWidth(PointCoordinateType size);

	inline PointCoordinateType getDisplayScale() const { return m_DisplayScale; };
	void setDisplayScale(PointCoordinateType size);

	CCVector3 getXYPlaneNormal() const;
	CCVector3 getYZPlaneNormal() const;
	CCVector3 getZXPlaneNormal() const;

	//! Default Display scale
	static constexpr PointCoordinateType DEFAULT_DISPLAY_SCALE = 1.0;
	//! Minimum Display scale
	static constexpr float MIN_DISPLAY_SCALE_F = 0.001f;
	//! Default Axis line width
	static constexpr PointCoordinateType AXIS_DEFAULT_WIDTH = 4.0;
	//! Minimum Axis line width
	static constexpr float MIN_AXIS_WIDTH_F = 1.0f;
	//! Maximum Axis line width
	static constexpr float MAX_AXIS_WIDTH_F = 16.0f;

protected:

	//inherited from ccDrawable
	void drawMeOnly(CC_DRAW_CONTEXT& context) override;

	//inherited from ccGenericPrimitive
	bool toFile_MeOnly(QFile& out, short dataVersion) const override;
	bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	short minimumFileVersion_MeOnly() const override;
	bool buildUp() override;

	ccPlane* createXYplane(const ccGLMatrix* transMat = nullptr) const;
	ccPlane* createYZplane(const ccGLMatrix* transMat = nullptr) const;
	ccPlane* createZXplane(const ccGLMatrix* transMat = nullptr) const;

	//! CoordinateSystem options
	PointCoordinateType m_DisplayScale;
	PointCoordinateType m_width;
	bool m_showAxisPlanes;
	bool m_showAxisLines;
	

};


