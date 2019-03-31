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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef ST_BLOCK_HEADER
#define ST_BLOCK_HEADER

//Local
#include "ccGenericPrimitive.h"
#include "ccFacet.h"

// block
class QCC_DB_LIB_API StBlock : public ccGenericPrimitive
{
public:

	//! Default constructor
	/** Input (2D) polyline represents the profile in (X,Y) plane.
		Extrusion is always done ine the 'Z' dimension.
		\param profile 2D profile to extrude
		\param height extrusion thickness
		\param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
		\param name name
	**/
	StBlock(const std::vector<CCVector2>& profile,
		PointCoordinateType bottom_height,
		PointCoordinateType top_height,
		const ccGLMatrix* transMat = 0,
		QString name = QString("Block"));

	StBlock(const std::vector<CCVector3>& top,
		const std::vector<CCVector3>& bottom,
		const ccGLMatrix* transMat = 0,
		QString name = QString("Block"));

	StBlock(CCLib::GenericIndexedCloudPersist* top_cloud,
		const std::vector<int>& top_index,
		CCLib::GenericIndexedCloudPersist* bottom_cloud,
		const std::vector<int>& bottom_index,
		const ccGLMatrix* transMat = 0,
		QString name = QString("Block"));

	StBlock(ccFacet* top,
		ccFacet* bottom,
		const ccGLMatrix* transMat = 0,
		QString name = QString("Block"));



	//! Simplified constructor
	/** For ccHObject factory only!
	**/
	StBlock(QString name = QString("Block"));

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::ST_BLOCK; }

	//inherited from ccGenericPrimitive
	virtual QString getTypeName() const override { return "Block"; }
	virtual ccGenericPrimitive* clone() const override;
	
	//! Returns profile
	const std::vector<CCVector3>& getTop() const;
	const std::vector<CCVector3>& getBottom() const;
	const std::vector<CCVector2>& getProfile() const;

protected:

	//inherited from ccGenericPrimitive
	virtual bool toFile_MeOnly(QFile& out) const override;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags) override;
	virtual bool buildUp() override;

	ccFacet* m_top;
	ccFacet* m_bottom;

};

#endif //ST_BLOCK_HEADER
