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
#include "ccPlanarEntityInterface.h"

class ccPlane;

// block
class QCC_DB_LIB_API StBlock : public ccGenericPrimitive, public ccPlanarEntityInterface
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


	StBlock(ccPlane * mainPlane, 
		PointCoordinateType top_height, 
		CCVector3 top_normal, 
		PointCoordinateType bottom_height,
		CCVector3 bottom_normal,
		QString name = QString("Block"));

	StBlock(ccPlane * mainPlane, 
		ccFacet * top_facet,
		ccFacet * bottom_facet,
		QString name = QString("Block"));

	static StBlock * Create(const std::vector<CCVector3>& top,
		const PointCoordinateType bottom_height,
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
// 	std::vector<CCVector3> getTop() { return m_top; };
// 	std::vector<CCVector3> getBottom() { return m_bottom; };
// 	CCVector3 getCenterTop();
// 	CCVector3 getCenterBottom();
//	std::vector<CCVector2> getProfile();

	ccFacet* getTopFacet();
	void setTopFacet(ccFacet* facet) { m_top_facet = facet; }
	ccFacet* getBottomFacet();
	void setBottomFacet(ccFacet* facet) { m_bottom_facet = facet; }

	void setTopHeight(double val);
	double getTopHeight() { return m_top_height; }
	void setBottomHeight(double val);
	double getBottomHeight() { return m_bottom_height; }

protected:

	//inherited from ccGenericPrimitive
	virtual bool toFile_MeOnly(QFile& out) const override;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags) override;
	virtual bool buildUp() override;
	bool buildFromFacet();

// 	ccFacet* m_top;
// 	ccFacet* m_bottom;

// 	std::vector<CCVector3> m_top;
// 	std::vector<CCVector3> m_bottom;

	ccFacet* m_top_facet;
	ccFacet* m_bottom_facet;

protected:
	ccPlane* m_mainPlane; // center, normal, profile
	
	//! top plane center=m_mainPlane.center+m_mainPlane.normal*m_top_height
	CCVector3 getTopCenter(); 
	double m_top_height;	
	CCVector3 m_top_normal;

	//! bottom plane center = m_mainPlane.center + m_mainPlane.normal*m_bottom_height
	CCVector3 getBottomCenter(); 
	double m_bottom_height;
	CCVector3 m_bottom_normal;

public:
	//inherited from ccPlanarEntityInterface //! for planar entity
	ccHObject* getPlane() override { return (ccHObject*)m_mainPlane; }
	//inherited from ccPlanarEntityInterface
	inline CCVector3 getNormal() const override;
	//inherited from ccPlanarEntityInterface //! Returns the facet center
	CCVector3 getCenter() const override;
	//inherited from ccPlanarEntityInterface
	void notifyPlanarEntityChanged(ccGLMatrix mat, bool trans) override;

};

#endif //ST_BLOCK_HEADER
