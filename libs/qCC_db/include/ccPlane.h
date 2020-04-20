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

#ifndef CC_PLANE_PRIMITIVE_HEADER
#define CC_PLANE_PRIMITIVE_HEADER

//Local
#include "ccGenericPrimitive.h"
#include "ccPlanarEntityInterface.h"

//! Plane (primitive)
/** 3D plane primitive
**/
class QCC_DB_LIB_API ccPlane : public ccGenericPrimitive, public ccPlanarEntityInterface
{
public:

	//! Default constructor
	/** Plane normal corresponds to 'Z' dimension
		\param xWidth plane width along 'X' dimension
		\param yWidth plane width along 'Y' dimension
		\param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
		\param name name
	**/
	ccPlane(PointCoordinateType xWidth,
			PointCoordinateType yWidth,
			const ccGLMatrix* transMat = 0,
			QString name = QString("Plane"));

	//! Simplified constructor
	/** For ccHObject factory only!
	**/
	ccPlane(QString name = QString("Plane"));

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::PLANE; }

	//inherited from ccGenericPrimitive
	virtual QString getTypeName() const override { return "Plane"; }
	virtual ccGenericPrimitive* clone() const override;

	//inherited from ccHObject
	virtual ccBBox getOwnFitBB(ccGLMatrix& trans) override;

	//! Returns 'X' width
	PointCoordinateType getXWidth() const { return m_xWidth; }

	//! Returns 'Y' width
	PointCoordinateType getYWidth() const { return m_yWidth; }

	//! Returns the center
	CCVector3 getCenter() const { return m_transformation.getTranslationAsVec3D(); }

	//! Sets 'X' width
	void setXWidth(PointCoordinateType w, bool autoUpdate = true) { m_xWidth = w; if (autoUpdate) updateRepresentation(); }

	//! Sets 'Y' width
	void setYWidth(PointCoordinateType h, bool autoUpdate = true) { m_yWidth = h; if (autoUpdate) updateRepresentation(); }

	//inherited from ccPlanarEntityInterface
	CCVector3 getNormal() const override { return m_transformation.getColumnAsVec3D(2); }

	//! Sets an image as texture
	/** \return The created material (if successful)
	**/
	ccMaterial::Shared setAsTexture(QImage image, QString imageFilename = QString());

	//! Sets an image as texture for a quad mesh
	/** \return The created material (if successful)
	**/
	static ccMaterial::Shared SetQuadTexture(ccMesh* quadMesh, QImage image, QString imageFilename = QString());

	//! Fits a plane primitive on a cloud
	/** The cloud can be any CCLib::GenericIndexedCloudPersist-derived object,
		i.e. even a ccPolyline object for instance.
		\param[in] cloud input cloud
		\param[out] rms plane fitting rms (optional)
		\return plane primitive (if successful)
	**/
	static ccPlane* Fit(CCLib::GenericIndexedCloudPersist * cloud, double* rms = 0);

	//! Returns the equation of the plane
	/** Equation:
		N.P + constVal = 0
		i.e. Nx.x + Ny.y + Nz.z + constVal = 0
	**/
	void getEquation(CCVector3& N, PointCoordinateType& constVal) const;
	
	
	//! Returns the equation of the plane
	/** Equation:
		planeEquation plane equation : [a, b, c, d] as 'ax+by+cz=d'
		Same equation used in Neighbourhood and DistanceComputationTools 
	**/
	const PointCoordinateType* getEquation();

	//! Flips the plane
	void flip();

protected:

	//inherited from ccDrawable
	void drawMeOnly(CC_DRAW_CONTEXT& context) override;

	//inherited from ccGenericPrimitive
	bool toFile_MeOnly(QFile& out) const override;
	bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	bool buildUp() override;

	//! Width along 'X' dimension
	PointCoordinateType m_xWidth;

	//! Width along 'Y' dimension
	PointCoordinateType m_yWidth;

	// Array [a,b,c,d] such that ax+by+cz = d
	PointCoordinateType m_PlaneEquation[4];
};

#endif //CC_PLANE_PRIMITIVE_HEADER
