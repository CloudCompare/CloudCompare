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

#ifndef CC_GENERIC_PRIMITIVE_HEADER
#define CC_GENERIC_PRIMITIVE_HEADER

//Local
#include "ccMesh.h"

class ccPointCloud;

//! Generic primitive interface
class QCC_DB_LIB_API ccGenericPrimitive : public ccMesh
{
public:

	//! Default constructor
	/** Warning: the associated transformation is purely for display purpose by default.
		Use 'ccHObject::applyGLTransformation_recursive' to 'apply' it to the mesh).
		\param name name
		\param transMat optional 3D transformation applied to the primitive vertices (can be set afterwards with ccDrawableObject::setGLTransformation + ccDrawableObject::applyGLTransformation_recursive)
		\param uniqueID unique ID (handle with care)
	**/
	ccGenericPrimitive(	QString name = QString(),
						const ccGLMatrix* transMat = nullptr,
						unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID);

	//! Returns type name (sphere, cylinder, etc.)
	virtual QString getTypeName() const = 0;

	//! Clones primitive
	virtual ccGenericPrimitive* clone() const = 0;

	//! Returns class ID
	inline CC_CLASS_ENUM getClassID() const override { return CC_TYPES::PRIMITIVE; }

	//! Sets primitive color (shortcut)
	/** \param col rgb color
	**/
	virtual void setColor(const ccColor::Rgb& col);

	//! Add operator
	/** Warning: simply copies the input primitive vertices/triangles to this primitive mesh!
	**/
	const ccGenericPrimitive& operator += (const ccGenericPrimitive& prim);

	//! Whether drawing is dependent on 'precision' parameter
	virtual inline bool hasDrawingPrecision() const { return false; }

	//! Minimum drawing precision
	/** \warning Never pass a 'constant initializer' by reference
	**/
	static const int MIN_DRAWING_PRECISION = 4;

	//! Sets drawing precision
	/** Warnings:
		- steps should always be >= ccGenericPrimitive::MIN_DRAWING_PRECISION
		- changes primitive content (calls ccGenericPrimitive::updateRepresentation)
		- may fail if not enough memory!
		\param steps drawing precision
		\return success (false if not enough memory)
	**/
	virtual bool setDrawingPrecision(unsigned steps);

	//! Returns drawing precision (or 0 if feature is not supported)
	virtual inline unsigned getDrawingPrecision() const { return m_drawPrecision; }

	//! Returns the transformation that is currently applied to the vertices
	virtual inline ccGLMatrix& getTransformation() { return m_transformation; }

	//! Returns the transformation that is currently applied to the vertices (const version)
	virtual inline const ccGLMatrix& getTransformation() const { return m_transformation; }

	//inherited methods (ccHObject)
	const ccGLMatrix& getGLTransformationHistory() const override;

protected:

	//! Inherited from ccGenericMesh
	void applyGLTransformation(const ccGLMatrix& trans) override;

	//inherited from ccMesh
	bool toFile_MeOnly(QFile& out) const override;
	bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;

	//! Builds primitive
	/** Transformation will be applied afterwards!
		\return success
	**/
	virtual bool buildUp() = 0;

	//! Updates internal representation (as a mesh)
	/** Calls buildUp then applyTransformationToVertices.
		\return success of buildUp
	**/
	virtual bool updateRepresentation();

	//! Inits internal structures
	/** Warning: resets all!
	**/
	bool init(unsigned vertCount, bool vertNormals, unsigned faceCount, unsigned faceNormCount);

	//! Applies associated transformation to vertices
	/** Should be called only when (re)constructing primitive!
	**/
	void applyTransformationToVertices();

	//! Finished 'clone' job (vertices color, etc.)
	/** \param primitive primitive to 'pimp'
		\return pimped primitive
	**/
	ccGenericPrimitive* finishCloneJob(ccGenericPrimitive* primitive) const;

	//! Returns vertices
	ccPointCloud* vertices();

	//! Associated transformation (applied to vertices)
	/** Different from ccDrawableObject::m_glTrans!
	**/
	ccGLMatrix m_transformation;

	//! Drawing precision (for primitives that support this feature)
	unsigned m_drawPrecision;
};

#endif //CC_GENERIC_PRIMITIVE_HEADER
