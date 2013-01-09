//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 1854                                                              $
//$LastChangedDate:: 2011-05-13 23:56:42 +0200 (ven., 13 mai 2011)         $
//**************************************************************************
//

#ifndef CC_GENERIC_PRIMITIVE_HEADER
#define CC_GENERIC_PRIMITIVE_HEADER

#include "ccMesh.h"

class ccPointCloud;

//! Generic primitive interface
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccGenericPrimitive : public ccMesh
#else
class ccGenericPrimitive : public ccMesh
#endif
{
public:

	//! Default constructor
	/** Warning: the associated transformation is purely for display purpose by default.
		Use 'ccHObject::applyGLTransformation_recursive' to 'apply' it to the mesh).
		\param name name
		\param transMat optional 3D transformation applied to the primitive vertices (can be set afterwards with ccDrawableObject::setGLTransformation + ccDrawableObject::applyGLTransformation_recursive)
	**/
	ccGenericPrimitive(QString name = QString(), const ccGLMatrix* transMat = 0);

	//! Clones primitive
	virtual ccGenericPrimitive* clone() const = 0;

    //! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const { return CC_PRIMITIVE; }

    //! Sets primitive color (shortcut)
    /** \param col rgb color
    **/
	virtual void setColor(const colorType* col);

	//! Add operator
	/** Warning: simply copies the input primitive vertices/triangles to this primitive mesh!
	**/
	const ccGenericPrimitive& operator += (const ccGenericPrimitive& prim);

	//! Whether drawing is dependent on 'precision' parameter
	virtual bool hasDrawingPrecision() const { return false; }

	//! Sets drawing precision
	/** Warnings:
		- steps should always be >4
		- changes primitive content (call buildUp)
		- may fail if not enough memory!
	**/
	virtual bool setDrawingPrecision(unsigned steps);

	//! Returns drawing precision (or 0 if feature is not supported)
	virtual unsigned getDrawingPrecision() const { return m_drawPrecision; }

	//! Inherited from ccGenericMesh
    virtual void applyGLTransformation(const ccGLMatrix& trans);

	//! Returns the transformation that is currently applied to the vertices
	virtual ccGLMatrix& getTransformation() { return m_transformation; }

	//! Returns the transformation that is currently applied to the vertices (const version)
	virtual const ccGLMatrix& getTransformation() const { return m_transformation; }

protected:
    
    //inherited from ccMesh
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion);

	//! Builds primitive
	/** Transformation will be applied afterwards!
		\return success
	**/
	virtual bool buildUp() = 0;

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

    //! Associated GL transformation (applied to vertices)
    ccGLMatrix m_transformation;

	//! Drawing precision (for primitives that support this feature)
	unsigned m_drawPrecision;
};

#endif
