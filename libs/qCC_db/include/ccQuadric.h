#pragma once

// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

// Local
#include "ccGenericPrimitive.h"

//! Quadric (primitive)
/** 2D1/2 quadric primitive
 **/
class QCC_DB_LIB_API ccQuadric : public ccGenericPrimitive
{
  public:
	//! Default drawing precision
	/** \warning Never pass a 'constant initializer' by reference
	 **/
	static const unsigned DEFAULT_DRAWING_PRECISION = 24;

	//! Default constructor
	/** Quadric orthogonal dimension is 'Z' by default
	    \param minCorner min corner of the 'representation' base area
	    \param maxCorner max corner of the 'representation' base area
	    \param eq equation coefficients ( Z = a + b.X + c.Y + d.X^2 + e.X.Y + f.Y^2)
	    \param toLocalOrientation optional local orientation matrix
	    \param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
	    \param name name
	    \param precision drawing precision
	**/
	ccQuadric(const CCVector2d&              minCorner,
	          const CCVector2d&              maxCorner,
	          const double                   eq[6],
	          const CCCoreLib::SquareMatrix* toLocalOrientation = nullptr,
	          const ccGLMatrixd*             transMat           = nullptr,
	          QString                        name               = QString("Quadric"),
	          unsigned                       precision          = DEFAULT_DRAWING_PRECISION);

	//! Simplified constructor
	/** For ccHObject factory only!
	 **/
	ccQuadric(QString name = QString("Plane"));

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override
	{
		return CC_TYPES::QUADRIC;
	}

	// inherited from ccGenericPrimitive
	virtual QString getTypeName() const override
	{
		return "Quadric";
	}
	virtual bool hasDrawingPrecision() const override
	{
		return true;
	}
	virtual ccGenericPrimitive* clone() const override;

	//! Returns min corner
	const CCVector2d& getMinCorner() const
	{
		return m_minCorner;
	}
	//! Returns max corner
	const CCVector2d& getMaxCorner() const
	{
		return m_maxCorner;
	}

	//! Returns the equation coefficients
	inline const double* getEquationCoefs() const
	{
		return m_eq;
	}

	//! Returns the equation local coordinate/orientation system
	inline const CCCoreLib::SquareMatrix& getLocalOrientation() const
	{
		return m_toLocalOrientation;
	}

	//! Projects a 3D point in the quadric coordinate system
	/** \param P input 3D point
	    \param[out] Q position of the input point in the quadric coordinate system
	    \return elevation of the input point (in the coordinate system quadric)
	**/
	double projectOnQuadric(const CCVector3d& P, CCVector3d& Q) const;

	//! Returns the equation coefficients as a string
	QString getEquationString() const;

	//! Fits a quadric primitive on a cloud
	/** The cloud can be any CCCoreLib::GenericIndexedCloudPersist-derived object.
	    \param[in] cloud input cloud
	    \param[out] rms quadric fitting rms (optional)
	    \return quadric primitive (if successful)
	**/
	static ccQuadric* Fit(CCCoreLib::GenericIndexedCloudPersist* cloud, double* rms = nullptr);

  protected:
	// inherited from ccGenericPrimitive
	bool  toFile_MeOnly(QFile& out, short dataVersion) const override;
	bool  fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	short minimumFileVersion_MeOnly() const override;
	bool  buildUp() override;

	//! Min corner
	CCVector2d m_minCorner;
	//! Max corner
	CCVector2d m_maxCorner;

	//! Equation coefficients
	double m_eq[6];

	//! Local quadric orientation matrix
	CCCoreLib::SquareMatrix m_toLocalOrientation;

	//! Min height
	double m_minZ;
	//! Max height
	double m_maxZ;
};
