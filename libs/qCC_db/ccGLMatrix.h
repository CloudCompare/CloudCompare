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
//$Rev:: 2172                                                              $
//$LastChangedDate:: 2012-06-24 18:33:24 +0200 (dim., 24 juin 2012)        $
//**************************************************************************
//

#ifndef CC_GL_MATRIX_HEADER
#define CC_GL_MATRIX_HEADER

//Local
#include "ccSerializableObject.h"

//CCLib
#include <CCGeom.h>
#include <Matrix.h>

//Matrix element shortcuts in (line,column) order
#define CC_MAT_R11 m_mat[0]
#define CC_MAT_R21 m_mat[1]
#define CC_MAT_R31 m_mat[2]
#define CC_MAT_R41 m_mat[3]

#define CC_MAT_R12 m_mat[4]
#define CC_MAT_R22 m_mat[5]
#define CC_MAT_R32 m_mat[6]
#define CC_MAT_R42 m_mat[7]

#define CC_MAT_R13 m_mat[8]
#define CC_MAT_R23 m_mat[9]
#define CC_MAT_R33 m_mat[10]
#define CC_MAT_R43 m_mat[11]

#define CC_MAT_R14 m_mat[12]
#define CC_MAT_R24 m_mat[13]
#define CC_MAT_R34 m_mat[14]
#define CC_MAT_R44 m_mat[15]

//! A 4x4 OpenGL float matrix
/** Transformation (M) is composed by a rotation (R) and a translation (T):
	M*X = R*X + T
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccGLMatrix : public ccSerializableObject
#else
class ccGLMatrix : public ccSerializableObject
#endif
{
public:

	//! Default constructor
	/** Matrix is set to identity (see toIdentity) by default.
	**/
	ccGLMatrix();

	//! Constructor from a GL matrix array
	/** \param mat16 a 16 elements array (OpenGL style matrix)
	**/
	ccGLMatrix(const float* mat16);

	//! Constructor from 4 columns (X,Y,Z,T)
	/** \param X 3 first elements of the 1st column (last one is 0)
		\param Y 3 first elements of the 2nd column (last one is 0)
		\param Z 3 first elements of the 3rd column (last one is 0)
		\param T 3 first elements of the last column (last one is 1)
	**/
	ccGLMatrix(const CCVector3& X, const CCVector3& Y, const CCVector3& Z, const CCVector3& T);

	//! Constructor from a 3x3 rotation matrix R and a vector T
	ccGLMatrix(const CCLib::SquareMatrix& R, const CCVector3& T);

	//! Constructor from a rotation center G, a 3x3 rotation matrix R and a vector T
	ccGLMatrix(const CCLib::SquareMatrix& R, const CCVector3& T, const CCVector3& rotCenter);

	//! Copy constructor
	ccGLMatrix(const ccGLMatrix& mat);

	//! Interpolates two matrices at relative position 'coef'
	/** \param coef interpolation position (should be between 0 and 1).
		\param glMat1 'left' matrix
		\param glMat2 'right' matrix
	**/
	static ccGLMatrix Interpolate(PointCoordinateType coef, const ccGLMatrix& glMat1, const ccGLMatrix& glMat2);

	//! Creates a transformation matrix that rotates a vector to another
	/** Adapted from  "Efficiently Building a Matrix to Rotate One Vector to Another"
		By Tomas Möller, John Hughes, Journal of Graphics Tools, 4(4):1-4, 1999
		\param from normalized non-zero source vector
		\param to normalized non-zero destination vector
	**/
	static ccGLMatrix FromToRotation(const CCVector3& from, const CCVector3& to);

	//! Clears matrix
	/** Matrix is set to 0.
	**/
	virtual void toZero();

	//! Sets matrix to identity
	virtual void toIdentity();

	//! Clears translation
	/** Translation is set to (0,0,0).
	**/
	void clearTranslation();

	//! Inits transformation from a rotation axis, an angle (in radians) and a translation
	void initFromParameters(PointCoordinateType alpha, const CCVector3& axis3D, const CCVector3& t3D);

	//! Inits transformation from 3 rotation angles (in radians), and a translation
	void initFromParameters(PointCoordinateType phi, PointCoordinateType theta, PointCoordinateType psi, const CCVector3& t3D);

	//! Gets transformation equivalent parameters
	/** Extracts parameters from transformation: a rotation axis,
		an angle (in radians) and a translation.
	**/
	void getParameters(PointCoordinateType& alpha, CCVector3& axis3D, CCVector3& t3D) const;

	//! Gets transformation equivalent parameters
	/** Extracts parameters from transformation: 3 rotation angles (in radians),
		and a translation.
	**/
	void getParameters(PointCoordinateType &phi, PointCoordinateType &theta, PointCoordinateType &psi, CCVector3& t3D) const;

	//! Returns a pointer to internal data
	inline float* data() { return m_mat; }

	//! Returns a const pointer to internal data
	inline const float* data() const { return m_mat; }

	//! Retruns a pointer to internal translation
	/** Translation corresponds to the begining of the
		third column of the matrix.
	**/
	inline  float* getTranslation() { return m_mat+12; }

	//! Retruns a const pointer to internal translation
	/** Translation corresponds to the begining of the
		third column of the matrix.
	**/
	inline const float* getTranslation() const { return m_mat+12; }

	//! Sets translation
	/** \param T 3D vector
	**/
	void setTranslation(const CCVector3& T);

	//! Returns a pointer to a given column
	inline float* getColumn(int index) { return m_mat+(index<<2); }

	//! Returns a const pointer to a given column
	inline const float* getColumn(int index) const { return m_mat+(index<<2); }

	//! Multiplication by a matrix operator
	ccGLMatrix operator * (const ccGLMatrix& mat) const;

	//! Multiplication by a vector operator
	inline CCVector3 operator * (const CCVector3& vec) const
	{
		return CCVector3(applyX(vec),applyY(vec),applyZ(vec));
	}

	//! (in place) Multiplication operator
	ccGLMatrix& operator *= (const ccGLMatrix& mat);

	//! (in place) Translation operator
	ccGLMatrix& operator += (const CCVector3& T);

	//! (in place) Translation operator
	ccGLMatrix& operator -= (const CCVector3& T);

	//! Applies transformation to a 3D vector (in place)
	/** Input vector is directly modified after calling this method
	**/
	inline void apply(CCVector3& vec) const
	{
		vec = (*this)*vec;
	}

	//! Get the resulting transformation along X dimension
	inline float applyX(const CCVector3& vec) const
	{
		return CC_MAT_R11*vec.x + CC_MAT_R12*vec.y + CC_MAT_R13*vec.z + CC_MAT_R14;
	}

	//! Get the resulting transformation along Y dimension
	inline float applyY(const CCVector3& vec) const
	{
		return CC_MAT_R21*vec.x + CC_MAT_R22*vec.y + CC_MAT_R23*vec.z + CC_MAT_R24;
	}

	//! Get the resulting transformation along Z dimension
	inline float applyZ(const CCVector3& vec) const
	{
		return CC_MAT_R31*vec.x + CC_MAT_R32*vec.y + CC_MAT_R33*vec.z + CC_MAT_R34;
	}

	//! Applies rotation only to a 3D vector (in place)
	/** Input vector is directly modified after calling this method
	**/
	inline void applyRotation(CCVector3& vec) const
	{
		applyRotation(vec.u);
	}

	//! Applies rotation only to a 3D vector (in place)
	/** Input vector is directly modified after calling this method
	**/
    inline void applyRotation(float* vec) const
	{
		register float vx=vec[0];
		register float vy=vec[1];
		register float vz=vec[2];

		vec[0] = CC_MAT_R11*vx + CC_MAT_R12*vy + CC_MAT_R13*vz;
		vec[1] = CC_MAT_R21*vx + CC_MAT_R22*vy + CC_MAT_R23*vz;
		vec[2] = CC_MAT_R31*vx + CC_MAT_R32*vy + CC_MAT_R33*vz;
	}

    //! Shifts rotation center
    /** Warning, this method only applies a shift (i.e. relatively to the
        current rotation center). This is not a way to set an
        absolute rotation center 'directly'.
    **/
    void shiftRotationCenter(const CCVector3& vec);

	//! Transposes matrix (in place)
	void transpose();

	//! Returns transposed matrix
	ccGLMatrix transposed() const;

	//! Inverts transformation
	void invert();

	//! Returns inverse transformation
	ccGLMatrix inverse() const;

	//! Saves matrix to an ASCII file
	virtual bool toAsciiFile(const char* filename) const;

	//! Loads matrix from an ASCII file
	virtual bool fomAsciiFile(const char* filename);

	//! Scales the whole matrix
	/** \param coef scaling coef.
	**/
	void scale(float coef);

	//! Scales one line of the matrix
	/** \param lineIndex (0-3)
		\param coef scaling coef.
	**/
	void scaleLine(unsigned lineIndex, float coef);

	//! Scales one column of the matrix
	/** \param colIndex (0-3)
		\param coef scaling coef.
	**/
	void scaleColumn(unsigned colIndex, float coef);

	//inherited from ccSerializableObject
	virtual bool isSerializable() const { return true; }
	virtual bool toFile(QFile& out) const;
	virtual bool fromFile(QFile& in, short dataVersion);

protected:

	//! Internal 4x4 GL-style matrix data
	float m_mat[16];
};

#endif
