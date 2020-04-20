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

#ifndef CC_GL_MATRIX_TPL_HEADER
#define CC_GL_MATRIX_TPL_HEADER

//Local
#include "ccSerializableObject.h"

//CCLib
#include <CCConst.h>
#include <CCGeom.h>

//Qt
#include <QTextStream>


//! Model view matrix size (OpenGL)
static const unsigned OPENGL_MATRIX_SIZE = 16;

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

//! A 4x4 'transformation' matrix (column major order)
/** Transformation (M) is composed by a rotation (R) and a translation (T):
	M*X = R*X + T
**/
template <typename T> class ccGLMatrixTpl : public ccSerializableObject
{
public:

	//! Default constructor
	/** Matrix is set to identity (see toIdentity) by default.
	**/
	ccGLMatrixTpl() { toIdentity(); }

	//! Copy constructor
	ccGLMatrixTpl(const ccGLMatrixTpl<T>& mat) { for (unsigned i = 0; i < OPENGL_MATRIX_SIZE; ++i) m_mat[i] = mat.m_mat[i]; }

	//! Constructor from a float GL matrix array
	/** \param mat16f a 16 elements array (column major order)
	**/
	explicit ccGLMatrixTpl(const float* mat16f) { for (unsigned i = 0; i < OPENGL_MATRIX_SIZE; ++i) m_mat[i] = static_cast<T>(mat16f[i]); internalRescale(); }

	//! Constructor from a double GL matrix array
	/** \warning Will implicitly cast the elements to float!
		\param mat16d a 16 elements array (column major order)
	**/
	explicit ccGLMatrixTpl(const double* mat16d) { for (unsigned i = 0; i < OPENGL_MATRIX_SIZE; ++i) m_mat[i] = static_cast<T>(mat16d[i]); internalRescale(); }

	//! Constructor from 4 columns (X,Y,Z,T)
	/** \param X 3 first elements of the 1st column (last one is 0)
		\param Y 3 first elements of the 2nd column (last one is 0)
		\param Z 3 first elements of the 3rd column (last one is 0)
		\param Tr 3 first elements of the last column (last one is 1)
	**/
	ccGLMatrixTpl(const Vector3Tpl<T>& X, const Vector3Tpl<T>& Y, const Vector3Tpl<T>& Z, const Vector3Tpl<T>& Tr)
	{
		CC_MAT_R11 = X.x;  CC_MAT_R21 = X.y;  CC_MAT_R31 = X.z;  CC_MAT_R41 = 0;
		CC_MAT_R12 = Y.x;  CC_MAT_R22 = Y.y;  CC_MAT_R32 = Y.z;  CC_MAT_R42 = 0;
		CC_MAT_R13 = Z.x;  CC_MAT_R23 = Z.y;  CC_MAT_R33 = Z.z;  CC_MAT_R43 = 0;
		CC_MAT_R14 = Tr.x; CC_MAT_R24 = Tr.y; CC_MAT_R34 = Tr.z; CC_MAT_R44 = static_cast<T>(1);
	}

	//! Interpolates two matrices at relative position 'coef'
	/** \param coef interpolation position (should be between 0 and 1).
		\param glMat1 'left' matrix
		\param glMat2 'right' matrix
	**/
	static ccGLMatrixTpl Interpolate(T coef, const ccGLMatrixTpl<T>& glMat1, const ccGLMatrixTpl<T>& glMat2)
	{
		//we compute the transformation matrix between glMat1 and glMat2
		ccGLMatrixTpl<T> invTrans1 = glMat1.inverse();
		ccGLMatrixTpl<T> m12 = invTrans1 * glMat2;

		Vector3Tpl<T> axis,Tr;
		T alpha;
		m12.getParameters(alpha, axis, Tr);

		//we only have to interpolate the angle value
		alpha *= coef;
		//and the translation
		Tr *= coef;

		//we build-up the resulting matrix
		m12.initFromParameters(alpha, axis, Tr);

		//eventually we build-up resulting transformation
		return glMat1 * m12;
	}

	//! Creates a transformation matrix that rotates a vector to another
	/** Adapted from  "Efficiently Building a Matrix to Rotate One Vector to Another"
		By Tomas Möller, John Hughes, Journal of Graphics Tools, 4(4):1-4, 1999
		\param from normalized non-zero source vector
		\param to normalized non-zero destination vector
	**/
	static ccGLMatrixTpl<T> FromToRotation(const Vector3Tpl<T>& from, const Vector3Tpl<T>& to)
	{
		T c = from.dot(to);
		T f = (c < 0 ? -c : c);
		ccGLMatrixTpl<T> result;

		if (1.0 - f < ZERO_TOLERANCE) //"from" and "to"-vector almost parallel
		{
			// "to" vector most nearly orthogonal to "from"
			Vector3Tpl<T> x(0, 0, 0);
			if (fabs(from.x) < fabs(from.y))
			{
				if (fabs(from.x) < fabs(from.z))
					x.x = static_cast<T>(1);
				else
					x.z = static_cast<T>(1);
			}
			else
			{
				if (fabs(from.y) < fabs(from.z))
					x.y = static_cast<T>(1);
				else
					x.z = static_cast<T>(1);
			}

			Vector3Tpl<T> u = x - from;
			Vector3Tpl<T> v = x - to;

			T c1 = 2 / u.dot(u);
			T c2 = 2 / v.dot(v);
			T c3 = c1 * c2  * u.dot(v);

			T* mat = result.data();
			for (unsigned i = 0; i < 3; i++)
			{
				for (unsigned j = 0; j < 3; j++)
				{
					mat[i*4+j] =  c3 * v.u[i] * u.u[j]
								- c2 * v.u[i] * v.u[j]
								- c1 * u.u[i] * u.u[j];
				}
				mat[i * 4 + i] += static_cast<T>(1);
			}
		}
		else  // the most common case, unless "from"="to", or "from"=-"to"
		{
			//see Efficiently Building a Matrix to Rotate One Vector to Another
			//T. Moller and J.F. Hugues (1999)
			Vector3Tpl<T> v = from.cross(to);
			T h = 1 / (1 + c);
			T hvx = h * v.x;
			T hvz = h * v.z;
			T hvxy = hvx * v.y;
			T hvxz = hvx * v.z;
			T hvyz = hvz * v.y;

			T* mat = result.data();
			mat[0] = c + hvx * v.x;
			mat[1] = hvxy + v.z;
			mat[2] = hvxz - v.y;

			mat[4] = hvxy - v.z;
			mat[5] = c + h * v.y * v.y;
			mat[6] = hvyz + v.x;

			mat[8] = hvxz + v.y;
			mat[9] = hvyz - v.x;
			mat[10] = c + hvz * v.z;
		}

		return result;
	}

	//! Converts a quaternion to a rotation matrix
	/** \param q quaternion (4 values: w,x,y,z)
		\return corresponding rotation matrix
	**/
	template<class Tq> static ccGLMatrixTpl<T> FromQuaternion(const Tq q[])
	{
		assert(q);

		ccGLMatrixTpl<T> rotMat;
		T* mat = rotMat.data();

		//diagonal
		{
			Tq q00 = q[0] * q[0];
			Tq q11 = q[1] * q[1];
			Tq q22 = q[2] * q[2];
			Tq q33 = q[3] * q[3];

			mat[0]	= static_cast<T>(q00 + q11 - q22 - q33);
			mat[5]	= static_cast<T>(q00 - q11 + q22 - q33);
			mat[10]	= static_cast<T>(q00 - q11 - q22 + q33);
			mat[15]	= static_cast<T>(1);
		}

		//non-diagonal elements
		{
			Tq q03 = q[0] * q[3];
			Tq q13 = q[1] * q[3];
			Tq q23 = q[2] * q[3];
			Tq q02 = q[0] * q[2];
			Tq q12 = q[1] * q[2];
			Tq q01 = q[0] * q[1];

			mat[1] = static_cast<T>((q12 + q03) * 2);
			mat[2] = static_cast<T>((q13 - q02) * 2);
			mat[4] = static_cast<T>((q12 - q03) * 2);
			mat[6] = static_cast<T>((q23 + q01) * 2);
			mat[8] = static_cast<T>((q13 + q02) * 2);
			mat[9] = static_cast<T>((q23 - q01) * 2);
		}

		return rotMat;
	}

	//! Generates a 'viewing' matrix from a looking vector and a 'up' direction
	/** \warning No translation is applied (pure rotation matrix)
		\param forward forward 'view' vector
		\param up up vector
		\return corresponding rotation matrix
	**/
	static ccGLMatrixTpl<T> FromViewDirAndUpDir(const Vector3Tpl<T>& forward, const Vector3Tpl<T>& up)
	{
		//normalize forward
		Vector3Tpl<T> uForward = forward;
		uForward.normalize();
		
		//side = forward x up
		Vector3Tpl<T> uSide = uForward.cross(up);
		uSide.normalize();
		
		//recompute 'up' as: up = side x forward
		Vector3Tpl<T> uUp = uSide.cross(uForward);
		uUp.normalize();

		ccGLMatrixTpl<T> matrix;
		{
			T* mat = matrix.data();
			mat[ 0] =  uSide.x;
			mat[ 4] =  uSide.y;
			mat[ 8] =  uSide.z;
			mat[12] =  0;
			mat[ 1] =  uUp.x;
			mat[ 5] =  uUp.y;
			mat[ 9] =  uUp.z;
			mat[13] =  0 ;
			mat[ 2] = -uForward.x;
			mat[ 6] = -uForward.y;
			mat[10] = -uForward.z;
			mat[14] =  0 ;
			mat[ 3] =  0;
			mat[ 7] =  0;
			mat[11] =  0;
			mat[15] =  static_cast<T>(1) ;
		}
		return matrix;
	}

	//! Converts a 'text' matrix to a ccGLMatrix
	/** \param[in] matText matrix text
		\param[out] success whether input matrix text is valid or not
	**/
	static ccGLMatrixTpl<T> FromString(const QString &matText, bool& success)
	{
		QStringList valuesStr = matText.simplified().split(QChar(' '), QString::SkipEmptyParts);
		if (valuesStr.size() != OPENGL_MATRIX_SIZE)
		{
			success = false;
			return ccGLMatrixTpl<T>();
		}

		ccGLMatrixTpl<T> matrix;
		T* matValues = matrix.data();
		for (unsigned i = 0; i < OPENGL_MATRIX_SIZE; ++i)
		{
			matValues[i] = static_cast<T>(valuesStr[(i % 4) * 4 + (i >> 2)].toDouble(&success));
			if (!success)
				return ccGLMatrixTpl<T>();
		}

		matrix.internalRescale();

		success = true;
		return matrix;
	}

	//! Returns matrix as a string
	/** \param precision numerical precision
		\param separator separator
		\return string
	**/
	QString toString(int precision = 12, QChar separator = ' ') const
	{
		QString str;
		for (unsigned l = 0; l < 4; ++l) //lines
		{
			for (unsigned c = 0; c < 4; ++c) //columns
			{
				str.append(QString::number(m_mat[c * 4 + l], 'f', precision));
				if (c != 3)
					str.append(separator);
			}
			if (l != 3)
				str.append("\n");
		}
		return str;
	}

	//! Saves matrix to an ASCII file
	/** \param filename output file name
		\param precision output digits precision
	***/
	virtual bool toAsciiFile(QString filename, int precision = 12) const
	{
		QFile fp(filename);
		if (!fp.open(QFile::WriteOnly | QFile::Text))
			return false;

		QTextStream stream(&fp);
		stream.setRealNumberPrecision(precision);
		stream.setRealNumberNotation(QTextStream::FixedNotation);
		for (unsigned i = 0; i < 4; ++i)
		{
			stream << m_mat[i] << " " << m_mat[i + 4] << " " << m_mat[i + 8] << " " << m_mat[i + 12] << endl;
		}

		return (fp.error() == QFile::NoError);
	}

	//! Loads matrix from an ASCII file
	/** \param filename input file name
	***/
	virtual bool fromAsciiFile(QString filename)
	{
		QFile fp(filename);
		if (!fp.open(QFile::ReadOnly | QFile::Text))
			return false;

		QTextStream stream(&fp);

		for (unsigned i = 0; i < 4; ++i)
		{
			stream >> m_mat[i     ];
			stream >> m_mat[i +  4];
			stream >> m_mat[i +  8];
			stream >> m_mat[i + 12];
		}

		internalRescale();

		return (fp.error() == QFile::NoError);
	}

	//! Returns the rotation component around X only
	ccGLMatrixTpl<T> xRotation() const
	{
		ccGLMatrixTpl<T> newRotMat;
		newRotMat.toIdentity();

		//we use a specific Euler angles convention here
		if (CC_MAT_R13 >= 1)
		{
			//simpler/faster to ignore this (very) specific case!
			return newRotMat;
		}
		T phi = -asin(CC_MAT_R13);
		T cos_phi = cos(phi);
		T theta = atan2(CC_MAT_R23 / cos_phi, CC_MAT_R33 / cos_phi);

		newRotMat.CC_MAT_R22 = newRotMat.CC_MAT_R33 = cos(theta);
		newRotMat.CC_MAT_R32 = newRotMat.CC_MAT_R23 = sin(theta);
		newRotMat.CC_MAT_R23 *= static_cast<T>(-1);

		newRotMat.setTranslation(getTranslation());

		return newRotMat;
	}

	//! Returns the rotation component around Y only
	ccGLMatrixTpl<T> yRotation() const
	{
		ccGLMatrixTpl<T> newRotMat;
		newRotMat.toIdentity();

		//we use a specific Euler angles convention here
		if (CC_MAT_R32 >= 1)
		{
			//simpler/faster to ignore this (very) specific case!
			return newRotMat;
		}
		T theta = asin(CC_MAT_R32);
		T cos_theta = cos(theta);
		T phi = atan2(-CC_MAT_R31 / cos_theta, CC_MAT_R33 / cos_theta);

		newRotMat.CC_MAT_R11 = newRotMat.CC_MAT_R33 = cos(phi);
		newRotMat.CC_MAT_R31 = newRotMat.CC_MAT_R13 = sin(phi);
		newRotMat.CC_MAT_R31 *= static_cast<T>(-1);

		newRotMat.setTranslation(getTranslation());

		return newRotMat;
	}

	//! Returns the rotation component around Z only
	ccGLMatrixTpl<T> zRotation() const
	{
		//we can use the standard Euler angles convention here
		T phi, theta, psi;
		Vector3Tpl<T> Tr;
		getParameters(phi, theta, psi, Tr);
		assert(Tr.norm2() == 0);

		ccGLMatrixTpl<T> newRotMat;
		newRotMat.initFromParameters(phi, 0, 0, Tr);

		return newRotMat;
	}

	//! Clears matrix
	/** Matrix is set to 0.
	**/
	inline virtual void toZero() { memset(m_mat, 0, OPENGL_MATRIX_SIZE * sizeof(T)); }

	//! Sets matrix to identity
	inline virtual void toIdentity() { toZero(); CC_MAT_R11 = CC_MAT_R22 = CC_MAT_R33 = CC_MAT_R44 = static_cast<T>(1); }

	//! Clears translation
	/** Translation is set to (0,0,0).
	**/
	inline void clearTranslation() { CC_MAT_R14 = CC_MAT_R24 = CC_MAT_R34 = 0; }

	//! Inits transformation from a rotation axis, an angle and a translation
	/** \param[in] alpha_rad rotation angle (in radians)
		\param[in] axis3D rotation axis
		\param[in] t3D translation
	**/
	void initFromParameters(T alpha_rad,
							const Vector3Tpl<T>& axis3D,
							const Vector3Tpl<T>& t3D)
	{
		T cos_t = cos(alpha_rad);
		T sin_t = sin(alpha_rad);
		T inv_cos_t = static_cast<T>(1) - cos_t;

		//normalize rotation axis
		Vector3Tpl<T> uAxis3D = axis3D;
		uAxis3D.normalize();

		const T& l1 = uAxis3D.x;
		const T& l2 = uAxis3D.y;
		const T& l3 = uAxis3D.z;

		T l1_inv_cos_t = l1 * inv_cos_t;
		T l3_inv_cos_t = l3 * inv_cos_t;

		//1st column
		CC_MAT_R11 = cos_t + l1 * l1_inv_cos_t;
		CC_MAT_R21 = l2 * l1_inv_cos_t + l3 * sin_t;
		CC_MAT_R31 = l3 * l1_inv_cos_t - l2 * sin_t;
		CC_MAT_R41 = 0;

		//2nd column
		CC_MAT_R12 = l2 * l1_inv_cos_t - l3 * sin_t;
		CC_MAT_R22 = cos_t + l2 * l2*inv_cos_t;
		CC_MAT_R32 = l2 * l3_inv_cos_t + l1 * sin_t;
		CC_MAT_R42 = 0;

		//3rd column
		CC_MAT_R13 = l3 * l1_inv_cos_t + l2 * sin_t;
		CC_MAT_R23 = l2 * l3_inv_cos_t - l1 * sin_t;
		CC_MAT_R33 = cos_t + l3 * l3_inv_cos_t;
		CC_MAT_R43 = 0;

		//4th column
		CC_MAT_R14 = t3D.x;
		CC_MAT_R24 = t3D.y;
		CC_MAT_R34 = t3D.z;
		CC_MAT_R44 = static_cast<T>(1);
	}


	//! Inits transformation from 3 rotation angles and a translation
	/** See http://en.wikipedia.org/wiki/Euler_angles (Tait-Bryan Z1Y2X3)
		\param[in] phi_rad Phi angle (in radians)
		\param[in] theta_rad Theta angle (in radians)
		\param[in] psi_rad Psi angle (in radians)
		\param[in] t3D translation
	**/
	void initFromParameters(	T phi_rad,
								T theta_rad,
								T psi_rad,
								const Vector3Tpl<T>& t3D)
	{
		T c1 = cos(phi_rad);
		T c2 = cos(theta_rad);
		T c3 = cos(psi_rad);

		T s1 = sin(phi_rad);
		T s2 = sin(theta_rad);
		T s3 = sin(psi_rad);

		//1st column
		CC_MAT_R11 = c2 * c1;
		CC_MAT_R21 = c2 * s1;
		CC_MAT_R31 = -s2;
		CC_MAT_R41 = 0;

		//2nd column
		CC_MAT_R12 = s3 * s2*c1 - c3 * s1;
		CC_MAT_R22 = s3 * s2*s1 + c3 * c1;
		CC_MAT_R32 = s3 * c2;
		CC_MAT_R42 = 0;

		//3rd column
		CC_MAT_R13 = c3 * s2*c1 + s3 * s1;
		CC_MAT_R23 = c3 * s2*s1 - s3 * c1;
		CC_MAT_R33 = c3 * c2;
		CC_MAT_R43 = 0;

		//4th column
		CC_MAT_R14 = t3D.x;
		CC_MAT_R24 = t3D.y;
		CC_MAT_R34 = t3D.z;
		CC_MAT_R44 = static_cast<T>(1);
	}

	//! Returns equivalent parameters: a rotation axis, an angle and a translation
	/** \param[out] alpha_rad rotation angle in radians (in [0;pi])
		\param[out] axis3D unit rotation axis
		\param[out] t3D translation
	**/
	void getParameters(	T& alpha_rad,
						Vector3Tpl<T>& axis3D,
						Vector3Tpl<T>& t3D) const
	{
		T trace = CC_MAT_R11 + CC_MAT_R22 + CC_MAT_R33;
		T cos_t = (trace - 1) / 2;
		if (fabs(cos_t) <= 1)
		{
			alpha_rad = acos(cos_t); //result in [0;pi]
		}
		else
		{
			alpha_rad = 0;
		}

		axis3D.x = CC_MAT_R32 - CC_MAT_R23;
		axis3D.y = CC_MAT_R13 - CC_MAT_R31;
		axis3D.z = CC_MAT_R21 - CC_MAT_R12;

		//normalize axis
		T n2 = axis3D.norm2();
		if (n2 > ZERO_TOLERANCE)
		{
			axis3D /= sqrt(n2);
		}
		else
		{
			//axis is too small!
			axis3D = Vector3Tpl<T>(0, 0, 1);
		}

		t3D.x = CC_MAT_R14;
		t3D.y = CC_MAT_R24;
		t3D.z = CC_MAT_R34;
	}

	//! Returns equivalent parameters: 3 rotation angles and a translation
	/** See http://en.wikipedia.org/wiki/Euler_angles (Tait-Bryan Z1Y2X3)
		\param[out] phi_rad Phi angle (in radians)
		\param[out] theta_rad Theta angle (in radians)
		\param[out] psi_rad Psi angle (in radians)
		\param[out] t3D translation
	**/
	void getParameters(	T &phi_rad,
						T &theta_rad,
						T &psi_rad,
						Vector3Tpl<T>& t3D) const
	{
		if (fabs(CC_MAT_R31) != 1)
		{
			theta_rad = -asin(CC_MAT_R31);
			T cos_theta = cos(theta_rad);
			psi_rad = atan2(CC_MAT_R32 / cos_theta, CC_MAT_R33 / cos_theta);
			phi_rad = atan2(CC_MAT_R21 / cos_theta, CC_MAT_R11 / cos_theta);

			//Other solution
			//theta = M_PI + asin(CC_MAT_R31);
			//T cos_theta = cos(theta);
			//psi = atan2(CC_MAT_R32 / cos_theta, CC_MAT_R33 / cos_theta);
			//phi = atan2(CC_MAT_R21 / cos_theta, CC_MAT_R11 / cos_theta);
		}
		else
		{
			phi_rad = 0;

			if (CC_MAT_R31 == -1)
			{
				theta_rad = static_cast<T>(M_PI_2);
				psi_rad = atan2(CC_MAT_R12, CC_MAT_R13);
			}
			else
			{
				theta_rad = -static_cast<T>(M_PI_2);
				psi_rad = -atan2(CC_MAT_R12, CC_MAT_R13);
			}
		}

		t3D.x = CC_MAT_R14;
		t3D.y = CC_MAT_R24;
		t3D.z = CC_MAT_R34;
	}

	//! Returns a pointer to internal data
	inline T* data() { return m_mat; }

	//! Returns a const pointer to internal data
	inline const T* data() const { return m_mat; }

	//! Retruns a pointer to internal translation
	/** Translation corresponds to the beginning of the
		third column of the matrix.
	**/
	inline T* getTranslation() { return m_mat+12; }

	//! Retruns a const pointer to internal translation
	/** Translation corresponds to the beginning of the
		third column of the matrix.
	**/
	inline const T* getTranslation() const { return m_mat+12; }

	//! Returns a copy of the translation as a CCVector3
	/** \return translation vector
	**/
	inline Vector3Tpl<T> getTranslationAsVec3D() const { return getColumnAsVec3D(3); }

	//! Sets translation (float version)
	/** \param Tr 3D vector **/
	inline void setTranslation(const Vector3Tpl<float>& Tr) { CC_MAT_R14 = static_cast<T>(Tr.x); CC_MAT_R24 = static_cast<T>(Tr.y); CC_MAT_R34 = static_cast<T>(Tr.z); }
	//! Sets translation (double version)
	/** \param Tr 3D vector **/
	inline void setTranslation(const Vector3Tpl<double>& Tr) { CC_MAT_R14 = static_cast<T>(Tr.x); CC_MAT_R24 = static_cast<T>(Tr.y); CC_MAT_R34 = static_cast<T>(Tr.z); }

	//! Sets translation from a float array
	/** \param Tr 3D vector as a float array
	**/
	void setTranslation(const float Tr[3]) { CC_MAT_R14 = static_cast<T>(Tr[0]); CC_MAT_R24 = static_cast<T>(Tr[1]); CC_MAT_R34 = static_cast<T>(Tr[2]); }
	//! Sets translation from a double array
	/** \param Tr 3D vector as a double array
	**/
	void setTranslation(const double Tr[3]) { CC_MAT_R14 = static_cast<T>(Tr[0]); CC_MAT_R24 = static_cast<T>(Tr[1]); CC_MAT_R34 = static_cast<T>(Tr[2]); }

	//! Returns a pointer to a given column
	/** \param index column index (between 0 and 3)
		\return pointer to the first element of the corresponding column
	**/
	inline T* getColumn(unsigned index) { return m_mat + (index << 2); }

	//! Returns a const pointer to a given column
	/** \param index column index (between 0 and 3)
		\return pointer to the first element of the corresponding column
	**/
	inline const T* getColumn(unsigned index) const { return m_mat + (index << 2); }

	//! Returns a copy of a given column as a CCVector3
	/** 4th value is ignored.
		\param index column index (between 0 and 3)
		\return copy of the three first elements of the corresponding column
	**/
	inline Vector3Tpl<T> getColumnAsVec3D(unsigned index) const { return Vector3Tpl<T>::fromArray(getColumn(index)); }

	//! Sets the content of a given column
	/** \param index column index (between 0 and 3)
		\param v new column values
	**/
	inline void setColumn(unsigned index, const Vector3Tpl<T>& v) { T* col = m_mat + (index << 2); col[0] = v.x; col[1] = v.y; col[2] = v.z; }

	//! Sets the content of a given column
	/** \param index column index (between 0 and 3)
		\param v new column values
	**/
	inline void setColumn(unsigned index, const Tuple4Tpl<T>& v) { T* col = m_mat + (index << 2); col[0] = v.x; col[1] = v.y; col[2] = v.z; col[3] = v.w; }

	//! Multiplication by a matrix operator
	ccGLMatrixTpl<T> operator * (const ccGLMatrixTpl<T>& mat) const
	{
		ccGLMatrixTpl<T> result;

		const T* A = m_mat;
		const T* B = mat.m_mat;
		T* C = result.m_mat;

		for (unsigned j = 0; j < 4; ++j, B += 4)
			for (unsigned i = 0; i < 4; ++i)
				*C++ = A[i] * B[0] + A[i + 4] * B[1] + A[i + 8] * B[2] + A[i + 12] * B[3];

		return result;
	}

	//! Multiplication by a vector operator (float version)
	inline Vector3Tpl<float> operator * (const Vector3Tpl<float>& vec) const { return Vector3Tpl<float>(applyX(vec), applyY(vec), applyZ(vec)); }
	//! Multiplication by a vector operator (double version)
	inline Vector3Tpl<double> operator * (const Vector3Tpl<double>& vec) const { return Vector3Tpl<double>(applyX(vec), applyY(vec), applyZ(vec)); }

	//! Multiplication by a 4D vector operator (float version)
	inline Tuple4Tpl<float> operator * (const Tuple4Tpl<float>& vec) const { return Tuple4Tpl<float>(applyX(vec), applyY(vec), applyZ(vec), applyW(vec)); }
	//! Multiplication by a 4D vector operator (double version)
	inline Tuple4Tpl<double> operator * (const Tuple4Tpl<double>& vec) const { return Tuple4Tpl<double>(applyX(vec), applyY(vec), applyZ(vec), applyW(vec)); }

	//! (in place) Addition operator
	ccGLMatrixTpl<T>& operator += (const ccGLMatrixTpl<T>& mat)
	{
		for (unsigned i = 0; i < OPENGL_MATRIX_SIZE; ++i)
			m_mat[i] += mat.m_mat[i];
		return (*this);
	}

	//! (in place) Difference operator
	ccGLMatrixTpl<T>& operator -= (const ccGLMatrixTpl<T>& mat)
	{
		for (unsigned i = 0; i < OPENGL_MATRIX_SIZE; ++i)
			m_mat[i] -= mat.m_mat[i];
		return (*this);
	}

	//! (in place) Multiplication operator
	ccGLMatrixTpl<T>& operator *= (const ccGLMatrixTpl<T>& mat)
	{
		(*this) = (*this) * mat;
		return (*this);
	}

	//! (in place) Forward translation operator (float version)
	ccGLMatrixTpl<T>& operator += (const Vector3Tpl<float>& Tr)
	{
		CC_MAT_R14 += static_cast<T>(Tr.x);
		CC_MAT_R24 += static_cast<T>(Tr.y);
		CC_MAT_R34 += static_cast<T>(Tr.z);
		return (*this);
	}
	//! (in place) Forward translation operator (double version)
	ccGLMatrixTpl<T>& operator += (const Vector3Tpl<double>& Tr)
	{
		CC_MAT_R14 += static_cast<T>(Tr.x);
		CC_MAT_R24 += static_cast<T>(Tr.y);
		CC_MAT_R34 += static_cast<T>(Tr.z);
		return (*this);
	}
	//! (in place) Backward translation operator (float version)
	ccGLMatrixTpl<T>& operator -= (const Vector3Tpl<float>& Tr)
	{
		CC_MAT_R14 -= static_cast<T>(Tr.x);
		CC_MAT_R24 -= static_cast<T>(Tr.y);
		CC_MAT_R34 -= static_cast<T>(Tr.z);
		return (*this);
	}
	//! (in place) Backward translation operator (double version)
	ccGLMatrixTpl<T>& operator -= (const Vector3Tpl<double>& Tr)
	{
		CC_MAT_R14 -= static_cast<T>(Tr.x);
		CC_MAT_R24 -= static_cast<T>(Tr.y);
		CC_MAT_R34 -= static_cast<T>(Tr.z);
		return (*this);
	}

	//! Returns the value at a given position
	T operator () (unsigned row, unsigned col) const
	{
		return m_mat[(col << 2) + row];
	}

	//! Applies transformation to a 3D vector (in place) - float version
	/** Input vector is directly modified after calling this method
	**/
	inline void apply(Vector3Tpl<float>& vec) const { vec = (*this)*vec; }
	//! Applies transformation to a 3D vector (in place) - double version
	/** Input vector is directly modified after calling this method
	**/
	inline void apply(Vector3Tpl<double>& vec) const { vec = (*this)*vec; }

	//! Applies transformation to a 4D vector (in place) - float version
	/** Input vector is directly modified after calling this method
	**/
	inline void apply(Tuple4Tpl<float>& vec) const { vec = (*this)*vec; }
	//! Applies transformation to a 3D vector (in place) - double version
	/** Input vector is directly modified after calling this method
	**/
	inline void apply(Tuple4Tpl<double>& vec) const { vec = (*this)*vec; }

	//! Get the resulting transformation along X dimension (float version)
	inline float applyX(const Vector3Tpl<float>& vec) const
	{
		return    static_cast<float>(CC_MAT_R11) * vec.x
				+ static_cast<float>(CC_MAT_R12) * vec.y
				+ static_cast<float>(CC_MAT_R13) * vec.z
				+ static_cast<float>(CC_MAT_R14);
	}
	//! Get the resulting transformation along X dimension (double version)
	inline double applyX(const Vector3Tpl<double>& vec) const
	{
		return    static_cast<double>(CC_MAT_R11) * vec.x
				+ static_cast<double>(CC_MAT_R12) * vec.y
				+ static_cast<double>(CC_MAT_R13) * vec.z
				+ static_cast<double>(CC_MAT_R14);
	}

	//! Get the resulting transformation along Y dimension (float version)
	inline float applyY(const Vector3Tpl<float>& vec) const
	{
		return    static_cast<float>(CC_MAT_R21) * vec.x
				+ static_cast<float>(CC_MAT_R22) * vec.y
				+ static_cast<float>(CC_MAT_R23) * vec.z
				+ static_cast<float>(CC_MAT_R24);
	}
	//! Get the resulting transformation along Y dimension (double version)
	inline double applyY(const Vector3Tpl<double>& vec) const
	{
		return    static_cast<double>(CC_MAT_R21) * vec.x
				+ static_cast<double>(CC_MAT_R22) * vec.y
				+ static_cast<double>(CC_MAT_R23) * vec.z
				+ static_cast<double>(CC_MAT_R24);
	}

	//! Get the resulting transformation along Z dimension (float version)
	inline float applyZ(const Vector3Tpl<float>& vec) const
	{
		return    static_cast<float>(CC_MAT_R31) * vec.x
				+ static_cast<float>(CC_MAT_R32) * vec.y
				+ static_cast<float>(CC_MAT_R33) * vec.z
				+ static_cast<float>(CC_MAT_R34);
	}
	//! Get the resulting transformation along Z dimension (double version)
	inline double applyZ(const Vector3Tpl<double>& vec) const
	{
		return    static_cast<double>(CC_MAT_R31) * vec.x
				+ static_cast<double>(CC_MAT_R32) * vec.y
				+ static_cast<double>(CC_MAT_R33) * vec.z
				+ static_cast<double>(CC_MAT_R34);
	}

	//! Get the resulting transformation along X dimension (float version)
	inline float applyX(const Tuple4Tpl<float>& vec) const
	{
		return    static_cast<float>(CC_MAT_R11) * vec.x
				+ static_cast<float>(CC_MAT_R12) * vec.y
				+ static_cast<float>(CC_MAT_R13) * vec.z
				+ static_cast<float>(CC_MAT_R14) * vec.w;
	}
	//! Get the resulting transformation along X dimension (double version)
	inline double applyX(const Tuple4Tpl<double>& vec) const
	{
		return    static_cast<double>(CC_MAT_R11) * vec.x
				+ static_cast<double>(CC_MAT_R12) * vec.y
				+ static_cast<double>(CC_MAT_R13) * vec.z
				+ static_cast<double>(CC_MAT_R14) * vec.w;
	}

	//! Get the resulting transformation along Y dimension (float version)
	inline float applyY(const Tuple4Tpl<float>& vec) const
	{
		return    static_cast<float>(CC_MAT_R21) * vec.x
				+ static_cast<float>(CC_MAT_R22) * vec.y
				+ static_cast<float>(CC_MAT_R23) * vec.z
				+ static_cast<float>(CC_MAT_R24) * vec.w;
	}
	//! Get the resulting transformation along Y dimension (double version)
	inline double applyY(const Tuple4Tpl<double>& vec) const
	{
		return    static_cast<double>(CC_MAT_R21) * vec.x
				+ static_cast<double>(CC_MAT_R22) * vec.y
				+ static_cast<double>(CC_MAT_R23) * vec.z
				+ static_cast<double>(CC_MAT_R24) * vec.w;
	}

	//! Get the resulting transformation along Z dimension (float version)
	inline float applyZ(const Tuple4Tpl<float>& vec) const
	{
		return    static_cast<float>(CC_MAT_R31) * vec.x
				+ static_cast<float>(CC_MAT_R32) * vec.y
				+ static_cast<float>(CC_MAT_R33) * vec.z
				+ static_cast<float>(CC_MAT_R34) * vec.w;
	}
	//! Get the resulting transformation along Z dimension (double version)
	inline double applyZ(const Tuple4Tpl<double>& vec) const
	{
		return    static_cast<double>(CC_MAT_R31) * vec.x
				+ static_cast<double>(CC_MAT_R32) * vec.y
				+ static_cast<double>(CC_MAT_R33) * vec.z
				+ static_cast<double>(CC_MAT_R34) * vec.w;
	}

	//! Get the resulting transformation along the 4th dimension (float version)
	inline float applyW(const Tuple4Tpl<float>& vec) const
	{
		return    static_cast<float>(CC_MAT_R41) * vec.x
				+ static_cast<float>(CC_MAT_R42) * vec.y
				+ static_cast<float>(CC_MAT_R43) * vec.z
				+ static_cast<float>(CC_MAT_R44) * vec.w;
	}
	//! Get the resulting transformation along the 4th dimension (double version)
	inline double applyW(const Tuple4Tpl<double>& vec) const
	{
		return    static_cast<double>(CC_MAT_R41) * vec.x
				+ static_cast<double>(CC_MAT_R42) * vec.y
				+ static_cast<double>(CC_MAT_R43) * vec.z
				+ static_cast<double>(CC_MAT_R44) * vec.w;
	}

	//! Applies rotation only to a 3D vector (in place) - float version
	/** Input vector is directly modified after calling this method
	**/
	inline void applyRotation(Vector3Tpl<float>& vec) const
	{
		float vx = vec.x;
		float vy = vec.y;
		float vz = vec.z;

		vec.x =   static_cast<float>(CC_MAT_R11) * vx
				+ static_cast<float>(CC_MAT_R12) * vy
				+ static_cast<float>(CC_MAT_R13) * vz;

		vec.y =   static_cast<float>(CC_MAT_R21) * vx
				+ static_cast<float>(CC_MAT_R22) * vy
				+ static_cast<float>(CC_MAT_R23) * vz;

		vec.z =   static_cast<float>(CC_MAT_R31) * vx
				+ static_cast<float>(CC_MAT_R32) * vy
				+ static_cast<float>(CC_MAT_R33) * vz;
	}
	//! Applies rotation only to a 3D vector (in place) - double version
	/** Input vector is directly modified after calling this method
	**/
	inline void applyRotation(Vector3Tpl<double>& vec) const
	{
		double vx = vec.x;
		double vy = vec.y;
		double vz = vec.z;

		vec.x =   static_cast<double>(CC_MAT_R11) * vx
				+ static_cast<double>(CC_MAT_R12) * vy
				+ static_cast<double>(CC_MAT_R13) * vz;

		vec.y =   static_cast<double>(CC_MAT_R21) * vx
				+ static_cast<double>(CC_MAT_R22) * vy
				+ static_cast<double>(CC_MAT_R23) * vz;

		vec.z =   static_cast<double>(CC_MAT_R31) * vx
				+ static_cast<double>(CC_MAT_R32) * vy
				+ static_cast<double>(CC_MAT_R33) * vz;
	}

	//! Applies rotation only to a 3D vector (in place) - float version
	/** Input array is directly modified after calling this method
	**/
	inline void applyRotation(float vec[3]) const
	{
		float vx = vec[0];
		float vy = vec[1];
		float vz = vec[2];

		vec[0] =  static_cast<float>(CC_MAT_R11) * vx
				+ static_cast<float>(CC_MAT_R12) * vy
				+ static_cast<float>(CC_MAT_R13) * vz;

		vec[1] =  static_cast<float>(CC_MAT_R21) * vx
				+ static_cast<float>(CC_MAT_R22) * vy
				+ static_cast<float>(CC_MAT_R23) * vz;

		vec[2] =  static_cast<float>(CC_MAT_R31) * vx
				+ static_cast<float>(CC_MAT_R32) * vy
				+ static_cast<float>(CC_MAT_R33) * vz;
	}
	//! Applies rotation only to a 3D vector (in place) - float version
	/** Input array is directly modified after calling this method
	**/
	inline void applyRotation(double vec[3]) const
	{
		double vx = vec[0];
		double vy = vec[1];
		double vz = vec[2];

		vec[0] =  static_cast<double>(CC_MAT_R11) * vx
				+ static_cast<double>(CC_MAT_R12) * vy
				+ static_cast<double>(CC_MAT_R13) * vz;

		vec[1] =  static_cast<double>(CC_MAT_R21) * vx
				+ static_cast<double>(CC_MAT_R22) * vy
				+ static_cast<double>(CC_MAT_R23) * vz;

		vec[2] =  static_cast<double>(CC_MAT_R31) * vx
				+ static_cast<double>(CC_MAT_R32) * vy
				+ static_cast<double>(CC_MAT_R33) * vz;
	}

	//! Shifts rotation center
	/** Warning, this method only applies a shift (i.e. relatively to the
		current rotation center). This is not a way to set an
		absolute rotation center 'directly'.
	**/
	void shiftRotationCenter(const Vector3Tpl<T>& vec)
	{
		//R(X-vec)+T+vec = R(X)+T + vec-R(vec)
		Vector3Tpl<T> Rvec = vec;
		applyRotation(Rvec);
		*this += (vec - Rvec);
	}

	//! Transposes matrix (in place)
	void transpose()
	{
		std::swap(CC_MAT_R21, CC_MAT_R12);
		std::swap(CC_MAT_R31, CC_MAT_R13);
		std::swap(CC_MAT_R41, CC_MAT_R14);
		std::swap(CC_MAT_R32, CC_MAT_R23);
		std::swap(CC_MAT_R42, CC_MAT_R24);
		std::swap(CC_MAT_R43, CC_MAT_R34);
	}

	//! Returns transposed matrix
	ccGLMatrixTpl<T> transposed() const
	{
		ccGLMatrixTpl<T> t(*this);
		t.transpose();
	
		return t;
	}

	//! Inverts transformation
	void invert()
	{
		//inverse scale as well!
		//we use the first column == X (its norm should be 1 for an 'unscaled' matrix ;)
		T s2 = getColumnAsVec3D(0).norm2();

		//we invert rotation
		std::swap(CC_MAT_R21, CC_MAT_R12);
		std::swap(CC_MAT_R31, CC_MAT_R13);
		std::swap(CC_MAT_R32, CC_MAT_R23);

		if (s2 != 0 && s2 != 1)
		{
			scaleRotation(1 / s2);
		}

		//eventually we invert translation
		applyRotation(m_mat + 12);
		CC_MAT_R14 = -CC_MAT_R14;
		CC_MAT_R24 = -CC_MAT_R24;
		CC_MAT_R34 = -CC_MAT_R34;

	}

	//! Returns inverse transformation
	ccGLMatrixTpl<T> inverse() const
	{
		ccGLMatrixTpl<T> t(*this);
		t.invert();

		return t;
	}

	//! Scales the rotation part of the matrix
	/**	\param coef scaling coef.
	**/
	inline void scaleRotation(T coef)
	{
		CC_MAT_R11 *= coef; CC_MAT_R12 *= coef; CC_MAT_R13 *= coef;
		CC_MAT_R21 *= coef; CC_MAT_R22 *= coef; CC_MAT_R23 *= coef;
		CC_MAT_R31 *= coef; CC_MAT_R32 *= coef; CC_MAT_R33 *= coef;
	}

	//! Scales one row of the matrix
	/** \param rowIndex tow index (0-3)
		\param coef scaling coef.
	**/
	void scaleRow(unsigned rowIndex, T coef)
	{
		assert(rowIndex < 4);
		m_mat[   rowIndex] *= coef;
		m_mat[ 4+rowIndex] *= coef;
		m_mat[ 8+rowIndex] *= coef;
		m_mat[12+rowIndex] *= coef;
	}

	//! Scales one column of the matrix
	/** \warning The last row is never scaled
		\param colIndex column index (0-3)
		\param coef scaling coef.
	**/
	void scaleColumn(unsigned colIndex, T coef)
	{
		assert(colIndex < 4);
		T* col = getColumn(colIndex);
		col[0] *= coef;
		col[1] *= coef;
		col[2] *= coef;
		//col[3] *= coef;
	}

	//inherited from ccSerializableObject
	bool isSerializable() const override { return true; }
	bool toFile(QFile& out) const override
	{
		assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));

		//data (dataVersion>=20)
		if (out.write(reinterpret_cast<const char*>(m_mat), sizeof(T)*OPENGL_MATRIX_SIZE) < 0)
			return WriteError();

		return true;
	}

	bool fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override
	{
		assert(in.isOpen() && (in.openMode() & QIODevice::ReadOnly));

		if (dataVersion < 20)
			return CorruptError();

		//data (dataVersion>=20)
		if (in.read(reinterpret_cast<char*>(m_mat), sizeof(T)*OPENGL_MATRIX_SIZE) < 0)
			return ReadError();

		return true;
	}

protected:

	//! Transfers the homogenous coordinate (scale) to the whole matrix
	void internalRescale()
	{
		if (CC_MAT_R44 != static_cast<T>(1) && CC_MAT_R44 != 0)
		{
			scaleRotation(1.0 / CC_MAT_R44);
			CC_MAT_R44 = static_cast<T>(1);
		}
	}

	//! Internal 4x4 GL-style matrix data
	T m_mat[OPENGL_MATRIX_SIZE];
};

#endif //CC_GL_MATRIX_TPL_HEADER
