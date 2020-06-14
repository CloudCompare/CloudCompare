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

#ifndef CC_INCLUDE_GL_HEADER
#define CC_INCLUDE_GL_HEADER

#include <cmath>

//Local
#include "ccGLMatrix.h"

//Qt
#include <QOpenGLFunctions_2_1>

//! Shortcuts to OpenGL commands independent on the input type
class ccGL
{
public:

	//type-less glVertex3Xv call (X=f,d)
	static inline void Vertex3v(QOpenGLFunctions_2_1* glFunc, const float* v) { glFunc->glVertex3fv(v); }
	static inline void Vertex3v(QOpenGLFunctions_2_1* glFunc, const double* v) { glFunc->glVertex3dv(v); }

	//type-less glVertex3X call (X=f,d)
	static inline void Vertex3(QOpenGLFunctions_2_1* glFunc, float x, float y, float z) { glFunc->glVertex3f(x, y, z); }
	static inline void Vertex3(QOpenGLFunctions_2_1* glFunc, double x, double y, double z) { glFunc->glVertex3d(x, y, z); }

	//type-less glScaleX call (X=f,d)
	static inline void Scale(QOpenGLFunctions_2_1* glFunc, float x, float y, float z) { glFunc->glScalef(x, y, z); }
	static inline void Scale(QOpenGLFunctions_2_1* glFunc, double x, double y, double z) { glFunc->glScaled(x, y, z); }

	//type-less glNormal3Xv call (X=f,d)
	static inline void Normal3v(QOpenGLFunctions_2_1* glFunc, const float* v) { glFunc->glNormal3fv(v); }
	static inline void Normal3v(QOpenGLFunctions_2_1* glFunc, const double* v) { glFunc->glNormal3dv(v); }

	//type-less glRotateX call (X=f,d)
	static inline void Rotate(QOpenGLFunctions_2_1* glFunc, float a, float x, float y, float z) { glFunc->glRotatef(a, x, y, z); }
	static inline void Rotate(QOpenGLFunctions_2_1* glFunc, double a, double x, double y, double z) { glFunc->glRotated(a, x, y, z); }

	//type-less glTranslateX call (X=f,d)
	static inline void Translate(QOpenGLFunctions_2_1* glFunc, float x, float y, float z) { glFunc->glTranslatef(x, y, z); }
	static inline void Translate(QOpenGLFunctions_2_1* glFunc, double x, double y, double z) { glFunc->glTranslated(x, y, z); }

	//type-less glColor3Xv call (X=f,ub)
	static inline void Color3v(QOpenGLFunctions_2_1* glFunc, const unsigned char* v) { glFunc->glColor3ubv(v); }
	static inline void Color3v(QOpenGLFunctions_2_1* glFunc, const float* v) { glFunc->glColor3fv(v); }

	//type-less glColor4Xv call (X=f,ub)
	static inline void Color4v(QOpenGLFunctions_2_1* glFunc, const unsigned char* v) { glFunc->glColor4ubv(v); }
	static inline void Color4v(QOpenGLFunctions_2_1* glFunc, const float* v) { glFunc->glColor4fv(v); }

public: //GLU equivalent methods

	static ccGLMatrixd Frustum(double left, double right, double bottom, double top, double znear, double zfar)
	{
		// invalid for: n<=0, f<=0, l=r, b=t, or n=f
		assert(znear > 0);
		assert(zfar > 0);
		assert(left != right);
		assert(bottom != top);
		assert(znear != zfar);

		ccGLMatrixd outMatrix;
		{
			double* matrix = outMatrix.data();

			double dX = right - left;
			double dY = top - bottom;
			double dZ = znear - zfar;

			matrix[0]  =  2*znear / dX;
			matrix[1]  =  0.0;
			matrix[2]  =  0.0;
			matrix[3]  =  0.0;

			matrix[4]  =  0.0;
			matrix[5]  =  2*znear / dY;
			matrix[6]  =  0.0;
			matrix[7]  =  0.0;

			matrix[8]  =  (right + left)/dX;
			matrix[9]  =  (top + bottom)/dY;
			matrix[10] =  (zfar + znear)/dZ;
			matrix[11] = -1.0;

			matrix[12] =  0.0;
			matrix[13] =  0.0;
			matrix[14] =  2*znear*zfar / dZ;
			matrix[15] =  0.0;
		}

		return outMatrix;
	}

	//inspired from https://www.opengl.org/wiki/GluPerspective_code and http://www.songho.ca/opengl/gl_projectionmatrix.html
	static ccGLMatrixd Perspective(double fovyInDegrees, double aspectRatio, double znear, double zfar)
	{
		ccGLMatrixd outMatrix;
		{
			double* matrix = outMatrix.data();

			double ymax = znear * std::tan(fovyInDegrees/2 * CC_DEG_TO_RAD);
			double xmax = ymax * aspectRatio;

			double dZ = zfar - znear;
			matrix[0]  =  znear / xmax;
			matrix[1]  =  0.0;
			matrix[2]  =  0.0;
			matrix[3]  =  0.0;

			matrix[4]  =  0.0;
			matrix[5]  =  znear / ymax;
			matrix[6]  =  0.0;
			matrix[7]  =  0.0;

			matrix[8]  =  0.0;
			matrix[9]  =  0.0;
			matrix[10] = -(zfar + znear) / dZ;
			matrix[11] = -1.0;

			matrix[12] =  0.0;
			matrix[13] =  0.0;
			matrix[14] =  -(2.0 * znear * zfar) / dZ;
			matrix[15] =  0.0;
		}

		return outMatrix;
	}

	//inspired from http://www.songho.ca/opengl/gl_projectionmatrix.html
	static ccGLMatrixd Ortho(double w, double h, double d)
	{
		ccGLMatrixd matrix;
		if (w != 0 && h != 0 && d != 0)
		{
			double* mat = matrix.data();
			mat[0]  = 1.0 / w;
			mat[1]  = 0.0;
			mat[2]  = 0.0;
			mat[3]  = 0.0;

			mat[4]  = 0.0;
			mat[5]  = 1.0 / h;
			mat[6]  = 0.0;
			mat[7]  = 0.0;

			mat[8]  = 0.0;
			mat[9]  = 0.0;
			mat[10] = - 1.0 / d;
			mat[11] = 0.0;

			mat[12] = 0.0;
			mat[13] = 0.0;
			mat[14] = 0.0;
			mat[15] = 1.0;
		}
		else
		{
			matrix.toIdentity();
		}

		return matrix;
	}

	template <typename iType, typename oType>
	static bool Project(const Vector3Tpl<iType>& input3D, const oType* modelview, const oType* projection, const int* viewport, Vector3Tpl<oType>& output2D, bool* inFrustum = nullptr)
	{
		//Modelview transform
		Tuple4Tpl<oType> Pm;
		{
			Pm.x = static_cast<oType>(modelview[0]*input3D.x + modelview[4]*input3D.y + modelview[ 8]*input3D.z + modelview[12]);
			Pm.y = static_cast<oType>(modelview[1]*input3D.x + modelview[5]*input3D.y + modelview[ 9]*input3D.z + modelview[13]);
			Pm.z = static_cast<oType>(modelview[2]*input3D.x + modelview[6]*input3D.y + modelview[10]*input3D.z + modelview[14]);
			Pm.w = static_cast<oType>(modelview[3]*input3D.x + modelview[7]*input3D.y + modelview[11]*input3D.z + modelview[15]);
		};

		//Projection transform
		Tuple4Tpl<oType> Pp;
		{
			Pp.x = static_cast<oType>(projection[0]*Pm.x + projection[4]*Pm.y + projection[ 8]*Pm.z + projection[12]*Pm.w);
			Pp.y = static_cast<oType>(projection[1]*Pm.x + projection[5]*Pm.y + projection[ 9]*Pm.z + projection[13]*Pm.w);
			Pp.z = static_cast<oType>(projection[2]*Pm.x + projection[6]*Pm.y + projection[10]*Pm.z + projection[14]*Pm.w);
			Pp.w = static_cast<oType>(projection[3]*Pm.x + projection[7]*Pm.y + projection[11]*Pm.z + projection[15]*Pm.w);
		};
		
		//The result normalizes between -1 and 1
		if (Pp.w == 0.0)
		{
			return false;
		}

		if (inFrustum)
		{
			//Check if the point is inside the frustum
			*inFrustum = (std::abs(Pp.x) <= Pp.w && std::abs(Pp.y) <= Pp.w && std::abs(Pp.z) <= Pp.w);
		}

		//Perspective division
		Pp.x /= Pp.w;
		Pp.y /= Pp.w;
		Pp.z /= Pp.w;
		//Window coordinates
		//Map x, y to range 0-1
		output2D.x = (1.0 + Pp.x) / 2 * viewport[2] + viewport[0];
		output2D.y = (1.0 + Pp.y) / 2 * viewport[3] + viewport[1];
		//This is only correct when glDepthRange(0.0, 1.0)
		output2D.z = (1.0 + Pp.z) / 2;	//Between 0 and 1

		return true;
	}
	
	inline static double MAT(const double* m, int r, int c) { return m[c*4+r]; }
	inline static float MAT(const float* m, int r, int c) { return m[c*4+r]; }

	inline static double& MAT(double* m, int r, int c) { return m[c*4+r]; }
	inline static float& MAT(float* m, int r, int c) { return m[c*4+r]; }
	
	template <typename Type>
	static bool InvertMatrix(const Type* m, Type* out)
	{
		Type wtmp[4][8];
		Type m0, m1, m2, m3, s;
		Type *r0, *r1, *r2, *r3;
		r0 = wtmp[0], r1 = wtmp[1], r2 = wtmp[2], r3 = wtmp[3];

		r0[0] = MAT(m, 0, 0), r0[1] = MAT(m, 0, 1),
		r0[2] = MAT(m, 0, 2), r0[3] = MAT(m, 0, 3),
		r0[4] = 1.0, r0[5] = r0[6] = r0[7] = 0.0,
		r1[0] = MAT(m, 1, 0), r1[1] = MAT(m, 1, 1),
		r1[2] = MAT(m, 1, 2), r1[3] = MAT(m, 1, 3),
		r1[5] = 1.0, r1[4] = r1[6] = r1[7] = 0.0,
		r2[0] = MAT(m, 2, 0), r2[1] = MAT(m, 2, 1),
		r2[2] = MAT(m, 2, 2), r2[3] = MAT(m, 2, 3),
		r2[6] = 1.0, r2[4] = r2[5] = r2[7] = 0.0,
		r3[0] = MAT(m, 3, 0), r3[1] = MAT(m, 3, 1),
		r3[2] = MAT(m, 3, 2), r3[3] = MAT(m, 3, 3),
		r3[7] = 1.0, r3[4] = r3[5] = r3[6] = 0.0;
		
		//choose pivot - or die
		if (std::abs(r3[0]) > std::abs(r2[0]))
			std::swap(r3, r2);
		if (std::abs(r2[0]) > std::abs(r1[0]))
			std::swap(r2, r1);
		if (std::abs(r1[0]) > std::abs(r0[0]))
			std::swap(r1, r0);
		if (0.0 == r0[0])
			return false;
		
		//eliminate first variable
		m1 = r1[0] / r0[0];
		m2 = r2[0] / r0[0];
		m3 = r3[0] / r0[0];
		s = r0[1];
		r1[1] -= m1 * s;
		r2[1] -= m2 * s;
		r3[1] -= m3 * s;
		s = r0[2];
		r1[2] -= m1 * s;
		r2[2] -= m2 * s;
		r3[2] -= m3 * s;
		s = r0[3];
		r1[3] -= m1 * s;
		r2[3] -= m2 * s;
		r3[3] -= m3 * s;
		s = r0[4];
		if (s != 0.0)
		{
			r1[4] -= m1 * s;
			r2[4] -= m2 * s;
			r3[4] -= m3 * s;
		}
		s = r0[5];
		if (s != 0.0)
		{
			r1[5] -= m1 * s;
			r2[5] -= m2 * s;
			r3[5] -= m3 * s;
		}
		s = r0[6];
		if (s != 0.0)
		{
			r1[6] -= m1 * s;
			r2[6] -= m2 * s;
			r3[6] -= m3 * s;
		}
		s = r0[7];
		if (s != 0.0)
		{
			r1[7] -= m1 * s;
			r2[7] -= m2 * s;
			r3[7] -= m3 * s;
		}
		
		//choose pivot - or die
		if (std::abs(r3[1]) > std::abs(r2[1]))
			std::swap(r3, r2);
		if (std::abs(r2[1]) > std::abs(r1[1]))
			std::swap(r2, r1);
		if (0.0 == r1[1])
			return false;
		
		//eliminate second variable
		m2 = r2[1] / r1[1];
		m3 = r3[1] / r1[1];
		r2[2] -= m2 * r1[2];
		r3[2] -= m3 * r1[2];
		r2[3] -= m2 * r1[3];
		r3[3] -= m3 * r1[3];
		s = r1[4];
		if (0.0 != s)
		{
			r2[4] -= m2 * s;
			r3[4] -= m3 * s;
		}
		s = r1[5];
		if (0.0 != s)
		{
			r2[5] -= m2 * s;
			r3[5] -= m3 * s;
		}
		s = r1[6];
		if (0.0 != s)
		{
			r2[6] -= m2 * s;
			r3[6] -= m3 * s;
		}
		s = r1[7];
		if (0.0 != s)
		{
			r2[7] -= m2 * s;
			r3[7] -= m3 * s;
		}
		
		//choose pivot - or die
		if (std::abs(r3[2]) > std::abs(r2[2]))
			std::swap(r3, r2);
		if (0.0 == r2[2])
			return false;
		
		//eliminate third variable
		m3 = r3[2] / r2[2];
		r3[3] -= m3 * r2[3], r3[4] -= m3 * r2[4],
		r3[5] -= m3 * r2[5], r3[6] -= m3 * r2[6], r3[7] -= m3 * r2[7];
		
		//last check
		if (0.0 == r3[3])
			return false;
		
		s = 1.0 / r3[3]; //now back substitute row 3
		r3[4] *= s;
		r3[5] *= s;
		r3[6] *= s;
		r3[7] *= s;
		m2 = r2[3]; //now back substitute row 2
		s = 1.0 / r2[2];
		r2[4] = s * (r2[4] - r3[4] * m2), r2[5] = s * (r2[5] - r3[5] * m2),
		r2[6] = s * (r2[6] - r3[6] * m2), r2[7] = s * (r2[7] - r3[7] * m2);
		m1 = r1[3];
		r1[4] -= r3[4] * m1, r1[5] -= r3[5] * m1,
		r1[6] -= r3[6] * m1, r1[7] -= r3[7] * m1;
		m0 = r0[3];
		r0[4] -= r3[4] * m0, r0[5] -= r3[5] * m0,
		r0[6] -= r3[6] * m0, r0[7] -= r3[7] * m0;
		m1 = r1[2]; //now back substitute row 1
		s = 1.0 / r1[1];
		r1[4] = s * (r1[4] - r2[4] * m1), r1[5] = s * (r1[5] - r2[5] * m1),
		r1[6] = s * (r1[6] - r2[6] * m1), r1[7] = s * (r1[7] - r2[7] * m1);
		m0 = r0[2];
		r0[4] -= r2[4] * m0, r0[5] -= r2[5] * m0,
		r0[6] -= r2[6] * m0, r0[7] -= r2[7] * m0;
		m0 = r0[1]; //now back substitute row 0
		s = 1.0 / r0[0];
		r0[4] = s * (r0[4] - r1[4] * m0), r0[5] = s * (r0[5] - r1[5] * m0),
		r0[6] = s * (r0[6] - r1[6] * m0), r0[7] = s * (r0[7] - r1[7] * m0);
		
		MAT(out, 0, 0) = r0[4];
		MAT(out, 0, 1) = r0[5], MAT(out, 0, 2) = r0[6];
		MAT(out, 0, 3) = r0[7], MAT(out, 1, 0) = r1[4];
		MAT(out, 1, 1) = r1[5], MAT(out, 1, 2) = r1[6];
		MAT(out, 1, 3) = r1[7], MAT(out, 2, 0) = r2[4];
		MAT(out, 2, 1) = r2[5], MAT(out, 2, 2) = r2[6];
		MAT(out, 2, 3) = r2[7], MAT(out, 3, 0) = r3[4];
		MAT(out, 3, 1) = r3[5], MAT(out, 3, 2) = r3[6];
		MAT(out, 3, 3) = r3[7];
		
		return true;
	}

	template <typename iType, typename oType>
	static bool Unproject(const Vector3Tpl<iType>& input2D, const oType* modelview, const oType* projection, const int* viewport, Vector3Tpl<oType>& output3D)
	{
		//compute projection x modelview
		ccGLMatrixTpl<oType> A = ccGLMatrixTpl<oType>(projection) * ccGLMatrixTpl<oType>(modelview);
		ccGLMatrixTpl<oType> m;

		if (!InvertMatrix(A.data(), m.data()))
		{
			return false;
		}

		ccGLMatrixTpl<oType> mA = m * A;

		//Transformation of normalized coordinates between -1 and 1
		Tuple4Tpl<oType> in;
		in.x = static_cast<oType>((input2D.x - static_cast<iType>(viewport[0])) / viewport[2] * 2 - 1);
		in.y = static_cast<oType>((input2D.y - static_cast<iType>(viewport[1])) / viewport[3] * 2 - 1);
		in.z = static_cast<oType>(2*input2D.z - 1);
		in.w = 1;

		//Objects coordinates
		Tuple4Tpl<oType> out = m * in;
		if (out.w == 0)
		{
			return false;
		}

		output3D = Vector3Tpl<oType>(out.u) / out.w;

		return true;
	}

	static void PickMatrix(double x, double y, double width, double height, int viewport[4], double m[16])
	{
		double sx = viewport[2] / width;
		double sy = viewport[3] / height;
		double tx = (viewport[2] + 2.0 * (viewport[0] - x)) / width;
		double ty = (viewport[3] + 2.0 * (viewport[1] - y)) / height;

		MAT(m, 0, 0) = sx;
		MAT(m, 0, 1) = 0.0;
		MAT(m, 0, 2) = 0.0;
		MAT(m, 0, 3) = tx;
		MAT(m, 1, 0) = 0.0;
		MAT(m, 1, 1) = sy;
		MAT(m, 1, 2) = 0.0;
		MAT(m, 1, 3) = ty;
		MAT(m, 2, 0) = 0.0;
		MAT(m, 2, 1) = 0.0;
		MAT(m, 2, 2) = 1.0;
		MAT(m, 2, 3) = 0.0;
		MAT(m, 3, 0) = 0.0;
		MAT(m, 3, 1) = 0.0;
		MAT(m, 3, 2) = 0.0;
		MAT(m, 3, 3) = 1.0;
	}
};

#endif //CC_INCLUDE_GL_HEADER
