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

#include "ccGLMatrix.h"

//System
#include <assert.h>
#include <math.h>

//CCLib
#include <CCConst.h>

//Matrix element shortcuts in (line,column) order
#define R11 m_mat[0]
#define R21 m_mat[1]
#define R31 m_mat[2]
#define R41 m_mat[3]

#define R12 m_mat[4]
#define R22 m_mat[5]
#define R32 m_mat[6]
#define R42 m_mat[7]

#define R13 m_mat[8]
#define R23 m_mat[9]
#define R33 m_mat[10]
#define R43 m_mat[11]

#define R14 m_mat[12]
#define R24 m_mat[13]
#define R34 m_mat[14]
#define R44 m_mat[15]

ccGLMatrix::ccGLMatrix()
{
	toIdentity();
}

ccGLMatrix::ccGLMatrix(const float* mat16)
{
    memcpy(m_mat, mat16, sizeof(float)*16);
}

ccGLMatrix::ccGLMatrix(const ccGLMatrix& mat)
{
	memcpy(m_mat, mat.m_mat, sizeof(float)*16);
}

ccGLMatrix::ccGLMatrix(const CCVector3& X, const CCVector3& Y, const CCVector3& Z, const CCVector3& T)
{
	m_mat[0] = X[0];
	m_mat[1] = X[1];
	m_mat[2] = X[2];
	m_mat[3] = 0.0;

	m_mat[4] = Y[0];
	m_mat[5] = Y[1];
	m_mat[6] = Y[2];
	m_mat[7] = 0.0;

	m_mat[8] = Z[0];
	m_mat[9] = Z[1];
	m_mat[10] = Z[2];
	m_mat[11] = 0.0;

	m_mat[12] = T[0];
	m_mat[13] = T[1];
	m_mat[14] = T[2];
	m_mat[15] = 1.0;
}

ccGLMatrix::ccGLMatrix(const CCLib::SquareMatrix& R, const CCVector3& T)
{
    toIdentity();

	if (R.size()==3)
	{
		//we copy each column
		float* mat = m_mat;
		for (unsigned j=0;j<3;++j)
		{
			*mat++ = (float)R.m_values[0][j];
			*mat++ = (float)R.m_values[1][j];
			*mat++ = (float)R.m_values[2][j];
			mat++;
		}
	}

    *this += T;
}

ccGLMatrix::ccGLMatrix(const CCLib::SquareMatrix& R, const CCVector3& T, const CCVector3& rotCenter)
{
    *this = ccGLMatrix(R,T);
    shiftRotationCenter(rotCenter);
}

void ccGLMatrix::toZero()
{
	memset(m_mat,0,16*sizeof(float));
}

void ccGLMatrix::toIdentity()
{
	memset(m_mat,0,16*sizeof(float));
	R11=R22=R33=R44=1.0;
}

bool ccGLMatrix::toAsciiFile(const char* filename) const
{
	FILE* fp = fopen(filename,"wt");
	if (!fp)
		return false;

	const float* mat = m_mat;
	for (int i=0;i<4;++i)
	{
		if (fprintf(fp,"%f %f %f %f\n",mat[0],mat[4],mat[8],mat[12])<0)
		{
			fclose(fp);
			return false;
		}
		++mat;
	}
	fclose(fp);

	return true;
}

bool ccGLMatrix::fomAsciiFile(const char* filename)
{
	FILE* fp = fopen(filename,"rt");
	if (!fp)
		return false;

	float* mat = m_mat;
	for (int i=0;i<4;++i)
	{
		if (fscanf(fp,"%f %f %f %f\n",mat,mat+4,mat+8,mat+12)<4)
		{
			fclose(fp);
			return false;
		}
		++mat;
	}
	fclose(fp);

	return true;
}

void ccGLMatrix::initFromParameters(float alpha, const CCVector3& axis3D, const CCVector3& t3D)
{
	float cosw = (float)cos(alpha);
	float sinw = (float)sin(alpha);
	float inv_cosw = 1.0f-cosw;

	float l1 = (float)axis3D.x;
	float l2 = (float)axis3D.y;
	float l3 = (float)axis3D.z;

	float l1_inv_cosw = l1*inv_cosw;
	float l3_inv_cosw = l3*inv_cosw;

	//1st column
	R11 = cosw+l1*l1_inv_cosw;
	R21 = l2*l1_inv_cosw+l3*sinw;
	R31 = l3*l1_inv_cosw-l2*sinw;

	//2nd column
	R12 = l2*l1_inv_cosw-l3*sinw;
	R22 = cosw+l2*l2*inv_cosw;
	R32 = l2*l3_inv_cosw+l1*sinw;

	//3rd column
	R13 = l3*l1_inv_cosw+l2*sinw;
	R23 = l2*l3_inv_cosw-l1*sinw;
	R33 = cosw+l3*l3_inv_cosw;

	//4th column
	R14 = t3D.x;
	R24 = t3D.y;
	R34 = t3D.z;
}

void ccGLMatrix::getParameters(float& alpha, CCVector3& axis3D, CCVector3& t3D) const
{
	float trace = R11 + R22 + R33;
	trace = 0.5f*(trace-1.0f);
	if (fabs(trace)<1.0)
	{
		alpha = acos(trace);
		if (alpha > (float)M_PI_2)
			alpha -= (float)M_PI;
	}
	else
		alpha = 0.0;

	axis3D.x = (float)(R32-R23);
	axis3D.y = (float)(R13-R31);
	axis3D.z = (float)(R21-R12);
	axis3D.normalize();

	t3D.x = R14;
	t3D.y = R24;
	t3D.z = R34;
}

void ccGLMatrix::initFromParameters(float phi, float theta, float psi, const CCVector3& t3D)
{
	float cos_phi = (float)cos(phi);
	float cos_theta = (float)cos(theta);
	float cos_psi = (float)cos(psi);

	float sin_phi = (float)sin(phi);
	float sin_theta = (float)sin(theta);
	float sin_psi = (float)sin(psi);

	//1st column
	R11 = cos_theta*cos_phi;
	R21 = cos_theta*sin_phi;
	R31 = -sin_theta;

	//2nd column
	R12 = sin_psi*sin_theta*cos_phi-cos_psi*sin_phi;
	R22 = sin_psi*sin_theta*sin_phi+cos_psi*cos_phi;
	R32 = sin_psi*cos_theta;

	//3rd column
	R13 = cos_psi*sin_theta*cos_phi+sin_psi*sin_phi;
	R23 = cos_psi*sin_theta*sin_phi-sin_psi*cos_phi;
	R33 = cos_psi*cos_theta;

	//4th column
	R14 = (float)t3D.x;
	R24 = (float)t3D.y;
	R34 = (float)t3D.z;
}

void ccGLMatrix::getParameters(float &phi, float &theta, float &psi, CCVector3& t3D) const
{
	if (fabs(R31)!=1.0)
	{
		theta = -asin(R31);
		float cos_theta = cos(theta);
		psi = atan2(R32/cos_theta,R33/cos_theta);
		phi = atan2(R21/cos_theta,R11/cos_theta);

		//Other solution
		/*theta = M_PI+asin(R31);
		UL_SCALAR cos_theta = cos(theta);
		psi = atan2(R32/cos_theta,R33/cos_theta);
		phi = atan2(R21/cos_theta,R11/cos_theta);
		//*/
	}
	else
	{
		phi = 0.0;
		float sign = (float)(R31 == -1.0 ? 1.0 : -1.0);
		theta = sign*(float)M_PI_2;
		psi = sign*atan2(R12,R13);
	}

	t3D.x = (float)R14;
	t3D.y = (float)R24;
	t3D.z = (float)R34;
}

void ccGLMatrix::clearTranslation()
{
	R14 = R24 = R34 = 0;
}

void ccGLMatrix::setTranslation(const CCVector3& T)
{
	R14 = (float)T.x;
	R24 = (float)T.y;
	R34 = (float)T.z;
}

ccGLMatrix ccGLMatrix::operator * (const ccGLMatrix& M) const
{
	ccGLMatrix result;

	const float* A = m_mat;
	const float* B = M.m_mat;
	float* C = result.m_mat;

	for (int j=0;j<4;++j)
	{
		for (int i=0;i<4;++i)
			*C++ = A[i]*B[0]+A[i+4]*B[1]+A[i+8]*B[2]+A[i+12]*B[3];

		B+=4;
	}

	return result;
}

ccGLMatrix& ccGLMatrix::operator *= (const ccGLMatrix& M)
{
	ccGLMatrix temp = (*this) * M;
	(*this) = temp;

	return (*this);
}

ccGLMatrix& ccGLMatrix::operator += (const CCVector3& T)
{
	R14 += (float)T.x;
	R24 += (float)T.y;
	R34 += (float)T.z;

	return (*this);
}

ccGLMatrix& ccGLMatrix::operator -= (const CCVector3& T)
{
	R14 -= (float)T.x;
	R24 -= (float)T.y;
	R34 -= (float)T.z;

	return (*this);
}

void ccGLMatrix::shiftRotationCenter(const CCVector3& vec)
{
    //R(X-vec)+T+vec = R(X)+T + vec-R(vec)
    CCVector3 Rvec = vec;
    applyRotation(Rvec.u);
    *this += (vec - Rvec);
}

void ccGLMatrix::transpose()
{
	std::swap(R21,R12);
	std::swap(R31,R13);
	std::swap(R41,R14);
	std::swap(R32,R23);
	std::swap(R42,R24);
	std::swap(R43,R34);
}

ccGLMatrix ccGLMatrix::transposed() const
{
	ccGLMatrix t(*this);
	t.transpose();
	
	return t;
}

void ccGLMatrix::invert()
{
	//we invert rotation
	std::swap(R21,R12);
	std::swap(R31,R13);
	std::swap(R32,R23);

	//we invert translation
	applyRotation(m_mat+12);
	R14 = -R14;
	R24 = -R24;
	R34 = -R34;
 }

ccGLMatrix ccGLMatrix::inverse() const
{
	ccGLMatrix result = *this;
	result.invert();

	return result;
}

ccGLMatrix ccGLMatrix::Interpolate(float coef, const ccGLMatrix& glMat1, const ccGLMatrix& glMat2)
{
	//we compute the transformation matrix between glMat1 and glMat2
	ccGLMatrix invTrans1 = glMat1.inverse();
	ccGLMatrix m12 = invTrans1 * glMat2;

    CCVector3 axis,tr;
	float alpha;
	m12.getParameters(alpha,axis,tr);

	//we only have to interpolate the angle value
	alpha *= coef;
	//and the translation
	tr *= coef;

	//we build-up the resulting matrix
	m12.initFromParameters(alpha,axis,tr);

	//eventually we build-up resulting transformation
	return glMat1 * m12;
}

void ccGLMatrix::scale(float coef)
{
	for (int i=0;i<16;++i)
		m_mat[i] *= coef;
}

void ccGLMatrix::scaleLine(unsigned lineIndex, float coef)
{
	assert(lineIndex<4);

	m_mat[   lineIndex] *= coef;
	m_mat[ 4+lineIndex] *= coef;
	m_mat[ 8+lineIndex] *= coef;
	m_mat[12+lineIndex] *= coef;
}

void ccGLMatrix::scaleColumn(unsigned colIndex, float coef)
{
	assert(colIndex<4);

	float* col = getColumn(colIndex);
	col[0] *= coef;
	col[1] *= coef;
	col[2] *= coef;
	col[3] *= coef;
}

ccGLMatrix ccGLMatrix::FromToRotation(const CCVector3& from, const CCVector3& to)
{
	float e = from.dot(to);
	float f = (e < 0 ? -e : e);
	ccGLMatrix result;
	float* mat = result.data();

	if (f > 1.0-ZERO_TOLERANCE)     //"from" and "to"-vector almost parallel
	{
		CCVector3 x;       // vector most nearly orthogonal to "from"
		x.x = (from.x > 0 ? from.x : -from.x);
		x.y = (from.y > 0 ? from.y : -from.y);
		x.z = (from.z > 0 ? from.z : -from.z);

		if (x.x < x.y)
		{
			if (x.x < x.z)
			{
				x.x = 1.0f; x.y = x.z = 0;
			}
			else
			{
				x.z = 1.0f; x.x = x.y = 0;
			}
		}
		else
		{
			if (x.y < x.z)
			{
				x.y = 1.0f; x.x = x.z = 0;
			}
			else
			{
				x.z = 1.0f; x.x = x.y = 0;
			}
		}

		CCVector3 u(x.x-from.x, x.y-from.y, x.z-from.z);
		CCVector3 v(x.x-to.x, x.y-to.y, x.z-to.z);

		float c1 = 2.0f / u.dot(u);
		float c2 = 2.0f / v.dot(v);
		float c3 = c1 * c2  * u.dot(v);

		for (unsigned i = 0; i < 3; i++)
		{
			for (unsigned j = 0; j < 3; j++)
			{
				mat[i*4+j]=  c3 * v.u[i] * u.u[j]
						   - c2 * v.u[i] * v.u[j]
						   - c1 * u.u[i] * u.u[j];
			}
			mat[i*4+i] += 1.0f;
		}
	}
	else  // the most common case, unless "from"="to", or "from"=-"to"
	{
		//hand optimized version (9 mults less)
		CCVector3 v = from.cross(to);
		float h = 1.0f/(1.0f + e);
		float hvx = h * v.x;
		float hvz = h * v.z;
		float hvxy = hvx * v.y;
		float hvxz = hvx * v.z;
		float hvyz = hvz * v.y;

		mat[0] = e + hvx * v.x;
		mat[1] = hvxy - v.z;
		mat[2] = hvxz + v.y;

		mat[4] = hvxy + v.z;
		mat[5] = e + h * v.y * v.y;
		mat[6] = hvyz - v.x;

		mat[8] = hvxz - v.y;
		mat[9] = hvyz + v.x;
		mat[10] = e + hvz * v.z;
	}

	return result;
}

bool ccGLMatrix::toFile(QFile& out) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));

	//data (dataVersion>=20)
	if (out.write((const char*)m_mat,sizeof(float)*16)<0)
		return WriteError();

	return true;
}

bool ccGLMatrix::fromFile(QFile& in, short dataVersion)
{
	assert(in.isOpen() && (in.openMode() & QIODevice::ReadOnly));

	if (dataVersion<20)
		return CorruptError();

	//data (dataVersion>=20)
	if (in.read((char*)m_mat,sizeof(float)*16)<0)
		return ReadError();

	return true;
}
