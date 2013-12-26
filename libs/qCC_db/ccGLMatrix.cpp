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

#include "ccGLMatrix.h"

//CCLib
#include <CCConst.h>

//Qt
#include <QStringList>
#include <QRegExp>

//System
#include <math.h>
#include <string.h>
#include <assert.h>

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

ccGLMatrix::ccGLMatrix(const float* mat16f)
{
    memcpy(m_mat, mat16f, sizeof(float)*OPENGL_MATRIX_SIZE);
}

ccGLMatrix::ccGLMatrix(const double* mat16d)
{
	for (unsigned i=0; i<16; ++i)
		m_mat[i] = static_cast<float>(mat16d[i]);
}

ccGLMatrix::ccGLMatrix(const ccGLMatrix& mat)
{
	memcpy(m_mat, mat.m_mat, sizeof(float)*OPENGL_MATRIX_SIZE);
}

ccGLMatrix::ccGLMatrix(const CCVector3& X, const CCVector3& Y, const CCVector3& Z, const CCVector3& T)
{
	R11 = static_cast<float>(X.x);
	R21 = static_cast<float>(X.y);
	R31 = static_cast<float>(X.z);
	R41 = 0;

	R12 = static_cast<float>(Y.x);
	R22 = static_cast<float>(Y.y);
	R32 = static_cast<float>(Y.z);
	R42 = 0;

	R13 = static_cast<float>(Z.x);
	R23 = static_cast<float>(Z.y);
	R33 = static_cast<float>(Z.z);
	R43 = 0;

	R14 = static_cast<float>(T.x);
	R24 = static_cast<float>(T.y);
	R34 = static_cast<float>(T.z);
	R44 = 1;
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
			*mat++ = static_cast<float>(R.m_values[0][j]);
			*mat++ = static_cast<float>(R.m_values[1][j]);
			*mat++ = static_cast<float>(R.m_values[2][j]);
			mat++;
		}
	}

    *this += T;
}

ccGLMatrix::ccGLMatrix(const CCLib::SquareMatrix& R, const CCVector3& T, PointCoordinateType S)
{
    toIdentity();

    if (R.size() == 3)
    {
        //we copy each column
        float* mat = m_mat;
        for (unsigned j=0; j<3; ++j)
        {
            *mat++ = static_cast<float>(R.m_values[0][j] * S);
            *mat++ = static_cast<float>(R.m_values[1][j] * S);
            *mat++ = static_cast<float>(R.m_values[2][j] * S);
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

ccGLMatrix ccGLMatrix::FromString(QString matText, bool& success)
{
	QStringList valuesStr = matText.split(QRegExp("\\s+"),QString::SkipEmptyParts);
	if (valuesStr.size() != 16)
	{
		success = false;
		return ccGLMatrix();
	}

	ccGLMatrix matrix;
	float* matValues = matrix.data();
	for (int i=0; i<16; ++i)
	{
		matValues[i] = valuesStr[(i%4)*4+(i>>2)].toFloat(&success);
		if (!success)
			return ccGLMatrix();
	}

	success = true;
	return matrix;
}

void ccGLMatrix::toZero()
{
	memset(m_mat,0,OPENGL_MATRIX_SIZE*sizeof(float));
}

void ccGLMatrix::toIdentity()
{
	memset(m_mat,0,OPENGL_MATRIX_SIZE*sizeof(float));
	R11=R22=R33=R44=1.0;
}

bool ccGLMatrix::toAsciiFile(const char* filename) const
{
	FILE* fp = fopen(filename,"wt");
	if (!fp)
		return false;

	const float* mat = m_mat;
	for (unsigned i=0; i<4; ++i)
	{
		if (fprintf(fp,"%f %f %f %f\n",mat[0],mat[4],mat[8],mat[12]) < 4)
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
	for (unsigned i=0; i<4; ++i)
	{
		if (fscanf(fp,"%f %f %f %f\n",mat,mat+4,mat+8,mat+12) < 4)
		{
			fclose(fp);
			return false;
		}
		++mat;
	}
	fclose(fp);

	return true;
}

void ccGLMatrix::initFromParameters(PointCoordinateType alpha_rad, const CCVector3& axis3D, const CCVector3& t3D)
{
	PointCoordinateType cosw = cos(alpha_rad);
	PointCoordinateType sinw = sin(alpha_rad);
	PointCoordinateType inv_cosw = 1 - cosw;

	//normalize rotation axis
	CCVector3 uAxis3D = CCVector3(0,0,1);
	PointCoordinateType n2 = axis3D.norm2();
	if (n2 > ZERO_TOLERANCE)
		uAxis3D = axis3D / sqrt(n2);

	uAxis3D.normalize();
	const PointCoordinateType& l1 = axis3D.x;
	const PointCoordinateType& l2 = axis3D.y;
	const PointCoordinateType& l3 = axis3D.z;

	PointCoordinateType l1_inv_cosw = l1*inv_cosw;
	PointCoordinateType l3_inv_cosw = l3*inv_cosw;

	//1st column
	R11 = static_cast<float>(cosw+l1*l1_inv_cosw);
	R21 = static_cast<float>(l2*l1_inv_cosw+l3*sinw);
	R31 = static_cast<float>(l3*l1_inv_cosw-l2*sinw);

	//2nd column
	R12 = static_cast<float>(l2*l1_inv_cosw-l3*sinw);
	R22 = static_cast<float>(cosw+l2*l2*inv_cosw);
	R32 = static_cast<float>(l2*l3_inv_cosw+l1*sinw);

	//3rd column
	R13 = static_cast<float>(l3*l1_inv_cosw+l2*sinw);
	R23 = static_cast<float>(l2*l3_inv_cosw-l1*sinw);
	R33 = static_cast<float>(cosw+l3*l3_inv_cosw);

	//4th column
	R14 = static_cast<float>(t3D.x);
	R24 = static_cast<float>(t3D.y);
	R34 = static_cast<float>(t3D.z);
}

void ccGLMatrix::getParameters(PointCoordinateType& alpha_rad, CCVector3& axis3D, CCVector3& t3D) const
{
	PointCoordinateType trace = R11 + R22 + R33;
	trace = (trace - 1)/2;
	if (fabs(trace) < 1)
	{
		alpha_rad = acos(trace);
		if (alpha_rad > static_cast<PointCoordinateType>(M_PI_2))
			alpha_rad -= static_cast<PointCoordinateType>(M_PI);
	}
	else
	{
		alpha_rad = 0;
	}

	axis3D.x = static_cast<PointCoordinateType>(R32-R23);
	axis3D.y = static_cast<PointCoordinateType>(R13-R31);
	axis3D.z = static_cast<PointCoordinateType>(R21-R12);
	PointCoordinateType n2 = axis3D.norm2();
	if (n2 > ZERO_TOLERANCE)
	{
		axis3D /= sqrt(n2);
	}
	else
	{
		//axis is too small!
		axis3D = CCVector3(0,0,1);
	}

	t3D.x = static_cast<PointCoordinateType>(R14);
	t3D.y = static_cast<PointCoordinateType>(R24);
	t3D.z = static_cast<PointCoordinateType>(R34);
}

void ccGLMatrix::initFromParameters(PointCoordinateType phi_rad,
									PointCoordinateType theta_rad,
									PointCoordinateType psi_rad,
									const CCVector3& t3D)
{
	PointCoordinateType cos_phi =	cos(phi_rad);
	PointCoordinateType cos_theta =	cos(theta_rad);
	PointCoordinateType cos_psi =	cos(psi_rad);

	PointCoordinateType sin_phi =	sin(phi_rad);
	PointCoordinateType sin_theta =	sin(theta_rad);
	PointCoordinateType sin_psi =	sin(psi_rad);

	//1st column
	R11 = static_cast<float>(cos_theta*cos_phi);
	R21 = static_cast<float>(cos_theta*sin_phi);
	R31 = static_cast<float>(-sin_theta);

	//2nd column
	R12 = static_cast<float>(sin_psi*sin_theta*cos_phi-cos_psi*sin_phi);
	R22 = static_cast<float>(sin_psi*sin_theta*sin_phi+cos_psi*cos_phi);
	R32 = static_cast<float>(sin_psi*cos_theta);

	//3rd column
	R13 = static_cast<float>(cos_psi*sin_theta*cos_phi+sin_psi*sin_phi);
	R23 = static_cast<float>(cos_psi*sin_theta*sin_phi-sin_psi*cos_phi);
	R33 = static_cast<float>(cos_psi*cos_theta);

	//4th column
	R14 = static_cast<float>(t3D.x);
	R24 = static_cast<float>(t3D.y);
	R34 = static_cast<float>(t3D.z);
}

void ccGLMatrix::getParameters(	PointCoordinateType &phi_rad,
								PointCoordinateType &theta_rad,
								PointCoordinateType &psi_rad,
								CCVector3& t3D) const
{
	if (fabs(R31) != 1)
	{
		theta_rad = -static_cast<PointCoordinateType>(asin(R31));
		PointCoordinateType cos_theta = cos(theta_rad);
		psi_rad = atan2(static_cast<PointCoordinateType>(R32)/cos_theta, static_cast<PointCoordinateType>(R33)/cos_theta);
		phi_rad = atan2(static_cast<PointCoordinateType>(R21)/cos_theta, static_cast<PointCoordinateType>(R11)/cos_theta);

		//Other solution
		/*theta = M_PI+asin(R31);
		PointCoordinateType cos_theta = cos(theta);
		psi = atan2(R32/cos_theta,R33/cos_theta);
		phi = atan2(R21/cos_theta,R11/cos_theta);
		//*/
	}
	else
	{
		phi_rad = 0;

		PointCoordinateType sign = (R31 == -1 ? PC_ONE : -PC_ONE);
		theta_rad = sign*static_cast<PointCoordinateType>(M_PI_2);
		psi_rad = sign*static_cast<PointCoordinateType>(atan2(R12,R13));
	}

	t3D.x = static_cast<PointCoordinateType>(R14);
	t3D.y = static_cast<PointCoordinateType>(R24);
	t3D.z = static_cast<PointCoordinateType>(R34);
}

void ccGLMatrix::clearTranslation()
{
	R14 = R24 = R34 = 0;
}

void ccGLMatrix::setTranslation(const CCVector3& T)
{
	R14 = static_cast<float>(T.x);
	R24 = static_cast<float>(T.y);
	R34 = static_cast<float>(T.z);
}

void ccGLMatrix::setTranslation(const float T[3])
{
	R14 = T[0];
	R24 = T[1];
	R34 = T[2];
}

ccGLMatrix ccGLMatrix::operator * (const ccGLMatrix& M) const
{
	ccGLMatrix result;

	const float* A = m_mat;
	const float* B = M.m_mat;
	float* C = result.m_mat;

	for (unsigned j=0; j<4; ++j, B+=4)
		for (unsigned i=0; i<4; ++i)
			*C++ = A[i]*B[0]+A[i+4]*B[1]+A[i+8]*B[2]+A[i+12]*B[3];

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
	R14 += static_cast<float>(T.x);
	R24 += static_cast<float>(T.y);
	R34 += static_cast<float>(T.z);

	return (*this);
}

ccGLMatrix& ccGLMatrix::operator -= (const CCVector3& T)
{
	R14 -= static_cast<float>(T.x);
	R24 -= static_cast<float>(T.y);
	R34 -= static_cast<float>(T.z);

	return (*this);
}

void ccGLMatrix::shiftRotationCenter(const CCVector3& vec)
{
    //R(X-vec)+T+vec = R(X)+T + vec-R(vec)
    CCVector3 Rvec = vec;
    applyRotation(Rvec);
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
	//inverse scale as well!
	//we use the first column == X (its norm should be 1 for an 'unscaled' matrix ;)
	float s2 = static_cast<float>(getColumnAsVec3D(0).norm2());

	//we invert rotation
	std::swap(R21,R12);
	std::swap(R31,R13);
	std::swap(R32,R23);

	if (s2 != 0 && s2 != 1)
	{
		R11 /= s2; R12 /= s2; R13 /= s2;
		R21 /= s2; R22 /= s2; R23 /= s2;
		R31 /= s2; R32 /= s2; R33 /= s2;
	}

	//eventually we invert translation
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

ccGLMatrix ccGLMatrix::Interpolate(	PointCoordinateType coef,
									const ccGLMatrix& glMat1,
									const ccGLMatrix& glMat2)
{
	//we compute the transformation matrix between glMat1 and glMat2
	ccGLMatrix invTrans1 = glMat1.inverse();
	ccGLMatrix m12 = invTrans1 * glMat2;

    CCVector3 axis,tr;
	PointCoordinateType alpha;
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
	for (unsigned i=0; i<OPENGL_MATRIX_SIZE; ++i)
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
	PointCoordinateType e = from.dot(to);
	PointCoordinateType f = (e < 0 ? -e : e);
	ccGLMatrix result;

	if (f > 1.0-ZERO_TOLERANCE) //"from" and "to"-vector almost parallel
	{
		CCVector3 x; // vector most nearly orthogonal to "from"
		x.x = (from.x > 0 ? from.x : -from.x);
		x.y = (from.y > 0 ? from.y : -from.y);
		x.z = (from.z > 0 ? from.z : -from.z);

		if (x.x < x.y)
		{
			if (x.x < x.z)
			{
				x.x = 1; x.y = x.z = 0;
			}
			else
			{
				x.z = 1; x.x = x.y = 0;
			}
		}
		else
		{
			if (x.y < x.z)
			{
				x.y = 1; x.x = x.z = 0;
			}
			else
			{
				x.z = 1; x.x = x.y = 0;
			}
		}

		CCVector3 u = x-from;
		CCVector3 v = x-to;

		PointCoordinateType c1 = 2 / u.dot(u);
		PointCoordinateType c2 = 2 / v.dot(v);
		PointCoordinateType c3 = c1 * c2  * u.dot(v);

		float* mat = result.data();
		for (unsigned i=0; i<3; i++)
		{
			for (unsigned j=0; j<3; j++)
			{
				mat[i*4+j] = static_cast<float>(  c3 * v.u[i] * u.u[j]
												- c2 * v.u[i] * v.u[j]
												- c1 * u.u[i] * u.u[j]);
			}
			mat[i*4+i] += 1.0f;
		}
	}
	else  // the most common case, unless "from"="to", or "from"=-"to"
	{
		//hand optimized version (9 mults less)
		CCVector3 v = from.cross(to);
		PointCoordinateType h = 1/(1 + e);
		PointCoordinateType hvx = h * v.x;
		PointCoordinateType hvz = h * v.z;
		PointCoordinateType hvxy = hvx * v.y;
		PointCoordinateType hvxz = hvx * v.z;
		PointCoordinateType hvyz = hvz * v.y;

		float* mat = result.data();
		mat[0]  = static_cast<float>(e + hvx * v.x);
		mat[1]  = static_cast<float>(hvxy - v.z);
		mat[2]  = static_cast<float>(hvxz + v.y);
			    
		mat[4]  = static_cast<float>(hvxy + v.z);
		mat[5]  = static_cast<float>(e + h * v.y * v.y);
		mat[6]  = static_cast<float>(hvyz - v.x);

		mat[8]  = static_cast<float>(hvxz - v.y);
		mat[9]  = static_cast<float>(hvyz + v.x);
		mat[10] = static_cast<float>(e + hvz * v.z);
	}

	return result;
}

bool ccGLMatrix::toFile(QFile& out) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));

	//data (dataVersion>=20)
	if (out.write((const char*)m_mat,sizeof(float)*OPENGL_MATRIX_SIZE)<0)
		return WriteError();

	return true;
}

bool ccGLMatrix::fromFile(QFile& in, short dataVersion, int flags)
{
	assert(in.isOpen() && (in.openMode() & QIODevice::ReadOnly));

	if (dataVersion<20)
		return CorruptError();

	//data (dataVersion>=20)
	if (in.read((char*)m_mat,sizeof(float)*OPENGL_MATRIX_SIZE)<0)
		return ReadError();

	return true;
}

ccGLMatrix ccGLMatrix::FromQuaternion(const float q[])
{
	assert(q);

	ccGLMatrix rotMat;
	float* mat = rotMat.data();

	//diagonal
	{
		float q00 = q[0]*q[0];
		float q11 = q[1]*q[1];
		float q22 = q[2]*q[2];
		float q33 = q[3]*q[3];
		mat[0]	= q00 + q11 - q22 - q33;
		mat[5]	= q00 - q11 + q22 - q33;
		mat[10]	= q00 - q11 - q22 + q33;
		mat[15]	= 1.0f;
	}

	//non-diagonal elements
	{
		float q03 = q[0]*q[3];
		float q13 = q[1]*q[3];
		float q23 = q[2]*q[3];
		float q02 = q[0]*q[2];
		float q12 = q[1]*q[2];
		float q01 = q[0]*q[1];

		mat[1]	= 2.0f*(q12+q03);
		mat[2]	= 2.0f*(q13-q02);

		mat[4]	= 2.0f*(q12-q03);
		mat[6]	= 2.0f*(q23+q01);

		mat[8]	= 2.0f*(q13+q02);
		mat[9]	= 2.0f*(q23-q01);
	}

	return rotMat;
}

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


ccGLMatrix ccGLMatrix::xRotation() const
{
	ccGLMatrix newRotMat;
	newRotMat.toIdentity();

	//we use a specific Euler angles convention here
	if (R13 >= 1.0f)
	{
		//simpler/faster to ignore this (very) specific case!
		return newRotMat;
	}
	float phi = -asin(R13);
	float cos_phi = cos(phi);
	float theta = atan2(R23/cos_phi,R33/cos_phi);

	newRotMat.R22 = newRotMat.R33 = cos(theta);
	newRotMat.R32 = newRotMat.R23 = sin(theta);
	newRotMat.R23 *= -1.0f;

	newRotMat.setTranslation(getTranslation());

	return newRotMat;
}

ccGLMatrix ccGLMatrix::yRotation() const
{
	ccGLMatrix newRotMat;
	newRotMat.toIdentity();

	//we use a specific Euler angles convention here
	if (R32 >= 1.0f)
	{
		//simpler/faster to ignore this (very) specific case!
		return newRotMat;
	}
	float theta = asin(R32);
	float cos_theta = cos(theta);
	float phi = atan2(-R31/cos_theta,R33/cos_theta);

	newRotMat.R11 = newRotMat.R33 = cos(phi);
	newRotMat.R31 = newRotMat.R13 = sin(phi);
	newRotMat.R31 *= -1.0f;

	newRotMat.setTranslation(getTranslation());

	return newRotMat;
}

ccGLMatrix ccGLMatrix::zRotation() const
{
	//we can use the standard Euler angles convention here
	PointCoordinateType phi,theta,psi;
	CCVector3 T;
	getParameters(phi,theta,psi,T);
	assert(T.norm2() == 0);

	ccGLMatrix newRotMat;
	newRotMat.initFromParameters(phi,0,0,T);

	return newRotMat;
}

QString ccGLMatrix::toString(int precision/*=12*/, QChar separator/*=' '*/) const
{
	QString str;
	for (unsigned l=0; l<4; ++l) //lines
	{
		for (unsigned c=0; c<4; ++c) //columns
		{
			str.append(QString::number(m_mat[c*4+l],'f',precision));
			if (c != 3)
				str.append(separator);
		}
		if (l != 3)
			str.append("\n");
	}
	return str;
}
