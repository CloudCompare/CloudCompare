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

#include "ccIndexedTransformation.h"

//Qt
#include <QFile>
#include <QTextStream>

//System
#include <assert.h>
#include <string>

ccIndexedTransformation::ccIndexedTransformation()
	: ccGLMatrix()
	, m_index(0)
{
}

ccIndexedTransformation::ccIndexedTransformation(const ccGLMatrix& matrix)
	: ccGLMatrix(matrix)
	, m_index(0)
{
}

ccIndexedTransformation::ccIndexedTransformation(const ccGLMatrix& matrix, double index)
	: ccGLMatrix(matrix)
	, m_index(index)
{
}

bool ccIndexedTransformation::toAsciiFile(QString filename, int precision/*=12*/) const
{
	QFile fp(filename);
	if (!fp.open(QFile::WriteOnly | QFile::Text))
		return false;

	QTextStream stream(&fp);
	stream.setRealNumberNotation(QTextStream::FixedNotation);
	stream.setRealNumberPrecision(precision);

	//save transformation first (so that it can be loaded as a ccGLMatrix!)
	for (unsigned i=0; i<4; ++i)
	{
		stream << m_mat[i] << " " << m_mat[i+4] << " " << m_mat[i+8] << " " << m_mat[i+12] << endl;
	}
	//save index next
	stream << m_index;

	return (fp.error() == QFile::NoError);
}

bool ccIndexedTransformation::fromAsciiFile(QString filename)
{
	QFile fp(filename);
	if (!fp.open(QFile::ReadOnly | QFile::Text))
		return false;

	QTextStream stream(&fp);

	//read the transformation first
	for (unsigned i=0; i<4; ++i)
	{
		stream >> m_mat[i];
		stream >> m_mat[i+4];
		stream >> m_mat[i+8];
		stream >> m_mat[i+12];
	}
	//read index
	stream >> m_index;

	return (fp.error() == QFile::NoError);
}

ccIndexedTransformation ccIndexedTransformation::operator * (const ccGLMatrix& M) const
{
	return ccIndexedTransformation( *static_cast<const ccGLMatrix*>(this) * M, m_index);
}

ccIndexedTransformation& ccIndexedTransformation::operator *= (const ccGLMatrix& M)
{
	ccGLMatrix temp = (*this) * M;
	(*this) = temp;

	return (*this);
}

//ccIndexedTransformation ccIndexedTransformation::operator * (const ccIndexedTransformation& trans) const
//{
//	return ccIndexedTransformation( static_cast<ccGLMatrix*>(this) * trans, m_index);
//}
//
//ccIndexedTransformation& ccIndexedTransformation::operator *= (const ccIndexedTransformation& trans)
//{
//	ccGLMatrix temp = (*this) * M;
//	(*this) = temp;
//
//	return (*this);
//}

ccIndexedTransformation& ccIndexedTransformation::operator += (const CCVector3& T)
{
	*static_cast<ccGLMatrix*>(this) += T;

	return (*this);
}

ccIndexedTransformation& ccIndexedTransformation::operator -= (const CCVector3& T)
{
	*static_cast<ccGLMatrix*>(this) -= T;

	return (*this);
}

ccIndexedTransformation ccIndexedTransformation::transposed() const
{
	ccIndexedTransformation t(*this);
	t.transpose();
	
	return t;
}

ccIndexedTransformation ccIndexedTransformation::inverse() const
{
	ccIndexedTransformation t(*this);
	t.invert();

	return t;
}

ccIndexedTransformation ccIndexedTransformation::Interpolate(	double index,
																const ccIndexedTransformation& trans1,
																const ccIndexedTransformation& trans2)
{
	double dt = trans2.getIndex() - trans1.getIndex();
	if (dt == 0.0)
	{
		assert(index == trans1.getIndex());
		return trans1;
	}

	//we compute the transformation matrix between trans1 and trans2
	double t = (index - trans1.getIndex())/dt;
	assert(t >= 0 && t <= 1);
	
	ccGLMatrix mat = ccGLMatrix::Interpolate(static_cast<PointCoordinateType>(t),trans1,trans2);

	return ccIndexedTransformation(mat, index);
}

bool ccIndexedTransformation::toFile(QFile& out) const
{
	if (!ccGLMatrix::toFile(out))
		return false;
	
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));

	//index (dataVersion>=34)
	if (out.write((const char*)&m_index,sizeof(double)) < 0)
		return WriteError();

	return true;
}

bool ccIndexedTransformation::fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccGLMatrix::fromFile(in, dataVersion, flags, oldToNewIDMap))
		return false;

	assert(in.isOpen() && (in.openMode() & QIODevice::ReadOnly));

	if (dataVersion<34)
		return CorruptError();

	//index (dataVersion>=34)
	if (in.read((char*)&m_index,sizeof(double)) < 0)
		return ReadError();

	return true;
}
