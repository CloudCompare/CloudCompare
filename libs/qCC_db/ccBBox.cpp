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

//Always first
#include "ccIncludeGL.h"

#include "ccBBox.h"

ccBBox::ccBBox()
	: m_bbMin(0,0,0)
	, m_bbMax(0,0,0)
	, m_valid(false)
{}

ccBBox::ccBBox(const CCVector3& bbMinCorner, const CCVector3& bbMaxCorner)
	: m_bbMin(bbMinCorner)
	, m_bbMax(bbMaxCorner)
	, m_valid(true)
{}

void ccBBox::clear()
{
	m_bbMin = m_bbMax = CCVector3(0,0,0);
	m_valid = false;
}

CCVector3 ccBBox::getCenter() const
{
	return (m_bbMax + m_bbMin) * static_cast<PointCoordinateType>(0.5);
}

CCVector3 ccBBox::getDiagVec() const
{
	return (m_bbMax - m_bbMin);
}

PointCoordinateType ccBBox::getMinBoxDim() const
{
	CCVector3 V = getDiagVec();

	return std::min(V.x,std::min(V.y,V.z));
}

PointCoordinateType ccBBox::getMaxBoxDim() const
{
	CCVector3 V = getDiagVec();

	return std::max(V.x,std::max(V.y,V.z));
}

double ccBBox::computeVolume() const
{
	CCVector3 V = getDiagVec();

	return static_cast<double>(V.x) * static_cast<double>(V.y) * static_cast<double>(V.z);
}

void ccBBox::draw(CC_DRAW_CONTEXT& context, const ccColor::Rgb& col) const
{
	if (!m_valid)
		return;
	
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert( glFunc != nullptr );
	
	if ( glFunc == nullptr )
		return;

	ccGL::Color3v(glFunc, col.rgb);

	glFunc->glBegin(GL_LINE_LOOP);
	ccGL::Vertex3v(glFunc, m_bbMin.u);
	ccGL::Vertex3(glFunc, m_bbMax.x, m_bbMin.y, m_bbMin.z);
	ccGL::Vertex3(glFunc, m_bbMax.x, m_bbMax.y, m_bbMin.z);
	ccGL::Vertex3(glFunc, m_bbMin.x, m_bbMax.y, m_bbMin.z);
	glFunc->glEnd();

	glFunc->glBegin(GL_LINE_LOOP);
	ccGL::Vertex3(glFunc, m_bbMin.x, m_bbMin.y, m_bbMax.z);
	ccGL::Vertex3(glFunc, m_bbMax.x, m_bbMin.y, m_bbMax.z);
	ccGL::Vertex3v(glFunc, m_bbMax.u);
	ccGL::Vertex3(glFunc, m_bbMin.x, m_bbMax.y, m_bbMax.z);
	glFunc->glEnd();

	glFunc->glBegin(GL_LINES);
	ccGL::Vertex3v(glFunc, m_bbMin.u);
	ccGL::Vertex3(glFunc, m_bbMin.x, m_bbMin.y, m_bbMax.z);
	ccGL::Vertex3(glFunc, m_bbMax.x, m_bbMin.y, m_bbMin.z);
	ccGL::Vertex3(glFunc, m_bbMax.x, m_bbMin.y, m_bbMax.z);
	ccGL::Vertex3(glFunc, m_bbMax.x, m_bbMax.y, m_bbMin.z);
	ccGL::Vertex3v(glFunc, m_bbMax.u);
	ccGL::Vertex3(glFunc, m_bbMin.x, m_bbMax.y, m_bbMin.z);
	ccGL::Vertex3(glFunc, m_bbMin.x, m_bbMax.y, m_bbMax.z);
	glFunc->glEnd();
}

ccBBox ccBBox::operator + (const ccBBox& aBBox) const
{
	if (!m_valid)
		return aBBox;
	if (!aBBox.isValid())
		return *this;

	ccBBox tempBox;
	{
		tempBox.m_bbMin.x = std::min(m_bbMin.x, aBBox.m_bbMin.x);
		tempBox.m_bbMin.y = std::min(m_bbMin.y, aBBox.m_bbMin.y);
		tempBox.m_bbMin.z = std::min(m_bbMin.z, aBBox.m_bbMin.z);
		tempBox.m_bbMax.x = std::max(m_bbMax.x, aBBox.m_bbMax.x);
		tempBox.m_bbMax.y = std::max(m_bbMax.y, aBBox.m_bbMax.y);
		tempBox.m_bbMax.z = std::max(m_bbMax.z, aBBox.m_bbMax.z);
		tempBox.setValidity(true);
	}

	return tempBox;
}

const ccBBox& ccBBox::operator += (const ccBBox& aBBox)
{
	if (aBBox.isValid())
	{
		add(aBBox.minCorner());
		add(aBBox.maxCorner());
	}

	return *this;
}

const ccBBox& ccBBox::operator += (const CCVector3& aVector)
{
	if (m_valid)
	{
		m_bbMin += aVector;
		m_bbMax += aVector;
	}

	return *this;
}

const ccBBox& ccBBox::operator -= (const CCVector3& aVector)
{
	if (m_valid)
	{
		m_bbMin -= aVector;
		m_bbMax -= aVector;
	}

	return *this;
}

const ccBBox& ccBBox::operator *= (PointCoordinateType scaleFactor)
{
	if (m_valid)
	{
		m_bbMin *= scaleFactor;
		m_bbMax *= scaleFactor;
	}

	return *this;
}

void ccBBox::add(const CCVector3& aPoint)
{
	if (m_valid)
	{
		if (aPoint.x < m_bbMin.x)
			m_bbMin.x = aPoint.x;
		else if (aPoint.x > m_bbMax.x)
			m_bbMax.x = aPoint.x;

		if (aPoint.y < m_bbMin.y)
			m_bbMin.y = aPoint.y;
		else if (aPoint.y > m_bbMax.y)
			m_bbMax.y = aPoint.y;

		if (aPoint.z < m_bbMin.z)
			m_bbMin.z = aPoint.z;
		else if (aPoint.z > m_bbMax.z)
			m_bbMax.z = aPoint.z;
	}
	else
	{
		m_bbMax = m_bbMin = aPoint;
		m_valid = true;
	}
}

const ccBBox& ccBBox::operator *= (const CCLib::SquareMatrix& mat)
{
	if (m_valid)
	{
		CCVector3 boxCorners[8];

		boxCorners[0] = m_bbMin;
		boxCorners[1] = CCVector3(m_bbMin.x,m_bbMin.y,m_bbMax.z);
		boxCorners[2] = CCVector3(m_bbMin.x,m_bbMax.y,m_bbMin.z);
		boxCorners[3] = CCVector3(m_bbMax.x,m_bbMin.y,m_bbMin.z);
		boxCorners[4] = m_bbMax;
		boxCorners[5] = CCVector3(m_bbMin.x,m_bbMax.y,m_bbMax.z);
		boxCorners[6] = CCVector3(m_bbMax.x,m_bbMax.y,m_bbMin.z);
		boxCorners[7] = CCVector3(m_bbMax.x,m_bbMin.y,m_bbMax.z);

		clear();

		for (int i=0;i<8;++i)
			add(mat*boxCorners[i]);
	}

	return *this;
}

const ccBBox ccBBox::operator * (const ccGLMatrix& mat)
{
	ccBBox rotatedBox;

	if (m_valid)
	{
		rotatedBox.add(mat * m_bbMin);
		rotatedBox.add(mat * CCVector3(m_bbMin.x,m_bbMin.y,m_bbMax.z));
		rotatedBox.add(mat * CCVector3(m_bbMin.x,m_bbMax.y,m_bbMin.z));
		rotatedBox.add(mat * CCVector3(m_bbMax.x,m_bbMin.y,m_bbMin.z));
		rotatedBox.add(mat * m_bbMax);
		rotatedBox.add(mat * CCVector3(m_bbMin.x,m_bbMax.y,m_bbMax.z));
		rotatedBox.add(mat * CCVector3(m_bbMax.x,m_bbMax.y,m_bbMin.z));
		rotatedBox.add(mat * CCVector3(m_bbMax.x,m_bbMin.y,m_bbMax.z));
	}

	return rotatedBox;
}

const ccBBox ccBBox::operator * (const ccGLMatrixd& mat)
{
	ccBBox rotatedBox;

	if (m_valid)
	{
		rotatedBox.add(mat * m_bbMin);
		rotatedBox.add(mat * CCVector3(m_bbMin.x,m_bbMin.y,m_bbMax.z));
		rotatedBox.add(mat * CCVector3(m_bbMin.x,m_bbMax.y,m_bbMin.z));
		rotatedBox.add(mat * CCVector3(m_bbMax.x,m_bbMin.y,m_bbMin.z));
		rotatedBox.add(mat * m_bbMax);
		rotatedBox.add(mat * CCVector3(m_bbMin.x,m_bbMax.y,m_bbMax.z));
		rotatedBox.add(mat * CCVector3(m_bbMax.x,m_bbMax.y,m_bbMin.z));
		rotatedBox.add(mat * CCVector3(m_bbMax.x,m_bbMin.y,m_bbMax.z));
	}

	return rotatedBox;
}

PointCoordinateType ccBBox::minDistTo(const ccBBox& box) const
{
	if (m_valid && box.isValid())
	{
		CCVector3 d(0,0,0);

		for (unsigned char dim=0; dim < 3; ++dim)
		{
			//if the boxes overlap in one dimension, the distance is zero (in this dimension)
			if (box.m_bbMin.u[dim] > m_bbMax.u[dim])
				d.u[dim] = box.m_bbMin.u[dim] - m_bbMax.u[dim];
			else if (box.m_bbMax.u[dim] < m_bbMin.u[dim])
				d.u[dim] = m_bbMin.u[dim] - box.m_bbMax.u[dim];
		}

		return d.norm();
	}
	else
	{
		return static_cast<PointCoordinateType>(-1.0);
	}
}
