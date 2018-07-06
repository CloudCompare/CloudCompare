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
