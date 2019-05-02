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
#include "StFootPrint.h"
#include <iostream>

StFootPrint::StFootPrint(GenericIndexedCloudPersist* associatedCloud)
	: ccPolyline(associatedCloud)
	, m_bottom(0)
	, m_hole(false)
	, m_componentId(-1)
{
}

StFootPrint::StFootPrint(StFootPrint& obj)
	: ccPolyline(obj)
	, m_bottom(obj.m_bottom)
	, m_hole(false)
	, m_componentId(-1)
{
	
}

StFootPrint::StFootPrint(ccPolyline& obj)
	: ccPolyline(obj)
	, m_bottom(0)
	, m_hole(false)
	, m_componentId(-1)
{
}

StFootPrint::~StFootPrint()
{
}

bool StFootPrint::reverseVertexOrder()
{
	try {
		getAssociatedCloud();
		unsigned int last = size();
		unsigned int first = 0;
		while ((first != last) && first != --last) {
			///< swap first and last
			unsigned int fist_index = getPointGlobalIndex(first);
			setPointIndex(first, getPointGlobalIndex(last));
			setPointIndex(last, fist_index);
			++first;
		}
	}
	catch (std::runtime_error& e) {
		std::cout << "unknown error happens: " << e.what() << std::endl;
		return false;
	}
	return true;
}

inline double StFootPrint::getHeight() const
{
	return getPoint(0)->z;
}

void StFootPrint::setHeight(double height)
{
	for (unsigned int i = 0; i < size(); i++) {
		CCVector3& P = const_cast<CCVector3&>(*getPoint(i));
		P.z = height;
	}
	invalidateBoundingBox();
}

void StFootPrint::drawMeOnly(CC_DRAW_CONTEXT & context)
{
	//call parent method
	ccPolyline::drawMeOnly(context);

	if (!MACRO_Draw3D(context))
		return;

	if (isSelected()) {
		unsigned vertCount = size();
		if (vertCount < 2)
			return;

		//get the set of OpenGL functions (version 2.1)
		QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
		assert(glFunc != nullptr);

		if (glFunc == nullptr)
			return;

		//standard case: list names pushing
		bool pushName = MACRO_DrawEntityNames(context);
		if (pushName)
			glFunc->glPushName(getUniqueIDForDisplay());

		if (isColorOverriden())
			ccGL::Color3v(glFunc, getTempColor().rgb);
		else if (colorsShown())
			ccGL::Color3v(glFunc, m_rgbColor.rgb);

		//display polyline
		bool line_bit_pushed = false;
		if (m_width != 0)
		{
			glFunc->glPushAttrib(GL_LINE_BIT);
			line_bit_pushed = true;
			glFunc->glLineWidth(static_cast<GLfloat>(m_width));
		}

		//DGM: we do the 'GL_LINE_LOOP' manually as I have a strange bug
	//on one on my graphic card with this mode!
	//glBegin(m_isClosed ? GL_LINE_LOOP : GL_LINE_STRIP);
		glFunc->glBegin(GL_LINE_STRIP);
		for (unsigned i = 0; i < vertCount; ++i)
		{
			CCVector3* p = const_cast<CCVector3*>(getPoint(i));
			float v[3];
			v[0] = p->x;
			v[1] = p->y;
			v[2] = m_top;
			ccGL::Vertex3v(glFunc, v);
		}
		if (m_isClosed)
		{
			CCVector3* p = const_cast<CCVector3*>(getPoint(0));
			float v[3];
			v[0] = p->x;
			v[1] = p->y;
			v[2] = m_top;
			ccGL::Vertex3v(glFunc, v);
		}
		glFunc->glEnd();

		if (line_bit_pushed/*m_width != 0*/)
		{
			glFunc->glPopAttrib();	//GL_LINE_BIT
		}

		if (pushName)
			glFunc->glPopName();
	}
}

bool StFootPrint::toFile_MeOnly(QFile & out) const
{
	if (!ccPolyline::toFile_MeOnly(out)) {
		return false;
	}
	
	if (out.write((const char*)&m_bottom, sizeof(double)) < 0)
		return WriteError();

	if (out.write((const char*)&m_hole, sizeof(bool)) < 0)
		return WriteError();

	if (out.write((const char*)&m_top, sizeof(double)) < 0)
		return WriteError();

	if (out.write((const char*)&m_componentId, sizeof(int)) < 0)
		return WriteError();

	QDataStream outStream(&out);
	outStream << m_plane_names.size();
	for (auto & name : m_plane_names) {
		outStream << name;
	}

	return true;
}

bool StFootPrint::fromFile_MeOnly(QFile & in, short dataVersion, int flags)
{
	if (!ccPolyline::fromFile_MeOnly(in, dataVersion, flags)) {
		return false;
	}

	if (in.read((char*)&m_bottom, sizeof(double)) < 0)
		return ReadError();

	if (in.read((char*)&m_hole, sizeof(bool)) < 0)
		return ReadError();

	if (in.read((char*)&m_top, sizeof(double)) < 0)
		return ReadError();

	if (in.read((char*)&m_componentId, sizeof(int)) < 0)
		return ReadError();

	QDataStream inStream(&in);
	int plane_num;
	inStream >> plane_num;
	for (size_t i = 0; i < plane_num; i++) {
		QString name;
		inStream >> name;
		m_plane_names.append(name);
	}

	return true;
}
