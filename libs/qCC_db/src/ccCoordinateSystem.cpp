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
//#          COPYRIGHT: Chris Brown                                        #
//#                                                                        #
//##########################################################################

#include "ccCoordinateSystem.h"

//qCC_db
#include "ccPlane.h"
#include "ccPointCloud.h"

ccCoordinateSystem::ccCoordinateSystem(PointCoordinateType displayScale, PointCoordinateType axisWidth, const ccGLMatrix* transMat/*= 0*/,
	QString name/*=QString("CoordinateSystem")*/)
	: ccGenericPrimitive(name, transMat), m_DisplayScale(displayScale), m_width(axisWidth)
{
	updateRepresentation();
}

ccCoordinateSystem::ccCoordinateSystem(QString name/*=QString("CoordinateSystem")*/)
	: ccGenericPrimitive(name), m_DisplayScale(1.0), m_width(4.0)
{
	updateRepresentation();
}

bool ccCoordinateSystem::buildUp()
{
	//clear triangles indexes
	if (m_triVertIndexes)
	{
		m_triVertIndexes->clear();
	}
	//clear per triangle normals
	removePerTriangleNormalIndexes();
	if (m_triNormals)
	{
		m_triNormals->clear();
	}
	//clear vertices
	ccPointCloud* verts = vertices();
	if (verts)
	{
		verts->clear();
	}

	//XYplane_Centered
	ccGLMatrix xyPlane_Centered;
	xyPlane_Centered.toIdentity();
	*this += ccPlane(m_DisplayScale, m_DisplayScale, &xyPlane_Centered);
	//xzPlane_Centered
	ccGLMatrix xzPlane_Centered;
	xzPlane_Centered.initFromParameters(static_cast<PointCoordinateType>(0),
										static_cast<PointCoordinateType>(-1.57079633),
										static_cast<PointCoordinateType>(-1.57079633),
										CCVector3());
	*this += ccPlane(m_DisplayScale, m_DisplayScale, &xzPlane_Centered);
	//YZplane_Centered
	ccGLMatrix yzPlane_Centered;
	yzPlane_Centered.initFromParameters(static_cast<PointCoordinateType>(1.57079633),
										static_cast<PointCoordinateType>(0), 
										static_cast<PointCoordinateType>(1.57079633),
										CCVector3());
	*this += ccPlane(m_DisplayScale, m_DisplayScale, &yzPlane_Centered);
	

	return (vertices() && vertices()->size() == 12 && this->size() == 6);
}

ccGenericPrimitive* ccCoordinateSystem::clone() const
{
	return finishCloneJob(new ccCoordinateSystem(m_DisplayScale, m_width, &m_transformation, getName()));
}

bool ccCoordinateSystem::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericPrimitive::toFile_MeOnly(out))
		return false;

	//parameters (dataVersion>=22)
	QDataStream outStream(&out);
	outStream << m_DisplayScale;

	return true;
}

bool ccCoordinateSystem::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//parameters (dataVersion>=22)
	QDataStream inStream(&in);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_DisplayScale, 1);

	return true;
}

void ccCoordinateSystem::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	//call parent method to draw the Planes
	ccGenericPrimitive::drawMeOnly(context);

	//show axis
	if (MACRO_Draw3D(context))
	{
		QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
		assert(glFunc != nullptr);

		if (glFunc == nullptr)
			return;

		glFunc->glPushMatrix();
		glFunc->glMatrixMode(GL_MODELVIEW);
		glFunc->glMultMatrixf(m_transformation.data());
		
		//standard case: list names pushing
		bool pushName = MACRO_DrawEntityNames(context);
		if (pushName)
		{
			glFunc->glPushName(getUniqueIDForDisplay());
		}

		if (m_width != 0)
		{
			glFunc->glPushAttrib(GL_LINE_BIT);
			glFunc->glLineWidth(static_cast<GLfloat>(m_width));
		}

		glFunc->glBegin(GL_LINES);
		glFunc->glColor3f(1.0f, 0.0f, 0.0f);
		glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
		glFunc->glVertex3f(m_DisplayScale, 0.0f, 0.0f);
		glFunc->glColor3f(0.0f, 1.0f, 0.0f);
		glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
		glFunc->glVertex3f(0.0f, m_DisplayScale, 0.0f);
		glFunc->glColor3f(0.0f, 0.7f, 1.0f);
		glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
		glFunc->glVertex3f(0.0f, 0.0f, m_DisplayScale);
		glFunc->glEnd();
		
		if (pushName)
		{
			glFunc->glPopName();
		}
		
		if (m_width != 0)
		{
			glFunc->glPopAttrib();
		}
		glFunc->glPopMatrix();
	}
}