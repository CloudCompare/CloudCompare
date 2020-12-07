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


ccCoordinateSystem::ccCoordinateSystem(PointCoordinateType displayScale, 
									   PointCoordinateType axisWidth, 
									   const ccGLMatrix* transMat/*= 0*/,
									   QString name/*=QString("CoordinateSystem")*/)
	: ccGenericPrimitive(name, transMat), 
	  m_DisplayScale(displayScale), 
	  m_width(axisWidth), 
	  m_showAxisPlanes(true), 
	  m_showAxisLines(true)
{
	updateRepresentation();
	showColors(true);
}

ccCoordinateSystem::ccCoordinateSystem(const ccGLMatrix* transMat/*= 0*/,
									   QString name/*=QString("CoordinateSystem")*/)
	: ccGenericPrimitive(name, transMat), 
	  m_DisplayScale(DEFAULT_DISPLAY_SCALE), 
	  m_width(AXIS_DEFAULT_WIDTH), 
	  m_showAxisPlanes(true), 
	  m_showAxisLines(true)
{
	updateRepresentation();
	showColors(true);
}


ccCoordinateSystem::ccCoordinateSystem(QString name/*=QString("CoordinateSystem")*/)
	: ccGenericPrimitive(name), 
	  m_DisplayScale(DEFAULT_DISPLAY_SCALE), 
	  m_width(AXIS_DEFAULT_WIDTH), 
	  m_showAxisPlanes(true), 
	  m_showAxisLines(true)
{
	updateRepresentation();
	showColors(true);
}

void ccCoordinateSystem::ShowAxisPlanes(bool show)
{
	m_showAxisPlanes = show;
}

void ccCoordinateSystem::ShowAxisLines(bool show)
{
	m_showAxisLines = show;
}


void ccCoordinateSystem::setAxisWidth(PointCoordinateType size)
{
	if (size == 0.0f)
	{
		m_width = AXIS_DEFAULT_WIDTH;
		return;
	}
	if (size >= MIN_AXIS_WIDTH_F && size <= MAX_AXIS_WIDTH_F)
	{
		m_width = size;
	}
}



void ccCoordinateSystem::setDisplayScale(PointCoordinateType size)
{
	if (size >= MIN_DISPLAY_SCALE_F)
	{
		m_DisplayScale = size;
		updateRepresentation();
	}
}

ccPlane ccCoordinateSystem::getXYplane() const
{
	ccPlane xyPlane = createXYplane(&m_transformation);
	return xyPlane;
}
ccPlane ccCoordinateSystem::getYZplane() const
{
	ccPlane yzPlane = createYZplane(&m_transformation);
	return yzPlane;
}
ccPlane ccCoordinateSystem::getZXplane() const
{
	ccPlane zxPlane = createZXplane(&m_transformation);
	return zxPlane;
}

ccPlane ccCoordinateSystem::createXYplane(const ccGLMatrix* transMat) const
{
	ccGLMatrix xyPlane_mtrx;
	xyPlane_mtrx.toIdentity();
	xyPlane_mtrx.setTranslation(CCVector3(m_DisplayScale / 2, m_DisplayScale / 2, 0.0));
	if (transMat)
	{
		xyPlane_mtrx = *transMat * xyPlane_mtrx;
	}
	ccPlane xyPlane(m_DisplayScale, m_DisplayScale, &xyPlane_mtrx);
	xyPlane.setColor(ccColor::red);
	return xyPlane;
}

ccPlane ccCoordinateSystem::createYZplane(const ccGLMatrix* transMat) const
{
	ccGLMatrix yzPlane_mtrx;
	yzPlane_mtrx.initFromParameters(static_cast<PointCoordinateType>(1.57079633),
		static_cast<PointCoordinateType>(0),
		static_cast<PointCoordinateType>(1.57079633),
		CCVector3(0.0, m_DisplayScale / 2, m_DisplayScale / 2));
	if (transMat)
	{
		yzPlane_mtrx = *transMat * yzPlane_mtrx;
	}
	ccPlane yzPlane(m_DisplayScale, m_DisplayScale, &yzPlane_mtrx);
	yzPlane.setColor(ccColor::green);
	return yzPlane;
}

ccPlane ccCoordinateSystem::createZXplane(const ccGLMatrix* transMat) const
{
	ccGLMatrix zxPlane_mtrx;
	zxPlane_mtrx.initFromParameters(static_cast<PointCoordinateType>(0),
		static_cast<PointCoordinateType>(-1.57079633),
		static_cast<PointCoordinateType>(-1.57079633),
		CCVector3(m_DisplayScale / 2, 0, m_DisplayScale / 2));
	if (transMat)
	{
		zxPlane_mtrx = *transMat * zxPlane_mtrx;
	}
	ccPlane zxPlane(m_DisplayScale, m_DisplayScale, &zxPlane_mtrx);
	zxPlane.setColor(ccColor::FromRgbfToRgb(ccColor::Rgbf(0.0f, 0.7f, 1.0f)));
	return zxPlane;
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

	*this += createXYplane();
	*this += createYZplane();
	*this += createZXplane();
	

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

	//parameters (dataVersion>=52)
	QDataStream outStream(&out);
	outStream << m_DisplayScale;
	outStream << m_width;

	return true;
}

bool ccCoordinateSystem::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//parameters (dataVersion>=52)
	QDataStream inStream(&in);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_DisplayScale, 1);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_width, 1);
	return true;
}

void ccCoordinateSystem::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (m_showAxisPlanes)
	{
		//call parent method to draw the Planes
		ccGenericPrimitive::drawMeOnly(context);
	}

	//show axis
	if (m_showAxisLines && MACRO_Draw3D(context))
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
		glFunc->glVertex3f(m_DisplayScale*2, 0.0f, 0.0f);
		glFunc->glColor3f(0.0f, 1.0f, 0.0f);
		glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
		glFunc->glVertex3f(0.0f, m_DisplayScale*2, 0.0f);
		glFunc->glColor3f(0.0f, 0.7f, 1.0f);
		glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
		glFunc->glVertex3f(0.0f, 0.0f, m_DisplayScale*2);
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
