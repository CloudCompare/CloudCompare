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
									   const ccGLMatrix* transMat/*=nullptr*/,
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

ccCoordinateSystem::ccCoordinateSystem(const ccGLMatrix* transMat/*=nullptr*/,
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

CCVector3 ccCoordinateSystem::getXYPlaneNormal() const
{
	return m_transformation.getColumnAsVec3D(2);
}

CCVector3 ccCoordinateSystem::getYZPlaneNormal() const
{
	return m_transformation.getColumnAsVec3D(0);
}

CCVector3 ccCoordinateSystem::getZXPlaneNormal() const
{
	return m_transformation.getColumnAsVec3D(1);
}

ccPlane* ccCoordinateSystem::createXYplane(const ccGLMatrix* transMat) const
{
	ccGLMatrix xyPlaneMat;
	xyPlaneMat.toIdentity();
	xyPlaneMat.setTranslation(CCVector3(m_DisplayScale / 2, m_DisplayScale / 2, 0.0));
	if (transMat)
	{
		xyPlaneMat = *transMat * xyPlaneMat;
	}
	ccPlane* xyPlane = new ccPlane(m_DisplayScale, m_DisplayScale, &xyPlaneMat);
	xyPlane->setColor(ccColor::red);
	return xyPlane;
}

ccPlane* ccCoordinateSystem::createYZplane(const ccGLMatrix* transMat) const
{
	ccGLMatrix yzPlaneMat;
	yzPlaneMat.initFromParameters(	static_cast<PointCoordinateType>(M_PI_2),
										static_cast<PointCoordinateType>(0),
										static_cast<PointCoordinateType>(M_PI_2),
										CCVector3(0.0, m_DisplayScale / 2, m_DisplayScale / 2));
	if (transMat)
	{
		yzPlaneMat = *transMat * yzPlaneMat;
	}
	ccPlane* yzPlane = new ccPlane(m_DisplayScale, m_DisplayScale, &yzPlaneMat);
	yzPlane->setColor(ccColor::green);
	return yzPlane;
}

ccPlane* ccCoordinateSystem::createZXplane(const ccGLMatrix* transMat) const
{
	ccGLMatrix zxPlaneMat;
	zxPlaneMat.initFromParameters(static_cast<PointCoordinateType>(0),
									static_cast<PointCoordinateType>(-M_PI_2),
									static_cast<PointCoordinateType>(-M_PI_2),
		CCVector3(m_DisplayScale / 2, 0, m_DisplayScale / 2));
	if (transMat)
	{
		zxPlaneMat = *transMat * zxPlaneMat;
	}
	ccPlane* zxPlane = new ccPlane(m_DisplayScale, m_DisplayScale, &zxPlaneMat);
	zxPlane->setColor(ccColor::FromRgbfToRgb(ccColor::Rgbf(0.0f, 0.7f, 1.0f)));
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

	*this += *QScopedPointer<ccPlane>(createXYplane());
	*this += *QScopedPointer<ccPlane>(createYZplane());
	*this += *QScopedPointer<ccPlane>(createZXplane());

	return (vertices() && vertices()->size() == 12 && this->size() == 6);
}


ccGenericPrimitive* ccCoordinateSystem::clone() const
{
	return finishCloneJob(new ccCoordinateSystem(m_DisplayScale, m_width, &m_transformation, getName()));
}

bool ccCoordinateSystem::toFile_MeOnly(QFile& out, short dataVersion) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));
	if (dataVersion < 52)
	{
		assert(false);
		return false;
	}

	if (!ccGenericPrimitive::toFile_MeOnly(out, dataVersion))
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

short ccCoordinateSystem::minimumFileVersion_MeOnly() const
{
	return std::max(static_cast<short>(52), ccGenericPrimitive::minimumFileVersion_MeOnly());
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

		//color-based entity picking
		bool entityPickingMode = MACRO_EntityPicking(context);
		ccColor::Rgb pickingColor;
		if (entityPickingMode)
		{
			//not fast at all!
			if (MACRO_FastEntityPicking(context))
			{
				return;
			}
			pickingColor = context.entityPicking.registerEntity(this);
			ccGL::Color(glFunc, pickingColor);
		}

		glFunc->glPushMatrix();
		glFunc->glMatrixMode(GL_MODELVIEW);
		glFunc->glMultMatrixf(m_transformation.data());

		if (m_width != 0)
		{
			glFunc->glPushAttrib(GL_LINE_BIT);
			glFunc->glLineWidth(static_cast<GLfloat>(m_width));
		}

		glFunc->glBegin(GL_LINES);
		if (!entityPickingMode)
			ccGL::Color(glFunc, ccColor::red);
		glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
		glFunc->glVertex3f(m_DisplayScale * 2, 0.0f, 0.0f);
		if (!entityPickingMode)
			ccGL::Color(glFunc, ccColor::green);
		glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
		glFunc->glVertex3f(0.0f, m_DisplayScale * 2, 0.0f);
		if (!entityPickingMode)
			ccGL::Color(glFunc, ccColor::blueCC);
		glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
		glFunc->glVertex3f(0.0f, 0.0f, m_DisplayScale * 2);
		glFunc->glEnd();

		if (m_width != 0)
		{
			glFunc->glPopAttrib();
		}
		glFunc->glPopMatrix();
	}
}
