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

#include "ccIncludeGL.h"

//Local
#include "cc2DLabel.h"
#include "ccBasicTypes.h"
#include "ccGenericPointCloud.h"
#include "ccPointCloud.h"
#include "ccSphere.h"
#include "ccGenericGLDisplay.h"
#include "ccScalarField.h"

//Qt
#include <QSharedPointer>

//System
#include <string.h>
#include <assert.h>

//'Delta' character
static const QChar MathSymbolDelta(0x0394);

cc2DLabel::cc2DLabel(QString name/*=QString()*/)
	: ccHObject(name.isEmpty() ? "label" : name)
	, m_showFullBody(true)
	, m_dispPointsLegend(false)
	, m_dispIn2D(true)
	, m_relMarkerScale(1.0f)
{
	m_screenPos[0] = m_screenPos[1] = 0.05f;

	clear(false);

	lockVisibility(false);
	setEnabled(true);
}

static const QString POINT_INDEX_0("pi0");
static const QString POINT_INDEX_1("pi1");
static const QString POINT_INDEX_2("pi2");
static const QString CLOUD_INDEX_0("ci0");
static const QString CLOUD_INDEX_1("ci1");
static const QString CLOUD_INDEX_2("ci2");

//return angle between two vectors (in degrees)
//warning: vectors will be normalized by default
static double GetAngle_deg(CCVector3 AB, CCVector3 AC)
{
	AB.normalize();
	AC.normalize();
	double dotprod = AB.dot(AC);
	//clamp value (just in case)
	if (dotprod <= -1.0)
		dotprod = -1.0;
	else if (dotprod > 1.0)
		dotprod = 1.0;
	return acos(dotprod) * CC_RAD_TO_DEG;
}

QString cc2DLabel::GetSFValueAsString(const LabelInfo1& info, int precision)
{
	if (info.hasSF)
	{
		if (!ccScalarField::ValidValue(info.sfValue))
		{
			return "NaN";
		}
		else
		{
			QString sfVal = QString::number(info.sfValue,'f',precision);
			if (info.sfValueIsShifted)
			{
				sfVal = QString::number(info.sfShiftedValue,'f',precision) + QString(" (shifted: %1)").arg(sfVal);
			}
			return sfVal;
		}
	}
	else
	{
		return QString();
	}
}

QString cc2DLabel::getTitle(int precision) const
{
	QString title;
	size_t count = m_points.size();
	if (count == 1)
	{
		title = m_name;
		title.replace(POINT_INDEX_0,QString::number(m_points[0].index));

		//if available, we display the point SF value
		LabelInfo1 info;
		getLabelInfo1(info);
		if (info.hasSF)
		{
			QString sfVal = GetSFValueAsString(info, precision);
			title = QString("%1 = %2").arg(info.sfName).arg(sfVal);
		}
	}
	else if (count == 2)
	{
		LabelInfo2 info;
		getLabelInfo2(info);
		//display distance by default
		double dist = info.diff.normd();
		title = QString("Distance: %1").arg(dist,0,'f',precision);
	}
	else if (count == 3)
	{
		LabelInfo3 info;
		getLabelInfo3(info);
		//display area by default
		title = QString("Area: %1").arg(info.area,0,'f',precision);
	}

	return title;
}

QString cc2DLabel::getName() const
{
	QString processedName = m_name;

	size_t count = m_points.size();
	if (count > 0)
	{
		processedName.replace(POINT_INDEX_0,QString::number(m_points[0].index));
		if (count > 1)
		{
			processedName.replace(POINT_INDEX_1,QString::number(m_points[1].index));
			if (m_points[0].cloud)
				processedName.replace(CLOUD_INDEX_0,QString::number(m_points[0].cloud->getUniqueID()));
			if (m_points[1].cloud)
				processedName.replace(CLOUD_INDEX_1,QString::number(m_points[1].cloud->getUniqueID()));
			if (count > 2)
			{
				processedName.replace(POINT_INDEX_2,QString::number(m_points[2].index));
				if (m_points[2].cloud)
					processedName.replace(CLOUD_INDEX_2,QString::number(m_points[2].cloud->getUniqueID()));
			}
		}
	}

	return processedName;
}

void cc2DLabel::setPosition(float x, float y)
{
	m_screenPos[0] = x;
	m_screenPos[1] = y;
}

bool cc2DLabel::move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight)
{
	assert(screenHeight > 0 && screenWidth > 0);
	
	m_screenPos[0] += static_cast<float>(dx) / screenWidth;
	m_screenPos[1] += static_cast<float>(dy) / screenHeight;

	return true;
}

void cc2DLabel::clear(bool ignoreDependencies)
{
	if (ignoreDependencies)
	{
		m_points.clear();
	}
	else
	{
		//remove all dependencies first!
		while (!m_points.empty())
		{
			m_points.back().cloud->removeDependencyWith(this);
			m_points.pop_back();
		}
	}

	m_lastScreenPos[0] = m_lastScreenPos[1] = -1;
	m_labelROI = QRect(0,0,0,0);
	setVisible(false);
	setName("Label");
}

void cc2DLabel::onDeletionOf(const ccHObject* obj)
{
	ccHObject::onDeletionOf(obj); //remove dependencies, etc.

	//check that associated clouds are not about to be deleted!
	size_t pointsToRemove = 0;
	{
		for (size_t i=0; i<m_points.size(); ++i)
			if (m_points[i].cloud == obj)
				++pointsToRemove;
	}

	if (pointsToRemove == 0)
		return;

	if (pointsToRemove == m_points.size())
	{
		clear(true); //don't call clear as we don't want/need to update input object's dependencies!
	}
	else
	{
		//remove only the necessary points
		size_t j=0;
		for (size_t i=0; i<m_points.size(); ++i)
		{
			if (m_points[i].cloud != obj)
			{
				if (i != j)
					std::swap(m_points[i],m_points[j]);
				j++;
			}
		}
		assert(j != 0);
		m_points.resize(j);
	}

	updateName();
}

void cc2DLabel::updateName()
{
	switch(m_points.size())
	{
	case 0:
		setName("Label");
		break;
	case 1:
		setName(QString("Point #") + POINT_INDEX_0);
		break;
	case 2:
		if (m_points[0].cloud == m_points[1].cloud)
			setName(QString("Vector #") + POINT_INDEX_0 + QString(" - #") + POINT_INDEX_1);
		else
			setName(QString("Vector #") + POINT_INDEX_0 + QString("@") + CLOUD_INDEX_0
			          + QString(" - #") + POINT_INDEX_1 + QString("@") + CLOUD_INDEX_1);
		break;
	case 3:
		if (m_points[0].cloud == m_points[2].cloud && m_points[1].cloud == m_points[2].cloud)
			setName(QString("Triplet #") + POINT_INDEX_0 + QString(" - #") + POINT_INDEX_1 + QString(" - #") + POINT_INDEX_2);
		else
			setName(QString("Triplet #") + POINT_INDEX_0 + QString("@") + CLOUD_INDEX_0
			           + QString(" - #") + POINT_INDEX_1 + QString("@") + CLOUD_INDEX_1
			           + QString(" - #") + POINT_INDEX_2 + QString("@") + CLOUD_INDEX_2);
		break;
	}
}

bool cc2DLabel::addPoint(ccGenericPointCloud* cloud, unsigned pointIndex)
{
	assert(cloud && cloud->size()>pointIndex);

	if (m_points.size() == 3)
		return false;

	try
	{
		m_points.resize(m_points.size()+1);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	m_points.back().cloud = cloud;
	m_points.back().index = pointIndex;

	updateName();

	//we want to be notified whenever an associated cloud is deleted (in which case
	//we'll automatically clear the label)
	cloud->addDependency(this,DP_NOTIFY_OTHER_ON_DELETE);
	//we must also warn the cloud whenever we delete this label
	//addDependency(cloud,DP_NOTIFY_OTHER_ON_DELETE); //DGM: automatically done by the previous call to addDependency!

	return true;
}

bool cc2DLabel::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//points count (dataVersion >= 20)
	uint32_t count = (uint32_t)m_points.size();
	if (out.write((const char*)&count,4) < 0)
		return WriteError();

	//points & associated cloud ID (dataVersion >= 20)
	for (std::vector<PickedPoint>::const_iterator it=m_points.begin(); it!=m_points.end(); ++it)
	{
		//point index
		uint32_t index = (uint32_t)it->index;
		if (out.write((const char*)&index,4) < 0)
			return WriteError();
		//cloud ID (will be retrieved later --> make sure that the cloud is saved alongside!)
		uint32_t cloudID = (uint32_t)it->cloud->getUniqueID();
		if (out.write((const char*)&cloudID,4) < 0)
			return WriteError();
	}

	//Relative screen position (dataVersion >= 20)
	if (out.write((const char*)m_screenPos,sizeof(float)*2) < 0)
		return WriteError();

	//Collapsed state (dataVersion >= 20)
	if (out.write((const char*)&m_showFullBody,sizeof(bool)) < 0)
		return WriteError();

	//Show in 2D boolean (dataVersion >= 21)
	if (out.write((const char*)&m_dispIn2D,sizeof(bool)) < 0)
		return WriteError();

	//Show point(s) legend boolean (dataVersion >= 21)
	if (out.write((const char*)&m_dispPointsLegend,sizeof(bool)) < 0)
		return WriteError();

	return true;
}

bool cc2DLabel::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags))
		return false;

	//points count (dataVersion >= 20)
	uint32_t count = 0;
	if (in.read((char*)&count,4) < 0)
		return ReadError();

	//points & associated cloud ID (dataVersion >= 20)
	assert(m_points.empty());
	for (uint32_t i=0; i<count; ++i)
	{
		//point index
		uint32_t index = 0;
		if (in.read((char*)&index,4) < 0)
			return ReadError();
		//cloud ID (will be retrieved later --> make sure that the cloud is saved alongside!)
		uint32_t cloudID = 0;
		if (in.read((char*)&cloudID,4) < 0)
			return ReadError();

		//[DIRTY] WARNING: temporarily, we set the cloud unique ID in the 'PickedPoint::cloud' pointer!!!
		PickedPoint pp;
		pp.index = (unsigned)index;
		*(uint32_t*)(&pp.cloud) = cloudID;
		m_points.push_back(pp);
		if (m_points.size() != i+1)
			return MemoryError();
	}

	//Relative screen position (dataVersion >= 20)
	if (in.read((char*)m_screenPos,sizeof(float)*2) < 0)
		return ReadError();

	//Collapsed state (dataVersion >= 20)
	if (in.read((char*)&m_showFullBody,sizeof(bool)) < 0)
		return ReadError();

	if (dataVersion > 20)
	{
		//Show in 2D boolean (dataVersion >= 21)
		if (in.read((char*)&m_dispIn2D,sizeof(bool)) < 0)
			return ReadError();

		//Show point(s) legend boolean (dataVersion >= 21)
		if (in.read((char*)&m_dispPointsLegend,sizeof(bool)) < 0)
			return ReadError();
	}

	return true;
}

void AddPointCoordinates(QStringList& body, unsigned pointIndex, ccGenericPointCloud* cloud, int precision, QString pointName = QString())
{
	assert(cloud);
	const CCVector3* P = cloud->getPointPersistentPtr(pointIndex);
	bool isShifted = cloud->isShifted();

	QString coordStr = QString("P#%0:").arg(pointIndex);
	if (!pointName.isEmpty())
		coordStr = QString("%1 (%2)").arg(pointName).arg(coordStr);
	if (isShifted)
	{
		body << coordStr;
		coordStr = QString("  [shifted]");
	}
	
	coordStr += QString(" (%1;%2;%3)").arg(P->x,0,'f',precision).arg(P->y,0,'f',precision).arg(P->z,0,'f',precision);
	body << coordStr;
	
	if (isShifted)
	{
		CCVector3d Pg = cloud->toGlobal3d(*P);
		QString globCoordStr = QString("  [original] (%1;%2;%3)").arg(Pg.x,0,'f',precision).arg(Pg.y,0,'f',precision).arg(Pg.z,0,'f',precision);
		body << globCoordStr;
	}
}

void cc2DLabel::getLabelInfo1(LabelInfo1& info) const
{
	info.cloud = 0;
	if (m_points.size() != 1)
		return;

	//cloud and point index
	info.cloud = m_points[0].cloud;
	if (!info.cloud)
	{
		assert(false);
		return;
	}
	info.pointIndex = m_points[0].index;
	//normal
	info.hasNormal = info.cloud->hasNormals();
	if (info.hasNormal)
	{
		info.normal = info.cloud->getPointNormal(info.pointIndex);
	}
	//color
	info.hasRGB = info.cloud->hasColors();
	if (info.hasRGB)
	{
		const ColorCompType* C = info.cloud->getPointColor(info.pointIndex);
		assert(C);
		info.rgb[0] = C[0];
		info.rgb[1] = C[1];
		info.rgb[2] = C[2];
	}
	//scalar field
	info.hasSF = info.cloud->hasDisplayedScalarField();
	if (info.hasSF)
	{
		info.sfValue = info.cloud->getPointScalarValue(info.pointIndex);

		info.sfName = "Scalar";
		//fetch the real scalar field name if possible
		if (info.cloud->isA(CC_TYPES::POINT_CLOUD))
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(info.cloud);
			if (pc->getCurrentDisplayedScalarField())
			{
				ccScalarField* sf = pc->getCurrentDisplayedScalarField();
				info.sfName = QString(sf->getName());
				if (ccScalarField::ValidValue(info.sfValue) && sf->getGlobalShift() != 0)
				{
					info.sfShiftedValue = sf->getGlobalShift() + info.sfValue;
					info.sfValueIsShifted = true;
				}
			}
		}
	}
}

void cc2DLabel::getLabelInfo2(LabelInfo2& info) const
{
	info.cloud1 = info.cloud2 = 0;
	if (m_points.size() != 2)
		return;

	//1st point
	info.cloud1 = m_points[0].cloud;
	info.point1Index = m_points[0].index;
	const CCVector3* P1 = info.cloud1->getPointPersistentPtr(info.point1Index);
	//2nd point
	info.cloud2 = m_points[1].cloud;
	info.point2Index = m_points[1].index;
	const CCVector3* P2 = info.cloud2->getPointPersistentPtr(info.point2Index);

	info.diff = *P2-*P1;
}

void cc2DLabel::getLabelInfo3(LabelInfo3& info) const
{
	info.cloud1 = info.cloud2 = info.cloud3 = 0;
	if (m_points.size() != 3)
		return;
	//1st point
	info.cloud1 = m_points[0].cloud;
	info.point1Index = m_points[0].index;
	const CCVector3* P1 = info.cloud1->getPointPersistentPtr(info.point1Index);
	//2nd point
	info.cloud2 = m_points[1].cloud;
	info.point2Index = m_points[1].index;
	const CCVector3* P2 = info.cloud2->getPointPersistentPtr(info.point2Index);
	//3rd point
	info.cloud3 = m_points[2].cloud;
	info.point3Index = m_points[2].index;
	const CCVector3* P3 = info.cloud3->getPointPersistentPtr(info.point3Index);

	//area
	CCVector3 P1P2 = *P2 - *P1;
	CCVector3 P1P3 = *P3 - *P1;
	CCVector3 P2P3 = *P3 - *P2;
	CCVector3 N = P1P2.cross(P1P3); //N = ABxAC
	info.area = N.norm()/2;

	//normal
	N.normalize();
	info.normal = N;

	//edges length
	info.edges.u[0] = P1P2.norm2d();  //edge 1-2
	info.edges.u[1] = P2P3.norm2d();  //edge 2-3
	info.edges.u[2] = P1P3.norm2d();  //edge 3-1

	//angle
	info.angles.u[0] = GetAngle_deg( P1P2,  P1P3);   //angleAtP1
	info.angles.u[1] = GetAngle_deg( P2P3, -P1P2);  //angleAtP2
	info.angles.u[2] = GetAngle_deg(-P1P3, -P2P3); //angleAtP3 (should be equal to 180-a1-a2!)
}

QStringList cc2DLabel::getLabelContent(int precision)
{
	QStringList body;

	switch(m_points.size())
	{
	case 0:
		//can happen if the associated cloud(s) has(ve) been deleted!
		body << "Deprecated";
		break;

	case 1: //point
		{
			LabelInfo1 info;
			getLabelInfo1(info);
			if (!info.cloud)
				break;

			//coordinates
			AddPointCoordinates(body,info.pointIndex,info.cloud,precision);

			//normal
			if (info.hasNormal)
			{
				QString normStr = QString("Normal: (%1;%2;%3)").arg(info.normal.x,0,'f',precision).arg(info.normal.y,0,'f',precision).arg(info.normal.z,0,'f',precision);
				body << normStr;
			}
			//color
			if (info.hasRGB)
			{
				QString colorStr = QString("Color: (%1;%2;%3)").arg(info.rgb[0]).arg(info.rgb[1]).arg(info.rgb[2]);
				body << colorStr;
			}
			//scalar field
			if (info.hasSF)
			{
				QString sfVal = GetSFValueAsString(info, precision);
				QString sfStr = QString("%1 = %2").arg(info.sfName).arg(sfVal);
				body << sfStr;
			}
		}
		break;

	case 2: //vector
		{
			LabelInfo2 info;
			getLabelInfo2(info);
			if (!info.cloud1 || !info.cloud2)
				break;

			//distance is now the default label title
			//PointCoordinateType dist = info.diff.norm();
			//QString distStr = QString("Distance = %1").arg(dist,0,'f',precision);
			//body << distStr;

			QString vecStr =	MathSymbolDelta + QString("X: %1\t").arg(info.diff.x,0,'f',precision)
							+	MathSymbolDelta + QString("Y: %1\t").arg(info.diff.y,0,'f',precision)
							+	MathSymbolDelta + QString("Z: %1"  ).arg(info.diff.z,0,'f',precision);

			body << vecStr;

			PointCoordinateType dXY = sqrt(info.diff.x*info.diff.x + info.diff.y*info.diff.y);
			PointCoordinateType dXZ = sqrt(info.diff.x*info.diff.x + info.diff.z*info.diff.z);
			PointCoordinateType dZY = sqrt(info.diff.z*info.diff.z + info.diff.y*info.diff.y);

			vecStr =	MathSymbolDelta + QString("XY: %1\t").arg(dXY,0,'f',precision)
					+	MathSymbolDelta + QString("XZ: %1\t").arg(dXZ,0,'f',precision)
					+	MathSymbolDelta + QString("ZY: %1"  ).arg(dZY,0,'f',precision);
			body << vecStr;

			AddPointCoordinates(body,info.point1Index,info.cloud1,precision);
			AddPointCoordinates(body,info.point2Index,info.cloud2,precision);
		}
		break;

	case 3: //triangle/plane
		{
			LabelInfo3 info;
			getLabelInfo3(info);

			//area
			QString areaStr = QString("Area = %1").arg(info.area,0,'f',precision);
			body << areaStr;

			//coordinates
			AddPointCoordinates(body,info.point1Index,info.cloud1,precision,"A");
			AddPointCoordinates(body,info.point2Index,info.cloud2,precision,"B");
			AddPointCoordinates(body,info.point3Index,info.cloud3,precision,"C");

			//normal
			QString normStr = QString("Normal: (%1;%2;%3)").arg(info.normal.x,0,'f',precision).arg(info.normal.y,0,'f',precision).arg(info.normal.z,0,'f',precision);
			body << normStr;

			//angles
			QString angleStr = QString("Angles: A=%1 - B=%2 - C=%3 deg.")
									.arg(info.angles.u[0],0,'f',precision)
									.arg(info.angles.u[1],0,'f',precision)
									.arg(info.angles.u[2],0,'f',precision);
			body << angleStr;

			//edges
			QString edgesStr = QString("Edges: AB=%1 - BC=%2 - CA=%3")
									.arg(info.edges.u[0],0,'f',precision)
									.arg(info.edges.u[1],0,'f',precision)
									.arg(info.edges.u[2],0,'f',precision);
			body << edgesStr;
		}
		break;

	default:
		assert(false);
		break;
	}

	return body;
}

bool cc2DLabel::acceptClick(int x, int y, Qt::MouseButton button)
{
	if (button == Qt::RightButton)
	{
		if (m_labelROI.contains(x - m_lastScreenPos[0], y - m_lastScreenPos[1]))
		{
			//toggle collapse state
			m_showFullBody = !m_showFullBody;
			return true;
		}
	}

	return false;
}

void cc2DLabel::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (m_points.empty())
		return;

	//2D foreground only
	if (!MACRO_Foreground(context))
		return;

	//Not compatible with virtual transformation (see ccDrawableObject::enableGLTransformation)
	if (MACRO_VirtualTransEnabled(context))
		return;

	if (MACRO_Draw3D(context))
		drawMeOnly3D(context);
	else if (MACRO_Draw2D(context))
		drawMeOnly2D(context);
}

//unit point marker
static QSharedPointer<ccSphere> c_unitPointMarker(0);

void cc2DLabel::drawMeOnly3D(CC_DRAW_CONTEXT& context)
{
	size_t count = m_points.size();
	if (count == 0)
	{
		return;
	}

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	if ( glFunc == nullptr )
	{
		assert(false);
		return;
	}

	//standard case: list names pushing
	bool pushName = MACRO_DrawEntityNames(context);
	if (pushName)
	{
		//not particularly fast
		if (MACRO_DrawFastNamesOnly(context))
			return;
		glFunc->glPushName(getUniqueIDForDisplay());
	}

	//we always project the points in 2D (maybe useful later, even when displaying the label during the 2D pass!)
	ccGLCameraParameters camera;
	//we can't use the context 'ccGLCameraParameters' (viewport, modelView matrix, etc. )
	//because it doesn't take the temporary 'GL transformation' into account!
	//context.display->getGLCameraParameters(camera);
	glFunc->glGetIntegerv(GL_VIEWPORT, camera.viewport);
	glFunc->glGetDoublev(GL_PROJECTION_MATRIX, camera.projectionMat.data());
	glFunc->glGetDoublev(GL_MODELVIEW_MATRIX, camera.modelViewMat.data());

	//don't do this in picking mode!
	if (!pushName)
	{
		for (size_t i = 0; i < count; i++)
		{
			//project the point in 2D
			const CCVector3* P3D = m_points[i].cloud->getPoint(m_points[i].index);
			camera.project(*P3D, m_points[i].pos2D);
		}
	}

	bool loop = false;
	switch (count)
	{
	case 3:
		{
			glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT);
			glFunc->glEnable(GL_BLEND);

			//we draw the triangle
			glFunc->glColor4ub(255,255,0,128);
			glFunc->glBegin(GL_TRIANGLES);
			ccGL::Vertex3v(glFunc, m_points[0].cloud->getPoint(m_points[0].index)->u);
			ccGL::Vertex3v(glFunc, m_points[1].cloud->getPoint(m_points[1].index)->u);
			ccGL::Vertex3v(glFunc, m_points[2].cloud->getPoint(m_points[2].index)->u);
			glFunc->glEnd();

			glFunc->glPopAttrib(); //GL_COLOR_BUFFER_BIT
			loop = true;
		}
	case 2:
		{
			//This is now done in 2D so that it's always displayed on top of the entities
			////segment width
			//const float c_sizeFactor = 4.0f;
			//glFunc->glPushAttrib(GL_LINE_BIT);
			//glFunc->glLineWidth(c_sizeFactor * context.renderZoom);

			////we draw the segments
			//if (isSelected())
			//	ccGL::Color3v(glFunc, ccColor::red.rgba);
			//else
			//	ccGL::Color3v(glFunc, ccColor::green.rgba);
			//
			//glFunc->glBegin(GL_LINES);
			//for (unsigned i=0; i<count; i++)
			//{
			//	if (i+1<count || loop)
			//	{
			//		ccGL::Vertex3v(glFunc, m_points[i].cloud->getPoint(m_points[i].index)->u);
			//		ccGL::Vertex3v(glFunc, m_points[(i+1)%count].cloud->getPoint(m_points[(i+1)%count].index)->u);
			//	}
			//}
			//glFunc->glEnd();
			//glFunc->glPopAttrib(); //GL_LINE_BIT
		}

	case 1:
		{
			//display point marker as spheres
			{
				if (!c_unitPointMarker)
				{
					c_unitPointMarker = QSharedPointer<ccSphere>(new ccSphere(1.0f, 0, "PointMarker", 12));
					c_unitPointMarker->showColors(true);
					c_unitPointMarker->setVisible(true);
					c_unitPointMarker->setEnabled(true);
				}
			
				//build-up point maker own 'context'
				CC_DRAW_CONTEXT markerContext = context;
				markerContext.drawingFlags &= (~CC_DRAW_ENTITY_NAMES); //we must remove the 'push name flag' so that the sphere doesn't push its own!
				markerContext.display = 0;

				if (isSelected() && !pushName)
					c_unitPointMarker->setTempColor(ccColor::red);
				else
					c_unitPointMarker->setTempColor(context.labelDefaultMarkerCol);

				const ccViewportParameters& viewportParams = context.display->getViewportParameters();
				for (size_t i = 0; i<count; i++)
				{
					glFunc->glMatrixMode(GL_MODELVIEW);
					glFunc->glPushMatrix();
					const CCVector3* P = m_points[i].cloud->getPoint(m_points[i].index);
					ccGL::Translate(glFunc, P->x, P->y, P->z);
					float scale = context.labelMarkerSize * m_relMarkerScale;
					if (viewportParams.perspectiveView && viewportParams.zFar > 0)
					{
						//in perspective view, the actual scale depends on the distance to the camera!
						double d = (camera.modelViewMat * CCVector3d::fromArray(P->u)).norm();
						double unitD = viewportParams.zFar / 2; //we consider that the 'standard' scale is at half the depth
						scale = static_cast<float>(scale * sqrt(d / unitD)); //sqrt = empirical (probably because the marker size is already partly compensated by ccGLWindow::computeActualPixelSize())
					}
					glFunc->glScalef(scale, scale, scale);
					c_unitPointMarker->draw(markerContext);
					glFunc->glPopMatrix();
				}
			}
		}
	}

	if (pushName)
	{
		glFunc->glPopName();
	}
}

//display parameters
static const int c_margin = 5;
static const int c_tabMarginX = 5;
static const int c_tabMarginY = 2;
static const int c_arrowBaseSize = 3;
//static const int c_buttonSize = 10;

static const ccColor::Rgba c_darkGreen(0,200,0,255);

//! Data table
struct Tab
{
	//! Default constructor
	Tab(int _maxBlockPerRow = 2)
		: maxBlockPerRow(_maxBlockPerRow)
		, blockCount(0)
		, rowCount(0)
		, colCount(0)
	{}

	//! Sets the maximum number of blocks per row
	/** \warning Must be called before adding data!
	**/
	inline void setMaxBlockPerRow(int maxBlock) { maxBlockPerRow = maxBlock; }

	//! Adds a 2x3 block (must be filled!)
	int add2x3Block()
	{
		//add columns (if necessary)
		if (colCount < maxBlockPerRow*2)
		{
			colCount += 2;
			colContent.resize(colCount);
			colWidth.resize(colCount,0);
		}
		int blockCol = (blockCount % maxBlockPerRow);
		//add new row
		if (blockCol == 0)
			rowCount += 3;
		++blockCount;

		//return the first column index of the block
		return blockCol*2;
	}

	//! Updates columns width table
	/** \return the total width
	**/
	int updateColumnsWidthTable(const QFontMetrics& fm)
	{
		//compute min width of each column
		int totalWidth = 0;
		for (int i=0; i<colCount; ++i)
		{
			int maxWidth = 0;
			for (int j=0; j<colContent[i].size(); ++j)
				maxWidth = std::max(maxWidth, fm.width(colContent[i][j]));
			colWidth[i] = maxWidth;
			totalWidth += maxWidth;
		}
		return totalWidth;
	}

	//! Maximum number of blocks per row
	int maxBlockPerRow;
	//! Number of 2x3 blocks
	int blockCount;
	//! Number of rows
	int rowCount;
	//! Number of columns
	int colCount;
	//! Columns width
	std::vector<int> colWidth;
	//! Columns content
	std::vector<QStringList> colContent;
};

void cc2DLabel::drawMeOnly2D(CC_DRAW_CONTEXT& context)
{
	assert(!m_points.empty());

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	if ( glFunc == nullptr )
	{
		assert(false);
		return;
	}

	//standard case: list names pushing
	bool pushName = MACRO_DrawEntityNames(context);
	if (pushName)
	{
		glFunc->glPushName(getUniqueID());
	}

	float halfW = context.glW / 2.0f;
	float halfH = context.glH / 2.0f;

	size_t count = m_points.size();
	assert(count != 0);

	//we should already be in orthoprojective & centered mode
	//glFunc->glOrtho(-halfW, halfW, -halfH, halfH, -maxS, maxS);

	//hack: we display the label connecting 'segments' and the point(s) legend
	//in 2D so that they always appear above the entities
	{
		//test if the label points are visible
		size_t visibleCount = 0;
		for (unsigned j = 0; j < count; ++j)
		{
			if (m_points[j].pos2D.z >= 0.0 && m_points[j].pos2D.z <= 1.0)
			{
				++visibleCount;
			}
		}

		if (visibleCount)
		{
			glFunc->glPushAttrib(GL_DEPTH_BUFFER_BIT);
			glFunc->glDisable(GL_DEPTH_TEST);

			//contour segments (before the labels!)
			if (count > 1)
			{
				//segment width
				const float c_sizeFactor = 4.0f;
				glFunc->glPushAttrib(GL_LINE_BIT);
				glFunc->glLineWidth(c_sizeFactor * context.renderZoom);

				//we draw the segments
				if (isSelected())
					ccGL::Color3v(glFunc, ccColor::red.rgba);
				else
					ccGL::Color3v(glFunc, context.labelDefaultMarkerCol.rgb/*ccColor::green.rgba*/);

				glFunc->glBegin(count == 2 ? GL_LINES : GL_LINE_LOOP);
				for (unsigned j = 0; j < count; ++j)
				{
					glFunc->glVertex2d(m_points[j].pos2D.x - halfW, m_points[j].pos2D.y - halfH);
				}
				glFunc->glEnd();
				glFunc->glPopAttrib(); //GL_LINE_BIT
			}

			//no need to display the point(s) legend in picking mode
			if (m_dispPointsLegend && !pushName)
			{
				QFont font(context.display->getTextDisplayFont()); //takes rendering zoom into account!
				//font.setPointSize(font.pointSize() + 2);
				font.setBold(true);
				static const QChar ABC[3] = { 'A', 'B', 'C' };

				//draw the label 'legend(s)'
				for (size_t j = 0; j < count; j++)
				{
					QString title;
					if (count == 1)
						title = getName(); //for single-point labels we prefer the name
					else if (count == 3)
						title = ABC[j]; //for triangle-labels, we only display "A","B","C"
					else
						title = QString("P#%0").arg(m_points[j].index);

					context.display->displayText(	title,
													static_cast<int>(m_points[j].pos2D.x) + context.labelMarkerTextShift_pix,
													static_cast<int>(m_points[j].pos2D.y) + context.labelMarkerTextShift_pix,
													ccGenericGLDisplay::ALIGN_DEFAULT,
													context.labelOpacity / 100.0f,
													ccColor::white.rgba,
													&font);
				}
			}

			glFunc->glPopAttrib(); //GL_DEPTH_BUFFER_BIT
		}
		else
		{
			//no need to draw anything (might be confusing)
			return;
		}
	}
	
	if (!m_dispIn2D)
	{
		//nothing to do
		return;
	}

	//label title
	const int precision = context.dispNumberPrecision;
	QString title = getTitle(precision);

#define DRAW_CONTENT_AS_TAB
#ifdef DRAW_CONTENT_AS_TAB
	//draw contents as an array
	Tab tab(4);
	int rowHeight = 0;
#else
//simply display the content as text
	QStringList body;
#endif

	//render zoom
	int margin        = static_cast<int>(c_margin        * context.renderZoom);
	int tabMarginX    = static_cast<int>(c_tabMarginX    * context.renderZoom);
	int tabMarginY    = static_cast<int>(c_tabMarginY    * context.renderZoom);
	int arrowBaseSize = static_cast<int>(c_arrowBaseSize * context.renderZoom);
	
	int titleHeight = 0;
	QFont bodyFont, titleFont;
	if (!pushName)
	{
		/*** label border ***/
		bodyFont = context.display->getLabelDisplayFont(); //takes rendering zoom into account!
		titleFont = bodyFont; //takes rendering zoom into account!
		//titleFont.setBold(true);

		QFontMetrics titleFontMetrics(titleFont);
		titleHeight = titleFontMetrics.height();

		QFontMetrics bodyFontMetrics(bodyFont);
		rowHeight = bodyFontMetrics.height();

		//get label box dimension
		int dx = 100;
		int dy = 0;
		//int buttonSize    = static_cast<int>(c_buttonSize * context.renderZoom);
		{
			//base box dimension
			dx = std::max(dx, titleFontMetrics.width(title));
			dy += margin;		//top vertical margin
			dy += titleHeight;	//title

			if (m_showFullBody)
			{
#ifdef DRAW_CONTENT_AS_TAB
				try
				{
					if (count == 1)
					{
						LabelInfo1 info;
						getLabelInfo1(info);

						bool isShifted = info.cloud->isShifted();
						//1st block: X, Y, Z (local)
						{
							int c = tab.add2x3Block();
							QChar suffix;
							if (isShifted)
							{
								suffix = 'l'; //'l' for local
							}
							const CCVector3* P = info.cloud->getPoint(info.pointIndex);
							tab.colContent[c] << QString("X") + suffix; tab.colContent[c+1] << QString::number(P->x,'f',precision);
							tab.colContent[c] << QString("Y") + suffix; tab.colContent[c+1] << QString::number(P->y,'f',precision);
							tab.colContent[c] << QString("Z") + suffix; tab.colContent[c+1] << QString::number(P->z,'f',precision);
						}
						//next block:  X, Y, Z (global)
						if (isShifted)
						{
							int c = tab.add2x3Block();
							CCVector3d P = info.cloud->toGlobal3d(*info.cloud->getPoint(info.pointIndex));
							tab.colContent[c] << "Xg"; tab.colContent[c+1] << QString::number(P.x,'f',precision);
							tab.colContent[c] << "Yg"; tab.colContent[c+1] << QString::number(P.y,'f',precision);
							tab.colContent[c] << "Zg"; tab.colContent[c+1] << QString::number(P.z,'f',precision);
						}
						//next block: normal
						if (info.hasNormal)
						{
							int c = tab.add2x3Block();
							tab.colContent[c] << "Nx"; tab.colContent[c+1] << QString::number(info.normal.x,'f',precision);
							tab.colContent[c] << "Ny"; tab.colContent[c+1] << QString::number(info.normal.y,'f',precision);
							tab.colContent[c] << "Nz"; tab.colContent[c+1] << QString::number(info.normal.z,'f',precision);
						}

						//next block: RGB color
						if (info.hasRGB)
						{
							int c = tab.add2x3Block();
							tab.colContent[c] <<"R"; tab.colContent[c+1] << QString::number(info.rgb.x);
							tab.colContent[c] <<"G"; tab.colContent[c+1] << QString::number(info.rgb.y);
							tab.colContent[c] <<"B"; tab.colContent[c+1] << QString::number(info.rgb.z);
						}
					}
					else if (count == 2)
					{
						LabelInfo2 info;
						getLabelInfo2(info);

						//1st block: dX, dY, dZ
						{
							int c = tab.add2x3Block();
							tab.colContent[c] << MathSymbolDelta + QString("X"); tab.colContent[c+1] << QString::number(info.diff.x,'f',precision);
							tab.colContent[c] << MathSymbolDelta + QString("Y"); tab.colContent[c+1] << QString::number(info.diff.y,'f',precision);
							tab.colContent[c] << MathSymbolDelta + QString("Z"); tab.colContent[c+1] << QString::number(info.diff.z,'f',precision);
						}
						//2nd block: dXY, dXZ, dZY
						{
							int c = tab.add2x3Block();
							PointCoordinateType dXY = sqrt(info.diff.x*info.diff.x + info.diff.y*info.diff.y);
							PointCoordinateType dXZ = sqrt(info.diff.x*info.diff.x + info.diff.z*info.diff.z);
							PointCoordinateType dZY = sqrt(info.diff.z*info.diff.z + info.diff.y*info.diff.y);
							tab.colContent[c] << MathSymbolDelta + QString("XY"); tab.colContent[c + 1] << QString::number(dXY, 'f', precision);
							tab.colContent[c] << MathSymbolDelta + QString("XZ"); tab.colContent[c + 1] << QString::number(dXZ, 'f', precision);
							tab.colContent[c] << MathSymbolDelta + QString("ZY"); tab.colContent[c + 1] << QString::number(dZY, 'f', precision);
						}
					}
					else if (count == 3)
					{
						LabelInfo3 info;
						getLabelInfo3(info);
						tab.setMaxBlockPerRow(2); //square tab (2x2 blocks)

						//next block: indexes
						{
							int c = tab.add2x3Block();
							tab.colContent[c] << "index.A"; tab.colContent[c + 1] << QString::number(info.point1Index);
							tab.colContent[c] << "index.B"; tab.colContent[c + 1] << QString::number(info.point2Index);
							tab.colContent[c] << "index.C"; tab.colContent[c + 1] << QString::number(info.point3Index);
						}
						//next block: edges length
						{
							int c = tab.add2x3Block();
							tab.colContent[c] << "AB"; tab.colContent[c + 1] << QString::number(info.edges.u[0], 'f', precision);
							tab.colContent[c] << "BC"; tab.colContent[c + 1] << QString::number(info.edges.u[1], 'f', precision);
							tab.colContent[c] << "CA"; tab.colContent[c + 1] << QString::number(info.edges.u[2], 'f', precision);
						}
						//next block: angles
						{
							int c = tab.add2x3Block();
							tab.colContent[c] << "angle.A"; tab.colContent[c + 1] << QString::number(info.angles.u[0], 'f', precision);
							tab.colContent[c] << "angle.B"; tab.colContent[c + 1] << QString::number(info.angles.u[1], 'f', precision);
							tab.colContent[c] << "angle.C"; tab.colContent[c + 1] << QString::number(info.angles.u[2], 'f', precision);
						}
						//next block: normal
						{
							int c = tab.add2x3Block();
							tab.colContent[c] << "Nx"; tab.colContent[c + 1] << QString::number(info.normal.x, 'f', precision);
							tab.colContent[c] << "Ny"; tab.colContent[c + 1] << QString::number(info.normal.y, 'f', precision);
							tab.colContent[c] << "Nz"; tab.colContent[c + 1] << QString::number(info.normal.z, 'f', precision);
						}
					}
				}
				catch (const std::bad_alloc&)
				{
					//not enough memory
					return;
				}

				//compute min width of each column
				int totalWidth = tab.updateColumnsWidthTable(bodyFontMetrics);

				int tabWidth = totalWidth + tab.colCount * (2 * tabMarginX); //add inner margins
				dx = std::max(dx, tabWidth);
				dy += tab.rowCount * (rowHeight + 2 * tabMarginY); //add inner margins
				//we also add a margin every 3 rows
				dy += std::max(0, (tab.rowCount / 3) - 1) * margin;
				dy += margin;		//bottom vertical margin
#else
				body = getLabelContent(precision);
				if (!body.empty())
				{
					dy += margin;	//vertical margin above separator
					for (int j=0; j<body.size(); ++j)
					{
						dx = std::max(dx,bodyFontMetrics.width(body[j]));
						dy += rowHeight; //body line height
					}
					dy += margin;	//vertical margin below text
				}
#endif //DRAW_CONTENT_AS_TAB
			}

			dx += margin * 2;	// horizontal margins
		}

		//main rectangle
		m_labelROI = QRect(0, 0, dx, dy);

		//close button
		//m_closeButtonROI.right()   = dx-margin;
		//m_closeButtonROI.left()    = m_closeButtonROI.right()-buttonSize;
		//m_closeButtonROI.bottom()  = margin;
		//m_closeButtonROI.top()     = m_closeButtonROI.bottom()+buttonSize;

		//automatically elide the title
		//title = titleFontMetrics.elidedText(title, Qt::ElideRight, m_closeButtonROI[0] - 2 * margin);
	}

	//draw label rectangle
	const int xStart = static_cast<int>(context.glW * m_screenPos[0]);
	const int yStart = static_cast<int>(context.glH * (1.0f - m_screenPos[1]));

	m_lastScreenPos[0] = xStart;
	m_lastScreenPos[1] = yStart - m_labelROI.height();

	//colors
	bool highlighted = (!pushName && isSelected());
	//default background color
	unsigned char alpha = static_cast<unsigned char>((context.labelOpacity / 100.0) * 255);
	ccColor::Rgbaub defaultBkgColor(context.labelDefaultBkgCol, alpha);
	//default border color (mustn't be totally transparent!)
	ccColor::Rgbaub defaultBorderColor(ccColor::red);
	if (!highlighted)
	{
		//apply only half of the transparency
		unsigned char halfAlpha = static_cast<unsigned char>((50.0 + context.labelOpacity / 200.0) * 255);
		defaultBorderColor = ccColor::Rgbaub(context.labelDefaultBkgCol, halfAlpha);
	}

	glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT);
	glFunc->glEnable(GL_BLEND);

	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();
	glFunc->glTranslatef(static_cast<GLfloat>(xStart - halfW), static_cast<GLfloat>(yStart - halfH), 0);

	if (!pushName)
	{
		//compute arrow base position relatively to the label rectangle (for 0 to 8)
		int arrowBaseConfig = 0;

		//compute arrow head position
		CCVector3d arrowDest2D(0, 0, 0);
		for (size_t i = 0; i < count; ++i)
		{
			arrowDest2D += m_points[i].pos2D;
		}
		arrowDest2D /= static_cast<PointCoordinateType>(count);
		//arrowDest2D.x -= halfW;
		//arrowDest2D.y -= halfH;
		
		int iArrowDestX = static_cast<int>(arrowDest2D.x - xStart);
		int iArrowDestY = static_cast<int>(arrowDest2D.y - yStart);
		{
			if (iArrowDestX < m_labelROI.left()) //left
				arrowBaseConfig += 0;
			else if (iArrowDestX > m_labelROI.right()) //Right
				arrowBaseConfig += 2;
			else  //Middle
				arrowBaseConfig += 1;

			if (iArrowDestY > -m_labelROI.top()) //Top
				arrowBaseConfig += 0;
			else if (iArrowDestY < -m_labelROI.bottom()) //Bottom
				arrowBaseConfig += 6;
			else  //Middle
				arrowBaseConfig += 3;
		}

		//we make the arrow base start from the nearest corner
		if (arrowBaseConfig != 4) //4 = label above point!
		{
			glFunc->glColor4ubv(defaultBorderColor.rgba);
			glFunc->glBegin(GL_TRIANGLE_FAN);
			glFunc->glVertex2i(iArrowDestX, iArrowDestY);
			switch(arrowBaseConfig)
			{
			case 0: //top-left corner
				glFunc->glVertex2i(m_labelROI.left(), -m_labelROI.top() - 2 * arrowBaseSize);
				glFunc->glVertex2i(m_labelROI.left(), -m_labelROI.top());
				glFunc->glVertex2i(m_labelROI.left() + 2 * arrowBaseSize, -m_labelROI.top());
				break;
			case 1: //top-middle edge
				glFunc->glVertex2i(std::max(m_labelROI.left(), iArrowDestX - arrowBaseSize), -m_labelROI.top());
				glFunc->glVertex2i(std::min(m_labelROI.right(), iArrowDestX + arrowBaseSize), -m_labelROI.top());
				break;
			case 2: //top-right corner
				glFunc->glVertex2i(m_labelROI.right(), -m_labelROI.top() - 2 * arrowBaseSize);
				glFunc->glVertex2i(m_labelROI.right(), -m_labelROI.top());
				glFunc->glVertex2i(m_labelROI.right() - 2 * arrowBaseSize, -m_labelROI.top());
				break;
			case 3: //middle-left edge
				glFunc->glVertex2i(m_labelROI.left(), std::min(-m_labelROI.top(), iArrowDestY + arrowBaseSize));
				glFunc->glVertex2i(m_labelROI.left(), std::max(-m_labelROI.bottom(), iArrowDestY - arrowBaseSize));
				break;
			case 4: //middle of rectangle!
				break;
			case 5: //middle-right edge
				glFunc->glVertex2i(m_labelROI.right(), std::min(-m_labelROI.top(), iArrowDestY + arrowBaseSize));
				glFunc->glVertex2i(m_labelROI.right(), std::max(-m_labelROI.bottom(), iArrowDestY - arrowBaseSize));
				break;
			case 6: //bottom-left corner
				glFunc->glVertex2i(m_labelROI.left(), -m_labelROI.bottom() + 2 * arrowBaseSize);
				glFunc->glVertex2i(m_labelROI.left(), -m_labelROI.bottom());
				glFunc->glVertex2i(m_labelROI.left() + 2 * arrowBaseSize, -m_labelROI.bottom());
				break;
			case 7: //bottom-middle edge
				glFunc->glVertex2i(std::max(m_labelROI.left(), iArrowDestX - arrowBaseSize), -m_labelROI.bottom());
				glFunc->glVertex2i(std::min(m_labelROI.right(), iArrowDestX + arrowBaseSize), -m_labelROI.bottom());
				break;
			case 8: //bottom-right corner
				glFunc->glVertex2i(m_labelROI.right(), -m_labelROI.bottom() + 2 * arrowBaseSize);
				glFunc->glVertex2i(m_labelROI.right(), -m_labelROI.bottom());
				glFunc->glVertex2i(m_labelROI.right() - 2 * arrowBaseSize, -m_labelROI.bottom());
				break;
			}
			glFunc->glEnd();
		}
	}

	//main rectangle
	glFunc->glColor4ubv(defaultBkgColor.rgba);
	glFunc->glBegin(GL_QUADS);
	glFunc->glVertex2i(m_labelROI.left(),  -m_labelROI.top());
	glFunc->glVertex2i(m_labelROI.left(),  -m_labelROI.bottom());
	glFunc->glVertex2i(m_labelROI.right(), -m_labelROI.bottom());
	glFunc->glVertex2i(m_labelROI.right(), -m_labelROI.top());
	glFunc->glEnd();

	//if (highlighted)
	{
		glFunc->glPushAttrib(GL_LINE_BIT);
		glFunc->glLineWidth(3.0f * context.renderZoom);
		glFunc->glColor4ubv(defaultBorderColor.rgba);
		glFunc->glBegin(GL_LINE_LOOP);
		glFunc->glVertex2i(m_labelROI.left(),  -m_labelROI.top());
		glFunc->glVertex2i(m_labelROI.left(),  -m_labelROI.bottom());
		glFunc->glVertex2i(m_labelROI.right(), -m_labelROI.bottom());
		glFunc->glVertex2i(m_labelROI.right(), -m_labelROI.top());
		glFunc->glEnd();
		glFunc->glPopAttrib(); //GL_LINE_BIT
	}

	//draw close button
	//glFunc->glColor3ubv(ccColor::black);
	//glFunc->glBegin(GL_LINE_LOOP);
	//glFunc->glVertex2i(m_closeButtonROI.left(),-m_closeButtonROI.top());
	//glFunc->glVertex2i(m_closeButtonROI.left(),-m_closeButtonROI.bottom());
	//glFunc->glVertex2i(m_closeButtonROI.right(),-m_closeButtonROI.bottom());
	//glFunc->glVertex2i(m_closeButtonROI.right(),-m_closeButtonROI.top());
	//glFunc->glEnd();
	//glFunc->glBegin(GL_LINES);
	//glFunc->glVertex2i(m_closeButtonROI.left()+2,-m_closeButtonROI.top()+2);
	//glFunc->glVertex2i(m_closeButtonROI.right()-2,-m_closeButtonROI.bottom()-2);
	//glFunc->glVertex2i(m_closeButtonROI.right()-2,-m_closeButtonROI.top()+2);
	//glFunc->glVertex2i(m_closeButtonROI.left()+2,-m_closeButtonROI.bottom()-2);
	//glFunc->glEnd();

	//display text
	if (!pushName)
	{
		int xStartRel = margin;
		int yStartRel = 0;
		yStartRel -= titleHeight;

		ccColor::Rgbub defaultTextColor;
		if (context.labelOpacity < 40)
		{
			//under a given opacity level, we use the default text color instead!
			defaultTextColor = context.textDefaultCol;
		}
		else
		{
			defaultTextColor = ccColor::Rgbub(	255 - context.labelDefaultBkgCol.r,
												255 - context.labelDefaultBkgCol.g,
												255 - context.labelDefaultBkgCol.b);
		}

		//label title
		context.display->displayText(	title,
										xStart + xStartRel,
										yStart + yStartRel,
										ccGenericGLDisplay::ALIGN_DEFAULT,
										0,
										defaultTextColor.rgb,
										&titleFont);
		yStartRel -= margin;
		
		if (m_showFullBody)
		{
#ifdef DRAW_CONTENT_AS_TAB
			int xCol = xStartRel;
			for (int c = 0; c < tab.colCount; ++c)
			{
				int width = tab.colWidth[c] + 2*tabMarginX;
				int height = rowHeight + 2*tabMarginY;

				int yRow = yStartRel;
				int actualRowCount = std::min(tab.rowCount,tab.colContent[c].size());

				bool labelCol = ((c & 1) == 0);
				const unsigned char* textColor = labelCol ? ccColor::white.rgba : defaultTextColor.rgb;
				
				for (int r=0; r<actualRowCount; ++r)
				{
					if (r && (r % 3) == 0)
						yRow -= margin;

					if (labelCol)
					{
						//draw background
						int rgbIndex = (r % 3);
						if (rgbIndex == 0)
							glFunc->glColor3ubv(ccColor::red.rgba);
						else if (rgbIndex == 1)
							glFunc->glColor3ubv(c_darkGreen.rgba);
						else if (rgbIndex == 2)
							glFunc->glColor3ubv(ccColor::blue.rgba);

						glFunc->glBegin(GL_QUADS);
						glFunc->glVertex2i(m_labelROI.left() + xCol, -m_labelROI.top() + yRow);
						glFunc->glVertex2i(m_labelROI.left() + xCol, -m_labelROI.top() + yRow - height);
						glFunc->glVertex2i(m_labelROI.left() + xCol + width, -m_labelROI.top() + yRow - height);
						glFunc->glVertex2i(m_labelROI.left() + xCol + width, -m_labelROI.top() + yRow);
						glFunc->glEnd();
					}

					const QString& str = tab.colContent[c][r];

					int xShift = 0;
					if (labelCol)
					{
						//align characters in the middle
						xShift = (tab.colWidth[c] - QFontMetrics(bodyFont).width(str)) / 2;
					}
					else
					{
						//align digits on the right
						xShift = tab.colWidth[c] - QFontMetrics(bodyFont).width(str);
					}

					context.display->displayText(	str,
													xStart + xCol + tabMarginX + xShift,
													yStart + yRow - rowHeight, ccGenericGLDisplay::ALIGN_DEFAULT, 0, textColor, &bodyFont);

					yRow -= height;
				}

				xCol += width;
			}
#else
			if (!body.empty())
			{
				//display body
				yStartRel -= margin;
				for (int i=0; i<body.size(); ++i)
				{
					yStartRel -= rowHeight;
					context.display->displayText(body[i],xStart+xStartRel,yStart+yStartRel,ccGenericGLDisplay::ALIGN_DEFAULT,0,defaultTextColor.rgb,&bodyFont);
				}
			}
#endif //DRAW_CONTENT_AS_TAB
		}
	}

	glFunc->glPopAttrib(); //GL_COLOR_BUFFER_BIT

	glFunc->glPopMatrix();

	if (pushName)
	{
		glFunc->glPopName();
	}
}
