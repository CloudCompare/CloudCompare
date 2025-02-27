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
#include "ccGenericGLDisplay.h"
#include "ccGenericPointCloud.h"
#include "ccPointCloud.h"
#include "ccGenericMesh.h"
#include "ccScalarField.h"
#include "ccSphere.h"

//Qt
#include <QFontMetrics>
#include <QSharedPointer>

//System
#include <assert.h>
#include <string.h>

//'Delta' character
static const QChar MathSymbolDelta(0x0394);

static const QString CENTER_STRING = QObject::tr("Center");
static const char POINT_INDEX_0[]  = "pi0";
static const char POINT_INDEX_1[]  = "pi1";
static const char POINT_INDEX_2[]  = "pi2";
static const char ENTITY_INDEX_0[] = "ei0";
static const char ENTITY_INDEX_1[] = "ei1";
static const char ENTITY_INDEX_2[] = "ei2";


QString cc2DLabel::PickedPoint::itemTitle() const
{
	if (entityCenterPoint)
	{
		QString title = CENTER_STRING;
		if (entity())
			title += QString("@%1").arg(entity()->getUniqueID());
		return title;
	}
	else
	{
		return QString::number(index);
	}
}

QString cc2DLabel::PickedPoint::prefix(const char* pointTag) const
{
	if (entityCenterPoint)
	{
		return CENTER_STRING;
	}
	else if (_cloud)
	{
		return QString("Point #") + pointTag;
	}
	else if (_mesh)
	{
		return QString("Point@Tri#") + pointTag;
	}

	assert(false);
	return QString();
}

CCVector3 cc2DLabel::PickedPoint::getPointPosition() const
{
	CCVector3 P;

	if (_cloud)
	{
		if (entityCenterPoint)
		{
			return _cloud->getOwnBB().getCenter();
		}
		else
		{
			P = *_cloud->getPointPersistentPtr(index);
		}
	}
	else if (_mesh)
	{
		if (entityCenterPoint)
		{
			return _mesh->getOwnBB().getCenter();
		}
		else
		{
			_mesh->computePointPosition(index, uv, P);
		}
	}
	else
	{
		assert(false);
	}

	return P;
}

unsigned cc2DLabel::PickedPoint::getUniqueID() const
{
	if (_cloud)
		return _cloud->getUniqueID();
	if (_mesh)
		return _mesh->getUniqueID();

	assert(false);
	return 0;
}

ccGenericPointCloud* cc2DLabel::PickedPoint::cloudOrVertices() const
{
	if (_cloud)
		return _cloud;
	if (_mesh)
		return _mesh->getAssociatedCloud();

	assert(false);
	return nullptr;
}

ccHObject* cc2DLabel::PickedPoint::entity() const
{
	if (_cloud)
		return _cloud;
	if (_mesh)
		return _mesh;

	assert(false);
	return nullptr;
}

cc2DLabel::cc2DLabel(QString name/*=QString()*/)
	: ccHObject(name.isEmpty() ? "label" : name)
	, m_showFullBody(true)
	, m_screenPos{ 0.05f, 0.5f }
	, m_lastScreenPos{ 0, 0 }
	, m_dispPointsLegend(false)
	, m_dispIn2D(true)
	, m_relMarkerScale(1.0f)
{
	clear(false);

	lockVisibility(false);
	setEnabled(true);
}

cc2DLabel::cc2DLabel(const cc2DLabel& label, bool copyPoints/*=true*/)
	: ccHObject(label)
	, m_showFullBody(label.m_showFullBody)
	, m_screenPos(label.m_screenPos)
	, m_lastScreenPos(label.m_lastScreenPos)
	, m_labelROI(label.m_labelROI)
	, m_dispPointsLegend(label.m_dispPointsLegend)
	, m_dispIn2D(label.m_dispIn2D)
	, m_relMarkerScale(label.m_relMarkerScale)
{
	if (copyPoints)
	{
		m_pickedPoints = label.m_pickedPoints;
	}
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
			return QString::number(info.sfValue, 'f', precision);
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
	size_t count = m_pickedPoints.size();
	if (count == 1)
	{
		title = m_name;
		title.replace(POINT_INDEX_0, m_pickedPoints[0].itemTitle());

		//if available, we display the point SF value
		LabelInfo1 info;
		getLabelInfo1(info);
		if (info.hasSF)
		{
			QString sfVal = GetSFValueAsString(info, precision);
			title = QString("%1 = %2").arg(info.sfName, sfVal);
		}
	}
	else if (count == 2)
	{
		LabelInfo2 info;
		getLabelInfo2(info);
		//display distance by default
		double dist = info.diff.normd();
		title = QString("Distance: %1").arg(dist, 0, 'f', precision);
	}
	else if (count == 3)
	{
		LabelInfo3 info;
		getLabelInfo3(info);
		//display area by default
		title = QString("Area: %1").arg(info.area, 0, 'f', precision);
	}

	return title;
}

QString cc2DLabel::getName() const
{
	QString processedName = m_name;

	size_t count = m_pickedPoints.size();
	if (count > 0)
	{
		processedName.replace(POINT_INDEX_0, m_pickedPoints[0].itemTitle());
		if (count > 1)
		{
			processedName.replace(ENTITY_INDEX_0, QString::number(m_pickedPoints[0].getUniqueID()));

			processedName.replace(POINT_INDEX_1, m_pickedPoints[1].itemTitle());
			processedName.replace(ENTITY_INDEX_1, QString::number(m_pickedPoints[1].getUniqueID()));

			if (count > 2)
			{
				processedName.replace(POINT_INDEX_2, m_pickedPoints[2].itemTitle());
				processedName.replace(ENTITY_INDEX_2, QString::number(m_pickedPoints[2].getUniqueID()));
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
		m_pickedPoints.resize(0);
	}
	else
	{
		//remove all dependencies first!
		while (!m_pickedPoints.empty())
		{
			PickedPoint& pp = m_pickedPoints.back();
			if (pp.entity())
				pp.entity()->removeDependencyWith(this);
			m_pickedPoints.pop_back();
		}
	}

	m_lastScreenPos[0] = m_lastScreenPos[1] = -1;
	m_labelROI = QRect(0, 0, 0, 0);
	setVisible(false);
	setName("Label");
}

void cc2DLabel::onDeletionOf(const ccHObject* obj)
{
	ccHObject::onDeletionOf(obj); //remove dependencies, etc.

	//check that associated clouds are not about to be deleted!
	size_t pointsToRemove = 0;
	{
		for (size_t i = 0; i < m_pickedPoints.size(); ++i)
			if (m_pickedPoints[i].entity() == obj)
				++pointsToRemove;
	}

	if (pointsToRemove == 0)
	{
		return;
	}

	if (pointsToRemove == m_pickedPoints.size())
	{
		clear(true); //don't call clear as we don't want/need to update input object's dependencies!
	}
	else
	{
		//remove only the necessary points
		size_t j = 0;
		for (size_t i = 0; i < m_pickedPoints.size(); ++i)
		{
			if (m_pickedPoints[i].entity() != obj)
			{
				if (i != j)
				{
					std::swap(m_pickedPoints[i], m_pickedPoints[j]);
				}
				j++;
			}
		}
		assert(j != 0);
		m_pickedPoints.resize(j);
	}

	updateName();
}

void cc2DLabel::updateName()
{
	switch (m_pickedPoints.size())
	{
	case 0:
	{
		setName("Label");
	}
	break;

	case 1:
	{
		setName(m_pickedPoints[0].prefix(POINT_INDEX_0));
	}
	break;

	case 2:
	{
		if (m_pickedPoints[0].entity() == m_pickedPoints[1].entity())
		{
			setName(	QString("Vector ") + m_pickedPoints[0].prefix(POINT_INDEX_0)
					+	QString(" - ")     + m_pickedPoints[1].prefix(POINT_INDEX_1) );
		}
		else
		{
			setName(	QString("Vector ") + m_pickedPoints[0].prefix(POINT_INDEX_0) + QString("@") + ENTITY_INDEX_0
					+	QString(" - ")     + m_pickedPoints[1].prefix(POINT_INDEX_1) + QString("@") + ENTITY_INDEX_1 );
		}
	}
	break;

	case 3:
	{
		if (	m_pickedPoints[0].entity() == m_pickedPoints[2].entity() && m_pickedPoints[1].entity() == m_pickedPoints[2].entity() )
		{
			setName(	QString("Triplet ") + m_pickedPoints[0].prefix(POINT_INDEX_0)
					+	QString(" - ")      + m_pickedPoints[1].prefix(POINT_INDEX_1)
					+	QString(" - ")      + m_pickedPoints[2].prefix(POINT_INDEX_2) );
		}
		else
		{
			setName(	QString("Triplet ") + m_pickedPoints[0].prefix(POINT_INDEX_0) + QString("@") + ENTITY_INDEX_0
					+	QString(" - ")      + m_pickedPoints[1].prefix(POINT_INDEX_1) + QString("@") + ENTITY_INDEX_1
					+	QString(" - ")      + m_pickedPoints[2].prefix(POINT_INDEX_2) + QString("@") + ENTITY_INDEX_2 );
		}
	}
	break;

	}
}

bool cc2DLabel::addPickedPoint(ccGenericPointCloud* cloud, unsigned pointIndex, bool entityCenter/*=false*/)
{
	if (!cloud || pointIndex >= cloud->size())
		return false;

	PickedPoint pp;
	pp._cloud = cloud;
	pp.index = pointIndex;
	pp.entityCenterPoint = entityCenter;

	return addPickedPoint(pp);

	return true;
}

bool cc2DLabel::addPickedPoint(ccGenericMesh* mesh, unsigned triangleIndex, const CCVector2d& uv, bool entityCenter/*=false*/)
{
	if (!mesh || triangleIndex >= mesh->size())
		return false;

	PickedPoint pp;
	pp._mesh = mesh;
	pp.index = triangleIndex;
	pp.uv = uv;
	pp.entityCenterPoint = entityCenter;

	return addPickedPoint(pp);
}

bool cc2DLabel::addPickedPoint(const PickedPoint& pp)
{
	if (m_pickedPoints.size() == 3)
	{
		return false;
	}

	try
	{
		m_pickedPoints.resize(m_pickedPoints.size() + 1);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	m_pickedPoints.back() = pp;

	//we want to be notified whenever an associated mesh is deleted (in which case
	//we'll automatically clear the label)
	if (pp.entity())
		pp.entity()->addDependency(this, DP_NOTIFY_OTHER_ON_DELETE);
	//we must also warn the cloud or mesh whenever we delete this label
	//--> DGM: automatically done by the previous call to addDependency!

	updateName();

	return true;
}

bool cc2DLabel::toFile_MeOnly(QFile& out, short dataVersion) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));
	if (dataVersion < 50)
	{
		assert(false);
		return false;
	}

	if (!ccHObject::toFile_MeOnly(out, dataVersion))
		return false;

	//points count (dataVersion >= 20)
	uint32_t count = (uint32_t)m_pickedPoints.size();
	if (out.write((const char*)&count, 4) < 0)
		return WriteError();

	//points & associated cloud ID (dataVersion >= 20)
	for (std::vector<PickedPoint>::const_iterator it = m_pickedPoints.begin(); it != m_pickedPoints.end(); ++it)
	{
		//point index
		uint32_t index = static_cast<uint32_t>(it->index);
		if (out.write((const char*)&index, 4) < 0)
			return WriteError();
		//cloud ID (will be retrieved later --> make sure that the cloud is saved alongside!)
		uint32_t cloudID = static_cast<uint32_t>(it->_cloud ? it->_cloud->getUniqueID() : 0);
		if (out.write((const char*)&cloudID, 4) < 0)
			return WriteError();

		//mesh ID (dataVersion >= 49 - will be retrieved later --> make sure that the mesh is saved alongside!)
		uint32_t meshID = static_cast<uint32_t>(it->_mesh ? it->_mesh->getUniqueID() : 0);
		if (out.write((const char*)&meshID, 4) < 0)
			return WriteError();

		//uv coordinates in the triangle (dataVersion >= 49)
		if (out.write((const char*)it->uv.u, sizeof(double) * 2) < 0)
			return WriteError();

		//entity center point (dataVersion >= 50)
		if (out.write((const char*)&(it->entityCenterPoint), sizeof(bool)) < 0)
			return WriteError();
	}

	//Relative screen position (dataVersion >= 20)
	if (out.write((const char*)m_screenPos.data(), sizeof(float) * 2) < 0)
		return WriteError();

	//Collapsed state (dataVersion >= 20)
	if (out.write((const char*)&m_showFullBody, sizeof(bool)) < 0)
		return WriteError();

	//Show in 2D boolean (dataVersion >= 21)
	if (out.write((const char*)&m_dispIn2D, sizeof(bool)) < 0)
		return WriteError();

	//Show point(s) legend boolean (dataVersion >= 21)
	if (out.write((const char*)&m_dispPointsLegend, sizeof(bool)) < 0)
		return WriteError();

	return true;
}

bool cc2DLabel::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//points count (dataVersion >= 20)
	uint32_t count = 0;
	if (in.read((char*)&count, 4) < 0)
		return ReadError();

	//points & associated cloud/mesh ID (dataVersion >= 20)
	assert(m_pickedPoints.empty());
	for (uint32_t i = 0; i < count; ++i)
	{
		//point index
		uint32_t index = 0;
		if (in.read((char*)&index, 4) < 0)
			return ReadError();

		//cloud ID (will be retrieved later)
		{
			uint32_t cloudID = 0;
			if (in.read((char*)&cloudID, 4) < 0)
				return ReadError();

			if (cloudID != 0)
			{
				try
				{
					m_pickedPoints.resize(m_pickedPoints.size() + 1);
					m_pickedPoints.back().index = static_cast<unsigned>(index);
					//[DIRTY] WARNING: temporarily, we set the cloud unique ID in the 'PickedPoint::_cloud' pointer!!!
					*(uint32_t*)(&m_pickedPoints.back()._cloud) = cloudID;
				}
				catch (const std::bad_alloc)
				{
					return MemoryError();
				}
			}
		}

		if (dataVersion >= 49)
		{
			//mesh ID (dataVersion >= 49 - will be retrieved later)
			uint32_t meshID = 0;
			if (in.read((char*)&meshID, 4) < 0)
				return ReadError();

			//uv coordinates in the triangle (dataVersion >= 49)
			CCVector2d uv;
			if (in.read((char*)uv.u, sizeof(double) * 2) < 0)
				return ReadError();

			if (meshID != 0)
			{
				try
				{
					m_pickedPoints.resize(m_pickedPoints.size() + 1);
					m_pickedPoints.back().index = static_cast<unsigned>(index);
					m_pickedPoints.back().uv = uv;
					//[DIRTY] WARNING: temporarily, we set the mesh unique ID in the 'PickedPoint::_mesh' pointer!!!
					*(uint32_t*)(&m_pickedPoints.back()._mesh) = meshID;
				}
				catch (const std::bad_alloc)
				{
					return MemoryError();
				}
			}
		}

		//entity center point (dataVersion >= 50)
		bool entityCenterPoint = false;
		if (dataVersion >= 50)
		{
			if (in.read((char*)&entityCenterPoint, sizeof(bool)) < 0)
				return ReadError();
		}
		m_pickedPoints.back().entityCenterPoint = entityCenterPoint;
	}

	//Relative screen position (dataVersion >= 20)
	if (in.read((char*)m_screenPos.data(), sizeof(float) * 2) < 0)
		return ReadError();

	//Collapsed state (dataVersion >= 20)
	if (in.read((char*)&m_showFullBody, sizeof(bool)) < 0)
		return ReadError();

	if (dataVersion > 20)
	{
		//Show in 2D boolean (dataVersion >= 21)
		if (in.read((char*)&m_dispIn2D, sizeof(bool)) < 0)
			return ReadError();

		//Show point(s) legend boolean (dataVersion >= 21)
		if (in.read((char*)&m_dispPointsLegend, sizeof(bool)) < 0)
			return ReadError();
	}

	return true;
}

short cc2DLabel::minimumFileVersion_MeOnly() const
{
	return std::max(static_cast<short>(50), ccHObject::minimumFileVersion_MeOnly());
}

void AddPointCoordinates(QStringList& body, QString pointShortName, const CCVector3& P, const ccShiftedObject& shiftedObject, int precision)
{
	bool isShifted = shiftedObject.isShifted();

	QString coordStr = pointShortName;
	if (isShifted)
	{
		body << coordStr;
		coordStr = QString("  [shifted]");
	}

	coordStr += QString(" (%1;%2;%3)").arg(P.x, 0, 'f', precision).arg(P.y, 0, 'f', precision).arg(P.z, 0, 'f', precision);
	body << coordStr;

	if (isShifted)
	{
		CCVector3d Pg = shiftedObject.toGlobal3d(P);
		QString globCoordStr = QString("  [original] (%1;%2;%3)").arg(Pg.x, 0, 'f', precision).arg(Pg.y, 0, 'f', precision).arg(Pg.z, 0, 'f', precision);
		body << globCoordStr;
	}
}

void AddPointCoordinates(QStringList& body, const cc2DLabel::PickedPoint& pp, int precision, QString pointName = QString())
{
	QString pointShortName;
	ccShiftedObject* shiftedObject = nullptr;

	if (pp._cloud)
	{
		shiftedObject = pp._cloud;
		if (pp.entityCenterPoint)
			pointShortName = CENTER_STRING + QString("@%1").arg(pp._cloud->getUniqueID());
		else
			pointShortName = QString("P#%0").arg(pp.index);
	}
	else if (pp._mesh)
	{
		ccGenericPointCloud* vertices = pp._mesh->getAssociatedCloud();
		if (!vertices)
		{
			assert(false);
			return;
		}
		shiftedObject = vertices;
		if (pp.entityCenterPoint)
			pointShortName = CENTER_STRING + QString("@%1").arg(pp._mesh->getUniqueID());
		else
			pointShortName = QString("Tri#%0").arg(pp.index);
	}

	if (!pointName.isEmpty())
		pointShortName = QString("%1 (%2)").arg(pointName, pointShortName);

	assert(shiftedObject);
	AddPointCoordinates(body, pointShortName, pp.getPointPosition(), *shiftedObject, precision);
}

void cc2DLabel::getLabelInfo1(LabelInfo1& info) const
{
	info = LabelInfo1();

	if (m_pickedPoints.size() != 1)
		return;

	const PickedPoint& pp = m_pickedPoints[0];

	if (!pp.entityCenterPoint)
	{
		//cloud and point index
		if (pp._cloud)
		{
			//normal
			info.hasNormal = pp._cloud->hasNormals();
			if (info.hasNormal)
			{
				info.normal = pp._cloud->getPointNormal(pp.index);
			}
			//color
			info.hasRGB = pp._cloud->hasColors();
			if (info.hasRGB)
			{
				info.color = pp._cloud->getPointColor(pp.index);
			}
			//scalar field
			info.hasSF = pp._cloud->hasDisplayedScalarField();
			if (info.hasSF)
			{
				ccScalarField* sf = nullptr;

				//fetch the real scalar field if possible
				if (pp._cloud->isA(CC_TYPES::POINT_CLOUD))
				{
					sf = static_cast<ccPointCloud*>(pp._cloud)->getCurrentDisplayedScalarField();
				}

				if (sf)
				{
					info.sfValue = sf->getValue(pp.index);
					info.sfName = QString::fromStdString(sf->getName());
				}
				else
				{
					info.sfValue = pp._cloud->getPointScalarValue(pp.index);
					info.sfName = "Scalar";
				}
			}
		}
		else if (pp._mesh)
		{
			CCVector3d w(pp.uv, 1.0 - pp.uv.x - pp.uv.y);
			//normal
			info.hasNormal = pp._mesh->hasNormals();
			if (info.hasNormal)
			{
				pp._mesh->interpolateNormalsBC(pp.index, w, info.normal);
			}
			//color
			info.hasRGB = pp._mesh->hasColors();
			if (info.hasRGB)
			{
				pp._mesh->interpolateColorsBC(pp.index, w, info.color);
			}
			//scalar field
			info.hasSF = pp._mesh->hasDisplayedScalarField();
			if (info.hasSF)
			{
				CCCoreLib::VerticesIndexes* vi = pp._mesh->getTriangleVertIndexes(pp.index);
				assert(vi);

				//fetch the real scalar field name if possible
				ccGenericPointCloud* vertices = pp._mesh->getAssociatedCloud();
				assert(vertices);

				ccScalarField* sf = nullptr;

				//fetch the real scalar field if possible
				if (vertices->isA(CC_TYPES::POINT_CLOUD))
				{
					sf = static_cast<ccPointCloud*>(vertices)->getCurrentDisplayedScalarField();
				}

				ScalarType s1 = CCCoreLib::NAN_VALUE;
				ScalarType s2 = CCCoreLib::NAN_VALUE;
				ScalarType s3 = CCCoreLib::NAN_VALUE;

				if (sf)
				{
					s1 = sf->getValue(vi->i1);
					s2 = sf->getValue(vi->i2);
					s3 = sf->getValue(vi->i3);
				}
				else
				{
					s1 = vertices->getPointScalarValue(vi->i1);
					s2 = vertices->getPointScalarValue(vi->i2);
					s3 = vertices->getPointScalarValue(vi->i3);
				}

				//interpolate the SF value
				if (ccScalarField::ValidValue(s1) && ccScalarField::ValidValue(s2) && ccScalarField::ValidValue(s3))
				{
					info.sfValue = static_cast<ScalarType>(s1 * w.u[0] + s2 * w.u[1] + s3 * w.u[2]);
				}

				if (sf)
				{
					info.sfName = QString::fromStdString(sf->getName());
				}
				else
				{
					info.sfName = "Scalar";
				}
			}
		}
	}
}

void cc2DLabel::getLabelInfo2(LabelInfo2& info) const
{
	info = LabelInfo2();

	if (m_pickedPoints.size() != 2)
		return;

	//1st point
	CCVector3 P1 = m_pickedPoints[0].getPointPosition();
	//2nd point
	CCVector3 P2 = m_pickedPoints[1].getPointPosition();

	info.diff = P2 - P1;
}

void cc2DLabel::getLabelInfo3(LabelInfo3& info) const
{
	info = LabelInfo3();

	if (m_pickedPoints.size() != 3)
		return;

	//1st point
	CCVector3 P1 = m_pickedPoints[0].getPointPosition();
	//2nd point
	CCVector3 P2 = m_pickedPoints[1].getPointPosition();
	//3rd point
	CCVector3 P3 = m_pickedPoints[2].getPointPosition();

	//area
	CCVector3 P1P2 = P2 - P1;
	CCVector3 P1P3 = P3 - P1;
	CCVector3 P2P3 = P3 - P2;
	CCVector3 N = P1P2.cross(P1P3); //N = ABxAC
	info.area = N.norm() / 2;

	//normal
	N.normalize();
	info.normal = N;

	//edges length
	info.edges.u[0] = P1P2.normd();  //edge 1-2
	info.edges.u[1] = P2P3.normd();  //edge 2-3
	info.edges.u[2] = P1P3.normd();  //edge 3-1

	//angle
	info.angles.u[0] = CCCoreLib::RadiansToDegrees( P1P2.angle_rad( P1P3) ); //angleAtP1
	info.angles.u[1] = CCCoreLib::RadiansToDegrees( P2P3.angle_rad(-P1P2) ); //angleAtP2
	info.angles.u[2] = CCCoreLib::RadiansToDegrees( P1P3.angle_rad( P2P3) ); //angleAtP3 (should be equal to 180-a1-a2!)
}

QStringList cc2DLabel::getLabelContent(int precision) const
{
	QStringList body;

	switch (m_pickedPoints.size())
	{
	case 0:
		//can happen if the associated cloud(s) has(ve) been deleted!
		body << "Deprecated";
		break;

	case 1: //point
	{
		LabelInfo1 info;
		getLabelInfo1(info);

		//coordinates
		AddPointCoordinates(body, m_pickedPoints[0], precision);

		//normal
		if (info.hasNormal)
		{
			QString normStr = QString("Normal: (%1;%2;%3)").arg(info.normal.x, 0, 'f', precision).arg(info.normal.y, 0, 'f', precision).arg(info.normal.z, 0, 'f', precision);
			body << normStr;
		}
		//color
		if (info.hasRGB)
		{
			QString colorStr = QString("Color: (%1;%2;%3;%4)").arg(info.color.r).arg(info.color.g).arg(info.color.b).arg(info.color.a);
			body << colorStr;
		}
		//scalar field
		if (info.hasSF)
		{
			QString sfVal = GetSFValueAsString(info, precision);
			QString sfStr = QString("%1 = %2").arg(info.sfName, sfVal);
			body << sfStr;
		}
	}
	break;

	case 2: //vector
	{
		LabelInfo2 info;
		getLabelInfo2(info);

		//distance is now the default label title
		//PointCoordinateType dist = info.diff.norm();
		//QString distStr = QString("Distance = %1").arg(dist,0,'f',precision);
		//body << distStr;

		QString vecStr =	MathSymbolDelta + QString("X: %1\t").arg(info.diff.x, 0, 'f', precision)
						+	MathSymbolDelta + QString("Y: %1\t").arg(info.diff.y, 0, 'f', precision)
						+	MathSymbolDelta + QString("Z: %1"  ).arg(info.diff.z, 0, 'f', precision);

		body << vecStr;

		PointCoordinateType dXY = sqrt(info.diff.x*info.diff.x + info.diff.y*info.diff.y);
		PointCoordinateType dXZ = sqrt(info.diff.x*info.diff.x + info.diff.z*info.diff.z);
		PointCoordinateType dZY = sqrt(info.diff.z*info.diff.z + info.diff.y*info.diff.y);

		vecStr =	MathSymbolDelta + QString("XY: %1\t").arg(dXY, 0, 'f', precision)
				+	MathSymbolDelta + QString("XZ: %1\t").arg(dXZ, 0, 'f', precision)
				+	MathSymbolDelta + QString("ZY: %1"  ).arg(dZY, 0, 'f', precision);
		body << vecStr;

		AddPointCoordinates(body, m_pickedPoints[0], precision);
		AddPointCoordinates(body, m_pickedPoints[1], precision);
	}
	break;

	case 3: //triangle/plane
	{
		LabelInfo3 info;
		getLabelInfo3(info);

		//area
		QString areaStr = QString("Area = %1").arg(info.area, 0, 'f', precision);
		body << areaStr;

		//coordinates
		AddPointCoordinates(body, m_pickedPoints[0], precision, "A");
		AddPointCoordinates(body, m_pickedPoints[1], precision, "B");
		AddPointCoordinates(body, m_pickedPoints[2], precision, "C");

		//normal
		QString normStr = QString("Normal: (%1;%2;%3)").arg(info.normal.x, 0, 'f', precision).arg(info.normal.y, 0, 'f', precision).arg(info.normal.z, 0, 'f', precision);
		body << normStr;

		//angles
		QString angleStr = QString("Angles: A=%1 - B=%2 - C=%3 deg.")
			.arg(info.angles.u[0], 0, 'f', precision)
			.arg(info.angles.u[1], 0, 'f', precision)
			.arg(info.angles.u[2], 0, 'f', precision);
		body << angleStr;

		//edges
		QString edgesStr = QString("Edges: AB=%1 - BC=%2 - CA=%3")
			.arg(info.edges.u[0], 0, 'f', precision)
			.arg(info.edges.u[1], 0, 'f', precision)
			.arg(info.edges.u[2], 0, 'f', precision);
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
	if (m_pickedPoints.empty())
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
static QSharedPointer<ccSphere> c_unitPointMarker(nullptr);

void cc2DLabel::drawMeOnly3D(CC_DRAW_CONTEXT& context)
{
	size_t count = m_pickedPoints.size();
	if (count == 0)
	{
		return;
	}

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	if (glFunc == nullptr)
	{
		assert(false);
		return;
	}

	//color-based entity picking
	bool entityPickingMode = MACRO_EntityPicking(context);
	ccColor::Rgb pickingColor;
	if (entityPickingMode)
	{
		//not particularly fast
		if (MACRO_FastEntityPicking(context))
			return;
		pickingColor = context.entityPicking.registerEntity(this);
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
	if (!entityPickingMode)
	{
		for (size_t i = 0; i < count; i++)
		{
			//project the point in 2D
			CCVector3 P3D = m_pickedPoints[i].getPointPosition();
			camera.project(P3D, m_pickedPoints[i].pos2D);
		}
	}

	//bool loop = false;
	switch (count)
	{
	case 3:
	{
		//we draw the triangle
		if (entityPickingMode)
		{
			ccGL::Color(glFunc, pickingColor);
		}
		else
		{
			glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT);
			glFunc->glEnable(GL_BLEND);

			static ccColor::Rgba DefaultTriangleColor(255, 255, 0, 128);
			ccGL::Color(glFunc, DefaultTriangleColor);
		}
		glFunc->glBegin(GL_TRIANGLES);
		CCVector3 P3D = m_pickedPoints[0].getPointPosition();
		ccGL::Vertex3v(glFunc, P3D.u);
		P3D = m_pickedPoints[1].getPointPosition();
		ccGL::Vertex3v(glFunc, P3D.u);
		P3D = m_pickedPoints[2].getPointPosition();
		ccGL::Vertex3v(glFunc, P3D.u);
		glFunc->glEnd();

		if (!entityPickingMode)
		{
			glFunc->glPopAttrib(); //GL_COLOR_BUFFER_BIT
		}
		//loop = true;
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
		//	ccGL::Color(glFunc, ccColor::red);
		//else
		//	ccGL::Color(glFunc, ccColor::green);
		//
		//glFunc->glBegin(GL_LINES);
		//for (unsigned i=0; i<count; i++)
		//{
		//	if (i+1<count || loop)
		//	{
		//		CCVector3 P3D = m_pickedPoints[i].getPointPosition();
		//		ccGL::Vertex3v(glFunc, P3D.u);
		//		P3D = m_pickedPoints[(i+1)%count].getPointPosition();
		//		ccGL::Vertex3v(glFunc, P3D.u);
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
				c_unitPointMarker.reset(new ccSphere(1.0f, nullptr, "PointMarker", 12));
				c_unitPointMarker->showColors(true);
				c_unitPointMarker->setVisible(true);
				c_unitPointMarker->setEnabled(true);
			}

			//build-up point maker own 'context'
			CC_DRAW_CONTEXT markerContext = context;
			markerContext.drawingFlags &= (~CC_ENTITY_PICKING); //we must remove the 'entity picking flag' so that the sphere doesn't override the picking color!
			markerContext.display = nullptr;

			if (entityPickingMode)
				c_unitPointMarker->setTempColor(pickingColor);
			else if (isSelected())
				c_unitPointMarker->setTempColor(ccColor::red);
			else
				c_unitPointMarker->setTempColor(context.labelDefaultMarkerCol);

			const ccViewportParameters& viewportParams = context.display->getViewportParameters();
			for (size_t i = 0; i < count; i++)
			{
				glFunc->glMatrixMode(GL_MODELVIEW);
				glFunc->glPushMatrix();
				CCVector3 P = m_pickedPoints[i].getPointPosition();
				ccGL::Translate(glFunc, P.x, P.y, P.z);
				float scale = context.labelMarkerSize * m_relMarkerScale;
				if (viewportParams.perspectiveView && viewportParams.zFar > 0)
				{
					//in perspective view, the actual scale depends on the distance to the camera!
					double d = (camera.modelViewMat * P).norm();
					double unitD = viewportParams.zFar / 2; //we consider that the 'standard' scale is at half the depth
					scale = static_cast<float>(scale * sqrt(d / unitD)); //sqrt = empirical (probably because the marker size is already partly compensated by ccGLWindowInterface::computeActualPixelSize())
				}
				scale = static_cast<float>(scale * context.devicePixelRatio);
				glFunc->glScalef(scale, scale, scale);
				m_pickedPoints[i].markerScale = scale;
				c_unitPointMarker->draw(markerContext);
				glFunc->glPopMatrix();
			}
		}
	}
	}
}

//display parameters
static const int c_margin = 5;
static const int c_tabMarginX = 5;
static const int c_tabMarginY = 2;
static const int c_arrowBaseSize = 3;
//static const int c_buttonSize = 10;

static const ccColor::Rgba c_darkGreen(0, 200, 0, 255);

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
		if (colCount < maxBlockPerRow * 2)
		{
			colCount += 2;
			colContent.resize(colCount);
			colWidth.resize(colCount, 0);
		}
		int blockCol = (blockCount % maxBlockPerRow);
		//add new row
		if (blockCol == 0)
			rowCount += 3;
		++blockCount;

		//return the first column index of the block
		return blockCol * 2;
	}

	//! Updates columns width table
	/** \return the total width
	**/
	int updateColumnsWidthTable(const QFontMetrics& fm)
	{
		//compute min width of each column
		int totalWidth = 0;
		for (int i = 0; i < colCount; ++i)
		{
			int maxWidth = 0;
			for (int j = 0; j < colContent[i].size(); ++j)
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
	assert(!m_pickedPoints.empty());

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	if (glFunc == nullptr)
	{
		assert(false);
		return;
	}

	//color-based entity picking
	bool entityPickingMode = MACRO_EntityPicking(context);
	ccColor::Rgb pickingColor;
	if (entityPickingMode)
	{
		pickingColor = context.entityPicking.registerEntity(this);
	}

	float halfW = context.glW / 2.0f;
	float halfH = context.glH / 2.0f;

	size_t count = m_pickedPoints.size();
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
			if (m_pickedPoints[j].pos2D.z >= 0.0 && m_pickedPoints[j].pos2D.z <= 1.0)
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
				if (entityPickingMode)
					ccGL::Color(glFunc, pickingColor);
				else if (isSelected())
					ccGL::Color(glFunc, ccColor::red);
				else
					ccGL::Color(glFunc, context.labelDefaultMarkerCol/*ccColor::green*/);

				glFunc->glBegin(count == 2 ? GL_LINES : GL_LINE_LOOP);
				for (unsigned j = 0; j < count; ++j)
				{
					glFunc->glVertex2d(m_pickedPoints[j].pos2D.x - halfW, m_pickedPoints[j].pos2D.y - halfH);
				}
				glFunc->glEnd();
				glFunc->glPopAttrib(); //GL_LINE_BIT
			}

			//no need to display the point(s) legend in picking mode
			if (m_dispPointsLegend && !entityPickingMode)
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

					context.display->displayText(	title,
													static_cast<int>(m_pickedPoints[j].pos2D.x) + context.labelMarkerTextShift_pix,
													static_cast<int>(m_pickedPoints[j].pos2D.y) + context.labelMarkerTextShift_pix,
													ccGenericGLDisplay::ALIGN_DEFAULT,
													context.labelOpacity / 100.0f,
													&ccColor::white,
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
	QFont bodyFont;
	QFont titleFont;
	if (!entityPickingMode)
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

						ccGenericPointCloud* cloud = m_pickedPoints[0].cloudOrVertices();
						assert(cloud);
						bool isShifted = cloud->isShifted();
						CCVector3 P = m_pickedPoints[0].getPointPosition();
						//1st block: X, Y, Z (local)
						{
							int c = tab.add2x3Block();
							QChar suffix;
							if (isShifted)
							{
								suffix = 'l'; //'l' for local
							}
							tab.colContent[c] << QString("X") + suffix; tab.colContent[c + 1] << QString::number(P.x, 'f', precision);
							tab.colContent[c] << QString("Y") + suffix; tab.colContent[c + 1] << QString::number(P.y, 'f', precision);
							tab.colContent[c] << QString("Z") + suffix; tab.colContent[c + 1] << QString::number(P.z, 'f', precision);
						}
						//next block:  X, Y, Z (global)
						if (isShifted)
						{
							int c = tab.add2x3Block();
							CCVector3d Pd = cloud->toGlobal3d(P);
							tab.colContent[c] << "Xg"; tab.colContent[c + 1] << QString::number(Pd.x, 'f', precision);
							tab.colContent[c] << "Yg"; tab.colContent[c + 1] << QString::number(Pd.y, 'f', precision);
							tab.colContent[c] << "Zg"; tab.colContent[c + 1] << QString::number(Pd.z, 'f', precision);
						}
						//next block: normal
						if (info.hasNormal)
						{
							int c = tab.add2x3Block();
							tab.colContent[c] << "Nx"; tab.colContent[c + 1] << QString::number(info.normal.x, 'f', precision);
							tab.colContent[c] << "Ny"; tab.colContent[c + 1] << QString::number(info.normal.y, 'f', precision);
							tab.colContent[c] << "Nz"; tab.colContent[c + 1] << QString::number(info.normal.z, 'f', precision);
						}

						//next block: RGB color
						if (info.hasRGB)
						{
							int c = tab.add2x3Block();
							tab.colContent[c] << "R"; tab.colContent[c + 1] << QString::number(info.color.r);
							tab.colContent[c] << "G"; tab.colContent[c + 1] << QString::number(info.color.g);
							tab.colContent[c] << "B"; tab.colContent[c + 1] << QString::number(info.color.b);
						}
					}
					else if (count == 2)
					{
						LabelInfo2 info;
						getLabelInfo2(info);

						//1st block: dX, dY, dZ
						{
							int c = tab.add2x3Block();
							tab.colContent[c] << MathSymbolDelta + QString("X"); tab.colContent[c + 1] << QString::number(info.diff.x, 'f', precision);
							tab.colContent[c] << MathSymbolDelta + QString("Y"); tab.colContent[c + 1] << QString::number(info.diff.y, 'f', precision);
							tab.colContent[c] << MathSymbolDelta + QString("Z"); tab.colContent[c + 1] << QString::number(info.diff.z, 'f', precision);
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
							tab.colContent[c] << "index.A"; tab.colContent[c + 1] << (m_pickedPoints[0].itemTitle());
							tab.colContent[c] << "index.B"; tab.colContent[c + 1] << (m_pickedPoints[1].itemTitle());
							tab.colContent[c] << "index.C"; tab.colContent[c + 1] << (m_pickedPoints[2].itemTitle());
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
					assert(!entityPickingMode);
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
					for (int j = 0; j < body.size(); ++j)
					{
						dx = std::max(dx, bodyFontMetrics.width(body[j]));
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
	}

	//draw label rectangle
	const int xStart = static_cast<int>(context.glW * m_screenPos[0]);
	const int yStart = static_cast<int>(context.glH * (1.0f - m_screenPos[1]));

	m_lastScreenPos[0] = xStart;
	m_lastScreenPos[1] = yStart - m_labelROI.height();

	//colors
	ccColor::Rgbaub defaultBkgColor(pickingColor, 255);
	ccColor::Rgbaub defaultBorderColor(pickingColor, 255);
	if (!entityPickingMode)
	{
		//default background color
		unsigned char alpha = static_cast<unsigned char>((context.labelOpacity / 100.0) * 255);
		defaultBkgColor = ccColor::Rgbaub(context.labelDefaultBkgCol, alpha);
		if (isSelected())
		{
			//default border color (mustn't be totally transparent!)
			defaultBorderColor = ccColor::Rgbaub(ccColor::red, 255);
		}
		else
		{
			//apply only half of the transparency
			unsigned char halfAlpha = static_cast<unsigned char>((50.0 + context.labelOpacity / 200.0) * 255);
			defaultBorderColor = ccColor::Rgbaub(context.labelDefaultBkgCol, halfAlpha);
		}

		glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT);
		glFunc->glEnable(GL_BLEND);
	}

	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();
	glFunc->glTranslatef(static_cast<GLfloat>(xStart - halfW), static_cast<GLfloat>(yStart - halfH), 0);

	//display the arrow from the 2D rectangle to the 3D point
	if (!entityPickingMode)
	{
		//compute arrow base position relatively to the label rectangle (for 0 to 8)
		int arrowBaseConfig = 0;

		//compute arrow head position
		CCVector3d arrowDest2D(0, 0, 0);
		for (size_t i = 0; i < count; ++i)
		{
			arrowDest2D += m_pickedPoints[i].pos2D;
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
			ccGL::Color(glFunc, defaultBorderColor);
			glFunc->glBegin(GL_TRIANGLE_FAN);
			glFunc->glVertex2i(iArrowDestX, iArrowDestY);
			switch (arrowBaseConfig)
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
	ccGL::Color(glFunc, defaultBkgColor);
	glFunc->glBegin(GL_QUADS);
	glFunc->glVertex2i(m_labelROI.left(), -m_labelROI.top());
	glFunc->glVertex2i(m_labelROI.left(), -m_labelROI.bottom());
	glFunc->glVertex2i(m_labelROI.right(), -m_labelROI.bottom());
	glFunc->glVertex2i(m_labelROI.right(), -m_labelROI.top());
	glFunc->glEnd();

	//border
	{
		glFunc->glPushAttrib(GL_LINE_BIT);
		glFunc->glLineWidth(3.0f * context.renderZoom);
		ccGL::Color(glFunc, defaultBorderColor);
		glFunc->glBegin(GL_LINE_LOOP);
		glFunc->glVertex2i(m_labelROI.left(), -m_labelROI.top());
		glFunc->glVertex2i(m_labelROI.left(), -m_labelROI.bottom());
		glFunc->glVertex2i(m_labelROI.right(), -m_labelROI.bottom());
		glFunc->glVertex2i(m_labelROI.right(), -m_labelROI.top());
		glFunc->glEnd();
		glFunc->glPopAttrib(); //GL_LINE_BIT
	}

	//display text
	if (!entityPickingMode)
	{
		int xStartRel = margin;
		int yStartRel = 0;
		yStartRel -= titleHeight;

		ccColor::Rgba defaultTextColor;
		if (context.labelOpacity < 40)
		{
			//under a given opacity level, we use the default text color instead!
			defaultTextColor = context.textDefaultCol;
		}
		else
		{
			defaultTextColor = ccColor::Rgba(	255 - context.labelDefaultBkgCol.r,
												255 - context.labelDefaultBkgCol.g,
												255 - context.labelDefaultBkgCol.b,
												context.labelDefaultBkgCol.a);
		}

		//label title
		context.display->displayText(title,
			xStart + xStartRel,
			yStart + yStartRel,
			ccGenericGLDisplay::ALIGN_DEFAULT,
			0,
			&defaultTextColor,
			&titleFont);
		yStartRel -= margin;

		if (m_showFullBody)
		{
#ifdef DRAW_CONTENT_AS_TAB
			int xCol = xStartRel;
			for (int c = 0; c < tab.colCount; ++c)
			{
				int width = tab.colWidth[c] + 2 * tabMarginX;
				int height = rowHeight + 2 * tabMarginY;

				int yRow = yStartRel;
				int actualRowCount = std::min(tab.rowCount, tab.colContent[c].size());

				bool labelCol = ((c & 1) == 0);
				const ccColor::Rgba* textColor = labelCol ? &ccColor::white : &defaultTextColor;

				for (int r = 0; r < actualRowCount; ++r)
				{
					if (r && (r % 3) == 0)
						yRow -= margin;

					if (labelCol)
					{
						//draw background
						int rgbIndex = (r % 3);
						if (rgbIndex == 0)
							ccGL::Color(glFunc, ccColor::red);
						else if (rgbIndex == 1)
							ccGL::Color(glFunc, c_darkGreen);
						else if (rgbIndex == 2)
							ccGL::Color(glFunc, ccColor::blue);

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

					context.display->displayText(str,
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
				for (int i = 0; i < body.size(); ++i)
				{
					yStartRel -= rowHeight;
					context.display->displayText(body[i], xStart + xStartRel, yStart + yStartRel, ccGenericGLDisplay::ALIGN_DEFAULT, 0, defaultTextColor.rgb, &bodyFont);
				}
			}
#endif //DRAW_CONTENT_AS_TAB
		}
	}

	if (!entityPickingMode)
	{
		glFunc->glPopAttrib(); //GL_COLOR_BUFFER_BIT
	}

	glFunc->glPopMatrix();
}

bool cc2DLabel::pointPicking(	const CCVector2d& clickPos,
								const ccGLCameraParameters& camera,
								int& nearestPointIndex,
								double& nearestSquareDist) const
{
	nearestPointIndex = -1;
	nearestSquareDist = -1.0;
	{
		//back project the clicked point in 3D
		CCVector3d clickPosd(clickPos.x, clickPos.y, 0.0);
		CCVector3d X(0, 0, 0);
		if (!camera.unproject(clickPosd, X))
		{
			return false;
		}

		clickPosd.z = 1.0;
		CCVector3d Y(0, 0, 0);
		if (!camera.unproject(clickPosd, Y))
		{
			return false;
		}

		CCVector3d xy = (Y - X);
		xy .normalize();

		for (unsigned i = 0; i < size(); ++i)
		{
			const PickedPoint& pp = getPickedPoint(i);
			if (pp.markerScale == 0)
			{
				//never displayed
				continue;
			}

			const CCVector3 P = pp.getPointPosition();

			//warning: we have to handle the relative GL transformation!
			ccGLMatrix trans;
			bool noGLTrans = pp.entity() ? !pp.entity()->getAbsoluteGLTransformation(trans) : true;

			CCVector3d Q2D;
			bool insideFrustum = false;
			if (noGLTrans)
			{
				camera.project(P, Q2D, &insideFrustum);
			}
			else
			{
				CCVector3 P3D = P;
				trans.apply(P3D);
				camera.project(P3D, Q2D, &insideFrustum);
			}

			if (!insideFrustum)
			{
				continue;
			}

			// closest distance to XY
			CCVector3d XP = (P.toDouble() - X);
			double squareDist = (XP - XP.dot(xy) * xy).norm2();

			if (squareDist <= static_cast<double>(pp.markerScale) * pp.markerScale)
			{
				if (nearestPointIndex < 0 || squareDist < nearestSquareDist)
				{
					nearestSquareDist = squareDist;
					nearestPointIndex = i;
				}
			}
		}
	}

	return (nearestPointIndex >= 0);
}
