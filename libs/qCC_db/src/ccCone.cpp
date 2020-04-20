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

#include "ccCone.h"

//Local
#include "ccPointCloud.h"
#include "ccNormalVectors.h"

ccCone::ccCone(PointCoordinateType bottomRadius,
				PointCoordinateType topRadius,
				PointCoordinateType height,
				PointCoordinateType xOff/*=0*/,
				PointCoordinateType yOff/*=0*/,
				const ccGLMatrix* transMat/*=0*/,
				QString name/*="Cylinder"*/,
				unsigned precision/*=DEFAULT_DRAWING_PRECISION*/,
				unsigned uniqueID/*=ccUniqueIDGenerator::InvalidUniqueID*/)

	: ccGenericPrimitive(name, transMat, uniqueID)
	, m_bottomRadius(fabs(bottomRadius))
	, m_topRadius(fabs(topRadius))
	, m_xOff(xOff)
	, m_yOff(yOff)
	, m_height(fabs(height))
{
	setDrawingPrecision(std::max<unsigned>(precision, MIN_DRAWING_PRECISION)); //automatically calls buildUp & applyTransformationToVertices
}

ccCone::ccCone(QString name/*="Cylinder"*/)
	: ccGenericPrimitive(name)
	, m_bottomRadius(0)
	, m_topRadius(0)
	, m_xOff(0)
	, m_yOff(0)
	, m_height(0)
{
}

ccGenericPrimitive* ccCone::clone() const
{
	return finishCloneJob(new ccCone(m_bottomRadius, m_topRadius, m_height, m_xOff, m_yOff, &m_transformation, getName(), m_drawPrecision));
}

bool ccCone::buildUp()
{
	if (m_drawPrecision < MIN_DRAWING_PRECISION)
		return false;

	//invalid dimensions?
	if (m_height < ZERO_TOLERANCE || m_bottomRadius + m_topRadius < ZERO_TOLERANCE)
		return false;

	//topology
	bool singlePointBottom = (m_bottomRadius < ZERO_TOLERANCE);
	bool singlePointTop = (m_topRadius < ZERO_TOLERANCE);
	assert(!singlePointBottom || !singlePointTop);

	unsigned steps = m_drawPrecision;

	//vertices
	unsigned vertCount = 2;
	if (!singlePointBottom)
		vertCount += steps;
	if (!singlePointTop)
		vertCount += steps;
	//normals
	unsigned faceNormCounts = steps + 2;
	//vertices
	unsigned facesCount = steps;
	if (!singlePointBottom)
		facesCount += steps;
	if (!singlePointTop)
		facesCount += steps;
	if (!singlePointBottom && !singlePointTop)
		facesCount += steps;

	//allocate (& clear) structures
	if (!init(vertCount, false, facesCount, faceNormCounts))
	{
		ccLog::Error("[ccCone::buildUp] Not enough memory");
		return false;
	}

	ccPointCloud* verts = vertices();
	assert(verts);
	assert(m_triNormals);

	//2 first points: centers of the top & bottom surfaces
	CCVector3 bottomCenter = CCVector3(m_xOff, m_yOff, -m_height) / 2;
	CCVector3 topCenter = CCVector3(-m_xOff, -m_yOff, m_height) / 2;
	{
		//bottom center
		verts->addPoint(bottomCenter);
		CompressedNormType nIndex = ccNormalVectors::GetNormIndex(CCVector3(0, 0, -1).u);
		m_triNormals->addElement(nIndex);
		//top center
		verts->addPoint(topCenter);
		nIndex = ccNormalVectors::GetNormIndex(CCVector3(0, 0, 1).u);
		m_triNormals->addElement(nIndex);
	}

	//then, angular sweep for top and/or bottom surfaces
	{
		PointCoordinateType angle_rad_step = static_cast<PointCoordinateType>(2.0*M_PI) / static_cast<PointCoordinateType>(steps);
		//bottom surface
		if (!singlePointBottom)
		{
			for (unsigned i = 0; i < steps; ++i)
			{
				CCVector3 P(bottomCenter.x + cos(angle_rad_step*i)*m_bottomRadius,
							bottomCenter.y + sin(angle_rad_step*i)*m_bottomRadius,
							bottomCenter.z);
				verts->addPoint(P);
			}
		}
		//top surface
		if (!singlePointTop)
		{
			for (unsigned i = 0; i < steps; ++i)
			{
				CCVector3 P(topCenter.x + cos(angle_rad_step*i)*m_topRadius,
							topCenter.y + sin(angle_rad_step*i)*m_topRadius,
							topCenter.z);
				verts->addPoint(P);
			}
		}
		//side normals
		{
			for (unsigned i = 0; i < steps; ++i)
			{
				//slope
				CCVector3 u(-sin(angle_rad_step*i), cos(angle_rad_step*i), 0);
				CCVector3 v(bottomCenter.x-topCenter.x + u.y*(m_bottomRadius-m_topRadius),
							bottomCenter.y-topCenter.y - u.x*(m_bottomRadius-m_topRadius),
							bottomCenter.z-topCenter.z);
				CCVector3 N = v.cross(u);
				N.normalize();

				CompressedNormType nIndex = ccNormalVectors::GetNormIndex(N.u);
				m_triNormals->addElement(nIndex);
			}
		}
	}
	
	//mesh faces
	{
		assert(m_triVertIndexes);

		unsigned bottomIndex = 2;
		unsigned topIndex = 2 + (singlePointBottom ? 0 : steps);

		//bottom surface
		if (!singlePointBottom)
		{
			for (unsigned i = 0; i < steps; ++i)
			{
				addTriangle(0, bottomIndex + (i + 1) % steps, bottomIndex + i);
				addTriangleNormalIndexes(0, 0, 0);
			}
		}
		//top surface
		if (!singlePointTop)
		{
			for (unsigned i = 0; i < steps; ++i)
			{
				addTriangle(1, topIndex + i, topIndex + (i + 1) % steps);
				addTriangleNormalIndexes(1, 1, 1);
			}
		}

		if (!singlePointBottom && !singlePointTop)
		{
			for (unsigned i = 0; i < steps; ++i)
			{
				unsigned iNext = (i + 1) % steps;
				addTriangle(bottomIndex + i, bottomIndex + iNext, topIndex + i);
				addTriangleNormalIndexes(2 + i, 2 + iNext, 2 + i);
				addTriangle(topIndex + i, bottomIndex + iNext, topIndex + iNext);
				addTriangleNormalIndexes(2 + i, 2 + iNext, 2 + iNext);
			}
		}
		else if (!singlePointTop)
		{
			for (unsigned i = 0; i < steps; ++i)
			{
				unsigned iNext = (i + 1) % steps;
				addTriangle(topIndex + i, 0, topIndex + iNext);
				addTriangleNormalIndexes(2 + i, 2 + iNext, 2 + iNext); //TODO: middle normal should be halfbetween?!
			}
		}
		else //if (!singlePointBottom)
		{
			for (unsigned i = 0; i < steps; ++i)
			{
				unsigned iNext = (i + 1) % steps;
				addTriangle(bottomIndex + i, bottomIndex + iNext, 1);
				addTriangleNormalIndexes(2 + i, 2 + iNext, 2 + i); //TODO: last normal should be halfbetween?!
			}
		}
	}

	notifyGeometryUpdate();
	showTriNorms(true);

	return true;
}

void ccCone::setHeight(PointCoordinateType height)
{
	if (m_height == height)
		return;

	assert(height > 0);
	m_height = height;
	
	buildUp();
	applyTransformationToVertices();
}

void ccCone::setBottomRadius(PointCoordinateType radius)
{
	if (m_bottomRadius == radius)
		return;

	assert(radius > 0);
	m_bottomRadius = radius;
	
	buildUp();
	applyTransformationToVertices();
}

void ccCone::setTopRadius(PointCoordinateType radius)
{
	if (m_topRadius == radius)
		return;

	assert(radius > 0);
	m_topRadius = radius;
	
	buildUp();
	applyTransformationToVertices();
}

CCVector3 ccCone::getBottomCenter() const
{
	CCVector3 bottomCenter = CCVector3(m_xOff, m_yOff, -m_height) / 2;
	ccGLMatrix trans = getGLTransformationHistory();
	trans.apply(bottomCenter);
	return bottomCenter;
}
CCVector3 ccCone::getTopCenter() const
{
	CCVector3 topCenter = CCVector3(-m_xOff, -m_yOff, m_height) / 2;
	ccGLMatrix trans = getGLTransformationHistory();
	trans.apply(topCenter);
	return topCenter;
}

CCVector3 ccCone::getSmallCenter() const
{
	if (m_topRadius <= m_bottomRadius)
	{
		return getTopCenter();
	}
	return getBottomCenter();
}

CCVector3 ccCone::getLargeCenter() const
{
	if (m_topRadius >= m_bottomRadius)
	{
		return getTopCenter();
	}
	return getBottomCenter();
}

PointCoordinateType ccCone::getSmallRadius() const
{
	if (m_topRadius <= m_bottomRadius)
	{
		return m_topRadius;
	}
	return m_bottomRadius;
}

PointCoordinateType ccCone::getLargeRadius() const
{
	if (m_topRadius >= m_bottomRadius)
	{
		return m_topRadius;
	}
	return m_bottomRadius;
}

bool ccCone::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericPrimitive::toFile_MeOnly(out))
		return false;

	//parameters (dataVersion>=21)
	QDataStream outStream(&out);
	outStream << m_bottomRadius;
	outStream << m_topRadius;
	outStream << m_xOff;
	outStream << m_yOff;
	outStream << m_height;

	return true;
}

bool ccCone::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//parameters (dataVersion>=21)
	QDataStream inStream(&in);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_bottomRadius);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_topRadius);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_xOff);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_yOff);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_height);

	return true;
}
