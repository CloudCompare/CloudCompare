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
//#                   COPYRIGHT: CloudCompare project                      #
//#                                                                        #
//##########################################################################

#include "ccCircle.h"

// qCC_db
#include <ccPointCloud.h>

ccCircle::ccCircle(	double radius/*=0.0*/,
					unsigned resolution/*=48*/,
					unsigned uniqueID/*=ccUniqueIDGenerator::InvalidUniqueID*/)
	: ccPolyline(new ccPointCloud("vertices"), uniqueID)
	, m_radius(std::max(0.0, radius))
	, m_resolution(std::max(resolution, 4u))
{
	if (radius > 0.0)
	{
		updateInternalRepresentation();
	}

	ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
	if (vertices)
	{
		vertices->setEnabled(false);
		addChild(vertices);
	}
	else
	{
		assert(false);
	}

	setName("Circle");
}

ccCircle::ccCircle(const ccCircle& circle)
	: ccCircle(circle.m_radius, circle.m_resolution)
{
	updateInternalRepresentation();
}

ccCircle* ccCircle::clone() const
{
	ccCircle* clonedCircle = new ccCircle(*this);
	clonedCircle->setLocked(false); //there's no reason to keep the clone locked

	return clonedCircle;
}

void ccCircle::applyGLTransformation(const ccGLMatrix& trans)
{
	//we call the ccHObject method instead of the ccPolyline one,
	//to only update the transformation history matrix, and not
	//trigger any coordinate modification
	ccHObject::applyGLTransformation(trans);

	//now we can update the vertices
	updateInternalRepresentation();

	//invalidate the bounding-box
	invalidateBoundingBox();
}

void ccCircle::setRadius(double radius)
{
	if (m_radius != radius)
	{
		m_radius = radius;
		updateInternalRepresentation();
	}
}

void ccCircle::setResolution(unsigned resolution)
{
	if (m_resolution != resolution)
	{
		m_resolution = resolution;
		updateInternalRepresentation();
	}
}

bool ccCircle::toFile_MeOnly(QFile& out, short dataVersion) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));
	if (dataVersion < 56)
	{
		assert(false);
		return false;
	}

	if (!ccPolyline::toFile_MeOnly(out, dataVersion))
	{
		return false;
	}

	QDataStream outStream(&out);

	//Radius (dataVersion>=56)
	outStream << m_radius;
	//Resolution (dataVersion>=56)
	outStream << m_resolution;

	return true;
}

bool ccCircle::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	ccLog::PrintVerbose(QString("Loading polyline %1...").arg(m_name));

	ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
	if (vertices)
	{
		removeChild(vertices);
		m_theAssociatedCloud = nullptr;
	}

	if (!ccPolyline::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
	{
		return false;
	}

	if (dataVersion < 56)
	{
		return false;
	}

	QDataStream inStream(&in);

	//Radius (dataVersion>=56)
	inStream >> m_radius;
	//Resolution (dataVersion>=56)
	inStream >> m_resolution;

	return true;
}

short ccCircle::minimumFileVersion_MeOnly() const
{
	short minVersion = 56;
	return std::max(minVersion, ccHObject::minimumFileVersion_MeOnly());
}

void ccCircle::updateInternalRepresentation()
{
	if (m_resolution < 4)
	{
		ccLog::Warning("[ccCircle::updateInternalRepresentation] Resolution is too small");
		return;
	}

	ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
	if (!vertices)
	{
		ccLog::Warning("Invalid vertices");
		return;
	}

	vertices->clear();
	resize(0); // should never fail

	// reset the vertices transformation history
	ccGLMatrix Id;
	Id.toIdentity();
	vertices->setGLTransformationHistory(Id);

	if (!vertices->reserve(m_resolution))
	{
		ccLog::Warning("[ccCircle::updateInternalRepresentation] Not enough memory");
		return;
	}

	double angleStep_rad = (2 * M_PI) / m_resolution;
	for (unsigned i = 0; i < m_resolution; ++i)
	{
		CCVector3 P = CCVector3::fromArray(CCVector3d(cos(i * angleStep_rad) * m_radius, sin(i * angleStep_rad) * m_radius, 0).u);
		vertices->addPoint(P);
	}

	addPointIndex(0, m_resolution);
	setClosed(true);

	vertices->applyGLTransformation_recursive(&m_glTransHistory);
}
