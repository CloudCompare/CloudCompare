//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccGenericPointCloud.h"

//CCLib
#include <Neighbourhood.h>
#include <DistanceComputationTools.h>

#include "ccOctree.h"
#include "ccSensor.h"

ccGenericPointCloud::ccGenericPointCloud(QString name)
	: ccShiftedObject(name)
	, m_pointsVisibility(0)
	, m_pointSize(0)
{
	setVisible(true);
	lockVisibility(false);
}

ccGenericPointCloud::ccGenericPointCloud(const ccGenericPointCloud& cloud)
	: ccShiftedObject(cloud)
	, m_pointsVisibility(cloud.m_pointsVisibility)
	, m_pointSize(cloud.m_pointSize)
{
}

ccGenericPointCloud::~ccGenericPointCloud()
{
	clear();
}

void ccGenericPointCloud::clear()
{
	unallocateVisibilityArray();
	deleteOctree();
	enableTempColor(false);
}

bool ccGenericPointCloud::resetVisibilityArray()
{
	if (!m_pointsVisibility)
	{
		m_pointsVisibility = new VisibilityTableType();
		m_pointsVisibility->link();
	}

	if (!m_pointsVisibility->resize(size()))
	{
		unallocateVisibilityArray();
		return false;
	}

	m_pointsVisibility->fill(POINT_VISIBLE); //by default, all points are visible

	return true;
}

void ccGenericPointCloud::unallocateVisibilityArray()
{
	if (m_pointsVisibility)
	{
		m_pointsVisibility->release();
		m_pointsVisibility = 0;
	}
}

bool ccGenericPointCloud::isVisibilityTableInstantiated() const
{
	return m_pointsVisibility && m_pointsVisibility->isAllocated();
}

unsigned char ccGenericPointCloud::testVisibility(const CCVector3& P) const
{
	unsigned char bestVisibility = 255; //impossible value

	for (ccHObject::Container::const_iterator it = m_children.begin(); it != m_children.end(); ++it)
	{
		if ((*it)->isKindOf(CC_TYPES::SENSOR))
		{
			unsigned char visibility = static_cast<ccSensor*>(*it)->checkVisibility(P);

			if (visibility == POINT_VISIBLE)
			{
				return POINT_VISIBLE; //shortcut
			}

			bestVisibility = std::min<unsigned char>(visibility,bestVisibility);
		}
	}

	return (bestVisibility == 255 ? POINT_VISIBLE : bestVisibility);
}

void ccGenericPointCloud::deleteOctree()
{
	ccOctree* oct = getOctree();
	if (oct)
		removeChild(oct);
}

ccOctree* ccGenericPointCloud::getOctree()
{
	for (size_t i=0; i<m_children.size(); ++i)
	{
		if (m_children[i]->isA(CC_TYPES::POINT_OCTREE))
			return static_cast<ccOctree*>(m_children[i]);
	}

	return NULL;
}

ccOctree* ccGenericPointCloud::computeOctree(CCLib::GenericProgressCallback* progressCb, bool autoAddChild/*=true*/)
{
	deleteOctree();
	ccOctree* octree = new ccOctree(this);
	if (octree->build(progressCb) > 0)
	{
		octree->setDisplay(getDisplay());
		octree->setVisible(true);
		octree->setEnabled(false);
		if (autoAddChild)
		{
			addChild(octree);
		}
	}
	else
	{
		delete octree;
		octree = NULL;
	}

	return octree;
}

CCLib::ReferenceCloud* ccGenericPointCloud::getTheVisiblePoints() const
{
	unsigned count = size();
	assert(count == m_pointsVisibility->currentSize());

	if (!m_pointsVisibility || m_pointsVisibility->currentSize() != count)
	{
		ccLog::Warning("[ccGenericPointCloud::getTheVisiblePoints] No visibility table instantiated!");
		return 0;
	}

	//count the number of points to copy
	unsigned pointCount = 0;
	{
		for (unsigned i=0; i<count; ++i)
			if (m_pointsVisibility->getValue(i) == POINT_VISIBLE)
				++pointCount;
	}

	if (pointCount == 0)
	{
		ccLog::Warning("[ccGenericPointCloud::getTheVisiblePoints] No point in selection");
		return 0;
	}

	//we create an entity with the 'visible' vertices only
	CCLib::ReferenceCloud* rc = new CCLib::ReferenceCloud(const_cast<ccGenericPointCloud*>(this));
	if (rc->reserve(pointCount))
	{
		for (unsigned i=0; i<count; ++i)
			if (m_pointsVisibility->getValue(i) == POINT_VISIBLE)
				rc->addPointIndex(i); //can't fail (see above)
	}
	else
	{
		delete rc;
		rc = 0;
		ccLog::Error("[ccGenericPointCloud::getTheVisiblePoints] Not enough memory!");
	}

	return rc;
}

ccBBox ccGenericPointCloud::getOwnBB(bool withGLFeatures/*=false*/)
{
	ccBBox box;

	if (size())
	{
		getBoundingBox(box.minCorner(), box.maxCorner());
		box.setValidity(true);
	}
	
	return box;
}

bool ccGenericPointCloud::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//'global shift & scale' (dataVersion>=39)
	saveShiftInfoToFile(out);
	
	//'visibility' array (dataVersion>=20)
	bool hasVisibilityArray = isVisibilityTableInstantiated();
	if (out.write((const char*)&hasVisibilityArray,sizeof(bool)) < 0)
		return WriteError();
	if (hasVisibilityArray)
	{
		assert(m_pointsVisibility);
		if (!ccSerializationHelper::GenericArrayToFile(*m_pointsVisibility,out))
			return false;
	}

	//'point size' (dataVersion>=24)
	if (out.write((const char*)&m_pointSize,1) < 0)
		return WriteError();

	return true;
}

bool ccGenericPointCloud::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags))
		return false;

	if (dataVersion < 20)
		return CorruptError();

	if (dataVersion < 33)
	{
		//'coordinates shift' (dataVersion>=20)
		if (in.read((char*)m_globalShift.u,sizeof(double)*3) < 0)
			return ReadError();

		m_globalScale = 1.0;
	}
	else
	{
		//'global shift & scale' (dataVersion>=33)
		if (!loadShiftInfoFromFile(in))
			return ReadError();
	}

	//'visibility' array (dataVersion>=20)
	bool hasVisibilityArray = false;
	if (in.read((char*)&hasVisibilityArray,sizeof(bool)) < 0)
		return ReadError();
	if (hasVisibilityArray)
	{
		if (!m_pointsVisibility)
		{
			m_pointsVisibility = new VisibilityTableType();
			m_pointsVisibility->link();
		}
		if (!ccSerializationHelper::GenericArrayFromFile(*m_pointsVisibility,in,dataVersion))
		{
			unallocateVisibilityArray();
			return false;
		}
	}

	//'point size' (dataVersion>=24)
	if (dataVersion >= 24)
	{
		if (in.read((char*)&m_pointSize,1) < 0)
			return WriteError();
	}
	else
	{
		m_pointSize = 0; //= follows default setting
	}

	return true;
}

void ccGenericPointCloud::importParametersFrom(const ccGenericPointCloud* cloud)
{
	if (!cloud)
	{
		assert(false);
		return;
	}

	//original center
	setGlobalShift(cloud->getGlobalShift());
	setGlobalScale(cloud->getGlobalScale());
	//keep the transformation history!
	setGLTransformationHistory(cloud->getGLTransformationHistory());
	//custom point size
	setPointSize(cloud->getPointSize());
	//meta-data
	setMetaData(cloud->metaData());
}
