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

#include "ccObject.h"

//Qt
#include <QSettings>

#ifdef USE_VLD
//VLD
#include <vld.h>
#endif

//System
#include <cstdint>

/** Versions:
	V1.0 = prior to 05/04/2012 = old version
	V2.0 - 05/04/2012 - upgrade to serialized version with version tracking
	V2.1 - 07/02/2012 - points & 2D labels upgraded
	V2.2 - 11/26/2012 - object name is now a QString
	V2.3 - 02/07/2013 - attribute 'm_selectionBehavior' added to ccHObject class
	v2.4 - 02/22/2013 - per-cloud point size + whether name is displayed in 3D or not
	v2.5 - 03/16/2013 - ccViewportParameters structure modified
	v2.6 - 04/03/2013 - strictly positive scalar field removed and 'hidden' values marker is now NaN
	v2.7 - 04/12/2013 - Customizable color scales
	v2.8 - 07/12/2013 - Poylines are now supported
	v2.9 - 08/14/2013 - ccMeshGroup removed, ccSubMesh added
	v3.0 - 08/30/2013 - QObject's meta data structure added
	v3.1 - 09/25/2013 - ccPolyline width added
	v3.2 - 10/11/2013 - ccFacet (2D polygons) are now supported
	v3.3 - 12/19/2013 - global scale information is now saved for point clouds
	v3.4 - 01/09/2014 - ccIndexedTransformation and ccIndexedTransformationBuffer added + CC_CLASS_ENUM is now coded on 64 bits
	v3.5 - 02/13/2014 - ccSensor class updated
	v3.6 - 05/30/2014 - ccGLWindow and associated structures (viewport, etc.) now use double precision
	v3.7 - 08/24/2014 - Textures are stored and saved as a single DB with only references to them in each material (key = absolute filename)
	v3.8 - 09/14/2014 - GBL and camera sensors structures have evolved
	v3.9 - 01/30/2015 - Shift & scale information are now saved for polylines (+ separate interface)
	v4.0 - 08/06/2015 - Custom labels added to color scales
	v4.1 - 09/01/2015 - Scan grids added to point clouds
	v4.2 - 10/07/2015 - Global shift added to the ccScalarField structure
	v4.3 - 01/07/2016 - Additional intrinsic parameters of a camera sensor (optical center)
	v4.4 - 07/07/2016 - Full WaveForm data added to point clouds
	v4.5 - 10/06/2016 - Transformation history is now saved
	v4.6 - 11/03/2016 - Null normal vector code added
	v4.7 - 12/22/2016 - Return index added to ccWaveform
	v4.8 - 10/19/2018 - The CC_CAMERA_BIT and CC_QUADRIC_BIT were wrongly defined
	v4.9 - 03/31/2019 - Point labels can now be picked on meshes
	v5.0 - 10/06/2019 - Point labels can now target the entity center
**/
const unsigned c_currentDBVersion = 50; //5.0

//! Default unique ID generator (using the system persistent settings as we did previously proved to be not reliable)
static ccUniqueIDGenerator::Shared s_uniqueIDGenerator(new ccUniqueIDGenerator);

void ccObject::SetUniqueIDGenerator(ccUniqueIDGenerator::Shared generator)
{
	if (generator == s_uniqueIDGenerator)
		return;

	//we hope that the previous generator has not been used!
	assert (!s_uniqueIDGenerator || s_uniqueIDGenerator->getLast() == 0);
	s_uniqueIDGenerator = generator;
}

ccUniqueIDGenerator::Shared ccObject::GetUniqueIDGenerator()
{
	return s_uniqueIDGenerator;
}

unsigned ccObject::GetCurrentDBVersion()
{
	return c_currentDBVersion;
}

unsigned ccObject::GetNextUniqueID()
{
	if (!s_uniqueIDGenerator)
	{
		assert(false);
		s_uniqueIDGenerator = ccUniqueIDGenerator::Shared(new ccUniqueIDGenerator);
	}
	return s_uniqueIDGenerator->fetchOne();
}

unsigned ccObject::GetLastUniqueID()
{
	return s_uniqueIDGenerator ? s_uniqueIDGenerator->getLast() : 0;
}

ccObject::ccObject(const QString& name, unsigned uniqueID/*=ccUniqueIDGenerator::InvalidUniqueID*/)
	: m_name(name.isEmpty() ? "unnamed" : name)
	, m_flags(CC_ENABLED)
	, m_uniqueID(uniqueID == ccUniqueIDGenerator::InvalidUniqueID ? GetNextUniqueID() : uniqueID)
{}

ccObject::ccObject(const ccObject& object)
	: m_name(object.m_name)
	, m_flags(object.m_flags)
	, m_uniqueID(GetNextUniqueID())
{}

void ccObject::setUniqueID(unsigned ID)
{
	m_uniqueID = ID;

	//updates last unique ID
	if (s_uniqueIDGenerator)
		s_uniqueIDGenerator->update(m_uniqueID);
	else
		assert(false);
}

void ccObject::setFlagState(CC_OBJECT_FLAG flag, bool state)
{
	if (state)
		m_flags |= unsigned(flag);
	else
		m_flags &= (~unsigned(flag));
}

bool ccObject::toFile(QFile& out) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));

	//class ID (dataVersion>=20)
	//DGM: on 64 bits since version 34
	uint64_t classID = static_cast<uint64_t>(getClassID());
	if (out.write((const char*)&classID,8) < 0)
		return WriteError();

	//unique ID (dataVersion>=20)
	//DGM: this ID will be useful to recreate dynamic links between entities!
	uint32_t uniqueID = (uint32_t)m_uniqueID;
	if (out.write((const char*)&uniqueID,4) < 0)
		return WriteError();

	//name (dataVersion>=22)
	{
		QDataStream outStream(&out);
		outStream << m_name;
	}

	//flags (dataVersion>=20)
	uint32_t objFlags = (uint32_t)m_flags;
	if (out.write((const char*)&objFlags,4) < 0)
		return WriteError();

	//meta data (dataVersion>=30)
	{
		//check for valid pieces of meta-data
		//DGM: some pieces of meta-data can't be properly streamed (the ones relying on 'Q_DECLARE_METATYPE' calls typically)
		uint32_t validMetaDataCount = 0;
		for (QVariantMap::const_iterator it = m_metaData.begin(); it != m_metaData.end(); ++it)
		{
			if (!it.key().contains(".nosave"))
			{
				++validMetaDataCount;
			}
		}

		//count
		if (out.write((const char*)&validMetaDataCount, 4) < 0)
			return WriteError();

		//"key + value" pairs
		QDataStream outStream(&out);
		for (QVariantMap::const_iterator it = m_metaData.begin(); it != m_metaData.end(); ++it)
		{
			if (!it.key().contains(".nosave"))
			{
				outStream << it.key();
				outStream << it.value();
			}
		}
	}

	return true;
}

CC_CLASS_ENUM ccObject::ReadClassIDFromFile(QFile& in, short dataVersion)
{
	assert(in.isOpen() && (in.openMode() & QIODevice::ReadOnly));

	//class ID (on 32 bits between version 2.0 and 3.3, then 64 bits from version 3.4)
	CC_CLASS_ENUM classID = CC_TYPES::OBJECT;
	if (dataVersion < 34)
	{
		uint32_t _classID = 0;
		if (in.read((char*)&_classID,4) < 0)
			return ReadError();
		classID = static_cast<CC_CLASS_ENUM>(_classID);
	}
	else
	{
		uint64_t _classID = 0;
		if (in.read((char*)&_classID,8) < 0)
			return ReadError();
		classID = static_cast<CC_CLASS_ENUM>(_classID);
	}

	return classID;
}

QVariant ccObject::getMetaData(const QString& key) const
{
	return m_metaData.value(key,QVariant());
}

bool ccObject::removeMetaData(const QString& key)
{
	return m_metaData.remove(key) != 0;
}

void ccObject::setMetaData(const QString& key, const QVariant& data)
{
	m_metaData.insert(key,data);
}

void ccObject::setMetaData(const QVariantMap& dataset, bool overwrite/*=false*/)
{
	for (QVariantMap::const_iterator it = dataset.begin(); it != dataset.end(); ++it)
	{
		if (overwrite || !m_metaData.contains(it.key()))
		{
			m_metaData[it.key()] = it.value();
		}
	}
}

bool ccObject::hasMetaData(const QString& key) const
{
	return m_metaData.contains(key);
}

bool ccObject::fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	assert(in.isOpen() && (in.openMode() & QIODevice::ReadOnly));

	if (dataVersion < 20)
		return CorruptError();

	//DGM: if we are here, we assume the class ID has already been read!
	//Call ccObject::readClassIDFromFile if necessary
	////class ID (dataVersion>=20)
	//uint32_t classID = 0;
	//if (in.read((char*)&classID,4) < 0)
	//	return ReadError();

	//unique ID (dataVersion>=20)
	uint32_t uniqueID = 0;
	if (in.read((char*)&uniqueID, 4) < 0)
		return ReadError();
	//DGM: this ID will be useful to recreate dynamic links between entities later!
	oldToNewIDMap.insert(uniqueID, m_uniqueID);

	//name
	if (dataVersion < 22) //old style
	{
		char name[256];
		if (in.read(name,256) < 0)
			return ReadError();
		setName(name);
	}
	else //(dataVersion>=22)
	{
		QDataStream inStream(&in);
		inStream >> m_name;
	}

	//flags (dataVersion>=20)
	uint32_t objFlags = 0;
	if (in.read((char*)&objFlags,4) < 0)
		return ReadError();
	m_flags = (unsigned)objFlags;

	//meta data (dataVersion>=30)
	if (dataVersion >= 30)
	{
		//count
		uint32_t metaDataCount = 0;
		if (in.read((char*)&metaDataCount,4) < 0)
			return ReadError();

		//"key + value" pairs
		for (uint32_t i=0; i<metaDataCount; ++i)
		{
			QDataStream inStream(&in);
			QString key;
			QVariant value;
			inStream >> key;
			inStream >> value;
			setMetaData(key,value);
		}
	}

	return true;
}
