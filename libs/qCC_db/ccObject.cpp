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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2172                                                              $
//$LastChangedDate:: 2012-06-24 18:33:24 +0200 (dim., 24 juin 2012)        $
//**************************************************************************
//

#include "ccObject.h"

//Qt
#include <QSettings>

//System
#include <string.h>
#include <assert.h>
#include <stdint.h>

void ccObject::ResetUniqueIDCounter()
{
    QSettings settings;
    //settings.beginGroup("UniqueID");
	settings.setValue("UniqueID",(unsigned)0);
	//settings.endGroup();
}

unsigned ccObject::GetNextUniqueID()
{
    unsigned lastID = GetLastUniqueID();
	++lastID;
	UpdateLastUniqueID(lastID);

	return lastID;
}

unsigned ccObject::GetLastUniqueID()
{
    return QSettings().value("UniqueID", 0).toInt();
}

void ccObject::UpdateLastUniqueID(unsigned lastID)
{
    QSettings().setValue("UniqueID", lastID);
}

ccObject::ccObject(QString name)
{
    m_flags = CC_ENABLED;
    m_uniqueID = GetNextUniqueID();
    setName(name.isEmpty() ? "unnamed" : name);
}

QString ccObject::getName() const
{
    return m_name;
}

void ccObject::setName(const QString& name)
{
	m_name = name;
}

unsigned ccObject::getUniqueID() const
{
    return m_uniqueID;
}

void ccObject::setUniqueID(unsigned ID)
{
	m_uniqueID = ID;

	//updates last unique ID
	if (m_uniqueID>GetLastUniqueID())
		UpdateLastUniqueID(m_uniqueID);
}

bool ccObject::getFlagState(CC_OBJECT_FLAG flag) const
{
    return (m_flags & flag);
}

void ccObject::setFlagState(CC_OBJECT_FLAG flag, bool state)
{
    if (state)
        m_flags |= unsigned(flag);
    else
        m_flags &= (~unsigned(flag));
}

bool ccObject::isEnabled() const
{
    return getFlagState(CC_ENABLED);
}

void ccObject::setEnabled(bool state)
{
    setFlagState(CC_ENABLED,state);
}

bool ccObject::isLocked() const
{
    return getFlagState(CC_LOCKED);
}

void ccObject::setLocked(bool state)
{
    setFlagState(CC_LOCKED,state);
}

bool ccObject::toFile(QFile& out) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));

	//class ID (dataVersion>=20)
	uint32_t classID = getClassID();
	if (out.write((const char*)&classID,4)<0)
		return WriteError();

	//unique ID (dataVersion>=20)
	//DGM: this ID will be usefull to recreate dynamic links between entities!
	uint32_t uniqueID = (uint32_t)m_uniqueID;
	if (out.write((const char*)&uniqueID,4)<0)
		return WriteError();

	//name (dataVersion>=22)
	{
		QDataStream outStream(&out);
		outStream << m_name;
	}

	//flags (dataVersion>=20)
	uint32_t flags = (uint32_t)m_flags;
	if (out.write((const char*)&flags,4)<0)
		return WriteError();

	return true;
}

bool ccObject::ReadClassIDFromFile(unsigned& classID, QFile& in, short dataVersion)
{
	assert(in.isOpen() && (in.openMode() & QIODevice::ReadOnly));

	//class ID (dataVersion>=20)
	uint32_t _classID = 0;
	if (in.read((char*)&_classID,4)<0)
		return ReadError();
	
	classID = (unsigned)_classID;
	
	return true;
}

bool ccObject::fromFile(QFile& in, short dataVersion)
{
	assert(in.isOpen() && (in.openMode() & QIODevice::ReadOnly));

	if (dataVersion<20)
		return CorruptError();

	//DGM: if we are here, we assume the class ID has already been read!
	//Call ccObject::readClassIDFromFile if necessary
	////class ID (dataVersion>=20)
	//uint32_t classID = 0;
	//if (in.read((char*)&classID,4)<0)
	//	return ReadError();

	//unique ID (dataVersion>=20)
	//DGM: this ID will be usefull to recreate dynamic links between entities!
	uint32_t uniqueID = 0;
	if (in.read((char*)&uniqueID,4)<0)
		return ReadError();
	m_uniqueID = (unsigned)uniqueID;

	//name
	if (dataVersion < 22) //old style
	{
		char name[256];
		if (in.read(name,256)<0)
			return ReadError();
		setName(name);
	}
	else //(dataVersion>=22)
	{
		QDataStream inStream(&in);
		inStream >> m_name;
	}

	//flags (dataVersion>=20)
	uint32_t flags = 0;
	if (in.read((char*)&flags,4)<0)
		return ReadError();
	m_flags = (unsigned)flags;

	return true;
}
