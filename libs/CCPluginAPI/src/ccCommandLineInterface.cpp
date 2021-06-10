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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include <QDir>

#include "ccGenericMesh.h"

#include "ccCommandLineInterface.h"


namespace
{
	constexpr char COMMAND_OPEN_SHIFT_ON_LOAD[] = "GLOBAL_SHIFT";	//!< Global shift
	constexpr char COMMAND_OPEN_SHIFT_ON_LOAD_AUTO[] = "AUTO";		//!< "AUTO" keyword
	constexpr char COMMAND_OPEN_SHIFT_ON_LOAD_FIRST[] = "FIRST";	//!< "FIRST" keyword	
}

//////
// CLEntityDesc

CLEntityDesc::CLEntityDesc(const QString &name)
	: basename( name )
	, path( QDir::currentPath() )
	, indexInFile( -1 )
{	
}

CLEntityDesc::CLEntityDesc(const QString &filename, int _indexInFile)
	: indexInFile(_indexInFile)
{
	if (filename.isNull())
	{
		basename = "unknown";
		path = QDir::currentPath();
	}
	else
	{
		QFileInfo fi(filename);
		basename = fi.completeBaseName();
		path = fi.path();
	}
}

CLEntityDesc::CLEntityDesc(const QString &_basename, const QString &_path, int _indexInFile)
	: basename(_basename)
	, path(_path)
	, indexInFile(_indexInFile)
{
}


//////
// CLGroupDesc

CLGroupDesc::CLGroupDesc(ccHObject *group, const QString &basename, const QString &path)
	: CLEntityDesc(basename, path)
	, groupEntity(group)
{}

ccHObject *CLGroupDesc::getEntity()
{
	return groupEntity;
}

const ccHObject *CLGroupDesc::getEntity() const
{
	return groupEntity;
}

CL_ENTITY_TYPE CLGroupDesc::getCLEntityType() const
{
	return CL_ENTITY_TYPE::GROUP;
}

//////
// CLCloudDesc

CLCloudDesc::CLCloudDesc()
	: CLEntityDesc("Unnamed cloud")
	, pc( nullptr )
{}

CLCloudDesc::CLCloudDesc(ccPointCloud *cloud, const QString &filename, int index)
	: CLEntityDesc(filename, index)
	, pc(cloud)
{}

CLCloudDesc::CLCloudDesc(ccPointCloud *cloud, const QString &basename, const QString &path, int index)
	: CLEntityDesc(basename, path, index)
	, pc(cloud)
{}

ccHObject *CLCloudDesc::getEntity()
{
	return static_cast<ccHObject*>(pc);
}

const ccHObject *CLCloudDesc::getEntity() const
{
	return static_cast<ccHObject*>(pc);
}

CL_ENTITY_TYPE CLCloudDesc::getCLEntityType() const
{
	return CL_ENTITY_TYPE::CLOUD;
}

//////
// CLMeshDesc

CLMeshDesc::CLMeshDesc()
	: CLEntityDesc("Unnamed mesh")
	, mesh( nullptr )
{}

CLMeshDesc::CLMeshDesc(ccGenericMesh *_mesh, const QString &filename, int index)
	: CLEntityDesc(filename, index)
	, mesh(_mesh)
{}

CLMeshDesc::CLMeshDesc(ccGenericMesh *_mesh, const QString &basename, const QString &path, int index)
	: CLEntityDesc(basename, path, index)
	, mesh(_mesh)
{}

ccHObject *CLMeshDesc::getEntity()
{
	return static_cast<ccHObject*>(mesh);
}

const ccHObject *CLMeshDesc::getEntity() const
{
	return static_cast<ccHObject*>(mesh);
}

CL_ENTITY_TYPE CLMeshDesc::getCLEntityType() const
{
	return CL_ENTITY_TYPE::MESH;
}

//////
// ccCommandLineInterface

ccCommandLineInterface::ccCommandLineInterface()
	: m_silentMode(false)
	, m_autoSaveMode(true)
	, m_addTimestamp(true)
	, m_precision(12)
	, m_coordinatesShiftWasEnabled(false)
{}

bool ccCommandLineInterface::IsCommand(const QString &token, const char *command)
{
	return token.startsWith("-") && token.mid(1).toUpper() == QString(command);
}

ccProgressDialog *ccCommandLineInterface::progressDialog()
{
	return nullptr;
}

QDialog *ccCommandLineInterface::widgetParent()
{
	return nullptr;
}

ccCommandLineInterface::CLLoadParameters &ccCommandLineInterface::fileLoadingParams()
{
	return m_loadingParameters;
}

std::vector<CLCloudDesc> &ccCommandLineInterface::clouds()
{
	return m_clouds;
}

const std::vector<CLCloudDesc> &ccCommandLineInterface::clouds() const
{
	return m_clouds;
}

std::vector<CLMeshDesc> &ccCommandLineInterface::meshes()
{
	return m_meshes;
}

const std::vector<CLMeshDesc> &ccCommandLineInterface::meshes() const
{
	return m_meshes;
}

void ccCommandLineInterface::toggleSilentMode(bool state)
{
	m_silentMode = state;
}

bool ccCommandLineInterface::silentMode() const
{
	return m_silentMode;
}

void ccCommandLineInterface::toggleAutoSaveMode(bool state)
{
	m_autoSaveMode = state;
}

bool ccCommandLineInterface::autoSaveMode() const
{
	return m_autoSaveMode;
}

void ccCommandLineInterface::toggleAddTimestamp(bool state)
{
	m_addTimestamp = state;
}

bool ccCommandLineInterface::addTimestamp() const
{
	return m_addTimestamp;
}

void ccCommandLineInterface::setNumericalPrecision(int p)
{
	m_precision = p;
}

int ccCommandLineInterface::numericalPrecision() const
{
	return m_precision;
}

bool ccCommandLineInterface::coordinatesShiftWasEnabled() const
{
	return m_coordinatesShiftWasEnabled;
}

const CCVector3d &ccCommandLineInterface::formerCoordinatesShift() const
{
	return m_formerCoordinatesShift;
}

void ccCommandLineInterface::storeCoordinatesShiftParams()
{ m_coordinatesShiftWasEnabled = m_loadingParameters.m_coordinatesShiftEnabled; m_formerCoordinatesShift = m_loadingParameters.m_coordinatesShift; }

bool ccCommandLineInterface::nextCommandIsGlobalShift() const
{
	return !arguments().empty() && IsCommand(arguments().front(), COMMAND_OPEN_SHIFT_ON_LOAD);
}

bool ccCommandLineInterface::processGlobalShiftCommand()
{
	if (arguments().empty())
	{
		return error(QObject::tr("Missing parameter: global shift vector or %1 or %2 after '%3'")
					 .arg(COMMAND_OPEN_SHIFT_ON_LOAD_AUTO, COMMAND_OPEN_SHIFT_ON_LOAD_FIRST, COMMAND_OPEN_SHIFT_ON_LOAD));
	}
	
	QString firstParam = arguments().takeFirst();
	
	m_loadingParameters.shiftHandlingMode = ccGlobalShiftManager::NO_DIALOG;
	m_loadingParameters.m_coordinatesShiftEnabled = false;
	m_loadingParameters.m_coordinatesShift = CCVector3d(0, 0, 0);
	
	if (firstParam.toUpper() == COMMAND_OPEN_SHIFT_ON_LOAD_AUTO)
	{
		//let CC handle the global shift automatically
		m_loadingParameters.shiftHandlingMode = ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT;
	}
	else if (firstParam.toUpper() == COMMAND_OPEN_SHIFT_ON_LOAD_FIRST)
	{
		//use the first encountered global shift value (if any)
		m_loadingParameters.shiftHandlingMode = ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT;
		m_loadingParameters.m_coordinatesShiftEnabled = m_coordinatesShiftWasEnabled;
		m_loadingParameters.m_coordinatesShift = m_formerCoordinatesShift;
	}
	else if (arguments().size() < 2)
	{
		return error(QObject::tr("Missing parameter: global shift vector after '%1' (3 values expected)").arg(COMMAND_OPEN_SHIFT_ON_LOAD));
	}
	else
	{
		bool ok = true;
		CCVector3d shiftOnLoadVec;
		shiftOnLoadVec.x = firstParam.toDouble(&ok);
		if (!ok)
			return error(QObject::tr("Invalid parameter: X coordinate of the global shift vector after '%1'").arg(COMMAND_OPEN_SHIFT_ON_LOAD));
		shiftOnLoadVec.y = arguments().takeFirst().toDouble(&ok);
		if (!ok)
			return error(QObject::tr("Invalid parameter: Y coordinate of the global shift vector after '%1'").arg(COMMAND_OPEN_SHIFT_ON_LOAD));
		shiftOnLoadVec.z = arguments().takeFirst().toDouble(&ok);
		if (!ok)
			return error(QObject::tr("Invalid parameter: Z coordinate of the global shift vector after '%1'").arg(COMMAND_OPEN_SHIFT_ON_LOAD));
		
		//set the user defined shift vector as default shift information
		m_loadingParameters.m_coordinatesShiftEnabled = true;
		m_loadingParameters.m_coordinatesShift = shiftOnLoadVec;
	}
	
	return true;
}

//////
// ccCommandLineInterface::Command

ccCommandLineInterface::Command::Command(const QString &name, const QString &keyword)
	: m_name(name)
	, m_keyword(keyword)
{}


//////
// ccCommandLineInterface::CLLoadParameters

ccCommandLineInterface::CLLoadParameters::CLLoadParameters()
	: FileIOFilter::LoadParameters()
	, m_coordinatesShiftEnabled(false)
	, m_coordinatesShift(0, 0, 0)
{
	shiftHandlingMode = ccGlobalShiftManager::NO_DIALOG;
	alwaysDisplayLoadDialog = false;
	autoComputeNormals = false;
	coordinatesShiftEnabled = &m_coordinatesShiftEnabled;
	coordinatesShift = &m_coordinatesShift;
}
