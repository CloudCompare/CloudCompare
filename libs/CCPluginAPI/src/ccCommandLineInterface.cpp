// ##########################################################################
// #                                                                        #
// #                            CLOUDCOMPARE                                #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 of the License.               #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: CloudCompare project                               #
// #                                                                        #
// ##########################################################################

#include "ccCommandLineInterface.h"

#include "ccArgumentParser.h"
#include "ccGenericMesh.h"

#include <QDir>

namespace
{
	constexpr char COMMAND_OPEN_SHIFT_ON_LOAD[]       = "GLOBAL_SHIFT"; //!< Global shift
	constexpr char COMMAND_OPEN_SHIFT_ON_LOAD_AUTO[]  = "AUTO";         //!< "AUTO" keyword
	constexpr char COMMAND_OPEN_SHIFT_ON_LOAD_FIRST[] = "FIRST";        //!< "FIRST" keyword
} // namespace

//////
// CLEntityDesc

CLEntityDesc::CLEntityDesc(const QString& name)
    : basename(name)
    , path(QDir::currentPath())
    , indexInFile(-1)
{
}

CLEntityDesc::CLEntityDesc(const QString& filename, int _indexInFile)
    : indexInFile(_indexInFile)
{
	if (filename.isNull())
	{
		basename = "unknown";
		path     = QDir::currentPath();
	}
	else
	{
		QFileInfo fi(filename);
		basename = fi.completeBaseName();
		path     = fi.path();
	}
}

CLEntityDesc::CLEntityDesc(const QString& _basename, const QString& _path, int _indexInFile)
    : basename(_basename)
    , path(_path)
    , indexInFile(_indexInFile)
{
}

//////
// CLGroupDesc

CLGroupDesc::CLGroupDesc(ccHObject* group, const QString& basename, const QString& path)
    : CLEntityDesc(basename, path)
    , groupEntity(group)
{
}

ccHObject* CLGroupDesc::getEntity()
{
	return groupEntity;
}

const ccHObject* CLGroupDesc::getEntity() const
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
    , pc(nullptr)
{
}

CLCloudDesc::CLCloudDesc(ccPointCloud* cloud, const QString& filename, int index)
    : CLEntityDesc(filename, index)
    , pc(cloud)
{
}

CLCloudDesc::CLCloudDesc(ccPointCloud* cloud, const QString& basename, const QString& path, int index)
    : CLEntityDesc(basename, path, index)
    , pc(cloud)
{
}

ccHObject* CLCloudDesc::getEntity()
{
	return static_cast<ccHObject*>(pc);
}

const ccHObject* CLCloudDesc::getEntity() const
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
    , mesh(nullptr)
{
}

CLMeshDesc::CLMeshDesc(ccGenericMesh* _mesh, const QString& filename, int index)
    : CLEntityDesc(filename, index)
    , mesh(_mesh)
{
}

CLMeshDesc::CLMeshDesc(ccGenericMesh* _mesh, const QString& basename, const QString& path, int index)
    : CLEntityDesc(basename, path, index)
    , mesh(_mesh)
{
}

ccHObject* CLMeshDesc::getEntity()
{
	return static_cast<ccHObject*>(mesh);
}

const ccHObject* CLMeshDesc::getEntity() const
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
{
}

bool ccCommandLineInterface::IsCommand(const QString& token, const char* command)
{
	return token.startsWith("-") && token.mid(1).toUpper() == QString(command);
}

ccProgressDialog* ccCommandLineInterface::progressDialog()
{
	return nullptr;
}

QDialog* ccCommandLineInterface::widgetParent()
{
	return nullptr;
}

ccCommandLineInterface::CLLoadParameters& ccCommandLineInterface::fileLoadingParams()
{
	return m_loadingParameters;
}

std::vector<CLCloudDesc>& ccCommandLineInterface::clouds()
{
	return m_clouds;
}

const std::vector<CLCloudDesc>& ccCommandLineInterface::clouds() const
{
	return m_clouds;
}

std::vector<CLMeshDesc>& ccCommandLineInterface::meshes()
{
	return m_meshes;
}

const std::vector<CLMeshDesc>& ccCommandLineInterface::meshes() const
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

bool ccCommandLineInterface::nextCommandIsGlobalShift() const
{
	return !arguments().empty() && IsCommand(arguments().front(), COMMAND_OPEN_SHIFT_ON_LOAD);
}

std::optional<ccCommandLineInterface::GlobalShiftOptions>
ccCommandLineInterface::ParseGlobalShiftOptions(ccArgumentParser& parser)
{
	GlobalShiftOptions options; // defaults: NO_GLOBAL_SHIFT, (0,0,0)

	if (parser.isEmpty())
	{
		ccLog::Error(QObject::tr("Missing parameter: global shift vector or %1 or %2 after '%3'")
		                 .arg(COMMAND_OPEN_SHIFT_ON_LOAD_AUTO, COMMAND_OPEN_SHIFT_ON_LOAD_FIRST, COMMAND_OPEN_SHIFT_ON_LOAD));
		return std::nullopt;
	}

	const QString firstParam = parser.takeNext();
	const QString firstUpper = firstParam.toUpper();

	if (firstUpper == COMMAND_OPEN_SHIFT_ON_LOAD_AUTO)
	{
		options.mode = GlobalShiftOptions::AUTO_GLOBAL_SHIFT;
		return options;
	}
	if (firstUpper == COMMAND_OPEN_SHIFT_ON_LOAD_FIRST)
	{
		options.mode = GlobalShiftOptions::FIRST_GLOBAL_SHIFT;
		return options;
	}

	// firstParam is the X coordinate of a custom shift vector — we need Y and Z too
	if (parser.size() < 2)
	{
		ccLog::Error(QObject::tr("Missing parameter: global shift vector after '%1' (3 values expected)").arg(COMMAND_OPEN_SHIFT_ON_LOAD));
		return std::nullopt;
	}

	const auto x = ccArgumentParser::ParseDouble(firstParam, QObject::tr("X coordinate of the global shift vector"));
	if (!x)
		return std::nullopt;

	const auto y = parser.takeDouble(QObject::tr("Y coordinate of the global shift vector"));
	if (!y)
		return std::nullopt;

	const auto z = parser.takeDouble(QObject::tr("Z coordinate of the global shift vector"));
	if (!z)
		return std::nullopt;

	options.mode              = GlobalShiftOptions::CUSTOM_GLOBAL_SHIFT;
	options.customGlobalShift = CCVector3d(*x, *y, *z);
	return options;
}

bool ccCommandLineInterface::processGlobalShiftCommand(GlobalShiftOptions& options)
{
	// defaults in case of an early exit (preserved for API compatibility)
	options.mode              = GlobalShiftOptions::NO_GLOBAL_SHIFT;
	options.customGlobalShift = CCVector3d(0, 0, 0);

	ccArgumentParser parser(arguments());
	auto             result = ParseGlobalShiftOptions(parser);
	if (!result)
	{
		return false; // error already logged
	}
	options = *result;
	return true;
}

//////
// ccCommandLineInterface::Command

ccCommandLineInterface::Command::Command(const QString& name, const QString& keyword)
    : m_name(name)
    , m_keyword(keyword)
{
}

//////
// ccCommandLineInterface::CLLoadParameters

ccCommandLineInterface::CLLoadParameters::CLLoadParameters()
    : FileIOFilter::LoadParameters()
    , coordinatesShiftEnabled(false)
    , coordinatesShift(0, 0, 0)
{
	shiftHandlingMode        = ccGlobalShiftManager::NO_DIALOG;
	alwaysDisplayLoadDialog  = false;
	autoComputeNormals       = false;
	_coordinatesShiftEnabled = &coordinatesShiftEnabled;
	_coordinatesShift        = &coordinatesShift;
}
