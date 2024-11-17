#include "ccCommandLineParser.h"

//Local
#include "ccCommandCrossSection.h"
#include "ccCommandLineCommands.h"
#include "ccCommandRaster.h"
#include "ccPluginInterface.h"

//qCC_db
#include <ccGenericMesh.h>
#include <ccHObjectCaster.h>
#include <ccProgressDialog.h>

//qCC_io
#include <AsciiFilter.h>
#include <BinFilter.h>

//qCC
#include "ccConsole.h"

#include <ui_commandLineDlg.h>

//Qt
#include <QDateTime>
#include <QElapsedTimer>
#include <QMessageBox>

//system
#include <unordered_set>

//commands
constexpr char COMMAND_HELP[]			= "HELP";
constexpr char COMMAND_SILENT_MODE[]	= "SILENT";

/*****************************************************/
/*************** ccCommandLineParser *****************/
/*****************************************************/

void ccCommandLineParser::printVerbose(const QString& message) const
{
	ccConsole::PrintVerbose(message);
}

void ccCommandLineParser::print(const QString& message) const
{
	ccConsole::Print(message);
}

void ccCommandLineParser::printHigh(const QString& message) const
{
	ccConsole::PrintHigh(message);
}

void ccCommandLineParser::printDebug(const QString& message) const
{
	ccConsole::PrintDebug(message);
}

void ccCommandLineParser::warning(const QString& message) const
{
	ccConsole::Warning(message);
}

void ccCommandLineParser::warningDebug(const QString& message) const
{
	ccConsole::WarningDebug(message);
}

bool ccCommandLineParser::error(const QString& message) const
{
	ccConsole::Error(message);

	return false;
}

bool ccCommandLineParser::errorDebug(const QString& message) const
{
	ccConsole::ErrorDebug(message);

	return false;
}

int ccCommandLineParser::Parse(const QStringList& arguments, ccPluginInterfaceList& plugins)
{
	if (arguments.size() < 2)
	{
		assert(false);
		return EXIT_SUCCESS;
	}

	//load arguments
	QScopedPointer<ccCommandLineParser> parser(new ccCommandLineParser);
	
	parser->registerBuiltInCommands();

	ccConsole::SetRefreshCycle(200);

	// 'massage' the arguments to properly handle single quotes
	{
		bool insideSingleQuoteSection = false;
		QString buffer;
		static const QChar SingleQuote{ '\'' };
		for (int currentArgIndex = 1; currentArgIndex < arguments.size(); ++currentArgIndex) 	// start from 1, as the first argument is always the executable file
		{
			QString arg = arguments[currentArgIndex];
			// argument starts with a single quote
			if (!insideSingleQuoteSection && arg.startsWith(SingleQuote))
			{
				if (arg.endsWith(SingleQuote))
				{
					// nothing to do, non-truncated argument
				}
				else
				{
					// we'll collect the next pieces to get the full argument
					insideSingleQuoteSection = true;
					buffer = arg.mid(1); // remove the single quote
				}
			}
			else if (insideSingleQuoteSection)
			{
				buffer += QChar(' ') + arg; // append the current argument to the previous one(s)
				if (arg.endsWith(SingleQuote))
				{
					insideSingleQuoteSection = false;
					arg = buffer.left(buffer.length() - 1); // remove the single quote
				}
			}

			if (!insideSingleQuoteSection)
			{
				parser->arguments().append(arg);
			}
		}

		if (insideSingleQuoteSection)
		{
			// the single quote section was not closed...
			parser->warning("Probably malformed command (missing closing simple quote)");
			// ...still, we'll try to proceed
			parser->arguments().append(buffer);
		}
	}

	//specific command: silent mode (will prevent the console dialog from appearing!
	if (ccCommandLineInterface::IsCommand(parser->arguments().front(), COMMAND_SILENT_MODE))
	{
		parser->arguments().pop_front();
		parser->toggleSilentMode(true);
	}

	QScopedPointer<QDialog> consoleDlg(nullptr);
	if (!parser->silentMode())
	{
		//show console
		consoleDlg.reset(new QDialog);
		Ui_commandLineDlg commandLineDlg;
		commandLineDlg.setupUi(consoleDlg.data());
		consoleDlg->show();
		ccConsole::Init(commandLineDlg.consoleWidget, consoleDlg.data());
		parser->fileLoadingParams().parentWidget = consoleDlg.data();
		QApplication::processEvents(); //Get rid of the spinner
	}
	else
	{
		//allows ccLog/ccConsole or ccCommandLineParser (print, warning, error) to output to the console 
		ccConsole::Init(nullptr, nullptr, nullptr, true); 
	}

	//load the plugins commands
	for ( ccPluginInterface *plugin : plugins )
	{
		if (!plugin)
		{
			assert(false);
			continue;
		}

		plugin->registerCommands(parser.data());
	}

	//parse input
	int result = parser->start(consoleDlg.data());

	if (!parser->silentMode())
	{
		if (result == EXIT_SUCCESS)
			QMessageBox::information(consoleDlg.data(), "Processed finished", "Job done");
		else
			QMessageBox::warning(consoleDlg.data(), "Processed finished", "An error occurred! Check console");
	}

	//release the parser before the console (as its dialogs may be chidren of the console)
	parser->cleanup();
	parser.reset();

	ccConsole::ReleaseInstance();

	return result;
}

ccCommandLineParser::ccCommandLineParser()
	: ccCommandLineInterface()
	, m_cloudExportFormat(BinFilter::GetFileFilter())
	, m_cloudExportExt(BinFilter::GetDefaultExtension())
	, m_meshExportFormat(BinFilter::GetFileFilter())
	, m_meshExportExt(BinFilter::GetDefaultExtension())
	, m_hierarchyExportFormat(BinFilter::GetFileFilter())
	, m_hierarchyExportExt(BinFilter::GetDefaultExtension())
	, m_orphans("orphans")
	, m_progressDialog(nullptr)
	, m_parentWidget(nullptr)
{
}

ccCommandLineParser::~ccCommandLineParser()
{
	if (m_progressDialog)
	{
		m_progressDialog->close();
		m_progressDialog->deleteLater();
	}
}

bool ccCommandLineParser::registerCommand(Command::Shared command)
{
	if (!command)
	{
		assert(false);
		return false;
	}

	if (m_commands.contains(command->m_keyword))
	{
		assert(false);
		warning(QString("Internal error: keyword '%1' already registered (by command '%2')").arg(command->m_keyword, m_commands[command->m_keyword]->m_name));
		return false;
	}

	m_commands.insert(command->m_keyword, command);

	return true;
}

QString ccCommandLineParser::getExportFilename(	const CLEntityDesc& entityDesc,
												QString extension/*=QString()*/,
												QString suffix/*=QString()*/,
												QString* baseOutputFilename/*=nullptr*/,
												bool forceNoTimestamp/*=false*/) const
{
	//fetch the real entity
	const ccHObject* entity = entityDesc.getEntity();
	if (!entity)
	{
		assert(false);
		warning("[getExportFilename] Internal error: invalid input entity!");
		return QString();
	}

	//sub-item?
	if (entityDesc.indexInFile >= 0)
	{
		if (suffix.isEmpty())
			suffix = QString("%1").arg(entityDesc.indexInFile);
		else
			suffix.prepend(QString("%1_").arg(entityDesc.indexInFile));
	}

	QString baseName = entityDesc.basename;
	if (!suffix.isEmpty())
	{
		baseName += QString("_") + suffix;
	}

	QString outputFilename = baseName;
	if (m_addTimestamp && !forceNoTimestamp)
	{
		outputFilename += QString("_%1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd_hh'h'mm_ss_zzz"));
	}

	if (!extension.isEmpty())
	{
		outputFilename += '.' + extension;
	}

	if (baseOutputFilename)
	{
		*baseOutputFilename = outputFilename;
	}

	if (!entityDesc.path.isEmpty())
	{
		outputFilename.prepend(entityDesc.path + '/');
	}

	return outputFilename;
}

QString ccCommandLineParser::exportEntity(	CLEntityDesc& entityDesc,
											const QString& suffix/*=QString()*/,
											QString* baseOutputFilename/*=nullptr*/,
											ccCommandLineInterface::ExportOptions options/*ExportOptiopn::NoOption*/)
{
	print("[SAVING]");

	//fetch the real entity
	ccHObject* entity = entityDesc.getEntity();
	if (!entity)
	{
		assert(false);
		return "[ExportEntity] Internal error: invalid input entity!";
	}

	bool anyForced = options.testFlag(ExportOption::ForceCloud) | options.testFlag(ExportOption::ForceHierarchy) | options.testFlag(ExportOption::ForceMesh);
	//specific case: clouds
	bool isCloud = entity->isA(CC_TYPES::POINT_CLOUD) || entityDesc.getCLEntityType() == CL_ENTITY_TYPE::CLOUD;

	//specific case: mesh
	bool isMesh = entity->isKindOf(CC_TYPES::MESH) || entityDesc.getCLEntityType() == CL_ENTITY_TYPE::MESH;

	QString extension = isCloud ? m_cloudExportExt : isMesh ? m_meshExportExt : m_hierarchyExportExt;
	QString format = isCloud ? m_cloudExportFormat : isMesh ? m_meshExportFormat : m_hierarchyExportFormat;
	if (anyForced)
	{
		if (options.testFlag(ExportOption::ForceCloud))
		{
			extension = m_cloudExportExt;
			format = m_cloudExportFormat;
		}
		if (options.testFlag(ExportOption::ForceMesh))
		{
			extension = m_meshExportExt;
			format = m_meshExportFormat;
		}
		if (options.testFlag(ExportOption::ForceHierarchy))
		{
			extension = m_hierarchyExportExt;
			format = m_hierarchyExportFormat;
		}
	}

	QString outputFilename = getExportFilename(	entityDesc,
												extension,
												suffix,
												baseOutputFilename,
												options.testFlag(ExportOption::ForceNoTimestamp) );
	if (outputFilename.isEmpty())
	{
		return QString();
	}

	//update the entity name as well
	{
		QString entName = entity->getName();
		if (entName.isEmpty())
		{
			entName = entityDesc.basename;
		}

		if (!suffix.isEmpty())
		{
			entName += QString("_") + suffix;
		}

		entity->setName(entName);
	}

	bool tempDependencyCreated = false;
	ccGenericMesh* mesh = nullptr;
	if (entity->isKindOf(CC_TYPES::MESH) && m_meshExportFormat == BinFilter::GetFileFilter())
	{
		//in a BIN file we must save the vertices cloud as well if it's not a child of the mesh!
		mesh = static_cast<ccGenericMesh*>(entity);
		ccGenericPointCloud* vertices = mesh->getAssociatedCloud();
		if (vertices && !mesh->isAncestorOf(vertices))
		{
			//we save the cloud first!
			vertices->addChild(mesh, ccHObject::DP_NONE); //we simply add a fake dependency
			entity = vertices;
			tempDependencyCreated = true;
		}
	}

	//save file
	FileIOFilter::SaveParameters parameters;
	{
		//no dialog by default for command line mode!
		parameters.alwaysDisplaySaveDialog = false;
		if (!silentMode() && ccConsole::TheInstance())
		{
			parameters.parentWidget = ccConsole::TheInstance()->parentWidget();
		}
	}

#ifdef _DEBUG
	print("Output filename: " + outputFilename);
#endif
	CC_FILE_ERROR result = FileIOFilter::SaveToFile(entity,
													outputFilename,
													parameters,
													format);

	//restore input state!
	if (tempDependencyCreated)
	{
		if (mesh && entity)
		{
			entity->detachChild(mesh);
		}
		else
		{
			assert(false);
		}
	}

	return (result != CC_FERR_NO_ERROR ? QString("Failed to save result in file '%1'").arg(outputFilename) : QString());
}

template<class EntityDesc > bool SelectEntities(ccCommandLineInterface::SelectEntitiesOptions options,
												const ccCommandLineParser& cmd,
												std::vector<EntityDesc>& selectedEntities,
												std::vector<EntityDesc>& unselectedEntities,
												QString entityType)
{
	//early abort if no cloud found
	if (selectedEntities.empty() && unselectedEntities.empty())
	{
		//do not stop execution, just warn the user and return
		cmd.warning(QObject::tr("\tNo %1 loaded. Load some with the -O command").arg(entityType));
		return true;
	}

	if (options.selectRegex && !options.regex.isValid())
	{
		return cmd.error(QObject::tr("Regex string invalid: %1").arg(options.regex.errorString()));
	}

	try
	{
		//store everthyng in the unselected vector
		unselectedEntities.insert(unselectedEntities.end(), selectedEntities.begin(), selectedEntities.end());
		selectedEntities.clear();

		//sort the unselected clouds by uniqueID (so as to restore the order in which they were loaded/created)
		std::sort(unselectedEntities.begin(), unselectedEntities.end(), [](const EntityDesc& a, const EntityDesc& b) { return (a.getEntity()->getUniqueID() < b.getEntity()->getUniqueID()); });

		//put elements to the front facing vector
		unsigned index = 0;
		assert(!unselectedEntities.empty()); // we have tested above that neither selectedEntities and unselectedEntities are both empty
		size_t lastIndex = unselectedEntities.size() - 1;
		for (typename std::vector<EntityDesc>::iterator it = unselectedEntities.begin(); it != unselectedEntities.end();)
		{
			QString nameToValidate = QObject::tr("%1/%2").arg(it->basename).arg(it->getEntity()->getName());
			bool toBeSelected = false;
			if (!options.reverse)
			{
				//first {n}
				if (options.selectFirst && index < options.firstNr)
				{
					toBeSelected = true;
				}

				//last {n}
				if (options.selectLast && index > lastIndex - options.lastNr)
				{
					toBeSelected = true;
				}
			}
			else
			{
				//not first {n}
				if (options.selectFirst && index >= options.firstNr && !options.selectLast)
				{
					toBeSelected = true;
				}

				//not last {n}
				if (options.selectLast && index <= lastIndex - options.lastNr && !options.selectFirst)
				{
					toBeSelected = true;
				}

				//not first and not last
				if (options.selectFirst && options.selectLast && index >= options.firstNr && index <= lastIndex - options.lastNr)
				{
					toBeSelected = true;
				}
			}

			//regex has higher priority than first/last overwrite
			if (options.selectRegex)
			{
				if (options.regex.indexIn(nameToValidate) > -1)
				{
					//regex matched
					toBeSelected = !options.reverse;
				}
				else
				{
					//regex not matched
					toBeSelected = options.reverse;
				}
			}

			//selectAll has higher priority than first/last/regex overwrite
			if (options.selectAll)
			{
				toBeSelected = !options.reverse;
			}

			if (toBeSelected)
			{
				cmd.print(QObject::tr("\t[*] UID: %2 name: %1").arg(nameToValidate).arg(it->getEntity()->getUniqueID()));
				selectedEntities.push_back(*it);
				it = unselectedEntities.erase(it);
			}
			else
			{
				cmd.print(QObject::tr("\t[ ] UID: %2 name: %1").arg(nameToValidate).arg(it->getEntity()->getUniqueID()));
				++it;
			}
			index++;
		}
	}
	catch (const std::bad_alloc&)
	{
		return cmd.error(QObject::tr("Not enough memory"));
	}

	return true;
}

bool ccCommandLineParser::selectClouds(const SelectEntitiesOptions& options)
{
	return SelectEntities(options, *this, m_clouds, m_unselectedClouds, "cloud");
}

bool ccCommandLineParser::selectMeshes(const SelectEntitiesOptions& options)
{
	return SelectEntities(options, *this, m_meshes, m_unselectedMeshes, "mesh");
}

void ccCommandLineParser::removeClouds(bool onlyLast/*=false*/)
{
	while (!m_clouds.empty())
	{
		delete m_clouds.back().pc;
		m_clouds.pop_back();

		if (onlyLast)
			break;
	}
}

void ccCommandLineParser::removeMeshes(bool onlyLast/*=false*/)
{
	while (!m_meshes.empty())
	{
		delete m_meshes.back().mesh;
		m_meshes.pop_back();

		if (onlyLast)
			break;
	}
}

//! Whether Global (coordinate) shift has already been defined
static bool s_firstCoordinatesShiftEnabled = false;
//! Global shift (if defined)
static CCVector3d s_firstGlobalShift;
//! First time the global shift is set/defined
static bool s_globalShiftFirstTime = true;

void ccCommandLineParser::setGlobalShiftOptions(const GlobalShiftOptions& globalShiftOptions)
{
	//default Global Shift handling parameters
	m_loadingParameters.shiftHandlingMode = ccGlobalShiftManager::NO_DIALOG;
	m_loadingParameters.coordinatesShiftEnabled = false;
	m_loadingParameters.coordinatesShift = CCVector3d(0, 0, 0);

	switch (globalShiftOptions.mode)
	{
	case GlobalShiftOptions::AUTO_GLOBAL_SHIFT:
		//let CC handle the global shift automatically
		m_loadingParameters.shiftHandlingMode = ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT;
		break;

	case GlobalShiftOptions::FIRST_GLOBAL_SHIFT:
		//use the first encountered global shift value (if any)
		if (s_globalShiftFirstTime)
		{
			ccLog::Warning("Can't reuse the first Global Shift (no global shift set yet)");
			m_loadingParameters.shiftHandlingMode = ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT;
		}
		else
		{
			m_loadingParameters.coordinatesShiftEnabled = s_firstCoordinatesShiftEnabled;
			m_loadingParameters.coordinatesShift = s_firstGlobalShift;
		}
		break;

	case GlobalShiftOptions::CUSTOM_GLOBAL_SHIFT:
		//set the user defined shift vector as default shift information
		m_loadingParameters.coordinatesShiftEnabled = true;
		m_loadingParameters.coordinatesShift = globalShiftOptions.customGlobalShift;
		break;

	default:
		//nothing to do
		break;
	}
}

void ccCommandLineParser::updateInteralGlobalShift(const GlobalShiftOptions& globalShiftOptions)
{
	if (globalShiftOptions.mode != GlobalShiftOptions::NO_GLOBAL_SHIFT)
	{
		if (s_globalShiftFirstTime)
		{
			// remember the first Global Shift parameters used
			s_firstCoordinatesShiftEnabled = m_loadingParameters.coordinatesShiftEnabled;
			s_firstGlobalShift = m_loadingParameters.coordinatesShift;
			s_globalShiftFirstTime = false;
		}
	}
}

bool ccCommandLineParser::importFile(QString filename, const GlobalShiftOptions& globalShiftOptions, FileIOFilter::Shared filter)
{
	printHigh(QString("Opening file: '%1'").arg(filename));

	setGlobalShiftOptions(globalShiftOptions);

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	ccHObject* db = nullptr;
	if (filter)
	{
		db = FileIOFilter::LoadFromFile(filename, m_loadingParameters, filter, result);
	}
	else
	{
		db = FileIOFilter::LoadFromFile(filename, m_loadingParameters, result, QString());
	}

	if (!db)
	{
		return false/*cmd.error(QString("Failed to open file '%1'").arg(filename))*/; //Error message already issued
	}

	updateInteralGlobalShift(globalShiftOptions);

	std::unordered_set<unsigned> verticesIDs;
	//first look for meshes inside loaded DB (so that we don't consider mesh vertices as clouds!)
	{
		ccHObject::Container meshes;
		size_t count = 0;
		//first look for all REAL meshes (so as to no consider sub-meshes)
		if (db->filterChildren(meshes, true, CC_TYPES::MESH, true) != 0)
		{
			count += meshes.size();
			for (size_t i = 0; i < meshes.size(); ++i)
			{
				ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(meshes[i]);
				if (mesh->getParent())
				{
					mesh->getParent()->detachChild(mesh);
				}

				ccGenericPointCloud* vertices = mesh->getAssociatedCloud();
				if (vertices)
				{
					verticesIDs.insert(vertices->getUniqueID());
					print(QString("Found one mesh with %1 faces and %2 vertices: '%3'").arg(mesh->size()).arg(mesh->getAssociatedCloud()->size()).arg(mesh->getName()));
					m_meshes.emplace_back(mesh, filename, count == 1 ? -1 : static_cast<int>(i));
				}
				else
				{
					delete mesh;
					mesh = nullptr;
					assert(false);
				}
			}
		}

		//then look for the other meshes
		meshes.clear();
		if (db->filterChildren(meshes, true, CC_TYPES::MESH, false) != 0)
		{
			size_t countBefore = count;
			count += meshes.size();
			for (size_t i = 0; i < meshes.size(); ++i)
			{
				ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(meshes[i]);
				if (mesh->getParent())
					mesh->getParent()->detachChild(mesh);

				ccGenericPointCloud* vertices = mesh->getAssociatedCloud();
				if (vertices)
				{
					verticesIDs.insert(vertices->getUniqueID());
					print(QString("Found one kind of mesh with %1 faces and %2 vertices: '%3'").arg(mesh->size()).arg(mesh->getAssociatedCloud()->size()).arg(mesh->getName()));
					m_meshes.emplace_back(mesh, filename, count == 1 ? -1 : static_cast<int>(countBefore + i));
				}
				else
				{
					delete mesh;
					mesh = nullptr;
					assert(false);
				}
			}
		}
	}

	//now look for the remaining clouds inside loaded DB
	{
		ccHObject::Container clouds;
		db->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD);
		size_t count = clouds.size();
		for (size_t i = 0; i < count; ++i)
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(clouds[i]);
			if (pc->getParent())
			{
				pc->getParent()->detachChild(pc);
			}

			//if the cloud is a set of vertices, we ignore it!
			if (verticesIDs.find(pc->getUniqueID()) != verticesIDs.end())
			{
				m_orphans.addChild(pc);
				continue;
			}
			print(QString("Found one cloud with %1 points").arg(pc->size()));
			m_clouds.emplace_back(pc, filename, count == 1 ? -1 : static_cast<int>(i));
		}
	}

	delete db;
	db = nullptr;

	return true;
}

bool ccCommandLineParser::saveClouds(QString suffix/*=QString()*/, bool allAtOnce/*=false*/, const QString* allAtOnceFileName/*=nullptr*/)
{
	//all-at-once: all clouds in a single file
	if (allAtOnce)
	{
		FileIOFilter::Shared filter = FileIOFilter::GetFilter(m_cloudExportFormat, false);
		bool multiple = false;
		if (filter)
		{
			bool exclusive = true;
			filter->canSave(CC_TYPES::POINT_CLOUD, multiple, exclusive);
		}

		if (multiple)
		{
			ccHObject tempContainer("Clouds");
			{
				for (CLCloudDesc& desc : m_clouds)
				{
					tempContainer.addChild(desc.getEntity(), ccHObject::DP_NONE);
				}
			}

			//save output
			CLGroupDesc desc(&tempContainer, "AllClouds", m_clouds.front().path);
			if (allAtOnceFileName)
			{
				CommandSave::SetFileDesc(desc, *allAtOnceFileName);
			}

			QString errorStr = exportEntity(desc, suffix, nullptr, ExportOption::ForceCloud);
			if (!errorStr.isEmpty())
				return error(errorStr);
			else
				return true;
		}
		else
		{
			error(QString("The currently selected output format for clouds (%1) doesn't handle multiple entities at once!").arg(m_cloudExportFormat));
			//will proceed with the standard way
		}
	}

	//standard way: one file per cloud
	{
		for (CLCloudDesc& desc : m_clouds)
		{
			//save output
			QString errorStr = exportEntity(desc, suffix);
			if (!errorStr.isEmpty())
				return error(errorStr);
		}
	}

	return true;
}

bool ccCommandLineParser::saveMeshes(QString suffix/*=QString()*/, bool allAtOnce/*=false*/, const QString* allAtOnceFileName/*=nullptr*/)
{
	//all-at-once: all meshes in a single file
	if (allAtOnce)
	{
		FileIOFilter::Shared filter = FileIOFilter::GetFilter(m_meshExportFormat, false);
		bool multiple = false;
		if (filter)
		{
			bool exclusive = true;
			filter->canSave(CC_TYPES::MESH, multiple, exclusive);
		}

		if (multiple)
		{
			ccHObject tempContainer("Meshes");
			{
				for (auto &mesh : m_meshes)
				{
					tempContainer.addChild(mesh.getEntity(), ccHObject::DP_NONE);
				}
			}

			//save output
			CLGroupDesc desc(&tempContainer, "AllMeshes", m_meshes.front().path);
			if (allAtOnceFileName)
			{
				CommandSave::SetFileDesc(desc, *allAtOnceFileName);
			}

			QString errorStr = exportEntity(desc, suffix, nullptr, ExportOption::ForceMesh);
			if (!errorStr.isEmpty())
				return error(errorStr);
			else
				return true;
		}
		else
		{
			error(QString("The currently selected output format for meshes (%1) doesn't handle multiple entities at once!").arg(m_meshExportFormat));
			//will proceed with the standard way
		}
	}

	//standard way: one file per mesh
	for (auto &mesh : m_meshes)
	{
		//save output
		QString errorStr = exportEntity(mesh, suffix);
		if (!errorStr.isEmpty())
			return error(errorStr);
	}

	return true;
}

void ccCommandLineParser::registerBuiltInCommands()
{
	registerCommand(Command::Shared(new CommandDebugCmdLine));
	registerCommand(Command::Shared(new CommandLoad));
	registerCommand(Command::Shared(new CommandLoadCommandFile));
	registerCommand(Command::Shared(new CommandSubsample));
	registerCommand(Command::Shared(new CommandExtractCCs));
	registerCommand(Command::Shared(new CommandCurvature));
	registerCommand(Command::Shared(new CommandApproxDensity));
	registerCommand(Command::Shared(new CommandDensity));
	registerCommand(Command::Shared(new CommandSFGradient));
	registerCommand(Command::Shared(new CommandRoughness));
	registerCommand(Command::Shared(new CommandApplyTransformation));
	registerCommand(Command::Shared(new CommandDropGlobalShift));
	registerCommand(Command::Shared(new CommandFilterBySFValue));
	registerCommand(Command::Shared(new CommandMergeClouds));
	registerCommand(Command::Shared(new CommandMergeMeshes));
	registerCommand(Command::Shared(new CommandSetActiveSF));
	registerCommand(Command::Shared(new CommandSetGlobalShift));
	registerCommand(Command::Shared(new CommandRemoveAllSFs));
	registerCommand(Command::Shared(new CommandRemoveSF));
	registerCommand(Command::Shared(new CommandRemoveRGB));
	registerCommand(Command::Shared(new CommandRemoveNormals));
	registerCommand(Command::Shared(new CommandRemoveScanGrids));
	registerCommand(Command::Shared(new CommandRemoveSensors));
	registerCommand(Command::Shared(new CommandMatchBBCenters));
	registerCommand(Command::Shared(new CommandMatchBestFitPlane));
	registerCommand(Command::Shared(new CommandOrientNormalsMST));
	registerCommand(Command::Shared(new CommandSORFilter));
	registerCommand(Command::Shared(new CommandNoiseFilter));
	registerCommand(Command::Shared(new CommandRemoveDuplicatePoints));
	registerCommand(Command::Shared(new CommandSampleMesh));
	registerCommand(Command::Shared(new CommandCompressFWF));
	registerCommand(Command::Shared(new CommandExtractVertices));
	registerCommand(Command::Shared(new CommandCrossSection));
	registerCommand(Command::Shared(new CommandCrop));
	registerCommand(Command::Shared(new CommandCrop2D));
	registerCommand(Command::Shared(new CommandCoordToSF));
	registerCommand(Command::Shared(new CommandSFToCoord));
	registerCommand(Command::Shared(new CommandColorBanding));
	registerCommand(Command::Shared(new CommandColorLevels));
	registerCommand(Command::Shared(new CommandC2MDist));
	registerCommand(Command::Shared(new CommandC2CDist));
    registerCommand(Command::Shared(new CommandCPS));
	registerCommand(Command::Shared(new CommandStatTest));
	registerCommand(Command::Shared(new CommandDelaunayTri));
	registerCommand(Command::Shared(new CommandSFArithmetic));
	registerCommand(Command::Shared(new CommandSFOperation));
    registerCommand(Command::Shared(new CommandSFOperationSF));
    registerCommand(Command::Shared(new CommandSFInterpolation));
	registerCommand(Command::Shared(new CommandColorInterpolation));
	registerCommand(Command::Shared(new CommandFilter));
	registerCommand(Command::Shared(new CommandRenameEntities));
	registerCommand(Command::Shared(new CommandSFRename));
	registerCommand(Command::Shared(new CommandSFAddConst));
	registerCommand(Command::Shared(new CommandSFAddId));
	registerCommand(Command::Shared(new CommandICP));
	registerCommand(Command::Shared(new CommandChangeCloudOutputFormat));
	registerCommand(Command::Shared(new CommandChangeMeshOutputFormat));
	registerCommand(Command::Shared(new CommandChangeHierarchyOutputFormat));
	registerCommand(Command::Shared(new CommandChangePLYExportFormat));
	registerCommand(Command::Shared(new CommandForceNormalsComputation));
	registerCommand(Command::Shared(new CommandSaveClouds));
	registerCommand(Command::Shared(new CommandSaveMeshes));
	registerCommand(Command::Shared(new CommandAutoSave));
	registerCommand(Command::Shared(new CommandLogFile));
	registerCommand(Command::Shared(new CommandSelectEntities));
	registerCommand(Command::Shared(new CommandClear));
	registerCommand(Command::Shared(new CommandClearClouds));
	registerCommand(Command::Shared(new CommandPopClouds));
	registerCommand(Command::Shared(new CommandClearMeshes));
	registerCommand(Command::Shared(new CommandPopMeshes));
	registerCommand(Command::Shared(new CommandSetNoTimestamp));
	registerCommand(Command::Shared(new CommandVolume25D));
	registerCommand(Command::Shared(new CommandRasterize));
	registerCommand(Command::Shared(new CommandOctreeNormal));
	registerCommand(Command::Shared(new CommandConvertNormalsToDipAndDipDir));
	registerCommand(Command::Shared(new CommandConvertNormalsToSFs));
	registerCommand(Command::Shared(new CommandConvertNormalsToHSV));
	registerCommand(Command::Shared(new CommandClearNormals));
	registerCommand(Command::Shared(new CommandInvertNormal));
	registerCommand(Command::Shared(new CommandComputeMeshVolume));
	registerCommand(Command::Shared(new CommandSFColorScale));
	registerCommand(Command::Shared(new CommandSFConvertToRGB));
	registerCommand(Command::Shared(new CommandMoment));
	registerCommand(Command::Shared(new CommandFeature));
	registerCommand(Command::Shared(new CommandRGBConvertToSF));
	registerCommand(Command::Shared(new CommandFlipTriangles));
	registerCommand(Command::Shared(new CommandSetVerbosity));
}

void ccCommandLineParser::cleanup()
{
	removeClouds();
	removeMeshes();
}

int ccCommandLineParser::start(QDialog* parent/*=nullptr*/)
{
	if (m_arguments.empty())
	{
		assert(false);
		return EXIT_FAILURE;
	}

	m_parentWidget = parent;
	//if (!m_silentMode)
	//{
	//	m_progressDialog = new ccProgressDialog(false, parent);
	//	//m_progressDialog->setAttribute(Qt::WA_DeleteOnClose);
	//	m_progressDialog->setAutoClose(false);
	//	m_progressDialog->hide();
	//}

	QElapsedTimer eTimer;
	eTimer.start();

	bool success = true;
	while (success && !m_arguments.empty())
	{
		QApplication::processEvents();	//Without this the console is just a spinner until the end of all processing
		QString argument = m_arguments.takeFirst();

		if (!argument.startsWith("-"))
		{
			error(QString("Command expected (commands start with '-'). Found '%1'").arg(argument));
			success = false;
			break;
		}
		QString keyword = argument.mid(1).toUpper();

		if (m_commands.contains(keyword))
		{
			assert(m_commands[keyword]);
			QElapsedTimer eTimerSubProcess;
			eTimerSubProcess.start();
			QString processName = m_commands[keyword]->m_name.toUpper();
			printHigh(QString("[%1]").arg(processName));
			success = m_commands[keyword]->process(*this);
			printHigh(QString("[%2] finished in %1 s.").arg(eTimerSubProcess.elapsed() / 1.0e3, 0, 'f', 2).arg(processName));
		}
		//silent mode (i.e. no console)
		else if (keyword == COMMAND_SILENT_MODE)
		{
			warning(QString("Misplaced command: '%1' (must be first)").arg(COMMAND_SILENT_MODE));
		}
		else if (keyword == COMMAND_HELP)
		{
			print("Available commands:");
			for (auto it = m_commands.constBegin(); it != m_commands.constEnd(); ++it)
			{
				print(QString("-%1: %2").arg(it.key().toUpper(), it.value()->m_name));
			}
		}
		else
		{
			error(QString("Unknown or misplaced command: '%1'").arg(argument));
			success = false;
			break;
		}
	}

	print(QString("Processed finished in %1 s.").arg(eTimer.elapsed() / 1.0e3, 0, 'f', 2));

	return success ? EXIT_SUCCESS : EXIT_FAILURE;
}
