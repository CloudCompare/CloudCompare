#include "ccCommandLineParser.h"

//Local
#include "ccCommandCrossSection.h"
#include "ccCommandLineCommands.h"
#include "ccCommandRaster.h"
#include "ccPluginInterface.h"

//qCC_db
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

void ccCommandLineParser::print(const QString& message) const
{
	ccConsole::Print(message);
	if (m_silentMode)
	{
		printf("%s\n", qPrintable(message));
	}
}

void ccCommandLineParser::warning(const QString& message) const
{
	ccConsole::Warning(message);
	if (m_silentMode)
	{
		printf("[WARNING] %s\n", qPrintable(message));
	}
}

bool ccCommandLineParser::error(const QString& message) const
{
	ccConsole::Error(message);
	if (m_silentMode)
	{
		printf("[ERROR] %s\n", qPrintable(message));
	}

	return false;
}

int ccCommandLineParser::Parse(int nargs, char** args, ccPluginInterfaceList& plugins)
{
	if (!args || nargs < 2)
	{
		assert(false);
		return EXIT_SUCCESS;
	}

	//load arguments
	QScopedPointer<ccCommandLineParser> parser(new ccCommandLineParser);
	
	parser->registerBuiltInCommands();
	
	for (int i = 1; i < nargs; ++i) //'i=1' because first argument is always program executable file!
	{
		parser->arguments().push_back(QString(args[i]));
	}
	
	assert(!parser->arguments().empty());

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
		warning(QString("Internal error: keyword '%' already registered (by command '%2')").arg(command->m_keyword, m_commands[command->m_keyword]->m_name));
		return false;
	}

	m_commands.insert(command->m_keyword, command);

	return true;
}

QString ccCommandLineParser::getExportFilename(	const CLEntityDesc& entityDesc,
												QString extension/*=QString()*/,
												QString suffix/*=QString()*/,
												QString* baseOutputFilename/*=0*/,
												bool forceNoTimestamp/*=false*/) const
{
	//fetch the real entity
	const ccHObject* entity = entityDesc.getEntity();
	if (!entity)
	{
		assert(false);
		warning("[ExportEntity] Internal error: invalid input entity!");
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
		outputFilename += QString(".%1").arg(extension);
	}

	if (baseOutputFilename)
	{
		*baseOutputFilename = outputFilename;
	}

	if (!entityDesc.path.isEmpty())
	{
		outputFilename.prepend(QString("%1/").arg(entityDesc.path));
	}

	return outputFilename;

}

QString ccCommandLineParser::exportEntity(	CLEntityDesc& entityDesc,
											QString suffix/*=QString()*/,
											QString* baseOutputFilename/*=0*/,
											bool forceIsCloud/*=false*/,
											bool forceNoTimestamp/*=false*/)
{
	print("[SAVING]");

	//fetch the real entity
	ccHObject* entity = entityDesc.getEntity();
	if (!entity)
	{
		assert(false);
		return "[ExportEntity] Internal error: invalid input entity!";
	}

	//specific case: clouds
	bool isCloud = entity->isA(CC_TYPES::POINT_CLOUD);
	isCloud |= forceIsCloud;
	QString extension = isCloud ? m_cloudExportExt : m_meshExportExt;

	QString outputFilename = getExportFilename(entityDesc, extension, suffix, baseOutputFilename, forceNoTimestamp);
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
													isCloud ? m_cloudExportFormat : m_meshExportFormat);

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
		CLMeshDesc& desc = m_meshes.back();
		delete desc.mesh;
		m_meshes.pop_back();
		if (onlyLast)
			break;
	}
}

bool ccCommandLineParser::importFile(QString filename, FileIOFilter::Shared filter)
{
	print(QString("Opening file: '%1'").arg(filename));

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

bool ccCommandLineParser::saveClouds(QString suffix/*=QString()*/, bool allAtOnce/*=false*/, const QString* allAtOnceFileName/*=0*/)
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

			QString errorStr = exportEntity(desc, suffix, nullptr, true);
			if (!errorStr.isEmpty())
				return error(errorStr);
			else
				return true;
		}
		else
		{
			error(QString("The currently selected ouput format for clouds (%1) doesn't handle multiple entities at once!").arg(m_cloudExportFormat));
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

bool ccCommandLineParser::saveMeshes(QString suffix/*=QString()*/, bool allAtOnce/*=false*/, const QString* allAtOnceFileName/*=0*/)
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

			QString errorStr = exportEntity(desc, suffix, nullptr, false);
			if (!errorStr.isEmpty())
				return error(errorStr);
			else
				return true;
		}
		else
		{
			error(QString("The currently selected ouput format for meshes (%1) doesn't handle multiple entities at once!").arg(m_meshExportFormat));
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
	registerCommand(Command::Shared(new CommandLoad));
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
	registerCommand(Command::Shared(new CommandRemoveAllSF));
	registerCommand(Command::Shared(new CommandRemoveScanGrids));
	registerCommand(Command::Shared(new CommandMatchBBCenters));
	registerCommand(Command::Shared(new CommandMatchBestFitPlane));
	registerCommand(Command::Shared(new CommandOrientNormalsMST));
	registerCommand(Command::Shared(new CommandSORFilter));
	registerCommand(Command::Shared(new CommandSampleMesh));
	registerCommand(Command::Shared(new CommandExtractVertices));
	registerCommand(Command::Shared(new CommandCrossSection));
	registerCommand(Command::Shared(new CommandCrop));
	registerCommand(Command::Shared(new CommandCrop2D));
	registerCommand(Command::Shared(new CommandCoordToSF));
	registerCommand(Command::Shared(new CommandColorBanding));
	registerCommand(Command::Shared(new CommandC2MDist));
	registerCommand(Command::Shared(new CommandC2CDist));
	registerCommand(Command::Shared(new CommandStatTest));
	registerCommand(Command::Shared(new CommandDelaunayTri));
	registerCommand(Command::Shared(new CommandSFArithmetic));
	registerCommand(Command::Shared(new CommandSFOperation));
	registerCommand(Command::Shared(new CommandICP));
	registerCommand(Command::Shared(new CommandChangeCloudOutputFormat));
	registerCommand(Command::Shared(new CommandChangeMeshOutputFormat));
	registerCommand(Command::Shared(new CommandChangePLYExportFormat));
	registerCommand(Command::Shared(new CommandForceNormalsComputation));
	registerCommand(Command::Shared(new CommandSaveClouds));
	registerCommand(Command::Shared(new CommandSaveMeshes));
	registerCommand(Command::Shared(new CommandAutoSave));
	registerCommand(Command::Shared(new CommandLogFile));
	registerCommand(Command::Shared(new CommandClear));
	registerCommand(Command::Shared(new CommandClearClouds));
	registerCommand(Command::Shared(new CommandPopClouds));
	registerCommand(Command::Shared(new CommandClearMeshes));
	registerCommand(Command::Shared(new CommandPopMeshes));
	registerCommand(Command::Shared(new CommandSetNoTimestamp));
	registerCommand(Command::Shared(new CommandVolume25D));
	registerCommand(Command::Shared(new CommandRasterize));
	registerCommand(Command::Shared(new CommandOctreeNormal));
	registerCommand(Command::Shared(new CommandClearNormals));
	registerCommand(Command::Shared(new CommandComputeMeshVolume));
	registerCommand(Command::Shared(new CommandSFColorScale));
	registerCommand(Command::Shared(new CommandSFConvertToRGB));
}

void ccCommandLineParser::cleanup()
{
	removeClouds();
	removeMeshes();
}

int ccCommandLineParser::start(QDialog* parent/*=0*/)
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
			success = m_commands[keyword]->process(*this);
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
