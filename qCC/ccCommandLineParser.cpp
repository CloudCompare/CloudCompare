#include "ccCommandLineParser.h"

//Local
#include "ccCommandLineCommands.h"
#include "ccCommandCrossSection.h"

//qCC_db
#include <ccProgressDialog.h>

//qCC_io
#include <AsciiFilter.h>
#include <BinFilter.h>

//qCC
#include "ccConsole.h"

//Qt
#include <QMessageBox>
#include <QElapsedTimer>

//system
#include <unordered_set>

//commands
static const char COMMAND_HELP[]							= "HELP";
static const char COMMAND_SILENT_MODE[]						= "SILENT";
static const char COMMAND_CLOUD_EXPORT_FORMAT[]				= "C_EXPORT_FMT";
static const char COMMAND_EXPORT_EXTENSION[]				= "EXT";
static const char COMMAND_ASCII_EXPORT_PRECISION[]			= "PREC";
static const char COMMAND_ASCII_EXPORT_SEPARATOR[]			= "SEP";
static const char COMMAND_ASCII_EXPORT_ADD_COL_HEADER[]		= "ADD_HEADER";
static const char COMMAND_MESH_EXPORT_FORMAT[]				= "M_EXPORT_FMT";
static const char COMMAND_ASCII_EXPORT_ADD_PTS_COUNT[]		= "ADD_PTS_COUNT";

struct CommandChangeOutputFormat : public ccCommandLineInterface::Command
{
	CommandChangeOutputFormat(QString name, QString keyword) : ccCommandLineInterface::Command(name, keyword) {}

	QString getFileFormatFilter(ccCommandLineInterface& cmd, QString& defaultExt)
	{
		QString fileFilter;
		defaultExt = QString();

		if (!cmd.arguments().isEmpty())
		{
			//test if the specified format corresponds to a known file type
			QString argument = cmd.arguments().front().toUpper();
			cmd.arguments().pop_front();

			const FileIOFilter::FilterContainer& filters = FileIOFilter::GetFilters();
			for (size_t i = 0; i < filters.size(); ++i)
			{
				if (argument == QString(filters[i]->getDefaultExtension()).toUpper())
				{
					//found
					fileFilter = filters[i]->getFileFilters(false).first(); //Take the first 'output' file filter by default (could we be smarter?)
					defaultExt = filters[i]->getDefaultExtension();
					break;
				}
			}

			//haven't found anything?
			if (fileFilter.isEmpty())
			{
				cmd.error(QString("Unhandled format specifier (%1)").arg(argument));
			}
		}
		else
		{
			cmd.error("Missing file format specifier!");
		}

		return fileFilter;
	}
};

struct CommandChangeCloudOutputFormat : public CommandChangeOutputFormat
{
	CommandChangeCloudOutputFormat() : CommandChangeOutputFormat("Change cloud output format", COMMAND_CLOUD_EXPORT_FORMAT) {}

	virtual bool process(ccCommandLineInterface& cmd) override
	{
		QString defaultExt;
		QString fileFilter = getFileFormatFilter(cmd, defaultExt);
		if (fileFilter.isEmpty())
			return false;

		ccCommandLineParser& parserCmd = static_cast<ccCommandLineParser&>(cmd);
		parserCmd.m_cloudExportFormat = fileFilter;
		parserCmd.m_cloudExportExt = defaultExt;

		cmd.print(QString("Output export format (clouds) set to: %1").arg(defaultExt.toUpper()));

		//default options for ASCII output
		if (fileFilter == AsciiFilter::GetFileFilter())
		{
			AsciiSaveDlg* saveDialog = AsciiFilter::GetSaveDialog();
			assert(saveDialog);
			saveDialog->setCoordsPrecision(cmd.numericalPrecision());
			saveDialog->setSfPrecision(cmd.numericalPrecision());
			saveDialog->setSeparatorIndex(0); //space
			saveDialog->enableSwapColorAndSF(false); //default order: point, color, SF, normal
			saveDialog->enableSaveColumnsNamesHeader(false);
			saveDialog->enableSavePointCountHeader(false);
		}

		//look for additional parameters
		while (!cmd.arguments().empty())
		{
			QString argument = cmd.arguments().front();
			if (ccCommandLineInterface::IsCommand(argument, COMMAND_EXPORT_EXTENSION))
			{
				//local option confirmed, we can move on
				cmd.arguments().pop_front();

				if (cmd.arguments().empty())
					return cmd.error(QString("Missing parameter: extension after '%1'").arg(COMMAND_EXPORT_EXTENSION));

				parserCmd.m_cloudExportExt = cmd.arguments().takeFirst();
				cmd.print(QString("New output extension for clouds: %1").arg(parserCmd.m_cloudExportExt));
			}
			else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ASCII_EXPORT_PRECISION))
			{
				//local option confirmed, we can move on
				cmd.arguments().pop_front();

				if (cmd.arguments().empty())
					return cmd.error(QString("Missing parameter: precision value after '%1'").arg(COMMAND_ASCII_EXPORT_PRECISION));
				bool ok;
				int precision = cmd.arguments().takeFirst().toInt(&ok);
				if (!ok || precision < 0)
					return cmd.error(QString("Invalid value for precision! (%1)").arg(COMMAND_ASCII_EXPORT_PRECISION));

				if (fileFilter != AsciiFilter::GetFileFilter())
					cmd.warning(QString("Argument '%1' is only applicable to ASCII format!").arg(argument));

				AsciiSaveDlg* saveDialog = AsciiFilter::GetSaveDialog();
				assert(saveDialog);
				if (saveDialog)
				{
					saveDialog->setCoordsPrecision(precision);
					saveDialog->setSfPrecision(precision);
				}
			}
			else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ASCII_EXPORT_SEPARATOR))
			{
				//local option confirmed, we can move on
				cmd.arguments().pop_front();

				if (cmd.arguments().empty())
					return cmd.error(QString("Missing parameter: separator character after '%1'").arg(COMMAND_ASCII_EXPORT_SEPARATOR));

				if (fileFilter != AsciiFilter::GetFileFilter())
					cmd.warning(QString("Argument '%1' is only applicable to ASCII format!").arg(argument));

				QString separatorStr = cmd.arguments().takeFirst().toUpper();
				//printf("%s\n",qPrintable(separatorStr));
				int index = -1;
				if (separatorStr == "SPACE")
					index = 0;
				else if (separatorStr == "SEMICOLON")
					index = 1;
				else if (separatorStr == "COMMA")
					index = 2;
				else if (separatorStr == "TAB")
					index = 3;
				else
					return cmd.error(QString("Invalid separator! ('%1')").arg(separatorStr));

				AsciiSaveDlg* saveDialog = AsciiFilter::GetSaveDialog();
				assert(saveDialog);
				if (saveDialog)
				{
					saveDialog->setSeparatorIndex(index);
				}
			}
			else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ASCII_EXPORT_ADD_COL_HEADER))
			{
				//local option confirmed, we can move on
				cmd.arguments().pop_front();

				if (fileFilter != AsciiFilter::GetFileFilter())
					cmd.warning(QString("Argument '%1' is only applicable to ASCII format!").arg(argument));

				AsciiSaveDlg* saveDialog = AsciiFilter::GetSaveDialog();
				assert(saveDialog);
				if (saveDialog)
				{
					saveDialog->enableSaveColumnsNamesHeader(true);
				}
			}
			else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ASCII_EXPORT_ADD_PTS_COUNT))
			{
				//local option confirmed, we can move on
				cmd.arguments().pop_front();

				if (fileFilter != AsciiFilter::GetFileFilter())
					cmd.warning(QString("Argument '%1' is only applicable to ASCII format!").arg(argument));

				AsciiSaveDlg* saveDialog = AsciiFilter::GetSaveDialog();
				assert(saveDialog);
				if (saveDialog)
				{
					saveDialog->enableSavePointCountHeader(true);
				}
			}
			else
			{
				break; //as soon as we encounter an unrecognized argument, we break the local loop to go back on the main one!
			}
		}

		return true;
	}
};

struct CommandChangeMeshOutputFormat : public CommandChangeOutputFormat
{
	CommandChangeMeshOutputFormat() : CommandChangeOutputFormat("Change mesh output format", COMMAND_MESH_EXPORT_FORMAT) {}

	virtual bool process(ccCommandLineInterface& cmd) override
	{
		QString defaultExt;
		QString fileFilter = getFileFormatFilter(cmd, defaultExt);
		if (fileFilter.isEmpty())
			return false;

		ccCommandLineParser& parserCmd = static_cast<ccCommandLineParser&>(cmd);
		parserCmd.m_meshExportFormat = fileFilter;
		parserCmd.m_meshExportExt = defaultExt;

		cmd.print(QString("Output export format (meshes) set to: %1").arg(defaultExt.toUpper()));

		//look for additional parameters
		while (!cmd.arguments().empty())
		{
			QString argument = cmd.arguments().front();

			if (ccCommandLineInterface::IsCommand(argument, COMMAND_EXPORT_EXTENSION))
			{
				//local option confirmed, we can move on
				cmd.arguments().pop_front();

				if (cmd.arguments().empty())
					return cmd.error(QString("Missing parameter: extension after '%1'").arg(COMMAND_EXPORT_EXTENSION));

				parserCmd.m_meshExportExt = cmd.arguments().takeFirst();
				cmd.print(QString("New output extension for meshes: %1").arg(parserCmd.m_meshExportExt));
			}
			else
			{
				break; //as soon as we encounter an unrecognized argument, we break the local loop to go back on the main one!
			}
		}

		return true;
	}
};

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

int ccCommandLineParser::Parse(int nargs, char** args, tPluginInfoList* plugins/*=0*/)
{
	if (!args || nargs < 2)
	{
		assert(false);
		return EXIT_SUCCESS;
	}

	//load arguments
	QScopedPointer<ccCommandLineParser> parser(new ccCommandLineParser);
	{
		for (int i = 1; i < nargs; ++i) //'i=1' because first argument is always program executable file!
		{
			parser->arguments().push_back(QString(args[i]));
		}
	}
	assert(!parser->arguments().empty());

	//specific command: silent mode (will prevent the console dialog from appearing!
	if (ccCommandLineInterface::IsCommand(parser->arguments().front(), COMMAND_SILENT_MODE))
	{
		parser->arguments().pop_front();
		parser->toggleSilentMode(true);
	}

	QScopedPointer<QDialog> consoleDlg(0);
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
	if (plugins)
	{
		for (tPluginInfo pluginInfo : *plugins)
		{
			if (!pluginInfo.object)
			{
				assert(false);
				continue;
			}
			ccPluginInterface* plugin = static_cast<ccPluginInterface*>(pluginInfo.object);
			plugin->registerCommands(parser.data());
		}
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
	, m_progressDialog(0)
	, m_parentWidget(0)
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
	registerCommand(Command::Shared(new CommandSetActiveSF));
	registerCommand(Command::Shared(new CommandRemoveAllSF));
	registerCommand(Command::Shared(new CommandMatchBBCenters));
	registerCommand(Command::Shared(new CommandMatchBestFitPlane));
	registerCommand(Command::Shared(new CommandOrientNormalsMST));
	registerCommand(Command::Shared(new CommandSORFilter));
	registerCommand(Command::Shared(new CommandSampleMesh));
	registerCommand(Command::Shared(new CommandCrossSection));
	registerCommand(Command::Shared(new CommandCrop));
	registerCommand(Command::Shared(new CommandCrop2D));
	registerCommand(Command::Shared(new CommandColorBanding));
	registerCommand(Command::Shared(new CommandBundler));
	registerCommand(Command::Shared(new CommandC2MDist));
	registerCommand(Command::Shared(new CommandC2CDist));
	registerCommand(Command::Shared(new CommandStatTest));
	registerCommand(Command::Shared(new CommandDelaunayTri));
	registerCommand(Command::Shared(new CommandSFArithmetic));
	registerCommand(Command::Shared(new CommandSFOperation));
	registerCommand(Command::Shared(new CommandICP));
	registerCommand(Command::Shared(new CommandChangeCloudOutputFormat));
	registerCommand(Command::Shared(new CommandChangeMeshOutputFormat));
	registerCommand(Command::Shared(new CommandChangeFBXOutputFormat));
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
	//registerCommand(Command::Shared(new XXX));
	//registerCommand(Command::Shared(new XXX));
	//registerCommand(Command::Shared(new XXX));
	//registerCommand(Command::Shared(new XXX));
	//registerCommand(Command::Shared(new XXX));
}

ccCommandLineParser::~ccCommandLineParser()
{
	removeClouds();
	removeMeshes();

	if (m_progressDialog)
	{
		m_progressDialog->close();
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
		warning(QString("Internal error: keyword '%' already registered (by command '%2')").arg(command->m_keyword).arg(m_commands[command->m_keyword]->m_name));
		return false;
	}

	m_commands.insert(command->m_keyword, command);

	return true;
}

QString ccCommandLineParser::exportEntity(	CLEntityDesc& entityDesc,
											QString suffix/*=QString()*/,
											QString* _outputFilename/*=0*/,
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

	//get its name
	QString entName = entity->getName();
	if (entName.isEmpty())
		entName = entityDesc.basename;

	//sub-item?
	if (entityDesc.indexInFile >= 0)
	{
		if (suffix.isEmpty())
			suffix = QString("%1").arg(entityDesc.indexInFile);
		else
			suffix.prepend(QString("%1_").arg(entityDesc.indexInFile));
	}

	//specific case: clouds
	bool isCloud = entity->isA(CC_TYPES::POINT_CLOUD);
	isCloud |= forceIsCloud;

	if (!suffix.isEmpty())
		entName += QString("_") + suffix;
	entity->setName(entName);

	QString baseName = entityDesc.basename;
	if (!suffix.isEmpty())
		baseName += QString("_") + suffix;

	QString outputFilename = baseName;
	if (m_addTimestamp && !forceNoTimestamp)
		outputFilename += QString("_%1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd_hh'h'mm_ss"));
	QString extension = isCloud ? m_cloudExportExt : m_meshExportExt;
	if (!extension.isEmpty())
		outputFilename += QString(".%1").arg(extension);

	if (_outputFilename)
		*_outputFilename = outputFilename;

	if (!entityDesc.path.isEmpty())
		outputFilename.prepend(QString("%1/").arg(entityDesc.path));

	bool tempDependencyCreated = false;
	ccGenericMesh* mesh = 0;
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

	CC_FILE_ERROR result = FileIOFilter::SaveToFile(entity,
		qPrintable(outputFilename),
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
		if (m_clouds.back().pc)
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
		if (desc.mesh)
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
	ccHObject* db = 0;
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
					m_meshes.push_back(CLMeshDesc(mesh, filename, count == 1 ? -1 : static_cast<int>(i)));
				}
				else
				{
					delete mesh;
					mesh = 0;
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
					m_meshes.push_back(CLMeshDesc(mesh, filename, count == 1 ? -1 : static_cast<int>(countBefore + i)));
				}
				else
				{
					delete mesh;
					mesh = 0;
					assert(false);
				}
			}
		}
	}

	//now look for the remaining clouds inside loaded DB
	{
		ccHObject::Container clouds;
		db->filterChildren(clouds, false, CC_TYPES::POINT_CLOUD);
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
			m_clouds.push_back(CLCloudDesc(pc, filename, count == 1 ? -1 : static_cast<int>(i)));
		}
	}

	delete db;
	db = 0;

	return true;
}

bool ccCommandLineParser::saveClouds(QString suffix/*=QString()*/, bool allAtOnce/*=false*/)
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
				for (size_t i = 0; i < m_clouds.size(); ++i)
				{
					tempContainer.addChild(m_clouds[i].getEntity(), ccHObject::DP_NONE);
				}
			}

			//save output
			CLGroupDesc desc(&tempContainer, "AllClouds", m_clouds.front().path);
			QString errorStr = exportEntity(desc, suffix, 0, true);
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
		for (size_t i = 0; i < m_clouds.size(); ++i)
		{
			//save output
			QString errorStr = exportEntity(m_clouds[i], suffix);
			if (!errorStr.isEmpty())
				return error(errorStr);
		}
	}

	return true;
}

bool ccCommandLineParser::saveMeshes(QString suffix/*=QString()*/, bool allAtOnce/*=false*/)
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
				for (size_t i = 0; i < m_meshes.size(); ++i)
					tempContainer.addChild(m_meshes[i].getEntity(), ccHObject::DP_NONE);
			}
			//save output
			CLGroupDesc desc(&tempContainer, "AllMeshes", m_meshes.front().path);
			QString errorStr = exportEntity(desc, suffix, 0, false);
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
	{
		for (size_t i = 0; i < m_meshes.size(); ++i)
		{
			//save output
			QString errorStr = exportEntity(m_meshes[i], suffix);
			if (!errorStr.isEmpty())
				return error(errorStr);
		}
	}

	return true;
}

int ccCommandLineParser::start(QDialog* parent/*=0*/)
{
	if (m_arguments.empty())
	{
		assert(false);
		return EXIT_FAILURE;
	}

	m_parentWidget = parent;
	if (!m_silentMode)
	{
		m_progressDialog = new ccProgressDialog(false, parent);
		m_progressDialog->setAttribute(Qt::WA_DeleteOnClose);
		m_progressDialog->setAutoClose(false);
	}

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
				print(QString("-%1: %2").arg(it.key().toUpper()).arg(it.value()->m_name));
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
