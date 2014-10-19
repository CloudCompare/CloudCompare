#include "ccCommandLineParser.h"

//CCLib
#include <CloudSamplingTools.h>
#include <WeibullDistribution.h>
#include <NormalDistribution.h>
#include <StatisticalTestingTools.h>
#include <Neighbourhood.h>

//qCC_db
#include <ccProgressDialog.h>
#include <ccOctree.h>
#include <ccPlane.h>
#include <ccNormalVectors.h>
#include <ccPolyline.h>
#include <ccScalarField.h>

//qCC_io
#include <BundlerFilter.h>
#include <AsciiFilter.h>
#include <FBXFilter.h>
#include <PTXFilter.h>
#include <BinFilter.h>

//qCC
#include "ccCommon.h"
#include <ui_commandLineDlg.h>
#include "ccConsole.h"
#include "mainwindow.h"
#include "ccComparisonDlg.h"
#include "ccRegistrationTools.h"

//Qt
#include <QMessageBox>
#include <QDialog>
#include <QFile>
#include <QFileInfo>
#include <QDateTime>
#include <QElapsedTimer>
#include <QStringList>
#include <QTextStream>

static const char COMMAND_SILENT_MODE[]						= "SILENT";
static const char COMMAND_OPEN[]							= "O";				//+file name
static const char COMMAND_OPEN_SKIP_LINES[]					= "SKIP";			//+number of lines to skip
static const char COMMAND_OPEN_SHIFT_ON_LOAD[]				= "GLOBAL_SHIFT";	//+global shift
static const char COMMAND_KEYWORD_AUTO[]					= "AUTO";			//"AUTO" keyword
static const char COMMAND_SUBSAMPLE[]						= "SS";				//+ method (RANDOM/SPATIAL/OCTREE) + parameter (resp. point count / spatial step / octree level)
static const char COMMAND_CURVATURE[]						= "CURV";			//+ curvature type (MEAN/GAUSS) +
static const char COMMAND_DENSITY[]							= "DENSITY";		//+ sphere radius
static const char COMMAND_DENSITY_TYPE[]					= "TYPE";			//+ density type
static const char COMMAND_APPROX_DENSITY[]					= "APPROX_DENSITY";
static const char COMMAND_SF_GRADIENT[]						= "SF_GRAD";
static const char COMMAND_ROUGHNESS[]						= "ROUGH";
static const char COMMAND_BUNDLER[]							= "BUNDLER_IMPORT"; //Import Bundler file + orthorectification
static const char COMMAND_BUNDLER_ALT_KEYPOINTS[]			= "ALT_KEYPOINTS";
static const char COMMAND_BUNDLER_SCALE_FACTOR[]			= "SCALE_FACTOR";
static const char COMMAND_BUNDLER_UNDISTORT[]				= "UNDISTORT";
static const char COMMAND_BUNDLER_COLOR_DTM[]				= "COLOR_DTM";
static const char COMMAND_C2M_DIST[]						= "C2M_DIST";
static const char COMMAND_C2M_DIST_FLIP_NORMALS[]			= "FLIP_NORMS";
static const char COMMAND_C2C_DIST[]						= "C2C_DIST";
static const char COMMAND_C2C_SPLIT_XYZ[]					= "SPLIT_XYZ";
static const char COMMAND_C2C_LOCAL_MODEL[]					= "MODEL";
static const char COMMAND_MAX_DISTANCE[]					= "MAX_DIST";
static const char COMMAND_OCTREE_LEVEL[]					= "OCTREE_LEVEL";
static const char COMMAND_SAMPLE_MESH[]						= "SAMPLE_MESH";
static const char COMMAND_MERGE_CLOUDS[]					= "MERGE_CLOUDS";
static const char COMMAND_STAT_TEST[]						= "STAT_TEST";
static const char COMMAND_FILTER_SF_BY_VALUE[]				= "FILTER_SF";
static const char COMMAND_CLEAR_CLOUDS[]					= "CLEAR_CLOUDS";
static const char COMMAND_CLEAR_MESHES[]					= "CLEAR_MESHES";
static const char COMMAND_CLEAR[]							= "CLEAR";
static const char COMMAND_BEST_FIT_PLANE[]					= "BEST_FIT_PLANE";
static const char COMMAND_BEST_FIT_PLANE_MAKE_HORIZ[]		= "MAKE_HORIZ";
static const char COMMAND_BEST_FIT_PLANE_KEEP_LOADED[]		= "KEEP_LOADED";
static const char COMMAND_MATCH_BB_CENTERS[]				= "MATCH_CENTERS";
static const char COMMAND_ICP[]								= "ICP";
static const char COMMAND_ICP_REFERENCE_IS_FIRST[]			= "REFERENCE_IS_FIRST";
static const char COMMAND_ICP_MIN_ERROR_DIIF[]				= "MIN_ERROR_DIFF";
static const char COMMAND_ICP_ITERATION_COUNT[]				= "ITER";
static const char COMMAND_ICP_ADJUST_SCALE[]				= "ADJUST_SCALE";
static const char COMMAND_ICP_RANDOM_SAMPLING_LIMIT[]		= "RANDOM_SAMPLING_LIMIT";
static const char COMMAND_ICP_ENABLE_FARTHEST_REMOVAL[]		= "FARTHEST_REMOVAL";
static const char COMMAND_CLOUD_EXPORT_FORMAT[]				= "C_EXPORT_FMT";
static const char COMMAND_ASCII_EXPORT_PRECISION[]			= "PREC";
static const char COMMAND_ASCII_EXPORT_SEPARATOR[]			= "SEP";
static const char COMMAND_FBX_EXPORT_FORMAT[]				= "FBX_EXPORT_FMT";
static const char COMMAND_MESH_EXPORT_FORMAT[]				= "M_EXPORT_FMT";
static const char COMMAND_EXPORT_EXTENSION[]				= "EXT";
static const char COMMAND_NO_TIMESTAMP[]					= "NO_TIMESTAMP";
static const char COMMAND_CROP[]							= "CROP";
static const char COMMAND_CROP_2D[]							= "CROP2D";
static const char COMMAND_CROP_OUTSIDE[]					= "OUTSIDE";
static const char COMMAND_SAVE_CLOUDS[]						= "SAVE_CLOUDS";
static const char COMMAND_SAVE_MESHES[]						= "SAVE_MESHES";
static const char COMMAND_AUTO_SAVE[]						= "AUTO_SAVE";
static const char COMMAND_SET_ACTIVE_SF[]					= "SET_ACTIVE_SF";
static const char COMMAND_PTX_COMPUTE_NORMALS[]				= "COMPUTE_PTX_NORMALS";

static const char OPTION_ALL_AT_ONCE[]						= "ALL_AT_ONCE";
static const char OPTION_ON[]								= "ON";
static const char OPTION_OFF[]								= "OFF";

//Current cloud(s) export format (can be modified with the 'COMMAND_CLOUD_EXPORT_FORMAT' option)
static QString s_CloudExportFormat(BinFilter::GetFileFilter());
//Current cloud(s) export extension (warning: can be anything)
static QString s_CloudExportExt(BinFilter::GetDefaultExtension());
//Current mesh(es) export format (can be modified with the 'COMMAND_MESH_EXPORT_FORMAT' option)
static QString s_MeshExportFormat(BinFilter::GetFileFilter());
//Current mesh(es) export extension (warning: can be anything)
static QString s_MeshExportExt(BinFilter::GetDefaultExtension());
//Default numerical precision for ASCII output
static int s_precision = 12;
//Whether a timestamp should be automatically added to output files or not
static bool s_addTimestamp = true;
//Whether silent mode is activated or not
static bool s_silentMode = false;
//Whether files should be automatically saved (after each process) or not
static bool s_autoSaveMode = true;

//Loading parameters
struct CmdLineLoadParameters : public FileIOFilter::LoadParameters
{
	CmdLineLoadParameters()
		: FileIOFilter::LoadParameters()
		, m_coordinatesShiftEnabled(false)
		, m_coordinatesShift(0,0,0)
	{
		shiftHandlingMode = ccGlobalShiftManager::NO_DIALOG;
		alwaysDisplayLoadDialog = false;
		coordinatesShiftEnabled = &m_coordinatesShiftEnabled;
		coordinatesShift = &m_coordinatesShift;
	}

	bool m_coordinatesShiftEnabled;
	CCVector3d m_coordinatesShift;
};
static CmdLineLoadParameters s_loadParameters;


bool IsCommand(const QString& token, const char* command)
{
	return token.startsWith("-") && token.mid(1).toUpper() == QString(command);
}

int ccCommandLineParser::Parse(int nargs, char** args)
{
	if (!args || nargs < 2)
	{
		assert(false);
		return EXIT_SUCCESS;
	}

	//reset default behavior(s)
	PTXFilter::SetNormalsComputationBehavior(PTXFilter::NEVER);
	s_MeshExportFormat = s_CloudExportFormat = BinFilter::GetFileFilter();
	s_MeshExportExt = s_CloudExportExt = BinFilter::GetDefaultExtension();
	s_precision = 12;
	s_addTimestamp = true;
	s_silentMode = false;
	s_autoSaveMode = true;

	//load arguments
	QStringList arguments;
	{
		for (int i=1; i<nargs; ++i) //'i=1' because first argument is always program executable file!
			arguments.push_back(QString(args[i]));
	}
	assert(!arguments.empty());

	//specific command: silent mode (will prevent the console dialog from appearing!
	if (IsCommand(arguments.front(),COMMAND_SILENT_MODE))
	{
		arguments.pop_front();
		s_silentMode = true;
	}

	QDialog consoleDlg;
	if (!s_silentMode)
	{
		//show console
		Ui_commandLineDlg commandLineDlg;
		commandLineDlg.setupUi(&consoleDlg);
		consoleDlg.show();
		ccConsole::Init(commandLineDlg.consoleWidget,&consoleDlg);
	}

	//parse input
	int result = ccCommandLineParser().parse(arguments,&consoleDlg);

	if (!s_silentMode)
	{
		if (result == EXIT_SUCCESS)
			QMessageBox::information(&consoleDlg,"Processed finished","Job done");
		else
			QMessageBox::warning(&consoleDlg,"Processed finished","An error occurred! Check console");
	}

	ccConsole::ReleaseInstance();

	return result;
}

ccCommandLineParser::ccCommandLineParser()
{
}

ccCommandLineParser::~ccCommandLineParser()
{
	removeClouds();
	removeMeshes();
}

static void Print(const QString& message)
{
	ccConsole::Print(message);
	if (s_silentMode)
		printf("%s\n",qPrintable(message));
}

static bool Error(const QString& message)
{
	ccConsole::Error(message);
	if (s_silentMode)
		printf("[ERROR] %s\n",qPrintable(message));

	return false;
}

void ccCommandLineParser::removeClouds()
{
	while (!m_clouds.empty())
	{
		if (m_clouds.back().pc)
			delete m_clouds.back().pc;
		m_clouds.pop_back();
	}
}

void ccCommandLineParser::removeMeshes()
{
	while (!m_meshes.empty())
	{
		if (m_meshes.back().mesh)
			delete m_meshes.back().mesh;
		m_meshes.pop_back();
	}
}

bool ccCommandLineParser::saveClouds(QString suffix/*=QString()*/, bool allAtOnce/*=false*/)
{
	//all-at-once: all clouds in a single file
	if (allAtOnce)
	{
		FileIOFilter::Shared filter = FileIOFilter::GetFilter(s_CloudExportFormat,false);
		bool multiple = false, exclusive = true;
		if (filter)
			filter->canSave(CC_TYPES::POINT_CLOUD,multiple,exclusive);
		
		if (multiple)
		{
			ccHObject tempContainer("Clouds");
			{
				for (unsigned i=0; i<m_clouds.size(); ++i)
					tempContainer.addChild(m_clouds[i].getEntity(),ccHObject::DP_NONE);
			}
			//save output
			GroupDesc desc(&tempContainer,"AllClouds",m_clouds.front().path);
			QString errorStr = Export(desc,suffix,0,true);
			if (!errorStr.isEmpty())
				return Error(errorStr);
			else
				return true;
		}
		else
		{
			ccConsole::Error(QString("The currently selected ouput format for clouds (%1) doesn't handle multiple entities at once!").arg(s_CloudExportFormat));
			//will proceed with the standard way
		}
	}

	//standard way: one file per cloud
	{
		for (unsigned i=0; i<m_clouds.size(); ++i)
		{
			//save output
			QString errorStr = Export(m_clouds[i],suffix);
			if (!errorStr.isEmpty())
				return Error(errorStr);
		}
	}

	return true;
}

bool ccCommandLineParser::saveMeshes(QString suffix/*=QString()*/, bool allAtOnce/*=false*/)
{
	//all-at-once: all meshes in a single file
	if (allAtOnce)
	{
		FileIOFilter::Shared filter = FileIOFilter::GetFilter(s_MeshExportFormat,false);
		bool multiple = false, exclusive = true;
		if (filter)
			filter->canSave(CC_TYPES::MESH,multiple,exclusive);
		
		if (multiple)
		{
			ccHObject tempContainer("Meshes");
			{
				for (unsigned i=0; i<m_meshes.size(); ++i)
					tempContainer.addChild(m_meshes[i].getEntity(),ccHObject::DP_NONE);
			}
			//save output
			GroupDesc desc(&tempContainer,"AllMeshes",m_meshes.front().path);
			QString errorStr = Export(desc,suffix,0,false);
			if (!errorStr.isEmpty())
				return Error(errorStr);
			else
				return true;
		}
		else
		{
			ccConsole::Error(QString("The currently selected ouput format for meshes (%1) doesn't handle multiple entities at once!").arg(s_MeshExportFormat));
			//will proceed with the standard way
		}
	}

	//standard way: one file per mesh
	{
		for (unsigned i=0; i<m_meshes.size(); ++i)
		{
			//save output
			QString errorStr = Export(m_meshes[i],suffix);
			if (!errorStr.isEmpty())
				return Error(errorStr);
		}
	}

	return true;
}

ccCommandLineParser::EntityDesc::EntityDesc(QString filename)
{
	if (filename.isNull())
	{
		basename = "unknown";
		path = QApplication::applicationDirPath();
	}
	else
	{
		QFileInfo fi(filename);
		basename = fi.baseName();
		path = fi.path();
	}
}

ccCommandLineParser::EntityDesc::EntityDesc(QString _basename, QString _path)
	: basename(_basename)
	, path(_path)
{
}

QString ccCommandLineParser::Export(EntityDesc& entDesc, QString suffix/*=QString()*/, QString* _outputFilename/*=0*/, bool forceIsCloud/*=false*/)
{
	Print("[SAVING]");

	//fetch the real entity
	ccHObject* entity = entDesc.getEntity();
	if (!entity)
	{
		assert(false);
		return QString("[Export] Internal error: invalid input entity!");
	}

	//get its name
	QString entName = entity->getName();
	if (entName.isEmpty())
		entName = entDesc.basename;

	//specific case: clouds
	bool isCloud = entity->isA(CC_TYPES::POINT_CLOUD);
	if (isCloud)
	{
		CloudDesc& cloudDesc = static_cast<CloudDesc&>(entDesc);
		if (cloudDesc.indexInFile >= 0)
		{
			if (suffix.isEmpty())
				suffix = QString("%1").arg(cloudDesc.indexInFile);
			else
				suffix.prepend(QString("%1_").arg(cloudDesc.indexInFile));
		}
	}
	isCloud |= forceIsCloud; //don't force this before this point (static cast to CloudDesc above!)

	if (!suffix.isEmpty())
		entName += QString("_") + suffix;
	entity->setName(entName);

	QString baseName = entDesc.basename;
	if (!suffix.isEmpty())
		baseName += QString("_") + suffix;

	QString outputFilename = baseName;
	if (s_addTimestamp)
		outputFilename += QString("_%1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd_hh'h'mm_ss"));
	QString extension = isCloud ? s_CloudExportExt : s_MeshExportExt;
	if (!extension.isEmpty())
		outputFilename += QString(".%1").arg(extension);

	if (_outputFilename)
		*_outputFilename = outputFilename;

	if (!entDesc.path.isEmpty())
		outputFilename.prepend(QString("%1/").arg(entDesc.path));

	//save file
	if (FileIOFilter::SaveToFile(	entity,
									qPrintable(outputFilename),
									isCloud ? s_CloudExportFormat : s_MeshExportFormat) != CC_FERR_NO_ERROR)
	{
		return QString("Failed to save result in file '%1'").arg(outputFilename);
	}

	return QString();
}

bool ccCommandLineParser::commandLoad(QStringList& arguments)
{
	Print("[LOADING]");
	if (arguments.empty())
		return Error(QString("Missing parameter: filename after \"-%1\"").arg(COMMAND_OPEN));

	//optional parameters
	int skipLines = 0;
	while (!arguments.empty())
	{
		QString argument = arguments.front();
		if (IsCommand(argument,COMMAND_OPEN_SKIP_LINES))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			if (arguments.empty())
				return Error(QString(QString("Missing parameter: number of lines after '%1'").arg(COMMAND_OPEN_SKIP_LINES)));

			bool ok;
			skipLines = arguments.takeFirst().toInt(&ok);
			if (!ok)
				return Error(QString(QString("Invalid parameter: number of lines after '%1'").arg(COMMAND_OPEN_SKIP_LINES)));
			
			Print(QString("Will skip %1 lines").arg(skipLines));
		}
		else if (IsCommand(argument,COMMAND_OPEN_SHIFT_ON_LOAD))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			if (arguments.empty())
				return Error(QString(QString("Missing parameter: global shift vector or %1 after '%2'").arg(COMMAND_KEYWORD_AUTO).arg(COMMAND_OPEN_SHIFT_ON_LOAD)));

			QString firstParam = arguments.takeFirst();

			s_loadParameters.shiftHandlingMode = ccGlobalShiftManager::NO_DIALOG;
			s_loadParameters.m_coordinatesShiftEnabled = false;
			s_loadParameters.m_coordinatesShift = CCVector3d(0,0,0);
			
			if (firstParam.toUpper() == COMMAND_KEYWORD_AUTO)
			{
				//let CC handles the global shift automatically
				s_loadParameters.shiftHandlingMode = ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT;
			}
			else if (arguments.size() < 2)
			{
				return Error(QString(QString("Missing parameter: global shift vector after '%1' (3 values expected)").arg(COMMAND_OPEN_SHIFT_ON_LOAD)));
			}
			else
			{
				bool ok = true;
				CCVector3d shiftOnLoadVec;
				shiftOnLoadVec.x = firstParam.toDouble(&ok);
				if (!ok)
					return Error(QString(QString("Invalid parameter: X coordinate of the global shift vector after '%1'").arg(COMMAND_OPEN_SKIP_LINES)));
				shiftOnLoadVec.y = arguments.takeFirst().toDouble(&ok);
				if (!ok)
					return Error(QString(QString("Invalid parameter: Y coordinate of the global shift vector after '%1'").arg(COMMAND_OPEN_SKIP_LINES)));
				shiftOnLoadVec.z = arguments.takeFirst().toDouble(&ok);
				if (!ok)
					return Error(QString(QString("Invalid parameter: Z coordinate of the global shift vector after '%1'").arg(COMMAND_OPEN_SKIP_LINES)));

				//set the user defined shift vector as default shift information
				s_loadParameters.m_coordinatesShiftEnabled = true;
				s_loadParameters.m_coordinatesShift = shiftOnLoadVec;
			}
		}
		else
		{
			break;
		}
	}

	if (skipLines > 0)
	{
		QSharedPointer<AsciiOpenDlg> openDialog = AsciiFilter::GetOpenDialog();
		assert(openDialog);
		openDialog->setSkippedLines(skipLines);
	}

	//open specified file
	QString filename(arguments.takeFirst());
	Print(QString("Opening file: '%1'").arg(filename));

	ccHObject* db = FileIOFilter::LoadFromFile(filename,s_loadParameters,QString());
	if (!db)
		return false/*Error(QString("Failed to open file '%1'").arg(filename))*/;

	//look for clouds inside loaded DB
	ccHObject::Container clouds;
	db->filterChildren(clouds,false,CC_TYPES::POINT_CLOUD);
	size_t count = clouds.size();
	for (size_t i=0; i<count; ++i)
	{
		ccPointCloud* pc = static_cast<ccPointCloud*>(clouds[i]);
		db->detachChild(pc);
		Print(QString("Found one cloud with %1 points").arg(pc->size()));
		m_clouds.push_back(CloudDesc(pc,filename,count == 1 ? -1 : static_cast<int>(i)));
	}

	//look for meshes inside loaded DB
	ccHObject::Container meshes;
	db->filterChildren(meshes,false,CC_TYPES::MESH);
	if (!meshes.empty())
	{
		ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(meshes[0]);
		db->detachChild(mesh);
		Print(QString("Found one mesh with %1 faces and %2 vertices").arg(mesh->size()).arg(mesh->getAssociatedCloud()->size()));
		m_meshes.push_back(MeshDesc(mesh,filename));
	}

	delete db;
	db = 0;

	return true;
}

bool ccCommandLineParser::commandSubsample(QStringList& arguments, ccProgressDialog* pDlg/*=0*/)
{
	Print("[SUBSAMPLING]");
	if (m_clouds.empty())
		return Error(QString("No point cloud to resample (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN).arg(COMMAND_SUBSAMPLE));

	if (arguments.empty())
		return Error(QString("Missing parameter: resampling method after \"-%1\"").arg(COMMAND_SUBSAMPLE));

	QString method = arguments.takeFirst().toUpper();
	Print(QString("\tMethod: ")+method);
	if (method == "RANDOM")
	{
		if (arguments.empty())
			return Error(QString("Missing parameter: number of points after \"-%1 RANDOM\"").arg(COMMAND_SUBSAMPLE));

		bool ok;
		unsigned count = arguments.takeFirst().toInt(&ok);
		if (!ok)
			return Error("Invalid number of points for random resampling!");
		Print(QString("\tOutput points: %1").arg(count));

		for (unsigned i=0; i<m_clouds.size(); ++i)
		{
			ccPointCloud* cloud = m_clouds[i].pc;
			Print(QString("\tProcessing cloud #%1 (%2)").arg(i+1).arg(!cloud->getName().isEmpty() ? cloud->getName() : "no name"));

			CCLib::ReferenceCloud* refCloud = CCLib::CloudSamplingTools::subsampleCloudRandomly(cloud,count,pDlg);
			if (!refCloud)
				return Error("Subsampling process failed!");
			Print(QString("\tResult: %1 points").arg(refCloud->size()));

			//save output
			ccPointCloud* result = cloud->partialClone(refCloud);
			delete refCloud;
			refCloud = 0;

			if (result)
			{
				CloudDesc cloudDesc(result,m_clouds[i].basename,m_clouds[i].path,m_clouds[i].indexInFile);
				QString errorStr = Export(cloudDesc,"RANDOM_SUBSAMPLED");
				delete result;
				result = 0;
				if (!errorStr.isEmpty())
					return Error(errorStr);
			}
			else
			{
				return Error("Not enough memory!");
			}
		}
	}
	else if (method == "SPATIAL")
	{
		if (arguments.empty())
			return Error(QString("Missing parameter: spatial step after \"-%1 SPATIAL\"").arg(COMMAND_SUBSAMPLE));

		bool ok;
		double step = arguments.takeFirst().toDouble(&ok);
		if (!ok || step <= 0)
			return Error("Invalid step value for spatial resampling!");
		Print(QString("\tSpatial step: %1").arg(step));

		for (unsigned i=0; i<m_clouds.size(); ++i)
		{
			ccPointCloud* cloud = m_clouds[i].pc;
			Print(QString("\tProcessing cloud #%1 (%2)").arg(i+1).arg(!cloud->getName().isEmpty() ? cloud->getName() : "no name"));

			CCLib::ReferenceCloud* refCloud = CCLib::CloudSamplingTools::resampleCloudSpatially(cloud,static_cast<PointCoordinateType>(step),0,pDlg);
			if (!refCloud)
				return Error("Subsampling process failed!");
			Print(QString("\tResult: %1 points").arg(refCloud->size()));

			//save output
			ccPointCloud* result = cloud->partialClone(refCloud);
			delete refCloud;
			refCloud = 0;

			if (result)
			{
				CloudDesc cloudDesc(result,m_clouds[i].basename,m_clouds[i].path,m_clouds[i].indexInFile);
				QString errorStr = Export(cloudDesc,"SPATIAL_SUBSAMPLED");

				delete result;
				result = 0;

				if (!errorStr.isEmpty())
					return Error(errorStr);
			}
			else
			{
				return Error("Not enough memory!");
			}
		}
	}
	else if (method == "OCTREE")
	{
		if (arguments.empty())
			return Error(QString("Missing parameter: octree level after \"-%1 OCTREE\"").arg(COMMAND_SUBSAMPLE));

		bool ok = false;
		int octreeLevel = arguments.takeFirst().toInt(&ok);
		if (!ok || octreeLevel<1 || octreeLevel>CCLib::DgmOctree::MAX_OCTREE_LEVEL)
			return Error("Invalid octree level!");
		Print(QString("\tOctree level: %1").arg(octreeLevel));

		for (unsigned i=0; i<m_clouds.size(); ++i)
		{
			ccPointCloud* cloud = m_clouds[i].pc;
			Print(QString("\tProcessing cloud #%1 (%2)").arg(i+1).arg(!cloud->getName().isEmpty() ? cloud->getName() : "no name"));

			CCLib::ReferenceCloud* refCloud = CCLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(cloud,static_cast<uchar>(octreeLevel),CCLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER,pDlg);
			if (!refCloud)
				return Error("Subsampling process failed!");
			Print(QString("\tResult: %1 points").arg(refCloud->size()));

			//save output
			ccPointCloud* result = cloud->partialClone(refCloud);
			delete refCloud;
			refCloud = 0;

			if (result)
			{
				CloudDesc cloudDesc(result,m_clouds[i].basename,m_clouds[i].path,m_clouds[i].indexInFile);
				QString errorStr = Export(cloudDesc,QString("OCTREE_LEVEL_%1_SUBSAMPLED").arg(octreeLevel));

				delete result;
				result = 0;

				if (!errorStr.isEmpty())
					return Error(errorStr);
			}
			else
			{
				return Error("Not enough memory!");
			}
		}
	}
	else
	{
		return Error("Unknown method!");
	}

	return true;
}

bool ccCommandLineParser::commandCurvature(QStringList& arguments, QDialog* parent/*=0*/)
{
	Print("[CURVATURE]");

	if (arguments.empty())
		return Error(QString("Missing parameter: curvature type after \"-%1\"").arg(COMMAND_CURVATURE));

	QString curvTypeStr = arguments.takeFirst().toUpper();
	CCLib::Neighbourhood::CC_CURVATURE_TYPE curvType = CCLib::Neighbourhood::MEAN_CURV;
	if (curvTypeStr == "MEAN")
	{
		//curvType = CCLib::Neighbourhood::MEAN_CURV;
	}
	else if (curvTypeStr == "GAUSS")
	{
		curvType = CCLib::Neighbourhood::GAUSSIAN_CURV;
	}
	else
	{
		return Error(QString("Invalid curvature type after \"-%1\". Got '%2' instead of MEAN or GAUSS.").arg(COMMAND_CURVATURE).arg(curvTypeStr));
	}

	if (arguments.empty())
		return Error("Missing parameter: kernel size after curvature type");

	bool paramOk = false;
	QString kernelStr = arguments.takeFirst();
	PointCoordinateType kernelSize = static_cast<PointCoordinateType>(kernelStr.toDouble(&paramOk));
	if (!paramOk)
		return Error(QString("Failed to read a numerical parameter: kernel size (after curvature type). Got '%1' instead.").arg(kernelStr));
	Print(QString("\tKernel size: %1").arg(kernelSize));

	if (m_clouds.empty())
		return Error(QString("No point cloud on which to compute curvature! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN).arg(COMMAND_CURVATURE));

	//Call MainWindow generic method
	void* additionalParameters[2] = {&curvType, &kernelSize};
	ccHObject::Container entities;
	entities.resize(m_clouds.size());
	for (unsigned i=0; i<m_clouds.size(); ++i)
		entities[i] = m_clouds[i].pc;

	if (MainWindow::ApplyCCLibAlgortihm(MainWindow::CCLIB_ALGO_CURVATURE,entities,parent,additionalParameters))
	{
		//save output
		if (s_autoSaveMode && !saveClouds(QString("%1_CURVATURE_KERNEL_%2").arg(curvTypeStr).arg(kernelSize)))
			return false;
	}
	return true;
}

bool ReadDensityType(QStringList& arguments, CCLib::GeometricalAnalysisTools::Density& density)
{
	if (arguments.empty())
		return Error(QString("Missing parameter: density type after \"-%1\" (KNN/SURFACE/VOLUME)").arg(COMMAND_DENSITY_TYPE));
	//read option confirmed, we can move on
	QString typeArg = arguments.takeFirst().toUpper();
	if (typeArg == "KNN")
	{
		density = CCLib::GeometricalAnalysisTools::DENSITY_KNN;
	}
	else if (typeArg == "SURFACE")
	{
		density = CCLib::GeometricalAnalysisTools::DENSITY_2D;
	}
	else if (typeArg == "VOLUME")
	{
		density = CCLib::GeometricalAnalysisTools::DENSITY_3D;
	}
	else
	{
		return Error(QString("Invalid parameter: density type is expected after \"-%1\" (KNN/SURFACE/VOLUME)").arg(COMMAND_DENSITY_TYPE));
	}

	return true;
}

bool ccCommandLineParser::commandApproxDensity(QStringList& arguments, QDialog* parent/*=0*/)
{
	Print("[APPROX DENSITY]");
	if (m_clouds.empty())
		return Error(QString("No point cloud on which to compute approx. density! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN).arg(COMMAND_APPROX_DENSITY));

	//Call MainWindow generic method
	ccHObject::Container entities;
	entities.resize(m_clouds.size());
	for (size_t i=0; i<m_clouds.size(); ++i)
		entities[i] = m_clouds[i].pc;

	//optional parameter: density type
	CCLib::GeometricalAnalysisTools::Density densityType = CCLib::GeometricalAnalysisTools::DENSITY_3D;
	if (!arguments.empty())
	{
		QString argument = arguments.front();
		if (IsCommand(argument,COMMAND_DENSITY_TYPE))
		{
			//local option confirmed, we can move on
			arguments.pop_front();
			if (arguments.empty())
				return Error(QString("Missing parameter: density type after \"-%1\" (KNN/SURFACE/VOLUME)").arg(COMMAND_DENSITY_TYPE));
			//read option confirmed, we can move on
			if (!ReadDensityType(arguments,densityType))
				return false;
		}
	}
	void* additionalParameters[] = { &densityType };

	if (MainWindow::ApplyCCLibAlgortihm(MainWindow::CCLIB_ALGO_APPROX_DENSITY,entities,parent,additionalParameters))
	{
		//save output
		if (s_autoSaveMode && !saveClouds("APPROX_DENSITY"))
			return false;
	}

	return true;
}

bool ccCommandLineParser::commandDensity(QStringList& arguments, QDialog* parent/*=0*/)
{
	Print("[DENSITY]");

	if (arguments.empty())
		return Error(QString("Missing parameter: sphere radius after \"-%1\"").arg(COMMAND_DENSITY));

	bool paramOk = false;
	QString kernelStr = arguments.takeFirst();
	PointCoordinateType kernelSize = static_cast<PointCoordinateType>(kernelStr.toDouble(&paramOk));
	if (!paramOk)
		return Error(QString("Failed to read a numerical parameter: sphere radius (after \"-%1\"). Got '%2' instead.").arg(COMMAND_DENSITY).arg(kernelStr));
	Print(QString("\tSphere radius: %1").arg(kernelSize));

	//optional parameter: density type
	CCLib::GeometricalAnalysisTools::Density densityType = CCLib::GeometricalAnalysisTools::DENSITY_3D;
	if (!arguments.empty())
	{
		QString argument = arguments.front();
		if (IsCommand(argument,COMMAND_DENSITY_TYPE))
		{
			//local option confirmed, we can move on
			arguments.pop_front();
			if (arguments.empty())
				return Error(QString("Missing parameter: density type after \"-%1\" (KNN/SURFACE/VOLUME)").arg(COMMAND_DENSITY_TYPE));
			//read option confirmed, we can move on
			if (!ReadDensityType(arguments,densityType))
				return false;
		}
	}

	if (m_clouds.empty())
		return Error(QString("No point cloud on which to compute density! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN).arg(COMMAND_DENSITY));

	//Call MainWindow generic method
	void* additionalParameters[] = { &kernelSize, &densityType };
	ccHObject::Container entities;
	entities.resize(m_clouds.size());
	for (unsigned i=0; i<m_clouds.size(); ++i)
		entities[i] = m_clouds[i].pc;

	if (MainWindow::ApplyCCLibAlgortihm(MainWindow::CCLIB_ALGO_ACCURATE_DENSITY,entities,parent,additionalParameters))
	{
		//save output
		if (s_autoSaveMode && !saveClouds("DENSITY"))
			return false;
	}

	return true;
}

bool ccCommandLineParser::commandSFGradient(QStringList& arguments, QDialog* parent/*=0*/)
{
	Print("[SF GRADIENT]");

	if (arguments.empty())
		return Error(QString("Missing parameter: boolean (whether SF is euclidean or not) after \"-%1\"").arg(COMMAND_SF_GRADIENT));

	QString euclideanStr = arguments.takeFirst().toUpper();
	bool euclidean = false;
	if (euclideanStr == "TRUE")
	{
		euclidean = true;
	}
	else if (euclideanStr != "FALSE")
	{
		return Error(QString("Invalid boolean value after \"-%1\". Got '%2' instead of TRUE or FALSE.").arg(COMMAND_SF_GRADIENT).arg(euclideanStr));
	}

	if (m_clouds.empty())
		return Error(QString("No point cloud on which to compute SF gradient! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN).arg(COMMAND_SF_GRADIENT));

	//Call MainWindow generic method
	void* additionalParameters[1] = {&euclidean};
	ccHObject::Container entities;
	entities.reserve(m_clouds.size());
	for (unsigned i=0; i<m_clouds.size(); ++i)
	{
		unsigned sfCount = m_clouds[i].pc->getNumberOfScalarFields();
		if (sfCount == 0)
			ccConsole::Warning(QString("Warning: cloud '%1' has no scalar field (it will be ignored)").arg(m_clouds[i].pc->getName()));
		else
		{
			if (sfCount>1)
				ccConsole::Warning(QString("Warning: cloud '%1' has several scalar fields (the active one will be used by default, or the first one if none is active)").arg(m_clouds[i].pc->getName()));

			if (!m_clouds[i].pc->getCurrentDisplayedScalarField())
				m_clouds[i].pc->setCurrentDisplayedScalarField(0);

			entities.push_back(m_clouds[i].pc);
		}
	}

	if (MainWindow::ApplyCCLibAlgortihm(MainWindow::CCLIB_ALGO_SF_GRADIENT,entities,parent,additionalParameters))
	{
		//save output
		if (s_autoSaveMode && !saveClouds(euclidean ? "EUCLIDEAN_SF_GRAD" : "SF_GRAD"))
			return false;
	}

	return true;
}

bool ccCommandLineParser::commandRoughness(QStringList& arguments, QDialog* parent/*=0*/)
{
	Print("[ROUGHNESS]");

	if (arguments.empty())
		return Error(QString("Missing parameter: kernel size after \"-%1\"").arg(COMMAND_ROUGHNESS));

	bool paramOk = false;
	QString kernelStr = arguments.takeFirst();
	PointCoordinateType kernelSize = static_cast<PointCoordinateType>(kernelStr.toDouble(&paramOk));
	if (!paramOk)
		return Error(QString("Failed to read a numerical parameter: kernel size (after \"-%1\"). Got '%2' instead.").arg(COMMAND_ROUGHNESS).arg(kernelStr));
	Print(QString("\tKernel size: %1").arg(kernelSize));

	if (m_clouds.empty())
		return Error(QString("No point cloud on which to compute roughness! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN).arg(COMMAND_ROUGHNESS));

	//Call MainWindow generic method
	void* additionalParameters[1] = {&kernelSize};
	ccHObject::Container entities;
	entities.resize(m_clouds.size());
	for (unsigned i=0; i<m_clouds.size(); ++i)
		entities[i]=m_clouds[i].pc;

	if (MainWindow::ApplyCCLibAlgortihm(MainWindow::CCLIB_ALGO_ROUGHNESS,entities,parent,additionalParameters))
	{
		//save output
		if (s_autoSaveMode && !saveClouds(QString("ROUGHNESS_KERNEL_%2").arg(kernelSize)))
			return false;
	}

	return true;
}

//special SF values that can be used instead of explicit ones
enum USE_SPECIAL_SF_VALUE { USE_NONE,
							USE_MIN,
							USE_DISP_MIN,
							USE_SAT_MIN,
							USE_MAX,
							USE_DISP_MAX,
							USE_SAT_MAX
};

bool ccCommandLineParser::commandFilterSFByValue(QStringList& arguments)
{
	Print("[FILTER BY VALUE]");

	USE_SPECIAL_SF_VALUE useValForMin = USE_NONE;
	ScalarType minVal = 0;
	QString minValStr;
	{
		if (arguments.empty())
			return Error(QString("Missing parameter: min value after \"-%1\"").arg(COMMAND_FILTER_SF_BY_VALUE));

		bool paramOk = false;
		minValStr = arguments.takeFirst();
		if (minValStr.toUpper() == "MIN")
		{
			useValForMin = USE_MIN;
		}
		else if (minValStr.toUpper() == "DISP_MIN")
		{
			useValForMin = USE_DISP_MIN;
		}
		else if (minValStr.toUpper() == "SAT_MIN")
		{
			useValForMin = USE_SAT_MIN;
		}
		else
		{
			minVal = static_cast<ScalarType>(minValStr.toDouble(&paramOk));
			if (!paramOk)
				return Error(QString("Failed to read a numerical parameter: min value (after \"-%1\"). Got '%2' instead.").arg(COMMAND_FILTER_SF_BY_VALUE).arg(minValStr));
		}
	}

	USE_SPECIAL_SF_VALUE useValForMax = USE_NONE;
	ScalarType maxVal = 0;
	QString maxValStr;
	{
		if (arguments.empty())
			return Error(QString("Missing parameter: max value after \"-%1\" {min}").arg(COMMAND_FILTER_SF_BY_VALUE));

		bool paramOk = false;
		maxValStr = arguments.takeFirst();
		if (maxValStr.toUpper() == "MAX")
		{
			useValForMax = USE_MAX;
		}
		else if (maxValStr.toUpper() == "DISP_MAX")
		{
			useValForMax = USE_DISP_MAX;
		}
		else if (maxValStr.toUpper() == "SAT_MAX")
		{
			useValForMax = USE_SAT_MAX;
		}
		else
		{
			maxVal = static_cast<ScalarType>(maxValStr.toDouble(&paramOk));
			if (!paramOk)
				return Error(QString("Failed to read a numerical parameter: max value (after min value). Got '%1' instead.").arg(COMMAND_FILTER_SF_BY_VALUE).arg(maxValStr));
		}
	}

	Print(QString("\tInterval: [%1 - %2]").arg(minValStr).arg(maxValStr));

	if (m_clouds.empty())
		return Error(QString("No point cloud on which to filter SF! (be sure to open one or generate one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN).arg(COMMAND_FILTER_SF_BY_VALUE));


	for (unsigned i=0; i<m_clouds.size(); ++i)
	{
		CCLib::ScalarField* sf = m_clouds[i].pc->getCurrentOutScalarField();
		if (sf)
		{
			ScalarType thisMinVal = minVal;
			{
				switch(useValForMin)
				{
				case USE_MIN:
					thisMinVal = sf->getMin();
					break;
				case USE_DISP_MIN:
					thisMinVal = static_cast<ccScalarField*>(sf)->displayRange().start();
					break;
				case USE_SAT_MIN:
					thisMinVal = static_cast<ccScalarField*>(sf)->saturationRange().start();
					break;
				default:
					//nothing to do
					break;
				}
			}

			ScalarType thisMaxVal = maxVal;
			{
				switch(useValForMax)
				{
				case USE_MAX:
					thisMaxVal = sf->getMax();
					break;
				case USE_DISP_MAX:
					thisMaxVal = static_cast<ccScalarField*>(sf)->displayRange().stop();
					break;
				case USE_SAT_MAX:
					thisMaxVal = static_cast<ccScalarField*>(sf)->saturationRange().stop();
					break;
				default:
					//nothing to do
					break;
				}
			}

			ccPointCloud* fitleredCloud = m_clouds[i].pc->filterPointsByScalarValue(thisMinVal,thisMaxVal);
			if (fitleredCloud)
			{
				CloudDesc resultDesc(fitleredCloud,m_clouds[i].basename,m_clouds[i].path,m_clouds[i].indexInFile);
				QString errorStr = Export(resultDesc,QString("FILTERED_[%1_%2]").arg(thisMinVal).arg(thisMaxVal));

				delete fitleredCloud;
				fitleredCloud = 0;

				if (!errorStr.isEmpty())
					return Error(errorStr);
			}
		}
	}

	return true;
}

bool ccCommandLineParser::commandMergeClouds(QStringList& arguments)
{
	Print("[MERGE CLOUDS]");

	if (m_clouds.size() < 2)
	{
		ccConsole::Warning("Less than 2 clouds! Nothing to do...");
		return true;
	}

	//merge clouds
	{
		for (size_t i=1; i<m_clouds.size(); ++i)
		{
			unsigned beforePts = m_clouds.front().pc->size();
			unsigned newPts = m_clouds[i].pc->size();
			*m_clouds.front().pc += m_clouds[i].pc;

			//success?
			if (m_clouds.front().pc->size() == beforePts + newPts)
			{
				delete m_clouds[i].pc;
				m_clouds[i].pc = 0;
			}
			else
			{
				return Error("Fusion failed! (not enough memory?)");
			}
		}
	}

	//clean the 'm_clouds' vector
	m_clouds.resize(1);
	//update the first one
	m_clouds.front().basename += QString("_MERGED");
	QString errorStr = Export(m_clouds.front());
	if (!errorStr.isEmpty())
		return Error(errorStr);

	return true;
}

bool ccCommandLineParser::setActiveSF(QStringList& arguments)
{
	if (arguments.empty())
		return Error(QString("Missing parameter: scalar field index after \"-%1\"").arg(COMMAND_SET_ACTIVE_SF));

	bool paramOk = false;
	QString sfIndexStr = arguments.takeFirst();
	int sfIndex = sfIndexStr.toInt(&paramOk);
	if (!paramOk)
		return Error(QString("Failed to read a numerical parameter: S.F. index (after \"-%1\"). Got '%2' instead.").arg(COMMAND_SET_ACTIVE_SF).arg(sfIndexStr));
	Print(QString("Set active S.F. index: %1").arg(sfIndex));

	if (m_clouds.empty())
		return Error(QString("No point cloud loaded! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN).arg(COMMAND_SET_ACTIVE_SF));

	for (unsigned i=0; i<m_clouds.size(); ++i)
	{
		if (m_clouds[i].pc && m_clouds[i].pc->hasScalarFields())
		{
			if (static_cast<int>(m_clouds[i].pc->getNumberOfScalarFields()) > sfIndex)
				m_clouds[i].pc->setCurrentScalarField(sfIndex);
			else
				ccConsole::Warning(QString("Cloud '%1' has less scalar fields than the index to select!").arg(m_clouds[i].pc->getName()));
		}
	}

	return true;
}

bool ccCommandLineParser::matchBBCenters(QStringList& arguments)
{
	Print("[MATCH B.B. CENTERS]");

	std::vector<EntityDesc*> entities;
	for (size_t i=0; i<m_clouds.size(); ++i)
		entities.push_back(&m_clouds[i]);
	for (size_t j=0; j<m_meshes.size(); ++j)
		entities.push_back(&m_meshes[j]);

	if (entities.empty())
	{
		return Error("No entity loaded!");
	}
	else if (entities.size() == 1)
	{
		ccConsole::Warning("Nothing to do: only one entity currently loaded!");
		return true;
	}

	CCVector3 firstCenter = entities.front()->getEntity()->getBBCenter();
	for (size_t i=1; i<entities.size(); ++i)
	{
		ccHObject* ent = entities[i]->getEntity();
		CCVector3 center = ent->getBBCenter();
		CCVector3 T = firstCenter-center;

		//transformation (used only for translation)
		ccGLMatrix glTrans;
		glTrans += T;

		//apply translation matrix
		ent->applyGLTransformation_recursive(&glTrans);
		Print(QString("Entity '%1' has been translated: (%2,%3,%4)").arg(ent->getName()).arg(T.x).arg(T.y).arg(T.z));
		QString errorStr = Export(*entities[i]);
		if (!errorStr.isEmpty())
			return Error(errorStr);
	}

	return true;
}

bool ccCommandLineParser::commandBestFitPlane(QStringList& arguments)
{
	Print("[COMPUTE BEST FIT PLANE]");

	//look for local options
	bool makeCloudsHoriz = false;
	bool keepLoaded = false;

	while (!arguments.empty())
	{
		QString argument = arguments.front();
		if (IsCommand(argument,COMMAND_BEST_FIT_PLANE_MAKE_HORIZ))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			makeCloudsHoriz = true;
		}
		else if (IsCommand(argument,COMMAND_BEST_FIT_PLANE_KEEP_LOADED))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			keepLoaded = true;
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back on the main one!
		}
	}

	if (m_clouds.empty())
		return Error(QString("No cloud available. Be sure to open one first!"));

	for (size_t i=0; i<m_clouds.size(); ++i)
	{
		ccPointCloud* pc = m_clouds[i].pc;

		//try to fit plane
		double rms = 0.0;
		ccPlane* pPlane = ccPlane::Fit(pc, &rms);
		if (pPlane)
		{
			Print(QString("Plane successfully fitted: rms = %1").arg(rms));

			CCVector3 N = pPlane->getNormal();
			CCVector3 C = *CCLib::Neighbourhood(pc).getGravityCenter();

			MeshDesc planeDesc;
			planeDesc.mesh = pPlane;
			planeDesc.basename = m_clouds[i].basename;
			planeDesc.path = m_clouds[i].path;

			//save plane as a BIN file
			QString outputFilename;
			QString errorStr = Export(planeDesc,"BEST_FIT_PLANE",&outputFilename);
			if (!errorStr.isEmpty())
				ccConsole::Warning(errorStr);

			//open text file to save plane related information
			QString txtFilename = QString("%1/%2_%3").arg(m_clouds[i].path).arg(m_clouds[i].basename).arg("BEST_FIT_PLANE_INFO");
			if (s_addTimestamp)
				txtFilename += QString("_%1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd_hh'h'mm"));
			txtFilename += QString(".txt");
			QFile txtFile(txtFilename);
			txtFile.open(QIODevice::WriteOnly | QIODevice::Text);
			QTextStream txtStream(&txtFile);

			txtStream << QString("Filename: %1").arg(outputFilename) << endl;
			txtStream << QString("Fitting RMS: %1").arg(rms) << endl;

			//We always consider the normal with a positive 'Z' by default!
			if (N.z < 0.0)
				N *= -1.0;
			txtStream << QString("Normal: (%1,%2,%3)").arg(N.x,0,'f',s_precision).arg(N.y,0,'f',s_precision).arg(N.z,0,'f',s_precision) << endl;

			//we compute strike & dip by the way
			{
				PointCoordinateType dip = 0, dipDir = 0;
				ccNormalVectors::ConvertNormalToDipAndDipDir(N,dip,dipDir);
				txtStream << ccNormalVectors::ConvertDipAndDipDirToString(dip,dipDir) << endl;
			}

			//compute the transformation matrix that would make this normal points towards +Z
			ccGLMatrix makeZPosMatrix = ccGLMatrix::FromToRotation(N,CCVector3(0,0,1.0f));
			CCVector3 Gt = C;
			makeZPosMatrix.applyRotation(Gt);
			makeZPosMatrix.setTranslation(C-Gt);
			makeZPosMatrix.invert();

			txtStream << "Orientation matrix:" << endl;
			txtStream << makeZPosMatrix.toString(s_precision,' ') << endl;

			//close the text file
			txtFile.close();

			if (keepLoaded)
			{
				//add the resulting plane (mesh) to the main set
				m_meshes.push_back(planeDesc);
			}

			if (makeCloudsHoriz)
			{
				//apply 'horizontal' matrix
				pc->applyGLTransformation_recursive(&makeZPosMatrix);
				Print(QString("Cloud '%1' has been transformed with the above matrix").arg(pc->getName()));
				m_clouds[i].basename += QString("_HORIZ");
				QString errorStr = Export(m_clouds[i]);
				if (!errorStr.isEmpty())
					ccConsole::Warning(errorStr);
			}
		}
		else
		{
			ccConsole::Warning(QString("Failed to compute best fit plane for cloud '%1'").arg(m_clouds[i].pc->getName()));
		}
	}

	return true;
}

bool ccCommandLineParser::commandSampleMesh(QStringList& arguments, ccProgressDialog* pDlg/*=0*/)
{
	Print("[SAMPLE POINTS ON MESH]");

	if (arguments.empty())
		return Error(QString("Missing parameter: sampling mode after \"-%1\" (POINTS/DENSITY)").arg(COMMAND_SAMPLE_MESH));

	bool useDensity = false;
	double parameter = 0;

	QString sampleMode = arguments.takeFirst().toUpper();
	if (sampleMode == "POINTS")
		useDensity = false;
	else if (sampleMode == "DENSITY")
		useDensity = true;
	else
		return Error(QString("Invalid parameter: unknown sampling mode \"%1\"").arg(sampleMode));

	if (arguments.empty())
		return Error(QString("Missing parameter: value after sampling mode"));
	bool conversionOk = false;
	parameter = arguments.takeFirst().toDouble(&conversionOk);
	if (!conversionOk)
		return Error(QString("Invalid parameter: value after sampling mode"));

	if (m_meshes.empty())
		return Error(QString("No mesh available. Be sure to open one first!"));

	for (size_t i=0; i<m_meshes.size(); ++i)
	{

		ccPointCloud* cloud = m_meshes[i].mesh->samplePoints(useDensity,parameter,true,true,true,pDlg);

		if (!cloud)
		{
			return Error(QString("Cloud sampling failed!"));
		}

		//add the resulting cloud to the main set
		Print(QString("Sampled cloud created: %1 points").arg(cloud->size()));
		m_clouds.push_back(CloudDesc(cloud,m_meshes[i].basename+QString("_SAMPLED_POINTS"),m_meshes[i].path));

		//save it as well
		QString errorStr = Export(m_clouds.back());
		if (!errorStr.isEmpty())
			return Error(errorStr);
	}

	return true;
}

bool ccCommandLineParser::commandCrop(QStringList& arguments)
{
	Print("[CROP]");

	if (arguments.empty())
		return Error(QString("Missing parameter: box extents after \"-%1\" (Xmin:Ymin:Zmin:Xmax:Ymax:Zmax)").arg(COMMAND_CROP));
	if (m_clouds.empty())
		return Error(QString("No point cloud available. Be sure to open or generate one first!"));

	//decode box extents
	CCVector3 boxMin,boxMax;
	{
		QString boxBlock = arguments.takeFirst();
		QStringList tokens = boxBlock.split(':');
		if (tokens.size() != 6)
			return Error(QString("Invalid parameter: box extents (expected format is 'Xmin:Ymin:Zmin:Xmax:Ymax:Zmax')").arg(COMMAND_CROP));

		for (int i=0; i<6; ++i)
		{
			CCVector3* vec = (i<3 ? &boxMin : &boxMax);
			bool ok = true;
			vec->u[i%3] = static_cast<PointCoordinateType>(tokens[i].toDouble(&ok));
			if (!ok)
			{
				return Error(QString("Invalid parameter: box extents (component #%1 is not a valid number)").arg(i+1));
			}
		}
	}

	//optional parameters
	bool inside = true;
	while (!arguments.empty())
	{
		QString argument = arguments.front();
		if (IsCommand(argument,COMMAND_CROP_OUTSIDE))
		{
			//local option confirmed, we can move on
			arguments.pop_front();
			inside = false;
		}
		else
		{
			break;
		}
	}

	ccBBox cropBox(boxMin,boxMax);
	for (unsigned i=0; i<m_clouds.size(); ++i)
	{
		CCLib::ReferenceCloud* ref = m_clouds[i].pc->crop(cropBox,inside);
		if (ref)
		{
			if (ref->size() != 0)
			{
				ccPointCloud* croppedCloud = m_clouds[i].pc->partialClone(ref);
				delete ref;
				ref = 0;

				if (croppedCloud)
				{
					delete m_clouds[i].pc;
					m_clouds[i].pc = croppedCloud;
					croppedCloud->setName(m_clouds[i].pc->getName() + QString(".cropped"));
					m_clouds[i].basename += "_CROPPED";
					Export(m_clouds[i]);
				}
				else
				{
					return Error(QString("Not enough memory to crop cloud '%1'!").arg(m_clouds[i].pc->getName()));
				}
			}
			else
			{
				delete ref;
				ref = 0;
				return Error (QString("No point of cloud '%1' falls inside the input box!").arg(m_clouds[i].pc->getName()));
			}
		}
		else
		{
			return Error(QString("Crop process failed! (not enough memory)"));
		}
	}

	return true;
}

bool ccCommandLineParser::commandCrop2D(QStringList& arguments)
{
	Print("[CROP 2D]");

	if (arguments.size() < 6)
		return Error(QString("Missing parameter(s) after \"-%1\" (ORTHO_DIM N X1 Y1 X2 Y2 ... XN YN)").arg(COMMAND_CROP_2D));
	if (m_clouds.empty())
		return Error(QString("No point cloud available. Be sure to open or generate one first!"));

	//decode poyline extents
	ccPointCloud vertices("polyline.vertices");
	ccPolyline poly(&vertices);

	//number of vertices
	unsigned char orthoDim = 2;
	{
		QString orthoDimStr = arguments.takeFirst().toUpper();
		if (orthoDimStr == "X")
			orthoDim = 0;
		else if (orthoDimStr == "Y")
			orthoDim = 1;
		else if (orthoDimStr == "Z")
			orthoDim = 2;
		else
			return Error(QString("Invalid parameter: orthogonal dimension after \"-%1\" (expected: X, Y or Z)").arg(COMMAND_CROP_2D));
	}

	//number of vertices
	bool ok = true;
	unsigned N = 0;
	{
		QString countStr = arguments.takeFirst();
		N = countStr.toUInt(&ok);
		if (!ok)
			return Error(QString("Invalid parameter: number of vertices for the 2D polyline after \"-%1\"").arg(COMMAND_CROP_2D));
	}

	//now read the vertices
	{
		unsigned char X = ((orthoDim+1) % 3);
		unsigned char Y = ((X+1) % 3);
		if (	!vertices.reserve(N)
			||	!poly.addPointIndex(0,N) )
		{
			return Error("Not enough memory!");
		}

		for (unsigned i=0; i<N; ++i)
		{
			if (arguments.size() < 2)
				return Error(QString("Missing parameter(s): vertex #%1 data and following").arg(i+1));

			CCVector3 P(0,0,0);

			QString coordStr = arguments.takeFirst();
			P.u[X] = static_cast<PointCoordinateType>( coordStr.toDouble(&ok) );
			if (!ok)
				return Error(QString("Invalid parameter: X-coordinate of vertex #%1").arg(i+1));
			/*QString */coordStr = arguments.takeFirst();
			P.u[Y] = static_cast<PointCoordinateType>( coordStr.toDouble(&ok) );
			if (!ok)
				return Error(QString("Invalid parameter: Y-coordinate of vertex #%1").arg(i+1));

			vertices.addPoint(P);
		}

		poly.setClosed(true);
	}

	//optional parameters
	bool inside = true;
	while (!arguments.empty())
	{
		QString argument = arguments.front();
		if (IsCommand(argument,COMMAND_CROP_OUTSIDE))
		{
			//local option confirmed, we can move on
			arguments.pop_front();
			inside = false;
		}
		else
		{
			break;
		}
	}

	//now we can crop the loaded cloud(s)
	for (unsigned i=0; i<m_clouds.size(); ++i)
	{
		CCLib::ReferenceCloud* ref = m_clouds[i].pc->crop2D(&poly,orthoDim,inside);
		if (ref)
		{
			if (ref->size() != 0)
			{
				ccPointCloud* croppedCloud = m_clouds[i].pc->partialClone(ref);
				delete ref;
				ref = 0;

				if (croppedCloud)
				{
					delete m_clouds[i].pc;
					m_clouds[i].pc = croppedCloud;
					croppedCloud->setName(m_clouds[i].pc->getName() + QString(".cropped"));
					m_clouds[i].basename += "_CROPPED";
					Export(m_clouds[i]);
				}
				else
				{
					return Error(QString("Not enough memory to crop cloud '%1'!").arg(m_clouds[i].pc->getName()));
				}
			}
			else
			{
				delete ref;
				ref = 0;
				ccConsole::Warning(QString("No point of cloud '%1' falls inside the input box!").arg(m_clouds[i].pc->getName()));
			}
		}
		else
		{
			return Error(QString("Crop process failed! (not enough memory)"));
		}
	}

	return true;
}

bool ccCommandLineParser::commandBundler(QStringList& arguments)
{
	Print("[BUNDLER]");
	if (arguments.empty())
		return Error(QString("Missing parameter: filename after \"-%1\"").arg(COMMAND_BUNDLER));

	//open specified file
	QString bundlerFilename(arguments.takeFirst());
	Print(QString("Importing Bundler file: '%1'").arg(bundlerFilename));

	QString altKeypointsFilename;
	bool undistortImages = false;
	bool generateColoredDTM = false;
	unsigned coloredDTMVerticesCount = 0;
	float scaleFactor = 1.0f;

	//inner loop for Bundler import options
	while (!arguments.empty())
	{
		QString argument = arguments.front();
		if (IsCommand(argument,COMMAND_BUNDLER_ALT_KEYPOINTS))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			if (arguments.empty())
				return Error(QString("Missing parameter: filename after \"-%1\"").arg(COMMAND_BUNDLER_ALT_KEYPOINTS));
			altKeypointsFilename = arguments.takeFirst();
		}
		else if (IsCommand(argument,COMMAND_BUNDLER_SCALE_FACTOR))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			if (arguments.empty())
				return Error(QString("Missing parameter: value after \"-%1\"").arg(COMMAND_BUNDLER_SCALE_FACTOR));
			bool conversionOk = false;
			scaleFactor = arguments.takeFirst().toFloat(&conversionOk);
			if (!conversionOk)
				return Error(QString("Invalid parameter: value after \"-%1\"").arg(COMMAND_BUNDLER_SCALE_FACTOR));
		}
		else if (IsCommand(argument,COMMAND_BUNDLER_UNDISTORT))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			undistortImages = true;
		}
		else if (IsCommand(argument,COMMAND_BUNDLER_COLOR_DTM))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			if (arguments.empty())
				return Error(QString("Missing parameter: vertices count after \"-%1\"").arg(COMMAND_BUNDLER_COLOR_DTM));
			bool conversionOk = false;
			coloredDTMVerticesCount = arguments.takeFirst().toUInt(&conversionOk);
			if (!conversionOk)
				return Error(QString("Invalid parameter: vertices count after \"-%1\"").arg(COMMAND_BUNDLER_COLOR_DTM));
			generateColoredDTM = true;
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back on the main one!
		}
	}

	ccHObject tempContainer;
	FileIOFilter::LoadParameters parameters;
	parameters.alwaysDisplayLoadDialog = false;
	BundlerFilter().loadFileExtended(	qPrintable(bundlerFilename),
										tempContainer,
										parameters,
										altKeypointsFilename,
										undistortImages,
										generateColoredDTM,
										coloredDTMVerticesCount,
										scaleFactor);
	return true;
}

bool ccCommandLineParser::commandDist(QStringList& arguments, bool cloud2meshDist, QDialog* parent/*=0*/)
{
	Print("[DISTANCE COMPUTATION]");

	//compared cloud
	if (m_clouds.empty())
		return Error(QString("No point cloud available. Be sure to open or generate one first!"));
	else if (cloud2meshDist && m_clouds.size() != 1)
		ccConsole::Warning("Multiple point clouds loaded! We take the first one by default");
	CloudDesc& compCloud = m_clouds.front();

	//reference entity
	ccHObject* refEntity = 0;
	if (cloud2meshDist)
	{
		if (m_meshes.empty())
			return Error(QString("No mesh available. Be sure to open one first!"));
		else if (m_meshes.size() != 1)
			ccConsole::Warning("Multiple meshes loaded! We take the first one by default");
		refEntity = m_meshes.front().mesh;
	}
	else
	{
		if (m_clouds.size() < 2)
			return Error(QString("Only one point cloud available. Be sure to open or generate a second one before performing C2C distance!"));
		else if (m_clouds.size() > 2)
			ccConsole::Warning("More than 3 point clouds loaded! We take the second one as reference by default");
		refEntity = m_clouds[1].pc;
	}

	//inner loop for Distance computation options
	bool flipNormals = false;
	double maxDist = 0.0;
	unsigned octreeLevel = 0;

	bool splitXYZ = false;
	int modelIndex = 0;
	bool useKNN = true;
	double nSize = 0;

	while (!arguments.empty())
	{
		QString argument = arguments.front();
		if (IsCommand(argument,COMMAND_C2M_DIST_FLIP_NORMALS))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			flipNormals = true;

			if (!cloud2meshDist)
				ccConsole::Warning("Parameter \"-%1\" ignored: only for C2M distance!");
		}
		else if (IsCommand(argument,COMMAND_MAX_DISTANCE))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			if (arguments.empty())
				return Error(QString("Missing parameter: value after \"-%1\"").arg(COMMAND_MAX_DISTANCE));
			bool conversionOk = false;
			maxDist = arguments.takeFirst().toDouble(&conversionOk);
			if (!conversionOk)
				return Error(QString("Invalid parameter: value after \"-%1\"").arg(COMMAND_MAX_DISTANCE));

			if (!cloud2meshDist)
				ccConsole::Warning("Parameter \"-%1\" ignored: only for C2M distance!");
		}
		else if (IsCommand(argument,COMMAND_OCTREE_LEVEL))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			if (arguments.empty())
				return Error(QString("Missing parameter: value after \"-%1\"").arg(COMMAND_OCTREE_LEVEL));
			bool conversionOk = false;
			octreeLevel = arguments.takeFirst().toUInt(&conversionOk);
			if (!conversionOk)
				return Error(QString("Invalid parameter: value after \"-%1\"").arg(COMMAND_OCTREE_LEVEL));
		}
		else if (IsCommand(argument,COMMAND_C2C_SPLIT_XYZ))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			splitXYZ = true;

			if (cloud2meshDist)
				ccConsole::Warning("Parameter \"-%1\" ignored: only for C2C distance!");
		}
		else if (IsCommand(argument,COMMAND_C2C_LOCAL_MODEL))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			if (!arguments.empty())
			{
				QString modelType = arguments.takeFirst().toUpper();
				if (modelType == "LS")
					modelIndex = 1;
				else if (modelType == "TRI")
					modelIndex = 2;
				else if (modelType == "HF")
					modelIndex = 3;
				else
					return Error(QString("Invalid parameter: unknown model type \"%1\"").arg(modelType));
			}
			else
			{
				return Error(QString("Missing parameter: model type after \"-%1\" (LS/TRI/HF)").arg(COMMAND_C2C_LOCAL_MODEL));
			}

			if (!arguments.empty())
			{
				QString nType = arguments.takeFirst().toUpper();
				if (nType == "KNN")
					useKNN = true;
				else if (nType == "SPHERE")
					useKNN = false;
				else
					return Error(QString("Invalid parameter: unknown neighborhood type \"%1\"").arg(nType));
			}
			else
			{
				return Error(QString("Missing parameter: expected neighborhood type after model type (KNN/SPHERE)"));
			}

			//neighborhood size
			if (!arguments.empty())
			{
				bool conversionOk = false;
				nSize = arguments.takeFirst().toDouble(&conversionOk);
				if (!conversionOk)
					return Error(QString("Invalid parameter: neighborhood size"));
			}
			else
			{
				return Error(QString("Missing parameter: expected neighborhood size after neighborhood type (neighbor count/sphere radius)"));
			}
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back on the main one!
		}
	}

	//spawn dialog (virtually) so as to prepare the comparison process
	ccComparisonDlg compDlg(compCloud.pc,
							refEntity,
							cloud2meshDist ? ccComparisonDlg::CLOUDMESH_DIST : ccComparisonDlg::CLOUDCLOUD_DIST,
							parent,
							true);

	//update parameters
	if (maxDist > 0)
	{
		compDlg.maxDistCheckBox->setChecked(true);
		compDlg.maxSearchDistSpinBox->setValue(maxDist);
	}
	if (octreeLevel > 0)
	{
		compDlg.octreeLevelCheckBox->setChecked(true);
		compDlg.octreeLevelSpinBox->setValue(octreeLevel);
	}

	//C2M-only parameters
	if (cloud2meshDist)
	{
		if (flipNormals)
			compDlg.flipNormalsCheckBox->setChecked(true);
	}
	//C2C-only parameters
	else
	{
		if (splitXYZ)
		{
			if (maxDist > 0)
				ccConsole::Warning("'Split XYZ' option is ignored if max distance is defined!");
			compDlg.split3DCheckBox->setChecked(true);
		}
		if (modelIndex != 0)
		{
			compDlg.localModelComboBox->setCurrentIndex(modelIndex);
			if (useKNN)
			{
				compDlg.lmKNNRadioButton->setChecked(true);
				compDlg.lmKNNSpinBox->setValue(static_cast<int>(nSize));
			}
			else
			{
				compDlg.lmRadiusRadioButton->setChecked(true);
				compDlg.lmRadiusDoubleSpinBox->setValue(nSize);
			}
		}
	}

	if (!compDlg.compute())
	{
		compDlg.cancelAndExit();
		return Error("An error occured during distances computation!");
	}

	compDlg.applyAndExit();

	QString suffix(cloud2meshDist ? "_C2M_DIST" : "_C2C_DIST");
	if (maxDist > 0)
		suffix += QString("_MAX_DIST_%1").arg(maxDist);

	compCloud.basename += suffix;

	QString errorStr = Export(compCloud);
	if (!errorStr.isEmpty())
		return Error(errorStr);


	return true;
}

bool ccCommandLineParser::commandStatTest(QStringList& arguments, ccProgressDialog* pDlg/*=0*/)
{
	Print("[STATISTICAL TEST]");

	//distribution
	CCLib::GenericDistribution* distrib = 0;
	{
		if (arguments.empty())
			return Error(QString("Missing parameter: distribution type after \"-%1\" (GAUSS/WEIBULL)").arg(COMMAND_STAT_TEST));

		QString distribStr = arguments.takeFirst().toUpper();
		if (distribStr == "GAUSS")
		{
			//mu
			if (arguments.empty())
				return Error(QString("Missing parameter: mean value after \"GAUSS\""));
			bool conversionOk = false;
			double mu = arguments.takeFirst().toDouble(&conversionOk);
			if (!conversionOk)
				return Error(QString("Invalid parameter: mean value after \"GAUSS\""));
			//sigma
			if (arguments.empty())
				return Error(QString("Missing parameter: sigma value after \"GAUSS\" {mu}"));
			conversionOk = false;
			double sigma = arguments.takeFirst().toDouble(&conversionOk);
			if (!conversionOk)
				return Error(QString("Invalid parameter: sigma value after \"GAUSS\" {mu}"));

			CCLib::NormalDistribution* N = new CCLib::NormalDistribution();
			N->setParameters(static_cast<ScalarType>(mu),static_cast<ScalarType>(sigma*sigma)); //warning: we input sigma2 here (not sigma)
			distrib = static_cast<CCLib::GenericDistribution*>(N);
		}
		else if (distribStr == "WEIBULL")
		{
			//a
			if (arguments.empty())
				return Error(QString("Missing parameter: a value after \"WEIBULL\""));
			bool conversionOk = false;
			double a = arguments.takeFirst().toDouble(&conversionOk);
			if (!conversionOk)
				return Error(QString("Invalid parameter: a value after \"WEIBULL\""));
			//b
			if (arguments.empty())
				return Error(QString("Missing parameter: b value after \"WEIBULL\" {a}"));
			conversionOk = false;
			double b = arguments.takeFirst().toDouble(&conversionOk);
			if (!conversionOk)
				return Error(QString("Invalid parameter: b value after \"WEIBULL\" {a}"));
			//c
			if (arguments.empty())
				return Error(QString("Missing parameter: shift value after \"WEIBULL\" {a} {b}"));
			conversionOk = false;
			double shift = arguments.takeFirst().toDouble(&conversionOk);
			if (!conversionOk)
				return Error(QString("Invalid parameter: shift value after \"WEIBULL\" {a} {b}"));

			CCLib::WeibullDistribution* N = new CCLib::WeibullDistribution();
			N->setParameters(static_cast<ScalarType>(a),static_cast<ScalarType>(b),static_cast<ScalarType>(shift));
			distrib = static_cast<CCLib::GenericDistribution*>(N);
		}
		else
		{
			return Error(QString("Invalid parameter: unknown distribution \"%1\"").arg(distribStr));
		}
	}

	//pValue
	double pValue = 0.0005;
	{
		if (arguments.empty())
			return Error(QString("Missing parameter: p-value after distribution"));
		bool conversionOk = false;
		pValue = arguments.takeFirst().toDouble(&conversionOk);
		if (!conversionOk)
			return Error(QString("Invalid parameter: p-value after distribution"));
	}

	//kNN
	unsigned kNN = 16;
	{
		if (arguments.empty())
			return Error(QString("Missing parameter: neighbors after p-value"));
		bool conversionOk = false;
		kNN = arguments.takeFirst().toUInt(&conversionOk);
		if (!conversionOk)
			return Error(QString("Invalid parameter: neighbors after p-value"));
	}

	if (m_clouds.empty())
		return Error(QString("No cloud available. Be sure to open one first!"));

	for (size_t i=0; i<m_clouds.size(); ++i)
	{
		ccPointCloud* pc = m_clouds[i].pc;

		//we apply method on currently 'output' SF
		CCLib::ScalarField* outSF = pc->getCurrentOutScalarField();
		if (outSF)
		{
			assert(outSF->isAllocated());

			//force Chi2 Distances field as 'IN' field (create it by the way if necessary)
			int chi2SfIdx = pc->getScalarFieldIndexByName(CC_CHI2_DISTANCES_DEFAULT_SF_NAME);
			if (chi2SfIdx < 0)
				chi2SfIdx = pc->addScalarField(CC_CHI2_DISTANCES_DEFAULT_SF_NAME);
			if (chi2SfIdx < 0)
			{
				if (distrib)
					delete distrib;
				return Error("Couldn't allocate a new scalar field for computing chi2 distances! Try to free some memory ...");
			}
			pc->setCurrentInScalarField(chi2SfIdx);

			//compute octree if necessary
			ccOctree* theOctree=pc->getOctree();
			if (!theOctree)
			{
				theOctree = pc->computeOctree(pDlg);
				if (!theOctree)
				{
					if (distrib)
						delete distrib;
					ccConsole::Error(QString("Couldn't compute octree for cloud '%1'!").arg(pc->getName()));
					break;
				}
			}

			double chi2dist = CCLib::StatisticalTestingTools::testCloudWithStatisticalModel(distrib,pc,kNN,pValue,pDlg,theOctree);

			Print(QString("[Chi2 Test] %1 test result = %2").arg(distrib->getName()).arg(chi2dist));

			//we set the theoretical Chi2 distance limit as the minimum displayed SF value so that all points below are grayed
			{
				ccScalarField* chi2SF = static_cast<ccScalarField*>(pc->getCurrentInScalarField());
				assert(chi2SF);
				chi2SF->computeMinAndMax();
				chi2dist *= chi2dist;
				chi2SF->setMinDisplayed(static_cast<ScalarType>(chi2dist));
				chi2SF->setSymmetricalScale(false);
				chi2SF->setSaturationStart(static_cast<ScalarType>(chi2dist));
				//chi2SF->setSaturationStop(chi2dist);
				pc->setCurrentDisplayedScalarField(chi2SfIdx);
				pc->showSF(true);
			}

			m_clouds[i].basename += QString("_STAT_TEST_%1").arg(distrib->getName());
			QString errorStr = Export(m_clouds[i]);
			if (!errorStr.isEmpty())
				return Error(errorStr);
		}
	}

	return true;
}

bool ccCommandLineParser::commandICP(QStringList& arguments, QDialog* parent/*=0*/)
{
	//look for local options
	bool referenceIsFirst = false;
	bool adjustScale = false;
	bool enableFarthestPointRemoval = false;
	double minErrorDiff = 1.0e-6;
	unsigned iterationCount = 0;
	unsigned randomSamplingLimit = 20000;

	while (!arguments.empty())
	{
		QString argument = arguments.front();
		if (IsCommand(argument,COMMAND_ICP_REFERENCE_IS_FIRST))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			referenceIsFirst = true;
		}
		else if (IsCommand(argument,COMMAND_ICP_ADJUST_SCALE))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			adjustScale = true;
		}
		else if (IsCommand(argument,COMMAND_ICP_ENABLE_FARTHEST_REMOVAL))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			enableFarthestPointRemoval = true;
		}
		else if (IsCommand(argument,COMMAND_ICP_MIN_ERROR_DIIF))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			if (arguments.empty())
				return Error(QString(QString("Missing parameter: min error difference after '%1'").arg(COMMAND_ICP_MIN_ERROR_DIIF)));
			bool ok;
			minErrorDiff = arguments.takeFirst().toDouble(&ok);
			if (!ok || minErrorDiff <= 0)
				return Error(QString("Invalid value for min. error difference! (%1)").arg(COMMAND_ICP_MIN_ERROR_DIIF));
		}
		else if (IsCommand(argument,COMMAND_ICP_ITERATION_COUNT))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			if (arguments.empty())
				return Error(QString(QString("Missing parameter: number of iterations after '%1'").arg(COMMAND_ICP_ITERATION_COUNT)));
			bool ok;
			iterationCount = arguments.takeFirst().toUInt(&ok);
			if (!ok || iterationCount == 0)
				return Error(QString("Invalid number of iterations! (%1)").arg(COMMAND_ICP_ITERATION_COUNT));
		}
		else if (IsCommand(argument,COMMAND_ICP_RANDOM_SAMPLING_LIMIT))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			if (arguments.empty())
				return Error(QString(QString("Missing parameter: random sampling limit value after '%1'").arg(COMMAND_ICP_RANDOM_SAMPLING_LIMIT)));
			bool ok;
			randomSamplingLimit = arguments.takeFirst().toUInt(&ok);
			if (!ok || randomSamplingLimit < 3)
				return Error(QString("Invalid random sampling limit! (%1)").arg(COMMAND_ICP_RANDOM_SAMPLING_LIMIT));
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back on the main one!
		}
	}

	//we'll get the first two entities
	EntityDesc* dataAndModel[2] = {0,0};
	{
		int index = 0;
		if (!m_clouds.empty())
		{
			dataAndModel[index++] = &m_clouds[0];
			if (m_clouds.size() > 1)
				dataAndModel[index++] = &m_clouds[1];
		}
		if (index < 2 && !m_meshes.empty())
		{
			dataAndModel[index++] = &m_meshes[0];
			if (index < 2 && m_meshes.size() > 1)
				dataAndModel[index++] = &m_meshes[1];
		}

		if (index < 2)
			return Error("Not enough loaded entities (expect at least 2!)");
	}

	if (referenceIsFirst)
		std::swap(dataAndModel[0],dataAndModel[1]);

	ccGLMatrix transMat;
	double finalError = 0.0;
	double finalScale = 1.0;
	if ( ccRegistrationTools::ICP(	dataAndModel[0]->getEntity(),
									dataAndModel[1]->getEntity(),
									transMat,
									finalScale,
									finalError,
									minErrorDiff,
									iterationCount,
									randomSamplingLimit,
									enableFarthestPointRemoval,
									iterationCount != 0 ? CCLib::ICPRegistrationTools::MAX_ITER_CONVERGENCE : CCLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE,
									adjustScale,
									false,
									false,
									CCLib::ICPRegistrationTools::SKIP_NONE,
									parent ))
	{
		ccHObject* data = dataAndModel[0]->getEntity();
		data->applyGLTransformation_recursive(&transMat);
		Print(QString("Entity '%1' has been registered").arg(data->getName()));

		//save matrix in a separate text file
		{
			QString txtFilename = QString("%1/%2_%3").arg(dataAndModel[0]->path).arg(dataAndModel[0]->basename).arg("_REGISTRATION_MATRIX");
			if (s_addTimestamp)
				txtFilename += QString("_%1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd_hh'h'mm"));
			txtFilename += QString(".txt");
			QFile txtFile(txtFilename);
			txtFile.open(QIODevice::WriteOnly | QIODevice::Text);
			QTextStream txtStream(&txtFile);
			txtStream << transMat.toString(s_precision,' ') << endl;
			txtFile.close();
		}

		dataAndModel[0]->basename += QString("_REGISTERED");
		QString errorStr = Export(*dataAndModel[0]);
		if (!errorStr.isEmpty())
			return Error(errorStr);
	}
	else
	{
		return false;
	}

	return true;
}

QString ccCommandLineParser::GetFileFormatFilter(QStringList& arguments, QString& defaultExt)
{
	QString fileFilter;
	defaultExt = QString();

	if (!arguments.isEmpty())
	{
		//test if the specified format corresponds to a known file type
		QString argument = arguments.front().toUpper();
		arguments.pop_front();

		const FileIOFilter::FilterContainer& filters = FileIOFilter::GetFilters();
		for (size_t i=0; i<filters.size(); ++i)
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
			ccConsole::Error(QString("Unhandled format specifier (%1)").arg(argument));
		}
	}
	else
	{
		ccConsole::Error("Missing file format specifier!");
	}

	return fileFilter;
}

bool ccCommandLineParser::commandChangeCloudOutputFormat(QStringList& arguments)
{
	QString defaultExt;
	QString fileFilter = GetFileFormatFilter(arguments,defaultExt);
	if (fileFilter.isEmpty())
		return false;

	s_CloudExportFormat = fileFilter;
	s_CloudExportExt = defaultExt;

	ccConsole::Print(QString("Output export format (clouds) set to: %1").arg(s_CloudExportExt.toUpper()));

	//default options for ASCII output
	if (fileFilter == AsciiFilter::GetFileFilter())
	{
		QSharedPointer<AsciiSaveDlg> saveDialog = AsciiFilter::GetSaveDialog();
		assert(saveDialog);
		saveDialog->setCoordsPrecision(s_precision);
		saveDialog->setSfPrecision(s_precision);
		saveDialog->setSeparatorIndex(0); //space
		saveDialog->enableSwapColorAndSF(false); //default order: point, color, SF, normal
		saveDialog->enableSaveColumnsNamesHeader(false);
		saveDialog->enableSavePointCountHeader(false);

		if (s_silentMode)
			saveDialog->setAutoShow(false);
	}

	//look for additional parameters
	while (!arguments.empty())
	{
		QString argument = arguments.front();
		if (IsCommand(argument,COMMAND_EXPORT_EXTENSION))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			if (arguments.empty())
				return Error(QString(QString("Missing parameter: extension after '%1'").arg(COMMAND_EXPORT_EXTENSION)));

			s_CloudExportExt = arguments.takeFirst();
			Print(QString("New output extension for clouds: %1").arg(s_CloudExportExt));
		}
		else if (IsCommand(argument,COMMAND_ASCII_EXPORT_PRECISION))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			if (arguments.empty())
				return Error(QString(QString("Missing parameter: precision value after '%1'").arg(COMMAND_ASCII_EXPORT_PRECISION)));
			bool ok;
			int precision = arguments.takeFirst().toInt(&ok);
			if (!ok || precision < 0)
				return Error(QString("Invalid value for precision! (%1)").arg(COMMAND_ASCII_EXPORT_PRECISION));

			if (fileFilter != AsciiFilter::GetFileFilter())
				ccConsole::Warning(QString("Argument '%1' is only applicable to ASCII format!").arg(argument));

			QSharedPointer<AsciiSaveDlg> saveDialog = AsciiFilter::GetSaveDialog();
			assert(saveDialog);
			if (saveDialog)
			{
				saveDialog->setCoordsPrecision(precision);
				saveDialog->setSfPrecision(precision);
				saveDialog->setAutoShow(false);
			}
		}
		else if (IsCommand(argument,COMMAND_ASCII_EXPORT_SEPARATOR))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			if (arguments.empty())
				return Error(QString(QString("Missing parameter: separator character after '%1'").arg(COMMAND_ASCII_EXPORT_SEPARATOR)));

			if (fileFilter != AsciiFilter::GetFileFilter())
				ccConsole::Warning(QString("Argument '%1' is only applicable to ASCII format!").arg(argument));

			QString separatorStr = arguments.takeFirst().toUpper();
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
				return Error(QString("Invalid separator! ('%1')").arg(COMMAND_ASCII_EXPORT_SEPARATOR));

			QSharedPointer<AsciiSaveDlg> saveDialog = AsciiFilter::GetSaveDialog();
			assert(saveDialog);
			if (saveDialog)
			{
				saveDialog->setSeparatorIndex(index);
				saveDialog->setAutoShow(false);
			}
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back on the main one!
		}
	}

	return true;
}

bool ccCommandLineParser::commandChangeMeshOutputFormat(QStringList& arguments)
{
	QString defaultExt;
	QString fileFilter = GetFileFormatFilter(arguments,defaultExt);
	if (fileFilter.isEmpty())
		return false;

	s_MeshExportFormat = fileFilter;
	s_MeshExportExt = defaultExt;

	ccConsole::Print(QString("Output export format (meshes) set to: %1").arg(s_MeshExportExt.toUpper()));

	//look for additional parameters
	while (!arguments.empty())
	{
		QString argument = arguments.front();

		if (IsCommand(argument,COMMAND_EXPORT_EXTENSION))
		{
			//local option confirmed, we can move on
			arguments.pop_front();

			if (arguments.empty())
				return Error(QString(QString("Missing parameter: extension after '%1'").arg(COMMAND_EXPORT_EXTENSION)));

			s_MeshExportExt = arguments.takeFirst();
			Print(QString("New output extension for meshes: %1").arg(s_MeshExportExt));
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back on the main one!
		}
	}

	return true;
}

bool ccCommandLineParser::commandChangeFBXOutputFormat(QStringList& arguments)
{
	if (arguments.empty())
		return Error(QString("Missing parameter: FBX format (string) after '%1'").arg(COMMAND_FBX_EXPORT_FORMAT));

	QString formatStr = arguments.takeFirst();
	ccConsole::Print(QString("FBX format: %1").arg(formatStr));

#ifdef CC_FBX_SUPPORT
	FBXFilter::SetDefaultOutputFormat(formatStr);
#endif

	return true;
}

bool ccCommandLineParser::commandForcePTXNormalsComputation(QStringList& arguments)
{
	//simply change the default filter behavior
	PTXFilter::SetNormalsComputationBehavior(PTXFilter::ALWAYS);

	return true;
}

bool ccCommandLineParser::commandSaveClouds(QStringList& arguments)
{
	bool allAtOnce = false;

	//look for additional parameters
	while (!arguments.empty())
	{
		QString argument = arguments.front();

		if (argument.toUpper() == OPTION_ALL_AT_ONCE)
		{
			//local option confirmed, we can move on
			arguments.pop_front();
			allAtOnce = true;
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back on the main one!
		}
	}

	return saveClouds(QString(),allAtOnce);
}

bool ccCommandLineParser::commandSaveMeshes(QStringList& arguments)
{
	bool allAtOnce = false;

	//look for additional parameters
	while (!arguments.empty())
	{
		QString argument = arguments.front();

		if (argument.toUpper() == OPTION_ALL_AT_ONCE)
		{
			//local option confirmed, we can move on
			arguments.pop_front();
			allAtOnce = true;
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back on the main one!
		}
	}

	return saveMeshes(QString(),allAtOnce);
}

bool ccCommandLineParser::commandAutoSave(QStringList& arguments)
{
	if (arguments.empty())
		return Error(QString(QString("Missing parameter: option after '%1' (%2/%2)").arg(COMMAND_AUTO_SAVE).arg(OPTION_ON).arg(OPTION_OFF)));

	QString option = arguments.takeFirst().toUpper();
	if (option == OPTION_ON)
	{
		Print("Auto-save is enabled");
		s_autoSaveMode = true;
	}
	else if (option == OPTION_OFF)
	{
		Print("Auto-save is disabled");
		s_autoSaveMode = false;
	}
	else
	{
		return Error(QString(QString("Unrecognized option afer '%1' (%2 or %3 expected)").arg(COMMAND_AUTO_SAVE).arg(OPTION_ON).arg(OPTION_OFF)));
	}

	return true;
}

int ccCommandLineParser::parse(QStringList& arguments, QDialog* parent/*=0*/)
{
	ccProgressDialog progressDlg(false,parent);

	QElapsedTimer eTimer;
	eTimer.start();

	bool success = true;
	while (success && !arguments.empty())
	{
		QString argument = arguments.takeFirst();

		// "O" OPEN FILE
		if (IsCommand(argument,COMMAND_OPEN))
		{
			success = commandLoad(arguments);
		}
		// "SS" SUBSAMPLING
		else if (IsCommand(argument,COMMAND_SUBSAMPLE))
		{
			success = commandSubsample(arguments,&progressDlg);
		}
		// "CURV" CURVATURE
		else if (IsCommand(argument,COMMAND_CURVATURE))
		{
			success = commandCurvature(arguments,parent);
		}
		// "DENSITY"
		else if (IsCommand(argument,COMMAND_DENSITY))
		{
			success = commandDensity(arguments,parent);
		}
		// "APPROX DENSITY"
		else if (IsCommand(argument,COMMAND_APPROX_DENSITY))
		{
			success = commandApproxDensity(arguments,parent);
		}
		// "SF_GRAD" SF GRADIENT
		else if (IsCommand(argument,COMMAND_SF_GRADIENT))
		{
			success = commandSFGradient(arguments,parent);
		}
		// "ROUGH" ROUGHNESS
		else if (IsCommand(argument,COMMAND_ROUGHNESS))
		{
			success = commandRoughness(arguments,parent);
		}
		//Import Bundler file + orthorectification
		else if (IsCommand(argument,COMMAND_BUNDLER))
		{
			success = commandBundler(arguments);
		}
		//Cloud-Mesh distance
		else if (IsCommand(argument,COMMAND_C2M_DIST))
		{
			success = commandDist(arguments,true,parent);
		}
		//Cloud-Cloud distance
		else if (IsCommand(argument,COMMAND_C2C_DIST))
		{
			success = commandDist(arguments,false,parent);
		}
		//Mesh sampling
		else if (IsCommand(argument,COMMAND_SAMPLE_MESH))
		{
			success = commandSampleMesh(arguments,&progressDlg);
		}
		//Best fit plane
		else if (IsCommand(argument,COMMAND_BEST_FIT_PLANE))
		{
			success = commandBestFitPlane(arguments);
		}
		//Match b.b. centers
		else if (IsCommand(argument,COMMAND_MATCH_BB_CENTERS))
		{
			success = matchBBCenters(arguments);
		}
		//Filter SF by values
		else if (IsCommand(argument,COMMAND_FILTER_SF_BY_VALUE))
		{
			success = commandFilterSFByValue(arguments);
		}
		//Merge clouds
		else if (IsCommand(argument,COMMAND_MERGE_CLOUDS))
		{
			success = commandMergeClouds(arguments);
		}
		//Perform statistical test
		else if (IsCommand(argument,COMMAND_STAT_TEST))
		{
			success = commandStatTest(arguments,&progressDlg);
		}
		//ICP registration
		else if (IsCommand(argument,COMMAND_ICP))
		{
			success = commandICP(arguments,parent);
		}
		//Crop
		else if (IsCommand(argument,COMMAND_CROP))
		{
			success = commandCrop(arguments);
		}
		//Crop 2D
		else if (IsCommand(argument,COMMAND_CROP_2D))
		{
			success = commandCrop2D(arguments);
		}
		//Change default cloud output format
		else if (IsCommand(argument,COMMAND_CLOUD_EXPORT_FORMAT))
		{
			success = commandChangeCloudOutputFormat(arguments);
		}
		//Change default mesh output format
		else if (IsCommand(argument,COMMAND_MESH_EXPORT_FORMAT))
		{
			success = commandChangeMeshOutputFormat(arguments);
		}
		//Set default FBX output format
		else if (IsCommand(argument,COMMAND_FBX_EXPORT_FORMAT))
		{
			success = commandChangeFBXOutputFormat(arguments);
		}
		//Force normal computation when importing PTX files
		else if (IsCommand(argument,COMMAND_PTX_COMPUTE_NORMALS))
		{
			success = commandForcePTXNormalsComputation(arguments);
		}
		else if (IsCommand(argument,COMMAND_SET_ACTIVE_SF))
		{
			success = setActiveSF(arguments);
		}
		//save all loaded clouds
		else if (IsCommand(argument,COMMAND_SAVE_CLOUDS))
		{
			success = commandSaveClouds(arguments);
		}
		//save all loaded meshes
		else if (IsCommand(argument,COMMAND_SAVE_MESHES))
		{
			success = commandSaveMeshes(arguments);
		}
		//auto-save mode
		else if (IsCommand(argument,COMMAND_AUTO_SAVE))
		{
			success = commandAutoSave(arguments);
		}
		//unload all loaded clouds
		else if (IsCommand(argument,COMMAND_CLEAR_CLOUDS))
		{
			removeClouds();
		}
		//unload all loaded meshes
		else if (IsCommand(argument,COMMAND_CLEAR_MESHES))
		{
			removeMeshes();
		}
		//unload all loaded entities
		else if (IsCommand(argument,COMMAND_CLEAR))
		{
			removeClouds();
			removeMeshes();
		}
		//no timestamp for output filenames
		else if (IsCommand(argument,COMMAND_NO_TIMESTAMP))
		{
			s_addTimestamp = false;
		}
		//silent mode (i.e. no console)
		else if (IsCommand(argument,COMMAND_SILENT_MODE))
		{
			ccConsole::Warning(QString("Misplaced command: '%1' (must be first)").arg(COMMAND_SILENT_MODE));
		}
		else
		{
			ccConsole::Error(QString("Unknown or misplaced command: '%1'").arg(argument));
			success = false;
		}
	}

	ccConsole::Print("Processed finished in %.2f s.",eTimer.elapsed()/1.0e3);

	return success ? EXIT_SUCCESS : EXIT_FAILURE;
}
