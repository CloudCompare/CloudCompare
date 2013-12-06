#include "ccCommandLineParser.h"

//CCLib
#include <CloudSamplingTools.h>
#include <WeibullDistribution.h>
#include <NormalDistribution.h>
#include <StatisticalTestingTools.h>
#include <Neighbourhood.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccGenericMesh.h>
#include <ccProgressDialog.h>
#include <ccOctree.h>

//qCC
#include "fileIO/FileIOFilter.h"
#include "fileIO/BundlerFilter.h"
#include "ccCommon.h"
#include <ui_commandLineDlg.h>
#include "ccConsole.h"
#include "mainwindow.h"
#include "ccComparisonDlg.h"

//Qt
#include <QMessageBox>
#include <QDialog>
#include <QFileInfo>
#include <QDateTime>
#include <QElapsedTimer>
#include <QStringList>

static const char COMMAND_SILENT_MODE[]				= "SILENT";
static const char COMMAND_OPEN[]					= "O";				//+file name
static const char COMMAND_SUBSAMPLE[]				= "SS";				//+ method (RANDOM/SPATIAL/OCTREE) + parameter (resp. point count / spatial step / octree level)
static const char COMMAND_CURVATURE[]				= "CURV";			//+ curvature type (MEAN/GAUSS) + 
static const char COMMAND_DENSITY[]					= "DENSITY";		//
static const char COMMAND_SF_GRADIENT[]				= "SF_GRAD";
static const char COMMAND_ROUGHNESS[]				= "ROUGH";
static const char COMMAND_BUNDLER[]					= "BUNDLER_IMPORT"; //Import Bundler file + orthorectification
static const char COMMAND_BUNDLER_ALT_KEYPOINTS[]	= "ALT_KEYPOINTS";
static const char COMMAND_BUNDLER_SCALE_FACTOR[]	= "SCALE_FACTOR";
static const char COMMAND_BUNDLER_UNDISTORT[]		= "UNDISTORT";
static const char COMMAND_BUNDLER_COLOR_DTM[]		= "COLOR_DTM";
static const char COMMAND_C2M_DIST[]				= "C2M_DIST";
static const char COMMAND_C2M_DIST_FLIP_NORMALS[]	= "FLIP_NORMS";
static const char COMMAND_C2C_DIST[]				= "C2C_DIST";
static const char COMMAND_C2C_SPLIT_XYZ[]			= "SPLIT_XYZ";
static const char COMMAND_C2C_LOCAL_MODEL[]			= "MODEL";
static const char COMMAND_MAX_DISTANCE[]			= "MAX_DIST";
static const char COMMAND_OCTREE_LEVEL[]			= "OCTREE_LEVEL";
static const char COMMAND_SAMPLE_MESH[]				= "SAMPLE_MESH";
static const char COMMAND_MERGE_CLOUDS[]			= "MERGE_CLOUDS";
static const char COMMAND_STAT_TEST[]				= "STAT_TEST";
static const char COMMAND_FILTER_SF_BY_VALUE[]		= "FILTER_SF";
static const char COMMAND_CLEAR_CLOUDS[]			= "CLEAR_CLOUDS";
static const char COMMAND_CLEAR_MESHES[]			= "CLEAR_MESHES";
static const char COMMAND_CLEAR[]					= "CLEAR";

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

	QStringList arguments;
	{
		for (int i=1; i<nargs; ++i) //'i=1' because first argument is always program executable file!
			arguments.push_back(QString(args[i]));
	}
	assert(!arguments.empty());

	//specific command: silent mode (will prevent the console dialog from appearing!
	bool silent = false;
	if (IsCommand(arguments.front(),COMMAND_SILENT_MODE))
	{
		arguments.pop_front();
		silent = true;
	}
	
	QDialog consoleDlg;
	if (!silent)
	{
		Ui_commandLineDlg commandLineDlg;
		commandLineDlg.setupUi(&consoleDlg);
		consoleDlg.show();
		ccConsole::Init(commandLineDlg.consoleWidget,&consoleDlg);
	}

	int result = ccCommandLineParser().parse(arguments,silent,&consoleDlg);

	if (!silent)
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

static void Print(const QString& message)
{
	ccConsole::Print(message);
	//printf("%s\n",qPrintable(message));
}

static bool Error(const QString& message)
{
	ccConsole::Error(message);
	//printf("[ERROR] %s\n",qPrintable(message));

	return false;
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


QString ccCommandLineParser::Export2BIN(CloudDesc& cloudDesc, QString suffix/*=QString()*/)
{
    assert(cloudDesc.pc);

	QString cloudName = (!cloudDesc.pc->getName().isEmpty() ? cloudDesc.pc->getName() : cloudDesc.basename);
	if (cloudDesc.indexInFile >= 0)
		cloudName += QString("_%1").arg(cloudDesc.indexInFile);
	if (!suffix.isEmpty())
		cloudName += QString("_") + suffix;
	cloudDesc.pc->setName(cloudName);

	QString baseName = cloudDesc.basename;
	if (!suffix.isEmpty())
		baseName += QString("_") + suffix;
	QString outputFilename = QString("%1/%2_%3.bin").arg(cloudDesc.path).arg(baseName).arg(QDateTime::currentDateTime().toString("yyyy-MM-dd_hh'h'mm"));

	ccHObject group;
	group.addChild(cloudDesc.pc,false);
	if (FileIOFilter::SaveToFile(&group,qPrintable(outputFilename),BIN) != CC_FERR_NO_ERROR)
		return QString("Failed to save result in file '%1'").arg(outputFilename);

	//Print(QString("--> result saved to file '%1'").arg(outputFilename)); //DGM: message already logged by FileIOFilter::SaveToFile (or BinFilter?)
    return QString();
}

bool ccCommandLineParser::commandLoad(QStringList& arguments)
{
	Print("[LOADING]");
	if (arguments.empty())
		return Error(QString("Missing parameter: filename after \"-%1\"").arg(COMMAND_OPEN));

	//open specified file
	QString filename(arguments.takeFirst());
	Print(QString("Opening file: '%1'").arg(filename));
	ccHObject* db = FileIOFilter::LoadFromFile(filename,UNKNOWN_FILE,false);
	if (!db)
		return Error(QString("Failed to open file '%1'").arg(filename));

	//look for clouds inside loaded DB
	ccHObject::Container clouds;
	db->filterChildren(clouds,false,CC_POINT_CLOUD);
	size_t count = clouds.size();
	for (size_t i=0;i<count;++i)
	{
		ccPointCloud* pc = static_cast<ccPointCloud*>(clouds[0]);
		pc->setFlagState(CC_FATHER_DEPENDENT,false);
		Print(QString("Found one cloud with %1 points").arg(pc->size()));
		m_clouds.push_back(CloudDesc(pc,filename,count == 1 ? -1 : static_cast<int>(i)));
	}

	//look for meshes inside loaded DB
	ccHObject::Container meshes;
	db->filterChildren(meshes,false,CC_MESH);
	if (!meshes.empty())
	{
		ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(meshes[0]);
		mesh->setFlagState(CC_FATHER_DEPENDENT,false);
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
				QString errorStr = Export2BIN(cloudDesc,"RANDOM_SUBSAMPLED");
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
				QString errorStr = Export2BIN(cloudDesc,"SPATIAL_SUBSAMPLED");

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
				QString errorStr = Export2BIN(cloudDesc,QString("OCTREE_LEVEL_%1_SUBSAMPLED").arg(octreeLevel));

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
		return Error("No point cloud on which to compute curvature! (be sure to open one with \"-O [cloud filename]\" before \"-CURV\")");

	//Call MainWindow generic method
	void* additionalParameters[2] = {&curvType, &kernelSize};
	ccHObject::Container entities;
	entities.resize(m_clouds.size());
	for (unsigned i=0;i<m_clouds.size();++i)
		entities[i]=m_clouds[i].pc;

	if (MainWindow::ApplyCCLibAlgortihm(MainWindow::CCLIB_ALGO_CURVATURE,entities,parent,additionalParameters))
	{
		for (unsigned i=0;i<m_clouds.size();++i)
		{
			//save output
			QString errorStr = Export2BIN(m_clouds[i],QString("%1_CURVATURE_KERNEL_%2").arg(curvTypeStr).arg(kernelSize));
			if (!errorStr.isEmpty())
				return Error(errorStr);
		}
	}
	return true;
}

bool ccCommandLineParser::commandDensity(QStringList& arguments, QDialog* parent/*=0*/)
{
	Print("[DENSITY]");
	if (m_clouds.empty())
		return Error(QString("No point cloud on which to compute density! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_DENSITY).arg(COMMAND_DENSITY));

	//Call MainWindow generic method
	ccHObject::Container entities;
	entities.resize(m_clouds.size());
	for (unsigned i=0; i<m_clouds.size(); ++i)
		entities[i] = m_clouds[i].pc;

	if (MainWindow::ApplyCCLibAlgortihm(MainWindow::CCLIB_ALGO_DENSITY,entities,parent))
	{
		for (unsigned i=0;i<m_clouds.size();++i)
		{
			//save output
			QString errorStr = Export2BIN(m_clouds[i],QString("DENSITY"));
			if (!errorStr.isEmpty())
				return Error(errorStr);
		}
	}

	return true;
}

bool ccCommandLineParser::commandSFGradient(QStringList& arguments, QDialog* parent/*=0*/)
{
	Print("[SF GRADIENT]");

	if (arguments.empty())
		return Error(QString("Missing parameter: boolean (whether SF is euclidian or not) after \"-%1\"").arg(COMMAND_SF_GRADIENT));

	QString euclidianStr = arguments.takeFirst().toUpper();
	bool euclidian = false;
	if (euclidianStr == "TRUE")
	{
		euclidian = true;
	}
	else if (euclidianStr != "FALSE")
	{
		return Error(QString("Invalid boolean value after \"-%1\". Got '%2' instead of TRUE or FALSE.").arg(COMMAND_SF_GRADIENT).arg(euclidianStr));
	}

	if (m_clouds.empty())
		return Error(QString("No point cloud on which to compute SF gradient! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN).arg(COMMAND_SF_GRADIENT));

	//Call MainWindow generic method
	void* additionalParameters[1] = {&euclidian};
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
		for (unsigned i=0; i<m_clouds.size(); ++i)
		{
			//save output
			QString errorStr = Export2BIN(m_clouds[i],euclidian ? "EUCLIDIAN_SF_GRAD" : "SF_GRAD");
			if (!errorStr.isEmpty())
				return Error(errorStr);
		}
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
		for (unsigned i=0; i<m_clouds.size(); ++i)
		{
			//save output
			QString errorStr = Export2BIN(m_clouds[i],QString("ROUGHNESS_KERNEL_%2").arg(kernelSize));
			if (!errorStr.isEmpty())
				return Error(errorStr);
		}
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
				Export2BIN(resultDesc,QString("FILTERED_[%1_%2]").arg(thisMinVal).arg(thisMaxVal));

				delete fitleredCloud;
				fitleredCloud = 0;
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
	Export2BIN(m_clouds.front());

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
		Export2BIN(m_clouds.back());
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
	BundlerFilter().loadFileExtended(	qPrintable(bundlerFilename),
										tempContainer,
										false,
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
	
	Export2BIN(compCloud);

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
			return Error(QString("Invalid parameter:  p-value after distribution"));
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
			Export2BIN(m_clouds[i]);
		}
	}

	return true;
}

int ccCommandLineParser::parse(QStringList& arguments, bool silent, QDialog* parent/*=0*/)
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
		else if (IsCommand(argument,COMMAND_FILTER_SF_BY_VALUE))
		{
			success = commandFilterSFByValue(arguments);
		}
		else if (IsCommand(argument,COMMAND_MERGE_CLOUDS))
		{
			success = commandMergeClouds(arguments);
		}
		else if (IsCommand(argument,COMMAND_STAT_TEST))
		{
			success = commandStatTest(arguments,&progressDlg);
		}
		else if (IsCommand(argument,COMMAND_CLEAR_CLOUDS))
		{
			removeClouds();
		}
		else if (IsCommand(argument,COMMAND_CLEAR_MESHES))
		{
			removeMeshes();
		}
		else if (IsCommand(argument,COMMAND_CLEAR))
		{
			removeClouds();
			removeMeshes();
		}
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
