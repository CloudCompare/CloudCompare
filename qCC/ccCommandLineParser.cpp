#include "ccCommandLineParser.h"

//CCLib
#include <CloudSamplingTools.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccGenericMesh.h>
#include <ccProgressDialog.h>
#include <Neighbourhood.h>

//qCC
#include "fileIO/FileIOFilter.h"
#include "fileIO/BundlerFilter.h"
#include <ui_commandLineDlg.h>
#include "ccConsole.h"
#include "mainwindow.h"

//Qt
#include <QMessageBox>
#include <QDialog>
#include <QFileInfo>
#include <QDateTime>
#include <QElapsedTimer>

int ccCommandLineParser::Parse(int nargs, char** args)
{
	if (!args || nargs<2)
	{
		assert(false);
		return EXIT_SUCCESS;
	}

	QDialog consoleDlg;

	bool silent = (QString(args[1]).toUpper() == "-SILENT");
	if (!silent)
	{
		Ui_commandLineDlg commandLineDlg;
		commandLineDlg.setupUi(&consoleDlg);
		consoleDlg.show();
		ccConsole::Init(commandLineDlg.consoleWidget,&consoleDlg);
	}

	int result = ccCommandLineParser().parse(nargs,args,silent,&consoleDlg);

	if (!silent)
	{
		if (result == EXIT_SUCCESS)
			QMessageBox::information(&consoleDlg,"Processed finished","Job done");
		else
			QMessageBox::warning(&consoleDlg,"Processed finished","An error occured! Check console");
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
		delete m_clouds.back().pc;
		m_clouds.pop_back();
	}
}

void ccCommandLineParser::removeMeshes()
{
	while (!m_meshes.empty())
	{
		delete m_meshes.back().first;
		m_meshes.pop_back();
	}
}

void ccCommandLineParser::Print(const QString& message)
{
	ccConsole::Print(message);
	//printf("%s\n",qPrintable(message));
}

int ccCommandLineParser::Error(const QString& message)
{
	ccConsole::Error(message);
	//printf("[ERROR] %s\n",qPrintable(message));

	return EXIT_FAILURE;
}

QString ccCommandLineParser::Export2BIN(CloudDesc& cloudDesc, QString suffix)
{
    assert(cloudDesc.pc);
	QFileInfo info(cloudDesc.filename);

	if (cloudDesc.indexInFile>=0)
		suffix.prepend(QString("%1_").arg(cloudDesc.indexInFile));
	QString cloudName = QString("%1_%2").arg(!cloudDesc.pc->getName().isEmpty() ? cloudDesc.pc->getName() : info.baseName()).arg(suffix);
	cloudDesc.pc->setName(cloudName);

	QString outputFilename = QString("%1/%2_%3_%4.bin").arg(info.path()).arg(info.baseName()).arg(suffix).arg(QDateTime::currentDateTime().toString("yyyy-MM-dd_hh'h'mm"));

	ccHObject group;
	group.addChild(cloudDesc.pc,false);
	if (FileIOFilter::SaveToFile(&group,qPrintable(outputFilename),BIN) != CC_FERR_NO_ERROR)
		return QString("Failed to save result in file '%1'").arg(outputFilename);

	Print(QString("--> result saved to file '%1'").arg(outputFilename));
    return QString();
}

int ccCommandLineParser::parse(int nargs, char** args, bool silent, QDialog* dialog/*=0*/)
{
	assert(nargs>0 && args);
	ccProgressDialog progressDlg(false,dialog);
	//ccProgressDialog* _progressDlg = (silent ? 0 : &progressDlg);
	ccProgressDialog* _progressDlg = &progressDlg;

	QElapsedTimer eTimer;
	eTimer.start();

	int i=1; //first one is always program executable file!
	while (i<nargs)
	{
		QString argument = QString(args[i]).toUpper();
		// "O" OPEN FILE
		if (argument == "-O")
		{
			if (++i==nargs)
				return Error("Missing parameter: filename after \"-O\"");

			//open specified file
			QString filename(args[i]);
			Print(QString("Opening file: '%1'").arg(filename));
			ccHObject* db = FileIOFilter::LoadFromFile(filename,UNKNOWN_FILE,false);
			if (!db)
				return Error(QString("Failed to open file '%1'").arg(filename));

			//look for clouds inside loaded DB
			ccHObject::Container clouds;
			db->filterChildren(clouds,false,CC_POINT_CLOUD);
			unsigned count = clouds.size();
			for (unsigned i=0;i<count;++i)
			{
				ccPointCloud* pc = static_cast<ccPointCloud*>(clouds[0]);
				pc->setFlagState(CC_FATHER_DEPENDANT,false);
				Print(QString("Found one cloud with %1 points").arg(pc->size()));
				m_clouds.push_back(CloudDesc(pc,filename,count == 1 ? -1 : (int)i));
			}

			//look for meshes inside loaded DB
			ccHObject::Container meshes;
			db->filterChildren(meshes,false,CC_MESH);
			if (!meshes.empty())
			{
				ccGenericMesh* mesh = static_cast<ccGenericMesh*>(meshes[0]);
				mesh->setFlagState(CC_FATHER_DEPENDANT,false);
				Print(QString("Found one mesh with %1 faces and %2 vertices").arg(mesh->size()).arg(mesh->getAssociatedCloud()->size()));
				m_meshes.push_back(std::pair<ccGenericMesh*,QString>(mesh,filename));
			}

			delete db;
			db=0;
		}
		// "SS" SUBSAMPLING
		else if (argument == "-SS")
		{
			Print("[SUBSAMPLING]");
			if (m_clouds.empty())
				return Error("No point cloud to resample (be sure to open one with \"-O [cloud filename]\" before \"-SS\")");

			if (++i==nargs)
				return Error("Missing parameter: resampling method after \"-SS\"");

			QString method = QString(args[i]).toUpper();
			Print(QString("\tMethod: ")+method);
			if (method == "RANDOM")
			{
				if (++i==nargs)
					return Error("Missing parameter: number of points after \"-SS RANDOM\"");

				bool ok;
				unsigned count = QString(args[i]).toInt(&ok);
				if (!ok)
					return Error("Invalid number of points for random resampling!");
				Print(QString("\tOutput points: %1").arg(count));

				for (unsigned i=0;i<m_clouds.size();++i)
				{
					ccPointCloud* cloud = m_clouds[i].pc;
					const QString& cloudFilename = m_clouds[i].filename;
					Print(QString("\tProcessing cloud #%1 (%2)").arg(i+1).arg(!cloud->getName().isEmpty() ? cloud->getName() : "no name"));

					CCLib::ReferenceCloud* refCloud = CCLib::CloudSamplingTools::subsampleCloudRandomly(cloud,count,_progressDlg);
					if (!refCloud)
						return Error("Subsampling process failed!");
					Print(QString("\tResult: %1 points").arg(refCloud->size()));

					//save output
					ccPointCloud result(refCloud,cloud);
					delete refCloud;
					refCloud=0;
					CloudDesc cloudDesc(&result,cloudFilename,m_clouds[i].indexInFile);
					QString errorStr = Export2BIN(cloudDesc,"RANDOM_SUBSAMPLED");
					if (!errorStr.isEmpty())
						return Error(errorStr);
				}

			}
			else if (method == "SPATIAL")
			{
				if (++i==nargs)
					return Error("Missing parameter: spatial step after \"-SS SPATIAL\"");

				bool ok;
				double step = QString(args[i]).toDouble(&ok);
				if (!ok || step<=0.0)
					return Error("Invalid step value for spatial resampling!");
				Print(QString("\tSpatial step: %1").arg(step));

				for (unsigned i=0;i<m_clouds.size();++i)
				{
					ccPointCloud* cloud = m_clouds[i].pc;
					const QString& cloudFilename = m_clouds[i].filename;
					Print(QString("\tProcessing cloud #%1 (%2)").arg(i+1).arg(!cloud->getName().isEmpty() ? cloud->getName() : "no name"));

					CCLib::ReferenceCloud* refCloud = CCLib::CloudSamplingTools::resampleCloudSpatially(cloud,step,0,_progressDlg);
					if (!refCloud)
						return Error("Subsampling process failed!");
					Print(QString("\tResult: %1 points").arg(refCloud->size()));

					//save output
					ccPointCloud result(refCloud,cloud);
					delete refCloud;
					refCloud=0;
					CloudDesc cloudDesc(&result,cloudFilename,m_clouds[i].indexInFile);
					QString errorStr = Export2BIN(cloudDesc,"SPATIAL_SUBSAMPLED");
					if (!errorStr.isEmpty())
						return Error(errorStr);
				}
			}
			else if (method == "OCTREE")
			{
				if (++i==nargs)
					return Error("Missing parameter: octree level after \"-SS OCTREE\"");

				bool ok;
				int octreeLevel = QString(args[i]).toInt(&ok);
				if (!ok || octreeLevel<1 || octreeLevel>CCLib::DgmOctree::MAX_OCTREE_LEVEL)
					return Error("Invalid octree level!");
				Print(QString("\tOctree level: %1").arg(octreeLevel));

				for (unsigned i=0;i<m_clouds.size();++i)
				{
					ccPointCloud* cloud = m_clouds[i].pc;
					const QString& cloudFilename = m_clouds[i].filename;
					Print(QString("\tProcessing cloud #%1 (%2)").arg(i+1).arg(!cloud->getName().isEmpty() ? cloud->getName() : "no name"));

					CCLib::ReferenceCloud* refCloud = CCLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(cloud,octreeLevel,CCLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER,_progressDlg);
					if (!refCloud)
						return Error("Subsampling process failed!");
					Print(QString("\tResult: %1 points").arg(refCloud->size()));

					//save output
					ccPointCloud result(refCloud,cloud);
					delete refCloud;
					refCloud=0;
					CloudDesc cloudDesc(&result,cloudFilename,m_clouds[i].indexInFile);
					QString errorStr = Export2BIN(cloudDesc,QString("OCTREE_LEVEL_%1_SUBSAMPLED").arg(octreeLevel));
					if (!errorStr.isEmpty())
						return Error(errorStr);
				}
			}
			else
			{
				return Error("Unknown method!");
			}
		}
		// "CURV" CURVATURE
		else if (argument == "-CURV")
		{
			Print("[CURVATURE]");
			if (m_clouds.empty())
				return Error("No point cloud on which to compute curvature! (be sure to open one with \"-O [cloud filename]\" before \"-CURV\")");

			if (++i==nargs)
				return Error("Missing parameter: curvature type after \"-CURV\"");

			QString curvTypeStr = QString(args[i]).toUpper();
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
				return Error(QString("Invalid curvature type after \"-CURV\". Got '%1' instead of MEAN or GAUSS.").arg(curvTypeStr));
			}

			if (++i==nargs)
				return Error("Missing parameter: kernel size after curvature type");

			bool paramOk=false;
			double kernelSize = QString(args[i]).toDouble(&paramOk);
			if (!paramOk)
				return Error(QString("Failed to read a numerical parameter: kernel size (after curvature type). Got '%1' instead.").arg(args[i]));
			Print(QString("\tKernel size: %1").arg(kernelSize));

			//Call MainWindow generic method
			void* additionalParameters[2] = {&curvType, &kernelSize};
			ccHObject::Container entities;
			entities.resize(m_clouds.size());
			for (unsigned i=0;i<m_clouds.size();++i)
				entities[i]=m_clouds[i].pc;
			
			if (MainWindow::ApplyCCLibAlgortihm(MainWindow::CCLIB_ALGO_CURVATURE,entities,dialog,additionalParameters))
			{
				for (unsigned i=0;i<m_clouds.size();++i)
				{
					//save output
					QString errorStr = Export2BIN(m_clouds[i],QString("%1_CURVATURE_KERNEL_%2").arg(curvTypeStr).arg(kernelSize));
					if (!errorStr.isEmpty())
						return Error(errorStr);
				}
			}
		}
		// "DENSITY"
		else if (argument == "-DENSITY")
		{
			Print("[DENSITY]");
			if (m_clouds.empty())
				return Error("No point cloud on which to compute density! (be sure to open one with \"-O [cloud filename]\" before \"-DENSITY\")");

			//Call MainWindow generic method
			ccHObject::Container entities;
			entities.resize(m_clouds.size());
			for (unsigned i=0;i<m_clouds.size();++i)
				entities[i]=m_clouds[i].pc;
			
			if (MainWindow::ApplyCCLibAlgortihm(MainWindow::CCLIB_ALGO_DENSITY,entities,dialog))
			{
				for (unsigned i=0;i<m_clouds.size();++i)
				{
					//save output
					QString errorStr = Export2BIN(m_clouds[i],QString("DENSITY"));
					if (!errorStr.isEmpty())
						return Error(errorStr);
				}
			}
		}
		// "SF_GRAD" SF GRADIENT
		else if (argument == "-SF_GRAD")
		{
			Print("[SF GRADIENT]");
			if (m_clouds.empty())
				return Error("No point cloud on which to compute SF gradient! (be sure to open one with \"-O [cloud filename]\" before \"-SF_GRAD\")");

			if (++i==nargs)
				return Error("Missing parameter: boolean (whether SF is euclidian or not) after \"-SF_GRAD\"");

			QString euclidianStr = QString(args[i]).toUpper();
			bool euclidian = false;
			if (euclidianStr == "TRUE")
			{
				euclidian = true;
			}
			else if (euclidianStr == "FALSE")
			{
				//euclidian = false;
			}
			else
			{
				return Error(QString("Invalid boolean value after \"-SF_GRAD\". Got '%1' instead of TRUE or FALSE.").arg(euclidianStr));
			}

			//Call MainWindow generic method
			void* additionalParameters[1] = {&euclidian};
			ccHObject::Container entities;
			entities.reserve(m_clouds.size());
			for (unsigned i=0;i<m_clouds.size();++i)
			{
				unsigned sfCount = m_clouds[i].pc->getNumberOfScalarFields();
				if (sfCount==0)
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
			
			if (MainWindow::ApplyCCLibAlgortihm(MainWindow::CCLIB_ALGO_SF_GRADIENT,entities,dialog,additionalParameters))
			{
				for (unsigned i=0;i<m_clouds.size();++i)
				{
					//save output
					QString errorStr = Export2BIN(m_clouds[i],euclidian ? "EUCLIDIAN_SF_GRAD" : "SF_GRAD");
					if (!errorStr.isEmpty())
						return Error(errorStr);
				}
			}
		}
		// "ROUGH" ROUGHNESS
		else if (argument == "-ROUGH")
		{
			Print("[ROUGHNESS]");
			if (m_clouds.empty())
				return Error("No point cloud on which to compute roughness! (be sure to open one with \"-O [cloud filename]\" before \"-ROUGH\")");

			if (++i==nargs)
				return Error("Missing parameter: kernel size after \"-ROUGH\"");

			bool paramOk=false;
			float kernelSize = QString(args[i]).toFloat(&paramOk);
			if (!paramOk)
				return Error(QString("Failed to read a numerical parameter: kernel size (after \"-ROUGH\"). Got '%1' instead.").arg(args[i]));
			Print(QString("\tKernel size: %1").arg(kernelSize));

			//Call MainWindow generic method
			void* additionalParameters[1] = {&kernelSize};
			ccHObject::Container entities;
			entities.resize(m_clouds.size());
			for (unsigned i=0;i<m_clouds.size();++i)
				entities[i]=m_clouds[i].pc;
			
			if (MainWindow::ApplyCCLibAlgortihm(MainWindow::CCLIB_ALGO_ROUGHNESS,entities,dialog,additionalParameters))
			{
				for (unsigned i=0;i<m_clouds.size();++i)
				{
					//save output
					QString errorStr = Export2BIN(m_clouds[i],QString("ROUGHNESS_KERNEL_%2").arg(kernelSize));
					if (!errorStr.isEmpty())
						return Error(errorStr);
				}
			}
		}
		else if (argument == "-BUNDLER_IMPORT") //Import Bundler file + orthorectification
		{
			if (++i==nargs)
				return Error("Missing parameter: filename after \"-BUNDLER_IMPORT\"");

			//open specified file
			QString bundlerFilename(args[i]);
			Print(QString("Importing Bundler file: '%1'").arg(bundlerFilename));
			//ccHObject* db = FileIOFilter::LoadFromFile(filename,UNKNOWN_FILE,false);
			//if (!db)
			//	return Error(QString("Failed to open file '%1'").arg(filename));

			QString altKeypointsFilename;
			bool undistortImages = false;
			bool generateColoredDTM = false;
			unsigned coloredDTMVerticesCount = 0;
			float scaleFactor = 1.0f;

			//inner loop for Bundler import options
			while (i+1<nargs)
			{
				QString argument = QString(args[i+1]).toUpper();
				if (argument == "-ALT_KEYPOINTS")
				{
					++i; //local option confirmed, we can move on
					if (++i==nargs)
						return Error("Missing parameter: filename after \"-ALT_KEYPOINTS\"");
					altKeypointsFilename = QString(args[i]);
				}
				else if (argument == "-SCALE_FACTOR")
				{
					++i; //local option confirmed, we can move on
					if (++i==nargs)
						return Error("Missing parameter: value after \"-SCALE_FACTOR\"");
					bool conversionOk=false;
					scaleFactor = QString(args[i]).toFloat(&conversionOk);
					if (!conversionOk)
						return Error("Invalid parameter: value after \"-SCALE_FACTOR\"");
				}
				else if (argument == "-UNDISTORT")
				{
					++i; //local option confirmed, we can move on
					undistortImages=true;
				}
				else if (argument == "-COLOR_DTM")
				{
					++i; //local option confirmed, we can move on
					if (++i==nargs)
						return Error("Missing parameter: vertices count after \"-COLOR_DTM\"");
					bool conversionOk=false;
					coloredDTMVerticesCount = QString(args[i]).toUInt(&conversionOk);
					if (!conversionOk)
						return Error("Invalid parameter: vertices count after \"-COLOR_DTM\"");
					generateColoredDTM=true;
				}
				else
				{
					break; //as soon as we encounter an unrecognized argument, we break the local loop to go back on the main one!
				}
				++i;
			}

			BundlerFilter bf;
			ccHObject tempContainer;
			bf.loadFileExtended(qPrintable(bundlerFilename),tempContainer,false,altKeypointsFilename,undistortImages,generateColoredDTM,coloredDTMVerticesCount,scaleFactor);
		
		}
		else if (argument == "-CLEAR_CLOUDS")
		{
            removeClouds();
		}
		else if (argument == "-CLEAR_MESHES")
		{
            removeMeshes();
		}
		else if (argument == "-CLEAR")
		{
            removeClouds();
            removeMeshes();
		}
		else
		{
			ccConsole::Error(QString("Unknown or misplaced command: '%1'").arg(argument));
		}

		++i;
	}

	ccConsole::Print("Processed finished in %.2f s.",eTimer.elapsed()/1.0e3);

	return EXIT_SUCCESS;
}
