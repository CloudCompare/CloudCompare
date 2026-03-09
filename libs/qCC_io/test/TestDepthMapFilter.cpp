#include "TestDepthMapFilter.h"

// qCC_db
#include <ccGBLSensor.h>
#include <ccPointCloud.h>

// qCC_io
#include <DepthMapFileFilter.h>
#include <FileIO.h>

// Qt
#include <QDir>
#include <QFileInfo>
#include <QTemporaryDir>

// System
#include <memory>

void TestDepthMapFilter::initTestCase()
{
	FileIO::setWriterInfo("TestDepthMapFilter", "1.0"); // done by ccApplication in the real application
}

void TestDepthMapFilter::multipleSensorsAreSavedNextToTheRequestedFile()
{
	auto cloud = std::make_unique<ccPointCloud>("cloud");
	cloud->reserve(4);
	cloud->addPoint(CCVector3(10, 0, 0));
	cloud->addPoint(CCVector3(10, 1, 0));
	cloud->addPoint(CCVector3(10, 0, 1));
	cloud->addPoint(CCVector3(10, 1, 1));

	// the sensors are children of the cloud, as in the application
	for (int i = 0; i < 2; ++i)
	{
		auto* sensor = new ccGBLSensor;
		sensor->setPitchRange(-0.5f, 0.5f);
		sensor->setYawRange(-0.5f, 0.5f);
		sensor->setPitchStep(0.1f);
		sensor->setYawStep(0.1f);
		cloud->addChild(sensor);
		int errorCode = 0;
		QVERIFY(sensor->computeDepthBuffer(cloud.get(), errorCode));
	}

	QTemporaryDir outputDir;
	QTemporaryDir workingDir;
	QVERIFY(outputDir.isValid() && workingDir.isValid());

	// a stray file is written in the current directory, so we move to an empty one
	const QString previousDir = QDir::currentPath();
	QVERIFY(QDir::setCurrent(workingDir.path()));

	DepthMapFileFilter           filter;
	FileIOFilter::SaveParameters saveParams;
	saveParams.alwaysDisplaySaveDialog = false;
	const CC_FILE_ERROR result         = filter.saveToFile(cloud.get(), outputDir.filePath("depth.txt"), saveParams);

	QVERIFY(QDir::setCurrent(previousDir));

	QCOMPARE(result, CC_FERR_NO_ERROR);
	QVERIFY(QFileInfo::exists(outputDir.filePath("depth_0.txt")));
	QVERIFY(QFileInfo::exists(outputDir.filePath("depth_1.txt")));
	QVERIFY(QDir(workingDir.path()).isEmpty());
}

QTEST_MAIN(TestDepthMapFilter)
