// qE57IO
#include <E57Filter.h>

// qCC_db
#include <ccHObject.h>
#include <ccPointCloud.h>

// qCC_io
#include <FileIO.h>
#include <FileIOFilter.h>

// Qt
#include <QDir>
#include <QObject>
#include <QTemporaryDir>
#include <QTest>

// System
#include <cmath>

static constexpr unsigned PointCount = 20;

// the filter stores the points as 32-bit floats (in the scan coordinate system)
static constexpr double CoordTolerance = 1.0e-4;

// normals are compressed in memory (see ccNormalCompressor), so a reloaded normal may fall in a neighboring cell
static constexpr double NormalTolerance = 0.01;

// sensor pose, in the format of ccGLMatrixd::toString: a rotation of 90 degrees about Z, then a translation
static const char* SensorPose = "0 -1 0 1\n1 0 0 2\n0 0 1 3\n0 0 0 1";

static ccPointCloud* CreateCloud()
{
	ccPointCloud* cloud = new ccPointCloud("cloud");
	cloud->reserve(PointCount);
	cloud->reserveTheNormsTable();
	for (unsigned i = 0; i < PointCount; ++i)
	{
		cloud->addPoint(CCVector3(1.0f + i * 0.1f, 10.0f + i * 0.37f, static_cast<PointCoordinateType>(-3.0 + sin(i))));
		// the normals are not aligned with Z, so that a missing rotation changes them
		double    a = i * 0.1;
		CCVector3 N(static_cast<PointCoordinateType>(cos(a)), static_cast<PointCoordinateType>(sin(a)), 0.5f);
		N.normalize();
		cloud->addNorm(N);
	}
	return cloud;
}

static CC_FILE_ERROR Save(FileIOFilter& filter, ccHObject* entity, const QString& filename)
{
	FileIOFilter::SaveParameters saveParams;
	saveParams.alwaysDisplaySaveDialog = false;
	return filter.saveToFile(entity, filename, saveParams);
}

static CC_FILE_ERROR Load(FileIOFilter& filter, const QString& filename, ccHObject& container)
{
	FileIOFilter::LoadParameters loadParams;
	CCVector3d                   shift(0, 0, 0);
	bool                         shiftEnabled = false;
	loadParams.alwaysDisplayLoadDialog        = false;
	loadParams.shiftHandlingMode              = ccGlobalShiftManager::Mode::NO_DIALOG;
	loadParams._coordinatesShiftEnabled       = &shiftEnabled;
	loadParams._coordinatesShift              = &shift;
	return filter.loadFile(filename, container, loadParams);
}

// Saves a small generated cloud with normals, loads it back and checks that the points and the
// normals did not move. With a sensor pose, the filter saves them in the scan coordinate system
// and the loader applies the pose again.
class TestE57Filter : public QObject
{
	Q_OBJECT

  private slots:
	void initTestCase()
	{
		FileIO::setWriterInfo("TestE57Filter", "1.0"); // done by ccApplication in the real application
	}

	void cloudRoundTrip_data()
	{
		QTest::addColumn<bool>("withPose");

		QTest::newRow("no pose") << false;
		QTest::newRow("sensor pose") << true;
	}

	void cloudRoundTrip()
	{
		QFETCH(bool, withPose);

		QTemporaryDir dir;
		QVERIFY(dir.isValid());
		const QString filename = QDir(dir.path()).filePath("cloud.e57");

		ccHObject     originalContainer;
		ccPointCloud* original = CreateCloud();
		originalContainer.addChild(original);
		if (withPose)
		{
			original->setMetaData("E57_pose", QString(SensorPose)); // the key the E57 loader uses to store the sensor pose
		}

		E57Filter filter;
		QCOMPARE(Save(filter, original, filename), CC_FERR_NO_ERROR);

		ccHObject container;
		QCOMPARE(Load(filter, filename, container), CC_FERR_NO_ERROR);

		ccHObject::Container clouds;
		container.filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, true);
		QCOMPARE(clouds.size(), static_cast<size_t>(1));
		ccPointCloud* loadedCloud = static_cast<ccPointCloud*>(clouds.front());

		QCOMPARE(loadedCloud->size(), PointCount);
		QVERIFY(loadedCloud->hasNormals());

		for (unsigned i = 0; i < PointCount; ++i)
		{
			CCVector3d Pg = original->toGlobal3d<PointCoordinateType>(*original->getPoint(i));
			CCVector3d Qg = loadedCloud->toGlobal3d<PointCoordinateType>(*loadedCloud->getPoint(i));
			for (unsigned d = 0; d < 3; ++d)
			{
				QVERIFY(std::abs(Qg.u[d] - Pg.u[d]) <= CoordTolerance);
			}

			const CCVector3& N  = original->getPointNormal(i);
			const CCVector3& LN = loadedCloud->getPointNormal(i);
			for (unsigned d = 0; d < 3; ++d)
			{
				QVERIFY(std::abs(LN.u[d] - N.u[d]) <= NormalTolerance);
			}
		}
	}
};

QTEST_MAIN(TestE57Filter)
#include "TestE57Filter.moc"
