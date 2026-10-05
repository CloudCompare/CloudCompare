#include "TestFileRoundTrip.h"

// qCC_db
#include <ccHObject.h>
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>

// qCC_io
#include <AsciiFilter.h>
#include <BinFilter.h>
#include <FileIO.h>
#include <FileIOFilter.h>
#include <PlyFilter.h>

// Qt
#include <QTemporaryDir>

// System
#include <cmath>
#include <limits>
#include <memory>

static constexpr unsigned PointCount = 100;

static ccPointCloud* CreateCloud()
{
	ccPointCloud* cloud = new ccPointCloud("cloud");
	cloud->reserve(PointCount);
	cloud->reserveTheRGBTable();
	cloud->reserveTheNormsTable();
	for (unsigned i = 0; i < PointCount; ++i)
	{
		cloud->addPoint(CCVector3(i * 0.1f, 10.0f + i * 0.37f, static_cast<PointCoordinateType>(-3.0 + sin(i))));
		cloud->addColor(static_cast<ColorCompType>(i), static_cast<ColorCompType>((i * 7) % 256), static_cast<ColorCompType>((i * 13) % 256));
		double    a = i * 0.1;
		CCVector3 N(static_cast<PointCoordinateType>(cos(a)), static_cast<PointCoordinateType>(sin(a)), 0.5f);
		N.normalize();
		cloud->addNorm(N);
	}
	int  sfIndex = cloud->addScalarField("Scalar field");
	auto sf      = cloud->getScalarField(sfIndex);
	for (unsigned i = 0; i < PointCount; ++i)
	{
		sf->setValue(i, 100.0 * cos(i * 0.7));
	}
	sf->computeMinAndMax();
	return cloud;
}

static constexpr unsigned GridSize = 10; // the mesh vertices form a GridSize x GridSize grid

static ccMesh* CreateMesh()
{
	ccPointCloud* vertices = new ccPointCloud("vertices");
	vertices->reserve(GridSize * GridSize);
	for (unsigned j = 0; j < GridSize; ++j)
	{
		for (unsigned i = 0; i < GridSize; ++i)
		{
			vertices->addPoint(CCVector3(i * 0.5f, j * 0.5f, static_cast<PointCoordinateType>(sin(i * 0.3) * cos(j * 0.3))));
		}
	}
	vertices->setEnabled(false);

	ccMesh* mesh = new ccMesh(vertices);
	mesh->addChild(vertices);
	mesh->reserve(2 * (GridSize - 1) * (GridSize - 1));
	for (unsigned j = 0; j + 1 < GridSize; ++j)
	{
		for (unsigned i = 0; i + 1 < GridSize; ++i)
		{
			unsigned k = j * GridSize + i;
			mesh->addTriangle(k, k + 1, k + GridSize);
			mesh->addTriangle(k + 1, k + GridSize + 1, k + GridSize);
		}
	}
	return mesh;
}

static constexpr int Exact   = -1; // stored as in memory
static constexpr int Float32 = -2; // stored as 32-bit floats

// allowed error for a value stored with the given precision (or with 'decimals' digits after the point)
static double Tolerance(int decimals, double value)
{
	if (decimals == Exact)
		return 0.0;
	if (decimals == Float32)
		return std::abs(value) * std::numeric_limits<float>::epsilon();
	return 0.5 * std::pow(10.0, -decimals) + std::abs(value) * std::numeric_limits<float>::epsilon();
}

void TestFileRoundTrip::initTestCase()
{
	FileIO::setWriterInfo("TestFileRoundTrip", "1.0"); // done by ccApplication in the real application
	AsciiFilter::SaveColumnsNamesHeader(true);         // so that the columns can be identified without the dialog
}

void TestFileRoundTrip::roundTrip_data()
{
	QTest::addColumn<QString>("extension");
	QTest::addColumn<int>("coordDecimals");
	QTest::addColumn<int>("normalDecimals");
	QTest::addColumn<int>("sfDecimals");

	QTest::newRow("BIN") << "bin" << Exact << Exact << Exact;
	QTest::newRow("PLY") << "ply" << Exact << Exact << Float32; // small SF values are saved as 32-bit floats
	QTest::newRow("ASCII") << "asc" << 8 << 6 << 6;             // AsciiFilter default precisions
}

void TestFileRoundTrip::roundTrip()
{
	QFETCH(QString, extension);
	QFETCH(int, coordDecimals);
	QFETCH(int, normalDecimals);
	QFETCH(int, sfDecimals);

	std::unique_ptr<FileIOFilter> filter;
	if (extension == "bin")
		filter.reset(new BinFilter);
	else if (extension == "ply")
		filter.reset(new PlyFilter);
	else
		filter.reset(new AsciiFilter);

	QTemporaryDir dir;
	QVERIFY(dir.isValid());
	const QString filename = dir.filePath("cloud." + extension);

	std::unique_ptr<ccPointCloud> original(CreateCloud());

	FileIOFilter::SaveParameters saveParams;
	saveParams.alwaysDisplaySaveDialog = false;
	QCOMPARE(filter->saveToFile(original.get(), filename, saveParams), CC_FERR_NO_ERROR);

	ccHObject                    container;
	FileIOFilter::LoadParameters loadParams;
	CCVector3d                   shift(0, 0, 0);
	bool                         shiftEnabled = false;
	loadParams.alwaysDisplayLoadDialog        = false;
	loadParams.shiftHandlingMode              = ccGlobalShiftManager::Mode::NO_DIALOG;
	loadParams._coordinatesShiftEnabled       = &shiftEnabled;
	loadParams._coordinatesShift              = &shift;
	QCOMPARE(filter->loadFile(filename, container, loadParams), CC_FERR_NO_ERROR);

	ccHObject::Container clouds;
	container.filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, true);
	QCOMPARE(clouds.size(), static_cast<size_t>(1));
	ccPointCloud* loaded = static_cast<ccPointCloud*>(clouds.front());

	QCOMPARE(loaded->size(), PointCount);
	QVERIFY(loaded->hasColors());
	QVERIFY(loaded->hasNormals());
	QCOMPARE(loaded->getNumberOfScalarFields(), 1u);
	auto loadedSF = loaded->getScalarField(0);
	QCOMPARE(QString::fromStdString(loadedSF->getName()), QString("Scalar field"));
	auto originalSF = original->getScalarField(0);

	for (unsigned i = 0; i < PointCount; ++i)
	{
		const CCVector3* P  = original->getPoint(i);
		const CCVector3* Q  = loaded->getPoint(i);
		CCVector3d       Pg = original->toGlobal3d<PointCoordinateType>(*P);
		CCVector3d       Qg = loaded->toGlobal3d<PointCoordinateType>(*Q);
		for (unsigned d = 0; d < 3; ++d)
		{
			QVERIFY(std::abs(Qg.u[d] - Pg.u[d]) <= Tolerance(coordDecimals, Pg.u[d]));
		}

		const ccColor::Rgba& c  = original->getPointColor(i);
		const ccColor::Rgba& lc = loaded->getPointColor(i);
		QCOMPARE(lc.r, c.r);
		QCOMPARE(lc.g, c.g);
		QCOMPARE(lc.b, c.b);

		const CCVector3& N  = original->getPointNormal(i);
		const CCVector3& LN = loaded->getPointNormal(i);
		for (unsigned d = 0; d < 3; ++d)
		{
			QVERIFY(std::abs(LN.u[d] - N.u[d]) <= Tolerance(normalDecimals, N.u[d]));
		}

		double v = originalSF->getValue(i);
		QVERIFY(std::abs(loadedSF->getValue(i) - v) <= Tolerance(sfDecimals, v));
	}
}

void TestFileRoundTrip::meshRoundTrip_data()
{
	QTest::addColumn<QString>("extension");

	QTest::newRow("BIN") << "bin";
	QTest::newRow("PLY") << "ply";
}

void TestFileRoundTrip::meshRoundTrip()
{
	QFETCH(QString, extension);

	std::unique_ptr<FileIOFilter> filter;
	if (extension == "bin")
		filter.reset(new BinFilter);
	else
		filter.reset(new PlyFilter);

	QTemporaryDir dir;
	QVERIFY(dir.isValid());
	const QString filename = dir.filePath("mesh." + extension);

	std::unique_ptr<ccMesh> original(CreateMesh());

	FileIOFilter::SaveParameters saveParams;
	saveParams.alwaysDisplaySaveDialog = false;
	QCOMPARE(filter->saveToFile(original.get(), filename, saveParams), CC_FERR_NO_ERROR);

	ccHObject                    container;
	FileIOFilter::LoadParameters loadParams;
	CCVector3d                   shift(0, 0, 0);
	bool                         shiftEnabled = false;
	loadParams.alwaysDisplayLoadDialog        = false;
	loadParams.shiftHandlingMode              = ccGlobalShiftManager::Mode::NO_DIALOG;
	loadParams._coordinatesShiftEnabled       = &shiftEnabled;
	loadParams._coordinatesShift              = &shift;
	QCOMPARE(filter->loadFile(filename, container, loadParams), CC_FERR_NO_ERROR);

	ccHObject::Container meshes;
	container.filterChildren(meshes, true, CC_TYPES::MESH, true);
	QCOMPARE(meshes.size(), static_cast<size_t>(1));
	ccMesh* loaded = static_cast<ccMesh*>(meshes.front());

	ccGenericPointCloud* vertices       = original->getAssociatedCloud();
	ccGenericPointCloud* loadedVertices = loaded->getAssociatedCloud();
	QVERIFY(loadedVertices != nullptr);
	QCOMPARE(loadedVertices->size(), vertices->size());
	QCOMPARE(loaded->size(), original->size());

	for (unsigned i = 0; i < vertices->size(); ++i)
	{
		CCVector3d Pg = vertices->toGlobal3d<PointCoordinateType>(*vertices->getPoint(i));
		CCVector3d Qg = loadedVertices->toGlobal3d<PointCoordinateType>(*loadedVertices->getPoint(i));
		for (unsigned d = 0; d < 3; ++d)
		{
			QCOMPARE(Qg.u[d], Pg.u[d]);
		}
	}

	for (unsigned t = 0; t < original->size(); ++t)
	{
		const CCCoreLib::VerticesIndexes* tri       = original->getTriangleVertIndexes(t);
		const CCCoreLib::VerticesIndexes* loadedTri = loaded->getTriangleVertIndexes(t);
		QCOMPARE(loadedTri->i1, tri->i1);
		QCOMPARE(loadedTri->i2, tri->i2);
		QCOMPARE(loadedTri->i3, tri->i3);
	}
}

QTEST_MAIN(TestFileRoundTrip)
