// qCC_db
#include <ccHObject.h>
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>

// qCC_io
#include <FileIOFilter.h>
#include <ShpFilter.h>

// Qt
#include <QObject>
#include <QtTest/QtTest>

// System
#include <array>

#define LINE_FILE ":/TestFiles/Data/shp/line.shp"
#define LINEM_FILE ":/TestFiles/Data/shp/linem.shp"
#define LINEZ_FILE ":/TestFiles/Data/shp/linez.shp"
#define MULTIPOINT_FILE ":/TestFiles/Data/shp/multipoint.shp"
#define MULTIPOINT_Z ":/TestFiles/Data/shp/multi_pointz.shp"
#define MULTIPATCH_FILE ":/TestFiles/Data/shp/multipatch.shp"
#define POLYGON_FILE ":/TestFiles/Data/shp/polygon.shp"
#define POLYGONZ_FILE ":/TestFiles/Data/shp/polygon_z.shp"
#define POINTZ_FILE ":/TestFiles/Data/shp/point_z.shp"

#define COMPARE_LARGE_COORDINATE(testedValue, trueValue, Precision) QVERIFY(std::abs(testedValue - trueValue) < Precision)

static void SetDefaultLoadParameters(FileIOFilter::LoadParameters& params, CCVector3d& shift, bool& shiftEnabled)
{
	params.alwaysDisplayLoadDialog  = false;
	params.shiftHandlingMode        = ccGlobalShiftManager::Mode::NO_DIALOG;
	params._coordinatesShiftEnabled = &shiftEnabled;
	params._coordinatesShift        = &shift;
	params.preserveShiftOnSave      = true;
}

class TestShpFilter : public QObject
{
	Q_OBJECT

  private slots:

	void readPolylineFile(const QString& filePath) const
	{
		ccHObject                    container;
		FileIOFilter::LoadParameters params;
		ShpFilter                    filter;
		CC_FILE_ERROR                error = filter.loadFile(filePath, container, params);
		QVERIFY(error == CC_FERR_NO_ERROR);

		QVERIFY(container.getChildrenNumber() == 2);

		for (unsigned i = 0; i < container.getChildrenNumber(); ++i)
		{
			ccHObject* child = container.getChild(i);
			QVERIFY(child->isA(CC_TYPES::POLY_LINE));
		}

		auto* firstPolyline = static_cast<ccPolyline*>(container.getChild(0));
		QVERIFY(!firstPolyline->is2DMode());
		QVERIFY(!firstPolyline->isClosed());
		auto* vertices = firstPolyline->getAssociatedCloud();
		QVERIFY(!vertices->isScalarFieldEnabled());
		QVERIFY(vertices->size() == 5);

		PointCoordinateType expectedXs[5]{1.0, 5.0, 5.0, 3.0, 1.0};
		PointCoordinateType expectedYs[5]{5.0, 5.0, 1.0, 3.0, 1.0};
		for (unsigned i = 0; i < 5; ++i)
		{
			const CCVector3* point = vertices->getPoint(i);
			QCOMPARE(point->x, expectedXs[i]);
			QCOMPARE(point->y, expectedYs[i]);
			QCOMPARE(point->z, 0.0);
		}

		auto* secondPolyline = static_cast<ccPolyline*>(container.getChild(1));
		QVERIFY(!secondPolyline->is2DMode());
		QVERIFY(!firstPolyline->isClosed());
		vertices = secondPolyline->getAssociatedCloud();
		QVERIFY(!vertices->isScalarFieldEnabled());
		QVERIFY(vertices->size() == 2);

		PointCoordinateType expectedXs2[2]{3.0, 2.0};
		PointCoordinateType expectedYs2[2]{2.0, 6.0};
		for (unsigned i = 0; i < 2; ++i)
		{
			const CCVector3* point = vertices->getPoint(i);
			QCOMPARE(point->x, expectedXs2[i]);
			QCOMPARE(point->y, expectedYs2[i]);
			QCOMPARE(point->z, 0.0);
		}
	}

	void readPolylineMFile(const QString& filePath) const
	{
		ccHObject                    container;
		FileIOFilter::LoadParameters params;
		ShpFilter                    filter;
		CC_FILE_ERROR                error = filter.loadFile(filePath, container, params);
		QVERIFY(error == CC_FERR_NO_ERROR);

		QVERIFY(container.getChildrenNumber() == 2);

		for (unsigned i = 0; i < container.getChildrenNumber(); ++i)
		{
			ccHObject* child = container.getChild(i);
			QVERIFY(child->isA(CC_TYPES::POLY_LINE));
		}

		auto* firstPolyline = static_cast<ccPolyline*>(container.getChild(0));
		QVERIFY(!firstPolyline->isClosed());
		QVERIFY(!firstPolyline->is2DMode());
		auto* vertices = firstPolyline->getAssociatedCloud();
		QVERIFY(vertices->size() == 5);

		PointCoordinateType expectedXs[5]{1.0, 5.0, 5.0, 3.0, 1.0};
		PointCoordinateType expectedYs[5]{5.0, 5.0, 1.0, 3.0, 1.0};
		for (unsigned i = 0; i < 5; ++i)
		{
			const CCVector3* point = vertices->getPoint(i);
			QCOMPARE(point->x, expectedXs[i]);
			QCOMPARE(point->y, expectedYs[i]);
			QCOMPARE(point->z, 0.0);
		}

		if (vertices->isScalarFieldEnabled())
		{
			QCOMPARE(vertices->getPointScalarValue(0), 0.0);
			QVERIFY(std::isnan(vertices->getPointScalarValue(1)));
			QCOMPARE(vertices->getPointScalarValue(2), 3.0);
			QVERIFY(std::isnan(vertices->getPointScalarValue(3)));
			QCOMPARE(vertices->getPointScalarValue(4), 0.0);
		}

		auto* secondPolyline = static_cast<ccPolyline*>(container.getChild(1));
		QVERIFY(!secondPolyline->isClosed());
		QVERIFY(!secondPolyline->is2DMode());
		vertices = secondPolyline->getAssociatedCloud();
		QVERIFY(vertices->size() == 2);
		QVERIFY(!vertices->isScalarFieldEnabled()); // All values are nan, so no scalarfield created

		PointCoordinateType expectedXs2[2]{3.0, 2.0};
		PointCoordinateType expectedYs2[2]{2.0, 6.0};
		for (unsigned i = 0; i < 2; ++i)
		{
			const CCVector3* point = vertices->getPoint(i);
			QCOMPARE(point->x, expectedXs2[i]);
			QCOMPARE(point->y, expectedYs2[i]);
			QCOMPARE(point->z, 0.0);
		}
	}

	void readPolylineZFile(const QString& filePath) const
	{
		ccHObject                    container;
		FileIOFilter::LoadParameters params;
		ShpFilter                    filter;
		CC_FILE_ERROR                error = filter.loadFile(filePath, container, params);
		QVERIFY(error == CC_FERR_NO_ERROR);

		QVERIFY(container.getChildrenNumber() == 3);
		for (unsigned i = 0; i < container.getChildrenNumber(); ++i)
		{
			ccHObject* child = container.getChild(i);
			QVERIFY(child->isA(CC_TYPES::POLY_LINE));
		}

		// 1st part of the polyline
		auto* firstPolyline = static_cast<ccPolyline*>(container.getChild(0));
		QVERIFY(!firstPolyline->isClosed());
		QVERIFY(!firstPolyline->is2DMode());
		auto* vertices = firstPolyline->getAssociatedCloud();
		QVERIFY(vertices->size() == 5);
		QVERIFY(!vertices->isScalarFieldEnabled()); // All values are nan, so no scalarfield created

		// 2nd part of the polyline
		auto* secondPolyline = static_cast<ccPolyline*>(container.getChild(1));
		QVERIFY(!secondPolyline->isClosed());
		QVERIFY(!secondPolyline->is2DMode());
		vertices = secondPolyline->getAssociatedCloud();
		QVERIFY(vertices->size() == 2);
		QVERIFY(!vertices->isScalarFieldEnabled()); // All values are nan, so no scalarfield created

		// 3rd part of the polyline
		auto* thirdPolyline = static_cast<ccPolyline*>(container.getChild(2));
		QVERIFY(!thirdPolyline->isClosed());
		QVERIFY(!thirdPolyline->is2DMode());
		vertices = thirdPolyline->getAssociatedCloud();
		QVERIFY(vertices->size() == 3);

		if (vertices->isScalarFieldEnabled())
		{
			QCOMPARE(vertices->getPointScalarValue(0), 0.0);
			QCOMPARE(vertices->getPointScalarValue(1), 3.0);
			QCOMPARE(vertices->getPointScalarValue(2), 2.0);
		}
	}

	void readMultiPointFile(const QString& filePath) const
	{
		ccHObject                    container;
		FileIOFilter::LoadParameters params;
		params.alwaysDisplayLoadDialog = false;
		ShpFilter filter;

		CC_FILE_ERROR error = filter.loadFile(filePath, container, params);
		QVERIFY(error == CC_FERR_NO_ERROR);

		QVERIFY(container.getChildrenNumber() == 1);
		QVERIFY(container.getChild(0)->isA(CC_TYPES::POINT_CLOUD));

		auto* cloud = static_cast<ccPointCloud*>(container.getChild(0));
		QVERIFY(cloud->size() == 2);
		QVERIFY(!cloud->isScalarFieldEnabled());

		const CCVector3* point = cloud->getPoint(0);
		QCOMPARE(point->x, 122.0);
		QCOMPARE(point->y, 37.0);
		QCOMPARE(point->z, 0.0);

		point = cloud->getPoint(1);
		QCOMPARE(point->x, 124.0);
		QCOMPARE(point->y, 32.0);
		QCOMPARE(point->z, 0.0);
	}

	void readMultiPointZFile(const QString& filePath) const
	{
		CCVector3d bbMin(1422671.7232666016, 4188903.4295959473, 71.99445343017578);
		CCVector3d shift        = ccGlobalShiftManager::BestShift(bbMin);
		bool       shiftEnabled = true;

		ccHObject                    container;
		FileIOFilter::LoadParameters params;
		SetDefaultLoadParameters(params, shift, shiftEnabled);
		ShpFilter filter;

		CC_FILE_ERROR error = filter.loadFile(filePath, container, params);
		QVERIFY(error == CC_FERR_NO_ERROR);

		QVERIFY(container.getChildrenNumber() == 1);
		ccHObject* item = container.getFirstChild();
		QVERIFY(item->isA(CC_TYPES::POINT_CLOUD));

		auto* pointCloud = static_cast<ccPointCloud*>(item);
		QVERIFY(pointCloud->size() == 4);
		QVERIFY(!pointCloud->isScalarFieldEnabled());

		static const double LargeCoorTestPrecision = 1.0e-4; // numerical accuracy can't be much better than 10^-4 due to the
		                                                     // Global Shift mechanism. It leaves X and Y coordinates around 10^3

		const CCVector3* point;
		CCVector3d       pg;

		point = pointCloud->getPoint(0);
		pg    = pointCloud->toGlobal3d(*point);
		COMPARE_LARGE_COORDINATE(pg.x, 1422671.7232666016, LargeCoorTestPrecision);
		COMPARE_LARGE_COORDINATE(pg.y, 4188903.4295959473, LargeCoorTestPrecision);
		QVERIFY(qFuzzyCompare(pg.z, 72.00995635986328));

		point = pointCloud->getPoint(1);
		pg    = pointCloud->toGlobal3d(*point);
		COMPARE_LARGE_COORDINATE(pg.x, 1422672.1022949219, LargeCoorTestPrecision);
		COMPARE_LARGE_COORDINATE(pg.y, 4188903.4295959473, LargeCoorTestPrecision);
		QVERIFY(qFuzzyCompare(pg.z, 72.0060806274414));

		point = pointCloud->getPoint(2);
		pg    = pointCloud->toGlobal3d(*point);
		COMPARE_LARGE_COORDINATE(pg.x, 1422671.9127807617, LargeCoorTestPrecision);
		COMPARE_LARGE_COORDINATE(pg.y, 4188903.7578430176, LargeCoorTestPrecision);
		QVERIFY(qFuzzyCompare(pg.z, 72.00220489501953));

		point = pointCloud->getPoint(3);
		pg    = pointCloud->toGlobal3d(*point);
		COMPARE_LARGE_COORDINATE(pg.x, 1422671.9127807617, LargeCoorTestPrecision);
		COMPARE_LARGE_COORDINATE(pg.y, 4188903.539001465, LargeCoorTestPrecision);
		QVERIFY(qFuzzyCompare(pg.z, 71.99445343017578));
	}

	void readMultipatchFile(const QString& filePath) const
	{
		ccHObject                    container;
		FileIOFilter::LoadParameters params;
		params.alwaysDisplayLoadDialog = false;
		ShpFilter filter;

		CC_FILE_ERROR error = filter.loadFile(filePath, container, params);
		QVERIFY(error == CC_FERR_NO_ERROR);

		QVERIFY(container.getChildrenNumber() == 2);

		for (unsigned i = 0; i < container.getChildrenNumber(); ++i)
		{
			QVERIFY(container.getChild(i)->isA(CC_TYPES::MESH));
		}

		constexpr unsigned expectedNumPoints = 10;
		auto*              firstMesh         = static_cast<ccMesh*>(container.getChild(0));
		auto*              vertices          = firstMesh->getAssociatedCloud();
		QVERIFY(vertices->size() == expectedNumPoints);
		PointCoordinateType expectedXs[expectedNumPoints]{0.0, 0.0, 5.0, 5.0, 5.0, 5.0, 0.0, 0.0, 0.0, 0.0};
		PointCoordinateType expectedYs[expectedNumPoints]{0.0, 0.0, 0.0, 0.0, 5.0, 5.0, 5.0, 5.0, 0.0, 0.0};
		PointCoordinateType expectedZs[expectedNumPoints]{0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0, 0.0, 3.0};
		for (unsigned i = 0; i < expectedNumPoints; ++i)
		{
			const CCVector3* point = vertices->getPoint(i);
			QCOMPARE(point->x, expectedXs[i]);
			QCOMPARE(point->y, expectedYs[i]);
			QCOMPARE(point->z, expectedZs[i]);
		}

		constexpr unsigned expectedNumPoints2 = 6;
		auto*              secondMesh         = static_cast<ccMesh*>(container.getChild(1));
		vertices                              = secondMesh->getAssociatedCloud();
		QVERIFY(vertices->size() == 6);
		PointCoordinateType expectedXs2[expectedNumPoints2]{2.5, 0.0, 5.0, 5.0, 0.0, 0.0};
		PointCoordinateType expectedYs2[expectedNumPoints2]{2.5, 0.0, 0.0, 5.0, 5.0, 0.0};
		PointCoordinateType expectedZs2[expectedNumPoints2]{5.0, 3.0, 3.0, 3.0, 3.0, 3.0};
		for (unsigned i = 0; i < expectedNumPoints2; ++i)
		{
			const CCVector3* point = vertices->getPoint(i);
			QCOMPARE(point->x, expectedXs2[i]);
			QCOMPARE(point->y, expectedYs2[i]);
			QCOMPARE(point->z, expectedZs2[i]);
		}
	}

	void readPolygonFile(const QString& filePath) const
	{
		const unsigned expectedNumPoints = 14; // File has 15 points but as its a polygon, CC will keep the 14 first pts
		CCVector3d     bbMin(-626146.0444521683, 5219675.646154184, 0);
		CCVector3d     shift        = ccGlobalShiftManager::BestShift(bbMin);
		bool           shiftEnabled = true;

		ccHObject                    container;
		FileIOFilter::LoadParameters params;
		SetDefaultLoadParameters(params, shift, shiftEnabled);
		ShpFilter filter;

		CC_FILE_ERROR error = filter.loadFile(filePath, container, params);
		QVERIFY(error == CC_FERR_NO_ERROR);

		QVERIFY(container.getChildrenNumber() == 1);
		ccHObject* item = container.getFirstChild();
		QVERIFY(item->isA(CC_TYPES::POLY_LINE));

		auto poly = static_cast<ccPolyline*>(item);
		QVERIFY(poly->size() == expectedNumPoints);
		QVERIFY(!poly->isScalarFieldEnabled());
		QVERIFY(!poly->is2DMode());
		QVERIFY(poly->isClosed());

		CCCoreLib::GenericIndexedCloudPersist* vertices = poly->getAssociatedCloud();
		auto                                   cloud    = dynamic_cast<ccPointCloud*>(vertices);
		QVERIFY(nullptr != cloud);
		QVERIFY(cloud->size() == expectedNumPoints);

		std::array<double, 14> expectedXs{-626146.0444521683, -187004.53123683017, -59884.61951660062, 169316.43343351, 180872.78904444003, 300288.4636907161, 914701.3703384919, 752912.3917854726, 880032.303505702, 749060.273248496, 473633.79785466543, 375404.77516176086, -212043.3017271784, -187004.53123683017};
		std::array<double, 14> expectedYs{6182705.280398346, 6409980.274079968, 6383015.444321131, 6488948.704087989, 6606438.319465778, 6650737.682641009, 6236634.939916019, 5878387.91597719, 5391094.921049644, 5271679.246403368, 5352573.735679878, 5219675.646154184, 5348721.617142901, 5789789.189626727};

		static const double LargeCoorTestPrecision = 1.0e-1; // numerical accuracy can't be much better than 10^-1 due to the
		                                                     // Global Shift mechanism. It leaves X and Y coordinates slightly below 10^6

		for (unsigned i = 0; i < expectedNumPoints; ++i)
		{
			const CCVector3d pg = cloud->toGlobal3d(*cloud->getPoint(i));
			COMPARE_LARGE_COORDINATE(pg.x, expectedXs[i], LargeCoorTestPrecision);
			COMPARE_LARGE_COORDINATE(pg.y, expectedYs[i], LargeCoorTestPrecision);
			QCOMPARE(pg.z, 0.0);
		}
	}

	void readPolygonZFile(const QString& filePath) const
	{
		CCVector3d bbMin(1422671.7232666016, 4188903.4295959473, 71.99445343017578);
		CCVector3d shift        = ccGlobalShiftManager::BestShift(bbMin);
		bool       shiftEnabled = true;

		ccHObject                    container;
		FileIOFilter::LoadParameters params;
		SetDefaultLoadParameters(params, shift, shiftEnabled);
		ShpFilter filter;

		CC_FILE_ERROR error = filter.loadFile(filePath, container, params);
		QVERIFY(error == CC_FERR_NO_ERROR);

		QVERIFY(container.getChildrenNumber() == 1);
		QVERIFY(container.getFirstChild()->isA(CC_TYPES::POLY_LINE));

		auto* polygon = static_cast<ccPolyline*>(container.getFirstChild());
		QVERIFY(polygon->size() == 72); // File has 73 points, but cc reads 72 as its a polygon
		QVERIFY(!polygon->is2DMode());
		QVERIFY(polygon->isClosed());
	}

	void testWritePolyline() const
	{
		ccHObject                    container;
		FileIOFilter::LoadParameters params;
		ShpFilter                    filter;
		CC_FILE_ERROR                error = filter.loadFile(LINE_FILE, container, params);
		QVERIFY(error == CC_FERR_NO_ERROR);

		QTemporaryDir                tmpDir;
		QString                      tmpLineM = tmpDir.path() + "line.shp";
		FileIOFilter::SaveParameters saveParams;
		saveParams.alwaysDisplaySaveDialog = false;
		filter.save3DPolyAs2D(true);
		filter.save3DPolyHeightInDBF(false);

		error = filter.saveToFile(&container, tmpLineM, saveParams);
		QVERIFY(error == CC_FERR_NO_ERROR);

		readPolylineFile(tmpLineM);
	}

	void testWritePolylineM() const
	{
		ccHObject                    container;
		FileIOFilter::LoadParameters params;
		ShpFilter                    filter;
		CC_FILE_ERROR                error = filter.loadFile(LINEM_FILE, container, params);
		QVERIFY(error == CC_FERR_NO_ERROR);

		QTemporaryDir                tmpDir;
		QString                      tmpLineM = tmpDir.path() + "linem.shp";
		FileIOFilter::SaveParameters saveParams;
		saveParams.alwaysDisplaySaveDialog = false;

		error = filter.saveToFile(&container, tmpLineM, saveParams);
		QVERIFY(error == CC_FERR_NO_ERROR);

		readPolylineMFile(tmpLineM);
	}

	void testWritePolylineZ() const
	{
		ccHObject                    container;
		FileIOFilter::LoadParameters params;
		ShpFilter                    filter;
		CC_FILE_ERROR                error = filter.loadFile(LINEZ_FILE, container, params);
		QVERIFY(error == CC_FERR_NO_ERROR);

		QTemporaryDir                tmpDir;
		QString                      tmpLineM = tmpDir.path() + "linez.shp";
		FileIOFilter::SaveParameters saveParams;
		saveParams.alwaysDisplaySaveDialog = false;

		error = filter.saveToFile(&container, tmpLineM, saveParams);
		QVERIFY(error == CC_FERR_NO_ERROR);

		readPolylineZFile(tmpLineM);
	}

	void testWriteMultpatchFile() const
	{
		ccHObject                    container;
		FileIOFilter::LoadParameters params;
		ShpFilter                    filter;
		CC_FILE_ERROR                error = filter.loadFile(MULTIPATCH_FILE, container, params);
		QVERIFY(error == CC_FERR_NO_ERROR);

		QTemporaryDir                tmpDir;
		QString                      tmpMultipatch = tmpDir.path() + "line.shp";
		FileIOFilter::SaveParameters saveParams;
		saveParams.alwaysDisplaySaveDialog = false;
		filter.save3DPolyAs2D(true);

		error = filter.saveToFile(&container, tmpMultipatch, saveParams);
		QVERIFY(error == CC_FERR_NO_ERROR);
		readMultipatchFile(tmpMultipatch);
	}

	void testWriteMultiPointFile() const
	{
		ccHObject                    container;
		FileIOFilter::LoadParameters params;
		ShpFilter                    filter;

		CC_FILE_ERROR error = filter.loadFile(MULTIPOINT_FILE, container, params);
		QVERIFY(error == CC_FERR_NO_ERROR);

		QTemporaryDir                tmpDir;
		QString                      tmpMultipatch = tmpDir.path() + "multipoint.shp";
		FileIOFilter::SaveParameters saveParams;
		saveParams.alwaysDisplaySaveDialog = false;

		error = filter.saveToFile(&container, tmpMultipatch, saveParams);
		QVERIFY(error == CC_FERR_NO_ERROR);
		readMultiPointFile(tmpMultipatch);
	}

	void testWriteMultiPointZFile() const
	{
		CCVector3d bbMin(1422671.7232666016, 4188903.4295959473, 71.99445343017578);
		CCVector3d shift        = ccGlobalShiftManager::BestShift(bbMin);
		bool       shiftEnabled = true;

		ccHObject                    container;
		FileIOFilter::LoadParameters params;
		SetDefaultLoadParameters(params, shift, shiftEnabled);
		ShpFilter filter;

		CC_FILE_ERROR error = filter.loadFile(MULTIPOINT_Z, container, params);
		QVERIFY(error == CC_FERR_NO_ERROR);

		QTemporaryDir                tmpDir;
		QString                      tmpMultipatch = tmpDir.path() + "multi_pointz.shp";
		FileIOFilter::SaveParameters saveParams;
		saveParams.alwaysDisplaySaveDialog = false;

		error = filter.saveToFile(&container, tmpMultipatch, saveParams);
		QVERIFY(error == CC_FERR_NO_ERROR);
		readMultiPointZFile(tmpMultipatch);
	}

	void testWritePolygonFile() const
	{
		CCVector3d bbMin(-626146.0444521683, 5219675.646154184, 0);
		CCVector3d shift        = ccGlobalShiftManager::BestShift(bbMin);
		bool       shiftEnabled = true;

		ccHObject                    container;
		FileIOFilter::LoadParameters params;
		SetDefaultLoadParameters(params, shift, shiftEnabled);
		ShpFilter filter;

		CC_FILE_ERROR error = filter.loadFile(POLYGON_FILE, container, params);
		QVERIFY(error == CC_FERR_NO_ERROR);

		QTemporaryDir                tmpDir;
		QString                      tmpMultipatch = tmpDir.path() + "polygon.shp";
		FileIOFilter::SaveParameters saveParams;
		saveParams.alwaysDisplaySaveDialog = false;

		filter.save3DPolyAs2D(true);
		filter.treatClosedPolylinesAsPolygons(true);
		error = filter.saveToFile(&container, tmpMultipatch, saveParams);

		QVERIFY(error == CC_FERR_NO_ERROR);
		readPolygonFile(tmpMultipatch);
	}

	void testWritePolygonZFile() const
	{
		CCVector3d bbMin(1422671.7232666016, 4188903.4295959473, 71.99445343017578);
		CCVector3d shift        = ccGlobalShiftManager::BestShift(bbMin);
		bool       shiftEnabled = true;

		ccHObject                    container;
		FileIOFilter::LoadParameters params;
		SetDefaultLoadParameters(params, shift, shiftEnabled);
		ShpFilter filter;

		CC_FILE_ERROR error = filter.loadFile(POLYGONZ_FILE, container, params);
		QVERIFY(error == CC_FERR_NO_ERROR);

		QTemporaryDir                tmpDir;
		QString                      tmpMultipatch = tmpDir.path() + "polygon_z.shp";
		FileIOFilter::SaveParameters saveParams;
		saveParams.alwaysDisplaySaveDialog = false;

		filter.save3DPolyAs2D(false);
		filter.treatClosedPolylinesAsPolygons(true);
		error = filter.saveToFile(&container, tmpMultipatch, saveParams);
		QVERIFY(error == CC_FERR_NO_ERROR);
		readPolygonZFile(tmpMultipatch);
	}

	void readSinglePointZFile(const QString& filePath) const
	{
		CCVector3d bbMin(1422459.0908050265, 4188942.211755641, 0.0);
		CCVector3d shift        = ccGlobalShiftManager::BestShift(bbMin);
		bool       shiftEnabled = true;

		ccHObject                    container;
		FileIOFilter::LoadParameters params;
		SetDefaultLoadParameters(params, shift, shiftEnabled);
		ShpFilter filter;

		CC_FILE_ERROR error = filter.loadFile(filePath, container, params);
		QVERIFY(error == CC_FERR_NO_ERROR);

		QVERIFY(container.getChildrenNumber() == 1);
		QVERIFY(container.getFirstChild()->isA(CC_TYPES::POINT_CLOUD));

		unsigned expectedNumPoints = 2;
		auto*    cloud             = static_cast<ccPointCloud*>(container.getFirstChild());
		QVERIFY(cloud->size() == expectedNumPoints);
		QVERIFY(!cloud->isScalarFieldEnabled());

		std::array<double, 2> expectedXs{1422464.3681007193, 1422459.0908050265};
		std::array<double, 2> expectedYs{4188962.3364355816, 4188942.211755641};
		std::array<double, 2> expectedZs{72.40956470558095, 72.58286959604922};
		for (unsigned i = 0; i < expectedNumPoints; ++i)
		{
			const CCVector3* p = cloud->getPoint(i);
			QCOMPARE(p->x, static_cast<PointCoordinateType>(expectedXs[i] + shift.x));
			QCOMPARE(p->y, static_cast<PointCoordinateType>(expectedYs[i] + shift.y));
			QCOMPARE(p->z, static_cast<PointCoordinateType>(expectedZs[i] + shift.z));
		}
	}
};

QTEST_GUILESS_MAIN(TestShpFilter)
#include "TestShpFilter.moc"
