
#ifndef CC_TEST_SHAPEFILE_HEADER
#define CC_TEST_SHAPEFILE_HEADER

#include <QObject>
#include <QtTest/QtTest>

#define LINE_FILE ":/TestFiles/Data/shp/line.shp"
#define LINEM_FILE ":/TestFiles/Data/shp/linem.shp"
#define LINEZ_FILE ":/TestFiles/Data/shp/linez.shp"
#define MULTIPOINT_FILE ":/TestFiles/Data/shp/multipoint.shp"
#define MULTIPOINT_Z ":/TestFiles/Data/shp/multi_pointz.shp"
#define MULTIPATCH_FILE ":/TestFiles/Data/shp/multipatch.shp"
#define POLYGON_FILE ":/TestFiles/Data/shp/polygon.shp"
#define POLYGONZ_FILE ":/TestFiles/Data/shp/polygon_z.shp"
#define POINTZ_FILE ":/TestFiles/Data/shp/point_z.shp"

class TestShpFilter : public QObject
{
Q_OBJECT
private slots:
	/* Reading tests */
	void readPolylineFile(const QString &filePath = LINE_FILE) const;

	void readPolylineMFile(const QString &filePath = LINEM_FILE) const;

	void readPolylineZFile(const QString &filePath = LINEZ_FILE) const;

	void readMultiPointFile(const QString &filePath = MULTIPOINT_FILE) const;

	void readMultiPointZFile(const QString &filePath = MULTIPOINT_Z) const;

	void readMultipatchFile(const QString &filePath = MULTIPATCH_FILE) const;

	void readPolygonFile(const QString &filePath = POLYGON_FILE) const;

	void readPolygonZFile(const QString &filePath = POLYGONZ_FILE) const;

	void readSinglePointZFile(const QString &filePath = POINTZ_FILE) const;

	/*
	 * Writing Tests, these tests do a cycle:
	 * 1) read original file
	 * 2) write it back
	 * 3) use its corresponding read test (functions above) to check the result of the writer
	 */
	void testWritePolyline() const;

	void testWritePolylineM() const;

	void testWritePolylineZ() const;

	void testWriteMultiPointFile() const;

	void testWriteMultiPointZFile() const;

	void testWriteMultpatchFile() const;

	void testWritePolygonFile() const;

	void testWritePolygonZFile() const;
};


#endif //CC_TEST_SHAPEFILE_HEADER
