// qCC_db
#include <ccHObject.h>
#include <ccPointCloud.h>

// qCC_io
#include <AsciiFilter.h>
#include <FileIOFilter.h>

// Qt
#include <QFile>
#include <QObject>
#include <QTemporaryDir>
#include <QTest>

static constexpr unsigned PointCount = 5;

//! Returns the content of a small ASCII file (one 'X Y Z' point per line) with the given line ending
static QByteArray CreateContent(const QByteArray& lineEnding)
{
	QByteArray content;
	for (unsigned i = 0; i < PointCount; ++i)
	{
		content += QByteArray::number(1.5 + i) + ' ' + QByteArray::number(10.0 + i) + ' ' + QByteArray::number(-1.0 - 2.25 * i) + lineEnding;
	}
	return content;
}

// Loads the same points with each line ending style, from a file and from memory
// (legacy Mac files use a lone CR)
class TestAsciiFilter : public QObject
{
	Q_OBJECT
  private slots:

	void lineEndings_data()
	{
		QTest::addColumn<QByteArray>("lineEnding");
		QTest::addColumn<bool>("fromFile");

		QTest::newRow("LF file") << QByteArray("\n") << true;
		QTest::newRow("CRLF file") << QByteArray("\r\n") << true;
		QTest::newRow("CR file") << QByteArray("\r") << true;
		QTest::newRow("LF data") << QByteArray("\n") << false;
		QTest::newRow("CRLF data") << QByteArray("\r\n") << false;
		QTest::newRow("CR data") << QByteArray("\r") << false;
	}

	void lineEndings()
	{
		QFETCH(QByteArray, lineEnding);
		QFETCH(bool, fromFile);

		const QByteArray content = CreateContent(lineEnding);

		AsciiFilter                  filter;
		ccHObject                    container;
		FileIOFilter::LoadParameters loadParams;
		CCVector3d                   shift(0, 0, 0);
		bool                         shiftEnabled = false;
		loadParams.alwaysDisplayLoadDialog        = false;
		loadParams.shiftHandlingMode              = ccGlobalShiftManager::Mode::NO_DIALOG;
		loadParams._coordinatesShiftEnabled       = &shiftEnabled;
		loadParams._coordinatesShift              = &shift;

		if (fromFile)
		{
			QTemporaryDir dir;
			QVERIFY(dir.isValid());
			const QString filename = dir.filePath("cloud.txt");
			QFile         file(filename);
			QVERIFY(file.open(QFile::WriteOnly)); // no QIODevice::Text flag: the line endings are written as they are
			QCOMPARE(file.write(content), static_cast<qint64>(content.size()));
			file.close();
			QCOMPARE(filter.loadFile(filename, container, loadParams), CC_FERR_NO_ERROR);
		}
		else
		{
			QCOMPARE(filter.loadAsciiData(content, "data", container, loadParams), CC_FERR_NO_ERROR);
		}

		ccHObject::Container clouds;
		container.filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, true);
		QCOMPARE(clouds.size(), static_cast<size_t>(1));
		ccPointCloud* cloud = static_cast<ccPointCloud*>(clouds.front());

		QCOMPARE(cloud->size(), PointCount);
		for (unsigned i = 0; i < PointCount; ++i)
		{
			const CCVector3* P = cloud->getPoint(i);
			QCOMPARE(P->x, static_cast<PointCoordinateType>(1.5 + i));
			QCOMPARE(P->y, static_cast<PointCoordinateType>(10.0 + i));
			QCOMPARE(P->z, static_cast<PointCoordinateType>(-1.0 - 2.25 * i));
		}
	}
};

QTEST_MAIN(TestAsciiFilter)
#include "TestAsciiFilter.moc"
