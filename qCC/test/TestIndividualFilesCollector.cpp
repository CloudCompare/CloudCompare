#include "ccIndividualFilesCollector.h"

// qCC_db
#include <ccGBLSensor.h>
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccSubMesh.h>

// qCC_io
#include <BinFilter.h>
#include <FileIO.h>
#include <FileIOFilter.h>

// Qt
#include <QDirIterator>
#include <QTemporaryDir>
#include <QTest>

// System
#include <memory>

static ccPointCloud* CreateCloud(const QString& name)
{
	ccPointCloud* cloud = new ccPointCloud(name);
	cloud->reserve(3);
	cloud->addPoint(CCVector3(0, 0, 0));
	cloud->addPoint(CCVector3(1, 0, 0));
	cloud->addPoint(CCVector3(0, 1, 0));
	return cloud;
}

// Returns the collected files as 'relativeFolder/baseName'
static QStringList RelativePaths(const ccIndividualFilesCollector& collector)
{
	QStringList paths;
	for (const ccIndividualFilesCollector::File& file : collector.files())
	{
		paths << (file.relativeFolder.isEmpty() ? file.baseName : file.relativeFolder + '/' + file.baseName);
	}
	return paths;
}

class TestIndividualFilesCollector : public QObject
{
	Q_OBJECT

  private:
	// DB tree:
	// root
	//  +- boulder
	//  |   +- stable
	//  |   |   +- b201, b202, dup, DUP, bad:name?
	//  |   +- sensor (can't be saved this way)
	//  |   +- mesh
	//  |       +- vertices (the mesh vertices)
	//  |       +- onMesh
	//  +- scan.bin (/data/scans) (group created when loading a file)
	//      +- CON
	ccHObject     m_root{"root"};
	ccHObject*    m_boulder   = nullptr;
	ccHObject*    m_stable    = nullptr;
	ccHObject*    m_fileGroup = nullptr;
	ccPointCloud* m_b201      = nullptr;
	ccGBLSensor*  m_sensor    = nullptr;
	ccMesh*       m_mesh      = nullptr;
	QString       m_plyFilter;

	ccIndividualFilesCollector createCollector(const QString& fileFilter) const
	{
		ccIndividualFilesCollector collector;
		collector.cloudFilter    = fileFilter;
		collector.meshFilter     = fileFilter;
		collector.polylineFilter = fileFilter;
		return collector;
	}

  private slots:
	void initTestCase()
	{
		FileIO::setWriterInfo("TestIndividualFilesCollector", "1.0"); // done by ccApplication in the real application
		FileIOFilter::InitInternalFilters();

		for (const FileIOFilter::Shared& filter : FileIOFilter::GetFilters())
		{
			if (filter->getDefaultExtension() == "ply")
			{
				m_plyFilter = filter->getFileFilters(false).first();
			}
		}
		QVERIFY(!m_plyFilter.isEmpty());

		m_boulder = new ccHObject("boulder");
		m_root.addChild(m_boulder);

		m_stable = new ccHObject("stable");
		m_boulder->addChild(m_stable);
		m_b201 = CreateCloud("b201");
		m_stable->addChild(m_b201);
		m_stable->addChild(CreateCloud("b202"));
		m_stable->addChild(CreateCloud("dup"));
		m_stable->addChild(CreateCloud("DUP"));
		m_stable->addChild(CreateCloud("bad:name?"));

		m_sensor = new ccGBLSensor();
		m_sensor->setName("sensor");
		m_boulder->addChild(m_sensor);

		ccPointCloud* vertices = CreateCloud("vertices");
		m_mesh                 = new ccMesh(vertices);
		m_mesh->setName("mesh");
		m_mesh->reserve(1);
		m_mesh->addTriangle(0, 1, 2);
		m_mesh->addChild(vertices);
		m_mesh->addChild(CreateCloud("onMesh"));
		m_boulder->addChild(m_mesh);

		m_fileGroup = new ccHObject("scan.bin (/data/scans)");
		m_root.addChild(m_fileGroup);
		m_fileGroup->addChild(CreateCloud("CON"));
	}

	void validFileNames()
	{
		QCOMPARE(ccIndividualFilesCollector::ToValidFileName("bad:name?"), QString("bad_name_"));
		QCOMPARE(ccIndividualFilesCollector::ToValidFileName("a/b\\c"), QString("a_b_c"));
		QCOMPARE(ccIndividualFilesCollector::ToValidFileName(" ..trail.. "), QString("..trail"));
		QCOMPARE(ccIndividualFilesCollector::ToValidFileName("..."), QString("unnamed"));
		QCOMPARE(ccIndividualFilesCollector::ToValidFileName("con"), QString("_con"));
		QCOMPARE(ccIndividualFilesCollector::ToValidFileName("LPT1.txt"), QString("_LPT1.txt"));
		QCOMPARE(ccIndividualFilesCollector::ToValidFileName("console"), QString("console"));
	}

	void subFolderNames()
	{
		QCOMPARE(ccIndividualFilesCollector::GetSubFolderName(m_fileGroup), QString("scan"));
		QCOMPARE(ccIndividualFilesCollector::GetSubFolderName(m_boulder), QString("boulder"));
	}

	void uniqueFilenames()
	{
		QSet<QString> used;
		QCOMPARE(ccIndividualFilesCollector::MakeUniqueFilename(used, "/out", "dup", "ply"), QString("/out/dup.ply"));
		QCOMPARE(ccIndividualFilesCollector::MakeUniqueFilename(used, "/out", "DUP", "ply"), QString("/out/DUP_1.ply"));
		QCOMPARE(ccIndividualFilesCollector::MakeUniqueFilename(used, "/out", "dup", "ply"), QString("/out/dup_2.ply"));
		QCOMPARE(ccIndividualFilesCollector::MakeUniqueFilename(used, "/other", "dup", "ply"), QString("/other/dup.ply"));
	}

	void removeDescendants()
	{
		ccHObject::Container selection = ccIndividualFilesCollector::RemoveDescendants({m_b201, m_boulder});
		QCOMPARE(selection.size(), size_t(1));
		QCOMPARE(selection.front(), m_boulder);
	}

	void hierarchyBelowSelection()
	{
		ccIndividualFilesCollector collector = createCollector(m_plyFilter);
		collector.collect(m_boulder);

		const QStringList expected{"boulder/stable/b201",
		                           "boulder/stable/b202",
		                           "boulder/stable/dup",
		                           "boulder/stable/DUP",
		                           "boulder/stable/bad_name_",
		                           "boulder/mesh",
		                           "boulder/mesh/onMesh"}; // the mesh vertices are not saved separately
		QCOMPARE(RelativePaths(collector), expected);
		QCOMPARE(collector.cloudCount(), 6u);
		QCOMPARE(collector.meshCount(), 1u);
		QCOMPARE(collector.polylineCount(), 0u);
		QCOMPARE(collector.ignored().size(), size_t(1));
		QCOMPARE(collector.ignored().front(), static_cast<ccHObject*>(m_sensor));
	}

	void fullTreeHierarchy()
	{
		ccIndividualFilesCollector collector = createCollector(m_plyFilter);
		for (ccHObject* entity : {m_stable, m_fileGroup})
		{
			collector.collect(entity, collector.getParentRelativeFolder(entity, &m_root));
		}

		const QStringList expected{"boulder/stable/b201",
		                           "boulder/stable/b202",
		                           "boulder/stable/dup",
		                           "boulder/stable/DUP",
		                           "boulder/stable/bad_name_",
		                           "scan/_CON"};
		QCOMPARE(RelativePaths(collector), expected);
	}

	void flat()
	{
		ccIndividualFilesCollector collector = createCollector(m_plyFilter);
		collector.createSubFolders           = false;
		collector.collect(&m_root);

		const QStringList expected{"b201", "b202", "dup", "DUP", "bad_name_", "mesh", "onMesh", "_CON"};
		QCOMPARE(RelativePaths(collector), expected);
	}

	void binIncludesChildren()
	{
		// a BIN file already contains the children of the saved entity
		ccIndividualFilesCollector collector = createCollector(BinFilter::GetFileFilter());
		collector.collect(m_mesh);
		QCOMPARE(RelativePaths(collector), QStringList{"mesh"});
	}

	void subMeshes()
	{
		ccHObject     root("root");
		ccPointCloud* vertices = CreateCloud("vertices");
		ccMesh*       mesh     = new ccMesh(vertices);
		mesh->setName("mesh");
		mesh->reserve(1);
		mesh->addTriangle(0, 1, 2);
		mesh->addChild(vertices);
		ccSubMesh* subMesh = new ccSubMesh(mesh);
		subMesh->setName("subMesh");
		subMesh->addTriangleIndex(0);
		mesh->addChild(subMesh);
		root.addChild(mesh);

		// a sub-mesh is a part of its parent mesh
		ccIndividualFilesCollector parentCollector = createCollector(m_plyFilter);
		parentCollector.collect(mesh);
		QCOMPARE(RelativePaths(parentCollector), QStringList{"mesh"});

		// but it is saved if explicitly selected
		ccIndividualFilesCollector subMeshCollector = createCollector(m_plyFilter);
		subMeshCollector.collect(subMesh);
		QCOMPARE(RelativePaths(subMeshCollector), QStringList{"subMesh"});
		QCOMPARE(subMeshCollector.meshCount(), 1u);
	}

	void siblingFoldersWithSameName()
	{
		// root
		//  +- group (A)
		//  |   +- cloud
		//  +- GROUP (B)
		//  |   +- cloud
		//  |   +- sub
		//  |       +- cloud
		//  +- a:b (C)
		//  |   +- cloud
		//  +- a?b (D)
		//      +- cloud
		ccHObject  root("root");
		ccHObject* groupA = new ccHObject("group");
		ccHObject* groupB = new ccHObject("GROUP");
		ccHObject* groupC = new ccHObject("a:b");
		ccHObject* groupD = new ccHObject("a?b");
		ccHObject* sub    = new ccHObject("sub");
		for (ccHObject* group : {groupA, groupB, groupC, groupD})
		{
			root.addChild(group);
			group->addChild(CreateCloud("cloud"));
		}
		groupB->addChild(sub);
		ccPointCloud* subCloud = CreateCloud("cloud");
		sub->addChild(subCloud);

		ccIndividualFilesCollector collector = createCollector(m_plyFilter);
		collector.collect(&root);
		const QStringList expected{"root/group/cloud", "root/GROUP_1/cloud", "root/GROUP_1/sub/cloud", "root/a_b/cloud", "root/a_b_1/cloud"};
		QCOMPARE(RelativePaths(collector), expected);

		// full tree mode: the ancestors get the same (unique) folders
		ccIndividualFilesCollector fullTreeCollector = createCollector(m_plyFilter);
		for (ccHObject* entity : {groupA, static_cast<ccHObject*>(subCloud)})
		{
			fullTreeCollector.collect(entity, fullTreeCollector.getParentRelativeFolder(entity, &root));
		}
		QCOMPARE(RelativePaths(fullTreeCollector), (QStringList{"group/cloud", "GROUP_1/sub/cloud"}));
	}

	void saveAndReload()
	{
		QTemporaryDir outputDir;
		QVERIFY(outputDir.isValid());

		ccIndividualFilesCollector collector = createCollector(m_plyFilter);
		collector.collect(m_boulder);

		FileIOFilter::SaveParameters saveParameters;
		saveParameters.alwaysDisplaySaveDialog = false;

		QSet<QString> usedFilenames;
		for (const ccIndividualFilesCollector::File& file : collector.files())
		{
			QString folder = outputDir.path() + '/' + file.relativeFolder;
			QVERIFY(QDir().mkpath(folder));
			QString filename = ccIndividualFilesCollector::MakeUniqueFilename(usedFilenames, folder, file.baseName, "ply");
			QCOMPARE(FileIOFilter::SaveToFile(file.entity, filename, saveParameters, file.fileFilter), CC_FERR_NO_ERROR);
		}

		QStringList  savedFiles;
		QDirIterator it(outputDir.path(), QDir::Files, QDirIterator::Subdirectories);
		while (it.hasNext())
		{
			savedFiles << QDir(outputDir.path()).relativeFilePath(it.next());
		}
		savedFiles.sort();

		const QStringList expected{"boulder/mesh.ply",
		                           "boulder/mesh/onMesh.ply",
		                           "boulder/stable/DUP_1.ply", // 'dup' and 'DUP' would be the same file on Windows
		                           "boulder/stable/b201.ply",
		                           "boulder/stable/b202.ply",
		                           "boulder/stable/bad_name_.ply",
		                           "boulder/stable/dup.ply"};
		QCOMPARE(savedFiles, expected);

		FileIOFilter::LoadParameters loadParameters;
		loadParameters.alwaysDisplayLoadDialog = false;
		CC_FILE_ERROR              result      = CC_FERR_NO_ERROR;
		std::unique_ptr<ccHObject> loaded(FileIOFilter::LoadFromFile(outputDir.path() + "/boulder/stable/b201.ply", loadParameters, result, m_plyFilter));
		QCOMPARE(result, CC_FERR_NO_ERROR);
		QVERIFY(loaded);

		ccHObject::Container clouds;
		loaded->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD);
		QCOMPARE(clouds.size(), size_t(1));
		QCOMPARE(static_cast<ccPointCloud*>(clouds.front())->size(), 3u);
	}
};

QTEST_GUILESS_MAIN(TestIndividualFilesCollector)
#include "TestIndividualFilesCollector.moc"
