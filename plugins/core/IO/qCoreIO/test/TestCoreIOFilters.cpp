// qCoreIO
#include <OFFFilter.h>
#include <ObjFilter.h>
#include <PTXFilter.h>
#include <STLFilter.h>
#include <SimpleBinFilter.h>
#include <VTKFilter.h>

// qCC_db
#include <ccHObject.h>
#include <ccMaterial.h>
#include <ccMaterialSet.h>
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>

// qCC_io
#include <FileIO.h>
#include <FileIOFilter.h>

// Qt
#include <QDir>
#include <QFile>
#include <QImage>
#include <QObject>
#include <QTemporaryDir>
#include <QTest>
#include <QTextStream>

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

static constexpr int Exact    = -1; // stored as in memory
static constexpr int Float32  = -2; // stored as 32-bit floats
static constexpr int NotSaved = -3; // not stored by the format

// allowed error for a value stored with the given precision (or with 'decimals' digits after the point)
static double Tolerance(int decimals, double value)
{
	if (decimals == Exact)
		return 0.0;
	if (decimals == Float32)
		return std::abs(value) * std::numeric_limits<float>::epsilon();
	return 0.5 * std::pow(10.0, -decimals) + std::abs(value) * std::numeric_limits<float>::epsilon();
}

// normals are compressed in memory (see ccNormalCompressor), so a reloaded normal may fall in a neighboring cell
static constexpr double NormalTolerance = 0.01;

static std::unique_ptr<FileIOFilter> CreateFilter(const QString& extension)
{
	if (extension == "obj")
		return std::make_unique<ObjFilter>();
	if (extension == "off")
		return std::make_unique<OFFFilter>();
	if (extension == "ptx")
		return std::make_unique<PTXFilter>();
	if (extension == "sbf")
		return std::make_unique<SimpleBinFilter>();
	if (extension == "stl")
		return std::make_unique<STLFilter>();
	if (extension == "vtk")
		return std::make_unique<VTKFilter>();
	return nullptr;
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

static bool WriteTextFile(const QString& filename, const QString& content)
{
	QFile file(filename);
	if (!file.open(QFile::WriteOnly | QFile::Text))
		return false;
	QTextStream(&file) << content;
	return true;
}

// returns the normals at the 3 corners of a triangle, whether the mesh stores them per triangle or per vertex
static void GetCornerNormals(ccMesh* mesh, unsigned triIndex, CCVector3 N[3])
{
	if (mesh->hasTriNormals())
	{
		const CCVector3* Na = nullptr;
		const CCVector3* Nb = nullptr;
		const CCVector3* Nc = nullptr;
		mesh->getTriangleNormals(triIndex, Na, Nb, Nc);
		N[0] = *Na;
		N[1] = *Nb;
		N[2] = *Nc;
	}
	else
	{
		const CCCoreLib::VerticesIndexes* tri      = mesh->getTriangleVertIndexes(triIndex);
		ccGenericPointCloud*              vertices = mesh->getAssociatedCloud();
		N[0]                                       = vertices->getPointNormal(tri->i1);
		N[1]                                       = vertices->getPointNormal(tri->i2);
		N[2]                                       = vertices->getPointNormal(tri->i3);
	}
}

// Saves a small generated cloud (or mesh) with each filter of the qCoreIO plugin, loads it back
// and checks that the data survived (not the file layout). Also loads a few hand-written files.
class TestCoreIOFilters : public QObject
{
	Q_OBJECT
  private slots:

	void initTestCase()
	{
		FileIO::setWriterInfo("TestCoreIOFilters", "1.0"); // done by ccApplication in the real application
	}

	void cloudRoundTrip_data()
	{
		QTest::addColumn<QString>("extension");
		QTest::addColumn<int>("coordDecimals");
		QTest::addColumn<bool>("withNormals");
		QTest::addColumn<bool>("withColors");
		QTest::addColumn<int>("sfDecimals");

		QTest::newRow("OBJ") << "obj" << 8 << true << false << NotSaved;
		QTest::newRow("VTK") << "vtk" << 8 << true << true << 8;
		QTest::newRow("SBF") << "sbf" << Float32 << false << false << Float32; // coordinates are saved relative to the first point
	}

	void cloudRoundTrip()
	{
		QFETCH(QString, extension);
		QFETCH(int, coordDecimals);
		QFETCH(bool, withNormals);
		QFETCH(bool, withColors);
		QFETCH(int, sfDecimals);

		std::unique_ptr<FileIOFilter> filter = CreateFilter(extension);
		QVERIFY(filter);

		QTemporaryDir dir;
		QVERIFY(dir.isValid());
		const QString filename = dir.filePath("cloud." + extension);

		std::unique_ptr<ccPointCloud> original(CreateCloud());
		QCOMPARE(Save(*filter, original.get(), filename), CC_FERR_NO_ERROR);

		ccHObject container;
		QCOMPARE(Load(*filter, filename, container), CC_FERR_NO_ERROR);

		ccHObject::Container clouds;
		container.filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, true);
		QCOMPARE(clouds.size(), static_cast<size_t>(1));
		ccPointCloud* loadedCloud = static_cast<ccPointCloud*>(clouds.front());

		QCOMPARE(loadedCloud->size(), PointCount);
		QCOMPARE(loadedCloud->hasNormals(), withNormals);
		QCOMPARE(loadedCloud->hasColors(), withColors);

		const bool withSF = (sfDecimals != NotSaved);
		QCOMPARE(loadedCloud->getNumberOfScalarFields(), withSF ? 1u : 0u);
		CCCoreLib::ScalarField::Shared loadedSF   = (withSF ? loadedCloud->getScalarField(0) : nullptr);
		CCCoreLib::ScalarField::Shared originalSF = original->getScalarField(0);
		if (loadedSF)
		{
			QCOMPARE(QString::fromStdString(loadedSF->getName()), QString("Scalar field"));
		}

		for (unsigned i = 0; i < PointCount; ++i)
		{
			CCVector3d Pg = original->toGlobal3d<PointCoordinateType>(*original->getPoint(i));
			CCVector3d Qg = loadedCloud->toGlobal3d<PointCoordinateType>(*loadedCloud->getPoint(i));
			for (unsigned d = 0; d < 3; ++d)
			{
				QVERIFY(std::abs(Qg.u[d] - Pg.u[d]) <= Tolerance(coordDecimals, Pg.u[d]));
			}

			if (withNormals)
			{
				const CCVector3& N  = original->getPointNormal(i);
				const CCVector3& LN = loadedCloud->getPointNormal(i);
				for (unsigned d = 0; d < 3; ++d)
				{
					QVERIFY(std::abs(LN.u[d] - N.u[d]) <= NormalTolerance);
				}
			}

			if (withColors)
			{
				const ccColor::Rgba& c  = original->getPointColor(i);
				const ccColor::Rgba& lc = loadedCloud->getPointColor(i);
				QCOMPARE(lc.r, c.r);
				QCOMPARE(lc.g, c.g);
				QCOMPARE(lc.b, c.b);
			}

			if (loadedSF)
			{
				double v = originalSF->getValue(i);
				QVERIFY(std::abs(loadedSF->getValue(i) - v) <= Tolerance(sfDecimals, v));
			}
		}
	}

	void meshRoundTrip_data()
	{
		QTest::addColumn<QString>("extension");
		QTest::addColumn<QString>("normals"); // "vertex" or "triangle" (computed before saving), "face" (computed by the format) or "none"
		QTest::addColumn<int>("coordDecimals");

		QTest::newRow("OBJ with per-vertex normals") << "obj" << "vertex" << 8;
		QTest::newRow("OBJ with per-triangle normals") << "obj" << "triangle" << 8;
		QTest::newRow("STL") << "stl" << "face" << Float32; // binary STL (no dialog), the vertices are merged back on loading
		QTest::newRow("OFF") << "off" << "none" << 12;
		QTest::newRow("VTK with per-vertex normals") << "vtk" << "vertex" << 8;
	}

	void meshRoundTrip()
	{
		QFETCH(QString, extension);
		QFETCH(QString, normals);
		QFETCH(int, coordDecimals);

		std::unique_ptr<FileIOFilter> filter = CreateFilter(extension);
		QVERIFY(filter);

		QTemporaryDir dir;
		QVERIFY(dir.isValid());
		const QString filename = dir.filePath("mesh." + extension);

		std::unique_ptr<ccMesh> original(CreateMesh());
		if (normals == "vertex")
		{
			QVERIFY(original->computePerVertexNormals());
		}
		else if (normals == "triangle")
		{
			QVERIFY(original->computePerTriangleNormals());
		}
		QCOMPARE(Save(*filter, original.get(), filename), CC_FERR_NO_ERROR);

		ccHObject container;
		QCOMPARE(Load(*filter, filename, container), CC_FERR_NO_ERROR);

		ccHObject::Container meshes;
		container.filterChildren(meshes, true, CC_TYPES::MESH, true);
		QCOMPARE(meshes.size(), static_cast<size_t>(1));
		ccMesh* loadedMesh = static_cast<ccMesh*>(meshes.front());

		ccGenericPointCloud* vertices       = original->getAssociatedCloud();
		ccGenericPointCloud* loadedVertices = loadedMesh->getAssociatedCloud();
		QVERIFY(loadedVertices != nullptr);
		QCOMPARE(loadedVertices->size(), vertices->size());
		QCOMPARE(loadedMesh->size(), original->size());
		QCOMPARE(loadedMesh->hasNormals(), normals != "none");

		// we compare the triangle corners, as the vertices may be in a different order (STL)
		for (unsigned t = 0; t < original->size(); ++t)
		{
			CCVector3 A, B, C;
			original->getTriangleVertices(t, A, B, C);
			CCVector3 LA, LB, LC;
			loadedMesh->getTriangleVertices(t, LA, LB, LC);

			const CCVector3* corners[3]{&A, &B, &C};
			const CCVector3* loadedCorners[3]{&LA, &LB, &LC};
			for (unsigned c = 0; c < 3; ++c)
			{
				CCVector3d Pg = vertices->toGlobal3d<PointCoordinateType>(*corners[c]);
				CCVector3d Qg = loadedVertices->toGlobal3d<PointCoordinateType>(*loadedCorners[c]);
				for (unsigned d = 0; d < 3; ++d)
				{
					QVERIFY(std::abs(Qg.u[d] - Pg.u[d]) <= Tolerance(coordDecimals, Pg.u[d]));
				}
			}

			if (normals == "none")
			{
				continue;
			}

			CCVector3 N[3];
			if (normals == "face")
			{
				N[0] = (B - A).cross(C - A);
				N[0].normalize();
				N[1] = N[2] = N[0];
			}
			else
			{
				GetCornerNormals(original.get(), t, N);
			}
			CCVector3 LN[3];
			GetCornerNormals(loadedMesh, t, LN);
			for (unsigned c = 0; c < 3; ++c)
			{
				for (unsigned d = 0; d < 3; ++d)
				{
					QVERIFY(std::abs(LN[c].u[d] - N[c].u[d]) <= NormalTolerance);
				}
			}
		}
	}

	void objTexturedMeshRoundTrip()
	{
		QTemporaryDir dir;
		QVERIFY(dir.isValid());
		QVERIFY(QDir(dir.path()).mkpath("out"));
		const QString filename = dir.filePath("out/mesh.obj");

		std::unique_ptr<ccMesh> original(CreateMesh());

		// one plain material and one textured material
		ccMaterialSet::Shared materials(new ccMaterialSet("materials"));
		{
			ccMaterial::Shared plain(new ccMaterial("plain"));
			plain->setDiffuse(ccColor::Rgbaf(1.0f, 0.0f, 0.0f, 1.0f));
			QCOMPARE(materials->addMaterial(plain), 0);

			QImage image(4, 2, QImage::Format_RGB32);
			for (int y = 0; y < image.height(); ++y)
			{
				for (int x = 0; x < image.width(); ++x)
				{
					image.setPixel(x, y, qRgb(x * 60, y * 120, 200));
				}
			}
			ccMaterial::Shared textured(new ccMaterial("textured"));
			// the texture must not come from the output folder, so that the reloaded one is read from the saved file
			textured->setTexture(image, dir.filePath("source/texture.png"));
			QCOMPARE(materials->addMaterial(textured), 1);
		}
		original->setMaterialSet(materials);

		// the first half of the triangles use the plain material, the second half the textured one
		QVERIFY(original->reservePerTriangleMtlIndexes());
		for (unsigned t = 0; t < original->size(); ++t)
		{
			original->addTriangleMtlIndex(t < original->size() / 2 ? 0 : 1);
		}

		// one texture coordinate per vertex
		TextureCoordsContainer::Shared texCoords(new TextureCoordsContainer);
		QVERIFY(texCoords->reserveSafe(GridSize * GridSize));
		for (unsigned j = 0; j < GridSize; ++j)
		{
			for (unsigned i = 0; i < GridSize; ++i)
			{
				texCoords->addElement(TexCoords2D(i / static_cast<float>(GridSize - 1), j / static_cast<float>(GridSize - 1)));
			}
		}
		original->setTexCoordinatesTable(texCoords);
		QVERIFY(original->reservePerTriangleTexCoordIndexes());
		for (unsigned t = 0; t < original->size(); ++t)
		{
			const CCCoreLib::VerticesIndexes* tri = original->getTriangleVertIndexes(t);
			original->addTriangleTexCoordIndexes(tri->i1, tri->i2, tri->i3);
		}

		ObjFilter filter;
		QCOMPARE(Save(filter, original.get(), filename), CC_FERR_NO_ERROR);
		QVERIFY(QFile::exists(dir.filePath("out/mesh.mtl")));
		QVERIFY(QFile::exists(dir.filePath("out/texture.png")));

		ccHObject container;
		QCOMPARE(Load(filter, filename, container), CC_FERR_NO_ERROR);

		ccHObject::Container meshes;
		container.filterChildren(meshes, true, CC_TYPES::MESH, true);
		QCOMPARE(meshes.size(), static_cast<size_t>(1));
		ccMesh* loadedMesh = static_cast<ccMesh*>(meshes.front());
		QCOMPARE(loadedMesh->size(), original->size());

		QVERIFY(loadedMesh->hasMaterials());
		ccMaterialSet::Shared loadedMaterials = loadedMesh->getMaterialSet();
		QCOMPARE(loadedMaterials->size(), materials->size());
		for (size_t m = 0; m < materials->size(); ++m)
		{
			ccMaterial::CShared mtl       = materials->at(m);
			ccMaterial::CShared loadedMtl = loadedMaterials->at(m);
			QCOMPARE(loadedMtl->getName(), mtl->getName());
			QCOMPARE(loadedMtl->getDiffuseFront().r, mtl->getDiffuseFront().r);
			QCOMPARE(loadedMtl->getDiffuseFront().g, mtl->getDiffuseFront().g);
			QCOMPARE(loadedMtl->getDiffuseFront().b, mtl->getDiffuseFront().b);
			QCOMPARE(loadedMtl->hasTexture(), mtl->hasTexture());
			if (mtl->hasTexture())
			{
				QCOMPARE(loadedMtl->getTexture().convertToFormat(QImage::Format_RGB32), mtl->getTexture().convertToFormat(QImage::Format_RGB32));
			}
		}

		QVERIFY(loadedMesh->hasPerTriangleTexCoordIndexes());
		for (unsigned t = 0; t < original->size(); ++t)
		{
			QCOMPARE(loadedMesh->getTriangleMtlIndex(t), original->getTriangleMtlIndex(t));

			const TexCoords2D* tc[3]{nullptr, nullptr, nullptr};
			original->getTriangleTexCoordinates(t, tc[0], tc[1], tc[2]);
			const TexCoords2D* ltc[3]{nullptr, nullptr, nullptr};
			loadedMesh->getTriangleTexCoordinates(t, ltc[0], ltc[1], ltc[2]);
			for (unsigned c = 0; c < 3; ++c)
			{
				QVERIFY(ltc[c] != nullptr);
				QVERIFY(std::abs(ltc[c]->tx - tc[c]->tx) <= Tolerance(8, tc[c]->tx));
				QVERIFY(std::abs(ltc[c]->ty - tc[c]->ty) <= Tolerance(8, tc[c]->ty));
			}
		}
	}

	void objWindowsSeparators_data()
	{
		QTest::addColumn<QString>("mtlReference");     // MTL file name, as written in the OBJ file
		QTest::addColumn<QString>("mtlFile");          // where the MTL file actually is, relative to the OBJ file
		QTest::addColumn<QString>("textureReference"); // texture file name, as written in the MTL file
		QTest::addColumn<QString>("textureFile");      // where the texture file actually is, relative to the MTL file

		QTest::newRow("relative paths") << "materials\\model.mtl"
		                                << "materials/model.mtl"
		                                << "textures\\texture.png"
		                                << "textures/texture.png";
		QTest::newRow("absolute paths from another machine") << "C:\\Users\\someone\\Desktop\\model.mtl"
		                                                     << "model.mtl"
		                                                     << "C:\\Users\\someone\\Desktop\\texture.png"
		                                                     << "texture.png";
	}

	// OBJ and MTL files exported on Windows can reference other files with backslashes (#2451)
	void objWindowsSeparators()
	{
		QFETCH(QString, mtlReference);
		QFETCH(QString, mtlFile);
		QFETCH(QString, textureReference);
		QFETCH(QString, textureFile);

		QTemporaryDir dir;
		QVERIFY(dir.isValid());
		const QDir objDir(dir.path());
		const QDir mtlDir(QFileInfo(objDir.filePath(mtlFile)).absolutePath());
		QVERIFY(objDir.mkpath(mtlDir.path()));
		QVERIFY(mtlDir.mkpath(QFileInfo(mtlDir.filePath(textureFile)).absolutePath()));

		QImage image(2, 2, QImage::Format_RGB32);
		image.fill(qRgb(10, 20, 30));
		QVERIFY(image.save(mtlDir.filePath(textureFile)));

		QVERIFY(WriteTextFile(objDir.filePath(mtlFile),
		                      QString("newmtl textured\n"
		                              "Kd 1 1 1\n"
		                              "map_Kd %1\n")
		                          .arg(textureReference)));

		const QString filename = objDir.filePath("model.obj");
		QVERIFY(WriteTextFile(filename,
		                      QString("mtllib %1\n"
		                              "v 0 0 0\n"
		                              "v 1 0 0\n"
		                              "v 0 1 0\n"
		                              "vt 0 0\n"
		                              "vt 1 0\n"
		                              "vt 0 1\n"
		                              "usemtl textured\n"
		                              "f 1/1 2/2 3/3\n")
		                          .arg(mtlReference)));

		ObjFilter filter;
		ccHObject container;
		QCOMPARE(Load(filter, filename, container), CC_FERR_NO_ERROR);

		ccHObject::Container meshes;
		container.filterChildren(meshes, true, CC_TYPES::MESH, true);
		QCOMPARE(meshes.size(), static_cast<size_t>(1));
		ccMesh* loadedMesh = static_cast<ccMesh*>(meshes.front());

		QVERIFY(loadedMesh->hasMaterials());
		QCOMPARE(loadedMesh->getMaterialSet()->size(), static_cast<size_t>(1));
		ccMaterial::CShared mtl = loadedMesh->getMaterialSet()->at(0);
		QVERIFY(mtl->hasTexture());
		QCOMPARE(mtl->getTexture().pixel(0, 0), qRgb(10, 20, 30));
	}

	void ptxLoad()
	{
		QTemporaryDir dir;
		QVERIFY(dir.isValid());
		const QString filename = dir.filePath("scan.ptx");

		// a 2 x 3 grid with one empty cell (0 0 0), and a cloud transformation with a translation
		QVERIFY(WriteTextFile(filename,
		                      "3\n"
		                      "2\n"
		                      "0 0 0\n"
		                      "1 0 0\n"
		                      "0 1 0\n"
		                      "0 0 1\n"
		                      "1 0 0 0\n"
		                      "0 1 0 0\n"
		                      "0 0 1 0\n"
		                      "10 20 30 1\n"
		                      "1 2 3 0.1 10 20 30\n"
		                      "4 5 6 0.2 40 50 60\n"
		                      "0 0 0 0.5 0 0 0\n"
		                      "7 8 9 0.3 70 80 90\n"
		                      "1 1 1 0.4 100 110 120\n"
		                      "2 2 2 0.6 130 140 150\n"));

		PTXFilter filter;
		ccHObject container;
		QCOMPARE(Load(filter, filename, container), CC_FERR_NO_ERROR);

		ccHObject::Container clouds;
		container.filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, true);
		QCOMPARE(clouds.size(), static_cast<size_t>(1));
		ccPointCloud* cloud = static_cast<ccPointCloud*>(clouds.front());

		const CCVector3d   points[5]{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {1, 1, 1}, {2, 2, 2}};
		const ScalarType   intensities[5]{0.1f, 0.2f, 0.3f, 0.4f, 0.6f};
		const ccColor::Rgb colors[5]{ccColor::Rgb(10, 20, 30), ccColor::Rgb(40, 50, 60), ccColor::Rgb(70, 80, 90), ccColor::Rgb(100, 110, 120), ccColor::Rgb(130, 140, 150)};
		const CCVector3d   translation(10, 20, 30);

		QCOMPARE(cloud->size(), 5u);
		QVERIFY(cloud->hasColors());
		QCOMPARE(cloud->getNumberOfScalarFields(), 1u);
		CCCoreLib::ScalarField::Shared intensitySF = cloud->getScalarField(0);
		QCOMPARE(QString::fromStdString(intensitySF->getName()), QString("Intensity"));

		for (unsigned i = 0; i < 5; ++i)
		{
			CCVector3d P = cloud->toGlobal3d<PointCoordinateType>(*cloud->getPoint(i));
			for (unsigned d = 0; d < 3; ++d)
			{
				double expected = points[i].u[d] + translation.u[d];
				QVERIFY(std::abs(P.u[d] - expected) <= Tolerance(Float32, expected));
			}
			QCOMPARE(intensitySF->getValue(i), intensities[i]);
			const ccColor::Rgba& c = cloud->getPointColor(i);
			QCOMPARE(c.r, colors[i].r);
			QCOMPARE(c.g, colors[i].g);
			QCOMPARE(c.b, colors[i].b);
		}

		// the scan grid keeps track of the empty cell
		QCOMPARE(cloud->gridCount(), static_cast<size_t>(1));
		const ccPointCloud::Grid::Shared& grid = cloud->grid(0);
		QCOMPARE(grid->w, 2u);
		QCOMPARE(grid->h, 3u);
		QCOMPARE(grid->validCount, 5u);
		const std::vector<int> expectedIndexes{0, 1, -1, 2, 3, 4};
		QCOMPARE(grid->indexes, expectedIndexes);
	}
};

QTEST_MAIN(TestCoreIOFilters)
#include "TestCoreIOFilters.moc"
