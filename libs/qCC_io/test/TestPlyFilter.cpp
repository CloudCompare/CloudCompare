// qCC_db
#include <ccHObject.h>
#include <ccMaterialSet.h>
#include <ccMesh.h>
#include <ccPointCloud.h>

// qCC_io
#include <PlyFilter.h>

// Qt
#include <QFile>
#include <QImage>
#include <QObject>
#include <QTemporaryDir>
#include <QTest>
#include <QTextStream>

class TestPlyFilter : public QObject
{
	Q_OBJECT

  private slots:
	// A face's "texnumber" property is an index into the file's declared textures.
	// A malformed/corrupted PLY (or one produced by third-party software) can carry
	// a texnumber that is out of range for the textures actually declared: this must
	// not end up stored verbatim in the resulting mesh, since consumers of
	// ccMesh::getTriangleMtlIndex() (e.g. ccMesh::drawMeOnly, getVertexColorFromMaterial)
	// index directly into the material set without re-checking the bound.
	void testOutOfRangeTextureIndexIsSanitized() const
	{
		QTemporaryDir tmpDir;
		QVERIFY(tmpDir.isValid());

		// a single, tiny, valid texture (so exactly one material can be resolved: index 0)
		QString texturePath = tmpDir.filePath("texture.bmp");
		QImage  texture(2, 2, QImage::Format_RGB32);
		texture.fill(Qt::white);
		QVERIFY(texture.save(texturePath, "BMP"));

		// one triangle whose 'texnumber' (5) is way out of range: only 1 texture is declared
		QString plyPath = tmpDir.filePath("bad_texnumber.ply");
		{
			QFile file(plyPath);
			QVERIFY(file.open(QIODevice::WriteOnly | QIODevice::Text));
			QTextStream out(&file);
			out << "ply\n"
			    << "format ascii 1.0\n"
			    << "comment TextureFile texture.bmp\n"
			    << "element vertex 3\n"
			    << "property float x\n"
			    << "property float y\n"
			    << "property float z\n"
			    << "element face 1\n"
			    << "property list uchar int vertex_indices\n"
			    << "property list uchar float texcoord\n"
			    << "property int texnumber\n"
			    << "end_header\n"
			    << "0 0 0\n"
			    << "1 0 0\n"
			    << "0 1 0\n"
			    << "3 0 1 2 6 0 0 1 0 0 1 5\n";
		}

		ccHObject                    container;
		FileIOFilter::LoadParameters params;
		params.alwaysDisplayLoadDialog = false;
		PlyFilter filter;

		CC_FILE_ERROR error = filter.loadFile(plyPath, container, params);
		QVERIFY(error == CC_FERR_NO_ERROR);

		QVERIFY(container.getChildrenNumber() >= 1);
		ccHObject* meshObject = container.getFirstChild();
		QVERIFY(meshObject != nullptr);
		QVERIFY(meshObject->isA(CC_TYPES::MESH));
		auto* mesh = static_cast<ccMesh*>(meshObject);

		QVERIFY(mesh->hasMaterials());
		const ccMaterialSet::Shared materials = mesh->getMaterialSet();
		QVERIFY(materials != nullptr);
		QCOMPARE(static_cast<int>(materials->size()), 1);

		QCOMPARE(mesh->size(), 1u);
		// the file's only face had an out-of-range texnumber (5, only 1 texture declared):
		// it must come out as 'no material' (-1), not as a stray index into the material set
		QCOMPARE(mesh->getTriangleMtlIndex(0), -1);
	}

	// Colors can also be stored as floating point values between 0 and 1. In an ASCII file
	// they are rounded (2/255 = 0.0078431... is written 0.007843), so they must be rounded
	// to the closest integer when loaded: truncating them gave 1 instead of 2 for instance.
	void testFloatColorsAreRounded() const
	{
		QTemporaryDir tmpDir;
		QVERIFY(tmpDir.isValid());

		// one point per color value (with a different value for each channel)
		auto expectedColor = [](unsigned k)
		{
			return ccColor::Rgb(static_cast<ColorCompType>(k),
			                    static_cast<ColorCompType>(255 - k),
			                    static_cast<ColorCompType>((k * 7) % 256));
		};

		QString plyPath = tmpDir.filePath("float_colors.ply");
		{
			QFile file(plyPath);
			QVERIFY(file.open(QIODevice::WriteOnly | QIODevice::Text));
			QTextStream out(&file);
			out << "ply\n"
			    << "format ascii 1.0\n"
			    << "element vertex 256\n"
			    << "property float x\n"
			    << "property float y\n"
			    << "property float z\n"
			    << "property float red\n"
			    << "property float green\n"
			    << "property float blue\n"
			    << "end_header\n";
			for (unsigned k = 0; k < 256; ++k)
			{
				out << k << " 0 0";
				for (ColorCompType value : expectedColor(k).rgb)
				{
					out << ' ' << QString::number(value / 255.0, 'f', 6);
				}
				out << '\n';
			}
		}

		ccHObject                    container;
		FileIOFilter::LoadParameters params;
		params.alwaysDisplayLoadDialog = false;
		PlyFilter filter;

		CC_FILE_ERROR error = filter.loadFile(plyPath, container, params);
		QVERIFY(error == CC_FERR_NO_ERROR);

		QCOMPARE(container.getChildrenNumber(), 1u);
		ccHObject* cloudObject = container.getFirstChild();
		QVERIFY(cloudObject->isA(CC_TYPES::POINT_CLOUD));
		auto* cloud = static_cast<ccPointCloud*>(cloudObject);
		QCOMPARE(cloud->size(), 256u);
		QVERIFY(cloud->hasColors());

		for (unsigned k = 0; k < 256; ++k)
		{
			const ccColor::Rgb   expected = expectedColor(k);
			const ccColor::Rgba& loaded   = cloud->getPointColor(k);
			QCOMPARE(static_cast<int>(loaded.r), static_cast<int>(expected.r));
			QCOMPARE(static_cast<int>(loaded.g), static_cast<int>(expected.g));
			QCOMPARE(static_cast<int>(loaded.b), static_cast<int>(expected.b));
		}
	}
};

QTEST_GUILESS_MAIN(TestPlyFilter)
#include "TestPlyFilter.moc"
