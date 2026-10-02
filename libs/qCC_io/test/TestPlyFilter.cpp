#include "TestPlyFilter.h"

// qCC_db
#include <ccHObject.h>
#include <ccMaterialSet.h>
#include <ccMesh.h>

// qCC_io
#include <PlyFilter.h>

// Qt
#include <QFile>
#include <QImage>
#include <QTemporaryDir>
#include <QTextStream>

// A face's "texnumber" property is an index into the file's declared textures.
// A malformed/corrupted PLY (or one produced by third-party software) can carry
// a texnumber that is out of range for the textures actually declared: this must
// not end up stored verbatim in the resulting mesh, since consumers of
// ccMesh::getTriangleMtlIndex() (e.g. ccMesh::drawMeOnly, getVertexColorFromMaterial)
// index directly into the material set without re-checking the bound.
void TestPlyFilter::testOutOfRangeTextureIndexIsSanitized() const
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

QTEST_MAIN(TestPlyFilter)
