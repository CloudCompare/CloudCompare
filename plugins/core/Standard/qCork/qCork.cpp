//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qCork                       #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include "qCork.h"

//qCC_db
#include <ccMesh.h>
#include <ccPointCloud.h>

//dialog
#include "ccCorkDlg.h"

//Qt
#include <QMainWindow>
#include <QtGui>
#include <QProgressDialog>
#include <QtConcurrentRun>

//Cork
#include <mesh/corkMesh.h>

//system
#if defined(CC_WINDOWS)
#include "windows.h"
#else
#include <time.h>
#include <unistd.h>
#endif


qCork::qCork(QObject* parent/*=0*/)
	: QObject(parent)
	, ccStdPluginInterface(":/CC/plugin/qCork/info.json")
	, m_action(nullptr)
{
}

QList<QAction *> qCork::getActions()
{
	//default action
	if (!m_action)
	{
		m_action = new QAction(getName(), this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		//connect signal
		connect(m_action, &QAction::triggered, this, &qCork::doAction);
	}

	return QList<QAction *>{ m_action };
}

void qCork::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
	{
		//we need two and only two meshes!
		m_action->setEnabled(selectedEntities.size() == 2
			&& selectedEntities[0]->isKindOf(CC_TYPES::MESH)
			&& selectedEntities[1]->isKindOf(CC_TYPES::MESH));
	}
}

bool ToCorkMesh(const ccMesh* in, CorkMesh& out, ccMainAppInterface* app = 0)
{
	if (!in || !in->getAssociatedCloud())
	{
		if (app)
			app->dispToConsole("[Cork] Invalid input mesh!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		assert(false);
		return false;
	}

	ccGenericPointCloud* vertices = in->getAssociatedCloud();
	assert(vertices);

	unsigned triCount = in->size();
	unsigned vertCount = vertices ? vertices->size() : 0;

	std::vector<CorkMesh::Tri>& outTris = out.getTris();
	std::vector<CorkVertex>& outVerts = out.getVerts();
	try
	{
		outVerts.resize(vertCount);
		outTris.resize(triCount);
	}
	catch (const std::bad_alloc&)
	{
		if (app)
			app->dispToConsole("[Cork] Not enough memory!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	if (outVerts.empty() || outTris.empty())
	{
		if (app)
			app->dispToConsole(QString("[Cork] Input mesh '%1' is empty?!").arg(in->getName()), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		return false;
	}

	//import triangle indexes
	{
		for (unsigned i = 0; i < triCount; i++)
		{
			const CCLib::VerticesIndexes* tsi = in->getTriangleVertIndexes(i);
			CorkTriangle corkTri;
			corkTri.a = tsi->i1;
			corkTri.b = tsi->i2;
			corkTri.c = tsi->i3;
			outTris[i].data = corkTri;
			//DGM: it seems that Cork doubles this information?!
			outTris[i].a = tsi->i1;
			outTris[i].b = tsi->i2;
			outTris[i].c = tsi->i3;
		}
	}

	//import vertices
	{
		for (unsigned i = 0; i < vertCount; i++)
		{
			const CCVector3* P = vertices->getPoint(i);
			outVerts[i].pos.x = static_cast<double>(P->x);
			outVerts[i].pos.y = static_cast<double>(P->y);
			outVerts[i].pos.z = static_cast<double>(P->z);
		}
	}

	return true;
}

ccMesh* FromCorkMesh(const CorkMesh& in, ccMainAppInterface* app = 0)
{
	const std::vector<CorkMesh::Tri>& inTris = in.getTris();
	const std::vector<CorkVertex>& inVerts = in.getVerts();

	if (inTris.empty() || inVerts.empty())
	{
		if (app)
			app->dispToConsole("[Cork] Output mesh is empty?!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		return 0;
	}

	unsigned triCount = static_cast<unsigned>(inTris.size());
	unsigned vertCount = static_cast<unsigned>(inVerts.size());

	ccPointCloud* vertices = new ccPointCloud("vertices");
	if (!vertices->reserve(vertCount))
	{
		if (app)
			app->dispToConsole("[Cork] Not enough memory!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		delete vertices;
		return 0;
	}

	ccMesh* mesh = new ccMesh(vertices);
	mesh->addChild(vertices);
	if (!mesh->reserve(triCount))
	{
		if (app)
			app->dispToConsole("[Cork] Not enough memory!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		delete mesh;
		return 0;
	}

	//import vertices
	{
		for (unsigned i = 0; i < vertCount; i++)
		{
			const CorkVertex& P = inVerts[i];
			CCVector3 Pout(static_cast<PointCoordinateType>(P.pos.x),
				static_cast<PointCoordinateType>(P.pos.y),
				static_cast<PointCoordinateType>(P.pos.z));
			vertices->addPoint(Pout);
		}
	}

	//import triangle indexes
	{
		for (unsigned i = 0; i < triCount; i++)
		{
			const CorkMesh::Tri& tri = inTris[i];
			mesh->addTriangle(tri.a, tri.b, tri.c);
		}
	}

	mesh->setVisible(true);
	vertices->setEnabled(false);

	return mesh;
}

//! Boolean operation parameters (for concurrent run)
struct BoolOpParameters
{
	BoolOpParameters()
		: operation(ccCorkDlg::UNION)
		, corkA(0)
		, corkB(0)
		, app(0)
		, meshesAreOk(false)
	{}

	ccCorkDlg::CSG_OPERATION operation;
	CorkMesh* corkA;
	CorkMesh* corkB;
	QString nameA;
	QString nameB;
	ccMainAppInterface* app;
	bool meshesAreOk;
};
static BoolOpParameters s_params;

bool doPerformBooleanOp()
{
	//invalid parameters
	if (!s_params.corkA || !s_params.corkB)
		return false;

	try
	{
		//check meshes
		s_params.meshesAreOk = true;
		if (false)
		{
			if (s_params.corkA->isSelfIntersecting())
			{
				if (s_params.app)
					s_params.app->dispToConsole(QString("[Cork] Mesh '%1' is self-intersecting! Result may be jeopardized!").arg(s_params.nameA), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				s_params.meshesAreOk = false;
			}
			else if (!s_params.corkA->isClosed())
			{
				if (s_params.app)
					s_params.app->dispToConsole(QString("[Cork] Mesh '%1' is not closed! Result may be jeopardized!").arg(s_params.nameA), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				s_params.meshesAreOk = false;
			}
			if (s_params.corkB->isSelfIntersecting())
			{
				if (s_params.app)
					s_params.app->dispToConsole(QString("[Cork] Mesh '%1' is self-intersecting! Result may be jeopardized!").arg(s_params.nameB), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				s_params.meshesAreOk = false;
			}
			else if (!s_params.corkB->isClosed())
			{
				if (s_params.app)
					s_params.app->dispToConsole(QString("[Cork] Mesh '%1' is not closed! Result may be jeopardized!").arg(s_params.nameB), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				s_params.meshesAreOk = false;
			}
		}

		//perform the boolean operation
		switch (s_params.operation)
		{
		case ccCorkDlg::UNION:
			s_params.corkA->boolUnion(*s_params.corkB);
			break;

		case ccCorkDlg::INTERSECT:
			s_params.corkA->boolIsct(*s_params.corkB);
			break;

		case ccCorkDlg::DIFF:
			s_params.corkA->boolDiff(*s_params.corkB);
			break;

		case ccCorkDlg::SYM_DIFF:
			s_params.corkA->boolXor(*s_params.corkB);
			break;

		default:
			assert(false);
			if (s_params.app)
				s_params.app->dispToConsole("Unhandled operation?!", ccMainAppInterface::WRN_CONSOLE_MESSAGE); //DGM: can't issue an error message (i.e. with dialog) in another thread!
			break;
		}
	}
	catch (const std::exception& e)
	{
		if (s_params.app)
			s_params.app->dispToConsole(QString("Exception caught: %1").arg(e.what()), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		return false;
	}

	return true;
}

void qCork::doAction()
{
	assert(m_app);
	if (!m_app)
		return;

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	size_t selNum = selectedEntities.size();
	if (selNum != 2
		|| !selectedEntities[0]->isKindOf(CC_TYPES::MESH)
		|| !selectedEntities[1]->isKindOf(CC_TYPES::MESH))
	{
		assert(false);
		m_app->dispToConsole("Select two and only two meshes!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccMesh* meshA = static_cast<ccMesh*>(selectedEntities[0]);
	ccMesh* meshB = static_cast<ccMesh*>(selectedEntities[1]);

	//show dialog to let the user choose the operation to perform
	ccCorkDlg cDlg(m_app->getMainWindow());
	cDlg.setNames(meshA->getName(), meshB->getName());
	if (!cDlg.exec())
		return;
	if (cDlg.isSwapped())
		std::swap(meshA, meshB);

	//try to convert both meshes to CorkMesh structures
	CorkMesh corkA;
	if (!ToCorkMesh(meshA, corkA, m_app))
		return;
	CorkMesh corkB;
	if (!ToCorkMesh(meshB, corkB, m_app))
		return;

	//launch process
	{
		//run in a separate thread
		QProgressDialog pDlg("Operation in progress", QString(), 0, 0, m_app->getMainWindow());
		pDlg.setWindowTitle("Cork");
		pDlg.show();
		QApplication::processEvents();

		s_params.app = m_app;
		s_params.corkA = &corkA;
		s_params.corkB = &corkB;
		s_params.nameA = meshA->getName();
		s_params.nameB = meshB->getName();
		s_params.operation = cDlg.getSelectedOperation();

		QFuture<bool> future = QtConcurrent::run(doPerformBooleanOp);

		//wait until process is finished!
		while (!future.isFinished())
		{
#if defined(CC_WINDOWS)
			::Sleep(500);
#else
			usleep(500 * 1000);
#endif

			pDlg.setValue(pDlg.value() + 1);
			QApplication::processEvents();
		}

		//just to be sure
		s_params.app = 0;
		s_params.corkA = s_params.corkB = 0;

		pDlg.hide();
		QApplication::processEvents();

		if (!future.result())
		{
			if (m_app)
				m_app->dispToConsole(s_params.meshesAreOk ? "Computation failed!" : "Computation failed! (check console)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			//an error occurred
			return;
		}
	}

	//convert the updated mesh (A) to a new ccMesh structure
	ccMesh* result = FromCorkMesh(corkA);

	if (result)
	{
		meshA->setEnabled(false);
		if (meshB->getDisplay() == meshA->getDisplay())
			meshB->setEnabled(false);

		//set name
		QString opName;
		switch (cDlg.getSelectedOperation())
		{
		case ccCorkDlg::UNION:
			opName = "union";
			break;
		case ccCorkDlg::INTERSECT:
			opName = "isect";
			break;
		case ccCorkDlg::DIFF:
			opName = "diff";
			break;
		case ccCorkDlg::SYM_DIFF:
			opName = "sym_diff";
			break;
		default:
			assert(false);
			break;
		}
		result->setName(QString("(%1).%2.(%3)").arg(meshA->getName()).arg(opName).arg(meshB->getName()));

		//normals
		bool hasNormals = false;
		if (meshA->hasTriNormals())
			hasNormals = result->computePerTriangleNormals();
		else if (meshA->hasNormals())
			hasNormals = result->computePerVertexNormals();
		meshA->showNormals(hasNormals && meshA->normalsShown());

		result->setDisplay(meshA->getDisplay());
		m_app->addToDB(result);
		result->redrawDisplay();
	}

	//currently selected entities appearance may have changed!
	m_app->refreshAll();
}
