//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qMeshBoolean                    #
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

#include "qMeshBoolean.h"

//qCC_db
#include <ccMesh.h>
#include <ccPointCloud.h>

//dialog
#include "ccMeshBooleanDialog.h"

//Qt
#include <QMainWindow>
#include <QtGui>
#include <QProgressDialog>
#include <QtConcurrentRun>

//system
#if defined(CC_WINDOWS)
#include "windows.h"
#else
#include <time.h>
#include <unistd.h>
#endif

//libIGL
#ifdef _MSC_VER
#pragma warning( disable: 4018 )
#pragma warning( disable: 4129 )
#pragma warning( disable: 4267 )
#pragma warning( disable: 4566 )
#endif
#include <igl/copyleft/cgal/mesh_boolean.h>

//! ligIGL mesh
struct IGLMesh
{
	Eigen::MatrixXd V; //!< Vertices
	Eigen::MatrixXi F; //!< Triangles
};

qMeshBoolean::qMeshBoolean(QObject* parent/*=nullptr*/)
	: QObject(parent)
	, ccStdPluginInterface(":/CC/plugin/qMeshBoolean/info.json")
	, m_action(nullptr)
{
}

QList<QAction *> qMeshBoolean::getActions()
{
	//default action
	if (!m_action)
	{
		m_action = new QAction(getName(), this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		
		//connect signal
		connect(m_action, &QAction::triggered, this, &qMeshBoolean::doAction);
	}

	return QList<QAction *>{ m_action };
}

void qMeshBoolean::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
	{
		//we need two and only two meshes!
		m_action->setEnabled(selectedEntities.size() == 2
			&& selectedEntities[0]->isKindOf(CC_TYPES::MESH)
			&& selectedEntities[1]->isKindOf(CC_TYPES::MESH));
	}
}

bool ToIGLMesh(const ccMesh* in, IGLMesh& out, ccMainAppInterface* app = nullptr)
{
	if (!in || !in->getAssociatedCloud())
	{
		if (app)
			app->dispToConsole("Invalid input mesh!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		assert(false);
		return false;
	}

	ccGenericPointCloud* vertices = in->getAssociatedCloud();
	assert(vertices);

	unsigned triCount = in->size();
	unsigned vertCount = vertices ? vertices->size() : 0;

	try
	{
		out.F.resize(triCount, 3);
		out.V.resize(vertCount, 3);
	}
	catch (const std::bad_alloc&)
	{
		if (app)
			app->dispToConsole("Not enough memory!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	//import triangle indexes
	{
		for (unsigned i = 0; i < triCount; i++)
		{
			const CCCoreLib::VerticesIndexes* tsi = in->getTriangleVertIndexes(i);
			out.F(i, 0) = tsi->i1;
			out.F(i, 1) = tsi->i2;
			out.F(i, 2) = tsi->i3;
		}
	}

	//import vertices
	{
		for (unsigned i = 0; i < vertCount; i++)
		{
			const CCVector3* P = vertices->getPoint(i);
			out.V(i, 0) = P->x;
			out.V(i, 1) = P->y;
			out.V(i, 2) = P->z;
		}
	}

	return true;
}

ccMesh* FromIGLMesh(const IGLMesh& in, ccMainAppInterface* app = nullptr)
{
	if (in.F.rows() == 0 || in.V.rows() == 0)
	{
		if (app)
			app->dispToConsole("[Mesh boolean] Output mesh is empty?!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		return nullptr;
	}

	unsigned triCount = static_cast<unsigned>(in.F.rows());
	unsigned vertCount = static_cast<unsigned>(in.V.rows());

	ccPointCloud* vertices = new ccPointCloud("vertices");
	if (!vertices->reserve(vertCount))
	{
		if (app)
			app->dispToConsole("[Mesh boolean] Not enough memory!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		delete vertices;
		return nullptr;
	}

	ccMesh* mesh = new ccMesh(vertices);
	mesh->addChild(vertices);
	if (!mesh->reserve(triCount))
	{
		if (app)
			app->dispToConsole("Not enough memory!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		delete mesh;
		return nullptr;
	}

	//import vertices
	{
		for (unsigned i = 0; i < vertCount; i++)
		{
			CCVector3 Pout(	static_cast<PointCoordinateType>(in.V(i, 0)),
							static_cast<PointCoordinateType>(in.V(i, 1)),
							static_cast<PointCoordinateType>(in.V(i, 2)) );
			vertices->addPoint(Pout);
		}
	}

	//import triangle indexes
	{
		for (unsigned i = 0; i < triCount; i++)
		{
			mesh->addTriangle(in.F(i, 0), in.F(i, 1), in.F(i, 2));
		}
	}

	mesh->setVisible(true);
	vertices->setEnabled(false);

	return mesh;
}

//! Boolean operation parameters (for concurrent run)
struct BoolOpParameters
{
	ccMeshBooleanDialog::CSG_OPERATION operation = ccMeshBooleanDialog::UNION;
	IGLMesh* meshA = nullptr;
	IGLMesh* meshB = nullptr;
	IGLMesh output;
	QString nameA;
	QString nameB;
	ccMainAppInterface* app = nullptr;
};
static BoolOpParameters s_params;

static bool DoPerformBooleanOp()
{
	//invalid parameters
	if (!s_params.meshA || !s_params.meshB)
	{
		assert(false);
		return false;
	}

	try
	{
		QElapsedTimer timer;
		timer.start();

		igl::MeshBooleanType booleanType = igl::NUM_MESH_BOOLEAN_TYPES; // = invalid
		//perform the boolean operation
		switch (s_params.operation)
		{
		case ccMeshBooleanDialog::UNION:
			booleanType = igl::MESH_BOOLEAN_TYPE_UNION;
			break;

		case ccMeshBooleanDialog::INTERSECT:
			booleanType = igl::MESH_BOOLEAN_TYPE_INTERSECT;
			break;

		case ccMeshBooleanDialog::DIFF:
			booleanType = igl::MESH_BOOLEAN_TYPE_MINUS;
			break;

		case ccMeshBooleanDialog::SYM_DIFF:
			booleanType = igl::MESH_BOOLEAN_TYPE_XOR;
			break;

		default:
			assert(false);
			if (s_params.app)
				s_params.app->dispToConsole("[Mesh boolean] Unhandled operation?!", ccMainAppInterface::WRN_CONSOLE_MESSAGE); //DGM: can't issue an error message (i.e. with dialog) in another thread!
			return false;
		}

		s_params.output = IGLMesh();
		if (!igl::copyleft::cgal::mesh_boolean(	s_params.meshA->V,
												s_params.meshA->F,
												s_params.meshB->V,
												s_params.meshB->F,
												booleanType,
												s_params.output.V,
												s_params.output.F ))
		{
			if (s_params.app)
				s_params.app->dispToConsole("[Mesh boolean] CSG operation failed", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			return false;
		}

		if (s_params.app)
		{
			// display the duration time
			s_params.app->dispToConsole(QString("[Mesh boolean] CSG operation duration: %1 s").arg(timer.elapsed() / 1000.0, 0, 'f', 2));
		}

	}
	catch (const std::exception& e)
	{
		if (s_params.app)
			s_params.app->dispToConsole(QString("[Mesh boolean] Exception caught: %1").arg(e.what()), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		return false;
	}

	return true;
}

void qMeshBoolean::doAction()
{
	if (!m_app)
	{
		assert(false);
		return;
	}

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	size_t selNum = selectedEntities.size();
	if (	selNum != 2
		||	!selectedEntities[0]->isKindOf(CC_TYPES::MESH)
		||	!selectedEntities[1]->isKindOf(CC_TYPES::MESH))
	{
		assert(false);
		m_app->dispToConsole("Select two and only two meshes!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccMesh* meshA = static_cast<ccMesh*>(selectedEntities[0]);
	ccMesh* meshB = static_cast<ccMesh*>(selectedEntities[1]);

	//show dialog to let the user choose the operation to perform
	ccMeshBooleanDialog cDlg(m_app->getMainWindow());
	cDlg.setNames(meshA->getName(), meshB->getName());
	if (!cDlg.exec())
		return;

	QElapsedTimer timer;
	timer.start();

	if (cDlg.isSwapped())
		std::swap(meshA, meshB);

	//try to convert both meshes to IGLMesh structures
	IGLMesh iglMeshA;
	if (!ToIGLMesh(meshA, iglMeshA, m_app))
		return;
	IGLMesh iglMeshB;
	if (!ToIGLMesh(meshB, iglMeshB, m_app))
		return;

	//launch process
	{
		//run in a separate thread
		QProgressDialog pDlg(tr("Operation in progress"), QString(), 0, 0, m_app->getMainWindow());
		pDlg.setWindowTitle("Mesh boolean");
		pDlg.show();
		QApplication::processEvents();

		s_params.app = m_app;
		s_params.meshA = &iglMeshA;
		s_params.meshB = &iglMeshB;
		s_params.nameA = meshA->getName();
		s_params.nameB = meshB->getName();
		s_params.operation = cDlg.getSelectedOperation();

		QFuture<bool> future = QtConcurrent::run(DoPerformBooleanOp);

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
		s_params.app = nullptr;
		s_params.meshA = s_params.meshB = nullptr;

		pDlg.hide();
		QApplication::processEvents();

		if (!future.result())
		{
			//an error occurred
			if (m_app)
				m_app->dispToConsole("Computation failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
	}

	//convert the updated mesh (A) to a new ccMesh structure
	ccMesh* result = FromIGLMesh(s_params.output);

	if (result)
	{
		meshA->setEnabled(false);
		if (meshB->getDisplay() == meshA->getDisplay())
			meshB->setEnabled(false);

		//set name
		QString opName;
		switch (cDlg.getSelectedOperation())
		{
		case ccMeshBooleanDialog::UNION:
			opName = "union";
			break;
		case ccMeshBooleanDialog::INTERSECT:
			opName = "isect";
			break;
		case ccMeshBooleanDialog::DIFF:
			opName = "diff";
			break;
		case ccMeshBooleanDialog::SYM_DIFF:
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

	if (m_app)
	{
		// display the duration time
		m_app->dispToConsole(QString("[Mesh boolean] Total duration: %1 s").arg(timer.elapsed() / 1000.0, 0, 'f', 2));
	}
}
