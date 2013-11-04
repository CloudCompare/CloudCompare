//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: qPoissonRecon                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#               COPYRIGHT: Daniel Girardeau-Montaut                      #
//#                                                                        #
//##########################################################################

#include "qPoissonRecon.h"

//Qt
#include <QtGui>
#include <QInputDialog>
#include <QtCore>
#include <QProgressDialog>
#include <QtConcurrentRun>

//PoissonRecon
#include <PoissonReconLib.h>

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccPlatform.h>

//System
#include <algorithm>
#if defined(CC_WINDOWS)
#include "Windows.h"
#else
#include <time.h>
#endif

qPoissonRecon::qPoissonRecon(QObject* parent/*=0*/)
	: QObject(parent)
	, m_action(0)
{
}

void qPoissonRecon::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
		m_action->setEnabled(selectedEntities.size()==1 && selectedEntities[0]->isA(CC_POINT_CLOUD));
}

void qPoissonRecon::getActions(QActionGroup& group)
{
	//default action
	if (!m_action)
	{
		m_action = new QAction(getName(),this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		//connect signal
		connect(m_action, SIGNAL(triggered()), this, SLOT(doAction()));
	}

	group.addAction(m_action);
}

static float* s_points = 0;
static float* s_normals = 0;
static unsigned s_count = 0;
static int s_depth = 0;
static CoredVectorMeshData* s_mesh;
static PoissonReconLib::PoissonReconResultInfo* s_info;

bool doReconstruct()
{
	//invalid parameters
	if (!s_points || !s_normals || !s_mesh || s_depth < 2  || s_count == 0)
		return false;

	return PoissonReconLib::reconstruct(s_count, s_points, s_normals, *s_mesh, s_depth, s_info);
}

void qPoissonRecon::doAction()
{
	assert(m_app);
	if (!m_app)
		return;

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();

	//we need one point cloud
    size_t selNum = selectedEntities.size();
    if (selNum != 1)
	{
		m_app->dispToConsole("Select only one cloud!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//a real point cloud
    ccHObject* ent = selectedEntities[0];
	if (!ent->isA(CC_POINT_CLOUD))
	{
		m_app->dispToConsole("Select a cloud!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//with normals!
    ccPointCloud* pc = static_cast<ccPointCloud*>(ent);
	if (!pc->hasNormals())
	{
		m_app->dispToConsole("Cloud must have normals!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	bool ok;
	#if (QT_VERSION >= QT_VERSION_CHECK(4, 5, 0))
	int depth = QInputDialog::getInt(0, "Poisson reconstruction","Octree depth:", 8, 1, 24, 1, &ok);
	#else
	int depth = QInputDialog::getInteger(0, "Poisson reconstruction","Octree depth:", 8, 1, 24, 1, &ok);
	#endif

	if (!ok)
		return;

	 //TODO: faster, lighter
	unsigned count = pc->size();
	float* points = new float[count*3];
	if (!points)
	{
		m_app->dispToConsole("Not enough memory!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	float* normals = new float[count*3];
	if (!normals)
	{
		delete[] points;
		m_app->dispToConsole("Not enough memory!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//copy points and normals in dedicated arrays
	{
		float* _points = points;
		float* _normals = normals;
		for (unsigned i=0; i<count; ++i)
		{
			const CCVector3* P = pc->getPoint(i);
			*_points++ = static_cast<float>(P->x);
			*_points++ = static_cast<float>(P->y);
			*_points++ = static_cast<float>(P->z);

			const PointCoordinateType* N = pc->getPointNormal(i);
			*_normals++ = static_cast<float>(N[0]);
			*_normals++ = static_cast<float>(N[1]);
			*_normals++ = static_cast<float>(N[2]);
		}
	}

	/*** RECONSTRUCTION PROCESS ***/

	CoredVectorMeshData mesh;
	PoissonReconLib::PoissonReconResultInfo info;

	//run in a separate thread
	bool result = false;
	{
		//progress dialog (Qtconcurrent::run can't be canceled!)
		QProgressDialog pDlg("Operation in progress",QString(),0,0,m_app->getMainWindow());
		pDlg.setWindowTitle("Poisson Reconstruction");
		pDlg.show();
		//QApplication::processEvents();

		//run in a separate thread
		s_points = points;
		s_normals = normals;
		s_count = count;
		s_depth = depth;
		s_mesh = &mesh;
		s_info = &info;
		QFuture<bool> future = QtConcurrent::run(doReconstruct);

		//wait until process is finished!
		while (!future.isFinished())
		{
			#if defined(CC_WINDOWS)
			::Sleep(500);
			#else
            usleep(500 * 1000);
			#endif

			pDlg.setValue(pDlg.value()+1);
			QApplication::processEvents();
		}

		result = future.result();

		pDlg.hide();
		QApplication::processEvents();
	}

	//release some memory
	delete[] points;
	points = 0;
	delete[] normals;
	normals = 0;

	if (!result || mesh.polygonCount() < 1)
	{
		m_app->dispToConsole("Reconstruction failed!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	unsigned nic         = static_cast<unsigned>(mesh.inCorePoints.size());
	unsigned noc         = static_cast<unsigned>(mesh.outOfCorePointCount());
	unsigned nr_faces    = static_cast<unsigned>(mesh.polygonCount());
	unsigned nr_vertices = nic+noc;

	ccPointCloud* newPC = new ccPointCloud("vertices");
	ccMesh* newMesh = new ccMesh(newPC);
	newMesh->addChild(newPC);
	
	if (newPC->reserve(nr_vertices) && newMesh->reserve(nr_faces))
	{
		//add 'in core' points
		{
			for (unsigned i=0; i<nic; i++)
			{
				const Point3D<float>& p = mesh.inCorePoints[i];
				CCVector3 p2(	p.coords[0]*info.scale + info.center[0],
								p.coords[1]*info.scale + info.center[1],
								p.coords[2]*info.scale + info.center[2] );
				newPC->addPoint(p2);
			}
		}
		//add 'out of core' points
		{
			for (unsigned i=0; i<noc; i++)
			{
				Point3D<float> p;
				mesh.nextOutOfCorePoint(p);
				CCVector3 p2(	p.coords[0]*info.scale + info.center[0],
								p.coords[1]*info.scale + info.center[1],
								p.coords[2]*info.scale + info.center[2] );
				newPC->addPoint(p2);
			}
		}

		//add faces
		{
			for (unsigned i=0; i<nr_faces; i++)
			{
				std::vector<CoredVertexIndex> vertices;
				mesh.nextPolygon(vertices);

				if (vertices.size() == 3)
				{
					for (std::vector<CoredVertexIndex>::iterator it = vertices.begin(); it != vertices.end(); ++it)
						if (!it->inCore)
							it->idx += nic;

					newMesh->addTriangle(	vertices[0].idx,
											vertices[1].idx,
											vertices[2].idx );
				}
				else
				{
					//Can't handle anything else than triangles yet!
					assert(false);
				}
			}
		}

		newMesh->setName(QString("Mesh[%1] (level %2)").arg(pc->getName()).arg(depth));
		newPC->setVisible(false);
		newMesh->setVisible(true);
		newMesh->computeNormals();

		//output mesh
		m_app->addToDB(newMesh);
	}
	else
	{
		delete newMesh;
		newMesh = 0;
		m_app->dispToConsole("Not enough memory!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	}

	//currently selected entities parameters may have changed!
	m_app->updateUI();
    //currently selected entities appearance may have changed!
	m_app->refreshAll();
}

QIcon qPoissonRecon::getIcon() const
{
    return QIcon(QString::fromUtf8(":/CC/plugin/qPoissonRecon/qPoissonRecon.png"));
}

Q_EXPORT_PLUGIN2(qPoissonRecon,qPoissonRecon);
