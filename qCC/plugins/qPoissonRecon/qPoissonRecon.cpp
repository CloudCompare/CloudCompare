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

//Qt
#include <QtGui>
#include <QInputDialog>
#include <QtCore>

#include "qPoissonRecon.h"

//PoissonRecon
#include <PoissonReconLib.h>

//qCC_db
#include <ccProgressDialog.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccMesh.h>

//System
#include <algorithm>
#if defined(_WIN32) || defined(WIN32)
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

static bool s_result = false;
static float* s_points = 0;
static float* s_normals = 0;
static unsigned s_count = 0;
static int s_depth = 0;
static CoredVectorMeshData* s_mesh;
static PoissonReconLib::PoissonReconResultInfo* s_info;

void doReconstruct()
{
	s_result = false;

	//invalid parameters
	if (!s_points || !s_normals || !s_mesh || s_depth < 2  || s_count==0)
		return;

	s_result = PoissonReconLib::reconstruct(s_count, s_points, s_normals, *s_mesh, s_depth, s_info);
}

void qPoissonRecon::doAction()
{
	assert(m_app);
	if (!m_app)
		return;

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();

	//we need one point cloud
    size_t selNum = selectedEntities.size();
    if (selNum!=1)
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
	unsigned i,count = pc->size();
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

	float* _points = points;
	float* _normals = normals;
	for (i=0;i<count;++i)
	{
		const CCVector3* P = pc->getPoint(i);
		*_points++ = (float)P->x;
		*_points++ = (float)P->y;
		*_points++ = (float)P->z;

		const PointCoordinateType* N = pc->getPointNormal(i);
		*_normals++ = (float)N[0];
		*_normals++ = (float)N[1];
		*_normals++ = (float)N[2];
	}

	/*** RECONSTRUCTION PROCESS ***/

	CoredVectorMeshData mesh;
	PoissonReconLib::PoissonReconResultInfo info;
	bool result = false;

	//progress dialog
	ccProgressDialog progressCb(false,m_app->getMainWindow());
	{
		progressCb.setCancelButton(0);
		progressCb.setRange(0,0);
		progressCb.setInfo("Operation in progress");
		progressCb.setMethodTitle("Poisson Reconstruction");
		progressCb.start();
		//QApplication::processEvents();

		//run in a separate thread
		s_points = points;
		s_normals = normals;
		s_count = count;
		s_depth = depth;
		s_mesh = &mesh;
		s_info = &info;
		QFuture<void> future = QtConcurrent::run(doReconstruct);

		unsigned progress = 0;
		while (!future.isFinished())
		{
			#if defined(_WIN32) || defined(WIN32)
			::Sleep(500);
			#else
			struct timespec waiter = {0, 500000000L};
			nanosleep(&waiter, NULL);
			#endif

			progressCb.update(++progress);
			//Qtconcurrent::run can't be canceled!
			/*if (progressCb.isCancelRequested())
			{
				future.cancel();
				future.waitForFinished();
				s_result = false;
				break;
			}
			//*/
		}

		result = s_result;

		progressCb.stop();
		QApplication::processEvents();
	}
	//else
	//{
	//	result = PoissonReconLib::reconstruct(count, points, normals, mesh, depth, &info);
	//}

	delete[] points;
	points=0;
	delete[] normals;
	normals=0;

	if (!result || mesh.polygonCount() < 1)
	{
		m_app->dispToConsole("Reconstruction failed!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	unsigned nic         = (unsigned)mesh.inCorePoints.size();
	unsigned noc         = (unsigned)mesh.outOfCorePointCount();
	unsigned nr_vertices = nic+noc;
	unsigned nr_faces    = (unsigned)mesh.polygonCount();

	ccPointCloud* newPC = new ccPointCloud("vertices");
	newPC->reserve(nr_vertices);

	//we enlarge bounding box a little bit (2%)
	PointCoordinateType bbMin[3],bbMax[3];
	pc->getBoundingBox(bbMin,bbMax);
	CCVector3 boxHalfDiag = (CCVector3(bbMax)-CCVector3(bbMin))*0.51f;
	CCVector3 boxCenter = (CCVector3(bbMax)+CCVector3(bbMin))*0.5f;
	CCVector3 filterMin = boxCenter-boxHalfDiag;
	CCVector3 filterMax = boxCenter+boxHalfDiag;

	Point3D<float> p;
	CCVector3 p2;
	for (i=0; i<nic; i++)
	{
		p = mesh.inCorePoints[i];
		p2.x = p.coords[0]*info.scale+info.center[0];
		p2.y = p.coords[1]*info.scale+info.center[1];
		p2.z = p.coords[2]*info.scale+info.center[2];
		newPC->addPoint(p2);
	}
	for (i=0; i<noc; i++)
	{
		mesh.nextOutOfCorePoint(p);
		p2.x = p.coords[0]*info.scale+info.center[0];
		p2.y = p.coords[1]*info.scale+info.center[1];
		p2.z = p.coords[2]*info.scale+info.center[2];
		newPC->addPoint(p2);
	}

	ccMesh* newMesh = new ccMesh(newPC);
	newMesh->setName(QString("Mesh[%1] (level %2)").arg(pc->getName()).arg(depth));
	newMesh->reserve(nr_faces);
	newMesh->addChild(newPC);

	std::vector<CoredVertexIndex> vertices;
	for (i=0; i < nr_faces; i++)
	{
		mesh.nextPolygon(vertices);

		if (vertices.size()!=3)
		{
			//Can't handle anything else than triangles yet!
			assert(false);
		}
		else
		{
			for (std::vector<CoredVertexIndex>::iterator it = vertices.begin(); it != vertices.end(); ++it)
				if (!it->inCore)
					it->idx += nic;

			newMesh->addTriangle(vertices[0].idx,
								vertices[1].idx,
								vertices[2].idx);
		}
	}

	newPC->setVisible(false);
	newMesh->setVisible(true);
	newMesh->computeNormals();

	//output mesh
	m_app->addToDB(newMesh);

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
