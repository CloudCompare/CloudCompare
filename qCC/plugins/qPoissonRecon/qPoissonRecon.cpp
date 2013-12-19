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

//dialog
#include "ui_poissonReconParamDlg.h"

//Qt
#include <QtGui>
#include <QInputDialog>
#include <QtCore>
#include <QProgressDialog>
#include <QtConcurrentRun>
#include <QMessageBox>
#include <QDialog>

//PoissonRecon
#include <PoissonReconLib.h>

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccPlatform.h>
#include <ccProgressDialog.h>

//CCLib
#include <DistanceComputationTools.h>

//System
#include <algorithm>
#if defined(CC_WINDOWS)
#include "Windows.h"
#else
#include <time.h>
#include <unistd.h>
#endif

//dialog for qPoissonRecon plugin
class PoissonReconParamDlg : public QDialog, public Ui::PoissonReconParamDialog
{
public:
    PoissonReconParamDlg(QWidget* parent = 0) : QDialog(parent), Ui::PoissonReconParamDialog()
	{
		setupUi(this);
		setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);
	}
};

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

static PoissonReconLib::Point* s_points = 0;
static PoissonReconLib::Point* s_normals = 0;
static unsigned s_count = 0;
static PoissonReconLib::Parameters s_params;
static CoredVectorMeshData<PoissonReconLib::Vertex>* s_mesh;
static int s_threadCountUsed = 0;
static PoissonReconLib* s_poisson = 0;

bool doInit()
{
	//invalid parameters
	if (!s_poisson || !s_points || !s_normals || s_params.depth < 2  || s_count == 0)
		return false;

	return s_poisson->init(s_count, s_points, s_normals, s_params, &s_threadCountUsed);
}

bool doReconstruct()
{
	//invalid parameters
	if (!s_poisson || !s_mesh)
		return false;

	return s_poisson->reconstruct(*s_mesh);
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

	PoissonReconParamDlg prpDlg(m_app->getMainWindow());
	//init dialog with semi-persistent settings
	prpDlg.octreeLevelSpinBox->setValue(s_params.depth);
	prpDlg.weightDoubleSpinBox->setValue(s_params.pointWeight);
	prpDlg.minDepthSpinBox->setValue(s_params.minDepth);
	prpDlg.samplesSpinBox->setValue(static_cast<int>(s_params.samplesPerNode));
	prpDlg.solverAccuracyDoubleSpinBox->setValue(s_params.solverAccuracy);

	if (!prpDlg.exec())
		return;

	//set parameters with dialog settings
	s_params.depth = prpDlg.octreeLevelSpinBox->value();
	s_params.pointWeight = static_cast<float>(prpDlg.weightDoubleSpinBox->value());
	s_params.minDepth = prpDlg.minDepthSpinBox->value();
	s_params.samplesPerNode = static_cast<float>(prpDlg.samplesSpinBox->value());
	s_params.solverAccuracy = static_cast<float>(prpDlg.solverAccuracyDoubleSpinBox->value());

	 //TODO: faster, lighter
	unsigned count = pc->size();
	PoissonReconLib::Point* points = new PoissonReconLib::Point[count];
	if (!points)
	{
		m_app->dispToConsole("Not enough memory!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	PoissonReconLib::Point* normals = new PoissonReconLib::Point[count];
	if (!normals)
	{
		delete[] points;
		m_app->dispToConsole("Not enough memory!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//copy points and normals in dedicated arrays
	{
		for (unsigned i=0; i<count; ++i)
		{
			const CCVector3* P = pc->getPoint(i);
			points[i] = PoissonReconLib::Point(	static_cast<float>(P->x),
													static_cast<float>(P->y),
													static_cast<float>(P->z) );

			const PointCoordinateType* N = pc->getPointNormal(i);
			normals[i] = PoissonReconLib::Point(	static_cast<float>(N[0]),
													static_cast<float>(N[1]),
													static_cast<float>(N[2]) );
		}
	}

	/*** RECONSTRUCTION PROCESS ***/

	CoredVectorMeshData<PoissonReconLib::Vertex> mesh;

	//run in a separate thread
	bool result = false;
	{
		//start message
		m_app->dispToConsole(QString("[PoissonRecon] Job started (level %1)").arg(s_params.depth),ccMainAppInterface::STD_CONSOLE_MESSAGE);

		//progress dialog (Qtconcurrent::run can't be canceled!)
		QProgressDialog pDlg("Initialization",QString(),0,0,m_app->getMainWindow());
		pDlg.setWindowTitle("Poisson Reconstruction");
		pDlg.show();
		//QApplication::processEvents();

		PoissonReconLib poisson;

		//run in a separate thread
		s_points = points;
		s_normals = normals;
		s_count = count;
		s_mesh = &mesh;
		s_threadCountUsed = 1;
		s_poisson = &poisson;

		//init
		{
			QFuture<bool> initFuture = QtConcurrent::run(doInit);

			//wait until process is finished!
			while (!initFuture.isFinished())
			{
				#if defined(CC_WINDOWS)
				::Sleep(500);
				#else
				usleep(500 * 1000);
				#endif

				pDlg.setValue(pDlg.value()+1);
				QApplication::processEvents();
			}

			result = initFuture.result();
		}

		//release some memory
		delete[] points;
		s_points = points = 0;
		delete[] normals;
		s_normals = normals = 0;

		//init successful?
		if (result)
		{
			m_app->dispToConsole(QString("[PoissonRecon] Initialization done... starting reconstruction (%1 thread(s))").arg(s_threadCountUsed),ccMainAppInterface::STD_CONSOLE_MESSAGE);

			pDlg.setLabelText(QString("Reconstruction in progress\nlevel: %1 [%2 thread(s)]").arg(s_params.depth).arg(s_threadCountUsed));
			QApplication::processEvents();
			
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
		}

		s_mesh = 0;
		s_poisson = 0;

		pDlg.hide();
		QApplication::processEvents();
	}

	if (!result || mesh.polygonCount() < 1)
	{
		m_app->dispToConsole("Reconstruction failed!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	mesh.resetIterator();
	unsigned nic         = static_cast<unsigned>(mesh.inCorePoints.size());
	unsigned noc         = static_cast<unsigned>(mesh.outOfCorePointCount());
	unsigned nr_faces    = static_cast<unsigned>(mesh.polygonCount());
	unsigned nr_vertices = nic+noc;

	//end message
	m_app->dispToConsole(QString("[PoissonRecon] Job finished (%1 triangles, %2 vertices)").arg(nr_faces).arg(nr_vertices),ccMainAppInterface::STD_CONSOLE_MESSAGE);

	ccPointCloud* newPC = new ccPointCloud("vertices");
	ccMesh* newMesh = new ccMesh(newPC);
	newMesh->addChild(newPC);
	
	if (newPC->reserve(nr_vertices) && newMesh->reserve(nr_faces))
	{
		//add 'in core' points
		{
			for (unsigned i=0; i<nic; i++)
			{
				PoissonReconLib::Vertex& p = mesh.inCorePoints[i];
				CCVector3 p2(	static_cast<PointCoordinateType>(p.point.coords[0]),
								static_cast<PointCoordinateType>(p.point.coords[1]),
								static_cast<PointCoordinateType>(p.point.coords[2]) );
				newPC->addPoint(p2);
			}
		}
		//add 'out of core' points
		{
			for (unsigned i=0; i<noc; i++)
			{
				PoissonReconLib::Vertex p;
				mesh.nextOutOfCorePoint(p);
				CCVector3 p2(	static_cast<PointCoordinateType>(p.point.coords[0]),
								static_cast<PointCoordinateType>(p.point.coords[1]),
								static_cast<PointCoordinateType>(p.point.coords[2]) );
				newPC->addPoint(p2);
			}
		}

		//add faces
		{
			for (unsigned i=0; i<nr_faces; i++)
			{
				std::vector<CoredVertexIndex> triangleIndexes;
				mesh.nextPolygon(triangleIndexes);

				if (triangleIndexes.size() == 3)
				{
					for (std::vector<CoredVertexIndex>::iterator it = triangleIndexes.begin(); it != triangleIndexes.end(); ++it)
						if (!it->inCore)
							it->idx += nic;

					newMesh->addTriangle(	triangleIndexes[0].idx,
											triangleIndexes[1].idx,
											triangleIndexes[2].idx );
				}
				else
				{
					//Can't handle anything else than triangles yet!
					assert(false);
				}
			}
		}

		//if the input cloud has colors, try to 'map' them on the resulting mesh
		if (pc->hasColors())
		{
			if (QMessageBox::question(m_app->getMainWindow(), "Poisson reconstruction","Import input cloud colors? (this may take some time)", QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes)
			{
				if (newPC->reserveTheRGBTable())
				{
					ccProgressDialog pDlg(true, m_app->getMainWindow());
					pDlg.setInfo("Importing input colors");
					pDlg.setMethodTitle("Poisson Reconstruction");
					//pDlg.start();

					//compute the closest-point set of 'newPc' relatively to 'pc'
					//(to get a mapping between the resulting vertices and the input points)
					int result = 0;
					CCLib::ReferenceCloud CPSet(pc);
					{
						CCLib::DistanceComputationTools::Cloud2CloudDistanceComputationParams params;
						params.CPSet = &CPSet;
						params.octreeLevel = 7; //static_cast<uchar>(std::min(depth,CCLib::DgmOctree::MAX_OCTREE_LEVEL)); //TODO: find a better way to set the computation level

						result = CCLib::DistanceComputationTools::computeHausdorffDistance(newPC, pc, params, &pDlg);
					}

					if (result >= 0)
					{
						assert(CPSet.size() == nr_vertices);
						for (unsigned i=0; i<nr_vertices; ++i)
						{
							unsigned index = CPSet.getPointGlobalIndex(i);
							newPC->addRGBColor(pc->getPointColor(index));
						}
						newPC->showColors(true);
						newMesh->showColors(true);
					}
					else
					{
						m_app->dispToConsole(QString("[PoissonReconstruction] Failed to transfer colors: an error (%1) occurred during closest-point set computation!").arg(result),ccMainAppInterface::WRN_CONSOLE_MESSAGE);
						newPC->unallocateColors();
					}
				}
				else
				{
					m_app->dispToConsole("[PoissonReconstruction] Failed to transfer colors: not enough memory!",ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				}
			}
		}

		newMesh->setName(QString("Mesh[%1] (level %2)").arg(pc->getName()).arg(s_params.depth));
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
