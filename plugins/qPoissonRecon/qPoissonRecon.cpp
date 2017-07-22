//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: qPoissonRecon                      #
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

#include "qPoissonRecon.h"

//dialog
#include "ui_poissonReconParamDlg.h"

//Qt
#include <QtGui>
#include <QInputDialog>
#include <QtCore>
#include <QtConcurrentRun>
#include <QDialog>
#include <QMainWindow>

//PoissonRecon
#include <PoissonReconLib.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>

//System
#if defined(CC_WINDOWS)
#include "windows.h"
#else
#include <time.h>
#include <unistd.h>
#endif

//Dedicated 'OrientedPointStream' for the ccPointCloud structure
template <class Real> class ccPointStream : public OrientedPointStream<Real>
{
public:
	explicit ccPointStream( ccPointCloud* cloud ) : m_cloud(cloud), m_index(0) {}
	virtual void reset( void ) { m_index = 0; }
	virtual bool nextPoint( OrientedPoint3D< Real >& out )
	{
		if (!m_cloud || m_index == m_cloud->size())
		{
			return false;
		}
		//point
		const CCVector3* P = m_cloud->getPoint(m_index);
		out.p[0] = static_cast<Real>(P->x);
		out.p[1] = static_cast<Real>(P->y);
		out.p[2] = static_cast<Real>(P->z);

		//normal
		assert(m_cloud->hasNormals());
		const CCVector3& N = m_cloud->getPointNormal(m_index);
		//DGM: strangely, this new version of PoissonRecon seems to require inverted normals
		out.n[0] = -static_cast<Real>(N.x);
		out.n[1] = -static_cast<Real>(N.y);
		out.n[2] = -static_cast<Real>(N.z);

		//auto-forward
		++m_index;

		return true;
	}

protected:
	ccPointCloud* m_cloud;
	unsigned m_index;
};

//Dedicated 'OrientedPointStream' for the ccPointCloud structure (with colors)
template <class Real> class ccColoredPointStream : public OrientedPointStreamWithData<Real , Point3D< Real > >
{
public:
	explicit ccColoredPointStream( ccPointCloud* cloud ) : m_cloud(cloud), m_index(0) { assert(cloud && cloud->hasColors()); }
	virtual void reset( void ) { m_index = 0; }
	virtual bool nextPoint( OrientedPoint3D< Real >& out, Point3D< Real >& d )
	{
		if (!m_cloud || m_index == m_cloud->size())
		{
			return false;
		}
		//point
		const CCVector3* P = m_cloud->getPoint(m_index);
		out.p[0] = static_cast<Real>(P->x);
		out.p[1] = static_cast<Real>(P->y);
		out.p[2] = static_cast<Real>(P->z);

		//normal
		assert(m_cloud->hasNormals());
		const CCVector3& N = m_cloud->getPointNormal(m_index);
		out.n[0] = -static_cast<Real>(N.x);
		out.n[1] = -static_cast<Real>(N.y);
		out.n[2] = -static_cast<Real>(N.z);

		//color
		assert(m_cloud->hasColors());
		const ColorCompType* rgb = m_cloud->getPointColor(m_index);
		d[0] = static_cast<Real>(rgb[0]);
		d[1] = static_cast<Real>(rgb[1]);
		d[2] = static_cast<Real>(rgb[2]);

		//auto-forward
		++m_index;

		return true;
	}

protected:
	ccPointCloud* m_cloud;
	unsigned m_index;
};

//dialog for qPoissonRecon plugin
class PoissonReconParamDlg : public QDialog, public Ui::PoissonReconParamDialog
{
public:
	explicit PoissonReconParamDlg(QWidget* parent = 0)
		: QDialog(parent, Qt::Tool)
		, Ui::PoissonReconParamDialog()
	{
		setupUi(this);
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
		m_action->setEnabled(selectedEntities.size()==1 && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD));
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

typedef PlyValueVertex< PointCoordinateType > Vertex;
typedef CoredVectorMeshData< Vertex > PoissonMesh;

typedef PlyColorAndValueVertex< PointCoordinateType > ColoredVertex;
typedef CoredVectorMeshData< ColoredVertex > ColoredPoissonMesh;

static PoissonReconLib::Parameters s_params;
static ccPointCloud* s_cloud = 0;
static PoissonMesh* s_mesh = 0;
static ColoredPoissonMesh* s_coloredMesh = 0;
static XForm4x4< PointCoordinateType > s_iXForm;

bool doReconstruct()
{
	//invalid parameters
	if (!s_cloud || !s_mesh)
	{
		return false;
	}

	ccPointStream<PointCoordinateType> pointStream(s_cloud);
	return PoissonReconLib::Reconstruct(s_params, &pointStream, *s_mesh, s_iXForm);
}

bool doReconstructWithColors()
{
	//invalid parameters
	if (!s_cloud || !s_coloredMesh || !s_cloud->hasColors())
	{
		return false;
	}

	ccColoredPointStream<PointCoordinateType> pointStream(s_cloud);
	return PoissonReconLib::Reconstruct(s_params, &pointStream, *s_coloredMesh, s_iXForm);
}

void qPoissonRecon::doAction()
{
	assert(m_app);
	if (!m_app)
	{
		return;
	}

	//we need one point cloud
	if (!m_app->haveOneSelection())
	{
		m_app->dispToConsole("Select only one cloud!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//a real point cloud
	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	
	ccHObject* ent = selectedEntities[0];
	if (!ent->isA(CC_TYPES::POINT_CLOUD))
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

	bool cloudHasColors = pc->hasColors();
	PoissonReconParamDlg prpDlg(m_app->getMainWindow());
	prpDlg.importColorsCheckBox->setVisible(cloudHasColors);

	//init dialog with semi-persistent settings
	prpDlg.octreeLevelSpinBox->setValue(s_params.depth);
	prpDlg.weightDoubleSpinBox->setValue(s_params.pointWeight);
	prpDlg.fullDepthSpinBox->setValue(s_params.fullDepth);
	prpDlg.samplesPerNodeSpinBox->setValue(s_params.samplesPerNode);
	prpDlg.densityCheckBox->setChecked(s_params.density);
	prpDlg.importColorsCheckBox->setChecked(true);
	switch (s_params.boundary)
	{
	case PoissonReconLib::Parameters::FREE:
		prpDlg.boundaryComboBox->setCurrentIndex(0);
		break;
	case PoissonReconLib::Parameters::DIRICHLET:
		prpDlg.boundaryComboBox->setCurrentIndex(1);
		break;
	case PoissonReconLib::Parameters::NEUMANN:
		prpDlg.boundaryComboBox->setCurrentIndex(2);
		break;
	default:
		assert(false);
		break;
	}

	if (!prpDlg.exec())
		return;

	//set parameters with dialog settings
	s_params.depth = prpDlg.octreeLevelSpinBox->value();
	s_params.pointWeight = static_cast<float>(prpDlg.weightDoubleSpinBox->value());
	s_params.fullDepth = prpDlg.fullDepthSpinBox->value();
	s_params.samplesPerNode = static_cast<float>(prpDlg.samplesPerNodeSpinBox->value());
	s_params.density = prpDlg.densityCheckBox->isChecked();
	switch (prpDlg.boundaryComboBox->currentIndex())
	{
	case 0:
		s_params.boundary = PoissonReconLib::Parameters::FREE;
		break;
	case 1:
		s_params.boundary = PoissonReconLib::Parameters::DIRICHLET;
		break;
	case 2:
		s_params.boundary = PoissonReconLib::Parameters::NEUMANN;
		break;
	default:
		assert(false);
		break;
	}
	bool withColors = pc->hasColors() && prpDlg.importColorsCheckBox->isChecked();

	/*** RECONSTRUCTION PROCESS ***/

	PoissonMesh mesh;
	ColoredPoissonMesh coloredMesh;
	s_cloud = 0;
	s_mesh = 0;
	s_coloredMesh = 0;

	//run in a separate thread
	bool result = false;
	{
		//start message
		m_app->dispToConsole(QString("[PoissonRecon] Job started (level %1)").arg(s_params.depth),ccMainAppInterface::STD_CONSOLE_MESSAGE);

		//progress dialog (Qtconcurrent::run can't be canceled!)
		QProgressDialog pDlg("Initialization", QString(), 0, 0, m_app->getMainWindow());
		pDlg.setWindowTitle("Poisson Reconstruction");
		pDlg.show();
		//QApplication::processEvents();

		pDlg.setLabelText(QString("Reconstruction in progress\nlevel: %1 [%2 thread(s)]").arg(s_params.depth).arg(s_params.threads));
		QApplication::processEvents();

		QFuture<bool> future;
			
		//run in a separate thread
		s_cloud = pc;
		if (withColors)
		{
			s_coloredMesh = &coloredMesh;
			future = QtConcurrent::run(doReconstructWithColors);
		}
		else
		{
			s_mesh = &mesh;
			future = QtConcurrent::run(doReconstruct);
		}

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

		result = future.result();

		pDlg.hide();
		QApplication::processEvents();
	}

	if (result
		&&	(!s_mesh || s_mesh->polygonCount() > 0)
		&&	(!s_coloredMesh || s_coloredMesh->polygonCount() > 0))
	{
		unsigned nic = 0, noc = 0, nr_faces = 0;
		if (s_coloredMesh)
		{
			s_coloredMesh->resetIterator();
			nic			= static_cast<unsigned>(s_coloredMesh->inCorePoints.size());
			noc			= static_cast<unsigned>(s_coloredMesh->outOfCorePointCount());
			nr_faces	= static_cast<unsigned>(s_coloredMesh->polygonCount());
		}
		else //if (s_mesh)
		{
			s_mesh->resetIterator();
			nic			= static_cast<unsigned>(s_mesh->inCorePoints.size());
			noc			= static_cast<unsigned>(s_mesh->outOfCorePointCount());
			nr_faces	= static_cast<unsigned>(s_mesh->polygonCount());
		}
		unsigned nr_vertices = nic+noc;

		//end message
		m_app->dispToConsole(QString("[PoissonRecon] Job finished (%1 triangles, %2 vertices)").arg(nr_faces).arg(nr_vertices),ccMainAppInterface::STD_CONSOLE_MESSAGE);

		ccPointCloud* newPC = new ccPointCloud("vertices");
		ccMesh* newMesh = new ccMesh(newPC);
		newMesh->addChild(newPC);

		if (newPC->reserve(nr_vertices) && newMesh->reserve(nr_faces))
		{
			ccScalarField* densitySF = 0;
			if (s_params.density)
			{
				densitySF = new ccScalarField("Density");
				if (!densitySF->reserve(nr_vertices))
				{
					m_app->dispToConsole(QString("[PoissonRecon] Failed to allocate memory for storing density!"),ccMainAppInterface::WRN_CONSOLE_MESSAGE);
					densitySF->release();
					densitySF = 0;
				}
			}

			if (s_coloredMesh)
			{
				bool importColors = newPC->reserveTheRGBTable();
				if (!importColors)
				{
					if (m_app)
						m_app->dispToConsole("Not enough memory to import colors!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				}
				//add 'in core' points
				{
					for (unsigned i = 0; i < nic; i++)
					{
						ColoredVertex p = s_iXForm * s_coloredMesh->inCorePoints[i];
						CCVector3 p2(	static_cast<PointCoordinateType>(p.point.coords[0]),
										static_cast<PointCoordinateType>(p.point.coords[1]),
										static_cast<PointCoordinateType>(p.point.coords[2]) );
						newPC->addPoint(p2);

						if (importColors)
						{
							ColorCompType C[3] = {	static_cast<ColorCompType>(std::min(255.0f, std::max<float>(p.color[0], 0.0))),
													static_cast<ColorCompType>(std::min(255.0f, std::max<float>(p.color[1], 0.0))),
													static_cast<ColorCompType>(std::min(255.0f, std::max<float>(p.color[2], 0.0))) };
							newPC->addRGBColor(C);
						}

						if (densitySF)
						{
							ScalarType sf = static_cast<ScalarType>(p.value);
							densitySF->addElement(sf);
						}
					}
				}
				//add 'out of core' points
				{
					for (unsigned i = 0; i < noc; i++)
					{
						ColoredVertex p;
						s_coloredMesh->nextOutOfCorePoint(p);
						p = s_iXForm * p;
						CCVector3 p2(	static_cast<PointCoordinateType>(p.point.coords[0]),
										static_cast<PointCoordinateType>(p.point.coords[1]),
										static_cast<PointCoordinateType>(p.point.coords[2]) );
						newPC->addPoint(p2);

						if (importColors)
						{
							ColorCompType C[3] = {	static_cast<ColorCompType>(std::min(255.0f, std::max<float>(p.color[0], 0.0))),
													static_cast<ColorCompType>(std::min(255.0f, std::max<float>(p.color[1], 0.0))),
													static_cast<ColorCompType>(std::min(255.0f, std::max<float>(p.color[2], 0.0))) };
							newPC->addRGBColor(C);
						}

						if (densitySF)
						{
							ScalarType sf = static_cast<ScalarType>(p.value);
							densitySF->addElement(sf);
						}
					}
				}

				newPC->showColors(importColors);
			}
			else
			{
				//add 'in core' points
				{
					for (unsigned i = 0; i < nic; i++)
					{
						Vertex p = s_iXForm * s_mesh->inCorePoints[i];
						CCVector3 p2(	static_cast<PointCoordinateType>(p.point.coords[0]),
										static_cast<PointCoordinateType>(p.point.coords[1]),
										static_cast<PointCoordinateType>(p.point.coords[2]) );
						newPC->addPoint(p2);

						if (densitySF)
						{
							ScalarType sf = static_cast<ScalarType>(p.value);
							densitySF->addElement(sf);
						}
					}
				}
				//add 'out of core' points
				{
					for (unsigned i = 0; i < noc; i++)
					{
						Vertex p;
						s_mesh->nextOutOfCorePoint(p);
						p = s_iXForm * p;
						CCVector3 p2(	static_cast<PointCoordinateType>(p.point.coords[0]),
										static_cast<PointCoordinateType>(p.point.coords[1]),
										static_cast<PointCoordinateType>(p.point.coords[2]) );
						newPC->addPoint(p2);

						if (densitySF)
						{
							ScalarType sf = static_cast<ScalarType>(p.value);
							densitySF->addElement(sf);
						}
					}
				}
				newPC->showColors(false);
			}

			// density SF
			if (densitySF)
			{
				densitySF->computeMinAndMax();
				densitySF->showNaNValuesInGrey(false);
				int sfIdx = newPC->addScalarField(densitySF);
				newPC->setCurrentDisplayedScalarField(sfIdx);
				newPC->showSF(true);
				newMesh->showColors(newPC->colorsShown());
				newMesh->showSF(true);
			}

			//add faces
			{
				for (unsigned i = 0; i < nr_faces; i++)
				{
					std::vector<CoredVertexIndex> triangleIndexes;
					if (s_mesh)
						s_mesh->nextPolygon(triangleIndexes);
					else
						s_coloredMesh->nextPolygon(triangleIndexes);

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

			newMesh->setName(QString("Mesh[%1] (level %2)").arg(pc->getName()).arg(s_params.depth));
			newPC->setEnabled(false);
			newMesh->setVisible(true);
			newMesh->computeNormals(true);
			newMesh->showColors(newMesh->hasColors());

			//copy Global Shift & Scale information
			newPC->setGlobalShift(pc->getGlobalShift());
			newPC->setGlobalScale(pc->getGlobalScale());

			//output mesh
			m_app->addToDB(newMesh);
		}
		else
		{
			delete newMesh;
			newMesh = 0;
			m_app->dispToConsole("Not enough memory!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		}

		//currently selected entities parameters may have changed!
		m_app->updateUI();
		//currently selected entities appearance may have changed!
		m_app->refreshAll();
	}
	else
	{
		m_app->dispToConsole("Reconstruction failed!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	}

	s_cloud = 0;
	s_mesh = 0;
	s_coloredMesh = 0;
}

QIcon qPoissonRecon::getIcon() const
{
	return QIcon(QString::fromUtf8(":/CC/plugin/qPoissonRecon/qPoissonRecon.png"));
}
