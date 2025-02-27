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
#include <QDialog>
#include <QInputDialog>
#include <QMainWindow>
#include <QProgressDialog>
#include <QtCore>
#include <QtConcurrentRun>
#include <QtGui>

//PoissonRecon
#include <PoissonReconLib.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>

//System
#if defined(CC_WINDOWS)
#include "Windows.h"
#else
#include <time.h>
#include <unistd.h>
#endif

template <typename Real>
class PointCloudWrapper : public PoissonReconLib::ICloud<Real>
{
public:
	explicit PointCloudWrapper( const ccPointCloud& cloud ) : m_cloud(cloud) {}

	virtual size_t size() const { return m_cloud.size(); }
	virtual bool hasNormals() const { return m_cloud.hasNormals(); }
	virtual bool hasColors() const { return m_cloud.hasColors(); }
	virtual void getPoint(size_t index, Real* coords) const
	{
		if (index >= m_cloud.size())
		{
			assert(false);
			return;
		}
		//point
		const CCVector3* P = m_cloud.getPoint(static_cast<unsigned>(index));
		coords[0] = static_cast<Real>(P->x);
		coords[1] = static_cast<Real>(P->y);
		coords[2] = static_cast<Real>(P->z);
	}

	virtual void getNormal(size_t index, Real* coords) const
	{
		if (index >= m_cloud.size() || !m_cloud.hasNormals())
		{
			assert(false);
			return;
		}

		const CCVector3& N = m_cloud.getPointNormal(static_cast<unsigned>(index));
		coords[0] = static_cast<Real>(N.x);
		coords[1] = static_cast<Real>(N.y);
		coords[2] = static_cast<Real>(N.z);
	}
	
	virtual void getColor(size_t index, Real* rgb) const
	{
		if (index >= m_cloud.size() || !m_cloud.hasColors())
		{
			assert(false);
			return;
		}

		const ccColor::Rgb& color = m_cloud.getPointColor(static_cast<unsigned>(index));
		rgb[0] = static_cast<Real>(color.r);
		rgb[1] = static_cast<Real>(color.g);
		rgb[2] = static_cast<Real>(color.b);
	}

protected:
	const ccPointCloud& m_cloud;
};

template <typename Real>
class MeshWrapper : public PoissonReconLib::IMesh<Real>
{
public:
	explicit MeshWrapper(ccMesh& mesh, ccPointCloud& vertices, CCCoreLib::ScalarField* densitySF = nullptr)
		: m_mesh(mesh)
		, m_vertices(vertices)
		, m_densitySF(densitySF)
		, m_error(false)
	{}

	bool checkMeshCapacity()
	{
		if (m_error)
		{
			//no need to go further
			return false;
		}
		if (m_mesh.size() == m_mesh.capacity() && !m_mesh.reserve(m_mesh.size() + 1024))
		{
			m_error = true;
			return false;
		}
		return true;
	}

	bool checkVertexCapacity()
	{
		if (m_error)
		{
			//no need to go further
			return false;
		}
		if (m_vertices.size() == m_vertices.capacity() && !m_vertices.reserve(m_vertices.size() + 4096))
		{
			m_error = true;
			return false;
		}
		return true;
	}

	virtual void addVertex(const Real* coords) override
	{
		if (!checkVertexCapacity())
		{
			return;
		}
		CCVector3 P = CCVector3::fromArray(coords);
		m_vertices.addPoint(P);
	}

	virtual void addNormal(const Real* coords) override
	{
		if (!checkVertexCapacity())
		{
			return;
		}
		if (!m_vertices.hasNormals() && !m_vertices.reserveTheNormsTable())
		{
			m_error = true;
			return;
		}
		CCVector3 N = CCVector3::fromArray(coords);
		m_vertices.addNorm(N);
	}

	virtual void addColor(const Real* rgb) override
	{
		if (!checkVertexCapacity())
		{
			return;
		}
		if (!m_vertices.hasColors())
		{
			if (!m_vertices.reserveTheRGBTable())
			{
				m_error = true;
				return;
			}
		}
		m_vertices.addColor(	static_cast<ColorCompType>(std::min((Real)255, std::max((Real)0, rgb[0]))),
								static_cast<ColorCompType>(std::min((Real)255, std::max((Real)0, rgb[1]))),
								static_cast<ColorCompType>(std::min((Real)255, std::max((Real)0, rgb[2]))) );
	}

	virtual void addDensity(double d) override
	{
		if (!m_densitySF)
		{
			return;
		}
		if (m_densitySF->size() == m_densitySF->capacity() && !m_densitySF->reserveSafe(m_densitySF->size() + 4096))
		{
			m_error = true;
			return;
		}
		m_densitySF->addElement(static_cast<ScalarType>(d));
	}

	void addTriangle(size_t i1, size_t i2, size_t i3) override
	{
		if (!checkMeshCapacity())
		{
			return;
		}
		m_mesh.addTriangle(static_cast<unsigned>(i1), static_cast<unsigned>(i2), static_cast<unsigned>(i3));
	}

	bool isInErrorState() const { return m_error; }

protected:
	ccMesh& m_mesh;
	ccPointCloud& m_vertices;
	bool m_error;
	CCCoreLib::ScalarField* m_densitySF;
};

//dialog for qPoissonRecon plugin
class PoissonReconParamDlg : public QDialog, public Ui::PoissonReconParamDialog
{
public:
	explicit PoissonReconParamDlg(QWidget* parent = nullptr)
		: QDialog(parent, Qt::Tool)
		, Ui::PoissonReconParamDialog()
	{
		setupUi(this);

		threadSpinBox->setRange(1, PoissonReconLib::Parameters::GetMaxThreadCount());
	}
};

qPoissonRecon::qPoissonRecon(QObject* parent/*=nullptr*/)
	: QObject(parent)
	, ccStdPluginInterface(":/CC/plugin/qPoissonRecon/info.json")
	, m_action(nullptr)
{
}

void qPoissonRecon::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
		m_action->setEnabled(selectedEntities.size()==1 && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD));
}

QList<QAction *> qPoissonRecon::getActions()
{
	//default action
	if (!m_action)
	{
		m_action = new QAction(getName(),this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		//connect signal
		connect(m_action, &QAction::triggered, this, &qPoissonRecon::doAction);
	}

	return QList<QAction *>{ m_action };
}

static PoissonReconLib::Parameters s_params;
static ccPointCloud* s_cloud = nullptr;
static ccMesh* s_mesh = nullptr;
static ccPointCloud* s_meshVertices = nullptr;
static CCCoreLib::ScalarField* s_densitySF = nullptr;

bool doReconstruct()
{
	//invalid parameters
	if (!s_cloud || !s_mesh || !s_meshVertices)
	{
		return false;
	}

	QElapsedTimer timer;
	timer.start();

	MeshWrapper<PointCoordinateType> meshWrapper(*s_mesh, *s_meshVertices, s_densitySF);
	PointCloudWrapper<PointCoordinateType> cloudWrapper(*s_cloud);
	
	if (!PoissonReconLib::Reconstruct(s_params, cloudWrapper, meshWrapper) || meshWrapper.isInErrorState())
	{
		return false;
	}

	qint64 elpased_msec = timer.elapsed();
	ccLog::Print(QString("[PoissonRecon] Duration: %1 s").arg(elpased_msec / 1000.0, 0, 'f', 1));

	return true;
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
		m_app->dispToConsole("Select only one cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//a real point cloud
	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();

	ccHObject* ent = selectedEntities[0];
	if (!ent->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select a cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//with normals!
	ccPointCloud* pc = static_cast<ccPointCloud*>(ent);
	if (!pc->hasNormals())
	{
		m_app->dispToConsole("Cloud must have normals!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	static unsigned s_lastEntityID = 0;
	static double s_defaultResolution = 0.0;
	static bool s_depthMode = true;
	if (s_defaultResolution == 0.0 || s_lastEntityID != pc->getUniqueID())
	{
		s_defaultResolution = pc->getOwnBB().getDiagNormd() / 200.0;
		s_lastEntityID = pc->getUniqueID();
	}
	
	bool cloudHasColors = pc->hasColors();
	PoissonReconParamDlg prpDlg(m_app->getMainWindow());
	prpDlg.importColorsCheckBox->setVisible(cloudHasColors);
	if (s_depthMode)
		prpDlg.depthRadioButton->setChecked(true);
	else
		prpDlg.resolutionRadioButton->setChecked(true);

	//init dialog with semi-persistent settings
	prpDlg.depthSpinBox->setValue(s_params.depth);
	prpDlg.resolutionDoubleSpinBox->setValue(s_defaultResolution);
	prpDlg.samplesPerNodeSpinBox->setValue(s_params.samplesPerNode);
	prpDlg.importColorsCheckBox->setChecked(s_params.withColors);
	prpDlg.densityCheckBox->setChecked(s_params.density);
	prpDlg.weightDoubleSpinBox->setValue(s_params.pointWeight);
	prpDlg.threadSpinBox->setValue(s_params.threads);
	prpDlg.linearFitCheckBox->setChecked(s_params.linearFit);
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
	s_depthMode = prpDlg.depthRadioButton->isChecked();
	s_defaultResolution = prpDlg.resolutionDoubleSpinBox->value();
	
	s_params.depth = (s_depthMode ? prpDlg.depthSpinBox->value() : 0);
	s_params.finestCellWidth = static_cast<float>(s_depthMode ? 0.0 : s_defaultResolution);
	s_params.samplesPerNode = static_cast<float>(prpDlg.samplesPerNodeSpinBox->value());
	s_params.withColors = prpDlg.importColorsCheckBox->isChecked();
	s_params.density = prpDlg.densityCheckBox->isChecked();
	s_params.pointWeight = static_cast<float>(prpDlg.weightDoubleSpinBox->value());
	s_params.threads = prpDlg.threadSpinBox->value();
	s_params.linearFit = prpDlg.linearFitCheckBox->isChecked();
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

	/*** RECONSTRUCTION PROCESS ***/

	assert(s_cloud == nullptr);
	assert(s_mesh == nullptr);
	assert(s_meshVertices == nullptr);

	ccScalarField* densitySF = nullptr;
	ccPointCloud* newPC = new ccPointCloud("vertices");
	ccMesh* newMesh = new ccMesh(newPC);
	newMesh->addChild(newPC);

	//run in a separate thread
	bool result = false;
	{
		//start message
		m_app->dispToConsole(QString("[PoissonRecon] Job started (level %1 - %2 threads)").arg(s_params.depth).arg(s_params.threads), ccMainAppInterface::STD_CONSOLE_MESSAGE);

		//progress dialog (Qtconcurrent::run can't be canceled!)
		QProgressDialog pDlg(tr("Initialization"), QString(), 0, 0, m_app->getMainWindow());
		pDlg.setWindowTitle("Poisson Reconstruction");
		pDlg.setCancelButton(nullptr); // it's not possible to cancel PoissonRecon currently
		pDlg.show();
		//QApplication::processEvents();

		QString progressLabel("Reconstruction in progress\n");
		if (s_depthMode)
			progressLabel += QString("level: %1").arg(s_params.depth);
		else
			progressLabel += QString("resolution: %1").arg(s_params.finestCellWidth);
		progressLabel += QString(" [%1 thread(s)]").arg(s_params.threads);

		pDlg.setLabelText(progressLabel);
		QApplication::processEvents();

		//run in a separate thread
		s_cloud = pc;
		s_mesh = newMesh;
		s_meshVertices = newPC;

		if (s_params.density)
		{
			s_densitySF = (densitySF = new ccScalarField("Density"));
		}

		QFuture<bool> future = QtConcurrent::run(doReconstruct);

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

		s_cloud = nullptr;
		s_mesh = nullptr;
		s_meshVertices = nullptr;

		pDlg.hide();
		QApplication::processEvents();
	}

	if (!result)
	{
		if (densitySF)
		{
			densitySF->release();
			densitySF = nullptr;
		}
		delete newMesh;
		newMesh = nullptr;
		m_app->dispToConsole("Reconstruction failed!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	
	//success message
	m_app->dispToConsole(QString("[PoissonRecon] Job finished (%1 triangles, %2 vertices)").arg(newMesh->size()).arg(newPC->size()), ccMainAppInterface::STD_CONSOLE_MESSAGE);

	newMesh->setName(QString("Mesh[%1] (level %2)").arg(pc->getName()).arg(s_params.depth));
	newPC->setEnabled(false);
	newMesh->setVisible(true);
	newMesh->computeNormals(true);
	if (!cloudHasColors)
	{
		newPC->unallocateColors();
		newPC->showColors(false);
	}
	newMesh->showColors(newPC->hasColors());

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

	//copy Global Shift & Scale information
	newPC->copyGlobalShiftAndScale(*pc);

	//output mesh
	m_app->addToDB(newMesh);
	m_app->setSelectedInDB(ent, false);
	m_app->setSelectedInDB(newMesh, true);

	//currently selected entities parameters may have changed!
	m_app->updateUI();
	//currently selected entities appearance may have changed!
	m_app->refreshAll();
}
