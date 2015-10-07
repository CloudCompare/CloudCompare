//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qGMMReg                       #
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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include "qGMMReg.h"

//qCC
#include <ccOrderChoiceDlg.h>

//qCC_db
#include <ccHObjectCaster.h>
#include <ccGenericPointCloud.h>
#include <ccGenericMesh.h>
#include <ccPointCloud.h>
#include <ccMesh.h>

//GMMReg
#include <gmmreg_tps.h>
#include <gmmreg_grbf.h>

//dialog
#include "ui_qGMMRegDialog.h"

//Qt
#include <QtGui>
#include <QElapsedTimer>
#include <QProgressDialog>
#include <QtConcurrentRun>
#include <QMainWindow>

//system
#if defined(CC_WINDOWS)
#include "Windows.h"
#else
#include <time.h>
#include <unistd.h>
#endif

qGMMRegPlugin::qGMMRegPlugin(QObject* parent/*=0*/)
	: QObject(parent)
	, m_action(0)
{}

void qGMMRegPlugin::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
		m_action->setEnabled(selectedEntities.size() == 2);
}

void qGMMRegPlugin::getActions(QActionGroup& group)
{
	//default action
	if (!m_action)
	{
		m_action = new QAction(getName(), this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		//connect appropriate signal
		connect(m_action, SIGNAL(triggered()), this, SLOT(doAction()));
	}

	group.addAction(m_action);
}

static bool CloudToVNLMatrix(const ccGenericPointCloud* cloud, vnl_matrix<double>& matrix, const CCVector3& C)
{
	if (!cloud)
	{
		assert(false);
		return false;
	}

	unsigned pointCount = cloud->size();
	try
	{
		matrix.set_size(pointCount,3);
	}
	catch (...)
	{
		//not enough memory?
		return false;
	}

	for (unsigned i=0; i<pointCount; ++i)
	{
		CCVector3 P = *const_cast<ccGenericPointCloud*>(cloud)->getPoint(i) - C;
		matrix(i,0) = P.x;
		matrix(i,1) = P.y;
		matrix(i,2) = P.z;
	}

	return true;
}

class GMMRegDialog : public QDialog, public Ui::GMMRegDialog
{
public:

	//! Default constructor
	GMMRegDialog(QWidget* parent = 0)
		: QDialog(parent)
		, Ui::GMMRegDialog()
	{
		setupUi(this);
		setWindowFlags(Qt::Tool);

		m_stepWidgets.push_back(StepWidgets(step1CheckBox, step1ScaleDoubleSpinBox, step1LambdaDoubleSpinBox, step1MaxIterSpinBox));
		m_stepWidgets.push_back(StepWidgets(step2CheckBox, step2ScaleDoubleSpinBox, step2LambdaDoubleSpinBox, step2MaxIterSpinBox));
		m_stepWidgets.push_back(StepWidgets(step3CheckBox, step3ScaleDoubleSpinBox, step3LambdaDoubleSpinBox, step3MaxIterSpinBox));
	}

	struct StepWidgets
	{
		StepWidgets(QCheckBox* c, QDoubleSpinBox*s, QDoubleSpinBox* l, QSpinBox* m) : checkbox(c), scale(s), lambda(l), maxIter(m) {}

		QCheckBox* checkbox;
		QDoubleSpinBox *scale, *lambda;
		QSpinBox* maxIter;
	};

	struct StepValues
	{
		StepValues()
			: enabled(false)
			, scale(0)
			, lambda(0)
			, maxIter(0)
		{}

		StepValues(const StepWidgets& w)
			: enabled(w.checkbox->isChecked())
			, scale(w.scale->value())
			, lambda(w.lambda->value())
			, maxIter(w.maxIter->value())
		{}

		bool enabled;
		double scale;
		double lambda;
		int maxIter;
	};

	static void SetWidgetValues(StepWidgets& w, const StepValues& v)
	{
		w.checkbox->setChecked(v.enabled);
		w.scale->setValue(v.scale);
		w.lambda->setValue(v.lambda);
		w.maxIter->setValue(v.maxIter);
	}

	std::vector<StepWidgets> m_stepWidgets;
};

gmmreg::Base* s_reg = 0;
bool doPerformRegistration()
{
	if (s_reg)
	{
		return (s_reg->Run() >= 0);
	}
	else
	{
		assert(false);
		return false;
	}
}

void qGMMRegPlugin::doAction()
{
	//m_app should have already been initialized by CC when plugin is loaded!
	if (!m_app)
	{
		assert(false);
		return;
	}

	/*** HERE STARTS THE ACTION ***/

	//check the input selection
	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	bool validSelection = true;
	if (selectedEntities.size() == 2)
	{
		for (size_t i = 0; i < selectedEntities.size(); ++i)
		{
			if (	!selectedEntities[i]->isKindOf(CC_TYPES::POINT_CLOUD)
				&&	!selectedEntities[i]->isKindOf(CC_TYPES::MESH) )
			{
				validSelection = false;
				break;
			}
		}
	}
	else
	{
		validSelection = false;
	}
	
	if (!validSelection)
	{
		assert(false);
		if (m_app)
			m_app->dispToConsole("Select two entities (clouds or meshes)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccHObject* data = selectedEntities[0];
	ccHObject* model = selectedEntities[1];

	if (model->isKindOf(CC_TYPES::MESH))
	{
		//by default we always set the mesh (if any) as 'data'
		std::swap(data, model);
	}

	//ask the user which one is the 'data' entity and which one is the 'model/reference' entity
	ccOrderChoiceDlg ocDlg(data, "data (deformed)", model, "reference (rigid)", m_app);
	if (!ocDlg.exec())
	{
		return;
	}
	data = ocDlg.getFirstEntity();
	model = ocDlg.getSecondEntity();

	ccGenericPointCloud* d = ccHObjectCaster::ToGenericPointCloud(data);
	ccGenericPointCloud* m = ccHObjectCaster::ToGenericPointCloud(model);
	if (!d || !m)
	{
		assert(false);
		return;
	}

	GMMRegDialog regDlg(m_app ? m_app->getMainWindow() : 0);

	//display a warning message if the clouds are too big
	regDlg.cloudSizeWarningLabel->setVisible(std::max(d->size(), m->size()) > 5000);

	//semi-persistent settings
	size_t maxStepCount = regDlg.m_stepWidgets.size();
	static std::vector<GMMRegDialog::StepValues> s_params(maxStepCount);
	static bool s_firstTime = true;
	static bool s_useControlPoints = true;
	static int s_ctrlPtsCount = 500;
	static int s_algoIndex = 0;
	if (!s_firstTime)
	{
		//init dialog with last time parameters
		for (size_t i=0; i<maxStepCount; ++i)
			GMMRegDialog::SetWidgetValues(regDlg.m_stepWidgets[i], s_params[i]);
		regDlg.useCtrlPtsCheckBox->setChecked(s_useControlPoints);
		regDlg.ctrlPtsSpinBox->setValue(s_ctrlPtsCount);
		regDlg.algoComboBox->setCurrentIndex(s_algoIndex);
	}

	if (!regDlg.exec())
	{
		//process cancelled by user
		return;
	}

	//save parameters for next time (and check some stuff ;)
	{
		size_t enabledSteps = 0;
		for (size_t i=0; i<maxStepCount; ++i)
		{
			s_params[i] = GMMRegDialog::StepValues(regDlg.m_stepWidgets[i]);
			if (s_params[i].enabled)
				++enabledSteps;
		}
		s_useControlPoints = regDlg.useCtrlPtsCheckBox->isChecked();
		s_ctrlPtsCount = regDlg.ctrlPtsSpinBox->value();
		s_algoIndex = regDlg.algoComboBox->currentIndex();
		s_firstTime = false;

		if (enabledSteps == 0)
		{
			if (m_app)
				m_app->dispToConsole("You must enabled at least one step!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
	}

	//we can now prepare GMMReg
	gmmreg::TpsRegistration_L2 tpsReg;
	gmmreg::GrbfRegistration_L2 grbfReg;
	gmmreg::Base* reg = s_algoIndex == 0 ? static_cast<gmmreg::Base*>(&tpsReg) : static_cast<gmmreg::Base*>(&grbfReg);

	CCVector3 C(0,0,0);
	{
		QProgressDialog progressDlg("Initialization in progress...", QString(), 0, 0, m_app ? m_app->getMainWindow() : 0);
		progressDlg.setWindowTitle("Non-rigid registration");
		progressDlg.show();
		QApplication::processEvents();

		QElapsedTimer timer;
		timer.start();

		ccBBox box = d->getOwnBB();
		box += m->getOwnBB();
		//C = box.getCenter();

		vnl_matrix<double> dataPts;
		if (!CloudToVNLMatrix(d, dataPts, C))
		{
			if (m_app)
				m_app->dispToConsole("Failed to convert data entity to VNL matrix (not enough memory?)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		vnl_matrix<double> modelPts;
		if (!CloudToVNLMatrix(m, modelPts, C))
		{
			if (m_app)
				m_app->dispToConsole("Failed to convert model entity to VNL matrix (not enough memory?)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		vnl_matrix<double> ctrlPts;
		if (s_useControlPoints)
		{
			CCVector3 diag = box.getDiagVec();
			CCVector3 bbMin = box.minCorner();

			//try to get as close as possible with a regular grid
			assert(s_ctrlPtsCount >= 1);
			PointCoordinateType l = pow((diag.x * diag.y * diag.z) / s_ctrlPtsCount, static_cast<PointCoordinateType>(1.0/3.0));
			unsigned nx = std::max<unsigned>(2,static_cast<unsigned>(ceil(diag.x / l)));
			unsigned ny = std::max<unsigned>(2,static_cast<unsigned>(ceil(diag.y / l)));
			unsigned nz = std::max<unsigned>(2,static_cast<unsigned>(ceil(diag.z / l)));
			unsigned realCount = nx * ny * nz;

			try
			{
				ctrlPts.set_size(realCount,3);
			}
			catch (...)
			{
				//not enough memory?
				if (m_app)
					m_app->dispToConsole("Not enough memory", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return;
			}

			ccPointCloud* test = 0;
#ifdef EXPORT_CONTROL_POINTS
			test = new ccPointCloud("grid");
			test->reserve(count);
#endif
			unsigned n = 0;
			for (unsigned k=0; k<nz; ++k)
			{
				CCVector3 P(0,0,bbMin.z + k * diag.z / (nz-1));
				for (unsigned j=0; j<ny; ++j)
				{
					P.y = bbMin.y + j * diag.y / (ny-1);
					for (unsigned i=0; i<nx; ++i, ++n)
					{
						P.x = bbMin.x + i * diag.x / (nx-1);

						assert(n < realCount);
						ctrlPts(n,0) = P.x;
						ctrlPts(n,1) = P.y;
						ctrlPts(n,2) = P.z;

#ifdef EXPORT_CONTROL_POINTS
						if (test)
							test->addPoint(P);
#endif
					}
				}
			}

#ifdef EXPORT_CONTROL_POINTS
			if (test)
			{
				if (m_app)
					m_app->addToDB(test);
				else
					delete test;
			}
#endif
		}
		
		//parameters
		int normalize = 1;
		
		unsigned level = 0; //<=3
		std::vector<float> v_scale(3);
		std::vector<int> v_func_evals(3);
		std::vector<float> v_lambda(3);
		std::vector<int> v_affine(3);

		for (size_t i=0; i<maxStepCount; ++i)
		{
			if (s_params[i].enabled)
			{
				v_scale     [level] = static_cast<float>(s_params[i].scale);
				v_func_evals[level] = s_params[i].maxIter;
				v_lambda    [level] = static_cast<float>(s_params[i].lambda);
				v_affine    [level] = 0;
				++level;
			}
		}

		assert(level >= 1 && level <= 3); //see previous tests

		//multi-scale options
		if (reg->MultiScaleOptions(level, v_scale, v_func_evals) < 0)
		{
			if (m_app)
				m_app->dispToConsole("Mutli-scale options setting failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		if (reg == &tpsReg)
		{
			//TPS options
			if (tpsReg.PrepareOwnOptions(v_lambda, v_affine) < 0)
			{
				if (m_app)
					m_app->dispToConsole("TPS options setting failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return;
			}
		}
		else if (reg == &grbfReg)
		{
			//GRBF options
			if (grbfReg.PrepareOwnOptions(v_lambda) < 0)
			{
				if (m_app)
					m_app->dispToConsole("GRBF options setting failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return;
			}
		}
  
		//initialization
		if (reg->Initialize(dataPts, modelPts, s_useControlPoints ? &ctrlPts : 0, normalize) < 0)
		{
			if (m_app)
				m_app->dispToConsole("GMM initialization failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		qint64 intiTime_ms = timer.elapsed();
		if (m_app)
			m_app->dispToConsole(QString("[GMMReg] Initialization done (%1 msec)").arg(intiTime_ms), ccMainAppInterface::STD_CONSOLE_MESSAGE);
		
		progressDlg.setLabelText("Registration in progress... please wait...");
		QApplication::processEvents();

		try
		{
			//eventually we can run the registration (in a separate thread)
			s_reg = reg;
			QFuture<bool> future = QtConcurrent::run(doPerformRegistration);

			//wait until process is finished!
			while (!future.isFinished())
			{
#if defined(CC_WINDOWS)
				::Sleep(500);
#else
				usleep(500 * 1000);
#endif

				progressDlg.setValue(progressDlg.value()+1);
				QApplication::processEvents();
			}

			s_reg = 0;

			if (!future.result())
			{
				if (m_app)
					m_app->dispToConsole("GMM registration failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return;
			}
		}
		catch (...)
		{
			if (m_app)
				m_app->dispToConsole("Unknown exception caught", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		qint64 endTime_ms = timer.elapsed();
		if (m_app)
		{
			m_app->dispToConsole(QString("[GMMReg] Registration done (%1 msec)").arg(endTime_ms-intiTime_ms), ccMainAppInterface::STD_CONSOLE_MESSAGE);
			m_app->dispToConsole(QString("[GMMReg] Total time: %1 s.").arg(endTime_ms / 1000.0), ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}
	}
	
	const vnl_matrix<double>& outputMatrix = reg->transformedModel();

	ccHObject* clonedData = 0;
	if (data->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		clonedData = static_cast<ccGenericPointCloud*>(data)->clone();
	}
	else if (data->isKindOf(CC_TYPES::MESH))
	{
		if (data->isA(CC_TYPES::MESH))
		{
			ccMesh* mesh = ccHObjectCaster::ToMesh(data);
			clonedData = mesh->cloneMesh();
		}
		else
		{
			if (m_app)
				m_app->dispToConsole("Can't duplicate sub-meshes (vertices will be updated directly!)", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			clonedData = data;
		}
	}

	if (!clonedData)
	{
		if (m_app)
			m_app->dispToConsole("Can't duplicate the input entity (not enough memory?). Vertices will be updated directly!)", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		clonedData = data;
	}

	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(clonedData);
	if (cloud && cloud->size() == outputMatrix.rows())
	{
		for (unsigned i = 0; i < outputMatrix.rows(); ++i)
		{
			CCVector3* P = const_cast<CCVector3*>(cloud->getPoint(i));
			P->x = static_cast<PointCoordinateType>(outputMatrix(i, 0)) + C.x;
			P->y = static_cast<PointCoordinateType>(outputMatrix(i, 1)) + C.y;
			P->z = static_cast<PointCoordinateType>(outputMatrix(i, 2)) + C.z;
		}
		if (cloud->isA(CC_TYPES::POINT_CLOUD))
		{
			static_cast<ccPointCloud*>(cloud)->invalidateBoundingBox();
		}
		clonedData->redrawDisplay();

		if (clonedData != data)
		{
			data->setEnabled(false);
			m_app->addToDB(clonedData);
		}
	}
	else
	{
		if (clonedData != data)
			delete clonedData;

		assert(false);
	}

	/*** HERE ENDS THE ACTION ***/
}

QIcon qGMMRegPlugin::getIcon() const
{
	return QIcon(":/CC/plugin/qGMMRegPlugin/icon.png");
}

#ifndef CC_QT5
Q_EXPORT_PLUGIN2(qGMMRegPlugin,qGMMRegPlugin);
#endif
