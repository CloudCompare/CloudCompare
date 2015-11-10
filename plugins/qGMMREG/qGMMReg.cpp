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

//Local
#include "ccGMMRegWrapper.h"

//qCC
#include <ccOrderChoiceDlg.h>

//qCC_db
#include <ccHObjectCaster.h>
#include <ccGenericPointCloud.h>
#include <ccGenericMesh.h>
#include <ccPointCloud.h>
#include <ccMesh.h>

//dialog
#include "ui_qGMMRegDialog.h"

//Qt
#include <QtGui>
#include <QMainWindow>

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

	static void SetValuesFromWidget(ccGMMRegWrapper::StepValues& values, const StepWidgets& w)
	{
		values.enabled = w.checkbox->isChecked();
		values.scale = w.scale->value();
		values.lambda = w.lambda->value();
		values.maxIter = w.maxIter->value();
	}

	static void SetWidgetValues(StepWidgets& w, const ccGMMRegWrapper::StepValues& v)
	{
		w.checkbox->setChecked(v.enabled);
		w.scale->setValue(v.scale);
		w.lambda->setValue(v.lambda);
		w.maxIter->setValue(v.maxIter);
	}

	std::vector<StepWidgets> m_stepWidgets;
};

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
	static std::vector<ccGMMRegWrapper::StepValues> s_params(maxStepCount);
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
			GMMRegDialog::SetValuesFromWidget(s_params[i], regDlg.m_stepWidgets[i]);
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

	std::vector<CCVector3> displacedPoints;
	if (!ccGMMRegWrapper::PerformRegistration(d, m, s_params, displacedPoints, /*useTPS=*/(s_algoIndex == 0), s_useControlPoints ? s_ctrlPtsCount : 0, m_app))
	{
		//an error occurred (error message should have already been output)
		return;
	}

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
	if (cloud && cloud->size() == displacedPoints.size())
	{
		for (unsigned i = 0; i < displacedPoints.size(); ++i)
		{
			CCVector3* P = const_cast<CCVector3*>(cloud->getPoint(i));
			*P = displacedPoints[i];
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
