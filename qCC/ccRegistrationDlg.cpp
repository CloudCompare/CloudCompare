//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccRegistrationDlg.h"

//Local
#include "mainwindow.h"

//common
#include <ccQtHelpers.h>

//CCLib
#include <DgmOctree.h>
#include <CloudSamplingTools.h>
#include <GeometricalAnalysisTools.h>
#include <ReferenceCloud.h>

//qCC_db
#include <ccHObject.h>

//Qt
#include <QThread>

//system
#include <assert.h>

static bool     s_adjustScale = false;
static unsigned s_randomSamplingLimit = 50000;
static double   s_rmsDifference = 1.0e-5;
static int      s_maxIterationCount = 20;
static bool     s_useErrorDifferenceCriterion = true;
static int      s_finalOverlap = 100;
static int      s_rotComboIndex = 0;
static bool     s_transCheckboxes[3] = { true, true, true };
static int		s_maxThreadCount = 0;


ccRegistrationDlg::ccRegistrationDlg(ccHObject *data, ccHObject *model, QWidget* parent/*=0*/)
	: QDialog(parent, Qt::Tool)
	, Ui::RegistrationDialog()
{
	assert(data && model);
	dataEntity = data;
	modelEntity = model;

	setupUi(this);
	QDoubleValidator* rmsValidator = new QDoubleValidator(rmsDifferenceLineEdit);
	rmsValidator->setRange(1.0e-7, 1.0);
	rmsDifferenceLineEdit->setValidator(rmsValidator);

	setColorsAndLabels();

	ccQtHelpers::SetButtonColor(dataColorButton, Qt::red);
	ccQtHelpers::SetButtonColor(modelColorButton, Qt::yellow);

	int idealThreadCount = QThread::idealThreadCount();
	maxThreadCountSpinBox->setRange(1, idealThreadCount);
	maxThreadCountSpinBox->setSuffix(QString(" / %1").arg(idealThreadCount));

	//restore semi-persistent settings
	{
		//semi-persistent options
		if (s_maxThreadCount == 0)
		{
			s_maxThreadCount = idealThreadCount;
		}
		maxThreadCountSpinBox->setValue(s_maxThreadCount);
		adjustScaleCheckBox->setChecked(s_adjustScale);
		randomSamplingLimitSpinBox->setValue(s_randomSamplingLimit);
		rmsDifferenceLineEdit->setText(QString::number(s_rmsDifference, 'e', 1));
		maxIterationCount->setValue(s_maxIterationCount);
		if (s_useErrorDifferenceCriterion)
			errorCriterion->setChecked(true);
		else
			iterationsCriterion->setChecked(true);
		overlapSpinBox->setValue(s_finalOverlap);
		rotComboBox->setCurrentIndex(s_rotComboIndex);
		TxCheckBox->setChecked(s_transCheckboxes[0]);
		TyCheckBox->setChecked(s_transCheckboxes[1]);
		TzCheckBox->setChecked(s_transCheckboxes[2]);
	}

	connect(swapButton, &QAbstractButton::clicked, this, &ccRegistrationDlg::swapModelAndData);
}

ccRegistrationDlg::~ccRegistrationDlg()
{
	if (modelEntity)
	{
		modelEntity->enableTempColor(false);
		modelEntity->prepareDisplayForRefresh_recursive();
	}
	if (dataEntity)
	{
		dataEntity->enableTempColor(false);
		dataEntity->prepareDisplayForRefresh_recursive();
	}

	MainWindow::RefreshAllGLWindow(false);
}

void ccRegistrationDlg::saveParameters() const
{
	s_maxThreadCount = getMaxThreadCount();
	s_adjustScale = adjustScale();
	s_randomSamplingLimit = randomSamplingLimit();
	s_rmsDifference = getMinRMSDecrease();
	s_maxIterationCount = getMaxIterationCount();
	s_useErrorDifferenceCriterion = errorCriterion->isChecked();
	s_finalOverlap = overlapSpinBox->value();
	s_rotComboIndex = rotComboBox->currentIndex();
	s_transCheckboxes[0] = TxCheckBox->isChecked();
	s_transCheckboxes[1] = TyCheckBox->isChecked();
	s_transCheckboxes[2] = TzCheckBox->isChecked();
}

ccHObject *ccRegistrationDlg::getDataEntity()
{
	return dataEntity;
}

ccHObject *ccRegistrationDlg::getModelEntity()
{
	return modelEntity;
}

bool ccRegistrationDlg::useDataSFAsWeights() const
{
	return checkBoxUseDataSFAsWeights->isEnabled() && checkBoxUseDataSFAsWeights->isChecked();
}

bool ccRegistrationDlg::useModelSFAsWeights() const
{
	return checkBoxUseModelSFAsWeights->isEnabled() && checkBoxUseModelSFAsWeights->isChecked();
}

bool ccRegistrationDlg::adjustScale() const
{
	return adjustScaleCheckBox->isChecked();
}

bool ccRegistrationDlg::removeFarthestPoints() const
{
	return pointsRemoval->isChecked();
}

unsigned ccRegistrationDlg::randomSamplingLimit() const
{
	return randomSamplingLimitSpinBox->value();
}

unsigned ccRegistrationDlg::getMaxIterationCount() const
{
	return static_cast<unsigned>(std::max(1,maxIterationCount->value()));
}

unsigned ccRegistrationDlg::getFinalOverlap() const
{
	return static_cast<unsigned>(std::max(10,overlapSpinBox->value()));
}

int ccRegistrationDlg::getMaxThreadCount() const
{
	return maxThreadCountSpinBox->value();
}

double ccRegistrationDlg::getMinRMSDecrease() const
{
	bool ok = true;
	double val = rmsDifferenceLineEdit->text().toDouble(&ok);
	assert(ok);

	return val;
}

ccRegistrationDlg::ConvergenceMethod ccRegistrationDlg::getConvergenceMethod() const
{
	if (errorCriterion->isChecked())
		return CCLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE;
	else
		return CCLib::ICPRegistrationTools::MAX_ITER_CONVERGENCE;
}

int ccRegistrationDlg::getTransformationFilters() const
{
	int filters = 0;
	switch (rotComboBox->currentIndex())
	{
	case 1:
		filters |= CCLib::RegistrationTools::SKIP_RYZ;
		break;
	case 2:
		filters |= CCLib::RegistrationTools::SKIP_RXZ;
		break;
	case 3:
		filters |= CCLib::RegistrationTools::SKIP_RXY;
		break;
	default:
		//nothing to do
		break;
	}

	if (!TxCheckBox->isChecked())
		filters |= CCLib::RegistrationTools::SKIP_TX;
	if (!TyCheckBox->isChecked())
		filters |= CCLib::RegistrationTools::SKIP_TY;
	if (!TzCheckBox->isChecked())
		filters |= CCLib::RegistrationTools::SKIP_TZ;

	return filters;
}

void ccRegistrationDlg::setColorsAndLabels()
{
	if (!modelEntity || !dataEntity)
		return;

	modelLineEdit->setText(modelEntity->getName());
	modelEntity->setVisible(true);
	modelEntity->setTempColor(ccColor::yellow);
	modelEntity->prepareDisplayForRefresh_recursive();

	dataLineEdit->setText(dataEntity->getName());
	dataEntity->setVisible(true);
	dataEntity->setTempColor(ccColor::red);
	dataEntity->prepareDisplayForRefresh_recursive();

	checkBoxUseDataSFAsWeights->setEnabled(dataEntity->hasDisplayedScalarField());
	checkBoxUseModelSFAsWeights->setEnabled(modelEntity->hasDisplayedScalarField());

	MainWindow::RefreshAllGLWindow(false);
}

void ccRegistrationDlg::swapModelAndData()
{
	std::swap(dataEntity,modelEntity);
	setColorsAndLabels();
	checkBoxUseModelSFAsWeights->setDisabled(modelEntity->isKindOf(CC_TYPES::MESH));
}
