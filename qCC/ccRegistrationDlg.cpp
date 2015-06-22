//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccRegistrationDlg.h"

//Local
#include "ccDisplayOptionsDlg.h"
#include "mainwindow.h"

//CCLib
#include <DgmOctree.h>
#include <CloudSamplingTools.h>
#include <GeometricalAnalysisTools.h>
#include <ReferenceCloud.h>

//qCC_db
#include <ccHObject.h>

//system
#include <assert.h>

//semi-persistent options
static bool     s_adjustScale = false;
static unsigned s_randomSamplingLimit = 50000;
static double   s_rmsDifference = 1.0e-5;
static int      s_maxIterationCount = 20;
static bool     s_useErrorDifferenceCriterion = true;
static int      s_finalOverlap = 100;
static int      s_rotComboIndex = 0;
static bool     s_transCheckboxes[3] = {true, true, true};

ccRegistrationDlg::ccRegistrationDlg(ccHObject *data, ccHObject *model, QWidget* parent/*=0*/)
	: QDialog(parent)
	, Ui::RegistrationDialog()
{
	assert(data && model);
	dataEntity = data;
	modelEntity = model;

	setupUi(this);
	rmsDifferenceLineEdit->setValidator(new QDoubleValidator(rmsDifferenceLineEdit));
	setWindowFlags(Qt::Tool);

	setColorsAndLabels();

	QColor qRed(255,0,0);
	QColor qYellow(255,255,0);
	ccDisplayOptionsDlg::SetButtonColor(dataColorButton,qRed);
	ccDisplayOptionsDlg::SetButtonColor(modelColorButton,qYellow);

	//restore semi-persistent settings
	adjustScaleCheckBox->setChecked(s_adjustScale);
	randomSamplingLimitSpinBox->setValue(s_randomSamplingLimit);
	rmsDifferenceLineEdit->setText(QString::number(s_rmsDifference,'e',1));
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

	connect(swapButton, SIGNAL(clicked()), this, SLOT(swapModelAndData()));
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
}
