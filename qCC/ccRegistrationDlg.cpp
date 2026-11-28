// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#include "ccRegistrationDlg.h"

// Local
#include "mainwindow.h"

// CCCoreLib
#include <CloudSamplingTools.h>
#include <DgmOctree.h>
#include <GeometricalAnalysisTools.h>
#include <ReferenceCloud.h>

// CCPluginAPI
#include <ccQtHelpers.h>

// qCC_db
#include <ccHObject.h>

// Qt
#include <QThread>

// System
#include <cassert>

static bool     s_adjustScale                 = false;
static bool     s_constrainScale              = false;
static double   s_minScale                    = 0.5;
static double   s_maxScale                    = 2.0;
static unsigned s_randomSamplingLimit         = 50000;
static double   s_rmsDifference               = 1.0e-5;
static int      s_maxIterationCount           = 20;
static bool     s_useErrorDifferenceCriterion = true;
static int      s_finalOverlap                = 100;
static int      s_rotComboIndex               = 0;
static bool     s_transCheckboxes[3]          = {true, true, true};
static int      s_maxThreadCount              = ccQtHelpers::GetMaxThreadCount();
static bool     s_pointsRemoval               = false;
static bool     s_useDataSFAsWeights          = false;
static bool     s_useModelSFAsWeights         = false;
static bool     s_useC2MSignedDistances       = false;
static bool     s_robustC2MSignedDistances    = true;
static int      s_normalsMatchingOption       = CCCoreLib::ICPRegistrationTools::NO_NORMAL;
static int      s_errorMetric                 = CCCoreLib::ICPRegistrationTools::POINT_TO_POINT;
static int      s_robustWeighting             = CCCoreLib::ICPRegistrationTools::NO_ROBUST_WEIGHTING;

ccRegistrationDlg::ccRegistrationDlg(ccHObject* data, ccHObject* model, QWidget* parent /*=nullptr*/)
    : QDialog(parent, Qt::Tool)
    , Ui::RegistrationDialog()
    , m_preferredErrorMetric(s_errorMetric)
{
	assert(data && model);
	dataEntity  = data;
	modelEntity = model;

	setupUi(this);

	m_errorMetricToolTip = errorMetricComboBox->toolTip();

	QDoubleValidator* rmsValidator = new QDoubleValidator(rmsDifferenceLineEdit);
	rmsValidator->setNotation(QDoubleValidator::ScientificNotation);
	rmsValidator->setRange(GetAbsoluteMinRMSDecrease(), 1.0, 1);
	rmsDifferenceLineEdit->setValidator(rmsValidator);

	updateGUI();

	ccQtHelpers::SetButtonColor(dataColorButton, Qt::red);
	ccQtHelpers::SetButtonColor(modelColorButton, Qt::yellow);

	static const int MaxThreadCount = QThread::idealThreadCount();
	maxThreadCountSpinBox->setRange(1, MaxThreadCount);
	maxThreadCountSpinBox->setSuffix(QString(" / %1").arg(MaxThreadCount));

	// restore semi-persistent settings
	{
		// semi-persistent options
		maxThreadCountSpinBox->setValue(s_maxThreadCount);
		adjustScaleGroupBox->setChecked(s_adjustScale);
		scaleRangeCheckBox->setChecked(s_constrainScale);
		minScaleDoubleSpinBox->setValue(s_minScale);
		maxScaleDoubleSpinBox->setValue(s_maxScale);
		randomSamplingLimitSpinBox->setValue(s_randomSamplingLimit);
		setMinRMSDecrease(s_rmsDifference);
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
		pointsRemoval->setChecked(s_pointsRemoval);
		checkBoxUseDataSFAsWeights->setChecked(s_useDataSFAsWeights);
		checkBoxUseModelSFAsWeights->setChecked(s_useModelSFAsWeights);
		useC2MSignedDistancesCheckBox->setChecked(s_useC2MSignedDistances);
		robustC2MDistsCheckBox->setChecked(s_robustC2MSignedDistances);
		normalsComboBox->setCurrentIndex(s_normalsMatchingOption);
		robustWeightingComboBox->setCurrentIndex(s_robustWeighting);
	}

	connect(swapButton, &QAbstractButton::clicked, this, &ccRegistrationDlg::swapModelAndData);
	connect(errorMetricComboBox, qOverload<int>(&QComboBox::currentIndexChanged), this, &ccRegistrationDlg::updateScaleState);
	// only the user's choices are remembered (not the forced point-to-point metric)
	connect(errorMetricComboBox, qOverload<int>(&QComboBox::activated), this, [this](int index)
	        { m_preferredErrorMetric = index; });
	updateErrorMetricState();
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
	s_maxThreadCount              = getMaxThreadCount();
	s_adjustScale                 = adjustScale();
	s_constrainScale              = scaleRangeCheckBox->isChecked();
	s_minScale                    = minScaleDoubleSpinBox->value();
	s_maxScale                    = maxScaleDoubleSpinBox->value();
	s_randomSamplingLimit         = randomSamplingLimit();
	s_rmsDifference               = getMinRMSDecrease();
	s_maxIterationCount           = getMaxIterationCount();
	s_useErrorDifferenceCriterion = errorCriterion->isChecked();
	s_finalOverlap                = overlapSpinBox->value();
	s_rotComboIndex               = rotComboBox->currentIndex();
	s_transCheckboxes[0]          = TxCheckBox->isChecked();
	s_transCheckboxes[1]          = TyCheckBox->isChecked();
	s_transCheckboxes[2]          = TzCheckBox->isChecked();
	s_pointsRemoval               = removeFarthestPoints();
	s_useDataSFAsWeights          = checkBoxUseDataSFAsWeights->isChecked();
	s_useModelSFAsWeights         = checkBoxUseModelSFAsWeights->isChecked();
	s_useC2MSignedDistances       = useC2MSignedDistancesCheckBox->isChecked();
	s_robustC2MSignedDistances    = robustC2MDistsCheckBox->isChecked();
	s_normalsMatchingOption       = normalsComboBox->currentIndex();
	s_errorMetric                 = m_preferredErrorMetric;
	s_robustWeighting             = robustWeightingComboBox->currentIndex();
}

ccHObject* ccRegistrationDlg::getDataEntity()
{
	return dataEntity;
}

ccHObject* ccRegistrationDlg::getModelEntity()
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

bool ccRegistrationDlg::useC2MSignedDistances(bool& robust) const
{
	robust = robustC2MDistsCheckBox->isChecked();

	return useC2MSignedDistancesCheckBox->isEnabled() && useC2MSignedDistancesCheckBox->isChecked();
}

CCCoreLib::ICPRegistrationTools::NORMALS_MATCHING ccRegistrationDlg::normalsMatchingOption() const
{
	if (normalsComboBox->isEnabled())
	{
		return static_cast<CCCoreLib::ICPRegistrationTools::NORMALS_MATCHING>(normalsComboBox->currentIndex());
	}
	return CCCoreLib::ICPRegistrationTools::NO_NORMAL;
}

CCCoreLib::ICPRegistrationTools::ERROR_METRIC ccRegistrationDlg::errorMetric() const
{
	if (errorMetricComboBox->isEnabled() && errorMetricComboBox->currentIndex() == 1)
	{
		return CCCoreLib::ICPRegistrationTools::POINT_TO_PLANE;
	}
	return CCCoreLib::ICPRegistrationTools::POINT_TO_POINT;
}

CCCoreLib::ICPRegistrationTools::ROBUST_WEIGHTING ccRegistrationDlg::robustWeighting() const
{
	return static_cast<CCCoreLib::ICPRegistrationTools::ROBUST_WEIGHTING>(robustWeightingComboBox->currentIndex());
}

bool ccRegistrationDlg::adjustScale() const
{
	return adjustScaleGroupBox->isEnabled() && adjustScaleGroupBox->isChecked();
}

void ccRegistrationDlg::updateScaleState()
{
	// the point-to-plane metric can't adjust the scale
	adjustScaleGroupBox->setEnabled(errorMetric() != CCCoreLib::ICPRegistrationTools::POINT_TO_PLANE);
}

std::pair<double, double> ccRegistrationDlg::constrainedScaleRange() const
{
	if (scaleRangeCheckBox->isChecked())
	{
		return {minScaleDoubleSpinBox->value(), maxScaleDoubleSpinBox->value()};
	}

	return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
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
	return static_cast<unsigned>(std::max(1, maxIterationCount->value()));
}

unsigned ccRegistrationDlg::getFinalOverlap() const
{
	return static_cast<unsigned>(std::max(10, overlapSpinBox->value()));
}

int ccRegistrationDlg::getMaxThreadCount() const
{
	return maxThreadCountSpinBox->value();
}

double ccRegistrationDlg::GetAbsoluteMinRMSDecrease()
{
	return 1.0e-7;
}

double ccRegistrationDlg::getMinRMSDecrease() const
{
	bool   ok  = true;
	double val = rmsDifferenceLineEdit->text().toDouble(&ok);

	if (!ok)
	{
		assert(false);
		val = std::numeric_limits<double>::quiet_NaN();
	}

	return val;
}

void ccRegistrationDlg::setMinRMSDecrease(double value)
{
	if (std::isnan(value))
	{
		// last input value was invalid, restoring default
		value = 1.0e-5;
	}
	rmsDifferenceLineEdit->setText(QString::number(value, 'E', 1));
}

ccRegistrationDlg::ConvergenceMethod ccRegistrationDlg::getConvergenceMethod() const
{
	if (errorCriterion->isChecked())
		return CCCoreLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE;
	return CCCoreLib::ICPRegistrationTools::MAX_ITER_CONVERGENCE;
}

int ccRegistrationDlg::getTransformationFilters() const
{
	int filters = CCCoreLib::RegistrationTools::SKIP_NONE;
	switch (rotComboBox->currentIndex())
	{
	case 0:
		break;
	case 1:
		filters |= CCCoreLib::RegistrationTools::SKIP_RYZ;
		break;
	case 2:
		filters |= CCCoreLib::RegistrationTools::SKIP_RXZ;
		break;
	case 3:
		filters |= CCCoreLib::RegistrationTools::SKIP_RXY;
		break;
	case 4:
		filters |= CCCoreLib::RegistrationTools::SKIP_ROTATION;
		break;
	default:
		assert(false);
		break;
	}

	if (!TxCheckBox->isChecked())
		filters |= CCCoreLib::RegistrationTools::SKIP_TX;
	if (!TyCheckBox->isChecked())
		filters |= CCCoreLib::RegistrationTools::SKIP_TY;
	if (!TzCheckBox->isChecked())
		filters |= CCCoreLib::RegistrationTools::SKIP_TZ;

	return filters;
}

void ccRegistrationDlg::updateGUI()
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
	checkBoxUseModelSFAsWeights->setEnabled(modelEntity->isKindOf(CC_TYPES::POINT_CLOUD) && modelEntity->hasDisplayedScalarField()); // only supported for clouds

	bool hasRefMesh = modelEntity->isKindOf(CC_TYPES::MESH);
	useC2MSignedDistancesCheckBox->setEnabled(hasRefMesh); // only supported if a mesh is the reference cloud
	robustC2MDistsCheckBox->setEnabled(hasRefMesh);
	normalsComboBox->setEnabled(dataEntity->hasNormals() && modelEntity->hasNormals()); // only supported if both the to-be-aligned and the reference entities have normals

	updateErrorMetricState();

	MainWindow::RefreshAllGLWindow(false);
}

void ccRegistrationDlg::updateErrorMetricState()
{
	// the point-to-plane metric needs the normals of the reference (or its triangles)
	bool canUsePointToPlane = (modelEntity->isKindOf(CC_TYPES::MESH) || modelEntity->hasNormals());

	// always show the metric that will actually be used
	errorMetricComboBox->setEnabled(canUsePointToPlane);
	errorMetricComboBox->setCurrentIndex(canUsePointToPlane ? m_preferredErrorMetric : CCCoreLib::ICPRegistrationTools::POINT_TO_POINT);
	errorMetricComboBox->setToolTip(canUsePointToPlane ? m_errorMetricToolTip
	                                                   : tr("The point-to-plane metric needs normals on the reference cloud (Edit > Normals > Compute), or a reference mesh"));

	updateScaleState();
}

void ccRegistrationDlg::swapModelAndData()
{
	std::swap(dataEntity, modelEntity);

	updateGUI();
}
