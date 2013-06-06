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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2241                                                              $
//$LastChangedDate:: 2012-09-21 23:22:39 +0200 (ven., 21 sept. 2012)       $
//**************************************************************************
//

#include "ccRegistrationDlg.h"

#include <DgmOctree.h>
#include <CloudSamplingTools.h>
#include <GeometricalAnalysisTools.h>
#include <ReferenceCloud.h>

#include "ccDisplayOptionsDlg.h"
#include "mainwindow.h"
#include <ccHObject.h>

#include <assert.h>

//semi-persistent options
static bool s_freeScaleParameter = false;
static unsigned s_randomSamplingLimit = 20000;
static double s_errorDifference = 1.0e-6;

ccRegistrationDlg::ccRegistrationDlg(ccHObject *data, ccHObject *model, QWidget* parent/*=0*/)
    : QDialog(parent), Ui::RegistrationDialog()
{
    assert(data && model);
    dataEntity = data;
    modelEntity = model;

    setupUi(this);
    errorDifferenceLineEdit->setValidator(new QDoubleValidator(errorDifferenceLineEdit));
    setWindowFlags(Qt::Tool);

    setColorsAndLabels();

    QColor qRed(255,0,0);
    QColor qYellow(255,255,0);
    ccDisplayOptionsDlg::SetButtonColor(dataColorButton,qRed);
    ccDisplayOptionsDlg::SetButtonColor(modelColorButton,qYellow);

	//restore semi-persistent settings
	checkBoxFreeScale->setChecked(s_freeScaleParameter);
	randomSamplingLimitSpinBox->setValue(s_randomSamplingLimit);
	errorDifferenceLineEdit->setText(QString::number(s_errorDifference,'e',3));

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

	MainWindow::RefreshAllGLWindow();
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
    return checkBoxUseDataSFAsWeights->isChecked();
}

bool ccRegistrationDlg::useModelSFAsWeights() const
{
    return checkBoxUseModelSFAsWeights->isChecked();
}

bool ccRegistrationDlg::useFreeScaleParameter() const
{
	//we save the parameter by the way ;)
    s_freeScaleParameter = checkBoxFreeScale->isChecked();
	return s_freeScaleParameter;
}

bool ccRegistrationDlg::removeFarthestPoints() const
{
    return pointsRemoval->isChecked();
}

unsigned ccRegistrationDlg::randomSamplingLimit() const
{
	//we save the parameter by the way ;)
	s_randomSamplingLimit = randomSamplingLimitSpinBox->value();
	return s_randomSamplingLimit;
}

unsigned ccRegistrationDlg::getMaxIterationCount() const
{
    return maxIterationCount->value();
}

double ccRegistrationDlg::getMinErrorDecrease() const
{
    bool ok = true;
    double val = errorDifferenceLineEdit->text().toDouble(&ok);
    assert(ok);

	//we save the parameter by the way ;)
	if (ok)
		s_errorDifference = val;

    return val;
}

ConvergenceMethod ccRegistrationDlg::getConvergenceMethod() const
{
    if(errorCriterion->isChecked())
        return CCLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE;
    else
        return CCLib::ICPRegistrationTools::MAX_ITER_CONVERGENCE;
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

    MainWindow::RefreshAllGLWindow();
}

void ccRegistrationDlg::swapModelAndData()
{
    std::swap(dataEntity,modelEntity);
    setColorsAndLabels();
}
