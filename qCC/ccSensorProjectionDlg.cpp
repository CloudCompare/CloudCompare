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
//$Rev:: 2101                                                              $
//$LastChangedDate:: 2012-05-03 18:19:18 +0200 (jeu., 03 mai 2012)         $
//**************************************************************************
//

#include "ccSensorProjectionDlg.h"

#include <QMessageBox>

#include <ccGBLSensor.h>

ccSensorProjectionDlg::ccSensorProjectionDlg(QWidget* parent) :
        QDialog(parent), Ui::SensorProjectDialog()
{
    setupUi(this);

    posXEdit->setValidator(new QDoubleValidator(this));
    posYEdit->setValidator(new QDoubleValidator(this));
    posZEdit->setValidator(new QDoubleValidator(this));

    x1rot->setValidator(new QDoubleValidator(this));
    x2rot->setValidator(new QDoubleValidator(this));
    x3rot->setValidator(new QDoubleValidator(this));
    y1rot->setValidator(new QDoubleValidator(this));
    y2rot->setValidator(new QDoubleValidator(this));
    y3rot->setValidator(new QDoubleValidator(this));
    z1rot->setValidator(new QDoubleValidator(this));
    z2rot->setValidator(new QDoubleValidator(this));
    z3rot->setValidator(new QDoubleValidator(this));

    //DGM: not sure that's such a good idea...
    //connect(rotationOrderComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(rotationOrderChanged(int)));

    connect(axisMatCheckBox, SIGNAL(stateChanged(int)), this, SLOT(axisMatrixStateChanged(int)));
}

void ccSensorProjectionDlg::rotationOrderChanged(int index)
{
    if (index==1)
    {
        if (baseSpinBox->value()==0)
            baseSpinBox->setValue(500.0);
    }
}

void ccSensorProjectionDlg::axisMatrixStateChanged(int checkState)
{
    bool state = (checkState == Qt::Unchecked);
    x1rot->setEnabled(state);
    x2rot->setEnabled(state);
    x3rot->setEnabled(state);
    y1rot->setEnabled(state);
    y2rot->setEnabled(state);
    y3rot->setEnabled(state);
    z1rot->setEnabled(state);
    z2rot->setEnabled(state);
    z3rot->setEnabled(state);
}

bool ccSensorProjectionDlg::isAxisMatIdentity()
{
    return (axisMatCheckBox->checkState()==Qt::Checked);
}

void ccSensorProjectionDlg::initWithGBLSensor(ccGBLSensor* sensor)
{
    if( !sensor)
        return;

    //center
    CCVector3 C = sensor->getSensorCenter();
    posXEdit->setText(QString("%1").arg(C.x));
    posYEdit->setText(QString("%1").arg(C.y));
    posZEdit->setText(QString("%1").arg(C.z));

    //rotation order
    if (sensor->getRotationOrder() == CCLib::GBL_THETA_PHI)
        rotationOrderComboBox->setCurrentIndex(0);
    else if (sensor->getRotationOrder() == CCLib::GBL_PHI_THETA)
        rotationOrderComboBox->setCurrentIndex(1);

    //base
    baseSpinBox->setValue(sensor->getSensorBase());

    //max range
    maxRangeDoubleSpinBox->setValue(sensor->getSensorRange());

    //uncertainty
    uncertaintyDoubleSpinBox->setValue(sensor->getUncertainty());

    //rotation matrix
    CCLib::SquareMatrix* A = sensor->getAxisMatrix();
    if (A)
    {
        axisMatCheckBox->setCheckState(Qt::Unchecked);
        x1rot->setText(QString("%1").arg(A->m_values[0][0]));
        x2rot->setText(QString("%1").arg(A->m_values[1][0]));
        x3rot->setText(QString("%1").arg(A->m_values[2][0]));
        y1rot->setText(QString("%1").arg(A->m_values[0][1]));
        y2rot->setText(QString("%1").arg(A->m_values[1][1]));
        y3rot->setText(QString("%1").arg(A->m_values[2][1]));
        z1rot->setText(QString("%1").arg(A->m_values[0][2]));
        z2rot->setText(QString("%1").arg(A->m_values[1][2]));
        z3rot->setText(QString("%1").arg(A->m_values[2][2]));
    }
    else
    {
        axisMatCheckBox->setCheckState(Qt::Checked);
        x1rot->setText(QString("1.0"));
        x2rot->setText(QString("0.0"));
        x3rot->setText(QString("0.0"));
        y1rot->setText(QString("0.0"));
        y2rot->setText(QString("1.0"));
        y3rot->setText(QString("0.0"));
        z1rot->setText(QString("0.0"));
        z2rot->setText(QString("0.0"));
        z3rot->setText(QString("1.0"));
    }

    //angular steps
    dPhiSpinBox->setValue(sensor->getDeltaPhi());
    dThetaSpinBox->setValue(sensor->getDeltaTheta());
}

void ccSensorProjectionDlg::updateGBLSensor(ccGBLSensor* sensor)
{
    if (!sensor)
        return;

    //rotation order
    CCLib::CC_SENSOR_ROTATION_ORDER rotOrder =
        (rotationOrderComboBox->currentIndex() == 0 ?
         CCLib::GBL_THETA_PHI : CCLib::GBL_PHI_THETA);
    sensor->setRotationOrder(rotOrder);

    //center
    CCVector3 C(posXEdit->text().toDouble(),
                posYEdit->text().toDouble(),
                posZEdit->text().toDouble());
    sensor->setSensorCenter(C);

    //base
    sensor->setSensorBase(baseSpinBox->value());

    //max. range
    sensor->setSensorRange(maxRangeDoubleSpinBox->value());

    //axis matrix
    CCLib::SquareMatrix* A = 0;
    if (!isAxisMatIdentity())
    {
        A = new CCLib::SquareMatrix(3);
        A->m_values[0][0] = x1rot->text().toDouble();
        A->m_values[1][0] = x2rot->text().toDouble();
        A->m_values[2][0] = x3rot->text().toDouble();
        A->m_values[0][1] = y1rot->text().toDouble();
        A->m_values[1][1] = y2rot->text().toDouble();
        A->m_values[2][1] = y3rot->text().toDouble();
        A->m_values[0][2] = z1rot->text().toDouble();
        A->m_values[1][2] = z2rot->text().toDouble();
        A->m_values[2][2] = z3rot->text().toDouble();
    }
    sensor->setAxisMatrix(A);

    //angular steps
    sensor->setDeltaPhi(dPhiSpinBox->value());
    sensor->setDeltaTheta(dThetaSpinBox->value());

    //uncertainty
    sensor->setUncertainty(uncertaintyDoubleSpinBox->value());
}
