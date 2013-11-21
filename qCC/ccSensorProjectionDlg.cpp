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

ccSensorProjectionDlg::ccSensorProjectionDlg(QWidget* parent)
	: QDialog(parent)
	, Ui::SensorProjectDialog()
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
}

void ccSensorProjectionDlg::initWithGBLSensor(const ccGBLSensor* sensor)
{
    if( !sensor)
        return;

    //center
    CCVector3 C = sensor->getSensorCenter();
    posXEdit->setText(QString("%1").arg(C.x));
    posYEdit->setText(QString("%1").arg(C.y));
    posZEdit->setText(QString("%1").arg(C.z));

    //rotation order
    if (sensor->getRotationOrder() == ccGBLSensor::THETA_PHI)
        rotationOrderComboBox->setCurrentIndex(0);
    else if (sensor->getRotationOrder() == ccGBLSensor::PHI_THETA)
        rotationOrderComboBox->setCurrentIndex(1);

    //base
    baseSpinBox->setValue(sensor->getSensorBase());

    //max range
    maxRangeDoubleSpinBox->setValue(sensor->getSensorRange());

    //uncertainty
    uncertaintyDoubleSpinBox->setValue(sensor->getUncertainty());

    //rotation matrix
	const ccGLMatrix& rot = sensor->getOrientationMatrix();
    {
		const float* mat = rot.data();
        x1rot->setText(QString::number(mat[0]));
        x2rot->setText(QString::number(mat[3]));
        x3rot->setText(QString::number(mat[8]));
        y1rot->setText(QString::number(mat[1]));
        y2rot->setText(QString::number(mat[5]));
        y3rot->setText(QString::number(mat[9]));
        z1rot->setText(QString::number(mat[2]));
        z2rot->setText(QString::number(mat[6]));
        z3rot->setText(QString::number(mat[10]));
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
	ccGBLSensor::ROTATION_ORDER rotOrder = (rotationOrderComboBox->currentIndex() == 0 ? ccGBLSensor::THETA_PHI : ccGBLSensor::PHI_THETA);
    sensor->setRotationOrder(rotOrder);

    //center
    CCVector3 C(static_cast<PointCoordinateType>(posXEdit->text().toDouble()),
                static_cast<PointCoordinateType>(posYEdit->text().toDouble()),
                static_cast<PointCoordinateType>(posZEdit->text().toDouble()));
    sensor->setSensorCenter(C);

    //base
    sensor->setSensorBase(static_cast<PointCoordinateType>(baseSpinBox->value()));

    //max. range
    sensor->setSensorRange(static_cast<ScalarType>(maxRangeDoubleSpinBox->value()));

    //orientation matrix
    ccGLMatrix rot;
    {
		float* mat = rot.data();
        mat[0]  = x1rot->text().toFloat();
        mat[4]  = x2rot->text().toFloat();
        mat[8]  = x3rot->text().toFloat();
        mat[1]  = y1rot->text().toFloat();
        mat[5]  = y2rot->text().toFloat();
        mat[9]  = y3rot->text().toFloat();
        mat[2]  = z1rot->text().toFloat();
        mat[6]  = z2rot->text().toFloat();
        mat[10] = z3rot->text().toFloat();
    }
	sensor->setOrientationMatrix(rot);

    //angular steps
    sensor->setDeltaPhi(static_cast<PointCoordinateType>(dPhiSpinBox->value()));
    sensor->setDeltaTheta(static_cast<PointCoordinateType>(dThetaSpinBox->value()));

    //uncertainty
    sensor->setUncertainty(static_cast<ScalarType>(uncertaintyDoubleSpinBox->value()));
}
