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

#include "ccGBLSensorProjectionDlg.h"

//local
#include "ccCustomDoubleValidator.h"

//qCC_db
#include <ccGBLSensor.h>

ccGBLSensorProjectionDlg::ccGBLSensorProjectionDlg(QWidget* parent)
	: QDialog(parent)
	, Ui::GBLSensorProjectDialog()
{
	setupUi(this);

	posXEdit->setValidator(new ccCustomDoubleValidator(this));
	posYEdit->setValidator(new ccCustomDoubleValidator(this));
	posZEdit->setValidator(new ccCustomDoubleValidator(this));

	x1rot->setValidator(new ccCustomDoubleValidator(this));
	x2rot->setValidator(new ccCustomDoubleValidator(this));
	x3rot->setValidator(new ccCustomDoubleValidator(this));
	y1rot->setValidator(new ccCustomDoubleValidator(this));
	y2rot->setValidator(new ccCustomDoubleValidator(this));
	y3rot->setValidator(new ccCustomDoubleValidator(this));
	z1rot->setValidator(new ccCustomDoubleValidator(this));
	z2rot->setValidator(new ccCustomDoubleValidator(this));
	z3rot->setValidator(new ccCustomDoubleValidator(this));
}

void ccGBLSensorProjectionDlg::initWithGBLSensor(const ccGBLSensor* sensor)
{
	if( !sensor)
		return;

	const int precision = sizeof(PointCoordinateType) == 8 ? 12 : 8;

	/***  Rotation order ***/
	{
		if (sensor->getRotationOrder() == ccGBLSensor::YAW_THEN_PITCH)
			rotationOrderComboBox->setCurrentIndex(0);
		else if (sensor->getRotationOrder() == ccGBLSensor::PITCH_THEN_YAW)
			rotationOrderComboBox->setCurrentIndex(1);
	}

	/*** Position + Orientation ***/
	{
		//rotation matrix
		const ccGLMatrix& rot = sensor->getRigidTransformation();
		{
			const float* mat = rot.data();
			x1rot->setText(QString::number(mat[0] ,'f',precision));
			y1rot->setText(QString::number(mat[1] ,'f',precision));
			z1rot->setText(QString::number(mat[2] ,'f',precision));

			x2rot->setText(QString::number(mat[4] ,'f',precision));
			y2rot->setText(QString::number(mat[5] ,'f',precision));
			z2rot->setText(QString::number(mat[6] ,'f',precision));

			x3rot->setText(QString::number(mat[8] ,'f',precision));
			y3rot->setText(QString::number(mat[9] ,'f',precision));
			z3rot->setText(QString::number(mat[10],'f',precision));
		}

		//center
		const float* C = sensor->getRigidTransformation().getTranslation();
		posXEdit->setText(QString::number(C[0],'f',precision));
		posYEdit->setText(QString::number(C[1],'f',precision));
		posZEdit->setText(QString::number(C[2],'f',precision));
	}

	/*** Angular steps ***/
	{
		//pitch step
		pitchStepSpinBox->setValue(sensor->getPitchStep() * CC_RAD_TO_DEG);
		//yaw step
		yawStepSpinBox->setValue(sensor->getYawStep() * CC_RAD_TO_DEG);
	}

	/*** Other ***/
	{
		//max range
		maxRangeDoubleSpinBox->setValue(sensor->getSensorRange());
		//uncertainty
		uncertaintyDoubleSpinBox->setValue(sensor->getUncertainty());
	}
}

void ccGBLSensorProjectionDlg::updateGBLSensor(ccGBLSensor* sensor)
{
	if (!sensor)
		return;

	/*** rotation order ***/
	{
		ccGBLSensor::ROTATION_ORDER rotOrder = (rotationOrderComboBox->currentIndex() == 0 ? ccGBLSensor::YAW_THEN_PITCH : ccGBLSensor::PITCH_THEN_YAW);
		sensor->setRotationOrder(rotOrder);
	}

	/*** Position + Orientation ***/
	{
		//orientation matrix
		ccGLMatrix rot;
		{
			float* mat = rot.data();
			mat[0]  = x1rot->text().toFloat();
			mat[1]  = y1rot->text().toFloat();
			mat[2]  = z1rot->text().toFloat();

			mat[4]  = x2rot->text().toFloat();
			mat[5]  = y2rot->text().toFloat();
			mat[6]  = z2rot->text().toFloat();

			mat[8]  = x3rot->text().toFloat();
			mat[9]  = y3rot->text().toFloat();
			mat[10] = z3rot->text().toFloat();
		}

		//center
		CCVector3 C(static_cast<PointCoordinateType>(posXEdit->text().toDouble()),
					static_cast<PointCoordinateType>(posYEdit->text().toDouble()),
					static_cast<PointCoordinateType>(posZEdit->text().toDouble()));
		rot.setTranslation(C);

		sensor->setRigidTransformation(rot);
	}

	/*** Angular steps ***/
	{
		//pitch step
		sensor->setPitchStep(static_cast<PointCoordinateType>(pitchStepSpinBox->value() * CC_DEG_TO_RAD));
		//yax step
		sensor->setYawStep(static_cast<PointCoordinateType>(yawStepSpinBox->value() * CC_DEG_TO_RAD));
	}

	/*** Other ***/
	{
		//max. range
		sensor->setSensorRange(static_cast<ScalarType>(maxRangeDoubleSpinBox->value()));

		//uncertainty
		sensor->setUncertainty(static_cast<ScalarType>(uncertaintyDoubleSpinBox->value()));
	}
}
