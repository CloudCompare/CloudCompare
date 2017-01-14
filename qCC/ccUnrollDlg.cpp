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

#include "ccUnrollDlg.h"

//Qt
#include <QSettings>

ccUnrollDlg::ccUnrollDlg(QWidget* parent/*=0*/)
	: QDialog(parent)
	, Ui::UnrollDialog()
{
	setupUi(this);

	connect(comboBoxUnrollShapeType, SIGNAL(currentIndexChanged(int)), this, SLOT(shapeTypeChanged(int)));
	connect(checkBoxAuto,            SIGNAL(stateChanged(int)),        this, SLOT(axisAutoStateChanged(int)));
	connect(comboBoxAxisDimension,   SIGNAL(currentIndexChanged(int)), this, SLOT(axisDimensionChanged(int)));

	checkBoxAuto->setChecked(true);

	shapeTypeChanged(comboBoxUnrollShapeType->currentIndex());
	axisDimensionChanged(comboBoxAxisDimension->currentIndex());
}

ccUnrollDlg::Type ccUnrollDlg::getType() const
{
	return static_cast<Type>(comboBoxUnrollShapeType->currentIndex());
}

int ccUnrollDlg::getAxisDimension() const
{
	return comboBoxAxisDimension->currentIndex();
}
 
bool ccUnrollDlg::isAxisPositionAuto() const
{
	return (checkBoxAuto->checkState() == Qt::Checked);
}

CCVector3 ccUnrollDlg::getAxisPosition() const
{
	return CCVector3(	static_cast<PointCoordinateType>(doubleSpinBoxAxisX->value()),
						static_cast<PointCoordinateType>(doubleSpinBoxAxisY->value()),
						static_cast<PointCoordinateType>(doubleSpinBoxAxisZ->value()));
}

double ccUnrollDlg::getRadius() const
{
	return radiusDoubleSpinBox->value();
}

double ccUnrollDlg::getAngle() const
{
	return angleDoubleSpinBox->value();
}

bool ccUnrollDlg::exportDeviationSF() const
{
	return exportDeviationSFCheckBox->isChecked();
}

void ccUnrollDlg::shapeTypeChanged(int index)
{
	switch (index)
	{
	case CYLINDER: //cylinder
	{
		angleFrame->setVisible(false);
		autoCenterFrame->setVisible(true);
		radiusFrame->setVisible(true);
		groupBoxAxisPosition->setTitle("Axis position");
		radiusLabel->setText("Radius");
		axisAutoStateChanged(checkBoxAuto->checkState());
	}
	break;
	case CONE: //cone
	{
		angleFrame->setVisible(true);
		autoCenterFrame->setVisible(false);
		radiusFrame->setVisible(false);
		radiusLabel->setText("Base radius");
		groupBoxAxisPosition->setTitle("Cone apex");
		axisAutoStateChanged(Qt::Unchecked);
	}
	break;
	case STRAIGHTENED_CONE: //straightened cone
	{
		angleFrame->setVisible(true);
		radiusFrame->setVisible(true);
		autoCenterFrame->setVisible(false);
		groupBoxAxisPosition->setTitle("Cone apex");
		axisAutoStateChanged(Qt::Unchecked);
	}
	break;
	};
}

void ccUnrollDlg::axisAutoStateChanged(int checkState)
{
	if (checkState == Qt::Unchecked)
	{
		axisFrame->setEnabled(true);
		axisDimensionChanged(comboBoxAxisDimension->currentIndex());
	}
	else
	{
		axisFrame->setEnabled(false);
	}
}

void ccUnrollDlg::axisDimensionChanged(int index)
{
	//if axis is in auto mode, no need to change anything
	if (comboBoxUnrollShapeType->currentIndex() != 0 || checkBoxAuto->checkState() == Qt::Checked)
	{
		return;
	}

	//in 'cylinder' mode, we hide the axis coordinate that is not needed
	doubleSpinBoxAxisX->setDisabled(index == 0);
	doubleSpinBoxAxisY->setDisabled(index == 1);
	doubleSpinBoxAxisZ->setDisabled(index == 2);
}

//semi-persistent settings
static CCVector3d s_axisCenter(0, 0, 0);

void ccUnrollDlg::toPersistentSettings() const
{
	QSettings settings;
	settings.beginGroup("Unroll");
	{
		settings.setValue("shapeType", comboBoxUnrollShapeType->currentIndex());
		settings.setValue("axisDimension", comboBoxAxisDimension->currentIndex());
		settings.setValue("angle", angleDoubleSpinBox->value());
		settings.setValue("radius", radiusDoubleSpinBox->value());
		settings.setValue("autoCenter", checkBoxAuto->isChecked());
		settings.setValue("exportDeviationSF", exportDeviationSFCheckBox->isChecked());

		//save the axis center as semi-persistent only
		s_axisCenter.x = doubleSpinBoxAxisX->value();
		s_axisCenter.y = doubleSpinBoxAxisY->value();
		s_axisCenter.z = doubleSpinBoxAxisZ->value();
	}
	settings.endGroup();
}

void ccUnrollDlg::fromPersistentSettings()
{
	QSettings settings;
	settings.beginGroup("Unroll");
	{
		int shapeType   = settings.value("shapeType",     comboBoxUnrollShapeType->currentIndex()).toInt();
		int axisDim     = settings.value("axisDimension", comboBoxAxisDimension->currentIndex()).toInt();
		double angle    = settings.value("angle",         angleDoubleSpinBox->value()).toDouble();
		double radius   = settings.value("radius",        radiusDoubleSpinBox->value()).toDouble();
		bool autoCenter = settings.value("autoCenter",    checkBoxAuto->isChecked()).toBool();
		bool exportDeviationSF = settings.value("exportDeviationSF", exportDeviationSFCheckBox->isChecked()).toBool();

		comboBoxUnrollShapeType->setCurrentIndex(shapeType);
		comboBoxAxisDimension->setCurrentIndex(axisDim);
		angleDoubleSpinBox->setValue(angle);
		radiusDoubleSpinBox->setValue(radius);
		checkBoxAuto->setChecked(autoCenter);
		exportDeviationSFCheckBox->setChecked(exportDeviationSF);

		doubleSpinBoxAxisX->setValue(s_axisCenter.x);
		doubleSpinBoxAxisY->setValue(s_axisCenter.y);
		doubleSpinBoxAxisZ->setValue(s_axisCenter.z);
	}
	settings.endGroup();
}
