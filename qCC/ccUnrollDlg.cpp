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
#include "ui_unrollDlg.h"

//Qt
#include <QSettings>

ccUnrollDlg::ccUnrollDlg(QWidget* parent/*=0*/)
	: QDialog(parent)
	, m_ui( new Ui::UnrollDialog )
{
	m_ui->setupUi(this);

	connect(m_ui->checkBoxAuto, &QCheckBox::stateChanged, this, &ccUnrollDlg::axisAutoStateChanged);
	connect(m_ui->comboBoxUnrollShapeType, qOverload<int>(&QComboBox::currentIndexChanged), this, &ccUnrollDlg::shapeTypeChanged);
	connect(m_ui->comboBoxAxisDimension, qOverload<int>(&QComboBox::currentIndexChanged), this, &ccUnrollDlg::axisDimensionChanged);

	m_ui->checkBoxAuto->setChecked(true);

	shapeTypeChanged(m_ui->comboBoxUnrollShapeType->currentIndex());
	axisDimensionChanged(m_ui->comboBoxAxisDimension->currentIndex());
}

ccUnrollDlg::~ccUnrollDlg()
{
	delete m_ui;
}

ccPointCloud::UnrollMode ccUnrollDlg::getType() const
{
	return static_cast<ccPointCloud::UnrollMode>(m_ui->comboBoxUnrollShapeType->currentIndex());
}

int ccUnrollDlg::getAxisDimension() const
{
	return m_ui->comboBoxAxisDimension->currentIndex();
}
 
bool ccUnrollDlg::isAxisPositionAuto() const
{
	return (m_ui->checkBoxAuto->checkState() == Qt::Checked);
}

void ccUnrollDlg::getAngleRange(double& start_deg, double& stop_deg) const
{
	start_deg = m_ui->startAngleDoubleSpinBox->value();
	stop_deg = m_ui->stopAngleDoubleSpinBox->value();
}

CCVector3 ccUnrollDlg::getAxisPosition() const
{
	return CCVector3(	static_cast<PointCoordinateType>(m_ui->doubleSpinBoxAxisX->value()),
						static_cast<PointCoordinateType>(m_ui->doubleSpinBoxAxisY->value()),
						static_cast<PointCoordinateType>(m_ui->doubleSpinBoxAxisZ->value()));
}

double ccUnrollDlg::getRadius() const
{
	return m_ui->radiusDoubleSpinBox->value();
}

double ccUnrollDlg::getConeHalfAngle() const
{
	return m_ui->halfAngleDoubleSpinBox->value();
}

bool ccUnrollDlg::exportDeviationSF() const
{
	return m_ui->exportDeviationSFCheckBox->isChecked();
}

void ccUnrollDlg::shapeTypeChanged(int index)
{
	switch (index)
	{
	case ccPointCloud::CYLINDER: //cylinder
	{
		m_ui->angleFrame->setVisible(false);
		m_ui->autoCenterFrame->setVisible(true);
		m_ui->radiusFrame->setVisible(true);
		m_ui->groupBoxAxisPosition->setTitle("Axis position");
		m_ui->radiusLabel->setText("Radius");
		axisAutoStateChanged(m_ui->checkBoxAuto->checkState());
	}
	break;
	case ccPointCloud::CONE: //cone
	{
		m_ui->angleFrame->setVisible(true);
		m_ui->autoCenterFrame->setVisible(false);
		m_ui->radiusFrame->setVisible(false);
		m_ui->radiusLabel->setText("Base radius");
		m_ui->groupBoxAxisPosition->setTitle("Cone apex");
		axisAutoStateChanged(Qt::Unchecked);
		//may be disabled if we were in cylinder mode previously
		m_ui->doubleSpinBoxAxisX->setDisabled(false);
		m_ui->doubleSpinBoxAxisY->setDisabled(false);
		m_ui->doubleSpinBoxAxisZ->setDisabled(false);
	}
	break;
	case ccPointCloud::STRAIGHTENED_CONE: //straightened cone (fixed radius)
	case ccPointCloud::STRAIGHTENED_CONE2: //straightened cone 2
	{
		m_ui->angleFrame->setVisible(true);
		m_ui->radiusFrame->setVisible(true);
		m_ui->autoCenterFrame->setVisible(false);
		m_ui->groupBoxAxisPosition->setTitle("Cone apex");
		axisAutoStateChanged(Qt::Unchecked);
		//may be disabled if we were in cylinder mode previously
		m_ui->doubleSpinBoxAxisX->setDisabled(false);
		m_ui->doubleSpinBoxAxisY->setDisabled(false);
		m_ui->doubleSpinBoxAxisZ->setDisabled(false);
	}
	break;
	};
}

void ccUnrollDlg::axisAutoStateChanged(int checkState)
{
	if (checkState == Qt::Unchecked)
	{
		m_ui->axisFrame->setEnabled(true);
		axisDimensionChanged(m_ui->comboBoxAxisDimension->currentIndex());
	}
	else
	{
		m_ui->axisFrame->setEnabled(false);
	}
}

void ccUnrollDlg::axisDimensionChanged(int index)
{
	//if axis is in auto mode, no need to change anything
	if ( (m_ui->comboBoxUnrollShapeType->currentIndex() != 0)
		 || (m_ui->checkBoxAuto->checkState() == Qt::Checked) )
	{
		return;
	}

	//in 'cylinder' mode, we hide the axis coordinate that is not needed
	m_ui->doubleSpinBoxAxisX->setDisabled(index == 0);
	m_ui->doubleSpinBoxAxisY->setDisabled(index == 1);
	m_ui->doubleSpinBoxAxisZ->setDisabled(index == 2);
}

//semi-persistent settings
static CCVector3d s_axisCenter(0, 0, 0);
static double s_startAngle_deg = 0.0;
static double s_stopAngle_deg = 360.0;

void ccUnrollDlg::toPersistentSettings() const
{
	QSettings settings;
	settings.beginGroup("Unroll");
	{
		settings.setValue("shapeType",			m_ui->comboBoxUnrollShapeType->currentIndex());
		settings.setValue("axisDimension",		m_ui->comboBoxAxisDimension->currentIndex());
		settings.setValue("angle",				m_ui->halfAngleDoubleSpinBox->value());
		settings.setValue("radius",				m_ui->radiusDoubleSpinBox->value());
		settings.setValue("autoCenter",			m_ui->checkBoxAuto->isChecked());
		settings.setValue("exportDeviationSF",	m_ui->exportDeviationSFCheckBox->isChecked());

		//save the axis center as semi-persistent only
		s_axisCenter.x = m_ui->doubleSpinBoxAxisX->value();
		s_axisCenter.y = m_ui->doubleSpinBoxAxisY->value();
		s_axisCenter.z = m_ui->doubleSpinBoxAxisZ->value();

		getAngleRange(s_startAngle_deg, s_stopAngle_deg);
	}
	settings.endGroup();
}

void ccUnrollDlg::fromPersistentSettings()
{
	QSettings settings;
	settings.beginGroup("Unroll");
	{
		int shapeType   = settings.value("shapeType",     m_ui->comboBoxUnrollShapeType->currentIndex()).toInt();
		int axisDim     = settings.value("axisDimension", m_ui->comboBoxAxisDimension->currentIndex()).toInt();
		double angle    = settings.value("angle",         m_ui->halfAngleDoubleSpinBox->value()).toDouble();
		double radius   = settings.value("radius",        m_ui->radiusDoubleSpinBox->value()).toDouble();
		bool autoCenter = settings.value("autoCenter",    m_ui->checkBoxAuto->isChecked()).toBool();
		bool exportDeviationSF = settings.value("exportDeviationSF", m_ui->exportDeviationSFCheckBox->isChecked()).toBool();

		m_ui->comboBoxUnrollShapeType->setCurrentIndex(shapeType);
		m_ui->comboBoxAxisDimension->setCurrentIndex(axisDim);
		m_ui->halfAngleDoubleSpinBox->setValue(angle);
		m_ui->radiusDoubleSpinBox->setValue(radius);
		m_ui->checkBoxAuto->setChecked(autoCenter);
		m_ui->exportDeviationSFCheckBox->setChecked(exportDeviationSF);

		m_ui->doubleSpinBoxAxisX->setValue(s_axisCenter.x);
		m_ui->doubleSpinBoxAxisY->setValue(s_axisCenter.y);
		m_ui->doubleSpinBoxAxisZ->setValue(s_axisCenter.z);

		m_ui->startAngleDoubleSpinBox->setValue(s_startAngle_deg);
		m_ui->stopAngleDoubleSpinBox ->setValue(s_stopAngle_deg);
	}
	settings.endGroup();
}
