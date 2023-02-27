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

#include "ccEntitySelectionDlg.h"
#include "ccUtils.h"

//qCC_db
#include <ccCylinder.h>

//Qt
#include <QSettings>

//semi-persistent settings
static double s_startAngle_deg = 0.0;
static double s_stopAngle_deg = 360.0;
static bool s_arbitraryOutputCS = false;

ccUnrollDlg::ccUnrollDlg(ccHObject* dbRootEntity, QWidget* parent/*=nullptr*/)
	: QDialog(parent)
	, m_ui( new Ui::UnrollDialog )
	, m_dbRootEntity(dbRootEntity)
{
	m_ui->setupUi(this);

	connect(m_ui->checkBoxAuto, &QCheckBox::stateChanged, this, &ccUnrollDlg::axisAutoStateChanged);
	connect(m_ui->comboBoxUnrollShapeType, qOverload<int>(&QComboBox::currentIndexChanged), this, &ccUnrollDlg::shapeTypeChanged);
	connect(m_ui->comboBoxAxisDimension, qOverload<int>(&QComboBox::currentIndexChanged), this, &ccUnrollDlg::axisDimensionChanged);
	connect(m_ui->flipxAxisToolButton, &QToolButton::clicked, [this] {	m_ui->axisXDoubleSpinBox->setValue(-m_ui->axisXDoubleSpinBox->value());
																		m_ui->axisYDoubleSpinBox->setValue(-m_ui->axisYDoubleSpinBox->value());
																		m_ui->axisZDoubleSpinBox->setValue(-m_ui->axisZDoubleSpinBox->value());
		});

	connect(m_ui->fromEntityToolButton, &QToolButton::clicked, this, &ccUnrollDlg::loadParametersFromEntity);
	connect(m_ui->pasteAxisToolButton, &QToolButton::clicked, this, &ccUnrollDlg::axisFromClipboard);
	connect(m_ui->pasteCenterToolButton, &QToolButton::clicked, this, &ccUnrollDlg::centerFromClipboard);

	m_ui->checkBoxAuto->setChecked(true);

	shapeTypeChanged(m_ui->comboBoxUnrollShapeType->currentIndex());
	axisDimensionChanged(m_ui->comboBoxAxisDimension->currentIndex());

	if (!m_dbRootEntity)
	{
		assert(false);
		m_ui->fromEntityToolButton->setVisible(false);
	}
}

ccUnrollDlg::~ccUnrollDlg()
{
	delete m_ui;
}

ccPointCloud::UnrollMode ccUnrollDlg::getType() const
{
	return static_cast<ccPointCloud::UnrollMode>(m_ui->comboBoxUnrollShapeType->currentIndex());
}

CCVector3d ccUnrollDlg::getAxis() const
{
	int axisDim = m_ui->comboBoxAxisDimension->currentIndex();
	switch (axisDim)
	{
	case 0:
		return CCVector3d(1.0, 0.0, 0.0);
		break;
	case 1:
		return CCVector3d(0.0, 1.0, 0.0);
		break;
	case 2:
		return CCVector3d(0.0, 0.0, 1.0);
		break;
	case 3:
	default:
		return CCVector3d(m_ui->axisXDoubleSpinBox->value(), m_ui->axisYDoubleSpinBox->value(), m_ui->axisZDoubleSpinBox->value());
	}

	return {};
}
 
bool ccUnrollDlg::isAxisPositionAuto() const
{
	return m_ui->checkBoxAuto->isChecked();
}

bool ccUnrollDlg::useArbitraryOutputCS() const
{
	return m_ui->arbitraryCSCheckBox->isChecked();
}

void ccUnrollDlg::getAngleRange(double& start_deg, double& stop_deg) const
{
	start_deg = m_ui->startAngleDoubleSpinBox->value();
	stop_deg = m_ui->stopAngleDoubleSpinBox->value();
}

CCVector3 ccUnrollDlg::getAxisPosition() const
{
	return CCVector3(	static_cast<PointCoordinateType>(m_ui->axisCenterXDoubleSpinBox->value()),
						static_cast<PointCoordinateType>(m_ui->axisCenterYDoubleSpinBox->value()),
						static_cast<PointCoordinateType>(m_ui->axisCenterZDoubleSpinBox->value()));
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
		m_ui->axisPositionGroupBox->setTitle("Axis position");
		m_ui->radiusLabel->setText("Radius");
		axisAutoStateChanged(m_ui->checkBoxAuto->checkState());
		m_ui->fromEntityToolButton->setEnabled(true);
	}
	break;
	case ccPointCloud::CONE: //cone
	{
		m_ui->angleFrame->setVisible(true);
		m_ui->autoCenterFrame->setVisible(false);
		m_ui->radiusFrame->setVisible(false);
		m_ui->radiusLabel->setText("Base radius");
		m_ui->axisPositionGroupBox->setTitle("Cone apex");
		axisAutoStateChanged(Qt::Unchecked);
		//may be disabled if we were in cylinder mode previously
		m_ui->axisCenterXDoubleSpinBox->setDisabled(false);
		m_ui->axisCenterYDoubleSpinBox->setDisabled(false);
		m_ui->axisCenterZDoubleSpinBox->setDisabled(false);
		m_ui->fromEntityToolButton->setEnabled(false); // not supported for now
	}
	break;
	case ccPointCloud::STRAIGHTENED_CONE: //straightened cone (fixed radius)
	case ccPointCloud::STRAIGHTENED_CONE2: //straightened cone 2
	{
		m_ui->angleFrame->setVisible(true);
		m_ui->radiusFrame->setVisible(true);
		m_ui->autoCenterFrame->setVisible(false);
		m_ui->axisPositionGroupBox->setTitle("Cone apex");
		axisAutoStateChanged(Qt::Unchecked);
		//may be disabled if we were in cylinder mode previously
		m_ui->axisCenterXDoubleSpinBox->setDisabled(false);
		m_ui->axisCenterYDoubleSpinBox->setDisabled(false);
		m_ui->axisCenterZDoubleSpinBox->setDisabled(false);
		m_ui->fromEntityToolButton->setEnabled(false); // not supported for now
	}
	break;
	};
}

void ccUnrollDlg::axisAutoStateChanged(int checkState)
{
	if (checkState == Qt::Unchecked)
	{
		m_ui->axisCenterFrame->setEnabled(true);
		axisDimensionChanged(m_ui->comboBoxAxisDimension->currentIndex());
	}
	else
	{
		m_ui->axisCenterFrame->setEnabled(false);
	}
}

void ccUnrollDlg::axisDimensionChanged(int index)
{
	m_ui->axisFrame->setEnabled(index == 3);

	if (	(m_ui->comboBoxUnrollShapeType->currentIndex() == 0)
		&&	(m_ui->checkBoxAuto->checkState() != Qt::Checked) )
	{
		//in 'cylinder' mode, we hide the axis coordinate that is not needed
		m_ui->axisCenterXDoubleSpinBox->setDisabled(index == 0);
		m_ui->axisCenterYDoubleSpinBox->setDisabled(index == 1);
		m_ui->axisCenterZDoubleSpinBox->setDisabled(index == 2);
	}
}

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
		settings.setValue("axis.x",				m_ui->axisXDoubleSpinBox->value());
		settings.setValue("axis.y",				m_ui->axisYDoubleSpinBox->value());
		settings.setValue("axis.z",				m_ui->axisZDoubleSpinBox->value());
		settings.setValue("axisCenter.x",		m_ui->axisCenterXDoubleSpinBox->value());
		settings.setValue("axisCenter.y",		m_ui->axisCenterYDoubleSpinBox->value());
		settings.setValue("axisCenter.z",		m_ui->axisCenterZDoubleSpinBox->value());

		getAngleRange(s_startAngle_deg, s_stopAngle_deg);

		s_arbitraryOutputCS = useArbitraryOutputCS();
	}
	settings.endGroup();
}

void ccUnrollDlg::fromPersistentSettings()
{
	QSettings settings;
	settings.beginGroup("Unroll");
	{
		int shapeType          = settings.value("shapeType",         m_ui->comboBoxUnrollShapeType->currentIndex()).toInt();
		int axisDim            = settings.value("axisDimension",     m_ui->comboBoxAxisDimension->currentIndex()).toInt();
		double angle           = settings.value("angle",             m_ui->halfAngleDoubleSpinBox->value()).toDouble();
		double radius          = settings.value("radius",            m_ui->radiusDoubleSpinBox->value()).toDouble();
		bool autoCenter        = settings.value("autoCenter",        m_ui->checkBoxAuto->isChecked()).toBool();
		bool exportDeviationSF = settings.value("exportDeviationSF", m_ui->exportDeviationSFCheckBox->isChecked()).toBool();

		CCVector3d axis;
		axis.x = settings.value("axis.x", m_ui->axisXDoubleSpinBox->value()).toDouble();
		axis.y = settings.value("axis.y", m_ui->axisYDoubleSpinBox->value()).toDouble();
		axis.z = settings.value("axis.z", m_ui->axisZDoubleSpinBox->value()).toDouble();

		CCVector3d axisCenter;
		axisCenter.x = settings.value("axisCenter.x", m_ui->axisCenterXDoubleSpinBox->value()).toDouble();
		axisCenter.y = settings.value("axisCenter.y", m_ui->axisCenterYDoubleSpinBox->value()).toDouble();
		axisCenter.z = settings.value("axisCenter.z", m_ui->axisCenterZDoubleSpinBox->value()).toDouble();

		m_ui->comboBoxUnrollShapeType->setCurrentIndex(shapeType);
		m_ui->comboBoxAxisDimension->setCurrentIndex(axisDim);
		m_ui->halfAngleDoubleSpinBox->setValue(angle);
		m_ui->radiusDoubleSpinBox->setValue(radius);
		m_ui->checkBoxAuto->setChecked(autoCenter);
		m_ui->exportDeviationSFCheckBox->setChecked(exportDeviationSF);

		m_ui->axisXDoubleSpinBox->setValue(axis.x);
		m_ui->axisYDoubleSpinBox->setValue(axis.y);
		m_ui->axisZDoubleSpinBox->setValue(axis.z);
		m_ui->axisCenterXDoubleSpinBox->setValue(axisCenter.x);
		m_ui->axisCenterYDoubleSpinBox->setValue(axisCenter.y);
		m_ui->axisCenterZDoubleSpinBox->setValue(axisCenter.z);

		m_ui->startAngleDoubleSpinBox->setValue(s_startAngle_deg);
		m_ui->stopAngleDoubleSpinBox ->setValue(s_stopAngle_deg);

		m_ui->arbitraryCSCheckBox->setChecked(s_arbitraryOutputCS);
	}
	settings.endGroup();
}

void ccUnrollDlg::loadParametersFromEntity()
{
	if (!m_dbRootEntity)
	{
		assert(false);
		return;
	}

	ccHObject::Container cylinders;
	m_dbRootEntity->filterChildren(cylinders, true, CC_TYPES::CYLINDER, false);

	if (cylinders.empty())
	{
		ccLog::Error("No cylinder in DB");
		return;
	}
	int selectedIndex = ccEntitySelectionDialog::SelectEntity(cylinders, -1, this, tr("Select a cylinder entity"));
	if (selectedIndex < 0)
	{
		//process cancelled by the user
		return;
	}

	const ccCylinder* cylinder = static_cast<const ccCylinder*>(cylinders[selectedIndex]);
	CCVector3 axis = cylinder->getTransformation().getColumnAsVec3D(2); // Z axis is the cylinder axis
	CCVector3 origin = cylinder->getTransformation().getTranslationAsVec3D();
	PointCoordinateType radius = cylinder->getBottomRadius();

	m_ui->comboBoxAxisDimension->setCurrentIndex(3); // custom
	m_ui->axisXDoubleSpinBox->setValue(axis.x);
	m_ui->axisYDoubleSpinBox->setValue(axis.y);
	m_ui->axisZDoubleSpinBox->setValue(axis.z);
	m_ui->axisCenterXDoubleSpinBox->setValue(origin.x);
	m_ui->axisCenterYDoubleSpinBox->setValue(origin.y);
	m_ui->axisCenterZDoubleSpinBox->setValue(origin.z);
	m_ui->checkBoxAuto->setChecked(false);
}

void ccUnrollDlg::axisFromClipboard()
{
	CCVector3d vector;
	if (ccUtils::GetVectorFromClipboard(vector))
	{
		m_ui->axisXDoubleSpinBox->setValue(vector.x);
		m_ui->axisYDoubleSpinBox->setValue(vector.y);
		m_ui->axisZDoubleSpinBox->setValue(vector.z);
	}
}

void ccUnrollDlg::centerFromClipboard()
{
	CCVector3d vector;
	if (ccUtils::GetVectorFromClipboard(vector))
	{
		m_ui->axisCenterXDoubleSpinBox->setValue(vector.x);
		m_ui->axisCenterYDoubleSpinBox->setValue(vector.y);
		m_ui->axisCenterZDoubleSpinBox->setValue(vector.z);
	}
}
