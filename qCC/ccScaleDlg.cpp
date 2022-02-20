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
//#                    COPYRIGHT: Daniel Girardeau-Montaut                 #
//#                                                                        #
//##########################################################################

#include "ccScaleDlg.h"
#include "ui_scaleDlg.h"

//semi persistent parameters
static CCVector3d s_lastScales(1.0, 1.0, 1.0);
static bool s_allAtOnce = true;
static bool s_keepInPlace = false;
static bool s_rescaleGlobalShift = true;

ccScaleDlg::ccScaleDlg(QWidget* parent/*=nullptr*/)
	: QDialog(parent)
	, m_ui( new Ui::ScaleDialog )
{
	m_ui->setupUi(this);

	connect(m_ui->sameForAllCheckBox, &QCheckBox::toggled, this, &ccScaleDlg::allDimsAtOnceToggled);
	connect(m_ui->fxSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ccScaleDlg::fxUpdated);

	//restore semi-persistent parameters
	m_ui->sameForAllCheckBox->setChecked(s_allAtOnce);
	m_ui->keepInPlaceCheckBox->setChecked(s_keepInPlace);
	m_ui->rescaleGlobalShiftCheckBox->setChecked(s_rescaleGlobalShift);
	m_ui->fzSpinBox->setValue(s_lastScales.z);
	m_ui->fySpinBox->setValue(s_lastScales.y);
	m_ui->fxSpinBox->setValue(s_lastScales.x); //always last in case 'same for all' is checked!
}

ccScaleDlg::~ccScaleDlg()
{
	delete m_ui;
}

void ccScaleDlg::saveState()
{
	s_allAtOnce = m_ui->sameForAllCheckBox->isChecked();
	s_lastScales = getScales();
	s_keepInPlace = keepInPlace();
	s_rescaleGlobalShift = rescaleGlobalShift();
}

void ccScaleDlg::fxUpdated(double val)
{
	if (m_ui->sameForAllCheckBox->isChecked())
	{
		m_ui->fySpinBox->setValue(val);
		m_ui->fzSpinBox->setValue(val);
	}
}

CCVector3d ccScaleDlg::getScales() const
{
	return CCVector3d(
		m_ui->fxSpinBox->value(),
		m_ui->fySpinBox->value(),
		m_ui->fzSpinBox->value()
		);
}

bool ccScaleDlg::keepInPlace() const
{
	return m_ui->keepInPlaceCheckBox->isChecked();
}

bool ccScaleDlg::rescaleGlobalShift() const
{
	return m_ui->rescaleGlobalShiftCheckBox->isChecked();
}

void ccScaleDlg::allDimsAtOnceToggled(bool state)
{
	if (state)
	{
		m_ui->fySpinBox->setValue(m_ui->fxSpinBox->value());
		m_ui->fzSpinBox->setValue(m_ui->fxSpinBox->value());
	}
	m_ui->fySpinBox->setEnabled(!state);
	m_ui->fzSpinBox->setEnabled(!state);
}
