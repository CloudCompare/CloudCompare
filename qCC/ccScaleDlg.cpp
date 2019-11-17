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

//semi persistent parameters
static CCVector3d s_lastScales(1.0, 1.0, 1.0);
static bool s_allAtOnce = true;
static bool s_keepInPlace = false;
static bool s_rescaleGlobalShift = true;

ccScaleDlg::ccScaleDlg(QWidget* parent/*=0*/)
	: QDialog(parent)
	, Ui::ScaleDialog()
{
	setupUi(this);

	connect(sameForAllCheckBox, &QCheckBox::toggled, this, &ccScaleDlg::allDimsAtOnceToggled);
	connect(fxSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccScaleDlg::fxUpdated);

	//restore semi-persistent parameters
	sameForAllCheckBox->setChecked(s_allAtOnce);
	keepInPlaceCheckBox->setChecked(s_keepInPlace);
	rescaleGlobalShiftCheckBox->setChecked(s_rescaleGlobalShift);
	fzSpinBox->setValue(s_lastScales.z);
	fySpinBox->setValue(s_lastScales.y);
	fxSpinBox->setValue(s_lastScales.x); //always last in case 'same for all' is checked!
}

void ccScaleDlg::saveState()
{
	s_allAtOnce = sameForAllCheckBox->isChecked();
	s_lastScales = getScales();
	s_keepInPlace = keepInPlace();
	s_rescaleGlobalShift = rescaleGlobalShift();
}

void ccScaleDlg::fxUpdated(double val)
{
	if (sameForAllCheckBox->isChecked())
	{
		fySpinBox->setValue(val);
		fzSpinBox->setValue(val);
	}
}

CCVector3d ccScaleDlg::getScales() const
{
	return CCVector3d(
		fxSpinBox->value(),
		fySpinBox->value(),
		fzSpinBox->value()
		);
}

bool ccScaleDlg::keepInPlace() const
{
	return keepInPlaceCheckBox->isChecked();
}

bool ccScaleDlg::rescaleGlobalShift() const
{
	return rescaleGlobalShiftCheckBox->isChecked();
}

void ccScaleDlg::allDimsAtOnceToggled(bool state)
{
	if (state)
	{
		fySpinBox->setValue(fxSpinBox->value());
		fzSpinBox->setValue(fxSpinBox->value());
	}
	fySpinBox->setEnabled(!state);
	fzSpinBox->setEnabled(!state);
}
