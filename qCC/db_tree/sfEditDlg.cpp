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

#include "sfEditDlg.h"

//qCC_db
#include <ccScalarField.h>

//CCLib
#include <CCConst.h>

//system
#include <math.h>
#include <assert.h>

//! Default steps per slider
const int SLIDERS_STEPS = 1000;

sfEditDlg::sfEditDlg(QWidget* parent/*=0*/)
	: QWidget(parent)
	, Ui::SFEditDlg()
	, m_associatedSF(0)
{
	setupUi(this);

	dispValSlider->setHandleMovementMode(QxtSpanSlider::NoCrossing);
	satValSlider->setHandleMovementMode(QxtSpanSlider::NoCrossing);

	dispValSlider->setRange(0,SLIDERS_STEPS);
	satValSlider->setRange(0,SLIDERS_STEPS);

	connect(minValSpinBox, SIGNAL(valueChanged(double)), this, SLOT(minValSBChanged(double)));
	connect(maxValSpinBox, SIGNAL(valueChanged(double)), this, SLOT(maxValSBChanged(double)));
	connect(minSatSpinBox, SIGNAL(valueChanged(double)), this, SLOT(minSatSBChanged(double)));
	connect(maxSatSpinBox, SIGNAL(valueChanged(double)), this, SLOT(maxSatSBChanged(double)));
	connect(dispValSlider, SIGNAL(spanChanged(int,int)), this, SLOT(dispValSliderChanged(int,int)));
	connect(satValSlider,  SIGNAL(spanChanged(int,int)), this, SLOT(satValSliderChanged(int,int)));

	//checkboxes
	connect(nanInGreyCheckBox,			SIGNAL(toggled(bool)), this, SLOT(nanInGrayChanged(bool)));
	connect(alwaysShow0CheckBox,		SIGNAL(toggled(bool)), this, SLOT(alwaysShow0Changed(bool)));
	connect(symmetricalScaleCheckBox,	SIGNAL(toggled(bool)), this, SLOT(symmetricalScaleChanged(bool)));
	connect(logScaleCheckBox,			SIGNAL(toggled(bool)), this, SLOT(logScaleChanged(bool)));

	show();
}

double sfEditDlg::dispSpin2slider(double val) const
{
	assert(m_associatedSF && m_associatedSF->displayRange().maxRange() != 0);
	return ((val - m_associatedSF->displayRange().min()) / m_associatedSF->displayRange().maxRange()) * (double)SLIDERS_STEPS;
}

double sfEditDlg::satSpin2slider(double val) const
{
	assert(m_associatedSF && m_associatedSF->saturationRange().maxRange() != 0);
	return ((val - m_associatedSF->saturationRange().min()) / m_associatedSF->saturationRange().maxRange()) * (double)SLIDERS_STEPS;
}

double sfEditDlg::dispSlider2spin(int pos) const
{
	assert(m_associatedSF);
	return m_associatedSF->displayRange().min() + (double)pos/(double)SLIDERS_STEPS * m_associatedSF->displayRange().maxRange();
}

double sfEditDlg::satSlider2spin(int pos) const
{
	assert(m_associatedSF);
	return m_associatedSF->saturationRange().min() + (double)pos/(double)SLIDERS_STEPS * m_associatedSF->saturationRange().maxRange();
}

void sfEditDlg::fillDialogWith(ccScalarField* sf)
{
	m_associatedSF = sf;
	if (!sf)
	{
		assert(false);
		setEnabled(false);
		return;
	}

	//options (checkboxes)
	{
		bool nanValuesInGrey = sf->areNaNValuesShownInGrey();
		bool alwaysShowZero = sf->isZeroAlwaysShown();
		bool symmetricalScale = sf->symmetricalScale();
		bool logScale = sf->logScale();
		bool absoluteScale = sf->getColorScale() && !sf->getColorScale()->isRelative();

		nanInGreyCheckBox->blockSignals(true);
		nanInGreyCheckBox->setChecked(nanValuesInGrey);
		nanInGreyCheckBox->blockSignals(false);

		alwaysShow0CheckBox->blockSignals(true);
		alwaysShow0CheckBox->setChecked(alwaysShowZero);
		alwaysShow0CheckBox->setEnabled(!logScale);
		alwaysShow0CheckBox->blockSignals(false);

		symmetricalScaleCheckBox->blockSignals(true);
		symmetricalScaleCheckBox->setChecked(symmetricalScale);
		symmetricalScaleCheckBox->setEnabled(!absoluteScale && !logScale);
		symmetricalScaleCheckBox->blockSignals(false);

		logScaleCheckBox->blockSignals(true);
		logScaleCheckBox->setChecked(logScale);
		logScaleCheckBox->blockSignals(false);

		if (logScale)
			satLabel->setText("log sat.");
		else if (symmetricalScale)
			satLabel->setText("abs. sat.");
		else
			satLabel->setText("saturation");

	}

	//displayed and saturation values
	{
		const ccScalarField::Range& displayRange = sf->displayRange();
		const ccScalarField::Range& saturationRange = sf->saturationRange();

		//special case: no need to actiate this widget for flat scalar field
		//(worse, divisions by zero may occur!)
		bool flatSF = (displayRange.maxRange() == 0);
		slidersFrame->setEnabled(!flatSF);

		minValSpinBox->blockSignals(true);
		minSatSpinBox->blockSignals(true);
		maxSatSpinBox->blockSignals(true);
		maxValSpinBox->blockSignals(true);
		dispValSlider->blockSignals(true);
		satValSlider->blockSignals(true);

		/*** sliders ***/
		if (!flatSF)
		{
			dispValSlider->setSpan((int)floor(dispSpin2slider(displayRange.start())),(int)ceil(dispSpin2slider(displayRange.stop())));
			satValSlider->setSpan(((int)floor(satSpin2slider(saturationRange.start()))),(int)ceil(satSpin2slider(saturationRange.stop())));
		}
		else
		{
			dispValSlider->setSpan(0,SLIDERS_STEPS);
			satValSlider->setSpan(0,SLIDERS_STEPS);
		}

		minValSpinBox->setEnabled(true);
		maxValSpinBox->setEnabled(true);

		/*** spinboxes ***/

		if (!flatSF)
		{
			//Minimum displayed value
			minValSpinBox->setRange(displayRange.min(),displayRange.stop());
			minValSpinBox->setSingleStep(displayRange.maxRange()/(double)SLIDERS_STEPS);
			minValSpinBox->setValue(displayRange.start());

			//Minimum color saturation value
			minSatSpinBox->setRange(saturationRange.min(),saturationRange.stop());
			minSatSpinBox->setSingleStep(saturationRange.maxRange()/(double)SLIDERS_STEPS);
			minSatSpinBox->setValue(saturationRange.start());

			// Maximum color saturation value slider
			maxSatSpinBox->setRange(saturationRange.start(),saturationRange.max());
			maxSatSpinBox->setSingleStep(saturationRange.maxRange()/(double)SLIDERS_STEPS);
			maxSatSpinBox->setValue(saturationRange.stop());

			// Maximum displayed value slider
			maxValSpinBox->setRange(displayRange.start(),displayRange.max());
			maxValSpinBox->setSingleStep(displayRange.maxRange()/(double)SLIDERS_STEPS);
			maxValSpinBox->setValue(displayRange.stop());
		}
		else
		{
			double uniqueVal = displayRange.min();
			minValSpinBox->setRange(uniqueVal,uniqueVal);
			minSatSpinBox->setRange(uniqueVal,uniqueVal);
			maxSatSpinBox->setRange(uniqueVal,uniqueVal);
			maxValSpinBox->setRange(uniqueVal,uniqueVal);
		}

		minValSpinBox->blockSignals(false);
		minSatSpinBox->blockSignals(false);
		maxSatSpinBox->blockSignals(false);
		maxValSpinBox->blockSignals(false);
		dispValSlider->blockSignals(false);
		satValSlider->blockSignals(false);
	}
}

void sfEditDlg::minValSBChanged(double val)
{
	if (!m_associatedSF)
		return;

	m_associatedSF->setMinDisplayed((ScalarType)val);

	maxValSpinBox->blockSignals(true);
	maxValSpinBox->setRange(m_associatedSF->displayRange().start(),m_associatedSF->displayRange().max());
	maxValSpinBox->blockSignals(false);

	dispValSlider->blockSignals(true);
	dispValSlider->setLowerValue((int)floor(dispSpin2slider(m_associatedSF->displayRange().start())));
	dispValSlider->blockSignals(false);
	QApplication::processEvents();

	emit entitySFHasChanged();
}

void sfEditDlg::maxValSBChanged(double val)
{
	if (!m_associatedSF)
		return;

	m_associatedSF->setMaxDisplayed((ScalarType)val);

	minValSpinBox->blockSignals(true);
	minValSpinBox->setRange(m_associatedSF->displayRange().min(),m_associatedSF->displayRange().stop());
	minValSpinBox->blockSignals(false);

	dispValSlider->blockSignals(true);
	int pos = (int)ceil(dispSpin2slider(m_associatedSF->displayRange().stop()));
	if (pos > SLIDERS_STEPS)
		pos = SLIDERS_STEPS;
	dispValSlider->setUpperPosition(pos);
	dispValSlider->blockSignals(false);
	QApplication::processEvents();

	emit entitySFHasChanged();
}

void sfEditDlg::minSatSBChanged(double val)
{
	if (!m_associatedSF)
		return;

	m_associatedSF->setSaturationStart((ScalarType)val);

	maxSatSpinBox->blockSignals(true);
	maxSatSpinBox->setRange(m_associatedSF->saturationRange().start(), m_associatedSF->saturationRange().max());
	maxSatSpinBox->blockSignals(false);

	satValSlider->blockSignals(true);
	satValSlider->setLowerPosition((int)floor(satSpin2slider(m_associatedSF->saturationRange().start())));
	satValSlider->blockSignals(false);
	QApplication::processEvents();

	emit entitySFHasChanged();
}

void sfEditDlg::maxSatSBChanged(double val)
{
	if (!m_associatedSF)
		return;

	m_associatedSF->setSaturationStop((ScalarType)val);

	minSatSpinBox->blockSignals(true);
	minSatSpinBox->setRange(m_associatedSF->saturationRange().min(), m_associatedSF->saturationRange().stop());
	minSatSpinBox->blockSignals(false);

	satValSlider->blockSignals(true);
	int pos = (int)ceil(satSpin2slider(m_associatedSF->saturationRange().stop()));
	if (pos > SLIDERS_STEPS)
		pos = SLIDERS_STEPS;
	satValSlider->setUpperPosition(pos);
	satValSlider->blockSignals(false);
	QApplication::processEvents();

	emit entitySFHasChanged();
}

void sfEditDlg::dispValSliderChanged(int minPos,int maxPos)
{
	if (!m_associatedSF)
		return;

	double minDispVal = dispSlider2spin(minPos);
	double maxDispVal = dispSlider2spin(maxPos);

	m_associatedSF->setMinDisplayed((ScalarType)minDispVal);
	m_associatedSF->setMaxDisplayed((ScalarType)maxDispVal);

	minValSpinBox->blockSignals(true);
	maxValSpinBox->blockSignals(true);

	minValSpinBox->setRange(m_associatedSF->displayRange().min(),m_associatedSF->displayRange().stop());
	minValSpinBox->setValue(m_associatedSF->displayRange().start());
	maxValSpinBox->setRange(m_associatedSF->displayRange().start(),m_associatedSF->displayRange().max());
	maxValSpinBox->setValue(m_associatedSF->displayRange().stop());

	minValSpinBox->blockSignals(false);
	maxValSpinBox->blockSignals(false);

	QApplication::processEvents();

	emit entitySFHasChanged();
}

void sfEditDlg::satValSliderChanged(int minPos,int maxPos)
{
	if (!m_associatedSF)
		return;

	double minSatVal = satSlider2spin(minPos);
	double maxSatVal = satSlider2spin(maxPos);

	m_associatedSF->setSaturationStart((ScalarType)minSatVal);
	m_associatedSF->setSaturationStop((ScalarType)maxSatVal);

	minSatSpinBox->blockSignals(true);
	maxSatSpinBox->blockSignals(true);

	minSatSpinBox->setRange(m_associatedSF->saturationRange().min(),m_associatedSF->saturationRange().stop());
	minSatSpinBox->setValue(m_associatedSF->saturationRange().start());
	maxSatSpinBox->setRange(m_associatedSF->saturationRange().start(),m_associatedSF->saturationRange().max());
	maxSatSpinBox->setValue(m_associatedSF->saturationRange().stop());

	minSatSpinBox->blockSignals(false);
	maxSatSpinBox->blockSignals(false);

	QApplication::processEvents();

	emit entitySFHasChanged();
}

void sfEditDlg::nanInGrayChanged(bool state)
{
	if (!m_associatedSF)
		return;

	if (m_associatedSF->areNaNValuesShownInGrey() != state)
	{
		m_associatedSF->showNaNValuesInGrey(state);
		emit entitySFHasChanged();
	}
}

void sfEditDlg::alwaysShow0Changed(bool state)
{
	if (!m_associatedSF)
		return;

	if (m_associatedSF->isZeroAlwaysShown() != state)
	{
		m_associatedSF->alwaysShowZero(state);
		emit entitySFHasChanged();
	}
}

void sfEditDlg::symmetricalScaleChanged(bool state)
{
	if (!m_associatedSF)
		return;

	if (m_associatedSF->symmetricalScale() != state)
	{
		m_associatedSF->setSymmetricalScale(state);
		fillDialogWith(m_associatedSF); //the saturation sliders may need to be updated!
		emit entitySFHasChanged();
	}
}

void sfEditDlg::logScaleChanged(bool state)
{
	if (!m_associatedSF)
		return;

	if (m_associatedSF->logScale() != state)
	{
		m_associatedSF->setLogScale(state);
		fillDialogWith(m_associatedSF);  //the saturation sliders + the symmetrical scale checkbox may need to be updated!
		emit entitySFHasChanged();
	}
}

