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
//$Rev:: 2172                                                              $
//$LastChangedDate:: 2012-06-24 18:33:24 +0200 (dim., 24 juin 2012)        $
//**************************************************************************
//

#include "sfEditDlg.h"

//qCC_db
#include <ccScalarField.h>

//CCLib
#include <CCConst.h>

//system
#include <math.h>
#include <assert.h>

#define SLIDERS_STEPS 1000

sfEditDlg::sfEditDlg(QWidget* parent)
	: QWidget(parent)
	, Ui::SFEditDlg()
    , m_associatedSF(0)
    , m_step1(0)
	, m_step2(0)
    , m_coef1(0)
	, m_coef2(0)
    , m_lowBound(0)
	, m_upBound(0)
    , m_satSpan(0)
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
	connect(dispValSlider, SIGNAL(spanChanged(int,int)), this, SLOT(dispValSLDChanged(int,int)));
	connect(satValSlider,  SIGNAL(spanChanged(int,int)), this, SLOT(satValSLDChanged(int,int)));

	//checkboxes
	connect(absSatCheckBox,		SIGNAL(stateChanged(int)), this, SLOT(absSatChanged(int)));
	connect(logScaleCheckBox,	SIGNAL(stateChanged(int)), this, SLOT(logScaleChanged(int)));
	connect(releaseBoundariesCheckBox,	SIGNAL(stateChanged(int)), this, SLOT(boundariesLockChanged(int)));

    show();
}

double sfEditDlg::spin2slider_1(double val)
{
    return (val-m_lowBound)*m_coef1;
}

double sfEditDlg::spin2slider_2(double val)
{
	return m_associatedSF->absoluteSaturation() ? val*m_coef2 : (val-m_lowBound)*m_coef2;
}

double sfEditDlg::slider2spin_1(int val)
{
    return (double)val/(double)SLIDERS_STEPS*(m_upBound-m_lowBound)+m_lowBound;
}

double sfEditDlg::slider2spin_2(int val)
{
	return m_associatedSF->absoluteSaturation() ? (double)val/(double)SLIDERS_STEPS*m_satSpan : slider2spin_1(val);
}

void sfEditDlg::SetValuesWith(ccScalarField* sf)
{
    assert(sf);
    m_associatedSF = sf;

    bool isPositive = sf->isPositive();
	bool absSaturation = sf->absoluteSaturation();
	bool logScale = sf->logScale();
	bool boundariesReleased = !sf->areBoundariesAutoUpdated();

	absSatCheckBox->setVisible(!isPositive);

	absSatCheckBox->blockSignals(true);
	absSatCheckBox->setChecked(absSaturation);
	absSatCheckBox->setEnabled(!logScale);
	absSatCheckBox->blockSignals(false);

	logScaleCheckBox->blockSignals(true);
	logScaleCheckBox->setChecked(logScale);
	logScaleCheckBox->blockSignals(false);
	
	releaseBoundariesCheckBox->blockSignals(true);
	releaseBoundariesCheckBox->setChecked(boundariesReleased);
	releaseBoundariesCheckBox->blockSignals(false);

    minValSpinBox->blockSignals(true);
	minSatSpinBox->blockSignals(true);
	maxSatSpinBox->blockSignals(true);
	maxValSpinBox->blockSignals(true);
	dispValSlider->blockSignals(true);
	satValSlider->blockSignals(true);

	//min/max values
	DistanceType minDist=sf->getMinDisplayed();
    DistanceType maxDist=sf->getMaxDisplayed();
    DistanceType minSat=sf->getMinSaturation();
    DistanceType maxSat=sf->getMaxSaturation();
    m_lowBound = sf->getMin();
    m_upBound  = sf->getMax();

	//in 'released' mode, min and max displayed values can be outbounds
	m_lowBound = std::min(m_lowBound,(double)minDist);
	m_upBound = std::max(m_upBound,(double)maxDist);

	m_step1 = (m_upBound-m_lowBound)*1e-6;
    m_coef1 = (m_upBound>m_lowBound ? (double)SLIDERS_STEPS/(m_upBound-m_lowBound) : 0.0);

    //saturation values
	m_satSpan = (absSaturation ? std::max(fabs(m_lowBound),fabs(m_upBound)) : m_upBound-m_lowBound);
    m_step2 = m_satSpan * 1e-6;
    m_coef2 = (m_satSpan>0 ? (double)SLIDERS_STEPS/m_satSpan : 0.0);

	/*** sliders ***/
	dispValSlider->setSpan((int)floor(spin2slider_1(minDist)),(int)ceil(spin2slider_1(maxDist)));
	satValSlider->setSpan(((int)floor(spin2slider_2(minSat))),(int)ceil(spin2slider_2(maxSat)));

	double dispLowBound = m_lowBound;
	double dispUpBound = m_upBound;
	if (boundariesReleased)
	{
		dispUpBound = (double)BIG_VALUE-(double)1;
		dispLowBound = (isPositive ? 0 : -dispUpBound);
	}

	/*** spinboxes ***/

    //Minimum displayed value
	minValSpinBox->setRange(dispLowBound,dispUpBound);
    minValSpinBox->setSingleStep(m_step1);
    minValSpinBox->setValue(minDist);

    //Minimum color saturation value
	if (absSaturation)
		minSatSpinBox->setRange(0,m_satSpan);
	else
		minSatSpinBox->setRange(m_lowBound,m_upBound);
    minSatSpinBox->setSingleStep(m_step2);
    minSatSpinBox->setValue(minSat);

    // Maximum color saturation value slider
	if (absSaturation)
		maxSatSpinBox->setRange(0,m_satSpan);
	else
		maxSatSpinBox->setRange(m_lowBound,m_upBound);
    maxSatSpinBox->setSingleStep(m_step2);
    maxSatSpinBox->setValue(maxSat);

    // Maximum displayed value slider
	maxValSpinBox->setRange(dispLowBound,dispUpBound);
    maxValSpinBox->setSingleStep(m_step1);
    maxValSpinBox->setValue(maxDist);

	minValSpinBox->blockSignals(false);
	minSatSpinBox->blockSignals(false);
	maxSatSpinBox->blockSignals(false);
	maxValSpinBox->blockSignals(false);
	dispValSlider->blockSignals(false);
	satValSlider->blockSignals(false);
}

void sfEditDlg::minValSBChanged(double val)
{
    if (!m_associatedSF)
        return;

    m_associatedSF->setMinDisplayed((DistanceType)val);

	if (m_associatedSF->areBoundariesAutoUpdated())
	{
	maxValSpinBox->blockSignals(true);
		maxValSpinBox->setRange(val,m_upBound);
	maxValSpinBox->blockSignals(false);
	}
	else
	{
		//'val' can be anything! If it
		if (val<m_associatedSF->getMin())
		{
			m_associatedSF->setBoundaries(val,m_associatedSF->getMax());
			SetValuesWith(m_associatedSF);
		}
		else if (val>m_associatedSF->getMax())
		{
			m_associatedSF->setBoundaries(val,val);
			maxValSpinBox->blockSignals(true);
			maxValSpinBox->setValue(val);
			maxValSpinBox->blockSignals(false);
			SetValuesWith(m_associatedSF);
		}
	}

	dispValSlider->blockSignals(true);
	dispValSlider->setLowerValue((int)floor(spin2slider_1(val)));
	dispValSlider->blockSignals(false);
    QApplication::processEvents();

    emit entitySFHasChanged();
}

void sfEditDlg::maxValSBChanged(double val)
{
    if (!m_associatedSF)
        return;

    m_associatedSF->setMaxDisplayed((DistanceType)val);

	if (m_associatedSF->areBoundariesAutoUpdated())
	{
	minValSpinBox->blockSignals(true);
		minValSpinBox->setRange(m_lowBound,val);
	minValSpinBox->blockSignals(false);
	}
	else
	{
		//'val' can be anything!
		if (val<m_associatedSF->getMin())
		{
			m_associatedSF->setBoundaries(val,val);
			minValSpinBox->blockSignals(true);
			minValSpinBox->setValue(val);
			minValSpinBox->blockSignals(false);
			SetValuesWith(m_associatedSF);
		}
		else if (val>m_associatedSF->getMax())
		{
			m_associatedSF->setBoundaries(m_associatedSF->getMin(),val);
			SetValuesWith(m_associatedSF);
		}
	}

	dispValSlider->blockSignals(true);
	int pos = (int)ceil(spin2slider_1(val));
	if (pos>SLIDERS_STEPS)
		pos=SLIDERS_STEPS;
	dispValSlider->setUpperPosition(pos);
	dispValSlider->blockSignals(false);
    QApplication::processEvents();

    emit entitySFHasChanged();
}

void sfEditDlg::minSatSBChanged(double val)
{
    if (!m_associatedSF)
        return;

	m_associatedSF->setMinSaturation(DistanceType(val));

	maxSatSpinBox->blockSignals(true);
	maxSatSpinBox->setRange(val,m_associatedSF->absoluteSaturation() ? m_satSpan : m_upBound);
	maxSatSpinBox->blockSignals(false);

	satValSlider->blockSignals(true);
	satValSlider->setLowerPosition((int)floor(spin2slider_2(val)));
	satValSlider->blockSignals(false);
    QApplication::processEvents();

    emit entitySFHasChanged();
}

void sfEditDlg::maxSatSBChanged(double val)
{
    if (!m_associatedSF)
        return;

	m_associatedSF->setMaxSaturation(DistanceType(val));

	minSatSpinBox->blockSignals(true);
	minSatSpinBox->setRange(m_associatedSF->absoluteSaturation() ? 0.0 : m_lowBound,val);
	minSatSpinBox->blockSignals(false);

	satValSlider->blockSignals(true);
	int pos = (int)ceil(spin2slider_2(val));
	if (pos>SLIDERS_STEPS)
		pos=SLIDERS_STEPS;
	satValSlider->setUpperPosition(pos);
	satValSlider->blockSignals(false);
    QApplication::processEvents();

    emit entitySFHasChanged();
}

void sfEditDlg::dispValSLDChanged(int minVal,int maxVal)
{
    if (!m_associatedSF)
        return;

	double dMinVal=slider2spin_1(minVal);
	double dMaxVal=slider2spin_1(maxVal);
	m_associatedSF->setMinDisplayed(DistanceType(dMinVal));
	m_associatedSF->setMaxDisplayed(DistanceType(dMaxVal));

	if (m_associatedSF->areBoundariesAutoUpdated())
	{
	minValSpinBox->blockSignals(true);
	maxValSpinBox->blockSignals(true);

		minValSpinBox->setRange(m_lowBound,dMaxVal);
    minValSpinBox->setValue(dMinVal);
		maxValSpinBox->setRange(dMinVal,m_upBound);
    maxValSpinBox->setValue(dMaxVal);

    minValSpinBox->blockSignals(false);
    maxValSpinBox->blockSignals(false);
	}

    QApplication::processEvents();

	emit entitySFHasChanged();
}

void sfEditDlg::satValSLDChanged(int minVal,int maxVal)
{
    if (!m_associatedSF)
        return;

	double dMinVal=slider2spin_2(minVal);
	double dMaxVal=slider2spin_2(maxVal);
	m_associatedSF->setMinSaturation(DistanceType(dMinVal));
	m_associatedSF->setMaxSaturation(DistanceType(dMaxVal));

	minSatSpinBox->blockSignals(true);
	maxSatSpinBox->blockSignals(true);

	minSatSpinBox->setRange(m_associatedSF->absoluteSaturation() ? 0.0 : m_lowBound,dMaxVal);
    minSatSpinBox->setValue(dMinVal);
	maxSatSpinBox->setRange(dMinVal,m_associatedSF->absoluteSaturation() ? m_satSpan : m_upBound);
    maxSatSpinBox->setValue(dMaxVal);

    minSatSpinBox->blockSignals(false);
    maxSatSpinBox->blockSignals(false);

    QApplication::processEvents();

	emit entitySFHasChanged();
}

void sfEditDlg::absSatChanged(int)
{
    if (!m_associatedSF)
        return;

	if (m_associatedSF->absoluteSaturation() != absSatCheckBox->isChecked())
	{
		m_associatedSF->setAbsoluteSaturation(absSatCheckBox->isChecked());
		SetValuesWith(m_associatedSF);
		emit entitySFHasChanged();
	}
}

void sfEditDlg::logScaleChanged(int)
{
    if (!m_associatedSF)
        return;

	if (m_associatedSF->logScale() != logScaleCheckBox->isChecked())
	{
		//we force absoute saturation
		bool logScale = logScaleCheckBox->isChecked();
		if (logScale)
			absSatCheckBox->setChecked(true);
		absSatCheckBox->setEnabled(!logScale);

		m_associatedSF->setLogScale(logScale);
		SetValuesWith(m_associatedSF);
		emit entitySFHasChanged();
	}
}

void sfEditDlg::boundariesLockChanged(int)
{
    if (!m_associatedSF)
        return;

	if (m_associatedSF->areBoundariesAutoUpdated() == releaseBoundariesCheckBox->isChecked())
	{
		//we change the 'auto update' state
		m_associatedSF->autoUpdateBoundaries(!releaseBoundariesCheckBox->isChecked());
		SetValuesWith(m_associatedSF);
		emit entitySFHasChanged();
	}
}