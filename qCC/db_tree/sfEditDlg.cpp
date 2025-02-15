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

#include "sfEditDlg.h"
#include "ui_sfEditDlg.h"

//Local
#include "ccHistogramWindow.h"

//qCC_db
#include <ccScalarField.h>

//CCCoreLib
#include <CCConst.h>

//system
#include <cassert>

//! Default number of steps for spin-boxes
const int SPIN_BOX_STEPS = 1000;

sfEditDlg::sfEditDlg(QWidget* parent/*=nullptr*/)
	: QWidget(parent)
	, m_associatedSF(nullptr)
	, m_associatedSFHisto(nullptr)
	, m_ui( new Ui::SFEditDlg )
{
	m_ui->setupUi(this);

	//histogram window
	{
		m_associatedSFHisto = new ccHistogramWindow;
		QHBoxLayout* hboxLayout = new QHBoxLayout(m_ui->histoFrame);
		hboxLayout->addWidget(m_associatedSFHisto);
		hboxLayout->setContentsMargins(0, 0, 0, 0);
		m_associatedSFHisto->setSFInteractionMode(ccHistogramWindow::SFInteractionMode::All);
		m_associatedSFHisto->xAxis->setTickLabels(false);
		m_associatedSFHisto->xAxis->ticker()->setTickCount(6);
		m_associatedSFHisto->yAxis->setTickLabels(false);
		m_associatedSFHisto->yAxis->ticker()->setTickCount(3);
	}

	connect(m_ui->minValSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this,	&sfEditDlg::minValSBChanged);
	connect(m_ui->maxValSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this,	&sfEditDlg::maxValSBChanged);
	connect(m_ui->minSatSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this,	&sfEditDlg::minSatSBChanged);
	connect(m_ui->maxSatSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this,	&sfEditDlg::maxSatSBChanged);

	connect(m_associatedSFHisto,	&ccHistogramWindow::sfMinDispValChanged,	this,	&sfEditDlg::minValHistoChanged);
	connect(m_associatedSFHisto,	&ccHistogramWindow::sfMaxDispValChanged,	this,	&sfEditDlg::maxValHistoChanged);
	connect(m_associatedSFHisto,	&ccHistogramWindow::sfMinSatValChanged,		this,	&sfEditDlg::minSatHistoChanged);
	connect(m_associatedSFHisto,	&ccHistogramWindow::sfMaxSatValChanged,		this,	&sfEditDlg::maxSatHistoChanged);

	//checkboxes
	connect(m_ui->nanInGreyCheckBox,		&QCheckBox::toggled,	this,	&sfEditDlg::nanInGrayChanged);
	connect(m_ui->alwaysShow0CheckBox,		&QCheckBox::toggled,	this,	&sfEditDlg::alwaysShow0Changed);
	connect(m_ui->symmetricalScaleCheckBox,	&QCheckBox::toggled,	this,	&sfEditDlg::symmetricalScaleChanged);
	connect(m_ui->logScaleCheckBox,			&QCheckBox::toggled,	this,	&sfEditDlg::logScaleChanged);

	show();
}

sfEditDlg::~sfEditDlg()
{
	delete m_ui;
}

void sfEditDlg::fillDialogWith(ccScalarField* sf)
{
	m_associatedSF = sf;
	if (!sf)
	{
		assert(false);
		setEnabled(false);
		m_ui->histoFrame->setVisible(false);
		return;
	}

	//options (checkboxes)
	{
		bool nanValuesInGrey	= sf->areNaNValuesShownInGrey();
		bool alwaysShowZero		= sf->isZeroAlwaysShown();
		bool symmetricalScale	= sf->symmetricalScale();
		bool logScale			= sf->logScale();
		bool absoluteScale		= sf->getColorScale() && !sf->getColorScale()->isRelative();

		m_ui->nanInGreyCheckBox->blockSignals(true);
		m_ui->nanInGreyCheckBox->setChecked(nanValuesInGrey);
		m_ui->nanInGreyCheckBox->blockSignals(false);

		m_ui->alwaysShow0CheckBox->blockSignals(true);
		m_ui->alwaysShow0CheckBox->setChecked(alwaysShowZero);
		m_ui->alwaysShow0CheckBox->setEnabled(!logScale);
		m_ui->alwaysShow0CheckBox->blockSignals(false);

		m_ui->symmetricalScaleCheckBox->blockSignals(true);
		m_ui->symmetricalScaleCheckBox->setChecked(symmetricalScale);
		m_ui->symmetricalScaleCheckBox->setEnabled(!absoluteScale && !logScale);
		m_ui->symmetricalScaleCheckBox->blockSignals(false);

		m_ui->logScaleCheckBox->blockSignals(true);
		m_ui->logScaleCheckBox->setChecked(logScale);
		m_ui->logScaleCheckBox->blockSignals(false);

		if (logScale)
			m_ui->satLabel->setText("log sat.");
		else if (symmetricalScale)
			m_ui->satLabel->setText("abs. sat.");
		else
			m_ui->satLabel->setText("saturation");
	}

	//displayed and saturation values
	{
		const ccScalarField::Range& displayRange = sf->displayRange();
		const ccScalarField::Range& saturationRange = sf->saturationRange();

		//special case: no need to actiate this widget for flat scalar field
		//(worse, divisions by zero may occur!)
		bool flatSF = (displayRange.maxRange() == 0);
		m_ui->slidersFrame->setEnabled(!flatSF);

		//show histogram
		m_ui->histoFrame->setVisible(true);
		{
			const ccScalarField::Histogram& histogram = m_associatedSF->getHistogram();
			unsigned classNumber = static_cast<unsigned>(histogram.size());
			if (classNumber == 0)
				classNumber = 128;
			m_associatedSFHisto->fromSF(m_associatedSF, classNumber, false);
		}

		/*** spinboxes ***/

		m_ui->minValSpinBox->blockSignals(true);
		m_ui->minSatSpinBox->blockSignals(true);
		m_ui->maxSatSpinBox->blockSignals(true);
		m_ui->maxValSpinBox->blockSignals(true);

		if (!flatSF)
		{
			//Minimum displayed value
			m_ui->minValSpinBox->setRange(displayRange.min(), displayRange.stop());
			m_ui->minValSpinBox->setSingleStep(displayRange.maxRange() / SPIN_BOX_STEPS);
			m_ui->minValSpinBox->setValue(displayRange.start());

			//Minimum color saturation value
			m_ui->minSatSpinBox->setRange(saturationRange.min(), saturationRange.stop());
			m_ui->minSatSpinBox->setSingleStep(saturationRange.maxRange() / SPIN_BOX_STEPS);
			m_ui->minSatSpinBox->setValue(saturationRange.start());

			// Maximum color saturation value slider
			m_ui->maxSatSpinBox->setRange(saturationRange.start(), saturationRange.max());
			m_ui->maxSatSpinBox->setSingleStep(saturationRange.maxRange() / SPIN_BOX_STEPS);
			m_ui->maxSatSpinBox->setValue(saturationRange.stop());

			// Maximum displayed value slider
			m_ui->maxValSpinBox->setRange(displayRange.start(), displayRange.max());
			m_ui->maxValSpinBox->setSingleStep(displayRange.maxRange() / SPIN_BOX_STEPS);
			m_ui->maxValSpinBox->setValue(displayRange.stop());
		}
		else
		{
			//unique value
			double uniqueVal = displayRange.min();
			m_ui->minValSpinBox->setRange(uniqueVal, uniqueVal);
			m_ui->minSatSpinBox->setRange(uniqueVal, uniqueVal);
			m_ui->maxSatSpinBox->setRange(uniqueVal, uniqueVal);
			m_ui->maxValSpinBox->setRange(uniqueVal, uniqueVal);
		}

		m_ui->minValSpinBox->blockSignals(false);
		m_ui->minSatSpinBox->blockSignals(false);
		m_ui->maxSatSpinBox->blockSignals(false);
		m_ui->maxValSpinBox->blockSignals(false);
	}
}

void sfEditDlg::minValSBChanged(double val)
{
	if (!m_associatedSF)
		return;

	m_associatedSFHisto->setMinDispValue(val);

	Q_EMIT entitySFHasChanged();

	QApplication::processEvents();

}

void sfEditDlg::maxValSBChanged(double val)
{
	if (!m_associatedSF)
		return;

	m_associatedSFHisto->setMaxDispValue(val);

	Q_EMIT entitySFHasChanged();

	QApplication::processEvents();
}

void sfEditDlg::minSatSBChanged(double val)
{
	if (!m_associatedSF)
		return;

	m_associatedSFHisto->setMinSatValue(val);

	Q_EMIT entitySFHasChanged();

	QApplication::processEvents();
}

void sfEditDlg::maxSatSBChanged(double val)
{
	if (!m_associatedSF)
		return;

	m_associatedSFHisto->setMaxSatValue(val);

	Q_EMIT entitySFHasChanged();

	QApplication::processEvents();
}

void sfEditDlg::minValHistoChanged(double val)
{
	if (!m_associatedSF)
		return;

	m_ui->minValSpinBox->blockSignals(true);
	m_ui->minValSpinBox->setValue(val);
	m_ui->minValSpinBox->blockSignals(false);

	Q_EMIT entitySFHasChanged();

	QApplication::processEvents();
}

void sfEditDlg::maxValHistoChanged(double val)
{
	if (!m_associatedSF)
		return;

	m_ui->maxValSpinBox->blockSignals(true);
	m_ui->maxValSpinBox->setValue(val);
	m_ui->maxValSpinBox->blockSignals(false);

	Q_EMIT entitySFHasChanged();

	QApplication::processEvents();
}

void sfEditDlg::minSatHistoChanged(double val)
{
	if (!m_associatedSF)
		return;

	m_ui->minSatSpinBox->blockSignals(true);
	m_ui->minSatSpinBox->setValue(val);
	m_ui->minSatSpinBox->blockSignals(false);

	Q_EMIT entitySFHasChanged();

	QApplication::processEvents();
}

void sfEditDlg::maxSatHistoChanged(double val)
{
	if (!m_associatedSF)
		return;

	m_ui->maxSatSpinBox->blockSignals(true);
	m_ui->maxSatSpinBox->setValue(val);
	m_ui->maxSatSpinBox->blockSignals(false);

	Q_EMIT entitySFHasChanged();

	QApplication::processEvents();
}

void sfEditDlg::nanInGrayChanged(bool state)
{
	if (!m_associatedSF)
		return;

	if (m_associatedSF->areNaNValuesShownInGrey() != state)
	{
		m_associatedSF->showNaNValuesInGrey(state);
		Q_EMIT entitySFHasChanged();

		//m_associatedSFHisto->refreshBars();
	}
}

void sfEditDlg::alwaysShow0Changed(bool state)
{
	if (!m_associatedSF)
		return;

	if (m_associatedSF->isZeroAlwaysShown() != state)
	{
		m_associatedSF->alwaysShowZero(state);
		Q_EMIT entitySFHasChanged();

		//m_associatedSFHisto->refreshBars();
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
		Q_EMIT entitySFHasChanged();

		//Saturation might change!
		m_associatedSFHisto->refresh();
		//m_associatedSFHisto->refreshBars();
	}
}

void sfEditDlg::logScaleChanged(bool state)
{
	if (!m_associatedSF)
		return;

	if (m_associatedSF->logScale() != state)
	{
		m_associatedSF->setLogScale(state);
		fillDialogWith(m_associatedSF); //the saturation sliders + the symmetrical scale checkbox may need to be updated!
		Q_EMIT entitySFHasChanged();

		//Saturation might change!
		m_associatedSFHisto->refresh();
		//m_associatedSFHisto->refreshBars();
	}
}
