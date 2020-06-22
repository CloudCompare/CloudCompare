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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include "ccColorFromScalarDlg.h"

//local
#include "ccConsole.h"
#include "ccHistogramWindow.h"

//qCC_db
#include <ccColorScale.h>
#include <ccGenericGLDisplay.h>
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>

//Qt
#include <QPushButton>

//ui
#include <ui_colorFromScalarDlg.h>

//system
#include <cassert>
#include <cstring>

ccColorFromScalarDlg::ccColorFromScalarDlg(QWidget* parent, ccPointCloud* pointCloud)
	: QDialog(parent, Qt::Tool)
	, m_cloud(pointCloud)
	, m_ui(new Ui::ColorFromScalarDialog)
	, m_systemInvalid(false)
{
	m_ui->setupUi(this);
	

	//populate boxes
	m_boxes_min[0] = m_ui->minInputSpinBoxR;
	m_boxes_max[0] = m_ui->maxInputSpinBoxR;
	m_boxes_min[1] = m_ui->minInputSpinBoxG;
	m_boxes_max[1] = m_ui->maxInputSpinBoxG;
	m_boxes_min[2] = m_ui->minInputSpinBoxB;
	m_boxes_max[2] = m_ui->maxInputSpinBoxB;
	m_boxes_min[3] = m_ui->minInputSpinBoxA;
	m_boxes_max[3] = m_ui->maxInputSpinBoxA;

	//populate combo boxes
	m_combos[0] = m_ui->channelComboR;
	m_combos[1] = m_ui->channelComboG;
	m_combos[2] = m_ui->channelComboB;
	m_combos[3] = m_ui->channelComboA;

	//populate labels
	m_labels_min[0] = m_ui->MinLabelR;
	m_labels_min[1] = m_ui->MinLabelG;
	m_labels_min[2] = m_ui->MinLabelB;
	m_labels_min[3] = m_ui->MinLabelA;
	m_labels_max[0] = m_ui->MaxLabelR;
	m_labels_max[1] = m_ui->MaxLabelG;
	m_labels_max[2] = m_ui->MaxLabelB;
	m_labels_max[3] = m_ui->MaxLabelA;
	m_reverse[0] = m_ui->reverseR;
	m_reverse[1] = m_ui->reverseG;
	m_reverse[2] = m_ui->reverseB;
	m_reverse[3] = m_ui->reverseA;

	m_ui->fixA->setChecked(true); //set alpha fixed to checked

	//create histograms
	QFrame* histoFrame[c_channelCount] = { m_ui->histoFrameR, m_ui->histoFrameG, m_ui->histoFrameB, m_ui->histoFrameA };
	for (unsigned i = 0; i < c_channelCount; i++)
	{
		m_histograms[i] = new ccHistogramWindow(this);
		m_histograms[i]->setRefreshAfterResize(false);
		auto layout = new QHBoxLayout;

		layout->setContentsMargins(0, 0, 0, 0);
		layout->addWidget(m_histograms[i]);

		histoFrame[i]->setLayout(layout);
	}

	//initialise colour scales
	for (unsigned i = 0; i < c_channelCount; i++)
	{
		m_colors[i] = ccColorScale::Shared(new ccColorScale(QString::asprintf("%d", i)));
	}
	if (m_cloud->getNumberOfScalarFields() > 0)
	{
		if (m_cloud->getCurrentDisplayedScalarFieldIndex() == -1)
		{
			m_cloud->setCurrentDisplayedScalarField(0);
		}
		ccScalarField* sf = static_cast<ccScalarField*>(m_cloud->getCurrentDisplayedScalarField());
		if (!sf) // I had this happen 1 time during testing but could never replicate
		{
			assert(false);
			ccLog::Error("[ccColorFromScalarDlg] Get current scalar field failed!");
			m_systemInvalid = true;
			disableAllButCancel();
		}
		else
		{
			m_storedOrigColorScale = sf->getColorScale();
			m_storedOrigSatRange = sf->saturationRange();
			m_storedOrigDisplayRange = sf->displayRange();

			for (unsigned i = 0; i < c_channelCount; i++)
			{
				m_scalars[i] = nullptr;
				m_prevFixed[i] = true;
				m_combos[i]->clear();
				for (unsigned int s = 0; s < m_cloud->getNumberOfScalarFields(); s++)
				{
					m_combos[i]->addItem(m_cloud->getScalarFieldName(s));
				}
				m_combos[i]->setCurrentIndex(m_cloud->getCurrentDisplayedScalarFieldIndex());
			}

			updateColormaps();

			//initialise histograms
			m_prevFixed[c_channelCount - 1] = false;
			updateChannel(0); //init first histogram
			for (unsigned i = 1; i < c_channelCount; i++) //copy data from this histogram into the next ones
			{
				m_scalars[i] = sf;
				setDefaultSatValuePerChannel(i);
				m_histograms[i]->fromBinArray(m_histograms[0]->histoValues(), m_histograms[0]->minVal(), m_histograms[0]->maxVal());
				updateHistogram(i);
			}

			sf->setColorScale(m_colors[c_channelCount - 1]); //set grey colour ramp to start with
			m_cloud->redrawDisplay();
		}
	}
	else
	{
		ccLog::Error("[ccColorFromScalarDlg] Current cloud has no scalar fields!");
		m_systemInvalid = true;
		disableAllButCancel();
	}
	
	//connect GUI elements
	connect(m_ui->channelComboR, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ccColorFromScalarDlg::onChannelChangedR);
	connect(m_ui->channelComboG, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ccColorFromScalarDlg::onChannelChangedG);
	connect(m_ui->channelComboB, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ccColorFromScalarDlg::onChannelChangedB);
	connect(m_ui->channelComboA, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ccColorFromScalarDlg::onChannelChangedA);

	connect(m_ui->buttonBox->button(QDialogButtonBox::Apply), &QPushButton::clicked, this, &ccColorFromScalarDlg::onApply);
	connect(m_ui->minInputSpinBoxR, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccColorFromScalarDlg::minSpinChangedR);
	connect(m_ui->maxInputSpinBoxR, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccColorFromScalarDlg::maxSpinChangedR);
	connect(m_ui->minInputSpinBoxG, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccColorFromScalarDlg::minSpinChangedG);
	connect(m_ui->maxInputSpinBoxG, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccColorFromScalarDlg::maxSpinChangedG);
	connect(m_ui->minInputSpinBoxB, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccColorFromScalarDlg::minSpinChangedB);
	connect(m_ui->maxInputSpinBoxB, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccColorFromScalarDlg::maxSpinChangedB);
	connect(m_ui->minInputSpinBoxA, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccColorFromScalarDlg::minSpinChangedA);
	connect(m_ui->maxInputSpinBoxA, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccColorFromScalarDlg::maxSpinChangedA);

	connect(m_ui->reverseR, &QCheckBox::stateChanged, this, &ccColorFromScalarDlg::toggleColors);
	connect(m_ui->reverseG, &QCheckBox::stateChanged, this, &ccColorFromScalarDlg::toggleColors);
	connect(m_ui->reverseB, &QCheckBox::stateChanged, this, &ccColorFromScalarDlg::toggleColors);
	connect(m_ui->reverseA, &QCheckBox::stateChanged, this, &ccColorFromScalarDlg::toggleColors);
	connect(m_ui->toggleHSV, &QRadioButton::toggled, this, &ccColorFromScalarDlg::toggleColorMode);
	connect(m_ui->toggleRGB, &QRadioButton::toggled, this, &ccColorFromScalarDlg::toggleColorMode);

	connect(m_ui->fixR, &QCheckBox::stateChanged, this, &ccColorFromScalarDlg::toggleFixedR);
	connect(m_ui->fixG, &QCheckBox::stateChanged, this, &ccColorFromScalarDlg::toggleFixedG);
	connect(m_ui->fixB, &QCheckBox::stateChanged, this, &ccColorFromScalarDlg::toggleFixedB);
	connect(m_ui->fixA, &QCheckBox::stateChanged, this, &ccColorFromScalarDlg::toggleFixedA);

	//connect histogram events
	connect(m_histograms[0], &ccHistogramWindow::sfMinSatValChanged, this, &ccColorFromScalarDlg::minChangedR);
	connect(m_histograms[0], &ccHistogramWindow::sfMaxSatValChanged, this, &ccColorFromScalarDlg::maxChangedR);
	connect(m_histograms[1], &ccHistogramWindow::sfMinSatValChanged, this, &ccColorFromScalarDlg::minChangedG);
	connect(m_histograms[1], &ccHistogramWindow::sfMaxSatValChanged, this, &ccColorFromScalarDlg::maxChangedG);
	connect(m_histograms[2], &ccHistogramWindow::sfMinSatValChanged, this, &ccColorFromScalarDlg::minChangedB);
	connect(m_histograms[2], &ccHistogramWindow::sfMaxSatValChanged, this, &ccColorFromScalarDlg::maxChangedB);
	connect(m_histograms[3], &ccHistogramWindow::sfMinSatValChanged, this, &ccColorFromScalarDlg::minChangedA);
	connect(m_histograms[3], &ccHistogramWindow::sfMaxSatValChanged, this, &ccColorFromScalarDlg::maxChangedA);

	for (int i = 1; i < c_channelCount; ++i)
	{
		updateChannel(i);
	}
}

ccColorFromScalarDlg::~ccColorFromScalarDlg()
{
	if (!m_systemInvalid)
	{
		ccScalarField* sf = static_cast<ccScalarField*>(m_cloud->getCurrentDisplayedScalarField());
		sf->setColorScale(m_storedOrigColorScale);
		sf->setSaturationStart(m_storedOrigSatRange.min());
		sf->setSaturationStop(m_storedOrigSatRange.max());
		sf->setMinDisplayed(m_storedOrigDisplayRange.min());
		sf->setMaxDisplayed(m_storedOrigDisplayRange.max());
		m_cloud->redrawDisplay();
	}
	delete m_ui;
}

void ccColorFromScalarDlg::updateColormaps()
{
	if (!m_systemInvalid)
	{
		//check for reversed
		bool reversed[c_channelCount] = { m_ui->reverseR->isChecked(), m_ui->reverseG->isChecked(), m_ui->reverseB->isChecked(), m_ui->reverseA->isChecked() };

		//create colourmaps for RGB
		if (m_ui->toggleRGB->isChecked())
		{
			//update labels
			m_ui->mRedLabel->setText(QStringLiteral("Red"));
			m_ui->mGreenLabel->setText(QStringLiteral("Green"));
			m_ui->mBlueLabel->setText(QStringLiteral("Blue"));
			m_ui->mAlphaLabel->setText(QStringLiteral("Alpha"));

			//populate colour ramps
			Qt::GlobalColor start_colors[c_channelCount] = { Qt::black , Qt::black , Qt::black , Qt::black };
			Qt::GlobalColor end_colors[c_channelCount] = { Qt::red , Qt::green , Qt::blue , Qt::white };

			for (unsigned i = 0; i < c_channelCount; i++)
			{
				m_colors[i]->clear();
				if (reversed[i]) //flip
				{
					m_colors[i]->insert(ccColorScaleElement(0.0, end_colors[i]));
					m_colors[i]->insert(ccColorScaleElement(1.0, start_colors[i]));
				}
				else
				{
					m_colors[i]->insert(ccColorScaleElement(0.0, start_colors[i]));
					m_colors[i]->insert(ccColorScaleElement(1.0, end_colors[i]));
				}

				m_colors[i]->update();
			}
		}
		else //create colourmaps for HSV
		{
			//update labels
			m_ui->mRedLabel->setText(QStringLiteral("Hue"));
			m_ui->mGreenLabel->setText(QStringLiteral("Sat"));
			m_ui->mBlueLabel->setText(QStringLiteral("Value"));
			m_ui->mAlphaLabel->setText(QStringLiteral("Alpha"));

			//populate colour ramps
			Qt::GlobalColor start_colors[c_channelCount] = { Qt::black , Qt::gray , Qt::black , Qt::black };
			Qt::GlobalColor end_colors[c_channelCount] = { Qt::red , Qt::green , Qt::white , Qt::white };

			for (unsigned i = 0; i < c_channelCount; i++)
			{
				m_colors[i]->clear();
				if (reversed[i]) //flip
				{
					m_colors[i]->insert(ccColorScaleElement(0.0, end_colors[i]));
					m_colors[i]->insert(ccColorScaleElement(1.0, start_colors[i]));
				}
				else
				{
					m_colors[i]->insert(ccColorScaleElement(0.0, start_colors[i]));
					m_colors[i]->insert(ccColorScaleElement(1.0, end_colors[i]));
				}

				m_colors[i]->update();
			}

			//overwrite first colourmap with hue rainbow
			int hue;
			m_colors[0]->clear();

			ccColor::Rgb col = ccColor::Convert::hsv2rgb(360.0f, 1.0f, 1.0f);
			m_colors[0]->insert(ccColorScaleElement(1.0, QColor(col.r, col.g, col.b, 255)));
			for (unsigned i = 0; i < 360; i += 2)
			{
				//calculate hue value
				if (reversed[0])
				{
					hue = 360 - i;
				}
				else
				{
					hue = i;
				}

				//calculate colour
				col = ccColor::Convert::hsv2rgb(hue, 1.0f, 1.0f);

				//add stop
				m_colors[0]->insert(ccColorScaleElement(static_cast<double>(i) / 360.0, QColor(col.r, col.g, col.b, 255)));
			}
			m_colors[0]->update();
		}
	}
}

void ccColorFromScalarDlg::toggleColors(int state)
{
	if (!m_systemInvalid)
	{
		//update colourmaps
		updateColormaps();

		//refresh histograms
		refreshDisplay();
	}
}

void ccColorFromScalarDlg::toggleColorMode(bool state)
{
	if (!m_systemInvalid)
	{
		//update colourmaps
		updateColormaps();
		refreshDisplay();
	}
}

// update/redraw histograms but don't reset postion of UI elements/sliders etc. 
void ccColorFromScalarDlg::refreshDisplay()
{
	if (!m_systemInvalid)
	{
		// refresh histograms
		for (int i = 0; i < c_channelCount; i++)
		{
			updateHistogram(i);
		}
	}
}


void ccColorFromScalarDlg::updateHistogram(int n)
{
	if (!m_systemInvalid)
	{
		if (n >= c_channelCount || n < 0)
		{
			ccLog::Error("[ccColorFromScalarDlg] updateHistogram called with an invalid channel index");
			return;
		}
		//first check if fixed or not and enable disable ui features based on this
		bool fixed[c_channelCount] = { m_ui->fixR->isChecked(), m_ui->fixG->isChecked(), m_ui->fixB->isChecked(), m_ui->fixA->isChecked() };
		m_histograms[n]->setEnabled(!fixed[n]);
		m_combos[n]->setEnabled(!fixed[n]);
		m_boxes_max[n]->setEnabled(!fixed[n]);
		m_labels_max[n]->setEnabled(!fixed[n]);
		m_reverse[n]->setEnabled(!fixed[n]);
		if (fixed[n]) //this channel is/has been fixed
		{
			if (fixed[n] != m_prevFixed[n])
			{
				m_labels_min[n]->setText("  Value:");
				m_boxes_min[n]->setMaximum(255.0);
				m_boxes_min[n]->setMinimum(0.0);
				m_boxes_max[n]->setMaximum(255.0);
				m_boxes_max[n]->setMinimum(0.0);
				m_boxes_max[n]->setValue(200.0);
				m_boxes_min[n]->setValue(200.0);
				m_boxes_min[n]->setSingleStep(1.0);
				m_boxes_max[n]->setSingleStep(1.0);

				//edge case for HSV values (0 - 360)
				if (n == 0 && m_ui->toggleHSV->isChecked())
				{
					m_boxes_min[n]->setMaximum(360);
				}
			}

			//and make histogram grey
			m_histograms[n]->clear();
			m_histograms[n]->setSFInteractionMode(ccHistogramWindow::SFInteractionMode::None); //disable interactivity
			m_histograms[n]->setAxisLabels("", "");
			m_histograms[n]->setAxisDisplayOption(ccHistogramWindow::AxisDisplayOption::None); //only display XAxis
			m_histograms[n]->refresh();
			m_histograms[n]->replot();
		}
		else
		{
			if (fixed[n] != m_prevFixed[n])
			{
				setDefaultSatValuePerChannel(n); //update slider positions
				m_labels_min[n]->setText("Minimum:"); //ensure label text is correct
			}

			//set scalar field
			m_scalars[n]->setColorScale(m_colors[n]);
			m_scalars[n]->setSaturationStart(m_boxes_min[n]->value());
			m_scalars[n]->setSaturationStop(m_boxes_max[n]->value());
			m_histograms[n]->setSFInteractionMode(ccHistogramWindow::SFInteractionMode::SaturationRange); //disable interactivity
			m_histograms[n]->setAxisLabels("", "");
			m_histograms[n]->setAxisDisplayOption(ccHistogramWindow::AxisDisplayOption::XAxis); //only display XAxis
			m_histograms[n]->refresh();
		}
		m_prevFixed[n] = fixed[n];
	}
}

void ccColorFromScalarDlg::updateSpinBoxLimits(int n)
{
	if (!m_systemInvalid)
	{
		if (n >= c_channelCount || n < 0)
		{
			ccLog::Error("[ccColorFromScalarDlg] updateSpinBoxLimits called with an invalid channel index");
			return;
		}
		bool fixed[c_channelCount] = { m_ui->fixR->isChecked(), m_ui->fixG->isChecked(), m_ui->fixB->isChecked(), m_ui->fixA->isChecked() };
		if (fixed[n])
		{
			return;
		}
		ccScalarField* sf = static_cast<ccScalarField*>(m_cloud->getScalarField(m_combos[n]->currentIndex()));
		if (sf)
		{
			m_minSat[n] = sf->getMin();
			m_maxSat[n] = sf->getMax();
			double singleStepSize = (m_maxSat[n] - m_minSat[n]) / 100.0;
			if (singleStepSize < 0.01)
			{
				singleStepSize = 0.01;
			}
			m_boxes_min[n]->setMinimum(m_minSat[n]);
			m_boxes_min[n]->setMaximum(m_maxSat[n]);
			m_boxes_min[n]->setSingleStep(singleStepSize);
			m_boxes_max[n]->setMinimum(m_minSat[n]);
			m_boxes_max[n]->setMaximum(m_maxSat[n]);
			m_boxes_max[n]->setSingleStep(singleStepSize);
			m_boxes_max[n]->setCorrectionMode(QAbstractSpinBox::CorrectionMode::CorrectToNearestValue);
		}
	}
}

void ccColorFromScalarDlg::updateChannel(int n)
{
	if (!m_systemInvalid)
	{
		if (n >= c_channelCount || n < 0)
		{
			ccLog::Error("[ccColorFromScalarDlg] updateChannel called with an invalid channel index");
			return;
		}

		ccScalarField* sf = static_cast<ccScalarField*>(m_cloud->getScalarField(m_combos[n]->currentIndex()));
		if (sf)
		{
			m_scalars[n] = sf;
			setDefaultSatValuePerChannel(n);
			m_histograms[n]->clear(); //clear last histogram
			m_histograms[n]->fromSF(m_scalars[n], 255, false, true); //generate new one
			updateHistogram(n); //update plotting etc.
		}
	}
}

void ccColorFromScalarDlg::setDefaultSatValuePerChannel(int n)
{
	if (!m_systemInvalid)
	{
		if (n >= c_channelCount || n < 0)
		{
			ccLog::Error("[ccColorFromScalarDlg] setDefaultSatValuePerChannel called with an invalid channel index");
			return;
		}
		//set default stretch (n.b. this is a cheap hack to avoid calculating percentiles [by assuming uniform data distribution])
		m_scalars[n]->setColorScale(m_colors[n]);
		m_minSat[n] = m_scalars[n]->getMin();
		m_maxSat[n] = m_scalars[n]->getMax();
		updateSpinBoxLimits(n);
		ScalarType range = m_maxSat[n] - m_minSat[n];
		m_histograms[n]->setMinSatValue(m_minSat[n] + 0.1 * range);
		m_boxes_min[n]->setValue(m_minSat[n] + 0.1 * range);
		m_histograms[n]->setMaxSatValue(m_maxSat[n] - 0.1 * range);
		m_boxes_max[n]->setValue(m_maxSat[n] - 0.1 * range);
	}
}

void ccColorFromScalarDlg::resizeEvent(QResizeEvent* event)
{
	refreshDisplay();
}

//mapping ranges changed
void ccColorFromScalarDlg::minChanged(int n, double val, bool slider)
{
	if (n >= c_channelCount || n < 0)
	{
		ccLog::Error("[ccColorFromScalarDlg] minChanged called with an invalid channel index");
		return;
	}
	if (val <= m_maxSat[n]) //valid value?
	{
		m_scalars[n]->setColorScale(m_colors[n]);
		m_minSat[n] = val;
		if (slider) //change was made with the slider
		{
			m_boxes_min[n]->setValue(val);
		}
		else //change was made with text box
		{
			m_histograms[n]->setMinSatValue(val);
		}
	}
	else
	{
		//ccLog::Warning(QString("minChanged Channel %1 failed New min > max, val = %2, maxval = %3").arg(n).arg(val).arg(m_maxSat[n]));
	}
}

void ccColorFromScalarDlg::maxChanged(int n, double val, bool slider)
{
	if (n >= c_channelCount || n < 0)
	{
		ccLog::Error("[ccColorFromScalarDlg] maxChanged called with an invalid channel index");
		return;
	}
	if (val >= m_minSat[n]) //valid value?
	{
		m_scalars[n]->setColorScale(m_colors[n]);
		m_maxSat[n] = val;
		if (slider) //change was made with the slider
		{
			m_boxes_max[n]->setValue(val);
		}
		else //change was made with text box
		{
			m_histograms[n]->setMaxSatValue(val);
		}
	}
	else
	{
		//ccLog::Warning(QString("maxChanged Channel %1 failed New max < min, val = %2, minval = %3").arg(n).arg(val).arg(m_minSat[n]));
	}
}

void ccColorFromScalarDlg::onApply()
{
	if (!m_systemInvalid)
	{
		if (!m_cloud->hasColors())
		{
			m_cloud->resizeTheRGBTable(false);
		}

		//which maps to flip?
		bool reversed[c_channelCount] = { m_ui->reverseR->isChecked(), m_ui->reverseG->isChecked(), m_ui->reverseB->isChecked(), m_ui->reverseA->isChecked() };

		//and which are fixed?
		bool fixed[c_channelCount] = { m_ui->fixR->isChecked(), m_ui->fixG->isChecked(), m_ui->fixB->isChecked(), m_ui->fixA->isChecked() };

		//map scalar values to RGB
		if (m_ui->toggleRGB->isChecked())
		{
			int col[c_channelCount];
			for (unsigned p = 0; p < m_cloud->size(); p++)
			{
				//get col
				for (unsigned i = 0; i < c_channelCount; i++)
				{
					if (fixed[i]) //fixed value
					{
						col[i] = static_cast<int>(m_boxes_min[i]->value());
					}
					else //map from scalar
					{
						col[i] = static_cast<int>(255.0 * (m_scalars[i]->getValue(p) - m_minSat[i]) / (m_maxSat[i] - m_minSat[i]));
					}

					//trim to range 0 - 255
					col[i] = std::max(col[i], 0);
					col[i] = std::min(col[i], 255);

					//flip?
					if (reversed[i] && !fixed[i])
					{
						col[i] = 255 - col[i];
					}
				}
				m_cloud->setPointColor(p, ccColor::FromQColora(QColor(col[0], col[1], col[2], col[3])));
			}
		}
		else //map scalar values to HSV (and then to RGB)
		{
			float col[c_channelCount];
			for (unsigned p = 0; p < m_cloud->size(); p++)
			{
				//get col
				for (unsigned i = 0; i < c_channelCount; i++)
				{
					if (fixed[i]) //fixed value
					{
						col[i] = m_boxes_min[i]->value() / m_boxes_min[i]->maximum(); //n.b. most 'fixed' boxes between 0 - 255, but hue between 0 and 360.
					}
					else //map from scalar
					{
						col[i] = (m_scalars[i]->getValue(p) - m_minSat[i]) / (m_maxSat[i] - m_minSat[i]);
					}

					//trim to range 0 - 1
					col[i] = std::max(col[i], 0.0f);
					col[i] = std::min(col[i], 1.0f);

					//flip?
					if (reversed[i] && !fixed[i])
					{
						col[i] = 1.0f - col[i];
					}
				}

				//calculate and set colour
				ccColor::Rgb rgb = ccColor::Convert::hsv2rgb(col[0] * 360.0f, col[1], col[2]);
				m_cloud->setPointColor(p, ccColor::Rgba(rgb, static_cast<int>(col[3] * ccColor::MAX)));
			}
		}
		m_cloud->colorsHaveChanged();
		m_cloud->showSF(false);
		m_cloud->showColors(true);
		m_cloud->redrawDisplay();
	}
}

void ccColorFromScalarDlg::disableAllButCancel()
{
	for (unsigned n = 0; n < c_channelCount; n++)
	{
		m_histograms[n]->setEnabled(false);
		m_combos[n]->setEnabled(false);
		m_boxes_min[n]->setEnabled(false);
		m_boxes_max[n]->setEnabled(false);
		m_labels_min[n]->setEnabled(false);
		m_labels_max[n]->setEnabled(false);
		m_reverse[n]->setEnabled(false);
	}
	m_ui->toggleHSV->setEnabled(false);
	m_ui->toggleRGB->setEnabled(false);
	m_ui->fixR->setEnabled(false);
	m_ui->fixG->setEnabled(false);
	m_ui->fixB->setEnabled(false);
	m_ui->fixA->setEnabled(false);
	m_ui->buttonBox->button(QDialogButtonBox::Apply)->setEnabled(false);
}