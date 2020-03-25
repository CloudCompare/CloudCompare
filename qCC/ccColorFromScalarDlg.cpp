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

#include "ccColorFromScalarDlg.h"

//local
#include "ccHistogramWindow.h"
//#include "ccGenericPointCloud.h"
#include "ccConsole.h"

//qCC_db
#include <ccGenericGLDisplay.h>
#include <ccPointCloud.h>
#include <ccHObjectCaster.h>
#include <ccColorScale.h>

//Qt
#include <QPushButton>

//system
#include <string.h>
#include <assert.h>

//persistent parameters
static int s_inputLevels[2] = {0,255};
static int s_outputLevels[2] = {0,255};
static bool s_outputLevelsEnabled = false;

ccColorFromScalarDlg::ccColorFromScalarDlg(QWidget* parent, ccPointCloud* pointCloud)
	: QDialog(parent, Qt::Tool)
	, Ui::ColorFromScalarDialog()
	, m_cloud(pointCloud)
{
	setupUi(this);

	//create histograms
	QFrame* histoFrame[3] = { histoFrameR, histoFrameG, histoFrameB };
	for (unsigned i = 0; i < 3; i++)
	{
		m_histograms[i] = new ccHistogramWindow(this);
		histoFrame[i]->setLayout(new QHBoxLayout());
		histoFrame[i]->setLayout(new QHBoxLayout());
		histoFrame[i]->layout()->addWidget(m_histograms[i]);
	}

	//populate boxes
	m_boxes_min[0] = minInputSpinBoxR;
	m_boxes_max[0] = maxInputSpinBoxR;
	m_boxes_min[1] = minInputSpinBoxG;
	m_boxes_max[1] = maxInputSpinBoxG;
	m_boxes_min[2] = minInputSpinBoxB;
	m_boxes_max[2] = maxInputSpinBoxB;

	//populate combo boxes
	m_combos[0] = channelComboR;
	m_combos[1] = channelComboG;
	m_combos[2] = channelComboB;
	for (unsigned i = 0; i < 3; i++)
	{
		m_combos[i]->clear();
		for (unsigned int s = 0; s < m_cloud->getNumberOfScalarFields(); s++)
		{
			m_combos[i]->addItem(m_cloud->getScalarFieldName(s));
		}
		m_combos[i]->setCurrentIndex( m_cloud->getCurrentDisplayedScalarFieldIndex() );

	}

	//connect GUI elements
	connect(channelComboR, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ccColorFromScalarDlg::onChannelChangedR);
	connect(channelComboG, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ccColorFromScalarDlg::onChannelChangedG);
	connect(channelComboB, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ccColorFromScalarDlg::onChannelChangedB);
	connect(buttonBox->button(QDialogButtonBox::Apply), &QPushButton::clicked, this, &ccColorFromScalarDlg::onApply);
	connect(minInputSpinBoxR, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccColorFromScalarDlg::minSpinChangedR);
	connect(maxInputSpinBoxR, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccColorFromScalarDlg::maxSpinChangedR);
	connect(minInputSpinBoxG, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccColorFromScalarDlg::minSpinChangedG);
	connect(maxInputSpinBoxG, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccColorFromScalarDlg::maxSpinChangedG);
	connect(minInputSpinBoxB, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccColorFromScalarDlg::minSpinChangedB);
	connect(maxInputSpinBoxB, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccColorFromScalarDlg::maxSpinChangedB);
	connect(reverseR, &QCheckBox::stateChanged, this, &ccColorFromScalarDlg::toggleColors);
	connect(reverseG, &QCheckBox::stateChanged, this, &ccColorFromScalarDlg::toggleColors);
	connect(reverseB, &QCheckBox::stateChanged, this, &ccColorFromScalarDlg::toggleColors);
	
	//connect histogram events
	connect(m_histograms[0], &ccHistogramWindow::sfMinSatValChanged, this, &ccColorFromScalarDlg::minChangedR);
	connect(m_histograms[0], &ccHistogramWindow::sfMaxSatValChanged, this, &ccColorFromScalarDlg::maxChangedR);
	connect(m_histograms[1], &ccHistogramWindow::sfMinSatValChanged, this, &ccColorFromScalarDlg::minChangedG);
	connect(m_histograms[1], &ccHistogramWindow::sfMaxSatValChanged, this, &ccColorFromScalarDlg::maxChangedG);
	connect(m_histograms[2], &ccHistogramWindow::sfMinSatValChanged, this, &ccColorFromScalarDlg::minChangedB);
	connect(m_histograms[2], &ccHistogramWindow::sfMaxSatValChanged, this, &ccColorFromScalarDlg::maxChangedB);
	
	//initialise colour scales
	for (unsigned i = 0; i < 4; i++)
	{
		m_colors[i] = ccColorScale::Shared(new ccColorScale(QString::asprintf("%d", i)));
	}
	updateColormaps();

	//update histograms
	onChannelChangedR(0);
	onChannelChangedG(1);
	onChannelChangedB(2);
	m_cloud->getCurrentDisplayedScalarField()->setColorScale(m_colors[3]); //set grey colour ramp to start with

	toggleHSV->setEnabled(false); //disable HSV for now (until I implement it...)
}

void ccColorFromScalarDlg::updateColormaps()
{
	//create colourmaps for RGB
	if (toggleRGB->isChecked())
	{

		//populate colour ramps
		Qt::GlobalColor start_colors[4] = { Qt::black , Qt::black , Qt::black , Qt::black };
		Qt::GlobalColor end_colors[4] = { Qt::red , Qt::green , Qt::blue , Qt::white };
		bool reversed[4] = { reverseR->isChecked(), reverseG->isChecked(), reverseB->isChecked(), false };
		for (unsigned i = 0; i < 4; i++)
		{
			m_colors[i]->clear();
			if (reversed[i]) //flip
			{
				m_colors[i]->insert(ccColorScaleElement(0.0, end_colors[i]));
				m_colors[i]->insert(ccColorScaleElement(1.0, start_colors[i]));
			} else
			{
				m_colors[i]->insert(ccColorScaleElement(0.0, start_colors[i]));
				m_colors[i]->insert(ccColorScaleElement(1.0, end_colors[i]));
			}
			
			m_colors[i]->update();
		}		
	}
	else //create colourmaps for HSV
	{
		//TODO
	}

}

void ccColorFromScalarDlg::toggleColors(int state)
{
	//update colourmaps
	updateColormaps();
	
	//update histograms
	updateHistogram(0); 
	updateHistogram(1); 
	updateHistogram(2); 
}

void ccColorFromScalarDlg::updateHistogram(int n)
{
	//set scalar field
	m_scalars[n]->setColorScale(m_colors[n]);

	//clear and build histogram
	m_histograms[n]->clear();
	m_histograms[n]->fromSF(m_scalars[n], 255, false, true);
	m_histograms[n]->enableSFInteractionMode(true); //enable interactivity
	m_histograms[n]->refresh();

	//hide axis (not sure why this code HAS to be here, but it does...)
	m_histograms[n]->setAxisLabels("", "");
	m_histograms[n]->yAxis->setVisible(false); //disable y-axis

	m_histograms[n]->repaint();
}

void ccColorFromScalarDlg::updateChannel(int n)
{
	ccScalarField* sf = static_cast<ccScalarField*>(m_cloud->getScalarField(m_combos[n]->currentIndex()));
	m_scalars[n] = sf;
	if (sf)
	{
		
		sf->computeMinAndMax();
		sf->setColorScale(m_colors[n]);

		m_minSat[n] = sf->getMin();
		m_maxSat[n] = sf->getMax();
		m_boxes_min[n]->setMinimum(m_minSat[n]);
		m_boxes_min[n]->setMaximum(m_maxSat[n]);
		m_boxes_max[n]->setMinimum(m_minSat[n]);
		m_boxes_max[n]->setMaximum(m_maxSat[n]);

		updateHistogram(n);

		//set default stretch (n.b. this is a cheap hack to avoid calculating percentiles [by assuming uniform data distribution])
		float range = m_maxSat[n] - m_minSat[n];
		m_histograms[n]->setMinSatValue(m_minSat[n] + 0.1 * range);
		m_histograms[n]->setMaxSatValue(m_maxSat[n] - 0.1 * range);

	}
}

//mapping ranges changed
void ccColorFromScalarDlg::minChanged(int n, double val)
{
	if (val < m_maxSat[n]) //valid value?
	{
		m_scalars[n]->setColorScale(m_colors[n]);
		m_minSat[n] = val;
		m_boxes_min[n]->setValue(val);
		m_histograms[n]->setMinSatValue(val);
	}
}

void ccColorFromScalarDlg::maxChanged(int n, double val)
{
	if (val > m_minSat[n]) //valid value?
	{
		m_scalars[n]->setColorScale(m_colors[n]);
		m_maxSat[n] = val;
		m_boxes_max[n]->setValue(val);
		m_histograms[n]->setMaxSatValue(val);
	}
}


void ccColorFromScalarDlg::onApply()
{

	//which maps to flip?
	bool reversed[4] = { reverseR->isChecked(), reverseG->isChecked(), reverseB->isChecked() };

	//map scalar values to RGB
	if (toggleRGB->isChecked())
	{
		int col[3];
		for (unsigned p = 0; p < m_cloud->size(); p++)
		{
			//get col
			for (unsigned i = 0; i < 3; i++)
			{
				col[i] = int( 255.0 * (m_scalars[i]->getValue(p) - m_minSat[i]) / (m_maxSat[i]-m_minSat[i]) );
				
				//trim to range 0 - 255
				col[i] = std::max(col[i], 0);
				col[i] = std::min(col[i], 255);

				//flip?
				if (reversed[i])
				{
					col[i] = 255 - col[i];
				}
			}
			m_cloud->setPointColor(p, ccColor::FromQColor(QColor(col[0], col[1], col[2], 255)));
		}
	}
	else //map scalar values to HSV (and then to RGB)
	{
		//TODO
	}
	
	m_cloud->colorsHaveChanged();
	m_cloud->showSF(false);
	m_cloud->showColors(true);
	m_cloud->redrawDisplay();
	m_cloud->prepareDisplayForRefresh();
	m_cloud->refreshDisplay();
	
}
