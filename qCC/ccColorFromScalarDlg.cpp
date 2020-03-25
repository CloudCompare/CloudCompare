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

	//create colour scales
	m_red = ccColorScale::Shared(new ccColorScale("Reds"));
	m_red->insert(ccColorScaleElement(0.0, Qt::black));
	m_red->insert(ccColorScaleElement(1.0, Qt::red));

	m_green = ccColorScale::Shared(new ccColorScale("Green"));
	m_green->insert(ccColorScaleElement(0.0, Qt::black));
	m_green->insert(ccColorScaleElement(1.0, Qt::green));

	m_blue = ccColorScale::Shared(new ccColorScale("Blues"));
	m_blue->insert(ccColorScaleElement(0.0, Qt::black));
	m_blue->insert(ccColorScaleElement(1.0, Qt::blue));

	//default colors (RGB mode)
	m_colors[0] = m_red;
	m_colors[1] = m_green;
	m_colors[2] = m_blue;

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
	connect(m_histograms[0], &ccHistogramWindow::sfMinSatValChanged, this, &ccColorFromScalarDlg::minChangedR);
	connect(m_histograms[0], &ccHistogramWindow::sfMaxSatValChanged, this, &ccColorFromScalarDlg::maxChangedR);
	connect(m_histograms[1], &ccHistogramWindow::sfMinSatValChanged, this, &ccColorFromScalarDlg::minChangedG);
	connect(m_histograms[1], &ccHistogramWindow::sfMaxSatValChanged, this, &ccColorFromScalarDlg::maxChangedG);
	connect(m_histograms[2], &ccHistogramWindow::sfMinSatValChanged, this, &ccColorFromScalarDlg::minChangedB);
	connect(m_histograms[2], &ccHistogramWindow::sfMaxSatValChanged, this, &ccColorFromScalarDlg::maxChangedB);

	//update histograms
	onChannelChangedR(0);
	onChannelChangedG(1);
	onChannelChangedB(2);

	toggleHSV->setEnabled(false); //disable HSV for now (until I implement it...)
}

void ccColorFromScalarDlg::updateHistogram(int n)
{
	//clear and build histogram
	m_histograms[n]->clear();
	m_histograms[n]->fromSF(m_scalars[n], 255, false, true);
	m_histograms[n]->enableSFInteractionMode(true); //enable interactivity
	m_histograms[n]->refresh();

	//hide axis (not sure why this code HAS to be here, but it does...)
	m_histograms[n]->setAxisLabels("", "");
	m_histograms[n]->yAxis->setVisible(false); //disable y-axis
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

void ccColorFromScalarDlg::onChannelChangedR(int index)
{
	updateChannel(0);
}

void ccColorFromScalarDlg::onChannelChangedG(int index)
{
	updateChannel(1);
}

void ccColorFromScalarDlg::onChannelChangedB(int index)
{
	updateChannel(2);
}

void ccColorFromScalarDlg::onApply()
{
	
}
