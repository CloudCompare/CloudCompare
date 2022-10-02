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

#include "ccColorLevelsDlg.h"

//local
#include "ccHistogramWindow.h"
#include "ccGenericPointCloud.h"

//qCC_db
#include <ccGenericGLDisplay.h>
#include <ccPointCloud.h>
#include <ccHObjectCaster.h>

//Qt
#include <QPushButton>

//system
#include <string.h>
#include <assert.h>

//persistent parameters
static int s_inputLevels[2] = {0,255};
static int s_outputLevels[2] = {0,255};
static bool s_outputLevelsEnabled = false;

ccColorLevelsDlg::ccColorLevelsDlg(QWidget* parent, ccGenericPointCloud* pointCloud)
	: QDialog(parent, Qt::Tool)
	, Ui::ColorLevelsDialog()
	, m_histogram(nullptr)
	, m_cloud(pointCloud)
{
	setupUi(this);

	//connect GUI elements
	connect(channelComboBox, qOverload<int>(&QComboBox::currentIndexChanged),	this,	&ccColorLevelsDlg::onChannelChanged);
	connect(buttonBox->button(QDialogButtonBox::Apply), &QPushButton::clicked,	this,	&ccColorLevelsDlg::onApply);

	//create histogram view
	m_histogram = new ccHistogramWindow(this);
	{
		m_histogram->setColorScheme(ccHistogramWindow::USE_SOLID_COLOR);
		m_histogram->setSolidColor(Qt::black);
		//add view
		histoFrame->setLayout(new QHBoxLayout());
		histoFrame->layout()->addWidget(m_histogram);		
	}

	//restore previous parameters
	minInputSpinBox->setValue(s_inputLevels[0]);
	maxInputSpinBox->setValue(s_inputLevels[1]);
	minOutputSpinBox->setValue(s_outputLevels[0]);
	maxOutputSpinBox->setValue(s_outputLevels[1]);
	outputLevelsCheckBox->setChecked(s_outputLevelsEnabled);

	updateHistogram();
}

void ccColorLevelsDlg::updateHistogram()
{
	if (m_histogram)
	{
		unsigned pointCount = (m_cloud ? m_cloud->size() : 0);
		if (pointCount == 0)
		{
			//nothing to do
			m_histogram->clear();
			return;
		}

		std::vector<unsigned> histoValues[3];
		try
		{
			for (int i = 0; i < 3; ++i)
			{
				if (	channelComboBox->currentIndex() == RGB
					||	channelComboBox->currentIndex() == i + 1)
				{
					histoValues[i].resize(1 << (sizeof(ColorCompType) * 8), 0);
				}
			}
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			m_histogram->clear();
			return;
		}

		std::vector<unsigned>* histoValuesR = (histoValues[0].empty() ? nullptr : histoValues);
		std::vector<unsigned>* histoValuesG = (histoValues[1].empty() ? nullptr : histoValues + 1);
		std::vector<unsigned>* histoValuesB = (histoValues[2].empty() ? nullptr : histoValues + 2);

		switch (channelComboBox->currentIndex())
		{
		case RGB:
			m_histogram->setSolidColor(Qt::black);
			m_histogram->setAxisLabels("R,G,B", QString());
			//test: for now we send all data into the same histogram!
			histoValuesG = histoValuesR;
			histoValuesB = histoValuesR;
			break;
		case RED:
			m_histogram->setSolidColor(Qt::red);
			m_histogram->setAxisLabels("Red", QString());
			break;
		case GREEN:
			m_histogram->setSolidColor(Qt::green);
			m_histogram->setAxisLabels("Green", QString());
			break;
		case BLUE:
			m_histogram->setSolidColor(Qt::blue);
			m_histogram->setAxisLabels("Blue", QString());
			break;
		}

		//project points
		{
			for (unsigned i = 0; i < pointCount; ++i)
			{
				const ccColor::Rgba& col = m_cloud->getPointColor(i);
				if (histoValuesR)
					histoValuesR->at(col.r)++;
				if (histoValuesG)
					histoValuesG->at(col.g)++;
				if (histoValuesB)
					histoValuesB->at(col.b)++;
			}
		}

		for (int i = 0; i < 3; ++i)
		{
			if (channelComboBox->currentIndex() == RGB || channelComboBox->currentIndex() == i + 1)
			{
				m_histogram->fromBinArray(histoValues[i], 0.0, 256.0);
				break;
			}
		}
		m_histogram->refresh();
	}
}

void ccColorLevelsDlg::onChannelChanged(int channel)
{
	updateHistogram();
}

void ccColorLevelsDlg::onApply()
{
	//save parameters
	s_inputLevels[0] = minInputSpinBox->value();
	s_inputLevels[1] = maxInputSpinBox->value();
	s_outputLevels[0] = minOutputSpinBox->value();
	s_outputLevels[1] = maxOutputSpinBox->value();
	s_outputLevelsEnabled = outputLevelsCheckBox->isChecked();

	if (	m_cloud
		&&	(	minInputSpinBox->value() != 0
			||	maxInputSpinBox->value() != 255
			||	minOutputSpinBox->value() != 0
			||	maxOutputSpinBox->value() != 255
			) )
	{

		bool applyRGB[3] = {	channelComboBox->currentIndex() == RGB || channelComboBox->currentIndex() == RED,
								channelComboBox->currentIndex() == RGB || channelComboBox->currentIndex() == GREEN,
								channelComboBox->currentIndex() == RGB || channelComboBox->currentIndex() == BLUE };

		ScaleColorFields(	m_cloud,
							s_inputLevels[0],
							s_inputLevels[1],
							s_outputLevels[0],
							s_outputLevels[1],
							applyRGB );

		//update display
		m_cloud->getDisplay()->redraw();

		//update histogram
		onChannelChanged(channelComboBox->currentIndex());
	}

	//after applying the filter we reset the boundaries to (0,255)
	//in case the user clicks multiple times on the "Apply" button!
	minInputSpinBox->setValue(0);
	maxInputSpinBox->setValue(255);
	minOutputSpinBox->setValue(0);
	maxOutputSpinBox->setValue(255);
}

bool ccColorLevelsDlg::ScaleColorFields(ccGenericPointCloud* cloud, int inputLevelMin, int inputLevelMax, int outputLevelMin, int outputLevelMax, const bool applyRGB[3])
{
	if (!cloud)
	{
		assert(false);
		return false;
	}
	if (!cloud->hasColors())
	{
		ccLog::Warning("(ccColorLevelsDlg::ScaleColorFields] Cloud has no colors");
		return false;
	}

	ccPointCloud* pc = ccHObjectCaster::ToPointCloud(cloud);

	unsigned pointCount = cloud->size();
	int qIn = inputLevelMax - inputLevelMin;
	if (qIn == 0)
	{
		ccLog::Warning("(ccColorLevelsDlg::ScaleColorFields] Flat input range (input range can't be 0)");
		return false;
	}

	int pOut = outputLevelMax - outputLevelMin;
	double convRatio = pOut / static_cast<double>(qIn);
	for (unsigned i = 0; i < pointCount; ++i)
	{
		const ccColor::Rgba& col = cloud->getPointColor(i);
		ccColor::Rgba newRgb = col;
		for (unsigned c = 0; c < 3; ++c)
		{
			if (applyRGB[c])
			{
				double newC = outputLevelMin + (static_cast<int>(col.rgba[c]) - inputLevelMin) * convRatio;
				newRgb.rgba[c] = static_cast<ColorCompType>(std::max(std::min(newC, static_cast<double>(ccColor::MAX)), 0.0));
			}
		}

		//set the new color
		if (pc)
		{
			pc->setPointColor(i, newRgb);
		}
		else
		{
			//DGM FIXME: dirty!
			const_cast<ccColor::Rgba&>(col) = newRgb;
		}
	}

	return true;
}
