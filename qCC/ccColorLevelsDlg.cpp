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
	, m_histogram(0)
	, m_cloud(pointCloud)
{
	setupUi(this);

	//connect GUI elements
	connect(channelComboBox,							SIGNAL(currentIndexChanged(int)),	this,	SLOT(onChannelChanged(int)));
	connect(buttonBox->button(QDialogButtonBox::Apply),	SIGNAL(clicked()),					this,	SLOT(onApply()));

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
			for (int i=0; i<3; ++i)
			{
				if (	channelComboBox->currentIndex() == RGB
					||	channelComboBox->currentIndex() == i+1 )
				{
					histoValues[i].resize(1 << (sizeof(ColorCompType)*8),0);
				}
			}
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			m_histogram->clear();
			return;
		}

		std::vector<unsigned>* histoValuesR = (histoValues[0].empty() ? 0 : histoValues);
		std::vector<unsigned>* histoValuesG = (histoValues[1].empty() ? 0 : histoValues+1);
		std::vector<unsigned>* histoValuesB = (histoValues[2].empty() ? 0 : histoValues+2);

		switch(channelComboBox->currentIndex())
		{
		case RGB:
			m_histogram->setSolidColor(Qt::black);
			m_histogram->setAxisLabels("R,G,B","");
			//test: for now we send all data into the same histogram!
			histoValuesG = histoValuesR;
			histoValuesB = histoValuesR;
			break;
		case RED:
			m_histogram->setSolidColor(Qt::red);
			m_histogram->setAxisLabels("Red","");
			break;
		case GREEN:
			m_histogram->setSolidColor(Qt::green);
			m_histogram->setAxisLabels("Green","");
			break;
		case BLUE:
			m_histogram->setSolidColor(Qt::blue);
			m_histogram->setAxisLabels("Blue","");
			break;
		}

		//project points
		{
			for (unsigned i=0; i<pointCount; ++i)
			{
				const ccColor::Rgb& rgb = m_cloud->getPointColor(i);
				if (histoValuesR)
					histoValuesR->at(rgb.r)++;
				if (histoValuesG)
					histoValuesG->at(rgb.g)++;
				if (histoValuesB)
					histoValuesB->at(rgb.b)++;
			}
		}

		for (int i=0; i<3; ++i)
		{
			if (channelComboBox->currentIndex() == RGB || channelComboBox->currentIndex() == i+1)
			{
				m_histogram->fromBinArray(histoValues[i],0.0,256.0);
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

		bool applyRGB[3] = {channelComboBox->currentIndex() == RGB || channelComboBox->currentIndex() == RED,
						channelComboBox->currentIndex() == RGB || channelComboBox->currentIndex() == GREEN,
						channelComboBox->currentIndex() == RGB || channelComboBox->currentIndex() == BLUE };

		//update display
		ccPointCloud* pc = ccHObjectCaster::ToPointCloud(m_cloud);

		unsigned pointCount = m_cloud->size();
		int qIn = s_inputLevels[1] - s_inputLevels[0];
		int pOut = s_outputLevels[1] - s_outputLevels[0];
		for (unsigned i=0; i<pointCount; ++i)
		{
			const ccColor::Rgb& rgb = m_cloud->getPointColor(i);
			ccColor::Rgb newRgb;
			for (unsigned c = 0; c < 3; ++c)
			{
				if (applyRGB[c])
				{
					double newC = s_outputLevels[0];
					if (qIn)
					{
						double u = (static_cast<double>(rgb.rgb[c]) - s_inputLevels[0]) / qIn;
						newC = s_outputLevels[0] + u * pOut; 
					}
					newRgb.rgb[c] = static_cast<ColorCompType>(std::max<double>(std::min<double>(newC, ccColor::MAX), 0.0));
				}
				else
				{
					newRgb.rgb[c] = rgb.rgb[c];
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
				const_cast<ccColor::Rgb&>(rgb) = newRgb;
			}
		}

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
