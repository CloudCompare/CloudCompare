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

#include "ccSubsamplingDlg.h"
#include "ui_subsamplingDlg.h"

//CCLib
#include <CloudSamplingTools.h>
#include <ScalarField.h>

//qCC_db
#include <ccGenericPointCloud.h>

//Exponent of the 'log' scale used for 'SPACE' interval
static const double SPACE_RANGE_EXPONENT = 0.05;

ccSubsamplingDlg::ccSubsamplingDlg(unsigned maxPointCount, double maxCloudRadius, QWidget* parent/*=0*/)
	: QDialog(parent, Qt::Tool)
	, m_maxPointCount(maxPointCount)
	, m_maxRadius(maxCloudRadius)
	, m_sfModEnabled(false)
	, m_sfMin(0)
	, m_sfMax(0)
	, m_ui( new Ui::SubsamplingDialog )
{
	m_ui->setupUi(this);

	m_ui->samplingMethod->addItem( tr( "Random" ) );
	m_ui->samplingMethod->addItem( tr( "Space" ) );
	m_ui->samplingMethod->addItem( tr( "Octree" ) );

	connect(m_ui->slider, &QSlider::sliderMoved, this, &ccSubsamplingDlg::sliderMoved);
	connect(m_ui->samplingValue,  static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccSubsamplingDlg::samplingRateChanged);
	connect(m_ui->samplingMethod, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),		  this, &ccSubsamplingDlg::changeSamplingMethod);

	m_ui->samplingMethod->setCurrentIndex(1);
	sliderMoved(m_ui->slider->sliderPosition());
}

ccSubsamplingDlg::~ccSubsamplingDlg()
{
	delete m_ui;
}

CCLib::ReferenceCloud* ccSubsamplingDlg::getSampledCloud(ccGenericPointCloud* cloud, CCLib::GenericProgressCallback* progressCb/*=0*/)
{
	if (!cloud || cloud->size() == 0)
	{
		ccLog::Warning("[ccSubsamplingDlg::getSampledCloud] Invalid input cloud!");
		return nullptr;
	}

	switch (m_ui->samplingMethod->currentIndex())
	{
	case RANDOM:
		{
			assert(m_ui->samplingValue->value() >= 0);
			unsigned count = static_cast<unsigned>(m_ui->samplingValue->value());
			return CCLib::CloudSamplingTools::subsampleCloudRandomly(	cloud,
																		count,
																		progressCb);
		}
		break;

	case SPACE:
		{
			ccOctree::Shared octree = cloud->getOctree();
			if (!octree)
				octree = cloud->computeOctree(progressCb);
			if (octree)
			{
				PointCoordinateType minDist = static_cast<PointCoordinateType>(m_ui->samplingValue->value());
				CCLib::CloudSamplingTools::SFModulationParams modParams;
				modParams.enabled = m_ui->sfGroupBox->isEnabled() && m_ui->sfGroupBox->isChecked();
				if (modParams.enabled)
				{
					double deltaSF = static_cast<double>(m_sfMax) - m_sfMin;
					assert(deltaSF >= 0);
					if (deltaSF > CCLib::ZERO_TOLERANCE)
					{
						double sfMinSpacing = m_ui->minSFSpacingDoubleSpinBox->value();
						double sfMaxSpacing = m_ui->maxSFSpacingDoubleSpinBox->value();
						modParams.a = (sfMaxSpacing - sfMinSpacing) / deltaSF;
						modParams.b = sfMinSpacing - modParams.a * m_sfMin;
					}
					else
					{
						modParams.a = 0.0;
						modParams.b = m_sfMin;
					}
				}
				return CCLib::CloudSamplingTools::resampleCloudSpatially(	cloud, 
																			minDist,
																			modParams,
																			octree.data(),
																			progressCb);
			}
			else
			{
				ccLog::Warning(QString("[ccSubsamplingDlg::getSampledCloud] Failed to compute octree for cloud '%1'").arg(cloud->getName()));
			}
		}
		break;

	case OCTREE:
		{
			ccOctree::Shared octree = cloud->getOctree();
			if (!octree)
				octree = cloud->computeOctree(progressCb);
			if (octree)
			{
				assert(m_ui->samplingValue->value() >= 0);
				unsigned char level = static_cast<unsigned char>(m_ui->samplingValue->value());
				return CCLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(	cloud,
																					level,
																					CCLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER,
																					progressCb,
																					octree.data());
			}
			else
			{
				ccLog::Warning(QString("[ccSubsamplingDlg::getSampledCloud] Failed to compute octree for cloud '%1'").arg(cloud->getName()));
			}
		}
		break;
	}

	//something went wrong!
	return nullptr;
}

void ccSubsamplingDlg::updateLabels()
{
	switch(m_ui->samplingMethod->currentIndex())
	{
	case RANDOM:
		m_ui->labelSliderMin->setText( tr( "none" ) );
		m_ui->labelSliderMax->setText( tr( "all" ) );
		m_ui->valueLabel->setText( tr( "remaining points" ) );
		break;
	case SPACE:
		m_ui->labelSliderMin->setText( tr( "large" ) );
		m_ui->labelSliderMax->setText( tr( "small" ) );
		m_ui->valueLabel->setText( tr( "min. space between points" ) );
		break;
	case OCTREE:
		m_ui->labelSliderMin->setText( tr( "min" ) );
		m_ui->labelSliderMax->setText( tr( "max" ) );
		m_ui->valueLabel->setText( tr( "subdivision level" ) );
		break;
	default:
		break;
	}
}

void ccSubsamplingDlg::sliderMoved(int sliderPos)
{
	double sliderRange = static_cast<double>(m_ui->slider->maximum())-m_ui->slider->minimum();
	double rate = static_cast<double>(sliderPos)/sliderRange;
	if (m_ui->samplingMethod->currentIndex() == SPACE)
	{
		rate = pow(rate, SPACE_RANGE_EXPONENT);
		rate = 1.0 - rate;
	}

	double valueRange = static_cast<double>(m_ui->samplingValue->maximum()-m_ui->samplingValue->minimum());
	m_ui->samplingValue->setValue(m_ui->samplingValue->minimum() + rate * valueRange);
}

void ccSubsamplingDlg::samplingRateChanged(double value)
{
	double valueRange = static_cast<double>(m_ui->samplingValue->maximum()-m_ui->samplingValue->minimum());
	double rate = static_cast<double>(value-m_ui->samplingValue->minimum())/valueRange;

	CC_SUBSAMPLING_METHOD method = static_cast<CC_SUBSAMPLING_METHOD>(m_ui->samplingMethod->currentIndex());
	if (method == SPACE)
	{
		rate = 1.0 - rate;
		rate = pow(rate, 1.0/SPACE_RANGE_EXPONENT);

		if (m_sfModEnabled && !m_ui->sfGroupBox->isChecked())
		{
			m_ui->minSFSpacingDoubleSpinBox->setValue(value);
			m_ui->maxSFSpacingDoubleSpinBox->setValue(value);
		}
	}

	m_ui->slider->blockSignals(true);
	double sliderRange = static_cast<double>(m_ui->slider->maximum())-m_ui->slider->minimum();
	m_ui->slider->setSliderPosition(m_ui->slider->minimum() + static_cast<int>(rate * sliderRange));
	m_ui->slider->blockSignals(false);
}

void ccSubsamplingDlg::changeSamplingMethod(int index)
{
	int oldSliderPos = m_ui->slider->sliderPosition();
	m_ui->sfGroupBox->setEnabled(false);

	//update the labels
	m_ui->samplingValue->blockSignals(true);
	switch(index)
	{
	case RANDOM:
		{
			m_ui->samplingValue->setDecimals(0);
			m_ui->samplingValue->setMinimum(1);
			m_ui->samplingValue->setMaximum(static_cast<double>(m_maxPointCount));
			m_ui->samplingValue->setSingleStep(1);
			m_ui->samplingValue->setEnabled(true);
		}
		break;
	case SPACE:
		{
			m_ui->samplingValue->setDecimals(4);
			m_ui->samplingValue->setMinimum(0.0);
			m_ui->samplingValue->setMaximum(m_maxRadius);
			double step = m_maxRadius / 1000.0;
			m_ui->samplingValue->setSingleStep(step);
			m_ui->minSFSpacingDoubleSpinBox->setMaximum(m_maxRadius);
			m_ui->minSFSpacingDoubleSpinBox->setSingleStep(step);
			m_ui->maxSFSpacingDoubleSpinBox->setMaximum(m_maxRadius);
			m_ui->maxSFSpacingDoubleSpinBox->setSingleStep(step);
			m_ui->sfGroupBox->setEnabled(m_sfModEnabled);
			m_ui->samplingValue->setDisabled(m_ui->sfGroupBox->isEnabled() && m_ui->sfGroupBox->isChecked());
		}
		break;
	case OCTREE:
		{
			m_ui->samplingValue->setDecimals(0);
			m_ui->samplingValue->setMinimum(1);
			m_ui->samplingValue->setMaximum(static_cast<double>(CCLib::DgmOctree::MAX_OCTREE_LEVEL));
			m_ui->samplingValue->setSingleStep(1);
			m_ui->samplingValue->setEnabled(true);
		}
		break;
	default:
		break;
	}
	m_ui->samplingValue->blockSignals(false);

	updateLabels();
	//slider->setSliderPosition(oldSliderPos);
	sliderMoved(oldSliderPos);
}

void ccSubsamplingDlg::enableSFModulation(ScalarType sfMin, ScalarType sfMax)
{
	m_sfModEnabled = CCLib::ScalarField::ValidValue(sfMin) && CCLib::ScalarField::ValidValue(sfMax);
	if (!m_sfModEnabled)
	{
		ccLog::Warning("[ccSubsamplingDlg::enableSFModulation] Invalid input SF values");
		return;
	}

	m_sfMin = sfMin;
	m_sfMax = sfMax;

	m_ui->sfGroupBox->setEnabled(m_ui->samplingMethod->currentIndex() == SPACE);
	m_ui->minSFlabel->setText(QString::number(sfMin));
	m_ui->maxSFlabel->setText(QString::number(sfMax));
}
