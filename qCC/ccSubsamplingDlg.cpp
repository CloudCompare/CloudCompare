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

//CCLib
#include <DgmOctree.h>
#include <ReferenceCloud.h>
#include <CloudSamplingTools.h>
#include <GeometricalAnalysisTools.h>
#include <ScalarField.h>

//qCC_db
#include <ccLog.h>
#include <ccGenericPointCloud.h>
#include <ccOctree.h>

//Exponent of the 'log' scale used for 'SPACE' interval
static const double SPACE_RANGE_EXPONENT = 0.05;

ccSubsamplingDlg::ccSubsamplingDlg(unsigned maxPointCount, double maxCloudRadius, QWidget* parent/*=0*/)
	: QDialog(parent, Qt::Tool)
	, Ui::SubsamplingDialog()
	, m_maxPointCount(maxPointCount)
	, m_maxRadius(maxCloudRadius)
	, m_sfModEnabled(false)
	, m_sfMin(0)
	, m_sfMax(0)
{
	setupUi(this);

	samplingMethod->addItem("Random");
	samplingMethod->addItem("Space");
	samplingMethod->addItem("Octree");

	connect(slider,         SIGNAL(sliderMoved(int)),         this, SLOT(sliderMoved(int)));
	connect(samplingValue,  SIGNAL(valueChanged(double)),     this, SLOT(samplingRateChanged(double)));
	connect(samplingMethod, SIGNAL(currentIndexChanged(int)), this, SLOT(changeSamplingMethod(int)));

	samplingMethod->setCurrentIndex(1);
	sliderMoved(slider->sliderPosition());
}

CCLib::ReferenceCloud* ccSubsamplingDlg::getSampledCloud(ccGenericPointCloud* cloud, CCLib::GenericProgressCallback* progressCb/*=0*/)
{
	if (!cloud || cloud->size() == 0)
	{
		ccLog::Warning("[ccSubsamplingDlg::getSampledCloud] Invalid input cloud!");
		return 0;
	}

	switch (samplingMethod->currentIndex())
	{
	case RANDOM:
		{
			assert(samplingValue->value() >= 0);
			unsigned count = static_cast<unsigned>(samplingValue->value());
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
				PointCoordinateType minDist = static_cast<PointCoordinateType>(samplingValue->value());
				CCLib::CloudSamplingTools::SFModulationParams modParams;
				modParams.enabled = sfGroupBox->isEnabled() && sfGroupBox->isChecked();
				if (modParams.enabled)
				{
					double deltaSF = m_sfMax - m_sfMin;
					assert(deltaSF >= 0);
					if (deltaSF > ZERO_TOLERANCE)
					{
						double sfMinSpacing = minSFSpacingDoubleSpinBox->value();
						double sfMaxSpacing = maxSFSpacingDoubleSpinBox->value();
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
				assert(samplingValue->value() >= 0);
				unsigned char level = static_cast<unsigned char>(samplingValue->value());
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
	return 0;
}

void ccSubsamplingDlg::updateLabels()
{
	switch(samplingMethod->currentIndex())
	{
	case RANDOM:
		labelSliderMin->setText("none");
		labelSliderMax->setText("all");
		valueLabel->setText("remaining points");
		break;
	case SPACE:
		labelSliderMin->setText("large");
		labelSliderMax->setText("small");
		valueLabel->setText("min. space between points");
		break;
	case OCTREE:
		labelSliderMin->setText("min");
		labelSliderMax->setText("max");
		valueLabel->setText("subdivision level");
		break;
	default:
		break;
	}
}

void ccSubsamplingDlg::sliderMoved(int sliderPos)
{
	double sliderRange = static_cast<double>(slider->maximum()-slider->minimum());
	double rate = static_cast<double>(sliderPos)/sliderRange;
	if (samplingMethod->currentIndex() == SPACE)
	{
		rate = pow(rate, SPACE_RANGE_EXPONENT);
		rate = 1.0 - rate;
	}

	double valueRange = static_cast<double>(samplingValue->maximum()-samplingValue->minimum());
	samplingValue->setValue(samplingValue->minimum() + rate * valueRange);
}

void ccSubsamplingDlg::samplingRateChanged(double value)
{
	double valueRange = static_cast<double>(samplingValue->maximum()-samplingValue->minimum());
	double rate = static_cast<double>(value-samplingValue->minimum())/valueRange;

	CC_SUBSAMPLING_METHOD method = static_cast<CC_SUBSAMPLING_METHOD>(samplingMethod->currentIndex());
	if (method == SPACE)
	{
		rate = 1.0 - rate;
		rate = pow(rate, 1.0/SPACE_RANGE_EXPONENT);

		if (m_sfModEnabled && !sfGroupBox->isChecked())
		{
			minSFSpacingDoubleSpinBox->setValue(value);
			maxSFSpacingDoubleSpinBox->setValue(value);
		}
	}

	slider->blockSignals(true);
	double sliderRange = static_cast<double>(slider->maximum()-slider->minimum());
	slider->setSliderPosition(slider->minimum() + static_cast<int>(rate * sliderRange));
	slider->blockSignals(false);
}

void ccSubsamplingDlg::changeSamplingMethod(int index)
{
	int oldSliderPos = slider->sliderPosition();
	sfGroupBox->setEnabled(false);

	//update the labels
	samplingValue->blockSignals(true);
	switch(index)
	{
	case RANDOM:
		{
			samplingValue->setDecimals(0);
			samplingValue->setMinimum(1);
			samplingValue->setMaximum(static_cast<double>(m_maxPointCount));
			samplingValue->setSingleStep(1);
			samplingValue->setEnabled(true);
		}
		break;
	case SPACE:
		{
			samplingValue->setDecimals(4);
			samplingValue->setMinimum(0.0);
			samplingValue->setMaximum(m_maxRadius);
			double step = m_maxRadius / 1000.0;
			samplingValue->setSingleStep(step);
			minSFSpacingDoubleSpinBox->setMaximum(m_maxRadius);
			minSFSpacingDoubleSpinBox->setSingleStep(step);
			maxSFSpacingDoubleSpinBox->setMaximum(m_maxRadius);
			maxSFSpacingDoubleSpinBox->setSingleStep(step);
			sfGroupBox->setEnabled(m_sfModEnabled);
			samplingValue->setDisabled(sfGroupBox->isEnabled() && sfGroupBox->isChecked());
		}
		break;
	case OCTREE:
		{
			samplingValue->setDecimals(0);
			samplingValue->setMinimum(1);
			samplingValue->setMaximum(static_cast<double>(CCLib::DgmOctree::MAX_OCTREE_LEVEL));
			samplingValue->setSingleStep(1);
			samplingValue->setEnabled(true);
		}
		break;
	default:
		break;
	}
	samplingValue->blockSignals(false);

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

	sfGroupBox->setEnabled(samplingMethod->currentIndex() == SPACE);
	minSFlabel->setText(QString::number(sfMin));
	maxSFlabel->setText(QString::number(sfMax));
}
