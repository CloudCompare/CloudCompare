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

#include "ccSubsamplingDlg.h"

//CCLib
#include <DgmOctree.h>
#include <ReferenceCloud.h>
#include <CloudSamplingTools.h>
#include <GeometricalAnalysisTools.h>

//qCC_db
#include <ccLog.h>
#include <ccGenericPointCloud.h>
#include <ccOctree.h>

//Exponent of the 'log' scale used for 'SPACE' interval
static const double SPACE_RANGE_EXPONENT = 0.05;

ccSubsamplingDlg::ccSubsamplingDlg(unsigned maxPointCount, double maxCloudRadius, QWidget* parent/*=0*/)
	: QDialog(parent)
	, Ui::SubsamplingDialog()
	, m_maxPointCount(maxPointCount)
	, m_maxRadius(maxCloudRadius)
{
	setupUi(this);
	setWindowFlags(Qt::Tool);

	samplingMethod->addItem("Random");
	samplingMethod->addItem("Space");
	samplingMethod->addItem("Octree");

	connect(slider, SIGNAL(sliderMoved(int)), this, SLOT(sliderMoved(int)));
	connect(samplingValue, SIGNAL(valueChanged(double)), this, SLOT(samplingRateChanged(double)));
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
			unsigned count = static_cast<unsigned>(samplingValue->value());
			return CCLib::CloudSamplingTools::subsampleCloudRandomly(	cloud,
																		count,
																		progressCb);
		}
		break;

	case SPACE:
		{
			ccOctree* octree = cloud->getOctree();
			if (!octree)
				octree = cloud->computeOctree(progressCb);
			if (octree)
			{
				PointCoordinateType minDist = static_cast<PointCoordinateType>(samplingValue->value());
				return CCLib::CloudSamplingTools::resampleCloudSpatially(	cloud, 
																			minDist,
																			octree,
																			progressCb);
			}
		}
		break;

	case OCTREE:
		{
			ccOctree* octree = cloud->getOctree();
			if (!octree)
				octree = cloud->computeOctree(progressCb);
			if (octree)
			{
				unsigned char level = static_cast<unsigned char>(samplingValue->value());
				return CCLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(	cloud,
																					level,
																					CCLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER,
																					progressCb,
																					octree);
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
		labelSliderMin->setText("None");
		labelSliderMax->setText("All");
		valueLabel->setText("remaining points");
		break;
	case SPACE:
		labelSliderMin->setText("Large");
		labelSliderMax->setText("Small");
		valueLabel->setText("min. space between points");
		break;
	case OCTREE:
		labelSliderMin->setText("Min");
		labelSliderMax->setText("Max");
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
	}

	slider->blockSignals(true);
	double sliderRange = static_cast<double>(slider->maximum()-slider->minimum());
	slider->setSliderPosition(slider->minimum() + static_cast<int>(rate * sliderRange));
	slider->blockSignals(false);
}

void ccSubsamplingDlg::changeSamplingMethod(int index)
{
	int oldSliderPos = slider->sliderPosition();

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
		}
		break;
	case SPACE:
		{
			samplingValue->setDecimals(4);
			samplingValue->setMinimum(0.0);
			samplingValue->setMaximum(m_maxRadius);
			samplingValue->setSingleStep(m_maxRadius / 1000.0);
		}
		break;
	case OCTREE:
		{
			samplingValue->setDecimals(0);
			samplingValue->setMinimum(1);
			samplingValue->setMaximum(static_cast<double>(CCLib::DgmOctree::MAX_OCTREE_LEVEL));
			samplingValue->setSingleStep(1);
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
