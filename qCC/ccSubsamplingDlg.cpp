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

//Qt
#include <QElapsedTimer>

ccSubsamplingDlg::ccSubsamplingDlg(ccGenericPointCloud* cloud, QWidget* parent/*=0*/)
    : QDialog(parent)
	, Ui::SubsamplingDialog()
	, m_pointCloud(cloud)
{
    setupUi(this);
    setWindowFlags(Qt::Tool);

    samplingMethod->addItem("Random");
    samplingMethod->addItem("Space");
    samplingMethod->addItem("Octree");
    samplingMethod->setCurrentIndex(RANDOM);

    slider->setSliderPosition(slider->maximum());
    changeSamplingMethod(samplingMethod->currentIndex());

    connect(slider, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(slider, SIGNAL(sliderMoved(int)), this, SLOT(sliderMoved(int)));
    connect(samplingValue, SIGNAL(valueChanged(double)), this, SLOT(samplingRateChanged(double)));
    connect(samplingMethod, SIGNAL(currentIndexChanged(int)), this, SLOT(changeSamplingMethod(int)));
}

CCLib::ReferenceCloud* ccSubsamplingDlg::getSampledCloud(CCLib::GenericProgressCallback* progressCb)
{
    CCLib::ReferenceCloud* sampledCloud=NULL;

	QElapsedTimer eTimer;
	eTimer.start();

	switch(samplingMethod->currentIndex())
	{
	case SPACE:
		{
			ccOctree* octree = m_pointCloud->getOctree();
			if (!octree)
				octree = m_pointCloud->computeOctree(progressCb);
			if (octree)
			{
				PointCoordinateType minDist = static_cast<PointCoordinateType>(samplingValue->value());
				sampledCloud = CCLib::CloudSamplingTools::resampleCloudSpatially(m_pointCloud, 
																					minDist,
																					octree,
																					progressCb);
			}
		}
		break;
	case OCTREE:
		{
			ccOctree* octree = m_pointCloud->getOctree();
			if (!octree)
				octree = m_pointCloud->computeOctree(progressCb);

			if (octree)
			{
				sampledCloud = CCLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(m_pointCloud,
																							(uchar)samplingValue->value(),
																							CCLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER,
																							progressCb,
																							octree);
			}
		}
		break;
	case RANDOM:
		sampledCloud = CCLib::CloudSamplingTools::subsampleCloudRandomly(m_pointCloud, (unsigned)(samplingValue->value()), progressCb);
		break;
	}

	ccLog::Print("[Subsampling] Timing: %3.3f s.",eTimer.elapsed()/1000.0);

	return sampledCloud;
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
            valueLabel->setText("space between points");
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

//SLOTS
void ccSubsamplingDlg::sliderReleased()
{
    sliderMoved(slider->sliderPosition());
}

void ccSubsamplingDlg::sliderMoved(int sliderPos)
{
    double rate = static_cast<double>(sliderPos)/static_cast<double>(slider->maximum()-slider->minimum());
    if (samplingMethod->currentIndex() == SPACE)
        rate = 1.0 - rate;

    samplingValue->setValue(samplingValue->minimum() + rate * static_cast<double>(samplingValue->maximum()-samplingValue->minimum()));
    //updateLabels();
}

void ccSubsamplingDlg::samplingRateChanged(double value)
{
    double rate = static_cast<double>(samplingValue->value()-samplingValue->minimum())/static_cast<double>(samplingValue->maximum()-samplingValue->minimum());

    CC_SUBSAMPLING_METHOD method = (CC_SUBSAMPLING_METHOD)samplingMethod->currentIndex();
    if (method == SPACE)
        rate = 1.0 - rate;

    slider->setSliderPosition(slider->minimum() + static_cast<int>(rate * static_cast<double>(slider->maximum()-slider->minimum())));
    //updateLabels();
}

void ccSubsamplingDlg::changeSamplingMethod(int index)
{
    int oldSliderPos = slider->sliderPosition();

    //Reste a changer les textes d'aide
    switch(index)
    {
        case OCTREE:
			{
				samplingValue->setDecimals(0);
				samplingValue->setMinimum(1);
				samplingValue->setMaximum((double)CCLib::DgmOctree::MAX_OCTREE_LEVEL);
				samplingValue->setSingleStep(1);
			}
            break;
        case SPACE:
			{
				samplingValue->setDecimals(4);
				samplingValue->setMinimum(0.0);
				CCVector3 min, max;
				m_pointCloud->getBoundingBox(min.u, max.u);
				double dist = static_cast<double>(CCVector3::vdistance(min.u, max.u));
				samplingValue->setMaximum(dist);
				samplingValue->setSingleStep(0.01);
			}
            break;
        case RANDOM:
			{
				samplingValue->setDecimals(0);
				samplingValue->setMinimum(0);
				samplingValue->setMaximum(static_cast<double>(m_pointCloud->size()));
				samplingValue->setSingleStep(1);
			}
            break;
        default:
            break;
    }

    updateLabels();

    slider->setSliderPosition(oldSliderPos);
    sliderReleased();
}
