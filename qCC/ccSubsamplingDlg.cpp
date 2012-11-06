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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2241                                                              $
//$LastChangedDate:: 2012-09-21 23:22:39 +0200 (ven., 21 sept. 2012)       $
//**************************************************************************
//

#include "ccSubsamplingDlg.h"

//CCLib
#include <DgmOctree.h>
#include <ReferenceCloud.h>
#include <CloudSamplingTools.h>
#include <GeometricalAnalysisTools.h>

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccOctree.h>

//qCC
#include "ccConsole.h"

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
				sampledCloud = CCLib::CloudSamplingTools::resampleCloudSpatially(m_pointCloud, 
																					samplingValue->value(), 
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
		//DGM: WTF?
		/*if(sampledCloud == NULL)
		{
		sampledCloud = new CCLib::ReferenceCloud(m_pointCloud);
		sampledCloud->reserve(m_pointCloud->size());
		sampledCloud->addPointIndex(0,m_pointCloud->size());
		}
		//*/
		break;
	}

	ccConsole::Print("[Subsampling] Timing: %3.3f s.",eTimer.elapsed()/1000.0);

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
    float rate = (float)sliderPos/(float)(slider->maximum()-slider->minimum());
    if(samplingMethod->currentIndex() == SPACE)
        rate = 1.-rate;

    samplingValue->setValue(samplingValue->minimum() + rate * (float)(samplingValue->maximum()-samplingValue->minimum()));
    //updateLabels();
}

void ccSubsamplingDlg::samplingRateChanged(double value)
{
    float rate = (float)(samplingValue->value()-samplingValue->minimum())/(float)(samplingValue->maximum()-samplingValue->minimum());

    CC_SUBSAMPLING_METHOD method = (CC_SUBSAMPLING_METHOD)samplingMethod->currentIndex();
    if(method == SPACE)
        rate = 1.-rate;

    slider->setSliderPosition(slider->minimum()+(unsigned)(rate * (float)(slider->maximum()-slider->minimum())));
    //updateLabels();
}

void ccSubsamplingDlg::changeSamplingMethod(int index)
{
    float dist;
    unsigned oldSliderPos;
    CCVector3 min, max;

    oldSliderPos = slider->sliderPosition();

    //Reste à changer les textes d'aide
    switch(index)
    {
        case OCTREE:
            samplingValue->setDecimals(0);
            samplingValue->setMinimum(1);
            samplingValue->setMaximum(CCLib::DgmOctree::MAX_OCTREE_LEVEL);
            samplingValue->setSingleStep(1);
            break;
        case SPACE:
            samplingValue->setDecimals(4);
            samplingValue->setMinimum(0.0);
            m_pointCloud->getBoundingBox(min.u, max.u);
            dist = CCVector3::vdistance(min.u, max.u);
            samplingValue->setMaximum(dist);
            samplingValue->setSingleStep(0.01);
            break;
        case RANDOM:
            samplingValue->setDecimals(0);
            samplingValue->setMinimum(0);
            samplingValue->setMaximum((float)m_pointCloud->size());
            samplingValue->setSingleStep(1);
            break;
        default:
            break;
    }

    updateLabels();

    slider->setSliderPosition(oldSliderPos);
    sliderReleased();
}
