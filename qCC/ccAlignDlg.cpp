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

#include "ccAlignDlg.h"
#include "mainwindow.h"
#include "ccDisplayOptionsDlg.h"

//CCLib
#include <CloudSamplingTools.h>
#include <GeometricalAnalysisTools.h>
#include <DgmOctree.h>
#include <ReferenceCloud.h>
#include <ChunkedPointCloud.h>

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccProgressDialog.h>

ccAlignDlg::ccAlignDlg(ccGenericPointCloud *data, ccGenericPointCloud *model, QWidget* parent)
    : QDialog(parent), Ui::AlignDialog()
{
    setupUi(this);
    setWindowFlags(Qt::Tool);

    samplingMethod->addItem("None");
    samplingMethod->addItem("Random");
    samplingMethod->addItem("Space");
    samplingMethod->addItem("Octree");
    samplingMethod->setCurrentIndex(NONE);

    QColor qRed(255,0,0);
    QColor qYellow(255,255,0);
    ccDisplayOptionsDlg::SetButtonColor(dataColorButton,qRed);
    ccDisplayOptionsDlg::SetButtonColor(modelColorButton,qYellow);

    dataObject = data;
    modelObject = model;
    setColorsAndLabels();

    changeSamplingMethod(samplingMethod->currentIndex());
    toggleNbMaxCandidates(isNbCandLimited->isChecked());

    connect(swapButton, SIGNAL(clicked()), this, SLOT(swapModelAndData()));
    connect(modelSample, SIGNAL(sliderReleased()), this, SLOT(modelSliderReleased()));
    connect(dataSample, SIGNAL(sliderReleased()), this, SLOT(dataSliderReleased()));
    connect(modelSamplingRate, SIGNAL(valueChanged(double)), this, SLOT(modelSamplingRateChanged(double)));
    connect(dataSamplingRate, SIGNAL(valueChanged(double)), this, SLOT(dataSamplingRateChanged(double)));
    connect(deltaEstimation, SIGNAL(clicked()), this, SLOT(estimateDelta()));
    connect(samplingMethod, SIGNAL(currentIndexChanged(int)), this, SLOT(changeSamplingMethod(int)));
    connect(isNbCandLimited, SIGNAL(toggled(bool)), this, SLOT(toggleNbMaxCandidates(bool)));
}

ccAlignDlg::~ccAlignDlg()
{
    modelObject->enableTempColor(false);
    dataObject->enableTempColor(false);
}

unsigned ccAlignDlg::getNbTries()
{
    return nbTries->value();
}

double ccAlignDlg::getOverlap()
{
    return overlap->value();
}

double ccAlignDlg::getDelta()
{
    return delta->value();
}

ccGenericPointCloud *ccAlignDlg::getModelObject()
{
    return modelObject;
}

ccGenericPointCloud *ccAlignDlg::getDataObject()
{
    return dataObject;
}

ccAlignDlg::CC_SAMPLING_METHOD ccAlignDlg::getSamplingMethod()
{
    return (CC_SAMPLING_METHOD)samplingMethod->currentIndex();
}

bool ccAlignDlg::isNumberOfCandidatesLimited()
{
    return isNbCandLimited->isChecked();
}

unsigned ccAlignDlg::getMaxNumberOfCandidates()
{
    return nbMaxCandidates->value();
}

CCLib::ReferenceCloud *ccAlignDlg::getSampledModel()
{
    CCLib::ReferenceCloud *sampledCloud;

    sampledCloud = NULL;
    switch(getSamplingMethod())
    {
        case SPACE:
            sampledCloud = CCLib::CloudSamplingTools::resampleCloudSpatially(modelObject, modelSamplingRate->value());
            break;
        case OCTREE:
            if(modelObject->getOctree())
                sampledCloud = CCLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(modelObject, (uchar)modelSamplingRate->value(),
                    CCLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER, NULL, (CCLib::DgmOctree*)modelObject->getOctree());
            break;
        case RANDOM:
            sampledCloud = CCLib::CloudSamplingTools::subsampleCloudRandomly(modelObject, (unsigned)(modelSamplingRate->value()));
            if(sampledCloud != NULL)
                break;
        default:
            sampledCloud = new CCLib::ReferenceCloud(modelObject);
            sampledCloud->reserve(modelObject->size());
			sampledCloud->addPointIndex(0,modelObject->size());
            break;
    }

    return sampledCloud;
}

CCLib::ReferenceCloud *ccAlignDlg::getSampledData()
{
    CCLib::ReferenceCloud *sampledCloud;

    sampledCloud = NULL;
    switch(getSamplingMethod())
    {
        case SPACE:
            sampledCloud = CCLib::CloudSamplingTools::resampleCloudSpatially(dataObject, dataSamplingRate->value());
            break;
        case OCTREE:
            if(dataObject->getOctree())
                sampledCloud = CCLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(dataObject, (uchar)dataSamplingRate->value(),
                    CCLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER, NULL, (CCLib::DgmOctree*)dataObject->getOctree());
            break;
        case RANDOM:
            sampledCloud = CCLib::CloudSamplingTools::subsampleCloudRandomly(dataObject, (unsigned)(dataSamplingRate->value()));
            if(sampledCloud != NULL)
                break;
        default:
            sampledCloud = new CCLib::ReferenceCloud(dataObject);
            sampledCloud->reserve(dataObject->size());
			sampledCloud->addPointIndex(0,dataObject->size());
            break;
    }
    return sampledCloud;
}

void ccAlignDlg::setColorsAndLabels()
{
    if (!modelObject || !dataObject)
        return;

    modelCloud->setText(modelObject->getName());
    modelObject->setVisible(true);
    modelObject->setTempColor(ccColor::red);
    modelObject->prepareDisplayForRefresh_recursive();

    dataCloud->setText(dataObject->getName());
    dataObject->setVisible(true);
    dataObject->setTempColor(ccColor::yellow);
    dataObject->prepareDisplayForRefresh_recursive();

    MainWindow::RefreshAllGLWindow();
}

//SLOTS
void ccAlignDlg::swapModelAndData()
{
    std::swap(dataObject,modelObject);
    setColorsAndLabels();
    changeSamplingMethod(samplingMethod->currentIndex());
}

void ccAlignDlg::modelSliderReleased()
{
    float rate;
    rate = (float)modelSample->sliderPosition()/(float)modelSample->maximum();
    if(getSamplingMethod() == SPACE)
        rate = 1.-rate;
    rate *= modelSamplingRate->maximum();
    modelSamplingRate->setValue(rate);
    modelSamplingRateChanged(rate);
}

void ccAlignDlg::dataSliderReleased()
{
    float rate;
    rate = (float)dataSample->sliderPosition()/(float)dataSample->maximum();
    if(getSamplingMethod() == SPACE)
        rate = 1.-rate;
    rate *= dataSamplingRate->maximum();
    dataSamplingRate->setValue(rate);
    dataSamplingRateChanged(rate);
}

void ccAlignDlg::modelSamplingRateChanged(double value)
{
    char text[256];
    unsigned remaining;
    CCLib::ReferenceCloud *tmpCloud;
    CC_SAMPLING_METHOD method;
    float rate;

    method = getSamplingMethod();
    rate = (float)modelSamplingRate->value()/(float)modelSamplingRate->maximum();
    if(method == SPACE)
        rate = 1.-rate;
    modelSample->setSliderPosition((unsigned)((float)modelSample->maximum()*rate));

    switch(method)
    {
        case SPACE:
            tmpCloud = getSampledModel();
            remaining = tmpCloud->size();
            sprintf(text, "distance units (%d remaining points)", remaining);
            delete tmpCloud;
            break;
        case RANDOM:
            sprintf(text, "remaining points (%.1f%%)", rate*100.);
            break;
        case OCTREE:
            tmpCloud = getSampledModel();
            remaining = tmpCloud->size();
            sprintf(text, "%d remaining points", remaining);
            delete tmpCloud;
            break;
        default:
            remaining = (unsigned)((float)modelObject->size()*rate);
            sprintf(text, "%% (%d remaining points)", remaining);
            break;
    }
    modelRemaining->setText(text);
}

void ccAlignDlg::dataSamplingRateChanged(double value)
{
    char text[256];
    unsigned remaining;
    CCLib::ReferenceCloud *tmpCloud;
    CC_SAMPLING_METHOD method;
    float rate;

    method = getSamplingMethod();
    rate = (float)dataSamplingRate->value()/(float)dataSamplingRate->maximum();
    if(method == SPACE)
        rate = 1-rate;
    dataSample->setSliderPosition((unsigned)((float)dataSample->maximum()*rate));

    switch(method)
    {
        case SPACE:
            tmpCloud = getSampledData();
            remaining = tmpCloud->size();
            sprintf(text, "distance units (%d remaining points)", remaining);
            delete tmpCloud;
            break;
        case RANDOM:
            sprintf(text, "remaining points (%.1f%%)", rate*100.);
            break;
        case OCTREE:
            tmpCloud = getSampledData();
            remaining = tmpCloud->size();
            sprintf(text, "%d remaining points", remaining);
            delete tmpCloud;
            break;
        default:
            remaining = (unsigned)((float)dataObject->size()*rate);
            sprintf(text, "%% (%d remaining points)", remaining);
            break;
    }
    dataRemaining->setText(text);
}

void ccAlignDlg::estimateDelta()
{
    unsigned i, nb;
    float meanDensity, meanSqrDensity, dev, value;
    ccProgressDialog pDlg(false,this);

    CCLib::ReferenceCloud *sampledData = getSampledData();
    //we have to work on a copy of the cloud in order to prevent the algorithms from modifying the original cloud.
    CCLib::ChunkedPointCloud* cloud = new CCLib::ChunkedPointCloud();
    cloud->reserve(sampledData->size());
    for(i=0; i<sampledData->size(); i++)
        cloud->addPoint(*sampledData->getPoint(i));
    cloud->enableScalarField();

    CCLib::GeometricalAnalysisTools::computeLocalDensity(cloud, &pDlg);
    nb = 0;
    meanDensity = 0.;
    meanSqrDensity = 0.;
    for(i=0; i<cloud->size(); i++)
    {
        value = cloud->getPointScalarValue(i);
        if(value > ZERO_TOLERANCE)
        {
            value = 1/value;
            meanDensity += value;
            meanSqrDensity += value*value;
            nb++;
        }
    }
    meanDensity /= (float)nb;
    meanSqrDensity /= (float)nb;
    dev = meanSqrDensity-(meanDensity*meanDensity);

    delta->setValue(meanDensity+dev);
    delete sampledData;
    delete cloud;
}

void ccAlignDlg::changeSamplingMethod(int index)
{
    float dist;
    unsigned oldSliderPos;
    CCVector3 min, max;

    //Reste à changer les textes d'aide
    switch(index)
    {
        case SPACE:
            modelSamplingRate->setDecimals(4);
            dataSamplingRate->setDecimals(4);
            oldSliderPos = modelSample->sliderPosition();
            modelObject->getBoundingBox(min.u, max.u);
            dist = (min-max).norm();
            modelSamplingRate->setMaximum(dist);
            modelSample->setSliderPosition(oldSliderPos);
            oldSliderPos = dataSample->sliderPosition();
            dataObject->getBoundingBox(min.u, max.u);
            dist = (min-max).norm();
            dataSamplingRate->setMaximum(dist);
            dataSample->setSliderPosition(oldSliderPos);
            modelSamplingRate->setSingleStep(0.01);
            dataSamplingRate->setSingleStep(0.01);
            modelSamplingRate->setMinimum(0.);
            dataSamplingRate->setMinimum(0.);
            break;
        case RANDOM:
            modelSamplingRate->setDecimals(0);
            dataSamplingRate->setDecimals(0);
            modelSamplingRate->setMaximum((float)modelObject->size());
            dataSamplingRate->setMaximum((float)dataObject->size());
            modelSamplingRate->setSingleStep(1.);
            dataSamplingRate->setSingleStep(1.);
            modelSamplingRate->setMinimum(0.);
            dataSamplingRate->setMinimum(0.);
            break;
        case OCTREE:
            if(!modelObject->getOctree())
                modelObject->computeOctree();
            if(!dataObject->getOctree())
                dataObject->computeOctree();
            modelSamplingRate->setDecimals(0);
            dataSamplingRate->setDecimals(0);
            modelSamplingRate->setMaximum((float)CCLib::DgmOctree::MAX_OCTREE_LEVEL);
            dataSamplingRate->setMaximum((float)CCLib::DgmOctree::MAX_OCTREE_LEVEL);
            modelSamplingRate->setMinimum(1.);
            dataSamplingRate->setMinimum(1.);
            modelSamplingRate->setSingleStep(1.);
            dataSamplingRate->setSingleStep(1.);
            break;
        default:
            modelSamplingRate->setDecimals(2);
            dataSamplingRate->setDecimals(2);
            modelSamplingRate->setMaximum(100.);
            dataSamplingRate->setMaximum(100.);
            modelSamplingRate->setSingleStep(0.01);
            dataSamplingRate->setSingleStep(0.01);
            modelSamplingRate->setMinimum(0.);
            dataSamplingRate->setMinimum(0.);
            break;
    }


    if(index == NONE)
    {
        modelSample->setSliderPosition(modelSample->maximum());
        dataSample->setSliderPosition(dataSample->maximum());
        modelSample->setEnabled(false);
        dataSample->setEnabled(false);
        modelSamplingRate->setEnabled(false);
        dataSamplingRate->setEnabled(false);
    }
    else
    {
        modelSample->setEnabled(true);
        dataSample->setEnabled(true);
        modelSamplingRate->setEnabled(true);
        dataSamplingRate->setEnabled(true);
    }

    modelSliderReleased();
    dataSliderReleased();
}

void ccAlignDlg::toggleNbMaxCandidates(bool activ)
{
    nbMaxCandidates->setEnabled(activ);
}
