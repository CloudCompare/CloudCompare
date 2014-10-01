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
	: QDialog(parent)
	, Ui::AlignDialog()
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
	CCLib::ReferenceCloud* sampledCloud=0;

	switch (getSamplingMethod())
	{
	case SPACE:
		{
			sampledCloud = CCLib::CloudSamplingTools::resampleCloudSpatially(modelObject, static_cast<PointCoordinateType>(modelSamplingRate->value()));
		}
		break;
	case OCTREE:
		if (modelObject->getOctree())
		{
			sampledCloud = CCLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(	modelObject,
																						(uchar)modelSamplingRate->value(),
																						CCLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER,
																						NULL,
																						(CCLib::DgmOctree*)modelObject->getOctree());
		}
		else
		{
			ccLog::Error("[ccAlignDlg::getSampledModel] Failed to get/compute model octree!");
		}
		break;
	case RANDOM:
		{
			sampledCloud = CCLib::CloudSamplingTools::subsampleCloudRandomly(modelObject, (unsigned)(modelSamplingRate->value()));
		}
		break;
	default:
		{
			sampledCloud = new CCLib::ReferenceCloud(modelObject);
			if (!sampledCloud->addPointIndex(0,modelObject->size()))
			{
				delete sampledCloud;
				sampledCloud = 0;
				ccLog::Error("[ccAlignDlg::getSampledModel] Not enough memory!");
			}
		}
		break;
	}

	return sampledCloud;
}

CCLib::ReferenceCloud *ccAlignDlg::getSampledData()
{
	CCLib::ReferenceCloud* sampledCloud=0;

	switch (getSamplingMethod())
	{
	case SPACE:
		{
			sampledCloud = CCLib::CloudSamplingTools::resampleCloudSpatially(dataObject, static_cast<PointCoordinateType>(dataSamplingRate->value()));
		}
		break;
	case OCTREE:
		if (dataObject->getOctree())
		{
			sampledCloud = CCLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(	dataObject,
																						static_cast<uchar>(dataSamplingRate->value()),
																						CCLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER,
																						NULL,
																						(CCLib::DgmOctree*)dataObject->getOctree());
		}
		else
		{
			ccLog::Error("[ccAlignDlg::getSampledData] Failed to get/compute data octree!");
		}
		break;
	case RANDOM:
		{
			sampledCloud = CCLib::CloudSamplingTools::subsampleCloudRandomly(dataObject, (unsigned)(dataSamplingRate->value()));
		}
		break;
	default:
		{
			sampledCloud = new CCLib::ReferenceCloud(dataObject);
			if (!sampledCloud->addPointIndex(0,dataObject->size()))
			{
				delete sampledCloud;
				sampledCloud = 0;
				ccLog::Error("[ccAlignDlg::getSampledData] Not enough memory!");
			}
		}
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
	double rate = static_cast<double>(modelSample->sliderPosition())/static_cast<double>(modelSample->maximum());
	if ( getSamplingMethod() == SPACE)
		rate = 1.0 - rate;
	rate *= modelSamplingRate->maximum();
	modelSamplingRate->setValue(rate);
	modelSamplingRateChanged(rate);
}

void ccAlignDlg::dataSliderReleased()
{
	double rate = static_cast<double>(dataSample->sliderPosition())/static_cast<double>(dataSample->maximum());
	if (getSamplingMethod() == SPACE)
		rate = 1.0 - rate;
	rate *= dataSamplingRate->maximum();
	dataSamplingRate->setValue(rate);
	dataSamplingRateChanged(rate);
}

void ccAlignDlg::modelSamplingRateChanged(double value)
{
	QString message("An error occurred");

	CC_SAMPLING_METHOD method = getSamplingMethod();
	float rate = (float)modelSamplingRate->value()/(float)modelSamplingRate->maximum();
	if (method == SPACE)
		rate = 1.0f-rate;
	modelSample->setSliderPosition((unsigned)((float)modelSample->maximum()*rate));

	switch(method)
	{
	case SPACE:
		{
			CCLib::ReferenceCloud* tmpCloud = getSampledModel(); //DGM FIXME: wow! you generate a spatially sampled cloud just to display its size?!
			if (tmpCloud)
			{
				message = QString("distance units (%1 remaining points)").arg(tmpCloud->size());
				delete tmpCloud;
			}
		}
		break;
	case RANDOM:
		{
			message = QString("remaining points (%1%)").arg(rate*100.0f,0,'f',1);
		}
		break;
	case OCTREE:
		{
			CCLib::ReferenceCloud* tmpCloud = getSampledModel(); //DGM FIXME: wow! you generate a spatially sampled cloud just to display its size?!
			if (tmpCloud)
			{
				message = QString("%1 remaining points").arg(tmpCloud->size());
				delete tmpCloud;
			}
		}
		break;
	default:
		{
			unsigned remaining = (unsigned)(rate * (float)modelObject->size());
			message = QString("%1 remaining points").arg(remaining);
		}
		break;
	}
	modelRemaining->setText(message);
}

void ccAlignDlg::dataSamplingRateChanged(double value)
{
	QString message("An error occurred");

	CC_SAMPLING_METHOD method = getSamplingMethod();
	float rate = (float)dataSamplingRate->value()/(float)dataSamplingRate->maximum();
	if (method == SPACE)
		rate = 1.0f-rate;
	dataSample->setSliderPosition((unsigned)((float)dataSample->maximum()*rate));

	switch(method)
	{
	case SPACE:
		{
			CCLib::ReferenceCloud* tmpCloud = getSampledData(); //DGM FIXME: wow! you generate a spatially sampled cloud just to display its size?!
			if (tmpCloud)
			{
				message = QString("distance units (%1 remaining points)").arg(tmpCloud->size());
				delete tmpCloud;
			}
		}
		break;
	case RANDOM:
		{
			message = QString("remaining points (%1%)").arg(rate*100.0f,0,'f',1);
		}
		break;
	case OCTREE:
		{
			CCLib::ReferenceCloud* tmpCloud = getSampledData(); //DGM FIXME: wow! you generate a spatially sampled cloud just to display its size?!
			if (tmpCloud)
			{
				message = QString("%1 remaining points").arg(tmpCloud->size());
				delete tmpCloud;
			}
		}
		break;
	default:
		{
			unsigned remaining = (unsigned)(rate * (float)dataObject->size());
			message = QString("%1 remaining points").arg(remaining);
		}
		break;
	}
	dataRemaining->setText(message);
}

void ccAlignDlg::estimateDelta()
{
	ccProgressDialog pDlg(false,this);

	CCLib::ReferenceCloud *sampledData = getSampledData();

	//we have to work on a copy of the cloud in order to prevent the algorithms from modifying the original cloud.
	CCLib::ChunkedPointCloud* cloud = new CCLib::ChunkedPointCloud();
	{
		cloud->reserve(sampledData->size());
		for (unsigned i=0; i<sampledData->size(); i++)
			cloud->addPoint(*sampledData->getPoint(i));
		cloud->enableScalarField();
	}

	CCLib::GeometricalAnalysisTools::computeLocalDensityApprox(cloud, CCLib::GeometricalAnalysisTools::DENSITY_KNN, &pDlg);
	unsigned count = 0;
	double meanDensity = 0;
	double meanSqrDensity = 0;
	for (unsigned i=0; i<cloud->size(); i++)
	{
		ScalarType value = cloud->getPointScalarValue(i);
		if (value == value)
		{
			meanDensity += value;
			meanSqrDensity += static_cast<double>(value)*value;
			count++;
		}
	}
	
	if (count)
	{
		meanDensity /= count;
		meanSqrDensity /= count;
	}
	double dev = meanSqrDensity - (meanDensity*meanDensity);

	delta->setValue(meanDensity+dev);
	delete sampledData;
	delete cloud;
}

void ccAlignDlg::changeSamplingMethod(int index)
{
	//Reste a changer les textes d'aide
	switch(index)
	{
	case SPACE:
		{
			modelSamplingRate->setDecimals(4);
			dataSamplingRate->setDecimals(4);
			int oldSliderPos = modelSample->sliderPosition();
			CCVector3 min, max;
			modelObject->getBoundingBox(min.u, max.u);
			double dist = (min-max).norm();
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
		}
		break;
	case RANDOM:
		{
			modelSamplingRate->setDecimals(0);
			dataSamplingRate->setDecimals(0);
			modelSamplingRate->setMaximum((float)modelObject->size());
			dataSamplingRate->setMaximum((float)dataObject->size());
			modelSamplingRate->setSingleStep(1.);
			dataSamplingRate->setSingleStep(1.);
			modelSamplingRate->setMinimum(0.);
			dataSamplingRate->setMinimum(0.);
		}
		break;
	case OCTREE:
		{
			if (!modelObject->getOctree())
				modelObject->computeOctree();
			if (!dataObject->getOctree())
				dataObject->computeOctree();
			modelSamplingRate->setDecimals(0);
			dataSamplingRate->setDecimals(0);
			modelSamplingRate->setMaximum((double)CCLib::DgmOctree::MAX_OCTREE_LEVEL);
			dataSamplingRate->setMaximum((double)CCLib::DgmOctree::MAX_OCTREE_LEVEL);
			modelSamplingRate->setMinimum(1.);
			dataSamplingRate->setMinimum(1.);
			modelSamplingRate->setSingleStep(1.);
			dataSamplingRate->setSingleStep(1.);
		}
		break;
	default:
		{
			modelSamplingRate->setDecimals(2);
			dataSamplingRate->setDecimals(2);
			modelSamplingRate->setMaximum(100.);
			dataSamplingRate->setMaximum(100.);
			modelSamplingRate->setSingleStep(0.01);
			dataSamplingRate->setSingleStep(0.01);
			modelSamplingRate->setMinimum(0.);
			dataSamplingRate->setMinimum(0.);
		}
		break;
	}

	if (index == NONE)
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
