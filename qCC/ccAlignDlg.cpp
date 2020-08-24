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

#include "ccAlignDlg.h"
#include "mainwindow.h"

//common
#include <ccQtHelpers.h>

//CCLib
#include <CloudSamplingTools.h>
#include <GeometricalAnalysisTools.h>
#include <DgmOctree.h>
#include <ReferenceCloud.h>
#include <PointCloud.h>

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccProgressDialog.h>

ccAlignDlg::ccAlignDlg(ccGenericPointCloud *data, ccGenericPointCloud *model, QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::AlignDialog()
{
	setupUi(this);

	samplingMethod->addItem("None");
	samplingMethod->addItem("Random");
	samplingMethod->addItem("Space");
	samplingMethod->addItem("Octree");
	samplingMethod->setCurrentIndex(NONE);

	ccQtHelpers::SetButtonColor(dataColorButton, Qt::red);
	ccQtHelpers::SetButtonColor(modelColorButton, Qt::yellow);

	dataObject = data;
	modelObject = model;
	setColorsAndLabels();

	changeSamplingMethod(samplingMethod->currentIndex());
	toggleNbMaxCandidates(isNbCandLimited->isChecked());

	connect(swapButton, &QPushButton::clicked, this, &ccAlignDlg::swapModelAndData);
	connect(modelSample, &QSlider::sliderReleased, this, &ccAlignDlg::modelSliderReleased);
	connect(dataSample,  &QSlider::sliderReleased, this, &ccAlignDlg::dataSliderReleased);
	connect(modelSamplingRate, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccAlignDlg::modelSamplingRateChanged);
	connect(dataSamplingRate,  static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccAlignDlg::dataSamplingRateChanged);
	connect(deltaEstimation, &QPushButton::clicked, this, &ccAlignDlg::estimateDelta);
	connect(samplingMethod, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ccAlignDlg::changeSamplingMethod);
	connect(isNbCandLimited, &QCheckBox::toggled, this, &ccAlignDlg::toggleNbMaxCandidates);
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
	CCLib::ReferenceCloud* sampledCloud = nullptr;

	switch (getSamplingMethod())
	{
	case SPACE:
		{
			CCLib::CloudSamplingTools::SFModulationParams modParams(false);
			sampledCloud = CCLib::CloudSamplingTools::resampleCloudSpatially(	modelObject,
																				static_cast<PointCoordinateType>(modelSamplingRate->value()),
																				modParams);
		}
		break;
	case OCTREE:
		if (modelObject->getOctree())
		{
			sampledCloud = CCLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(	modelObject,
																						static_cast<unsigned char>(modelSamplingRate->value()),
																						CCLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER,
																						nullptr,
																						modelObject->getOctree().data());
		}
		else
		{
			ccLog::Error("[ccAlignDlg::getSampledModel] Failed to get/compute model octree!");
		}
		break;
	case RANDOM:
		{
			sampledCloud = CCLib::CloudSamplingTools::subsampleCloudRandomly(	modelObject,
																				static_cast<unsigned>(modelSamplingRate->value()));
		}
		break;
	default:
		{
			sampledCloud = new CCLib::ReferenceCloud(modelObject);
			if (!sampledCloud->addPointIndex(0, modelObject->size()))
			{
				delete sampledCloud;
				sampledCloud = nullptr;
				ccLog::Error("[ccAlignDlg::getSampledModel] Not enough memory!");
			}
		}
		break;
	}

	return sampledCloud;
}

CCLib::ReferenceCloud *ccAlignDlg::getSampledData()
{
	CCLib::ReferenceCloud* sampledCloud = nullptr;

	switch (getSamplingMethod())
	{
	case SPACE:
		{
			CCLib::CloudSamplingTools::SFModulationParams modParams(false);
			sampledCloud = CCLib::CloudSamplingTools::resampleCloudSpatially(dataObject, static_cast<PointCoordinateType>(dataSamplingRate->value()),modParams);
		}
		break;
	case OCTREE:
		if (dataObject->getOctree())
		{
			sampledCloud = CCLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(	dataObject,
																						static_cast<unsigned char>(dataSamplingRate->value()),
																						CCLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER,
																						nullptr,
																						dataObject->getOctree().data());
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
				sampledCloud = nullptr;
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

	MainWindow::RefreshAllGLWindow(false);
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
	double rate = static_cast<double>(modelSample->sliderPosition())/modelSample->maximum();
	if ( getSamplingMethod() == SPACE)
		rate = 1.0 - rate;
	rate *= modelSamplingRate->maximum();
	modelSamplingRate->setValue(rate);
	modelSamplingRateChanged(rate);
}

void ccAlignDlg::dataSliderReleased()
{
	double rate = static_cast<double>(dataSample->sliderPosition())/dataSample->maximum();
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
	float rate = static_cast<float>(modelSamplingRate->value())/modelSamplingRate->maximum();
	if (method == SPACE)
		rate = 1.0f-rate;
	modelSample->setSliderPosition(static_cast<int>(rate * modelSample->maximum()));

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
			unsigned remaining = static_cast<unsigned>(rate * modelObject->size());
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
	double rate = static_cast<float>(dataSamplingRate->value()/dataSamplingRate->maximum());
	if (method == SPACE)
		rate = 1.0 - rate;
	dataSample->setSliderPosition(static_cast<int>(rate * dataSample->maximum()));

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
			message = QString("remaining points (%1%)").arg(rate*100.0,0,'f',1);
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
			unsigned remaining = static_cast<unsigned>(rate * dataObject->size());
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
	CCLib::PointCloud cloud;
	{
		cloud.reserve(sampledData->size());
		for (unsigned i = 0; i < sampledData->size(); i++)
		{
			cloud.addPoint(*sampledData->getPoint(i));
		}
		if (!cloud.enableScalarField())
		{
			ccLog::Error("Not enough memory");
			return;
		}
	}

	if (CCLib::GeometricalAnalysisTools::ComputeLocalDensityApprox(&cloud, CCLib::GeometricalAnalysisTools::DENSITY_KNN, &pDlg) != CCLib::GeometricalAnalysisTools::NoError)
	{
		ccLog::Error("Failed to compute approx. density");
		return;
	}
	unsigned count = 0;
	double meanDensity = 0;
	double meanSqrDensity = 0;
	for (unsigned i = 0; i < cloud.size(); i++)
	{
		ScalarType value = cloud.getPointScalarValue(i);
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

	delta->setValue(meanDensity + dev);
	delete sampledData;
}

void ccAlignDlg::changeSamplingMethod(int index)
{
	//Reste a changer les textes d'aide
	switch (index)
	{
	case SPACE:
		{
			//model
			{
				modelSamplingRate->setDecimals(4);
				int oldSliderPos = modelSample->sliderPosition();
				CCVector3 bbMin;
				CCVector3 bbMax;
				modelObject->getBoundingBox(bbMin, bbMax);
				double dist = (bbMin-bbMax).norm();
				modelSamplingRate->setMaximum(dist);
				modelSample->setSliderPosition(oldSliderPos);
				modelSamplingRate->setSingleStep(0.01);
				modelSamplingRate->setMinimum(0.);
			}
			//data
			{
				dataSamplingRate->setDecimals(4);
				int oldSliderPos = dataSample->sliderPosition();
				CCVector3 bbMin;
				CCVector3 bbMax;
				dataObject->getBoundingBox(bbMin, bbMax);
				double dist = (bbMin-bbMax).norm();
				dataSamplingRate->setMaximum(dist);
				dataSample->setSliderPosition(oldSliderPos);
				dataSamplingRate->setSingleStep(0.01);
				dataSamplingRate->setMinimum(0.);
			}
		}
		break;
	case RANDOM:
		{
			//model
			{
				modelSamplingRate->setDecimals(0);
				modelSamplingRate->setMaximum(static_cast<float>(modelObject->size()));
				modelSamplingRate->setSingleStep(1.);
				modelSamplingRate->setMinimum(0.);
			}
			//data
			{
				dataSamplingRate->setDecimals(0);
				dataSamplingRate->setMaximum(static_cast<float>(dataObject->size()));
				dataSamplingRate->setSingleStep(1.);
				dataSamplingRate->setMinimum(0.);
			}
		}
		break;
	case OCTREE:
		{
			//model
			{
				if (!modelObject->getOctree())
					modelObject->computeOctree();
				modelSamplingRate->setDecimals(0);
				modelSamplingRate->setMaximum(static_cast<double>(CCLib::DgmOctree::MAX_OCTREE_LEVEL));
				modelSamplingRate->setMinimum(1.);
				modelSamplingRate->setSingleStep(1.);
			}
			//data
			{
				if (!dataObject->getOctree())
					dataObject->computeOctree();
				dataSamplingRate->setDecimals(0);
				dataSamplingRate->setMaximum(static_cast<double>(CCLib::DgmOctree::MAX_OCTREE_LEVEL));
				dataSamplingRate->setMinimum(1.);
				dataSamplingRate->setSingleStep(1.);
			}
		}
		break;
	default:
		{
			//model
			{
				modelSamplingRate->setDecimals(2);
				modelSamplingRate->setMaximum(100.);
				modelSamplingRate->setSingleStep(0.01);
				modelSamplingRate->setMinimum(0.);
			}
			//data
			{
				dataSamplingRate->setDecimals(2);
				dataSamplingRate->setMaximum(100.);
				dataSamplingRate->setSingleStep(0.01);
				dataSamplingRate->setMinimum(0.);
			}
		}
		break;
	}

	if (index == NONE)
	{
		//model
		modelSample->setSliderPosition(modelSample->maximum());
		modelSample->setEnabled(false);
		modelSamplingRate->setEnabled(false);
		//data
		dataSample->setSliderPosition(dataSample->maximum());
		dataSample->setEnabled(false);
		dataSamplingRate->setEnabled(false);
	}
	else
	{
		//model
		modelSample->setEnabled(true);
		modelSamplingRate->setEnabled(true);
		//data
		dataSample->setEnabled(true);
		dataSamplingRate->setEnabled(true);
	}

	modelSliderReleased();
	dataSliderReleased();
}

void ccAlignDlg::toggleNbMaxCandidates(bool activ)
{
	nbMaxCandidates->setEnabled(activ);
}
