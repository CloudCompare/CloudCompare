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

#include "ccHeightGridGenerationDlg.h"

//Local
#include "ccBoundingBoxEditorDlg.h"

//Qt
#include <QSettings>
#include <QPushButton>

//System
#include <assert.h>

ccHeightGridGenerationDlg::ccHeightGridGenerationDlg(const ccBBox& gridBBox, QWidget* parent/*=0*/)
	: QDialog(parent)
	, Ui::HeightGridGenerationDialog()
	, m_bbEditorDlg(0)
{
	setupUi(this);

	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

#ifndef CC_GDAL_SUPPORT
	generateRasterCheckBox->setDisabled(true);
	generateRasterCheckBox->setChecked(false);
#endif

	connect(buttonBox,					SIGNAL(accepted()),					this,	SLOT(saveSettings()));
	connect(fillEmptyCells,				SIGNAL(currentIndexChanged(int)),	this,	SLOT(projectionChanged(int)));
	connect(generateCloudGroupBox,		SIGNAL(toggled(bool)),				this,	SLOT(toggleFillEmptyCells(bool)));
	connect(generateImageCheckBox,		SIGNAL(toggled(bool)),				this,	SLOT(toggleFillEmptyCells(bool)));
	connect(generateRasterCheckBox,		SIGNAL(toggled(bool)),				this,	SLOT(toggleFillEmptyCells(bool)));
	connect(generateASCIICheckBox,		SIGNAL(toggled(bool)),				this,	SLOT(toggleFillEmptyCells(bool)));
	connect(typeOfProjectionComboBox,	SIGNAL(currentIndexChanged(int)),	this,	SLOT(projectionTypeChanged(int)));

	//custom bbox editor
	if (gridBBox.isValid())
	{
		m_bbEditorDlg = new ccBoundingBoxEditorDlg(this);
		m_bbEditorDlg->setBaseBBox(gridBBox,false);
		connect(editGridToolButton, SIGNAL(clicked()), this, SLOT(showGridBoxEditor()));
	}
	else
	{
		editGridToolButton->setEnabled(false);
	}

	loadSettings();
}

void ccHeightGridGenerationDlg::projectionTypeChanged(int index)
{
	//we can't use the 'resample origin cloud' option with 'average height' projection
	resampleOriginalCloudCheckBox->setEnabled(index != 1);
}

void ccHeightGridGenerationDlg::showGridBoxEditor()
{
	if (m_bbEditorDlg)
	{
		assert(dimensionComboBox->currentIndex() < 255);
		m_bbEditorDlg->set2DMode(true,static_cast<uchar>(dimensionComboBox->currentIndex()));
		m_bbEditorDlg->exec();
	}
}

ccBBox ccHeightGridGenerationDlg::getCustomBBox() const
{
	return (m_bbEditorDlg ? m_bbEditorDlg->getBox() : ccBBox());
}

double ccHeightGridGenerationDlg::getGridStep() const
{
	return gridStep->value();
}

bool ccHeightGridGenerationDlg::generateCloud() const
{
	return generateCloudGroupBox->isChecked();
}

bool ccHeightGridGenerationDlg::generateCountSF() const
{
	return generateCloud() && generateCountSFcheckBox->isChecked();
}

bool ccHeightGridGenerationDlg::resampleOriginalCloud() const
{
	return generateCloud() && resampleOriginalCloudCheckBox->isEnabled() && resampleOriginalCloudCheckBox->isChecked();
}

bool ccHeightGridGenerationDlg::generateImage() const
{
	return generateImageCheckBox->isChecked();
}

bool ccHeightGridGenerationDlg::generateRaster() const
{
	return generateRasterCheckBox->isChecked();
}

bool ccHeightGridGenerationDlg::generateASCII() const
{
	return generateASCIICheckBox->isChecked();
}

unsigned char ccHeightGridGenerationDlg::getProjectionDimension() const
{
	int dim = dimensionComboBox->currentIndex();
	assert(dim>=0 && dim<3);

	return (unsigned char)dim;
}

void ccHeightGridGenerationDlg::projectionChanged(int)
{
	emptyValueDoubleSpinBox->setEnabled(getFillEmptyCellsStrategy() == ccHeightGridGeneration::FILL_CUSTOM_HEIGHT);
}

void ccHeightGridGenerationDlg::toggleFillEmptyCells(bool)
{
	emptyCellsFrame->setEnabled(generateCloudGroupBox->isChecked() || generateImageCheckBox->isChecked() || generateRasterCheckBox->isChecked());
}

double ccHeightGridGenerationDlg::getCustomHeightForEmptyCells() const
{
	return emptyValueDoubleSpinBox->value();
}

ccHeightGridGeneration::ProjectionType ccHeightGridGenerationDlg::getTypeOfProjection() const
{
	switch (typeOfProjectionComboBox->currentIndex())
	{
	case 0:
		return ccHeightGridGeneration::PROJ_MINIMUM_HEIGHT;
	case 1:
		return ccHeightGridGeneration::PROJ_AVERAGE_HEIGHT;
	case 2:
		return ccHeightGridGeneration::PROJ_MAXIMUM_HEIGHT;
	default:
		//shouldn't be possible for this option!
		assert(false);
	}

	return ccHeightGridGeneration::INVALID_PROJECTION_TYPE;
}

ccHeightGridGeneration::ProjectionType ccHeightGridGenerationDlg::getTypeOfSFInterpolation() const
{
	if (!interpolateSFFrame->isEnabled() || !interpolateSFCheckBox->isChecked())
		return ccHeightGridGeneration::INVALID_PROJECTION_TYPE; //means that we don't want to keep SF values

	switch (scalarFieldProjection->currentIndex())
	{
	case 0:
		return ccHeightGridGeneration::PROJ_MINIMUM_HEIGHT;
	case 1:
		return ccHeightGridGeneration::PROJ_AVERAGE_HEIGHT;
	case 2:
		return ccHeightGridGeneration::PROJ_MAXIMUM_HEIGHT;
	default:
		//shouldn't be possible for this option!
		assert(false);
	}

	return ccHeightGridGeneration::INVALID_PROJECTION_TYPE;
}

ccHeightGridGeneration::EmptyCellFillOption ccHeightGridGenerationDlg::getFillEmptyCellsStrategy() const
{
	switch (fillEmptyCells->currentIndex())
	{
	case 0:
		return ccHeightGridGeneration::LEAVE_EMPTY;
	case 1:
		return ccHeightGridGeneration::FILL_MINIMUM_HEIGHT;
	case 2:
		return ccHeightGridGeneration::FILL_AVERAGE_HEIGHT;
	case 3:
		return ccHeightGridGeneration::FILL_MAXIMUM_HEIGHT;
	case 4:
		return ccHeightGridGeneration::FILL_CUSTOM_HEIGHT;
	default:
		//shouldn't be possible for this option!
		assert(false);
	}

	return ccHeightGridGeneration::LEAVE_EMPTY;
}

void ccHeightGridGenerationDlg::loadSettings()
{
	QSettings settings;
	settings.beginGroup("HeightGridGeneration");
	int projType		= settings.value("ProjectionType",typeOfProjectionComboBox->currentIndex()).toInt();
	int fillStrategy	= settings.value("FillStrategy",fillEmptyCells->currentIndex()).toInt();
	bool sfProj			= settings.value("SfProjEnabled",interpolateSFCheckBox->isChecked()).toBool();
	int sfProjStrategy	= settings.value("SfProjStrategy",scalarFieldProjection->currentIndex()).toInt();
	int projDim			= settings.value("ProjectionDim",dimensionComboBox->currentIndex()).toInt();
	double step			= settings.value("GridStep",gridStep->value()).toDouble();
	double emptyHeight	= settings.value("EmptyCellsHeight",emptyValueDoubleSpinBox->value()).toDouble();
	bool genCloud		= settings.value("GenerateCloud",generateCloudGroupBox->isChecked()).toBool();
	bool genImage		= settings.value("GenerateImage",generateImageCheckBox->isChecked()).toBool();
#ifdef CC_GDAL_SUPPORT
	bool genRaster		= settings.value("GenerateRaster",generateImageCheckBox->isChecked()).toBool();
#endif
	bool genASCII		= settings.value("GenerateASCII",generateASCIICheckBox->isChecked()).toBool();
	bool genCountSF		= settings.value("GenerateCountSF",generateCountSFcheckBox->isChecked()).toBool();
	bool resampleCloud	= settings.value("ResampleOrigCloud",resampleOriginalCloudCheckBox->isChecked()).toBool();
	settings.endGroup();

	gridStep->setValue(step);
	typeOfProjectionComboBox->setCurrentIndex(projType);
	fillEmptyCells->setCurrentIndex(fillStrategy);
	emptyValueDoubleSpinBox->setValue(emptyHeight);
	generateCloudGroupBox->setChecked(genCloud);
	generateImageCheckBox->setChecked(genImage);
	generateASCIICheckBox->setChecked(genASCII);
	dimensionComboBox->setCurrentIndex(projDim);
	interpolateSFCheckBox->setChecked(sfProj);
	scalarFieldProjection->setCurrentIndex(sfProjStrategy);
	generateCountSFcheckBox->setChecked(genCountSF);
	resampleOriginalCloudCheckBox->setChecked(resampleCloud);
#ifdef CC_GDAL_SUPPORT
	generateRasterCheckBox->setChecked(genRaster);
#endif

	toggleFillEmptyCells(false);
}

void ccHeightGridGenerationDlg::saveSettings()
{
	QSettings settings;
	settings.beginGroup("HeightGridGeneration");
	settings.setValue("ProjectionType",typeOfProjectionComboBox->currentIndex());
	settings.setValue("ProjectionDim",dimensionComboBox->currentIndex());
	settings.setValue("SfProjEnabled",interpolateSFCheckBox->isChecked());
	settings.setValue("SfProjStrategy",scalarFieldProjection->currentIndex());
	settings.setValue("FillStrategy",fillEmptyCells->currentIndex());
	settings.setValue("GridStep",gridStep->value());
	settings.setValue("EmptyCellsHeight",emptyValueDoubleSpinBox->value());
	settings.setValue("GenerateCloud",generateCloudGroupBox->isChecked());
	settings.setValue("GenerateImage",generateImageCheckBox->isChecked());
	settings.setValue("GenerateASCII",generateASCIICheckBox->isChecked());
	settings.setValue("GenerateCountSF",generateCountSFcheckBox->isChecked());
	settings.setValue("ResampleOrigCloud",resampleOriginalCloudCheckBox->isChecked());
#ifdef CC_GDAL_SUPPORT
	settings.setValue("GenerateRaster",generateRasterCheckBox->isChecked());
#endif
	settings.endGroup();

	accept();
}
