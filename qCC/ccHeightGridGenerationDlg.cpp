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
//$Rev:: 2274                                                              $
//$LastChangedDate:: 2012-10-17 19:17:38 +0200 (mer., 17 oct. 2012)        $
//**************************************************************************
//

#include "ccHeightGridGenerationDlg.h"

#include <QSettings>
#include <QPushButton>

#include <assert.h>

ccHeightGridGenerationDlg::ccHeightGridGenerationDlg(QWidget* parent/*=0*/)
    : QDialog(parent), Ui::HeightGridGenerationDialog()
{
    setupUi(this);

    setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

    connect(buttonBox, SIGNAL(accepted()), this, SLOT(saveSettings()));
    connect(fillEmptyCells, SIGNAL(currentIndexChanged(int)), this, SLOT(projectionChanged(int)));
    connect(generateCloudCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleFillEmptyCells(bool)));
    connect(generateImageCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleFillEmptyCells(bool)));
    connect(generateASCIICheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleFillEmptyCells(bool)));

    loadSettings();
}

double ccHeightGridGenerationDlg::getGridStep() const
{
    return gridStep->value();
}

bool ccHeightGridGenerationDlg::generateCloud() const
{
    return generateCloudCheckBox->isChecked();
}

bool ccHeightGridGenerationDlg::generateImage() const
{
    return generateImageCheckBox->isChecked();
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
    emptyCellsFrame->setEnabled(generateCloudCheckBox->isChecked() || generateImageCheckBox->isChecked());
}

double ccHeightGridGenerationDlg::getCustomHeightForEmptyCells() const
{
    return emptyValueDoubleSpinBox->value();
}

ccHeightGridGeneration::ProjectionType ccHeightGridGenerationDlg::getTypeOfProjection() const
{
    switch (typeOfProjection->currentIndex())
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
    int projType        = settings.value("ProjectionType",typeOfProjection->currentIndex()).toInt();
    int fillStrategy    = settings.value("FillStrategy",fillEmptyCells->currentIndex()).toInt();
	bool sfProj			= settings.value("SfProjEnabled",interpolateSFCheckBox->isChecked()).toBool();
    int sfProjStrategy  = settings.value("SfProjStrategy",scalarFieldProjection->currentIndex()).toInt();
    int projDim			= settings.value("ProjectionDim",dimensionComboBox->currentIndex()).toInt();
    double step         = settings.value("GridStep",gridStep->value()).toDouble();
    double emptyHeight  = settings.value("EmptyCellsHeight",emptyValueDoubleSpinBox->value()).toDouble();
    bool genCloud       = settings.value("GenerateCloud",generateCloudCheckBox->isChecked()).toBool();
    bool genImage       = settings.value("GenerateImage",generateImageCheckBox->isChecked()).toBool();
    bool genASCII       = settings.value("GenerateASCII",generateASCIICheckBox->isChecked()).toBool();
    settings.endGroup();

    gridStep->setValue(step);
    typeOfProjection->setCurrentIndex(projType);
    fillEmptyCells->setCurrentIndex(fillStrategy);
    emptyValueDoubleSpinBox->setValue(emptyHeight);
    generateCloudCheckBox->setChecked(genCloud);
    generateImageCheckBox->setChecked(genImage);
    generateASCIICheckBox->setChecked(genASCII);
	dimensionComboBox->setCurrentIndex(projDim);
	interpolateSFCheckBox->setChecked(sfProj);
	scalarFieldProjection->setCurrentIndex(sfProjStrategy);

    toggleFillEmptyCells(false);
}

void ccHeightGridGenerationDlg::saveSettings()
{
    QSettings settings;
    settings.beginGroup("HeightGridGeneration");
    settings.setValue("ProjectionType",typeOfProjection->currentIndex());
    settings.setValue("ProjectionDim",dimensionComboBox->currentIndex());
	settings.setValue("SfProjEnabled",interpolateSFCheckBox->isChecked());
    settings.setValue("SfProjStrategy",scalarFieldProjection->currentIndex());
    settings.setValue("FillStrategy",fillEmptyCells->currentIndex());
    settings.setValue("GridStep",gridStep->value());
    settings.setValue("EmptyCellsHeight",emptyValueDoubleSpinBox->value());
    settings.setValue("GenerateCloud",generateCloudCheckBox->isChecked());
    settings.setValue("GenerateImage",generateImageCheckBox->isChecked());
    settings.setValue("GenerateASCII",generateASCIICheckBox->isChecked());
    settings.endGroup();

    accept();
}
