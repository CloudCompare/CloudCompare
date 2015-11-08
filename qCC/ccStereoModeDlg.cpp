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

#include "ccStereoModeDlg.h"

//system
#include <assert.h>

//combo-box items order
const int COMBO_INDEX_RED_BLUE  = 0;
const int COMBO_INDEX_RED_CYAN  = 1;
const int COMBO_INDEX_NV_VISION = 2;

ccStereoModeDlg::ccStereoModeDlg(QWidget* parent)
	: QDialog(parent)
	, Ui::StereoModeDialog()
{
	setupUi(this);

	glassTypeChanged(glassTypeComboBox->currentIndex());
	setWindowFlags(Qt::Tool);

	connect(glassTypeComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(glassTypeChanged(int)));
}

void ccStereoModeDlg::glassTypeChanged(int index)
{
	NVVisionWarningTextEdit->setVisible(index == COMBO_INDEX_NV_VISION);
}

ccGLWindow::StereoParams ccStereoModeDlg::getParameters() const
{
	ccGLWindow::StereoParams params;

	//glass type
	switch (glassTypeComboBox->currentIndex())
	{
	case COMBO_INDEX_RED_BLUE:
		params.glassType = ccGLWindow::StereoParams::RED_BLUE;
		break;
	case COMBO_INDEX_RED_CYAN:
	default:
		params.glassType = ccGLWindow::StereoParams::RED_CYAN;
		break;
	case COMBO_INDEX_NV_VISION:
		params.glassType = ccGLWindow::StereoParams::NVIDIA_VISION;
		break;
	}

	//focal
	params.autoFocal = autoFocalCheckBox->isChecked();
	params.focalDist = focalDoubleSpinBox->value();

	//eye separation
	params.eyeSepFactor = eyeSepDoubleSpinBox->value();

	return params;
}

void ccStereoModeDlg::setParameters(const ccGLWindow::StereoParams& params)
{
	//glass type
	switch (params.glassType)
	{
	case ccGLWindow::StereoParams::RED_BLUE:
		glassTypeComboBox->setCurrentIndex(COMBO_INDEX_RED_BLUE);
		break;
	case ccGLWindow::StereoParams::RED_CYAN:
		glassTypeComboBox->setCurrentIndex(COMBO_INDEX_RED_CYAN);
		break;
	case ccGLWindow::StereoParams::NVIDIA_VISION:
		glassTypeComboBox->setCurrentIndex(COMBO_INDEX_NV_VISION);
		break;
	default:
		assert(false);
		break;
	}

	//focal
	autoFocalCheckBox->setChecked(params.autoFocal);
	focalDoubleSpinBox->setValue(params.focalDist);

	//eye separation
	eyeSepDoubleSpinBox->setValue(params.eyeSepFactor);
}