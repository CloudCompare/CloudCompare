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

#include "ccStereoModeDlg.h"

//system
#include <assert.h>

//combo-box items order
const int COMBO_INDEX_RED_BLUE  = 0;
const int COMBO_INDEX_BLUE_RED  = 1;
const int COMBO_INDEX_RED_CYAN  = 2;
const int COMBO_INDEX_CYAN_RED  = 3;
const int COMBO_INDEX_NV_VISION = 4;
const int COMBO_INDEX_OCULUS    = 5;

ccStereoModeDlg::ccStereoModeDlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::StereoModeDialog()
{
	setupUi(this);

	glassTypeChanged(glassTypeComboBox->currentIndex());

	connect(glassTypeComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(glassTypeChanged(int)));
}

void ccStereoModeDlg::glassTypeChanged(int index)
{
	switch (index)
	{
	case COMBO_INDEX_RED_BLUE:
	case COMBO_INDEX_RED_CYAN:
	case COMBO_INDEX_BLUE_RED:
	case COMBO_INDEX_CYAN_RED:
		paramsGroupBox->setEnabled(true);
		warningTextEdit->setVisible(false);
		break;
	case COMBO_INDEX_NV_VISION:
		paramsGroupBox->setEnabled(true);
		warningTextEdit->setVisible(true);
		warningTextEdit->setHtml(
			"To make this mode work properly make sure that:\
			<ul>\
			<li>the NVidia Vision IR emitter is plugged and enabled (<i>dim green light</i>)</li>\
			<li>3D stereo mode is activated in the NVidia Control Pannel</li>\
			<li><b>the screen frequency is set to 120Hz</b></li>\
			<li>the glasses are switched on</li>\
			</ul>\
			Note: the current 3D view will be automatically displayed in exclusive full screen mode (<i>press F11 to quit this mode</i>)"
			);
		break;
	case COMBO_INDEX_OCULUS:
		paramsGroupBox->setEnabled(false);
		warningTextEdit->setVisible(true);
		warningTextEdit->setText(
			"To use the Oculus Rift make sure that:\
			<ul>\
			<li>the entities units are expressed in <b>meters</b> (<i>use the 'Edit > Scale' tool if necessary</i>)</li>\
			<li>position the headset in a neutral position before clicking on 'OK'</li>\
			</ul>\
			Note: this mode works best in 'bubble view' mode"
			);
		break;
	default:
		assert(false);
		paramsGroupBox->setEnabled(false);
		warningTextEdit->setVisible(false);
		break;
	}
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
	case COMBO_INDEX_BLUE_RED:
		params.glassType = ccGLWindow::StereoParams::BLUE_RED;
		break;
	case COMBO_INDEX_RED_CYAN:
	default:
		params.glassType = ccGLWindow::StereoParams::RED_CYAN;
		break;
	case COMBO_INDEX_CYAN_RED:
		params.glassType = ccGLWindow::StereoParams::CYAN_RED;
		break;
	case COMBO_INDEX_NV_VISION:
		params.glassType = ccGLWindow::StereoParams::NVIDIA_VISION;
		break;
	case COMBO_INDEX_OCULUS:
		params.glassType = ccGLWindow::StereoParams::OCULUS;
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
	case ccGLWindow::StereoParams::BLUE_RED:
		glassTypeComboBox->setCurrentIndex(COMBO_INDEX_BLUE_RED);
		break;
	case ccGLWindow::StereoParams::RED_CYAN:
		glassTypeComboBox->setCurrentIndex(COMBO_INDEX_RED_CYAN);
		break;
	case ccGLWindow::StereoParams::CYAN_RED:
		glassTypeComboBox->setCurrentIndex(COMBO_INDEX_CYAN_RED);
		break;
	case ccGLWindow::StereoParams::NVIDIA_VISION:
		glassTypeComboBox->setCurrentIndex(COMBO_INDEX_NV_VISION);
		break;
	case ccGLWindow::StereoParams::OCULUS:
		glassTypeComboBox->setCurrentIndex(COMBO_INDEX_OCULUS);
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