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

#include "ui_stereoModeDlg.h"

//system
#include <cassert>

//combo-box items order
constexpr int COMBO_INDEX_RED_BLUE  = 0;
constexpr int COMBO_INDEX_BLUE_RED  = 1;
constexpr int COMBO_INDEX_RED_CYAN  = 2;
constexpr int COMBO_INDEX_CYAN_RED  = 3;
constexpr int COMBO_INDEX_NV_VISION = 4;
constexpr int COMBO_INDEX_OCULUS    = 5;
constexpr int COMBO_INDEX_GENERIC   = 6;

ccStereoModeDlg::ccStereoModeDlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, m_ui( new Ui::StereoModeDialog )
{
	m_ui->setupUi(this);

	glassTypeChanged(m_ui->glassTypeComboBox->currentIndex());

	connect(m_ui->glassTypeComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ccStereoModeDlg::glassTypeChanged);
}

ccStereoModeDlg::~ccStereoModeDlg()
{
	delete m_ui;
	m_ui = nullptr;
}

void ccStereoModeDlg::glassTypeChanged(int index)
{
	switch (index)
	{
	case COMBO_INDEX_RED_BLUE:
	case COMBO_INDEX_RED_CYAN:
	case COMBO_INDEX_BLUE_RED:
	case COMBO_INDEX_CYAN_RED:
		m_ui->paramsGroupBox->setEnabled(true);
		m_ui->warningTextEdit->setVisible(false);
		break;
	case COMBO_INDEX_NV_VISION:
		m_ui->paramsGroupBox->setEnabled(true);
		m_ui->warningTextEdit->setVisible(true);
		m_ui->warningTextEdit->setHtml(
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
		m_ui->paramsGroupBox->setEnabled(false);
		m_ui->warningTextEdit->setVisible(true);
		m_ui->warningTextEdit->setText(
			"To use the Oculus Rift make sure that:\
			<ul>\
			<li>the entities units are expressed in <b>meters</b> (<i>use the 'Edit > Scale' tool if necessary</i>)</li>\
			<li>position the headset in a neutral position before clicking on 'OK'</li>\
			</ul>\
			Note: this mode works best in 'bubble view' mode"
			);
		break;
	case COMBO_INDEX_GENERIC:
		m_ui->paramsGroupBox->setEnabled(true);
		m_ui->warningTextEdit->setVisible(true);
		m_ui->warningTextEdit->setHtml(
			"This mode works better with 'Auto-pick rotation center' (see left toolbar).\
			Note: the current 3D view will be automatically displayed in exclusive full screen mode (<i>press F11 to quit this mode</i>)"
		);
		break;
	default:
		assert(false);
		m_ui->paramsGroupBox->setEnabled(false);
		m_ui->warningTextEdit->setVisible(false);
		break;
	}
}

ccGLWindow::StereoParams ccStereoModeDlg::getParameters() const
{
	ccGLWindow::StereoParams params;

	//glass type
	switch (m_ui->glassTypeComboBox->currentIndex())
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
	case COMBO_INDEX_GENERIC:
		params.glassType = ccGLWindow::StereoParams::GENERIC_STEREO_DISPLAY;
		break;
	}

	//viewport
	params.screenWidth_mm = m_ui->screenWidthSpinBox->value();
	params.screenDistance_mm = m_ui->screenDiistanceSpinBox->value();
	params.eyeSeparation_mm = m_ui->eyeDistanceSpinBox->value();
	params.stereoStrength = m_ui->stereoStrengthHorizontalSlider->sliderPosition();

	return params;
}

void ccStereoModeDlg::setParameters(const ccGLWindow::StereoParams& params)
{
	//glass type
	switch (params.glassType)
	{
	case ccGLWindow::StereoParams::RED_BLUE:
		m_ui->glassTypeComboBox->setCurrentIndex(COMBO_INDEX_RED_BLUE);
		break;
	case ccGLWindow::StereoParams::BLUE_RED:
		m_ui->glassTypeComboBox->setCurrentIndex(COMBO_INDEX_BLUE_RED);
		break;
	case ccGLWindow::StereoParams::RED_CYAN:
		m_ui->glassTypeComboBox->setCurrentIndex(COMBO_INDEX_RED_CYAN);
		break;
	case ccGLWindow::StereoParams::CYAN_RED:
		m_ui->glassTypeComboBox->setCurrentIndex(COMBO_INDEX_CYAN_RED);
		break;
	case ccGLWindow::StereoParams::NVIDIA_VISION:
		m_ui->glassTypeComboBox->setCurrentIndex(COMBO_INDEX_NV_VISION); //TODO FIXME
		break;
	case ccGLWindow::StereoParams::OCULUS:
		m_ui->glassTypeComboBox->setCurrentIndex(COMBO_INDEX_OCULUS);
		break;
	case ccGLWindow::StereoParams::GENERIC_STEREO_DISPLAY:
		m_ui->glassTypeComboBox->setCurrentIndex(COMBO_INDEX_GENERIC);
		break;
	default:
		assert(false);
		break;
	}

	//viewport
	m_ui->screenWidthSpinBox->setValue(params.screenWidth_mm);
	m_ui->screenDiistanceSpinBox->setValue(params.screenDistance_mm);
	m_ui->eyeDistanceSpinBox->setValue(params.eyeSeparation_mm);
	m_ui->stereoStrengthHorizontalSlider->setSliderPosition(params.stereoStrength);
}

bool ccStereoModeDlg::updateFOV() const
{
	return m_ui->glassTypeComboBox->currentIndex() != COMBO_INDEX_OCULUS && m_ui->autoFocalCheckBox->isChecked();
}
