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

ccStereoModeDlg::ccStereoModeDlg(QWidget* parent)
	: QDialog(parent)
	, Ui::StereoModeDialog()
{
	setupUi(this);

	setWindowFlags(Qt::Tool);
}

ccGLWindow::StereoParams ccStereoModeDlg::getParameters() const
{
	ccGLWindow::StereoParams params;

	//glass type
	switch (glassTypeComboBox->currentIndex())
	{
	case 0:

		params.glassType = ccGLWindow::StereoParams::RED_BLUE;
		break;
	case 1:
	default:
		params.glassType = ccGLWindow::StereoParams::RED_CYAN;
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
		glassTypeComboBox->setCurrentIndex(0);
		break;
	case ccGLWindow::StereoParams::RED_CYAN:
		glassTypeComboBox->setCurrentIndex(1);
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