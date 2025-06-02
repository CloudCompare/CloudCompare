// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #                  COPYRIGHT: Daniel Girardeau-Montaut                   #
// #                                                                        #
// ##########################################################################

#include "ccFitSphereDlg.h"

ccFitSphereDlg::ccFitSphereDlg(double   maxOutliersRatio,
                               double   confidence,
                               bool     autoDetectSphereRadius,
                               double   sphereRadius,
                               QWidget* parent /*=nullptr*/)
    : QDialog(parent)
    , Ui::FitSphereDialog()
{
	setupUi(this);

	outliersPercentDoubleSpinBox->setValue(maxOutliersRatio * 100.0);
	confidenceDoubleSpinBox->setValue(confidence * 100.0);
	autoDetectRadiusCheckBox->setChecked(autoDetectSphereRadius);
	radiusDoubleSpinBox->setValue(sphereRadius);
}

double ccFitSphereDlg::maxOutliersRatio() const
{
	return outliersPercentDoubleSpinBox->value() / 100.0;
}

double ccFitSphereDlg::confidence() const
{
	return confidenceDoubleSpinBox->value() / 100.0;
}

bool ccFitSphereDlg::autoDetectSphereRadius() const
{
	return autoDetectRadiusCheckBox->isChecked();
}

double ccFitSphereDlg::sphereRadius() const
{
	return radiusDoubleSpinBox->value();
}
