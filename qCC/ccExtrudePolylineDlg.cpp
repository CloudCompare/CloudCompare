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
// ##########################################################################

#include "ccExtrudePolylineDlg.h"

ccExtrudePolylineDlg::ccExtrudePolylineDlg(QWidget* parent /*=nullptr*/)
    : QDialog(parent, Qt::Tool)
    , Ui::ExtrudePolylineDialog()
{
	setupUi(this);
}

double ccExtrudePolylineDlg::heightAbove() const
{
	return heightAboveSpinBox->value();
}

double ccExtrudePolylineDlg::depthBelow() const
{
	return depthBelowSpinBox->value();
}

void ccExtrudePolylineDlg::setHeightAbove(double value)
{
	heightAboveSpinBox->setValue(value);
}

void ccExtrudePolylineDlg::setDepthBelow(double value)
{
	depthBelowSpinBox->setValue(value);
}
