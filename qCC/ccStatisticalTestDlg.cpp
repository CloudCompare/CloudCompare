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

#include "ccStatisticalTestDlg.h"

ccStatisticalTestDlg::ccStatisticalTestDlg(	QString p1Label,
											QString p2Label,
											QString p3Label/*=QString()*/,
											QString windowTitle/*=QString()*/,
											QWidget* parent/*=0*/)
	: QDialog(parent, Qt::Tool)
	, Ui::StatisticalTestDialog()
{
	setupUi(this);

	param1Label->setText(p1Label);
	param2Label->setText(p2Label);
	if (!p3Label.isNull())
	{
		param3Label->setText(p3Label);
	}
	else
	{
		param3Label->setVisible(false);
		param3SpinBox->setVisible(false);
	}

	neighborsSpinBox->setValue(16);

	if (!windowTitle.isNull())
		setWindowTitle(windowTitle);
}

double ccStatisticalTestDlg::getParam1() const
{
	return param1SpinBox->value();
}

double ccStatisticalTestDlg::getParam2() const
{
	return param2SpinBox->value();
}

double ccStatisticalTestDlg::getParam3() const
{
	return param3SpinBox->value();
}

int ccStatisticalTestDlg::getNeighborsNumber() const
{
	return neighborsSpinBox->value();
}

double ccStatisticalTestDlg::getProba() const
{
	return probaSpinBox->value();
}
