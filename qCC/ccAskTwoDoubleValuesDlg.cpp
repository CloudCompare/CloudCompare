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

#include "ccAskTwoDoubleValuesDlg.h"

ccAskTwoDoubleValuesDlg::ccAskTwoDoubleValuesDlg(	const char* vName1,
													const char* vName2,
													double minVal,
													double maxVal,
													double defaultVal1,
													double defaultVal2,
													int precision/*=6*/,
													const char* windowTitle/*=0*/,
													QWidget* parent/*=0*/)
	: QDialog(parent)
	, Ui::AskTwoDoubleValuesDialog()
{
	setupUi(this);

	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

	label1->setText(vName1);
	label2->setText(vName2);
	doubleSpinBox1->setDecimals(precision);
	doubleSpinBox2->setDecimals(precision);
	doubleSpinBox1->setRange(minVal,maxVal);
	doubleSpinBox2->setRange(minVal,maxVal);
	doubleSpinBox1->setValue(defaultVal1);
	doubleSpinBox2->setValue(defaultVal2);

	if (windowTitle)
		setWindowTitle(windowTitle);
}
