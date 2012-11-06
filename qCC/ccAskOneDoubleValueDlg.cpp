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
//$Rev:: 1987                                                              $
//$LastChangedDate:: 2012-01-13 00:55:14 +0100 (ven., 13 janv. 2012)       $
//**************************************************************************
//

#include "ccAskOneDoubleValueDlg.h"

ccAskOneDoubleValueDlg::ccAskOneDoubleValueDlg(const char* valueName,
											   double minVal,
											   double maxVal,
											   double defaultVal,
											   int precision/*=6*/,
											   const char* windowTitle/*=0*/,
											   QWidget* parent/*=0*/)
	: QDialog(parent)
	, Ui::AskOneDoubleValueDialog()
{
	setupUi(this);

	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

	valueLabel->setText(valueName);
	dValueSpinBox->setRange(minVal,maxVal);
	dValueSpinBox->setDecimals(precision);
	dValueSpinBox->setValue(defaultVal);

	if (windowTitle)
		setWindowTitle(windowTitle);
}
