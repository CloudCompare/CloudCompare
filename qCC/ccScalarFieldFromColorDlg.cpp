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

#include "ccScalarFieldFromColorDlg.h"

//Qt
#include <QPushButton>

//qCC_db
#include <ccPointCloud.h>

#include <assert.h>
#ifdef _MSC_VER
#include <windows.h>
#endif

ccScalarFieldFromColorDlg::ccScalarFieldFromColorDlg(QWidget* parent/*=0*/)
	: QDialog(parent)
	, Ui::scalarFieldFromColorDlg()
{
	setupUi(this);
	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);
}

bool ccScalarFieldFromColorDlg::getRStatus()
{
	return this->checkBoxR->isChecked();
}

bool ccScalarFieldFromColorDlg::getGStatus()
{
	return this->checkBoxG->isChecked();
}

bool ccScalarFieldFromColorDlg::getBStatus()
{
	return this->checkBoxB->isChecked();
}

bool ccScalarFieldFromColorDlg::getCompositeStatus()
{
	return this->checkBoxComposite->isChecked();
}
