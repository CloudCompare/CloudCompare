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

#include "ccPickOneElementDlg.h"

ccPickOneElementDlg::ccPickOneElementDlg(	QString label,
											QString windowTitle/*=QString()*/,
											QWidget* parent/*=0*/)
	: QDialog(parent)
	, Ui::PickOneElementDialog()
{
	setupUi(this);

	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

	if (!windowTitle.isNull())
		setWindowTitle(windowTitle);

	comboLabel->setText(label);
}

void ccPickOneElementDlg::addElement(QString elementName)
{
	comboBox->addItem(elementName);
}

void ccPickOneElementDlg::setDefaultIndex(int index)
{
	comboBox->setCurrentIndex(index);
}

int ccPickOneElementDlg::getSelectedIndex()
{
	return comboBox->currentIndex();
}
