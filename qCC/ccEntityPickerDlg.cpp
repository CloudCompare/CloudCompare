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

#include "ccEntityPickerDlg.h"

//Qt
#include <QDialog>

ccEntityPickerDlg::ccEntityPickerDlg(	const ccHObject::Container& entities,
										int selectedIndex/*=0*/,
										QWidget* parent/*=0*/,
										QString labelStr/*=QString()*/)
	: QDialog(parent)
	, Ui::PickEntityDlg()
{
	setupUi(this);

	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

	for (size_t i=0; i<entities.size(); ++i)
	{
		//add one line per entity in the combo-box
		comboBox->addItem(QString("%1 (ID=%2)").arg(entities[i]->getName()).arg(entities[i]->getUniqueID()));
	}

	if (!labelStr.isNull())
		label->setText(labelStr);
}

int ccEntityPickerDlg::getSelectedIndex() const
{
	return comboBox->currentIndex();
}

int ccEntityPickerDlg::SelectEntity(const ccHObject::Container& entities,
									int selectedIndex/*=0*/,
									QWidget* parent/*=0*/,
									QString label/*=QString()*/)
{
	ccEntityPickerDlg epDlg(entities,selectedIndex,parent,label);
	if (!epDlg.exec())
		return -1;

	return epDlg.getSelectedIndex();
}
