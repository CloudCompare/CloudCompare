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
#include <QListWidgetItem>

ccEntityPickerDlg::ccEntityPickerDlg(	const ccHObject::Container& entities,
										bool multiSelectionEnabled,
										int defaultSelectedIndex/*=0*/,
										QWidget* parent/*=0*/,
										QString labelStr/*=QString()*/)
	: QDialog(parent)
	, Ui::PickEntityDlg()
{
	setupUi(this);

	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

	//multi-selection mode
	if (multiSelectionEnabled)
	{
		label->setText("Please select one or several entities:\n(press CTRL+A to select all)");
		listWidget->setSelectionMode(QAbstractItemView::ExtendedSelection);
	}

	for (size_t i=0; i<entities.size(); ++i)
	{
		//add one line per entity in the combo-box
		listWidget->insertItem(static_cast<int>(i), new QListWidgetItem(QString("%1 (ID=%2)").arg(entities[i]->getName()).arg(entities[i]->getUniqueID())));
	}
	
	//default selection
	if (defaultSelectedIndex >= 0 && static_cast<size_t>(defaultSelectedIndex) < entities.size())
		listWidget->setItemSelected(listWidget->item(defaultSelectedIndex),true);

	if (!labelStr.isNull())
		label->setText(labelStr);
}

int ccEntityPickerDlg::getSelectedIndex() const
{
	//get selected items
	QList<QListWidgetItem*> list = listWidget->selectedItems();
	return list.empty() ? -1 : listWidget->row(list.front());
}

void ccEntityPickerDlg::getSelectedIndexes(std::vector<int>& indexes) const
{
	//get selected items
	QList<QListWidgetItem*> list = listWidget->selectedItems();

	try
	{
		indexes.resize(static_cast<size_t>(list.size()));
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory?!
		return;
	}
	
	for (int i=0; i<list.size(); ++i)
	{
		indexes[i]  = listWidget->row(list[i]);
	}
}

int ccEntityPickerDlg::SelectEntity(const ccHObject::Container& entities,
									int selectedIndex/*=0*/,
									QWidget* parent/*=0*/,
									QString label/*=QString()*/)
{
	ccEntityPickerDlg epDlg(entities,false,selectedIndex,parent,label);
	if (!epDlg.exec())
		return -1;

	return epDlg.getSelectedIndex();
}

bool ccEntityPickerDlg::SelectEntities(const ccHObject::Container& entities,
									  std::vector<int>& selectedIndexes,
									  QWidget* parent/*=0*/,
									  QString label/*=QString()*/)
{
	selectedIndexes.clear();

	ccEntityPickerDlg epDlg(entities,true,-1,parent,label);
	if (!epDlg.exec())
		return false;

	epDlg.getSelectedIndexes(selectedIndexes);
	return true;
}
