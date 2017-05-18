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

#include "ccItemSelectionDlg.h"

//Qt
#include <QDialog>
#include <QListWidgetItem>

ccItemSelectionDlg::ccItemSelectionDlg(	bool multiSelectionEnabled,
										QWidget* parent/*=0*/,
										QString itemName/*="entities"*/,
										QString labelStr/*=QString()*/)
	: QDialog(parent, Qt::Tool)
	, Ui::ItemSelectionDlg()
{
	setupUi(this);

	//multi-selection mode
	if (multiSelectionEnabled)
	{
		label->setText(tr("Please select one or several %1:\n(press CTRL+A to select all)").arg(itemName));
		listWidget->setSelectionMode(QAbstractItemView::ExtendedSelection);
	}
	else
	{
		label->setText(tr("Please select one %1").arg(itemName));
	}

	if (!labelStr.isNull())
	{
		label->setText(labelStr);
	}
}

void ccItemSelectionDlg::setItems(const QStringList& items, int defaultSelectedIndex)
{
	for (int i = 0; i < items.size(); ++i)
	{
		//add one line per entity in the combo-box
		listWidget->insertItem(static_cast<int>(i), new QListWidgetItem(items[i]));
	}

	//default selection
	if (defaultSelectedIndex >= 0 && defaultSelectedIndex < items.size())
	{
		listWidget->setItemSelected(listWidget->item(defaultSelectedIndex), true);
	}
}

int ccItemSelectionDlg::getSelectedIndex() const
{
	//get selected items
	QList<QListWidgetItem*> list = listWidget->selectedItems();
	return list.empty() ? -1 : listWidget->row(list.front());
}

void ccItemSelectionDlg::getSelectedIndexes(std::vector<int>& indexes) const
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
	
	for (int i = 0; i < list.size(); ++i)
	{
		indexes[i]  = listWidget->row(list[i]);
	}
}

int ccItemSelectionDlg::SelectEntity(const ccHObject::Container& entities,
									int selectedIndex/*=0*/,
									QWidget* parent/*=0*/,
									QString label/*=QString()*/)
{
	ccItemSelectionDlg epDlg(false, parent, tr("entity"), label);

	QStringList items;
	for (size_t i = 0; i < entities.size(); ++i)
	{
		//add one line per entity
		items << QString("%1 (ID=%2)").arg(entities[i]->getName()).arg(entities[i]->getUniqueID());
	}
	epDlg.setItems(items, selectedIndex);

	if (!epDlg.exec())
	{
		//cancelled by the user
		return -1;
	}

	return epDlg.getSelectedIndex();
}

bool ccItemSelectionDlg::SelectEntities(const ccHObject::Container& entities,
									  std::vector<int>& selectedIndexes,
									  QWidget* parent/*=0*/,
									  QString label/*=QString()*/)
{
	selectedIndexes.clear();

	ccItemSelectionDlg epDlg(true, parent, tr("entities"), label);

	QStringList items;
	for (size_t i = 0; i < entities.size(); ++i)
	{
		//add one line per entity
		items << QString("%1 (ID=%2)").arg(entities[i]->getName()).arg(entities[i]->getUniqueID());
	}
	epDlg.setItems(items, -1);

	if (!epDlg.exec())
		return false;

	epDlg.getSelectedIndexes(selectedIndexes);
	return true;
}
