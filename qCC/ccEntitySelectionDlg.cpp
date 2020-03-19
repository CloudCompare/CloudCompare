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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#include "ccEntitySelectionDlg.h"

//ui
#include <ui_entitySelectionDlg.h>

//Qt
#include <QDialog>
#include <QListWidgetItem>

ccEntitySelectionDialog::ccEntitySelectionDialog(	const ccHObject::Container& entities,
													bool multiSelectionEnabled,
													int defaultSelectedIndex/*=0*/,
													QWidget* parent/*=0*/,
													QString labelStr/*=QString()*/)
	: QDialog(parent, Qt::Tool)
	, m_ui(new Ui_EntitySelectionDialog)
{
	m_ui->setupUi(this);

	//multi-selection mode
	if (multiSelectionEnabled)
	{
		m_ui->label->setText(tr("Select one or several entities:"));
		m_ui->listWidget->setSelectionMode(QAbstractItemView::ExtendedSelection);
		connect(m_ui->selectAllPushButton,  &QPushButton::clicked, this, &ccEntitySelectionDialog::selectAll);
		connect(m_ui->selectNonePushButton, &QPushButton::clicked, this, &ccEntitySelectionDialog::selectNone);
	}
	else
	{
		m_ui->selectAllPushButton->setVisible(false);
		m_ui->selectNonePushButton->setVisible(false);
	}

	for (size_t i = 0; i < entities.size(); ++i)
	{
		//add one line per entity in the combo-box
		m_ui->listWidget->insertItem(static_cast<int>(i), new QListWidgetItem(QString("[ID %1] %2").arg(entities[i]->getUniqueID()).arg(entities[i]->getName())));
	}
	
	//default selection
	if (defaultSelectedIndex >= 0 && static_cast<size_t>(defaultSelectedIndex) < entities.size())
	{
		m_ui->listWidget->setItemSelected(m_ui->listWidget->item(defaultSelectedIndex), true);
	}

	//custom lalel
	if (!labelStr.isNull())
	{
		m_ui->label->setText(labelStr);
	}
}

ccEntitySelectionDialog::~ccEntitySelectionDialog()
{
	if (m_ui)
	{
		delete m_ui;
		m_ui = nullptr;
	}
}

void ccEntitySelectionDialog::selectAll()
{
	m_ui->listWidget->selectAll();
}

void ccEntitySelectionDialog::selectNone()
{
	m_ui->listWidget->clearSelection();
}

int ccEntitySelectionDialog::getSelectedIndex() const
{
	//get selected items
	QList<QListWidgetItem*> list = m_ui->listWidget->selectedItems();
	return list.empty() ? -1 : m_ui->listWidget->row(list.front());
}

void ccEntitySelectionDialog::getSelectedIndexes(std::vector<int>& indexes) const
{
	//get selected items
	QList<QListWidgetItem*> list = m_ui->listWidget->selectedItems();

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
		indexes[i] = m_ui->listWidget->row(list[i]);
	}
}

int ccEntitySelectionDialog::SelectEntity(	const ccHObject::Container& entities,
											int selectedIndex/*=0*/,
											QWidget* parent/*=0*/,
											QString label/*=QString()*/)
{
	ccEntitySelectionDialog epDlg(entities, false, selectedIndex, parent, label);
	if (!epDlg.exec())
	{
		return -1;
	}

	return epDlg.getSelectedIndex();
}

bool ccEntitySelectionDialog::SelectEntities(	const ccHObject::Container& entities,
												std::vector<int>& selectedIndexes,
												QWidget* parent/*=0*/,
												QString label/*=QString()*/)
{
	selectedIndexes.clear();

	ccEntitySelectionDialog epDlg(entities, true, -1, parent, label);
	if (!epDlg.exec())
	{
		return false;
	}

	epDlg.getSelectedIndexes(selectedIndexes);
	return true;
}
