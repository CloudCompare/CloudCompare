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

#include "ccMatchScalesDlg.h"

//Qt
#include <QDialog>
#include <QListWidgetItem>
#include <QDoubleValidator>

//system
#include <assert.h>

ccMatchScalesDlg::ccMatchScalesDlg(	const ccHObject::Container& entities,
										int defaultSelectedIndex/*=0*/,
										QWidget* parent/*=0*/)
	: QDialog(parent)
	, Ui::MatchScalesDialog()
{
	setupUi(this);

	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

	for (size_t i=0; i<entities.size(); ++i)
	{
		//add one line per entity in the combo-box
		listWidget->insertItem(static_cast<int>(i), new QListWidgetItem(QString("%1 (ID=%2)").arg(entities[i]->getName()).arg(entities[i]->getUniqueID())));
	}
	
	//default selection
	if (defaultSelectedIndex >= 0 && static_cast<size_t>(defaultSelectedIndex) < entities.size())
		listWidget->setItemSelected(listWidget->item(defaultSelectedIndex),true);

	rmsDifferenceLineEdit->setValidator(new QDoubleValidator(rmsDifferenceLineEdit));
}

int ccMatchScalesDlg::getSelectedIndex() const
{
	//get selected items
	QList<QListWidgetItem*> list = listWidget->selectedItems();
	return list.empty() ? -1 : listWidget->row(list.front());
}

void ccMatchScalesDlg::setSelectedAlgorithm(MainWindow::ScaleMatchingAlgorithm algorithm)
{
	switch(algorithm)
	{
	case MainWindow::BB_MAX_DIM:
		bbMaxDimRadioButton->setChecked(true);
		return;
	case MainWindow::BB_VOLUME:
		bbVolumeRadioButton->setChecked(true);
		return;
	case MainWindow::PCA_MAX_DIM:
		pcaRadioButton->setChecked(true);
		return;
	case MainWindow::ICP_SCALE:
		icpRadioButton->setChecked(true);
		return;
	default:
		assert(false);
		break;
	}
}

MainWindow::ScaleMatchingAlgorithm ccMatchScalesDlg::getSelectedAlgorithm() const
{
	if (bbMaxDimRadioButton->isChecked())
	{
		return MainWindow::BB_MAX_DIM;
	}
	else if (bbVolumeRadioButton->isChecked())
	{
		return MainWindow::BB_VOLUME;
	}
	else if (pcaRadioButton->isChecked())
	{
		return MainWindow::PCA_MAX_DIM;
	}
	else if (icpRadioButton->isChecked())
	{
		return MainWindow::ICP_SCALE;
	}

	assert(false);
	return MainWindow::PCA_MAX_DIM;
}
