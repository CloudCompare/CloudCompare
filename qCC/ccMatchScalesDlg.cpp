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

#include "ccMatchScalesDlg.h"

//Qt
#include <QListWidgetItem>
#include <QDoubleValidator>

//system
#include <assert.h>

ccMatchScalesDlg::ccMatchScalesDlg(	const ccHObject::Container& entities,
										int defaultSelectedIndex/*=0*/,
										QWidget* parent/*=0*/)
	: QDialog(parent, Qt::Tool)
	, Ui::MatchScalesDialog()
{
	setupUi(this);

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

void ccMatchScalesDlg::setSelectedAlgorithm(ccLibAlgorithms::ScaleMatchingAlgorithm algorithm)
{
	switch(algorithm)
	{
	case ccLibAlgorithms::BB_MAX_DIM:
		bbMaxDimRadioButton->setChecked(true);
		return;
	case ccLibAlgorithms::BB_VOLUME:
		bbVolumeRadioButton->setChecked(true);
		return;
	case ccLibAlgorithms::PCA_MAX_DIM:
		pcaRadioButton->setChecked(true);
		return;
	case ccLibAlgorithms::ICP_SCALE:
		icpRadioButton->setChecked(true);
		return;
	default:
		assert(false);
		break;
	}
}

ccLibAlgorithms::ScaleMatchingAlgorithm ccMatchScalesDlg::getSelectedAlgorithm() const
{
	if (bbMaxDimRadioButton->isChecked())
	{
		return ccLibAlgorithms::BB_MAX_DIM;
	}
	else if (bbVolumeRadioButton->isChecked())
	{
		return ccLibAlgorithms::BB_VOLUME;
	}
	else if (pcaRadioButton->isChecked())
	{
		return ccLibAlgorithms::PCA_MAX_DIM;
	}
	else if (icpRadioButton->isChecked())
	{
		return ccLibAlgorithms::ICP_SCALE;
	}

	assert(false);
	return ccLibAlgorithms::PCA_MAX_DIM;
}
