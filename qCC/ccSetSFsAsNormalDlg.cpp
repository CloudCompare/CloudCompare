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
//#                   COPYRIGHT: CloudCompare project                      #
//#                                                                        #
//##########################################################################

#include "ccSetSFsAsNormalDlg.h"

// qCC_db
#include <ccPointCloud.h>

static int ToComboBoxIndex(int index)
{
	if (index >= 0)
	{
		// valid SF index
		return index + 2;
	}
	else if (index == ccSetSFsAsNormalDialog::SF_INDEX_NO)
	{
		return -1;
	}
	else if (index == ccSetSFsAsNormalDialog::SF_INDEX_ZERO)
	{
		return 0;
	}
	else if (index == ccSetSFsAsNormalDialog::SF_INDEX_ONE)
	{
		return 1;
	}
	else
	{
		ccLog::Warning("[ccSetSFsAsNormalDialog] Invalid SF index" + QString::number(index));
		return -1;
	}
}


static int FromComboBoxIndex(int index)
{
	if (index == 0)
	{
		return ccSetSFsAsNormalDialog::SF_INDEX_ZERO;
	}
	else if (index == 1)
	{
		return ccSetSFsAsNormalDialog::SF_INDEX_ONE;
	}
	else if (index >= 2)
	{
		// valid SF index
		return index - 2;
	}
	else
	{
		return -1;
	}
}

ccSetSFsAsNormalDialog::ccSetSFsAsNormalDialog(const ccPointCloud* cloud, QWidget* parent/*=nullptr*/)
	: QDialog(parent, Qt::Tool)
	, Ui::SetSFsAsNormalDialog()
{
	setupUi(this);

	comboBoxNx->addItem("0");
	comboBoxNx->addItem("1");

	comboBoxNy->addItem("0");
	comboBoxNy->addItem("1");

	comboBoxNz->addItem("0");
	comboBoxNz->addItem("1");

	// default fields ('0 0 1' by default)
	int nxIndex = SF_INDEX_ZERO;
	int nyIndex = SF_INDEX_ZERO;
	int nzIndex = SF_INDEX_ONE;

	if (cloud && cloud->hasScalarFields())
	{
		for (unsigned i = 0; i < cloud->getNumberOfScalarFields(); ++i)
		{
			CCCoreLib::ScalarField* sf = cloud->getScalarField(i);
			if (sf)
			{
				QString sfName = sf->getName();
				comboBoxNx->addItem(sfName);
				comboBoxNy->addItem(sfName);
				comboBoxNz->addItem(sfName);

				sfName = sfName.toUpper();

				if (nxIndex < 0 && sfName.contains("NX"))
				{
					nxIndex = static_cast<int>(i);
				}
				if (nyIndex < 0 && sfName.contains("NY"))
				{
					nyIndex = static_cast<int>(i);
				}
				if (nzIndex < 0 && sfName.contains("NZ"))
				{
					nzIndex = static_cast<int>(i);
				}
			}
		}
	}
	else
	{
		ccLog::Warning("Cloud has no scalar field");
	}

	setSFIndexes(nxIndex, nyIndex, nzIndex);
}

void ccSetSFsAsNormalDialog::setSFIndexes(int sf1Index, int sf2Index, int sf3Index)
{
	comboBoxNx->setCurrentIndex(ToComboBoxIndex(sf1Index));
	comboBoxNy->setCurrentIndex(ToComboBoxIndex(sf2Index));
	comboBoxNz->setCurrentIndex(ToComboBoxIndex(sf3Index));
}


void ccSetSFsAsNormalDialog::getSFIndexes(int& sf1Index, int& sf2Index, int& sf3Index) const
{
	sf1Index = FromComboBoxIndex(comboBoxNx->currentIndex());
	sf2Index = FromComboBoxIndex(comboBoxNy->currentIndex());
	sf3Index = FromComboBoxIndex(comboBoxNz->currentIndex());
}
