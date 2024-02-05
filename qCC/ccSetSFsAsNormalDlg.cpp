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

ccSetSFsAsNormalDialog::ccSetSFsAsNormalDialog(const ccPointCloud* cloud, QWidget* parent/*=nullptr*/)
	: QDialog(parent, Qt::Tool)
	, Ui::SetSFsAsNormalDialog()
	, m_constFields(0)
{
	setupUi(this);

	QStringList fields{ "0", "1" };

	// default fields ('0 0 1' by default)
	int nxIndex = SF_INDEX_ZERO;
	int nyIndex = SF_INDEX_ZERO;
	int nzIndex = SF_INDEX_ONE;

	if (cloud->hasNormals())
	{
		fields << tr("Unchanged");

		nxIndex = SF_INDEX_UNCHANGED;
		nyIndex = SF_INDEX_UNCHANGED;
		nzIndex = SF_INDEX_UNCHANGED;
	}

	m_constFields = fields.size();

	if (cloud && cloud->hasScalarFields())
	{
		for (unsigned i = 0; i < cloud->getNumberOfScalarFields(); ++i)
		{
			CCCoreLib::ScalarField* sf = cloud->getScalarField(i);
			if (sf)
			{
				QString sfName = sf->getName();
				fields << sfName;

				sfName = sfName.toUpper();
				if (nxIndex < m_constFields && sfName.contains("NX"))
				{
					nxIndex = static_cast<int>(i);
				}
				if (nyIndex < m_constFields && sfName.contains("NY"))
				{
					nyIndex = static_cast<int>(i);
				}
				if (nzIndex < m_constFields && sfName.contains("NZ"))
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

	comboBoxNx->addItems(fields);
	comboBoxNy->addItems(fields);
	comboBoxNz->addItems(fields);

	setSFIndexes(nxIndex, nyIndex, nzIndex);
}

int ccSetSFsAsNormalDialog::toComboBoxIndex(int index) const
{
	if (index >= 0)
	{
		// valid SF index
		index += m_constFields;
		if (index >= comboBoxNx->count())
		{
			// we probably have less SFs as before
			return 0;
		}
		else
		{
			return index;
		}
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
	else if (index == ccSetSFsAsNormalDialog::SF_INDEX_UNCHANGED)
	{
		if (m_constFields == 3)
		{
			return 2;
		}
		else
		{
			// we don't have a 'Unchanged' field anymore, let's switch to Zero by default
			return 0;
		}
	}
	else
	{
		ccLog::Warning("[ccSetSFsAsNormalDialog] Invalid SF index " + QString::number(index));
		return -1;
	}
}

int ccSetSFsAsNormalDialog::fromComboBoxIndex(int index) const
{
	if (index == 0)
	{
		return ccSetSFsAsNormalDialog::SF_INDEX_ZERO;
	}
	else if (index == 1)
	{
		return ccSetSFsAsNormalDialog::SF_INDEX_ONE;
	}
	else if (index == 2 && m_constFields == 3)
	{
		return ccSetSFsAsNormalDialog::SF_INDEX_UNCHANGED;
	}
	else if (index >= m_constFields)
	{
		// valid SF index
		return index - m_constFields;
	}
	else
	{
		return -1;
	}
}

void ccSetSFsAsNormalDialog::setSFIndexes(int sf1Index, int sf2Index, int sf3Index)
{
	comboBoxNx->setCurrentIndex(toComboBoxIndex(sf1Index));
	comboBoxNy->setCurrentIndex(toComboBoxIndex(sf2Index));
	comboBoxNz->setCurrentIndex(toComboBoxIndex(sf3Index));
}


void ccSetSFsAsNormalDialog::getSFIndexes(int& sf1Index, int& sf2Index, int& sf3Index) const
{
	sf1Index = fromComboBoxIndex(comboBoxNx->currentIndex());
	sf2Index = fromComboBoxIndex(comboBoxNy->currentIndex());
	sf3Index = fromComboBoxIndex(comboBoxNz->currentIndex());
}
