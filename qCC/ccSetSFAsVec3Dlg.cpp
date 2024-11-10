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

#include "ccSetSFAsVec3Dlg.h"

// qCC_db
#include <ccPointCloud.h>

ccSetSFsAsVec3Dialog::ccSetSFsAsVec3Dialog(	const ccPointCloud* cloud,
											const QString& xLabel,
											const QString& yLabel,
											const QString& zLabel,
											bool allowUnchanged,
											QWidget* parent/*=nullptr*/)
	: QDialog(parent, Qt::Tool)
	, Ui::SetSFsAsVec3Dialog()
	, m_constFields(0)
{
	setupUi(this);

	QStringList fields{ "0", "1" };

	// default fields ('0 0 1' by default)
	int xIndex = SF_INDEX_ZERO;
	int yIndex = SF_INDEX_ZERO;
	int zIndex = SF_INDEX_ONE;

	if (allowUnchanged)
	{
		fields << tr("Unchanged");

		xIndex = SF_INDEX_UNCHANGED;
		yIndex = SF_INDEX_UNCHANGED;
		zIndex = SF_INDEX_UNCHANGED;
	}

	m_constFields = fields.size();

	if (cloud && cloud->hasScalarFields())
	{
		QString upperXLabel = xLabel.toUpper();
		QString upperYLabel = yLabel.toUpper();
		QString upperZLabel = zLabel.toUpper();

		for (unsigned i = 0; i < cloud->getNumberOfScalarFields(); ++i)
		{
			CCCoreLib::ScalarField* sf = cloud->getScalarField(i);
			if (sf)
			{
				QString sfName = QString::fromStdString(sf->getName());
				fields << sfName;

				sfName = sfName.toUpper();
				if (xIndex < m_constFields && sfName.contains(upperXLabel))
				{
					xIndex = static_cast<int>(i);
				}
				if (yIndex < m_constFields && sfName.contains(upperYLabel))
				{
					yIndex = static_cast<int>(i);
				}
				if (zIndex < m_constFields && sfName.contains(upperZLabel))
				{
					zIndex = static_cast<int>(i);
				}
			}
		}
	}
	else
	{
		ccLog::Warning("Cloud has no scalar field");
	}

	xComboBox->addItems(fields);
	yComboBox->addItems(fields);
	zComboBox->addItems(fields);

	setSFIndexes(xIndex, yIndex, zIndex);
}

int ccSetSFsAsVec3Dialog::toComboBoxIndex(int index) const
{
	if (index >= 0)
	{
		// valid SF index
		index += m_constFields;
		if (index >= xComboBox->count())
		{
			// we probably have less SFs as before
			return 0;
		}
		else
		{
			return index;
		}
	}
	else if (index == ccSetSFsAsVec3Dialog::SF_INDEX_NO)
	{
		return -1;
	}
	else if (index == ccSetSFsAsVec3Dialog::SF_INDEX_ZERO)
	{
		return 0;
	}
	else if (index == ccSetSFsAsVec3Dialog::SF_INDEX_ONE)
	{
		return 1;
	}
	else if (index == ccSetSFsAsVec3Dialog::SF_INDEX_UNCHANGED)
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
		ccLog::Warning("[ccSetSFsAsVec3Dialog] Invalid SF index " + QString::number(index));
		return -1;
	}
}

int ccSetSFsAsVec3Dialog::fromComboBoxIndex(int index) const
{
	if (index == 0)
	{
		return ccSetSFsAsVec3Dialog::SF_INDEX_ZERO;
	}
	else if (index == 1)
	{
		return ccSetSFsAsVec3Dialog::SF_INDEX_ONE;
	}
	else if (index == 2 && m_constFields == 3)
	{
		return ccSetSFsAsVec3Dialog::SF_INDEX_UNCHANGED;
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

void ccSetSFsAsVec3Dialog::setSFIndexes(int sf1Index, int sf2Index, int sf3Index)
{
	xComboBox->setCurrentIndex(toComboBoxIndex(sf1Index));
	yComboBox->setCurrentIndex(toComboBoxIndex(sf2Index));
	zComboBox->setCurrentIndex(toComboBoxIndex(sf3Index));
}


void ccSetSFsAsVec3Dialog::getSFIndexes(int& sf1Index, int& sf2Index, int& sf3Index) const
{
	sf1Index = fromComboBoxIndex(xComboBox->currentIndex());
	sf2Index = fromComboBoxIndex(yComboBox->currentIndex());
	sf3Index = fromComboBoxIndex(zComboBox->currentIndex());
}
