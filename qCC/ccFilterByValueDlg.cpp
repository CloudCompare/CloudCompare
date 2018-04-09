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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include "ccFilterByValueDlg.h"

ccFilterByValueDlg::ccFilterByValueDlg(	double minRange,
										double maxRange,
										double minVal/*=-1.0e9*/,
										double maxVal/*=1.0e9*/,
										QWidget* parent/*=0*/)
	: QDialog(parent, Qt::Tool)
	, Ui::FilterByValueDialog()
	, m_mode(CANCEL)
{
	setupUi(this);

	minDoubleSpinBox->setRange(minVal, maxVal);
	maxDoubleSpinBox->setRange(minVal, maxVal);
	minDoubleSpinBox->setValue(minRange);
	maxDoubleSpinBox->setValue(maxRange);

	connect(exportPushButton, &QAbstractButton::clicked, this, &ccFilterByValueDlg::onExport);
	connect(splitPushButton, &QAbstractButton::clicked, this, &ccFilterByValueDlg::onSplit);
}
