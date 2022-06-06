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

#include "ccGraphicalSegmentationOptionsDlg.h"

#include <QSettings>

ccGraphicalSegmentationOptionsDlg::ccGraphicalSegmentationOptionsDlg(const QString windowTitle/*=QString()*/,
	QWidget* parent/*=0*/)
	: QDialog(parent, Qt::Tool)
	, Ui::GraphicalSegmentationOptionsDlg()
{
	setupUi(this);

	QSettings settings;
	settings.beginGroup("SegmentationToolOptions");
	QString remainingSuffix = settings.value("Remaining", ".remaining").toString();
	QString segmentedSuffix = settings.value("Segmented", ".segmented").toString();
	settings.endGroup();

	text_remaining->setText(remainingSuffix);
	text_segmented->setText(segmentedSuffix);

	if (!windowTitle.isEmpty())
	{
		setWindowTitle(windowTitle);
	}
}

void ccGraphicalSegmentationOptionsDlg::accept()
{
	QSettings settings;
	settings.beginGroup("SegmentationToolOptions");
	settings.setValue("Remaining", text_remaining->text());
	settings.setValue("Segmented", text_segmented->text());
	settings.endGroup();

	QDialog::accept();
}
