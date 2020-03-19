//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qFacets                       #
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
//#                      COPYRIGHT: Thomas Dewez, BRGM                     #
//#                                                                        #
//##########################################################################

#include "facetsExportDlg.h"

//Qt
#include <QFileDialog>

//System
#include <assert.h>

FacetsExportDlg::FacetsExportDlg(IOMode mode, QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::FacetsExportDlg()
	, m_mode(mode)
{
	setupUi(this);

	connect(browseToolButton, &QAbstractButton::clicked, this, &FacetsExportDlg::browseDestination);
}

void FacetsExportDlg::browseDestination()
{
	QString saveFileFilter;
	switch(m_mode)
	{
	case SHAPE_FILE_IO:
		saveFileFilter = "Shapefile (*.shp)";
		break;
	case ASCII_FILE_IO:
		saveFileFilter = "ASCII table (*.csv)";
		break;
	default:
		assert(false);
		return;
	}

	//open file saving dialog
	QString outputFilename = QFileDialog::getSaveFileName(nullptr, "Select destination", destinationPathLineEdit->text(), saveFileFilter);

	if (outputFilename.isEmpty())
		return;

	destinationPathLineEdit->setText(outputFilename);
}
