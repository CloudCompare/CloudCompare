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

#ifndef QFACET_FACETS_EXPORT_DLG_HEADER
#define QFACET_FACETS_EXPORT_DLG_HEADER

#include <QDialog>

#include "ui_facetsExportDlg.h"

//! Dialog for exporting facets or facets info (qFacets plugin)
class FacetsExportDlg : public QDialog, public Ui::FacetsExportDlg
{
	Q_OBJECT

public:

	//! Usage mode
	enum IOMode { SHAPE_FILE_IO, ASCII_FILE_IO };

	//! Default constructor
	FacetsExportDlg(IOMode mode, QWidget* parent = 0);

protected slots:

	//! Called when the 'browse' tool button is pressed
	void browseDestination();

protected:

	//! Current I/O mode
	IOMode m_mode;
};

#endif //QFACET_FACETS_EXPORT_DLG_HEADER
