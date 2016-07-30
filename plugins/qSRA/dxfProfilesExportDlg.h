//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qSRA                         #
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
//#                           COPYRIGHT: EDF                               #
//#                                                                        #
//##########################################################################

#ifndef QSRA_DXF_PROFILE_IMPORT_DLG_HEADER
#define QSRA_DXF_PROFILE_IMPORT_DLG_HEADER

#include "ui_dxfProfilesExportDlg.h"

//! Dialog for export multiple 2D profiles in a single DXF file (qSRA plugin)
class DxfProfilesExportDlg : public QDialog, public Ui::DxfProfilesExportDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit DxfProfilesExportDlg(QWidget* parent = 0);

	//! Returns vert. profiles output filename (on completion)
	QString getVertFilename() const;
	//! Returns horiz. profiles output filename (on completion)
	QString getHorizFilename() const;

protected slots:

	//! Called when the vert. 'browse' tool button is pressed
	void browseVertFile();
	//! Called when the horiz. 'browse' tool button is pressed
	void browseHorizFile();

	//! Saves dialog state to persistent settings
	void acceptAndSaveSettings();

protected:

	//! Inits dialog state from persistent settings
	void initFromPersistentSettings();

};

#endif //QSRA_PROFILE_IMPORT_DLG_HEADER
